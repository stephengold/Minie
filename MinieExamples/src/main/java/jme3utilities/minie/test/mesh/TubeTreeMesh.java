/*
 Copyright (c) 2019, Stephen Gold
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package jme3utilities.minie.test.mesh;

import com.jme3.animation.Bone;
import com.jme3.animation.Skeleton;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.util.BufferUtils;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;
import java.util.BitSet;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Misc;
import jme3utilities.MyString;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;

/**
 * An animated triangle-mode mesh for a branching 3-D shape that conforms to a
 * Skeleton. Can be used to visualize ropes, hoses, snakes, and such. TODO add
 * texture coordinates
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TubeTreeMesh extends Mesh {
    // *************************************************************************
    // constants and loggers

    /**
     * maximum number of weights per vertex
     */
    final private static int maxWpv = 4;
    /**
     * number of axes in a vector
     */
    final private static int numAxes = 3;
    /**
     * number of vertices per triangle
     */
    final private static int vpt = 3;
    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(TubeTreeMesh.class.getName());
    /**
     * local copy of {@link com.jme3.math.Vector3f#ZERO}
     */
    final private static Vector3f translateIdentity = new Vector3f(0f, 0f, 0f);
    // *************************************************************************
    // fields

    /**
     * overshoot distance for leaf bones (in mesh units, default=0)
     */
    private float leafOvershoot;
    /**
     * radius of each mesh loop (in mesh units)
     */
    private float radius;
    /**
     * normal buffer
     */
    private FloatBuffer normalBuffer;
    /**
     * weight buffer
     */
    private FloatBuffer weightBuffer;
    /**
     * position buffer
     */
    private FloatBuffer positionBuffer;
    /**
     * total number of triangles in the mesh
     */
    private int numTriangles;
    /**
     * total number of vertices in the mesh
     */
    private int numVertices;
    /**
     * number of mesh loops in each tube segment (default=3)
     */
    private int loopsPerSegment;
    /**
     * number of sample points per mesh loop (default=12)
     */
    private int samplesPerLoop;
    /**
     * index buffer
     */
    private ShortBuffer indexBuffer;
    /**
     * skeleton used to construct the mesh
     */
    private Skeleton skeleton;
    /**
     * cached sample positions for a unit circle in the X-Y plane
     */
    private Vector3f[] circleSamples = null;
    /**
     * reusable samples to transform
     */
    private Vector3f[] reusable = {
        new Vector3f(), new Vector3f(), new Vector3f(), new Vector3f(),
        new Vector3f(), new Vector3f(), new Vector3f(), new Vector3f()
    };
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public TubeTreeMesh() {
    }

    /**
     * Instantiate a tube-tree mesh based on the specified Skeleton and radius.
     *
     * @param skeleton (not null, in bind pose, unaffected)
     * @param radius the radius of each mesh loop (in mesh units, &gt;0)
     */
    public TubeTreeMesh(Skeleton skeleton, float radius) {
        this(skeleton, radius, 0f, 3, 12);
    }

    /**
     * Instantiate a tube-tree mesh based on the specified Skeleton and
     * parameters.
     *
     * @param skeleton (not null, in bind pose, unaffected)
     * @param radius the radius of each mesh loop (in mesh units, &gt;0)
     * @param leafOvershoot the overshoot distance for leaf bones (in mesh
     * units)
     * @param loopsPerSegment the number of mesh loops in each tube segment
     * (&ge;1)
     * @param samplesPerLoop the number of samples in each mesh loop (&ge;3)
     */
    public TubeTreeMesh(Skeleton skeleton, float radius, float leafOvershoot,
            int loopsPerSegment, int samplesPerLoop) {
        Validate.nonNull(skeleton, "skeleton");
        Validate.positive(radius, "radius");
        Validate.positive(loopsPerSegment, "loops per segment");
        Validate.inRange(samplesPerLoop, "samples per loop", 3,
                Integer.MAX_VALUE);

        this.skeleton = (Skeleton) Misc.deepCopy(skeleton);
        this.radius = radius;
        this.leafOvershoot = leafOvershoot;
        this.loopsPerSegment = loopsPerSegment;
        this.samplesPerLoop = samplesPerLoop;

        setMaxNumWeights(2);
        updateAll();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Enumerate the indices of all cap vertices for the named Bone.
     *
     * @param boneName the name of the Bone
     * @return a new BitSet of vertex indices (not null)
     */
    public BitSet listCapVertices(String boneName) {
        Bone bone = skeleton.getBone(boneName);
        if (bone == null) {
            String message = "no such bone: " + MyString.quote(boneName);
            throw new IllegalArgumentException(message);
        }

        int numBones = skeleton.getBoneCount();
        int trianglesPerCap = samplesPerLoop - 2;
        int trianglesPerLoop = 2 * samplesPerLoop;
        int trianglesPerSegment = trianglesPerLoop * loopsPerSegment;

        BitSet result = new BitSet(numVertices);
        int triCount = 0;

        for (int childIndex = 0; childIndex < numBones; ++childIndex) {
            Bone child = skeleton.getBone(childIndex);
            Bone parent = child.getParent();
            if (parent != null) {
                triCount += trianglesPerSegment;

                if (capChild(child)) {
                    if (child == bone) {
                        int fromIndex = vpt * triCount;
                        int toIndex = vpt * (triCount + trianglesPerCap);
                        result.set(fromIndex, toIndex);
                    }
                    triCount += trianglesPerCap;
                }

                if (capParent(parent)) {
                    if (parent == bone) {
                        int fromIndex = vpt * triCount;
                        int toIndex = vpt * (triCount + trianglesPerCap);
                        result.set(fromIndex, toIndex);
                    }
                    triCount += trianglesPerCap;
                }
            }
        }

        return result;
    }

    /**
     * Calculate the number of vertices in each cap.
     *
     * @return count (&ge;3)
     */
    public int verticesPerCap() {
        int trianglesPerCap = samplesPerLoop - 2;
        int result = vpt * trianglesPerCap;

        return result;
    }
    // *************************************************************************
    // Mesh methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned mesh into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this mesh (not null)
     * @param original the mesh from which this mesh was shallow-cloned (unused)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);

        weightBuffer = cloner.clone(weightBuffer);
        normalBuffer = cloner.clone(normalBuffer);
        positionBuffer = cloner.clone(positionBuffer);
        indexBuffer = cloner.clone(indexBuffer);
        // skeleton not cloned (read-only)
        assert circleSamples == null : circleSamples;
        reusable = cloner.clone(reusable);
    }

    /**
     * De-serialize this mesh from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);
        InputCapsule capsule = importer.getCapsule(this);

        leafOvershoot = capsule.readFloat("leafOvershoot", 0f);
        radius = capsule.readFloat("radius", 1f);
        loopsPerSegment = capsule.readInt("loopsPerSegment", 3);
        samplesPerLoop = capsule.readInt("samplesPerLoop", 12);
        skeleton = (Skeleton) capsule.readSavable("skeleton", null);
        /*
         * Recalculate the derived properties.
         */
        updateDerivedProperties();
    }

    /**
     * Serialize this Mesh to the specified exporter, for example when saving to
     * a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        super.write(exporter);
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(leafOvershoot, "leafOvershoot", 0f);
        capsule.write(radius, "radius", 1f);
        capsule.write(loopsPerSegment, "loopsPerSegment", 3);
        capsule.write(samplesPerLoop, "samplesPerLoop", 12);
        capsule.write(skeleton, "skeleton", null);
    }
    // *************************************************************************
    // private methods

    /**
     * Allocate new buffers for mesh data.
     */
    private void allocateBuffers() {
        int weightCount = maxWpv * numVertices;
        // TODO use a ByteBuffer if possible
        indexBuffer = BufferUtils.createShortBuffer(weightCount);
        setBuffer(VertexBuffer.Type.BoneIndex, maxWpv, indexBuffer);
        weightBuffer = BufferUtils.createFloatBuffer(weightCount);
        setBuffer(VertexBuffer.Type.BoneWeight, maxWpv, weightBuffer);

        positionBuffer = BufferUtils.createVector3Buffer(numVertices);
        setBuffer(VertexBuffer.Type.BindPosePosition, numAxes, positionBuffer);
        normalBuffer = BufferUtils.createVector3Buffer(numVertices);
        setBuffer(VertexBuffer.Type.BindPoseNormal, numAxes, normalBuffer);

        FloatBuffer pb = BufferUtils.createVector3Buffer(numVertices);
        setBuffer(VertexBuffer.Type.Position, numAxes, pb);
        FloatBuffer nb = BufferUtils.createVector3Buffer(numVertices);
        setBuffer(VertexBuffer.Type.Normal, numAxes, nb);
    }

    /**
     * Test whether segments having the specified Bone as the child should be
     * capped at the child's end.
     *
     * @param child the Bone to test (not null, unaffected)
     * @return true if capped, otherwise false
     */
    private static boolean capChild(Bone child) {
        int numChildren = child.getChildren().size();
        if (numChildren == 1) {
            return false;
        } else {
            return true; // child is a leaf or fork: cap it
        }
    }

    /**
     * Test whether segments having the specified Bone as the parent should be
     * capped at the parent's end.
     *
     * @param parent the Bone to test (not null, unaffected)
     * @return true if capped, otherwise false
     */
    private static boolean capParent(Bone parent) {
        Bone grandparent = parent.getParent();
        boolean isCapped;
        if (grandparent == null) {
            isCapped = true; // parent is a root: cap it
        } else {
            int numChildren = parent.getChildren().size();
            if (numChildren > 1) {
                isCapped = true; // parent is a fork: cap it
            } else {
                isCapped = false;
            }
        }
        return isCapped;
    }

    /**
     * Pre-compute sample coordinates for a unit circle in the X-Y plane,
     * centered at the origin.
     */
    private void initCircleSamples() {
        circleSamples = new Vector3f[samplesPerLoop + 1];
        for (int sampleI = 0; sampleI <= samplesPerLoop; ++sampleI) {
            float theta = (2f * FastMath.PI / samplesPerLoop) * sampleI;
            float cos = FastMath.cos(theta);
            float sin = FastMath.sin(theta);
            circleSamples[sampleI] = new Vector3f(cos, sin, 0f);
        }
    }

    /**
     * Test whether the specified Bone is a leaf.
     *
     * @param bone the Bone to test (not null, unaffected)
     * @return
     */
    private static boolean isLeaf(Bone bone) {
        boolean result = bone.getChildren().isEmpty();
        return result;
    }

    /**
     * Write bone indices and weights for a triangle animated entirely by a
     * single Bone.
     *
     * @param boneIndex the index of the bone that animates the triangle
     */
    private void putAnimationForTriangle(int boneIndex) {
        assert boneIndex >= 0 : boneIndex;
        assert boneIndex <= Short.MAX_VALUE : boneIndex;

        for (int vertexIndex = 0; vertexIndex < vpt; ++vertexIndex) {
            indexBuffer.put((short) boneIndex);
            weightBuffer.put(1f);

            for (int weightIndex = 1; weightIndex < maxWpv; ++weightIndex) {
                indexBuffer.put((short) 0);
                weightBuffer.put(0f);
            }
        }
    }

    /**
     * Write bone indices and weights for a vertex animated by 2 bones.
     *
     * @param boneIndex1 the index of the first bone (&ge;0)
     * @param boneIndex2 the index of the 2nd bone (&ge;0)
     * @param weight1 the weight for the first bone
     */
    private void putAnimationForVertex(int boneIndex1, int boneIndex2,
            float weight1) {
        assert boneIndex1 >= 0 : boneIndex1;
        assert boneIndex1 <= Short.MAX_VALUE : boneIndex1;
        assert boneIndex2 >= 0 : boneIndex2;
        assert boneIndex2 <= Short.MAX_VALUE : boneIndex2;
        assert boneIndex1 != boneIndex2 : boneIndex1;

        weight1 = FastMath.clamp(weight1, 0f, 1f);

        int weightIndex;
        if (weight1 != 0f) {
            indexBuffer.put((short) boneIndex1);
            weightBuffer.put(weight1);
            weightIndex = 2;
        } else {
            weightIndex = 1;
        }

        indexBuffer.put((short) boneIndex2);
        weightBuffer.put(1f - weight1);

        while (weightIndex < maxWpv) {
            indexBuffer.put((short) 0);
            weightBuffer.put(0f);
            ++weightIndex;
        }
    }

    /**
     * Write a flat, circular cap to the buffers. TODO more cap-shape options
     *
     * @param centerPos the position of the center of the cap (in mesh
     * coordinates, not null, unaffected)
     * @param orientation the orientation of the cap (cap lies in the X-Y plane)
     * (in mesh coordinates, not null, unaffected)
     * @param posZ the Z component of the cap position (in local coordinates)
     * @param normalZ the Z component of the normal direction (in local
     * coordinates, +1 or -1)
     * @param boneIndex (&ge;0)
     */
    private void putCap(Vector3f centerPos, Quaternion orientation, float posZ,
            float normalZ, int boneIndex) {
        Vector3f a = reusable[0];
        Vector3f b = reusable[1];
        Vector3f c = reusable[2];

        int startBufferOffset = positionBuffer.position();
        /*
         * Put a triangle for each sample point except the first and last.
         */
        for (int triIndex = 1; triIndex < samplesPerLoop - 1; ++triIndex) {
            Vector3f pos1 = circleSamples[0];
            Vector3f pos2, pos3;
            if (normalZ > 0f) {
                pos2 = circleSamples[triIndex];
                pos3 = circleSamples[triIndex + 1];
            } else {
                pos2 = circleSamples[triIndex + 1];
                pos3 = circleSamples[triIndex];

                //int i = (positionBuffer.position() - startBufferOffset) / numAxes;
                //System.out.printf("+%d: %d%n", i, 0);
                //System.out.printf("+%d: %d%n", i + 1, triIndex + 1);
                //System.out.printf("+%d: %d%n", i + 2, triIndex);
            }

            a.set(radius * pos1.x, radius * pos1.y, posZ);
            b.set(radius * pos2.x, radius * pos2.y, posZ);
            c.set(radius * pos3.x, radius * pos3.y, posZ);
            putTransformedTriangle(positionBuffer, centerPos, orientation,
                    a, b, c);

            a.set(0f, 0f, normalZ);
            b.set(0f, 0f, normalZ);
            c.set(0f, 0f, normalZ);
            putTransformedTriangle(normalBuffer, translateIdentity, orientation,
                    a, b, c);

            putAnimationForTriangle(boneIndex);
        }
        int numFloats = positionBuffer.position() - startBufferOffset;
        assert numFloats == numAxes * verticesPerCap();
    }

    /**
     * Rotate and translate each vertex of a triangle and write the transformed
     * coordinates to the specified buffer.
     *
     * @param buffer the buffer to write to (not null, modified)
     * @param offset the translation to apply (not null, unaffected)
     * @param rotation the rotation to apply (not null, unaffected)
     * @param v1 the first vertex vector to transform (not null, modified)
     * @param v2 the 2nd vertex vector to transform (not null, modified)
     * @param v3 the 3rd vertex vector to transform (not null, modified)
     */
    private void putTransformedTriangle(FloatBuffer buffer, Vector3f offset,
            Quaternion rotation, Vector3f v1, Vector3f v2, Vector3f v3) {
        rotation.mult(v1, v1);
        v1.addLocal(offset);
        buffer.put(v1.x).put(v1.y).put(v1.z);

        rotation.mult(v2, v2);
        v2.addLocal(offset);
        buffer.put(v2.x).put(v2.y).put(v2.z);

        rotation.mult(v3, v3);
        v3.addLocal(offset);
        buffer.put(v3.x).put(v3.y).put(v3.z);
    }

    /**
     * Write an uncapped, cylindrical tube segment to the buffers.
     *
     * @param origin the origin of the segment's local coordinate system (in
     * mesh coordinates, not null, unaffected)
     * @param orientation the orientation of the segment's local coordinate
     * system (in mesh coordinates, the local +Z axis being the length axis, not
     * null, unaffected)
     * @param startZ the Z component of the start position (in local
     * coordinates)
     * @param endZ the Z component of the end position (in local coordinates)
     * @param startBoneIndex (&ge;0)
     * @param endBoneIndex (&ge;0)
     */
    private void putTube(Vector3f origin, Quaternion orientation, float startZ,
            float endZ, int startBoneIndex, int endBoneIndex) {
        assert startBoneIndex != endBoneIndex;

        Vector3f pos11 = reusable[0];
        Vector3f pos12 = reusable[1];
        Vector3f pos21 = reusable[2];
        Vector3f pos22 = reusable[3];
        Vector3f norm11 = reusable[4];
        Vector3f norm12 = reusable[5];
        Vector3f norm21 = reusable[6];
        Vector3f norm22 = reusable[7];

        float fractionStep = 1f / loopsPerSegment;
        for (int loopIndex = 0; loopIndex < loopsPerSegment; ++loopIndex) {
            float fraction1 = fractionStep * loopIndex;
            float fraction2 = fractionStep * (loopIndex + 1);
            float z1 = MyMath.lerp(fraction1, startZ, endZ);
            float z2 = MyMath.lerp(fraction2, startZ, endZ);

            // a rectangular patch for each sample point
            for (int rectIndex = 0; rectIndex < samplesPerLoop; ++rectIndex) {
                float x1 = circleSamples[rectIndex].x * radius;
                float y1 = circleSamples[rectIndex].y * radius;
                float x2 = circleSamples[rectIndex + 1].x * radius;
                float y2 = circleSamples[rectIndex + 1].y * radius;
                Vector3f normal1 = circleSamples[rectIndex];
                Vector3f normal2 = circleSamples[rectIndex + 1];
                /*
                 * Put the lower-right triangle.
                 */
                pos11.set(x1, y1, z1);
                pos21.set(x2, y2, z1);
                pos22.set(x2, y2, z2);
                putTransformedTriangle(positionBuffer, origin,
                        orientation, pos11, pos21, pos22);

                norm11.set(normal1.x, normal1.y, 0f);
                norm21.set(normal2.x, normal2.y, 0f);
                norm22.set(normal2.x, normal2.y, 0f);
                putTransformedTriangle(normalBuffer, translateIdentity,
                        orientation, norm11, norm21, norm22);

                putAnimationForVertex(endBoneIndex, startBoneIndex, fraction1);
                putAnimationForVertex(endBoneIndex, startBoneIndex, fraction1);
                putAnimationForVertex(endBoneIndex, startBoneIndex, fraction2);
                /*
                 * Put the upper-left triangle.
                 */
                pos11.set(x1, y1, z1);
                pos12.set(x1, y1, z2);
                pos22.set(x2, y2, z2);
                putTransformedTriangle(positionBuffer, origin,
                        orientation, pos11, pos22, pos12);

                norm11.set(normal1.x, normal1.y, 0f);
                norm12.set(normal1.x, normal1.y, 0f);
                norm22.set(normal2.x, normal2.y, 0f);
                putTransformedTriangle(normalBuffer, translateIdentity,
                        orientation, norm11, norm22, norm12);

                putAnimationForVertex(endBoneIndex, startBoneIndex, fraction1);
                putAnimationForVertex(endBoneIndex, startBoneIndex, fraction2);
                putAnimationForVertex(endBoneIndex, startBoneIndex, fraction2);
            }
        }
    }

    /**
     * Rebuild this mesh after a parameter change.
     */
    private void updateAll() {
        updateDerivedProperties();
        updateBuffers();
        updateBound();
    }

    /**
     * Allocate mesh buffers and fill them with data.
     */
    private void updateBuffers() {
        initCircleSamples();
        allocateBuffers();

        int numBones = skeleton.getBoneCount();
        for (int childIndex = 0; childIndex < numBones; ++childIndex) {
            Bone child = skeleton.getBone(childIndex);
            Bone parent = child.getParent();
            if (parent != null) { // child is not a root bone
                int parentIndex = skeleton.getBoneIndex(parent);
                Vector3f childPosition = child.getModelSpacePosition();
                Vector3f parentPosition = parent.getModelSpacePosition();
                Vector3f offset = parentPosition.subtract(childPosition);
                float endZ = offset.length();

                Vector3f direction = offset.clone();
                Vector3f axis1 = new Vector3f();
                Vector3f axis2 = new Vector3f();
                MyVector3f.generateBasis(direction, axis1, axis2);
                Quaternion orientation = new Quaternion();
                orientation.fromAxes(axis1, axis2, direction);

                float startZ = isLeaf(child) ? -leafOvershoot : 0f;

                putTube(childPosition, orientation, startZ, endZ, childIndex,
                        parentIndex);

                if (capChild(child)) {
                    // Cap the child's end: the "start" of the current segment.
                    putCap(childPosition, orientation, startZ, -1f, childIndex);
                }

                if (capParent(parent)) {
                    // Cap the parent's end: the "end" of the current segment.
                    putCap(childPosition, orientation, endZ, 1f, parentIndex);
                }
            }
        }

        circleSamples = null;
    }

    /**
     * Update 2 basic properties of the mesh: triangleCount and vertexCount.
     */
    private void updateDerivedProperties() {
        int numSegments = 0;
        int numCaps = 0;
        int numBones = skeleton.getBoneCount();
        for (int childIndex = 0; childIndex < numBones; ++childIndex) {
            Bone child = skeleton.getBone(childIndex);
            Bone parent = child.getParent();
            if (parent != null) {
                ++numSegments;
                if (capChild(child)) {
                    ++numCaps;
                }
                if (capParent(parent)) {
                    ++numCaps;
                }
            }
        }

        int trianglesPerCap = samplesPerLoop - 2;
        int trianglesPerLoop = 2 * samplesPerLoop;
        int trianglesPerSegment = trianglesPerLoop * loopsPerSegment;
        numTriangles
                = trianglesPerCap * numCaps + trianglesPerSegment * numSegments;
        logger.log(Level.INFO, "{0} triangles", numTriangles);

        numVertices = vpt * numTriangles;
        logger.log(Level.INFO, "{0} vertices", numVertices);
        assert numVertices <= Short.MAX_VALUE : numVertices;
    }
}
