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
package jme3utilities.minie.test;

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
import java.io.IOException;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;
import java.util.BitSet;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Misc;
import jme3utilities.MyString;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * An animated triangle-mode mesh for a branching 3-D shape that conforms to a
 * Skeleton. Can be used to visualize ropes, hoses, snakes, and such. TODO
 * texture coordinates
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TubeTreeMesh extends Mesh {
    // *************************************************************************
    // constants and loggers

    /**
     * maximum number of bone weights per vertex
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
     * radius of each mesh ring (in mesh units, default=1)
     */
    private float radius = 1f;
    /**
     * bone-weight buffer
     */
    private FloatBuffer boneWeightBuffer;
    /**
     * normal buffer
     */
    private FloatBuffer normalBuffer;
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
     * number of mesh rings in each tube segment connecting 2 bone heads
     * (default=8)
     */
    private int ringsPerSegment = 8;
    /**
     * number of sample points per mesh ring (default=12)
     */
    private int samplesPerRing = 12;
    /**
     * bone-index buffer
     */
    private ShortBuffer boneIndexBuffer;
    /**
     * skeleton used to construct the mesh
     */
    private Skeleton skeleton;
    /**
     * cached sample positions for a unit circle in the X-Y plane
     */
    private Vector3f[] circleSamples;
    /**
     * reusable samples to transform
     */
    final private static Vector3f[] reusable = new Vector3f[]{
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
     * Instantiate a tube-tree mesh based on the specified skeleton and radius.
     *
     * @param skeleton (not null, in bind pose, unaffected)
     * @param radius the radius of each mesh ring (in mesh units, &gt;0)
     */
    public TubeTreeMesh(Skeleton skeleton, float radius) {
        this(skeleton, radius, 8, 12);
    }

    /**
     * Instantiate a tube-tree mesh based on the specified skeleton and
     * parameters.
     *
     * @param skeleton (not null, in bind pose, unaffected)
     * @param radius the radius of each mesh ring (in mesh units, &gt;0)
     * @param ringsPerSegment the number of mesh rings in each tube segment
     * (&ge;1)
     * @param samplesPerRing the number of samples in each mesh ring (&ge;3)
     */
    public TubeTreeMesh(Skeleton skeleton, float radius, int ringsPerSegment,
            int samplesPerRing) {
        Validate.nonNull(skeleton, "skeleton");
        Validate.positive(radius, "radius");
        Validate.positive(ringsPerSegment, "rings per segment");
        Validate.inRange(samplesPerRing, "samples per ring", 3, Integer.MAX_VALUE);

        this.skeleton = (Skeleton) Misc.deepCopy(skeleton);
        this.radius = radius;
        this.ringsPerSegment = ringsPerSegment;
        this.samplesPerRing = samplesPerRing;

        setMaxNumWeights(2);
        updateAll();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Enumerate the indices of all cap vertices for the named bone.
     *
     * @param boneName the name of the bone
     * @return a new BitSet of vertex indices (not null)
     */
    public BitSet listCapVertices(String boneName) {
        Bone bone = skeleton.getBone(boneName);
        if (bone == null) {
            String message = "no such bone: " + MyString.quote(boneName);
            throw new IllegalArgumentException(message);
        }

        int numBones = skeleton.getBoneCount();
        int trianglesPerCap = samplesPerRing - 2;
        int trianglesPerRing = 2 * samplesPerRing;
        int trianglesPerSegment = trianglesPerRing * ringsPerSegment;

        BitSet result = new BitSet(numVertices);
        int triCount = 0;

        for (int i = 0; i < numBones; ++i) {
            Bone child = skeleton.getBone(i);
            Bone parent = child.getParent();
            if (parent != null) {
                triCount += trianglesPerSegment;
                if (child.getChildren().size() != 1) {
                    if (child == bone) {
                        int fromIndex = vpt * triCount;
                        int toIndex = vpt * (triCount + trianglesPerCap);
                        result.set(fromIndex, toIndex);
                    }
                    triCount += trianglesPerCap;
                }
                Bone grandparent = parent.getParent();
                if (grandparent == null || parent.getChildren().size() > 1) {
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
        int trianglesPerCap = samplesPerRing - 2;
        int result = vpt * trianglesPerCap;

        return result;
    }
    // *************************************************************************
    // Mesh methods

    /**
     * De-serialize this mesh, for example when loading from a J3O file.
     *
     * @param importer the importer (not null)
     * @throws IOException from importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);
        InputCapsule capsule = importer.getCapsule(this);

        radius = capsule.readFloat("radius", 1f);
        ringsPerSegment = capsule.readInt("ringsPerSegment", 8);
        samplesPerRing = capsule.readInt("samplesPerRing", 12);
        skeleton = (Skeleton) capsule.readSavable("skeleton", null);
        /*
         * Recalculate the derived properties.
         */
        updateDerivedProperties();
    }

    /**
     * Serialize this mesh, for example when saving to a J3O file.
     *
     * @param exporter the exporter (not null)
     * @throws IOException from exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        super.write(exporter);
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(radius, "radius", 1f);
        capsule.write(ringsPerSegment, "ringsPerTube", 8);
        capsule.write(samplesPerRing, "samplesPerRing", 12);
        capsule.write(skeleton, "skeleton", null);
    }
    // *************************************************************************
    // private methods

    /*
     * Allocate new buffers for mesh data.
     */
    private void allocateBuffers() {
        int weightCount = maxWpv * numVertices;
        // TODO use a ByteBuffer if possible
        boneIndexBuffer = BufferUtils.createShortBuffer(weightCount);
        setBuffer(VertexBuffer.Type.BoneIndex, maxWpv, boneIndexBuffer);
        boneWeightBuffer = BufferUtils.createFloatBuffer(weightCount);
        setBuffer(VertexBuffer.Type.BoneWeight, maxWpv, boneWeightBuffer);

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
     * Pre-compute sample coordinates for a unit circle in the X-Y plane,
     * centered at the origin.
     */
    private void initCircleSamples() {
        circleSamples = new Vector3f[samplesPerRing + 1];
        for (int sampleI = 0; sampleI <= samplesPerRing; ++sampleI) {
            float theta = (2f * FastMath.PI / samplesPerRing) * sampleI;
            float cos = FastMath.cos(theta);
            float sin = FastMath.sin(theta);
            circleSamples[sampleI] = new Vector3f(cos, sin, 0f);
        }
    }

    /**
     * Write bone indices and weights for a triangle animated entirely by a
     * single bone.
     *
     * @param boneIndex the index of the bone that animates the triangle
     */
    private void putAnimationForTriangle(int boneIndex) {
        assert boneIndex >= 0 : boneIndex;
        assert boneIndex <= Short.MAX_VALUE : boneIndex;

        for (int vertexIndex = 0; vertexIndex < vpt; ++vertexIndex) {
            boneIndexBuffer.put((short) boneIndex);
            boneWeightBuffer.put(1f);

            for (int weightIndex = 1; weightIndex < maxWpv; ++weightIndex) {
                boneIndexBuffer.put((short) 0);
                boneWeightBuffer.put(0f);
            }
        }
    }

    /**
     * Write bone indices and weights for a vertex animated by 2 bones.
     *
     * @param boneIndex1 the index of the 1st bone (&ge;0)
     * @param boneIndex2 the index of the 2nd bone (&ge;0)
     * @param weight1 the weight for the 1st bone (&ge;0, &le;1)
     */
    private void putAnimationForVertex(int boneIndex1, int boneIndex2,
            float weight1) {
        assert boneIndex1 >= 0 : boneIndex1;
        assert boneIndex1 <= Short.MAX_VALUE : boneIndex1;
        assert boneIndex2 >= 0 : boneIndex2;
        assert boneIndex2 <= Short.MAX_VALUE : boneIndex2;
        assert boneIndex1 != boneIndex2 : boneIndex1;
        assert weight1 >= 0f : weight1;
        assert weight1 <= 1f : weight1;

        int weightIndex;
        if (weight1 != 0f) {
            boneIndexBuffer.put((short) boneIndex1);
            boneWeightBuffer.put(weight1);
            weightIndex = 2;
        } else {
            weightIndex = 1;
        }

        boneIndexBuffer.put((short) boneIndex2);
        boneWeightBuffer.put(1f - weight1);

        while (weightIndex < maxWpv) {
            boneIndexBuffer.put((short) 0);
            boneWeightBuffer.put(0f);
            ++weightIndex;
        }
    }

    /**
     * Write a circular cap to the buffers. TODO more cap-shape options
     *
     * @param centerPos the position of the center of the cap (in mesh
     * coordinates, not null, unaffected)
     * @param orientation the orientation of the cap (cap lies in the X-Y plane)
     * (in mesh coordinates, not null, unaffected)
     * @param posZ the local Z-component of the position vector (+1 or -1)
     * @param normalZ the local Z-component of the normal direction (+1 or -1)
     * @param boneIndex (&ge;0)
     */
    private void putCap(Vector3f centerPos, Quaternion orientation, float posZ,
            float normalZ, int boneIndex) {
        Vector3f a = reusable[0];
        Vector3f b = reusable[1];
        Vector3f c = reusable[2];
        int startOffset = positionBuffer.position();
        /*
         * Put a triangle for each sample point except the 1st and last.
         */
        for (int triIndex = 1; triIndex < samplesPerRing - 1; ++triIndex) {
            Vector3f pos1 = circleSamples[0];
            Vector3f pos2, pos3;
            if (normalZ > 0f) {
                pos2 = circleSamples[triIndex];
                pos3 = circleSamples[triIndex + 1];
            } else {
                pos2 = circleSamples[triIndex + 1];
                pos3 = circleSamples[triIndex];
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
        int numFloats = positionBuffer.position() - startOffset;
        int numCapTriangles = samplesPerRing - 2;
        assert numFloats == numAxes * vpt * numCapTriangles;
    }

    /**
     * Rotate and translate each vertex of a triangle and write the transformed
     * coordinates to the specified buffer.
     *
     * @param buffer the buffer to write to (not null, modified)
     * @param offset the translation to apply (not null, unaffected)
     * @param rotation the rotation to apply (not null, unaffected)
     * @param v1 the 1st vertex vector to transform (not null, modified)
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
     * @param startCenter the position of the center of the tube's start (in
     * mesh coordinates, not null, unaffected)
     * @param orientation the orientation of the segment (in mesh space, the
     * local +Z axis being the length axis, not null, unaffected)
     * @param length the length of the segment (in mesh units)
     * @param startBoneIndex (&ge;0)
     * @param endBoneIndex (&ge;0)
     */
    private void putTube(Vector3f startCenter, Quaternion orientation,
            float length, int startBoneIndex, int endBoneIndex) {
        assert startBoneIndex != endBoneIndex;

        Vector3f pos11 = reusable[0];
        Vector3f pos12 = reusable[1];
        Vector3f pos21 = reusable[2];
        Vector3f pos22 = reusable[3];
        Vector3f norm11 = reusable[4];
        Vector3f norm12 = reusable[5];
        Vector3f norm21 = reusable[6];
        Vector3f norm22 = reusable[7];

        float fractionStep = 1f / ringsPerSegment;
        for (int ringIndex = 0; ringIndex < ringsPerSegment; ++ringIndex) {
            float fraction1 = fractionStep * ringIndex;
            float fraction2 = fractionStep * (ringIndex + 1);
            float z1 = fraction1 * length;
            float z2 = fraction2 * length;

            // a rectangular patch for each sample point
            for (int rectIndex = 0; rectIndex < samplesPerRing; ++rectIndex) {
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
                putTransformedTriangle(positionBuffer, startCenter,
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
                putTransformedTriangle(positionBuffer, startCenter,
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
                Vector3f tubeStart = child.getModelSpacePosition();
                Vector3f tubeEnd = parent.getModelSpacePosition();
                Vector3f tubeOffset = tubeEnd.subtract(tubeStart);
                float tubeLength = tubeOffset.length();

                Vector3f direction = tubeOffset.clone();
                Vector3f axis1 = new Vector3f();
                Vector3f axis2 = new Vector3f();
                MyVector3f.generateBasis(direction, axis1, axis2);
                Quaternion orientation = new Quaternion();
                orientation.fromAxes(axis1, axis2, direction);

                putTube(tubeStart, orientation, tubeLength, childIndex,
                        parentIndex);

                if (child.getChildren().size() != 1) {
                    // Cap the child's end of the tube.
                    putCap(tubeStart, orientation, 0f, -1f, childIndex);
                }

                Bone grandparent = parent.getParent();
                if (grandparent == null || parent.getChildren().size() > 1) {
                    // Cap the parent's end of the tube.
                    putCap(tubeStart, orientation, tubeLength, 1f, parentIndex);
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
        for (int boneIndex = 0; boneIndex < numBones; ++boneIndex) {
            Bone child = skeleton.getBone(boneIndex);
            Bone parent = child.getParent();
            if (parent != null) {
                ++numSegments;
                if (child.getChildren().size() != 1) {
                    ++numCaps;
                }
                Bone grandparent = parent.getParent();
                if (grandparent == null || parent.getChildren().size() > 1) {
                    ++numCaps;
                }
            }
        }

        int trianglesPerCap = samplesPerRing - 2;
        int trianglesPerRing = 2 * samplesPerRing;
        int trianglesPerSegment = trianglesPerRing * ringsPerSegment;
        numTriangles
                = trianglesPerCap * numCaps + trianglesPerSegment * numSegments;
        logger.log(Level.INFO, "{0} triangles", numTriangles);

        numVertices = vpt * numTriangles;
        logger.log(Level.INFO, "{0} vertices", numVertices);
        assert numVertices <= Short.MAX_VALUE : numVertices;
    }
}
