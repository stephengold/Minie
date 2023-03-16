/*
 * Copyright (c) 2019-2023 jMonkeyEngine
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of 'jMonkeyEngine' nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package com.jme3.bullet.collision.shapes.infos;

import com.jme3.bullet.NativePhysicsObject;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.math.Plane;
import com.jme3.math.Transform;
import com.jme3.math.Triangle;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.mesh.IndexBuffer;
import com.jme3.util.BufferUtils;
import com.jme3.util.clone.Cloner;
import com.jme3.util.clone.JmeCloneable;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.nio.ShortBuffer;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.MyMesh;
import jme3utilities.Validate;
import jme3utilities.math.DistinctVectorValues;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;

/**
 * An indexed triangle mesh based on Bullet's {@code btIndexedMesh}. Immutable
 * except for {@link #read(com.jme3.export.JmeImporter)}.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class IndexedMesh
        extends NativePhysicsObject
        implements JmeCloneable, Savable {
    // *************************************************************************
    // constants and loggers

    /**
     * number of bytes in a float
     */
    final private static int floatBytes = 4;
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
    final public static Logger logger
            = Logger.getLogger(IndexedMesh.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagIndexInts = "indexInts";
    final private static String tagIndexStride = "indexStride";
    final private static String tagNumTriangles = "numTriangles";
    final private static String tagNumVertices = "numVertices";
    final private static String tagVertexStride = "vertexStride";
    final private static String tagVertices = "vertices";
    // *************************************************************************
    // fields

    /**
     * configured position data: 3 floats per vertex (not null, direct, never
     * flipped)
     */
    private FloatBuffer vertexPositions;
    /**
     * configured index data (not null, direct, never flipped)
     */
    private IndexBuffer indices;
    /**
     * configured bytes per triangle in the index buffer (3, 6, or 12)
     */
    private int indexStride;
    /**
     * configured number of triangles in the mesh (&ge;0)
     */
    private int numTriangles;
    /**
     * configured number of vertices in the mesh (&ge;0)
     */
    private int numVertices;
    /**
     * configured bytes per vertex in the position buffer (12)
     */
    private int vertexStride;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected IndexedMesh() {
    }

    /**
     * Instantiate an IndexedMesh from the debug mesh of the specified
     * CollisionShape.
     *
     * @param shape the input shape (not null, unaffected)
     * @param resolution ignored for concave shapes (0=low, 1=high)
     * @param dedup true&rarr;deduplicate vertices, false&rarr;don't deduplicate
     */
    public IndexedMesh(CollisionShape shape, int resolution, boolean dedup) {
        Validate.nonNull(shape, "shape");
        Validate.inRange(resolution, "resolution",
                DebugShapeFactory.lowResolution,
                DebugShapeFactory.highResolution);

        FloatBuffer positionBuffer
                = DebugShapeFactory.getDebugTriangles(shape, resolution);
        Mesh jmeMesh = new Mesh();
        jmeMesh.setBuffer(VertexBuffer.Type.Position, numAxes, positionBuffer);

        if (dedup) {
            jmeMesh = MyMesh.addIndices(jmeMesh);
        }
        create(jmeMesh, null);
    }

    /**
     * Instantiate an IndexedMesh based on the specified JME mesh, without
     * transforming coordinates.
     *
     * @param jmeMesh the input JME mesh (not null, unaffected,
     * mode=Triangles/TriangleFan/TriangleStrip)
     */
    public IndexedMesh(Mesh jmeMesh) {
        Validate.nonNull(jmeMesh, "JME mesh");
        Validate.require(MyMesh.hasTriangles(jmeMesh),
                "mode=Triangles/TriangleFan/TriangleStrip");

        create(jmeMesh, null);
    }

    /**
     * Instantiate an IndexedMesh based on the specified JME mesh and coordinate
     * transform.
     *
     * @param jmeMesh the input JME mesh (not null, unaffected,
     * mode=Triangles/TriangleFan/TriangleStrip)
     * @param transform the Transform to apply to vertex positions (unaffected)
     * or null to use untransformed vertex positions
     */
    public IndexedMesh(Mesh jmeMesh, Transform transform) {
        Validate.nonNull(jmeMesh, "JME mesh");
        Validate.require(MyMesh.hasTriangles(jmeMesh),
                "mode=Triangles/TriangleFan/TriangleStrip");
        Validate.nonNull(transform, "transform");

        create(jmeMesh, transform);
    }

    /**
     * Instantiate an IndexedMesh based on the specified positions and indices.
     *
     * @param positionArray (not null, unaffected)
     * @param indexArray (not null, unaffected, length a multiple of 3)
     */
    public IndexedMesh(Vector3f[] positionArray, int[] indexArray) {
        Validate.nonNull(positionArray, "position array");
        Validate.nonNull(indexArray, "index array");
        int numIndices = indexArray.length;
        Validate.require(numIndices % vpt == 0, "length a multiple of 3");

        this.numVertices = positionArray.length;
        this.vertexPositions = BufferUtils.createFloatBuffer(positionArray);
        this.vertexStride = numAxes * floatBytes;

        this.numTriangles = numIndices / vpt;
        IntBuffer buffer = BufferUtils.createIntBuffer(indexArray);
        this.indices = IndexBuffer.wrapIndexBuffer(buffer);
        int indexBytes = indices.getFormat().getComponentSize();
        this.indexStride = vpt * indexBytes;

        createMesh();
    }

    /**
     * Instantiate an IndexedMesh based on the specified vertex positions. An
     * index will be assigned to each distinct position.
     *
     * @param buffer the vertex positions of a non-indexed triangle mesh (not
     * null, flipped, limit a multiple of 9, unaffected)
     */
    public IndexedMesh(FloatBuffer buffer) {
        Validate.nonNull(buffer, "buffer");
        int numFloats = buffer.limit();
        Validate.require(numFloats % 9 == 0, "limit a multiple of 9");

        // Assign an index to each distinct vertex position.
        DistinctVectorValues dvv
                = new DistinctVectorValues(buffer, 0, numFloats);

        this.numVertices = dvv.countDistinct();
        this.vertexPositions
                = BufferUtils.createFloatBuffer(numAxes * numVertices);
        this.vertexStride = numAxes * floatBytes;

        int numIndices = numFloats / numAxes;
        this.numTriangles = numIndices / vpt;
        this.indices = IndexBuffer.createIndexBuffer(numVertices, numIndices);
        int indexBytes = indices.getFormat().getComponentSize();
        this.indexStride = vpt * indexBytes;

        Vector3f tmpVector = new Vector3f();
        for (int oldVi = 0; oldVi < numIndices; ++oldVi) {
            int newVi = dvv.findVvid(oldVi);
            assert newVi >= 0 : newVi;
            indices.put(oldVi, newVi);

            int readPosition = numAxes * oldVi;
            MyBuffer.get(buffer, readPosition, tmpVector);
            int writePosition = numAxes * newVi;
            MyBuffer.put(vertexPositions, writePosition, tmpVector);
            // Some vertex positions may be written multiple times!
        }

        createMesh();
    }

    /**
     * Instantiate an IndexedMesh based on the specified positions and indices.
     *
     * @param positionBuffer (not null, not flipped, length a multiple of 3,
     * alias created)
     * @param indexBuffer (not null, not flipped, length a multiple of 3, alias
     * created)
     */
    public IndexedMesh(FloatBuffer positionBuffer, IntBuffer indexBuffer) {
        Validate.nonNull(positionBuffer, "position buffer");
        Validate.nonNull(indexBuffer, "index buffer");
        int numFloats = positionBuffer.capacity();
        Validate.require(numFloats % numAxes == 0, "capacity a multiple of 3");
        int numIndices = indexBuffer.capacity();
        Validate.require(numIndices % vpt == 0, "capacity a multiple of 3");

        this.numVertices = numFloats / numAxes;
        this.vertexPositions = positionBuffer;
        this.vertexStride = numAxes * floatBytes;

        this.numTriangles = numIndices / vpt;
        this.indices = IndexBuffer.wrapIndexBuffer(indexBuffer);
        int indexBytes = indices.getFormat().getComponentSize();
        this.indexStride = vpt * indexBytes;

        createMesh();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the triangle indices.
     *
     * @return a new, direct, unflipped buffer
     */
    public IntBuffer copyIndices() {
        int numInts = indices.size();
        IntBuffer result = BufferUtils.createIntBuffer(numInts);
        for (int bufPos = 0; bufPos < numInts; ++bufPos) {
            int tmpIndex = indices.get(bufPos);
            result.put(tmpIndex);
        }

        return result;
    }

    /**
     * Copy the vertex positions of the specified triangle.
     *
     * @param triangleIndex the index of the source triangle (&ge;0)
     * @param destination storage for the result (not null, modified)
     */
    public void copyTriangle(int triangleIndex, Triangle destination) {
        Validate.inRange(triangleIndex, "triangle index", 0, numTriangles - 1);
        Validate.nonNull(destination, "destination");

        int startPosition = triangleIndex * vpt; // within the indices buffer
        Vector3f tmpVector = new Vector3f();
        for (int vertexI = 0; vertexI < vpt; ++vertexI) {
            int indexPosition = startPosition + vertexI;
            int vertexIndex = indices.get(indexPosition);
            MyBuffer.get(vertexPositions, vertexIndex * numAxes, tmpVector);
            destination.set(vertexI, tmpVector);
        }
    }

    /**
     * Copy the vertex positions to a new buffer.
     *
     * @return a new, direct, unflipped buffer
     */
    public FloatBuffer copyVertexPositions() {
        int numFloats = numVertices * numAxes;
        FloatBuffer result = BufferUtils.createFloatBuffer(numFloats);
        for (int bufPos = 0; bufPos < numFloats; ++bufPos) {
            float tmpFloat = vertexPositions.get(bufPos);
            result.put(tmpFloat);
        }

        return result;
    }

    /**
     * Count how many triangles are in this mesh.
     *
     * @return the count (&ge;0)
     */
    public int countTriangles() {
        assert numTriangles >= 0 : numTriangles;
        return numTriangles;
    }

    /**
     * Count how many vertices are in this mesh.
     *
     * @return the count (&ge;0)
     */
    public int countVertices() {
        assert numVertices >= 0 : numVertices;
        return numVertices;
    }

    /**
     * Find the maximum and minimum coordinates for each axis among the vertices
     * in this mesh.
     *
     * @param storeMaxima storage for the maxima (not null, modified)
     * @param storeMinima storage for the minima (not null, modified)
     */
    public void maxMin(Vector3f storeMaxima, Vector3f storeMinima) {
        Validate.nonNull(storeMaxima, "store maxima");
        Validate.nonNull(storeMinima, "store minima");

        int numFloats = numVertices * numAxes;
        MyBuffer.maxMin(
                vertexPositions, 0, numFloats, storeMaxima, storeMinima);
    }

    /**
     * Attempt to divide this mesh into 2 meshes.
     *
     * @param splittingPlane the splitting plane (not null, unaffected)
     * @return a pair of meshes, the first mesh generated by the plane's minus
     * side and the 2nd mesh generated by its plus side; either mesh may be
     * null, indicating an empty mesh
     */
    public IndexedMesh[] split(Plane splittingPlane) {
        Validate.nonNull(splittingPlane, "splitting plane");

        FloatBuffer[] buffers = new FloatBuffer[2];
        int cap = 2 * numTriangles * vpt * numAxes;
        buffers[0] = BufferUtils.createFloatBuffer(cap);
        buffers[1] = BufferUtils.createFloatBuffer(cap);
        /*
         * Split each triangle and write the resulting positions to the
         * new buffers, without concern for duplicates.
         */
        Triangle triangle = new Triangle();
        for (int triangleI = 0; triangleI < numTriangles; ++triangleI) {
            copyTriangle(triangleI, triangle);
            splitTriangle(triangle, splittingPlane, buffers);
        }

        IndexedMesh[] result = new IndexedMesh[2];
        int numMinus = buffers[0].position();
        int numPlus = buffers[1].position();
        if (numMinus == 0 || numPlus == 0) {
            // Degenerate case:  all triangles lie to one side of the plane.
            if (numMinus > 0) {
                result[0] = this;
            } else if (numPlus > 0) {
                result[1] = this;
            }

        } else {
            // Convert each buffer to a new mesh.
            for (int sideI = 0; sideI < 2; ++sideI) {
                buffers[sideI].flip();
                result[sideI] = new IndexedMesh(buffers[sideI]);
            }
        }

        return result;
    }
    // *************************************************************************
    // JmeCloneable methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned mesh into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this mesh (not null)
     * @param original the instance from which this mesh was shallow-cloned (not
     * null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        IndexedMesh originalMesh = (IndexedMesh) original;

        int numFloats = vertexPositions.capacity();
        this.vertexPositions = BufferUtils.createFloatBuffer(numFloats);
        for (int offset = 0; offset < numFloats; ++offset) {
            float tmpFloat = originalMesh.vertexPositions.get(offset);
            vertexPositions.put(offset, tmpFloat);
        }

        int numIndices = indices.getBuffer().capacity();
        this.indices = IndexBuffer.createIndexBuffer(numVertices, numIndices);
        for (int offset = 0; offset < numIndices; ++offset) {
            int tmpIndex = originalMesh.indices.get(offset);
            indices.put(offset, tmpIndex);
        }

        unassignNativeObject();
        createMesh();
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public IndexedMesh jmeClone() {
        try {
            IndexedMesh clone = (IndexedMesh) clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }
    // *************************************************************************
    // Savable methods

    /**
     * De-serialize this mesh from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        InputCapsule capsule = importer.getCapsule(this);

        this.indexStride = capsule.readInt(tagIndexStride, 12);
        this.numTriangles = capsule.readInt(tagNumTriangles, 0);
        this.numVertices = capsule.readInt(tagNumVertices, 0);

        this.vertexStride = capsule.readInt(tagVertexStride, 12);
        assert vertexStride == numAxes * floatBytes : vertexStride;

        int[] intArray = capsule.readIntArray(tagIndexInts, new int[0]);
        int numIndices = intArray.length;
        assert numIndices == numTriangles * vpt;
        switch (indexStride) {
            case 3:
                ByteBuffer byteBuf = BufferUtils.createByteBuffer(numIndices);
                indices = IndexBuffer.wrapIndexBuffer(byteBuf);
                break;
            case 6:
                ShortBuffer sBuf = BufferUtils.createShortBuffer(numIndices);
                indices = IndexBuffer.wrapIndexBuffer(sBuf);
                break;
            case 12:
                IntBuffer intBuffer = BufferUtils.createIntBuffer(numIndices);
                indices = IndexBuffer.wrapIndexBuffer(intBuffer);
                break;
            default:
                // invalid J3O?
                throw new RuntimeException("indexStride = " + indexStride);
        }
        for (int offset = 0; offset < numIndices; ++offset) {
            int tmpIndex = intArray[offset];
            indices.put(offset, tmpIndex);
        }

        float[] floatArray = capsule.readFloatArray(tagVertices, new float[0]);
        assert floatArray.length == numVertices * numAxes;
        this.vertexPositions = BufferUtils.createFloatBuffer(floatArray);

        createMesh();
    }

    /**
     * Serialize this mesh to the specified exporter, for example when saving to
     * a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        OutputCapsule capsule = exporter.getCapsule(this);

        int numIndices = numTriangles * vpt;
        int[] intArray = new int[numIndices];
        for (int offset = 0; offset < numIndices; ++offset) {
            intArray[offset] = indices.get(offset);
        }
        capsule.write(intArray, tagIndexInts, null);

        capsule.write(indexStride, tagIndexStride, 12);
        capsule.write(numTriangles, tagNumTriangles, 0);
        capsule.write(numVertices, tagNumVertices, 0);
        capsule.write(vertexStride, tagVertexStride, 12);

        int numFloats = vertexPositions.capacity();
        float[] floatArray = new float[numFloats];
        for (int offset = 0; offset < numFloats; ++offset) {
            floatArray[offset] = vertexPositions.get(offset);
        }
        capsule.write(floatArray, tagVertices, null);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Configure and create a new {@code btIndexedMesh} from the specified JME
     * mesh and Transform.
     *
     * @param jmeMesh the input JME mesh (not null, unaffected,
     * mode=Triangles/TriangleFan/TriangleStrip)
     * @param transform the Transform to apply to vertex positions (unaffected)
     * or null to use untransformed vertex positions
     */
    private void create(Mesh jmeMesh, Transform transform) {
        assert MyMesh.hasTriangles(jmeMesh);

        this.numVertices = jmeMesh.getVertexCount();
        if (numVertices <= 0) {
            this.numVertices = 0;
        }

        FloatBuffer meshVs = jmeMesh.getFloatBuffer(VertexBuffer.Type.Position);
        int numFloats = numAxes * numVertices;
        this.vertexPositions = BufferUtils.createFloatBuffer(numFloats);
        for (int offset = 0; offset < numFloats; ++offset) {
            float temp = meshVs.get(offset);
            vertexPositions.put(offset, temp);
        }
        this.vertexStride = numAxes * floatBytes;

        if (transform != null && !MyMath.isIdentity(transform)) {
            MyBuffer.transform(vertexPositions, 0, numFloats, transform);
        }

        this.numTriangles = jmeMesh.getTriangleCount();
        if (numTriangles <= 0) {
            this.numTriangles = 0;
        }
        int numIndices = vpt * numTriangles;

        this.indices = IndexBuffer.createIndexBuffer(numVertices, numIndices);

        IndexBuffer triangleIndices = jmeMesh.getIndicesAsList();
        for (int offset = 0; offset < numIndices; ++offset) {
            int index = triangleIndices.get(offset);
            assert index >= 0 : index;
            assert index < numVertices : index;
            indices.put(offset, index);
        }
        int indexBytes = indices.getFormat().getComponentSize();
        this.indexStride = vpt * indexBytes;

        createMesh();
    }

    /**
     * Create a {@code btIndexedMesh} using the current configuration.
     */
    private void createMesh() {
        assert vertexStride == 12 : vertexStride;

        long meshId;
        switch (indexStride) {
            case 3:
                ByteBuffer byteBuffer = (ByteBuffer) indices.getBuffer();
                meshId = createByte(byteBuffer, vertexPositions, numTriangles,
                        numVertices, vertexStride, indexStride);
                break;
            case 6:
                ShortBuffer shortBuffer = (ShortBuffer) indices.getBuffer();
                meshId = createShort(shortBuffer, vertexPositions,
                        numTriangles, numVertices, vertexStride, indexStride);
                break;
            case 12:
                IntBuffer intBuffer = (IntBuffer) indices.getBuffer();
                meshId = createInt(intBuffer, vertexPositions, numTriangles,
                        numVertices, vertexStride, indexStride);
                break;
            default:
                throw new RuntimeException("indexStride = " + indexStride);
        }
        setNativeId(meshId);

        logger.log(Level.FINE, "Created {0}", this);
    }

    /**
     * Free the identified tracked native object. Invoked by reflection.
     *
     * @param meshId the native identifier (not zero)
     */
    private static void freeNativeObject(long meshId) {
        assert meshId != 0L;
        finalizeNative(meshId);
    }

    /**
     * Write the specified triangle, but only if all 3 vertices are distinct.
     *
     * @param buffer the buffer to write to (not null)
     * @param v1 the first vertex to write (not null, unaffected)
     * @param v2 the 2nd vertex to write (not null, unaffected)
     * @param v3 the 3rd vertex to write (not null, unaffected)
     */
    private static void putTriangle(
            FloatBuffer buffer, Vector3f v1, Vector3f v2, Vector3f v3) {
        if (v1.equals(v2) || v1.equals(v3) || v2.equals(v3)) {
            // The triangle is obviously degenerate.  Skip it.
            return;
        }

        buffer.put(v1.x).put(v1.y).put(v1.z);
        buffer.put(v2.x).put(v2.y).put(v2.z);
        buffer.put(v3.x).put(v3.y).put(v3.z);
    }

    /**
     * Attempt to divide the specified triangle into (up to) 3 triangles.
     *
     * @param input the triangle to be split (not null, unaffected)
     * @param splittingPlane the splitting plane (not null, unaffected)
     * @param putResults the buffers to write the triangles
     */
    private static void splitTriangle(
            Triangle input, Plane splittingPlane, FloatBuffer[] putResults) {
        Vector3f a = input.get1(); // alias
        Vector3f b = input.get2(); // alias
        Vector3f c = input.get3(); // alias
        float aa = splittingPlane.pseudoDistance(a);
        float bb = splittingPlane.pseudoDistance(b);
        float cc = splittingPlane.pseudoDistance(c);

        if (aa == 0f && bb == 0f && cc == 0f) {
            // both sides
            putTriangle(putResults[0], a, b, c);
            putTriangle(putResults[1], a, b, c);

        } else if (aa <= 0f && bb <= 0f && cc <= 0f) {
            // negative side
            putTriangle(putResults[0], a, b, c);

        } else if (aa >= 0f && bb >= 0f && cc >= 0f) {
            // positive side
            putTriangle(putResults[1], a, b, c);

        } else if (aa >= 0f && bb <= 0f && cc <= 0f) {
            // A on the positive side, split AB and AC.
            float tab = aa / (aa - bb);
            Vector3f ab = MyVector3f.lerp(tab, a, b, null);
            float tac = aa / (aa - cc);
            Vector3f ac = MyVector3f.lerp(tac, a, c, null);
            putTriangle(putResults[0], b, c, ac);
            putTriangle(putResults[0], b, ac, ab);
            putTriangle(putResults[1], a, ab, ac);

        } else if (aa <= 0f && bb >= 0f && cc >= 0f) {
            // A on the negative side, split AB and AC.
            float tab = -aa / (bb - aa);
            Vector3f ab = MyVector3f.lerp(tab, a, b, null);
            float tac = -aa / (cc - aa);
            Vector3f ac = MyVector3f.lerp(tac, a, c, null);
            putTriangle(putResults[1], b, c, ac);
            putTriangle(putResults[1], b, ac, ab);
            putTriangle(putResults[0], a, ab, ac);

        } else if (aa <= 0f && bb >= 0f && cc <= 0f) {
            // B on the positive side, split AB and BC.
            float tab = -aa / (bb - aa);
            Vector3f ab = MyVector3f.lerp(tab, a, b, null);
            float tbc = bb / (bb - cc);
            Vector3f bc = MyVector3f.lerp(tbc, b, c, null);
            putTriangle(putResults[0], a, bc, c);
            putTriangle(putResults[0], a, ab, bc);
            putTriangle(putResults[1], b, bc, ab);

        } else if (aa >= 0f && bb <= 0f && cc >= 0f) {
            // B on the negative side, split AB and BC.
            float tab = aa / (aa - bb);
            Vector3f ab = MyVector3f.lerp(tab, a, b, null);
            float tbc = -bb / (cc - bb);
            Vector3f bc = MyVector3f.lerp(tbc, b, c, null);
            putTriangle(putResults[1], a, bc, c);
            putTriangle(putResults[1], a, ab, bc);
            putTriangle(putResults[0], b, bc, ab);

        } else if (aa <= 0f && bb <= 0f && cc >= 0f) {
            // C on the positive side, split AC and BC.
            float tac = -aa / (cc - aa);
            Vector3f ac = MyVector3f.lerp(tac, a, c, null);
            float tbc = -bb / (cc - bb);
            Vector3f bc = MyVector3f.lerp(tbc, b, c, null);
            putTriangle(putResults[0], a, b, bc);
            putTriangle(putResults[0], a, bc, ac);
            putTriangle(putResults[1], ac, bc, c);

        } else {
            assert aa >= 0f : aa;
            assert bb >= 0f : bb;
            assert cc <= 0f : cc;
            // C on the negative side, split AC and BC.
            float tac = aa / (aa - cc);
            Vector3f ac = MyVector3f.lerp(tac, a, c, null);
            float tbc = bb / (bb - cc);
            Vector3f bc = MyVector3f.lerp(tbc, b, c, null);
            putTriangle(putResults[1], a, b, bc);
            putTriangle(putResults[1], a, bc, ac);
            putTriangle(putResults[0], ac, bc, c);
        }
    }
    // *************************************************************************
    // native private methods

    native private static long createByte(ByteBuffer indices,
            FloatBuffer vertexPositions, int numTriangles, int numVertices,
            int vertexStride, int indexStride);

    native private static long createInt(IntBuffer indices,
            FloatBuffer vertexPositions, int numTriangles, int numVertices,
            int vertexStride, int indexStride);

    native private static long createShort(ShortBuffer indices,
            FloatBuffer vertexPositions, int numTriangles, int numVertices,
            int vertexStride, int indexStride);

    native private static void finalizeNative(long meshId);
}
