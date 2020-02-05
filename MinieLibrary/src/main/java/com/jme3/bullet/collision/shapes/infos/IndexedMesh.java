/*
 * Copyright (c) 2019-2020 jMonkeyEngine
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

import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.mesh.IndexBuffer;
import com.jme3.util.BufferUtils;
import com.jme3.util.clone.Cloner;
import com.jme3.util.clone.JmeCloneable;
import java.io.IOException;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.MyMesh;
import jme3utilities.Validate;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyMath;

/**
 * An indexed mesh based on Bullet's btIndexedMesh. Immutable except for
 * {@link #read(com.jme3.export.JmeImporter)}.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class IndexedMesh implements JmeCloneable, Savable {
    // *************************************************************************
    // constants and loggers

    /**
     * number of bytes in a float
     */
    final private static int floatSize = 4;
    /**
     * number of bytes in an int
     */
    final private static int intSize = 4;
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
     * configured position data: typically 3 floats per vertex (not null,
     * direct, never flipped)
     */
    private FloatBuffer vertexPositions;
    /**
     * configured index data: typically 3 ints per triangle (not null, direct,
     * never flipped)
     */
    private IntBuffer indices; // TODO use an IndexBuffer to conserve memory
    /**
     * configured bytes per triangle in the index buffer (typically 12)
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
     * configured bytes per vertex in the position buffer (typically 12)
     */
    private int vertexStride;
    /**
     * Unique identifier of the btIndexedMesh. The constructor sets this to a
     * non-zero value. Once set, the identifier never changes.
     */
    private long nativeId = 0L;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public IndexedMesh() {
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

        vertexPositions = BufferUtils.createFloatBuffer(positionArray);
        indices = BufferUtils.createIntBuffer(indexArray);
        indexStride = vpt * intSize;
        numTriangles = indexArray.length / vpt;
        numVertices = positionArray.length;
        vertexStride = numAxes * floatSize;

        createMesh();
    }
    // *************************************************************************
    // new methods exposed

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
     * Read the ID of the btIndexedMesh.
     *
     * @return the unique identifier (not zero)
     */
    public long nativeId() {
        assert nativeId != 0L;
        return nativeId;
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
        vertexPositions = BufferUtils.createFloatBuffer(numFloats);

        for (int offset = 0; offset < numFloats; ++offset) {
            float tmpFloat = originalMesh.vertexPositions.get(offset);
            vertexPositions.put(tmpFloat);
        }

        int numIndices = indices.capacity();
        indices = BufferUtils.createIntBuffer(numIndices);
        for (int offset = 0; offset < numIndices; ++offset) {
            int tmpIndex = originalMesh.indices.get(offset);
            indices.put(tmpIndex);
        }

        nativeId = 0L;
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
            IndexedMesh clone = (IndexedMesh) super.clone();
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

        indexStride = capsule.readInt(tagIndexStride, 12);
        numTriangles = capsule.readInt(tagNumTriangles, 0);
        numVertices = capsule.readInt(tagNumVertices, 0);
        vertexStride = capsule.readInt(tagVertexStride, numAxes * floatSize);

        int[] intArray = capsule.readIntArray(tagIndexInts, new int[0]);
        assert intArray.length == numTriangles * vpt;
        indices = BufferUtils.createIntBuffer(intArray);

        float[] floatArray = capsule.readFloatArray(tagVertices, new float[0]);
        assert floatArray.length == numVertices * vpt;
        vertexPositions = BufferUtils.createFloatBuffer(floatArray);

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

        int numIndices = indices.capacity();
        int[] intArray = new int[numIndices];
        for (int offset = 0; offset < numIndices; ++offset) {
            intArray[offset] = indices.get(offset);
        }
        capsule.write(intArray, tagIndexInts, null);

        capsule.write(indexStride, tagIndexStride, 12);
        capsule.write(numTriangles, tagNumTriangles, 0);
        capsule.write(numVertices, tagNumVertices, 0);
        capsule.write(vertexStride, tagVertexStride, numAxes * floatSize);

        int numFloats = vertexPositions.capacity();
        float[] floatArray = new float[numFloats];
        for (int offset = 0; offset < numFloats; ++offset) {
            floatArray[offset] = vertexPositions.get(offset);
        }
        capsule.write(floatArray, tagVertices, null);
    }
    // *************************************************************************
    // Object methods

    /**
     * Finalize this mesh just before it is destroyed. Should be invoked only by
     * a subclass or by the garbage collector.
     *
     * @throws Throwable ignored by the garbage collector
     */
    @Override
    protected void finalize() throws Throwable {
        super.finalize();
        logger.log(Level.FINE, "Finalizing IndexedMesh {0}",
                Long.toHexString(nativeId));
        finalizeNative(nativeId);
    }
    // *************************************************************************
    // private methods

    /**
     * Configure and create a new btIndexedMesh from the specified JME mesh and
     * Transform.
     *
     * @param jmeMesh the input JME mesh (not null, unaffected,
     * mode=Triangles/TriangleFan/TriangleStrip)
     * @param transform the Transform to apply to vertex positions (unaffected)
     * or null to use untransformed vertex positions
     */
    private void create(Mesh jmeMesh, Transform transform) {
        assert MyMesh.hasTriangles(jmeMesh);

        numVertices = jmeMesh.getVertexCount();
        vertexStride = numAxes * floatSize;

        FloatBuffer meshVs = jmeMesh.getFloatBuffer(VertexBuffer.Type.Position);
        int numFloats = numAxes * numVertices;
        vertexPositions = BufferUtils.createFloatBuffer(numFloats);
        for (int position = 0; position < numFloats; ++position) {
            float temp = meshVs.get(position);
            vertexPositions.put(position, temp);
        }

        if (transform != null && !MyMath.isIdentity(transform)) {
            MyBuffer.transform(vertexPositions, 0, numFloats, transform);
        }

        indexStride = vpt * intSize;
        numTriangles = jmeMesh.getTriangleCount();

        int numIndices = vpt * numTriangles;
        indices = BufferUtils.createIntBuffer(numIndices);

        IndexBuffer triangleIndices = jmeMesh.getIndicesAsList();
        for (int position = 0; position < numIndices; ++position) {
            int index = triangleIndices.get(position);
            assert index >= 0 : index;
            assert index < numVertices : index;
            indices.put(position, index);
        }

        createMesh();
    }

    /**
     * Create a new btIndexedMesh using the current configuration.
     */
    private void createMesh() {
        assert nativeId == 0L;

        nativeId = createInt(indices, vertexPositions, numTriangles,
                numVertices, vertexStride, indexStride);
        assert nativeId != 0L;
        logger.log(Level.FINE, "Created IndexedMesh {0}",
                Long.toHexString(nativeId));
    }
    // *************************************************************************
    // native methods

    native private long createInt(IntBuffer indices,
            FloatBuffer vertexPositions, int numTriangles, int numVertices,
            int vertexStride, int indexStride);

    native private void finalizeNative(long nativeId);
}
