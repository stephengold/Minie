/*
 * Copyright (c) 2019 jMonkeyEngine
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
import jme3utilities.Validate;

/**
 * A scalable mesh based on Bullet's btStridingMeshInterface.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class StridingMesh implements JmeCloneable, Savable {
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
            = Logger.getLogger(StridingMesh.class.getName());
    /**
     * field names for serialization
     */
    final private static String INDEX_STRIDE = "indexStride";
    final private static String INDICES = "indices";
    final private static String NUM_TRIANGLES = "numTriangles";
    final private static String NUM_VERTICES = "numVertices";
    final private static String SCALE = "scale";
    final private static String VERTEX_STRIDE = "vertexStride";
    final private static String VERTICES = "vertices";
    // *************************************************************************
    // fields

    /**
     * configured position data: 3 floats per vertex (not null)
     */
    private FloatBuffer vertexPositions;
    /**
     * configured bytes per triangle in the index buffer (12)
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
    /**
     * configured index data: 3 ints per triangle (not null)
     */
    private IntBuffer indices;
    /**
     * Unique identifier of the btStridingMeshInterface. The constructor sets
     * this to a non-zero value.
     */
    private long nativeId = 0L;
    /**
     * copy of scaling factors: one for each local axis (default=(1,1,1))
     */
    protected Vector3f scale = new Vector3f(1f, 1f, 1f);
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public StridingMesh() {
    }

    /**
     * Instantiate based on the specified JME mesh.
     *
     * @param mesh the JME mesh (not null, unaffected)
     */
    public StridingMesh(Mesh mesh) {
        Validate.nonNull(mesh, "mesh");
        create(mesh);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Count how many triangles are in this mesh.
     *
     * @return the count (&ge;0)
     */
    public int countTriangles() {
        assert numTriangles >= 0 : numVertices;
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
     * Copy the scaling factors.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the scaling factor for each local axis (either storeResult or a
     * new vector, not null)
     */
    public Vector3f getScale(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        assert checkScale(result);
        result.set(scale);
        
        return result;
    }

    /**
     * Read the ID of the btStridingMeshInterface.
     *
     * @return the unique identifier (not zero)
     */
    public long nativeId() {
        assert nativeId != 0L;
        return nativeId;
    }

    /**
     * Alter the scaling factors.
     *
     * @param scale the desired scaling factor for each local axis (not null, no
     * negative component, unaffected, default=(1,1,1))
     */
    public void setScale(Vector3f scale) {
        Validate.nonNull(scale, "scale");
        assert nativeId != 0L;

        setScaling(nativeId, scale);
        logger.log(Level.FINE, "Scaled StridingMesh {0}",
                Long.toHexString(nativeId));
        this.scale.set(scale);
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
        StridingMesh originalMesh = (StridingMesh) original;

        int numIndices = vpt * numTriangles;
        indices = BufferUtils.createIntBuffer(numIndices);
        for (int offset = 0; offset < numIndices; ++offset) {
            int tmpInt = originalMesh.indices.get(offset);
            indices.put(tmpInt);
        }

        int numFloats = numAxes * numVertices;
        vertexPositions = BufferUtils.createFloatBuffer(numFloats);
        for (int offset = 0; offset < numFloats; ++offset) {
            float tmpFloat = originalMesh.vertexPositions.get(offset);
            vertexPositions.put(tmpFloat);
        }

        scale = cloner.clone(scale);

        nativeId = 0L;
        createTiva();
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public StridingMesh jmeClone() {
        try {
            StridingMesh clone = (StridingMesh) super.clone();
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

        indexStride = capsule.readInt(INDEX_STRIDE, vpt * intSize);

        int[] indexArray = capsule.readIntArray(INDICES, new int[0]);
        indices = BufferUtils.createIntBuffer(indexArray);

        numTriangles = capsule.readInt(NUM_TRIANGLES, 0);
        numVertices = capsule.readInt(NUM_VERTICES, 0);
        scale = (Vector3f) capsule.readSavable(SCALE, new Vector3f(1f, 1f, 1f));
        vertexStride = capsule.readInt(VERTEX_STRIDE, numAxes * floatSize);

        float[] floatArray = capsule.readFloatArray(VERTICES, new float[0]);
        vertexPositions = BufferUtils.createFloatBuffer(floatArray);

        createTiva();
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

        capsule.write(indexStride, INDEX_STRIDE, vpt * intSize);

        int numIndices = vpt * numTriangles;
        int[] indexArray = new int[numIndices];
        for (int offset = 0; offset < numIndices; ++offset) {
            indexArray[offset] = indices.get(offset);
        }
        capsule.write(indexArray, INDICES, null);

        capsule.write(numTriangles, NUM_TRIANGLES, 0);
        capsule.write(numVertices, NUM_VERTICES, 0);
        capsule.write(scale, SCALE, null);
        capsule.write(vertexStride, VERTEX_STRIDE, numAxes * floatSize);

        int numFloats = numAxes * numVertices;
        float[] floatArray = new float[numFloats];
        for (int offset = 0; offset < numFloats; ++offset) {
            floatArray[offset] = vertexPositions.get(offset);
        }
        capsule.write(floatArray, VERTICES, null);
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
        logger.log(Level.FINE, "Finalizing StridingMesh {0}",
                Long.toHexString(nativeId));
        finalizeNative(nativeId);
    }
    // *************************************************************************
    // private methods

    /**
     * Compare Bullet's scaling factors to the local copy.
     *
     * @param tempVector caller-allocated temporary storage (not null)
     * @return true if scaling factors are exactly equal, otherwise false
     */
    private boolean checkScale(Vector3f tempVector) {
        assert tempVector != null;

        getScaling(nativeId, tempVector);
        boolean result = scale.equals(tempVector);

        return result;
    }

    /**
     * Configure and create a new instance from the specified JME mesh.
     *
     * @param the JME mesh (not null, unaffected)
     */
    private void create(Mesh mesh) {
        assert mesh.getMode() == Mesh.Mode.Triangles; // TODO other modes

        numVertices = mesh.getVertexCount();
        vertexStride = numAxes * floatSize;

        FloatBuffer meshVs = mesh.getFloatBuffer(VertexBuffer.Type.Position);
        int numFloats = numAxes * numVertices;
        vertexPositions = BufferUtils.createFloatBuffer(numFloats);
        for (int offset = 0; offset < numFloats; ++offset) {
            float temp = meshVs.get(offset);
            meshVs.put(temp);
        }

        numTriangles = mesh.getTriangleCount();
        indexStride = vpt * intSize;

        IndexBuffer meshIs = mesh.getIndicesAsList();
        int numInts = vpt * numTriangles;
        indices = BufferUtils.createIntBuffer(numInts);
        for (int offset = 0; offset < numInts; ++offset) {
            int temp = meshIs.get(offset);
            indices.put(temp);
        }

        createTiva();
    }

    /**
     * Create a new instance from the current configuration.
     */
    private void createTiva() {
        assert nativeId == 0L;

        nativeId = createTiva(indices, vertexPositions, numTriangles,
                numVertices, vertexStride, indexStride);
        assert nativeId != 0L;
        logger.log(Level.FINE, "Created StridingMesh {0}",
                Long.toHexString(nativeId));
    }

    /**
     * Create a new btTriangleIndexVertexArray (that implements
     * btStridingMeshInterface).
     *
     * @param indices (not null, unaffected)
     * @param vertexPositions (not null, unaffected)
     * @param numTriangles the number of triangles in the mesh (&ge;0)
     * @param numVertices the number of vertices in the mesh (&ge;0)
     * @param vertexStride (in bytes, 12)
     * @param indexStride (in bytes, 12)
     * @return the unique identifier of the resulting btTriangleIndexVertexArray
     * (not 0)
     */
    native private long createTiva(IntBuffer indices,
            FloatBuffer vertexPositions, int numTriangles, int numVertices,
            int vertexStride, int indexStride);

    native private void finalizeNative(long nativeId);

    native private void getScaling(long nativeId, Vector3f storeVector);

    native private void setScaling(long nativeId, Vector3f scale);
}
