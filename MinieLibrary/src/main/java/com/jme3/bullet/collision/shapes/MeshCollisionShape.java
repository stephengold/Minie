/*
 * Copyright (c) 2009-2018 jMonkeyEngine
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
package com.jme3.bullet.collision.shapes;

import com.jme3.bullet.util.NativeMeshUtil;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer.Type;
import com.jme3.scene.mesh.IndexBuffer;
import com.jme3.util.BufferUtils;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * A mesh CollisionShape based on Bullet's btBvhTriangleMeshShape. TODO add
 * shape based on btScaledBvhTriangleMeshShape
 *
 * @author normenhansen
 */
public class MeshCollisionShape extends CollisionShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(MeshCollisionShape.class.getName());
    /**
     * field names for serialization
     */
    private static final String NATIVE_BVH = "nativeBvh";
    private static final String NUM_TRIANGLES = "numTriangles";
    private static final String NUM_VERTICES = "numVertices";
    private static final String TRIANGLE_INDEX_BASE = "triangleIndexBase";
    private static final String TRIANGLE_INDEX_STRIDE = "triangleIndexStride";
    private static final String USE_COMPRESSION = "useCompression";
    private static final String VERTEX_BASE = "vertexBase";
    private static final String VERTEX_STRIDE = "vertexStride";
    // *************************************************************************
    // fields

    /**
     * if true, use quantized AABB compression (default=true)
     */
    private boolean useCompression;

    private ByteBuffer triangleIndexBase;
    private ByteBuffer vertexBase;

    private int numTriangles;
    private int numVertices;
    private int triangleIndexStride;
    private int vertexStride;
    /**
     * Unique identifier of the Bullet mesh. The constructor sets this to a
     * non-zero value.
     */
    private long meshId = 0L;
    private long nativeBVHBuffer = 0L;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public MeshCollisionShape() {
    }

    /**
     * An advanced constructor. Passing incorrect values can lead to a crash.
     * Usually you don't want to use this. Use at own risk.
     * <p>
     * This constructor bypasses all copy logic normally used, this allows for
     * faster collision shape generation when using procedurally generated
     * meshes.
     *
     * @param indices the raw index buffer
     * @param vertices the raw vertex buffer
     * @param useCompression use quantized AABB compression: less memory but
     * slower
     */
    public MeshCollisionShape(ByteBuffer indices, ByteBuffer vertices,
            boolean useCompression) {
        triangleIndexBase = indices;
        vertexBase = vertices;
        numVertices = vertices.limit() / 4 / 3;
        numTriangles = triangleIndexBase.limit() / 4 / 3;
        vertexStride = 12;
        triangleIndexStride = 12;
        this.useCompression = useCompression;

        createShape(null);
    }

    /**
     * Instantiate a shape based on the specified JME mesh, using quantized AABB
     * compression.
     *
     * @param mesh the mesh on which to base the shape (not null)
     */
    public MeshCollisionShape(Mesh mesh) {
        this(mesh, true);
    }

    /**
     * Instantiate a shape based on the specified JME mesh.
     *
     * @param mesh the mesh on which to base the shape (not null)
     * @param useCompression true to use quantized AABB compression
     */
    public MeshCollisionShape(Mesh mesh, boolean useCompression) {
        this.useCompression = useCompression;
        createCollisionMesh(mesh);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Count how many vertices are in the mesh.
     *
     * @return the count (&gt;0)
     */
    public int countMeshVertices() {
        assert numVertices > 0 : numVertices;
        return numVertices;
    }
    // *************************************************************************
    // CollisionShape methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned shape into a deep-cloned one, using the specified cloner
     * and original to resolve copied fields.
     *
     * @param cloner the cloner that's cloning this shape (not null)
     * @param original the instance from which this instance was shallow-cloned
     * (unused)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);

        triangleIndexBase.position(0);
        byte[] copyIndices = new byte[triangleIndexBase.limit()];
        triangleIndexBase.get(copyIndices);
        triangleIndexBase = BufferUtils.createByteBuffer(copyIndices);

        vertexBase.position(0);
        byte[] copyVertices = new byte[vertexBase.limit()];
        vertexBase.get(copyVertices);
        vertexBase = BufferUtils.createByteBuffer(copyVertices);

        meshId = 0L;
        nativeBVHBuffer = 0L;

        createShape(null);
    }

    /**
     * Finalize this shape just before it is destroyed. Should be invoked only
     * by a subclass or by the garbage collector.
     *
     * @throws Throwable ignored by the garbage collector
     */
    @Override
    protected void finalize() throws Throwable {
        super.finalize();
        logger2.log(Level.FINE, "Finalizing Mesh {0}", Long.toHexString(meshId));
        if (meshId != 0L) {
            finalizeNative(meshId, nativeBVHBuffer);
        }
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public MeshCollisionShape jmeClone() {
        try {
            MeshCollisionShape clone = (MeshCollisionShape) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }

    /**
     * De-serialize this shape, for example when loading from a J3O file.
     *
     * @param im importer (not null)
     * @throws IOException from importer
     */
    @Override
    public void read(JmeImporter im) throws IOException {
        super.read(im);
        InputCapsule capsule = im.getCapsule(this);

        numVertices = capsule.readInt(NUM_VERTICES, 0);
        numTriangles = capsule.readInt(NUM_TRIANGLES, 0);
        vertexStride = capsule.readInt(VERTEX_STRIDE, 12);
        triangleIndexStride = capsule.readInt(TRIANGLE_INDEX_STRIDE, 12);
        useCompression = capsule.readBoolean(USE_COMPRESSION, true);

        triangleIndexBase = BufferUtils.createByteBuffer(
                capsule.readByteArray(TRIANGLE_INDEX_BASE, null));
        vertexBase = BufferUtils.createByteBuffer(
                capsule.readByteArray(VERTEX_BASE, null));

        byte[] nativeBvh = capsule.readByteArray(NATIVE_BVH, null);
        createShape(nativeBvh);
    }

    /**
     * Serialize this shape, for example when saving to a J3O file.
     *
     * @param ex exporter (not null)
     * @throws IOException from exporter
     */
    @Override
    public void write(JmeExporter ex) throws IOException {
        super.write(ex);
        OutputCapsule capsule = ex.getCapsule(this);

        capsule.write(numVertices, NUM_VERTICES, 0);
        capsule.write(numTriangles, NUM_TRIANGLES, 0);
        capsule.write(vertexStride, VERTEX_STRIDE, 12);
        capsule.write(triangleIndexStride, TRIANGLE_INDEX_STRIDE, 12);
        capsule.write(useCompression, USE_COMPRESSION, true);

        triangleIndexBase.position(0);
        byte[] copyIndices = new byte[triangleIndexBase.limit()];
        triangleIndexBase.get(copyIndices);
        capsule.write(copyIndices, TRIANGLE_INDEX_BASE, null);

        vertexBase.position(0);
        byte[] copyVertices = new byte[vertexBase.limit()];
        vertexBase.get(copyVertices);
        capsule.write(copyVertices, VERTEX_BASE, null);

        byte[] data = saveBVH(objectId);
        capsule.write(data, NATIVE_BVH, null);
    }
    // *************************************************************************
    // private methods

    private void createCollisionMesh(Mesh mesh) {
        assert mesh.getMode() == Mesh.Mode.Triangles; // TODO other modes

        triangleIndexBase = BufferUtils.createByteBuffer(mesh.getTriangleCount() * 3 * 4);
        vertexBase = BufferUtils.createByteBuffer(mesh.getVertexCount() * 3 * 4);
        numVertices = mesh.getVertexCount();
        vertexStride = 12; // 3 verts * 4 bytes per.
        numTriangles = mesh.getTriangleCount();
        triangleIndexStride = 12; // 3 index entries * 4 bytes each.

        IndexBuffer indices = mesh.getIndicesAsList();
        FloatBuffer vertices = mesh.getFloatBuffer(Type.Position);
        vertices.rewind();

        int verticesLength = mesh.getVertexCount() * 3;
        for (int i = 0; i < verticesLength; ++i) {
            float tempFloat = vertices.get();
            vertexBase.putFloat(tempFloat);
        }

        int indicesLength = mesh.getTriangleCount() * 3;
        for (int i = 0; i < indicesLength; ++i) {
            triangleIndexBase.putInt(indices.get(i));
        }
        vertices.rewind();
        vertices.clear();

        createShape(null);
    }

    /**
     * Instantiate the configured shape in Bullet.
     *
     * @param bvh built BVH data, or null if the BVH has not yet been built
     */
    private void createShape(byte bvh[]) {
        assert meshId == 0L;
        assert nativeBVHBuffer == 0L;

        meshId = NativeMeshUtil.createTriangleIndexVertexArray(
                triangleIndexBase, vertexBase, numTriangles,
                numVertices, vertexStride, triangleIndexStride);
        assert meshId != 0L;
        logger2.log(Level.FINE, "Created Mesh {0}", Long.toHexString(meshId));

        boolean buildBvh = (bvh == null || bvh.length == 0);
        objectId = createShape(useCompression, buildBvh, meshId);
        logger2.log(Level.FINE, "Created Shape {0}",
                Long.toHexString(objectId));
        if (!buildBvh) {
            nativeBVHBuffer = setBVH(bvh, objectId);
            assert nativeBVHBuffer != 0L;
        }

        setScale(scale);
        setMargin(margin);
    }

    native private long createShape(boolean useCompression, boolean buildBvh,
            long meshId);

    native private void finalizeNative(long objectId, long nativeBVHBuffer);

    native private byte[] saveBVH(long objectId);

    /**
     * Read the ID of the native buffer used by the in-place de-serialized
     * shape. The buffer must be freed when no longer used.
     */
    native private long setBVH(byte[] buffer, long objectid);
}
