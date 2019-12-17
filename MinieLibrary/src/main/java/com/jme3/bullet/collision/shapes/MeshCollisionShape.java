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

import com.jme3.bullet.collision.shapes.infos.CompoundMesh;
import com.jme3.bullet.collision.shapes.infos.IndexedMesh;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.scene.Mesh;
import com.jme3.system.JmeSystem;
import com.jme3.system.Platform;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Validate;

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
    final private static String tagNativeBvh = "nativeBvh";
    final private static String tagNativePlatform = "nativePlatform";
    final private static String tagNativeMesh = "nativeMesh";
    final private static String tagUseCompression = "useCompression";
    // *************************************************************************
    // fields

    /**
     * if true, use quantized AABB compression (default=true)
     */
    private boolean useCompression;
    /**
     * native mesh used to construct this shape
     */
    private CompoundMesh nativeMesh;
    /**
     * unique identifier of the native BVH data
     */
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
     * Instantiate a shape based on the specified native mesh(es).
     *
     * @param submeshes the mesh(es) on which to base the shape (not null, not
     * empty)
     * @param useCompression true to use quantized AABB compression
     */
    public MeshCollisionShape(boolean useCompression,
            IndexedMesh... submeshes) {
        Validate.nonEmpty(submeshes, "submeshes");

        this.useCompression = useCompression;
        nativeMesh = new CompoundMesh();
        for (IndexedMesh submesh : submeshes) {
            nativeMesh.add(submesh);
        }
        createShape(null);
    }

    /**
     * Instantiate a shape based on the specified JME mesh(es), using quantized
     * AABB compression.
     *
     * @param jmeMeshes the mesh(es) on which to base the shape (not null, not
     * empty, unaffected)
     */
    public MeshCollisionShape(Mesh... jmeMeshes) {
        Validate.nonEmpty(jmeMeshes, "JME meshes");

        useCompression = true;
        nativeMesh = new CompoundMesh(jmeMeshes);
        createShape(null);
    }

    /**
     * Instantiate a shape based on the specified JME mesh.
     *
     * @param mesh the mesh on which to base the shape (not null, unaffected)
     * @param useCompression true to use quantized AABB compression
     */
    public MeshCollisionShape(Mesh mesh, boolean useCompression) {
        Validate.nonNull(mesh, "mesh");

        this.useCompression = useCompression;
        nativeMesh = new CompoundMesh(mesh);
        createShape(null);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Count how many triangles are in the mesh.
     *
     * @return the count (&ge;0)
     */
    public int countMeshTriangles() {
        int result = nativeMesh.countTriangles();
        return result;
    }

    /**
     * Count how many vertices are in the mesh.
     *
     * @return the count (&ge;0)
     */
    public int countMeshVertices() {
        int numVertices = nativeMesh.countVertices();
        return numVertices;
    }
    // *************************************************************************
    // CollisionShape methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned shape into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this shape (not null)
     * @param original the instance from which this shape was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);

        nativeMesh = cloner.clone(nativeMesh);
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
        if (nativeBVHBuffer != 0L) {
            finalizeBVH(nativeBVHBuffer);
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
     * De-serialize this shape from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);
        InputCapsule capsule = importer.getCapsule(this);

        byte[] nativeBvh = capsule.readByteArray(tagNativeBvh, null);
        Platform writePlatform
                = capsule.readEnum(tagNativePlatform, Platform.class, null);
        if (writePlatform == null || writePlatform != JmeSystem.getPlatform()) {
            nativeBvh = null; // will re-create the BVH for the new platform
        }

        nativeMesh = (CompoundMesh) capsule.readSavable(tagNativeMesh, null);
        useCompression = capsule.readBoolean(tagUseCompression, true);

        createShape(nativeBvh);
    }

    /**
     * Recalculate this shape's bounding box if necessary.
     */
    @Override
    protected void recalculateAabb() {
        long shapeId = getObjectId();
        recalcAabb(shapeId);
    }

    /**
     * Serialize this shape to the specified exporter, for example when saving
     * to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        super.write(exporter);
        OutputCapsule capsule = exporter.getCapsule(this);

        long shapeId = getObjectId();
        byte[] data = saveBVH(shapeId);
        capsule.write(data, tagNativeBvh, null);

        Platform nativePlatform = JmeSystem.getPlatform();
        capsule.write(nativePlatform, tagNativePlatform, null);

        capsule.write(nativeMesh, tagNativeMesh, null);
        capsule.write(useCompression, tagUseCompression, true);
    }
    // *************************************************************************
    // private methods

    /**
     * Instantiate the configured btBvhTriangleMeshShape.
     *
     * @param bvh built BVH data, or null if the BVH needs to be built
     */
    private void createShape(byte bvh[]) {
        boolean buildBvh = (bvh == null || bvh.length == 0);
        long meshId = nativeMesh.nativeId();
        long shapeId = createShape(useCompression, buildBvh, meshId);
        setNativeId(shapeId);

        if (!buildBvh) {
            nativeBVHBuffer = setBVH(bvh, shapeId);
            assert nativeBVHBuffer != 0L;
        }

        setScale(scale);
        setMargin(margin);
    }
    // *************************************************************************
    // native methods

    native private long createShape(boolean useCompression, boolean buildBvh,
            long meshId);

    native private void finalizeBVH(long nativeBVHBufferId);

    native private void recalcAabb(long shapeId);

    native private byte[] saveBVH(long objectId);

    /**
     * Read the ID of the native buffer used by the in-place de-serialized
     * shape. The buffer must be explicitly freed when no longer needed.
     */
    native private long setBVH(byte[] buffer, long objectid);
}
