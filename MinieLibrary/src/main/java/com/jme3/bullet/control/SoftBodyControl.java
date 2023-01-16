/*
 * Copyright (c) 2009-2016 jMonkeyEngine
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
package com.jme3.bullet.control;

import com.jme3.bullet.PhysicsSoftSpace;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.bullet.util.NativeSoftBodyUtil;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.mesh.IndexBuffer;
import com.jme3.util.BufferUtils;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.List;
import java.util.logging.Logger;
import jme3utilities.MySpatial;
import jme3utilities.Validate;

/**
 * A PhysicsControl to link a PhysicsSoftBody to a Spatial.
 *
 * @author dokthar
 */
public class SoftBodyControl extends AbstractPhysicsControl {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(SoftBodyControl.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagBody = "body";
    final private static String tagGeometry = "geometry";
    final private static String tagMergeVertices = "mergeVertices";
    final private static String tagUpdateNormals = "updateNormals";
    // *************************************************************************
    // fields

    /**
     * true&rarr;merge duplicate vertices in the soft body, false&rarr;don't
     * merge duplicate vertices
     */
    private boolean mergeVertices = true;
    /**
     * true&rarr;update normals, false&rarr;don't update normals
     */
    private boolean updateNormals = true;
    /**
     * Geometry that corresponds to the soft body
     */
    private Geometry geometry = null;
    /**
     * map from mesh-vertex indices to body-node indices, or null for identity
     */
    private IntBuffer indexMap = null; // TODO use an IndexBuffer to save memory
    /**
     * underlying collision object
     */
    private PhysicsSoftBody body = null;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled Control for a soft body based on a single
     * Geometry. Merge duplicate vertices, match physics-space coordinates to
     * world coordinates, and update mesh normals if present.
     */
    public SoftBodyControl() {
        // do nothing
    }

    /**
     * Instantiate an enabled Control for a soft body based on a single
     * Geometry.
     *
     * @param localPhysics true &rarr; match physics-space coordinates to the
     * spatial's local coordinates, false &rarr; match physics-space coordinates
     * to world coordinates
     * @param updateNormals true&rarr;update mesh normals if present,
     * false&rarr;never update mesh normals
     * @param mergeVertices true&rarr;merge duplicate vertices in the soft body,
     * false&rarr;don't merge duplicate vertices, see
     * {@link com.jme3.bullet.util.NativeSoftBodyUtil#generateIndexMap}
     */
    public SoftBodyControl(boolean localPhysics, boolean updateNormals,
            boolean mergeVertices) {
        super.setApplyPhysicsLocal(localPhysics);
        this.mergeVertices = mergeVertices;
        this.updateNormals = updateNormals;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Access the soft body managed by this Control.
     *
     * @return the pre-existing instance, or null if the Control is not added to
     * a Spatial
     */
    public PhysicsSoftBody getBody() {
        return body;
    }
    // *************************************************************************
    // AbstractPhysicsControl methods

    /**
     * Add all managed physics objects to the PhysicsSpace.
     */
    @Override
    protected void addPhysics() {
        PhysicsSpace space = getPhysicsSpace();
        space.addCollisionObject(body);
    }

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned Control into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this Control (not null, modified)
     * @param original the instance from which this Control was shallow-cloned
     * (not null)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);

        this.geometry = cloner.clone(geometry);
        this.body = cloner.clone(body);

        if (indexMap != null) {
            SoftBodyControl originalControl = (SoftBodyControl) original;
            int numIndices = indexMap.limit();
            this.indexMap = BufferUtils.createIntBuffer(numIndices);
            for (int offset = 0; offset < numIndices; ++offset) {
                int tmpIndex = originalControl.indexMap.get(offset);
                indexMap.put(tmpIndex);
            }
        }
    }

    /**
     * Create spatial-dependent data. Invoked when this Control is added to a
     * Spatial.
     *
     * @param spatial the controlled Spatial (not null, alias created)
     */
    @Override
    protected void createSpatialData(Spatial spatial) {
        this.body = new PhysicsSoftBody();
        this.body.setUserObject(spatial); // link from collision object

        List<Geometry> geometries = MySpatial.listGeometries(spatial);
        this.geometry = geometries.get(0); // TODO use name
        Mesh mesh = geometry.getMesh();
        if (mesh.getBuffer(VertexBuffer.Type.Normal) == null) {
            this.updateNormals = false;
        }
        appendFromGeometry();
    }

    /**
     * De-serialize this Control from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);
        InputCapsule capsule = importer.getCapsule(this);

        this.body = (PhysicsSoftBody) capsule.readSavable(tagBody, null);
        this.geometry = (Geometry) capsule.readSavable(tagGeometry, null);
        this.mergeVertices = capsule.readBoolean(tagMergeVertices, false);
        this.updateNormals = capsule.readBoolean(tagUpdateNormals, false);

        if (body != null) {
            Spatial controlled = getSpatial();
            body.setUserObject(controlled);
        }
    }

    /**
     * Remove all managed physics objects from the PhysicsSpace.
     */
    @Override
    protected void removePhysics() {
        PhysicsSpace space = getPhysicsSpace();
        space.removeCollisionObject(body);
    }

    /**
     * Destroy spatial-dependent data. Invoked when this Control is removed from
     * its Spatial.
     *
     * @param spatial the Spatial to which this Control was added (unused)
     */
    @Override
    protected void removeSpatialData(Spatial spatial) {
        body.setUserObject(null);
        this.body = null;
    }

    /**
     * Translate the soft body to the specified location.
     *
     * @param location the desired location (in physics-space coordinates, not
     * null, finite, unaffected)
     */
    @Override
    public void setPhysicsLocation(Vector3f location) {
        Validate.finite(location, "location");
        body.setPhysicsLocation(location);
    }

    /**
     * Rotate the soft body to the specified orientation.
     *
     * @param orientation the desired orientation (in physics-space coordinates,
     * not null, unaffected)
     */
    @Override
    protected void setPhysicsRotation(Quaternion orientation) {
        // TODO
    }

    /**
     * If enabled, add this control's physics object to the specified
     * PhysicsSpace. If not enabled, alter where the object would be added. The
     * object is removed from any other space it's currently in.
     *
     * @param newSpace where to add, or null to simply remove
     */
    @Override
    public void setPhysicsSpace(PhysicsSpace newSpace) {
        if (newSpace == null || newSpace instanceof PhysicsSoftSpace) {
            super.setPhysicsSpace(newSpace);
        } else {
            throw new IllegalArgumentException(
                    "The PhysicsSpace must be a PhysicsSoftSpace or null.");
        }
    }

    /**
     * Update this Control. Invoked once per frame during the logical-state
     * update, provided the Control is added to a scene. Do not invoke directly
     * from user code.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void update(float tpf) {
        if (!isEnabled()) {
            return;
        }

        Spatial spatial = getSpatial();

        Transform physicsToMesh;
        Transform meshToWorld = geometry.getWorldTransform(); // alias
        Transform worldToMesh = meshToWorld.invert();
        if (isApplyPhysicsLocal()) {
            Transform localToWorld = spatial.getWorldTransform(); // alias
            Transform localToMesh
                    = localToWorld.clone().combineWithParent(worldToMesh);
            physicsToMesh = localToMesh; // alias
        } else {
            physicsToMesh = worldToMesh; // alias
        }

        Mesh mesh = geometry.getMesh();
        boolean localFlag = false; // copy physics-space locations, not local
        NativeSoftBodyUtil.updateMesh(
                body, indexMap, mesh, localFlag, updateNormals, physicsToMesh);

        spatial.updateModelBound(); // TODO needed?
    }

    /**
     * Serialize this Control to the specified exporter, for example when saving
     * to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        super.write(exporter);
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(body, tagBody, null);
        capsule.write(geometry, tagGeometry, null);
        capsule.write(mergeVertices, tagMergeVertices, false);
        capsule.write(updateNormals, tagUpdateNormals, false);
    }
    // *************************************************************************
    // private methods

    /**
     * Append soft-body nodes, links, and faces to the empty body, based on
     * buffer data.
     */
    private void appendFromGeometry() {
        assert body.isEmpty();

        Mesh mesh = geometry.getMesh();
        FloatBuffer positions = mesh.getFloatBuffer(VertexBuffer.Type.Position);
        IndexBuffer links = null;
        IndexBuffer faces = null;
        switch (mesh.getMode()) {
            case Lines:
            case LineLoop:
            case LineStrip:
                links = mesh.getIndicesAsList();
                break;
            case Triangles:
            case TriangleFan:
            case TriangleStrip:
                faces = mesh.getIndicesAsList();
                break;
            default:
                throw new IllegalStateException(mesh.getMode().name());
        }

        if (mergeVertices) {
            indexMap = NativeSoftBodyUtil.generateIndexMap(positions);
            positions
                    = NativeSoftBodyUtil.mapVertexData(indexMap, positions, 3);
            if (links != null) {
                links = NativeSoftBodyUtil.mapIndices(indexMap, links, null);
            }
            if (faces != null) {
                faces = NativeSoftBodyUtil.mapIndices(indexMap, faces, null);
            }
        } else {
            indexMap = null;
        }

        body.appendNodes(positions);
        if (links != null) {
            body.appendLinks(links);
        }
        if (faces != null) {
            body.appendFaces(faces);
        }

        Transform meshToPhysics;
        Transform meshToWorld = geometry.getWorldTransform(); // alias
        if (isApplyPhysicsLocal()) {
            Spatial spatial = getSpatial();
            Transform localToWorld = spatial.getWorldTransform(); // alias
            Transform worldToLocal = localToWorld.invert();
            Transform meshToLocal
                    = meshToWorld.clone().combineWithParent(worldToLocal);
            meshToPhysics = meshToLocal; // alias
        } else {
            meshToPhysics = meshToWorld; // alias
        }
        body.applyTransform(meshToPhysics);
    }
}
