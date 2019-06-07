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
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.control.Control;
import com.jme3.scene.mesh.IndexBuffer;
import com.jme3.util.TempVars;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.List;
import java.util.logging.Logger;
import jme3utilities.MySpatial;

/**
 * A PhysicsControl to link a PhysicsSoftBody to a Spatial. TODO extend
 * AbstractPhysicsControl
 *
 * @author dokthar
 */
public class SoftBodyControl
        extends PhysicsSoftBody
        implements PhysicsControl {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger3
            = Logger.getLogger(SoftBodyControl.class.getName());
    /**
     * local copy of {@link com.jme3.math.Quaternion#IDENTITY}
     */
    final private static Quaternion rotateIdentity = new Quaternion();
    /**
     * local copy of {@link com.jme3.math.Vector3f#UNIT_XYZ}
     */
    final private static Vector3f scaleIdentity = new Vector3f(1f, 1f, 1f);
    /**
     * local copy of {@link com.jme3.math.Vector3f#ZERO}
     */
    final private static Vector3f translateIdentity = new Vector3f(0f, 0f, 0f);
    // *************************************************************************
    // fields

    /**
     * true&rarr;body is added to a PhysicsSpace, false&rarr;not added
     */
    private boolean added = false;
    /**
     * true&rarr;Control is enabled, false&rarr;Control is disabled
     * (default=true)
     */
    private boolean enabled = true;
    /**
     * true &rarr; match physics-space coordinates to the spatial's local
     * coordinates, false &rarr; match physics-space coordinates to world
     * coordinates (default=false)
     */
    private boolean localPhysics = false;
    /**
     * true&rarr;merge duplicate vertices in the soft body, false&rarr;don't
     * merge duplicate vertices (default=true)
     */
    private boolean mergeVertices = true;
    /**
     * true&rarr;update normals, false&rarr;don't update normals (default=true)
     */
    private boolean updateNormals = true;
    /**
     * Geometry that corresponds to the soft body
     */
    private Geometry geometry = null;
    /**
     * map from mesh-vertex indices to body-node indices, or null for identity
     */
    private IntBuffer indexMap = null;
    /**
     * space to which the body is (or would be) added
     */
    private PhysicsSoftSpace space = null;
    /**
     * Spatial to which this Control is added, or null if none
     */
    private Spatial spatial = null;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled Control for a soft body based on a single
     * Geometry. Merge duplicate vertices, match physics-space coordinates to
     * world coordinates, and update mesh normals.
     */
    public SoftBodyControl() {
    }

    /**
     * Instantiate an enabled Control for a soft body based on a single
     * Geometry.
     *
     * @param localPhysics true &rarr; match physics-space coordinates to the
     * spatial's local coordinates, false &rarr; match physics-space coordinates
     * to world coordinates
     * @param updateNormals true&rarr;update mesh normals, false&rarr;don't
     * update mesh normals
     * @param mergeVertices true&rarr;merge duplicate vertices in the soft body,
     * false&rarr;don't merge duplicate vertices, see
     * {@link com.jme3.bullet.util.NativeSoftBodyUtil#generateIndexMap}
     */
    public SoftBodyControl(boolean localPhysics, boolean updateNormals,
            boolean mergeVertices) {
        this.localPhysics = localPhysics;
        this.mergeVertices = mergeVertices;
        this.updateNormals = updateNormals;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Access the Spatial to which this Control is added.
     *
     * @return the Spatial, or null if none
     */
    public Spatial getSpatial() {
        return spatial;
    }

    /**
     * Test whether physics-space coordinates should match the controlled
     * spatial's local coordinates.
     *
     * @return true if matching local coordinates, false if matching world
     * coordinates
     */
    public boolean isLocalPhysics() {
        return localPhysics;
    }

    /**
     * If enabled, add this control's physics object to the specified
     * PhysicsSpace. If not enabled, alter where the object would be added. The
     * object is removed from any other space it's currently in.
     *
     * @param newSpace where to add, or null to simply remove
     */
    public void setPhysicsSoftSpace(PhysicsSoftSpace newSpace) {
        if (space == newSpace) {
            return;
        }
        if (added) {
            space.removeCollisionObject(this);
            added = false;
        }
        if (newSpace != null && isEnabled()) {
            newSpace.addCollisionObject(this);
            added = true;
        }
        /*
         * If this Control isn't enabled, its physics object will be
         * added to the new space when the Control becomes enabled.
         */
        space = newSpace;
    }
    // *************************************************************************
    // PhysicsControl methods

    /**
     * Clone this Control for a different Spatial. No longer used as of JME 3.1.
     *
     * @param spatial (unused)
     * @return never
     * @throws UnsupportedOperationException always
     */
    @Override
    public Control cloneForSpatial(Spatial spatial) {
        throw new UnsupportedOperationException();
    }

    /**
     * Access the PhysicsSpace to which the body is (or would be) added.
     *
     * @return the pre-existing space, or null for none
     */
    @Override
    public PhysicsSoftSpace getPhysicsSpace() {
        return space;
    }

    /**
     * Test whether this Control is enabled.
     *
     * @return true if enabled, otherwise false
     */
    @Override
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Render this Control. Invoked once per ViewPort per frame, provided the
     * Control is added to a scene. Should be invoked only by a subclass or by
     * the RenderManager.
     *
     * @param rm the RenderManager (unused)
     * @param vp the ViewPort to render (unused)
     */
    @Override
    public void render(RenderManager rm, ViewPort vp) {
        // do nothing
    }

    /**
     * Enable or disable this Control.
     * <p>
     * When the Control is disabled, the body is removed from any PhysicsSpace.
     * When the Control is enabled again, the body is moved to the current
     * location of the Spatial and then added to the PhysicsSpace.
     *
     * @param enabled true&rarr;enable the Control, false&rarr;disable it
     */
    @Override
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (space != null) {
            if (enabled && !added) {
                if (spatial != null) {
                    setPhysicsLocation(spatial.getWorldTranslation());
                    TempVars vars = TempVars.get();
                    try {
                        if (localPhysics) {
                            getPhysicsLocation(vars.vect1);
                            spatial.getParent().worldToLocal(vars.vect1, vars.vect2);
                            spatial.setLocalTranslation(vars.vect2);
                        } else {
                            spatial.getParent().worldToLocal(Vector3f.ZERO, vars.vect2);
                            spatial.setLocalTranslation(vars.vect2);
                        }
                        if (!spatial.getWorldRotation().equals(Quaternion.IDENTITY)) {
                            Spatial parent = spatial.getParent();
                            if (parent != null) {
                                Quaternion rot = parent.getWorldRotation().inverse();
                                //rot.multLocal(Quaternion.IDENTITY);
                                spatial.setLocalRotation(rot);
                            } else {
                                spatial.setLocalRotation(new Quaternion(Quaternion.IDENTITY));
                            }
                        }
                    } finally {
                        vars.release();
                    }
                }
                space.addCollisionObject(this);
                added = true;
            } else if (!enabled && added) {
                space.removeCollisionObject(this);
                added = false;
                // TODO also remove all joints
            }
        }
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
            setPhysicsSoftSpace((PhysicsSoftSpace) newSpace);
        } else {
            throw new IllegalArgumentException(
                    "The PhysicsSpace must be a PhysicsSoftSpace or null.");
        }
    }

    /**
     * Alter which Spatial is controlled. Invoked when the Control is added to
     * or removed from a Spatial. Should be invoked only by a subclass or from
     * Spatial. Do not invoke directly from user code.
     *
     * @param controlledSpatial the Spatial to control (or null)
     */
    @Override
    public void setSpatial(Spatial controlledSpatial) {
        if (spatial == controlledSpatial) {
            return;
        }

        spatial = controlledSpatial;
        setUserObject(spatial); // link from collision object

        if (spatial != null && geometry == null) {
            // TODO clear any pre-existing nodes/links/faces
            List<Geometry> geometries
                    = MySpatial.listSpatials(spatial, Geometry.class, null);
            geometry = geometries.get(0); // TODO use name
            Mesh mesh = geometry.getMesh();
            if (mesh.getBuffer(VertexBuffer.Type.Normal) == null) {
                updateNormals = false;
            }
            appendFromGeometry();
        }
    }

    /**
     * Update this Control. Invoked once per frame, during the logical-state
     * update, provided the Control is added to a scene. Do not invoke directly
     * from user code.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void update(float tpf) {
        if (!enabled) {
            return;
        }

        Transform physicsToMesh;
        Transform meshToWorld = geometry.getWorldTransform(); // alias
        Transform worldToMesh = meshToWorld.invert();
        if (localPhysics) {
            Transform localToWorld = spatial.getWorldTransform(); // alias
            Transform localToMesh
                    = localToWorld.clone().combineWithParent(worldToMesh);
            physicsToMesh = localToMesh; // alias
        } else {
            physicsToMesh = worldToMesh; // alias
        }

        Mesh mesh = geometry.getMesh();
        boolean localFlag = false; // copy physics-space locations
        NativeSoftBodyUtil.updateMesh(this, indexMap, mesh, localFlag,
                updateNormals, physicsToMesh);

        spatial.updateModelBound();
    }
    // *************************************************************************
    // PhysicsSoftBody methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned control into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this Control (not null)
     * @param original the Control from which this Control was shallow-cloned
     * (unused)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);

        indexMap = cloner.clone(indexMap);
        geometry = cloner.clone(geometry);
        spatial = cloner.clone(spatial);
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new Control (not null)
     */
    @Override
    public SoftBodyControl jmeClone() {
        try {
            SoftBodyControl clone = (SoftBodyControl) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
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

        enabled = capsule.readBoolean("enabled", true);
        geometry = (Geometry) capsule.readSavable("geometry", null);
        localPhysics = capsule.readBoolean("localPhysics", false);
        mergeVertices = capsule.readBoolean("mergeVertices", false);
        spatial = (Spatial) capsule.readSavable("spatial", null);
        updateNormals = capsule.readBoolean("updateNormals", false);
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

        capsule.write(enabled, "enabled", true);
        capsule.write(geometry, "geometry", null);
        capsule.write(localPhysics, "localPhysics", false);
        capsule.write(mergeVertices, "mergeVertices", false);
        capsule.write(spatial, "spatial", null);
        capsule.write(updateNormals, "updateNormals", false);
    }
    // *************************************************************************
    // private methods

    /**
     * Append soft-body nodes, links, and faces to an empty body, based on
     * buffer data.
     */
    private void appendFromGeometry() {
        assert isEmpty();

        Mesh mesh = geometry.getMesh();
        FloatBuffer positions = mesh.getFloatBuffer(VertexBuffer.Type.Position);
        IndexBuffer links = null;
        IndexBuffer faces = null;
        switch (mesh.getMode()) {
            case Lines:
                links = mesh.getIndexBuffer();
                break;
            case Triangles:
                faces = mesh.getIndexBuffer();
                break;
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

        appendNodes(positions);
        if (links != null) {
            appendLinks(links);
        }
        if (faces != null) {
            appendFaces(faces);
        }

        Transform meshToPhysics;
        Transform meshToWorld = geometry.getWorldTransform(); // alias
        if (localPhysics) {
            Transform localToWorld = spatial.getWorldTransform(); // alias
            Transform worldToLocal = localToWorld.invert();
            Transform meshToLocal
                    = meshToWorld.clone().combineWithParent(worldToLocal);
            meshToPhysics = meshToLocal; // alias
        } else {
            meshToPhysics = meshToWorld; // alias
        }
        applyTransform(meshToPhysics);
    }
}
