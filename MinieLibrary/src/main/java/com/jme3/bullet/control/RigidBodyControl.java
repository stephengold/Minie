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
package com.jme3.bullet.control;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.infos.RigidBodyMotionState;
import com.jme3.bullet.util.CollisionShapeFactory;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.control.Control;
import com.jme3.scene.shape.Box;
import com.jme3.scene.shape.Sphere;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.MySpatial;
import jme3utilities.math.MyMath;

/**
 * A PhysicsControl to link a PhysicsRigidBody to a Spatial.
 *
 * @author normenhansen
 */
public class RigidBodyControl
        extends PhysicsRigidBody
        implements PhysicsControl {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger3
            = Logger.getLogger(RigidBodyControl.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagApplyLocalPhysics = "applyLocalPhysics";
    final private static String tagApplyScale = "applyScale";
    final private static String tagEnabled = "enabled";
    final private static String tagKinematicSpatial = "kinematicSpatial";
    final private static String tagSpatial = "spatial";
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
     * true&rarr;enable shape scaling (to the extent that the CollisionShape
     * supports it), false&rarr;disable shape scaling
     */
    private boolean applyScale = false;
    /**
     * true&rarr;Control is enabled, false&rarr;Control is disabled
     */
    private boolean enabled = true;
    /**
     * true&rarr; kinematic body follows Spatial, false&rarr;Spatial follows
     * kinematic body
     */
    private boolean kinematicSpatial = true;
    /**
     * space to which the body is (or would be) added
     */
    private PhysicsSpace space = null;
    /**
     * Spatial to which this Control is added, or null if none
     */
    private Spatial spatial;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected RigidBodyControl() {
    }

    /**
     * Instantiate an enabled control. The new instance is incomplete because it
     * lacks a collision shape, so it CANNOT be immediately added to a space.
     *
     * Its shape will be auto-generated when it is added to a Spatial. If the
     * controlled spatial is a Geometry with a box or sphere mesh, a matching
     * box or sphere CollisionShape will be generated. Otherwise
     * {@link com.jme3.bullet.util.CollisionShapeFactory} will be used.
     *
     * @param mass the desired mass (&ge;0)
     */
    public RigidBodyControl(float mass) {
        this.mass = mass;
    }

    /**
     * Instantiate an enabled Control with a responsive dynamic body, mass=1,
     * and the specified CollisionShape.
     *
     * @param shape the desired shape (not null, alias created)
     */
    public RigidBodyControl(CollisionShape shape) {
        super(shape);
    }

    /**
     * Instantiate an enabled Control with a responsive dynamic or static body
     * and the specified CollisionShape and mass.
     *
     * @param shape the desired shape (not null, alias created)
     * @param mass the desired mass (&ge;0)
     */
    public RigidBodyControl(CollisionShape shape, float mass) {
        super(shape, mass);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Access the controlled spatial.
     *
     * @return the Spatial, or null if none
     */
    public Spatial getSpatial() {
        return spatial;
    }

    /**
     * Test whether physics-space coordinates should match the spatial's local
     * coordinates.
     *
     * @return true if matching local coordinates, false if matching world
     * coordinates
     */
    public boolean isApplyPhysicsLocal() {
        RigidBodyMotionState ms = getMotionState();
        boolean result = ms.isApplyPhysicsLocal();

        return result;
    }

    /**
     * Test whether the collision-shape scale should match the spatial's scale.
     *
     * @return true if matching scales, otherwise false
     */
    public boolean isApplyScale() {
        return applyScale;
    }

    /**
     * Test whether this Control is in kinematic mode. Kinematic mode has no
     * effect when the body isn't kinematic.
     *
     * @return true if the spatial's location and rotation would be applied to a
     * kinematic body, otherwise false
     */
    final public boolean isKinematicSpatial() {
        return kinematicSpatial;
    }

    /**
     * Alter whether physics-space coordinates should match the spatial's local
     * coordinates.
     *
     * @param applyPhysicsLocal true&rarr;match local coordinates,
     * false&rarr;match world coordinates (default=false)
     */
    public void setApplyPhysicsLocal(boolean applyPhysicsLocal) {
        RigidBodyMotionState ms = getMotionState();
        ms.setApplyPhysicsLocal(applyPhysicsLocal);
    }

    /**
     * Alter whether the collision-shape scale should match the spatial's scale.
     * CAUTION: Not all shapes can be scaled arbitrarily.
     * <p>
     * Note that if the shape is shared (between collision objects and/or
     * compound shapes) scaling can have unintended consequences.
     *
     * @param setting true &rarr; enable shape scaling (to the extent the
     * CollisionShape supports it), false &rarr; disable shape scaling
     * (default=false)
     */
    public void setApplyScale(boolean setting) {
        this.applyScale = setting;
    }

    /**
     * Enable or disable kinematic mode for this Control. If both the rigid body
     * and controlled spatial are kinematic, the spatial's location and rotation
     * will be applied to the body during each update. Kinematic mode has no
     * effect when the body isn't kinematic.
     *
     * @param kinematicSpatial true&rarr;kinematic, false&rarr;dynamic
     * (default=true)
     */
    public void setKinematicSpatial(boolean kinematicSpatial) {
        this.kinematicSpatial = kinematicSpatial;
    }
    // *************************************************************************
    // new protected methods

    /**
     * Set the body's CollisionShape based on the controlled spatial and its
     * descendants.
     */
    protected void createCollisionShape() {
        if (spatial == null) {
            return;
        }

        CollisionShape shape = null;
        if (spatial instanceof Geometry) {
            Mesh mesh = ((Geometry) spatial).getMesh();
            if (mesh instanceof Sphere) {
                float radius = ((Sphere) mesh).getRadius();
                shape = new SphereCollisionShape(radius);
            } else if (mesh instanceof Box) {
                Box box = (Box) mesh;
                shape = new BoxCollisionShape(
                        box.getXExtent(), box.getYExtent(), box.getZExtent());
            }
        }
        if (shape == null) {
            if (mass > massForStatic) {
                shape = CollisionShapeFactory.createDynamicMeshShape(spatial);
            } else {
                shape = CollisionShapeFactory.createMeshShape(spatial);
            }
        }
        setCollisionShape(shape);
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
        throw new UnsupportedOperationException(
                "cloneForSpatial() isn't implemented.");
    }

    /**
     * Access the PhysicsSpace to which the body is (or would be) added.
     *
     * @return the pre-existing space, or null for none
     */
    @Override
    public PhysicsSpace getPhysicsSpace() {
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
     * When the Control is disabled, the body is removed from PhysicsSpace. When
     * the Control is enabled again, the body is moved to the current location
     * of the Spatial and then added to the PhysicsSpace.
     *
     * @param enabled true&rarr;enable the Control, false&rarr;disable it
     */
    @Override
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (space != null) {
            if (enabled && !added) {
                if (spatial != null) {
                    setPhysicsLocation(getSpatialTranslation());
                    setPhysicsRotation(getSpatialRotation());
                }
                space.addCollisionObject(this);
                this.added = true;
            } else if (!enabled && added) {
                space.removeCollisionObject(this);
                this.added = false;
                // TODO also remove all joints
            }
        }
    }

    /**
     * If enabled, add this control's body to the specified PhysicsSpace. In not
     * enabled, alter where the body would be added. The body is removed from
     * any other space it's currently in.
     *
     * @param newSpace where to add, or null to simply remove
     */
    @Override
    public void setPhysicsSpace(PhysicsSpace newSpace) {
        if (space == newSpace) {
            return;
        }
        if (added) {
            space.removeCollisionObject(this);
            this.added = false;
        }
        if (newSpace != null && isEnabled()) {
            if (!hasAssignedNativeObject()) {
                String message = "Cannot add an incomplete RigidBodyControl "
                        + "to a PhysicsSpace.";
                throw new IllegalStateException(message);
            }
            newSpace.addCollisionObject(this);
            this.added = true;
        }
        /*
         * If this Control isn't enabled, its body will be
         * added to the new space when the Control becomes enabled.
         */
        this.space = newSpace;
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
        setUserObject(controlledSpatial); // link from collision object

        if (controlledSpatial != null) {
            if (getCollisionShape() == null) {
                createCollisionShape();
                rebuildRigidBody();
            }
            setPhysicsLocation(getSpatialTranslation());
            setPhysicsRotation(getSpatialRotation());
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

        if (isKinematic() && kinematicSpatial) {
            setPhysicsLocation(getSpatialTranslation());
            setPhysicsRotation(getSpatialRotation()); // TODO garbage
            if (applyScale) {
                CollisionShape shape = getCollisionShape();
                Vector3f newScale = copySpatialScale(null);
                if (!shape.canScale(newScale)) {
                    float factor = MyMath.cubeRoot(
                            newScale.x * newScale.y * newScale.z);
                    newScale.set(factor, factor, factor);
                }
                if (shape.canScale(newScale)) {
                    setPhysicsScale(newScale);
                }
            }

        } else if (!MySpatial.isIgnoringTransforms(spatial)) {
            getMotionState().applyTransform(spatial);
            if (applyScale) {
                applySpatialScale();
            }
        }
    }
    // *************************************************************************
    // PhysicsRigidBody methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned Control into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this Control (not null)
     * @param original the instance from which this Control was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        assert !hasAssignedNativeObject();
        RigidBodyControl old = (RigidBodyControl) original;
        assert old != this;
        assert old.hasAssignedNativeObject();

        super.cloneFields(cloner, original);
        if (hasAssignedNativeObject()) {
            return;
        }

        this.spatial = cloner.clone(spatial);
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

        this.enabled = capsule.readBoolean(tagEnabled, true);
        this.kinematicSpatial = capsule.readBoolean(tagKinematicSpatial, true);
        this.spatial = (Spatial) capsule.readSavable(tagSpatial, null);
        RigidBodyMotionState ms = getMotionState();
        ms.setApplyPhysicsLocal(
                capsule.readBoolean(tagApplyLocalPhysics, false));
        this.applyScale = capsule.readBoolean(tagApplyScale, false);

        setUserObject(spatial);
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

        capsule.write(enabled, tagEnabled, true);
        RigidBodyMotionState ms = getMotionState();
        capsule.write(ms.isApplyPhysicsLocal(), tagApplyLocalPhysics, false);
        capsule.write(kinematicSpatial, tagKinematicSpatial, true);
        capsule.write(applyScale, tagApplyScale, false);
        capsule.write(spatial, tagSpatial, null);
    }
    // *************************************************************************
    // private methods

    /**
     * Update whichever scale vector corresponds to the shape scale.
     */
    private void applySpatialScale() {
        Vector3f scale = getScale(null); // TODO garbage
        if (!isApplyPhysicsLocal()) {
            Node parent = spatial.getParent();
            if (parent != null) {
                Vector3f parentScale = parent.getWorldScale(); // alias
                if (parentScale.x == 0f || parentScale.y == 0f
                        || parentScale.z == 0f) {
                    throw new IllegalStateException("Zero in parent scale.");
                }
                scale.divideLocal(parentScale); // convert world to local
            }
        }
        spatial.setLocalScale(scale);
    }

    /**
     * Copy whichever scale vector corresponds to the shape scale.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the scale factor for each local axis of the shape (either
     * storeResult or a new vector, not null)
     */
    private Vector3f copySpatialScale(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        if (MySpatial.isIgnoringTransforms(spatial)) {
            result.set(scaleIdentity);
        } else if (isApplyPhysicsLocal()) {
            Vector3f scale = spatial.getLocalScale(); // alias
            result.set(scale);
        } else {
            Vector3f scale = spatial.getWorldScale(); // alias
            result.set(scale);
        }

        return result;
    }

    /**
     * Access whichever rotation corresponds to the physics rotation.
     *
     * @return the pre-existing Quaternion (not null)
     */
    private Quaternion getSpatialRotation() {
        Quaternion result;
        if (MySpatial.isIgnoringTransforms(spatial)) {
            result = rotateIdentity;
        } else {
            RigidBodyMotionState ms = getMotionState();
            if (ms.isApplyPhysicsLocal()) {
                result = spatial.getLocalRotation(); // alias
            } else {
                result = spatial.getWorldRotation(); // alias
            }
        }

        return result;
    }

    /**
     * Access whichever translation corresponds to the physics location.
     *
     * @return the pre-existing vector (not null)
     */
    private Vector3f getSpatialTranslation() {
        Vector3f result;
        if (MySpatial.isIgnoringTransforms(spatial)) {
            result = translateIdentity;
        } else {
            RigidBodyMotionState ms = getMotionState();
            if (ms.isApplyPhysicsLocal()) {
                result = spatial.getLocalTranslation(); // alias
            } else {
                result = spatial.getWorldTranslation(); // alias
            }
        }

        return result;
    }
}
