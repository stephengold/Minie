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
import com.jme3.scene.shape.AbstractBox;
import com.jme3.scene.shape.Box;
import com.jme3.scene.shape.Sphere;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.MySpatial;
import jme3utilities.math.MyMath;

/**
 * A physics control to link a PhysicsRigidBody to a spatial.
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
     * spatial to which this control is added, or null if none
     */
    protected Spatial spatial;
    /**
     * true &rarr; enable shape scaling (to the extent the collision shape
     * supports it), false &rarr; disable shape scaling (default=false)
     */
    private boolean applyScale = false;
    /**
     * true&rarr;control is enabled, false&rarr;control is disabled TODO
     * privatize
     */
    protected boolean enabled = true;
    /**
     * true&rarr;body is added to the physics space, false&rarr;not added
     */
    protected boolean added = false;
    /**
     * space to which the body is (or would be) added TODO privatize
     */
    protected PhysicsSpace space = null;
    /**
     * true&rarr;body is kinematic, false&rarr;body is static or dynamic
     */
    protected boolean kinematicSpatial = true;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public RigidBodyControl() {
    }

    /**
     * When using this constructor, the CollisionShape for the RigidBody is
     * generated automatically when the control is added to a spatial.
     *
     * @param mass When not 0, a HullCollisionShape is generated, otherwise a
     * MeshCollisionShape is used. For geometries with box or sphere meshes the
     * proper box or sphere collision shape is used.
     */
    public RigidBodyControl(float mass) {
        this.mass = mass;
    }
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled control with mass=1 and the specified collision
     * shape.
     *
     * @param shape the desired shape (not null, alias created)
     */
    public RigidBodyControl(CollisionShape shape) {
        super(shape);
    }

    /**
     * Instantiate an enabled control with the specified collision shape and
     * mass.
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
     * Test whether physics-space coordinates should match the spatial's local
     * coordinates.
     *
     * @return true if matching local coordinates, false if matching world
     * coordinates
     */
    public boolean isApplyPhysicsLocal() {
        return motionState.isApplyPhysicsLocal();
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
     * Test whether this control is in kinematic mode.
     *
     * @return true if the spatial location and rotation are applied to the
     * rigid body, otherwise false
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
        motionState.setApplyPhysicsLocal(applyPhysicsLocal);
    }

    /**
     * Alter whether the collision-shape scale should match the spatial's scale.
     * CAUTION: Not all shapes can be scaled arbitrarily.
     * <p>
     * Note that if the shape is shared (between collision objects and/or
     * compound shapes) scaling can have unintended consequences.
     *
     * @param setting true &rarr; enable shape scaling (to the extent the
     * collision shape supports it), false &rarr; disable shape scaling
     * (default=false)
     */
    public void setApplyScale(boolean setting) {
        applyScale = setting;
    }

    /**
     * Enable or disable kinematic mode. In kinematic mode, the spatial's
     * location and rotation will be applied to the rigid body.
     *
     * @param kinematicSpatial true&rarr;kinematic, false&rarr;dynamic or static
     */
    public void setKinematicSpatial(boolean kinematicSpatial) {
        this.kinematicSpatial = kinematicSpatial;
    }
    // *************************************************************************
    // new protected methods

    /**
     * Set the collision shape based on the controlled spatial and its
     * descendants.
     */
    protected void createCollisionShape() {
        if (spatial == null) {
            return;
        }
        if (spatial instanceof Geometry) {
            Geometry geom = (Geometry) spatial;
            Mesh mesh = geom.getMesh();
            if (mesh instanceof Sphere) {
                collisionShape = new SphereCollisionShape(
                        ((Sphere) mesh).getRadius());
                return;
            } else if (mesh instanceof Box) {
                collisionShape = new BoxCollisionShape(
                        new Vector3f(((AbstractBox) mesh).getXExtent(),
                                ((AbstractBox) mesh).getYExtent(),
                                ((AbstractBox) mesh).getZExtent()));
                return;
            }
        }
        if (mass > massForStatic) {
            collisionShape
                    = CollisionShapeFactory.createDynamicMeshShape(spatial);
        } else {
            collisionShape = CollisionShapeFactory.createMeshShape(spatial);
        }
    }
    // *************************************************************************
    // PhysicsControl methods

    /**
     * Clone this control for a different spatial. No longer used as of JME 3.1.
     *
     * @param spatial the spatial for the clone to control (or null)
     * @return a new control (not null)
     */
    @Override
    public Control cloneForSpatial(Spatial spatial) {
        throw new UnsupportedOperationException();
    }

    /**
     * Access the physics space to which the body is (or would be) added.
     *
     * @return the pre-existing space, or null for none
     */
    @Override
    public PhysicsSpace getPhysicsSpace() {
        return space;
    }

    /**
     * Test whether this control is enabled.
     *
     * @return true if enabled, otherwise false
     */
    @Override
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Render this control. Invoked once per view port per frame, provided the
     * control is added to a scene. Should be invoked only by a subclass or by
     * the RenderManager.
     *
     * @param rm the render manager (not null)
     * @param vp the view port to render (not null)
     */
    @Override
    public void render(RenderManager rm, ViewPort vp) {
    }

    /**
     * Enable or disable this control.
     * <p>
     * When the control is disabled, the body is removed from physics space.
     * When the control is enabled again, the body is moved to the current
     * location of the spatial and then added to the physics space.
     *
     * @param enabled true&rarr;enable the control, false&rarr;disable it
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
                added = true;
            } else if (!enabled && added) {
                space.removeCollisionObject(this);
                added = false;
                // TODO also remove all joints
            }
        }
    }

    /**
     * If enabled, add this control's body to the specified physics space. In
     * not enabled, alter where the body would be added. The body is removed
     * from any other space it's currently in.
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
            added = false;
        }
        if (newSpace != null && isEnabled()) {
            newSpace.addCollisionObject(this);
            added = true;
        }
        /*
         * If this control isn't enabled, its body will be
         * added to the new space when the control becomes enabled.
         */
        space = newSpace;
    }

    /**
     * Alter which spatial is controlled. Invoked when the control is added to
     * or removed from a spatial. Should be invoked only by a subclass or from
     * Spatial. Do not invoke directly from user code.
     *
     * @param controlledSpatial the spatial to control (or null)
     */
    @Override
    public void setSpatial(Spatial controlledSpatial) {
        if (spatial == controlledSpatial) {
            return;
        }

        spatial = controlledSpatial;
        setUserObject(controlledSpatial); // link from collision object

        if (controlledSpatial != null) {
            if (collisionShape == null) {
                createCollisionShape();
                rebuildRigidBody();
            }
            setPhysicsLocation(getSpatialTranslation());
            setPhysicsRotation(getSpatialRotation());
        }
    }

    /**
     * Update this control. Invoked once per frame, during the logical-state
     * update, provided the control is added to a scene.
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
            setPhysicsRotation(getSpatialRotation());
            if (applyScale) {
                Vector3f newScale = copySpatialScale(null);
                if (!collisionShape.canScale(newScale)) {
                    float factor = MyMath.cubeRoot(
                            newScale.x * newScale.y * newScale.z);
                    newScale.set(factor, factor, factor);
                }
                setPhysicsScale(newScale);
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
     * shallow-cloned control into a deep-cloned one, using the specified cloner
     * and original to resolve copied fields.
     *
     * @param cloner the cloner that's cloning this control (not null)
     * @param original the control from which this control was shallow-cloned
     * (unused)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);
        spatial = cloner.clone(spatial);
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new control (not null)
     */
    @Override
    public RigidBodyControl jmeClone() {
        try {
            RigidBodyControl clone = (RigidBodyControl) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }

    /**
     * De-serialize this control, for example when loading from a J3O file.
     *
     * @param im importer (not null)
     * @throws IOException from importer
     */
    @Override
    public void read(JmeImporter im) throws IOException {
        super.read(im);
        InputCapsule ic = im.getCapsule(this);
        enabled = ic.readBoolean("enabled", true);
        kinematicSpatial = ic.readBoolean("kinematicSpatial", true);
        spatial = (Spatial) ic.readSavable("spatial", null);
        motionState.setApplyPhysicsLocal(ic.readBoolean("applyLocalPhysics", false));
        applyScale = ic.readBoolean("applyScale", false);
    }

    /**
     * Serialize this control, for example when saving to a J3O file.
     *
     * @param ex exporter (not null)
     * @throws IOException from exporter
     */
    @Override
    public void write(JmeExporter ex) throws IOException {
        super.write(ex);
        OutputCapsule oc = ex.getCapsule(this);
        oc.write(enabled, "enabled", true);
        oc.write(motionState.isApplyPhysicsLocal(), "applyLocalPhysics", false);
        oc.write(kinematicSpatial, "kinematicSpatial", true);
        oc.write(applyScale, "applyScale", false);
        oc.write(spatial, "spatial", null);
    }
    // *************************************************************************
    // private methods

    /**
     * Update whichever spatial scale corresponds to the shape scale.
     */
    private void applySpatialScale() {
        Vector3f scale = getPhysicsScale(null);
        if (!isApplyPhysicsLocal()) {
            Node parent = spatial.getParent();
            if (parent != null) {
                Vector3f parentScale = parent.getWorldScale();
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
     * Copy whichever spatial scale corresponds to the shape scale.
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
            Vector3f scale = spatial.getLocalScale();
            result.set(scale);
        } else {
            Vector3f scale = spatial.getWorldScale();
            result.set(scale);
        }

        return result;
    }

    /**
     * Access whichever spatial translation corresponds to the physics location.
     *
     * @return the pre-existing vector (not null)
     */
    private Vector3f getSpatialTranslation() {
        if (MySpatial.isIgnoringTransforms(spatial)) {
            return translateIdentity;
        } else if (motionState.isApplyPhysicsLocal()) {
            return spatial.getLocalTranslation();
        } else {
            return spatial.getWorldTranslation();
        }
    }

    /**
     * Access whichever spatial rotation corresponds to the physics rotation.
     *
     * @return the pre-existing quaternion (not null)
     */
    private Quaternion getSpatialRotation() {
        if (MySpatial.isIgnoringTransforms(spatial)) {
            return rotateIdentity;
        } else if (motionState.isApplyPhysicsLocal()) {
            return spatial.getLocalRotation();
        } else {
            return spatial.getWorldRotation();
        }
    }
}
