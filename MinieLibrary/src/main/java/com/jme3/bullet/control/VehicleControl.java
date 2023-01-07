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
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.objects.PhysicsVehicle;
import com.jme3.bullet.objects.VehicleWheel;
import com.jme3.bullet.objects.infos.RigidBodyMotionState;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Spatial;
import com.jme3.scene.control.Control;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.MySpatial;

/**
 * A PhysicsControl to link a PhysicsVehicle to a Spatial.
 *
 * @author normenhansen
 */
public class VehicleControl
        extends PhysicsVehicle
        implements PhysicsControl {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger4
            = Logger.getLogger(VehicleControl.class.getName());
    /**
     * local copy of {@link com.jme3.math.Quaternion#IDENTITY}
     */
    final private static Quaternion rotateIdentity = new Quaternion();
    /**
     * field names for serialization
     */
    final private static String tagApplyLocalPhysics = "applyLocalPhysics";
    final private static String tagEnabled = "enabled";
    final private static String tagSpatial = "spatial";
    /**
     * local copy of {@link com.jme3.math.Vector3f#ZERO}
     */
    final private static Vector3f translateIdentity = new Vector3f(0f, 0f, 0f);
    // *************************************************************************
    // fields

    /**
     * true&rarr;vehicle is added to a PhysicsSpace, false&rarr;not added
     */
    private boolean added = false;
    /**
     * true&rarr;Control is enabled, false&rarr;Control is disabled
     */
    private boolean enabled = true;
    /**
     * space to which the vehicle is (or would be) added
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
    protected VehicleControl() {
    }

    /**
     * Instantiate an enabled Control with mass=1 and the specified
     * CollisionShape.
     *
     * @param shape the desired shape (not null, alias created)
     */
    public VehicleControl(CollisionShape shape) {
        super(shape);
    }

    /**
     * Instantiate an enabled Control with the specified CollisionShape and
     * mass.
     *
     * @param shape the desired shape (not null, alias created)
     * @param mass the desired mass (&gt;0)
     */
    public VehicleControl(CollisionShape shape, float mass) {
        super(shape, mass);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Access the controlled Spatial.
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
        return ms.isApplyPhysicsLocal();
    }

    /**
     * Alter whether physics-space coordinates should match the spatial's local
     * coordinates.
     *
     * @param applyPhysicsLocal true&rarr;match local coordinates,
     * false&rarr;match world coordinates (default=false)
     */
    public void setApplyPhysicsLocal(boolean applyPhysicsLocal) {
        RigidBodyMotionState motionsState = getMotionState();
        motionsState.setApplyPhysicsLocal(applyPhysicsLocal);

        int numWheels = getNumWheels();
        for (int wheelIndex = 0; wheelIndex < numWheels; ++wheelIndex) {
            VehicleWheel wheel = getWheel(wheelIndex);
            wheel.setApplyLocal(applyPhysicsLocal);
        }
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
                "cloneForSpatial() isn't implemented."
        );
    }

    /**
     * Access the PhysicsSpace to which the vehicle is (or would be) added.
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
     * When the Control is disabled, the vehicle is removed from PhysicsSpace.
     * When the Control is enabled again, the vehicle is moved to the current
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
     * If enabled, add this control's physics object to the specified
     * PhysicsSpace. In not enabled, alter where the object would be added. The
     * object is removed from any other space it's currently in.
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
         * If this Control isn't enabled, its physics object will be
         * added to the new space when the Control becomes enabled.
         */
        space = newSpace;
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
        spatial = controlledSpatial;
        setUserObject(controlledSpatial);

        if (controlledSpatial != null) {
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

        if (spatial != null) {
            if (getMotionState().applyTransform(spatial)) {
                spatial.getWorldTransform(); // updates the world transform
                applyWheelTransforms();
            }
        } else {
            applyWheelTransforms();
        }
    }
    // *************************************************************************
    // PhysicsVehicle methods

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
        VehicleControl old = (VehicleControl) original;
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

        enabled = capsule.readBoolean(tagEnabled, true);
        spatial = (Spatial) capsule.readSavable(tagSpatial, null);
        RigidBodyMotionState ms = getMotionState();
        ms.setApplyPhysicsLocal(
                capsule.readBoolean(tagApplyLocalPhysics, false));

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
        capsule.write(spatial, tagSpatial, null);
    }
    // *************************************************************************
    // private methods

    /**
     * Access whichever rotation corresponds to the physics rotation.
     *
     * @return the pre-existing Quaternion (not null)
     */
    private Quaternion getSpatialRotation() {
        if (MySpatial.isIgnoringTransforms(spatial)) {
            return rotateIdentity;
        } else {
            RigidBodyMotionState ms = getMotionState();
            if (ms.isApplyPhysicsLocal()) {
                return spatial.getLocalRotation(); // alias
            } else {
                return spatial.getWorldRotation(); // alias
            }
        }
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
