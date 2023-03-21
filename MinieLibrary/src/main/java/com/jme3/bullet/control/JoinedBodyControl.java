/*
 * Copyright (c) 2023 jMonkeyEngine
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
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.infos.RigidBodyMotionState;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Spatial;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A custom PhysicsControl for a dynamic rigid body that can be joined to other
 * bodies. When the Control is removed from a space, all its joints are
 * automatically destroyed.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class JoinedBodyControl extends AbstractPhysicsControl {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger3
            = Logger.getLogger(JoinedBodyControl.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagRigidBody = "rigidBody";
    // *************************************************************************
    // fields

    /**
     * main rigid body
     */
    private PhysicsRigidBody rigidBody;
    /**
     * temporary storage for a quaternion
     */
    final private static Quaternion tmpUpdateOrientation = new Quaternion();
    /**
     * temporary storage for a vector
     */
    final private static Vector3f tmpUpdateLocation = new Vector3f();
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected JoinedBodyControl() {
    }

    /**
     * Instantiate an enabled Control in dynamic mode.
     *
     * @param bodyShape the desired shape for the rigid body (not null, alias
     * created)
     * @param mass the desired mass for the rigid body (gt;0)
     */
    public JoinedBodyControl(CollisionShape bodyShape, float mass) {
        Validate.nonNull(bodyShape, "shape");
        Validate.positive(mass, "mass");

        this.rigidBody = new PhysicsRigidBody(bodyShape, mass);
        rigidBody.setUserObject(this);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Access the rigid body managed by this Control.
     *
     * @return the pre-existing rigid body (not null)
     */
    public PhysicsRigidBody getRigidBody() {
        return rigidBody;
    }

    /**
     * Test whether the body is in kinematic mode.
     *
     * @return true if in kinematic mode, otherwise false (in dynamic mode)
     */
    public boolean isKinematic() {
        boolean result = !rigidBody.isDynamic();
        return result;
    }

    /**
     * Transition the body from kinematic mode to dynamic mode or vice versa.
     *
     * @param newSetting true&rarr;set kinematic mode, false&rarr;set dynamic
     * mode (default=false)
     */
    public void setKinematic(boolean newSetting) {
        rigidBody.setKinematic(newSetting);
    }
    // *************************************************************************
    // AbstractPhysicsControl methods

    /**
     * Add all managed physics objects to the PhysicsSpace.
     */
    @Override
    protected void addPhysics() {
        PhysicsSpace space = getPhysicsSpace();
        space.addCollisionObject(rigidBody);
        for (PhysicsJoint joint : rigidBody.listJoints()) {
            space.addJoint(joint);
        }
    }

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned Control into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this Control (not null, modified)
     * @param original the instance from which this Control was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);
        this.rigidBody = cloner.clone(rigidBody);
    }

    /**
     * Create spatial-dependent data. Invoked when this Control is added to a
     * Spatial.
     *
     * @param spatial the controlled Spatial (not null, alias created)
     */
    @Override
    protected void createSpatialData(Spatial spatial) {
        // do nothing
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
        this.rigidBody
                = (PhysicsRigidBody) capsule.readSavable(tagRigidBody, null);
        rigidBody.setUserObject(this);
    }

    /**
     * Remove all managed physics objects from the PhysicsSpace.
     */
    @Override
    protected void removePhysics() {
        PhysicsSpace space = getPhysicsSpace();
        PhysicsJoint[] joints = rigidBody.listJoints();
        for (PhysicsJoint joint : joints) {
            joint.destroy();
            space.removeJoint(joint);
        }
        space.removeCollisionObject(rigidBody);
    }

    /**
     * Destroy spatial-dependent data. Invoked when this Control is removed from
     * its Spatial.
     *
     * @param spatial the Spatial to which this Control was added (unused)
     */
    @Override
    protected void removeSpatialData(Spatial spatial) {
        // do nothing
    }

    /**
     * Translate the body instantly to the specified location.
     *
     * @param newLocation the desired location (in physics-space coordinates,
     * not null, finite, unaffected)
     */
    @Override
    public void setPhysicsLocation(Vector3f newLocation) {
        Validate.finite(newLocation, "new location");
        rigidBody.setPhysicsLocation(newLocation);
    }

    /**
     * Rotate the body instantly to the specified orientation.
     *
     * @param newOrientation the desired orientation (in physics-space
     * coordinates, not null, not zero, unaffected)
     */
    @Override
    protected void setPhysicsRotation(Quaternion newOrientation) {
        Validate.nonZero(newOrientation, "new orientation");
        rigidBody.setPhysicsRotation(newOrientation);
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
        if (!isEnabled()) {
            return;
        }

        if (rigidBody.isDynamic()) {
            RigidBodyMotionState motionState = rigidBody.getMotionState();
            motionState.getLocation(tmpUpdateLocation);
            motionState.getOrientation(tmpUpdateOrientation);
            applyPhysicsTransform(tmpUpdateLocation, tmpUpdateOrientation);
        } else { // kinematic
            tmpUpdateLocation.set(getSpatialTranslation());
            setPhysicsLocation(tmpUpdateLocation);
            tmpUpdateOrientation.set(getSpatialRotation());
            setPhysicsRotation(tmpUpdateOrientation);
        }
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
        capsule.write(rigidBody, tagRigidBody, null);
    }
}
