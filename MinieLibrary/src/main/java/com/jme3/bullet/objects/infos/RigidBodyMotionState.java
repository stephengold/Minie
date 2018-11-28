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
package com.jme3.bullet.objects.infos;

import com.jme3.bullet.objects.PhysicsVehicle;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Spatial;
import com.jme3.util.clone.Cloner;
import com.jme3.util.clone.JmeCloneable;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * The motion state (transform) of a rigid body, with thread-safe access.
 *
 * @author normenhansen
 */
public class RigidBodyMotionState implements JmeCloneable {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(RigidBodyMotionState.class.getName());
    // *************************************************************************
    // fields

    /**
     * Unique identifier of the native object. Constructors are responsible for
     * setting this to a non-zero value. After that, the id never changes.
     */
    private long motionStateId;

    private Quaternion tmp_inverseWorldRotation = new Quaternion();
    private PhysicsVehicle vehicle = null;
    /**
     * true &rarr; physics coordinates match local transform, false &rarr;
     * physics coordinates match world transform
     */
    private boolean applyPhysicsLocal = false;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a motion state.
     */
    public RigidBodyMotionState() {
        motionStateId = createMotionState();
        assert motionStateId != 0L;
        logger.log(Level.FINE, "Created MotionState {0}",
                Long.toHexString(motionStateId));
    }
    // *************************************************************************
    // new methods exposed

    /**
     * If the motion state has been updated, apply the new transform to the
     * specified spatial.
     *
     * @param spatial where to apply the physics transform (not null, modified)
     * @return true if changed
     */
    public boolean applyTransform(Spatial spatial) {
        Vector3f localLocation = spatial.getLocalTranslation();
        Quaternion localRotationQuat = spatial.getLocalRotation();
        boolean physicsLocationDirty = applyTransform(motionStateId,
                localLocation, localRotationQuat);
        if (!physicsLocationDirty) {
            return false;
        }
        if (!applyPhysicsLocal && spatial.getParent() != null) {
            localLocation.subtractLocal(
                    spatial.getParent().getWorldTranslation());
            localLocation.divideLocal(spatial.getParent().getWorldScale());
            tmp_inverseWorldRotation.set(spatial.getParent().getWorldRotation())
                    .inverseLocal().multLocal(localLocation);
            tmp_inverseWorldRotation.mult(localRotationQuat, localRotationQuat);

            spatial.setLocalTranslation(localLocation);
            spatial.setLocalRotation(localRotationQuat);
        } else {
            spatial.setLocalTranslation(localLocation);
            spatial.setLocalRotation(localRotationQuat);
        }
        if (vehicle != null) {
            vehicle.updateWheels();
        }

        return true;
    }

    /**
     * Read the unique id of the native object.
     *
     * @return id (not zero)
     */
    public long getObjectId() {
        return motionStateId;
    }

    /**
     * Copy the location from this motion state.
     *
     * @param storeResult (modified if not null)
     * @return the location vector (in physics-space coordinates, either
     * storeResult of a new vector, not null)
     */
    public Vector3f getLocation(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getWorldLocation(motionStateId, result);

        assert Vector3f.isValidVector(result);
        return result;
    }

    /**
     * Copy the orientation from this motion state (as a matrix).
     *
     * @param storeResult (modified if not null)
     * @return the rotation matrix (in physics-space coordinates, either
     * storeResult of a new vector, not null)
     */
    public Matrix3f getOrientation(Matrix3f storeResult) {
        Matrix3f result = (storeResult == null) ? new Matrix3f() : storeResult;
        getWorldRotation(motionStateId, result);
        return result;
    }

    /**
     * Copy the orientation from this motion state (as a quaternion).
     *
     * @param storeResult (modified if not null)
     * @return the rotation quaternion (in physics-space coordinates, either
     * storeResult of a new vector, not null)
     */
    public Quaternion getOrientation(Quaternion storeResult) {
        Quaternion result
                = (storeResult == null) ? new Quaternion() : storeResult;
        getWorldRotationQuat(motionStateId, result);
        return result;
    }

    /**
     * Test whether physics-space coordinates should match the spatial's local
     * coordinates.
     *
     * @return true if matching local coordinates, false if matching world
     * coordinates
     */
    public boolean isApplyPhysicsLocal() {
        return applyPhysicsLocal;
    }

    /**
     * Calculate the body's physics transform.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the body's transform in physics-space coordinates (either
     * storeResult or a new instance, not null)
     */
    public Transform physicsTransform(Transform storeResult) {
        Transform transform;
        if (storeResult == null) {
            transform = new Transform();
        } else {
            transform = storeResult.setScale(1f);
        }

        getWorldLocation(motionStateId, transform.getTranslation());
        getWorldRotationQuat(motionStateId, transform.getRotation());

        return transform;
    }

    /**
     * Alter whether physics-space coordinates should match the spatial's local
     * coordinates.
     *
     * @param applyPhysicsLocal true&rarr;match local coordinates,
     * false&rarr;match world coordinates (default=false)
     */
    public void setApplyPhysicsLocal(boolean applyPhysicsLocal) {
        this.applyPhysicsLocal = applyPhysicsLocal;
    }

    /**
     * Alter which vehicle uses this motion state.
     *
     * @param vehicle the desired vehicle, or null for none (alias created)
     */
    public void setVehicle(PhysicsVehicle vehicle) {
        this.vehicle = vehicle;
    }
    // *************************************************************************
    // JmeCloneable methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned state into a deep-cloned one, using the specified cloner
     * and original to resolve copied fields.
     *
     * @param cloner the cloner that's cloning this state (not null)
     * @param original the instance from which this instance was shallow-cloned
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        motionStateId = createMotionState();
        assert motionStateId != 0L;

        tmp_inverseWorldRotation = cloner.clone(tmp_inverseWorldRotation);
        vehicle = cloner.clone(vehicle);
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public RigidBodyMotionState jmeClone() {
        try {
            RigidBodyMotionState clone = (RigidBodyMotionState) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }
    // *************************************************************************
    // Object methods

    /**
     * Finalize this motion state just before it is destroyed. Should be invoked
     * only by a subclass or by the garbage collector.
     *
     * @throws Throwable ignored by the garbage collector
     */
    @Override
    protected void finalize() throws Throwable {
        super.finalize();
        logger.log(Level.FINE, "Finalizing MotionState {0}",
                Long.toHexString(motionStateId));
        finalizeNative(motionStateId);
    }
    // *************************************************************************
    // private methods

    native private boolean applyTransform(long stateId, Vector3f location,
            Quaternion rotation);

    native private long createMotionState();

    native private void finalizeNative(long objectId);

    native private void getWorldLocation(long stateId, Vector3f storeResult);

    native private void getWorldRotation(long stateId, Matrix3f storeResult);

    native private void getWorldRotationQuat(long stateId,
            Quaternion storeResult);
}
