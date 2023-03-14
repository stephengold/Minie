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

import com.jme3.bullet.NativePhysicsObject;
import com.jme3.bullet.objects.PhysicsVehicle;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Spatial;
import com.jme3.util.clone.Cloner;
import com.jme3.util.clone.JmeCloneable;
import com.simsilica.mathd.Matrix3d;
import com.simsilica.mathd.Quatd;
import com.simsilica.mathd.Vec3d;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * The motion state (transform) of a rigid body, with thread-safe access.
 *
 * @author normenhansen
 */
public class RigidBodyMotionState
        extends NativePhysicsObject
        implements JmeCloneable, Savable {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(RigidBodyMotionState.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagApplyPhysicsLocal = "applyPhysicsLocal";
    final private static String tagVehicle = "vehicle";
    // *************************************************************************
    // fields

    /**
     * true &rarr; physics transform matches the spatial's local transform,
     * false &rarr; physics transform matches the spatial's world transform
     */
    private boolean applyPhysicsLocal = false;
    /**
     * vehicle reference, or null if the rigid body is a vehicle
     */
    private PhysicsVehicle vehicle = null;
    /**
     * temporary storage for a Quaternion
     */
    private Quaternion tmpInverseWorldRotation = new Quaternion();
    // *************************************************************************
    // constructors

    /**
     * Instantiate a motion state.
     */
    public RigidBodyMotionState() {
        long motionStateId = createMotionState();
        super.setNativeId(motionStateId);
        logger.log(Level.FINE, "Created {0}", this);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * If the motion state has been updated, apply the new transform to the
     * specified Spatial.
     *
     * @param spatial where to apply the physics transform (not null, modified)
     * @return true if changed
     */
    public boolean applyTransform(Spatial spatial) {
        long motionStateId = nativeId();
        Vector3f localLocation = spatial.getLocalTranslation(); // alias
        Quaternion localRotationQuat = spatial.getLocalRotation(); // alias
        boolean physicsLocationDirty = applyTransform(motionStateId,
                localLocation, localRotationQuat);
        if (!physicsLocationDirty) {
            return false;
        }
        if (!applyPhysicsLocal && spatial.getParent() != null) {
            localLocation.subtractLocal(
                    spatial.getParent().getWorldTranslation());
            localLocation.divideLocal(spatial.getParent().getWorldScale());
            tmpInverseWorldRotation.set(spatial.getParent().getWorldRotation())
                    .inverseLocal().multLocal(localLocation);
            tmpInverseWorldRotation.mult(localRotationQuat, localRotationQuat);

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
     * Copy the location to a Vector3f.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the location vector (in physics-space coordinates, either
     * storeResult or a new vector, not null)
     */
    public Vector3f getLocation(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long motionStateId = nativeId();
        getWorldLocation(motionStateId, result);

        assert Vector3f.isValidVector(result);
        return result;
    }

    /**
     * Copy the location to a Vector3d.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the location vector (in physics-space coordinates, either
     * storeResult or a new vector, not null, finite)
     */
    public Vec3d getLocationDp(Vec3d storeResult) {
        Vec3d result = (storeResult == null) ? new Vec3d() : storeResult;

        long motionStateId = nativeId();
        getWorldLocationDp(motionStateId, result);

        assert result.isFinite() : result;
        return result;
    }

    /**
     * Return the ID of the native object.
     *
     * @return the native identifier (not zero)
     * @deprecated use {@link NativePhysicsObject#nativeId()}
     */
    @Deprecated
    public long getObjectId() {
        long motionStateId = nativeId();
        return motionStateId;
    }

    /**
     * Copy the orientation to a Matrix3f.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the orientation (in physics-space coordinates, either storeResult
     * or a new matrix, not null)
     */
    public Matrix3f getOrientation(Matrix3f storeResult) {
        Matrix3f result = (storeResult == null) ? new Matrix3f() : storeResult;

        long motionStateId = nativeId();
        getWorldRotation(motionStateId, result);

        return result;
    }

    /**
     * Copy the orientation to a Quaternion.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the orientation (in physics-space coordinates, either storeResult
     * or a new instance, not null)
     */
    public Quaternion getOrientation(Quaternion storeResult) {
        Quaternion result
                = (storeResult == null) ? new Quaternion() : storeResult;

        long motionStateId = nativeId();
        getWorldRotationQuat(motionStateId, result);

        return result;
    }

    /**
     * Copy the orientation to a Matrix3d.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the orientation (in physics-space coordinates, either storeResult
     * or a new matrix, not null)
     */
    public Matrix3d getOrientationMatrixDp(Matrix3d storeResult) {
        Matrix3d result = (storeResult == null) ? new Matrix3d() : storeResult;

        long motionStateId = nativeId();
        getWorldRotationDp(motionStateId, result);

        return result;
    }

    /**
     * Copy the orientation to a Quatd.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the orientation (in physics-space coordinates, either storeResult
     * or a new instance, not null)
     */
    public Quatd getOrientationQuaternionDp(Quatd storeResult) {
        Quatd result = (storeResult == null) ? new Quatd() : storeResult;

        long motionStateId = nativeId();
        getWorldRotationQuatDp(motionStateId, result);

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

        long motionStateId = nativeId();
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
     * shallow-cloned state into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this state (not null)
     * @param original the instance from which this state was shallow-cloned
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        long motionStateId = createMotionState();
        reassignNativeId(motionStateId);

        this.tmpInverseWorldRotation = cloner.clone(tmpInverseWorldRotation);
        this.vehicle = cloner.clone(vehicle);
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public RigidBodyMotionState jmeClone() {
        try {
            RigidBodyMotionState clone = (RigidBodyMotionState) clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }
    // *************************************************************************
    // Savable methods

    /**
     * De-serialize this state from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        InputCapsule capsule = importer.getCapsule(this);

        this.applyPhysicsLocal
                = capsule.readBoolean(tagApplyPhysicsLocal, false);
        this.vehicle = (PhysicsVehicle) capsule.readSavable(tagVehicle, null);
    }

    /**
     * Serialize this object to the specified exporter, for example when saving
     * to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(applyPhysicsLocal, tagApplyPhysicsLocal, false);
        capsule.write(vehicle, tagVehicle, null);
        // tmpInverseWorldRotation is never written.
    }
    // *************************************************************************
    // Java private methods

    /**
     * Free the identified tracked native object. Invoked by reflection.
     *
     * @param stateId the native identifier (not zero)
     */
    private static void freeNativeObject(long stateId) {
        assert stateId != 0L;
        finalizeNative(stateId);
    }
    // *************************************************************************
    // native private methods

    native private static boolean applyTransform(
            long stateId, Vector3f location, Quaternion rotation);

    native private static long createMotionState();

    native private static void finalizeNative(long objectId);

    native private static void
            getWorldLocation(long stateId, Vector3f storeVector);

    native private static void
            getWorldLocationDp(long stateId, Vec3d storeVector);

    native private static void
            getWorldRotation(long stateId, Matrix3f storeMatrix);

    native private static void
            getWorldRotationDp(long stateId, Matrix3d storeMatrix);

    native private static void
            getWorldRotationQuat(long stateId, Quaternion storeQuat);

    native private static void
            getWorldRotationQuatDp(long stateId, Quatd storeQuat);
}
