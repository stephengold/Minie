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
package com.jme3.bullet.joints;

import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.math.MyVector3f;

/**
 * A single degree-of-freedom joint based on Bullet's btHingeConstraint.
 * <p>
 * <i>From the Bullet manual:</i><br>
 * Hinge constraint, or revolute joint restricts two additional angular degrees
 * of freedom, so the body can only rotate around one axis, the hinge axis. This
 * can be useful to represent doors or wheels rotating around one axis. The user
 * can specify limits and motor for the hinge.
 *
 * @author normenhansen
 */
public class HingeJoint extends Constraint {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(HingeJoint.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagAngularOnly = "angularOnly";
    final private static String tagAxisA = "axisA";
    final private static String tagAxisB = "axisB";
    final private static String tagBiasFactor = "biasFactor";
    final private static String tagEnableAngularMotor = "enableAngularMotor";
    final private static String tagLimitSoftness = "limitSoftness";
    final private static String tagLowerLimit = "lowerLimit";
    final private static String tagMaxMotorImpulse = "maxMotorImpulse";
    final private static String tagRelaxationFactor = "relaxationFactor";
    final private static String tagTargetVelocity = "targetVelocity";
    final private static String tagUpperLimit = "upperLimit";
    // *************************************************************************
    // fields

    /**
     * copy of the angular-only flag
     */
    private boolean angularOnly = false;
    /**
     * copy of the "use reference frame A" flag
     */
    private boolean useReferenceFrameA = false;
    /**
     * copy of the limit's bias factor: how strictly position errors are
     * corrected
     */
    private float biasFactor = 0.3f;
    /**
     * copy of the limit's softness: the range fraction at which velocity-error
     * correction starts operating
     */
    private float limitSoftness = 0.9f;
    /**
     * copy of the limit's relaxation factor: the rate at which velocity errors
     * are corrected
     */
    private float relaxationFactor = 1f;
    /**
     * copy of the joint axis in A's local coordinates (unit vector)
     */
    private Vector3f axisA;
    /**
     * copy of the joint axis: in B's local coordinates for a double-ended
     * joint, or in physics-space coordinates for a single-ended joint (unit
     * vector)
     */
    private Vector3f axisB;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected HingeJoint() {
    }

    /**
     * Instantiate a single-ended HingeJoint.
     * <p>
     * To be effective, the joint must be added to the body's PhysicsSpace and
     * the body must be dynamic.
     *
     * @param rigidBodyA the body to constrain (not null, alias created)
     * @param pivotInA the pivot location in A's scaled local coordinates (not
     * null, unaffected)
     * @param pivotInWorld the pivot location in physics-space coordinates (not
     * null, unaffected)
     * @param axisInA the joint axis in A's local coordinates (unit vector,
     * unaffected)
     * @param axisInWorld the joint axis in physics-space coordinates (unit
     * vector, unaffected)
     * @param referenceFrame which end to use as the reference frame (not null)
     */
    public HingeJoint(PhysicsRigidBody rigidBodyA, Vector3f pivotInA,
            Vector3f pivotInWorld, Vector3f axisInA, Vector3f axisInWorld,
            JointEnd referenceFrame) {
        super(rigidBodyA, JointEnd.A, pivotInA, pivotInWorld);

        assert axisInA.isUnitVector() : axisInA;
        assert axisInWorld.isUnitVector() : axisInWorld;
        this.axisA = axisInA.clone();
        this.axisB = axisInWorld.clone();
        this.useReferenceFrameA = (referenceFrame == JointEnd.A);
        createJoint();

        // Synchronize the btHingeConstraint parameters with the local copies.
        long constraintId = super.nativeId();
        setAngularOnly(constraintId, angularOnly);

        float low = getLowerLimit();
        float high = getUpperLimit();
        setLimit(constraintId, low, high, limitSoftness, biasFactor,
                relaxationFactor);
    }

    /**
     * Instantiate a double-ended HingeJoint.
     * <p>
     * To be effective, the joint must be added to the PhysicsSpace of both
     * bodies. Also, the bodies must be distinct and at least one of them must
     * be dynamic.
     *
     * @param rigidBodyA the body for the A end (not null, alias created)
     * @param rigidBodyB the body for the B end (not null, alias created)
     * @param pivotInA the pivot location in A's scaled local coordinates (not
     * null, unaffected)
     * @param pivotInB the pivot location in B's scaled local coordinates (not
     * null, unaffected)
     * @param axisInA the joint axis in A's local coordinates (unit vector,
     * unaffected)
     * @param axisInB the joint axis in B's local coordinates (unit vector,
     * unaffected)
     */
    public HingeJoint(PhysicsRigidBody rigidBodyA, PhysicsRigidBody rigidBodyB,
            Vector3f pivotInA, Vector3f pivotInB, Vector3f axisInA,
            Vector3f axisInB) {
        super(rigidBodyA, rigidBodyB, pivotInA, pivotInB);

        assert axisInA.isUnitVector() : axisInA;
        assert axisInB.isUnitVector() : axisInB;
        this.axisA = axisInA.clone();
        this.axisB = axisInB.clone();
        createJoint();

        // Synchronize btHingeConstraint parameters with local copies.
        long constraintId = super.nativeId();
        setAngularOnly(constraintId, angularOnly);

        float low = getLowerLimit();
        float high = getUpperLimit();
        setLimit(constraintId, low, high, limitSoftness, biasFactor,
                relaxationFactor);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Enable or disable this joint's motor.
     *
     * @param enable true to enable, false to disable (default=false)
     * @param targetVelocity the desired target velocity
     * @param maxMotorImpulse the desired maximum rotational force
     */
    public void enableMotor(
            boolean enable, float targetVelocity, float maxMotorImpulse) {
        long constraintId = nativeId();
        enableMotor(constraintId, enable, targetVelocity, maxMotorImpulse);
    }

    /**
     * Read this joint's bias factor.
     *
     * @return the magnitude of the position correction: how strictly position
     * errors (drift) are corrected
     */
    public float getBiasFactor() {
        return biasFactor;
    }

    /**
     * Test whether this joint's motor is enabled.
     *
     * @return true if enabled, otherwise false
     */
    public boolean getEnableMotor() {
        long constraintId = nativeId();
        boolean result = getEnableAngularMotor(constraintId);

        return result;
    }

    /**
     * Copy the joint's frame transform relative to the specified end.
     *
     * @param end which end (not null)
     * @param storeResult storage for the result (modified if not null)
     * @return the transform of the constraint space relative to the end
     */
    public Transform getFrameTransform(JointEnd end, Transform storeResult) {
        Transform result
                = (storeResult == null) ? new Transform() : storeResult;

        long constraintId = nativeId();
        switch (end) {
            case A:
                getFrameOffsetA(constraintId, result);
                break;
            case B:
                getFrameOffsetB(constraintId, result);
                break;
            default:
                String message = "end = " + end;
                throw new IllegalArgumentException(message);
        }

        return result;
    }

    /**
     * Read the hinge angle.
     *
     * @return the angle (in radians)
     */
    public float getHingeAngle() {
        long constraintId = nativeId();
        float result = getHingeAngle(constraintId);

        return result;
    }

    /**
     * Read this joint's limit softness.
     *
     * @return the range fraction at which velocity-error correction starts
     * operating
     */
    public float getLimitSoftness() {
        return limitSoftness;
    }

    /**
     * Read the lower limit of the hinge angle.
     *
     * @return the angle (in radians)
     */
    final public float getLowerLimit() {
        long constraintId = nativeId();
        float result = getLowerLimit(constraintId);

        return result;
    }

    /**
     * Read the motor's target velocity.
     *
     * @return velocity
     */
    public float getMotorTargetVelocity() {
        long constraintId = nativeId();
        float result = getMotorTargetVelocity(constraintId);

        return result;
    }

    /**
     * Read the motor's maximum impulse.
     *
     * @return impulse
     */
    public float getMaxMotorImpulse() {
        long constraintId = nativeId();
        float result = getMaxMotorImpulse(constraintId);

        return result;
    }

    /**
     * Read this joint's relaxation factor.
     *
     * @return the rate at which velocity errors are corrected
     */
    public float getRelaxationFactor() {
        return relaxationFactor;
    }

    /**
     * Read the upper limit of the hinge angle.
     *
     * @return angle (in radians)
     */
    final public float getUpperLimit() {
        long constraintId = nativeId();
        float result = getUpperLimit(constraintId);

        return result;
    }

    /**
     * Test whether this joint is angular-only, meaning no constraints on
     * translation.
     *
     * @return true if angular-only, otherwise false
     */
    public boolean isAngularOnly() {
        return angularOnly;
    }

    /**
     * Alter whether this joint is angular-only, meaning no constraints on
     * translation.
     *
     * @param angularOnly true&rarr;translation is free, false&rarr;translation
     * is constrained (default=false)
     */
    public void setAngularOnly(boolean angularOnly) {
        this.angularOnly = angularOnly;
        long constraintId = nativeId();
        setAngularOnly(constraintId, angularOnly);
    }

    /**
     * Alter the angular limits for this joint.
     *
     * @param low the desired lower limit of the hinge angle (in radians,
     * default=1)
     * @param high the desired upper limit of the hinge angle (in radians,
     * default=-1)
     */
    public void setLimit(float low, float high) {
        long constraintId = nativeId();
        setLimit(constraintId, low, high, limitSoftness, biasFactor,
                relaxationFactor);
    }

    /**
     * Alter the angular limits for this joint.
     * <p>
     * If you're above the softness, velocities that would shoot through the
     * actual limit are slowed down. The bias should be in the range of 0.2 -
     * 0.5.
     *
     * @param low the desired lower limit of the hinge angle (in radians,
     * default=1)
     * @param high the desired upper limit of the hinge angle (in radians,
     * default=-1)
     * @param softness the desired range fraction at which velocity-error
     * correction starts operating. A softness of 0.9 means that the correction
     * starts at 90% of the limit range. (default=0.9)
     * @param bias the desired magnitude of the position correction: how
     * strictly position errors (drift) are corrected (default=0.3)
     * @param relaxation the desired rate at which velocity errors are
     * corrected. This can be seen as the strength of the limits. A low value
     * will make the limits more spongy. (default=1)
     */
    public void setLimit(float low, float high, float softness, float bias,
            float relaxation) {
        long constraintId = nativeId();
        this.biasFactor = bias;
        this.relaxationFactor = relaxation;
        this.limitSoftness = softness;
        setLimit(constraintId, low, high, softness, bias, relaxation);
    }
    // *************************************************************************
    // Constraint methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned object into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this joint (not null)
     * @param original the instance from which this joint was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        assert !hasAssignedNativeObject();
        HingeJoint old = (HingeJoint) original;
        assert old != this;
        assert old.hasAssignedNativeObject();
        assert !hasAssignedNativeObject();

        super.cloneFields(cloner, original);
        if (hasAssignedNativeObject()) {
            return;
        }

        this.axisA = cloner.clone(axisA);
        this.axisB = cloner.clone(axisB);
        createJoint();

        setAngularOnly(angularOnly);
        copyConstraintProperties(old);

        float low = old.getLowerLimit();
        float high = old.getUpperLimit();
        setLimit(low, high, limitSoftness, biasFactor, relaxationFactor);

        boolean enable = old.getEnableMotor();
        float targetVelocity = old.getMotorTargetVelocity();
        float maxImpulse = old.getMaxMotorImpulse();
        enableMotor(enable, targetVelocity, maxImpulse);
    }

    /**
     * De-serialize this joint from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);
        InputCapsule capsule = importer.getCapsule(this);

        this.axisA = (Vector3f) capsule.readSavable(tagAxisA, new Vector3f());
        this.axisB = (Vector3f) capsule.readSavable(tagAxisB, new Vector3f());

        this.angularOnly = capsule.readBoolean(tagAngularOnly, false);

        float lowerLimit = capsule.readFloat(tagLowerLimit, 1e30f);
        float upperLimit = capsule.readFloat(tagUpperLimit, -1e30f);
        this.biasFactor = capsule.readFloat(tagBiasFactor, 0.3f);
        this.relaxationFactor = capsule.readFloat(tagRelaxationFactor, 1f);
        this.limitSoftness = capsule.readFloat(tagLimitSoftness, 0.9f);

        createJoint();
        readConstraintProperties(capsule);

        boolean enableAngularMotor
                = capsule.readBoolean(tagEnableAngularMotor, false);
        float targetVelocity = capsule.readFloat(tagTargetVelocity, 0f);
        float maxMotorImpulse = capsule.readFloat(tagMaxMotorImpulse, 0f);
        enableMotor(enableAngularMotor, targetVelocity, maxMotorImpulse);

        setAngularOnly(angularOnly);
        setLimit(lowerLimit, upperLimit, limitSoftness, biasFactor,
                relaxationFactor);
    }

    /**
     * Serialize this joint to the specified exporter, for example when saving
     * to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        super.write(exporter);
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(axisA, tagAxisA, null);
        capsule.write(axisB, tagAxisB, null);

        capsule.write(angularOnly, tagAngularOnly, false);

        capsule.write(getLowerLimit(), tagLowerLimit, 1e30f);
        capsule.write(getUpperLimit(), tagUpperLimit, -1e30f);

        capsule.write(biasFactor, tagBiasFactor, 0.3f);
        capsule.write(relaxationFactor, tagRelaxationFactor, 1f);
        capsule.write(limitSoftness, tagLimitSoftness, 0.9f);

        capsule.write(getEnableMotor(), tagEnableAngularMotor, false);
        capsule.write(getMotorTargetVelocity(), tagTargetVelocity, 0f);
        capsule.write(getMaxMotorImpulse(), tagMaxMotorImpulse, 0f);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Create the configured joint in Bullet.
     */
    private void createJoint() {
        PhysicsRigidBody a = getBodyA();
        long aId = a.nativeId();
        assert pivotA != null;
        assert axisA.isUnitVector() : axisA;

        assert pivotB != null;
        assert axisB.isUnitVector() : axisB;
        PhysicsRigidBody b = getBodyB();

        long constraintId;
        if (b == null) {
            /*
             * Create a single-ended joint.  Bullet assumes single-ended
             * btHingeConstraints are satisfied at creation, so we
             * temporarily re-position the body to satisfy the constraint.
             */
            Vector3f saveLocation = a.getPhysicsLocation(null);
            Quaternion saveRotation = a.getPhysicsRotation(null);

            Vector3f cross = axisB.cross(axisA);
            float sinAngle = cross.length();
            float cosAngle = axisB.dot(axisA);
            float angle = FastMath.atan2(sinAngle, cosAngle);
            MyVector3f.normalizeLocal(cross);
            Quaternion rotation = new Quaternion();
            rotation.fromAngleNormalAxis(angle, cross);
            a.setPhysicsRotation(rotation);

            Vector3f offset = pivotB.subtract(pivotA);
            a.setPhysicsLocation(offset);

            constraintId = createJoint1(aId, pivotA, axisA, useReferenceFrameA);

            a.setPhysicsLocation(saveLocation);
            a.setPhysicsRotation(saveRotation);

        } else { // Create a double-ended joint.
            assert !useReferenceFrameA;
            long bId = b.nativeId();
            constraintId = createJoint(aId, bId, pivotA, axisA, pivotB, axisB);
        }

        assert getConstraintType(constraintId) == 4;
        setNativeId(constraintId);
    }
    // *************************************************************************
    // native private methods

    native private static long createJoint(long objectIdA, long objectIdB,
            Vector3f pivotInA, Vector3f axisInA, Vector3f pivotInB,
            Vector3f axisInB);

    native private static long createJoint1(long objectIdA, Vector3f pivotInA,
            Vector3f axisInA, boolean useReferenceFrameA);

    native private static void enableMotor(long jointId, boolean enable,
            float targetVelocity, float maxMotorImpulse);

    native private static boolean getEnableAngularMotor(long jointId);

    native private static void
            getFrameOffsetA(long jointId, Transform frameInA);

    native private static void
            getFrameOffsetB(long jointId, Transform frameInB);

    native private static float getHingeAngle(long jointId);

    native private static float getLowerLimit(long jointId);

    native private static float getMaxMotorImpulse(long jointId);

    native private static float getMotorTargetVelocity(long jointId);

    native private static float getUpperLimit(long jointId);

    native private static void
            setAngularOnly(long jointId, boolean angularOnly);

    native private static void setLimit(long jointId, float low, float high,
            float softness, float biasFactor, float relaxationFactor);
}
