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
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;

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
public class HingeJoint extends PhysicsJoint {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(HingeJoint.class.getName());
    // *************************************************************************
    // fields

    /**
     * copy of the "use reference frame A" flag (default=false)
     */
    private boolean useReferenceFrameA = false;
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
    /**
     * copy of the angular-only flag (default=false)
     */
    private boolean angularOnly = false;
    /**
     * copy of the limit's bias factor: how strictly position errors (drift) is
     * corrected (default=0.3)
     */
    private float biasFactor = 0.3f;
    /**
     * copy of the limit's relaxation factor: the rate at which velocity errors
     * are corrected (default=1)
     */
    private float relaxationFactor = 1f;
    /**
     * copy of the limit's softness: the range fraction at which velocity-error
     * correction starts operating (default=0.9)
     */
    private float limitSoftness = 0.9f;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public HingeJoint() {
    }

    /**
     * Instantiate a single-ended HingeJoint.
     * <p>
     * To be effective, the joint must be added to the physics space of the body
     * and the body must be dynamic.
     *
     * @param nodeA the body to constrain (not null, alias created)
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
    public HingeJoint(PhysicsRigidBody nodeA, Vector3f pivotInA,
            Vector3f pivotInWorld, Vector3f axisInA, Vector3f axisInWorld,
            JointEnd referenceFrame) {
        super(nodeA, JointEnd.A, pivotInA, pivotInWorld);

        assert axisInA.isUnitVector() : axisInA;
        assert axisInWorld.isUnitVector() : axisInWorld;
        axisA = axisInA.clone();
        axisB = axisInWorld.clone();
        useReferenceFrameA = (referenceFrame == JointEnd.A);
        createJoint();
        /*
         * Synchronize the btHingeConstraint parameters with the local copies.
         */
        setAngularOnly(objectId, angularOnly);

        float low = getLowerLimit();
        float high = getUpperLimit();
        setLimit(objectId, low, high, limitSoftness, biasFactor,
                relaxationFactor);
    }

    /**
     * Instantiate a double-ended HingeJoint.
     * <p>
     * To be effective, the joint must be added to the physics space of the 2
     * bodies. Also, the bodies must be dynamic and distinct.
     *
     * @param nodeA the body for the A end (not null, alias created)
     * @param nodeB the body for the B end (not null, alias created)
     * @param pivotInA the pivot location in A's scaled local coordinates (not
     * null, unaffected)
     * @param pivotInB the pivot location in B's scaled local coordinates (not
     * null, unaffected)
     * @param axisInA the joint axis in A's local coordinates (unit vector,
     * unaffected)
     * @param axisInB the joint axis in B's local coordinates (unit vector,
     * unaffected)
     */
    public HingeJoint(PhysicsRigidBody nodeA, PhysicsRigidBody nodeB,
            Vector3f pivotInA, Vector3f pivotInB, Vector3f axisInA,
            Vector3f axisInB) {
        super(nodeA, nodeB, pivotInA, pivotInB);

        assert axisInA.isUnitVector() : axisInA;
        assert axisInB.isUnitVector() : axisInB;
        axisA = axisInA.clone();
        axisB = axisInB.clone();
        createJoint();
        /*
         * Synchronize btHingeConstraint parameters with local copies.
         */
        setAngularOnly(objectId, angularOnly);

        float low = getLowerLimit();
        float high = getUpperLimit();
        setLimit(objectId, low, high, limitSoftness, biasFactor,
                relaxationFactor);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Enable or disable this joint's motor.
     *
     * @param enable true to enable, false to disable
     * @param targetVelocity the desired target velocity
     * @param maxMotorImpulse the desired maximum rotational force
     */
    public void enableMotor(boolean enable, float targetVelocity,
            float maxMotorImpulse) {
        enableMotor(objectId, enable, targetVelocity, maxMotorImpulse);
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
        return getEnableAngularMotor(objectId);
    }

    /**
     * Read the hinge angle.
     *
     * @return the angle (in radians)
     */
    public float getHingeAngle() {
        return getHingeAngle(objectId);
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
        return getLowerLimit(objectId);
    }

    /**
     * Read the motor's target velocity.
     *
     * @return velocity
     */
    public float getMotorTargetVelocity() {
        return getMotorTargetVelocity(objectId);
    }

    /**
     * Read the motor's maximum impulse.
     *
     * @return impulse
     */
    public float getMaxMotorImpulse() {
        return getMaxMotorImpulse(objectId);
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
        return getUpperLimit(objectId);
    }

    /**
     * Test whether this joint is angular-only.
     *
     * @return true if angular only, otherwise false
     */
    public boolean isAngularOnly() {
        return angularOnly;
    }

    /**
     * Alter whether this joint is angular-only.
     *
     * @param angularOnly the desired setting (default=false)
     */
    public void setAngularOnly(boolean angularOnly) {
        this.angularOnly = angularOnly;
        setAngularOnly(objectId, angularOnly);
    }

    /**
     * Alter the angular limits for this joint.
     * <p>
     * If you're above the softness, velocities that would shoot through the
     * actual limit are slowed down. The bias should be in the range of 0.2 -
     * 0.5.
     *
     * @param low the desired lower limit of the hinge angle (in radians)
     * @param high the desired upper limit of the joint angle (in radians)
     * @param _softness the desired range fraction at which velocity-error
     * correction starts operating. A softness of 0.9 means that the correction
     * starts at 90% of the limit range. (default=0.9)
     * @param _biasFactor the desired magnitude of the position correction: how
     * strictly position errors (drift) are corrected (default=0.3)
     * @param _relaxationFactor the desired rate at which velocity errors are
     * corrected. This can be seen as the strength of the limits. A low value
     * will make the limits more spongy. (default=1)
     */
    public void setLimit(float low, float high, float _softness,
            float _biasFactor, float _relaxationFactor) {
        biasFactor = _biasFactor;
        relaxationFactor = _relaxationFactor;
        limitSoftness = _softness;
        setLimit(objectId, low, high, _softness, _biasFactor, _relaxationFactor);
    }
    // *************************************************************************
    // PhysicsJoint methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned object into a deep-cloned one, using the specified cloner
     * and original to resolve copied fields.
     *
     * @param cloner the cloner that's cloning this shape (not null)
     * @param original the instance from which this instance was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);

        axisA = cloner.clone(axisA);
        axisB = cloner.clone(axisB);
        createJoint();

        setAngularOnly(angularOnly);

        HingeJoint old = (HingeJoint) original;

        float bit = old.getBreakingImpulseThreshold();
        setBreakingImpulseThreshold(bit);

        boolean enableJoint = old.isEnabled();
        setEnabled(enableJoint);

        float low = old.getLowerLimit();
        float high = old.getUpperLimit();
        setLimit(low, high, limitSoftness, biasFactor, relaxationFactor);

        boolean enable = old.getEnableMotor();
        float targetVelocity = old.getMotorTargetVelocity();
        float maxImpulse = old.getMaxMotorImpulse();
        enableMotor(enable, targetVelocity, maxImpulse);
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public HingeJoint jmeClone() {
        try {
            HingeJoint clone = (HingeJoint) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }

    /**
     * De-serialize this joint, for example when loading from a J3O file.
     *
     * @param im importer (not null)
     * @throws IOException from importer
     */
    @Override
    public void read(JmeImporter im) throws IOException {
        super.read(im);
        InputCapsule capsule = im.getCapsule(this);

        float breakingImpulseThreshold = capsule.readFloat(
                "breakingImpulseThreshold", Float.MAX_VALUE);
        boolean isEnabled = capsule.readBoolean("isEnabled", true);

        axisA = (Vector3f) capsule.readSavable("axisA", new Vector3f());
        axisB = (Vector3f) capsule.readSavable("axisB", new Vector3f());

        angularOnly = capsule.readBoolean("angularOnly", false);

        float lowerLimit = capsule.readFloat("lowerLimit", 1e30f);
        float upperLimit = capsule.readFloat("upperLimit", -1e30f);
        biasFactor = capsule.readFloat("biasFactor", 0.3f);
        relaxationFactor = capsule.readFloat("relaxationFactor", 1f);
        limitSoftness = capsule.readFloat("limitSoftness", 0.9f);

        boolean enableAngularMotor
                = capsule.readBoolean("enableAngularMotor", false);
        float targetVelocity = capsule.readFloat("targetVelocity", 0f);
        float maxMotorImpulse = capsule.readFloat("maxMotorImpulse", 0f);

        createJoint();

        setBreakingImpulseThreshold(breakingImpulseThreshold);
        setEnabled(isEnabled);

        enableMotor(enableAngularMotor, targetVelocity, maxMotorImpulse);
        setAngularOnly(angularOnly);
        setLimit(lowerLimit, upperLimit, limitSoftness, biasFactor,
                relaxationFactor);
    }

    /**
     * Serialize this joint, for example when saving to a J3O file.
     *
     * @param ex exporter (not null)
     * @throws IOException from exporter
     */
    @Override
    public void write(JmeExporter ex) throws IOException {
        super.write(ex);
        OutputCapsule capsule = ex.getCapsule(this);

        capsule.write(axisA, "axisA", null);
        capsule.write(axisB, "axisB", null);

        capsule.write(angularOnly, "angularOnly", false);

        capsule.write(getLowerLimit(), "lowerLimit", 1e30f);
        capsule.write(getUpperLimit(), "upperLimit", -1e30f);

        capsule.write(biasFactor, "biasFactor", 0.3f);
        capsule.write(relaxationFactor, "relaxationFactor", 1f);
        capsule.write(limitSoftness, "limitSoftness", 0.9f);

        capsule.write(getEnableMotor(), "enableAngularMotor", false);
        capsule.write(getMotorTargetVelocity(), "targetVelocity", 0f);
        capsule.write(getMaxMotorImpulse(), "maxMotorImpulse", 0f);
    }
    // *************************************************************************
    // private methods

    /**
     * Create the configured joint in Bullet.
     */
    private void createJoint() {
        assert objectId == 0L;
        assert nodeA != null;
        assert pivotA != null;
        assert axisA.isUnitVector() : axisA;
        assert pivotB != null;
        assert axisB.isUnitVector() : axisB;

        if (nodeB == null) {
            /*
             * Create a single-ended joint.  Bullet assumes single-ended
             * btHingeConstraints are satisfied at creation, so we
             * temporarily re-position the body to satisfy the constraint.
             */
            Vector3f saveLocation = nodeA.getPhysicsLocation(null);
            Quaternion saveRotation = nodeA.getPhysicsRotation(null);

            Vector3f cross = axisB.cross(axisA);
            float sinAngle = cross.length();
            float cosAngle = axisB.dot(axisA);
            float angle = FastMath.atan2(sinAngle, cosAngle);
            cross.normalizeLocal();
            Quaternion rotation = new Quaternion();
            rotation.fromAngleNormalAxis(angle, cross);
            nodeA.setPhysicsRotation(rotation);

            Vector3f offset = pivotB.subtract(pivotA);
            nodeA.setPhysicsLocation(offset);

            objectId = createJoint1(nodeA.getObjectId(), pivotA, axisA,
                    useReferenceFrameA);

            nodeA.setPhysicsLocation(saveLocation);
            nodeA.setPhysicsRotation(saveRotation);

        } else {
            /*
             * Create a double-ended joint.
             */
            assert !useReferenceFrameA;
            objectId = createJoint(nodeA.getObjectId(), nodeB.getObjectId(),
                    pivotA, axisA, pivotB, axisB);
        }
        assert objectId != 0L;
        logger2.log(Level.FINE, "Created Joint {0}",
                Long.toHexString(objectId));
    }

    native private long createJoint(long objectIdA, long objectIdB,
            Vector3f pivotInA, Vector3f axisInA, Vector3f pivotInB,
            Vector3f axisInB);

    native private long createJoint1(long objectIdA, Vector3f pivotInA,
            Vector3f axisInA, boolean useReferenceFrameA);

    native private void enableMotor(long jointId, boolean enable,
            float targetVelocity, float maxMotorImpulse);

    native private boolean getEnableAngularMotor(long jointId);

    native private float getHingeAngle(long jointId);

    native private float getLowerLimit(long jointId);

    native private float getMaxMotorImpulse(long jointId);

    native private float getMotorTargetVelocity(long jointId);

    native private float getUpperLimit(long jointId);

    native private void setAngularOnly(long jointId, boolean angularOnly);

    native private void setLimit(long jointId, float low, float high,
            float softness, float biasFactor, float relaxationFactor);
}
