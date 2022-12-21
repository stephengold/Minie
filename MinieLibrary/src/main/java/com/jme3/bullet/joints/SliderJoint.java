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
import com.jme3.math.Matrix3f;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Logger;

/**
 * A 2 degree-of-freedom joint based on Bullet's btSliderConstraint. The axis of
 * the joint always parallels the physics-space X axis.
 * <p>
 * <i>From the Bullet manual:</i><br>
 * The slider constraint allows the body to rotate around one axis and translate
 * along this axis.
 *
 * @author normenhansen
 */
public class SliderJoint extends Constraint {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(SliderJoint.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagDampingDirAng = "dampingDirAng";
    final private static String tagDampingDirLin = "dampingDirLin";
    final private static String tagDampingLimAng = "dampingLimAng";
    final private static String tagDampingLimLin = "dampingLimLin";
    final private static String tagDampingOrthoAng = "dampingOrthoAng";
    final private static String tagDampingOrthoLin = "dampingOrthoLin";
    final private static String tagLowerAngLimit = "lowerAngLimit";
    final private static String tagLowerLinLimit = "lowerLinLimit";
    final private static String tagMaxAngMotorForce = "maxAngMotorForce";
    final private static String tagMaxLinMotorForce = "maxLinMotorForce";
    final private static String tagPoweredAngMotor = "poweredAngMotor";
    final private static String tagPoweredLinMotor = "poweredLinMotor";
    final private static String tagRestitutionDirAng = "restitutionDirAng";
    final private static String tagRestitutionDirLin = "restitutionDirLin";
    final private static String tagRestitutionLimAng = "restitutionLimAng";
    final private static String tagRestitutionLimLin = "restitutionLimLin";
    final private static String tagRestitutionOrthoAng = "restitutionOrthoAng";
    final private static String tagRestitutionOrthoLin = "restitutionOrthoLin";
    final private static String tagRotA = "rotA";
    final private static String tagRotB = "rotB";
    final private static String tagSoftnessDirAng = "softnessDirAng";
    final private static String tagSoftnessDirLin = "softnessDirLin";
    final private static String tagSoftnessLimAng = "softnessLimAng";
    final private static String tagSoftnessLimLin = "softnessLimLin";
    final private static String tagSoftnessOrthoAng = "softnessOrthoAng";
    final private static String tagSoftnessOrthoLin = "softnessOrthoLin";
    final private static String tagTargetAngMotorVelocity
            = "targetAngMotorVelocity";
    final private static String tagTargetLinMotorVelocity
            = "targetLinMotorVelocity";
    final private static String tagUpperAngLimit = "upperAngLimit";
    final private static String tagUpperLinLimit = "upperLinLimit";
    final private static String tagUseLinearReferenceFrameA
            = "useLinearReferenceFrameA";
    // *************************************************************************
    // fields

    /**
     * true&rarr;limits give the allowable range of movement of frameB in frameA
     * space, false&rarr;limits give the allowable range of movement of frameA
     * in frameB space
     */
    private boolean useLinearReferenceFrameA;
    /**
     * copy of the joint orientation: in physics-space coordinates if bodyA is
     * null, or else in A's local coordinates (rotation matrix)
     */
    private Matrix3f rotA;
    /**
     * copy of the joint orientation in B's local coordinates (rotation matrix)
     */
    private Matrix3f rotB;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected SliderJoint() {
    }

    /**
     * Instantiate a single-ended SliderJoint.
     * <p>
     * To be effective, the joint must be added to the body's PhysicsSpace and
     * the body must be dynamic.
     *
     * @param rigidBodyB the body to constrain (not null, alias created)
     * @param pivotInB the pivot location in B's scaled local coordinates (not
     * null, unaffected)
     * @param pivotInWorld the pivot location in physics-space coordinates (not
     * null, unaffected)
     * @param linearReferenceFrame which end to use as the linear reference (not
     * null)
     */
    public SliderJoint(PhysicsRigidBody rigidBodyB, Vector3f pivotInB,
            Vector3f pivotInWorld, JointEnd linearReferenceFrame) {
        super(rigidBodyB, JointEnd.B, pivotInB, pivotInWorld);
        this.rotA = new Matrix3f();
        this.rotB = new Matrix3f();

        this.useLinearReferenceFrameA = (linearReferenceFrame == JointEnd.A);
        createJoint();
    }

    /**
     * Instantiate a double-ended SliderJoint.
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
     * @param rotInA the joint orientation in A's local coordinates (not null,
     * alias unaffected)
     * @param rotInB the joint orientation in B's local coordinates (not null,
     * alias unaffected)
     * @param useLinearReferenceFrameA true&rarr;use body A, false&rarr;use body
     * B
     */
    public SliderJoint(PhysicsRigidBody rigidBodyA, PhysicsRigidBody rigidBodyB,
            Vector3f pivotInA, Vector3f pivotInB, Matrix3f rotInA,
            Matrix3f rotInB, boolean useLinearReferenceFrameA) {
        super(rigidBodyA, rigidBodyB, pivotInA, pivotInB);

        this.useLinearReferenceFrameA = useLinearReferenceFrameA;
        this.rotA = rotInA.clone();
        this.rotB = rotInB.clone();
        createJoint();
    }

    /**
     * Instantiate a double-ended SliderJoint.
     * <p>
     * To be effective, the joint must be added to the body's PhysicsSpace and
     * the body must be dynamic.
     *
     * @param rigidBodyA the first body to constrain (not null, alias created)
     * @param rigidBodyB the 2nd body to constrain (not null, alias created)
     * @param pivotInA the pivot location in A's scaled local coordinates (not
     * null, unaffected)
     * @param pivotInB the pivot location in B's scaled local coordinates (not
     * null, unaffected)
     * @param useLinearReferenceFrameA true&rarr;use body A, false&rarr;use body
     * B
     */
    public SliderJoint(PhysicsRigidBody rigidBodyA, PhysicsRigidBody rigidBodyB,
            Vector3f pivotInA, Vector3f pivotInB,
            boolean useLinearReferenceFrameA) {
        super(rigidBodyA, rigidBodyB, pivotInA, pivotInB);

        this.useLinearReferenceFrameA = useLinearReferenceFrameA;
        this.rotA = new Matrix3f();
        this.rotB = new Matrix3f();
        createJoint();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Read the joint's damping for on-axis rotation between the limits.
     *
     * @return the viscous damping ratio (0&rarr;no damping, 1&rarr;critically
     * damped)
     */
    public float getDampingDirAng() {
        long constraintId = nativeId();
        float result = getDampingDirAng(constraintId);

        return result;
    }

    /**
     * Read the joint's damping for on-axis translation between the limits.
     *
     * @return the viscous damping ratio (0&rarr;no damping, 1&rarr;critically
     * damped)
     */
    public float getDampingDirLin() {
        long constraintId = nativeId();
        float result = getDampingDirLin(constraintId);

        return result;
    }

    /**
     * Read the joint's damping for on-axis rotation hitting the limits.
     *
     * @return the viscous damping ratio (0&rarr;no damping, 1&rarr;critically
     * damped)
     */
    public float getDampingLimAng() {
        long constraintId = nativeId();
        float result = getDampingLimAng(constraintId);

        return result;
    }

    /**
     * Read the joint's damping for on-axis translation hitting the limits.
     *
     * @return the viscous damping ratio (0&rarr;no damping, 1&rarr;critically
     * damped)
     */
    public float getDampingLimLin() {
        long constraintId = nativeId();
        float result = getDampingLimLin(constraintId);

        return result;
    }

    /**
     * Read the joint's damping for off-axis rotation.
     *
     * @return the viscous damping ratio (0&rarr;no damping, 1&rarr;critically
     * damped)
     */
    public float getDampingOrthoAng() {
        long constraintId = nativeId();
        float result = getDampingOrthoAng(constraintId);

        return result;
    }

    /**
     * Read the joint's damping for off-axis translation.
     *
     * @return the viscous damping ratio (0&rarr;no damping, 1&rarr;critically
     * damped)
     */
    public float getDampingOrthoLin() {
        long constraintId = nativeId();
        float result = getDampingOrthoLin(constraintId);

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
     * Read the joint's lower limit for on-axis rotation.
     *
     * @return the lower limit angle (in radians)
     */
    public float getLowerAngLimit() {
        long constraintId = nativeId();
        float result = getLowerAngLimit(constraintId);

        return result;
    }

    /**
     * Read the joint's lower limit for on-axis translation.
     *
     * @return the lower limit
     */
    public float getLowerLinLimit() {
        long constraintId = nativeId();
        float result = getLowerLinLimit(constraintId);

        return result;
    }

    /**
     * Read the maximum force of the rotation motor.
     *
     * @return the maximum force
     */
    public float getMaxAngMotorForce() {
        long constraintId = nativeId();
        float result = getMaxAngMotorForce(constraintId);

        return result;
    }

    /**
     * Read the maximum force of the translation motor.
     *
     * @return the maximum force
     */
    public float getMaxLinMotorForce() {
        long constraintId = nativeId();
        float result = getMaxLinMotorForce(constraintId);

        return result;
    }

    /**
     * Read the joint's restitution for on-axis rotation between the limits.
     *
     * @return the restitution (bounce) factor
     */
    public float getRestitutionDirAng() {
        long constraintId = nativeId();
        float result = getRestitutionDirAng(constraintId);

        return result;
    }

    /**
     * Read the joint's restitution for on-axis translation between the limits.
     *
     * @return the restitution (bounce) factor
     */
    public float getRestitutionDirLin() {
        long constraintId = nativeId();
        float result = getRestitutionDirLin(constraintId);

        return result;
    }

    /**
     * Read the joint's restitution for on-axis rotation hitting the limits.
     *
     * @return the restitution (bounce) factor
     */
    public float getRestitutionLimAng() {
        long constraintId = nativeId();
        float result = getRestitutionLimAng(constraintId);

        return result;
    }

    /**
     * Read the joint's restitution for on-axis translation hitting the limits.
     *
     * @return the restitution (bounce) factor
     */
    public float getRestitutionLimLin() {
        long constraintId = nativeId();
        float result = getRestitutionLimLin(constraintId);

        return result;
    }

    /**
     * Read the joint's restitution for off-axis rotation.
     *
     * @return the restitution (bounce) factor
     */
    public float getRestitutionOrthoAng() {
        long constraintId = nativeId();
        float result = getRestitutionOrthoAng(constraintId);

        return result;
    }

    /**
     * Read the joint's restitution for off-axis translation.
     *
     * @return the restitution (bounce) factor
     */
    public float getRestitutionOrthoLin() {
        long constraintId = nativeId();
        float result = getRestitutionOrthoLin(constraintId);

        return result;
    }

    /**
     * Read the joint's softness for on-axis rotation between the limits.
     *
     * @return the softness
     */
    public float getSoftnessDirAng() {
        long constraintId = nativeId();
        float result = getSoftnessDirAng(constraintId);

        return result;
    }

    /**
     * Read the joint's softness for on-axis translation between the limits.
     *
     * @return the softness
     */
    public float getSoftnessDirLin() {
        long constraintId = nativeId();
        float result = getSoftnessDirLin(constraintId);

        return result;
    }

    /**
     * Read the joint's softness for on-axis rotation hitting the limits.
     *
     * @return the softness
     */
    public float getSoftnessLimAng() {
        long constraintId = nativeId();
        float result = getSoftnessLimAng(constraintId);

        return result;
    }

    /**
     * Read the joint's softness for on-axis translation hitting the limits.
     *
     * @return the softness
     */
    public float getSoftnessLimLin() {
        long constraintId = nativeId();
        float result = getSoftnessLimLin(constraintId);

        return result;
    }

    /**
     * Read the joint's softness for off-axis rotation.
     *
     * @return the softness
     */
    public float getSoftnessOrthoAng() {
        long constraintId = nativeId();
        float result = getSoftnessOrthoAng(constraintId);

        return result;
    }

    /**
     * Read the joint's softness for off-axis translation.
     *
     * @return the softness
     */
    public float getSoftnessOrthoLin() {
        long constraintId = nativeId();
        float result = getSoftnessOrthoLin(constraintId);

        return result;
    }

    /**
     * Read the velocity target of the rotation motor.
     *
     * @return the velocity target (in radians per second)
     */
    public float getTargetAngMotorVelocity() {
        long constraintId = nativeId();
        float result = getTargetAngMotorVelocity(constraintId);

        return result;
    }

    /**
     * Read the velocity target of the translation motor.
     *
     * @return the velocity target
     */
    public float getTargetLinMotorVelocity() {
        long constraintId = nativeId();
        float result = getTargetLinMotorVelocity(constraintId);

        return result;
    }

    /**
     * Read the joint's upper limit for on-axis rotation.
     *
     * @return the upper limit angle (in radians)
     */
    public float getUpperAngLimit() {
        long constraintId = nativeId();
        float result = getUpperAngLimit(constraintId);

        return result;
    }

    /**
     * Read the joint's upper limit for on-axis translation.
     *
     * @return the upper limit
     */
    public float getUpperLinLimit() {
        long constraintId = nativeId();
        float result = getUpperLinLimit(constraintId);

        return result;
    }

    /**
     * Test whether the rotation motor is powered.
     *
     * @return true if powered, otherwise false
     */
    public boolean isPoweredAngMotor() {
        long constraintId = nativeId();
        boolean result = isPoweredAngMotor(constraintId);

        return result;
    }

    /**
     * Test whether the translation motor is powered.
     *
     * @return true if powered, otherwise false
     */
    public boolean isPoweredLinMotor() {
        long constraintId = nativeId();
        boolean result = isPoweredLinMotor(constraintId);

        return result;
    }

    /**
     * Alter the joint's damping for on-axis rotation between the limits.
     *
     * @param dampingDirAng the desired viscous damping ratio (0&rarr;no
     * damping, 1&rarr;critically damped, default=0)
     */
    public void setDampingDirAng(float dampingDirAng) {
        long constraintId = nativeId();
        setDampingDirAng(constraintId, dampingDirAng);
    }

    /**
     * Alter the joint's damping for on-axis translation between the limits.
     *
     * @param dampingDirLin the desired viscous damping ratio (0&rarr;no
     * damping, 1&rarr;critically damped, default=0)
     */
    public void setDampingDirLin(float dampingDirLin) {
        long constraintId = nativeId();
        setDampingDirLin(constraintId, dampingDirLin);
    }

    /**
     * Alter the joint's damping for on-axis rotation hitting the limits.
     *
     * @param dampingLimAng the desired viscous damping ratio (0&rarr;no
     * damping, 1&rarr;critically damped, default=1)
     */
    public void setDampingLimAng(float dampingLimAng) {
        long constraintId = nativeId();
        setDampingLimAng(constraintId, dampingLimAng);
    }

    /**
     * Alter the joint's damping for on-axis translation hitting the limits.
     *
     * @param dampingLimLin the desired viscous damping ratio (0&rarr;no
     * damping, 1&rarr;critically damped, default=1)
     */
    public void setDampingLimLin(float dampingLimLin) {
        long constraintId = nativeId();
        setDampingLimLin(constraintId, dampingLimLin);
    }

    /**
     * Alter the joint's damping for off-axis rotation.
     *
     * @param dampingOrthoAng the desired viscous damping ratio (0&rarr;no
     * damping, 1&rarr;critically damped, default=1)
     */
    public void setDampingOrthoAng(float dampingOrthoAng) {
        long constraintId = nativeId();
        setDampingOrthoAng(constraintId, dampingOrthoAng);
    }

    /**
     * Alter the joint's damping for off-axis translation.
     *
     * @param dampingOrthoLin the desired viscous damping ratio (0&rarr;no
     * damping, 1&rarr;critically damped, default=1)
     */
    public void setDampingOrthoLin(float dampingOrthoLin) {
        long constraintId = nativeId();
        setDampingOrthoLin(constraintId, dampingOrthoLin);
    }

    /**
     * Alter the joint's lower limit for on-axis rotation.
     *
     * @param lowerAngLimit the desired lower limit angle (in radians,
     * default=0)
     */
    public void setLowerAngLimit(float lowerAngLimit) {
        long constraintId = nativeId();
        setLowerAngLimit(constraintId, lowerAngLimit);
    }

    /**
     * Alter the joint's lower limit for on-axis translation.
     *
     * @param lowerLinLimit the desired lower limit (default=1)
     */
    public void setLowerLinLimit(float lowerLinLimit) {
        long constraintId = nativeId();
        setLowerLinLimit(constraintId, lowerLinLimit);
    }

    /**
     * Alter the maximum force of the rotation motor.
     *
     * @param maxAngMotorForce the desired maximum force (default=0)
     */
    public void setMaxAngMotorForce(float maxAngMotorForce) {
        long constraintId = nativeId();
        setMaxAngMotorForce(constraintId, maxAngMotorForce);
    }

    /**
     * Alter the maximum force of the translation motor.
     *
     * @param maxLinMotorForce the desired maximum force (default=0)
     */
    public void setMaxLinMotorForce(float maxLinMotorForce) {
        long constraintId = nativeId();
        setMaxLinMotorForce(constraintId, maxLinMotorForce);
    }

    /**
     * Alter whether the rotation motor is powered.
     *
     * @param poweredAngMotor true to power the motor, false to de-power it
     * (default=false)
     */
    public void setPoweredAngMotor(boolean poweredAngMotor) {
        long constraintId = nativeId();
        setPoweredAngMotor(constraintId, poweredAngMotor);
    }

    /**
     * Alter whether the translation motor is powered.
     *
     * @param poweredLinMotor true to power the motor, false to de-power it
     * (default=false)
     */
    public void setPoweredLinMotor(boolean poweredLinMotor) {
        long constraintId = nativeId();
        setPoweredLinMotor(constraintId, poweredLinMotor);
    }

    /**
     * Alter the joint's restitution for on-axis rotation between the limits.
     *
     * @param restitutionDirAng the desired restitution (bounce) factor
     * (default=0.7)
     */
    public void setRestitutionDirAng(float restitutionDirAng) {
        long constraintId = nativeId();
        setRestitutionDirAng(constraintId, restitutionDirAng);
    }

    /**
     * Alter the joint's restitution for on-axis translation between the limits.
     *
     * @param restitutionDirLin the desired restitution (bounce) factor
     * (default=0.7)
     */
    public void setRestitutionDirLin(float restitutionDirLin) {
        long constraintId = nativeId();
        setRestitutionDirLin(constraintId, restitutionDirLin);
    }

    /**
     * Alter the joint's restitution for on-axis rotation hitting the limits.
     *
     * @param restitutionLimAng the desired restitution (bounce) factor
     * (default=0.7)
     */
    public void setRestitutionLimAng(float restitutionLimAng) {
        long constraintId = nativeId();
        setRestitutionLimAng(constraintId, restitutionLimAng);
    }

    /**
     * Alter the joint's restitution for on-axis translation hitting the limits.
     *
     * @param restitutionLimLin the desired restitution (bounce) factor
     * (default=0.7)
     */
    public void setRestitutionLimLin(float restitutionLimLin) {
        long constraintId = nativeId();
        setRestitutionLimLin(constraintId, restitutionLimLin);
    }

    /**
     * Alter the joint's restitution for off-axis rotation.
     *
     * @param restitutionOrthoAng the desired restitution (bounce) factor
     * (default=0.7)
     */
    public void setRestitutionOrthoAng(float restitutionOrthoAng) {
        long constraintId = nativeId();
        setRestitutionOrthoAng(constraintId, restitutionOrthoAng);
    }

    /**
     * Alter the joint's restitution for off-axis translation.
     *
     * @param restitutionOrthoLin the desired restitution (bounce) factor
     * (default=0.7)
     */
    public void setRestitutionOrthoLin(float restitutionOrthoLin) {
        long constraintId = nativeId();
        setRestitutionOrthoLin(constraintId, restitutionOrthoLin);
    }

    /**
     * Alter the joint's softness for on-axis rotation between the limits.
     *
     * @param softnessDirAng the desired softness (default=1)
     */
    public void setSoftnessDirAng(float softnessDirAng) {
        long constraintId = nativeId();
        setSoftnessDirAng(constraintId, softnessDirAng);
    }

    /**
     * Alter the joint's softness for on-axis translation between the limits.
     *
     * @param softnessDirLin the desired softness (default=1)
     */
    public void setSoftnessDirLin(float softnessDirLin) {
        long constraintId = nativeId();
        setSoftnessDirLin(constraintId, softnessDirLin);
    }

    /**
     * Alter the joint's softness for on-axis rotation hitting the limits.
     *
     * @param softnessLimAng the desired softness (default=1)
     */
    public void setSoftnessLimAng(float softnessLimAng) {
        long constraintId = nativeId();
        setSoftnessLimAng(constraintId, softnessLimAng);
    }

    /**
     * Alter the joint's softness for on-axis translation hitting the limits.
     *
     * @param softnessLimLin the desired softness (default=1)
     */
    public void setSoftnessLimLin(float softnessLimLin) {
        long constraintId = nativeId();
        setSoftnessLimLin(constraintId, softnessLimLin);
    }

    /**
     * Alter the joint's softness for off-axis rotation.
     *
     * @param softnessOrthoAng the desired softness (default=1)
     */
    public void setSoftnessOrthoAng(float softnessOrthoAng) {
        long constraintId = nativeId();
        setSoftnessOrthoAng(constraintId, softnessOrthoAng);
    }

    /**
     * Alter the joint's softness for off-axis translation.
     *
     * @param softnessOrthoLin the desired softness (default=1)
     */
    public void setSoftnessOrthoLin(float softnessOrthoLin) {
        long constraintId = nativeId();
        setSoftnessOrthoLin(constraintId, softnessOrthoLin);
    }

    /**
     * Alter the velocity target of the rotation motor.
     *
     * @param targetAngMotorVelocity the desired velocity target (in radians per
     * second, default=0)
     */
    public void setTargetAngMotorVelocity(float targetAngMotorVelocity) {
        long constraintId = nativeId();
        setTargetAngMotorVelocity(constraintId, targetAngMotorVelocity);
    }

    /**
     * Alter the velocity target of the translation motor.
     *
     * @param targetLinMotorVelocity the desired velocity target (default=0)
     */
    public void setTargetLinMotorVelocity(float targetLinMotorVelocity) {
        long constraintId = nativeId();
        setTargetLinMotorVelocity(constraintId, targetLinMotorVelocity);
    }

    /**
     * Alter the joint's upper limit for on-axis rotation.
     *
     * @param upperAngLimit the desired upper limit angle (in radians,
     * default=0)
     */
    public void setUpperAngLimit(float upperAngLimit) {
        long constraintId = nativeId();
        setUpperAngLimit(constraintId, upperAngLimit);
    }

    /**
     * Alter the joint's upper limit for on-axis translation.
     *
     * @param upperLinLimit the desired upper limit (default=-1)
     */
    public void setUpperLinLimit(float upperLinLimit) {
        long constraintId = nativeId();
        setUpperLinLimit(constraintId, upperLinLimit);
    }
    // *************************************************************************
    // Constraint methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned joint into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this joint (not null)
     * @param original the instance from which this joint was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        assert !hasAssignedNativeObject();
        SliderJoint old = (SliderJoint) original;
        assert old != this;
        assert old.hasAssignedNativeObject();

        super.cloneFields(cloner, original);
        if (hasAssignedNativeObject()) {
            return;
        }

        this.rotA = cloner.clone(rotA);
        this.rotB = cloner.clone(rotB);
        createJoint();

        copyConstraintProperties(old);

        setDampingDirAng(old.getDampingDirAng());
        setDampingDirLin(old.getDampingDirLin());
        setDampingLimAng(old.getDampingLimAng());
        setDampingLimLin(old.getDampingLimLin());
        setDampingOrthoAng(old.getDampingOrthoAng());
        setDampingOrthoLin(old.getDampingOrthoLin());

        setLowerAngLimit(old.getLowerAngLimit());
        setLowerLinLimit(old.getLowerLinLimit());

        setMaxAngMotorForce(old.getMaxAngMotorForce());
        setMaxLinMotorForce(old.getMaxLinMotorForce());

        setPoweredAngMotor(old.isPoweredAngMotor());
        setPoweredLinMotor(old.isPoweredLinMotor());

        setRestitutionDirAng(old.getRestitutionDirAng());
        setRestitutionDirLin(old.getRestitutionDirLin());
        setRestitutionLimAng(old.getRestitutionLimAng());
        setRestitutionLimLin(old.getRestitutionLimLin());
        setRestitutionOrthoAng(old.getRestitutionOrthoAng());
        setRestitutionOrthoLin(old.getRestitutionOrthoLin());

        setSoftnessDirAng(old.getSoftnessDirAng());
        setSoftnessDirLin(old.getSoftnessDirLin());
        setSoftnessLimAng(old.getSoftnessLimAng());
        setSoftnessLimLin(old.getSoftnessLimLin());
        setSoftnessOrthoAng(old.getSoftnessOrthoAng());
        setSoftnessOrthoLin(old.getSoftnessOrthoLin());

        setTargetAngMotorVelocity(old.getTargetAngMotorVelocity());
        setTargetLinMotorVelocity(old.getTargetLinMotorVelocity());

        setUpperAngLimit(old.getUpperAngLimit());
        setUpperLinLimit(old.getUpperLinLimit());
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

        float dampingDirAng = capsule.readFloat(tagDampingDirAng, 0f);
        float dampingDirLin = capsule.readFloat(tagDampingDirLin, 0f);
        float dampingLimAng = capsule.readFloat(tagDampingLimAng, 1f);
        float dampingLimLin = capsule.readFloat(tagDampingLimLin, 1f);
        float dampingOrthoAng = capsule.readFloat(tagDampingOrthoAng, 1f);
        float dampingOrthoLin = capsule.readFloat(tagDampingOrthoLin, 1f);

        float lowerAngLimit = capsule.readFloat(tagLowerAngLimit, 0f);
        float lowerLinLimit = capsule.readFloat(tagLowerLinLimit, 1f);
        float maxAngMotorForce = capsule.readFloat(tagMaxAngMotorForce, 0f);
        float maxLinMotorForce = capsule.readFloat(tagMaxLinMotorForce, 0f);
        boolean poweredAngMotor
                = capsule.readBoolean(tagPoweredAngMotor, false);
        boolean poweredLinMotor
                = capsule.readBoolean(tagPoweredLinMotor, false);

        float restitutionDirAng = capsule.readFloat(tagRestitutionDirAng, 0.7f);
        float restitutionDirLin = capsule.readFloat(tagRestitutionDirLin, 0.7f);
        float restitutionLimAng = capsule.readFloat(tagRestitutionLimAng, 0.7f);
        float restitutionLimLin = capsule.readFloat(tagRestitutionLimLin, 0.7f);
        float restitutionOrthoAng
                = capsule.readFloat(tagRestitutionOrthoAng, 0.7f);
        float restitutionOrthoLin
                = capsule.readFloat(tagRestitutionOrthoLin, 0.7f);

        float softnessDirAng = capsule.readFloat(tagSoftnessDirAng, 1f);
        float softnessDirLin = capsule.readFloat(tagSoftnessDirLin, 1f);
        float softnessLimAng = capsule.readFloat(tagSoftnessLimAng, 1f);
        float softnessLimLin = capsule.readFloat(tagSoftnessLimLin, 1f);
        float softnessOrthoAng = capsule.readFloat(tagSoftnessOrthoAng, 1f);
        float softnessOrthoLin = capsule.readFloat(tagSoftnessOrthoLin, 1f);

        float targetAngMotorVelocity
                = capsule.readFloat(tagTargetAngMotorVelocity, 0f);
        float targetLinMotorVelocity
                = capsule.readFloat(tagTargetLinMotorVelocity, 0f);

        float upperAngLimit = capsule.readFloat(tagUpperAngLimit, 0f);
        float upperLinLimit = capsule.readFloat(tagUpperLinLimit, -1f);

        this.rotA = (Matrix3f) capsule.readSavable(tagRotA, new Matrix3f());
        this.rotB = (Matrix3f) capsule.readSavable(tagRotB, new Matrix3f());
        this.useLinearReferenceFrameA = capsule.readBoolean(
                tagUseLinearReferenceFrameA, false);

        createJoint();
        readConstraintProperties(capsule);

        setDampingDirAng(dampingDirAng);
        setDampingDirLin(dampingDirLin);
        setDampingLimAng(dampingLimAng);
        setDampingLimLin(dampingLimLin);
        setDampingOrthoAng(dampingOrthoAng);
        setDampingOrthoLin(dampingOrthoLin);
        setLowerAngLimit(lowerAngLimit);
        setLowerLinLimit(lowerLinLimit);
        setMaxAngMotorForce(maxAngMotorForce);
        setMaxLinMotorForce(maxLinMotorForce);
        setPoweredAngMotor(poweredAngMotor);
        setPoweredLinMotor(poweredLinMotor);
        setRestitutionDirAng(restitutionDirAng);
        setRestitutionDirLin(restitutionDirLin);
        setRestitutionLimAng(restitutionLimAng);
        setRestitutionLimLin(restitutionLimLin);
        setRestitutionOrthoAng(restitutionOrthoAng);
        setRestitutionOrthoLin(restitutionOrthoLin);

        setSoftnessDirAng(softnessDirAng);
        setSoftnessDirLin(softnessDirLin);
        setSoftnessLimAng(softnessLimAng);
        setSoftnessLimLin(softnessLimLin);
        setSoftnessOrthoAng(softnessOrthoAng);
        setSoftnessOrthoLin(softnessOrthoLin);

        setTargetAngMotorVelocity(targetAngMotorVelocity);
        setTargetLinMotorVelocity(targetLinMotorVelocity);

        setUpperAngLimit(upperAngLimit);
        setUpperLinLimit(upperLinLimit);
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

        capsule.write(getDampingDirAng(), tagDampingDirAng, 0f);
        capsule.write(getDampingDirLin(), tagDampingDirLin, 0f);
        capsule.write(getDampingLimAng(), tagDampingLimAng, 1f);
        capsule.write(getDampingLimLin(), tagDampingLimLin, 1f);
        capsule.write(getDampingOrthoAng(), tagDampingOrthoAng, 1f);
        capsule.write(getDampingOrthoLin(), tagDampingOrthoLin, 1f);

        capsule.write(getLowerAngLimit(), tagLowerAngLimit, 0f);
        capsule.write(getLowerLinLimit(), tagLowerLinLimit, 1f);
        capsule.write(getMaxAngMotorForce(), tagMaxAngMotorForce, 0f);
        capsule.write(getMaxLinMotorForce(), tagMaxLinMotorForce, 0f);
        capsule.write(isPoweredAngMotor(), tagPoweredAngMotor, false);
        capsule.write(isPoweredLinMotor(), tagPoweredLinMotor, false);

        capsule.write(getRestitutionDirAng(), tagRestitutionDirAng, 0.7f);
        capsule.write(getRestitutionDirLin(), tagRestitutionDirLin, 0.7f);
        capsule.write(getRestitutionLimAng(), tagRestitutionLimAng, 0.7f);
        capsule.write(getRestitutionLimLin(), tagRestitutionLimLin, 0.7f);
        capsule.write(getRestitutionOrthoAng(), tagRestitutionOrthoAng, 0.7f);
        capsule.write(getRestitutionOrthoLin(), tagRestitutionOrthoLin, 0.7f);

        capsule.write(getSoftnessDirAng(), tagSoftnessDirAng, 1f);
        capsule.write(getSoftnessDirLin(), tagSoftnessDirLin, 1f);
        capsule.write(getSoftnessLimAng(), tagSoftnessLimAng, 1f);
        capsule.write(getSoftnessLimLin(), tagSoftnessLimLin, 1f);
        capsule.write(getSoftnessOrthoAng(), tagSoftnessOrthoAng, 1f);
        capsule.write(getSoftnessOrthoLin(), tagSoftnessOrthoLin, 1f);

        capsule.write(getTargetAngMotorVelocity(),
                tagTargetAngMotorVelocity, 0f);
        capsule.write(getTargetLinMotorVelocity(),
                tagTargetLinMotorVelocity, 0f);

        capsule.write(getUpperAngLimit(), tagUpperAngLimit, 0f);
        capsule.write(getUpperLinLimit(), tagUpperLinLimit, -1f);

        capsule.write(useLinearReferenceFrameA, tagUseLinearReferenceFrameA,
                false);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Create the configured joint in Bullet.
     */
    private void createJoint() {
        PhysicsRigidBody a = getBodyA();
        assert pivotA != null;
        assert rotA != null;

        PhysicsRigidBody b = getBodyB();
        long bId = b.nativeId();
        assert pivotB != null;
        assert rotB != null;

        long constraintId;
        if (a == null) {
            /*
             * Create a single-ended joint.  Bullet assumes single-ended
             * btSliderConstraints are satisfied at creation, so we
             * temporarily re-position the body to satisfy the constraint.
             */
            Vector3f saveLocation = b.getPhysicsLocation(null);

            Vector3f offset = pivotA.subtract(pivotB);
            b.setPhysicsLocation(offset);
            constraintId
                    = createJoint1(bId, pivotB, rotB, useLinearReferenceFrameA);

            b.setPhysicsLocation(saveLocation);

        } else { // Create a double-ended joint.
            long aId = a.nativeId();
            constraintId = createJoint(aId, bId, pivotA, rotA, pivotB, rotB,
                    useLinearReferenceFrameA);
        }

        assert getConstraintType(constraintId) == 7;
        setNativeId(constraintId);
    }
    // *************************************************************************
    // native private methods

    native private static long createJoint(long bodyIdA, long bodyIdB,
            Vector3f pivotInA, Matrix3f rotInA, Vector3f pivotInB,
            Matrix3f rotInB, boolean useLinearReferenceFrameA);

    native private static long createJoint1(long bodyIdB, Vector3f pivotInB,
            Matrix3f rotInB, boolean useLinearReferenceFrameA);

    native private static float getDampingDirAng(long jointId);

    native private static float getDampingDirLin(long jointId);

    native private static float getDampingLimAng(long jointId);

    native private static float getDampingLimLin(long jointId);

    native private static float getDampingOrthoAng(long jointId);

    native private static float getDampingOrthoLin(long jointId);

    native private static void
            getFrameOffsetA(long jointId, Transform frameInA);

    native private static void
            getFrameOffsetB(long jointId, Transform frameInB);

    native private static float getLowerAngLimit(long jointId);

    native private static float getLowerLinLimit(long jointId);

    native private static float getMaxAngMotorForce(long jointId);

    native private static float getMaxLinMotorForce(long jointId);

    native private static float getRestitutionDirAng(long jointId);

    native private static float getRestitutionDirLin(long jointId);

    native private static float getRestitutionLimAng(long jointId);

    native private static float getRestitutionLimLin(long jointId);

    native private static float getRestitutionOrthoAng(long jointId);

    native private static float getRestitutionOrthoLin(long jointId);

    native private static float getSoftnessDirAng(long jointId);

    native private static float getSoftnessDirLin(long jointId);

    native private static float getSoftnessLimAng(long jointId);

    native private static float getSoftnessLimLin(long jointId);

    native private static float getSoftnessOrthoAng(long jointId);

    native private static float getSoftnessOrthoLin(long jointId);

    native private static float getTargetAngMotorVelocity(long jointId);

    native private static float getTargetLinMotorVelocity(long jointId);

    native private static float getUpperAngLimit(long jointId);

    native private static float getUpperLinLimit(long jointId);

    native private static boolean isPoweredAngMotor(long jointId);

    native private static boolean isPoweredLinMotor(long jointId);

    native private static void setDampingDirAng(long jointId, float value);

    native private static void setDampingDirLin(long jointId, float value);

    native private static void setDampingLimAng(long jointId, float value);

    native private static void setDampingLimLin(long jointId, float value);

    native private static void setDampingOrthoAng(long jointId, float value);

    native private static void setDampingOrthoLin(long jointId, float value);

    native private static void setLowerAngLimit(long jointId, float value);

    native private static void setLowerLinLimit(long jointId, float value);

    native private static void setMaxAngMotorForce(long jointId, float value);

    native private static void setMaxLinMotorForce(long jointId, float value);

    native private static void setPoweredAngMotor(long jointId, boolean value);

    native private static void setPoweredLinMotor(long jointId, boolean value);

    native private static void setRestitutionDirAng(long jointId, float value);

    native private static void setRestitutionDirLin(long jointId, float value);

    native private static void setRestitutionLimAng(long jointId, float value);

    native private static void setRestitutionLimLin(long jointId, float value);

    native private static void
            setRestitutionOrthoAng(long jointId, float value);

    native private static void
            setRestitutionOrthoLin(long jointId, float value);

    native private static void setSoftnessDirAng(long jointId, float value);

    native private static void setSoftnessDirLin(long jointId, float value);

    native private static void setSoftnessLimAng(long jointId, float value);

    native private static void setSoftnessLimLin(long jointId, float value);

    native private static void setSoftnessOrthoAng(long jointId, float value);

    native private static void setSoftnessOrthoLin(long jointId, float value);

    native private static void
            setTargetAngMotorVelocity(long jointId, float value);

    native private static void
            setTargetLinMotorVelocity(long jointId, float value);

    native private static void setUpperAngLimit(long jointId, float value);

    native private static void setUpperLinLimit(long objectId, float value);
}
