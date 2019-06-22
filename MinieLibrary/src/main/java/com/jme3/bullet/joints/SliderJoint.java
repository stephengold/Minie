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
import java.util.logging.Level;
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
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public SliderJoint() {
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
        rotA = new Matrix3f();
        rotB = new Matrix3f();

        useLinearReferenceFrameA = (linearReferenceFrame == JointEnd.A);
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
        rotA = rotInA.clone();
        rotB = rotInB.clone();
        createJoint();
    }

    /**
     * Instantiate a double-ended SliderJoint.
     * <p>
     * To be effective, the joint must be added to the body's PhysicsSpace and
     * the body must be dynamic.
     *
     * @param rigidBodyA the 1st body to constrain (not null, alias created)
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
        rotA = new Matrix3f();
        rotB = new Matrix3f();
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
        return getDampingDirAng(objectId);
    }

    /**
     * Read the joint's damping for on-axis translation between the limits.
     *
     * @return the viscous damping ratio (0&rarr;no damping, 1&rarr;critically
     * damped)
     */
    public float getDampingDirLin() {
        return getDampingDirLin(objectId);
    }

    /**
     * Read the joint's damping for on-axis rotation hitting the limits.
     *
     * @return the viscous damping ratio (0&rarr;no damping, 1&rarr;critically
     * damped)
     */
    public float getDampingLimAng() {
        return getDampingLimAng(objectId);
    }

    /**
     * Read the joint's damping for on-axis translation hitting the limits.
     *
     * @return the viscous damping ratio (0&rarr;no damping, 1&rarr;critically
     * damped)
     */
    public float getDampingLimLin() {
        return getDampingLimLin(objectId);
    }

    /**
     * Read the joint's damping for off-axis rotation.
     *
     * @return the viscous damping ratio (0&rarr;no damping, 1&rarr;critically
     * damped)
     */
    public float getDampingOrthoAng() {
        return getDampingOrthoAng(objectId);
    }

    /**
     * Read the joint's damping for off-axis translation.
     *
     * @return the viscous damping ratio (0&rarr;no damping, 1&rarr;critically
     * damped)
     */
    public float getDampingOrthoLin() {
        return getDampingOrthoLin(objectId);
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

        switch (end) {
            case A:
                getFrameOffsetA(objectId, result);
                break;
            case B:
                getFrameOffsetB(objectId, result);
                break;
            default:
                String message = "end = " + end.toString();
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
        return getLowerAngLimit(objectId);
    }

    /**
     * Read the joint's lower limit for on-axis translation.
     *
     * @return the lower limit
     */
    public float getLowerLinLimit() {
        return getLowerLinLimit(objectId);
    }

    /**
     * Read the maximum force of the rotation motor.
     *
     * @return the maximum force
     */
    public float getMaxAngMotorForce() {
        return getMaxAngMotorForce(objectId);
    }

    /**
     * Read the maximum force of the translation motor.
     *
     * @return the maximum force
     */
    public float getMaxLinMotorForce() {
        return getMaxLinMotorForce(objectId);
    }

    /**
     * Read the joint's restitution for on-axis rotation between the limits.
     *
     * @return the restitution (bounce) factor
     */
    public float getRestitutionDirAng() {
        return getRestitutionDirAng(objectId);
    }

    /**
     * Read the joint's restitution for on-axis translation between the limits.
     *
     * @return the restitution (bounce) factor
     */
    public float getRestitutionDirLin() {
        return getRestitutionDirLin(objectId);
    }

    /**
     * Read the joint's restitution for on-axis rotation hitting the limits.
     *
     * @return the restitution (bounce) factor
     */
    public float getRestitutionLimAng() {
        return getRestitutionLimAng(objectId);
    }

    /**
     * Read the joint's restitution for on-axis translation hitting the limits.
     *
     * @return the restitution (bounce) factor
     */
    public float getRestitutionLimLin() {
        return getRestitutionLimLin(objectId);
    }

    /**
     * Read the joint's restitution for off-axis rotation.
     *
     * @return the restitution (bounce) factor
     */
    public float getRestitutionOrthoAng() {
        return getRestitutionOrthoAng(objectId);
    }

    /**
     * Read the joint's restitution for off-axis translation.
     *
     * @return the restitution (bounce) factor
     */
    public float getRestitutionOrthoLin() {
        return getRestitutionOrthoLin(objectId);
    }

    /**
     * Read the joint's softness for on-axis rotation between the limits.
     *
     * @return the softness
     */
    public float getSoftnessDirAng() {
        return getSoftnessDirAng(objectId);
    }

    /**
     * Read the joint's softness for on-axis translation between the limits.
     *
     * @return the softness
     */
    public float getSoftnessDirLin() {
        return getSoftnessDirLin(objectId);
    }

    /**
     * Read the joint's softness for on-axis rotation hitting the limits.
     *
     * @return the softness
     */
    public float getSoftnessLimAng() {
        return getSoftnessLimAng(objectId);
    }

    /**
     * Read the joint's softness for on-axis translation hitting the limits.
     *
     * @return the softness
     */
    public float getSoftnessLimLin() {
        return getSoftnessLimLin(objectId);
    }

    /**
     * Read the joint's softness for off-axis rotation.
     *
     * @return the softness
     */
    public float getSoftnessOrthoAng() {
        return getSoftnessOrthoAng(objectId);
    }

    /**
     * Read the joint's softness for off-axis translation.
     *
     * @return the softness
     */
    public float getSoftnessOrthoLin() {
        return getSoftnessOrthoLin(objectId);
    }

    /**
     * Read the velocity target of the rotation motor.
     *
     * @return the velocity target (in radians per second)
     */
    public float getTargetAngMotorVelocity() {
        return getTargetAngMotorVelocity(objectId);
    }

    /**
     * Read the velocity target of the translation motor.
     *
     * @return the velocity target
     */
    public float getTargetLinMotorVelocity() {
        return getTargetLinMotorVelocity(objectId);
    }

    /**
     * Read the joint's upper limit for on-axis rotation.
     *
     * @return the upper limit angle (in radians)
     */
    public float getUpperAngLimit() {
        return getUpperAngLimit(objectId);
    }

    /**
     * Read the joint's upper limit for on-axis translation.
     *
     * @return the upper limit
     */
    public float getUpperLinLimit() {
        return getUpperLinLimit(objectId);
    }

    /**
     * Test whether the rotation motor is powered.
     *
     * @return true if powered, otherwise false
     */
    public boolean isPoweredAngMotor() {
        return isPoweredAngMotor(objectId);
    }

    /**
     * Test whether the translation motor is powered.
     *
     * @return true if powered, otherwise false
     */
    public boolean isPoweredLinMotor() {
        return isPoweredLinMotor(objectId);
    }

    /**
     * Alter the joint's damping for on-axis rotation between the limits.
     *
     * @param dampingDirAng the desired viscous damping ratio (0&rarr;no
     * damping, 1&rarr;critically damped, default=0)
     */
    public void setDampingDirAng(float dampingDirAng) {
        setDampingDirAng(objectId, dampingDirAng);
    }

    /**
     * Alter the joint's damping for on-axis translation between the limits.
     *
     * @param dampingDirLin the desired viscous damping ratio (0&rarr;no
     * damping, 1&rarr;critically damped, default=0)
     */
    public void setDampingDirLin(float dampingDirLin) {
        setDampingDirLin(objectId, dampingDirLin);
    }

    /**
     * Alter the joint's damping for on-axis rotation hitting the limits.
     *
     * @param dampingLimAng the desired viscous damping ratio (0&rarr;no
     * damping, 1&rarr;critically damped, default=1)
     */
    public void setDampingLimAng(float dampingLimAng) {
        setDampingLimAng(objectId, dampingLimAng);
    }

    /**
     * Alter the joint's damping for on-axis translation hitting the limits.
     *
     * @param dampingLimLin the desired viscous damping ratio (0&rarr;no
     * damping, 1&rarr;critically damped, default=1)
     */
    public void setDampingLimLin(float dampingLimLin) {
        setDampingLimLin(objectId, dampingLimLin);
    }

    /**
     * Alter the joint's damping for off-axis rotation.
     *
     * @param dampingOrthoAng the desired viscous damping ratio (0&rarr;no
     * damping, 1&rarr;critically damped, default=1)
     */
    public void setDampingOrthoAng(float dampingOrthoAng) {
        setDampingOrthoAng(objectId, dampingOrthoAng);
    }

    /**
     * Alter the joint's damping for off-axis translation.
     *
     * @param dampingOrthoLin the desired viscous damping ratio (0&rarr;no
     * damping, 1&rarr;critically damped, default=1)
     */
    public void setDampingOrthoLin(float dampingOrthoLin) {
        setDampingOrthoLin(objectId, dampingOrthoLin);
    }

    /**
     * Alter the joint's lower limit for on-axis rotation.
     *
     * @param lowerAngLimit the desired lower limit angle (in radians,
     * default=0)
     */
    public void setLowerAngLimit(float lowerAngLimit) {
        setLowerAngLimit(objectId, lowerAngLimit);
    }

    /**
     * Alter the joint's lower limit for on-axis translation.
     *
     * @param lowerLinLimit the desired lower limit (default=-1)
     */
    public void setLowerLinLimit(float lowerLinLimit) {
        setLowerLinLimit(objectId, lowerLinLimit);
    }

    /**
     * Alter the maximum force of the rotation motor.
     *
     * @param maxAngMotorForce the desired maximum force (default=0)
     */
    public void setMaxAngMotorForce(float maxAngMotorForce) {
        setMaxAngMotorForce(objectId, maxAngMotorForce);
    }

    /**
     * Alter the maximum force of the translation motor.
     *
     * @param maxLinMotorForce the desired maximum force (default=0)
     */
    public void setMaxLinMotorForce(float maxLinMotorForce) {
        setMaxLinMotorForce(objectId, maxLinMotorForce);
    }

    /**
     * Alter whether the rotation motor is powered.
     *
     * @param poweredAngMotor true to power the motor, false to de-power it
     * (default=false)
     */
    public void setPoweredAngMotor(boolean poweredAngMotor) {
        setPoweredAngMotor(objectId, poweredAngMotor);
    }

    /**
     * Alter whether the translation motor is powered.
     *
     * @param poweredLinMotor true to power the motor, false to de-power it
     * (default=false)
     */
    public void setPoweredLinMotor(boolean poweredLinMotor) {
        setPoweredLinMotor(objectId, poweredLinMotor);
    }

    /**
     * Alter the joint's restitution for on-axis rotation between the limits.
     *
     * @param restitutionDirAng the desired restitution (bounce) factor
     * (default=0.7)
     */
    public void setRestitutionDirAng(float restitutionDirAng) {
        setRestitutionDirAng(objectId, restitutionDirAng);
    }

    /**
     * Alter the joint's restitution for on-axis translation between the limits.
     *
     * @param restitutionDirLin the desired restitution (bounce) factor
     * (default=0.7)
     */
    public void setRestitutionDirLin(float restitutionDirLin) {
        setRestitutionDirLin(objectId, restitutionDirLin);
    }

    /**
     * Alter the joint's restitution for on-axis rotation hitting the limits.
     *
     * @param restitutionLimAng the desired restitution (bounce) factor
     * (default=0.7)
     */
    public void setRestitutionLimAng(float restitutionLimAng) {
        setRestitutionLimAng(objectId, restitutionLimAng);
    }

    /**
     * Alter the joint's restitution for on-axis translation hitting the limits.
     *
     * @param restitutionLimLin the desired restitution (bounce) factor
     * (default=0.7)
     */
    public void setRestitutionLimLin(float restitutionLimLin) {
        setRestitutionLimLin(objectId, restitutionLimLin);
    }

    /**
     * Alter the joint's restitution for off-axis rotation.
     *
     * @param restitutionOrthoAng the desired restitution (bounce) factor
     * (default=0.7)
     */
    public void setRestitutionOrthoAng(float restitutionOrthoAng) {
        setRestitutionOrthoAng(objectId, restitutionOrthoAng);
    }

    /**
     * Alter the joint's restitution for off-axis translation.
     *
     * @param restitutionOrthoLin the desired restitution (bounce) factor
     * (default=0.7)
     */
    public void setRestitutionOrthoLin(float restitutionOrthoLin) {
        setRestitutionOrthoLin(objectId, restitutionOrthoLin);
    }

    /**
     * Alter the joint's softness for on-axis rotation between the limits.
     *
     * @param softnessDirAng the desired softness (default=1)
     */
    public void setSoftnessDirAng(float softnessDirAng) {
        setSoftnessDirAng(objectId, softnessDirAng);
    }

    /**
     * Alter the joint's softness for on-axis translation between the limits.
     *
     * @param softnessDirLin the desired softness (default=1)
     */
    public void setSoftnessDirLin(float softnessDirLin) {
        setSoftnessDirLin(objectId, softnessDirLin);
    }

    /**
     * Alter the joint's softness for on-axis rotation hitting the limits.
     *
     * @param softnessLimAng the desired softness (default=1)
     */
    public void setSoftnessLimAng(float softnessLimAng) {
        setSoftnessLimAng(objectId, softnessLimAng);
    }

    /**
     * Alter the joint's softness for on-axis translation hitting the limits.
     *
     * @param softnessLimLin the desired softness (default=1)
     */
    public void setSoftnessLimLin(float softnessLimLin) {
        setSoftnessLimLin(objectId, softnessLimLin);
    }

    /**
     * Alter the joint's softness for off-axis rotation.
     *
     * @param softnessOrthoAng the desired softness (default=1)
     */
    public void setSoftnessOrthoAng(float softnessOrthoAng) {
        setSoftnessOrthoAng(objectId, softnessOrthoAng);
    }

    /**
     * Alter the joint's softness for off-axis translation.
     *
     * @param softnessOrthoLin the desired softness (default=1)
     */
    public void setSoftnessOrthoLin(float softnessOrthoLin) {
        setSoftnessOrthoLin(objectId, softnessOrthoLin);
    }

    /**
     * Alter the velocity target of the rotation motor.
     *
     * @param targetAngMotorVelocity the desired velocity target (in radians per
     * second, default=0)
     */
    public void setTargetAngMotorVelocity(float targetAngMotorVelocity) {
        setTargetAngMotorVelocity(objectId, targetAngMotorVelocity);
    }

    /**
     * Alter the velocity target of the translation motor.
     *
     * @param targetLinMotorVelocity the desired velocity target (default=0)
     */
    public void setTargetLinMotorVelocity(float targetLinMotorVelocity) {
        setTargetLinMotorVelocity(objectId, targetLinMotorVelocity);
    }

    /**
     * Alter the joint's upper limit for on-axis rotation.
     *
     * @param upperAngLimit the desired upper limit angle (in radians,
     * default=0)
     */
    public void setUpperAngLimit(float upperAngLimit) {
        setUpperAngLimit(objectId, upperAngLimit);
    }

    /**
     * Alter the joint's upper limit for on-axis translation.
     *
     * @param upperLinLimit the desired upper limit (default=1)
     */
    public void setUpperLinLimit(float upperLinLimit) {
        setUpperLinLimit(objectId, upperLinLimit);
    }
    // *************************************************************************
    // PhysicsJoint methods

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
        super.cloneFields(cloner, original);

        rotA = cloner.clone(rotA);
        rotB = cloner.clone(rotB);
        createJoint();

        SliderJoint old = (SliderJoint) original;

        float bit = old.getBreakingImpulseThreshold();
        setBreakingImpulseThreshold(bit);

        boolean enableJoint = old.isEnabled();
        setEnabled(enableJoint);

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
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public SliderJoint jmeClone() {
        try {
            SliderJoint clone = (SliderJoint) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
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

        float dampingDirAng = capsule.readFloat("dampingDirAng", 0f);
        float dampingDirLin = capsule.readFloat("dampingDirLin", 0f);
        float dampingLimAng = capsule.readFloat("dampingLimAng", 0f);
        float dampingLimLin = capsule.readFloat("dampingLimLin", 0f);
        float dampingOrthoAng = capsule.readFloat("dampingOrthoAng", 0f);
        float dampingOrthoLin = capsule.readFloat("dampingOrthoLin", 0f);
        float lowerAngLimit = capsule.readFloat("lowerAngLimit", 0f);
        float lowerLinLimit = capsule.readFloat("lowerLinLimit", 0f);
        float maxAngMotorForce = capsule.readFloat("maxAngMotorForce", 0f);
        float maxLinMotorForce = capsule.readFloat("maxLinMotorForce", 0f);
        boolean poweredAngMotor = capsule.readBoolean("poweredAngMotor", false);
        boolean poweredLinMotor = capsule.readBoolean("poweredLinMotor", false);
        float restitutionDirAng = capsule.readFloat("restitutionDirAng", 0f);
        float restitutionDirLin = capsule.readFloat("restitutionDirLin", 0f);
        float restitutionLimAng = capsule.readFloat("restitutionLimAng", 0f);
        float restitutionLimLin = capsule.readFloat("restitutionLimLin", 0f);
        float restitutionOrthoAng
                = capsule.readFloat("restitutionOrthoAng", 0f);
        float restitutionOrthoLin
                = capsule.readFloat("restitutionOrthoLin", 0f);

        float softnessDirAng = capsule.readFloat("softnessDirAng", 0f);
        float softnessDirLin = capsule.readFloat("softnessDirLin", 0f);
        float softnessLimAng = capsule.readFloat("softnessLimAng", 0f);
        float softnessLimLin = capsule.readFloat("softnessLimLin", 0f);
        float softnessOrthoAng = capsule.readFloat("softnessOrthoAng", 0f);
        float softnessOrthoLin = capsule.readFloat("softnessOrthoLin", 0f);

        float targetAngMotorVelocity
                = capsule.readFloat("targetAngMotorVelicoty", 0f); //oops
        float targetLinMotorVelocity
                = capsule.readFloat("targetLinMotorVelicoty", 0f); //oops

        float upperAngLimit = capsule.readFloat("upperAngLimit", 0f);
        float upperLinLimit = capsule.readFloat("upperLinLimit", 0f);

        rotA = (Matrix3f) capsule.readSavable("rotA", new Matrix3f());
        rotB = (Matrix3f) capsule.readSavable("rotB", new Matrix3f());
        useLinearReferenceFrameA = capsule.readBoolean(
                "useLinearReferenceFrameA", false);

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

        //TODO: standard values..
        capsule.write(getDampingDirAng(), "dampingDirAng", 0f);
        capsule.write(getDampingDirLin(), "dampingDirLin", 0f);
        capsule.write(getDampingLimAng(), "dampingLimAng", 0f);
        capsule.write(getDampingLimLin(), "dampingLimLin", 0f);
        capsule.write(getDampingOrthoAng(), "dampingOrthoAng", 0f);
        capsule.write(getDampingOrthoLin(), "dampingOrthoLin", 0f);
        capsule.write(getLowerAngLimit(), "lowerAngLimit", 0f);
        capsule.write(getLowerLinLimit(), "lowerLinLimit", 0f);
        capsule.write(getMaxAngMotorForce(), "maxAngMotorForce", 0f);
        capsule.write(getMaxLinMotorForce(), "maxLinMotorForce", 0f);
        capsule.write(isPoweredAngMotor(), "poweredAngMotor", false);
        capsule.write(isPoweredLinMotor(), "poweredLinMotor", false);
        capsule.write(getRestitutionDirAng(), "restitutionDirAng", 0f);
        capsule.write(getRestitutionDirLin(), "restitutionDirLin", 0f);
        capsule.write(getRestitutionLimAng(), "restitutionLimAng", 0f);
        capsule.write(getRestitutionLimLin(), "restitutionLimLin", 0f);
        capsule.write(getRestitutionOrthoAng(), "restitutionOrthoAng", 0f);
        capsule.write(getRestitutionOrthoLin(), "restitutionOrthoLin", 0f);

        capsule.write(getSoftnessDirAng(), "softnessDirAng", 0f);
        capsule.write(getSoftnessDirLin(), "softnessDirLin", 0f);
        capsule.write(getSoftnessLimAng(), "softnessLimAng", 0f);
        capsule.write(getSoftnessLimLin(), "softnessLimLin", 0f);
        capsule.write(getSoftnessOrthoAng(), "softnessOrthoAng", 0f);
        capsule.write(getSoftnessOrthoLin(), "softnessOrthoLin", 0f);

        capsule.write(getTargetAngMotorVelocity(),
                "targetAngMotorVelicoty", 0f); //oops
        capsule.write(getTargetLinMotorVelocity(),
                "targetLinMotorVelicoty", 0f); //oops

        capsule.write(getUpperAngLimit(), "upperAngLimit", 0f);
        capsule.write(getUpperLinLimit(), "upperLinLimit", 0f);

        capsule.write(useLinearReferenceFrameA, "useLinearReferenceFrameA",
                false);
    }
    // *************************************************************************
    // private methods

    /**
     * Create the configured joint in Bullet.
     */
    private void createJoint() {
        assert objectId == 0L;
        assert pivotA != null;
        assert rotA != null;
        assert bodyB != null;
        assert pivotB != null;
        assert rotB != null;

        if (bodyA == null) {
            /*
             * Create a single-ended joint.  Bullet assumes single-ended
             * btSliderConstraints are satisfied at creation, so we
             * temporarily re-position the body to satisfy the constraint.
             */
            Vector3f saveLocation = bodyB.getPhysicsLocation(null);

            Vector3f offset = pivotA.subtract(pivotB);
            bodyB.setPhysicsLocation(offset);
            objectId = createJoint1(bodyB.getObjectId(), pivotB, rotB,
                    useLinearReferenceFrameA);

            bodyB.setPhysicsLocation(saveLocation);

        } else {
            /*
             * Create a double-ended joint.
             */
            objectId = createJoint(bodyA.getObjectId(), bodyB.getObjectId(),
                    pivotA, rotA, pivotB, rotB, useLinearReferenceFrameA);
        }
        assert objectId != 0L;
        logger2.log(Level.FINE, "Created Joint {0}",
                Long.toHexString(objectId));
    }

    native private long createJoint(long bodyIdA, long bodyIdB,
            Vector3f pivotInA, Matrix3f rotInA, Vector3f pivotInB,
            Matrix3f rotInB, boolean useLinearReferenceFrameA);

    native private long createJoint1(long bodyIdB, Vector3f pivotInB,
            Matrix3f rotInB, boolean useLinearReferenceFrameA);

    native private float getDampingDirAng(long jointId);

    native private float getDampingDirLin(long jointId);

    native private float getDampingLimAng(long jointId);

    native private float getDampingLimLin(long jointId);

    native private float getDampingOrthoAng(long jointId);

    native private float getDampingOrthoLin(long jointId);

    native private void getFrameOffsetA(long jointId, Transform frameInA);

    native private void getFrameOffsetB(long jointId, Transform frameInB);

    native private float getLowerAngLimit(long jointId);

    native private float getLowerLinLimit(long jointId);

    native private float getMaxAngMotorForce(long jointId);

    native private float getMaxLinMotorForce(long jointId);

    native private float getRestitutionDirAng(long jointId);

    native private float getRestitutionDirLin(long jointId);

    native private float getRestitutionLimAng(long jointId);

    native private float getRestitutionLimLin(long jointId);

    native private float getRestitutionOrthoAng(long jointId);

    native private float getRestitutionOrthoLin(long jointId);

    native private float getSoftnessDirAng(long jointId);

    native private float getSoftnessDirLin(long jointId);

    native private float getSoftnessLimAng(long jointId);

    native private float getSoftnessLimLin(long jointId);

    native private float getSoftnessOrthoAng(long jointId);

    native private float getSoftnessOrthoLin(long jointId);

    native private float getTargetAngMotorVelocity(long jointId);

    native private float getTargetLinMotorVelocity(long jointId);

    native private float getUpperAngLimit(long jointId);

    native private float getUpperLinLimit(long jointId);

    native private boolean isPoweredAngMotor(long jointId);

    native private boolean isPoweredLinMotor(long jointId);

    native private void setDampingDirAng(long jointId, float value);

    native private void setDampingDirLin(long jointId, float value);

    native private void setDampingLimAng(long jointId, float value);

    native private void setDampingLimLin(long jointId, float value);

    native private void setDampingOrthoAng(long jointId, float value);

    native private void setDampingOrthoLin(long jointId, float value);

    native private void setLowerAngLimit(long jointId, float value);

    native private void setLowerLinLimit(long jointId, float value);

    native private void setMaxAngMotorForce(long jointId, float value);

    native private void setMaxLinMotorForce(long jointId, float value);

    native private void setPoweredAngMotor(long jointId, boolean value);

    native private void setPoweredLinMotor(long jointId, boolean value);

    native private void setRestitutionDirAng(long jointId, float value);

    native private void setRestitutionDirLin(long jointId, float value);

    native private void setRestitutionLimAng(long jointId, float value);

    native private void setRestitutionLimLin(long jointId, float value);

    native private void setRestitutionOrthoAng(long jointId, float value);

    native private void setRestitutionOrthoLin(long jointId, float value);

    native private void setSoftnessDirAng(long jointId, float value);

    native private void setSoftnessDirLin(long jointId, float value);

    native private void setSoftnessLimAng(long jointId, float value);

    native private void setSoftnessLimLin(long jointId, float value);

    native private void setSoftnessOrthoAng(long jointId, float value);

    native private void setSoftnessOrthoLin(long jointId, float value);

    native private void setTargetAngMotorVelocity(long jointId, float value);

    native private void setTargetLinMotorVelocity(long jointId, float value);

    native private void setUpperAngLimit(long jointId, float value);

    native private void setUpperLinLimit(long objectId, float value);
}
