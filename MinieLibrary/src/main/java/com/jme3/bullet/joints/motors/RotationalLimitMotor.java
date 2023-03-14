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
package com.jme3.bullet.joints.motors;

import com.jme3.bullet.NativePhysicsObject;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A motor based on Bullet's btRotationalLimitMotor, used to control the
 * rotation of a SixDofJoint.
 *
 * @author normenhansen
 */
public class RotationalLimitMotor extends NativePhysicsObject {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(RotationalLimitMotor.class.getName());
    // *************************************************************************
    // constructors

    /**
     * Instantiate a motor. Used internally.
     *
     * @param nativeId the ID of the pre-existing btRotationalLimitMotor (not
     * zero)
     */
    public RotationalLimitMotor(long nativeId) {
        Validate.nonZero(nativeId, "native ID");
        super.setNativeIdNotTracked(nativeId);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Return the accumulated impulse (native field: m_accumulatedImpulse).
     *
     * @return the total impulse
     */
    public float getAccumulatedImpulse() {
        long motorId = nativeId();
        float result = getAccumulatedImpulse(motorId);

        return result;
    }

    /**
     * Return this motor's current rotation angle (native field:
     * m_currentPosition).
     *
     * @return the angle (in radians)
     */
    public float getAngle() {
        long motorId = nativeId();
        float result = getCurrentPosition(motorId);

        return result;
    }

    /**
     * Return this motor's damping (native field: m_damping).
     *
     * @return the viscous damping ratio (0&rarr;no damping, 1&rarr;critically
     * damped)
     */
    public float getDamping() {
        long motorId = nativeId();
        float result = getDamping(motorId);

        return result;
    }

    /**
     * Return this motor's error-reduction parameter at the limits (native
     * field: m_stopERP).
     *
     * @return the error-reduction parameter (&ge;0)
     */
    public float getERP() {
        long motorId = nativeId();
        float result = getERP(motorId);

        return result;
    }

    /**
     * Return this motor's limit softness (native field: m_limitSoftness).
     *
     * @return the limit softness (or relaxation factor)
     */
    public float getLimitSoftness() {
        long motorId = nativeId();
        float result = getLimitSoftness(motorId);

        return result;
    }

    /**
     * Return this motor's constraint lower limit (native field: m_loLimit).
     *
     * @return the limit value
     */
    public float getLowerLimit() {
        long motorId = nativeId();
        float result = getLoLimit(motorId);

        return result;
    }

    /**
     * Return the limit maximum force (native field: m_maxLimitForce).
     *
     * @return the maximum force on the limit (default=300)
     */
    public float getMaxLimitForce() {
        long motorId = nativeId();
        float result = getMaxLimitForce(motorId);

        return result;
    }

    /**
     * Return this motor's maximum force for normal conditions (native field:
     * m_maxMotorForce).
     *
     * @return the maximum force
     */
    public float getMaxMotorForce() {
        long motorId = nativeId();
        float result = getMaxMotorForce(motorId);

        return result;
    }

    /**
     * Return the ID of the native object ({@code btRotationalLimitMotor}).
     *
     * @return the native identifier (not zero)
     * @deprecated use {@link NativePhysicsObject#nativeId()}
     */
    @Deprecated
    public long getMotor() {
        long motorId = nativeId();
        return motorId;
    }

    /**
     * Return this motor's constraint-force mixing parameter for normal
     * conditions (native field: m_normalCFM).
     *
     * @return the mixing parameter (&ge;0)
     */
    public float getNormalCFM() {
        long motorId = nativeId();
        float result = getNormalCFM(motorId);

        return result;
    }

    /**
     * Return this motor's restitution at the limits (native field: m_bounce).
     *
     * @return the restitution (bounce) factor
     */
    public float getRestitution() {
        long motorId = nativeId();
        float result = getBounce(motorId);

        return result;
    }

    /**
     * Return this motor's constraint-force mixing parameter at the limits
     * (native field: m_stopCFM).
     *
     * @return the mixing parameter (&ge;0)
     */
    public float getStopCFM() {
        long motorId = nativeId();
        float result = getStopCFM(motorId);

        return result;
    }

    /**
     * Return this motor's target velocity (native field: m_targetVelocity).
     *
     * @return the target velocity (in radians per second)
     */
    public float getTargetVelocity() {
        long motorId = nativeId();
        float result = getTargetVelocity(motorId);

        return result;
    }

    /**
     * Return this motor's constraint upper limit (native field: m_hiLimit).
     *
     * @return the limit value
     */
    public float getUpperLimit() {
        long motorId = nativeId();
        float result = getHiLimit(motorId);

        return result;
    }

    /**
     * Test whether this motor is enabled (native field: m_enableMotor).
     *
     * @return true if enabled, otherwise false
     */
    public boolean isEnableMotor() {
        long motorId = nativeId();
        boolean result = isEnableMotor(motorId);

        return result;
    }

    /**
     * Alter the accumulated impulse (native field: m_accumulatedImpulse).
     *
     * @param accumulatedImpulse the desired total (default=0)
     */
    public void setAccumulatedImpulse(float accumulatedImpulse) {
        long motorId = nativeId();
        setAccumulatedImpulse(motorId, accumulatedImpulse);
    }

    /**
     * Alter this motor's damping (native field: m_damping).
     *
     * @param damping the desired viscous damping ratio (0&rarr;no damping,
     * 1&rarr;critically damped, default=1)
     */
    public void setDamping(float damping) {
        long motorId = nativeId();
        setDamping(motorId, damping);
    }

    /**
     * Enable or disable this motor (native field: m_enableMotor).
     *
     * @param enableMotor true&rarr;enable, false&rarr;disable (default=false)
     */
    public void setEnableMotor(boolean enableMotor) {
        long motorId = nativeId();
        setEnableMotor(motorId, enableMotor);
    }

    /**
     * Alter this motor's error-reduction parameter at the limits (native field:
     * m_stopERP).
     *
     * @param erp the desired error tolerance at limits (&ge;0, default=0.2)
     */
    public void setERP(float erp) {
        long motorId = nativeId();
        setERP(motorId, erp);
    }

    /**
     * Alter this motor's limit softness (native field: m_limitSoftness).
     *
     * @param limitSoftness the desired limit softness (or relaxation factor)
     * (default=0.5)
     */
    public void setLimitSoftness(float limitSoftness) {
        long motorId = nativeId();
        setLimitSoftness(motorId, limitSoftness);
    }

    /**
     * Alter this motor's constraint lower limit (native field: m_loLimit).
     *
     * @param lowerLimit the desired limit value (default=1)
     */
    public void setLowerLimit(float lowerLimit) {
        long motorId = nativeId();
        setLoLimit(motorId, lowerLimit);
    }

    /**
     * Alter the limit maximum force (native field: m_maxLimitForce).
     *
     * @param maxLimitForce the desired maximum force on the limit (default=300)
     */
    public void setMaxLimitForce(float maxLimitForce) {
        long motorId = nativeId();
        setMaxLimitForce(motorId, maxLimitForce);
    }

    /**
     * Alter this motor's maximum force (native field: m_maxMotorForce).
     *
     * @param maxMotorForce the desired maximum force (default=6)
     */
    public void setMaxMotorForce(float maxMotorForce) {
        long motorId = nativeId();
        setMaxMotorForce(motorId, maxMotorForce);
    }

    /**
     * Alter this motor's constraint-force mixing parameter for normal
     * conditions (native field: m_normalCFM).
     *
     * @param cfm the desired mixing parameter (&ge;0, default=0)
     */
    public void setNormalCFM(float cfm) {
        long motorId = nativeId();
        setNormalCFM(motorId, cfm);
    }

    /**
     * Alter this motor's restitution at the limits (native field: m_bounce).
     *
     * @param restitution the desired restitution (bounce) factor (default=0)
     */
    public void setRestitution(float restitution) {
        long motorId = nativeId();
        setBounce(motorId, restitution);
    }

    /**
     * Alter this motor's constraint-force mixing parameter at the limits
     * (native field: m_stopCFM).
     *
     * @param cfm the desired mixing parameter (&ge;0, default=0)
     */
    public void setStopCFM(float cfm) {
        long motorId = nativeId();
        setStopCFM(motorId, cfm);
    }

    /**
     * Alter this motor's target velocity (native field: m_targetVelocity).
     *
     * @param targetVelocity the desired target velocity (in radians per second,
     * default=0)
     */
    public void setTargetVelocity(float targetVelocity) {
        long motorId = nativeId();
        setTargetVelocity(motorId, targetVelocity);
    }

    /**
     * Alter this motor's constraint upper limit (native field: m_hiLimit).
     *
     * @param upperLimit the desired limit value (default=-1)
     */
    public void setUpperLimit(float upperLimit) {
        long motorId = nativeId();
        setHiLimit(motorId, upperLimit);
    }
    // *************************************************************************
    // native private methods

    native private static float getAccumulatedImpulse(long motorId);

    native private static float getBounce(long motorId);

    native private static float getCurrentPosition(long motorId);

    native private static float getDamping(long motorId);

    native private static float getERP(long motorId);

    native private static float getHiLimit(long motorId);

    native private static float getLimitSoftness(long motorId);

    native private static float getLoLimit(long motorId);

    native private static float getMaxLimitForce(long motorId);

    native private static float getMaxMotorForce(long motorId);

    native private static float getNormalCFM(long motorId);

    native private static float getStopCFM(long motorId);

    native private static float getTargetVelocity(long motorId);

    native private static boolean isEnableMotor(long motorId);

    native private static void
            setAccumulatedImpulse(long motorId, float vector);

    native private static void setBounce(long motorId, float bounce);

    native private static void setDamping(long motorId, float damping);

    native private static void
            setEnableMotor(long motorId, boolean enableMotor);

    native private static void setERP(long motorId, float erp);

    native private static void setHiLimit(long motorId, float hiLimit);

    native private static void
            setLimitSoftness(long motorId, float limitSoftness);

    native private static void setLoLimit(long motorId, float loLimit);

    native private static void
            setMaxLimitForce(long motorId, float maxLimitForce);

    native private static void
            setMaxMotorForce(long motorId, float maxMotorForce);

    native private static void setNormalCFM(long motorId, float normalCFM);

    native private static void setStopCFM(long motorId, float stopCFM);

    native private static void
            setTargetVelocity(long motorId, float targetVelocity);
}
