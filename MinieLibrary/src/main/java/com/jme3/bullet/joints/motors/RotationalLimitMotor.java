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

import java.util.logging.Logger;

/**
 * A motor based on Bullet's btRotationalLimitMotor. Motors are used to drive
 * joints.
 *
 * @author normenhansen
 */
public class RotationalLimitMotor {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(RotationalLimitMotor.class.getName());
    // *************************************************************************
    // fields

    /**
     * Unique identifier of the btRotationalLimitMotor. The constructor sets
     * this to a non-zero value. After that, the id never changes.
     */
    private long motorId = 0L;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a motor for the identified btRotationalLimitMotor.
     *
     * @param motor the unique identifier (not zero)
     */
    public RotationalLimitMotor(long motor) {
        assert motor != 0L;
        motorId = motor;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Read the accumulated impulse (m_accumulatedImpulse).
     *
     * @return the impulse amount
     */
    public float getAccumulatedImpulse() {
        return getAccumulatedImpulse(motorId);
    }

    /**
     * Read this motor's current rotation angle (m_currentPosition).
     *
     * @return the angle (in radians)
     */
    public float getAngle() {
        return getCurrentPosition(motorId);
    }

    /**
     * Read this motor's damping (m_damping).
     *
     * @return the viscous damping ratio (0&rarr;no damping, 1&rarr;critically
     * damped)
     */
    public float getDamping() {
        return getDamping(motorId);
    }

    /**
     * Read this motor's error-reduction parameter at the limits (m_stopERP).
     *
     * @return the the error-reduction parameter (&ge;0)
     */
    public float getERP() {
        return getERP(motorId);
    }

    /**
     * Read this motor's limit softness (m_limitSoftness).
     *
     * @return the limit softness (or relaxation factor)
     */
    public float getLimitSoftness() {
        return getLimitSoftness(motorId);
    }

    /**
     * Read this motor's constraint lower limit (m_loLimit).
     *
     * @return the limit value
     */
    public float getLowerLimit() {
        return getLoLimit(motorId);
    }

    /**
     * Read the limit maximum force (m_maxLimitForce).
     *
     * @return the maximum force on the limit (default=300)
     */
    public float getMaxLimitForce() {
        return getMaxLimitForce(motorId);
    }

    /**
     * Read this motor's maximum force for normal conditions (m_maxMotorForce).
     *
     * @return the maximum force
     */
    public float getMaxMotorForce() {
        return getMaxMotorForce(motorId);
    }

    /**
     * Read the id of the btRotationalLimitMotor.
     *
     * @return the unique identifier (not zero)
     */
    public long getMotor() {
        assert motorId != 0L;
        return motorId;
    }

    /**
     * Read this motor's constraint-force mixing parameter for normal conditions
     * (m_normalCFM).
     *
     * @return the mixing parameter (&ge;0)
     */
    public float getNormalCFM() {
        return getNormalCFM(motorId);
    }

    /**
     * Read this motor's restitution at the limits (m_bounce).
     *
     * @return the restitution (bounce) factor
     */
    public float getRestitution() {
        return getBounce(motorId);
    }

    /**
     * Read this motor's constraint-force mixing parameter at the limits
     * (m_stopCFM).
     *
     * @return the mixing parameter (&ge;0)
     */
    public float getStopCFM() {
        return getStopCFM(motorId);
    }

    /**
     * Read this motor's target velocity (m_targetVelocity).
     *
     * @return the target velocity (in radians per second)
     */
    public float getTargetVelocity() {
        return getTargetVelocity(motorId);
    }

    /**
     * Read this motor's constraint upper limit (m_hiLimit).
     *
     * @return the limit value
     */
    public float getUpperLimit() {
        return getHiLimit(motorId);
    }

    /**
     * Test whether this motor is enabled (m_enableMotor).
     *
     * @return true if enabled, otherwise false
     */
    public boolean isEnableMotor() {
        return isEnableMotor(motorId);
    }

    /**
     * Alter this motor's damping (m_damping).
     *
     * @param damping the desired viscous damping ratio (0&rarr;no damping,
     * 1&rarr;critically damped, default=1)
     */
    public void setDamping(float damping) {
        setDamping(motorId, damping);
    }

    /**
     * Enable or disable this motor (m_enableMotor).
     *
     * @param enableMotor true&rarr;enable, false&rarr;disable
     */
    public void setEnableMotor(boolean enableMotor) {
        setEnableMotor(motorId, enableMotor);
    }

    /**
     * Alter this motor's error-reduction parameter at the limits (m_stopERP).
     *
     * @param erp the desired error tolerance at limits (&ge;0, default=0.2)
     */
    public void setERP(float erp) {
        setERP(motorId, erp);
    }

    /**
     * Alter this motor's limit softness (m_limitSoftness).
     *
     * @param limitSoftness the desired limit softness (or relaxation factor)
     * (default=0.5)
     */
    public void setLimitSoftness(float limitSoftness) {
        setLimitSoftness(motorId, limitSoftness);
    }

    /**
     * Alter this motor's constraint lower limit (m_loLimit).
     *
     * @param lowerLimit the desired limit value
     */
    public void setLowerLimit(float lowerLimit) {
        setLoLimit(motorId, lowerLimit);
    }

    /**
     * Alter the limit maximum force (m_maxLimitForce).
     *
     * @param maxLimitForce the desired maximum force on the limit (default=300)
     */
    public void setMaxLimitForce(float maxLimitForce) {
        setMaxLimitForce(motorId, maxLimitForce);
    }

    /**
     * Alter this motor's maximum force (m_maxMotorForce).
     *
     * @param maxMotorForce the desired maximum force
     */
    public void setMaxMotorForce(float maxMotorForce) {
        setMaxMotorForce(motorId, maxMotorForce);
    }

    /**
     * Alter this motor's constraint-force mixing parameter for normal
     * conditions (m_normalCFM).
     *
     * @param cfm the desired mixing parameter (&ge;0)
     */
    public void setNormalCFM(float cfm) {
        setNormalCFM(motorId, cfm);
    }

    /**
     * Alter this motor's restitution at the limits (m_bounce).
     *
     * @param restitution the desired restitution (bounce) factor (default=0)
     */
    public void setRestitution(float restitution) {
        setBounce(motorId, restitution);
    }

    /**
     * Alter this motor's constraint-force mixing parameter at the limits
     * (m_stopCFM).
     *
     * @param cfm the desired mixing parameter (&ge;0)
     */
    public void setStopCFM(float cfm) {
        setStopCFM(motorId, cfm);
    }

    /**
     * Alter this motor's target velocity (m_targetVelocity).
     *
     * @param targetVelocity the desired target velocity (in radians per second)
     */
    public void setTargetVelocity(float targetVelocity) {
        setTargetVelocity(motorId, targetVelocity);
    }

    /**
     * Alter this motor's constraint upper limit (m_hiLimit).
     *
     * @param upperLimit the desired limit value
     */
    public void setUpperLimit(float upperLimit) {
        setHiLimit(motorId, upperLimit);
    }
    // *************************************************************************
    // private methods

    native private float getAccumulatedImpulse(long motorId);

    native private float getBounce(long motorId);

    native private float getCurrentPosition(long motorId);

    native private float getDamping(long motorId);

    native private float getERP(long motorId);

    native private float getHiLimit(long motorId);

    native private float getLimitSoftness(long motorId);

    native private float getLoLimit(long motorId);

    native private float getMaxLimitForce(long motorId);

    native private float getMaxMotorForce(long motorId);

    native private float getNormalCFM(long motorId);

    native private float getStopCFM(long motorId);

    native private float getTargetVelocity(long motorId);

    native private boolean isEnableMotor(long motorId);

    native private void setBounce(long motorId, float limitSoftness);

    native private void setDamping(long motorId, float damping);

    native private void setEnableMotor(long motorId, boolean enableMotor);

    native private void setERP(long motorId, float ERP);

    native private void setHiLimit(long motorId, float hiLimit);

    native private void setLimitSoftness(long motorId, float limitSoftness);

    native private void setLoLimit(long motorId, float loLimit);

    native private void setMaxLimitForce(long motorId, float maxLimitForce);

    native private void setMaxMotorForce(long motorId, float maxMotorForce);

    native private void setNormalCFM(long motorId, float normalCFM);

    native private void setStopCFM(long motorId, float stopCFM);

    native private void setTargetVelocity(long motorId, float targetVelocity);
}
