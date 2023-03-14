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
import com.jme3.math.Vector3f;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A motor based on Bullet's btTranslationalLimitMotor, used to control the
 * translation of a SixDofJoint.
 *
 * @author normenhansen
 */
public class TranslationalLimitMotor extends NativePhysicsObject {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(TranslationalLimitMotor.class.getName());
    // *************************************************************************
    // constructors

    /**
     * Instantiate a motor. Used internally.
     *
     * @param nativeId the ID of the pre-existing btTranslationalLimitMotor (not
     * zero)
     */
    public TranslationalLimitMotor(long nativeId) {
        Validate.nonZero(nativeId, "native ID");
        super.setNativeIdNotTracked(nativeId);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the accumulated impulse (native field: m_accumulatedImpulse).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a new vector (not null)
     */
    public Vector3f getAccumulatedImpulse(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long motorId = nativeId();
        getAccumulatedImpulse(motorId, result);

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
     * Copy this motor's error-reduction parameters at the limits (native field:
     * m_stopERP).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the error-reduction parameter for each axis (either storeResult
     * or a new instance, not null)
     */
    public Vector3f getERP(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long motorId = nativeId();
        getERP(motorId, result);

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
     * Copy this motor's constraint lower limits (native field: m_lowerLimit).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the lower limit for each axis (either storeResult or a new
     * instance)
     */
    public Vector3f getLowerLimit(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long motorId = nativeId();
        getLowerLimit(motorId, result);

        return result;
    }

    /**
     * Copy this motor's maximum motor forces for normal conditions (native
     * field: m_maxMotorForce).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the maximum force for each axis (either storeResult or a new
     * instance)
     */
    public Vector3f getMaxMotorForce(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long motorId = nativeId();
        getMaxMotorForce(motorId, result);

        return result;
    }

    /**
     * Return the ID of the native object ({@code btTranslationalLimitMotor}).
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
     * Copy this motor's constraint-force mixing parameters for normal
     * conditions (native field: m_normalCFM).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the mixing parameter for each axis (either storeResult or a new
     * instance)
     */
    public Vector3f getNormalCFM(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long motorId = nativeId();
        getNormalCFM(motorId, result);

        return result;
    }

    /**
     * Return the offset of the constraint frames (native field:
     * m_currentLinearDiff).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the offset (either storeResult or a new instance)
     */
    public Vector3f getOffset(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long motorId = nativeId();
        getOffset(motorId, result);

        return result;
    }

    /**
     * Return this motor's restitution at the limits (native field:
     * m_restitution).
     *
     * @return the restitution (bounce) factor
     */
    public float getRestitution() {
        long motorId = nativeId();
        float result = getRestitution(motorId);

        return result;
    }

    /**
     * Copy this motor's constraint-force mixing parameters at the limits
     * (native field: m_stopCFM).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the mixing parameter for each axis (either storeResult or a new
     * instance)
     */
    public Vector3f getStopCFM(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long motorId = nativeId();
        getStopCFM(motorId, result);

        return result;
    }

    /**
     * Copy this motor's target velocity (native field: m_targetVelocity).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the target velocity vector (either storeResult or a new instance)
     */
    public Vector3f getTargetVelocity(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long motorId = nativeId();
        getTargetVelocity(motorId, result);

        return result;
    }

    /**
     * Copy this motor's constraint upper limits (native field: m_upperLimit).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the upper limit for each axis (either storeResult or a new
     * instance)
     */
    public Vector3f getUpperLimit(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long motorId = nativeId();
        getUpperLimit(motorId, result);

        return result;
    }

    /**
     * Test whether the indexed axis is enabled (native field: m_enableMotor).
     *
     * @param axisIndex which axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     * @return true if enabled, otherwise false
     */
    public boolean isEnabled(int axisIndex) {
        Validate.axisIndex(axisIndex, "axis index");

        long motorId = nativeId();
        boolean result = isEnabled(motorId, axisIndex);

        return result;
    }

    /**
     * Alter the accumulated impulse (native field: m_accumulatedImpulse).
     *
     * @param accumulatedImpulse the desired vector (not null, unaffected,
     * default=(0,0,0))
     */
    public void setAccumulatedImpulse(Vector3f accumulatedImpulse) {
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
     * Enable or disable the indexed axis (native field: m_enableMotor).
     *
     * @param axisIndex which axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     * @param enableMotor true&rarr;enable, false&rarr;disable (default=false)
     */
    public void setEnabled(int axisIndex, boolean enableMotor) {
        Validate.axisIndex(axisIndex, "axis index");

        long motorId = nativeId();
        setEnabled(motorId, axisIndex, enableMotor);
    }

    /**
     * Alter this motor's error-reduction parameters at the limits (native
     * field: m_stopERP).
     *
     * @param erp the desired error-reduction parameter parameter for each axis
     * (not null, unaffected, default=(0.2,0.2,0.2))
     */
    public void setERP(Vector3f erp) {
        long motorId = nativeId();
        setERP(motorId, erp);
    }

    /**
     * Alter the limit softness (native field: m_limitSoftness).
     *
     * @param limitSoftness the desired limit softness (default=0.7)
     */
    public void setLimitSoftness(float limitSoftness) {
        long motorId = nativeId();
        setLimitSoftness(motorId, limitSoftness);
    }

    /**
     * Alter this motor's constraint lower limits (native field: m_lowerLimit).
     *
     * @param lowerLimit the desired limit value for each axis (unaffected, not
     * null, default=(0,0,0))
     */
    public void setLowerLimit(Vector3f lowerLimit) {
        long motorId = nativeId();
        setLowerLimit(motorId, lowerLimit);
    }

    /**
     * Alter this motor's maximum forces (native field: m_maxMotorForce).
     *
     * @param maxForce the desired maximum force for each axis (not null,
     * unaffected, default=(0,0,0))
     */
    public void setMaxMotorForce(Vector3f maxForce) {
        long motorId = nativeId();
        setMaxMotorForce(motorId, maxForce);
    }

    /**
     * Alter this motor's constraint-force mixing parameters for normal
     * conditions (native field: m_normalCFM).
     *
     * @param cfm the desired mixing parameter for each axis (not null,
     * unaffected, default=(0,0,0))
     */
    public void setNormalCFM(Vector3f cfm) {
        long motorId = nativeId();
        setNormalCFM(motorId, cfm);
    }

    /**
     * Alter this motor's restitution at the limits (native field:
     * m_restitution).
     *
     * @param restitution the desired restitution (bounce) factor (default=0.5)
     */
    public void setRestitution(float restitution) {
        long motorId = nativeId();
        setRestitution(motorId, restitution);
    }

    /**
     * Alter this motor's constraint-force mixing parameters at the limits
     * (native field: m_stopCFM).
     *
     * @param cfm the desired mixing parameter for each axis (not null,
     * unaffected, default=(0,0,0))
     */
    public void setStopCFM(Vector3f cfm) {
        long motorId = nativeId();
        setStopCFM(motorId, cfm);
    }

    /**
     * Alter this motor's target velocity (native field: m_targetVelocity).
     *
     * @param velocity the desired velocity vector (not null, unaffected,
     * default=(0,0,0))
     */
    public void setTargetVelocity(Vector3f velocity) {
        long motorId = nativeId();
        setTargetVelocity(motorId, velocity);
    }

    /**
     * Alter this motor's constraint upper limits (native field: m_upperLimit).
     *
     * @param upperLimit the desired limit value for each axis (unaffected, not
     * null, default=(0,0,0))
     */
    public void setUpperLimit(Vector3f upperLimit) {
        long motorId = nativeId();
        setUpperLimit(motorId, upperLimit);
    }
    // *************************************************************************
    // native private methods

    native private static void
            getAccumulatedImpulse(long motorId, Vector3f vector);

    native private static float getDamping(long motorId);

    native private static void getERP(long motorId, Vector3f vector);

    native private static float getLimitSoftness(long motorId);

    native private static void getLowerLimit(long motorId, Vector3f vector);

    native private static void getMaxMotorForce(long motorId, Vector3f vector);

    native private static void getNormalCFM(long motorId, Vector3f vector);

    native private static void getOffset(long motorId, Vector3f vector);

    native private static float getRestitution(long motorId);

    native private static void getStopCFM(long motorId, Vector3f vector);

    native private static void
            getTargetVelocity(long motorId, Vector3f velocity);

    native private static void getUpperLimit(long motorId, Vector3f vector);

    native private static boolean isEnabled(long motorId, int axisIndex);

    native private static void
            setAccumulatedImpulse(long motorId, Vector3f vector);

    native private static void setDamping(long motorId, float damping);

    native private static void
            setEnabled(long motorId, int axisIndex, boolean enableMotor);

    native private static void setERP(long motorId, Vector3f vector);

    native private static void
            setLimitSoftness(long motorId, float limitSoftness);

    native private static void setLowerLimit(long motorId, Vector3f vector);

    native private static void setMaxMotorForce(long motorId, Vector3f force);

    native private static void setNormalCFM(long motorId, Vector3f vector);

    native private static void setRestitution(long motorId, float restitution);

    native private static void setStopCFM(long motorId, Vector3f stopCFM);

    native private static void
            setTargetVelocity(long motorId, Vector3f velocity);

    native private static void setUpperLimit(long motorId, Vector3f vector);
}
