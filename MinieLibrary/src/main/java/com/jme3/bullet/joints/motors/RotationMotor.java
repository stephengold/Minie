/*
 * Copyright (c) 2019-2022 jMonkeyEngine
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
 * A single-axis motor based on Bullet's btRotationalLimitMotor2, used to
 * control the rotation of a New6Dof constraint.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class RotationMotor extends NativePhysicsObject {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(RotationMotor.class.getName());
    // *************************************************************************
    // constructors

    /**
     * Instantiate a motor. Used internally.
     *
     * @param nativeId the ID of a pre-existing btRotationalLimitMotor2 (not
     * zero)
     */
    public RotationMotor(long nativeId) {
        Validate.nonZero(nativeId, "native ID");
        super.setNativeIdNotTracked(nativeId);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Return the value of the specified parameter.
     *
     * @param param which parameter (not null)
     * @return the parameter value
     */
    public float get(MotorParam param) {
        long motorId = nativeId();

        float result;
        switch (param) {
            case Bounce:
                result = getBounce(motorId);
                break;

            case Damping:
                result = getDamping(motorId);
                break;

            case Equilibrium:
                result = getEquilibrium(motorId);
                break;

            case LowerLimit:
                result = getLowerLimit(motorId);
                break;

            case MaxMotorForce:
                result = getMaxMotorForce(motorId);
                break;

            case ServoTarget:
                result = getServoTarget(motorId);
                break;

            case Stiffness:
                result = getStiffness(motorId);
                break;

            case TargetVelocity:
                result = getTargetVelocity(motorId);
                break;

            case UpperLimit:
                result = getUpperLimit(motorId);
                break;

            default:
                int paramIndex = param.nativeIndex();
                result = getParameter(motorId, paramIndex);
        }

        return result;
    }

    /**
     * Test whether the spring's damping is limited (native field:
     * m_springDampingLimited).
     *
     * @return true if limited, otherwise false
     */
    public boolean isDampingLimited() {
        long motorId = nativeId();
        boolean result = isDampingLimited(motorId);

        return result;
    }

    /**
     * Test whether the motor is enabled (native field: m_enableMotor).
     *
     * @return true if enabled, otherwise false
     */
    public boolean isMotorEnabled() {
        long motorId = nativeId();
        boolean result = isMotorEnabled(motorId);

        return result;
    }

    /**
     * Test whether the servo is enabled (native field: m_servoMotor).
     *
     * @return true if enabled, otherwise false
     */
    public boolean isServoEnabled() {
        long motorId = nativeId();
        boolean result = isServoEnabled(motorId);

        return result;
    }

    /**
     * Test whether the spring is enabled (native field: m_enableSpring).
     *
     * @return true if enabled, otherwise false
     */
    public boolean isSpringEnabled() {
        long motorId = nativeId();
        boolean result = isSpringEnabled(motorId);

        return result;
    }

    /**
     * Test whether the spring's stiffness is limited
     * (m_springStiffnessLimited).
     *
     * @return true if limited, otherwise false
     */
    public boolean isStiffnessLimited() {
        long motorId = nativeId();
        boolean result = isStiffnessLimited(motorId);

        return result;
    }

    /**
     * Alter the specified parameter.
     *
     * @param param which parameter (not null)
     * @param value the desired parameter value
     */
    public void set(MotorParam param, float value) {
        long motorId = nativeId();
        switch (param) {
            case Bounce:
                setBounce(motorId, value);
                break;

            case Damping:
                setDamping(motorId, value);
                break;

            case Equilibrium:
                setEquilibrium(motorId, value);
                break;

            case LowerLimit:
                setLowerLimit(motorId, value);
                break;

            case MaxMotorForce:
                setMaxMotorForce(motorId, value);
                break;

            case ServoTarget:
                setServoTarget(motorId, value);
                break;

            case Stiffness:
                setStiffness(motorId, value);
                break;

            case TargetVelocity:
                setTargetVelocity(motorId, value);
                break;

            case UpperLimit:
                setUpperLimit(motorId, value);
                break;

            default:
                int paramIndex = param.nativeIndex();
                setParameter(motorId, paramIndex, value);
        }
    }

    /**
     * Limit or unlimit the spring's damping (native field:
     * m_springDampingLimited).
     *
     * @param limitDamping true&rarr;limit, false&rarr;don't limit
     * (default=false)
     */
    public void setDampingLimited(boolean limitDamping) {
        long motorId = nativeId();
        setDampingLimited(motorId, limitDamping);
    }

    /**
     * Enable or disable this motor (native field: m_enableMotor).
     *
     * @param enableFlag true&rarr;enable, false&rarr;disable (default=false)
     */
    public void setMotorEnabled(boolean enableFlag) {
        long motorId = nativeId();
        setMotorEnabled(motorId, enableFlag);
    }

    /**
     * Enable or disable the servo (native field: m_servoMotor). This setting
     * has no effect if the motor is disabled.
     *
     * @param enableFlag true&rarr;enable, false&rarr;disable (default=false)
     */
    public void setServoEnabled(boolean enableFlag) {
        long motorId = nativeId();
        setServoEnabled(motorId, enableFlag);
    }

    /**
     * Enable or disable the spring (native field: m_enableSpring).
     *
     * @param enableFlag true&rarr;enable, false&rarr;disable (default=false)
     */
    public void setSpringEnabled(boolean enableFlag) {
        long motorId = nativeId();
        setSpringEnabled(motorId, enableFlag);
    }

    /**
     * Limit or unlimit the spring's stiffness (native field:
     * m_springStiffnessLimited).
     *
     * @param limitStiffness true&rarr;limit, false&rarr;don't limit
     * (default=false)
     */
    public void setStiffnessLimited(boolean limitStiffness) {
        long motorId = nativeId();
        setStiffnessLimited(motorId, limitStiffness);
    }
    // *************************************************************************
    // native private methods

    native private static float getBounce(long motorId);

    native private static float getDamping(long motorId);

    native private static float getEquilibrium(long motorId);

    native private static float getLowerLimit(long motorId);

    native private static float getMaxMotorForce(long motorId);

    native private static float getParameter(long motorId, int parameterIndex);

    native private static float getServoTarget(long motorId);

    native private static float getStiffness(long motorId);

    native private static float getTargetVelocity(long motorId);

    native private static float getUpperLimit(long motorId);

    native private static boolean isDampingLimited(long motorId);

    native private static boolean isMotorEnabled(long motorId);

    native private static boolean isServoEnabled(long motorId);

    native private static boolean isSpringEnabled(long motorId);

    native private static boolean isStiffnessLimited(long motorId);

    native private static void setBounce(long motorId, float bounce);

    native private static void setDamping(long motorId, float damping);

    native private static void
            setDampingLimited(long motorId, boolean limitFlag);

    native private static void setEquilibrium(long motorId, float angle);

    native private static void setLowerLimit(long motorId, float angle);

    native private static void setMaxMotorForce(long motorId, float force);

    native private static void setMotorEnabled(long motorId,
            boolean enableFlag);

    native private static void
            setParameter(long motorId, int parameterIndex, float value);

    native private static void
            setServoEnabled(long motorId, boolean enableFlag);

    native private static void setServoTarget(long motorId, float target);

    native private static void
            setSpringEnabled(long motorId, boolean enableFlag);

    native private static void setStiffness(long motorId, float stiffness);

    native private static void
            setStiffnessLimited(long motorId, boolean limitFlag);

    native private static void setTargetVelocity(long motorId, float velocity);

    native private static void setUpperLimit(long motorId, float angle);
}
