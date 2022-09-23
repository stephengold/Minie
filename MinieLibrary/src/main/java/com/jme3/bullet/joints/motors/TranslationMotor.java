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
import com.jme3.math.Vector3f;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A 3-axis motor based on Bullet's btTranslationalLimitMotor2, used to control
 * the translation of a New6Dof constraint.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TranslationMotor extends NativePhysicsObject {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(TranslationMotor.class.getName());
    // *************************************************************************
    // constructors

    /**
     * Instantiate a motor. Used internally.
     *
     * @param nativeId the ID of a pre-existing btTranslationalLimitMotor2 (not
     * zero)
     */
    public TranslationMotor(long nativeId) {
        Validate.nonZero(nativeId, "native ID");
        super.setNativeIdNotTracked(nativeId);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the specified parameter of this motor.
     *
     * @param param which parameter (not null)
     * @param storeResult storage for the result (modified if not null)
     * @return the parameter value for each axis (either storeResult or a new
     * instance)
     */
    public Vector3f get(MotorParam param, Vector3f storeResult) {
        long motorId = nativeId();

        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        switch (param) {
            case Bounce:
                getBounce(motorId, result);
                break;

            case Damping:
                getDamping(motorId, result);
                break;

            case Equilibrium:
                getEquilibrium(motorId, result);
                break;

            case LowerLimit:
                getLowerLimit(motorId, result);
                break;

            case MaxMotorForce:
                getMaxMotorForce(motorId, result);
                break;

            case ServoTarget:
                getServoTarget(motorId, result);
                break;

            case Stiffness:
                getStiffness(motorId, result);
                break;

            case TargetVelocity:
                getTargetVelocity(motorId, result);
                break;

            case UpperLimit:
                getUpperLimit(motorId, result);
                break;

            default:
                int paramIndex = param.nativeIndex();
                getParameter(motorId, paramIndex, result);
        }

        return result;
    }

    /**
     * Test whether the indexed spring's damping is limited
     * (m_springDampingLimited).
     *
     * @param axisIndex which axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     * @return true if limited, otherwise false
     */
    public boolean isDampingLimited(int axisIndex) {
        Validate.axisIndex(axisIndex, "axis index");

        long motorId = nativeId();
        boolean result = isDampingLimited(motorId, axisIndex);

        return result;
    }

    /**
     * Test whether the indexed motor is enabled (m_enableMotor).
     *
     * @param axisIndex which axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     * @return true if enabled, otherwise false
     */
    public boolean isMotorEnabled(int axisIndex) {
        Validate.axisIndex(axisIndex, "axis index");

        long motorId = nativeId();
        boolean result = isMotorEnabled(motorId, axisIndex);

        return result;
    }

    /**
     * Test whether the indexed servo is enabled (m_servoMotor).
     *
     * @param axisIndex which axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     * @return true if enabled, otherwise false
     */
    public boolean isServoEnabled(int axisIndex) {
        Validate.axisIndex(axisIndex, "axis index");

        long motorId = nativeId();
        boolean result = isServoEnabled(motorId, axisIndex);

        return result;
    }

    /**
     * Test whether the indexed spring is enabled (m_enableSpring).
     *
     * @param axisIndex which axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     * @return true if enabled, otherwise false
     */
    public boolean isSpringEnabled(int axisIndex) {
        Validate.axisIndex(axisIndex, "axis index");

        long motorId = nativeId();
        boolean result = isSpringEnabled(motorId, axisIndex);

        return result;
    }

    /**
     * Test whether the indexed spring's stiffness is limited
     * (m_springStiffnessLimited).
     *
     * @param axisIndex which axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     * @return true if limited, otherwise false
     */
    public boolean isStiffnessLimited(int axisIndex) {
        Validate.axisIndex(axisIndex, "axis index");

        long motorId = nativeId();
        boolean result = isStiffnessLimited(motorId, axisIndex);

        return result;
    }

    /**
     * Alter the specified parameter of this motor.
     *
     * @param param which parameter (not null)
     * @param values the desired parameter value for each axis (not null,
     * unaffected)
     */
    public void set(MotorParam param, Vector3f values) {
        Validate.nonNull(values, "value");

        long motorId = nativeId();
        switch (param) {
            case Bounce:
                setBounce(motorId, values);
                break;

            case Damping:
                setDamping(motorId, values);
                break;

            case Equilibrium:
                setEquilibrium(motorId, values);
                break;

            case LowerLimit:
                setLowerLimit(motorId, values);
                break;

            case MaxMotorForce:
                setMaxMotorForce(motorId, values);
                break;

            case ServoTarget:
                setServoTarget(motorId, values);
                break;

            case Stiffness:
                setStiffness(motorId, values);
                break;

            case TargetVelocity:
                setTargetVelocity(motorId, values);
                break;

            case UpperLimit:
                setUpperLimit(motorId, values);
                break;

            default:
                int paramIndex = param.nativeIndex();
                setParameter(motorId, paramIndex, values);
        }
    }

    /**
     * Limit or unlimit the damping of the indexed spring
     * (m_springDampingLimited).
     *
     * @param axisIndex which axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     * @param limitDamping true&rarr;limit, false&rarr;don't limit
     * (default=false)
     */
    public void setDampingLimited(int axisIndex, boolean limitDamping) {
        Validate.axisIndex(axisIndex, "axis index");

        long motorId = nativeId();
        setDampingLimited(motorId, axisIndex, limitDamping);
    }

    /**
     * Enable or disable the indexed axis (m_enableMotor).
     *
     * @param axisIndex which axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     * @param enableFlag true&rarr;enable, false&rarr;disable (default=false)
     */
    public void setMotorEnabled(int axisIndex, boolean enableFlag) {
        Validate.axisIndex(axisIndex, "axis index");

        long motorId = nativeId();
        setMotorEnabled(motorId, axisIndex, enableFlag);
    }

    /**
     * Enable or disable the indexed servo (m_servoMotor). This setting has no
     * effect if the corresponding axis is disabled.
     *
     * @param axisIndex which axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     * @param enableFlag true&rarr;enable, false&rarr;disable (default=false)
     */
    public void setServoEnabled(int axisIndex, boolean enableFlag) {
        Validate.axisIndex(axisIndex, "axis index");

        long motorId = nativeId();
        setServoEnabled(motorId, axisIndex, enableFlag);
    }

    /**
     * Enable or disable the indexed spring (m_enableSpring).
     *
     * @param axisIndex which axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     * @param enableFlag true&rarr;enable, false&rarr;disable (default=false)
     */
    public void setSpringEnabled(int axisIndex, boolean enableFlag) {
        Validate.axisIndex(axisIndex, "axis index");

        long motorId = nativeId();
        setSpringEnabled(motorId, axisIndex, enableFlag);
    }

    /**
     * Limit or unlimit the stiffness of the indexed spring
     * (m_springStiffnessLimited).
     *
     * @param axisIndex which axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     * @param limitFlag true&rarr;limit, false&rarr;don't limit (default=false)
     */
    public void setStiffnessLimited(int axisIndex, boolean limitFlag) {
        Validate.axisIndex(axisIndex, "axis index");

        long motorId = nativeId();
        setStiffnessLimited(motorId, axisIndex, limitFlag);
    }
    // *************************************************************************
    // native private methods

    native private static void getBounce(long motorId, Vector3f storeVector);

    native private static void getDamping(long motorId, Vector3f storeVector);

    native private static void
            getEquilibrium(long motorId, Vector3f storeVector);

    native private static void
            getLowerLimit(long motorId, Vector3f storeVector);

    native private static void
            getMaxMotorForce(long motorId, Vector3f storeVector);

    native private static void getParameter(
            long motorId, int parameterIndex, Vector3f storeVector);

    native private static void
            getServoTarget(long motorId, Vector3f storeVector);

    native private static void getStiffness(long motorId, Vector3f storeVector);

    native private static void
            getTargetVelocity(long motorId, Vector3f storeVector);

    native private static void
            getUpperLimit(long motorId, Vector3f storeVector);

    native private static boolean isDampingLimited(long motorId, int axisIndex);

    native private static boolean isMotorEnabled(long motorId, int axisIndex);

    native private static boolean isServoEnabled(long motorId, int axisIndex);

    native private static boolean isSpringEnabled(long motorId, int axisIndex);

    native private static boolean
            isStiffnessLimited(long motorId, int axisIndex);

    native private static void setBounce(long motorId, Vector3f bounceVector);

    native private static void setDamping(long motorId, Vector3f dampingVector);

    native private static void
            setDampingLimited(long motorId, int axisIndex, boolean limitFlag);

    native private static void
            setEquilibrium(long motorId, Vector3f offsetVector);

    native private static void
            setLowerLimit(long motorId, Vector3f offsetVector);

    native private static void
            setMaxMotorForce(long motorId, Vector3f forceVector);

    native private static void
            setMotorEnabled(long motorId, int axisIndex, boolean enableFlag);

    native private static void
            setParameter(long motorId, int parameterIndex, Vector3f vector);

    native private static void
            setServoEnabled(long motorId, int axisIndex, boolean enableFlag);

    native private static void
            setServoTarget(long motorId, Vector3f targetVector);

    native private static void
            setSpringEnabled(long motorId, int axisIndex, boolean enableFlag);

    native private static void
            setStiffness(long motorId, Vector3f stiffnessVector);

    native private static void
            setStiffnessLimited(long motorId, int axisIndex, boolean limitFlag);

    native private static void
            setTargetVelocity(long motorId, Vector3f velocityVector);

    native private static void
            setUpperLimit(long motorId, Vector3f offsetVector);
}
