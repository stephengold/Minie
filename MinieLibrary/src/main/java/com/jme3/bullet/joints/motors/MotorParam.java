/*
 * Copyright (c) 2019 jMonkeyEngine
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

/**
 * Enumerate certain parameters of a RotationMotor or TranslationMotor, based on
 * Bullet's btConstraintParams.
 *
 * @author Stephen Gold sgold@sonic.net
 * @see RotationMotor#get(com.jme3.bullet.joints.motors.MotorParam)
 * @see RotationMotor#set(com.jme3.bullet.joints.motors.MotorParam, float)
 * @see TranslationMotor#get(com.jme3.bullet.joints.motors.MotorParam,
 * com.jme3.math.Vector3f)
 * @see TranslationMotor#set(com.jme3.bullet.joints.motors.MotorParam,
 * com.jme3.math.Vector3f)
 */
public enum MotorParam {
    // *************************************************************************
    // values

    /**
     * restitution/bounce factor at the limits (m_bounce, default=0)
     */
    Bounce,
    /**
     * spring's viscous damping ratio (m_springDamping, default=0)
     */
    Damping,
    /**
     * spring's equilibrium point (m_equilibriumPoint, default=0)
     */
    Equilibrium,
    /**
     * constraint lower limit (m_loLimit/m_lowerLimit, default=1[rot] or
     * 0[translate])
     */
    LowerLimit,
    /**
     * maximum motor force (m_maxMotorForce, default=6[rot] or 0[translate])
     */
    MaxMotorForce,
    /**
     * constraint-force mixing parameter between limits (m_motorCFM, native
     * name: BT_CONSTRAINT_CFM, default=0)
     */
    MotorCfm,
    /**
     * error-reduction parameter between limits (m_motorERP, native name:
     * BT_CONSTRAINT_ERP, default=0.9)
     */
    MotorErp,
    /**
     * servo's target (m_servoTarget, default=0)
     */
    ServoTarget,
    /**
     * spring's stiffness (m_springStiffness, default=0)
     */
    Stiffness,
    /**
     * constraint-force mixing parameter at limits (m_stopCFM, native name:
     * BT_CONSTRAINT_STOP_CFM, default=0)
     */
    StopCfm,
    /**
     * error-reduction parameter at limits (m_stopERP, native name:
     * BT_CONSTRAINT_STOP_ERP, default=0.2)
     */
    StopErp,
    /**
     * motor's target velocity (m_targetVelocity, default=0)
     */
    TargetVelocity,
    /**
     * constraint upper limit (m_hiLimit/m_upperLimit, default=-1[rot] or
     * 0[translate])
     */
    UpperLimit;
    // *************************************************************************
    // new methods exposed

    /**
     * Test whether this parameter can be set to the specified value.
     *
     * @param value the desired parameter value
     * @return true if settable, otherwise false
     */
    public boolean canSet(float value) {
        boolean result = (minValue() <= value) && (value <= maxValue());
        return result;
    }

    /**
     * Determine the default value for this parameter in a RotationMotor.
     *
     * @return the default parameter value
     */
    public float defaultForRotationMotor() {
        float result;
        switch (this) {
            case UpperLimit:
                result = -1f;
                break;

            case Bounce:
            case Damping:
            case Equilibrium:
            case MotorCfm:
            case ServoTarget:
            case Stiffness:
            case StopCfm:
            case TargetVelocity:
                result = 0f;
                break;

            case StopErp:
                result = 0.2f;
                break;

            case MotorErp:
                result = 0.9f;
                break;

            case LowerLimit:
                result = 1f;
                break;

            case MaxMotorForce:
                result = 6f;
                break;

            default:
                throw new IllegalArgumentException(toString());
        }

        return result;
    }

    /**
     * Determine the default value for this parameter in a TranslationMotor.
     *
     * @return the default parameter value
     */
    public float defaultForTranslationMotor() {
        float result;
        switch (this) {
            case Bounce:
            case Damping:
            case Equilibrium:
            case LowerLimit:
            case MaxMotorForce:
            case MotorCfm:
            case ServoTarget:
            case Stiffness:
            case StopCfm:
            case TargetVelocity:
            case UpperLimit:
                result = 0f;
                break;

            case StopErp:
                result = 0.2f;
                break;

            case MotorErp:
                result = 0.9f;
                break;

            default:
                throw new IllegalArgumentException(toString());
        }

        return result;
    }

    /**
     * Determine the maximum value for this parameter.
     *
     * @return a maximum value, or Float.MAX_VALUE if there's no maximum
     */
    public float maxValue() {
        float result;
        switch (this) {
            case Bounce:
            case MotorCfm:
            case MotorErp:
            case StopCfm:
            case StopErp:
                result = 1f;
                break;

            case Damping:
            case Equilibrium:
            case LowerLimit:
            case MaxMotorForce:
            case ServoTarget:
            case Stiffness:
            case TargetVelocity:
            case UpperLimit:
                result = Float.MAX_VALUE;
                break;

            default:
                throw new IllegalArgumentException(toString());
        }

        return result;
    }

    /**
     * Determine the minimum value for this parameter.
     *
     * @return a minimum value, or -Float.MAX_VALUE if there's no minimum
     */
    public float minValue() {
        float result;
        switch (this) {
            case Equilibrium:
            case LowerLimit:
            case ServoTarget:
            case TargetVelocity:
            case UpperLimit:
                result = -Float.MAX_VALUE;
                break;

            case Bounce:
            case Damping:
            case MaxMotorForce:
            case MotorCfm:
            case MotorErp:
            case Stiffness:
            case StopCfm:
            case StopErp:
                result = 0f;
                break;

            default:
                throw new IllegalArgumentException(toString());
        }

        return result;
    }

    /**
     * Determine the parameter's index in native code.
     *
     * @return the index
     */
    public int nativeIndex() {
        switch (this) {
            case MotorCfm:
                return 3;
            case MotorErp:
                return 1;
            case StopCfm:
                return 4;
            case StopErp:
                return 2;

            default:
                throw new IllegalArgumentException(toString());
        }
    }

    /**
     * Determine the serialization-tag suffix for this parameter.
     *
     * @return the tag suffix
     */
    public String tagSuffix() {
        return "_" + toString();
    }
}
