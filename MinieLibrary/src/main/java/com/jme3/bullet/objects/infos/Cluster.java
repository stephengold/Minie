/*
 * Copyright (c) 2019-2020 jMonkeyEngine
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
package com.jme3.bullet.objects.infos;

/**
 * Enumerate the float-valued parameters in a soft-body cluster.
 *
 * @author Stephen Gold sgold@sonic.net
 * @see com.jme3.bullet.objects.PhysicsSoftBody#set(Cluster, int, float)
 */
public enum Cluster {
    // *************************************************************************
    // values

    /**
     * angular damping coefficient (default=0, native field: m_adamping)
     */
    AngularDamping,
    /**
     * linear damping coefficient (default=0, native field: m_ldamping)
     */
    LinearDamping,
    /**
     * matching coefficient (default=0, native field: m_matching)
     */
    Matching,
    /**
     * maximum self-collision impulse (default=100, native field:
     * m_maxSelfCollisionImpulse)
     */
    MaxSelfImpulse,
    /**
     * node-damping coefficient (default=0, native field: m_ndamping)
     */
    NodeDamping,
    /**
     * self-collision impulse factor (default=0.01, native field:
     * m_selfCollisionImpulseFactor)
     */
    SelfImpulse;
    // *************************************************************************
    // new methods exposed

    /**
     * Test whether this parameter can be set to the specified value.
     *
     * @param value the desired parameter value
     * @return true if settable, otherwise false
     */
    public boolean canSet(float value) {
        boolean result = (value >= minValue()) && (value <= maxValue());
        return result;
    }

    /**
     * Determine the default value for this parameter.
     *
     * @return the default parameter value
     */
    public float defValue() {
        float result;
        switch (this) {
            case AngularDamping:
            case LinearDamping:
            case Matching:
            case NodeDamping:
                result = 0f;
                break;

            case MaxSelfImpulse:
                result = 100f;
                break;

            case SelfImpulse:
                result = 0.01f;
                break;

            default:
                throw new IllegalArgumentException("parameter = " + this);
        }

        return result;
    }

    /**
     * Determine the maximum value for this parameter.
     *
     * @return a maximum value, or Float.MAX_VALUE if there's no maximum
     */
    public float maxValue() {
        switch (this) {
            case AngularDamping:
            case LinearDamping:
            case Matching:
            case MaxSelfImpulse:
            case NodeDamping:
            case SelfImpulse:
                return Float.MAX_VALUE;

            default:
                throw new IllegalArgumentException("parameter = " + this);
        }
    }

    /**
     * Determine the minimum value for this parameter.
     *
     * @return a minimum value, or -Float.MAX_VALUE if there's no minimum
     */
    public float minValue() {
        switch (this) {
            case AngularDamping:
            case LinearDamping:
            case Matching:
            case MaxSelfImpulse:
            case NodeDamping:
            case SelfImpulse:
                return -Float.MAX_VALUE;

            default:
                throw new IllegalArgumentException("parameter = " + this);
        }
    }
}
