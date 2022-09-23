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
 * Enumerate the float-valued parameters in a SoftBodyConfig.
 *
 * @author Stephen Gold sgold@sonic.net
 * @see SoftBodyConfig#get(Sbcp)
 */
public enum Sbcp {
    // *************************************************************************
    // values

    /**
     * anchor hardness coefficient (&ge;0, &le;1, default=0.7, native field:
     * kAHR)
     */
    AnchorHardness,
    /**
     * cluster-versus-kinetic hardness coefficient (&ge;0, &le;1, default=1,
     * native field: kSKHR_CL)
     */
    ClusterKineticHardness,
    /**
     * cluster-versus-kinetic impulse-split coefficient (&ge;0, &le;1,
     * default=0.5, native field: kSK_SPLT_CL)
     */
    ClusterKineticSplit,
    /**
     * cluster-versus-rigidBody hardness coefficient (&ge;0, &le;1, default=0.1,
     * native field: kSRHR_CL)
     */
    ClusterRigidHardness,
    /**
     * cluster-versus-rigidBody impulse-split coefficient (&ge;0, &le;1,
     * default=0.5, native field: kSR_SPLT_CL)
     */
    ClusterRigidSplit,
    /**
     * cluster-versus-softBody hardness coefficient (&ge;0, &le;1, default=0.5,
     * native field: kSSHR_CL)
     */
    ClusterSoftHardness,
    /**
     * cluster-versus-softBody impulse-split coefficient (&ge;0, &le;1,
     * default=0.5, native field: kSS_SPLT_CL).
     */
    ClusterSoftSplit,
    /**
     * damping coefficient (&ge;0, &le;1, default=0, native field: kDP)
     */
    Damping,
    /**
     * drag coefficient (&ge;0, default=0, native field: kDG)
     */
    Drag,
    /**
     * dynamic friction coefficient (&ge;0, &lt;1, default=0.2, native field:
     * kDF)
     */
    DynamicFriction,
    /**
     * contact hardness coefficient for static or kinematic rigid bodies (&ge;0,
     * &le;1, default=0.1, native field: kKHR)
     */
    KineticHardness,
    /**
     * lift coefficient (&ge;0, default=0, native field: kLF)
     */
    Lift,
    /**
     * maximum volume ratio for the pose (default=1, native field: maxvolume)
     */
    MaxVolumeRatio,
    /**
     * pose-matching coefficient: how strongly the soft body will tend to return
     * to its default pose (&ge;0, &le;1, default=0, native field: kMT)
     */
    PoseMatching,
    /**
     * pressure coefficient (default=0, native field: kPR)
     */
    Pressure,
    /**
     * contact hardness coefficient for dynamic rigid bodies (&ge;0, &le;1,
     * default=1, native field: kCHR)
     */
    RigidHardness,
    /**
     * soft-body contact hardness coefficient (&ge;0, &le;1, default=1, native
     * field: kSHR)
     */
    SoftHardness,
    /**
     * time scale (default=1, native field: timescale)
     */
    TimeScale,
    /**
     * velocity correction factor (Baumgarte) (default=1, native field: kVCF)
     */
    VelocityCorrection,
    /**
     * volume conservation coefficient (&ge;0, default=0, native field: kVC)
     */
    VolumeConservation;
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
     * Determine the default value for this parameter.
     *
     * @return the default parameter value
     */
    public float defValue() {
        float result;
        switch (this) {
            case Damping:
            case Drag:
            case Lift:
            case PoseMatching:
            case Pressure:
            case VolumeConservation:
                result = 0f;
                break;

            case ClusterRigidHardness:
            case KineticHardness:
                result = 0.1f;
                break;

            case DynamicFriction:
                result = 0.2f;
                break;

            case ClusterKineticSplit:
            case ClusterRigidSplit:
            case ClusterSoftHardness:
            case ClusterSoftSplit:
                result = 0.5f;
                break;

            case AnchorHardness:
                result = 0.7f;
                break;

            case ClusterKineticHardness:
            case MaxVolumeRatio:
            case RigidHardness:
            case SoftHardness:
            case TimeScale:
            case VelocityCorrection:
                result = 1f;
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
            case AnchorHardness:
            case ClusterKineticHardness:
            case ClusterKineticSplit:
            case ClusterRigidHardness:
            case ClusterRigidSplit:
            case ClusterSoftHardness:
            case ClusterSoftSplit:
            case DynamicFriction:
            case KineticHardness:
            case PoseMatching:
            case RigidHardness:
            case SoftHardness:
                return 1f;

            case Damping:
            case Drag:
            case Lift:
            case MaxVolumeRatio:
            case Pressure:
            case TimeScale:
            case VelocityCorrection:
            case VolumeConservation:
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
            case MaxVolumeRatio:
            case Pressure:
            case TimeScale:
            case VelocityCorrection:
                return -Float.MAX_VALUE;

            case AnchorHardness:
            case ClusterKineticHardness:
            case ClusterKineticSplit:
            case ClusterRigidHardness:
            case ClusterRigidSplit:
            case ClusterSoftHardness:
            case ClusterSoftSplit:
            case Damping:
            case Drag:
            case DynamicFriction:
            case KineticHardness:
            case Lift:
            case PoseMatching:
            case RigidHardness:
            case SoftHardness:
            case VolumeConservation:
                return 0f;

            default:
                throw new IllegalArgumentException("parameter = " + this);
        }
    }
}
