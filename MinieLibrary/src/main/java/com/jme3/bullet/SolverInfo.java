/*
 * Copyright (c) 2020-2022 jMonkeyEngine
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
package com.jme3.bullet;

import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * Parameters used by the contact-and-constraint solver, based on Bullet's
 * btContactSolverInfo.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class SolverInfo extends NativePhysicsObject {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(SolverInfo.class.getName());
    // *************************************************************************
    // constructors

    /**
     * Instantiate an info that refers to the identified native object. Used
     * internally.
     *
     * @param nativeId the ID of a pre-existing btContactSolverInfo (not zero)
     */
    SolverInfo(long nativeId) {
        Validate.nonZero(nativeId, "native ID");
        super.setNativeIdNotTracked(nativeId);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Return the error-reduction parameter for contact constraints (native
     * field: m_erp2).
     *
     * @return the parameter value
     */
    public float contactErp() {
        long infoId = nativeId();
        float result = getContactErp(infoId);

        return result;
    }

    /**
     * Copy all parameter values from the specified info.
     *
     * @param source the info to copy from (not null, unaffected)
     */
    public void copyAll(SolverInfo source) {
        long thisId = nativeId();
        long sourceId = source.nativeId();
        copyAllParameters(thisId, sourceId);
    }

    /**
     * Determine the global constraint-force mixing parameter (native field:
     * m_globalCfm).
     *
     * @return the parameter value (&ge;0)
     */
    public float globalCfm() {
        long infoId = nativeId();
        float result = getGlobalCfm(infoId);

        return result;
    }

    /**
     * Test whether split impulse is enabled globally (native field:
     * m_splitImpulse).
     *
     * @return true if using split impulse, otherwise false
     */
    public boolean isSplitImpulseEnabled() {
        long infoId = nativeId();
        boolean result = isSplitImpulseEnabled(infoId);

        return result;
    }

    /**
     * Return the error-reduction parameter for non-contact constraints (native
     * field: m_erp).
     *
     * @return the parameter value
     */
    public float jointErp() {
        long infoId = nativeId();
        float result = getJointErp(infoId);
        return result;
    }

    /**
     * Determine the minimum batch size (native field:
     * m_minimumSolverBatchSize).
     *
     * @return the number of constraints in a batch (&ge;1)
     */
    public int minBatch() {
        long infoId = nativeId();
        int result = getMinBatch(infoId);

        return result;
    }

    /**
     * Determine the mode flags (native field: m_solverMode).
     *
     * @return the bitmask value
     * @see com.jme3.bullet.SolverMode
     */
    public int mode() {
        long infoId = nativeId();
        int result = getMode(infoId);

        return result;
    }

    /**
     * Determine the number of iterations (native field: m_numIterations).
     *
     * @return the count (&gt;0)
     */
    public int numIterations() {
        long infoId = nativeId();
        int result = getNumIterations(infoId);

        return result;
    }

    /**
     * Alter the error-reduction parameter for contact constraints (native
     * field: m_erp2).
     *
     * @param erp the desired parameter value (default=0.2)
     */
    public void setContactErp(float erp) {
        long infoId = nativeId();
        setContactErp(infoId, erp);
    }

    /**
     * Alter the global constraint-force mixing parameter (native field:
     * m_globalCfm).
     *
     * @param cfm the desired parameter value (&ge;0, default=0)
     */
    public void setGlobalCfm(float cfm) {
        Validate.nonNegative(cfm, "mixing parameter");

        long infoId = nativeId();
        setGlobalCfm(infoId, cfm);
    }

    /**
     * Alter the error-reduction parameter for non-contact constraints (native
     * field: m_erp).
     *
     * @param erp the desired parameter value (default=0.2)
     */
    public void setJointErp(float erp) {
        long infoId = nativeId();
        setJointErp(infoId, erp);
    }

    /**
     * Alter the minimum batch size (native field: m_minimumSolverBatchSize).
     *
     * @param numConstraints the desired number of constraints per batch (&ge;1,
     * default=128)
     */
    public void setMinBatch(int numConstraints) {
        Validate.positive(numConstraints, "number of constraints");

        long infoId = nativeId();
        setMinBatch(infoId, numConstraints);
    }

    /**
     * Alter the mode flags (native field: m_solverMode).
     *
     * @param flags the desired bitmask (default=0x114 for a MultiBodySpace,
     * otherwise 0x104)
     * @see com.jme3.bullet.SolverMode
     */
    public void setMode(int flags) {
        long infoId = nativeId();
        setMode(infoId, flags);
    }

    /**
     * Alter the number of iterations (native field: m_numIterations).
     * <p>
     * Use 4 for low quality, 20 for high quality.
     *
     * @param numIterations the desired number of iterations (&ge;1, default=10)
     */
    public void setNumIterations(int numIterations) {
        Validate.positive(numIterations, "number of iterations");

        long infoId = nativeId();
        setNumIterations(infoId, numIterations);
    }

    /**
     * Alter whether split impulse is enabled globally (native field:
     * m_splitImpulse).
     *
     * @param setting the desired setting (default=true)
     */
    public void setSplitImpulseEnabled(boolean setting) {
        long infoId = nativeId();
        setSplitImpulseEnabled(infoId, setting);
    }

    /**
     * Alter the error-reduction parameter (ERP) used with split impulse (native
     * field: m_splitImpulseTurnErp).
     *
     * @param erp the parameter (default=0.1)
     */
    public void setSplitImpulseErp(float erp) {
        long infoId = nativeId();
        setSplitImpulseErp(infoId, erp);
    }

    /**
     * Alter the degree of penetration at which split impulse will be used. This
     * setting no effect while split impulse is enabled globally (native field:
     * m_splitImpulsePenetrationThreshold).
     *
     * @param penetration the penetration threshold (in physics-space units,
     * default=-0.04)
     */
    public void setSplitImpulseThreshold(float penetration) {
        long infoId = nativeId();
        setSplitImpulseThreshold(infoId, penetration);
    }

    /**
     * Determine the error-reduction parameter (ERP) used with split impulse
     * (native field: m_splitImpulseTurnErp).
     *
     * @return the parameter value
     */
    public float splitImpulseErp() {
        long infoId = nativeId();
        float result = getSplitImpulseErp(infoId);

        return result;
    }

    /**
     * Determine the minimum degree of penetration at which split impulse would
     * be used, assuming it's not enabled globally (native field:
     * m_splitImpulsePenetrationThreshold).
     *
     * @return the parameter value
     */
    public float splitImpulseThreshold() {
        long infoId = nativeId();
        float result = getSplitImpulseThreshold(infoId);

        return result;
    }
    // *************************************************************************
    // native private methods

    native private static void copyAllParameters(long targetId, long sourceId);

    native private static float getContactErp(long infoId);

    native private static float getGlobalCfm(long infoId);

    native private static float getJointErp(long infoId);

    native private static int getMinBatch(long infoId);

    native private static int getMode(long infoId);

    native private static int getNumIterations(long infoId);

    native private static float getSplitImpulseErp(long infoId);

    native private static float getSplitImpulseThreshold(long infoId);

    native private static boolean isSplitImpulseEnabled(long infoId);

    native private static void setContactErp(long infoId, float erp);

    native private static void setGlobalCfm(long infoId, float cfm);

    native private static void setJointErp(long infoId, float erp);

    native private static void setMinBatch(long infoId, int numConstraints);

    native private static void setMode(long infoId, int flags);

    native private static void setNumIterations(long infoId, int numIterations);

    native private static void setSplitImpulseEnabled(long infoId,
            boolean enable);

    native private static void setSplitImpulseErp(long infoId, float erp);

    native private static void setSplitImpulseThreshold(long infoId,
            float penetration);
}
