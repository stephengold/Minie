/*
 * Copyright (c) 2020 jMonkeyEngine
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
public class SolverInfo {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(SolverInfo.class.getName());
    // *************************************************************************
    // fields

    /**
     * unique identifier of the btContactSolverInfo (not zero)
     */
    private long nativeId;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an info that refers to the identified native object. Used
     * internally.
     *
     * @param nativeId the pre-existing btContactSolverInfo to refer to (not
     * zero)
     */
    SolverInfo(long nativeId) {
        assert nativeId != 0L;
        this.nativeId = nativeId;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy all parameter values from the specified info.
     *
     * @param source the info to copy from (not null, unaffected)
     */
    public void copyAll(SolverInfo source) {
        long sourceId = source.nativeId();
        copyAllParameters(nativeId, sourceId);
    }

    /**
     * Determine the global constraint-force mixing parameter (native field:
     * m_globalCfm).
     *
     * @return the parameter value (&ge;0)
     */
    public float globalCfm() {
        float result = getGlobalCfm(nativeId);
        return result;
    }

    /**
     * Determine the minimum batch size (native field:
     * m_minimumSolverBatchSize).
     *
     * @return the number of constraints in a batch (&ge;1)
     */
    public int minBatch() {
        int result = getMinBatch(nativeId);
        return result;
    }

    /**
     * Determine the mode flags (native field: m_solverMode).
     *
     * @return the bitmask value
     * @see com.jme3.bullet.SolverMode
     */
    public int mode() {
        int result = getMode(nativeId);
        return result;
    }

    /**
     * Determine the unique ID of the btContactSolverInfo.
     *
     * @return the identifier (not zero)
     */
    public long nativeId() {
        assert nativeId != 0L;
        return nativeId;
    }

    /**
     * Determine the number of iterations (native field: m_numIterations).
     *
     * @return the count (&gt;0)
     */
    public int numIterations() {
        int result = getNumIterations(nativeId);
        return result;
    }

    /**
     * Alter the global constraint-force mixing parameter (native field:
     * m_globalCfm).
     *
     * @param cfm the desired parameter value (&ge;0, default=0)
     */
    public void setGlobalCfm(float cfm) {
        Validate.nonNegative(cfm, "mixing parameter");
        setGlobalCfm(nativeId, cfm);
    }

    /**
     * Alter the minimum batch size (native field: m_minimumSolverBatchSize).
     *
     * @param numConstraints the desired number of constraints per batch (&ge;1,
     * default=128)
     */
    public void setMinBatch(int numConstraints) {
        Validate.positive(numConstraints, "number of constraints");
        setMinBatch(nativeId, numConstraints);
    }

    /**
     * Alter the mode flags (native field: m_solverMode).
     *
     * @param flags the desired bitmask (default=0x114 for a MultiBodySpace,
     * otherwise 0x104)
     * @see com.jme3.bullet.SolverMode
     */
    public void setMode(int flags) {
        setMode(nativeId, flags);
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
        setNumIterations(nativeId, numIterations);
    }
    // *************************************************************************
    // native methods

    native private void copyAllParameters(long targetId, long sourceId);

    native private float getGlobalCfm(long infoId);

    native private int getMinBatch(long infoId);

    native private int getMode(long infoId);

    native private int getNumIterations(long infoId);

    native private void setGlobalCfm(long infoId, float cfm);

    native private void setMinBatch(long infoId, int numConstraints);

    native private void setMode(long infoId, int flags);

    native private void setNumIterations(long infoId, int numIterations);
}
