/*
Copyright (c) 2016, Riccardo Balbo
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package vhacd;

import com.jme3.util.BufferUtils;
import com.jme3.util.SafeArrayList;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.List;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * Utility class to perform Volumetric-Hierarchical Approximate Convex
 * Decomposition on an indexed mesh.
 */
public class VHACD {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(VHACD.class.getName());
    // *************************************************************************
    // fields

    /**
     * list of registered progress listeners
     */
    final private static List<VHACDProgressListener> progressListeners
            = new SafeArrayList<>(VHACDProgressListener.class);
    /**
     * list of hulls computed during the latest decomposition
     */
    private static VHACDResults results;
    // *************************************************************************
    // new methods exposed

    /**
     * Register the specified progress listener.
     *
     * @param listener the listener to register (not null, alias created)
     */
    public static void addProgressListener(VHACDProgressListener listener) {
        Validate.nonNull(listener, "listener");
        assert !progressListeners.contains(listener);

        progressListeners.add(listener);
    }

    public static VHACDResults compute(float positions[], int indexes[],
            VHACDParameters params) {
        FloatBuffer b_pos = BufferUtils.createFloatBuffer(positions);
        IntBuffer b_ind = BufferUtils.createIntBuffer(indexes);
        results = new VHACDResults();
        compute(b_pos, b_ind, params.getId(), params.getDebugEnabled());

        return results;
    }

    /**
     * De-register the specified progress listener.
     *
     * @param listener the listener to de-register (not null, unaffected)
     */
    public static void removeProgressListener(VHACDProgressListener listener) {
        Validate.nonNull(listener, "listener");

        boolean success = progressListeners.remove(listener);
        assert success;
    }
    // *************************************************************************
    // private methods

    /**
     * Add a hull to the result.
     * <p>
     * This method is invoked from native code.
     */
    private static void addHull(long hullId) {
        VHACDHull hull = new VHACDHull(hullId);
        results.add(hull);
    }

    /**
     * Update all progress listeners.
     * <p>
     * This method is invoked from native code.
     *
     * @param overallPercent an overall completion percentage (&ge;0, &le;100)
     * @param stagePercent a completion percentage for the current stage (&ge;0,
     * &le;100)
     * @param operationPercent a completion percentage for the current operation
     * (&ge;0, &le;100)
     * @param stageName the name of the current stage
     * @param operationName the name of the current operation
     */
    private static void update(double overallPercent, double stagePercent,
            double operationPercent, String stageName, String operationName) {
        for (VHACDProgressListener listener : progressListeners) {
            listener.update(overallPercent, stagePercent,
                    operationPercent, stageName, operationName);
        }
    }
    // *************************************************************************
    // native methods

    native private static void compute(FloatBuffer positions, IntBuffer indices,
            long paramsId, boolean debugEnabled);
}
