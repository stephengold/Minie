/*
 Copyright (c) 2022, Stephen Gold
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package jme3utilities.minie.test.shapes;

import java.util.logging.Logger;
import vhacd.VHACDProgressListener;

/**
 * A simple progress listener for V-HACD.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class ProgressListener implements VHACDProgressListener {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(ProgressListener.class.getName());
    // *************************************************************************
    // fields

    /**
     * overall completion percentage as of the latest update
     */
    private double lastOP = -1.0;
    /**
     * text printed at the start of each output line
     */
    final private String prefix;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a listener for the specified prefix.
     *
     * @param prefix the prefix for each line of output
     */
    ProgressListener(String prefix) {
        this.prefix = prefix;
    }
    // *************************************************************************
    // VHACDProgressListener methods

    /**
     * Callback invoked (by native code) for progress updates.
     *
     * @param overallPercent an overall completion percentage (&ge;0, &le;100)
     * @param stagePercent a completion percentage for the current stage (&ge;0,
     * &le;100)
     * @param operationPercent a completion percentage for the current operation
     * (&ge;0, &le;100)
     * @param stageName the name of the current stage
     * @param operationName the name of the current operation
     */
    @Override
    public void update(double overallPercent, double stagePercent,
            double operationPercent, String stageName, String operationName) {
        if (overallPercent != lastOP) {
            System.out.printf("%s %.0f%% complete%n", prefix, overallPercent);
            this.lastOP = overallPercent;
        }
    }
}
