/*
 Copyright (c) 2019, Stephen Gold
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
package jme3utilities.minie.wizard;

import com.jme3.bullet.animation.RangeOfMotion;
import java.util.concurrent.Callable;
import java.util.logging.Logger;

/**
 * A callable for asynchronously estimating the ranges of motion of the loaded
 * C-G model.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class RomCallable implements Callable<RangeOfMotion[]> {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final static Logger logger
            = Logger.getLogger(RomCallable.class.getName());
    // *************************************************************************
    // fields

    /**
     * loaded C-G model
     */
    final private Model model;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a callable for the specified C-G model.
     */
    RomCallable(Model loadedModel) {
        model = loadedModel;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Estimate the range of motion of each linked bone over all animations in
     * the loaded C-G model.
     *
     * @return a new map from bone indices to ranges of motion
     */
    @Override
    public RangeOfMotion[] call() throws Exception {
        long sleepMilliseconds = 3000;
        Thread.sleep(sleepMilliseconds);
        /*
         * Calculate ranges of motion.
         */
        int numBones = model.countBones();
        RangeOfMotion[] roms = new RangeOfMotion[numBones];
        for (int boneIndex = 0; boneIndex < numBones; ++boneIndex) {
            if (model.isBoneLinked(boneIndex)) {
                roms[boneIndex] = new RangeOfMotion(
                        1f, -1f, 1f, -1f, 1f, -1f);
            }
        }

        return roms;
    }
}
