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

import java.util.ArrayList;
import java.util.Collection;
import java.util.logging.Logger;

/**
 * Named mode bits for contact-and-constraint solvers, based on btSolverMode.
 *
 * @author Stephen Gold sgold@sonic.net
 * @see com.jme3.bullet.SolverInfo#setMode(int)
 */
final public class SolverMode {
    // *************************************************************************
    // constants and loggers

    /**
     * randomize order
     */
    final public static int RandomOrder = 0x1;
    /**
     * friction separate
     */
    final public static int Separate = 0x2;
    /**
     * use warm start
     */
    final public static int WarmStart = 0x4;
    /**
     * use 2 friction directions
     */
    final public static int Use2Directions = 0x10;
    /**
     * enable friction-direction caching
     */
    final public static int CacheDirection = 0x20;
    /**
     * disable velocity-dependent friction direction
     */
    final public static int NoVelocityDependent = 0x40;
    /**
     * cache friendly
     */
    final public static int CacheFriendly = 0x80;
    /**
     * SIMD
     */
    final public static int SIMD = 0x100;
    /**
     * interleave contact and friction constraints
     */
    final public static int Interleave = 0x200;
    /**
     * allow zero-length friction directions
     */
    final public static int AllowZeroLength = 0x400;
    /**
     * disable implicit cone friction
     */
    final public static int NoCone = 0x800;
    /**
     * use articulated warm start
     */
    final public static int ArticulatedWarmStart = 0x1000;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(SolverMode.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private SolverMode() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Generate a textual description of the specified flags.
     *
     * @param flags the solver-mode flags to describe
     * @return description (not null, may be empty)
     */
    public static String describe(int flags) {
        Collection<String> flagList = new ArrayList<>(12);

        if ((flags & RandomOrder) != 0x0) {
            flagList.add("RandomOrder");
        }
        if ((flags & Separate) != 0x0) {
            flagList.add("Separate");
        }
        if ((flags & WarmStart) != 0x0) {
            flagList.add("WarmStart");
        }
        if ((flags & Use2Directions) != 0x0) {
            flagList.add("Use2Directions");
        }
        if ((flags & NoVelocityDependent) == 0x0) {
            flagList.add("VelocityDependent");
        }
        if ((flags & CacheFriendly) != 0x0) {
            flagList.add("CacheFriendly");
        }
        if ((flags & SIMD) != 0x0) {
            flagList.add("SIMD");
        }
        if ((flags & Interleave) != 0x0) {
            flagList.add("Interleave");
        }
        if ((flags & AllowZeroLength) != 0x0) {
            flagList.add("AllowZeroLength");
        }
        if ((flags & NoCone) == 0x0) {
            flagList.add("Cone");
        }
        if ((flags & ArticulatedWarmStart) != 0x0) {
            flagList.add("ArticulatedWarmStart");
        }

        StringBuilder result = new StringBuilder(40);
        boolean addSeparators = false;
        for (String flagName : flagList) {
            if (addSeparators) {
                result.append(',');
            } else {
                addSeparators = true;
            }
            result.append(flagName);
        }

        return result.toString();
    }
}
