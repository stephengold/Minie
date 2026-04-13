/*
 * Copyright (c) 2026 jMonkeyEngine
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
 * Named flags used when stepping a {@code PhysicsSpace} simulation.
 *
 * @author Stephen Gold sgold@sonic.net
 * @see com.jme3.bullet.PhysicsSpace#update(float, int, int)
 */
final public class StepFlag {
    // *************************************************************************
    // constants and loggers

    /**
     * enable {@code onContactEnded()} callbacks
     */
    final public static int contactEnded = 0x1;
    /**
     * enable {@code onContactProcessed()} callbacks
     */
    final public static int contactProcessed = 0x2;
    /**
     * enable {@code onContactStarted()} callbacks
     */
    final public static int contactStarted = 0x4;
    /**
     * enable {@code onContactConceived()} callbacks
     */
    final public static int contactConceived = 0x8;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(StepFlag.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private StepFlag() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Generate a textual description of the specified flags.
     *
     * @param flags the step flags to describe, ORed together
     * @return description (not null, may be empty)
     */
    public static String describe(int flags) {
        Collection<String> flagList = new ArrayList<>(3);
        if ((flags & contactConceived) != 0x0) {
            flagList.add("contactConceived");
        }
        if ((flags & contactStarted) != 0x0) {
            flagList.add("contactStarted");
        }
        if ((flags & contactProcessed) != 0x0) {
            flagList.add("contactProcessed");
        }
        if ((flags & contactEnded) != 0x0) {
            flagList.add("contactEnded");
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
    // *************************************************************************
    // native private methods

    native private static void generateJniHeaderFile(); // never invoked
}
