/*
 * Copyright (c) 2019-2022 jMonkeyEngine
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
package com.jme3.bullet.collision;

import java.util.logging.Logger;

/**
 * Named activation states for use with a PhysicsCollisionObject.
 *
 * @author Stephen Gold sgold@sonic.net
 * @see PhysicsCollisionObject#getActivationState()
 */
final public class Activation {
    // *************************************************************************
    // constants and loggers

    /**
     * an active collision object that's subject to deactivation when its timer
     * runs out (native name: ACTIVE_TAG)
     */
    final public static int active = 1;
    /**
     * a deactivated collision object (native name: ISLAND_SLEEPING)
     */
    final public static int sleeping = 2;
    /**
     * a collision object that's timed out but hasn't been deactivated yet?
     * (native name: WANTS_DEACTIVATION)
     */
    final public static int wantsDeactivation = 3;
    /**
     * a collision object that's exempt from deactivation, such as a vehicle or
     * a kinematic rigid body (native name: DISABLE_DEACTIVATION)
     */
    final public static int exempt = 4;
    /**
     * a disabled collision object: this usually indicates that an error has
     * occurred (native name: DISABLE_SIMULATION)
     */
    final public static int error = 5;
    /**
     * lowest legal value, for range checks
     */
    final public static int firstValue = 1;
    /**
     * highest legal value, for range checks
     */
    final public static int lastValue = 5;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(Activation.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private Activation() {
    }
}
