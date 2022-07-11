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
 * Named anisotropic friction modes for use with a PhysicsCollisionObject.
 *
 * @author Stephen Gold sgold@sonic.net
 * @see PhysicsCollisionObject#hasAnisotropicFriction(int)
 * @see PhysicsCollisionObject#setAnisotropicFriction(com.jme3.math.Vector3f,
 * int)
 */
final public class AfMode {
    // *************************************************************************
    // constants and loggers

    /**
     * no anisotropic friction (native name: CF_ANISOTROPIC_FRICTION_DISABLED)
     */
    final public static int none = 0x0;
    /**
     * basic anisotropic friction mode (native name: CF_ANISOTROPIC_FRICTION)
     */
    final public static int basic = 0x1;
    /**
     * anisotropic rolling friction mode (native name:
     * CF_ANISOTROPIC_ROLLING_FRICTION)
     */
    final public static int rolling = 0x2;
    /**
     * bitmask for either kind of anisotropic friction
     */
    final public static int either = 0x3;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(AfMode.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private AfMode() {
    }
}
