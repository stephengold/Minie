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
package com.jme3.bullet.collision;

import java.util.logging.Logger;

/**
 * Named flags for a PhysicsCollisionEvent. See btContactPointFlags in
 * https://github.com/bulletphysics/bullet3/blob/master/src/BulletCollision/NarrowPhaseCollision/btManifoldPoint.h
 *
 * @author Stephen Gold sgold@sonic.net
 * @see com.jme3.bullet.collision.ManifoldPoints#setFlags(long, int)
 * @see com.jme3.bullet.collision.PhysicsCollisionEvent#getFlags()
 */
final public class ContactPointFlag {
    // *************************************************************************
    // constants and loggers

    /**
     * lateral friction is initialized
     */
    final public static int LATERAL_FRICTION = 0x1;
    /**
     * contact has a constraint-force modifier
     */
    final public static int HAS_CONTACT_CFM = 0x2;
    /**
     * contact has an error-reduction parameter
     */
    final public static int HAS_CONTACT_ERP = 0x4;
    /**
     * contact has stiffness damping
     */
    final public static int CONTACT_STIFFNESS_DAMPING = 0x8;
    /**
     * contact has a friction anchor
     */
    final public static int FRICTION_ANCHOR = 0x10;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(ContactPointFlag.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private ContactPointFlag() {
    }
}
