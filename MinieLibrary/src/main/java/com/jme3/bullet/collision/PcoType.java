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
 * Named constants for types of PhysicsCollisionObject. See enum
 * CollisionObjectTypes in
 * https://github.com/bulletphysics/bullet3/blob/master/src/BulletCollision/CollisionDispatch/btCollisionObject.h
 *
 * @author Stephen Gold sgold@sonic.net
 * @see PhysicsCollisionObject#getInternalType(long)
 */
final public class PcoType {
    // *************************************************************************
    // constants and loggers

    /**
     * default value, unused in Libbulletjme (native name: CO_COLLISION_OBJECT)
     */
    final public static int generic = 1;
    /**
     * value for a PhysicsVehicle chassis or PhysicsRigidBody (native name:
     * CO_RIGID_BODY)
     */
    final public static int rigid = 2;
    /**
     * value for a PhysicsGhostObject (native name: CO_GHOST_OBJECT)
     */
    final public static int ghost = 4;
    /**
     * value for a PhysicsSoftBody (native name: CO_SOFT_BODY)
     */
    final public static int soft = 8;
    /**
     * unused (native name: CO_HF_FLUID)
     */
    final public static int fluid = 16;
    /**
     * unused (native name: CO_USER_TYPE)
     */
    final public static int user = 32;
    /**
     * value for a MultiBodyCollider (native name: CO_FEATHERSTONE_LINK)
     */
    final public static int collider = 64;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(PcoType.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private PcoType() {
    }
}
