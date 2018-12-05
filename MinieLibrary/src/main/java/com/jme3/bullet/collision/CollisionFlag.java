/*
 * Copyright (c) 2018 jMonkeyEngine
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
 * Named collision flags for a PhysicsCollisionObject.
 *
 * @author Stephen Gold sgold@sonic.net
 * @see PhysicsCollisionObject#setCollisionFlags(long, int)
 */
public class CollisionFlag {
    // *************************************************************************
    // constants and loggers

    /**
     * flag for a static object
     */
    final public static int staticObject = 0x0001;
    /**
     * flag for a kinematic object
     */
    final public static int kinematicObject = 0x0002;
    /**
     * flag for an object with no contact response, such as a PhysicsGhostObject
     */
    final public static int noContactResponse = 0x0004;
    /**
     * flag to enable a custom material callback for per-triangle
     * friction/restitution (not supported by Minie)
     */
    final public static int customMaterialCallback = 0x0008;
    /**
     * flag for a character object, such as a PhysicsCharacter
     */
    final public static int characterObject = 0x0010;
    /**
     * flag to disable debug visualization (not supported by Minie)
     */
    final public static int disableVisualizeObject = 0x0020;
    /**
     * flag to disable parallel/SPU processing (not supported by Minie)
     */
    final public static int disableSpuCollisionProcessing = 0x0040;
    /**
     * flag not supported by Minie
     */
    final public static int hasContactStiffnessDamping = 0x0080;
    /**
     * flag not supported by Minie: use
     * {@link PhysicsCollisionObject#setDebugMaterial(com.jme3.material.Material)}
     * instead
     */
    final public static int hasCustomDebugRenderingColor = 0x0100;
    /**
     * flag not supported by Minie
     */
    final public static int hasFrictionAnchor = 0x0200;
    /**
     * flag not supported by Minie
     */
    final public static int hasCollisionSoundTrigger = 0x0400;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(CollisionFlag.class.getName());
}
