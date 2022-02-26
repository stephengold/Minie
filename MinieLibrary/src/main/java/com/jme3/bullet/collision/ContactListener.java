/*
 * Copyright (c) 2022 jMonkeyEngine
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

/**
 * Interface to receive immediate notifications when 2 collision objects come
 * into contact. Note that Bullet implements persistent contacts, so
 * btManifoldPoint instances are reused from step to step.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public interface ContactListener {
    /**
     * Invoked immediately after a contact manifold is removed. Invoked once for
     * each contact point, up to 4 times per manifold.
     *
     * @param objectA the first involved object (not null)
     * @param objectB the 2nd involved object (not null)
     * @param manifoldPointId the native ID of the btManifoldPoint (not 0)
     */
    void onContactEnded(PhysicsCollisionObject objectA,
            PhysicsCollisionObject objectB, long manifoldPointId);

    /**
     * Invoked immediately after a contact point is refreshed without being
     * removed. Skipped for Sphere-Sphere contacts.
     *
     * @param objectA the first involved object (not null)
     * @param objectB the 2nd involved object (not null)
     * @param manifoldPointId the native ID of the btManifoldPoint (not 0)
     */
    void onContactProcessed(PhysicsCollisionObject objectA,
            PhysicsCollisionObject objectB, long manifoldPointId);

    /**
     * Invoked immediately after a new contact manifold is created. Invoked once
     * for each contact point, up to 4 times per manifold.
     *
     * @param objectA the first involved object (not null)
     * @param objectB the 2nd involved object (not null)
     * @param manifoldPointId the native ID of the btManifoldPoint (not 0)
     */
    void onContactStarted(PhysicsCollisionObject objectA,
            PhysicsCollisionObject objectB, long manifoldPointId);
}
