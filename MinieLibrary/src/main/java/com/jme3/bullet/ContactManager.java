/*
 * Copyright (c) 2023 jMonkeyEngine
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

import com.jme3.bullet.collision.ContactListener;
import com.jme3.bullet.collision.PhysicsCollisionListener;

/**
 * Manage notifications when collision objects in a specific PhysicsSpace come
 * into contact.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public interface ContactManager extends ContactListener {
    // *************************************************************************
    // new methods exposed

    /**
     * Register the specified listener for new contacts.
     * <p>
     * During distributeEvents(), registered listeners are notified of all new
     * contacts since the previous distributeEvents().
     *
     * @param listener the listener to register (not null, alias created)
     */
    void addCollisionListener(PhysicsCollisionListener listener);

    /**
     * Register the specified listener for immediate contact notifications.
     *
     * @param listener the listener to register (not null, alias created)
     * @param doEnded true to enable {@code onContactEnded()} callbacks for the
     * listener, false to skip them
     * @param doProcessed true to enable {@code onContactProcessed()} callbacks
     * for the listener, false to skip them
     * @param doStarted true to enable {@code onContactStarted()} callbacks for
     * the listener, false to skip them
     */
    void addContactListener(ContactListener listener,
            boolean doEnded, boolean doProcessed, boolean doStarted);

    /**
     * Register the specified listener for ongoing contacts.
     * <p>
     * During distributeEvents(), registered listeners are notified of all
     * ongoing contacts EXCEPT Sphere-Sphere contacts.
     *
     * @param listener the listener to register (not null, alias created)
     */
    void addOngoingCollisionListener(PhysicsCollisionListener listener);

    /**
     * Count how many collision listeners are registered.
     *
     * @return the count (&ge;0)
     */
    int countCollisionListeners();

    /**
     * Distribute queued collision events to registered listeners.
     */
    void distributeEvents();

    /**
     * De-register the specified listener for new contacts.
     *
     * @param listener the listener to de-register (not null)
     */
    void removeCollisionListener(PhysicsCollisionListener listener);

    /**
     * De-register the specified listener for immediate contact notifications.
     *
     * @param listener the listener to de-register (not null)
     */
    void removeContactListener(ContactListener listener);

    /**
     * De-register the specified listener for ongoing contacts.
     *
     * @param listener the listener to de-register (not null)
     */
    void removeOngoingCollisionListener(PhysicsCollisionListener listener);

    /**
     * Update the associated PhysicsSpace. This method should be invoked from
     * the thread that created the space.
     *
     * @param timeInterval the time interval to simulate (in seconds, &ge;0)
     * @param maxSteps the maximum number of steps of size {@code accuracy}
     * (&ge;1) or 0 for a single step of size {@code timeInterval}
     */
    void update(float timeInterval, int maxSteps);
}
