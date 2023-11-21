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
import com.jme3.bullet.collision.PersistentManifolds;
import com.jme3.bullet.collision.PhysicsCollisionEvent;
import com.jme3.bullet.collision.PhysicsCollisionListener;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Deque;
import java.util.List;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * Manage notifications when collision objects in a specific PhysicsSpace come
 * into contact.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class ContactManager implements ContactListener {
    // *************************************************************************
    // constants and loggers

    /**
     * bitmask to indicate that the {@code onContactEnded()} method of a
     * particular ContactListener should be invoked
     */
    final private static int invokeEnded = 0x2;
    /**
     * bitmask to indicate that the {@code onContactProcessed()} method of a
     * particular ContactListener should be invoked
     */
    final private static int invokeProcessed = 0x10;
    /**
     * bitmask to indicate that the {@code onContactStarted()} method of a
     * particular ContactListener should be invoked
     */
    final private static int invokeStarted = 0x80;
    /**
     * message logger for this class
     */
    final static Logger logger
            = Logger.getLogger(ContactManager.class.getName());
    // *************************************************************************
    // fields

    /**
     * true to request {@code onContactEnded()} callbacks for the space, false
     * to skip them
     */
    private boolean doEnded = false;
    /**
     * true to request {@code onContactProcessed()} callbacks for the space,
     * false to skip them
     */
    private boolean doProcessed = false;
    /**
     * true to request {@code onContactStarted()} callbacks for the space, false
     * to skip them
     */
    private boolean doStarted = false;
    /**
     * registered listeners for delayed notification of ongoing contacts
     */
    final private Collection<PhysicsCollisionListener> ongoingListeners
            = new ArrayList<>(4);
    /**
     * registered listeners for delayed notification of new contacts
     */
    final private Collection<PhysicsCollisionListener> startedListeners
            = new ArrayList<>(4);
    /**
     * contact-processed PCEs not yet distributed to listeners
     */
    final private Deque<PhysicsCollisionEvent> ongoingEvents
            = new ArrayDeque<>(99);
    /**
     * contact-started PCEs not yet distributed to listeners
     */
    final private Deque<PhysicsCollisionEvent> startedEvents
            = new ArrayDeque<>(99);
    /**
     * list of registered listeners for immediate contact notifications
     * (parallel with {@code immediateListenerFlags})
     */
    final private List<ContactListener> immediateListeners = new ArrayList<>(4);
    /**
     * list of invocation flags for immediate contact notifications (parallel
     * with {@code immediateListeners})
     */
    final private List<Integer> immediateListenerFlags = new ArrayList<>(4);
    /**
     * PhysicsSpace whose notifications are being managed
     */
    final private PhysicsSpace space;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a manager for the specified PhysicsSpace.
     *
     * @param space the PhysicsSpace whose notifications will be managed (not
     * null, alias created)
     */
    ContactManager(PhysicsSpace space) {
        Validate.nonNull(space, "space");
        this.space = space;
    }
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
    synchronized void addCollisionListener(PhysicsCollisionListener listener) {
        Validate.nonNull(listener, "listener");
        assert listener != this;
        assert listener != space;
        assert !startedListeners.contains(listener);

        startedListeners.add(listener);
        this.doStarted = true;
    }

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
    synchronized void addContactListener(ContactListener listener,
            boolean doEnded, boolean doProcessed, boolean doStarted) {
        Validate.nonNull(listener, "listener");
        assert listener != this;
        assert listener != space;
        assert !immediateListeners.contains(listener);

        immediateListeners.add(listener);

        int encodedFlags = (doEnded ? invokeEnded : 0x0)
                | (doProcessed ? invokeProcessed : 0x0)
                | (doStarted ? invokeStarted : 0x0);
        immediateListenerFlags.add(encodedFlags);

        if (doEnded) {
            this.doEnded = true;
        }
        if (doProcessed) {
            this.doProcessed = true;
        }
        if (doStarted) {
            this.doStarted = true;
        }
    }

    /**
     * Register the specified listener for ongoing contacts.
     * <p>
     * During distributeEvents(), registered listeners are notified of all
     * ongoing contacts EXCEPT Sphere-Sphere contacts.
     *
     * @param listener the listener to register (not null, alias created)
     */
    synchronized void addOngoingCollisionListener(
            PhysicsCollisionListener listener) {
        Validate.nonNull(listener, "listener");
        assert listener != this;
        assert listener != space;
        assert !ongoingListeners.contains(listener);

        ongoingListeners.add(listener);
        this.doProcessed = true;
    }

    /**
     * Count how many collision listeners are registered.
     *
     * @return the count (&ge;0)
     */
    synchronized int countCollisionListeners() {
        int result = ongoingListeners.size() + startedListeners.size();
        return result;
    }

    /**
     * Distribute queued collision events to registered listeners.
     */
    synchronized void distributeEvents() {
        while (!startedEvents.isEmpty()) {
            PhysicsCollisionEvent event = startedEvents.pop();
            for (PhysicsCollisionListener listener : startedListeners) {
                listener.collision(event);
            }
        }

        while (!ongoingEvents.isEmpty()) {
            PhysicsCollisionEvent event = ongoingEvents.pop();
            for (PhysicsCollisionListener listener : ongoingListeners) {
                listener.collision(event);
            }
        }
    }

    /**
     * De-register the specified listener for new contacts.
     *
     * @param listener the listener to de-register (not null)
     */
    synchronized void removeCollisionListener(
            PhysicsCollisionListener listener) {
        Validate.nonNull(listener, "listener");

        boolean success = startedListeners.remove(listener);
        assert success;

        updateFlags();
    }

    /**
     * De-register the specified listener for immediate contact notifications.
     *
     * @param listener the listener to de-register (not null)
     */
    synchronized void removeContactListener(ContactListener listener) {
        Validate.nonNull(listener, "listener");

        int index = immediateListeners.indexOf(listener);
        assert index >= 0 : index;
        immediateListeners.remove(index);
        immediateListenerFlags.remove(index);

        updateFlags();
    }

    /**
     * De-register the specified listener for ongoing contacts.
     *
     * @param listener the listener to de-register (not null)
     */
    synchronized void removeOngoingCollisionListener(
            PhysicsCollisionListener listener) {
        Validate.nonNull(listener, "listener");

        boolean success = ongoingListeners.remove(listener);
        assert success;

        updateFlags();
    }

    /**
     * Update the associated PhysicsSpace. This method should be invoked from
     * the thread that created the space.
     *
     * @param timeInterval the time interval to simulate (in seconds, &ge;0)
     * @param maxSteps the maximum number of steps of size {@code accuracy}
     * (&ge;1) or 0 for a single step of size {@code timeInterval}
     */
    void update(float timeInterval, int maxSteps) {
        assert Validate.nonNegative(timeInterval, "time interval");
        assert Validate.nonNegative(maxSteps, "max steps");

        space.update(timeInterval, maxSteps, doEnded, doProcessed, doStarted);
    }
    // *************************************************************************
    // ContactListener methods

    /**
     * Invoked immediately after a contact manifold is destroyed. Skipped if
     * stepSimulation() was invoked with doEnded=false.
     *
     * @param manifoldId the native ID of the {@code btPersistentManifold} (not
     * zero)
     */
    @Override
    public void onContactEnded(long manifoldId) {
        int numImmediateListeners = immediateListeners.size();
        for (int i = 0; i < numImmediateListeners; ++i) {
            int flags = immediateListenerFlags.get(i);
            if ((flags & invokeEnded) != 0x0) {
                ContactListener listener = immediateListeners.get(i);
                listener.onContactEnded(manifoldId);
            }
        }
    }

    /**
     * Invoked immediately after a contact point is refreshed without being
     * destroyed. Skipped for Sphere-Sphere contacts. Skipped if
     * stepSimulation() was invoked with doProcessed=false.
     *
     * @param pcoA the first involved object (not null)
     * @param pcoB the 2nd involved object (not null)
     * @param pointId the native ID of the {@code btManifoldPoint} (not zero)
     */
    @Override
    public void onContactProcessed(PhysicsCollisionObject pcoA,
            PhysicsCollisionObject pcoB, long pointId) {
        int numImmediateListeners = immediateListeners.size();
        for (int i = 0; i < numImmediateListeners; ++i) {
            int flags = immediateListenerFlags.get(i);
            if ((flags & invokeProcessed) != 0x0) {
                ContactListener listener = immediateListeners.get(i);
                listener.onContactProcessed(pcoA, pcoB, pointId);
            }
        }

        if (!ongoingListeners.isEmpty()) {
            PhysicsCollisionEvent event
                    = new PhysicsCollisionEvent(pcoA, pcoB, pointId);

            // Queue the event to be handled later by distributeEvents().
            ongoingEvents.add(event);
        }
    }

    /**
     * Invoked immediately after a contact manifold is created. Skipped if
     * stepSimulation() was invoked with doStarted=false.
     *
     * @param manifoldId the native ID of the {@code btPersistentManifold} (not
     * zero)
     */
    @Override
    public void onContactStarted(long manifoldId) {
        int numImmediateListeners = immediateListeners.size();
        for (int i = 0; i < numImmediateListeners; ++i) {
            int flags = immediateListenerFlags.get(i);
            if ((flags & invokeStarted) != 0x0) {
                ContactListener listener = immediateListeners.get(i);
                listener.onContactStarted(manifoldId);
            }
        }

        if (startedListeners.isEmpty()) {
            return;
        }
        int numPoints = PersistentManifolds.countPoints(manifoldId);
        if (numPoints == 0) {
            return;
        }

        long bodyAId = PersistentManifolds.getBodyAId(manifoldId);
        PhysicsCollisionObject pcoA
                = PhysicsCollisionObject.findInstance(bodyAId);
        long bodyBId = PersistentManifolds.getBodyBId(manifoldId);
        PhysicsCollisionObject pcoB
                = PhysicsCollisionObject.findInstance(bodyBId);

        for (int i = 0; i < numPoints; ++i) {
            long pointId = PersistentManifolds.getPointId(manifoldId, i);
            PhysicsCollisionEvent event
                    = new PhysicsCollisionEvent(pcoA, pcoB, pointId);

            // Queue the event to be handled later by distributeEvents().
            startedEvents.add(event);
        }
    }
    // *************************************************************************
    // new private methods

    /**
     * Update the doEnded, doProcessed, and doStarted flags after a listener is
     * removed.
     */
    private void updateFlags() {
        int union = 0x0;
        for (int flags : immediateListenerFlags) {
            union |= flags;
        }

        this.doEnded = ((union & invokeEnded) != 0x0);
        this.doProcessed = ((union & invokeProcessed) != 0x0)
                || !ongoingListeners.isEmpty();
        this.doStarted = ((union & invokeStarted) != 0x0)
                || !startedListeners.isEmpty();
    }
}
