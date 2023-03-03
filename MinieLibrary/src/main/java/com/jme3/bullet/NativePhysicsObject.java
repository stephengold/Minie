/*
 * Copyright (c) 2020-2023 jMonkeyEngine
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

import java.lang.ref.ReferenceQueue;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * An abstract class to represent a native (Bullet) physics object.
 *
 * @author Stephen Gold sgold@sonic.net
 */
abstract public class NativePhysicsObject
        implements Comparable<NativePhysicsObject> {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger loggerN
            = Logger.getLogger(NativePhysicsObject.class.getName());
    // *************************************************************************
    // fields

    /**
     * identifier (64-bit address) of the assigned native object, or zero if
     * none
     */
    private long id = 0L;
    /**
     * map native IDs to their trackers
     */
    final private static Map<Long, NpoTracker> map
            = new ConcurrentHashMap<>(999);
    /**
     * weak references to all instances whose assigned native objects are
     * tracked and known to be unused
     */
    final static ReferenceQueue<NativePhysicsObject> weakReferenceQueue
            = new ReferenceQueue<>();
    // *************************************************************************
    // constructors

    /**
     * Instantiate with no tracker and no assigned native object.
     */
    protected NativePhysicsObject() { // to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Count how many native objects are being tracked. This method is intended
     * for debugging.
     *
     * @return the count (&ge;0)
     */
    final public static int countTrackers() {
        int result = map.size();
        return result;
    }

    /**
     * Dump all native-object trackers to {@code System.out}. This method is
     * intended for debugging.
     */
    final public static void dumpTrackers() {
        System.out.println("Active trackers:");
        for (NpoTracker tracker : map.values()) {
            System.out.println(" " + tracker);
        }
        System.out.flush();
    }

    /**
     * Free any assigned native objects that are known to be unused.
     */
    final public static void freeUnusedObjects() {
        while (true) {
            try {
                NpoTracker tracker = (NpoTracker) weakReferenceQueue.remove();
                tracker.freeTrackedObject();
            } catch (InterruptedException exception) {
                break;
            }
        }
    }

    /**
     * Test whether a native object is assigned to this instance.
     *
     * @return true if one is assigned, otherwise false
     */
    final public boolean hasAssignedNativeObject() {
        if (id == 0L) {
            return false;
        } else {
            return true;
        }
    }

    /**
     * Return the ID of the assigned native object, assuming that one is
     * assigned.
     *
     * @return the native identifier (not zero)
     */
    public long nativeId() {
        assert hasAssignedNativeObject();
        return id;
    }

    /**
     * Remove the identified tracker from the map.
     *
     * @param nativeId the native identifier (not zero)
     */
    static void removeTracker(long nativeId) {
        assert nativeId != 0L;

        NpoTracker tracker = map.remove(nativeId);
        assert tracker != null;
    }
    // *************************************************************************
    // new protected methods

    /**
     * Assign a tracked native object to this instance, unassigning (but not
     * freeing) any previously assigned one. Typically invoked while cloning a
     * subclass. Typically invoked while cloning a subclass.
     *
     * @param nativeId the identifier of the native object to assign (not zero)
     */
    final protected void reassignNativeId(long nativeId) {
        Validate.nonZero(nativeId, "nativeId");

        if (nativeId != id) {
            this.id = nativeId;
            NpoTracker tracker = new NpoTracker(this);
            NpoTracker previous = map.put(nativeId, tracker);
            assert previous == null : Long.toHexString(id);
        }
    }

    /**
     * Assign a tracked native object to this instance, assuming that no native
     * object is assigned.
     *
     * @param nativeId the identifier of the native object to assign (not zero)
     */
    protected void setNativeId(long nativeId) {
        Validate.nonZero(nativeId, "nativeId");
        assert !hasAssignedNativeObject() : Long.toHexString(id);

        this.id = nativeId;
        NpoTracker tracker = new NpoTracker(this);
        NpoTracker previous = map.put(nativeId, tracker);
        assert previous == null : Long.toHexString(id);
    }

    /**
     * Assign an untracked native object to this instance, assuming that no
     * native object is assigned.
     *
     * @param nativeId the identifier of the native object to assign (not zero)
     */
    final protected void setNativeIdNotTracked(long nativeId) {
        Validate.nonZero(nativeId, "nativeId");
        assert !hasAssignedNativeObject() : Long.toHexString(id);

        this.id = nativeId;
    }

    /**
     * Unassign (but don't free) the assigned native object, assuming that one
     * is assigned. Typically invoked while cloning, destroying, or rebuilding a
     * subclass.
     */
    final protected void unassignNativeObject() {
        assert hasAssignedNativeObject();
        this.id = 0L;
    }
    // *************************************************************************
    // Comparable methods

    /**
     * Compare (by ID) with another native object.
     *
     * @param other (not null, unaffected)
     * @return 0 if the objects have the same native ID; negative if this comes
     * before other; positive if this comes after other
     */
    @Override
    public int compareTo(NativePhysicsObject other) {
        long objectId = nativeId();
        long otherId = other.nativeId();
        int result = Long.compare(objectId, otherId);

        return result;
    }
    // *************************************************************************
    // Object methods

    /**
     * Test for ID equality with another object.
     *
     * @param otherObject the object to compare (may be null, unaffected)
     * @return true if {@code this} and {@code otherObject} have the same ID,
     * otherwise false
     */
    @Override
    public boolean equals(Object otherObject) {
        boolean result;
        if (otherObject == this) {
            result = true;
        } else if (otherObject != null
                && otherObject.getClass() == getClass()) {
            NativePhysicsObject otherNpo = (NativePhysicsObject) otherObject;
            long otherId = otherNpo.nativeId();
            result = (id == otherId);
        } else {
            result = false;
        }

        return result;
    }

    /**
     * Return the hash code for this instance.
     * <p>
     * Note: operations that alter the native ID are likely to affect the hash
     * code as well!
     *
     * @return a 32-bit value for use in hashing
     */
    @Override
    public int hashCode() {
        int hash = (int) (id >> 4);
        return hash;
    }

    /**
     * Represent this instance as a String.
     *
     * @return a descriptive string of text (not null, not empty)
     */
    @Override
    public String toString() {
        String result = getClass().getSimpleName();
        result += "#" + Long.toHexString(id);

        return result;
    }
}
