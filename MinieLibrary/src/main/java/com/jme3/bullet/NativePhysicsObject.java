/*
 * Copyright (c) 2020 jMonkeyEngine
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

import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * An abstract class to represent a native (Bullet) physics object.
 */
abstract public class NativePhysicsObject {
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
    // *************************************************************************
    // new methods exposed

    /**
     * Test whether a native object is assigned.
     *
     * @return true if one is assigned, otherwise false
     */
    final public boolean hasAssignedNativeObject() {
        if (id == 0) {
            return false;
        } else {
            return true;
        }
    }

    /**
     * Read the identifier of the assigned native object, assuming that one is
     * assigned.
     *
     * @return the identifier (not zero)
     */
    public long nativeId() {
        assert hasAssignedNativeObject();
        return id;
    }
    // *************************************************************************
    // new protected methods

    /**
     * Reassign a native object, unassigning any previous-assigned one.
     * Typically invoked when cloning a subclass.
     *
     * @param nativeId the identifier (address) of the native object to assign
     * (not zero)
     */
    protected void reassignNativeId(long nativeId) {
        Validate.nonZero(nativeId, "nativeId");
        id = nativeId;
    }

    /**
     * Assign a native object, assuming that none is assigned. Typically invoked
     * when instantiating a subclass.
     *
     * @param nativeId the identifier (address) of the native object to assign
     * (not zero)
     */
    protected void setNativeId(long nativeId) {
        Validate.nonZero(nativeId, "nativeId");
        assert !hasAssignedNativeObject() : id;

        id = nativeId;
    }

    /**
     * Unassign the native object, assuming that one is assigned.
     */
    protected void unassignNativeObject() {
        assert hasAssignedNativeObject();
        id = 0L;
    }
    // *************************************************************************
    // Object methods

    /**
     * Test for ID equality with another object.
     *
     * @param otherObject the object to compare to (may be null, unaffected)
     * @return true if the objects have the same ID, otherwise false
     */
    @Override
    public boolean equals(Object otherObject) {
        boolean result;
        if (otherObject == this) {
            result = true;
        } else if (otherObject != null
                && otherObject.getClass() == getClass()) {
            NativePhysicsObject otherJoint = (NativePhysicsObject) otherObject;
            long otherId = otherJoint.nativeId();
            result = (id == otherId);
        } else {
            result = false;
        }

        return result;
    }

    /**
     * Generate the hash code for this instance.
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
