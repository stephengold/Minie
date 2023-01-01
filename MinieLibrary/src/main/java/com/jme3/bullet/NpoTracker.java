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
package com.jme3.bullet;

import java.lang.ref.WeakReference;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.logging.Logger;

/**
 * Metadata used to track and free a NativePhysicsObject. Immutable.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class NpoTracker extends WeakReference<NativePhysicsObject> {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(NpoTracker.class.getName());
    // *************************************************************************
    // fields

    /**
     * type of the referent (not null)
     */
    final private Class<? extends NativePhysicsObject> referentClass;
    /**
     * identifier of the referent's native object (not zero)
     */
    final private long id;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a tracker for the specified referent.
     *
     * @param referent (must have an assigned native object)
     */
    NpoTracker(NativePhysicsObject referent) {
        super(referent, NativePhysicsObject.weakReferenceQueue);

        this.referentClass = referent.getClass();
        this.id = referent.nativeId();
        assert id != 0L;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Free the tracked native object by attempting to invoke
     * freeNativeObject(id) on its class and superclasses thereof.
     */
    void freeTrackedObject() {
        int invocationCount = 0;

        // Remove this tracker from the map BEFORE freeing the native object.
        NativePhysicsObject.removeTracker(id);
        Method[] methods = FreeingMethods.listMethods(referentClass);
        // Avoid re-boxing in case more than one method is invoked.
        Object[] boxedId = {id};
        for (Method method : methods) {
            try {
                method.invoke(null, boxedId);
                ++invocationCount;
            } catch (IllegalAccessException | IllegalArgumentException
                   | InvocationTargetException exception) {
                throw new RuntimeException(exception);
            }
        }

        assert invocationCount > 0 : invocationCount;
    }
    // *************************************************************************
    // Object methods

    /**
     * Represent this tracker as a String.
     *
     * @return a descriptive string of text (not null, not empty)
     */
    @Override
    public String toString() {
        String result = referentClass.getSimpleName();
        result += "_" + Long.toHexString(id);

        return result;
    }
}
