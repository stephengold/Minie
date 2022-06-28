/*
 * Copyright (c) 2021-2022 jMonkeyEngine
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

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Logger;

/**
 * Cache the methods used (by the physics cleaner thread) to free each type of
 * NativePhysicsObject. This is measurably faster than traversing the class
 * hierarchy for every instance.
 */
final class FreeingMethods {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(FreeingMethods.class.getName());
    // *************************************************************************
    // fields

    /**
     * map classes to methods - initialized lazily
     */
    final private static Map<Class<? extends NativePhysicsObject>, Method[]> map
            = new ConcurrentHashMap<>(30);
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private FreeingMethods() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Enumerate the methods used to free an instance of the specified class.
     *
     * @param clazz which class (not null)
     * @return an internal array (not null, do not modify!)
     */
    static Method[] listMethods(Class<? extends NativePhysicsObject> clazz) {
        Method[] result = map.get(clazz);
        if (result == null) {
            result = generate(clazz);
            map.put(clazz, result);
        }

        return result;
    }
    // *************************************************************************
    // private methods

    /**
     * Generate the map entry for the specified class.
     *
     * @param clazz the class to be freed (not null)
     * @return a new array
     */
    private static Method[] generate(
            Class<? extends NativePhysicsObject> clazz) {
        Collection<Method> methods = new ArrayList<>(4);
        for (Class<?> c = clazz; c != Object.class; c = c.getSuperclass()) {
            try {
                Method method
                        = c.getDeclaredMethod("freeNativeObject", long.class);
                method.setAccessible(true);
                methods.add(method);
            } catch (IllegalArgumentException
                    | NoClassDefFoundError
                    | SecurityException exception) {
                System.out.println("c = " + c.getName());
                throw new RuntimeException(exception);
            } catch (NoSuchMethodException exception) {
                // do nothing
            }
        }

        int numMethods = methods.size();
        Method[] result = new Method[numMethods];
        methods.toArray(result);

        return result;
    }
}
