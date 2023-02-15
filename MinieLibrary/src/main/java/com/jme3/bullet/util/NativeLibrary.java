/*
 * Copyright (c) 2019-2023 jMonkeyEngine
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
package com.jme3.bullet.util;

import com.jme3.bullet.NativePhysicsObject;
import com.jme3.math.Vector3f;
import java.util.logging.Logger;

/**
 * Static interface to the Libbulletjme native library.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class NativeLibrary {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(NativeLibrary.class.getName());
    /**
     * expected version string of the native library
     */
    final public static String expectedVersion = "18.1.0";
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private NativeLibrary() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Count the cumulative number of clamped CCD motions (native variable:
     * gNumClampedCcdMotions).
     * <p>
     * For debugging. The value shouldn't grow too large.
     *
     * @return the count (&ge;0)
     */
    native public static int countClampedCcdMotions();

    /**
     * Count how many threads are available for task scheduling.
     *
     * @return the count (&ge;0)
     */
    native public static int countThreads();

    /**
     * Crash the JVM with an EXCEPTION_ACCESS_VIOLATION or SIGILL. Intended for
     * testing only!
     */
    native public static void crash();

    /**
     * Dump all native-memory allocation/free events to standard output. This
     * feature is enabled only in native libraries built with the
     * BT_DEBUG_MEMORY_ALLOCATIONS macro defined.
     *
     * @return the number of bytes outstanding (&ge;0), or -1 if this feature is
     * not enabled
     */
    native public static int dumpMemoryLeaks();

    /**
     * Dump all Quickprof data to standard output. This feature is enabled only
     * in native libraries built with the BT_ENABLE_PROFILE macro defined. Must
     * be invoked on the designated physics thread.
     *
     * @return the number of frames profiled since the last reset (&ge;0), or -1
     * if this feature is not enabled
     */
    native public static int dumpQuickprof();

    /**
     * Execute btAssert(0). This has no effect on Release builds, but if the
     * native library was built with debugging enabled, it should terminate the
     * Java Virtual Machine. Intended for testing only!
     */
    native public static void fail();

    /**
     * Test whether the native library was built with debugging enabled.
     *
     * @return true if Debug buildType, false if Release buildType
     */
    native public static boolean isDebug();

    /**
     * Test whether the native library uses double-precision arithmetic.
     *
     * @return true if double-precision, false if single-precision
     */
    native public static boolean isDoublePrecision();

    /**
     * Test whether the specified point and triangle are within the specified
     * distance of each other. Used for testing Bullet's
     * btTriangleShape::isInside().
     *
     * @param testPoint the location of the test point (not null, unaffected)
     * @param maxSeparation the maximum separation allowed
     * @param v0 the first vertex of the triangle (not null, unaffected)
     * @param v1 the 2nd vertex of the triangle (not null, unaffected)
     * @param v2 the 3rd vertex of the triangle (not null, unaffected)
     * @return true if within the specified distance, otherwise false
     */
    native public static boolean isInsideTriangle(Vector3f testPoint,
            float maxSeparation, Vector3f v0, Vector3f v1, Vector3f v2);

    /**
     * Test whether the native library includes Quickprof profiling.
     *
     * @return true if included, otherwise false
     */
    native public static boolean isQuickprof();

    /**
     * Test whether the native library was built thread-safe.
     *
     * @return true if thread-safe, otherwise false
     */
    native public static boolean isThreadSafe();

    /**
     * Return the address of the current thread's JNIEnv. For debugging and
     * testing.
     *
     * @return the virtual address of the (native) object (not zero)
     */
    native public static long jniEnvId();

    /**
     * Reset Quickprof. This feature is enabled only in native libraries built
     * with the BT_ENABLE_PROFILE macro defined. Must be invoked on the
     * designated physics thread.
     */
    native public static void resetQuickprof();

    /**
     * Alter whether the native library should invoke the reinitialization()
     * callback.
     *
     * @param callbackFlag true &rarr; invoke, false &rarr; don't invoke
     * (default=false)
     */
    native public static void
            setReinitializationCallbackEnabled(boolean callbackFlag);

    /**
     * Alter whether the native library will print its startup message during
     * initialization.
     *
     * @param printFlag true &rarr; print message, false &rarr; no message
     * (default=true)
     */
    native public static void setStartupMessageEnabled(boolean printFlag);

    /**
     * Determine the native library's version string.
     *
     * @return the version string (typically of the form Major.Minor.Patch)
     */
    native public static String versionNumber();
    // *************************************************************************
    // Java private methods

    /**
     * Callback invoked (by native code) upon successful initialization of the
     * native library, to start the Physics Cleaner thread.
     */
    private static void postInitialization() {
        String lbjVersion = versionNumber();
        if (!lbjVersion.equals(expectedVersion)) {
            logger.warning("Expected a v" + expectedVersion
                    + " native library but loaded v" + lbjVersion + "!");
        }

        Thread physicsCleaner = new Thread("Physics Cleaner") {
            @Override
            public void run() {
                NativePhysicsObject.freeUnusedObjects();
            }
        };
        physicsCleaner.setDaemon(true);
        physicsCleaner.start();
    }

    /**
     * Callback invoked (by native code) for each attempt to re-initialize the
     * native library while setReinitializationCallbackEnabled(true).
     */
    private static void reinitialization() {
        // do nothing, for now
    }
}
