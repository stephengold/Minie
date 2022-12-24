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

import java.util.logging.Logger;

/**
 * Utility class to access fields of Bullet's {@code btPersistentManifold}
 * class.
 */
final public class PersistentManifolds {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(PersistentManifolds.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private PersistentManifolds() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Return the number of points in the specified manifold (native field:
     * m_cachedPoints).
     *
     * @param persistentManifoldId the native ID of a
     * {@code btPersistentManifold} (not zero)
     * @return the number of btManifoldPoints in the
     * {@code btPersistentManifold} (&ge;0, &le;4)
     */
    native public static int countPoints(long persistentManifoldId);

    /**
     * Return the native ID of the first collision object (native field:
     * m_body0).
     *
     * @param persistentManifoldId the native ID of a
     * {@code btPersistentManifold} (not zero)
     * @return the native ID of the btCollisionObject (not zero)
     * @see PhysicsCollisionObject#findInstance(long)
     */
    native public static long getBodyAId(long persistentManifoldId);

    /**
     * Return the native ID of the 2nd collision object (native field: m_body1).
     *
     * @param persistentManifoldId the native ID of a
     * {@code btPersistentManifold} (not zero)
     * @return the native ID of the btCollisionObject (not zero)
     * @see PhysicsCollisionObject#findInstance(long)
     */
    native public static long getBodyBId(long persistentManifoldId);

    /**
     * Return the indexed btManifoldPoint in the specified manifold.
     *
     * @param persistentManifoldId the native ID of a
     * {@code btPersistentManifold} (not zero)
     * @param pointIndex the index of the point (&ge;0, &lt;4)
     * @return the native ID of the btManifoldPoint (not zero)
     */
    native public static long
            getPointId(long persistentManifoldId, int pointIndex);

    /**
     * Enumerate the native IDs of all points in the specified manifold.
     *
     * @param persistentManifoldId the native ID of a
     * {@code btPersistentManifold} (not zero)
     * @return a new array of btManifoldPoint IDs (not null, may be empty)
     * @see com.jme3.bullet.collision.ManifoldPoints
     */
    public static long[] listPointIds(long persistentManifoldId) {
        int numPoints = countPoints(persistentManifoldId);
        long[] result = new long[numPoints];

        for (int pointIndex = 0; pointIndex < numPoints; ++pointIndex) {
            long manifoldId = getPointId(persistentManifoldId, pointIndex);
            result[pointIndex] = manifoldId;
        }

        return result;
    }
}
