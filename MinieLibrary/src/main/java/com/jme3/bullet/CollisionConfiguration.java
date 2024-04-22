/*
 * Copyright (c) 2024 jMonkeyEngine
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
 * Tuning parameters for a CollisionSpace, based on Bullet's
 * btDefaultCollisionConstructionInfo. Immutable.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class CollisionConfiguration extends NativePhysicsObject {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(CollisionConfiguration.class.getName());
    // *************************************************************************
    // constructors

    /**
     * Instantiate an instance with the default parameter values.
     */
    public CollisionConfiguration() {
        this(4_096, 1);
    }

    /**
     * Instantiate an instance with the specified parameter values.
     *
     * @param maxManifolds the desired size of the persistent-manifold pool
     * (&gt;0, default=4096)
     * @param penetrationDepthSolver 0 for
     * {@code btMinkowskiPenetrationDepthSolver} or 1 for
     * {@code btGjkEpaPenetrationDepthSolver} (default=1)
     */
    public CollisionConfiguration(
            int maxManifolds, int penetrationDepthSolver) {
        Validate.positive(maxManifolds, "max manifolds");
        Validate.inRange(
                penetrationDepthSolver, "penetration depth solver", 0, 1);

        long configurationId
                = createNative(maxManifolds, penetrationDepthSolver);
        super.setNativeId(configurationId);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Return the size of the persistent-manifold pool. (native field:
     * m_defaultMaxPersistentManifoldPoolSize)
     *
     * @return the count (&gt;0)
     */
    public int maxManifolds() {
        long configurationId = nativeId();
        int result = maxManifolds(configurationId);
        return result;
    }

    /**
     * Return which penetration-depth solver is used. (native field:
     * m_useEpaPenetrationAlgorithm)
     *
     * @return 0 for {@code btMinkowskiPenetrationDepthSolver} or 1 for
     * {@code btGjkEpaPenetrationDepthSolver}
     */
    public int penetrationDepthSolver() {
        long configurationId = nativeId();
        int result = penetrationDepthSolver(configurationId);
        return result;
    }
    // *************************************************************************
    // Java private methods

    /**
     * Free the identified tracked native object. Invoked by reflection.
     *
     * @param configurationId the native identifier (not zero)
     */
    private static void freeNativeObject(long configurationId) {
        assert configurationId != 0L;
        finalizeNative(configurationId);
    }
    // *************************************************************************
    // native private methods

    native private static long createNative(
            int maxManifolds, int penetrationDepthSolver);

    native private static void finalizeNative(long configurationId);

    native private static int maxManifolds(long configurationId);

    native private static int penetrationDepthSolver(long configurationId);
}
