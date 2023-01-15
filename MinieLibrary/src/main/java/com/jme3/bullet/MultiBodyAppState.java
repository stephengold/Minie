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

import com.jme3.bullet.debug.BulletDebugAppState;
import com.jme3.bullet.debug.DebugConfiguration;
import com.jme3.bullet.debug.MultiBodyDebugAppState;
import com.jme3.math.Vector3f;
import java.util.logging.Logger;

/**
 * An AppState to manage a single MultiBodySpace.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class MultiBodyAppState extends BulletAppState {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(MultiBodyAppState.class.getName());
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled AppState to manage a space with DBVT broadphase
     * collision detection.
     */
    public MultiBodyAppState() { // to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Access the space managed by this state. Normally there is none until the
     * state is attached.
     *
     * @return the pre-existing instance, or null if no simulation running
     */
    public MultiBodySpace getMultiBodySpace() {
        MultiBodySpace pSpace = (MultiBodySpace) getPhysicsSpace();
        return pSpace;
    }
    // *************************************************************************
    // BulletAppState methods

    /**
     * Create the configured debug-visualization app state.
     *
     * @return a new instance (not null)
     */
    @Override
    protected BulletDebugAppState createDebugAppState() {
        DebugConfiguration debugConfig = getDebugConfiguration();
        BulletDebugAppState appState = new MultiBodyDebugAppState(debugConfig);

        return appState;
    }

    /**
     * Create the configured MultiBodySpace.
     *
     * @param min the minimum coordinate value for each axis (not null,
     * unaffected)
     * @param max the maximum coordinate value for each axis (not null,
     * unaffected)
     * @param broadphaseType which broadphase accelerator to use (not null)
     * @return a new instance (not null)
     */
    @Override
    protected PhysicsSpace createPhysicsSpace(Vector3f min, Vector3f max,
            PhysicsSpace.BroadphaseType broadphaseType) {
        SolverType solverType = getSolverType();
        PhysicsSpace result
                = new MultiBodySpace(min, max, broadphaseType, solverType);

        return result;
    }
}
