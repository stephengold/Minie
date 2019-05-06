/*
 * Copyright (c) 2009-2015 jMonkeyEngine
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
import com.jme3.bullet.debug.DebugInitListener;
import com.jme3.bullet.debug.SoftDebugAppState;
import com.jme3.math.Vector3f;
import com.jme3.renderer.ViewPort;
import java.util.logging.Logger;

/**
 * An AppState to manage a single PhysicsSoftSpace.
 *
 * @author Stephen Gold sgold@sonic.net
 *
 * Based on BulletSoftBodyAppState by dokthar.
 */
public class SoftPhysicsAppState extends BulletAppState {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(SoftPhysicsAppState.class.getName());
    // *************************************************************************
    // new methods exposed

    /**
     * Access the PhysicsSoftSpace managed by this state. Normally there is none
     * until the state is attached.
     *
     * @return the pre-existing instance, or null if no simulation running
     */
    public PhysicsSoftSpace getPhysicsSoftSpace() {
        PhysicsSoftSpace pSpace = (PhysicsSoftSpace) getPhysicsSpace();
        return pSpace;
    }
    // *************************************************************************
    // BulletAppState methods

    /**
     * Create the configured debug app state.
     *
     * @param space the PhysicsSpace (not null, alias created)
     * @param viewPorts the view ports in which to render (not null)
     * @param filter the display filter, or null for none
     * @param listener the initialization listener, or null for none
     * @return a new instance (not null)
     */
    @Override
    protected BulletDebugAppState createDebugAppState(PhysicsSpace space,
            ViewPort[] viewPorts,
            BulletDebugAppState.DebugAppStateFilter filter,
            DebugInitListener listener) {
        PhysicsSoftSpace softSpace = getPhysicsSoftSpace();
        BulletDebugAppState result = new SoftDebugAppState(softSpace,
                viewPorts, filter, listener);

        return result;
    }

    /**
     * Create the configured PhysicsSpace.
     *
     * @param min the minimum coordinate values (not null, unaffected)
     * @param max the maximum coordinate values (not null, unaffected)
     * @param type the broadphase collision-detection algorithm (not null)
     * @return a new instance (not null)
     */
    @Override
    protected PhysicsSpace createPhysicsSpace(Vector3f min, Vector3f max,
            PhysicsSpace.BroadphaseType type) {
        return new PhysicsSoftSpace(min, max, type);
    }
}
