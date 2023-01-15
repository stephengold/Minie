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
import com.jme3.bullet.debug.DebugConfiguration;
import com.jme3.bullet.debug.SoftDebugAppState;
import com.jme3.math.Vector3f;
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
    // fields

    /**
     * limit which clusters are visualized, or null to visualize no clusters
     */
    private BulletDebugAppState.DebugAppStateFilter clusterFilter;
    /**
     * limit which wind velocities are visualized, or null to visualize none
     */
    private BulletDebugAppState.DebugAppStateFilter windVelocityFilter;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled app state with no filters to manage a space with
     * the DBVT broadphase collision-detection algorithm.
     */
    public SoftPhysicsAppState() { // to avoid a warning from JDK 18 javadoc
    }
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

    /**
     * Alter which soft-body clusters are included in the debug visualization.
     *
     * @param filter the desired filter, or null to visualize no clusters
     */
    public void setDebugClusterFilter(
            BulletDebugAppState.DebugAppStateFilter filter) {
        SoftDebugAppState sdas = (SoftDebugAppState) getDebugAppState();
        if (sdas != null) {
            sdas.setClusterFilter(filter);
        }
        clusterFilter = filter;
    }

    /**
     * Alter which wind velocities are included in the debug visualization.
     *
     * @param filter the filter to use (alias created) or null to visualize no
     * wind velocities (default=null)
     */
    public void setWindVelocityFilter(
            BulletDebugAppState.DebugAppStateFilter filter) {
        SoftDebugAppState sdas = (SoftDebugAppState) getDebugAppState();
        if (sdas != null) {
            sdas.setWindVelocityFilter(filter);
        }
        this.windVelocityFilter = filter;
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
        SoftDebugAppState appState = new SoftDebugAppState(debugConfig);
        appState.setClusterFilter(clusterFilter);
        appState.setWindVelocityFilter(windVelocityFilter);

        return appState;
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
        PhysicsSpace result = new PhysicsSoftSpace(min, max, type);
        return result;
    }
}
