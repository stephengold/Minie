/*
 * Copyright (c) 2009-2018 jMonkeyEngine
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

import com.jme3.app.Application;
import com.jme3.app.state.AppState;
import com.jme3.app.state.AppStateManager;
import com.jme3.bullet.PhysicsSpace.BroadphaseType;
import com.jme3.bullet.debug.BulletDebugAppState;
import com.jme3.bullet.debug.DebugInitListener;
import com.jme3.math.Vector3f;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * An app state to manage a single Bullet physics space.
 *
 * @author normenhansen
 */
public class BulletAppState
        implements AppState, PhysicsTickListener {
    // *************************************************************************
    // classes and enums

    /**
     * Enumerate threading modes.
     */
    public enum ThreadingType {
        /**
         * Default mode: user update, physics update, and rendering happen
         * sequentially. (single-threaded)
         */
        SEQUENTIAL,
        /**
         * Parallel threaded mode: physics update and rendering are executed in
         * parallel, update order is maintained.
         */
        PARALLEL
    }
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(BulletAppState.class.getName());
    // *************************************************************************
    // fields

    /**
     * manager that manages this state, set during attach
     */
    private AppStateManager stateManager;
    /**
     * true if-and-only-if debug visualization is enabled
     */
    private boolean debugEnabled = false;
    /**
     * true if-and-only-if this state is enabled
     */
    private boolean isEnabled = true;
    /**
     * true if-and-only-if the physics simulation is running (started but not
     * yet stopped)
     */
    private boolean isRunning = false;
    /**
     * broadphase collision-detection algorithm for the physics space to use
     * (not null)
     */
    private BroadphaseType broadphaseType = BroadphaseType.DBVT;
    /**
     * app state to manage the debug visualization, or null if none
     */
    private BulletDebugAppState debugAppState;
    /**
     * filter to limit which objects are visualized in the debug visualization,
     * or null to visualize all objects
     */
    private BulletDebugAppState.DebugAppStateFilter filter = null;

    private Callable<Boolean> detachedPhysicsUpdate = new Callable<Boolean>() {
        @Override
        public Boolean call() throws Exception {
            pSpace.update(getPhysicsSpace().getAccuracy() * getSpeed());
            pSpace.distributeEvents();
            long update = System.currentTimeMillis() - detachedPhysicsLastUpdate;
            detachedPhysicsLastUpdate = System.currentTimeMillis();
            executor.schedule(detachedPhysicsUpdate,
                    Math.round(getPhysicsSpace().getAccuracy() * 1_000_000.0f) - (update * 1000),
                    TimeUnit.MICROSECONDS);

            return true;
        }
    };
    final private Callable<Boolean> parallelPhysicsUpdate
            = new Callable<Boolean>() {
        @Override
        public Boolean call() throws Exception {
            pSpace.update(tpf * getSpeed());
            return true;
        }
    };
    /**
     * registered debug init listener, or null if none
     */
    private DebugInitListener debugInitListener = null;
    /**
     * simulation speed multiplier (default=1, paused=0)
     */
    private float speed = 1f;
    /**
     * time interval between frames (in seconds) from the most recent update
     */
    private float tpf;
    /**
     * current physics task, or null if none
     */
    private Future physicsFuture;

    private long detachedPhysicsLastUpdate = 0L;
    /**
     * physics space managed by this state, or null if no simulation running
     */
    private PhysicsSpace pSpace;
    /**
     * executor service for physics tasks, or null if parallel simulation is not
     * running
     */
    private ScheduledThreadPoolExecutor executor;
    /**
     * threading mode to use (not null)
     */
    private ThreadingType threadingType = ThreadingType.SEQUENTIAL;
    /**
     * maximum coordinate values for the physics space when using AXIS_SWEEP
     * broadphase algorithms (not null)
     */
    final private Vector3f worldMax = new Vector3f(10000f, 10000f, 10000f);
    /**
     * minimum coordinate values for the physics space when using AXIS_SWEEP
     * broadphase algorithms (not null)
     */
    final private Vector3f worldMin = new Vector3f(-10000f, -10000f, -10000f);
    /**
     * view ports in which to render the debug visualization
     */
    private ViewPort[] debugViewPorts = null;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an app state to manage a new PhysicsSpace with DBVT collision
     * detection.
     * <p>
     * Use getStateManager().addState(bulletAppState) to start physics.
     */
    public BulletAppState() {
    }

    /**
     * Instantiate an app state to manage a new PhysicsSpace.
     * <p>
     * Use getStateManager().addState(bulletAppState) to start physics.
     *
     * @param broadphaseType which broadphase collision-detection algorithm to
     * use (not null)
     */
    public BulletAppState(BroadphaseType broadphaseType) {
        this(new Vector3f(-10000f, -10000f, -10000f),
                new Vector3f(10000f, 10000f, 10000f),
                broadphaseType);
    }

    /**
     * Instantiate an app state to manage a new PhysicsSpace with AXIS_SWEEP_3
     * collision detection.
     * <p>
     * Use getStateManager().addState(bulletAppState) to start physics.
     *
     * @param worldMin the desired minimum coordinate values (not null,
     * unaffected, default=-10k,-10k,-10k)
     * @param worldMax the desired maximum coordinate values (not null,
     * unaffected, default=10k,10k,10k)
     */
    public BulletAppState(Vector3f worldMin, Vector3f worldMax) {
        this(worldMin, worldMax, BroadphaseType.AXIS_SWEEP_3);
    }

    /**
     * Instantiate an app state to manage a new PhysicsSpace.
     * <p>
     * Use getStateManager().addState(bulletAppState) to enable physics.
     *
     * @param worldMin the desired minimum coordinate values (not null,
     * unaffected, default=-10k,-10k,-10k)
     * @param worldMax the desired maximum coordinate values (not null,
     * unaffected, default=10k,10k,10k)
     * @param broadphaseType which broadphase collision-detection algorithm to
     * use (not null)
     */
    public BulletAppState(Vector3f worldMin, Vector3f worldMax,
            BroadphaseType broadphaseType) {
        Validate.nonNull(worldMin, "world min");
        Validate.nonNull(worldMax, "world max");
        Validate.nonNull(broadphaseType, "broadphase type");

        this.worldMin.set(worldMin);
        this.worldMax.set(worldMax);
        this.broadphaseType = broadphaseType;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Read which broadphase collision-detection algorithm the physics space
     * will use.
     *
     * @return enum value (not null)
     */
    public BroadphaseType getBroadphaseType() {
        return broadphaseType;
    }

    /**
     * Access the PhysicsSpace managed by this state. Normally there is none
     * until the state is attached.
     *
     * @return the pre-existing instance, or null if no simulation running
     */
    public PhysicsSpace getPhysicsSpace() {
        return pSpace;
    }

    /**
     * Read the simulation speed.
     *
     * @return speed (&ge;0, default=1)
     */
    public float getSpeed() {
        return speed;
    }

    /**
     * Read which type of threading this app state uses.
     *
     * @return the threadingType (not null)
     */
    public ThreadingType getThreadingType() {
        return threadingType;
    }

    /**
     * Test whether debug visualization is enabled.
     *
     * @return true if enabled, otherwise false
     */
    public boolean isDebugEnabled() {
        return debugEnabled;
    }

    /**
     * Alter the broadphase type the physics space will use. Not allowed after
     * attaching the app state.
     *
     * @param broadphaseType an enum value (not null, default=DBVT)
     */
    public void setBroadphaseType(BroadphaseType broadphaseType) {
        Validate.nonNull(broadphaseType, "broadphase type");
        assert !isRunning;

        this.broadphaseType = broadphaseType;
    }

    /**
     * Alter whether debug visualization is enabled. Changes take effect on the
     * next update.
     *
     * @param debugEnabled true &rarr; enable, false &rarr; disable
     */
    public void setDebugEnabled(boolean debugEnabled) {
        this.debugEnabled = debugEnabled;
    }

    /**
     * Alter which objects are included in the debug visualization.
     *
     * @param filter the desired filter, or null to visualize all objects
     */
    public void setDebugFilter(BulletDebugAppState.DebugAppStateFilter filter) {
        if (debugAppState != null) {
            debugAppState.setFilter(filter);
        }
        this.filter = filter;
    }

    /**
     * Register the init listener for the BulletDebugAppState.
     *
     * @param listener the listener to register, or null to de-register
     */
    public void setDebugInitListener(DebugInitListener listener) {
        debugInitListener = listener;
    }

    /**
     * Alter which view ports will render the debug visualization.
     *
     * @param viewPorts (not null, alias created)
     */
    public void setDebugViewPorts(ViewPort[] viewPorts) {
        Validate.nonNull(viewPorts, "view ports");

        if (debugAppState != null) {
            debugAppState.setViewPorts(viewPorts);
        }
        int length = viewPorts.length;
        debugViewPorts = new ViewPort[length];
        System.arraycopy(viewPorts, 0, debugViewPorts, 0, length);
    }

    /**
     * Alter the simulation speed.
     *
     * @param speed the desired speed (&ge;0, default=1)
     */
    public void setSpeed(float speed) {
        Validate.nonNegative(speed, "speed");
        this.speed = speed;
    }

    /**
     * Alter which type of threading this app state uses. Not allowed after
     * attaching the app state.
     *
     * @param threadingType the desired type (not null, default=SEQUENTIAL)
     */
    public void setThreadingType(ThreadingType threadingType) {
        assert !isRunning;
        this.threadingType = threadingType;
    }

    /**
     * Alter the coordinate range. Not allowed after attaching the app state.
     *
     * @param worldMax the desired maximum coordinate values when using
     * AXIS_SWEEP broadphase algorithms (not null, unaffected,
     * default=10k,10k,10k)
     */
    public void setWorldMax(Vector3f worldMax) {
        Validate.nonNull(worldMin, "world max");
        assert !isRunning;

        this.worldMax.set(worldMax);
    }

    /**
     * Alter the coordinate range. Not allowed after attaching the app state.
     *
     * @param worldMin the desired minimum coordinate values when using
     * AXIS_SWEEP broadphase algorithms (not null, unaffected,
     * default=-10k,-10k,-10k)
     */
    public void setWorldMin(Vector3f worldMin) {
        Validate.nonNull(worldMin, "world min");
        assert !isRunning;

        this.worldMin.set(worldMin);
    }

    /**
     * Allocate a physics space and start physics.
     * <p>
     * Physics starts automatically after the state is attached. To start it
     * sooner, invoke this method.
     */
    public void startPhysics() {
        if (isRunning) {
            return;
        }

        switch (threadingType) {
            case PARALLEL:
                boolean success = startPhysicsOnExecutor();
                assert success;
                assert pSpace != null;
                break;

            case SEQUENTIAL:
                pSpace = new PhysicsSpace(worldMin, worldMax, broadphaseType);
                pSpace.addTickListener(this);
                break;

            default:
                throw new IllegalStateException(threadingType.toString());
        }

        isRunning = true;
    }
    // *************************************************************************
    // AppState methods

    /**
     * Transition this state from terminating to detached. Should be invoked
     * only by a subclass or by the AppStateManager. Invoked once for each time
     * {@link #initialize(com.jme3.app.state.AppStateManager, com.jme3.app.Application)}
     * is invoked.
     */
    @Override
    public void cleanup() {
        if (debugAppState != null) {
            stateManager.detach(debugAppState);
            debugAppState = null;
        }
        stopPhysics();
    }

    /**
     * Initialize this state prior to its 1st update. Should be invoked only by
     * a subclass or by the AppStateManager.
     *
     * @param stateManager the manager for this state (not null)
     * @param app the application which owns this state (not null)
     */
    @Override
    public void initialize(AppStateManager stateManager, Application app) {
        if (debugViewPorts == null) {
            debugViewPorts = new ViewPort[1];
            debugViewPorts[0] = app.getViewPort();
        }
        startPhysics();
    }

    /**
     * Test whether this state is enabled.
     *
     * @return true if enabled, otherwise false
     */
    @Override
    public boolean isEnabled() {
        return isEnabled;
    }

    /**
     * Test whether the physics simulation is running (started but not yet
     * stopped).
     *
     * @return true if running, otherwise false
     */
    @Override
    public boolean isInitialized() {
        return isRunning;
    }

    /**
     * Update this state after all rendering commands are flushed. Should be
     * invoked only by a subclass or by the AppStateManager. Invoked once per
     * frame, provided the state is attached and enabled.
     */
    @Override
    public void postRender() {
        if (physicsFuture != null) {
            try {
                physicsFuture.get();
                physicsFuture = null;
            } catch (InterruptedException | ExecutionException ex) {
                logger.log(Level.SEVERE, null, ex);
            }
        }
    }

    /**
     * Render this state. Should be invoked only by a subclass or by the
     * AppStateManager. Invoked once per frame, provided the state is attached
     * and enabled.
     *
     * @param rm the render manager (not null)
     */
    @Override
    public void render(RenderManager rm) {
        if (threadingType == ThreadingType.PARALLEL) {
            physicsFuture = executor.submit(parallelPhysicsUpdate);
        } else if (threadingType == ThreadingType.SEQUENTIAL) {
            pSpace.update(isEnabled ? tpf * speed : 0f);
        }
    }

    /**
     * Enable or disable this state.
     *
     * @param enabled true &rarr; enable, false &rarr; disable
     */
    @Override
    public void setEnabled(boolean enabled) {
        this.isEnabled = enabled;
    }

    /**
     * Transition this state from detached to initializing. Should be invoked
     * only by a subclass or by the AppStateManager.
     *
     * @param stateManager (not null)
     */
    @Override
    public void stateAttached(AppStateManager stateManager) {
        this.stateManager = stateManager;
        if (!isRunning) {
            startPhysics();
        }
        if (threadingType == ThreadingType.PARALLEL) {
            PhysicsSpace.setLocalThreadPhysicsSpace(pSpace);
        }
    }

    /**
     * Transition this state from running to terminating. Should be invoked only
     * by a subclass or by the AppStateManager.
     *
     * @param stateManager (not null)
     */
    @Override
    public void stateDetached(AppStateManager stateManager) {
    }

    /**
     * Update this state prior to rendering. Should be invoked only by a
     * subclass or by the AppStateManager. Invoked once per frame, provided the
     * state is attached and enabled.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void update(float tpf) {
        this.tpf = tpf;

        if (debugEnabled && debugAppState == null) {
            assert pSpace != null;
            assert debugViewPorts != null;
            debugAppState = new BulletDebugAppState(pSpace, debugViewPorts,
                    filter, debugInitListener);
            stateManager.attach(debugAppState);

        } else if (!debugEnabled && debugAppState != null) {
            stateManager.detach(debugAppState);
            debugAppState = null;
        }

        pSpace.distributeEvents();
    }
    // *************************************************************************
    // PhysicsTickListener methods

    /**
     * Callback from Bullet, invoked just after the physics is stepped. A good
     * time to clear/apply forces.
     *
     * @param space the space that is about to be stepped (not null)
     * @param timeStep the time per physics step (in seconds, &ge;0)
     */
    @Override
    public void physicsTick(PhysicsSpace space, float timeStep) {
    }

    /**
     * Callback from Bullet, invoked just before the physics is stepped. A good
     * time to clear/apply forces.
     *
     * @param space the space that is about to be stepped (not null)
     * @param timeStep the time per physics step (in seconds, &ge;0)
     */
    @Override
    public void prePhysicsTick(PhysicsSpace space, float timeStep) {
    }
    // *************************************************************************
    // private methods

    /**
     * Allocate the physics space and start physics for ThreadingType.PARALLEL.
     *
     * @return true if successful, otherwise false
     */
    private boolean startPhysicsOnExecutor() {
        if (executor != null) {
            executor.shutdown();
        }
        executor = new ScheduledThreadPoolExecutor(1);
        final BulletAppState appState = this;
        Callable<Boolean> call = new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                detachedPhysicsLastUpdate = System.currentTimeMillis();
                pSpace = new PhysicsSpace(worldMin, worldMax, broadphaseType);
                pSpace.addTickListener(appState);
                return true;
            }
        };
        try {
            return executor.submit(call).get();
        } catch (InterruptedException | ExecutionException ex) {
            logger.log(Level.SEVERE, null, ex);
            return false;
        }
    }

    /**
     * Stop physics after this state is detached.
     */
    private void stopPhysics() {
        if (!isRunning) {
            return;
        }

        if (executor != null) {
            executor.shutdown();
            executor = null;
        }
        pSpace.removeTickListener(this);
        pSpace = null;
        isRunning = false;
    }
}
