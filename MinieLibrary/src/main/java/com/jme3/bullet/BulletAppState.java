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
import com.jme3.app.state.AbstractAppState;
import com.jme3.app.state.AppStateManager;
import com.jme3.bullet.PhysicsSpace.BroadphaseType;
import com.jme3.bullet.debug.BulletDebugAppState;
import com.jme3.bullet.debug.DebugConfiguration;
import com.jme3.bullet.debug.DebugInitListener;
import com.jme3.bullet.util.NativeLibrary;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * An AppState to manage a single PhysicsSpace.
 *
 * @author normenhansen
 */
public class BulletAppState
        extends AbstractAppState
        implements PhysicsTickListener {
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
     * true if-and-only-if the physics simulation is running (started but not
     * yet stopped)
     */
    private volatile boolean isRunning = false;
    /**
     * broadphase collision-detection algorithm for the PhysicsSpace to use (not
     * null)
     */
    private BroadphaseType broadphaseType = BroadphaseType.DBVT;
    /**
     * AppState to manage the debug visualization, or null if none
     */
    private BulletDebugAppState debugAppState;

    final private Callable<Boolean> parallelPhysicsUpdate
            = new Callable<Boolean>() {
        @Override
        public Boolean call() throws Exception {
            PhysicsSpace pSpace = debugConfig.getSpace();
            pSpace.update(isEnabled() ? tpf * speed : 0f);
            return true;
        }
    };
    /**
     * configuration for debug visualization
     */
    final private DebugConfiguration debugConfig = new DebugConfiguration();
    /**
     * simulation speed multiplier (paused=0)
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
    /**
     * number of solvers in the thread-safe pool
     */
    private int numSolvers = NativeLibrary.countThreads();
    /**
     * executor service for physics tasks, or null if parallel simulation is not
     * running
     */
    private ScheduledThreadPoolExecutor executor;
    /**
     * constraint solver for the PhysicsSpace to use (not null)
     */
    private SolverType solverType = SolverType.SI;
    /**
     * threading mode to use (not null)
     */
    private ThreadingType threadingType = ThreadingType.SEQUENTIAL;
    /**
     * maximum coordinate values for the PhysicsSpace when using AXIS_SWEEP
     * broadphase algorithms (not null)
     */
    final private Vector3f worldMax = new Vector3f(10000f, 10000f, 10000f);
    /**
     * minimum coordinate values for the PhysicsSpace when using AXIS_SWEEP
     * broadphase algorithms (not null)
     */
    final private Vector3f worldMin = new Vector3f(-10000f, -10000f, -10000f);
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled app state to manage a new space with the DBVT
     * broadphase collision-detection algorithm.
     * <p>
     * Use {@code getStateManager().addState(bulletAppState)} to start
     * simulating physics.
     */
    public BulletAppState() {
        // do nothing
    }

    /**
     * Instantiate an enabled app state to manage a new PhysicsSpace.
     * <p>
     * Use {@code getStateManager().addState(bulletAppState)} to start
     * simulating physics.
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
     * Instantiate an enabled app state to manage a new space with the
     * AXIS_SWEEP_3 broadphase collision-detection algorithm.
     * <p>
     * Use {@code getStateManager().addState(bulletAppState)} to start
     * simulating physics.
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
     * Instantiate an enabled AppState to manage a new space.
     * <p>
     * Use {@code getStateManager().addState(bulletAppState)} to start
     * simulating physics.
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
        Validate.finite(worldMin, "world min");
        Validate.finite(worldMax, "world max");
        Validate.nonNull(broadphaseType, "broadphase type");

        this.worldMin.set(worldMin);
        this.worldMax.set(worldMax);
        this.broadphaseType = broadphaseType;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Count the solvers in the thread-safe pool.
     *
     * @return the count
     */
    public int countSolvers() {
        return numSolvers;
    }

    /**
     * Determine the length of the debug axis arrows.
     *
     * @return length (in physics-space units, &ge;0)
     */
    public float debugAxisLength() {
        float result = debugConfig.axisArrowLength();
        return result;
    }

    /**
     * Determine the line width of the debug axis arrows.
     *
     * @return width (in pixels, &ge;1) or 0 for solid arrows
     */
    public float debugAxisLineWidth() {
        float result = debugConfig.axisLineWidth();
        return result;
    }

    /**
     * Determine which broadphase collision-detection algorithm the PhysicsSpace
     * will use.
     *
     * @return enum value (not null)
     */
    public BroadphaseType getBroadphaseType() {
        return broadphaseType;
    }

    /**
     * Access the Camera used for debug visualization.
     *
     * @return the pre-existing instance, or null if unknown
     */
    public Camera getDebugCamera() {
        Camera result = debugConfig.getCamera();
        return result;
    }

    /**
     * Access the PhysicsSpace managed by this state. Normally there is none
     * until the state is attached.
     *
     * @return the pre-existing instance, or null if no simulation running
     */
    public PhysicsSpace getPhysicsSpace() {
        PhysicsSpace result = debugConfig.getSpace();
        return result;
    }

    /**
     * Determine which constraint solver the PhysicsSpace will use.
     *
     * @return the enum value (not null)
     */
    public SolverType getSolverType() {
        assert solverType != null;
        return solverType;
    }

    /**
     * Determine the physics simulation speed.
     *
     * @return the speedup factor (&ge;0, default=1)
     */
    public float getSpeed() {
        assert speed >= 0f : speed;
        return speed;
    }

    /**
     * Determine which type of threading this app state uses.
     *
     * @return the threadingType (not null)
     */
    public ThreadingType getThreadingType() {
        assert threadingType != null;
        return threadingType;
    }

    /**
     * Test whether debug visualization is enabled.
     *
     * @return true if enabled, otherwise false
     */
    public boolean isDebugEnabled() {
        boolean result = debugConfig.isEnabled();
        return result;
    }

    /**
     * Test whether the physics simulation is running (started but not yet
     * stopped).
     *
     * @return true if running, otherwise false
     */
    public boolean isRunning() {
        return isRunning;
    }

    /**
     * Alter the broadphase type the PhysicsSpace will use. Not allowed after
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
     * Alter which angular velocities are included in the debug visualization.
     *
     * @param filter the filter to use (alias created) or null to visualize no
     * angular velocities (default=null)
     */
    public void setDebugAngularVelocityFilter(
            BulletDebugAppState.DebugAppStateFilter filter) {
        if (debugAppState == null) {
            debugConfig.setAngularVelocityFilter(filter);
        } else {
            debugAppState.setAngularVelocityFilter(filter);
        }
    }

    /**
     * Alter the length of the debug axis arrows.
     *
     * @param length the desired length (in physics-space units, &ge;0)
     */
    public void setDebugAxisLength(float length) {
        Validate.nonNegative(length, "length");
        debugConfig.setAxisArrowLength(length);
    }

    /**
     * Alter the line width for debug axis arrows.
     *
     * @param width the desired width (in pixels, &ge;1) or 0 for solid arrows
     * (default=1)
     */
    public void setDebugAxisLineWidth(float width) {
        Validate.inRange(width, "width", 0f, Float.MAX_VALUE);
        debugConfig.setAxisLineWidth(width);
    }

    /**
     * Alter which bounding boxes are included in the debug visualization.
     *
     * @param filter the filter to use (alias created) or null to visualize no
     * bounding boxes (default=null)
     */
    public void setDebugBoundingBoxFilter(
            BulletDebugAppState.DebugAppStateFilter filter) {
        if (debugAppState == null) {
            debugConfig.setBoundingBoxFilter(filter);
        } else {
            debugAppState.setBoundingBoxFilter(filter);
        }
    }

    /**
     * Replace the Camera used for debug visualization.
     *
     * @param camera the Camera to use (alias created) or null for unknown
     * (defaults to the application's main camera)
     */
    public void setDebugCamera(Camera camera) {
        debugConfig.setCamera(camera);
    }

    /**
     * Enable or disable debug visualization. Changes take effect on the next
     * update.
     *
     * @param debugEnabled true &rarr; enable, false &rarr; disable
     * (default=false)
     */
    public void setDebugEnabled(boolean debugEnabled) {
        debugConfig.setEnabled(debugEnabled);
    }

    /**
     * Alter which objects are included in the debug visualization.
     *
     * @param filter the filter to use (alias created) or null to visualize all
     * objects (default=null)
     */
    public void setDebugFilter(BulletDebugAppState.DebugAppStateFilter filter) {
        debugConfig.setFilter(filter);
    }

    /**
     * Alter which gravity vectors are included in the debug visualization.
     *
     * @param filter the filter to use (alias created) or null to visualize no
     * gravity vectors (default=null)
     */
    public void setDebugGravityVectorFilter(
            BulletDebugAppState.DebugAppStateFilter filter) {
        if (debugAppState == null) {
            debugConfig.setGravityVectorFilter(filter);
        } else {
            debugAppState.setGravityVectorFilter(filter);
        }
    }

    /**
     * Replace or remove the init listener for the BulletDebugAppState.
     *
     * @param listener the listener to register, or null to de-register the
     * current listener (default=null)
     */
    public void setDebugInitListener(DebugInitListener listener) {
        debugConfig.setInitListener(listener);
    }

    /**
     * Alter the line width for PhysicsJoint debug arrows.
     *
     * @param width (in pixels, &ge;1, default=1)
     */
    public void setDebugJointLineWidth(float width) {
        Validate.inRange(width, "width", 1f, Float.MAX_VALUE);

        if (debugAppState == null) {
            debugConfig.setJointLineWidth(width);
        } else {
            debugAppState.setJointLineWidth(width);
        }
    }

    /**
     * Alter the shadow mode of the debug root node.
     *
     * @param mode the desired value (not null, default=Off)
     */
    public void setDebugShadowMode(RenderQueue.ShadowMode mode) {
        Validate.nonNull(mode, "mode");

        if (debugAppState != null) {
            Node node = debugAppState.getRootNode();
            node.setShadowMode(mode);
        }
        debugConfig.setShadowMode(mode);
    }

    /**
     * Alter which swept spheres are included in the debug visualization.
     *
     * @param filter the filter to use (alias created) or null to visualize no
     * swept spheres (default=null)
     */
    public void setDebugSweptSphereFilter(
            BulletDebugAppState.DebugAppStateFilter filter) {
        if (debugAppState == null) {
            debugConfig.setSweptSphereFilter(filter);
        } else {
            debugAppState.setSweptSphereFilter(filter);
        }
    }

    /**
     * Alter which velocity vectors are included in the debug visualization.
     *
     * @param filter the filter to use (alias created) or null to visualize no
     * velocity vectors (default=null)
     */
    public void setDebugVelocityVectorFilter(
            BulletDebugAppState.DebugAppStateFilter filter) {
        if (debugAppState == null) {
            debugConfig.setVelocityVectorFilter(filter);
        } else {
            debugAppState.setVelocityVectorFilter(filter);
        }
    }

    /**
     * Alter which view ports will render the debug visualization.
     *
     * @param viewPorts (not null, aliases created)
     */
    public void setDebugViewPorts(ViewPort... viewPorts) {
        Validate.nonNull(viewPorts, "view ports");
        debugConfig.setViewPorts(viewPorts);
    }

    /**
     * Alter the number of solvers in the thread-safe pool.
     *
     * @param numSolvers the desired number of solvers in the thread-safe pool
     * (&ge;1, &le;64, default=numThreads)
     */
    public void setNumSolvers(int numSolvers) {
        Validate.inRange(numSolvers, "number of solvers", 1, 64);
        assert !isRunning();

        this.numSolvers = numSolvers;
    }

    /**
     * Alter which constraint solver the PhysicsSpace will use. Not allowed
     * after attaching the AppState.
     *
     * @param solver an enum value (not null, default=SI)
     */
    public void setSolverType(SolverType solver) {
        Validate.nonNull(solver, "solver");
        assert !isRunning();

        this.solverType = solver;
    }

    /**
     * Alter the physics simulation speed.
     *
     * @param speed the desired speedup factor (&ge;0, default=1)
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
     * Alter the conversion from physics-space coordinates to world coordinates.
     *
     * @param spatial the Spatial to use for coordinate transformations (alias
     * created) or null for physics=world (default=null)
     */
    public void setTransformSpatial(Spatial spatial) {
        debugConfig.setTransformSpatial(spatial);
    }

    /**
     * Alter the coordinate range. Not allowed after attaching the app state.
     *
     * @param worldMax the desired maximum coordinate values when using
     * AXIS_SWEEP broadphase algorithms (not null, unaffected,
     * default=(10k,10k,10k))
     */
    public void setWorldMax(Vector3f worldMax) {
        Validate.finite(worldMin, "world max");
        assert !isRunning;

        this.worldMax.set(worldMax);
    }

    /**
     * Alter the coordinate range. Not allowed after attaching the app state.
     *
     * @param worldMin the desired minimum coordinate values when using
     * AXIS_SWEEP broadphase algorithms (not null, unaffected,
     * default=(-10k,-10k,-10k))
     */
    public void setWorldMin(Vector3f worldMin) {
        Validate.finite(worldMin, "world min");
        assert !isRunning;

        this.worldMin.set(worldMin);
    }

    /**
     * Allocate a PhysicsSpace and start simulating physics.
     * <p>
     * Simulation starts automatically after the state is attached. To start it
     * sooner, invoke this method.
     */
    public void startPhysics() {
        if (isRunning) {
            return;
        }

        PhysicsSpace pSpace;
        switch (threadingType) {
            case PARALLEL:
                boolean success = startPhysicsOnExecutor();
                assert success;

                pSpace = debugConfig.getSpace();
                assert pSpace != null;
                break;

            case SEQUENTIAL:
                pSpace = createPhysicsSpace(worldMin, worldMax, broadphaseType);
                debugConfig.setSpace(pSpace);
                pSpace.addTickListener(this);
                break;

            default:
                throw new IllegalStateException(threadingType.toString());
        }

        setRunning(true);
    }

    /**
     * Stop physics after this state is detached.
     */
    public void stopPhysics() {
        if (!isRunning) {
            return;
        }

        if (executor != null) {
            executor.shutdown();
            this.executor = null;
        }
        PhysicsSpace pSpace = debugConfig.getSpace();
        pSpace.removeTickListener(this);
        setPhysicsSpace(null);
        setRunning(false);
    }

    /**
     * Copy the maximum coordinate values for AXIS_SWEEP.
     *
     * @return a new location vector (in physics-space coordinates, not null)
     */
    public Vector3f worldMax() {
        Vector3f result = worldMax.clone();
        return result;
    }

    /**
     * Copy the minimum coordinate values for AXIS_SWEEP.
     *
     * @return a new location vector (in physics-space coordinates, not null)
     */
    public Vector3f worldMin() {
        Vector3f result = worldMin.clone();
        return result;
    }
    // *************************************************************************
    // new protected methods

    /**
     * Create the configured debug-visualization app state.
     *
     * @return a new instance (not null)
     */
    protected BulletDebugAppState createDebugAppState() {
        BulletDebugAppState appState = new BulletDebugAppState(debugConfig);
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
    protected PhysicsSpace createPhysicsSpace(Vector3f min, Vector3f max,
            BroadphaseType type) {
        PhysicsSpace result;
        if (solverType == SolverType.SI) {
            result = new PhysicsSpace(min, max, type, numSolvers);
        } else if (numSolvers == 1) {
            result = new PhysicsSpace(min, max, type, solverType);
        } else {
            String message
                    = String.format("num=%d, type=%s", numSolvers, solverType);
            throw new IllegalArgumentException(message);
        }

        return result;
    }

    /**
     * Access the AppState to manage the debug visualization.
     *
     * @return the pre-existing instance, or null if none
     */
    protected BulletDebugAppState getDebugAppState() {
        return debugAppState;
    }

    /**
     * Access the configuration for debug visualization.
     *
     * @return the pre-existing instance (not null)
     */
    protected DebugConfiguration getDebugConfiguration() {
        assert debugConfig != null;
        return debugConfig;
    }

    /**
     * Alter which PhysicsSpace is managed by this state.
     *
     * @param newSpace the space to be managed (may be null)
     */
    protected void setPhysicsSpace(PhysicsSpace newSpace) {
        debugConfig.setSpace(newSpace);
    }

    /**
     * Alter whether the physics simulation is running (started but not yet
     * stopped).
     *
     * @param desiredSetting true&rarr;running, false&rarr;not running
     */
    protected void setRunning(boolean desiredSetting) {
        this.isRunning = desiredSetting;
    }

    /**
     * Allocate the PhysicsSpace and start simulating physics using
     * ThreadingType.PARALLEL.
     *
     * @return true if successful, otherwise false
     */
    protected boolean startPhysicsOnExecutor() {
        if (executor != null) {
            executor.shutdown();
        }
        this.executor = new ScheduledThreadPoolExecutor(1);
        final BulletAppState appState = this;
        Callable<Boolean> call = new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                PhysicsSpace pSpace = createPhysicsSpace(
                        worldMin, worldMax, broadphaseType);
                debugConfig.setSpace(pSpace);
                pSpace.addTickListener(appState);
                return true;
            }
        };
        try {
            boolean success = executor.submit(call).get();
            return success;
        } catch (InterruptedException | ExecutionException exception) {
            logger.log(Level.SEVERE, null, exception);
            return false;
        }
    }
    // *************************************************************************
    // AbstractAppState methods

    /**
     * Transition this state from terminating to detached. Should be invoked
     * only by a subclass or by the AppStateManager.
     * <p>
     * Invoked once for each time {@link #initialize(
     * com.jme3.app.state.AppStateManager, com.jme3.app.Application)} is
     * invoked.
     */
    @Override
    public void cleanup() {
        super.cleanup();

        if (debugAppState != null) {
            stateManager.detach(debugAppState);
            this.debugAppState = null;
        }
        stopPhysics();
    }

    /**
     * Initialize this state prior to its first update. Should be invoked only
     * by a subclass or by the AppStateManager.
     *
     * @param stateManager the manager for this state (not null)
     * @param app the application which owns this state (not null)
     */
    @Override
    public void initialize(AppStateManager stateManager, Application app) {
        super.initialize(stateManager, app);

        debugConfig.initialize(app);
        startPhysics();
    }

    /**
     * Update this state after all rendering commands are flushed. Should be
     * invoked only by a subclass or by the AppStateManager. Invoked once per
     * frame, provided the state is attached and enabled.
     */
    @Override
    public void postRender() {
        super.postRender();

        if (physicsFuture != null) {
            try {
                physicsFuture.get();
                this.physicsFuture = null;
            } catch (InterruptedException | ExecutionException exception) {
                throw new RuntimeException(exception);
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
        super.render(rm);

        if (threadingType == ThreadingType.PARALLEL) {
            this.physicsFuture = executor.submit(parallelPhysicsUpdate);
        } else if (threadingType == ThreadingType.SEQUENTIAL) {
            PhysicsSpace pSpace = debugConfig.getSpace();
            pSpace.update(isEnabled() ? tpf * speed : 0f);
        }
    }

    /**
     * Transition this state from detached to initializing. Should be invoked
     * only by a subclass or by the AppStateManager.
     *
     * @param stateManager (not null)
     */
    @Override
    public void stateAttached(AppStateManager stateManager) {
        super.stateAttached(stateManager);

        this.stateManager = stateManager;
        if (!isRunning) {
            startPhysics();
        }
        if (threadingType == ThreadingType.PARALLEL) {
            PhysicsSpace pSpace = debugConfig.getSpace();
            CollisionSpace.setLocalThreadPhysicsSpace(pSpace);
        }
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
        super.update(tpf);
        this.tpf = tpf;

        boolean enable = debugConfig.isEnabled();
        if (enable && debugAppState == null) {
            // Start debug visualization.
            this.debugAppState = createDebugAppState();
            this.stateManager.attach(debugAppState);

        } else if (!enable && debugAppState != null) {
            // Stop debug visualization.
            this.stateManager.detach(debugAppState);
            this.debugAppState = null;
        }

        PhysicsSpace pSpace = debugConfig.getSpace();
        pSpace.distributeEvents();
    }
    // *************************************************************************
    // PhysicsTickListener methods

    /**
     * Callback from Bullet, invoked just after the physics is stepped. A good
     * time to clear/apply forces. Meant to be overridden.
     *
     * @param space the space that's about to be stepped (not null)
     * @param timeStep the time per simulation step (in seconds, &ge;0)
     */
    @Override
    public void physicsTick(PhysicsSpace space, float timeStep) {
        // do nothing
    }

    /**
     * Callback from Bullet, invoked just before the physics is stepped. A good
     * time to clear/apply forces. Meant to be overridden.
     *
     * @param space the space that's about to be stepped (not null)
     * @param timeStep the time per simulation step (in seconds, &ge;0)
     */
    @Override
    public void prePhysicsTick(PhysicsSpace space, float timeStep) {
        // do nothing
    }
}
