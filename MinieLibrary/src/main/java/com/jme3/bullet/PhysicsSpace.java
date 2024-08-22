/*
 * Copyright (c) 2009-2021 jMonkeyEngine
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

import com.jme3.app.AppTask;
import com.jme3.bullet.collision.ContactListener;
import com.jme3.bullet.collision.PhysicsCollisionListener;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.control.PhysicsControl;
import com.jme3.bullet.joints.Constraint;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsCharacter;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsVehicle;
import com.jme3.bullet.util.NativeLibrary;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.util.SafeArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.concurrent.Callable;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.Future;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A CollisionSpace to simulate dynamic physics, with its own
 * {@code btDiscreteDynamicsWorld}.
 *
 * @author normenhansen
 */
public class PhysicsSpace
        extends CollisionSpace
        implements ContactListener {
    // *************************************************************************
    // enums

    /**
     * Enumerate the available accelerators for broadphase collision detection.
     */
    public enum BroadphaseType {
        /**
         * btSimpleBroadphase: a brute-force reference implementation for
         * debugging purposes
         */
        SIMPLE,
        /**
         * btAxisSweep3: incremental 3-D sweep and prune, requires world bounds,
         * limited to 16_384 physics objects
         */
        AXIS_SWEEP_3,
        /**
         * bt32BitAxisSweep3: incremental 3-D sweep and prune, requires world
         * bounds, limited to 1_500_000 physics objects
         */
        AXIS_SWEEP_3_32,
        /**
         * btDbvtBroadphase: a fast, dynamic bounding-volume hierarchy based on
         * AABB tree to allow quicker addition/removal of physics objects
         */
        DBVT
    }
    // *************************************************************************
    // constants and loggers

    /**
     * index of the X axis
     */
    final public static int AXIS_X = 0;
    /**
     * index of the Y axis
     */
    final public static int AXIS_Y = 1;
    /**
     * index of the Z axis
     */
    final public static int AXIS_Z = 2;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(PhysicsSpace.class.getName());
    // *************************************************************************
    // fields

    /**
     * list of registered tick listeners
     */
    final private Collection<PhysicsTickListener> tickListeners
            = new SafeArrayList<>(PhysicsTickListener.class);
    /**
     * time step (in seconds, &gt;0) ignored when maxSubSteps=0
     */
    private float accuracy = 1f / 60f;
    /**
     * maximum time step (in seconds, &gt;0) ignored when maxSubSteps>0
     */
    private float maxTimeStep = 0.1f;
    /**
     * manage contact/collision listeners and events
     */
    private ContactManager manager = new DefaultContactManager(this);
    /**
     * maximum number of simulation steps per frame (&gt;0) or 0 for a variable
     * time step
     */
    private int maxSubSteps = 4;
    /**
     * map character IDs to added collision objects
     */
    final private Map<Long, PhysicsCharacter> characterMap
            = new ConcurrentHashMap<>(64);
    /**
     * map joint IDs to added objects
     */
    final private Map<Long, PhysicsJoint> jointMap
            = new ConcurrentHashMap<>(64);
    /**
     * map rigid-body IDs to added collision object (including vehicles)
     */
    final private Map<Long, PhysicsRigidBody> rigidMap
            = new ConcurrentHashMap<>(64);
    /**
     * map vehicle-controller IDs to added collision objects
     */
    final private Map<Long, PhysicsVehicle> vehicleMap
            = new ConcurrentHashMap<>(64);
    /**
     * first-in/first-out (FIFO) queue of physics tasks
     */
    final private Queue<AppTask<?>> pQueue = new ConcurrentLinkedQueue<>();
    /**
     * parameters used by the contact-and-constraint solver
     */
    private SolverInfo solverInfo;
    /**
     * type of contact-and-constraint solver (not null)
     */
    private SolverType solverType = SolverType.SI;
    /**
     * first-in/first-out (FIFO) queue of physics tasks for each thread
     */
    final protected static ThreadLocal<Queue<AppTask<?>>> pQueueTL
            = new ThreadLocal<Queue<AppTask<?>>>() { // TODO privatize
        @Override
        protected ConcurrentLinkedQueue<AppTask<?>> initialValue() {
            return new ConcurrentLinkedQueue<>();
        }
    };
    /**
     * copy of the gravity-acceleration vector for newly-added bodies (default
     * is 9.81 in the -Y direction, approximating Earth-normal gravity in MKS
     * units for a Y-up coordinate system)
     */
    final private Vector3f gravity = new Vector3f(0f, -9.81f, 0f);
    // *************************************************************************
    // constructors

    /**
     * Instantiate a PhysicsSpace with sequential-impulse solvers. Must be
     * invoked on the designated physics thread.
     *
     * @param broadphaseType which broadphase accelerator to use (not null)
     */
    public PhysicsSpace(BroadphaseType broadphaseType) {
        this(new Vector3f(-10_000f, -10_000f, -10_000f),
                new Vector3f(10_000f, 10_000f, 10_000f), broadphaseType);
    }

    /**
     * Instantiate a PhysicsSpace with an AXIS_SWEEP_3 broadphase accelerator
     * and sequential-impulse solvers. Must be invoked on the designated physics
     * thread.
     *
     * @param worldMin the desired minimum coordinate values (not null,
     * unaffected)
     * @param worldMax the desired maximum coordinate values (not null,
     * unaffected)
     */
    public PhysicsSpace(Vector3f worldMin, Vector3f worldMax) {
        this(worldMin, worldMax, BroadphaseType.AXIS_SWEEP_3);
    }

    /**
     * Instantiate a PhysicsSpace with sequential-impulse solvers. Must be
     * invoked on the designated physics thread.
     *
     * @param worldMin the desired minimum coordinate values (not null,
     * unaffected, default=(-10k,-10k,-10k))
     * @param worldMax the desired maximum coordinate values (not null,
     * unaffected, default=(10k,10k,10k))
     * @param broadphaseType which broadphase accelerator to use (not null)
     */
    public PhysicsSpace(Vector3f worldMin, Vector3f worldMax,
            BroadphaseType broadphaseType) {
        super(worldMin, worldMax, broadphaseType, NativeLibrary.countThreads());
    }

    /**
     * Instantiate a PhysicsSpace with the specified number of
     * sequential-impulse solvers. Must be invoked on the designated physics
     * thread.
     *
     * @param worldMin the desired minimum coordinate values (not null,
     * unaffected, default=(-10k,-10k,-10k))
     * @param worldMax the desired maximum coordinate values (not null,
     * unaffected, default=(10k,10k,10k))
     * @param broadphaseType which broadphase accelerator to use (not null)
     * @param numSolvers the desired number of solvers in the thread-safe pool
     * (&ge;1, &le;64, default=numThreads)
     */
    public PhysicsSpace(Vector3f worldMin, Vector3f worldMax,
            BroadphaseType broadphaseType, int numSolvers) {
        super(worldMin, worldMax, broadphaseType, numSolvers);
    }

    /**
     * Instantiate a PhysicsSpace with the specified number of
     * sequential-impulse solvers. Must be invoked on the designated physics
     * thread.
     *
     * @param worldMin the desired minimum coordinate values (not null,
     * unaffected, default=(-10k,-10k,-10k))
     * @param worldMax the desired maximum coordinate values (not null,
     * unaffected, default=(10k,10k,10k))
     * @param broadphaseType which broadphase accelerator to use (not null)
     * @param numSolvers the desired number of solvers in the thread-safe pool
     * (&ge;1, &le;64, default=numThreads)
     * @param configuration the desired configuration (not null)
     */
    public PhysicsSpace(
            Vector3f worldMin, Vector3f worldMax, BroadphaseType broadphaseType,
            int numSolvers, CollisionConfiguration configuration) {
        super(worldMin, worldMax, broadphaseType, numSolvers, configuration);
    }

    /**
     * Instantiate a PhysicsSpace with the specified contact-and-constraint
     * solver. Must be invoked on the designated physics thread.
     *
     * @param worldMin the desired minimum coordinate values (not null,
     * unaffected, default=(-10k,-10k,-10k))
     * @param worldMax the desired maximum coordinate values (not null,
     * unaffected, default=(10k,10k,10k))
     * @param broadphaseType which broadphase accelerator to use (not null)
     * @param solverType the desired contact-and-constraint solver (not null)
     */
    public PhysicsSpace(Vector3f worldMin, Vector3f worldMax,
            BroadphaseType broadphaseType, SolverType solverType) {
        this(worldMin, worldMax, broadphaseType, solverType,
                new CollisionConfiguration());
    }

    /**
     * Instantiate a PhysicsSpace with the specified contact-and-constraint
     * solver. Must be invoked on the designated physics thread.
     *
     * @param worldMin the desired minimum coordinate values (not null,
     * unaffected, default=(-10k,-10k,-10k))
     * @param worldMax the desired maximum coordinate values (not null,
     * unaffected, default=(10k,10k,10k))
     * @param broadphaseType which broadphase accelerator to use (not null)
     * @param solverType the desired contact-and-constraint solver (not null)
     * @param configuration the desired configuration (not null)
     */
    public PhysicsSpace(Vector3f worldMin, Vector3f worldMax,
            BroadphaseType broadphaseType, SolverType solverType,
            CollisionConfiguration configuration) {
        super(worldMin, worldMax, broadphaseType, 1, configuration);
        Validate.nonNull(solverType, "solver type");

        if (this.solverType != solverType) {
            this.solverType = solverType;
            updateSolver();
        }
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Activate all rigid bodies in the space.
     *
     * @param forceFlag true to force activation
     */
    public void activateAll(boolean forceFlag) {
        for (PhysicsRigidBody rigidBody : rigidMap.values()) {
            rigidBody.activate(forceFlag);
        }
    }

    /**
     * Add all physics controls in the specified subtree of the scene graph to
     * the space (e.g. after loading from disk). For compatibility with the
     * jme3-jbullet library.
     * <p>
     * Does not add joints unless they are managed by a PhysicsControl; the
     * jme3-jbullet version attempts to add ALL joints.
     * <p>
     * Note: recursive!
     *
     * @param spatial the root of the subtree (not null)
     */
    public void addAll(Spatial spatial) {
        add(spatial);

        if (spatial instanceof Node) {
            List<Spatial> children = ((Node) spatial).getChildren();
            for (Spatial child : children) {
                addAll(child);
            }
        }
    }

    /**
     * Register the specified listener for new contacts.
     * <p>
     * During distributeEvents(), registered listeners are notified of all new
     * contacts since the previous distributeEvents().
     *
     * @param listener the listener to register (not null, alias created)
     */
    public void addCollisionListener(PhysicsCollisionListener listener) {
        Validate.nonNull(listener, "listener");
        manager.addCollisionListener(listener);
    }

    /**
     * Register the specified listener for immediate contact notifications.
     *
     * @param listener the listener to register (not null, alias created)
     */
    public void addContactListener(ContactListener listener) {
        Validate.nonNull(listener, "listener");
        manager.addContactListener(listener, true, true, true);
    }

    /**
     * Register the specified listener for immediate contact notifications.
     *
     * @param listener the listener to register (not null, alias created)
     * @param doEnded true to enable {@code onContactEnded()} callbacks for the
     * listener, false to skip them (default=true)
     * @param doProcessed true to enable {@code onContactProcessed()} callbacks
     * for the listener, false to skip them (default=true)
     * @param doStarted true to enable {@code onContactStarted()} callbacks for
     * the listener, false to skip them (default=true)
     */
    public void addContactListener(ContactListener listener, boolean doEnded,
            boolean doProcessed, boolean doStarted) {
        Validate.nonNull(listener, "listener");
        manager.addContactListener(listener, doEnded, doProcessed, doStarted);
    }

    /**
     * Add the specified PhysicsJoint to the space.
     *
     * @param joint the joint to add (not null, alias created)
     */
    public void addJoint(PhysicsJoint joint) {
        Validate.nonNull(joint, "joint");
        if (contains(joint)) {
            logger.log(Level.WARNING, "{0} is already added to {1}.",
                    new Object[]{joint, this});
            return;
        }
        assert joint.getPhysicsSpace() == null;

        // Warn if the jointed bodies aren't already added to the space.
        PhysicsBody a = joint.getBodyA();
        if (a != null && !contains(a)) {
            logger.log(Level.WARNING,
                    "{0} at the A end of {1} has not yet been added to {2}.",
                    new Object[]{a, joint, this});
        }
        PhysicsBody b = joint.getBodyB();
        if (b != null && !contains(b)) {
            logger.log(Level.WARNING,
                    "{0} at the B end of {1} has not yet been added to {2}.",
                    new Object[]{b, joint, this});
        }
        assert a != b : a;

        if (logger.isLoggable(Level.FINE)) {
            logger.log(Level.FINE, "Adding {0} to {1}.",
                    new Object[]{joint, this});
        }
        long jointId = joint.nativeId();
        jointMap.put(jointId, joint);
        joint.setPhysicsSpace(this);

        if (joint instanceof Constraint) {
            long spaceId = nativeId();
            boolean disableCollisions = false; // ignore lists are already set!
            addConstraintC(spaceId, jointId, disableCollisions);
        }
    }

    /**
     * Register the specified listener for ongoing contacts.
     * <p>
     * During distributeEvents(), registered listeners are notified of all
     * ongoing contacts EXCEPT Sphere-Sphere contacts.
     *
     * @param listener the listener to register (not null, alias created)
     */
    public void addOngoingCollisionListener(PhysicsCollisionListener listener) {
        Validate.nonNull(listener, "listener");
        manager.addOngoingCollisionListener(listener);
    }

    /**
     * Register the specified tick listener with the space.
     * <p>
     * Tick listeners are notified before and after each simulation step. A
     * simulation step is not necessarily the same as a frame; it is more
     * influenced by the accuracy of the PhysicsSpace.
     *
     * @see #setAccuracy(float)
     *
     * @param listener the listener to register (not null, alias created)
     */
    public void addTickListener(PhysicsTickListener listener) {
        Validate.nonNull(listener, "listener");
        assert !tickListeners.contains(listener);

        tickListeners.add(listener);
    }

    /**
     * Test whether the specified PhysicsJoint is added to the space.
     *
     * @param joint the joint to test (not null, unaffected)
     * @return true if currently added, otherwise false
     */
    public boolean contains(PhysicsJoint joint) {
        long jointId = joint.nativeId();
        boolean result = jointMap.containsKey(jointId);

        return result;
    }

    /**
     * Count how many collision listeners are registered with the space.
     *
     * @return the count (&ge;0)
     */
    public int countCollisionListeners() {
        int result = manager.countCollisionListeners();
        return result;
    }

    /**
     * Count the joints in the space.
     *
     * @return the count (&ge;0)
     */
    public int countJoints() {
        long spaceId = nativeId();
        int count = getNumConstraints(spaceId);

        assert count == jointMap.size() : count;
        return count;
    }

    /**
     * Count the collision manifolds in the space.
     *
     * @return the current number of btPersistentManifolds (&ge;0)
     */
    public int countManifolds() {
        long spaceId = nativeId();
        int result = countManifolds(spaceId);

        return result;
    }

    /**
     * Count the rigid bodies in the space, including vehicles.
     *
     * @return count (&ge;0)
     */
    public int countRigidBodies() {
        int count = rigidMap.size();
        return count;
    }

    /**
     * Count how many tick listeners are registered with the space.
     *
     * @return the count (&ge;0)
     */
    public int countTickListeners() {
        int count = tickListeners.size();
        return count;
    }

    /**
     * Distribute queued collision events to registered listeners.
     */
    public void distributeEvents() {
        manager.distributeEvents();
    }

    /**
     * Invoke the specified callable during the next simulation step. This is
     * useful for applying forces.
     *
     * @param <V> the return type of the Callable
     * @param callable the Callable to invoke
     * @return a new AppTask
     */
    public <V> Future<V> enqueue(Callable<V> callable) {
        AppTask<V> task = new AppTask<>(callable);
        pQueue.add(task);

        return task;
    }

    /**
     * Enqueue a callable on the currently executing thread.
     *
     * @param <V> the task's result type
     * @param callable the task to be executed
     * @return a new task (not null)
     */
    public static <V> Future<V> enqueueOnThisThread(Callable<V> callable) {
        AppTask<V> task = new AppTask<>(callable);
        pQueueTL.get().add(task);

        return task;
    }

    /**
     * Return the simulation accuracy: the time step used when maxSubSteps&gt;0.
     *
     * @return the time step (in seconds, &gt;0)
     */
    public float getAccuracy() {
        return accuracy;
    }

    /**
     * Enumerate physics characters that have been added to the space and not
     * yet removed.
     *
     * @return a new unmodifiable collection of pre-existing instances (not
     * null)
     */
    public Collection<PhysicsCharacter> getCharacterList() {
        Collection<PhysicsCharacter> result = characterMap.values();
        result = Collections.unmodifiableCollection(result);

        return result;
    }

    /**
     * Access the current ContactManager.
     *
     * @return the pre-existing instance (not null)
     */
    public ContactManager getContactManager() {
        assert manager != null;
        return manager;
    }

    /**
     * Copy the gravitational acceleration for newly-added bodies.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the acceleration vector (in physics-space coordinates, either
     * storeResult or a new instance, not null)
     */
    public Vector3f getGravity(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        assert checkGravity(result);
        result.set(gravity);

        return result;
    }

    /**
     * Enumerate physics joints that have been added to the space and not yet
     * removed.
     *
     * @return a new unmodifiable collection of pre-existing instances (not
     * null)
     */
    public Collection<PhysicsJoint> getJointList() {
        Collection<PhysicsJoint> result = jointMap.values();
        result = Collections.unmodifiableCollection(result);

        return result;
    }

    /**
     * Access the PhysicsSpace <b>running on this thread</b>. For parallel
     * physics, this may be invoked from the OpenGL thread.
     *
     * @return the pre-existing PhysicsSpace running on this thread
     */
    public static PhysicsSpace getPhysicsSpace() {
        CollisionSpace result = getCollisionSpace();
        return (PhysicsSpace) result;
    }

    /**
     * Enumerate rigid bodies (including vehicles) that have been added to this
     * space and not yet removed.
     *
     * @return a new unmodifiable collection of pre-existing instances (not
     * null)
     */
    public Collection<PhysicsRigidBody> getRigidBodyList() {
        Collection<PhysicsRigidBody> result = rigidMap.values();
        result = Collections.unmodifiableCollection(result);

        return result;
    }

    /**
     * Access parameters used by the contact-and-constraint solver.
     *
     * @return the pre-existing instance (not null)
     */
    public SolverInfo getSolverInfo() {
        return solverInfo;
    }

    /**
     * Return the number of iterations used by the contact-and-constraint
     * solver, for compatibility with the jme3-jbullet library.
     *
     * @return the number of iterations used (&ge;1)
     */
    public int getSolverNumIterations() {
        return solverInfo.numIterations();
    }

    /**
     * Return the type of contact-and-constraint solver in use.
     *
     * @return an enum value (not null)
     */
    public SolverType getSolverType() {
        return solverType;
    }

    /**
     * Enumerate physics vehicles that have been added to the space and not yet
     * removed.
     *
     * @return a new unmodifiable collection of pre-existing instances (not
     * null)
     */
    public Collection<PhysicsVehicle> getVehicleList() {
        Collection<PhysicsVehicle> result = vehicleMap.values();
        result = Collections.unmodifiableCollection(result);

        return result;
    }

    /**
     * Test whether CCD checks for collisions with static and kinematic bodies
     * (native field: m_ccdWithStaticOnly).
     *
     * @return true if checks are limited, false if checking also for collisions
     * with dynamic bodies
     */
    public boolean isCcdWithStaticOnly() {
        long spaceId = nativeId();
        boolean result = isCcdWithStaticOnly(spaceId);

        return result;
    }

    /**
     * Test whether the space uses Speculative Contact Restitution (native
     * field: m_applySpeculativeContactRestitution).
     *
     * @return true if using SCR, otherwise false
     */
    public boolean isUsingScr() {
        long spaceId = nativeId();
        boolean result = isSpeculativeContactRestitution(spaceId);

        return result;
    }

    /**
     * Enumerate the native IDs of all collision manifolds in the space.
     *
     * @return a new array (not null, may be empty)
     * @see com.jme3.bullet.collision.PersistentManifolds
     */
    public long[] listManifoldIds() {
        long spaceId = nativeId();
        int numManifolds = countManifolds(spaceId);
        long[] result = new long[numManifolds];

        for (int index = 0; index < numManifolds; ++index) {
            long manifoldId = getManifoldByIndex(spaceId, index);
            result[index] = manifoldId;
        }

        return result;
    }

    /**
     * Return the maximum number of simulation steps per frame.
     *
     * @return the number of steps (&gt;1) or 0 for a variable time step
     */
    public int maxSubSteps() {
        assert maxSubSteps >= 0 : maxSubSteps;
        return maxSubSteps;
    }

    /**
     * Return the maximum time step (imposed when maxSubSteps=0).
     *
     * @return the maximum time step (in seconds, &gt;0, default=0.1)
     */
    public float maxTimeStep() {
        assert maxTimeStep > 0f : maxTimeStep;
        return maxTimeStep;
    }

    /**
     * Remove all physics controls in the specified subtree of the scene graph
     * from the space (e.g. before saving to disk). For compatibility with the
     * jme3-jbullet library.
     * <p>
     * Does not remove joints unless they are managed by a PhysicsControl; the
     * jme3-jbullet version attempts to remove ALL joints.
     * <p>
     * Note: recursive!
     *
     * @param spatial the root of the subtree (not null)
     */
    public void removeAll(Spatial spatial) {
        remove(spatial);

        if (spatial instanceof Node) {
            List<Spatial> children = ((Node) spatial).getChildren();
            for (Spatial child : children) {
                removeAll(child);
            }
        }
    }

    /**
     * De-register the specified listener for new contacts.
     *
     * @see
     * #addCollisionListener(com.jme3.bullet.collision.PhysicsCollisionListener)
     * @param listener the listener to de-register (not null)
     */
    public void removeCollisionListener(PhysicsCollisionListener listener) {
        Validate.nonNull(listener, "listener");
        manager.removeCollisionListener(listener);
    }

    /**
     * De-register the specified listener for immediate contact notifications.
     *
     * @see #addContactListener(com.jme3.bullet.collision.ContactListener)
     * @param listener the listener to de-register (not null)
     */
    public void removeContactListener(ContactListener listener) {
        Validate.nonNull(listener, "listener");
        manager.removeContactListener(listener);
    }

    /**
     * Remove the specified PhysicsJoint from the space.
     *
     * @param joint the joint to remove (not null)
     */
    public void removeJoint(PhysicsJoint joint) {
        Validate.nonNull(joint, "joint");
        long jointId = joint.nativeId();
        if (!jointMap.containsKey(jointId)) {
            logger.log(Level.WARNING, "{0} does not exist in {1}.",
                    new Object[]{joint, this});
            return;
        }
        assert joint.getPhysicsSpace() == this;

        if (logger.isLoggable(Level.FINE)) {
            logger.log(Level.FINE, "Removing {0} from {1}.",
                    new Object[]{joint, this});
        }
        jointMap.remove(jointId);
        joint.setPhysicsSpace(null);

        if (joint instanceof Constraint) {
            long spaceId = nativeId();
            removeConstraint(spaceId, jointId);
        }
    }

    /**
     * De-register the specified listener for ongoing contacts.
     *
     * @see #addOngoingCollisionListener(
     * com.jme3.bullet.collision.PhysicsCollisionListener)
     * @param listener the listener to de-register (not null)
     */
    public void removeOngoingCollisionListener(
            PhysicsCollisionListener listener) {
        Validate.nonNull(listener, "listener");
        manager.removeOngoingCollisionListener(listener);
    }

    /**
     * De-register the specified tick listener.
     *
     * @see #addTickListener(com.jme3.bullet.PhysicsTickListener)
     * @param listener the listener to de-register (not null, unaffected)
     */
    public void removeTickListener(PhysicsTickListener listener) {
        Validate.nonNull(listener, "listener");

        boolean success = tickListeners.remove(listener);
        assert success;
    }

    /**
     * Alter the accuracy (time step used when maxSubSteps&gt;0).
     * <p>
     * In general, the smaller the time step, the more accurate (and
     * compute-intensive) the simulation will be.
     *
     * @param accuracy the desired time step (in seconds, &gt;0, default=1/60)
     */
    public void setAccuracy(float accuracy) {
        Validate.positive(accuracy, "accuracy");
        this.accuracy = accuracy;
    }

    /**
     * Alter whether CCD checks for collisions with static and kinematic bodies
     * (native field: m_ccdWithStaticOnly).
     *
     * @param setting true to limit checking, false to check also for collisions
     * with dynamic bodies (default=false)
     */
    public void setCcdWithStaticOnly(boolean setting) {
        long spaceId = nativeId();
        setCcdWithStaticOnly(spaceId, setting);
    }

    /**
     * Replace the current ContactManager with the specified one.
     *
     * @param manager the desired manager (not null)
     */
    public void setContactManager(ContactManager manager) {
        Validate.nonNull(manager, "manager");
        this.manager = manager;
    }

    /**
     * Alter the gravitational acceleration acting on newly-added bodies.
     * <p>
     * Typically, when a body is added to a space, the body's gravity gets set
     * to that of the space. Thus, it is preferable to set the space's gravity
     * before adding any bodies.
     *
     * @param gravity the desired acceleration vector (in physics-space
     * coordinates, not null, unaffected, default=(0,-9.81,0))
     */
    public void setGravity(Vector3f gravity) {
        this.gravity.set(gravity);
        long spaceId = nativeId();
        setGravity(spaceId, gravity);
    }

    /**
     * Alter the maximum number of simulation steps per frame.
     * <p>
     * Extra simulation steps help maintain determinism when the render fps
     * drops below 1/accuracy. For example a value of 2 can compensate for frame
     * rates as low as 30fps, assuming the physics has an accuracy of 1/60 sec.
     * <p>
     * Setting this value too high can depress the frame rate.
     *
     * @param steps the desired maximum number of steps (&ge;1) or 0 for a
     * variable time step (default=4)
     */
    public void setMaxSubSteps(int steps) {
        Validate.nonNegative(steps, "steps");
        this.maxSubSteps = steps;
    }

    /**
     * Alter the maximum time step (imposed when maxSubSteps=0).
     * <p>
     * In general, the smaller the time step, the more accurate (and
     * compute-intensive) the simulation will be.
     *
     * @param maxTimeStep the desired maximum time step (in seconds, &gt;0,
     * default=0.1)
     */
    public void setMaxTimeStep(float maxTimeStep) {
        Validate.positive(maxTimeStep, "max time step");
        this.maxTimeStep = maxTimeStep;
    }

    /**
     * Alter the number of iterations used by the contact-and-constraint solver,
     * for compatibility with the jme3-jbullet library.
     * <p>
     * Use 4 for low quality, 20 for high quality.
     *
     * @param numIterations the desired number of iterations (&ge;1, default=10)
     */
    public void setSolverNumIterations(int numIterations) {
        Validate.positive(numIterations, "number of iterations");
        solverInfo.setNumIterations(numIterations);
    }

    /**
     * Update the space. Can be used to single-step the physics simulation, if
     * maxSubSteps is set to 0 or 1. This method should be invoked on the thread
     * that created the space.
     *
     * @see #setMaxSubSteps(int)
     * @param timeInterval the time interval to simulate (in seconds, &ge;0)
     */
    public void update(float timeInterval) {
        assert Validate.nonNegative(timeInterval, "time interval");

        float interval;
        if (maxSubSteps == 0) {
            interval = Math.min(timeInterval, maxTimeStep);
        } else {
            interval = timeInterval;
            assert maxSubSteps > 0 : maxSubSteps;
        }
        manager.update(interval, maxSubSteps);
    }

    /**
     * Update the space. This method should be invoked on the thread that
     * created the space.
     *
     * @param timeInterval the time interval to simulate (in seconds, &ge;0)
     * @param maxSteps the maximum number of steps of size {@code accuracy}
     * (&ge;1) or 0 for a single step of size {@code timeInterval}
     */
    public void update(float timeInterval, int maxSteps) {
        assert Validate.nonNegative(timeInterval, "time interval");
        assert Validate.nonNegative(maxSteps, "max steps");

        manager.update(timeInterval, maxSteps);
    }

    /**
     * Update the space. This method should be invoked from the thread that
     * created the space.
     *
     * @param timeInterval the time interval to simulate (in seconds, &ge;0)
     * @param maxSteps the maximum number of steps of size {@code accuracy}
     * (&ge;1) or 0 for a single step of size {@code timeInterval}
     * @param doEnded true to enable {@code onContactEnded()} callbacks, false
     * to skip them
     * @param doProcessed true to enable {@code onContactProcessed()} callbacks,
     * false to skip them
     * @param doStarted true to enable {@code onContactStarted()} callbacks,
     * false to skip them
     */
    public void update(float timeInterval, int maxSteps, boolean doEnded,
            boolean doProcessed, boolean doStarted) {
        assert Validate.nonNegative(timeInterval, "time interval");
        assert Validate.nonNegative(maxSteps, "max steps");

        if (NativeLibrary.jniEnvId() != jniEnvId()) {
            logger.log(Level.WARNING, "invoked from wrong thread");
        }

        long spaceId = nativeId();
        assert accuracy > 0f : accuracy;
        stepSimulation(spaceId, timeInterval, maxSteps, accuracy, doEnded,
                doProcessed, doStarted);
    }

    /**
     * Alter whether the space uses Speculative Contact Restitution (native
     * field: m_applySpeculativeContactRestitution).
     *
     * @param setting true to enable SCR, false to disable it (default=false)
     */
    public void useScr(boolean setting) {
        long spaceId = nativeId();
        setSpeculativeContactRestitution(spaceId, setting);
    }
    // *************************************************************************
    // new protected methods

    /**
     * Access the map from native IDs to physics joints.
     *
     * @return the pre-existing instance (not null)
     */
    protected Map<Long, PhysicsJoint> getJointMap() {
        return jointMap;
    }

    /**
     * Determine the type of the underlying btDynamicsWorld.
     *
     * @param spaceId the Bullet identifier of the space (non-zero)
     * @return 2 (for a discrete world) or 4 (for a soft-rigid world)
     */
    native protected static int getWorldType(long spaceId);

    /**
     * Initialize the solverInfo field during create().
     */
    protected void initSolverInfo() {
        long spaceId = nativeId();
        long solverInfoId = getSolverInfo(spaceId);
        this.solverInfo = new SolverInfo(solverInfoId);
    }

    /**
     * Replace the existing contact-and-constraint solver with a new one of the
     * correct type.
     */
    protected void updateSolver() {
        long spaceId = nativeId();
        int ordinal = solverType.ordinal();
        setSolverType(spaceId, ordinal);
    }
    // *************************************************************************
    // CollisionSpace methods

    /**
     * Add the specified object to the space. For compatibility with the
     * jme3-jbullet library.
     * <p>
     * The jme3-jbullet version allows the argument to be null.
     *
     * @param object the PhysicsControl, Spatial-with-PhysicsControl, collision
     * object, or PhysicsJoint to add (not null)
     */
    @Override
    public void add(Object object) {
        Validate.nonNull(object, "object");

        if (object instanceof PhysicsControl) {
            ((PhysicsControl) object).setPhysicsSpace(this);
        } else if (object instanceof Spatial) {
            Spatial spatial = (Spatial) object;
            for (int i = 0; i < spatial.getNumControls(); ++i) {
                if (spatial.getControl(i) instanceof PhysicsControl) {
                    add(spatial.getControl(i));
                }
            }
        } else if (object instanceof PhysicsJoint) {
            addJoint((PhysicsJoint) object);
        } else {
            super.add(object);
        }
    }

    /**
     * Add the specified collision object to the space.
     *
     * @param pco the collision object to add (not null)
     */
    @Override
    public void addCollisionObject(PhysicsCollisionObject pco) {
        Validate.nonNull(pco, "collision object");

        if (pco instanceof PhysicsRigidBody) {
            addRigidBody((PhysicsRigidBody) pco);
        } else if (pco instanceof PhysicsCharacter) {
            addCharacter((PhysicsCharacter) pco);
        } else {
            super.addCollisionObject(pco);
        }
    }

    /**
     * Test whether the specified collision object is added to the space.
     *
     * @param pco the object to test (not null, unaffected)
     * @return true if currently added, otherwise false
     */
    @Override
    public boolean contains(PhysicsCollisionObject pco) {
        boolean result;
        long pcoId = pco.nativeId();
        if (pco instanceof PhysicsRigidBody) {
            result = rigidMap.containsKey(pcoId);
        } else if (pco instanceof PhysicsCharacter) {
            result = characterMap.containsKey(pcoId);
        } else {
            result = super.contains(pco);
        }

        return result;
    }

    /**
     * Must be invoked on the designated physics thread.
     */
    @Override
    protected void create() {
        int broadphase = getBroadphaseType().ordinal();
        Vector3f max = getWorldMax(null);
        Vector3f min = getWorldMin(null);
        int numSolvers = countSolvers();
        CollisionConfiguration configuration = getConfiguration();
        long configurationId = configuration.nativeId();
        long nativeId = createPhysicsSpace(
                min, max, broadphase, numSolvers, configurationId);
        assert nativeId != 0L;

        assert getWorldType(nativeId) == 2 // BT_DISCRETE_DYNAMICS_WORLD
                : getWorldType(nativeId);
        initThread(nativeId);
        initSolverInfo();
        logger.log(Level.FINE, "Created {0}.", this);
    }

    /**
     * Remove all collision objects and physics joints.
     */
    @Override
    public void destroy() {
        super.destroy();

        for (PhysicsCharacter character : characterMap.values()) {
            removeCharacter(character);
        }
        for (PhysicsJoint joint : jointMap.values()) {
            removeJoint(joint);
        }
        for (PhysicsRigidBody rigidBody : rigidMap.values()) {
            removeRigidBody(rigidBody);
        }
    }

    /**
     * Enumerate collision objects that have been added to the space and not yet
     * removed.
     *
     * @return a new modifiable collection of pre-existing instances (not null)
     */
    @Override
    public Collection<PhysicsCollisionObject> getPcoList() {
        Collection<PhysicsCollisionObject> result = super.getPcoList();
        result.addAll(rigidMap.values());
        result.addAll(characterMap.values());

        return result;
    }

    /**
     * Test whether the space is empty.
     *
     * @return true if empty, otherwise false
     */
    @Override
    public boolean isEmpty() {
        boolean result = super.isEmpty()
                && characterMap.isEmpty()
                && rigidMap.isEmpty()
                && jointMap.isEmpty();

        return result;
    }

    /**
     * Remove the specified object from the space. For compatibility with the
     * jme3-jbullet library.
     * <p>
     * The jme3-jbullet version allows the argument to be null.
     *
     * @param object the PhysicsControl, Spatial-with-PhysicsControl, collision
     * object, or PhysicsJoint to remove, or null
     */
    @Override
    public void remove(Object object) {
        if (object instanceof PhysicsControl) {
            ((PhysicsControl) object).setPhysicsSpace(null);
        } else if (object instanceof Spatial) {
            Spatial spatial = (Spatial) object;
            for (int i = 0; i < spatial.getNumControls(); ++i) {
                if (spatial.getControl(i) instanceof PhysicsControl) {
                    remove((spatial.getControl(i)));
                }
            }
        } else if (object instanceof PhysicsJoint) {
            removeJoint((PhysicsJoint) object);
        } else {
            super.remove(object);
        }
    }

    /**
     * Remove the specified collision object from the space.
     *
     * @param pco the collision object to remove (not null)
     */
    @Override
    public void removeCollisionObject(PhysicsCollisionObject pco) {
        Validate.nonNull(pco, "collision object");

        if (pco instanceof PhysicsRigidBody) {
            removeRigidBody((PhysicsRigidBody) pco);
        } else if (pco instanceof PhysicsCharacter) {
            removeCharacter((PhysicsCharacter) pco);
        } else {
            super.removeCollisionObject(pco);
        }
    }
    // *************************************************************************
    // ContactListener methods

    /**
     * Invoked by native code immediately after a contact manifold is destroyed.
     * Skipped if stepSimulation() was invoked with doEnded=false.
     *
     * @param manifoldId the native ID of the {@code btPersistentManifold} (not
     * zero)
     */
    @Override
    public void onContactEnded(long manifoldId) {
        assert NativeLibrary.jniEnvId() == jniEnvId() : "wrong thread";
        assert manifoldId != 0L;
        manager.onContactEnded(manifoldId);
    }

    /**
     * Invoked by native code immediately after a contact point is refreshed
     * without being destroyed. Skipped for Sphere-Sphere contacts. Skipped if
     * stepSimulation() was invoked with doProcessed=false.
     *
     * @param pcoA the first involved object (not null)
     * @param pcoB the 2nd involved object (not null)
     * @param pointId the native ID of the {@code btManifoldPoint} (not zero)
     */
    @Override
    public void onContactProcessed(PhysicsCollisionObject pcoA,
            PhysicsCollisionObject pcoB, long pointId) {
        assert NativeLibrary.jniEnvId() == jniEnvId() : "wrong thread";
        assert pcoA != null;
        assert pcoB != null;
        assert pointId != 0L;
        manager.onContactProcessed(pcoA, pcoB, pointId);
    }

    /**
     * Invoked by native code immediately after a contact manifold is created.
     * Skipped if stepSimulation() was invoked with doStarted=false.
     *
     * @param manifoldId the native ID of the {@code btPersistentManifold} (not
     * zero)
     */
    @Override
    public void onContactStarted(long manifoldId) {
        assert NativeLibrary.jniEnvId() == jniEnvId() : "wrong thread";
        assert manifoldId != 0L;
        manager.onContactStarted(manifoldId);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Add the specified PhysicsCharacter to the space.
     *
     * @param character the character to add (not null, alias created)
     */
    private void addCharacter(PhysicsCharacter character) {
        if (contains(character)) {
            logger.log(Level.WARNING, "{0} is already added to {1}.",
                    new Object[]{character, this});
            return;
        }
        assert !character.isInWorld();

        if (logger.isLoggable(Level.FINE)) {
            logger.log(Level.FINE, "Adding {0} to {1}.",
                    new Object[]{character, this});
        }
        long characterId = character.nativeId();
        characterMap.put(characterId, character);

        long spaceId = nativeId();
        addCharacterObject(spaceId, characterId);

        long actionId = character.getControllerId();
        addAction(spaceId, actionId);
    }

    /**
     * Add the specified PhysicsRigidBody to the space.
     * <p>
     * NOTE: When a rigid body is added, its gravity gets set to that of the
     * space.
     *
     * @param rigidBody the body to add (not null, modified)
     */
    private void addRigidBody(PhysicsRigidBody rigidBody) {
        if (contains(rigidBody)) {
            logger.log(Level.WARNING, "{0} is already added to {1}.",
                    new Object[]{rigidBody, this});
            return;
        }
        assert !rigidBody.isInWorld();

        if (logger.isLoggable(Level.FINE)) {
            logger.log(Level.FINE, "Adding {0} to {1}.",
                    new Object[]{rigidBody, this});
        }
        long rigidBodyId = rigidBody.nativeId();
        rigidMap.put(rigidBodyId, rigidBody);
        /*
         * Workaround:
         * It seems that adding a Kinematic RigidBody to the dynamicWorld
         * prevents it from being dynamic again afterward.
         * So we add it dynamic, then set it kinematic.
         */
        boolean kinematic = false;
        if (rigidBody.isKinematic()) {
            kinematic = true;
            rigidBody.setKinematic(false);
        }

        boolean useStaticGroup = rigidBody.isStatic();
        int proxyGroup = useStaticGroup ? 2 : 1;
        int proxyMask = useStaticGroup ? -3 : -1;
        long spaceId = nativeId();
        addRigidBody(spaceId, rigidBodyId, proxyGroup, proxyMask);

        if (kinematic) {
            rigidBody.setKinematic(true);
        }

        if (rigidBody instanceof PhysicsVehicle) {
            PhysicsVehicle vehicle = (PhysicsVehicle) rigidBody;
            if (logger.isLoggable(Level.FINE)) {
                logger.log(Level.FINE, "Adding action for {0} to {1}.",
                        new Object[]{vehicle, this});
            }

            vehicle.createVehicle(this);
            long actionId = vehicle.getVehicleId();
            vehicleMap.put(actionId, vehicle);
            addAction(spaceId, actionId);
        }
    }

    /**
     * Compare Bullet's gravity vector to the local copy.
     *
     * @param storeVector caller-allocated temporary storage (not null)
     * @return true if scale factors are exactly equal, otherwise false
     */
    private boolean checkGravity(Vector3f storeVector) {
        assert storeVector != null;

        long spaceId = nativeId();
        getGravity(spaceId, storeVector);
        boolean result = gravity.equals(storeVector);

        return result;
    }

    /**
     * Callback invoked just after the physics is stepped.
     *
     * @param timeStep the duration of the simulation step (in seconds, &ge;0)
     */
    private void postTick(float timeStep) {
        assert NativeLibrary.jniEnvId() == jniEnvId() : "wrong thread";

        for (PhysicsTickListener listener : tickListeners) {
            listener.physicsTick(this, timeStep);
        }
    }

    /**
     * Callback invoked just before the physics is stepped.
     *
     * @param timeStep the duration of the simulation step (in seconds, &ge;0)
     */
    private void preTick(float timeStep) {
        assert NativeLibrary.jniEnvId() == jniEnvId() : "wrong thread";

        while (true) {
            AppTask task = pQueue.poll();
            if (task == null) {
                task = pQueueTL.get().poll();
            }
            if (task == null) {
                break;
            } else if (!task.isCancelled()) {
                try {
                    task.invoke();
                } catch (RuntimeException exception) {
                    logger.log(Level.SEVERE, null, exception);
                }
            }
        }

        for (PhysicsTickListener listener : tickListeners) {
            listener.prePhysicsTick(this, timeStep);
        }
    }

    /**
     * Remove the specified PhysicsCharacter from the space.
     *
     * @param character the character to remove (not null)
     */
    private void removeCharacter(PhysicsCharacter character) {
        long characterId = character.nativeId();
        if (!characterMap.containsKey(characterId)) {
            logger.log(Level.WARNING, "{0} does not exist in {1}.",
                    new Object[]{character, this});
            return;
        }

        if (logger.isLoggable(Level.FINE)) {
            logger.log(Level.FINE, "Removing {0} from {1}.",
                    new Object[]{character, this});
        }
        characterMap.remove(characterId);

        long spaceId = nativeId();
        long actionId = character.getControllerId();
        removeAction(spaceId, actionId);

        removeCharacterObject(spaceId, characterId);
    }

    /**
     * Remove the specified PhysicsRigidBody from the space.
     *
     * @param rigidBody the body to remove (not null, modified)
     */
    private void removeRigidBody(PhysicsRigidBody rigidBody) {
        long rigidBodyId = rigidBody.nativeId();
        if (!rigidMap.containsKey(rigidBodyId)) {
            logger.log(Level.WARNING, "{0} does not exist in {1}.",
                    new Object[]{rigidBody, this});
            return;
        }

        long spaceId = nativeId();
        if (rigidBody instanceof PhysicsVehicle) {
            PhysicsVehicle vehicle = (PhysicsVehicle) rigidBody;
            if (logger.isLoggable(Level.FINE)) {
                logger.log(Level.FINE, "Removing action for {0} from {1}.",
                        new Object[]{vehicle, this});
            }

            long actionId = vehicle.getVehicleId();
            vehicleMap.remove(actionId);

            removeAction(spaceId, actionId);
        }

        if (logger.isLoggable(Level.FINE)) {
            logger.log(Level.FINE, "Removing {0} from {1}.",
                    new Object[]{rigidBody, this});
        }
        rigidMap.remove(rigidBodyId);

        removeRigidBody(spaceId, rigidBodyId);
    }
    // *************************************************************************
    // native private methods

    native private static void addAction(long spaceId, long actionId);

    native private static void
            addCharacterObject(long spaceId, long characterId);

    native private static void addConstraintC(
            long spaceId, long constraintId, boolean disableCollisions);

    native private static void addRigidBody(
            long spaceId, long rigidBodyId, int proxyGroup, int proxyMask);

    native private static int countManifolds(long spaceId);

    native private long createPhysicsSpace(
            Vector3f minVector, Vector3f maxVector, int broadphaseType,
            int numSolvers, long configurationId);

    native private static void getGravity(long spaceId, Vector3f storeVector);

    native private static long
            getManifoldByIndex(long spaceId, int manifoldIndex);

    native private static int getNumConstraints(long spaceId);

    native private static long getSolverInfo(long spaceId);

    native private static boolean isCcdWithStaticOnly(long spaceId);

    native private static boolean isSpeculativeContactRestitution(long spaceId);

    native private static void removeAction(long spaceId, long actionId);

    native private static void
            removeCharacterObject(long spaceId, long characterId);

    native private static void
            removeConstraint(long spaceId, long constraintId);

    native private static void removeRigidBody(long spaceId, long rigidBodyId);

    native private static void
            setCcdWithStaticOnly(long spaceId, boolean setting);

    native private static void setGravity(long spaceId, Vector3f gravityVector);

    native private static void setSolverType(long spaceId, int solverType);

    native private static void
            setSpeculativeContactRestitution(long spaceId, boolean apply);

    native private static void stepSimulation(long spaceId, float timeInterval,
            int maxSubSteps, float accuracy, boolean enableContactEndedCallback,
            boolean enableContactProcessedCallback,
            boolean enableContactStartedCallback);
}
