/*
 * Copyright (c) 2009-2020 jMonkeyEngine
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
import com.jme3.bullet.collision.PhysicsCollisionEvent;
import com.jme3.bullet.collision.PhysicsCollisionListener;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.control.PhysicsControl;
import com.jme3.bullet.joints.Constraint;
import com.jme3.bullet.joints.JointEnd;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsCharacter;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsVehicle;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.util.SafeArrayList;
import java.util.ArrayDeque;
import java.util.Collection;
import java.util.Deque;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.TreeSet;
import java.util.concurrent.Callable;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.Future;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A CollisionSpace to simulate dynamic physics, with its own btDynamicsWorld.
 *
 * @author normenhansen
 */
public class PhysicsSpace extends CollisionSpace {
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
     * queue of collision events not yet distributed to listeners
     */
    final private Deque<PhysicsCollisionEvent> collisionEvents
            = new ArrayDeque<>(20);
    /**
     * time step (in seconds, &gt;0) ignored when maxSubSteps=0
     */
    private float accuracy = 1f / 60f;
    /**
     * maximum time step (in seconds, &gt;0) ignored when maxSubSteps>0
     */
    private float maxTimeStep = 0.1f;
    /**
     * maximum number of time steps per frame, or 0 for a variable time step
     * (&ge;0, default=4)
     */
    private int maxSubSteps = 4;
    /**
     * copy of number of iterations used by the contact-and-constraint solver
     * (default=10)
     */
    private int solverNumIterations = 10;
    /**
     * list of registered collision listeners
     */
    final private List<PhysicsCollisionListener> collisionListeners
            = new SafeArrayList<>(PhysicsCollisionListener.class);
    /**
     * list of registered tick listeners
     */
    final private List<PhysicsTickListener> tickListeners
            = new SafeArrayList<>(PhysicsTickListener.class);
    /**
     * map character IDs to added objects TODO rename
     */
    final private Map<Long, PhysicsCharacter> physicsCharacters
            = new ConcurrentHashMap<>(64);
    /**
     * map joint IDs to added objects TODO rename
     */
    final protected Map<Long, PhysicsJoint> physicsJoints
            = new ConcurrentHashMap<>(64);
    /**
     * map rigid-body IDs to added objects (including vehicles)
     */
    final private Map<Long, PhysicsRigidBody> physicsBodies
            = new ConcurrentHashMap<>(64);
    /**
     * map vehicle IDs to added objects TODO rename
     */
    final private Map<Long, PhysicsVehicle> physicsVehicles
            = new ConcurrentHashMap<>(64);
    /**
     * first-in/first-out (FIFO) queue of physics tasks
     */
    final protected Queue<AppTask<?>> pQueue
            = new ConcurrentLinkedQueue<>();
    /**
     * physics-space reference for each thread
     */
    final protected static ThreadLocal<PhysicsSpace> physicsSpaceTL
            = new ThreadLocal<PhysicsSpace>();
    /**
     * first-in/first-out (FIFO) queue of physics tasks for each thread
     */
    final protected static ThreadLocal<Queue<AppTask<?>>> pQueueTL
            = new ThreadLocal<Queue<AppTask<?>>>() {
        @Override
        protected ConcurrentLinkedQueue<AppTask<?>> initialValue() {
            return new ConcurrentLinkedQueue<>();
        }
    };
    /**
     * copy of gravity-acceleration vector for newly-added bodies (default is
     * 9.81 in the -Y direction, corresponding to Earth-normal in MKS units)
     */
    final private Vector3f gravity = new Vector3f(0, -9.81f, 0);
    // *************************************************************************
    // constructors

    /**
     * Instantiate a PhysicsSpace with the specified broadphase accelerator.
     * Must be invoked on the designated physics thread.
     *
     * @param broadphaseType which broadphase accelerator to use (not null)
     */
    public PhysicsSpace(BroadphaseType broadphaseType) {
        this(new Vector3f(-10000f, -10000f, -10000f),
                new Vector3f(10000f, 10000f, 10000f), broadphaseType);
    }

    /**
     * Instantiate a PhysicsSpace with an AXIS_SWEEP_3 broadphase accelerator.
     * Must be invoked on the designated physics thread.
     *
     * @param worldMin the desired minimum coordinates values (not null,
     * unaffected)
     * @param worldMax the desired minimum coordinates values (not null,
     * unaffected)
     */
    public PhysicsSpace(Vector3f worldMin, Vector3f worldMax) {
        this(worldMin, worldMax, BroadphaseType.AXIS_SWEEP_3);
    }

    /**
     * Instantiate a PhysicsSpace. Must be invoked on the designated physics
     * thread.
     *
     * @param worldMin the desired minimum coordinates values (not null,
     * unaffected, default=-10k,-10k,-10k)
     * @param worldMax the desired minimum coordinates values (not null,
     * unaffected, default=10k,10k,10k)
     * @param broadphaseType which broadphase accelerator to use (not null)
     */
    public PhysicsSpace(Vector3f worldMin, Vector3f worldMax,
            BroadphaseType broadphaseType) {
        super(worldMin, worldMax, broadphaseType);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Add all physics controls in the specified subtree of the scene graph to
     * this space (e.g. after loading from disk).
     * <p>
     * Does not add any joints.
     * <p>
     * Note: recursive!
     *
     * @param spatial the root of the subtree (not null)
     */
    public void addAll(Spatial spatial) {
        add(spatial);
        //recursion
        if (spatial instanceof Node) {
            List<Spatial> children = ((Node) spatial).getChildren();
            for (Spatial spat : children) {
                addAll(spat);
            }
        }
    }

    /**
     * Register the specified collision listener with this space.
     * <p>
     * Collision listeners are notified when collisions occur in the space.
     *
     * @param listener the listener to register (not null, alias created)
     */
    public void addCollisionListener(PhysicsCollisionListener listener) {
        Validate.nonNull(listener, "listener");
        assert !collisionListeners.contains(listener);

        collisionListeners.add(listener);
    }

    /**
     * Register the specified tick listener with this space.
     * <p>
     * Tick listeners are notified before and after each physics step. A physics
     * step is not necessarily the same as a frame; it is more influenced by the
     * accuracy of the PhysicsSpace.
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
     * Test whether the specified PhysicsJoint is added to this space.
     *
     * @param joint the joint to test (not null, unaffected)
     * @return true if currently added, otherwise false
     */
    public boolean contains(PhysicsJoint joint) {
        long jointId = joint.getObjectId();
        boolean result = physicsJoints.containsKey(jointId);

        return result;
    }

    /**
     * Count how many collision listeners are registered with this space.
     *
     * @return the count (&ge;0)
     */
    public int countCollisionListeners() {
        int count = collisionListeners.size();
        return count;
    }

    /**
     * Count the joints in this space.
     *
     * @return count (&ge;0)
     */
    public int countJoints() {
        long spaceId = getSpaceId();
        int count = getNumConstraints(spaceId);
        assert count == physicsJoints.size() : count;
        return count;
    }

    /**
     * Count the rigid bodies in this space, including vehicles.
     *
     * @return count (&ge;0)
     */
    public int countRigidBodies() {
        int count = physicsBodies.size();
        return count;
    }

    /**
     * Count how many tick listeners are registered with this space.
     *
     * @return the count (&ge;0)
     */
    public int countTickListeners() {
        int count = tickListeners.size();
        return count;
    }

    /**
     * For compatibility with the jme3-bullet library.
     */
    public void destroy() {
    }

    /**
     * Distribute each collision event to all listeners.
     */
    public void distributeEvents() {
        while (!collisionEvents.isEmpty()) {
            PhysicsCollisionEvent event = collisionEvents.pop();
            for (PhysicsCollisionListener listener : collisionListeners) {
                listener.collision(event);
            }
        }
    }

    /**
     * Invoke the specified callable during the next physics tick. This is
     * useful for applying forces.
     *
     * @param <V> the return type of the callable
     * @param callable which callable to invoke
     * @return Future object
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
     * Read the accuracy: the time step used when maxSubSteps&gt;0.
     *
     * @return the time step (in seconds, &gt;0)
     */
    public float getAccuracy() {
        return accuracy;
    }

    /**
     * Enumerate physics characters that have been added to this space and not
     * yet removed.
     *
     * @return a new collection of pre-existing instances (not null)
     */
    public Collection<PhysicsCharacter> getCharacterList() {
        TreeSet<PhysicsCharacter> result = new TreeSet<>();
        result.addAll(physicsCharacters.values());

        return result;
    }

    /**
     * Determine the global CFM for this space.
     *
     * @return the CFM value
     */
    public float getGlobalCfm() {
        long spaceId = getSpaceId();
        float result = getGlobalCfm(spaceId);

        return result;
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
     * Enumerate physics joints that have been added to this space and not yet
     * removed.
     *
     * @return a new collection of pre-existing instances (not null)
     */
    public Collection<PhysicsJoint> getJointList() {
        TreeSet<PhysicsJoint> result = new TreeSet<>();
        result.addAll(physicsJoints.values());

        return result;
    }

    /**
     * Access the PhysicsSpace <b>running on this thread</b>. For parallel
     * physics, this can be invoked from the OpenGL thread.
     *
     * @return the pre-existing PhysicsSpace running on this thread
     */
    public static PhysicsSpace getPhysicsSpace() {
        return (PhysicsSpace) getCollisionSpace();
    }

    /**
     * Enumerate rigid bodies (including vehicles) that have been added to this
     * space and not yet removed.
     *
     * @return a new collection of pre-existing instances (not null)
     */
    public Collection<PhysicsRigidBody> getRigidBodyList() {
        return new TreeSet<>(physicsBodies.values());
    }

    /**
     * Read the number of iterations used by the contact-and-constraint solver
     * (native field: m_numIterations).
     *
     * @return the number of iterations used (&ge;1)
     */
    public int getSolverNumIterations() {
        return solverNumIterations;
    }

    /**
     * Enumerate physics vehicles that have been added to this space and not yet
     * removed.
     *
     * @return a new collection of pre-existing instances (not null)
     */
    public Collection<PhysicsVehicle> getVehicleList() {
        return new TreeSet<>(physicsVehicles.values());
    }

    /**
     * Read the maximum number of time steps per frame.
     *
     * @return number of steps (&gt;0) or 0 for a variable time step
     */
    public int maxSubSteps() {
        assert maxSubSteps >= 0 : maxSubSteps;
        return maxSubSteps;
    }

    /**
     * Read the maximum time step (imposed when maxSubSteps=0).
     *
     * @return the maximum time step (in seconds, &gt;0, default=0.1)
     */
    public float maxTimeStep() {
        assert maxTimeStep > 0f : maxTimeStep;
        return maxTimeStep;
    }

    /**
     * Remove all physics controls in the specified subtree of the scene graph
     * from this space (e.g. before saving to disk).
     * <p>
     * Does not remove any joints.
     * <p>
     * Note: recursive!
     *
     * @param spatial the root of the subtree (not null)
     */
    public void removeAll(Spatial spatial) {
        remove(spatial);
        //recursion
        if (spatial instanceof Node) {
            List<Spatial> children = ((Node) spatial).getChildren();
            for (Spatial spat : children) {
                removeAll(spat);
            }
        }
    }

    /**
     * De-register the specified collision listener.
     *
     * @see
     * #addCollisionListener(com.jme3.bullet.collision.PhysicsCollisionListener)
     * @param listener the listener to de-register (not null)
     */
    public void removeCollisionListener(PhysicsCollisionListener listener) {
        Validate.nonNull(listener, "listener");

        boolean success = collisionListeners.remove(listener);
        assert success;
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
     * Alter the global CFM for this space.
     *
     * @param cfm the desired value (default=0)
     */
    public void setGlobalCfm(float cfm) {
        long spaceId = getSpaceId();
        setGlobalCfm(spaceId, cfm);
    }

    /**
     * Alter the gravitational acceleration acting on newly-added bodies.
     * <p>
     * Whenever a rigid body is added to a space, the body's gravity gets set to
     * that of the space. Thus it makes sense to set the space's vector before
     * adding any bodies to the space.
     *
     * @param gravity the desired acceleration vector (not null, unaffected,
     * default=0,-9.81,0)
     */
    public void setGravity(Vector3f gravity) {
        this.gravity.set(gravity);
        long spaceId = getSpaceId();
        setGravity(spaceId, gravity);
    }

    /**
     * Alter the maximum number of time steps per frame.
     * <p>
     * Extra physics steps help maintain determinism when the render fps drops
     * below 1/accuracy. For example a value of 2 can compensate for frame rates
     * as low as 30fps, assuming the physics has an accuracy of 1/60 sec.
     * <p>
     * Setting this value too high can depress the frame rate.
     *
     * @param steps the desired maximum number of steps (&ge;1) or 0 for a
     * variable time step (default=4)
     */
    public void setMaxSubSteps(int steps) {
        Validate.nonNegative(steps, "steps");
        maxSubSteps = steps;
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
     * Alter the number of iterations used by the contact-and-constraint solver.
     * <p>
     * Use 4 for low quality, 20 for high quality.
     *
     * @param numIterations the desired number of iterations (&ge;1, default=10)
     */
    public void setSolverNumIterations(int numIterations) {
        Validate.positive(numIterations, "number of iterations");

        this.solverNumIterations = numIterations;
        long spaceId = getSpaceId();
        setSolverNumIterations(spaceId, numIterations);
    }

    /**
     * Update this space. Invoked (by the BulletAppState) once per frame while
     * the app state is attached and enabled. Can also be used to single-step
     * the physics simulation, if maxSubSteps is set to 0 or 1.
     *
     * @see #setMaxSubSteps(int)
     * @param timeInterval time-per-frame multiplied by speed (in seconds,
     * &ge;0)
     */
    public void update(float timeInterval) {
        Validate.nonNegative(timeInterval, "time interval");

        long spaceId = getSpaceId();
        assert maxSubSteps >= 0 : maxSubSteps;
        assert accuracy > 0f : accuracy;
        if (maxSubSteps == 0) {
            timeInterval = Math.min(timeInterval, maxTimeStep);
        }
        stepSimulation(spaceId, timeInterval, maxSubSteps, accuracy);
    }

    /**
     * Update this space.
     *
     * @param timeInterval the time interval to simulate (in seconds, &ge;0)
     * @param maxSteps the maximum number of steps of size accuracy (&ge;1) or 0
     * for a single step of size timeInterval
     */
    public void update(float timeInterval, int maxSteps) {
        Validate.nonNegative(timeInterval, "time interval");
        Validate.nonNegative(maxSteps, "max steps");

        long spaceId = getSpaceId();
        assert accuracy > 0f : accuracy;
        stepSimulation(spaceId, timeInterval, maxSteps, accuracy);
    }
    // *************************************************************************
    // new protected methods

    /**
     * Determine the type of the underlying btDynamicsWorld.
     *
     * @param spaceId the Bullet identifier for this space (non-zero)
     * @return 2 (for a discrete world) or 4 (for a soft-rigid world)
     */
    native protected int getWorldType(long spaceId);
    // *************************************************************************
    // CollisionSpace methods

    /**
     * Add the specified object to this space.
     *
     * @param object the PhysicsControl, Spatial-with-PhysicsControl,
     * PhysicsCollisionObject, or PhysicsJoint to add (not null)
     */
    @Override
    public void add(Object object) {
        Validate.nonNull(object, "object");

        if (object instanceof PhysicsControl) {
            ((PhysicsControl) object).setPhysicsSpace(this);
        } else if (object instanceof Spatial) {
            Spatial node = (Spatial) object;
            for (int i = 0; i < node.getNumControls(); ++i) {
                if (node.getControl(i) instanceof PhysicsControl) {
                    add(node.getControl(i));
                }
            }
        } else if (object instanceof PhysicsJoint) {
            addJoint((PhysicsJoint) object);
        } else {
            super.add(object);
        }
    }

    /**
     * Add the specified collision object to this space.
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
     * Test whether the specified collision object is added to this space.
     *
     * @param pco the object to test (not null, unaffected)
     * @return true if currently added, otherwise false
     */
    @Override
    public boolean contains(PhysicsCollisionObject pco) {
        boolean result;
        long pcoId = pco.getObjectId();
        if (pco instanceof PhysicsRigidBody) {
            result = physicsBodies.containsKey(pcoId);
        } else if (pco instanceof PhysicsCharacter) {
            result = physicsCharacters.containsKey(pcoId);
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
        BroadphaseType type = super.getBroadphaseType();
        Vector3f max = super.getWorldMax(null);
        Vector3f min = super.getWorldMin(null);
        long spaceId = createPhysicsSpace(min.x, min.y, min.z,
                max.x, max.y, max.z, type.ordinal());
        assert spaceId != 0L;
        logger.log(Level.FINE, "Created {0}.", this);

        assert getWorldType(spaceId) == 2 // BT_DISCRETE_DYNAMICS_WORLD
                : getWorldType(spaceId);
        initThread(spaceId);
    }

    /**
     * Enumerate collision objects that have been added to this space and not
     * yet removed.
     *
     * @return a new collection of pre-existing instances (not null)
     */
    @Override
    public Collection<PhysicsCollisionObject> getPcoList() {
        Collection<PhysicsCollisionObject> result = super.getPcoList();
        result.addAll(physicsBodies.values());
        result.addAll(physicsCharacters.values());

        return result;
    }

    /**
     * Test whether this space is empty.
     *
     * @return true if empty, otherwise false
     */
    @Override
    public boolean isEmpty() {
        boolean result = super.isEmpty()
                && physicsCharacters.isEmpty()
                && physicsBodies.isEmpty()
                && physicsJoints.isEmpty()
                && physicsVehicles.isEmpty(); // TODO unnecessary

        return result;
    }

    /**
     * Remove the specified object from this space.
     *
     * @param object the PhysicsControl, Spatial-with-PhysicsControl,
     * PhysicsCollisionObject, or PhysicsJoint to remove, or null
     */
    @Override
    public void remove(Object object) {
        if (object == null) {
            return;
        }
        if (object instanceof PhysicsControl) {
            ((PhysicsControl) object).setPhysicsSpace(null);
        } else if (object instanceof Spatial) {
            Spatial node = (Spatial) object;
            for (int i = 0; i < node.getNumControls(); ++i) {
                if (node.getControl(i) instanceof PhysicsControl) {
                    remove((node.getControl(i)));
                }
            }
        } else if (object instanceof PhysicsJoint) {
            removeJoint((PhysicsJoint) object);
        } else {
            super.remove(object);
        }
    }

    /**
     * Remove the specified collision object from this space.
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
    // private Java methods

    /**
     * Add the specified PhysicsCharacter to this space.
     *
     * @param character the character to add (not null, alias created)
     */
    private void addCharacter(PhysicsCharacter character) {
        if (contains(character)) {
            logger.log(Level.WARNING, "{0} is already added to {1}.",
                    new Object[]{character, this});
            return;
        }

        logger.log(Level.FINE, "Adding {0} to {1}.",
                new Object[]{character, this});

        long spaceId = getSpaceId();
        long characterId = character.getObjectId();
        physicsCharacters.put(characterId, character);
        addCharacterObject(spaceId, characterId);

        long actionId = character.getControllerId();
        addAction(spaceId, actionId);
    }

    /**
     * This method is invoked from native code.
     */
    private void addCollisionEvent_native(PhysicsCollisionObject pcoA,
            PhysicsCollisionObject pcoB, long manifoldPointId) {
        if (!collisionListeners.isEmpty()) {
            PhysicsCollisionEvent event
                    = new PhysicsCollisionEvent(pcoA, pcoB, manifoldPointId);
            collisionEvents.add(event);
        }
    }

    /**
     * Add the specified PhysicsJoint to this space.
     *
     * @param joint the joint to add (not null, alias created)
     */
    private void addJoint(PhysicsJoint joint) {
        if (contains(joint)) {
            logger.log(Level.WARNING, "{0} is already added to {1}.",
                    new Object[]{joint, this});
            return;
        }
        /*
         * Warn if the jointed bodies aren't already added to this space.
         */
        PhysicsBody a = joint.getBody(JointEnd.A);
        if (a != null && !contains(a)) {
            logger.log(Level.WARNING,
                    "{0} at the A end of {1} has not yet been added to {2}.",
                    new Object[]{a, joint, this});
        }
        PhysicsBody b = joint.getBody(JointEnd.B);
        if (b != null && !contains(b)) {
            logger.log(Level.WARNING,
                    "{0} at the B end of {1} has not yet been added to {2}.",
                    new Object[]{b, joint, this});
        }

        logger.log(Level.FINE, "Adding {0} to {1}.", new Object[]{joint, this});
        long jointId = joint.getObjectId();
        physicsJoints.put(jointId, joint);

        if (joint instanceof Constraint) {
            long spaceId = getSpaceId();
            Constraint constr = (Constraint) joint;
            boolean allowCollision = constr.isCollisionBetweenLinkedBodies();
            addConstraintC(spaceId, jointId, !allowCollision);
        }
    }

    /**
     * Add the specified PhysicsRigidBody to this space.
     * <p>
     * NOTE: When a rigid body is added, its gravity gets set to that of the
     * space.
     *
     * @param rigidBody the body to add (not null, alias created)
     */
    private void addRigidBody(PhysicsRigidBody rigidBody) {
        if (contains(rigidBody)) {
            logger.log(Level.WARNING, "{0} is already added to {1}.",
                    new Object[]{rigidBody, this});
            return;
        }

        logger.log(Level.FINE, "Adding {0} to {1}.",
                new Object[]{rigidBody, this});
        long rigidBodyId = rigidBody.getObjectId();
        physicsBodies.put(rigidBodyId, rigidBody);

        //Workaround
        //It seems that adding a Kinematic RigidBody to the dynamicWorld
        //prevents it from being dynamic again afterward.
        //So we add it dynamic, then set it kinematic.
        boolean kinematic = false;
        if (rigidBody.isKinematic()) {
            kinematic = true;
            rigidBody.setKinematic(false);
        }
        long spaceId = getSpaceId();
        addRigidBody(spaceId, rigidBodyId);
        if (kinematic) {
            rigidBody.setKinematic(true);
        }

        if (rigidBody instanceof PhysicsVehicle) {
            PhysicsVehicle vehicle = (PhysicsVehicle) rigidBody;
            logger.log(Level.FINE, "Adding action for {0} to {1}.",
                    new Object[]{vehicle, this});

            vehicle.createVehicle(this);
            long actionId = vehicle.getVehicleId();
            physicsVehicles.put(actionId, vehicle);
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

        long spaceId = getSpaceId();
        getGravity(spaceId, storeVector);
        boolean result = gravity.equals(storeVector);

        return result;
    }

    /**
     * Callback invoked just after the physics is stepped.
     * <p>
     * This method is invoked from native code.
     *
     * @param timeStep the time per physics step (in seconds, &ge;0)
     */
    private void postTick_native(float timeStep) {
        for (PhysicsTickListener listener : tickListeners) {
            listener.physicsTick(this, timeStep);
        }
    }

    /**
     * Callback invoked just before the physics is stepped.
     * <p>
     * This method is invoked from native code.
     *
     * @param timeStep the time per physics step (in seconds, &ge;0)
     */
    private void preTick_native(float timeStep) {
        AppTask task;
        while ((task = pQueue.poll()) != null) {
            if (task.isCancelled()) {
                continue;
            }
            try {
                task.invoke();
            } catch (Exception exception) {
                logger.log(Level.SEVERE, null, exception);
            }
        }

        for (PhysicsTickListener listener : tickListeners) {
            listener.prePhysicsTick(this, timeStep);
        }
    }

    /**
     * Remove the specified PhysicsCharacter from this space.
     *
     * @param character the character to remove (not null)
     */
    private void removeCharacter(PhysicsCharacter character) {
        long characterId = character.getObjectId();
        if (!physicsCharacters.containsKey(characterId)) {
            logger.log(Level.WARNING, "{0} does not exist in {1}.",
                    new Object[]{character, this});
            return;
        }

        physicsCharacters.remove(characterId);
        logger.log(Level.FINE, "Removing {0} from {1}.",
                new Object[]{character, this});
        long spaceId = getSpaceId();
        removeAction(spaceId, character.getControllerId());
        removeCharacterObject(spaceId, characterId);
    }

    /**
     * Remove the specified PhysicsJoint from this space.
     *
     * @param joint the joint to remove (not null)
     */
    private void removeJoint(PhysicsJoint joint) {
        long jointId = joint.getObjectId();
        if (!physicsJoints.containsKey(jointId)) {
            logger.log(Level.WARNING, "{0} does not exist in {1}.",
                    new Object[]{joint, this});
            return;
        }
        logger.log(Level.FINE, "Removing {0} from {1}.",
                new Object[]{joint, this});
        physicsJoints.remove(jointId);

        if (joint instanceof Constraint) {
            long spaceId = getSpaceId();
            removeConstraint(spaceId, jointId);
        }
    }

    /**
     * Remove the specified PhysicsRigidBody from this space.
     *
     * @param rigidBody the body to remove (not null)
     */
    private void removeRigidBody(PhysicsRigidBody rigidBody) {
        long rigidBodyId = rigidBody.getObjectId();
        if (!physicsBodies.containsKey(rigidBodyId)) {
            logger.log(Level.WARNING, "{0} does not exist in {1}.",
                    new Object[]{rigidBody, this});
            return;
        }

        long spaceId = getSpaceId();
        if (rigidBody instanceof PhysicsVehicle) {
            PhysicsVehicle vehicle = (PhysicsVehicle) rigidBody;
            long vehicleId = vehicle.getVehicleId();
            logger.log(Level.FINE, "Removing action for {0} from {1}.",
                    new Object[]{vehicle, this});
            physicsVehicles.remove(vehicleId);
            removeAction(spaceId, vehicleId);
        }

        logger.log(Level.FINE, "Removing {0} from {1}.",
                new Object[]{rigidBody, this});
        physicsBodies.remove(rigidBodyId);
        removeRigidBody(spaceId, rigidBodyId);
    }
    // *************************************************************************
    // native methods

    native private void addAction(long spaceId, long actionId);

    native private void addCharacterObject(long spaceId, long characterId);

    native private void addConstraintC(long spaceId, long constraintId,
            boolean collisionBetweenLinkedBodies);

    native private void addRigidBody(long spaceId, long rigidBodyId);

    native private long createPhysicsSpace(float minX, float minY, float minZ,
            float maxX, float maxY, float maxZ, int broadphaseType);

    native private float getGlobalCfm(long spaceId);

    native private void getGravity(long spaceId, Vector3f storeVector);

    native private int getNumConstraints(long spaceId);

    native private void removeAction(long spaceId, long actionId);

    native private void removeCharacterObject(long spaceId, long characterId);

    native private void removeConstraint(long spaceId, long constraintId);

    native private void removeRigidBody(long spaceId, long rigidBodyId);

    native private void setGlobalCfm(long spaceId, float cfm);

    native private void setGravity(long spaceId, Vector3f gravityVector);

    native private void setSolverNumIterations(long spaceId,
            int numIterations);

    native private void stepSimulation(long spaceId, float timeInterval,
            int maxSubSteps, float accuracy);
}
