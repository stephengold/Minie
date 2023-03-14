/*
 * Copyright (c) 2009-2023 jMonkeyEngine
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

import com.jme3.bullet.collision.PhysicsCollisionGroupListener;
import com.jme3.bullet.collision.PhysicsCollisionListener;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.PhysicsRayTestResult;
import com.jme3.bullet.collision.PhysicsSweepTestResult;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.ConvexShape;
import com.jme3.bullet.objects.PhysicsGhostObject;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.simsilica.mathd.Vec3d;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.TreeSet;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A Bullet-JME collision space with its own {@code btCollisionWorld}.
 *
 * @author normenhansen
 */
public class CollisionSpace extends NativePhysicsObject {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger loggerC
            = Logger.getLogger(CollisionSpace.class.getName());
    // *************************************************************************
    // fields

    /**
     * type of acceleration data structure
     */
    final private PhysicsSpace.BroadphaseType broadphaseType;
    /**
     * comparator for raytest results
     */
    final private static Comparator<PhysicsRayTestResult> hitFractionComparator
            = new Comparator<PhysicsRayTestResult>() {
        @Override
        public int compare(PhysicsRayTestResult r1, PhysicsRayTestResult r2) {
            float r1Fraction = r1.getHitFraction();
            float r2Fraction = r2.getHitFraction();
            int result = Float.compare(r1Fraction, r2Fraction);
            return result;
        }
    };
    /**
     * number of contact-and-constraint solvers (&ge;1, &le;64, default=1)
     */
    final private int numSolvers;
    /**
     * options that influence ray tests (bitmask)
     */
    private int rayTestFlags = RayTestFlag.SubSimplexRaytest;
    /**
     * map from collision groups to registered group listeners
     */
    final private Map<Integer, PhysicsCollisionGroupListener> cgListeners
            = new ConcurrentHashMap<>(20);
    /**
     * map ghost IDs to added objects
     */
    final private Map<Long, PhysicsGhostObject> ghostMap
            = new ConcurrentHashMap<>(64);
    /**
     * physics-space reference for each thread
     */
    final private static ThreadLocal<CollisionSpace> physicsSpaceTL
            = new ThreadLocal<>();
    /**
     * copy of maximum coordinate values when using AXIS_SWEEP broadphase
     * algorithms
     */
    final private Vector3f worldMax = new Vector3f(10000f, 10000f, 10000f);
    /**
     * copy of minimum coordinate values when using AXIS_SWEEP broadphase
     * algorithms
     */
    final private Vector3f worldMin = new Vector3f(-10000f, -10000f, -10000f);
    // *************************************************************************
    // constructors

    /**
     * Instantiate a CollisionSpace. Must be invoked on the designated physics
     * thread.
     *
     * @param worldMin the desired minimum coordinate values (not null,
     * unaffected, default=(-10k,-10k,-10k))
     * @param worldMax the desired maximum coordinate values (not null,
     * unaffected, default=(10k,10k,10k))
     * @param broadphaseType which broadphase accelerator to use (not null)
     */
    public CollisionSpace(Vector3f worldMin, Vector3f worldMax,
            PhysicsSpace.BroadphaseType broadphaseType) {
        this(worldMin, worldMax, broadphaseType, 1);
    }

    /**
     * Used internally.
     *
     * @param worldMin the desired minimum coordinate values (not null,
     * unaffected, default=(-10k,-10k,-10k))
     * @param worldMax the desired maximum coordinate values (not null,
     * unaffected, default=(10k,10k,10k))
     * @param broadphaseType which broadphase accelerator to use (not null)
     * @param numSolvers the number of contact-and-constraint solvers in the
     * thread-safe pool (&ge;1, &le;64, default=1)
     */
    protected CollisionSpace(Vector3f worldMin, Vector3f worldMax,
            PhysicsSpace.BroadphaseType broadphaseType, int numSolvers) {
        Validate.finite(worldMin, "world min");
        Validate.finite(worldMax, "world max");
        Validate.nonNull(broadphaseType, "broadphase type");
        Validate.inRange(numSolvers, "number of solvers", 1, 64);

        this.worldMin.set(worldMin);
        this.worldMax.set(worldMax);
        this.broadphaseType = broadphaseType;
        this.numSolvers = numSolvers;
        create();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Add the specified object to this space.
     *
     * @param object the collision object to add (not null)
     */
    public void add(Object object) {
        Validate.nonNull(object, "object");

        if (object instanceof PhysicsCollisionObject) {
            addCollisionObject((PhysicsCollisionObject) object);
        } else {
            String typeName = object.getClass().getCanonicalName();
            String msg = "Cannot add a " + typeName + " to a collision space.";
            throw new IllegalArgumentException(msg);
        }
    }

    /**
     * Register the specified collision-group listener with the specified
     * collision group of this space. For compatibility with the jme3-jbullet
     * library.
     * <p>
     * Such a listener can disable collisions when they occur. There can be only
     * one listener per collision group per space.
     *
     * @param listener the listener to register (not null, alias created)
     * @param collisionGroup which group it should listen for (bitmask with
     * exactly one bit set)
     */
    public void addCollisionGroupListener(
            PhysicsCollisionGroupListener listener, int collisionGroup) {
        Validate.nonNull(listener, "listener");
        assert !cgListeners.containsKey(collisionGroup);
        Validate.require(
                Integer.bitCount(collisionGroup) == 1, "exactly one bit set");

        cgListeners.put(collisionGroup, listener);
    }

    /**
     * Add the specified collision object to this space.
     *
     * @param pco the collision object to add (not null, modified)
     */
    public void addCollisionObject(PhysicsCollisionObject pco) {
        Validate.nonNull(pco, "collision object");

        if (pco instanceof PhysicsGhostObject) {
            addGhostObject((PhysicsGhostObject) pco);
        } else {
            String typeName = pco.getClass().getCanonicalName();
            String msg = "Unknown type of collision object: " + typeName;
            throw new IllegalArgumentException(msg);
        }
    }

    /**
     * Perform a contact test. This will not detect contacts with soft bodies.
     *
     * @param pco the collision object to test (not null, unaffected)
     * @param listener the callback for reporting contacts (may be null)
     * @return the number of times the listener was invoked, or would have been
     * if it weren't null (&ge;0)
     */
    public int contactTest(
            PhysicsCollisionObject pco, PhysicsCollisionListener listener) {
        long spaceId = nativeId();
        long pcoId = pco.nativeId();
        int result = contactTest(spaceId, pcoId, listener);

        return result;
    }

    /**
     * Test whether the specified collision object is added to this space.
     *
     * @param pco the object to test (not null, unaffected)
     * @return true if currently added, otherwise false
     */
    public boolean contains(PhysicsCollisionObject pco) {
        boolean result;
        long pcoId = pco.nativeId();
        if (pco instanceof PhysicsGhostObject) {
            result = ghostMap.containsKey(pcoId);
        } else {
            String typeName = pco.getClass().getCanonicalName();
            String msg = "Unknown type of collision object: " + typeName;
            throw new IllegalArgumentException(msg);
        }

        return result;
    }

    /**
     * Count how many collision-group listeners are registered with this space.
     *
     * @return the count (&ge;0)
     */
    public int countCollisionGroupListeners() {
        int count = cgListeners.size();
        return count;
    }

    /**
     * Count the collision objects in this space.
     *
     * @return the count (&ge;0)
     */
    public int countCollisionObjects() {
        long spaceId = nativeId();
        int count = getNumCollisionObjects(spaceId);

        return count;
    }

    /**
     * Count the solvers.
     *
     * @return the count (&ge;1, &le;64)
     */
    public int countSolvers() {
        return numSolvers;
    }

    /**
     * Remove all collision objects and physics joints.
     */
    public void destroy() {
        for (PhysicsGhostObject character : ghostMap.values()) {
            removeGhostObject(character);
        }
    }

    /**
     * Read the type of acceleration structure used for broadphase collision
     * detection.
     *
     * @return an enum value (not null)
     */
    public PhysicsSpace.BroadphaseType getBroadphaseType() {
        return broadphaseType;
    }

    /**
     * Access the CollisionSpace <b>running on this thread</b>. For parallel
     * physics, this may be invoked from the OpenGL thread.
     *
     * @return the pre-existing CollisionSpace running on this thread
     */
    public static CollisionSpace getCollisionSpace() {
        CollisionSpace result = physicsSpaceTL.get();
        return result;
    }

    /**
     * Enumerate ghost objects that have been added to this space and not yet
     * removed.
     *
     * @return a new unmodifiable collection of pre-existing instances (not
     * null)
     */
    public Collection<PhysicsGhostObject> getGhostObjectList() {
        Collection<PhysicsGhostObject> result = ghostMap.values();
        result = Collections.unmodifiableCollection(result);

        return result;
    }

    /**
     * Enumerate collision objects that have been added to this space and not
     * yet removed.
     *
     * @return a new modifiable collection of pre-existing instances (not null)
     */
    public Collection<PhysicsCollisionObject> getPcoList() {
        Collection<PhysicsCollisionObject> result = new TreeSet<>();
        result.addAll(ghostMap.values());

        return result;
    }

    /**
     * Return the flags used in ray tests (native field: m_flags).
     *
     * @return which flags are used
     * @see com.jme3.bullet.RayTestFlag
     */
    public int getRayTestFlags() {
        return rayTestFlags;
    }

    /**
     * Return the ID of the native object.
     *
     * @return the native identifier (not zero)
     * @deprecated use {@link NativePhysicsObject#nativeId()}
     */
    @Deprecated
    final public long getSpaceId() {
        long spaceId = nativeId();
        return spaceId;
    }

    /**
     * Copy the maximum coordinate values for this space.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the maximum coordinates (either storeResult or a new vector, not
     * null)
     */
    public Vector3f getWorldMax(Vector3f storeResult) {
        Vector3f result;
        if (storeResult == null) {
            result = worldMax.clone();
        } else {
            result = storeResult.set(worldMax);
        }

        return result;
    }

    /**
     * Copy the minimum coordinate values for this space.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the minimum coordinates (either storeResult or a new vector, not
     * null)
     */
    public Vector3f getWorldMin(Vector3f storeResult) {
        Vector3f result;
        if (storeResult == null) {
            result = worldMin.clone();
        } else {
            result = storeResult.set(worldMin);
        }

        return result;
    }

    /**
     * Test whether a nontrivial closest-points algorithm is registered for the
     * specified collision shapes.
     *
     * @param shape0 the first shape to test (not null, unaffected)
     * @param shape1 the 2nd shape to test (not null, unaffected)
     * @return true if registered, otherwise false
     */
    public boolean hasClosest(CollisionShape shape0, CollisionShape shape1) {
        long spaceId = nativeId();
        int shape0Type = shape0.getShapeType();
        int shape1Type = shape1.getShapeType();
        boolean result = hasClosest(spaceId, shape0Type, shape1Type);

        return result;
    }

    /**
     * Test whether a nontrivial contact-points algorithm is registered for the
     * specified collision shapes.
     *
     * @param shape0 the first shape to test (not null, unaffected)
     * @param shape1 the 2nd shape to test (not null, unaffected)
     * @return true if registered, otherwise false
     */
    public boolean hasContact(CollisionShape shape0, CollisionShape shape1) {
        long spaceId = nativeId();
        int shape0Type = shape0.getShapeType();
        int shape1Type = shape1.getShapeType();
        boolean result = hasContact(spaceId, shape0Type, shape1Type);

        return result;
    }

    /**
     * Test whether this space is empty.
     *
     * @return true if empty, otherwise false
     */
    public boolean isEmpty() {
        boolean result = ghostMap.isEmpty();
        return result;
    }

    /**
     * Test whether the bounding boxes of inactive collision objects should be
     * recomputed during each {@code update()} (native field:
     * m_forceUpdateAllAabbs).
     *
     * @return true to recompute all AABBs, false to skip inactive objects
     */
    public boolean isForceUpdateAllAabbs() {
        long spaceId = nativeId();
        return isForceUpdateAllAabbs(spaceId);
    }

    /**
     * Test whether the "deterministic overlapping pairs" option is enabled in
     * the collision dispatcher (native field: m_deterministicOverlappingPairs).
     *
     * @return true if using sorted/deterministic order, otherwise false
     */
    public boolean isUsingDeterministicDispatch() {
        long spaceId = nativeId();
        boolean result = getDeterministicOverlappingPairs(spaceId);

        return result;
    }

    /**
     * Return the address of the JNIEnv that this space uses for callbacks. For
     * debugging and testing.
     *
     * @return the virtual address of the (native) object (not zero)
     */
    public long jniEnvId() {
        long spaceId = nativeId();
        long result = getJniEnvId(spaceId);

        return result;
    }

    /**
     * Callback to determine whether the specified objects should be allowed to
     * collide. Invoked during broadphase, after axis-aligned bounding boxes,
     * ignore lists, and collision groups have been checked. Override this
     * method to implement dynamic collision filtering.
     *
     * @param pcoA the first collision object (not null)
     * @param pcoB the 2nd collision object (not null)
     * @return true to simulate collisions between pcoA and pcoB, false to
     * ignore such collisions during this timestep
     */
    public boolean needsCollision(
            PhysicsCollisionObject pcoA, PhysicsCollisionObject pcoB) {
        PhysicsCollisionGroupListener listenerA
                = cgListeners.get(pcoA.getCollisionGroup());
        PhysicsCollisionGroupListener listenerB
                = cgListeners.get(pcoB.getCollisionGroup());
        boolean result = true;

        if (listenerA != null) {
            result = listenerA.collide(pcoA, pcoB);
        }
        if (listenerB != null
                && pcoA.getCollisionGroup() != pcoB.getCollisionGroup()) {
            result = listenerB.collide(pcoA, pcoB) && result;
        }

        return result;
    }

    /**
     * Perform a pair test. The collision objects need not be added to the
     * space.
     *
     * @param pcoA the first collision object to test (not null, unaffected)
     * @param pcoB the 2nd collision object to test (not null, unaffected)
     * @param listener the callback for reporting contacts (may be null)
     * @return the number of times the listener was invoked, or would have been
     * if it weren't null (&ge;0)
     */
    public int pairTest(PhysicsCollisionObject pcoA,
            PhysicsCollisionObject pcoB, PhysicsCollisionListener listener) {
        long spaceId = nativeId();
        long aId = pcoA.nativeId();
        long bId = pcoB.nativeId();
        int result = pairTest(spaceId, aId, bId, listener);

        return result;
    }

    /**
     * Perform a ray-collision test (raycast) and sort the results by ascending
     * hitFraction.
     *
     * @param from the starting location (physics-space coordinates, not null,
     * unaffected)
     * @param to the ending location (in physics-space coordinates, not null,
     * unaffected)
     * @return a new list of sorted results (not null)
     */
    public List<PhysicsRayTestResult> rayTest(Vector3f from, Vector3f to) {
        List<PhysicsRayTestResult> results = new ArrayList<>(10);
        rayTest(from, to, results);

        return results;
    }

    /**
     * Perform a ray-collision test (raycast) and sort the results by ascending
     * hitFraction.
     *
     * @param from the starting location (in physics-space coordinates, not
     * null, unaffected)
     * @param to the ending location (in physics-space coordinates, not null,
     * unaffected)
     * @param results the list to hold results (not null, modified)
     * @return results (sorted)
     */
    public List<PhysicsRayTestResult> rayTest(
            Vector3f from, Vector3f to, List<PhysicsRayTestResult> results) {
        results.clear();
        long spaceId = nativeId();
        rayTestNative(from, to, spaceId, results, rayTestFlags);

        results.sort(hitFractionComparator);
        return results;
    }

    /**
     * Perform a ray-collision test (raycast) and sort the results by ascending
     * hitFraction.
     *
     * @param from the starting location (in physics-space coordinates, not
     * null, unaffected)
     * @param to the ending location (in physics-space coordinates, not null,
     * unaffected)
     * @param results the list to hold results (not null, modified)
     * @return results (sorted)
     */
    public List<PhysicsRayTestResult> rayTestDp(
            Vec3d from, Vec3d to, List<PhysicsRayTestResult> results) {
        results.clear();
        long spaceId = nativeId();
        rayTestNativeDp(from, to, spaceId, results, rayTestFlags);

        results.sort(hitFractionComparator);
        return results;
    }

    /**
     * Perform a ray-collision test (raycast) and return the results in
     * arbitrary order.
     *
     * @param from the starting location (in physics-space coordinates, not
     * null, unaffected)
     * @param to the ending location (in physics-space coordinates, not null,
     * unaffected)
     * @return a new list of unsorted results (not null)
     */
    public List<PhysicsRayTestResult> rayTestRaw(Vector3f from, Vector3f to) {
        List<PhysicsRayTestResult> results = new ArrayList<>(10);
        rayTestRaw(from, to, results);

        return results;
    }

    /**
     * Perform a ray-collision test (raycast) and return the results in
     * arbitrary order.
     *
     * @param from the starting location (in physics-space coordinates, not
     * null, unaffected)
     * @param to the ending location (in physics-space coordinates, not null,
     * unaffected)
     * @param results the list to hold results (not null, modified)
     * @return results (unsorted)
     */
    public List<PhysicsRayTestResult> rayTestRaw(
            Vector3f from, Vector3f to, List<PhysicsRayTestResult> results) {
        results.clear();
        long spaceId = nativeId();
        rayTestNative(from, to, spaceId, results, rayTestFlags);

        return results;
    }

    /**
     * Remove the specified object from this space.
     *
     * @param object the collision object to remove, or null
     */
    public void remove(Object object) {
        if (object == null) {
            return;
        }
        if (object instanceof PhysicsCollisionObject) {
            removeCollisionObject((PhysicsCollisionObject) object);
        } else {
            String typeName = object.getClass().getCanonicalName();
            String msg = "Cannot remove a " + typeName
                    + " from a collision space.";
            throw new IllegalArgumentException(msg);
        }
    }

    /**
     * De-register the specified collision-group listener. For compatibility
     * with the jme3-jbullet library.
     *
     * @see #addCollisionGroupListener(
     * com.jme3.bullet.collision.PhysicsCollisionGroupListener, int)
     * @param collisionGroup the group of the listener to de-register (bitmask
     * with exactly one bit set)
     */
    public void removeCollisionGroupListener(int collisionGroup) {
        assert cgListeners.containsKey(collisionGroup);
        Validate.require(
                Integer.bitCount(collisionGroup) == 1, "exactly one bit set");

        cgListeners.remove(collisionGroup);
    }

    /**
     * Remove the specified collision object from this space.
     *
     * @param pco the collision object to remove (not null, modified)
     */
    public void removeCollisionObject(PhysicsCollisionObject pco) {
        Validate.nonNull(pco, "collision object");

        if (pco instanceof PhysicsGhostObject) {
            removeGhostObject((PhysicsGhostObject) pco);
        } else {
            String typeName = pco.getClass().getCanonicalName();
            String msg = "Unknown type of collision object: " + typeName;
            throw new IllegalArgumentException(msg);
        }
    }

    /**
     * Alter whether the bounding boxes of inactive collision objects should be
     * recomputed during each {@code update()} (native field:
     * m_forceUpdateAllAabbs).
     *
     * @param desiredSetting true &rarr; recompute all AABBs, false &rarr; skip
     * inactive objects (default=true)
     */
    public void setForceUpdateAllAabbs(boolean desiredSetting) {
        long spaceId = nativeId();
        setForceUpdateAllAabbs(spaceId, desiredSetting);
    }

    /**
     * Used internally.
     *
     * @param space which space to simulate on the current thread
     */
    static void setLocalThreadPhysicsSpace(CollisionSpace space) {
        physicsSpaceTL.set(space);
    }

    /**
     * Alter the flags used in ray tests (native field: m_flags).
     *
     * @param flags the desired flags, ORed together (default=SubSimplexRaytest)
     * @see com.jme3.bullet.RayTestFlag
     */
    public void setRayTestFlags(int flags) {
        this.rayTestFlags = flags;
    }

    /**
     * For compatibility with the jme3-jbullet library.
     *
     * @param shape the shape to sweep (not null, convex, unaffected)
     * @param start the starting physics-space transform (not null, unaffected)
     * @param end the ending physics-space transform (not null, unaffected)
     * @return a new list of results
     */
    public List<PhysicsSweepTestResult> sweepTest(
            ConvexShape shape, Transform start, Transform end) {
        List<PhysicsSweepTestResult> results = new LinkedList<>();
        sweepTest(shape, start, end, results);
        return results;
    }

    /**
     * For compatibility with the jme3-jbullet library.
     *
     * @param shape the shape to sweep (not null, convex, unaffected)
     * @param start the starting physics-space transform (not null, unaffected)
     * @param end the ending physics-space transform (not null, unaffected)
     * @param results the list to hold results (not null, modified)
     * @return results
     */
    public List<PhysicsSweepTestResult> sweepTest(
            ConvexShape shape, Transform start, Transform end,
            List<PhysicsSweepTestResult> results) {
        List<PhysicsSweepTestResult> result
                = sweepTest(shape, start, end, results, 0f);
        return result;
    }

    /**
     * Perform a sweep-collision test and store the results in an existing list,
     * in arbitrary order.
     * <p>
     * The starting and ending locations must be at least 0.4 physics-space
     * units apart.
     * <p>
     * A sweep test will miss a collision if it starts inside an object and
     * sweeps away from the object's center. Also, sweep tests do not detect
     * soft bodies.
     *
     * @param shape the shape to sweep (not null, convex, unaffected)
     * @param start the starting physics-space transform (not null, unaffected)
     * @param end the ending physics-space transform (not null, unaffected)
     * @param results the list to hold results (not null, modified)
     * @param allowedCcdPenetration (in physics-space units)
     * @return results
     */
    public List<PhysicsSweepTestResult> sweepTest(
            ConvexShape shape, Transform start, Transform end,
            List<PhysicsSweepTestResult> results, float allowedCcdPenetration) {
        Validate.nonNull(start, "start");
        Validate.nonNull(end, "end");
        Validate.nonNull(results, "results");

        long shapeId = shape.nativeId();
        long spaceId = nativeId();
        results.clear();
        sweepTestNative(
                shapeId, start, end, spaceId, results, allowedCcdPenetration);

        return results;
    }

    /**
     * Enable or disable the "deterministic overlapping pairs" option in the
     * collision dispatcher (native field: m_deterministicOverlappingPairs).
     *
     * @param desiredSetting true &rarr; sorted/deterministic order, false
     * &rarr; hashed/arbitrary order (default=false)
     */
    public void useDeterministicDispatch(boolean desiredSetting) {
        long spaceId = nativeId();
        setDeterministicOverlappingPairs(spaceId, desiredSetting);
    }
    // *************************************************************************
    // new protected methods

    /**
     * Must be invoked on the designated physics thread.
     */
    protected void create() {
        assert numSolvers == 1 : numSolvers;

        int broadphase = getBroadphaseType().ordinal();
        long spaceId = createCollisionSpace(worldMin.x, worldMin.y, worldMin.z,
                worldMax.x, worldMax.y, worldMax.z, broadphase);
        assert spaceId != 0L;

        initThread(spaceId);
        loggerC.log(Level.FINE, "Created {0}.", this);
    }

    /**
     * Must be invoked on the designated physics thread.
     *
     * @param spaceId the Bullet identifier for this space (non-zero)
     */
    protected void initThread(long spaceId) {
        setNativeId(spaceId);
        physicsSpaceTL.set(this);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Add the specified PhysicsGhostObject to this space.
     *
     * @param ghost the object to add (not null, alias created)
     */
    private void addGhostObject(PhysicsGhostObject ghost) {
        if (contains(ghost)) {
            loggerC.log(Level.WARNING, "{0} is already added to {1}.",
                    new Object[]{ghost, this});
            return;
        }
        assert !ghost.isInWorld();

        if (loggerC.isLoggable(Level.FINE)) {
            loggerC.log(Level.FINE, "Adding {0} to {1}.",
                    new Object[]{ghost, this});
        }

        long ghostId = ghost.nativeId();
        ghostMap.put(ghostId, ghost);

        long spaceId = nativeId();
        addCollisionObject(spaceId, ghostId);
    }

    /**
     * Free the identified tracked native object. Invoked by reflection.
     *
     * @param spaceId the native identifier (not zero)
     */
    private static void freeNativeObject(long spaceId) {
        Validate.nonZero(spaceId, "space ID");
        finalizeNative(spaceId);
    }

    /**
     * This method is invoked by native code to determine whether the specified
     * objects should be allowed to collide. Invoked during broadphase, after
     * axis-aligned bounding boxes, ignore lists, and collision groups have been
     * checked.
     *
     * @param pcoA the first collision object (not null)
     * @param pcoB the 2nd collision object (not null)
     * @return true to simulate collisions between pcoA and pcoB, false to
     * ignore such collisions during this timestep
     */
    private boolean notifyCollisionGroupListeners(
            PhysicsCollisionObject pcoA, PhysicsCollisionObject pcoB) {
        boolean result = needsCollision(pcoA, pcoB);
        return result;
    }

    /**
     * Remove the specified PhysicsGhostObject from this space.
     *
     * @param ghost the object to remove (not null)
     */
    private void removeGhostObject(PhysicsGhostObject ghost) {
        long ghostId = ghost.nativeId();
        if (!ghostMap.containsKey(ghostId)) {
            loggerC.log(Level.WARNING, "{0} does not exist in {1}.",
                    new Object[]{ghost, this});
            return;
        }

        ghostMap.remove(ghostId);
        if (loggerC.isLoggable(Level.FINE)) {
            loggerC.log(Level.FINE, "Removing {0} from {1}.",
                    new Object[]{ghost, this});
        }

        long spaceId = nativeId();
        removeCollisionObject(spaceId, ghostId);
    }
    // *************************************************************************
    // native private methods

    native private static void addCollisionObject(long spaceId, long pcoId);

    native private static int contactTest(
            long spaceId, long pcoId, PhysicsCollisionListener listener);

    native private long createCollisionSpace(float minX, float minY, float minZ,
            float maxX, float maxY, float maxZ, int broadphaseType);

    native private static void finalizeNative(long spaceId);

    native private static boolean
            getDeterministicOverlappingPairs(long spaceId);

    native private static long getJniEnvId(long spaceId);

    native private static int getNumCollisionObjects(long spaceId);

    native private static boolean
            hasClosest(long spaceId, int shape0Type, int shape1Type);

    native private static boolean
            hasContact(long spaceId, int shape0Type, int shape1Type);

    native private static boolean isForceUpdateAllAabbs(long spaceId);

    native private static int pairTest(long spaceId, long aId, long bId,
            PhysicsCollisionListener listener);

    native private static void rayTestNative(
            Vector3f fromLocation, Vector3f toLocation, long spaceId,
            List<PhysicsRayTestResult> addToList, int flags);

    native private static void rayTestNativeDp(
            Vec3d fromLocation, Vec3d toLocation, long spaceId,
            List<PhysicsRayTestResult> addToList, int flags);

    native private static void removeCollisionObject(long spaceId, long pcoId);

    native private static void setDeterministicOverlappingPairs(
            long spaceId, boolean desiredSetting);

    native private static void
            setForceUpdateAllAabbs(long spaceId, boolean desiredSetting);

    native private static void sweepTestNative(long shapeId, Transform from,
            Transform to, long spaceId, List<PhysicsSweepTestResult> addToList,
            float allowedCcdPenetration);
}
