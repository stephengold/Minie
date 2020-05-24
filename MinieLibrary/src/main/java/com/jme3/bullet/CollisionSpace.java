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

import com.jme3.bullet.collision.PhysicsCollisionGroupListener;
import com.jme3.bullet.collision.PhysicsCollisionListener;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.PhysicsRayTestResult;
import com.jme3.bullet.collision.PhysicsSweepTestResult;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.objects.PhysicsGhostObject;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A Bullet-JME collision space with its own btCollisionWorld.
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
     * type of acceleration structure
     */
    private PhysicsSpace.BroadphaseType broadphaseType
            = PhysicsSpace.BroadphaseType.DBVT;
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
     * flags used in ray tests (default=SubSimplexRaytest)
     */
    private int rayTestFlags = RayTestFlag.SubSimplexRaytest;
    /**
     * map from collision groups to registered group listeners
     */
    final private Map<Integer, PhysicsCollisionGroupListener> collisionGroupListeners
            = new ConcurrentHashMap<>(20);
    /**
     * map ghost IDs to added objects
     */
    final private Map<Long, PhysicsGhostObject> ghostMap
            = new ConcurrentHashMap<>(64);
    /**
     * physics-space reference for each thread
     */
    final protected static ThreadLocal<CollisionSpace> physicsSpaceTL
            = new ThreadLocal<CollisionSpace>();
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
     * @param worldMin the desired minimum coordinates values (not null,
     * unaffected, default=-10k,-10k,-10k)
     * @param worldMax the desired minimum coordinates values (not null,
     * unaffected, default=10k,10k,10k)
     * @param broadphaseType which broadphase collision-detection algorithm to
     * use (not null)
     */
    public CollisionSpace(Vector3f worldMin, Vector3f worldMax,
            PhysicsSpace.BroadphaseType broadphaseType) {
        Validate.finite(worldMin, "world min");
        Validate.finite(worldMax, "world max");
        Validate.nonNull(broadphaseType, "broadphase type");

        this.worldMin.set(worldMin);
        this.worldMax.set(worldMax);
        this.broadphaseType = broadphaseType;
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
     * collision group of this space.
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
        assert !collisionGroupListeners.containsKey(collisionGroup);
        Validate.require(Integer.bitCount(collisionGroup) == 1,
                "exactly one bit set");

        collisionGroupListeners.put(collisionGroup, listener);
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
     * Perform a contact test.
     *
     * @param pco the collision object to test (not null, unaffected)
     * @param listener the callback for reporting contacts (may be null)
     * @return the number of times the listener was invoked, or would have been
     * if it weren't null (&ge;0)
     */
    public int contactTest(PhysicsCollisionObject pco,
            PhysicsCollisionListener listener) {
        Validate.nonNull(pco, "collision object");

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
        int count = collisionGroupListeners.size();
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
     * physics, this can be invoked from the OpenGL thread.
     *
     * @return the pre-existing CollisionSpace running on this thread
     */
    public static CollisionSpace getCollisionSpace() {
        return physicsSpaceTL.get();
    }

    /**
     * Enumerate ghost objects that have been added to this space and not yet
     * removed.
     *
     * @return a new collection of pre-existing instances (not null)
     */
    public Collection<PhysicsGhostObject> getGhostObjectList() {
        // TODO use Collections.unmodifiableCollection
        return new TreeSet<>(ghostMap.values());
    }

    /**
     * Enumerate collision objects that have been added to this space and not
     * yet removed.
     *
     * @return a new modifiable collection of pre-existing instances (not null)
     */
    public Collection<PhysicsCollisionObject> getPcoList() {
        Set<PhysicsCollisionObject> result = new TreeSet<>();
        result.addAll(ghostMap.values());

        return result;
    }

    /**
     * Read the flags used in ray tests (native field: m_flags).
     *
     * @return which flags are used
     * @see com.jme3.bullet.RayTestFlag
     */
    public int getRayTestFlags() {
        return rayTestFlags;
    }

    /**
     * Read the identifier of the native object. For compatibility with the
     * jme3-bullet library.
     *
     * @return the ID (not zero)
     */
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
        if (storeResult == null) {
            return worldMax.clone();
        } else {
            return storeResult.set(worldMax);
        }
    }

    /**
     * Copy the minimum coordinate values for this space.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the minimum coordinates (either storeResult or a new vector, not
     * null)
     */
    public Vector3f getWorldMin(Vector3f storeResult) {
        if (storeResult == null) {
            return worldMin.clone();
        } else {
            return storeResult.set(worldMin);
        }
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
     * Perform a ray-collision test and sort the results by ascending
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
     * Perform a ray-collision test and sort the results by ascending
     * hitFraction.
     *
     * @param from the starting location (in physics-space coordinates, not
     * null, unaffected)
     * @param to the ending location (in physics-space coordinates, not null,
     * unaffected)
     * @param results the list to hold results (not null, modified)
     * @return sorted results
     */
    public List<PhysicsRayTestResult> rayTest(Vector3f from, Vector3f to,
            List<PhysicsRayTestResult> results) {
        results.clear();
        long spaceId = nativeId();
        rayTest_native(from, to, spaceId, results, rayTestFlags);

        Collections.sort(results, hitFractionComparator);
        return results;
    }

    /**
     * Perform a ray-collision test and return the results in arbitrary order.
     *
     * @param from the starting location (in physics-space coordinates, not
     * null, unaffected)
     * @param to the ending location (in physics-space coordinates, not null,
     * unaffected)
     * @return a new list of unsorted results (not null)
     */
    public List rayTestRaw(Vector3f from, Vector3f to) {
        List<PhysicsRayTestResult> results = new ArrayList<>(10);
        rayTestRaw(from, to, results);

        return results;
    }

    /**
     * Perform a ray-collision test and return the results as a list of
     * PhysicsRayTestResults in arbitrary order.
     *
     * @param from the starting location (in physics-space coordinates, not
     * null, unaffected)
     * @param to the ending location (in physics-space coordinates, not null,
     * unaffected)
     * @param results the list to hold results (not null, modified)
     * @return unsorted results
     */
    public List<PhysicsRayTestResult> rayTestRaw(Vector3f from, Vector3f to,
            List<PhysicsRayTestResult> results) {
        results.clear();
        long spaceId = nativeId();
        rayTest_native(from, to, spaceId, results, rayTestFlags);

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
     * De-register the specified collision-group listener.
     *
     * @see
     * #addCollisionGroupListener(com.jme3.bullet.collision.PhysicsCollisionGroupListener,
     * int)
     * @param collisionGroup the group of the listener to de-register (bitmask
     * with exactly one bit set)
     */
    public void removeCollisionGroupListener(int collisionGroup) {
        assert collisionGroupListeners.containsKey(collisionGroup);
        Validate.require(Integer.bitCount(collisionGroup) == 1,
                "exactly one bit set");

        collisionGroupListeners.remove(collisionGroup);
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
     * Used internally
     *
     * @param space which space to simulate on the current thread
     */
    static void setLocalThreadPhysicsSpace(CollisionSpace space) {
        physicsSpaceTL.set(space);
    }

    /**
     * Alter the m_flags used in ray tests.
     *
     * @param flags the desired flags, ORed together (default=SubSimplexRaytest)
     * @see com.jme3.bullet.RayTestFlag
     */
    public void setRayTestFlags(int flags) {
        rayTestFlags = flags;
    }

    /**
     * For compatibility with the jme3-bullet library.
     *
     * @param shape the shape to sweep (not null, convex, unaffected) TODO
     * declare as ConvexShape
     * @param start the starting physics-space transform (not null, unaffected)
     * @param end the ending physics-space transform (not null, unaffected)
     * @return a new list of results
     */
    public List<PhysicsSweepTestResult> sweepTest(CollisionShape shape,
            Transform start, Transform end) {
        List<PhysicsSweepTestResult> results = new LinkedList<>();
        sweepTest(shape, start, end, results);
        return results;
    }

    /**
     * For compatibility with the jme3-bullet library.
     *
     * @param shape the shape to sweep (not null, convex, unaffected) TODO
     * declare as ConvexShape
     * @param start the starting physics-space transform (not null, unaffected)
     * @param end the ending physics-space transform (not null, unaffected)
     * @param results the list to hold results (not null, modified)
     * @return results
     */
    public List<PhysicsSweepTestResult> sweepTest(CollisionShape shape,
            Transform start, Transform end,
            List<PhysicsSweepTestResult> results) {
        return sweepTest(shape, start, end, results, 0f);
    }

    /**
     * Perform a sweep-collision test and store the results in an existing list.
     * <p>
     * The starting and ending locations must be at least 0.4 physics-space
     * units apart.
     * <p>
     * A sweep test will miss a collision if it starts inside an object and
     * sweeps away from the object's center.
     *
     * @param shape the shape to sweep (not null, convex, unaffected) TODO
     * declare as ConvexShape
     * @param start the starting physics-space transform (not null, unaffected)
     * @param end the ending physics-space transform (not null, unaffected)
     * @param results the list to hold results (not null, modified)
     * @param allowedCcdPenetration (in physics-space units)
     * @return results
     */
    public List<PhysicsSweepTestResult> sweepTest(CollisionShape shape,
            Transform start, Transform end,
            List<PhysicsSweepTestResult> results, float allowedCcdPenetration) {
        Validate.nonNull(start, "start");
        Validate.nonNull(end, "end");
        Validate.nonNull(results, "results");
        assert shape.isConvex();

        long shapeId = shape.nativeId();
        results.clear();
        long spaceId = nativeId();
        sweepTest_native(shapeId, start, end, spaceId, results,
                allowedCcdPenetration);

        return results;
    }
    // *************************************************************************
    // new protected methods

    /**
     * Must be invoked on the designated physics thread.
     */
    protected void create() {
        long spaceId = createCollisionSpace(worldMin.x, worldMin.y, worldMin.z,
                worldMax.x, worldMax.y, worldMax.z, broadphaseType.ordinal());
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
    // NativePhysicsObject methods

    /**
     * Finalize this space just before it is destroyed. Should be invoked only
     * by a subclass or by the garbage collector.
     *
     * @throws Throwable ignored by the garbage collector
     */
    @Override
    protected void finalize() throws Throwable {
        try {
            loggerC.log(Level.FINE, "Finalizing {0}.", this);
            for (PhysicsCollisionObject pco : getPcoList()) {
                removeCollisionObject(pco);
            }
            long spaceId = nativeId();
            finalizeNative(spaceId);
        } finally {
            super.finalize();
        }
    }
    // *************************************************************************
    // private Java methods

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

        loggerC.log(Level.FINE, "Adding {0} to {1}.",
                new Object[]{ghost, this});

        long ghostId = ghost.nativeId();
        ghostMap.put(ghostId, ghost);

        long spaceId = nativeId();
        addCollisionObject(spaceId, ghostId);
    }

    /**
     * This method is invoked from native code.
     */
    private boolean notifyCollisionGroupListeners_native(
            PhysicsCollisionObject pcoA, PhysicsCollisionObject pcoB) {
        PhysicsCollisionGroupListener listenerA
                = collisionGroupListeners.get(pcoA.getCollisionGroup());
        PhysicsCollisionGroupListener listenerB
                = collisionGroupListeners.get(pcoB.getCollisionGroup());
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
        loggerC.log(Level.FINE, "Removing {0} from {1}.",
                new Object[]{ghost, this});

        long spaceId = nativeId();
        removeCollisionObject(spaceId, ghostId);
    }
    // *************************************************************************
    // native methods

    native private void addCollisionObject(long spaceId, long pcoId);

    native private int contactTest(long spaceId, long pcoId,
            PhysicsCollisionListener listener);

    native private long createCollisionSpace(float minX, float minY, float minZ,
            float maxX, float maxY, float maxZ, int broadphaseType);

    native private void finalizeNative(long spaceId);

    native private int getNumCollisionObjects(long spaceId);

    native private void rayTest_native(Vector3f fromLocation,
            Vector3f toLocation, long spaceId,
            List<PhysicsRayTestResult> addToList, int flags);

    native private void removeCollisionObject(long spaceId, long pcoId);

    native private void sweepTest_native(long shapeId, Transform from,
            Transform to, long spaceId, List<PhysicsSweepTestResult> addToList,
            float allowedCcdPenetration);
}
