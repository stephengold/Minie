/*
 * Copyright (c) 2009-2022 jMonkeyEngine
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

import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.math.Vector3f;
import java.util.Collection;
import java.util.Collections;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * A MultiBodySpace that supports deformables, with its own
 * {@code btDeformableMultiBodyDynamicsWorld}.
 *
 * @author Stephen Gold sgold@sonic.net
 *
 * Based on PhysicsSoftSpace by Dokthar.
 */
public class DeformableSpace extends MultiBodySpace {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger3
            = Logger.getLogger(DeformableSpace.class.getName());
    // *************************************************************************
    // fields

    /**
     * map soft-body IDs to added objects
     */
    final private Map<Long, PhysicsSoftBody> softBodyMap
            = new ConcurrentHashMap<>(64);
    /**
     * parameters applied when soft bodies are added to this space
     */
    final private SoftBodyWorldInfo worldInfo;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a DeformableSpace with the specified contact-and-constraint
     * solver. Must be invoked on the designated physics thread.
     *
     * @param worldMin the desired minimum coordinate value for each axis (not
     * null, unaffected, default=(-10k,-10k,-10k))
     * @param worldMax the desired maximum coordinate value for each axis (not
     * null, unaffected, default=(10k,10k,10k))
     * @param broadphaseType which broadphase accelerator to use (not null)
     * @param solverType the desired constraint solver (not null)
     */
    public DeformableSpace(Vector3f worldMin, Vector3f worldMax,
            BroadphaseType broadphaseType, SolverType solverType) {
        super(worldMin, worldMax, broadphaseType, solverType);

        long spaceId = super.nativeId();
        long worldInfoId = getWorldInfo(spaceId);
        this.worldInfo = new SoftBodyWorldInfo(worldInfoId);

        // Ensure that the same gravity is applied to all bodies.
        Vector3f gravity = super.getGravity(null);
        worldInfo.setGravity(gravity);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Count the soft bodies in this space.
     *
     * @return the count (&ge;0)
     */
    public int countSoftBodies() {
        long spaceId = nativeId();
        int count = getNumSoftBodies(spaceId);

        assert count == softBodyMap.size();
        return count;
    }

    /**
     * Access the DeformableSpace <b>running on this thread</b>. For parallel
     * physics, this may be invoked from the OpenGL thread.
     *
     * @return the pre-existing DeformableSpace running on this thread
     */
    public static DeformableSpace getDeformableSpace() {
        CollisionSpace result = getCollisionSpace();
        return (DeformableSpace) result;
    }

    /**
     * Enumerate soft bodies that have been added to this space and not yet
     * removed.
     *
     * @return a new unmodifiable collection of pre-existing instances (not
     * null)
     */
    public Collection<PhysicsSoftBody> getSoftBodyList() {
        Collection<PhysicsSoftBody> result = softBodyMap.values();
        result = Collections.unmodifiableCollection(result);

        return result;
    }

    /**
     * Access the parameters applied when soft bodies are added to this space.
     *
     * @return the pre-existing instance (not null)
     */
    public SoftBodyWorldInfo getWorldInfo() {
        assert worldInfo != null;
        assert worldInfo.nativeId() == getWorldInfo(nativeId());
        return worldInfo;
    }
    // *************************************************************************
    // MultiBodySpace methods

    /**
     * Add the specified collision object to this space.
     *
     * @param pco the collision object to add (not null)
     */
    @Override
    public void addCollisionObject(PhysicsCollisionObject pco) {
        if (pco instanceof PhysicsSoftBody) {
            addSoftBody((PhysicsSoftBody) pco);
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
        if (pco instanceof PhysicsSoftBody) {
            long pcoId = pco.nativeId();
            result = softBodyMap.containsKey(pcoId);
        } else {
            result = super.contains(pco);
        }

        return result;
    }

    /**
     * Count the joints in this space, including anchors and soft-body joints.
     *
     * @return the count (&ge;0)
     */
    @Override
    public int countJoints() {
        // can't use super.countJoints() because it includes a failing assertion
        Map<Long, PhysicsJoint> map = getJointMap();
        int result = map.size();

        return result;
    }

    /**
     * Must be invoked on the designated physics thread.
     */
    @Override
    protected void create() {
        int numSolvers = countSolvers();
        assert numSolvers == 1 : numSolvers;

        int broadphase = getBroadphaseType().ordinal();
        Vector3f max = getWorldMax(null);
        Vector3f min = getWorldMin(null);
        long nativeId = createSpace(min, max, broadphase);
        assert nativeId != 0L;

        assert getWorldType(nativeId)
                == 7 // BT_DEFORMABLE_MULTIBODY_DYNAMICS_WORLD
                : getWorldType(nativeId);
        initThread(nativeId);
        initSolverInfo();
        logger3.log(Level.FINE, "Created {0}.", this);
    }

    /**
     * Remove all collision objects and physics joints.
     */
    @Override
    public void destroy() {
        super.destroy();

        for (PhysicsSoftBody softBody : softBodyMap.values()) {
            removeSoftBody(softBody);
        }
    }

    /**
     * Enumerate collision objects that have been added to this space and not
     * yet removed.
     *
     * @return a new modifiable collection of pre-existing instances (not null)
     */
    @Override
    public Collection<PhysicsCollisionObject> getPcoList() {
        Collection<PhysicsCollisionObject> result = super.getPcoList();
        result.addAll(softBodyMap.values());

        return result;
    }

    /**
     * Test whether this space is empty.
     *
     * @return true if empty, otherwise false
     */
    @Override
    public boolean isEmpty() {
        boolean result = super.isEmpty();
        result = result && softBodyMap.isEmpty();

        return result;
    }

    /**
     * Remove the specified collision object from this space.
     *
     * @param pco the collision object to remove (not null)
     */
    @Override
    public void removeCollisionObject(PhysicsCollisionObject pco) {
        if (pco instanceof PhysicsSoftBody) {
            removeSoftBody((PhysicsSoftBody) pco);
        } else {
            super.removeCollisionObject(pco);
        }
    }

    /**
     * Alter the gravitational acceleration acting on newly-added bodies.
     * <p>
     * Typically, when a body is added to a space, the body's gravity gets set
     * to that of the space. Thus, it is preferable to set the space's gravity
     * before adding any bodies to the space.
     *
     * @param gravity the desired acceleration vector (not null, unaffected,
     * default=(0,-9.81,0))
     */
    @Override
    public void setGravity(Vector3f gravity) {
        super.setGravity(gravity);
        worldInfo.setGravity(gravity);
    }
    // *************************************************************************
    // Java private methods

    /**
     * NOTE: When a soft body is added, its world info may get replaced with
     * that of the space.
     *
     * @param softBody the body to add (not null, not already in the space)
     */
    private void addSoftBody(PhysicsSoftBody softBody) {
        long softBodyId = softBody.nativeId();
        if (softBodyMap.containsKey(softBodyId)) {
            logger3.log(Level.WARNING, "{0} is already added to {1}.",
                    new Object[]{softBody, this});
            return;
        }
        assert !softBody.isInWorld();

        softBodyMap.put(softBodyId, softBody);
        if (logger3.isLoggable(Level.FINE)) {
            logger3.log(Level.FINE, "Adding {0} to {1}.",
                    new Object[]{softBody, this});
        }

        long spaceId = nativeId();
        addSoftBody(spaceId, softBodyId);

        if (!softBody.isWorldInfoProtected()) { // replace the world info
            softBody.setWorldInfo(getWorldInfo());
        }
    }

    private void removeSoftBody(PhysicsSoftBody softBody) {
        long softBodyId = softBody.nativeId();
        if (!softBodyMap.containsKey(softBodyId)) {
            logger3.log(Level.WARNING, "{0} does not exist in {1}.",
                    new Object[]{softBody, this});
            return;
        }
        if (logger3.isLoggable(Level.FINE)) {
            logger3.log(Level.FINE, "Removing {0} from {1}.",
                    new Object[]{softBody, this});
        }
        softBodyMap.remove(softBodyId);
        long spaceId = nativeId();
        removeSoftBody(spaceId, softBodyId);
    }
    // *************************************************************************
    // native private methods

    native private static void addSoftBody(long spaceId, long softBodyId);

    native private long createSpace(
            Vector3f minVector, Vector3f maxVector, int broadphaseType);

    native private static int getNumSoftBodies(long spaceId);

    native private static long getWorldInfo(long spaceId);

    native private static void removeSoftBody(long spaceId, long softBodyId);
}
