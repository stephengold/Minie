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

import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.math.Vector3f;
import java.util.Collection;
import java.util.Map;
import java.util.TreeSet;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * A Bullet-JME physics space with its own btSoftRigidDynamicsWorld.
 *
 * @author dokthar
 */
public class PhysicsSoftSpace extends PhysicsSpace {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(PhysicsSoftSpace.class.getName());
    // *************************************************************************
    // fields

    /**
     * map soft-body IDs to added objects
     */
    final private Map<Long, PhysicsSoftBody> softBodiesAdded
            = new ConcurrentHashMap<>(64);
    /**
     * parameters applied when soft bodies are added to this space
     */
    final private SoftBodyWorldInfo worldInfo;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a PhysicsSoftSpace. Must be invoked on the designated physics
     * thread.
     *
     * @param worldMin the desired minimum coordinates values (not null,
     * unaffected, default=-10k,-10k,-10k)
     * @param worldMax the desired minimum coordinates values (not null,
     * unaffected, default=10k,10k,10k)
     * @param broadphaseType which broadphase collision-detection algorithm to
     * use (not null)
     */
    public PhysicsSoftSpace(Vector3f worldMin, Vector3f worldMax,
            BroadphaseType broadphaseType) {
        super(worldMin, worldMax, broadphaseType);

        long spaceId = getSpaceId();
        long worldInfoId = getWorldInfo(spaceId);
        worldInfo = new SoftBodyWorldInfo(worldInfoId);
        /*
         * Make sure the same gravity is applied to both hard bodies
         * and soft ones.
         */
        Vector3f gravity = super.getGravity(null);
        worldInfo.setGravity(gravity);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Count the soft bodies in this space.
     *
     * @return count (&ge;0)
     */
    public int countSoftBodies() {
        int count = softBodiesAdded.size();
        return count;
    }

    /**
     * Enumerate soft bodies that have been added to this space and not yet
     * removed.
     *
     * @return a new collection of pre-existing instances (not null)
     */
    public Collection<PhysicsSoftBody> getSoftBodyList() {
        return new TreeSet<>(softBodiesAdded.values());
    }

    /**
     * Access the PhysicsSoftSpace <b>running on this thread</b>. For parallel
     * physics, this can be invoked from the OpenGL thread.
     *
     * @return the pre-existing PhysicsSoftSpace running on this thread
     */
    public static PhysicsSoftSpace getSoftSpace() {
        return (PhysicsSoftSpace) physicsSpaceTL.get();
    }

    /**
     * Access the parameters applied when soft bodies are added to this space.
     *
     * @return the pre-existing instance (not null)
     */
    public SoftBodyWorldInfo getWorldInfo() {
        return worldInfo;
    }
    // *************************************************************************
    // PhysicsSpace methods

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
            long pcoId = pco.getObjectId();
            result = softBodiesAdded.containsKey(pcoId);
        } else {
            result = super.contains(pco);
        }

        return result;
    }

    /**
     * Count the joints in this space, including anchors and soft-body joints.
     *
     * @return count (&ge;0)
     */
    @Override
    public int countJoints() {
        int count = physicsJoints.size();
        return count;
    }

    /**
     * Must be invoked on the designated physics thread.
     */
    @Override
    protected void create() {
        long nativeId = createPhysicsSoftSpace(getWorldMin(null),
                getWorldMax(null), getBroadphaseType().ordinal(), false);
        assert nativeId != 0L;
        logger2.log(Level.FINE, "Created {0}.", this);

        assert getWorldType(nativeId) == 4 // BT_SOFT_RIGID_DYNAMICS_WORLD
                : getWorldType(nativeId);
        initThread(nativeId);
    }

    /**
     * Enumerate collision objects that have been added to this space and not
     * yet removed.
     *
     * @return a new collection of pre-existing instances (not null)
     */
    @Override
    @SuppressWarnings("unchecked")
    public Collection<PhysicsCollisionObject> getPcoList() {
        Collection<PhysicsCollisionObject> result = super.getPcoList();
        result.addAll(softBodiesAdded.values());

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
        result = result && softBodiesAdded.isEmpty();

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
     * Whenever a body is added to a space, the body's gravity gets set to that
     * of the space. Thus it makes sense to set the space's gravity before
     * adding any bodies to the space.
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
    // private methods

    /**
     * NOTE: When a soft body is added, its world info gets set to that of the
     * space.
     *
     * @param softBody the body to add (not null, not already in the space)
     */
    private void addSoftBody(PhysicsSoftBody softBody) {
        long softBodyId = softBody.getObjectId();
        if (softBodiesAdded.containsKey(softBodyId)) {
            logger2.log(Level.WARNING, "{0} is already added to {1}.",
                    new Object[]{softBody, this});
            return;
        }

        softBodiesAdded.put(softBodyId, softBody);
        logger2.log(Level.FINE, "Adding {0} to {1}.",
                new Object[]{softBody, this});

        long spaceId = getSpaceId();
        addSoftBody(spaceId, softBodyId);
        softBody.setWorldInfo(getWorldInfo());
    }

    private void removeSoftBody(PhysicsSoftBody softBody) {
        long softBodyId = softBody.getObjectId();
        if (!softBodiesAdded.containsKey(softBodyId)) {
            logger2.log(Level.WARNING, "{0} does not exist in {1}.",
                    new Object[]{softBody, this});
            return;
        }
        logger2.log(Level.FINE, "Removing {0} from {1}.",
                new Object[]{softBody, this});
        softBodiesAdded.remove(softBodyId);
        long spaceId = getSpaceId();
        removeSoftBody(spaceId, softBodyId);
    }
    // *************************************************************************
    // native methods

    native private void addSoftBody(long softSpaceId, long softBodyId);

    native private long createPhysicsSoftSpace(Vector3f minVector,
            Vector3f maxVector, int broadphaseType, boolean threading);

    native private long getWorldInfo(long softSpaceId);

    native private void removeSoftBody(long softSpaceId, long softBodyId);
}
