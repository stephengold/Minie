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
package com.jme3.bullet.collision;

import com.jme3.math.Vector3f;
import com.jme3.scene.Spatial;
import java.util.EventObject;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * Describe a collision in the physics world.
 *
 * @author normenhansen
 */
public class PhysicsCollisionEvent extends EventObject {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(PhysicsCollisionEvent.class.getName());
    // *************************************************************************
    // fields

    /**
     * 1st involved object
     */
    final private PhysicsCollisionObject nodeA;
    /**
     * 2nd involved object
     */
    final private PhysicsCollisionObject nodeB;
    /**
     * Bullet identifier of the btManifoldPoint
     */
    private long manifoldPointObjectId = 0L;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a collision event.
     *
     * @param nodeA 1st involved object (not null, alias created)
     * @param nodeB 2nd involved object (not null, alias created)
     * @param manifoldPointId Bullet identifier of the btManifoldPoint (not 0)
     */
    public PhysicsCollisionEvent(PhysicsCollisionObject nodeA,
            PhysicsCollisionObject nodeB, long manifoldPointId) {
        super(nodeA);
        Validate.nonNull(nodeA, "node A");
        Validate.nonNull(nodeB, "node B");
        Validate.nonZero(manifoldPointId, "manifold point id");

        this.nodeA = nodeA;
        this.nodeB = nodeB;
        manifoldPointObjectId = manifoldPointId;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Access the user object of collision object A, provided it's a Spatial.
     *
     * @return the pre-existing Spatial, or null if none
     */
    public Spatial getNodeA() {
        Spatial result = null;
        Object userObject = nodeA.getUserObject();
        if (userObject instanceof Spatial) {
            result = (Spatial) userObject;
        }

        return result;
    }

    /**
     * Access the user object of collision object B, provided it's a Spatial.
     *
     * @return the pre-existing Spatial, or null if none
     */
    public Spatial getNodeB() {
        Spatial result = null;
        Object userObject = nodeB.getUserObject();
        if (userObject instanceof Spatial) {
            result = (Spatial) userObject;
        }

        return result;
    }

    /**
     * Access collision object A.
     *
     * @return the pre-existing object (not null)
     */
    public PhysicsCollisionObject getObjectA() {
        assert nodeA != null;
        return nodeA;
    }

    /**
     * Access collision object B.
     *
     * @return the pre-existing object (not null)
     */
    public PhysicsCollisionObject getObjectB() {
        assert nodeB != null;
        return nodeB;
    }

    /**
     * Read the collision's applied impulse.
     *
     * @return impulse
     */
    public float getAppliedImpulse() {
        return getAppliedImpulse(manifoldPointObjectId);
    }

    /**
     * Read the collision's applied lateral impulse #1.
     *
     * @return impulse
     */
    public float getAppliedImpulseLateral1() {
        return getAppliedImpulseLateral1(manifoldPointObjectId);
    }

    /**
     * Read the collision's applied lateral impulse #2.
     *
     * @return impulse
     */
    public float getAppliedImpulseLateral2() {
        return getAppliedImpulseLateral2(manifoldPointObjectId);
    }

    /**
     * Read the collision's combined friction.
     *
     * @return friction
     */
    public float getCombinedFriction() {
        return getCombinedFriction(manifoldPointObjectId);
    }

    /**
     * Read the collision's combined restitution.
     *
     * @return restitution
     */
    public float getCombinedRestitution() {
        return getCombinedRestitution(manifoldPointObjectId);
    }

    /**
     * Read the collision's distance #1.
     *
     * @return distance
     */
    public float getDistance1() {
        return getDistance1(manifoldPointObjectId);
    }

    /**
     * Read the collision's index 0.
     *
     * @return index
     */
    public int getIndex0() {
        return getIndex0(manifoldPointObjectId);
    }

    /**
     * Read the collision's index 1.
     *
     * @return index
     */
    public int getIndex1() {
        return getIndex1(manifoldPointObjectId);
    }

    /**
     * Copy the collision's lateral friction direction #1.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a direction vector (either storeResult or a new instance)
     */
    public Vector3f getLateralFrictionDir1(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getLateralFrictionDir1(manifoldPointObjectId, result);
        return result;
    }

    /**
     * Copy the collision's lateral friction direction #2.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a direction vector (either storeResult or a new instance)
     */
    public Vector3f getLateralFrictionDir2(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getLateralFrictionDir2(manifoldPointObjectId, result);
        return result;
    }

    /**
     * Test whether the collision's lateral friction is initialized.
     *
     * @return true if initialized, otherwise false
     */
    public boolean isLateralFrictionInitialized() {
        return isLateralFrictionInitialized(manifoldPointObjectId);
    }

    /**
     * Read the collision's lifetime.
     *
     * @return lifetime
     */
    public int getLifeTime() {
        return getLifeTime(manifoldPointObjectId);
    }

    /**
     * Copy the collision's location in the local coordinates of object A.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in local coordinates, either storeResult or a
     * new instance)
     */
    public Vector3f getLocalPointA(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getLocalPointA(manifoldPointObjectId, result);
        return result;
    }

    /**
     * Copy the collision's location in the local coordinates of object B.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in local coordinates, either storeResult or a
     * new instance)
     */
    public Vector3f getLocalPointB(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getLocalPointB(manifoldPointObjectId, result);
        return result;
    }

    /**
     * Copy the collision's normal on object B.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a normal vector (in physics-space coordinates, either storeResult
     * or a new instance)
     */
    public Vector3f getNormalWorldOnB(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getNormalWorldOnB(manifoldPointObjectId, result);
        return result;
    }

    /**
     * Read part identifier 0.
     *
     * @return identifier
     */
    public int getPartId0() {
        return getPartId0(manifoldPointObjectId);
    }

    /**
     * Read part identifier 1.
     *
     * @return identifier
     */
    public int getPartId1() {
        return getPartId1(manifoldPointObjectId);
    }

    /**
     * Copy the collision's location.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in physics-space coordinates, either
     * storeResult or a new instance)
     */
    public Vector3f getPositionWorldOnA(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getPositionWorldOnA(manifoldPointObjectId, result);
        return result;
    }

    /**
     * Copy the collision's location.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in physics-space coordinates, either
     * storeResult or a new instance)
     */
    public Vector3f getPositionWorldOnB(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getPositionWorldOnB(manifoldPointObjectId, result);
        return result;
    }
    // *************************************************************************
    // private methods

    native private float getAppliedImpulse(long manifoldPointObjectId);

    native private float getAppliedImpulseLateral1(long manifoldPointObjectId);

    native private float getAppliedImpulseLateral2(long manifoldPointObjectId);

    native private float getCombinedFriction(long manifoldPointObjectId);

    native private float getCombinedRestitution(long manifoldPointObjectId);

    native private float getDistance1(long manifoldPointObjectId);

    native private int getIndex0(long manifoldPointObjectId);

    native private int getIndex1(long manifoldPointObjectId);

    native private void getLateralFrictionDir1(long manifoldPointObjectId,
            Vector3f lateralFrictionDir1);

    native private void getLateralFrictionDir2(long manifoldPointObjectId,
            Vector3f lateralFrictionDir2);

    native private boolean isLateralFrictionInitialized(
            long manifoldPointObjectId);

    native private int getLifeTime(long manifoldPointObjectId);

    native private void getLocalPointA(long manifoldPointObjectId,
            Vector3f localPointA);

    native private void getLocalPointB(long manifoldPointObjectId,
            Vector3f localPointB);

    native private void getNormalWorldOnB(long manifoldPointObjectId,
            Vector3f normalWorldOnB);

    native private int getPartId0(long manifoldPointObjectId);

    native private int getPartId1(long manifoldPointObjectId);

    native private void getPositionWorldOnA(long manifoldPointObjectId,
            Vector3f positionWorldOnA);

    native private void getPositionWorldOnB(long manifoldPointObjectId,
            Vector3f positionWorldOnB);
}
