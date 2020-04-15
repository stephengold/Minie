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
 * Describe a collision between 2 collision objects in a PhysicsSpace.
 * <p>
 * Even though this class inherits the java.io.Serializable interface, it isn't
 * serializable.
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
     * unique identifier of the btManifoldPoint
     */
    private long nativeId = 0L;
    /**
     * first collision object involved
     */
    final private PhysicsCollisionObject pcoA;
    /**
     * 2nd collision object involved
     */
    final private PhysicsCollisionObject pcoB;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a collision event.
     *
     * @param pcoA the first involved object (not null, alias created)
     * @param pcoB the 2nd involved object (not null, alias created)
     * @param manifoldPointId the native ID of the btManifoldPoint (not 0)
     */
    public PhysicsCollisionEvent(PhysicsCollisionObject pcoA,
            PhysicsCollisionObject pcoB, long manifoldPointId) {
        super(pcoA);
        Validate.nonNull(pcoA, "object A");
        Validate.nonNull(pcoB, "object B");
        Validate.nonZero(manifoldPointId, "manifold point ID");

        this.pcoA = pcoA;
        this.pcoB = pcoB;
        nativeId = manifoldPointId;
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
        Object userObject = pcoA.getUserObject();
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
        Object userObject = pcoB.getUserObject();
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
        assert pcoA != null;
        return pcoA;
    }

    /**
     * Access collision object B.
     *
     * @return the pre-existing object (not null)
     */
    public PhysicsCollisionObject getObjectB() {
        assert pcoB != null;
        return pcoB;
    }

    /**
     * Read the collision's applied impulse (native field: m_appliedImpulse).
     *
     * @return impulse
     */
    public float getAppliedImpulse() {
        return getAppliedImpulse(nativeId);
    }

    /**
     * Read the collision's applied lateral impulse #1 (native field:
     * m_appliedImpulseLateral1).
     *
     * @return impulse
     */
    public float getAppliedImpulseLateral1() {
        return getAppliedImpulseLateral1(nativeId);
    }

    /**
     * Read the collision's applied lateral impulse #2 (native field:
     * m_appliedImpulseLateral2).
     *
     * @return impulse
     */
    public float getAppliedImpulseLateral2() {
        return getAppliedImpulseLateral2(nativeId);
    }

    /**
     * Read the combined friction, which is the sum of the collision-object
     * frictions (native field: m_combinedFriction).
     *
     * @return the friction sum
     */
    public float getCombinedFriction() {
        return getCombinedFriction(nativeId);
    }

    /**
     * Read the combined restitution, which is the product of the
     * collision-object restitutions (native field: m_combinedRestitution).
     *
     * @return the restitution product
     */
    public float getCombinedRestitution() {
        return getCombinedRestitution(nativeId);
    }

    /**
     * Read the combined rolling friction (native field:
     * m_combinedRollingFriction).
     *
     * @return the combined friction
     */
    public float getCombinedRollingFriction() {
        return getCombinedRollingFriction(nativeId);
    }

    /**
     * Read the combined spinning friction (native field:
     * m_combinedSpinningFriction).
     *
     * @return the combined friction
     */
    public float getCombinedSpinningFriction() {
        return getCombinedSpinningFriction(nativeId);
    }

    /**
     * Read the collision's distance #1 (native field: m_distance1).
     *
     * @return the distance (in physics-space units)
     */
    public float getDistance1() {
        return getDistance1(nativeId);
    }

    /**
     * Read the contact-point flags (native field: m_contactPointFlags).
     *
     * @return a bitmask
     */
    public int getFlags() {
        return getFlags(nativeId);
    }

    /**
     * Read the triangle index from the shape of collision object A at the point
     * of contact (native field: m_index0).
     * <p>
     * If shape is convex, the index is undefined.
     * <p>
     * If shape is a CompoundCollisionShape, the index identifies a child shape.
     * <p>
     * If the shape is a GImpactCollisionShape or MeshCollisionShape, the index
     * identifies a triangle in an IndexedMesh.
     * <p>
     * If the shape is a HeightfieldCollisionShape, the index indicates a grid
     * column.
     *
     * @return the index of the collision-shape triangle (&ge;0) or -1 if
     * undefined
     */
    public int getIndex0() {
        return getIndex0(nativeId);
    }

    /**
     * Read the triangle index from the shape of collision object B at the point
     * of contact (native field: m_index1).
     * <p>
     * If shape is convex, the index is undefined.
     * <p>
     * If shape is a CompoundCollisionShape, the index identifies a child shape.
     * <p>
     * If the shape is a GImpactCollisionShape or MeshCollisionShape, the index
     * identifies a triangle in an IndexedMesh.
     * <p>
     * If the shape is a HeightfieldCollisionShape, the index indicates a grid
     * column.
     *
     * @return the index of the collision-shape triangle (&ge;0) or -1 if
     * undefined
     */
    public int getIndex1() {
        return getIndex1(nativeId);
    }

    /**
     * Copy the collision's lateral friction direction #1 (native field:
     * m_lateralFrictionDir1).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a direction vector (either storeResult or a new instance)
     */
    public Vector3f getLateralFrictionDir1(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getLateralFrictionDir1(nativeId, result);
        return result;
    }

    /**
     * Copy the collision's lateral friction direction #2 (native field:
     * m_lateralFrictionDir2).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a direction vector (either storeResult or a new instance)
     */
    public Vector3f getLateralFrictionDir2(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getLateralFrictionDir2(nativeId, result);
        return result;
    }

    /**
     * Read the collision's lifetime (native name: m_lifeTime).
     *
     * @return the duration (in ticks, &ge;0)
     */
    public int getLifeTime() {
        return getLifeTime(nativeId);
    }

    /**
     * Copy the collision's location in the local coordinates of object A
     * (native name: m_localPointA).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in local coordinates, either storeResult or a
     * new instance)
     */
    public Vector3f getLocalPointA(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getLocalPointA(nativeId, result);
        return result;
    }

    /**
     * Copy the collision's location in the local coordinates of object B
     * (native name: m_localPointB).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in local coordinates, either storeResult or a
     * new instance)
     */
    public Vector3f getLocalPointB(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getLocalPointB(nativeId, result);
        return result;
    }

    /**
     * Copy the collision's normal on object B (native name: m_normalWorldOnB).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a normal vector (in physics-space coordinates, either storeResult
     * or a new instance)
     */
    public Vector3f getNormalWorldOnB(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getNormalWorldOnB(nativeId, result);
        return result;
    }

    /**
     * Read the part index from the shape of collision object A at the point of
     * contact (native field: m_partId0).
     * <p>
     * If the shape is compound or convex, the index is undefined.
     * <p>
     * If the shape is a GImpactCollisionShape or MeshCollisionShape, the index
     * identifies an IndexedMesh.
     * <p>
     * If the shape is a HeightfieldCollisionShape, the index identifies a grid
     * row.
     *
     * @return the index of the collision-shape part (&ge;0) or -1 if undefined
     */
    public int getPartId0() {
        return getPartId0(nativeId);
    }

    /**
     * Read the part index from the shape of collision object B at the point of
     * contact (native field: m_partId1).
     * <p>
     * If the shape is compound or convex, the index is undefined.
     * <p>
     * If the shape is a GImpactCollisionShape or MeshCollisionShape, the index
     * identifies an IndexedMesh.
     * <p>
     * If the shape is a HeightfieldCollisionShape, the index identifies a grid
     * row.
     *
     * @return the index of the collision-shape part (&ge;0) or -1 if undefined
     */
    public int getPartId1() {
        return getPartId1(nativeId);
    }

    /**
     * Copy the collision's location (native field: m_positionWorldOnA).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in physics-space coordinates, either
     * storeResult or a new instance)
     */
    public Vector3f getPositionWorldOnA(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getPositionWorldOnA(nativeId, result);
        return result;
    }

    /**
     * Copy the collision's location (native field: m_positionWorldOnB).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in physics-space coordinates, either
     * storeResult or a new instance)
     */
    public Vector3f getPositionWorldOnB(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getPositionWorldOnB(nativeId, result);
        return result;
    }

    /**
     * Test whether the collision's lateral friction is initialized. For
     * compatibility with jme3-bullet.
     *
     * @return true if initialized, otherwise false
     */
    public boolean isLateralFrictionInitialized() {
        int flags = getFlags();
        boolean result = (flags & ContactPointFlag.LATERAL_FRICTION) != 0x0;

        return result;
    }
    // *************************************************************************
    // native methods

    native private float getAppliedImpulse(long manifoldPointId);

    native private float getAppliedImpulseLateral1(long manifoldPointId);

    native private float getAppliedImpulseLateral2(long manifoldPointId);

    native private float getCombinedFriction(long manifoldPointId);

    native private float getCombinedRestitution(long manifoldPointId);

    native private float getCombinedRollingFriction(long manifoldPointId);

    native private float getCombinedSpinningFriction(long manifoldPointId);

    native private float getDistance1(long manifoldPointId);

    native private int getFlags(long manifoldPointId);

    native private int getIndex0(long manifoldPointId);

    native private int getIndex1(long manifoldPointId);

    native private void getLateralFrictionDir1(long manifoldPointId,
            Vector3f storeVector);

    native private void getLateralFrictionDir2(long manifoldPointId,
            Vector3f storeVector);

    native private int getLifeTime(long manifoldPointId);

    native private void getLocalPointA(long manifoldPointId,
            Vector3f storeVector);

    native private void getLocalPointB(long manifoldPointId,
            Vector3f storeVector);

    native private void getNormalWorldOnB(long manifoldPointId,
            Vector3f storeVector);

    native private int getPartId0(long manifoldPointId);

    native private int getPartId1(long manifoldPointId);

    native private void getPositionWorldOnA(long manifoldPointId,
            Vector3f storeVector);

    native private void getPositionWorldOnB(long manifoldPointId,
            Vector3f storeVector);
}
