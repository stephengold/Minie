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

import com.jme3.bullet.NativePhysicsObject;
import com.jme3.math.Vector3f;
import com.jme3.scene.Spatial;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * Describe a point of contact between 2 collision objects in a PhysicsSpace,
 * based on Bullet's btManifoldPoint.
 *
 * @author normenhansen
 */
public class PhysicsCollisionEvent extends NativePhysicsObject {
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
     * first object involved in the collision (typically a PhysicsRigidBody)
     */
    final private PhysicsCollisionObject pcoA;
    /**
     * 2nd object involved in the collision (typically a PhysicsRigidBody)
     */
    final private PhysicsCollisionObject pcoB;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an event.
     *
     * @param pcoA the first involved object (not null, alias created)
     * @param pcoB the 2nd involved object (not null, alias created)
     * @param manifoldPointId the native ID of the btManifoldPoint (not zero)
     */
    public PhysicsCollisionEvent(PhysicsCollisionObject pcoA,
            PhysicsCollisionObject pcoB, long manifoldPointId) {
        Validate.nonNull(pcoA, "object A");
        Validate.nonNull(pcoB, "object B");
        Validate.nonZero(manifoldPointId, "manifold point ID");

        this.pcoA = pcoA;
        this.pcoB = pcoB;
        super.setNativeIdNotTracked(manifoldPointId);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Access the user object of collision object A, provided it's a Spatial.
     * For compatibility with jme3-jbullet.
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
     * For compatibility with jme3-jbullet.
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
     * Return the contact point's applied impulse (native field:
     * m_appliedImpulse).
     *
     * @return the calculated impulse, or zero if the WarmStart bit is cleared
     * in the solver mode
     */
    public float getAppliedImpulse() {
        long nativeId = nativeId();
        float result = ManifoldPoints.getAppliedImpulse(nativeId);

        return result;
    }

    /**
     * Return the contact point's applied lateral impulse #1 (native field:
     * m_appliedImpulseLateral1).
     *
     * @return the calculated impulse component
     */
    public float getAppliedImpulseLateral1() {
        long nativeId = nativeId();
        float result = ManifoldPoints.getAppliedImpulseLateral1(nativeId);

        return result;
    }

    /**
     * Return the contact point's applied lateral impulse #2 (native field:
     * m_appliedImpulseLateral2).
     *
     * @return the calculate impulse component
     */
    public float getAppliedImpulseLateral2() {
        long nativeId = nativeId();
        float result = ManifoldPoints.getAppliedImpulseLateral2(nativeId);

        return result;
    }

    /**
     * Return the contact point's combined friction, which is usually the
     * product of the collision-object frictions (native field:
     * m_combinedFriction).
     *
     * @return the friction product
     */
    public float getCombinedFriction() {
        long nativeId = nativeId();
        float result = ManifoldPoints.getCombinedFriction(nativeId);

        return result;
    }

    /**
     * Return the contact point's combined restitution, which is usually the
     * product of the collision-object restitutions (native field:
     * m_combinedRestitution).
     *
     * @return the restitution product
     */
    public float getCombinedRestitution() {
        long nativeId = nativeId();
        float result = ManifoldPoints.getCombinedRestitution(nativeId);

        return result;
    }

    /**
     * Return the contact point's combined rolling friction (native field:
     * m_combinedRollingFriction).
     *
     * @return the combined friction
     */
    public float getCombinedRollingFriction() {
        long nativeId = nativeId();
        float result = ManifoldPoints.getCombinedRollingFriction(nativeId);

        return result;
    }

    /**
     * Return the contact point's combined spinning friction (native field:
     * m_combinedSpinningFriction).
     *
     * @return the combined friction
     */
    public float getCombinedSpinningFriction() {
        long nativeId = nativeId();
        float result = ManifoldPoints.getCombinedSpinningFriction(nativeId);

        return result;
    }

    /**
     * Return the contact point's signed separation distance (native field:
     * m_distance1). This value is the negative of the penetration depth.
     *
     * @return the distance (in physics-space units)
     */
    public float getDistance1() {
        long nativeId = nativeId();
        float result = ManifoldPoints.getDistance1(nativeId);

        return result;
    }

    /**
     * Return the contact-point flags (native field: m_contactPointFlags).
     *
     * @return a bitmask
     * @see com.jme3.bullet.collision.ContactPointFlag
     */
    public int getFlags() {
        long nativeId = nativeId();
        int result = ManifoldPoints.getFlags(nativeId);

        return result;
    }

    /**
     * Return the triangle index in the shape of object A at the point of
     * contact (native field: m_index0).
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
        long nativeId = nativeId();
        int result = ManifoldPoints.getIndex0(nativeId);

        return result;
    }

    /**
     * Return the triangle index in the shape of object B at the point of
     * contact (native field: m_index1).
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
        long nativeId = nativeId();
        int result = ManifoldPoints.getIndex1(nativeId);

        return result;
    }

    /**
     * Return the contact point's lateral friction direction #1 (native field:
     * m_lateralFrictionDir1).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a direction vector (either storeResult or a new instance)
     */
    public Vector3f getLateralFrictionDir1(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long nativeId = nativeId();
        ManifoldPoints.getLateralFrictionDir1(nativeId, result);

        return result;
    }

    /**
     * Return the contact point's lateral friction direction #2 (native field:
     * m_lateralFrictionDir2).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a direction vector (either storeResult or a new instance)
     */
    public Vector3f getLateralFrictionDir2(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long nativeId = nativeId();
        ManifoldPoints.getLateralFrictionDir2(nativeId, result);

        return result;
    }

    /**
     * Return the contact point's lifetime (native name: m_lifeTime).
     *
     * @return the duration (in timesteps, &ge;0)
     */
    public int getLifeTime() {
        long nativeId = nativeId();
        int result = ManifoldPoints.getLifeTime(nativeId);

        return result;
    }

    /**
     * Return the contact point's location in the local coordinates of object A
     * (native name: m_localPointA).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (either storeResult or a new instance)
     */
    public Vector3f getLocalPointA(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long nativeId = nativeId();
        ManifoldPoints.getLocalPointA(nativeId, result);

        return result;
    }

    /**
     * Return the contact point's location in the local coordinates of object B
     * (native name: m_localPointB).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (either storeResult or a new instance)
     */
    public Vector3f getLocalPointB(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long nativeId = nativeId();
        ManifoldPoints.getLocalPointB(nativeId, result);

        return result;
    }

    /**
     * For compatibility with jme3-jbullet.
     *
     * @return a new direction vector
     */
    public Vector3f getNormalWorldOnB() {
        Vector3f result = getNormalWorldOnB(null);
        return result;
    }

    /**
     * Return the contact point's normal on object B in physics-space
     * coordinates (native name: m_normalWorldOnB).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a direction vector (either storeResult or a new instance)
     */
    public Vector3f getNormalWorldOnB(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long nativeId = nativeId();
        ManifoldPoints.getNormalWorldOnB(nativeId, result);

        return result;
    }

    /**
     * Return the part index in the shape of object A at the point of contact
     * (native field: m_partId0).
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
        long nativeId = nativeId();
        int result = ManifoldPoints.getPartId0(nativeId);

        return result;
    }

    /**
     * Return the part index in the shape of object B at the point of contact
     * (native field: m_partId1).
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
        long nativeId = nativeId();
        int result = ManifoldPoints.getPartId1(nativeId);

        return result;
    }

    /**
     * For compatibility with jme3-jbullet.
     *
     * @return a new location vector
     */
    public Vector3f getPositionWorldOnA() {
        Vector3f result = getPositionWorldOnA(null);
        return result;
    }

    /**
     * Return the contact point's location on object A in physics-space
     * coordinates (native field: m_positionWorldOnA).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (either storeResult or a new instance)
     */
    public Vector3f getPositionWorldOnA(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long nativeId = nativeId();
        ManifoldPoints.getPositionWorldOnA(nativeId, result);

        return result;
    }

    /**
     * For compatibility with jme3-jbullet.
     *
     * @return a new location vector
     */
    public Vector3f getPositionWorldOnB() {
        Vector3f result = getPositionWorldOnB(null);
        return result;
    }

    /**
     * Return the contact point's location on object B in physics-space
     * coordinates (native field: m_positionWorldOnB).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (either storeResult or a new instance)
     */
    public Vector3f getPositionWorldOnB(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long nativeId = nativeId();
        ManifoldPoints.getPositionWorldOnB(nativeId, result);

        return result;
    }

    /**
     * Test whether the contact point's lateral friction is initialized.
     *
     * @return true if initialized, otherwise false
     */
    public boolean isLateralFrictionInitialized() {
        int flags = getFlags();
        boolean result = (flags & ContactPointFlag.LATERAL_FRICTION) != 0x0;

        return result;
    }
}
