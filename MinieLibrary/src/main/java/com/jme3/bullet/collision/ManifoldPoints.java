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
import com.simsilica.mathd.Vec3d;
import java.util.logging.Logger;

/**
 * Utility class to access fields of Bullet's {@code btManifoldPoint} class.
 * <p>
 * Based on PhysicsCollisionEvent.
 */
final public class ManifoldPoints {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(ManifoldPoints.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private ManifoldPoints() {
    }
    // *************************************************************************
    // new native methods exposed

    /**
     * Create a manifold point (for testing) that will never be freed. For
     * internal use only.
     *
     * @return the native ID of a new {@code btManifoldPoint} (not zero)
     */
    native public static long createTestPoint();

    /**
     * Return the applied impulse of the specified point (native field:
     * m_appliedImpulse).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @return the impulse calculated during the previous timestep, or zero if
     * the WarmStart bit is cleared in the solver mode
     */
    native public static float getAppliedImpulse(long manifoldPointId);

    /**
     * Return the applied lateral impulse #1 of the specified point (native
     * field: m_appliedImpulseLateral1).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @return the impulse
     */
    native public static float getAppliedImpulseLateral1(long manifoldPointId);

    /**
     * Return the applied lateral impulse #2 of the specified point (native
     * field: m_appliedImpulseLateral2).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @return the impulse
     */
    native public static float getAppliedImpulseLateral2(long manifoldPointId);

    /**
     * Return the combined friction of the specified point, which is the product
     * of the collision-object frictions (native field: m_combinedFriction).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @return the friction product
     */
    native public static float getCombinedFriction(long manifoldPointId);

    /**
     * Return the combined restitution of the specified point, which is the
     * product of the collision-object restitutions (native field:
     * m_combinedRestitution).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @return the restitution product
     */
    native public static float getCombinedRestitution(long manifoldPointId);

    /**
     * Return the combined rolling friction of the specified point (native
     * field: m_combinedRollingFriction).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @return the combined friction
     */
    native public static float
            getCombinedRollingFriction(long manifoldPointId);

    /**
     * Return the combined spinning friction of the specified point (native
     * field: m_combinedSpinningFriction).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @return the combined friction
     */
    native public static float
            getCombinedSpinningFriction(long manifoldPointId);

    /**
     * Return the contact motion #1 of the specified point (native field:
     * m_contactMotion1).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @return the motion
     */
    native public static float getContactMotion1(long manifoldPointId);

    /**
     * Return the contact motion #2 of the specified point (native field:
     * m_contactMotion2).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @return the motion
     */
    native public static float getContactMotion2(long manifoldPointId);

    /**
     * Return the separation distance of the specified point (native field:
     * m_distance1). This is the negative of the penetration depth.
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @return the distance (in physics-space units)
     */
    native public static float getDistance1(long manifoldPointId);

    /**
     * Return the flags of the specified point (native field:
     * m_contactPointFlags).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @return a bitmask
     * @see com.jme3.bullet.collision.ContactPointFlag
     */
    native public static int getFlags(long manifoldPointId);

    /**
     * Return the triangle index from the shape of object A at the specified
     * point of contact (native field: m_index0).
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
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @return the index of the collision-shape triangle (&ge;0) or -1 if
     * undefined
     */
    native public static int getIndex0(long manifoldPointId);

    /**
     * Return the triangle index from the shape of object B at the specified
     * point of contact (native field: m_index1).
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
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @return the index of the collision-shape triangle (&ge;0) or -1 if
     * undefined
     */
    native public static int getIndex1(long manifoldPointId);

    /**
     * Determine the lateral friction direction #1 of the specified point
     * (native field: m_lateralFrictionDir1).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @param storeVector storage for the result (not null, modified)
     */
    native public static void
            getLateralFrictionDir1(long manifoldPointId, Vector3f storeVector);

    /**
     * Determine the lateral friction direction #2 of the specified point
     * (native field: m_lateralFrictionDir2).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @param storeVector storage for the result (not null, modified)
     */
    native public static void
            getLateralFrictionDir2(long manifoldPointId, Vector3f storeVector);

    /**
     * Return the lifetime of the specified point (native name: m_lifeTime).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @return the duration (in timesteps, &ge;0)
     */
    native public static int getLifeTime(long manifoldPointId);

    /**
     * Determine the location of the specified point in the local coordinates of
     * object A (native name: m_localPointA).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @param storeVector storage for the result (not null, modified)
     */
    native public static void
            getLocalPointA(long manifoldPointId, Vector3f storeVector);

    /**
     * Determine the location of the specified point in the local coordinates of
     * object B (native name: m_localPointB).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @param storeVector storage for the result (not null, modified)
     */
    native public static void
            getLocalPointB(long manifoldPointId, Vector3f storeVector);

    /**
     * Determine the normal on object B of the specified point in physics-space
     * coordinates (native name: m_normalWorldOnB).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @param storeVector storage for the result (not null, modified)
     */
    native public static void
            getNormalWorldOnB(long manifoldPointId, Vector3f storeVector);

    /**
     * Return the part index from the shape of object A at the specified point
     * of contact (native field: m_partId0).
     * <p>
     * If the shape is compound or convex, the index is undefined.
     * <p>
     * If the shape is a GImpactCollisionShape or MeshCollisionShape, the index
     * identifies an IndexedMesh.
     * <p>
     * If the shape is a HeightfieldCollisionShape, the index identifies a grid
     * row.
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @return the index of the collision-shape part (&ge;0) or -1 if undefined
     */
    native public static int getPartId0(long manifoldPointId);

    /**
     * Return the part index from the shape of object B at the specified point
     * of contact (native field: m_partId1).
     * <p>
     * If the shape is compound or convex, the index is undefined.
     * <p>
     * If the shape is a GImpactCollisionShape or MeshCollisionShape, the index
     * identifies an IndexedMesh.
     * <p>
     * If the shape is a HeightfieldCollisionShape, the index identifies a grid
     * row.
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @return the index of the collision-shape part (&ge;0) or -1 if undefined
     */
    native public static int getPartId1(long manifoldPointId);

    /**
     * Determine the location of the specified point on object A in
     * physics-space coordinates (native field: m_positionWorldOnA).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @param storeVector storage for the result (not null, modified)
     */
    native public static void
            getPositionWorldOnA(long manifoldPointId, Vector3f storeVector);

    /**
     * Determine the location of the specified point on object A in
     * physics-space coordinates (native field: m_positionWorldOnA).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @param storeVector storage for the result (not null, modified)
     */
    native public static void
            getPositionWorldOnADp(long manifoldPointId, Vec3d storeVector);

    /**
     * Determine the location of the specified point on object B in
     * physics-space coordinates (native field: m_positionWorldOnB).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @param storeVector storage for the result (modified if not null)
     */
    native public static void
            getPositionWorldOnB(long manifoldPointId, Vector3f storeVector);

    /**
     * Determine the location of the specified point on object B in
     * physics-space coordinates (native field: m_positionWorldOnB).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @param storeVector storage for the result (modified if not null)
     */
    native public static void
            getPositionWorldOnBDp(long manifoldPointId, Vec3d storeVector);

    /**
     * Determine how many points are used to calculate the area of the convex
     * hull of a contact point.
     *
     * @return true if using 3 points (faster), false if using 4 points (more
     * accurate)
     */
    native public static boolean isContactCalcArea3Points();

    /**
     * Alter the applied impulse of the specified point (native field:
     * m_appliedImpulse).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @param impulse the desired impulse
     */
    native public static void
            setAppliedImpulse(long manifoldPointId, float impulse);

    /**
     * Alter the applied lateral impulse #1 of the specified point (native
     * field: m_appliedImpulseLateral1).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @param impulse the desired impulse
     */
    native public static void
            setAppliedImpulseLateral1(long manifoldPointId, float impulse);

    /**
     * Alter the applied lateral impulse #2 of the specified point (native
     * field: m_appliedImpulseLateral2).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @param impulse the desired impulse
     */
    native public static void
            setAppliedImpulseLateral2(long manifoldPointId, float impulse);

    /**
     * Alter the combined friction of the specified point (native field:
     * m_combinedFriction).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @param friction the desired friction
     */
    native public static void
            setCombinedFriction(long manifoldPointId, float friction);

    /**
     * Alter the combined restitution of the specified point (native field:
     * m_combinedRestitution).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @param restitution the desired restitution
     */
    native public static void
            setCombinedRestitution(long manifoldPointId, float restitution);

    /**
     * Alter the combined rolling friction of the specified point (native field:
     * m_combinedRollingFriction).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @param friction the desired friction
     */
    native public static void
            setCombinedRollingFriction(long manifoldPointId, float friction);

    /**
     * Alter the combined spinning friction of the specified point (native
     * field: m_combinedSpinningFriction).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @param friction the combined friction
     */
    native public static void
            setCombinedSpinningFriction(long manifoldPointId, float friction);

    /**
     * Alter the number of points used to calculate the area of the convex hull
     * of a contact point.
     *
     * @param setting true to use 3 points (faster), false to use 4 points (more
     * accurate) (default=true)
     */
    native public static void setContactCalcArea3Points(boolean setting);

    /**
     * Alter the contact motion #1 of the specified point (native field:
     * m_contactMotion1).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @param motion the desired motion
     */
    native public static void
            setContactMotion1(long manifoldPointId, float motion);

    /**
     * Alter the contact motion #2 of the specified point (native field:
     * m_contactMotion2).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @param motion the desired motion
     */
    native public static void
            setContactMotion2(long manifoldPointId, float motion);

    /**
     * Alter the separation distance of the specified point (native field:
     * m_distance1). This is the negative of the penetration depth.
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @param distance the desired distance (in physics-space units)
     */
    native public static void
            setDistance1(long manifoldPointId, float distance);

    /**
     * Alter the flags of the specified point (native field:
     * m_contactPointFlags).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @param bitmask the desired bitmask
     * @see com.jme3.bullet.collision.ContactPointFlag
     */
    native public static void setFlags(long manifoldPointId, int bitmask);

    /**
     * Alter the lateral friction direction #1 of the specified point (native
     * field: m_lateralFrictionDir1).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @param direction the desired direction (in physics-space coordinates, not
     * null, unaffected)
     */
    native public static void
            setLateralFrictionDir1(long manifoldPointId, Vector3f direction);

    /**
     * Alter the lateral friction direction #2 of the specified point (native
     * field: m_lateralFrictionDir2).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @param direction the desired direction (in physics-space coordinates, not
     * null, unaffected)
     */
    native public static void
            setLateralFrictionDir2(long manifoldPointId, Vector3f direction);

    /**
     * Alter the location of the specified point in the local coordinates of
     * object A (native name: m_localPointA).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @param locationVector the desired location (not null, unaffected)
     */
    native public static void
            setLocalPointA(long manifoldPointId, Vector3f locationVector);

    /**
     * Alter the location of the specified point in the local coordinates of
     * object B (native name: m_localPointB).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @param locationVector the desired location (not null, unaffected)
     */
    native public static void
            setLocalPointB(long manifoldPointId, Vector3f locationVector);

    /**
     * Alter the normal on object B of the specified point in physics-space
     * coordinates (native name: m_normalWorldOnB).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @param normalVector the desired normal (not null, unaffected)
     */
    native public static void
            setNormalWorldOnB(long manifoldPointId, Vector3f normalVector);

    /**
     * Alter the location of the specified point on object A in physics-space
     * coordinates (native field: m_positionWorldOnA).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @param locationVector the desired location (not null, unaffected)
     */
    native public static void
            setPositionWorldOnA(long manifoldPointId, Vector3f locationVector);

    /**
     * Alter the location of the specified point on object B in physics-space
     * coordinates (native field: m_positionWorldOnB).
     *
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     * @param locationVector the desired location (not null, unaffected)
     */
    native public static void
            setPositionWorldOnB(long manifoldPointId, Vector3f locationVector);
}
