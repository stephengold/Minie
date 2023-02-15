/*
 * Copyright (c) 2022-2023 jMonkeyEngine
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
package com.jme3.bullet.objects.infos;

import com.jme3.bullet.collision.AfMode;
import com.jme3.bullet.collision.CollisionFlag;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Matrix3f;
import com.jme3.math.Vector3f;
import com.simsilica.mathd.Vec3d;
import java.util.logging.Logger;

/**
 * Copy certain properties of a PhysicsRigidBody in order to re-apply them
 * later. Immutable.
 * <p>
 * Snapshots are used for rebuilding, so they don't include the collision shape,
 * inertia, joints, kinematic flag, mass, motion state, or physics space.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class RigidBodySnapshot {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(RigidBodySnapshot.class.getName());
    /**
     * local copy of {@link com.jme3.math.Vector3f#UNIT_XYZ}
     */
    final private static Vector3f scaleIdentity = new Vector3f(1f, 1f, 1f);
    // *************************************************************************
    // fields

    /**
     * contact-response flag
     */
    final private boolean contactResponse;
    /**
     * gravity-protection flag
     */
    final private boolean protectGravity;
    /**
     * angular damping fraction
     */
    final private float angularDamping;
    /**
     * angular-motion sleeping threshold (in radians per second)
     */
    final private float angularSleepingThreshold;
    /**
     * continuous collision detection motion threshold (in physics-space units)
     */
    final private float ccdMotionThreshold;
    /**
     * radius of the sphere used for continuous collision detection (in
     * physics-space units)
     */
    final private float ccdSweptSphereRadius;
    /**
     * contact damping
     */
    private Float contactDamping;
    /**
     * contact processing threshold (in physics-space units)
     */
    final private float contactProcessingThreshold;
    /**
     * contact stiffness
     */
    private Float contactStiffness;
    /**
     * deactivation time (in seconds)
     */
    final private float deactivationTime;
    /**
     * friction parameter
     */
    final private float friction;
    /**
     * linear damping fraction
     */
    final private float linearDamping;
    /**
     * linear-motion threshold (in physics-space units per second)
     */
    final private float linearSleepingThreshold;
    /**
     * restitution (bounciness)
     */
    final private float restitution;
    /**
     * rolling friction
     */
    final private float rollingFriction;
    /**
     * spinning friction
     */
    final private float spinningFriction;
    /**
     * anisotropic friction modes (bitmask)
     */
    final private int anisotropicFrictionModes;
    /**
     * basis of the local coordinate system (in physics-space coordinates)
     */
    final private Matrix3f rotationMatrix;
    /**
     * collision objects in the ignore list
     */
    final private PhysicsCollisionObject[] ignoreList;
    /**
     * angular velocity (in physics-space coordinates)
     */
    final private Vec3d angularVelocity;
    /**
     * linear velocity (in physics-space coordinates)
     */
    final private Vec3d linearVelocity;
    /**
     * location of the center (in physics-space coordinates)
     */
    final private Vec3d location;
    /**
     * angular factors
     */
    final private Vector3f angularFactor;
    /**
     * anisotropic friction components
     */
    final private Vector3f anisotropicFrictionComponents;
    /**
     * linear factors
     */
    final private Vector3f linearFactor;
    /**
     * applied central force (excluding contact forces, damping, and gravity)
     */
    final private Vector3f totalAppliedForce;
    /**
     * applied torque (excluding contact forces and damping)
     */
    final private Vector3f totalAppliedTorque;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a snapshot of the specified body.
     *
     * @param body the body to capture (not null)
     */
    public RigidBodySnapshot(PhysicsRigidBody body) {
        int flags = body.collisionFlags();
        boolean hasCsd
                = (flags & CollisionFlag.HAS_CONTACT_STIFFNESS_DAMPING) != 0;

        // boolean
        this.contactResponse = body.isContactResponse();
        this.protectGravity = body.isGravityProtected();

        // float
        this.angularDamping = body.getAngularDamping();
        this.angularSleepingThreshold = body.getAngularSleepingThreshold();
        this.ccdMotionThreshold = body.getCcdMotionThreshold();
        this.ccdSweptSphereRadius = body.getCcdSweptSphereRadius();
        if (hasCsd) {
            this.contactDamping = body.getContactDamping();
        }
        this.contactProcessingThreshold = body.getContactProcessingThreshold();
        if (hasCsd) {
            this.contactStiffness = body.getContactStiffness();
        }
        this.deactivationTime = body.getDeactivationTime();
        this.friction = body.getFriction();
        this.linearDamping = body.getLinearDamping();
        this.linearSleepingThreshold = body.getLinearSleepingThreshold();
        this.restitution = body.getRestitution();
        this.rollingFriction = body.getRollingFriction();
        this.spinningFriction = body.getSpinningFriction();

        int afMode = AfMode.none;
        for (int bitPosition = 0; bitPosition < 2; ++bitPosition) {
            int bitMask = 1 << bitPosition;
            if (body.hasAnisotropicFriction(bitMask)) {
                afMode |= bitMask;
            }
        }
        this.anisotropicFrictionModes = afMode;

        this.ignoreList = body.listIgnoredPcos();
        this.rotationMatrix = body.getPhysicsRotationMatrix(null);

        // Vec3d
        if (body.isDynamic()) {
            this.angularVelocity = body.getAngularVelocityDp(null);
            this.linearVelocity = body.getLinearVelocityDp(null);
        } else {
            this.angularVelocity = new Vec3d();
            this.linearVelocity = new Vec3d();
        }
        this.location = body.getPhysicsLocationDp(null);

        // Vector3f
        this.anisotropicFrictionComponents = body.getAnisotropicFriction(null);
        this.angularFactor = body.getAngularFactor(null);
        this.linearFactor = body.getLinearFactor(null);
        this.totalAppliedForce = body.totalAppliedForce(null);
        this.totalAppliedTorque = body.totalAppliedTorque(null);
    }
    // *********************************************************************
    // new methods exposed

    /**
     * Apply the properties to the specified body.
     *
     * @param body the target body (not null, modified)
     */
    public void applyTo(PhysicsRigidBody body) {
        // boolean
        body.setContactResponse(contactResponse);
        body.setProtectGravity(protectGravity);

        // float
        body.setAngularDamping(angularDamping);
        body.setAngularSleepingThreshold(angularSleepingThreshold);
        body.setCcdMotionThreshold(ccdMotionThreshold);
        body.setCcdSweptSphereRadius(ccdSweptSphereRadius);
        if (contactDamping != null) {
            body.setContactDamping(contactDamping);
        }
        body.setContactProcessingThreshold(contactProcessingThreshold);
        if (contactStiffness != null) {
            body.setContactStiffness(contactStiffness);
        }
        // deactivation time is set below
        body.setFriction(friction);
        body.setLinearDamping(linearDamping);
        body.setLinearSleepingThreshold(linearSleepingThreshold);
        body.setRestitution(restitution);
        body.setRollingFriction(rollingFriction);
        body.setSpinningFriction(spinningFriction);

        body.setAnisotropicFriction(
                anisotropicFrictionComponents, anisotropicFrictionModes);
        body.setIgnoreList(ignoreList);
        body.setPhysicsRotation(rotationMatrix);
        body.setPhysicsLocationDp(location);
        body.clearForces();

        // Linear factors affect the application of central force.
        body.setLinearFactor(scaleIdentity); // temporary setting
        body.applyCentralForce(totalAppliedForce);
        body.setLinearFactor(linearFactor);

        // Angular factors affect the application of torque.
        body.setAngularFactor(scaleIdentity); // temporary setting
        body.applyTorque(totalAppliedTorque);
        body.setAngularFactor(angularFactor);

        // Velocities affect deactivation time.
        if (body.isDynamic()) {
            body.setAngularVelocityDp(angularVelocity);
            body.setLinearVelocityDp(linearVelocity);
        }
        body.setDeactivationTime(deactivationTime);
    }
}
