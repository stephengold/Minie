/*
 * Copyright (c) 2022 jMonkeyEngine
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
    final protected boolean contactResponse;
    /**
     * gravity-protection flag
     */
    final protected boolean protectGravity;
    /**
     * angular damping fraction
     */
    final protected float angularDamping;
    /**
     * angular-motion sleeping threshold (in radians per second)
     */
    final protected float angularSleepingThreshold;
    /**
     * continuous collision detection motion threshold (in physics-space units)
     */
    final protected float ccdMotionThreshold;
    /**
     * radius of the sphere used for continuous collision detection (in
     * physics-space units)
     */
    final protected float ccdSweptSphereRadius;
    /**
     * contact damping
     */
    final protected float contactDamping;
    /**
     * contact processing threshold (in physics-space units)
     */
    final protected float contactProcessingThreshold;
    /**
     * contact stiffness
     */
    final protected float contactStiffness;
    /**
     * deactivation time (in seconds)
     */
    final protected float deactivationTime;
    /**
     * friction parameter
     */
    final protected float friction;
    /**
     * linear damping fraction
     */
    final protected float linearDamping;
    /**
     * linear-motion threshold (in physics-space units per second)
     */
    final protected float linearSleepingThreshold;
    /**
     * restitution (bounciness)
     */
    final protected float restitution;
    /**
     * rolling friction
     */
    final protected float rollingFriction;
    /**
     * spinning friction
     */
    final protected float spinningFriction;
    /**
     * anisotopic friction modes (bitmask)
     */
    final protected int anisotopicFrictionModes;
    /**
     * native IDs of all collision objects in the ignore list
     */
    final protected long[] ignoreList;
    /**
     * basis of the local coordinate system (in physics-space coordinates)
     */
    final protected Matrix3f rotationMatrix;
    /**
     * angular velocity (in physics-space coordinates)
     */
    final protected Vec3d angularVelocity;
    /**
     * linear velocity (in physics-space coordinates)
     */
    final protected Vec3d linearVelocity;
    /**
     * location of the center (in physics-space coordinates)
     */
    final protected Vec3d location;
    /**
     * anisotropic friction components
     */
    final protected Vector3f anisotopicFrictionComponents;
    /**
     * angular factors
     */
    final protected Vector3f angularFactor;
    /**
     * linear factors
     */
    final protected Vector3f linearFactor;
    /**
     * applied central force (excluding contact forces, damping, and gravity)
     */
    final protected Vector3f totalAppliedForce;
    /**
     * applied torque (excluding contact forces and damping)
     */
    final protected Vector3f totalAppliedTorque;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a snapshot of the specified body.
     *
     * @param body the body to capture (not null)
     */
    public RigidBodySnapshot(PhysicsRigidBody body) {
        // boolean
        this.contactResponse = body.isContactResponse();
        this.protectGravity = body.isGravityProtected();

        // float
        this.angularDamping = body.getAngularDamping();
        this.angularSleepingThreshold = body.getAngularSleepingThreshold();
        this.ccdMotionThreshold = body.getCcdMotionThreshold();
        this.ccdSweptSphereRadius = body.getCcdSweptSphereRadius();
        this.contactDamping = body.getContactDamping();
        this.contactProcessingThreshold = body.getContactProcessingThreshold();
        this.contactStiffness = body.getContactStiffness();
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
        this.anisotopicFrictionModes = afMode;

        this.ignoreList = body.listIgnoredIds();
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
        this.anisotopicFrictionComponents = body.getAnisotropicFriction(null);
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
        body.setContactDamping(contactDamping);
        body.setContactProcessingThreshold(contactProcessingThreshold);
        body.setContactStiffness(contactStiffness);
        // deactivation time is set below
        body.setFriction(friction);
        body.setLinearDamping(linearDamping);
        body.setLinearSleepingThreshold(linearSleepingThreshold);
        body.setRestitution(restitution);
        body.setRollingFriction(rollingFriction);
        body.setSpinningFriction(spinningFriction);

        body.setAnisotropicFriction(
                anisotopicFrictionComponents, anisotopicFrictionModes);
        body.setIgnoreList(ignoreList);
        body.setPhysicsRotation(rotationMatrix);
        body.setPhysicsLocationDp(location);
        body.clearForces();

        // Linear factors affect the application of central force.
        body.setLinearFactor(scaleIdentity); // temporary setting
        body.applyCentralForce(totalAppliedForce);
        body.setLinearFactor(linearFactor);

        // Angular factors affect the application of torque.
        body.setAngularFactor(scaleIdentity); // temporary settting
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
