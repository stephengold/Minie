/*
 * Copyright (c) 2009-2019 jMonkeyEngine
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
package com.jme3.bullet.objects;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.Activation;
import com.jme3.bullet.collision.CollisionFlag;
import com.jme3.bullet.collision.PcoType;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.HeightfieldCollisionShape;
import com.jme3.bullet.objects.infos.RigidBodyMotionState;
import com.jme3.bullet.objects.infos.RigidBodySnapshot;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.FastMath;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import com.simsilica.mathd.Matrix3d;
import com.simsilica.mathd.Quatd;
import com.simsilica.mathd.Vec3d;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyQuaternion;
import jme3utilities.math.MyVector3f;

/**
 * A collision object to simulate a rigid body, based on Bullet's
 * {@code btRigidBody}.
 *
 * @author normenhansen
 */
public class PhysicsRigidBody extends PhysicsBody {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(PhysicsRigidBody.class.getName());
    /**
     * local copy of {@link com.jme3.math.Matrix3f#IDENTITY}
     */
    final private static Matrix3f matrixIdentity = new Matrix3f();
    /**
     * field names for serialization
     */
    final private static String tagAngularDamping = "angularDamping";
    final private static String tagAngularFactor = "angularFactor";
    final private static String tagAngularSleepingThreshold
            = "angularSleepingThreshold";
    final private static String tagAngularVelocity = "angularVelocity";
    final private static String tagAppliedForce = "appliedForce";
    final private static String tagAppliedTorque = "appliedTorque";
    final private static String tagContactResponse = "contactResponse";
    final private static String tagInverseInertia = "inverseInertia";
    final private static String tagKinematic = "kinematic";
    final private static String tagLinearDamping = "linearDamping";
    final private static String tagLinearFactor = "linearFactor";
    final private static String tagLinearSleepingThreshold
            = "linearSleepingThreshold";
    final private static String tagLinearVelocity = "linearVelocity";
    final private static String tagMass = "mass";
    final private static String tagMotionState = "motionState";
    final private static String tagPhysicsLocation = "physicsLocation";
    final private static String tagPhysicsRotation = "physicsRotation";
    final private static String tagProtectGravity = "tagProtectGravity";
    /**
     * local copy of {@link com.jme3.math.Vector3f#UNIT_XYZ}
     */
    final private static Vector3f scaleIdentity = new Vector3f(1f, 1f, 1f);
    /**
     * local copy of {@link com.jme3.math.Vector3f#ZERO}
     */
    final private static Vector3f translateIdentity = new Vector3f(0f, 0f, 0f);
    // *************************************************************************
    // fields

    /**
     * copy of kinematic flag: true&rarr;set kinematic mode,
     * false&rarr;dynamic/static mode
     */
    private boolean kinematic = false;
    /**
     * copy of the mass (&gt;0) of a dynamic body, or 0 for a static body
     */
    protected float mass = 1f;
    /**
     * motion state
     */
    private RigidBodyMotionState motionState = new RigidBodyMotionState();
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected PhysicsRigidBody() {
        // do nothing
    }

    /**
     * Instantiate a responsive, dynamic body with mass=1 and the specified
     * CollisionShape. The new body is not added to any PhysicsSpace.
     *
     * @param shape the desired shape (not null, alias created)
     */
    public PhysicsRigidBody(CollisionShape shape) {
        Validate.nonNull(shape, "shape");

        super.setCollisionShape(shape);
        rebuildRigidBody();

        assert isContactResponse();
        assert !isInWorld();
        assert !isKinematic();
        assert !isStatic();
        assert mass == 1f : mass;
    }

    /**
     * Instantiate a responsive dynamic or static body with the specified
     * CollisionShape and mass. The new body is not added to any PhysicsSpace.
     *
     * @param shape the desired shape (not null, alias created)
     * @param mass if 0, a static body is created; otherwise a dynamic body is
     * created (&ge;0, default=1)
     */
    public PhysicsRigidBody(CollisionShape shape, float mass) {
        Validate.nonNull(shape, "shape");
        Validate.nonNegative(mass, "mass");

        this.mass = mass;
        super.setCollisionShape(shape);
        rebuildRigidBody();

        assert isContactResponse();
        assert !isInWorld();
        assert !isKinematic();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Reactivate this body if it has been deactivated due to lack of motion.
     */
    public void activate() {
        activate(true);
        assert isActive();
    }

    /**
     * Apply a central force to the body. Effective on the next time step, then
     * cleared.
     * <p>
     * To apply an impulse, use
     * {@link #applyCentralImpulse(com.jme3.math.Vector3f)}.
     *
     * @param force the force vector (mass times physics-space units per second
     * squared in physics-space coordinates, not null, unaffected)
     */
    public void applyCentralForce(Vector3f force) {
        Validate.finite(force, "force");

        long objectId = nativeId();
        applyCentralForce(objectId, force);
        activate();
    }

    /**
     * Apply a central impulse to the body.
     *
     * @param impulse the impulse vector (mass times physics-space units per
     * second in physics-space coordinates, not null, unaffected)
     */
    public void applyCentralImpulse(Vector3f impulse) {
        Validate.finite(impulse, "impulse");

        long objectId = nativeId();
        applyCentralImpulse(objectId, impulse);
        activate();
    }

    /**
     * Apply a force to the body. Effective on the next time step, then cleared.
     * <p>
     * To apply an impulse, use
     * {@link #applyImpulse(com.jme3.math.Vector3f, com.jme3.math.Vector3f)}.
     *
     * @param force the force vector (mass times physics-space units per second
     * squared in physics-space coordinates, not null, unaffected)
     * @param offset the location to apply the force (relative to the body's
     * center in physics-space coordinates, not null, unaffected)
     */
    public void applyForce(Vector3f force, Vector3f offset) {
        Validate.finite(force, "force");
        Validate.finite(offset, "offset");

        long objectId = nativeId();
        applyForce(objectId, force, offset);
        activate();
    }

    /**
     * Apply an impulse to the body.
     *
     * @param impulse the impulse vector (mass times physics-space units per
     * second in physics-space coordinates, not null, unaffected)
     * @param offset the location to apply the impulse (relative to the body's
     * center in physics-space coordinates, not null, unaffected)
     */
    public void applyImpulse(Vector3f impulse, Vector3f offset) {
        Validate.finite(impulse, "impulse");
        Validate.finite(offset, "offset");

        long objectId = nativeId();
        applyImpulse(objectId, impulse, offset);
        activate();
    }

    /**
     * Apply a torque to the body. Effective on the next time step, then
     * cleared.
     * <p>
     * To apply a torque impulse, use
     * {@link #applyTorqueImpulse(com.jme3.math.Vector3f)}.
     *
     * @param torque the torque vector (mass times physics-space units squared
     * per second squared in physics-space coordinates, not null, unaffected)
     */
    public void applyTorque(Vector3f torque) {
        Validate.finite(torque, "torque");

        long objectId = nativeId();
        applyTorque(objectId, torque);
        activate();
    }

    /**
     * Apply a torque impulse to the body.
     *
     * @param torqueImpulse the torque impulse vector (mass times physics-space
     * units squared per second in physics-space coordinates, not null,
     * unaffected)
     */
    public void applyTorqueImpulse(Vector3f torqueImpulse) {
        Validate.finite(torqueImpulse, "torque impulse");

        long objectId = nativeId();
        applyTorqueImpulse(objectId, torqueImpulse);
        activate();
    }

    /**
     * Clear all forces and torques acting on this body, including gravity.
     */
    public void clearForces() {
        long objectId = nativeId();
        clearForces(objectId);
    }

    /**
     * Read this body's angular damping.
     *
     * @return the damping fraction (&ge;0, &le;1)
     */
    public float getAngularDamping() {
        long objectId = nativeId();
        float result = getAngularDamping(objectId);

        assert result >= 0f : result;
        assert result <= 1f : result;
        return result;
    }

    /**
     * For compatibility with the jme3-jbullet library.
     *
     * @return the angular factor
     */
    public float getAngularFactor() {
        return getAngularFactor(null).x;
    }

    /**
     * Copy this body's angular factor.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the angular factor for each axis (either storeResult or a new
     * vector, not null)
     */
    public Vector3f getAngularFactor(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long objectId = nativeId();
        getAngularFactor(objectId, result);

        return result;
    }

    /**
     * Read this body's angular-motion sleeping threshold. Note that "sleeping"
     * is synonym for "deactivation".
     *
     * @return the angular-motion threshold (in radians per second, &ge;0)
     */
    public float getAngularSleepingThreshold() {
        long objectId = nativeId();
        float result = getAngularSleepingThreshold(objectId);

        return result;
    }

    /**
     * For compatibility with the jme3-jbullet library.
     *
     * @return a new velocity vector (radians per second in physics-space
     * coordinates, not null)
     */
    public Vector3f getAngularVelocity() {
        assert isDynamic();
        return getAngularVelocity(null);
    }

    /**
     * Copy this body's angular velocity. The body must be in dynamic mode.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a velocity vector (radians per second in physics-space
     * coordinates, either storeResult or a new vector, not null)
     */
    public Vector3f getAngularVelocity(Vector3f storeResult) {
        assert isDynamic();
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long objectId = nativeId();
        getAngularVelocity(objectId, result);

        return result;
    }

    /**
     * Copy this body's angular velocity. The body must be in dynamic mode.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a velocity vector (radians per second in physics-space
     * coordinates, either storeResult or a new vector, not null)
     */
    public Vec3d getAngularVelocityDp(Vec3d storeResult) {
        assert isDynamic();
        Vec3d result = (storeResult == null) ? new Vec3d() : storeResult;

        long objectId = nativeId();
        getAngularVelocityDp(objectId, result);

        return result;
    }

    /**
     * Calculate this body's angular velocity in its local coordinates. The body
     * must be in dynamic mode.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a velocity vector (radians per second in local coordinates,
     * either storeResult or a new vector, not null)
     */
    public Vector3f getAngularVelocityLocal(Vector3f storeResult) {
        assert isDynamic();
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long objectId = nativeId();
        getAngularVelocity(objectId, result);
        Quaternion localToWorld = getPhysicsRotation(null);
        Quaternion worldToLocal = localToWorld.inverse();
        worldToLocal.mult(result, result);

        return result;
    }

    /**
     * Copy this body's gravitational acceleration.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return an acceleration vector in physics-space coordinates (either
     * storeResult or a new vector, not null)
     */
    public Vec3d getGravityDp(Vec3d storeResult) {
        Vec3d result = (storeResult == null) ? new Vec3d() : storeResult;

        long objectId = nativeId();
        getGravityDp(objectId, result);

        return result;
    }

    /**
     * Copy the principal (diagonal) elements of the inverse inertia tensor in
     * the body's local coordinates.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a vector (either storeResult or a new vector, not null)
     */
    public Vector3f getInverseInertiaLocal(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long objectId = nativeId();
        getInverseInertiaLocal(objectId, result);

        return result;
    }

    /**
     * Compute the inverse inertia tensor in physics-space coordinates.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a matrix (either storeResult or a new matrix, not null)
     */
    public Matrix3f getInverseInertiaWorld(Matrix3f storeResult) {
        Matrix3f result = (storeResult == null) ? new Matrix3f() : storeResult;

        long objectId = nativeId();
        getInverseInertiaWorld(objectId, result);

        return result;
    }

    /**
     * Read this body's linear damping.
     *
     * @return the damping fraction (&ge;0, &le;1)
     */
    public float getLinearDamping() {
        long objectId = nativeId();
        float result = getLinearDamping(objectId);

        assert result >= 0f : result;
        assert result <= 1f : result;
        return result;
    }

    /**
     * Copy this body's linear factors.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the linear factor for each axis (either storeResult or a new
     * vector, not null)
     */
    public Vector3f getLinearFactor(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long objectId = nativeId();
        getLinearFactor(objectId, result);

        return result;
    }

    /**
     * Read this body's linear-motion sleeping threshold. Note that "sleeping"
     * is synonym for "deactivation".
     *
     * @return the linear-motion threshold (in physics-space units per second,
     * &ge;0)
     */
    public float getLinearSleepingThreshold() {
        long objectId = nativeId();
        float result = getLinearSleepingThreshold(objectId);

        return result;
    }

    /**
     * For compatibility with the jme3-jbullet library.
     *
     * @return a new velocity vector (physics-space units per second in
     * physics-space coordinates, not null)
     */
    public Vector3f getLinearVelocity() {
        assert isDynamic();
        return getLinearVelocity(null);
    }

    /**
     * Copy the linear velocity of this body's center of mass. The body must be
     * in dynamic mode.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a velocity vector (physics-space units per second in
     * physics-space coordinates, either storeResult or a new vector, not null)
     */
    public Vector3f getLinearVelocity(Vector3f storeResult) {
        assert isDynamic();
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long objectId = nativeId();
        getLinearVelocity(objectId, result);

        return result;
    }

    /**
     * Copy the linear velocity of this body's center of mass. The body must be
     * in dynamic mode.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a velocity vector (physics-space units per second in
     * physics-space coordinates, either storeResult or a new vector, not null)
     */
    public Vec3d getLinearVelocityDp(Vec3d storeResult) {
        assert isDynamic();
        Vec3d result = (storeResult == null) ? new Vec3d() : storeResult;

        long objectId = nativeId();
        getLinearVelocityDp(objectId, result);

        return result;
    }

    /**
     * Access this body's motion state.
     *
     * @return the pre-existing instance (not null)
     */
    public RigidBodyMotionState getMotionState() {
        return motionState;
    }

    /**
     * For compatibility with the jme3-jbullet library.
     *
     * @return a new Quaternion (relative to physics-space coordinates, not
     * null)
     */
    public Quaternion getPhysicsRotation() {
        return getPhysicsRotation(null);
    }

    /**
     * Calculate the squared speed of this body. The body must be in dynamic
     * mode.
     *
     * @return the squared speed (in physics-space units squared per second
     * squared, &ge;0)
     */
    public float getSquaredSpeed() {
        assert isDynamic();

        long objectId = nativeId();
        float result = getSquaredSpeed(objectId);

        return result;
    }

    /**
     * Test whether this body is in dynamic mode.
     *
     * @return true if in dynamic mode, otherwise false (static/kinematic mode)
     */
    public boolean isDynamic() {
        boolean result = (mass > massForStatic && !kinematic);
        return result;
    }

    /**
     * Test whether this body's gravity can be overwritten by PhysicsSpace.
     *
     * @return false if this body's gravity can be overwritten, otherwise true
     */
    public boolean isGravityProtected() {
        long objectId = nativeId();
        boolean result = !getUseSpaceGravity(objectId);

        return result;
    }

    /**
     * Test whether this body is in kinematic mode.
     * <p>
     * In kinematic mode, the body is not influenced by physics but can affect
     * other physics objects. Its kinetic force is calculated based on its
     * movement and weight.
     *
     * @return true if in kinematic mode, otherwise false (dynamic/static mode)
     */
    final public boolean isKinematic() {
        assert checkKinematicFlag() : "copy of flag = " + kinematic;
        return kinematic;
    }

    /**
     * Calculate this body's kinetic energy (linear + angular). The body must be
     * in dynamic mode.
     *
     * @return the total kinetic energy (&ge;0)
     */
    public double kineticEnergy() {
        assert isDynamic();

        double mv2 = mass * getSquaredSpeed();

        Vector3f vec = new Vector3f(); // TODO garbage
        getAngularVelocityLocal(vec);
        double xx = vec.x;
        double yy = vec.y;
        double zz = vec.z;
        Vector3f invI = getInverseInertiaLocal(null);
        double iw2 = xx * xx / invI.x + yy * yy / invI.y + zz * zz / invI.z;

        double result = (mv2 + iw2) / 2.0;

        assert result >= 0.0 : result;
        return result;
    }

    /**
     * Calculate the mechanical energy of this body (kinetic + potential)
     * assuming a uniform gravitational field. The body must be in dynamic mode.
     *
     * @return the total mechanical energy
     */
    public double mechanicalEnergy() {
        assert isDynamic();

        Vector3f gravity = getGravity(null);
        Vector3f location = getPhysicsLocation(null);
        double potentialEnergy = -mass * MyVector3f.dot(gravity, location);
        double result = potentialEnergy + kineticEnergy();

        return result;
    }

    /**
     * Rebuild this rigid body with a new native object.
     */
    public void rebuildRigidBody() {
        long oldId = 0L;
        PhysicsSpace removedFrom = null;
        RigidBodySnapshot snapshot = null;

        if (hasAssignedNativeObject()) {
            // Gather information regarding the existing native object.
            oldId = nativeId();
            removedFrom = (PhysicsSpace) getCollisionSpace();
            if (removedFrom != null) {
                removedFrom.removeCollisionObject(this);
            }
            snapshot = new RigidBodySnapshot(this);

            logger2.log(Level.INFO, "Clearing {0}.", this);
            clearIgnoreList();
            unassignNativeObject();
        }

        preRebuild();

        long msId = motionState.nativeId();
        CollisionShape shape = getCollisionShape();
        long shapeId = shape.nativeId();
        long objectId = createRigidBody(mass, msId, shapeId);
        setNativeId(objectId);
        assert getInternalType(objectId) == PcoType.rigid :
                getInternalType(objectId);
        if (logger2.isLoggable(Level.INFO)) {
            if (oldId == 0L) {
                logger2.log(
                        Level.INFO, "Created {0}.", Long.toHexString(objectId));
            } else {
                logger2.log(
                        Level.INFO, "Substituted {0} for {1}.", new Object[]{
                            Long.toHexString(objectId), Long.toHexString(oldId)
                        });
            }
        }

        if (mass != massForStatic) {
            setKinematic(kinematic);
        }

        postRebuild();

        if (removedFrom != null) {
            removedFrom.addCollisionObject(this);
        }
        if (snapshot != null) {
            snapshot.applyTo(this);
        }
        // TODO physics joints
    }

    /**
     * Alter this body's angular damping.
     *
     * @param angularDamping the desired angular damping fraction (&ge;0, &le;1,
     * default=0)
     */
    public void setAngularDamping(float angularDamping) {
        Validate.fraction(angularDamping, "angular damping");

        long objectId = nativeId();
        setAngularDamping(objectId, angularDamping);
    }

    /**
     * Alter this body's angular factor.
     *
     * @param factor the desired angular factor for all axes (not null,
     * unaffected, default=1)
     */
    public void setAngularFactor(float factor) {
        long objectId = nativeId();
        setAngularFactor(objectId, new Vector3f(factor, factor, factor));
    }

    /**
     * Alter this body's angular factors, used to scale applied torques.
     *
     * @param factor the desired angular factor for each axis (not null,
     * unaffected, default=(1,1,1))
     */
    public void setAngularFactor(Vector3f factor) {
        long objectId = nativeId();
        setAngularFactor(objectId, factor);
    }

    /**
     * Alter this body's angular-motion sleeping threshold. Note that "sleeping"
     * is synonym for "deactivation".
     *
     * @param threshold the desired threshold (in radians per second, &ge;0,
     * default=1)
     */
    public void setAngularSleepingThreshold(float threshold) {
        long objectId = nativeId();
        setAngularSleepingThreshold(objectId, threshold);
    }

    /**
     * Alter this body's angular velocity.
     *
     * @param omega the desired angular velocity (in physics-space coordinates,
     * not null, unaffected)
     */
    public void setAngularVelocity(Vector3f omega) {
        Validate.finite(omega, "omega");

        long objectId = nativeId();
        setAngularVelocity(objectId, omega);
        activate();
    }

    /**
     * Alter this body's angular velocity.
     *
     * @param omega the desired angular velocity (in physics-space coordinates,
     * not null, unaffected)
     */
    public void setAngularVelocityDp(Vec3d omega) {
        long objectId = nativeId();
        setAngularVelocityDp(objectId, omega);
        activate();
    }

    /**
     * Apply the specified CollisionShape to this body. The body gets rebuilt on
     * the native side.
     *
     * @param collisionShape the shape to apply (not null, alias created)
     */
    @Override
    public void setCollisionShape(CollisionShape collisionShape) {
        Validate.nonNull(collisionShape, "collision shape");
        if (isDynamic()) {
            validateDynamicShape(collisionShape);
        }

        super.setCollisionShape(collisionShape);

        if (hasAssignedNativeObject()) {
            long objectId = nativeId();
            long shapeId = collisionShape.nativeId();
            setCollisionShape(objectId, shapeId);
            updateMassProps(objectId, shapeId, mass);

        } else {
            rebuildRigidBody();
        }
    }

    /**
     * Enable/disable this body's contact response.
     *
     * @param newState true to respond to contacts, false to ignore them
     * (default=true)
     */
    public void setContactResponse(boolean newState) {
        int flags = collisionFlags();
        if (newState) {
            flags &= ~CollisionFlag.NO_CONTACT_RESPONSE;
        } else {
            flags |= CollisionFlag.NO_CONTACT_RESPONSE;
        }
        long objectId = nativeId();
        setCollisionFlags(objectId, flags);
    }

    /**
     * Alter this body's damping.
     *
     * @param linearDamping the desired linear damping fraction (&ge;0, &le;1,
     * default=0)
     * @param angularDamping the desired angular damping fraction (&ge;0, &le;1,
     * default=0)
     */
    public void setDamping(float linearDamping, float angularDamping) {
        Validate.fraction(linearDamping, "linear damping");
        Validate.fraction(angularDamping, "angular damping");

        long objectId = nativeId();
        setDamping(objectId, linearDamping, angularDamping);
    }

    /**
     * Alter this body's activation state to allow/disallow sleeping. Note that
     * "sleeping" is synonym for "deactivation".
     *
     * @param setting true&rarr;enable sleeping, false&rarr;disable sleeping
     * (default=true)
     */
    public void setEnableSleep(boolean setting) {
        long objectId = nativeId();
        if (setting) {
            setActivationState(objectId, Activation.active);
        } else {
            setActivationState(objectId, Activation.exempt);
        }
    }

    /**
     * Alter this body's gravitational acceleration.
     * <p>
     * Invoke this method <em>after</em> adding the body to a PhysicsSpace.
     * Unless protection is set, adding a body to a PhysicsSpace overrides its
     * gravity.
     *
     * @see #setProtectGravity(boolean)
     *
     * @param acceleration the desired acceleration vector (in physics-space
     * coordinates, not null, unaffected, default=(0,0,0))
     */
    public void setGravityDp(Vec3d acceleration) {
        Validate.nonNull(acceleration, "acceleration");
        if (!isInWorld() && !isGravityProtected()) {
            logger2.warning(
                    "The body isn't in any PhysicsSpace, and its gravity isn't"
                    + " protected. Unless protection is set, adding it"
                    + " to a PhysicsSpace will override its gravity."
            );
        }

        long objectId = nativeId();
        setGravityDp(objectId, acceleration);
    }

    /**
     * Alter the principal (diagonal) components of the local inertia tensor in
     * the body's local coordinates.
     *
     * @param inverseInertia the desired component values (not null, unaffected)
     */
    public void setInverseInertiaLocal(Vector3f inverseInertia) {
        Validate.nonNull(inverseInertia, "inverse inertia");

        long objectId = nativeId();
        setInverseInertiaLocal(objectId, inverseInertia);
    }

    /**
     * Put this body into kinematic mode or take it out of kinematic mode.
     * <p>
     * In kinematic mode, the body is not influenced by physics but can affect
     * other physics objects. Its kinetic force is calculated based on its mass
     * and motion.
     *
     * @param kinematic true&rarr;set kinematic mode, false&rarr;set dynamic
     * (default=false)
     */
    public void setKinematic(boolean kinematic) {
        if (mass == massForStatic) {
            throw new IllegalStateException(
                    "Cannot set/clear kinematic mode on a static body!");
        }
        assert !isStatic();

        this.kinematic = kinematic;
        long objectId = nativeId();
        setKinematic(objectId, kinematic);

        assert isKinematic() == kinematic : "copy of flag = " + kinematic;
    }

    /**
     * Alter this body's linear damping.
     *
     * @param linearDamping the desired linear damping fraction (&ge;0, &le;1,
     * default=0)
     */
    public void setLinearDamping(float linearDamping) {
        Validate.fraction(linearDamping, "linear damping");

        long objectId = nativeId();
        float angularDamping = getAngularDamping();
        setDamping(objectId, linearDamping, angularDamping);
    }

    /**
     * Alter this body's linear factors.
     *
     * @param factor the desired linear factor for each axis (not null,
     * unaffected, default=(1,1,1))
     */
    public void setLinearFactor(Vector3f factor) {
        Validate.nonNull(factor, "factor");

        long objectId = nativeId();
        setLinearFactor(objectId, factor);
    }

    /**
     * Alter this body's linear-motion sleeping threshold. Note that "sleeping"
     * is synonym for "deactivation".
     *
     * @param threshold the desired threshold (in physics-space units per
     * second, &ge;0, default=0.8)
     */
    public void setLinearSleepingThreshold(float threshold) {
        long objectId = nativeId();
        setLinearSleepingThreshold(objectId, threshold);
    }

    /**
     * Alter the linear velocity of this body's center of mass.
     *
     * @param velocity the desired velocity (physics-space units per second in
     * physics-space coordinates, not null, finite, unaffected)
     */
    public void setLinearVelocity(Vector3f velocity) {
        Validate.finite(velocity, "velocity");

        long objectId = nativeId();
        setLinearVelocity(objectId, velocity);
        activate();
    }

    /**
     * Alter the linear velocity of this body's center of mass.
     *
     * @param velocity the desired velocity (physics-space units per second in
     * physics-space coordinates, not null, unaffected)
     */
    public void setLinearVelocityDp(Vec3d velocity) {
        Validate.nonNull(velocity, "velocity");

        long objectId = nativeId();
        setLinearVelocityDp(objectId, velocity);
        activate();
    }

    /**
     * Directly relocate this body's center of mass.
     *
     * @param location the desired location (in physics-space coordinates, not
     * null, unaffected)
     */
    public void setPhysicsLocationDp(Vec3d location) {
        Validate.nonNull(location, "location");

        long objectId = nativeId();
        setPhysicsLocationDp(objectId, location);
    }

    /**
     * Directly alter this body's orientation.
     *
     * @param orientation the desired orientation (rotation matrix relative to
     * physics-space coordinates, not null, unaffected)
     */
    public void setPhysicsRotation(Matrix3f orientation) {
        Validate.nonNull(orientation, "rotation");
        if (getCollisionShape() instanceof HeightfieldCollisionShape
                && !orientation.isIdentity()) {
            throw new IllegalArgumentException("No rotation of heightfields.");
        }

        long objectId = nativeId();
        setPhysicsRotation(objectId, orientation);
    }

    /**
     * Directly reorient this body.
     *
     * @param orientation the desired orientation (relative to physics-space
     * coordinates, not null, not zero, unaffected)
     */
    public void setPhysicsRotation(Quaternion orientation) {
        Validate.nonZero(orientation, "orientation");
        if (getCollisionShape() instanceof HeightfieldCollisionShape
                && !MyQuaternion.isRotationIdentity(orientation)) {
            throw new IllegalArgumentException("No rotation of heightfields.");
        }

        long objectId = nativeId();
        setPhysicsRotation(objectId, orientation);
    }

    /**
     * Directly reorient this body.
     *
     * @param orientation the desired orientation (rotation matrix relative to
     * physics-space coordinates, not null, unaffected)
     */
    public void setPhysicsRotationDp(Matrix3d orientation) {
        Validate.nonNull(orientation, "orientation");
        if (getCollisionShape() instanceof HeightfieldCollisionShape
                && !orientation.isIdentity()) {
            throw new IllegalArgumentException("No rotation of heightfields.");
        }

        long objectId = nativeId();
        setPhysicsRotationDp(objectId, orientation);
    }

    /**
     * Directly reorient this body.
     *
     * @param orientation the desired orientation (relative to physics-space
     * coordinates, not null, unaffected)
     */
    public void setPhysicsRotationDp(Quatd orientation) {
        Validate.nonNull(orientation, "orientation");
        if (getCollisionShape() instanceof HeightfieldCollisionShape
                && !orientation.isRotationIdentity()) {
            throw new IllegalArgumentException("No rotation of heightfields.");
        }

        long objectId = nativeId();
        setPhysicsRotationDp(objectId, orientation);
    }

    /**
     * Rescale this body. Note that if it has joints, their pivot locations will
     * not be adjusted.
     *
     * @param newScale the desired scale factor for each local axis (not null,
     * no negative component, unaffected, default=(1,1,1))
     */
    public void setPhysicsScale(Vector3f newScale) {
        CollisionShape shape = getCollisionShape();
        Vector3f oldScale = shape.getScale(null); // TODO garbage
        if (MyVector3f.ne(oldScale, newScale)) {
            shape.setScale(newScale);
            setCollisionShape(shape);
        }
    }

    /**
     * Directly alter this body's transform, including the scale of its shape.
     * If the body has joints, their pivot points will not be adjusted for scale
     * changes.
     *
     * @param transform the desired transform (relative to physics-space
     * coordinates, not null, unaffected)
     */
    public void setPhysicsTransform(Transform transform) {
        setPhysicsLocation(transform.getTranslation());
        setPhysicsRotation(transform.getRotation());
        setPhysicsScale(transform.getScale());
    }

    /**
     * Alter whether this body's gravity should be overwritten if the body gets
     * added to a PhysicsSpace or the gravity of the PhysicsSpace changes.
     *
     * @param newState true to preserve this body's gravity, false to allow it
     * to be overwritten (default=false)
     */
    public void setProtectGravity(boolean newState) {
        long objectId = nativeId();
        setUseSpaceGravity(objectId, !newState);
    }

    /**
     * Alter this body's sleeping thresholds. Note that "sleeping" is synonym
     * for "deactivation".
     * <p>
     * These thresholds influence whether the body will be deactivated to save
     * resources. Low values keep the body active when it barely moves.
     *
     * @param linear the desired linear threshold (in physics-space units per
     * second, &ge;0, default=0.8)
     * @param angular the desired angular threshold (in radians per second,
     * &ge;0, default=1)
     */
    public void setSleepingThresholds(float linear, float angular) {
        Validate.nonNegative(linear, "linear threshold");
        Validate.nonNegative(angular, "angular threshold");

        long objectId = nativeId();
        setSleepingThresholds(objectId, linear, angular);
    }

    /**
     * Determine the total force applied to this body (excluding contact forces,
     * damping, and gravity).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the total force (either storeResult or a new vector, mass times
     * physics-space units per second squared in physics-space coordinates)
     */
    public Vector3f totalAppliedForce(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long objectId = nativeId();
        getTotalForce(objectId, result);

        return result;
    }

    /**
     * Determine the total torque applied to this body (excluding contact forces
     * and damping).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the total torque (either storeResult or a new vector, mass times
     * physics-space units squared per second squared in physics-space
     * coordinates)
     */
    public Vector3f totalAppliedTorque(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long objectId = nativeId();
        getTotalTorque(objectId, result);

        return result;
    }
    // *************************************************************************
    // new protected methods

    /**
     * For use by subclasses.
     */
    protected void postRebuild() {
        int flags = collisionFlags();
        if (mass == massForStatic) {
            flags |= CollisionFlag.STATIC_OBJECT;
        } else {
            flags &= ~CollisionFlag.STATIC_OBJECT;
        }
        long objectId = nativeId();
        setCollisionFlags(objectId, flags);

        initUserPointer();
    }

    /**
     * For use by subclasses.
     */
    protected void preRebuild() {
        // do nothing
    }
    // *************************************************************************
    // PhysicsBody methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned body into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this body (not null)
     * @param original the instance from which this body was shallow-cloned (not
     * null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        assert !hasAssignedNativeObject();
        PhysicsRigidBody old = (PhysicsRigidBody) original;
        assert old != this;
        assert old.hasAssignedNativeObject();

        RigidBodySnapshot snapshot = new RigidBodySnapshot(old);
        super.cloneFields(cloner, original);
        if (hasAssignedNativeObject()) {
            return;
        }

        this.motionState = cloner.clone(motionState);
        assert !hasAssignedNativeObject();

        CollisionShape shape = getCollisionShape();
        long objectId = createRigidBody(
                mass, motionState.nativeId(), shape.nativeId());
        setNativeId(objectId);
        assert getInternalType(objectId) == PcoType.rigid :
                getInternalType(objectId);
        logger2.log(Level.INFO, "Created " + Long.toHexString(objectId));
        if (mass != massForStatic) {
            setKinematic(kinematic);
        }
        postRebuild();
        snapshot.applyTo(this);

        Vector3f tmpVector = new Vector3f(); // TODO garbage
        setInverseInertiaLocal(old.getInverseInertiaLocal(tmpVector));

        clearIgnoreList();
        cloneIgnoreList(cloner, old);

        cloneJoints(cloner, old);
    }

    /**
     * Copy this body's gravitational acceleration.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return an acceleration vector in physics-space coordinates (either
     * storeResult or a new vector, not null)
     */
    @Override
    public Vector3f getGravity(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long objectId = nativeId();
        getGravity(objectId, result);

        return result;
    }

    /**
     * Read this body's mass.
     *
     * @return the mass (&gt;0) or zero for a static body
     */
    @Override
    public float getMass() {
        assert checkMass();
        return mass;
    }

    /**
     * De-serialize this body from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);

        InputCapsule capsule = importer.getCapsule(this);
        this.kinematic = capsule.readBoolean(tagKinematic, false);
        this.mass = capsule.readFloat(tagMass, 1f);
        this.motionState = (RigidBodyMotionState) capsule
                .readSavable(tagMotionState, null);
        if (motionState == null) {
            // de-serializing a model serialized prior to v7.0.0
            this.motionState = new RigidBodyMotionState();
        }

        rebuildRigidBody();

        setAngularVelocity((Vector3f) capsule
                .readSavable(tagAngularVelocity, translateIdentity));
        setLinearVelocity((Vector3f) capsule
                .readSavable(tagLinearVelocity, translateIdentity));
        applyCentralForce((Vector3f) capsule
                .readSavable(tagAppliedForce, translateIdentity));
        applyTorque((Vector3f) capsule
                .readSavable(tagAppliedTorque, translateIdentity));
        /*
         * Set force, torque, velocities, and kinematic flag
         * BEFORE readPcoProperties() because
         * those setters also affect deactivation time.
         */
        readPcoProperties(capsule);

        setContactResponse(capsule.readBoolean(tagContactResponse, true));
        setInverseInertiaLocal((Vector3f) capsule
                .readSavable(tagInverseInertia, scaleIdentity));
        setAngularFactor((Vector3f) capsule
                .readSavable(tagAngularFactor, scaleIdentity));
        setLinearFactor((Vector3f) capsule
                .readSavable(tagLinearFactor, scaleIdentity));
        setDamping(capsule.readFloat(tagLinearDamping, 0f),
                capsule.readFloat(tagAngularDamping, 0f));
        setSleepingThresholds(
                capsule.readFloat(tagLinearSleepingThreshold, 0.8f),
                capsule.readFloat(tagAngularSleepingThreshold, 1f));
        setProtectGravity(capsule.readBoolean(tagProtectGravity, false));

        setPhysicsLocation((Vector3f) capsule
                .readSavable(tagPhysicsLocation, translateIdentity));
        setPhysicsRotation((Matrix3f) capsule
                .readSavable(tagPhysicsRotation, matrixIdentity));

        readJoints(capsule);
    }

    /**
     * Alter this body's gravitational acceleration.
     * <p>
     * Invoke this method <em>after</em> adding the body to a PhysicsSpace.
     * Unless protection is set, adding a body to a PhysicsSpace overrides its
     * gravity.
     *
     * @see #setProtectGravity(boolean)
     * @see #setGravityDp(com.simsilica.mathd.Vec3d)
     *
     * @param acceleration the desired acceleration vector (in physics-space
     * coordinates, not null, finite, unaffected, default=(0,0,0))
     */
    @Override
    public void setGravity(Vector3f acceleration) {
        Validate.finite(acceleration, "acceleration");
        if (!isInWorld() && !isGravityProtected()) {
            logger2.warning(
                    "The body isn't in any PhysicsSpace, and its gravity isn't"
                    + " protected. Unless protection is set, adding it"
                    + " to a PhysicsSpace will override its gravity."
            );
        }

        long objectId = nativeId();
        setGravity(objectId, acceleration);
    }

    /**
     * Alter this body's mass. Bodies with mass=0 are static. For dynamic
     * bodies, it is best to keep the mass on the order of 1.
     *
     * @param mass the desired mass (&gt;0) or 0 for a static body (default=1)
     */
    @Override
    public void setMass(float mass) {
        Validate.nonNegative(mass, "mass");
        CollisionShape shape = getCollisionShape();
        assert shape != null;
        if (mass != massForStatic) {
            validateDynamicShape(shape);
        }
        assert hasAssignedNativeObject();

        if (mass == this.mass) {
            return;

        } else if (this.mass == massForStatic) {
            // Static to non-static requires a rebuild.
            this.mass = mass;
            rebuildRigidBody();
            return;
        }

        this.mass = mass;
        long objectId = nativeId();
        updateMassProps(objectId, shape.nativeId(), mass);

        int flags = collisionFlags();
        if (mass == massForStatic) {
            flags |= CollisionFlag.STATIC_OBJECT;
        } else {
            flags &= ~CollisionFlag.STATIC_OBJECT;
        }
        setCollisionFlags(objectId, flags);
    }

    /**
     * Directly relocate this body's center of mass.
     *
     * @param location the desired location (in physics-space coordinates, not
     * null, finite, unaffected)
     */
    @Override
    public void setPhysicsLocation(Vector3f location) {
        Validate.finite(location, "location");

        long objectId = nativeId();
        setPhysicsLocation(objectId, location);
    }

    /**
     * Serialize this body to the specified exporter, for example when saving to
     * a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        super.write(exporter);
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(getMass(), tagMass, 1f);
        capsule.write(isContactResponse(), tagContactResponse, true);
        capsule.write(getAngularFactor(null), tagAngularFactor, null);
        capsule.write(getLinearFactor(null), tagLinearFactor, null);
        capsule.write(kinematic, tagKinematic, false);
        capsule.write(motionState, tagMotionState, null);

        capsule.write(getInverseInertiaLocal(null), tagInverseInertia, null);
        capsule.write(getLinearDamping(), tagLinearDamping, 0f);
        capsule.write(getAngularDamping(), tagAngularDamping, 0f);
        capsule.write(
                getLinearSleepingThreshold(), tagLinearSleepingThreshold, 0.8f);
        capsule.write(
                getAngularSleepingThreshold(), tagAngularSleepingThreshold, 1f);
        capsule.write(isGravityProtected(), tagProtectGravity, false);

        capsule.write(getPhysicsLocation(null), tagPhysicsLocation, null);
        capsule.write(getPhysicsRotationMatrix(null), tagPhysicsRotation, null);
        if (isDynamic()) {
            capsule.write(getLinearVelocity(null), tagLinearVelocity, null);
            capsule.write(getAngularVelocity(null), tagAngularVelocity, null);
        }
        capsule.write(totalAppliedForce(null), tagAppliedForce, null);
        capsule.write(totalAppliedTorque(null), tagAppliedTorque, null);

        writeJoints(capsule);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Compare Bullet's mass to the local copy.
     *
     * @return true if the masses are approximately equal, otherwise false
     */
    private boolean checkMass() {
        long objectId = nativeId();
        float nativeMass = getMass(objectId);
        boolean result = FastMath.approximateEquals(nativeMass, mass);

        return result;
    }

    /**
     * Compare Bullet's kinematic flag to the local copy.
     *
     * @return true if the flags are equal, otherwise false
     */
    private boolean checkKinematicFlag() {
        if (mass == massForStatic) {
            return true; // TODO
        }

        int flags = collisionFlags();
        boolean nativeKinematicFlag
                = (flags & CollisionFlag.KINEMATIC_OBJECT) != 0x0;

        if (kinematic == nativeKinematicFlag) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Validate a shape as suitable for a dynamic body.
     *
     * @param shape (not null, unaffected)
     */
    private static void validateDynamicShape(CollisionShape shape) {
        assert shape != null;

        if (shape.isNonMoving()) {
            throw new IllegalStateException(
                    "Dynamic rigid body can't have a non-moving shape!");
        }
    }
    // *************************************************************************
    // native private methods

    native private static void applyCentralForce(long objectId, Vector3f force);

    native private static void
            applyCentralImpulse(long objectId, Vector3f impulse);

    native private static void
            applyForce(long objectId, Vector3f force, Vector3f localOffset);

    native private static void
            applyImpulse(long objectId, Vector3f impulse, Vector3f localOffset);

    native private static void applyTorque(long objectId, Vector3f torque);

    native private static void
            applyTorqueImpulse(long objectId, Vector3f torqueImpulse);

    native private static void clearForces(long objectId);

    native private static long createRigidBody(
            float mass, long motionStateId, long collisionShapeId);

    native private static float getAngularDamping(long objectId);

    native private static void
            getAngularFactor(long objectId, Vector3f storeResult);

    native private static float getAngularSleepingThreshold(long objectId);

    native private static void
            getAngularVelocity(long objectId, Vector3f storeResult);

    native private static void
            getAngularVelocityDp(long objectId, Vec3d storeResult);

    native private static void getGravity(long objectId, Vector3f storeResult);

    native private static void getGravityDp(long objectId, Vec3d storeResult);

    native private static void
            getInverseInertiaLocal(long objectId, Vector3f storeResult);

    native private static void
            getInverseInertiaWorld(long objectId, Matrix3f storeResult);

    native private static float getLinearDamping(long objectId);

    native private static void
            getLinearFactor(long objectId, Vector3f storeResult);

    native private static float getLinearSleepingThreshold(long objectId);

    native private static void
            getLinearVelocity(long objectId, Vector3f storeResult);

    native private static void
            getLinearVelocityDp(long objectId, Vec3d storeResult);

    native private static float getMass(long objectId);

    native private static float getSquaredSpeed(long objectId);

    native private static void
            getTotalForce(long objectId, Vector3f storeResult);

    native private static void
            getTotalTorque(long objectId, Vector3f storeResult);

    native private static boolean getUseSpaceGravity(long objectId);

    native private static void
            setAngularDamping(long objectId, float dampingFraction);

    native private static void setAngularFactor(long objectId, Vector3f factor);

    native private static void
            setAngularSleepingThreshold(long objectId, float threshold);

    native private static void setAngularVelocity(long objectId, Vector3f vec);

    native private static void setAngularVelocityDp(long objectId, Vec3d vec);

    native private static void
            setCollisionShape(long objectId, long collisionShapeId);

    native private static void
            setDamping(long objectId, float linear, float angular);

    native private static void setGravity(long objectId, Vector3f gravity);

    native private static void setGravityDp(long objectId, Vec3d gravity);

    native private static void setInverseInertiaLocal(
            long objectId, Vector3f inverseInertialLocal);

    native private static void setKinematic(long objectId, boolean kinematic);

    native private static void setLinearFactor(long objectId, Vector3f factor);

    native private static void
            setLinearSleepingThreshold(long objectId, float threshold);

    native private static void setLinearVelocity(long objectId, Vector3f vec);

    native private static void setLinearVelocityDp(long objectId, Vec3d vec);

    native private static void
            setPhysicsLocation(long objectId, Vector3f location);

    native private static void
            setPhysicsLocationDp(long objectId, Vec3d location);

    native private static void
            setPhysicsRotation(long objectId, Matrix3f rotation);

    native private static void
            setPhysicsRotation(long objectId, Quaternion rotation);

    native private static void
            setPhysicsRotationDp(long objectId, Matrix3d rotation);

    native private static void
            setPhysicsRotationDp(long objectId, Quatd rotation);

    native private static void
            setSleepingThresholds(long objectId, float linear, float angular);

    native private static void
            setUseSpaceGravity(long objectId, boolean useSpaceGravity);

    native private static void
            updateMassProps(long objectId, long collisionShapeId, float mass);
}
