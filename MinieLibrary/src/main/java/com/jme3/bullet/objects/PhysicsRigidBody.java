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
package com.jme3.bullet.objects;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.HeightfieldCollisionShape;
import com.jme3.bullet.collision.shapes.MeshCollisionShape;
import com.jme3.bullet.collision.shapes.PlaneCollisionShape;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.objects.infos.RigidBodyMotionState;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyQuaternion;
import jme3utilities.math.MyVector3f;

/**
 * A collision object for a rigid body, based on Bullet's btRigidBody.
 *
 * @author normenhansen
 */
public class PhysicsRigidBody extends PhysicsCollisionObject {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(PhysicsRigidBody.class.getName());
    /**
     * magic mass value used to specify a static body
     */
    final public static float massForStatic = 0f;
    // *************************************************************************
    // fields

    /**
     * motion state
     */
    protected RigidBodyMotionState motionState = new RigidBodyMotionState();
    /**
     * copy of the mass (&gt;0) of a dynamic body, or 0 for a static body
     * (default=1)
     */
    protected float mass = 1f;
    /**
     * copy of kinematic flag: true&rarr;set kinematic mode (spatial controls
     * body), false&rarr;dynamic/static mode (body controls spatial)
     * (default=false)
     */
    private boolean kinematic = false;
    /**
     * list of joints that connect to this body: The list isn't filled until the
     * body is added to a PhysicsSpace.
     */
    private ArrayList<PhysicsJoint> joints = new ArrayList<>(4);
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public PhysicsRigidBody() {
    }

    /**
     * Instantiate a dynamic body with mass=1 and the specified collision shape.
     *
     * @param shape the desired shape (not null, alias created)
     */
    public PhysicsRigidBody(CollisionShape shape) {
        Validate.nonNull(shape, "shape");

        collisionShape = shape;
        rebuildRigidBody();
    }

    /**
     * Instantiate a body with the specified collision shape and mass.
     *
     * @param shape the desired shape (not null, alias created)
     * @param mass if 0, a static body is created; otherwise a dynamic body is
     * created (&ge;0)
     */
    public PhysicsRigidBody(CollisionShape shape, float mass) {
        Validate.nonNull(shape, "shape");
        Validate.nonNegative(mass, "mass");

        collisionShape = shape;
        this.mass = mass;
        rebuildRigidBody();
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
     * Do not invoke directly! Joints are added automatically when created.
     *
     * @param joint the joint to add (not null)
     */
    public void addJoint(PhysicsJoint joint) {
        if (!joints.contains(joint)) {
            joints.add(joint);
        }
    }

    /**
     * Apply a force to the PhysicsRigidBody. Effective only if the next physics
     * update steps the physics.
     * <p>
     * To apply an impulse, use
     * {@link #applyImpulse(com.jme3.math.Vector3f, com.jme3.math.Vector3f)}.
     *
     * @param force the force (not null, unaffected)
     */
    public void applyCentralForce(Vector3f force) {
        applyCentralForce(objectId, force);
        activate();
    }

    /**
     * Apply a force to the PhysicsRigidBody. Effective only if the next physics
     * update steps the physics.
     * <p>
     * To apply an impulse, use applyImpulse, use applyContinuousForce to apply
     * continuous force.
     *
     * @param force the force (not null, unaffected)
     * @param location the location of the force
     */
    public void applyForce(Vector3f force, Vector3f location) {
        applyForce(objectId, force, location);
        activate();
    }

    /**
     * Apply an impulse to the body on the next physics update.
     *
     * @param impulse applied impulse (not null, unaffected)
     * @param rel_pos the location (in local coordinates, not null, unaffected)
     */
    public void applyImpulse(Vector3f impulse, Vector3f rel_pos) {
        applyImpulse(objectId, impulse, rel_pos);
        activate();
    }

    /**
     * Apply a torque to the PhysicsRigidBody. Effective only if the next
     * physics update steps the physics.
     * <p>
     * To apply an impulse, use
     * {@link #applyImpulse(com.jme3.math.Vector3f, com.jme3.math.Vector3f)}.
     *
     * @param torque the torque (not null, unaffected)
     */
    public void applyTorque(Vector3f torque) {
        applyTorque(objectId, torque);
        activate();
    }

    /**
     * Apply a torque impulse to the body in the next physics update.
     *
     * @param vec the torque to apply (not null, unaffected)
     */
    public void applyTorqueImpulse(Vector3f vec) {
        applyTorqueImpulse(objectId, vec);
        activate();
    }

    /**
     * Clear all forces acting on this body.
     */
    public void clearForces() {
        clearForces(objectId);
    }

    /**
     * Read this body's angular damping.
     *
     * @return damping value
     */
    public float getAngularDamping() {
        return getAngularDamping(objectId);
    }

    /**
     * Read this body's angular factor for the X axis.
     *
     * @return the angular factor
     */
    public float getAngularFactor() {
        return getAngularFactor(null).getX();
    }

    /**
     * Copy this body's angular factors.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the angular factor for each axis (either storeResult or a new
     * vector, not null)
     */
    public Vector3f getAngularFactor(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getAngularFactor(objectId, result);
        return result;
    }

    /**
     * Read this body's angular sleeping threshold.
     *
     * @return the angular sleeping threshold (&ge;0)
     */
    public float getAngularSleepingThreshold() {
        return getAngularSleepingThreshold(objectId);
    }

    /**
     * Copy this body's angular velocity.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a velocity vector (in physics-space coordinates, either
     * storeResult or a new vector, not null))
     */
    public Vector3f getAngularVelocity(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getAngularVelocity(objectId, result);
        return result;
    }

    /**
     * Calculate this body's continuous collision detection (CCD) motion
     * threshold.
     *
     * @return the threshold velocity (in physics-space units per second, &ge;0)
     */
    public float getCcdMotionThreshold() {
        return getCcdMotionThreshold(objectId);
    }

    /**
     * Calculate the square of this body's continuous collision detection (CCD)
     * motion threshold.
     *
     * @return the threshold velocity squared (&ge;0)
     */
    public float getCcdSquareMotionThreshold() {
        return getCcdSquareMotionThreshold(objectId);
    }

    /**
     * Read the radius of the swept sphere used for continuous collision
     * detection (CCD).
     *
     * @return radius (in physics-space units, &ge;0)
     */
    public float getCcdSweptSphereRadius() {
        return getCcdSweptSphereRadius(objectId);
    }

    /**
     * Read this body's friction.
     *
     * @return friction value
     */
    public float getFriction() {
        return getFriction(objectId);
    }

    /**
     * Copy this body's gravitational acceleration.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return an acceleration vector (in physics-space units per second
     * squared, either storeResult or a new vector, not null)
     */
    public Vector3f getGravity(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getGravity(objectId, result);
        return result;
    }

    /**
     * Copy the principal components of the local inverse inertia tensor. TODO
     * provide access to the whole tensor
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a vector (either storeResult or a new vector, not null)
     */
    public Vector3f getInverseInertiaLocal(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getInverseInertiaLocal(objectId, result);
        return result;
    }

    /**
     * Read this body's linear damping.
     *
     * @return damping value
     */
    public float getLinearDamping() {
        return getLinearDamping(objectId);
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
        getLinearFactor(objectId, result);
        return result;
    }

    /**
     * Read this body's linear sleeping threshold.
     *
     * @return the linear sleeping threshold (&ge;0)
     */
    public float getLinearSleepingThreshold() {
        return getLinearSleepingThreshold(objectId);
    }

    /**
     * Copy the linear velocity of this body's center of mass.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a velocity vector (in physics-space coordinates, either
     * storeResult or a new vector, not null)
     */
    public Vector3f getLinearVelocity(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getLinearVelocity(objectId, result);
        return result;
    }

    /**
     * Read this body's mass.
     *
     * @return the mass (&gt;0) or zero for a static body
     */
    public float getMass() {
        return mass;
    }

    /**
     * Access this body's motion state.
     *
     * @return the pre-existing instance
     */
    public RigidBodyMotionState getMotionState() {
        return motionState;
    }

    /**
     * Copy the location of this body's center of mass.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the location (in physics-space coordinates, either storeResult or
     * a new vector, not null)
     */
    public Vector3f getPhysicsLocation(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getPhysicsLocation(objectId, result);

        assert Vector3f.isValidVector(result);
        return result;
    }

    /**
     * Copy this body's orientation to a quaternion.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the orientation (in physics-space coordinates, either storeResult
     * or a new quaternion, not null)
     */
    public Quaternion getPhysicsRotation(Quaternion storeResult) {
        Quaternion result
                = (storeResult == null) ? new Quaternion() : storeResult;
        getPhysicsRotation(objectId, result);
        return result;
    }

    /**
     * Copy this body's orientation to a matrix.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the orientation (in physics-space coordinates, either storeResult
     * or a new matrix, not null)
     */
    public Matrix3f getPhysicsRotationMatrix(Matrix3f storeResult) {
        Matrix3f result = (storeResult == null) ? new Matrix3f() : storeResult;
        getPhysicsRotationMatrix(objectId, result);
        return result;
    }

    /**
     * Copy the scale of the body's shape.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the scaling factor for each local axis (either storeResult or a
     * new vector, not null)
     */
    public Vector3f getPhysicsScale(Vector3f storeResult) {
        Vector3f result = collisionShape.getScale(storeResult);

        assert Vector3f.isValidVector(result);
        return result;
    }

    /**
     * Copy the transform of this body, including the scale of its shape.
     *
     * @param storeResult (modified if not null)
     * @return the transform (in physics-space coordinates, either storeResult
     * of a new object, not null)
     */
    public Transform getPhysicsTransform(Transform storeResult) {
        Transform result
                = (storeResult == null) ? new Transform() : storeResult;

        motionState.getLocation(result.getTranslation());
        motionState.getOrientation(result.getRotation());
        getPhysicsScale(result.getScale());

        return result;
    }

    /**
     * Read this body's restitution (bounciness).
     *
     * @return restitution value
     */
    public float getRestitution() {
        return getRestitution(objectId);
    }

    /**
     * Test whether this body has been deactivated due to lack of motion. TODO
     * move to PhysicsCollisionObject
     *
     * @return true if still active, false if deactivated
     */
    public boolean isActive() {
        return isActive(objectId);
    }

    /**
     * Test whether this body is added to any physics space.
     *
     * @return true&rarr;in a space, false&rarr;not in a space
     */
    public boolean isInWorld() {
        return isInWorld(objectId);
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
    public boolean isKinematic() {
        return kinematic;
    }

    // TODO add isDynamic() and isStatic()
    /**
     * Enumerate the joints connected to this body.
     *
     * @return a new array of pre-existing objects, or null if this body is not
     * added to any space
     */
    public PhysicsJoint[] listJoints() {
        PhysicsJoint[] result;
        if (isInWorld(objectId)) {
            int numJoints = joints.size();
            result = new PhysicsJoint[numJoints];
            joints.toArray(result);
        } else {
            result = null;
        }

        return result;
    }

    /**
     * Do not invoke directly! Joints are removed automatically when destroyed.
     *
     * @param joint the joint to remove (not null)
     */
    public void removeJoint(PhysicsJoint joint) {
        joints.remove(joint);
    }

    /**
     * Alter this body's angular damping.
     *
     * @param angularDamping the desired angular damping value (default=0)
     */
    public void setAngularDamping(float angularDamping) {
        setAngularDamping(objectId, angularDamping);
    }

    /**
     * Alter this body's angular factor.
     *
     * @param factor the desired angular factor for all axes (not null,
     * unaffected, default=1)
     */
    public void setAngularFactor(float factor) {
        setAngularFactor(objectId, new Vector3f(factor, factor, factor));
    }

    /**
     * Alter this body's angular factors.
     *
     * @param factor the desired angular factor for each axis (not null,
     * unaffected, default=(1,1,1))
     */
    public void setAngularFactor(Vector3f factor) {
        setAngularFactor(objectId, factor);
    }

    /**
     * Alter this body's angular sleeping threshold.
     *
     * @param angularSleepingThreshold the desired threshold (&ge;0, default=1)
     */
    public void setAngularSleepingThreshold(float angularSleepingThreshold) {
        float lst = getLinearSleepingThreshold(); // work around JME issue #911
        setSleepingThresholds(objectId, lst, angularSleepingThreshold);
    }

    /**
     * Alter this body's angular velocity.
     *
     * @param vec the desired angular velocity vector (not null, unaffected)
     */
    public void setAngularVelocity(Vector3f vec) {
        setAngularVelocity(objectId, vec);
        activate();
    }

    /**
     * Alter the amount of motion required to activate continuous collision
     * detection (CCD).
     * <p>
     * This addresses the issue of fast objects passing through other objects
     * with no collision detected.
     *
     * @param threshold the desired threshold velocity (in physics-space units
     * per second, &gt;0) or zero to disable CCD (default=0)
     */
    public void setCcdMotionThreshold(float threshold) {
        setCcdMotionThreshold(objectId, threshold);
    }

    /**
     * Alter the radius of the swept sphere used for continuous collision
     * detection (CCD).
     *
     * @param radius the desired radius (in physics-space units, &ge;0,
     * default=0)
     */
    public void setCcdSweptSphereRadius(float radius) {
        setCcdSweptSphereRadius(objectId, radius);
    }

    /**
     * Apply the specified CollisionShape to this body, which must not be in any
     * physics space. The body gets rebuilt on the physics side.
     *
     * @param collisionShape the shape to apply (not null, alias created)
     */
    @Override
    public void setCollisionShape(CollisionShape collisionShape) {
        Validate.nonNull(collisionShape, "collision shape");
        //if (isInWorld()) {
        //    throw new IllegalStateException(
        //            "Cannot reshape body while in physics space!");
        // }
        if (mass != massForStatic) {
            validateDynamicShape(collisionShape);
        }

        super.setCollisionShape(collisionShape);

        if (objectId == 0L) {
            rebuildRigidBody();
        } else {
            setCollisionShape(objectId, collisionShape.getObjectId());
            updateMassProps(objectId, collisionShape.getObjectId(), mass);
        }
    }

    /**
     * Alter this body's damping.
     *
     * @param linearDamping the desired linear damping value (default=0)
     * @param angularDamping the desired angular damping value (default=0)
     */
    public void setDamping(float linearDamping, float angularDamping) {
        setDamping(objectId, linearDamping, angularDamping);
    }

    /**
     * Alter this body's friction.
     *
     * @param friction the desired friction value (default=0.5)
     */
    public void setFriction(float friction) {
        setFriction(objectId, friction);
    }

    /**
     * Alter this body's gravitational acceleration.
     * <p>
     * Invoke this method <em>after</em> adding the body to a PhysicsSpace.
     * Adding a body to a PhysicsSpace alters its gravity.
     *
     * @param gravity the desired acceleration vector (in physics-space units
     * per second squared, not null, unaffected)
     */
    public void setGravity(Vector3f gravity) {
        setGravity(objectId, gravity);
    }

    /**
     * Alter the principal components of the local inertia tensor. TODO provide
     * access to the whole tensor
     *
     * @param inverseInertia (not null, unaffected)
     */
    public void setInverseInertiaLocal(Vector3f inverseInertia) {
        Validate.nonNull(inverseInertia, "inverse inertia");
        setInverseInertiaLocal(objectId, inverseInertia);
    }

    /**
     * Put this body into kinematic mode or take it out of kinematic mode.
     * <p>
     * In kinematic mode, the body is not influenced by physics but can affect
     * other physics objects. Its kinetic force is calculated based on its
     * movement and weight.
     *
     * @param kinematic true&rarr;set kinematic mode, false&rarr;set
     * dynamic/static mode (default=false)
     */
    public void setKinematic(boolean kinematic) {
        this.kinematic = kinematic;
        setKinematic(objectId, kinematic);
    }

    /**
     * Alter this body's linear damping.
     *
     * @param linearDamping the desired linear damping value (default=0)
     */
    public void setLinearDamping(float linearDamping) {
        setDamping(objectId, linearDamping, getAngularDamping());
    }

    /**
     * Alter this body's linear factors.
     *
     * @param factor the desired linear factor for each axis (not null,
     * unaffected, default=(1,1,1))
     */
    public void setLinearFactor(Vector3f factor) {
        setLinearFactor(objectId, factor);
    }

    /**
     * Alter this body's linear sleeping threshold.
     *
     * @param linearSleepingThreshold the desired threshold (&ge;0, default=0.8)
     */
    public void setLinearSleepingThreshold(float linearSleepingThreshold) {
        float ast = getAngularSleepingThreshold(); // work around JME issue #911
        setSleepingThresholds(objectId, linearSleepingThreshold, ast);
    }

    /**
     * Alter the linear velocity of this body's center of mass.
     *
     * @param vec the desired velocity vector (in physics-space coordinates, not
     * null, unaffected)
     */
    public void setLinearVelocity(Vector3f vec) {
        setLinearVelocity(objectId, vec);
        activate();
    }

    /**
     * Alter this body's mass. Bodies with mass=0 are static. For dynamic
     * bodies, it is best to keep the mass on the order of 1.
     *
     * @param mass the desired mass (&gt;0) or 0 for a static body (default=1)
     */
    public void setMass(float mass) {
        Validate.nonNegative(mass, "mass");
        if (mass != massForStatic) {
            validateDynamicShape(collisionShape);
        }

        this.mass = mass;
        if (objectId != 0L) { // TODO necessary?
            if (collisionShape != null) {
                updateMassProps(objectId, collisionShape.getObjectId(), mass);
            }
            if (mass == massForStatic) {
                setStatic(objectId, true);
            } else {
                setStatic(objectId, false);
            }
        }
    }

    /**
     * Directly alter the location of this body's center of mass.
     *
     * @param location the desired location (in physics-space coordinates, not
     * null, unaffected)
     */
    public void setPhysicsLocation(Vector3f location) {
        Validate.finite(location, "location");
        if (collisionShape instanceof HeightfieldCollisionShape
                && !MyVector3f.isZero(location)) {
            throw new IllegalArgumentException(
                    "No translation of heightfields.");
        }

        setPhysicsLocation(objectId, location);
    }

    /**
     * Directly alter this body's orientation.
     *
     * @param rotation the desired orientation (rotation matrix in physics-space
     * coordinates, not null, unaffected)
     */
    public void setPhysicsRotation(Matrix3f rotation) {
        Validate.nonNull(rotation, "rotation");
        if (collisionShape instanceof HeightfieldCollisionShape
                && !rotation.isIdentity()) {
            throw new IllegalArgumentException("No rotation of heightfields.");
        }

        setPhysicsRotation(objectId, rotation);
    }

    /**
     * Directly alter this body's orientation.
     *
     * @param rotation the desired orientation (unit quaternion in physics-space
     * coordinates, not null, unaffected)
     */
    public void setPhysicsRotation(Quaternion rotation) {
        Validate.nonNull(rotation, "rotation");
        if (collisionShape instanceof HeightfieldCollisionShape
                && !MyQuaternion.isRotationIdentity(rotation)) {
            throw new IllegalArgumentException("No rotation of heightfields.");
        }

        setPhysicsRotation(objectId, rotation);
    }

    /**
     * Rescale this body. Note that if it has joints, their pivot points will
     * not be adjusted.
     *
     * @param newScale the desired scaling factor for each local axis (not null,
     * no negative component, unaffected, default=1,1,1)
     */
    public void setPhysicsScale(Vector3f newScale) {
        CollisionShape shape = getCollisionShape();
        Vector3f oldScale = shape.getScale(null);
        if (!newScale.equals(oldScale)) {
            //if (isInWorld()) {
            //    throw new IllegalStateException(
            //            "Cannot scale body while in physics space!");
            //}
            shape.setScale(newScale);
            setCollisionShape(shape);
        }
    }

    /**
     * Alter this body's transform, including the scale of its shape. Note that
     * if it has joints, their pivot points will not be adjusted for scale
     * changes.
     *
     * @param newTransform the desired transform (in physics-space coordinates,
     * not null, unaffected)
     */
    public void setPhysicsTransform(Transform newTransform) {
        setPhysicsLocation(newTransform.getTranslation());
        setPhysicsRotation(newTransform.getRotation());
        setPhysicsScale(newTransform.getScale());
    }

    /**
     * Alter this body's restitution (bounciness). For best performance, set
     * restitution=0.
     *
     * @param restitution the desired value (default=0)
     */
    public void setRestitution(float restitution) {
        setRestitution(objectId, restitution);
    }

    /**
     * Alter this body's sleeping thresholds.
     * <p>
     * These thresholds determine when the body can be deactivated to save
     * resources. Low values keep the body active when it barely moves.
     *
     * @param linear the desired linear sleeping threshold (&ge;0, default=0.8)
     * @param angular the desired angular sleeping threshold (&ge;0, default=1)
     */
    public void setSleepingThresholds(float linear, float angular) {
        setSleepingThresholds(objectId, linear, angular);
    }
    // *************************************************************************
    // new protected methods

    /**
     * For use by subclasses.
     */
    protected void postRebuild() {
        if (mass == massForStatic) {
            setStatic(objectId, true);
        } else {
            setStatic(objectId, false);
        }
        initUserPointer();
    }

    /**
     * For use by subclasses.
     */
    protected void preRebuild() {
    }

    /**
     * Build/rebuild this body after parameters have changed.
     */
    protected void rebuildRigidBody() {
        boolean removed = false;
        if (mass != massForStatic) {
            validateDynamicShape(collisionShape);
        }

        if (objectId != 0L) {
            if (isInWorld(objectId)) {
                PhysicsSpace.getPhysicsSpace().remove(this);
                removed = true;
            }
            logger2.log(Level.FINE, "Clearing RigidBody {0}",
                    Long.toHexString(objectId));
            finalizeNative(objectId);
        }
        preRebuild();
        objectId = createRigidBody(mass, motionState.getObjectId(),
                collisionShape.getObjectId());
        logger2.log(Level.FINE, "Created RigidBody {0}",
                Long.toHexString(objectId));
        postRebuild();
        if (removed) {
            PhysicsSpace.getPhysicsSpace().add(this);
        }
    }

    // *************************************************************************
    // PhysicsCollisionObject methods
    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned body into a deep-cloned one, using the specified cloner
     * and original to resolve copied fields.
     *
     * @param cloner the cloner that's cloning this body (not null)
     * @param original the instance from which this instance was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);
        rebuildRigidBody();

        joints = cloner.clone(joints);
        motionState = cloner.clone(motionState);

        PhysicsRigidBody old = (PhysicsRigidBody) original;
        setAngularDamping(old.getAngularDamping());
        setAngularFactor(old.getAngularFactor());
        setAngularSleepingThreshold(old.getAngularSleepingThreshold());
        setAngularVelocity(old.getAngularVelocity(null));
        setCcdMotionThreshold(old.getCcdMotionThreshold());
        setCcdSweptSphereRadius(old.getCcdSweptSphereRadius());
        setFriction(old.getFriction());
        setGravity(old.getGravity(null));
        setLinearDamping(old.getLinearDamping());
        setLinearFactor(old.getLinearFactor(null));
        setLinearSleepingThreshold(old.getLinearSleepingThreshold());
        setPhysicsLocation(old.getPhysicsLocation(null));
        setPhysicsRotation(old.getPhysicsRotationMatrix(null));
        setRestitution(old.getRestitution());
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public PhysicsRigidBody jmeClone() {
        try {
            PhysicsRigidBody clone = (PhysicsRigidBody) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }

    /**
     * De-serialize this body, for example when loading from a J3O file.
     *
     * @param im importer (not null)
     * @throws IOException from importer
     */
    @Override
    @SuppressWarnings("unchecked")
    public void read(JmeImporter im) throws IOException {
        super.read(im);

        InputCapsule capsule = im.getCapsule(this);
        mass = capsule.readFloat("mass", 1f);
        rebuildRigidBody();
        setGravity((Vector3f) capsule.readSavable("gravity",
                Vector3f.ZERO.clone()));
        setFriction(capsule.readFloat("friction", 0.5f));
        setKinematic(capsule.readBoolean("kinematic", false));

        setRestitution(capsule.readFloat("restitution", 0f));
        Vector3f angularFactor = (Vector3f) capsule.readSavable("angularFactor",
                null);
        if (angularFactor == null) {
            setAngularFactor(capsule.readFloat("angularFactor", 1f));
        } else {
            setAngularFactor(angularFactor);
            setLinearFactor((Vector3f) capsule.readSavable("linearFactor",
                    Vector3f.UNIT_XYZ.clone()));
        }
        setDamping(capsule.readFloat("linearDamping", 0f),
                capsule.readFloat("angularDamping", 0f));
        setSleepingThresholds(
                capsule.readFloat("linearSleepingThreshold", 0.8f),
                capsule.readFloat("angularSleepingThreshold", 1f));
        setCcdMotionThreshold(capsule.readFloat("ccdMotionThreshold", 0f));
        setCcdSweptSphereRadius(capsule.readFloat("ccdSweptSphereRadius", 0f));

        setPhysicsLocation((Vector3f) capsule.readSavable("physicsLocation",
                new Vector3f()));
        setPhysicsRotation((Matrix3f) capsule.readSavable("physicsRotation",
                new Matrix3f()));

        joints = capsule.readSavableArrayList("joints", null);
    }

    /**
     * Serialize this body, for example when saving to a J3O file.
     *
     * @param ex exporter (not null)
     * @throws IOException from exporter
     */
    @Override
    public void write(JmeExporter ex) throws IOException {
        super.write(ex);
        OutputCapsule capsule = ex.getCapsule(this);

        capsule.write(getMass(), "mass", 1f);

        capsule.write(getGravity(null), "gravity", Vector3f.ZERO);
        capsule.write(getFriction(), "friction", 0.5f);
        capsule.write(getRestitution(), "restitution", 0f);
        Vector3f angularFactor = getAngularFactor(null);
        if (angularFactor.x == angularFactor.y
                && angularFactor.y == angularFactor.z) {
            capsule.write(getAngularFactor(), "angularFactor", 1f);
        } else {
            capsule.write(getAngularFactor(null), "angularFactor",
                    Vector3f.UNIT_XYZ);
            capsule.write(getLinearFactor(null), "linearFactor",
                    Vector3f.UNIT_XYZ);
        }
        capsule.write(kinematic, "kinematic", false);

        capsule.write(getLinearDamping(), "linearDamping", 0f);
        capsule.write(getAngularDamping(), "angularDamping", 0f);
        capsule.write(getLinearSleepingThreshold(), "linearSleepingThreshold",
                0.8f);
        capsule.write(getAngularSleepingThreshold(), "angularSleepingThreshold",
                1f);

        capsule.write(getCcdMotionThreshold(), "ccdMotionThreshold", 0f);
        capsule.write(getCcdSweptSphereRadius(), "ccdSweptSphereRadius", 0f);

        capsule.write(getPhysicsLocation(new Vector3f()), "physicsLocation",
                new Vector3f());
        capsule.write(getPhysicsRotationMatrix(new Matrix3f()),
                "physicsRotation", new Matrix3f());

        capsule.writeSavableArrayList(joints, "joints", null);
    }
    // *************************************************************************
    // private methods

    native private void applyCentralForce(long objectId, Vector3f force);

    native private void applyForce(long objectId, Vector3f force,
            Vector3f location);

    native private void applyImpulse(long objectId, Vector3f impulse,
            Vector3f rel_pos);

    native private void applyTorque(long objectId, Vector3f vec);

    native private void applyTorqueImpulse(long objectId, Vector3f vec);

    native private void clearForces(long objectId);

    native private long createRigidBody(float mass, long motionStateId,
            long collisionShapeId);

    native private float getAngularDamping(long objectId);

    native private void getAngularFactor(long objectId, Vector3f storeResult);

    native private float getAngularSleepingThreshold(long objectId);

    native private void getAngularVelocity(long objectId, Vector3f storeResult);

    native private float getCcdMotionThreshold(long objectId);

    native private float getCcdSquareMotionThreshold(long objectId);

    native private float getCcdSweptSphereRadius(long objectId);

    native private float getFriction(long objectId);

    native private void getGravity(long objectId, Vector3f storeResult);

    native private void getInverseInertiaLocal(long objectId,
            Vector3f storeResult);

    native private float getLinearDamping(long objectId);

    native private void getLinearFactor(long objectId, Vector3f storeResult);

    native private float getLinearSleepingThreshold(long objectId);

    native private void getLinearVelocity(long objectId, Vector3f storeResult);

    native private void getPhysicsLocation(long objectId, Vector3f storeResult);

    native private void getPhysicsRotation(long objectId,
            Quaternion storeResult);

    native private void getPhysicsRotationMatrix(long objectId,
            Matrix3f storeResult);

    native private float getRestitution(long objectId);

    native private boolean isActive(long objectId);

    native private boolean isInWorld(long objectId);

    native private void setAngularDamping(long objectId, float factor);

    native private void setAngularFactor(long objectId, Vector3f factor);

    native private void setAngularVelocity(long objectId, Vector3f vec);

    native private void setCcdMotionThreshold(long objectId, float threshold);

    native private void setCcdSweptSphereRadius(long objectId, float radius);

    native private void setCollisionShape(long objectId, long collisionShapeId);

    native private void setDamping(long objectId, float linearDamping,
            float angularDamping);

    native private void setFriction(long objectId, float friction);

    native private void setGravity(long objectId, Vector3f gravity);

    native private void setInverseInertiaLocal(long objectId,
            Vector3f inverseInertialLocal);

    native private void setKinematic(long objectId, boolean kinematic);

    native private void setLinearFactor(long objectId, Vector3f factor);

    native private void setLinearVelocity(long objectId, Vector3f vec);

    native private void setPhysicsLocation(long objectId, Vector3f location);

    native private void setPhysicsRotation(long objectId, Matrix3f rotation);

    native private void setPhysicsRotation(long objectId, Quaternion rotation);

    native private void setRestitution(long objectId, float factor);

    native private void setSleepingThresholds(long objectId, float linear,
            float angular);

    native private void setStatic(long objectId, boolean state);

    native private long updateMassProps(long objectId, long collisionShapeId,
            float mass);

    /**
     * Validate a shape suitable for a dynamic body.
     *
     * @param shape (not null, unaffected)
     */
    private static void validateDynamicShape(CollisionShape shape) {
        assert shape != null;

        if (shape instanceof HeightfieldCollisionShape) {
            throw new IllegalStateException(
                    "Dynamic rigid body can't have heightfield shape!");
        } else if (shape instanceof MeshCollisionShape) {
            throw new IllegalStateException(
                    "Dynamic rigid body can't have mesh shape!");
        } else if (shape instanceof PlaneCollisionShape) {
            throw new IllegalStateException(
                    "Dynamic rigid body can't have plane shape!");
        }
    }
}
