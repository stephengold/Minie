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
package com.jme3.bullet.control;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.PhysicsRayTestResult;
import com.jme3.bullet.collision.PhysicsSweepTestResult;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Spatial;
import com.jme3.util.TempVars;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * This class is intended to replace the CharacterControl class.
 * <p>
 * A rigid body with an offset CapsuleCollisionShape is used. Its linear
 * velocity is updated continuously. A ray test is used to determine whether the
 * character is on the ground.
 * <p>
 * The character maintains a local coordinate system in which gravity determines
 * the Y axis.
 * <p>
 * Motion in the local X-Z plane is damped.
 *
 * @author normenhansen
 */
public class BetterCharacterControl
        extends AbstractPhysicsControl
        implements PhysicsTickListener {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(BetterCharacterControl.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagBody = "body";
    final private static String tagDuckedFactor = "duckedFactor";
    final private static String tagHeight = "height";
    final private static String tagJumpForce = "jumpForce";
    final private static String tagMass = "mass";
    final private static String tagPhysicsDamping = "physicsDamping";
    final private static String tagRadius = "radius";
    final private static String tagViewDirection = "viewDirection";
    final private static String tagWalkDirection = "walkDirection";
    // *************************************************************************
    // fields

    /**
     * true when the character is ducked (its height scaled by duckedFactor)
     */
    private boolean ducked = false;
    /**
     * true when a jump has been requested for the next simulation step
     */
    private boolean jump = false;
    /**
     * true when a collision object is directly below the character
     */
    private boolean onGround = false;
    /**
     * true when un-ducking is requested
     */
    private boolean wantToUnDuck = false;
    /**
     * relative height of the collision shape when ducked (as a fraction of its
     * initial height, &gt;0, &le;1)
     */
    private float duckedFactor = 0.6f;
    /**
     * initial height of the collision shape (in physics-space units, includes
     * both hemispheres and the cylindrical part)
     */
    private float height;
    /**
     * mass of the rigid body (&gt;0)
     */
    private float mass;
    /**
     * multiplier for horizontal motion, applied during each simulation step
     */
    private float physicsDamping = 0.9f;
    /**
     * initial radius of the collision shape (in physics-space units, &gt;0)
     */
    private float radius;
    /**
     * underlying rigid body
     */
    private PhysicsRigidBody rigidBody;
    /**
     * orientation of the character's body (in physics-space coordinates)
     */
    private Quaternion localForwardRotation = new Quaternion();
    /**
     * orientation of the character's viewpoint (in physics-space coordinates)
     */
    private Quaternion rotation = new Quaternion();
    /**
     * cached collision shape for sweep casts
     */
    private SphereCollisionShape sweepShape;
    /**
     * temporary starting transform for ray/sweep casts
     */
    private Transform castBegin = new Transform();
    /**
     * temporary ending transform for ray/sweep casts
     */
    private Transform castEnd = new Transform();
    /**
     * impulse applied at the start of each jump (in local coordinates)
     */
    private Vector3f jumpForce = new Vector3f();
    /**
     * local +Z direction (unit vector in physics-space coordinates)
     */
    private Vector3f localForward = new Vector3f(0f, 0f, 1f);
    /**
     * local +X direction (unit vector in physics-space coordinates)
     */
    private Vector3f localLeft = new Vector3f(1f, 0f, 0f);
    /**
     * local +Y direction (unit vector in physics-space coordinates)
     */
    private Vector3f localUp = new Vector3f(0f, 1f, 0f);
    /**
     * rigid-body base location (in physics-space coordinates)
     */
    private Vector3f location = new Vector3f();
    /**
     * view direction (in physics-space coordinates)
     */
    private Vector3f rotatedViewDirection = new Vector3f(0f, 0f, 1f);
    /**
     * scale factors applied when generating the collision shape (the X
     * component is ignored)
     */
    private Vector3f scale = new Vector3f(1f, 1f, 1f);
    /**
     * linear velocity of the rigid body (in physics-space coordinates)
     */
    private Vector3f velocity = new Vector3f();
    /**
     * view direction (in local coordinates)
     */
    private Vector3f viewDirection = new Vector3f(0f, 0f, 1f);
    /**
     * requested walk velocity (in physics-space coordinates)
     */
    private Vector3f walkDirection = new Vector3f();
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected BetterCharacterControl() {
    }

    /**
     * Instantiate an enabled Control with the specified properties.
     * <p>
     * The height must exceed 2x the radius, even when ducked.
     *
     * @param radius the initial radius for the collision shape (in
     * physics-space coordinates, &gt;0)
     * @param height the initial height for the character's CollisionShape (in
     * physics-space coordinates, &gt;2*radius)
     * @param mass the character's mass (&ge;0)
     */
    public BetterCharacterControl(float radius, float height, float mass) {
        Validate.positive(radius, "radius");
        Validate.require(
                height > 2f * radius, "height more than 2x the radius");
        Validate.positive(mass, "mass");

        this.radius = radius;
        this.height = height;
        this.mass = mass;

        CollisionShape shape = getShape();
        this.rigidBody = new PhysicsRigidBody(shape, mass);

        float upwardImpulse = 5f * mass;
        this.jumpForce = new Vector3f(0f, upwardImpulse, 0f);

        this.rigidBody.setAngularFactor(0f);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Return the collision-shape height multiplier for ducking.
     *
     * @return the factor by which the initial height will be multiplied (&gt;0,
     * &le;1)
     */
    public float getDuckedFactor() {
        return duckedFactor;
    }

    /**
     * Copy the character's gravity vector.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return an acceleration vector in physics-space coordinates (either
     * {@code storeResult} or a new vector, not null)
     */
    public Vector3f getGravity(Vector3f storeResult) {
        Vector3f result = rigidBody.getGravity(storeResult);
        return result;
    }

    /**
     * Copy the impulse applied at the start of each jump.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return an impulse vector in local coordinates (either
     * {@code storeResult} or a new vector, not null)
     */
    public Vector3f getJumpForce(Vector3f storeResult) {
        if (storeResult == null) {
            return jumpForce.clone();
        } else {
            return storeResult.set(jumpForce);
        }
    }

    /**
     * Return the multiplier for horizontal motion.
     *
     * @return the multiplier for motion in the local X-Z plane (applied during
     * each simulation step, 0&rarr;no damping, 1=horizontal forces have no
     * effect, &ge;0, &le;1)
     */
    public float getPhysicsDamping() {
        return physicsDamping;
    }

    /**
     * Access the rigid body managed by this control.
     *
     * @return the pre-existing rigid body (not null)
     */
    public PhysicsRigidBody getRigidBody() {
        assert rigidBody != null;
        return rigidBody;
    }

    /**
     * For compatibility with the jme3-jbullet library. The jme3-jbullet version
     * returns a pre-existing vector instead of a new one.
     *
     * @return a new velocity vector
     */
    public Vector3f getVelocity() {
        Vector3f result = getVelocity(null);
        return result;
    }

    /**
     * Copy the character's linear velocity.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a velocity vector (either {@code storeResult} or a new vector,
     * not null)
     */
    public Vector3f getVelocity(Vector3f storeResult) {
        Vector3f result;
        if (storeResult == null) {
            result = velocity.clone();
        } else {
            result = storeResult.set(velocity);
        }

        return result;
    }

    /**
     * For compatibility with the jme3-jbullet library. The jme3-jbullet version
     * returns a pre-existing vector instead of a new one.
     *
     * @return a new direction vector (in physics-space coordinates)
     */
    public Vector3f getViewDirection() {
        return getViewDirection(null);
    }

    /**
     * Copy the character's view direction. This need not agree with the
     * spatial's forward direction.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a direction vector (in local coordinates, either
     * {@code storeResult} or a new vector, not null)
     */
    public Vector3f getViewDirection(Vector3f storeResult) {
        Vector3f result;
        if (storeResult == null) {
            result = viewDirection.clone();
        } else {
            result = storeResult.set(viewDirection);
        }

        return result;
    }

    /**
     * Copy the character's walk velocity. The length of the vector defines the
     * requested speed.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a velocity vector (in physics-space coordinates, either
     * {@code storeResult} or a new vector, not null)
     */
    public Vector3f getWalkDirection(Vector3f storeResult) {
        Vector3f result;
        if (storeResult == null) {
            result = walkDirection.clone();
        } else {
            result = storeResult.set(walkDirection);
        }

        return result;
    }

    /**
     * Determine whether the character is in a ducked state, either due to user
     * input or because an overhead obstacle prevents it from unducking.
     *
     * @return true if ducking, otherwise false
     */
    public boolean isDucked() {
        return ducked;
    }

    /**
     * Test whether the character is supported. Uses a ray test from the center
     * of the character and might return false even if the character is not
     * falling yet.
     *
     * @return true if supported, otherwise false
     */
    public boolean isOnGround() {
        return onGround;
    }

    /**
     * Apply a jump impulse during the next simulation step if the character is
     * on ground.
     */
    public void jump() {
        this.jump = true;
    }

    /**
     * Alter the character's forward (+Z) direction.
     *
     * @param vec the desired direction (in physics-space coordinates) or null
     * for (0,0,1)
     */
    public void resetForward(Vector3f vec) {
        if (vec == null) {
            localForward.set(0f, 0f, 1f);
        } else {
            localForward.set(vec);
        }
        updateLocalCoordinateSystem();
    }

    /**
     * Alter the character's ducking state. When ducked, the character's
     * collision-shape height is scaled by duckedFactor to make it shorter.
     * Before unducking, the character performs a ray test; it grows taller only
     * if there's room above its head. You can test the state using
     * {@link #isDucked()}.
     *
     * @param enabled true&rarr;duck, false&rarr;unduck
     */
    public void setDucked(boolean enabled) {
        if (enabled) {
            setHeightPercent(duckedFactor);
            this.ducked = true;
            this.wantToUnDuck = false;
        } else {
            if (checkCanUnDuck()) {
                setHeightPercent(1f);
                this.ducked = false;
            } else {
                this.wantToUnDuck = true;
            }
        }
    }

    /**
     * Alter the collision-shape height multiplier for ducking.
     *
     * @param factor the factor by which the initial height will be multiplied
     * (&gt;0, &le;1, default=0.6)
     */
    public void setDuckedFactor(float factor) {
        Validate.fraction(factor, "factor");
        this.duckedFactor = factor;
    }

    /**
     * Alter the gravity acting on this character. Note that this also realigns
     * the local coordinate system of the character so that continuous changes
     * in gravity direction are possible while maintaining a sensible control
     * over the character.
     *
     * @param gravity the desired acceleration vector (in physics-space
     * coordinates, not null, finite, unaffected)
     */
    public void setGravity(Vector3f gravity) {
        Validate.finite(gravity, "gravity");

        rigidBody.setGravity(gravity);
        localUp.set(gravity).normalizeLocal().negateLocal();
        updateLocalCoordinateSystem();
    }

    /**
     * Alter the impulse applied at the start of each jump.
     *
     * @param jumpForce the desired impulse (in local coordinates, not null,
     * finite, unaffected, default=5*mass in +Y direction)
     */
    public void setJumpForce(Vector3f jumpForce) {
        Validate.finite(jumpForce, "jump force");

        this.jumpForce.set(jumpForce);
    }

    /**
     * Alter the damping multiplier for horizontal motion.
     *
     * @param physicsDamping the desired multiplier for motion in the local X-Z
     * plane (applied during each simulation step, 0&rarr;no damping,
     * 1=horizontal forces have no effect, &ge;0, &le;1, default=0.9)
     */
    public void setPhysicsDamping(float physicsDamping) {
        Validate.fraction(physicsDamping, "physics damping");
        this.physicsDamping = physicsDamping;
    }

    /**
     * Alter the character's view direction. Note this doesn't affect the
     * orientation of its body.
     *
     * @param vec a direction vector in local coordinates (not null, not zero,
     * unaffected)
     */
    public void setViewDirection(Vector3f vec) {
        assert Validate.nonZero(vec, "vec");

        viewDirection.set(vec);
        updateLocalViewDirection();
    }

    /**
     * Alter the character's walk velocity. The length of the vector determines
     * the requested speed, in physics-space units per second.
     *
     * @param vec the requested velocity vector (in physics-space coordinates,
     * not null, unaffected)
     */
    public void setWalkDirection(Vector3f vec) {
        walkDirection.set(vec);
    }

    /**
     * Translate the character to the specified location.
     *
     * @param vec the desired location (in physics-space coordinates, not null,
     * finite, unaffected)
     */
    public void warp(Vector3f vec) {
        Validate.finite(vec, "vec");
        setPhysicsLocation(vec);
    }
    // *************************************************************************
    // AbstractPhysicsControl methods

    /**
     * Add all managed physics objects to the PhysicsSpace.
     */
    @Override
    protected void addPhysics() {
        PhysicsSpace space = getPhysicsSpace();

        space.getGravity(localUp);
        localUp.normalizeLocal();
        localUp.negateLocal();
        updateLocalCoordinateSystem();

        space.addCollisionObject(rigidBody);
        space.addTickListener(this);
    }

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned Control into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this Control (not null, modified)
     * @param original the instance from which this Control was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);

        this.sweepShape = null;
        this.castEnd = cloner.clone(castEnd);
        this.castBegin = cloner.clone(castBegin);
        this.jumpForce = cloner.clone(jumpForce);
        this.localForward = cloner.clone(localForward);
        this.localForwardRotation = cloner.clone(localForwardRotation);
        this.localLeft = cloner.clone(localLeft);
        this.localUp = cloner.clone(localUp);
        this.location = cloner.clone(location);
        this.rigidBody = cloner.clone(rigidBody);
        this.rotatedViewDirection = cloner.clone(rotatedViewDirection);
        this.rotation = cloner.clone(rotation);
        this.scale = cloner.clone(scale);
        this.velocity = cloner.clone(velocity);
        this.viewDirection = cloner.clone(viewDirection);
        this.walkDirection = cloner.clone(walkDirection);
    }

    /**
     * Create spatial-dependent data. Invoked when this Control is added to a
     * Spatial.
     *
     * @param spatial the controlled Spatial (not null, alias created)
     */
    @Override
    protected void createSpatialData(Spatial spatial) {
        rigidBody.setUserObject(spatial);
    }

    /**
     * De-serialize this Control from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);
        InputCapsule capsule = importer.getCapsule(this);

        this.radius = capsule.readFloat(tagRadius, 1f);
        this.height = capsule.readFloat(tagHeight, 2f);
        this.mass = capsule.readFloat(tagMass, 80f);
        this.jumpForce = (Vector3f) capsule
                .readSavable(tagJumpForce, new Vector3f(0f, mass * 5f, 0f));
        this.physicsDamping = capsule.readFloat(tagPhysicsDamping, 0.9f);
        this.duckedFactor = capsule.readFloat(tagDuckedFactor, 0.6f);
        this.viewDirection = (Vector3f) capsule
                .readSavable(tagViewDirection, new Vector3f(0f, 0f, 1f));
        this.walkDirection = (Vector3f) capsule
                .readSavable(tagWalkDirection, new Vector3f(0f, 0f, 1f));
        this.rigidBody = (PhysicsRigidBody) capsule.readSavable(tagBody, null);

        Spatial controlled = getSpatial();
        rigidBody.setUserObject(controlled);
        // sweepShape, castBegin, and castEnd are not read
    }

    /**
     * Remove all managed physics objects from the PhysicsSpace.
     */
    @Override
    protected void removePhysics() {
        PhysicsSpace space = getPhysicsSpace();
        space.removeCollisionObject(rigidBody);
        space.removeTickListener(this);
    }

    /**
     * Destroy spatial-dependent data. Invoked when this Control is removed from
     * its Spatial.
     *
     * @param spatial the Spatial to which this Control was added (unused)
     */
    @Override
    protected void removeSpatialData(Spatial spatial) {
        rigidBody.setUserObject(null);
    }

    /**
     * Translate the character to the specified location.
     *
     * @param location the desired location (in physics-space coordinates, not
     * null, finite, unaffected)
     */
    @Override
    protected void setPhysicsLocation(Vector3f location) {
        Validate.finite(location, "location");

        rigidBody.setPhysicsLocation(location);
        this.location.set(location);
    }

    /**
     * Rotate the character's viewpoint to the specified orientation.
     * <p>
     * We don't set the body orientation here, but the view rotation, which
     * might be changed by the calculateNewForward() method.
     *
     * @param orientation the desired orientation (in physics-space coordinates,
     * not null, not zero, unaffected)
     */
    @Override
    protected void setPhysicsRotation(Quaternion orientation) {
        Validate.nonZero(orientation, "orientation");

        rotation.set(orientation);
        rotatedViewDirection.set(viewDirection);
        rotation.multLocal(rotatedViewDirection);
        updateLocalViewDirection();
    }

    /**
     * Update this Control. Invoked once per frame during the logical-state
     * update, provided the Control is added to a scene. Do not invoke directly
     * from user code.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void update(float tpf) {
        if (!isEnabled()) {
            return;
        }

        rigidBody.getPhysicsLocation(location);
        // rotation has been set through viewDirection
        applyPhysicsTransform(location, rotation);
    }

    /**
     * Serialize this Control to the specified exporter, for example when saving
     * to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        super.write(exporter);
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(radius, tagRadius, 1f);
        capsule.write(height, tagHeight, 2f);
        capsule.write(mass, tagMass, 80f);
        capsule.write(jumpForce, tagJumpForce, null);
        capsule.write(physicsDamping, tagPhysicsDamping, 0.9f);
        capsule.write(duckedFactor, tagDuckedFactor, 0.6f);
        capsule.write(viewDirection, tagViewDirection, null);
        capsule.write(walkDirection, tagWalkDirection, null);
        capsule.write(rigidBody, tagBody, null);
        // sweepShape, castBegin, and castEnd are not written
    }
    // *************************************************************************
    // PhysicsTickListener methods

    /**
     * Callback from Bullet, invoked just after the physics has been stepped.
     *
     * @param space the space that was just stepped (not null)
     * @param timeStep the time per simulation step (in seconds, &ge;0)
     */
    @Override
    public void physicsTick(PhysicsSpace space, float timeStep) {
        rigidBody.getLinearVelocity(velocity);
    }

    /**
     * Callback from Bullet, invoked just before the physics is stepped.
     *
     * @param space the space that's about to be stepped (not null)
     * @param timeStep the time per simulation step (in seconds, &ge;0)
     */
    @Override
    public void prePhysicsTick(PhysicsSpace space, float timeStep) {
        checkOnGround();
        if (wantToUnDuck && checkCanUnDuck()) {
            setHeightPercent(1f);
            this.wantToUnDuck = false;
            this.ducked = false;
        }
        TempVars vars = TempVars.get();

        Vector3f currentVelocity = vars.vect2.set(velocity);

        // Attenuate any horizontal motion with a counteracting delta V.
        float existingLeftVelocity = velocity.dot(localLeft);
        float existingForwardVelocity = velocity.dot(localForward);
        existingLeftVelocity *= physicsDamping;
        existingForwardVelocity *= physicsDamping;
        Vector3f counter = vars.vect1; // alias
        counter.set(-existingLeftVelocity, 0, -existingForwardVelocity);
        localForwardRotation.multLocal(counter);
        velocity.addLocal(counter);

        float requestedSpeed = walkDirection.length();
        if (requestedSpeed > 0f) {
            Vector3f localWalkDirection = vars.vect1; // alias
            localWalkDirection.set(walkDirection);
            localWalkDirection.normalizeLocal();

            // Calculate current velocity component in the desired direction.
            float speed = velocity.dot(localWalkDirection);

            // Adjust the linear velocity.
            float additionalSpeed = requestedSpeed - speed;
            localWalkDirection.multLocal(additionalSpeed);
            velocity.addLocal(localWalkDirection);
        }
        if (currentVelocity.distance(velocity) > FastMath.ZERO_TOLERANCE) {
            rigidBody.setLinearVelocity(velocity);
        }

        if (jump && onGround) {
            Vector3f impulseInWorld = vars.vect1; // alias
            impulseInWorld.set(jumpForce);
            localForwardRotation.multLocal(impulseInWorld);
            rigidBody.applyCentralImpulse(impulseInWorld);
        }
        this.jump = false;

        vars.release();
    }
    // *************************************************************************
    // new protected methods

    /**
     * Adjust the specified direction vector so it is perpendicular to the
     * specified local "up" direction.
     *
     * @param rotation storage for an orientation in which {@code worldUpVector}
     * is +Y and {@code direction} is +Z (modified if not null)
     * @param direction the direction in physics-space coordinates (modified if
     * not null)
     * @param worldUpVector the local "up" direction in physics-space
     * coordinates (not null, not zero, unaffected)
     */
    protected void calculateNewForward(Quaternion rotation,
            Vector3f direction, Vector3f worldUpVector) {
        if (direction == null) {
            return;
        }
        TempVars vars = TempVars.get();
        Vector3f newLeft = vars.vect1; // alias
        Vector3f newLeftNegate = vars.vect2; // alias

        newLeft.set(worldUpVector);
        newLeft.crossLocal(direction);
        newLeft.normalizeLocal();
        if (MyVector3f.isZero(newLeft)) {
            if (direction.x != 0) {
                newLeft.set(direction.y, -direction.x, 0f);
                newLeft.normalizeLocal();
            } else {
                newLeft.set(0f, direction.z, -direction.y);
                newLeft.normalizeLocal();
            }
            if (logger2.isLoggable(Level.INFO)) {
                logger2.log(Level.INFO, "Zero left for direction {0}, up {1}",
                        new Object[]{direction, worldUpVector});
            }
        }
        newLeftNegate.set(newLeft);
        newLeftNegate.negateLocal();
        direction.set(worldUpVector);
        direction.crossLocal(newLeftNegate);
        direction.normalizeLocal();
        if (MyVector3f.isZero(direction)) {
            direction.set(0f, 0f, 1f);
            if (logger2.isLoggable(Level.INFO)) {
                logger2.log(Level.INFO, "Zero left for left {0}, up {1}",
                        new Object[]{newLeft, worldUpVector});
            }
        }
        if (rotation != null) {
            rotation.fromAxes(newLeft, worldUpVector, direction);
        }
        vars.release();
    }

    /**
     * Determine whether the character can emerge from the ducked state at its
     * current location.
     *
     * @return true if able to unduck, otherwise false
     */
    protected boolean checkCanUnDuck() {
        /*
         * Cast a sphere upward, from the current location
         * of the upper hemisphere to its desired location.
         */
        Vector3f startLocation = castBegin.getTranslation(); // alias
        startLocation.set(location);
        float currentHeight = getFinalHeight();
        float bodyRadius = getFinalRadius();
        MyVector3f.accumulateScaled(
                startLocation, localUp, currentHeight - bodyRadius);

        Vector3f endLocation = castEnd.getTranslation(); // alias
        endLocation.set(location);
        MyVector3f.accumulateScaled(endLocation, localUp, height - bodyRadius);

        if (sweepShape == null || sweepShape.getRadius() != bodyRadius) {
            this.sweepShape = new SphereCollisionShape(bodyRadius);
        }
        PhysicsSpace space = getPhysicsSpace();
        List<PhysicsSweepTestResult> results
                = space.sweepTest(sweepShape, castBegin, castEnd);

        // Search for a collision object other than the character's body.
        boolean isObstructed = false;
        for (PhysicsSweepTestResult result : results) {
            PhysicsCollisionObject object = result.getCollisionObject();
            if (!object.equals(rigidBody)) {
                isObstructed = true;
                break;
            }
        }

        return !isObstructed;
    }

    /**
     * Update the internal {@code onGround} status.
     */
    protected void checkOnGround() {
        /*
         * Cast a ray downward, from the peak of the rigid body
         * to 0.1 physics-space unit below its base.
         */
        Vector3f startLocation = castBegin.getTranslation(); // alias
        startLocation.set(location);
        float scaledHeight = getFinalHeight();
        MyVector3f.accumulateScaled(startLocation, localUp, scaledHeight);

        Vector3f endLocation = castEnd.getTranslation(); // alias
        endLocation.set(location);
        MyVector3f.accumulateScaled(endLocation, localUp, -0.1f);

        PhysicsSpace space = getPhysicsSpace();
        List<PhysicsRayTestResult> results
                = space.rayTestRaw(startLocation, endLocation);

        // Search for a collision object other than the character's body.
        boolean isRayObstructed = false;
        for (PhysicsRayTestResult result : results) {
            PhysicsCollisionObject object = result.getCollisionObject();
            if (!object.equals(rigidBody)) {
                isRayObstructed = true;
                break;
            }
        }

        // Update the status.
        this.onGround = isRayObstructed;
    }

    /**
     * Return the scaled height of the collision shape, including both
     * hemispheres and the cylindrical part.
     *
     * @return the height (in physics-space units, &gt;0)
     */
    protected float getFinalHeight() {
        float result = height * scale.y;

        assert result > 0f : result;
        return result;
    }

    /**
     * Return the scaled radius of the collision shape.
     *
     * @return the radius (in physics-space units, &gt;0)
     */
    protected float getFinalRadius() {
        float result = radius * scale.z;

        assert result > 0f : result;
        return result;
    }

    /**
     * Create a CollisionShape based on the {@code scale} parameter.
     *
     * @return a new compound shape containing a capsule (not null)
     */
    protected CollisionShape getShape() {
        float scaledRadius = getFinalRadius();
        float totalHeight = getFinalHeight();
        float cylinderHeight = totalHeight - 2f * scaledRadius;
        CapsuleCollisionShape capsule
                = new CapsuleCollisionShape(scaledRadius, cylinderHeight);

        CompoundCollisionShape result = new CompoundCollisionShape(1);
        float yOffset = totalHeight / 2f;
        result.addChildShape(capsule, 0f, yOffset, 0f);

        return result;
    }

    /**
     * Alter the height of the collision shape.
     *
     * @param fraction the desired height, as a fraction of the initial height
     * (default=1)
     */
    protected void setHeightPercent(float fraction) {
        scale.setY(fraction);
        CollisionShape newShape = getShape();
        rigidBody.setCollisionShape(newShape);
    }

    /**
     * Update the local coordinate system from the localForward and localUp
     * vectors, adapts localForward, sets localForwardRotation to local
     * Z-forward rotation.
     */
    protected void updateLocalCoordinateSystem() {
        /*
         * The gravity vector may have changed,
         * so update localForwardRotation and localForward.
         */
        calculateNewForward(localForwardRotation, localForward, localUp);
        localLeft.set(localUp);
        localLeft.crossLocal(localForward);
        rigidBody.setPhysicsRotation(localForwardRotation);
        updateLocalViewDirection();
    }

    /**
     * Update the character's viewpoint.
     */
    protected void updateLocalViewDirection() {
        rotatedViewDirection.set(viewDirection);
        localForwardRotation.multLocal(rotatedViewDirection);
        /*
         * The gravity vector may have changed,
         * so update rotation and rotatedViewDirection.
         */
        calculateNewForward(rotation, rotatedViewDirection, localUp);
    }
}
