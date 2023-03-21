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
 * velocity is updated continuously. A sweep test is used to determine whether
 * the character is on the ground.
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
    private boolean isDucked = false;
    /**
     * true when a collision object is directly below the character
     */
    private boolean onGround = false;
    /**
     * true when a jump has been requested for the next simulation step
     */
    private boolean wantToJump = false;
    /**
     * true when un-ducking has been requested for the next simulation step
     */
    private boolean wantToUnDuck = false;
    /**
     * damping factor for horizontal motion, applied before each simulation step
     */
    private float dampingFactor = 0.9f;
    /**
     * relative height of the collision shape when ducked (as a fraction of its
     * initial height, &gt;0, &le;1)
     */
    private float duckedFactor = 0.6f;
    /**
     * initial height of the collision shape (in physics-space units, includes
     * both hemispheres and the cylindrical part)
     */
    private float initialHeight;
    /**
     * initial radius of the collision shape (in physics-space units, &gt;0)
     */
    private float initialRadius;
    /**
     * mass of the rigid body (&gt;0)
     */
    private float mass;
    /**
     * underlying rigid body
     */
    private PhysicsRigidBody rigidBody;
    /**
     * orientation of the character's body (in physics-space coordinates)
     */
    private Quaternion localToWorld = new Quaternion();
    /**
     * orientation of the character's viewpoint (in physics-space coordinates)
     */
    private Quaternion viewToWorld = new Quaternion();
    /**
     * cached collision shape for sweep tests
     */
    private SphereCollisionShape sweepShape;
    /**
     * temporary starting transform for sweep tests
     */
    private Transform sweepBegin = new Transform();
    /**
     * temporary ending transform for sweep tests
     */
    private Transform sweepEnd = new Transform();
    /**
     * rigid-body base location (in physics-space coordinates)
     */
    private Vector3f baseLocation = new Vector3f();
    /**
     * impulse applied at the start of each jump (in local coordinates)
     */
    private Vector3f jumpImpulse = new Vector3f();
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
     * view direction (in physics-space coordinates)
     */
    private Vector3f viewDirInWorld = new Vector3f(0f, 0f, 1f);
    /**
     * requested walk velocity (in physics-space coordinates)
     */
    private Vector3f walkVelocity = new Vector3f();
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
     * physics-space units, &gt;0)
     * @param height the initial height for the character's CollisionShape (in
     * physics-space units, &gt;2*radius)
     * @param mass the character's mass (&ge;0)
     */
    public BetterCharacterControl(float radius, float height, float mass) {
        Validate.positive(radius, "radius");
        Validate.require(
                height > 2f * radius, "height more than 2x the radius");
        Validate.positive(mass, "mass");

        this.initialRadius = radius;
        this.initialHeight = height;
        this.mass = mass;

        CollisionShape shape = getShape();
        this.rigidBody = new PhysicsRigidBody(shape, mass);

        float upwardImpulse = 5f * mass;
        this.jumpImpulse = new Vector3f(0f, upwardImpulse, 0f);

        rigidBody.setAngularFactor(0f);
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
            return jumpImpulse.clone();
        } else {
            return storeResult.set(jumpImpulse);
        }
    }

    /**
     * Return the damping factor for horizontal motion.
     *
     * @return the damping factor for motion in the local X-Z plane (applied
     * before each simulation step, 0&rarr;no damping, 1=horizontal forces have
     * no effect, &ge;0, &le;1)
     */
    public float getPhysicsDamping() {
        return dampingFactor;
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
     * Copy the character's view direction.
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
            result = walkVelocity.clone();
        } else {
            result = storeResult.set(walkVelocity);
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
        return isDucked;
    }

    /**
     * Test whether the character is in kinematic mode.
     *
     * @return true if in kinematic mode, otherwise false (in dynamic mode)
     */
    public boolean isKinematic() {
        boolean result = !rigidBody.isDynamic();
        return result;
    }

    /**
     * Test whether the character is supported by another collision object.
     *
     * @return true if supported, otherwise false
     */
    public boolean isOnGround() {
        return onGround;
    }

    /**
     * Apply a jump impulse during the next simulation step if the character is
     * on ground and not in kinematic mode.
     */
    public void jump() {
        this.wantToJump = true;
    }

    /**
     * Alter the character's forward (+Z) direction, provided it's in dynamic
     * mode.
     *
     * @param vec the desired direction (in physics-space coordinates) or null
     * for (0,0,1)
     */
    public void resetForward(Vector3f vec) {
        if (!rigidBody.isDynamic()) {
            return;
        }

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
     * Before unducking, the character performs a sweep test; it grows taller
     * only if there's room above its head. You can test the state using
     * {@link #isDucked()}.
     *
     * @param newState true&rarr;duck, false&rarr;unduck
     */
    public void setDucked(boolean newState) {
        if (newState) {
            setHeightPercent(duckedFactor);
            this.isDucked = true;
            this.wantToUnDuck = false;
        } else {
            this.wantToUnDuck = true;
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
     * @param newGravity the desired acceleration vector (in physics-space
     * coordinates, not null, finite, unaffected)
     */
    public void setGravity(Vector3f newGravity) {
        Validate.finite(newGravity, "new gravity");

        rigidBody.setGravity(newGravity);
        localUp.set(newGravity).normalizeLocal().negateLocal();
        updateLocalCoordinateSystem();
    }

    /**
     * Alter the impulse applied at the start of each jump.
     *
     * @param newImpulse the desired impulse (in local coordinates, not null,
     * finite, unaffected, default=5*mass in +Y direction)
     */
    public void setJumpForce(Vector3f newImpulse) {
        Validate.finite(newImpulse, "new impulse");
        jumpImpulse.set(newImpulse);
    }

    /**
     * Transition the character from kinematic mode to dynamic mode or vice
     * versa.
     *
     * @param newSetting true&rarr;set kinematic mode, false&rarr;set dynamic
     * mode (default=false)
     */
    public void setKinematic(boolean newSetting) {
        rigidBody.setKinematic(newSetting);
    }

    /**
     * Alter the damping factor for horizontal motion.
     *
     * @param newFactor the desired damping factor for motion in the local X-Z
     * plane (applied before each simulation step, 0&rarr;no damping,
     * 1=horizontal forces have no effect, &ge;0, &le;1, default=0.9)
     */
    public void setPhysicsDamping(float newFactor) {
        Validate.fraction(newFactor, "new factor");
        this.dampingFactor = newFactor;
    }

    /**
     * Alter the character's view direction, provided it's in dynamic mode. View
     * direction is used to orient the rigid body.
     *
     * @param newDirection a direction vector in local coordinates (not null,
     * not zero, unaffected)
     */
    public void setViewDirection(Vector3f newDirection) {
        assert Validate.nonZero(newDirection, "new direction");

        if (rigidBody.isDynamic()) {
            viewDirection.set(newDirection);
            updateLocalViewDirection();
        }
    }

    /**
     * Alter the character's walk velocity. The length of the vector determines
     * the requested speed, in physics-space units per second.
     *
     * @param newVelocity the requested velocity (in physics-space coordinates,
     * not null, unaffected)
     */
    public void setWalkDirection(Vector3f newVelocity) {
        walkVelocity.set(newVelocity);
    }

    /**
     * Translate the character instantly to the specified location, provided
     * it's in dynamic mode.
     *
     * @param newLocation the desired location of the base (in physics-space
     * coordinates, not null, finite, unaffected)
     */
    public void warp(Vector3f newLocation) {
        Validate.finite(newLocation, "new location");
        if (rigidBody.isDynamic()) {
            setPhysicsLocation(newLocation);
        }
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
        this.sweepEnd = cloner.clone(sweepEnd);
        this.sweepBegin = cloner.clone(sweepBegin);
        this.jumpImpulse = cloner.clone(jumpImpulse);
        this.localForward = cloner.clone(localForward);
        this.localToWorld = cloner.clone(localToWorld);
        this.localLeft = cloner.clone(localLeft);
        this.localUp = cloner.clone(localUp);
        this.baseLocation = cloner.clone(baseLocation);
        this.rigidBody = cloner.clone(rigidBody);
        this.viewDirInWorld = cloner.clone(viewDirInWorld);
        this.viewToWorld = cloner.clone(viewToWorld);
        this.scale = cloner.clone(scale);
        this.velocity = cloner.clone(velocity);
        this.viewDirection = cloner.clone(viewDirection);
        this.walkVelocity = cloner.clone(walkVelocity);
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

        this.initialRadius = capsule.readFloat(tagRadius, 1f);
        this.initialHeight = capsule.readFloat(tagHeight, 2f);
        this.mass = capsule.readFloat(tagMass, 80f);
        this.jumpImpulse = (Vector3f) capsule
                .readSavable(tagJumpForce, new Vector3f(0f, mass * 5f, 0f));
        this.dampingFactor = capsule.readFloat(tagPhysicsDamping, 0.9f);
        this.duckedFactor = capsule.readFloat(tagDuckedFactor, 0.6f);
        this.viewDirection = (Vector3f) capsule
                .readSavable(tagViewDirection, new Vector3f(0f, 0f, 1f));
        this.walkVelocity = (Vector3f) capsule
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
     * Translate the character instantly to the specified location.
     *
     * @param newLocation the desired location of the base (in physics-space
     * coordinates, not null, finite, unaffected)
     */
    @Override
    protected void setPhysicsLocation(Vector3f newLocation) {
        Validate.finite(newLocation, "new location");

        rigidBody.setPhysicsLocation(newLocation);
        baseLocation.set(newLocation);
    }

    /**
     * Rotate the character's viewpoint to the specified orientation.
     * <p>
     * We don't set the body orientation here, but the view rotation, which
     * might be changed by the calculateNewForward() method.
     *
     * @param newOrientation the desired orientation (in physics-space
     * coordinates, not null, not zero, unaffected)
     */
    @Override
    protected void setPhysicsRotation(Quaternion newOrientation) {
        Validate.nonZero(newOrientation, "new orientation");

        viewToWorld.set(newOrientation);
        viewDirInWorld.set(viewDirection);
        viewToWorld.multLocal(viewDirInWorld);
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

        if (rigidBody.isDynamic()) {
            rigidBody.getPhysicsLocation(baseLocation);
            // viewToWorld has been set by updateLocalCoordinateSystem()
            applyPhysicsTransform(baseLocation, viewToWorld);
        } else { // kinematic
            baseLocation.set(getSpatialTranslation());
            setPhysicsLocation(baseLocation);
            viewToWorld.set(getSpatialRotation());
            setPhysicsRotation(viewToWorld);
        }
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

        capsule.write(initialRadius, tagRadius, 1f);
        capsule.write(initialHeight, tagHeight, 2f);
        capsule.write(mass, tagMass, 80f);
        capsule.write(jumpImpulse, tagJumpForce, null);
        capsule.write(dampingFactor, tagPhysicsDamping, 0.9f);
        capsule.write(duckedFactor, tagDuckedFactor, 0.6f);
        capsule.write(viewDirection, tagViewDirection, null);
        capsule.write(walkVelocity, tagWalkDirection, null);
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
        if (rigidBody.isDynamic()) {
            rigidBody.getLinearVelocity(velocity);
        } else { // kinematic
            velocity.zero();
        }
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
            this.isDucked = false;
        }

        if (rigidBody.isDynamic()) {
            dynamicPreTick();
        }
        this.wantToJump = false;
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
    protected void calculateNewForward(
            Quaternion rotation, Vector3f direction, Vector3f worldUpVector) {
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
            } else {
                newLeft.set(0f, direction.z, -direction.y);
            }
            newLeft.normalizeLocal();
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
         * Sweep a sphere upward, from the current location
         * of the upper hemisphere to its (hypothetical) unducked location.
         */
        Vector3f startLocation = sweepBegin.getTranslation(); // alias
        startLocation.set(baseLocation);
        float currentHeight = getFinalHeight();
        float bodyRadius = getFinalRadius();
        MyVector3f.accumulateScaled(
                startLocation, localUp, currentHeight - bodyRadius);

        Vector3f endLocation = sweepEnd.getTranslation(); // alias
        endLocation.set(baseLocation);
        MyVector3f.accumulateScaled(
                endLocation, localUp, initialHeight - bodyRadius);

        if (sweepShape == null || sweepShape.getRadius() != bodyRadius) {
            this.sweepShape = new SphereCollisionShape(bodyRadius);
        }
        PhysicsSpace space = getPhysicsSpace();
        List<PhysicsSweepTestResult> results
                = space.sweepTest(sweepShape, sweepBegin, sweepEnd);

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
         * Sweep a sphere downward, from the center of the capsule
         * to one collision margin below the center of its lower hemisphere.
         */
        Vector3f startLocation = sweepBegin.getTranslation(); // alias
        startLocation.set(baseLocation);
        float scaledHeight = getFinalHeight();
        MyVector3f.accumulateScaled(startLocation, localUp, scaledHeight / 2f);

        Vector3f endLocation = sweepEnd.getTranslation(); // alias
        endLocation.set(baseLocation);
        float bodyRadius = getFinalRadius();
        float margin = rigidBody.getCollisionShape().getMargin();
        MyVector3f.accumulateScaled(endLocation, localUp, bodyRadius - margin);

        if (sweepShape == null || sweepShape.getRadius() != bodyRadius) {
            this.sweepShape = new SphereCollisionShape(bodyRadius);
        }
        PhysicsSpace space = getPhysicsSpace();
        List<PhysicsSweepTestResult> results
                = space.sweepTest(sweepShape, sweepBegin, sweepEnd);

        // Search for a collision object other than the character's body.
        boolean isSupported = false;
        for (PhysicsSweepTestResult result : results) {
            PhysicsCollisionObject object = result.getCollisionObject();
            if (!object.equals(rigidBody)) {
                isSupported = true;
                break;
            }
        }

        // Update the status.
        this.onGround = isSupported;
    }

    /**
     * Return the scaled height of the collision shape, including both
     * hemispheres and the cylindrical part.
     *
     * @return the height (in physics-space units, &gt;0)
     */
    protected float getFinalHeight() {
        float result = initialHeight * scale.y;

        assert result > 0f : result;
        return result;
    }

    /**
     * Return the scaled radius of the collision shape.
     *
     * @return the radius (in physics-space units, &gt;0)
     */
    protected float getFinalRadius() {
        float result = initialRadius * scale.z;

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
     * vectors, adapts localForward and localToWorld.
     */
    protected void updateLocalCoordinateSystem() {
        /*
         * The gravity vector may have changed,
         * so update localToWorld and localForward.
         */
        calculateNewForward(localToWorld, localForward, localUp);
        localLeft.set(localUp);
        localLeft.crossLocal(localForward);

        rigidBody.setPhysicsRotation(localToWorld);
        updateLocalViewDirection();
    }

    /**
     * Update the character's viewpoint.
     */
    protected void updateLocalViewDirection() {
        viewDirInWorld.set(viewDirection);
        localToWorld.multLocal(viewDirInWorld);
        /*
         * The gravity vector may have changed,
         * so update viewToWorld and viewDirInWorld.
         */
        calculateNewForward(viewToWorld, viewDirInWorld, localUp);
    }
    // *************************************************************************
    // private methods

    /**
     * Apply impulses and delta vees to the dynamic rigid body. Invoked just
     * before the physics is stepped.
     */
    private void dynamicPreTick() {
        TempVars vars = TempVars.get();
        Vector3f currentVelocity = vars.vect2; // alias
        currentVelocity.set(velocity);

        // Attenuate any horizontal motion with a counteracting delta V.
        float left = velocity.dot(localLeft) * dampingFactor;
        float forward = velocity.dot(localForward) * dampingFactor;
        Vector3f counter = vars.vect1; // alias
        counter.set(-left, 0f, -forward);
        localToWorld.multLocal(counter);
        velocity.addLocal(counter);

        float requestedSpeed = walkVelocity.length();
        if (requestedSpeed > 0f) {
            Vector3f localWalkDirection = vars.vect1; // alias
            localWalkDirection.set(walkVelocity);
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

        if (wantToJump && onGround) {
            Vector3f impulseInWorld = vars.vect1; // alias
            impulseInWorld.set(jumpImpulse);
            localToWorld.multLocal(impulseInWorld);
            rigidBody.applyCentralImpulse(impulseInWorld);
        }
        vars.release();
    }
}
