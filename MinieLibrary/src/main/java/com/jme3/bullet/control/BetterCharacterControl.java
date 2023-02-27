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
import com.jme3.bullet.collision.PhysicsRayTestResult;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
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
 * A rigid body with CapsuleCollisionShape is used and its velocity is set
 * continuously. A ray test is used to test whether the character is on the
 * ground.
 * <p>
 * The character keeps their own local coordinate system which adapts based on
 * the gravity working on the character, so they will always stand upright.
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
     * true when jump impulse will be applied during the next simulation step
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
     * relative height when ducked (&gt;0, &le;1, 1=full height)
     */
    private float duckedFactor = 0.6f;
    /**
     * initial height of the collision shape (in physics-space units, includes
     * both hemispheres and the cylindrical part)
     */
    private float height;
    /**
     * mass of this character (&gt;0)
     */
    private float mass;
    /**
     * X-Z motion attenuation factor (0&rarr;no damping, 1=no external forces)
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
     * Local z-forward Quaternion for the "local absolute" z-forward direction.
     */
    private Quaternion localForwardRotation = new Quaternion();
    /**
     * spatial rotation, a Z-forward rotation based on the view direction and
     * local X-Z plane.
     *
     * @see #rotatedViewDirection
     */
    private Quaternion rotation = new Quaternion();
    /**
     * impulse applied at the start of a jump (in local coordinates)
     */
    private Vector3f jumpForce = new Vector3f();
    /**
     * Local absolute z-forward direction, derived from gravity and UNIT_Z,
     * updated continuously when gravity changes.
     */
    private Vector3f localForward = new Vector3f(0f, 0f, 1f);
    /**
     * Local left direction, derived from up and forward.
     */
    private Vector3f localLeft = new Vector3f(1f, 0f, 0f);
    /**
     * local up direction, derived from gravity
     */
    private Vector3f localUp = new Vector3f(0f, 1f, 0f);
    /**
     * spatial location, corresponds to RigidBody location.
     */
    private Vector3f location = new Vector3f();
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
     * a Z-forward vector based on the view direction and the local X-Z plane.
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
     * The final height when ducking must be larger than 2x radius. The
     * jumpForce will be set to an upward force of 5x mass.
     *
     * @param radius the radius of the character's CollisionShape (&gt;0)
     * @param height the height of the character's CollisionShape (&gt;2*radius)
     * @param mass the character's mass (&ge;0)
     */
    public BetterCharacterControl(float radius, float height, float mass) {
        Validate.positive(radius, "radius");
        assert height > 2f * radius : height;
        Validate.positive(mass, "mass");

        this.radius = radius;
        this.height = height;
        this.mass = mass;
        this.rigidBody = new PhysicsRigidBody(getShape(), mass);
        this.jumpForce = new Vector3f(0f, mass * 5f, 0f);
        this.rigidBody.setAngularFactor(0f);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Read the height multiplier for ducking.
     *
     * @return the factor (&ge;0, &le;1)
     */
    public float getDuckedFactor() {
        return duckedFactor;
    }

    /**
     * Copy the character's gravity vector.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return an acceleration vector (either the provided storage or a new
     * vector, not null)
     */
    public Vector3f getGravity(Vector3f storeResult) {
        return rigidBody.getGravity(storeResult);
    }

    /**
     * Copy the character's jump force.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a force vector (either the provided storage or a new vector, not
     * null)
     */
    public Vector3f getJumpForce(Vector3f storeResult) {
        if (storeResult == null) {
            return jumpForce.clone();
        } else {
            return storeResult.set(jumpForce);
        }
    }

    /**
     * Read how much motion in the local X-Z plane is damped.
     *
     * @return the damping factor (0&rarr;no damping, 1=no external forces)
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
     * For compatibility with the jme3-bullet library.
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
     * @return a velocity vector (either the provided storage or a new vector,
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
     * For compatibility with the jme3-bullet library.
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
     * @return a direction vector (in physics-space coordinates, either the
     * provided storage or a new vector, not null)
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
     * speed.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a velocity vector (in physics-space units per second, either the
     * provided storage or a new vector, not null)
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
     * Check if the character is ducking, either due to user input or due to
     * unducking being impossible at the moment (obstacle above).
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
     * Makes the character jump with the set jump force.
     */
    public void jump() {
        // TODO: debounce over some frames
        if (!onGround) {
            return;
        }
        this.jump = true;
    }

    /**
     * Realign the local forward vector to given direction vector. If null is
     * supplied, Vector3f.UNIT_Z is used. The input vector must be perpendicular
     * to gravity vector. This normally only needs to be invoked when the
     * gravity direction changed continuously and the local forward vector is
     * off due to drift. E.g. after walking around on a sphere "planet" for a
     * while and then going back to a Y-up coordinate system the local Z-forward
     * might not be 100% aligned with the Z axis.
     *
     * @param vec the desired forward vector (perpendicular to the gravity
     * vector, may be null, default=(0,0,1))
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
                setHeightPercent(1);
                this.ducked = false;
            } else {
                this.wantToUnDuck = true;
            }
        }
    }

    /**
     * Alter the height multiplier for ducking.
     *
     * @param factor the factor by which the height should be multiplied when
     * ducking (&ge;0, &le;1)
     */
    public void setDuckedFactor(float factor) {
        this.duckedFactor = factor;
    }

    /**
     * Alter the gravity acting on this character. Note that this also realigns
     * the local coordinate system of the character so that continuous changes
     * in gravity direction are possible while maintaining a sensible control
     * over the character.
     *
     * @param gravity an acceleration vector (not null, unaffected)
     */
    public void setGravity(Vector3f gravity) {
        rigidBody.setGravity(gravity);
        localUp.set(gravity).normalizeLocal().negateLocal();
        updateLocalCoordinateSystem();
    }

    /**
     * Alter the jump force. The jump force is local to the character's
     * coordinate system, which normally is always z-forward (in world
     * coordinates, parent coordinates when set to applyLocalPhysics)
     *
     * @param jumpForce the desired jump force (not null, finite, unaffected,
     * default=5*mass in +Y direction)
     */
    public void setJumpForce(Vector3f jumpForce) {
        Validate.finite(jumpForce, "jump force");

        this.jumpForce.set(jumpForce);
    }

    /**
     * Alter how much motion in the local X-Z plane is damped.
     *
     * @param physicsDamping the desired damping factor (0&rarr;no damping, 1=no
     * external forces, default=0.9)
     */
    public void setPhysicsDamping(float physicsDamping) {
        this.physicsDamping = physicsDamping;
    }

    /**
     * Alter the character's view direction. Note this only defines the
     * orientation in the local X-Z plane.
     *
     * @param vec a direction vector (not null, unaffected)
     */
    public void setViewDirection(Vector3f vec) {
        viewDirection.set(vec);
        updateLocalViewDirection();
    }

    /**
     * Alter the character's walk velocity, which will be applied on every
     * simulation step.
     *
     * @param vec the desired velocity (in physics-space units per second)
     */
    public void setWalkDirection(Vector3f vec) {
        walkDirection.set(vec);
    }

    /**
     * Move the character somewhere. Note the character also warps to the
     * location of the Spatial when the Control is added.
     *
     * @param vec the desired character location (in physics-space coordinates,
     * not null, finite, unaffected)
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
        space.getGravity(localUp).normalizeLocal().negateLocal();
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
        location.set(location);
    }

    /**
     * Rotate the character to the specified orientation.
     * <p>
     * We don't set the actual physics rotation but the view rotation here. It
     * might actually be altered by the calculateNewForward method.
     *
     * @param orientation the desired orientation (in physics-space coordinates,
     * not null, not zero, unaffected)
     */
    @Override
    protected void setPhysicsRotation(Quaternion orientation) {
        Validate.nonZero(orientation, "orientation");

        rotation.set(orientation);
        rotation.multLocal(rotatedViewDirection.set(viewDirection));
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
            setHeightPercent(1);
            this.wantToUnDuck = false;
            this.ducked = false;
        }
        TempVars vars = TempVars.get();

        Vector3f currentVelocity = vars.vect2.set(velocity);

        // Attenuate any existing X-Z motion.
        float existingLeftVelocity = velocity.dot(localLeft);
        float existingForwardVelocity = velocity.dot(localForward);
        Vector3f counter = vars.vect1;
        existingLeftVelocity *= physicsDamping;
        existingForwardVelocity *= physicsDamping;
        counter.set(-existingLeftVelocity, 0, -existingForwardVelocity);
        localForwardRotation.multLocal(counter);
        velocity.addLocal(counter);

        float designatedVelocity = walkDirection.length();
        if (designatedVelocity > 0) {
            Vector3f localWalkDirection = vars.vect1;
            // normalize walk direction
            localWalkDirection.set(walkDirection).normalizeLocal();
            // check for the existing velocity in the desired direction
            float existingVelocity = velocity.dot(localWalkDirection);
            // calculate the final velocity in the desired direction
            float finalVelocity = designatedVelocity - existingVelocity;
            localWalkDirection.multLocal(finalVelocity);
            // add resulting vector to existing velocity
            velocity.addLocal(localWalkDirection);
        }
        if (currentVelocity.distance(velocity) > FastMath.ZERO_TOLERANCE) {
            rigidBody.setLinearVelocity(velocity);
        }
        if (jump) {
            // TODO: precalculate jump force
            Vector3f rotatedJumpForce = vars.vect1;
            rotatedJumpForce.set(jumpForce);
            rigidBody.applyCentralImpulse(
                    localForwardRotation.multLocal(rotatedJumpForce));
            this.jump = false;
        }
        vars.release();
    }
    // *************************************************************************
    // new protected methods

    /**
     * This method works similar to Camera.lookAt but where lookAt sets the
     * priority on the direction, this method sets the priority on the up vector
     * so that the result direction vector and rotation is guaranteed to be
     * perpendicular to the up vector.
     *
     * @param rotation The rotation to set the result on or null to create a new
     * Quaternion, this will be set to the new "z-forward" rotation if not null
     * @param direction The direction to base the new look direction on, will be
     * set to the new direction
     * @param worldUpVector The up vector to use, the result direction will be
     * perpendicular to this
     */
    protected void calculateNewForward(Quaternion rotation,
            Vector3f direction, Vector3f worldUpVector) {
        if (direction == null) {
            return;
        }
        TempVars vars = TempVars.get();
        Vector3f newLeft = vars.vect1;
        Vector3f newLeftNegate = vars.vect2;

        newLeft.set(worldUpVector).crossLocal(direction).normalizeLocal();
        if (MyVector3f.isZero(newLeft)) {
            if (direction.x != 0) {
                newLeft.set(direction.y, -direction.x, 0f).normalizeLocal();
            } else {
                newLeft.set(0f, direction.z, -direction.y).normalizeLocal();
            }
            if (logger2.isLoggable(Level.INFO)) {
                logger2.log(Level.INFO, "Zero left for direction {0}, up {1}",
                        new Object[]{direction, worldUpVector});
            }
        }
        newLeftNegate.set(newLeft).negateLocal();
        direction.set(worldUpVector).crossLocal(newLeftNegate).normalizeLocal();
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
     * This checks if the character can go from ducked to unducked state by
     * doing a ray test.
     *
     * @return true if able to unduck, otherwise false
     */
    protected boolean checkCanUnDuck() {
        TempVars vars = TempVars.get();
        Vector3f loc = vars.vect1;
        Vector3f rayVector = vars.vect2;
        loc.set(localUp).multLocal(FastMath.ZERO_TOLERANCE)
                .addLocal(this.location);
        rayVector.set(localUp)
                .multLocal(height + FastMath.ZERO_TOLERANCE).addLocal(loc);
        List<PhysicsRayTestResult> results
                = getPhysicsSpace().rayTestRaw(loc, rayVector);
        vars.release();
        for (PhysicsRayTestResult physicsRayTestResult : results) {
            if (!physicsRayTestResult.getCollisionObject().equals(rigidBody)) {
                return false;
            }
        }
        return true;
    }

    /**
     * Test whether the character is on the ground, by means of a ray test.
     */
    protected void checkOnGround() {
        TempVars vars = TempVars.get();
        Vector3f loc = vars.vect1;
        Vector3f rayVector = vars.vect2;
        float scaledHeight = getFinalHeight();
        loc.set(localUp).multLocal(scaledHeight).addLocal(location);
        rayVector.set(localUp).multLocal(-scaledHeight - 0.1f).addLocal(loc);
        List<PhysicsRayTestResult> results
                = getPhysicsSpace().rayTestRaw(loc, rayVector);
        vars.release();

        for (PhysicsRayTestResult physicsRayTestResult : results) {
            if (!physicsRayTestResult.getCollisionObject().equals(rigidBody)) {
                this.onGround = true;
                return;
            }
        }
        this.onGround = false;
    }

    /**
     * Calculate the character's scaled height.
     *
     * @return the height
     */
    protected float getFinalHeight() {
        return height * scale.getY();
    }

    /**
     * Calculate the character's scaled radius.
     *
     * @return the radius
     */
    protected float getFinalRadius() {
        return radius * scale.getZ();
    }

    /**
     * Create a CollisionShape based on the scale parameter. The new shape is a
     * compound shape containing a capsule.
     *
     * @return a new compound shape (not null)
     */
    protected CollisionShape getShape() {
        // TODO: cleanup size mess
        CapsuleCollisionShape capsuleCollisionShape
                = new CapsuleCollisionShape(getFinalRadius(),
                        (getFinalHeight() - (2f * getFinalRadius())));
        CompoundCollisionShape compoundCollisionShape
                = new CompoundCollisionShape(1);
        compoundCollisionShape.addChildShape(
                capsuleCollisionShape, 0f, getFinalHeight() / 2f, 0f);

        return compoundCollisionShape;
    }

    /**
     * Alter the height of the CollisionShape.
     *
     * @param percent the desired height, as a fraction of the initial height
     * (default=1)
     */
    protected void setHeightPercent(float percent) {
        scale.setY(percent);
        rigidBody.setCollisionShape(getShape());
    }

    /**
     * Updates the local coordinate system from the localForward and localUp
     * vectors, adapts localForward, sets localForwardRotation to local
     * Z-forward rotation.
     */
    protected void updateLocalCoordinateSystem() {
        /*
         * gravity vector has possibly changed,
         * calculate new world forward (UNIT_Z)
         */
        calculateNewForward(localForwardRotation, localForward, localUp);
        localLeft.set(localUp).crossLocal(localForward);
        rigidBody.setPhysicsRotation(localForwardRotation);
        updateLocalViewDirection();
    }

    /**
     * Updates the local X-Z view direction and the corresponding rotation
     * Quaternion for the Spatial.
     */
    protected void updateLocalViewDirection() {
        //update local rotation Quaternion to use for view rotation
        localForwardRotation.multLocal(rotatedViewDirection.set(viewDirection));
        calculateNewForward(rotation, rotatedViewDirection, localUp);
    }
}
