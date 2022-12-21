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
import com.jme3.bullet.collision.CollisionFlag;
import com.jme3.bullet.collision.PcoType;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.ConvexShape;
import com.jme3.bullet.objects.infos.CharacterController;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import com.simsilica.mathd.Vec3d;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A collision object for simplified character simulation, based on Bullet's
 * {@code btKinematicCharacterController}.
 *
 * @author normenhansen
 */
public class PhysicsCharacter extends PhysicsCollisionObject {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(PhysicsCharacter.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagContactResponse = "contactResponse";
    final private static String tagController = "controller";
    final private static String tagPhysicsLocation = "physicsLocation";
    /**
     * default gravity vector -- differs from that of
     * btKinematicCharacterController!
     */
    final private static Vector3f defaultGravity = new Vector3f(0f, -29.4f, 0f);
    /**
     * local copy of {@link com.jme3.math.Vector3f#ZERO}
     */
    final private static Vector3f translateIdentity = new Vector3f(0f, 0f, 0f);
    /**
     * local copy of {@link com.jme3.math.Vector3f#UNIT_Y}
     */
    final private static Vector3f unitY = new Vector3f(0f, 1f, 0f);
    // *************************************************************************
    // fields

    /**
     * controller or "action" for this character
     */
    private CharacterController controller;
    /**
     * temporary storage for one vector per thread
     */
    final private static ThreadLocal<Vector3f> threadTmpVector
            = new ThreadLocal<Vector3f>() {
        @Override
        protected Vector3f initialValue() {
            return new Vector3f();
        }
    };
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected PhysicsCharacter() {
    }

    /**
     * Instantiate a responsive character with the specified CollisionShape and
     * step height.
     *
     * @param shape the desired shape (not null, convex, alias created)
     * @param stepHeight the maximum amount of vertical movement without jumping
     * or falling (in physics-space units)
     */
    public PhysicsCharacter(ConvexShape shape, float stepHeight) {
        Validate.nonNull(shape, "shape");

        super.setCollisionShape(shape);
        buildObject();
        setStepHeight(stepHeight);
        /*
         * The default gravity for a Bullet btKinematicCharacterController
         * is (0,0,-29.4), which makes no sense for JME.
         * So override the default.
         */
        setGravity(defaultGravity);
        setUp(unitY);

        // Initialize the location.
        warp(translateIdentity);

        assert isContactResponse();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Determine this character's angular damping.
     *
     * @return the viscous damping ratio (0&rarr;no damping, 1&rarr;critically
     * damped)
     */
    public float getAngularDamping() {
        float result = controller.getAngularDamping();
        return result;
    }

    /**
     * Determine this character's angular velocity.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the velocity vector (either storeResult or a new vector, not
     * null)
     */
    public Vector3f getAngularVelocity(Vector3f storeResult) {
        Vector3f result = controller.getAngularVelocity(storeResult);
        return result;
    }

    /**
     * Determine the ID of the btKinematicCharacterController. Used internally.
     *
     * @return the unique identifier (not zero)
     */
    public long getControllerId() {
        long result = controller.nativeId();
        return result;
    }

    /**
     * Determine this character's maximum fall speed (terminal velocity).
     *
     * @return the speed (in physics-space units per second)
     */
    public float getFallSpeed() {
        float result = controller.getFallSpeed();
        return result;
    }

    /**
     * Determine this character's gravitational acceleration.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return an acceleration vector (in physics-space units per second
     * squared, direction opposite the "up" vector, either storeResult or a new
     * vector, not null)
     */
    public Vector3f getGravity(Vector3f storeResult) {
        Vector3f result = controller.getGravity(storeResult);
        return result;
    }

    /**
     * Determine this character's jump speed.
     *
     * @return the speed (in physics-space units per second)
     */
    public float getJumpSpeed() {
        float result = controller.getJumpSpeed();
        return result;
    }

    /**
     * Determine this character's linear damping.
     *
     * @return the viscous damping ratio (0&rarr;no damping, 1&rarr;critically
     * damped)
     */
    public float getLinearDamping() {
        float result = controller.getLinearDamping();
        return result;
    }

    /**
     * Determine the linear velocity of this character's center.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a vector (either storeResult or a new vector, not null)
     */
    public Vector3f getLinearVelocity(Vector3f storeResult) {
        Vector3f result = controller.getLinearVelocity(storeResult);
        return result;
    }

    /**
     * Determine this character's maximum penetration depth.
     *
     * @return the depth (in physics-space units)
     */
    public float getMaxPenetrationDepth() {
        float result = controller.getMaxPenetrationDepth();
        return result;
    }

    /**
     * Determine this character's maximum slope angle.
     *
     * @return the angle relative to the horizontal (in radians)
     */
    public float getMaxSlope() {
        float result = controller.getMaxSlope();
        return result;
    }

    /**
     * Determine this character's step height.
     *
     * @return the maximum amount of vertical movement without jumping or
     * falling (in physics-space units)
     */
    public float getStepHeight() {
        float result = controller.getStepHeight();
        return result;
    }

    /**
     * Determine this character's "up" direction.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a unit vector (in physics-space coordinates, in the direction
     * opposite the gravity vector, either storeResult or a new vector, not
     * null)
     */
    public Vector3f getUpDirection(Vector3f storeResult) {
        Vector3f result = controller.getUpDirection(storeResult);
        return result;
    }

    /**
     * Determine the character's walk offset.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return an offset vector (either storeResult or a new vector, not null)
     */
    public Vector3f getWalkDirection(Vector3f storeResult) {
        Vector3f result = controller.getWalkDirection(storeResult);
        return result;
    }

    /**
     * Test whether the ghost's convex-sweep test is in use.
     *
     * @return true if using the ghost's test, otherwise false
     */
    public boolean isUsingGhostSweepTest() {
        boolean result = controller.isUsingGhostSweepTest();
        return result;
    }

    /**
     * Jump in the "up" direction.
     */
    public void jump() {
        jump(translateIdentity);
    }

    /**
     * Jump in the specified direction.
     *
     * @param direction desired jump direction (not null, unaffected) or (0,0,0)
     * to jump in the "up" direction
     */
    public void jump(Vector3f direction) {
        controller.jump(direction);
    }

    /**
     * Test whether this character is on the ground.
     *
     * @return true if on the ground, otherwise false
     */
    public boolean onGround() {
        boolean result = controller.onGround();
        return result;
    }

    /**
     * Reset this character, including its velocity.
     *
     * @param space (not null)
     */
    public void reset(PhysicsSpace space) {
        Validate.nonNull(space, "space");
        assert space == getCollisionSpace();

        controller.reset(space);
    }

    /**
     * Alter this character's angular damping.
     *
     * @param damping the desired viscous damping ratio (0&rarr;no damping,
     * 1&rarr;critically damped, default=0)
     */
    public void setAngularDamping(float damping) {
        controller.setAngularDamping(damping);
    }

    /**
     * Alter this character's angular velocity.
     *
     * @param angularVelocity the desired angular velocity vector (not null,
     * unaffected)
     */
    public void setAngularVelocity(Vector3f angularVelocity) {
        Validate.nonNull(angularVelocity, "angular velocity");
        controller.setAngularVelocity(angularVelocity);
    }

    /**
     * Apply the specified CollisionShape to this character. Note that the
     * character should not be in any PhysicsSpace while changing shape; the
     * character gets rebuilt on the physics side.
     *
     * @param collisionShape the shape to apply (not null, convex, alias
     * created)
     */
    @Override
    public void setCollisionShape(CollisionShape collisionShape) {
        assert collisionShape.isConvex();
        assert !isInWorld();

        super.setCollisionShape(collisionShape);
        if (hasAssignedNativeObject()) {
            long objectId = nativeId();
            long shapeId = collisionShape.nativeId();
            attachCollisionShape(objectId, shapeId);
        } else {
            buildObject();
        }
    }

    /**
     * Enable/disable this character's contact response.
     *
     * @param newState true to respond to contact forces, false to ignore them
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
     * Alter this character's maximum fall speed (terminal velocity).
     *
     * @param fallSpeed the desired speed (in physics-space units per second,
     * default=55)
     */
    public void setFallSpeed(float fallSpeed) {
        controller.setFallSpeed(fallSpeed);
    }

    /**
     * Alter the character's gravitational acceleration without altering its
     * "up" vector.
     *
     * @param downwardAcceleration the desired downward acceleration (in
     * physics-space units per second squared, default=29.4)
     */
    public void setGravity(float downwardAcceleration) {
        Vector3f gVector = threadTmpVector.get();
        getUpDirection(gVector);
        gVector.multLocal(-downwardAcceleration);
        setGravity(gVector);
    }

    /**
     * Alter this character's gravitational acceleration. This may also alter
     * its "up" vector.
     *
     * @param gravity the desired acceleration vector (in physics-space units
     * per second squared, not null, unaffected, default=(0,-29.4,0))
     */
    public void setGravity(Vector3f gravity) {
        Validate.nonNull(gravity, "gravity");
        controller.setGravity(gravity);
    }

    /**
     * Alter this character's jump speed.
     *
     * @param jumpSpeed the desired speed (in physics-space units per second,
     * default=10)
     */
    public void setJumpSpeed(float jumpSpeed) {
        controller.setJumpSpeed(jumpSpeed);
    }

    /**
     * Alter this character's linear damping.
     *
     * @param damping the desired viscous damping ratio (0&rarr;no damping,
     * 1&rarr;critically damped, default=0)
     */
    public void setLinearDamping(float damping) {
        controller.setLinearDamping(damping);
    }

    /**
     * Alter the linear velocity of this character's center.
     *
     * @param velocity the desired velocity vector (not null)
     */
    public void setLinearVelocity(Vector3f velocity) {
        Validate.nonNull(velocity, "velocity");
        controller.setLinearVelocity(velocity);
    }

    /**
     * Alter this character's maximum penetration depth.
     *
     * @param depth the desired depth (in physics-space units, default=0.2)
     */
    public void setMaxPenetrationDepth(float depth) {
        controller.setMaxPenetrationDepth(depth);
    }

    /**
     * Alter this character's maximum slope angle.
     *
     * @param slopeRadians the desired angle above to the horizontal plane (in
     * radians, &ge;0, &le;Pi/2, default=Pi/4)
     */
    public void setMaxSlope(float slopeRadians) {
        Validate.inRange(slopeRadians, "slope radians", 0f, FastMath.HALF_PI);
        controller.setMaxSlope(slopeRadians);
    }

    /**
     * Directly alter this character's location. (This is equivalent to
     * {@link #warp(com.jme3.math.Vector3f)}.)
     *
     * @param location the desired location (not null, finite, unaffected)
     */
    public void setPhysicsLocation(Vector3f location) {
        Validate.finite(location, "location");
        controller.warp(location);
    }

    /**
     * Directly alter this character's location.
     *
     * @param location the desired location (not null, unaffected)
     */
    public void setPhysicsLocationDp(Vec3d location) {
        Validate.nonNull(location, "location");
        controller.warpDp(location);
    }

    /**
     * Alter this character's step height.
     *
     * @param height the desired maximum amount of vertical movement without
     * jumping or falling (in physics-space units, default=1)
     */
    public void setStepHeight(float height) {
        controller.setStepHeight(height);
    }

    /**
     * Alter which convex-sweep test is used.
     *
     * @param useGhostSweepTest true to use the ghost's test, false to use the
     * world's test (default=true)
     */
    public void setSweepTest(boolean useGhostSweepTest) {
        controller.setSweepTest(useGhostSweepTest);
    }

    /**
     * Alter this character's "up" direction. This may also alter its gravity
     * vector.
     *
     * @param direction the desired direction (not null, not zero, unaffected,
     * default=(0,1,0))
     */
    public void setUp(Vector3f direction) {
        Validate.nonZero(direction, "direction");
        controller.setUp(direction);
    }

    /**
     * Alter this character's walk offset. The offset must be perpendicular to
     * the "up" direction. It will continue to be applied until altered again.
     *
     * @param offset the desired location increment for each simulation step (in
     * physics-space coordinates, not null, unaffected, default=(0,0,0))
     */
    public void setWalkDirection(Vector3f offset) {
        Validate.nonNull(offset, "offset");
        controller.setWalkDirection(offset);
    }

    /**
     * Directly alter the location of this character's center.
     *
     * @param location the desired physics location (not null, finite,
     * unaffected)
     */
    public void warp(Vector3f location) {
        Validate.finite(location, "location");
        controller.warp(location);
    }
    // *************************************************************************
    // PhysicsCollisionObject methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned character into a deep-cloned one, using the specified
     * Cloner and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this character (not null)
     * @param original the instance from which this character was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);
        unassignNativeObject();

        this.controller = null;
        buildObject();

        PhysicsCharacter old = (PhysicsCharacter) original;
        cloneIgnoreList(cloner, old);
        copyPcoProperties(old);
        setContactResponse(old.isContactResponse());
        setPhysicsLocation(old.getPhysicsLocation(null));

        controller.copyAll(old.controller);
    }

    /**
     * De-serialize this character from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);

        InputCapsule capsule = importer.getCapsule(this);
        buildObject();
        readPcoProperties(capsule);

        CharacterController tmp = (CharacterController) capsule.readSavable(
                tagController, null);
        controller.copyAll(tmp);

        setContactResponse(capsule.readBoolean(tagContactResponse, true));
        warp((Vector3f) capsule.readSavable(tagPhysicsLocation,
                new Vector3f()));
    }

    /**
     * Serialize this character to the specified exporter, for example when
     * saving to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        super.write(exporter);
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(isContactResponse(), tagContactResponse, true);
        capsule.write(controller, tagController, null);
        capsule.write(getPhysicsLocation(), tagPhysicsLocation,
                null);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Create the configured objects in Bullet.
     */
    private void buildObject() {
        if (!hasAssignedNativeObject()) {
            long objectId = createGhostObject();
            setNativeId(objectId);
            assert getInternalType(objectId) == PcoType.ghost :
                    getInternalType(objectId);
            logger2.log(Level.FINE, "Creating {0}.", this);

            initUserPointer();
        }

        long objectId = nativeId();
        setCharacterFlags(objectId);

        CollisionShape shape = getCollisionShape();
        assert shape.isConvex();
        long shapeId = shape.nativeId();
        attachCollisionShape(objectId, shapeId);

        this.controller = new CharacterController(this);
        logger2.log(Level.FINE, "Creating {0}.", this);
    }
    // *************************************************************************
    // native private methods

    native private static long createGhostObject();

    native private static void setCharacterFlags(long ghostId);
}
