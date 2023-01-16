/*
 * Copyright (c) 2009-2023 jMonkeyEngine
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

import com.jme3.bullet.NativePhysicsObject;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.objects.PhysicsCharacter;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import com.jme3.util.clone.JmeCloneable;
import com.simsilica.mathd.Vec3d;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * The "action" (controller) portion of a PhysicsCharacter, based on Bullet's
 * btKinematicCharacterController.
 *
 * @author Stephen Gold sgold@sonic.net
 *
 * Based on PhysicsCharacter by normenhansen.
 */
public class CharacterController
        extends NativePhysicsObject
        implements JmeCloneable, Savable {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(CharacterController.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagAngularDamping = "angularDamping";
    final private static String tagAngularVelocity = "angularVelocity";
    final private static String tagFallSpeed = "fallSpeed";
    final private static String tagGhostSweepTest = "ghostSweepTest";
    final private static String tagGravityVector = "gravityVector";
    final private static String tagJumpSpeed = "jumpSpeed";
    final private static String tagLinearDamping = "linearDamping";
    final private static String tagLinearVelocity = "linearVelocity";
    final private static String tagMaxPenetrationDepth = "maxPenetrationDepth";
    final private static String tagMaxSlope = "maxSlope";
    final private static String tagPco = "pco";
    final private static String tagStepHeight = "stepHeight";
    final private static String tagUpDirection = "upDirection";
    final private static String tagWalkDirection = "walkDirection";
    // *************************************************************************
    // fields

    /**
     * collision object being controlled
     */
    private PhysicsCharacter pco;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected CharacterController() {
    }

    /**
     * Instantiate a controller for the specified collision object.
     *
     * @param character the collision object to control (not null)
     */
    public CharacterController(PhysicsCharacter character) {
        Validate.nonNull(character, "collision object");

        this.pco = character;
        createController();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy all parameter values from the specified controller.
     *
     * @param source the controller to copy from (not null, unaffected)
     */
    public void copyAll(CharacterController source) {
        setAngularDamping(source.getAngularDamping());
        setAngularVelocity(source.getAngularVelocity(null));
        setFallSpeed(source.getFallSpeed());
        setGravity(source.getGravity(null));
        setJumpSpeed(source.getJumpSpeed());
        setLinearDamping(source.getLinearDamping());

        // Walk direction affects linear velocity, so set it first!
        setWalkDirection(source.getWalkDirection(null));
        setLinearVelocity(source.getLinearVelocity(null));

        setMaxPenetrationDepth(source.getMaxPenetrationDepth());
        setMaxSlope(source.getMaxSlope());
        setStepHeight(source.getStepHeight());
        setSweepTest(source.isUsingGhostSweepTest());
        setUp(source.getUpDirection(null));
    }

    /**
     * Determine the character's angular damping.
     *
     * @return the viscous damping ratio (0&rarr;no damping, 1&rarr;critically
     * damped)
     */
    public float getAngularDamping() {
        long controllerId = nativeId();
        float result = getAngularDamping(controllerId);

        return result;
    }

    /**
     * Determine the character's angular velocity.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the velocity vector (either storeResult or a new vector, not
     * null)
     */
    public Vector3f getAngularVelocity(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long controllerId = nativeId();
        getAngularVelocity(controllerId, result);

        return result;
    }

    /**
     * Determine the character's maximum fall speed (terminal velocity).
     *
     * @return the speed (in physics-space units per second)
     */
    public float getFallSpeed() {
        long controllerId = nativeId();
        float result = getFallSpeed(controllerId);

        return result;
    }

    /**
     * Determine the character's gravitational acceleration.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return an acceleration vector (in physics-space units per second
     * squared, direction opposite the "up" vector, either storeResult or a new
     * vector, not null)
     */
    public Vector3f getGravity(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long controllerId = nativeId();
        getGravity(controllerId, result);

        return result;
    }

    /**
     * Determine the character's jump speed.
     *
     * @return the speed (in physics-space units per second)
     */
    public float getJumpSpeed() {
        long controllerId = nativeId();
        float result = getJumpSpeed(controllerId);

        return result;
    }

    /**
     * Determine the character's linear damping.
     *
     * @return the viscous damping ratio (0&rarr;no damping, 1&rarr;critically
     * damped)
     */
    public float getLinearDamping() {
        long controllerId = nativeId();
        float result = getLinearDamping(controllerId);

        return result;
    }

    /**
     * Determine the linear velocity of the character's center.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a vector (either storeResult or a new vector, not null)
     */
    public Vector3f getLinearVelocity(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long controllerId = nativeId();
        getLinearVelocity(controllerId, result);

        return result;
    }

    /**
     * Determine the character's maximum penetration depth.
     *
     * @return the depth (in physics-space units)
     */
    public float getMaxPenetrationDepth() {
        long controllerId = nativeId();
        float result = getMaxPenetrationDepth(controllerId);

        return result;
    }

    /**
     * Determine the character's maximum slope angle.
     *
     * @return the angle relative to the horizontal (in radians)
     */
    public float getMaxSlope() {
        long controllerId = nativeId();
        float result = getMaxSlope(controllerId);

        return result;
    }

    /**
     * Determine the character's step height.
     *
     * @return the maximum amount of vertical movement without jumping or
     * falling (in physics-space units)
     */
    public float getStepHeight() {
        long controllerId = nativeId();
        float result = getStepHeight(controllerId);

        return result;
    }

    /**
     * Determine the character's "up" direction.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a unit vector (in physics-space coordinates, in the direction
     * opposite the gravity vector, either storeResult or a new vector, not
     * null)
     */
    public Vector3f getUpDirection(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        long controllerId = nativeId();
        getUpDirection(controllerId, result);

        return result;
    }

    /**
     * Determine the character's walk offset.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return an offset vector (either storeResult or a new vector, not null)
     */
    public Vector3f getWalkDirection(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long controllerId = nativeId();
        getWalkOffset(controllerId, result);

        return result;
    }

    /**
     * Test whether the ghost's convex-sweep test is in use.
     *
     * @return true if using the ghost's convex-sweep test, otherwise false
     */
    public boolean isUsingGhostSweepTest() {
        long controllerId = nativeId();
        boolean result = isUsingGhostSweepTest(controllerId);

        return result;
    }

    /**
     * Jump in the specified direction.
     *
     * @param direction desired jump direction (not null, unaffected) or (0,0,0)
     * to jump in the "up" direction
     */
    public void jump(Vector3f direction) {
        long controllerId = nativeId();
        jump(controllerId, direction);
    }

    /**
     * Test whether the character is on the ground.
     *
     * @return true if on the ground, otherwise false
     */
    public boolean onGround() {
        long controllerId = nativeId();
        boolean result = onGround(controllerId);

        return result;
    }

    /**
     * Reset this controller, including its velocity.
     *
     * @param space (not null)
     */
    public void reset(PhysicsSpace space) {
        long controllerId = nativeId();
        long spaceId = space.nativeId();
        reset(controllerId, spaceId);
    }

    /**
     * Alter the character's angular damping.
     *
     * @param damping the desired viscous damping ratio (0&rarr;no damping,
     * 1&rarr;critically damped, default=0)
     */
    public void setAngularDamping(float damping) {
        long controllerId = nativeId();
        setAngularDamping(controllerId, damping);
    }

    /**
     * Alter the character's angular velocity.
     *
     * @param angularVelocity the desired angular velocity vector (not null,
     * unaffected)
     */
    public void setAngularVelocity(Vector3f angularVelocity) {
        long controllerId = nativeId();
        setAngularVelocity(controllerId, angularVelocity);
    }

    /**
     * Alter the character's maximum fall speed (terminal velocity).
     *
     * @param fallSpeed the desired speed (in physics-space units per second,
     * default=55)
     */
    public void setFallSpeed(float fallSpeed) {
        long controllerId = nativeId();
        setFallSpeed(controllerId, fallSpeed);
    }

    /**
     * Alter the character's gravitational acceleration. This may also alter its
     * "up" vector.
     *
     * @param gravity the desired acceleration vector (in physics-space units
     * per second squared, not null, finite, unaffected, default=(0,-29.4,0))
     */
    public void setGravity(Vector3f gravity) {
        Validate.finite(gravity, "gravity");

        long controllerId = nativeId();
        setGravity(controllerId, gravity);
    }

    /**
     * Alter the character's jump speed.
     *
     * @param jumpSpeed the desired speed (in physics-space units per second,
     * default=10)
     */
    public void setJumpSpeed(float jumpSpeed) {
        long controllerId = nativeId();
        setJumpSpeed(controllerId, jumpSpeed);
    }

    /**
     * Alter the character's linear damping.
     *
     * @param damping the desired viscous damping ratio (0&rarr;no damping,
     * 1&rarr;critically damped, default=0)
     */
    public void setLinearDamping(float damping) {
        long controllerId = nativeId();
        setLinearDamping(controllerId, damping);
    }

    /**
     * Alter the linear velocity of the character's center.
     *
     * @param velocity the desired velocity vector (not null, finite)
     */
    public void setLinearVelocity(Vector3f velocity) {
        Validate.finite(velocity, "velocity");

        long controllerId = nativeId();
        setLinearVelocity(controllerId, velocity);
    }

    /**
     * Alter the character's maximum penetration depth.
     *
     * @param depth the desired depth (in physics-space units, default=0.2)
     */
    public void setMaxPenetrationDepth(float depth) {
        long controllerId = nativeId();
        setMaxPenetrationDepth(controllerId, depth);
    }

    /**
     * Alter the character's maximum slope angle.
     *
     * @param slopeRadians the desired angle relative to the horizontal (in
     * radians, default=Pi/4)
     */
    public void setMaxSlope(float slopeRadians) {
        long controllerId = nativeId();
        setMaxSlope(controllerId, slopeRadians);
    }

    /**
     * Alter the character's step height.
     *
     * @param height the desired maximum amount of vertical movement without
     * jumping or falling (in physics-space units, default=1)
     */
    public void setStepHeight(float height) {
        long controllerId = nativeId();
        setStepHeight(controllerId, height);
    }

    /**
     * Alter which convex-sweep test is used.
     *
     * @param useGhostSweepTest true to use the ghost's test, false to use the
     * world's test (default=true)
     */
    public void setSweepTest(boolean useGhostSweepTest) {
        long controllerId = nativeId();
        setUseGhostSweepTest(controllerId, useGhostSweepTest);
    }

    /**
     * Alter the character's "up" direction. This may also alter its gravity
     * vector.
     *
     * @param direction the desired direction (not null, not zero, unaffected,
     * default=(0,1,0))
     */
    public void setUp(Vector3f direction) {
        Validate.nonZero(direction, "direction");

        long controllerId = nativeId();
        setUp(controllerId, direction);
    }

    /**
     * Alter the character's walk offset. The offset must be perpendicular to
     * the "up" direction. It will continue to be applied until altered again.
     *
     * @param offset the desired location increment for each simulation step (in
     * physics-space coordinates, not null, finite, unaffected, default=(0,0,0))
     */
    public void setWalkDirection(Vector3f offset) {
        Validate.finite(offset, "offset");

        long controllerId = nativeId();
        setWalkDirection(controllerId, offset);
    }

    /**
     * Directly alter the location of the character's center.
     *
     * @param location the desired physics location (not null, finite,
     * unaffected)
     */
    public void warp(Vector3f location) {
        Validate.finite(location, "location");

        long controllerId = nativeId();
        warp(controllerId, location);
    }

    /**
     * Directly alter the location of the character's center.
     *
     * @param location the desired physics location (not null, unaffected)
     */
    public void warpDp(Vec3d location) {
        Validate.nonNull(location, "location");

        long controllerId = nativeId();
        warpDp(controllerId, location);
    }
    // *************************************************************************
    // JmeCloneable methods

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
        CharacterController old = (CharacterController) original;
        copyAll(old);
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public CharacterController jmeClone() {
        try {
            CharacterController clone = (CharacterController) clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }
    // *************************************************************************
    // Savable methods

    /**
     * De-serialize this controller from the specified importer, for example
     * when loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        InputCapsule capsule = importer.getCapsule(this);

        this.pco = (PhysicsCharacter) capsule.readSavable(tagPco, null);
        createController();

        setAngularDamping(capsule.readFloat(tagAngularDamping, 0f));
        setAngularVelocity((Vector3f) capsule.readSavable(tagAngularVelocity,
                new Vector3f()));
        setFallSpeed(capsule.readFloat(tagFallSpeed, 55f));
        setSweepTest(capsule.readBoolean(tagGhostSweepTest, true));
        Vector3f g = (Vector3f) capsule.readSavable(tagGravityVector,
                new Vector3f(0f, -9.81f, 0f));
        setGravity(g);
        setJumpSpeed(capsule.readFloat(tagJumpSpeed, 10f));
        setLinearDamping(capsule.readFloat(tagLinearDamping, 0f));

        // Walk direction affects linear velocity, so set it first!
        setWalkDirection((Vector3f) capsule.readSavable(tagWalkDirection,
                new Vector3f()));
        setLinearVelocity((Vector3f) capsule.readSavable(tagLinearVelocity,
                new Vector3f()));

        setMaxPenetrationDepth(capsule.readFloat(tagMaxPenetrationDepth, 0.2f));
        setMaxSlope(capsule.readFloat(tagMaxSlope, FastMath.QUARTER_PI));
        setStepHeight(capsule.readFloat(tagStepHeight, 1f));
        if (MyVector3f.isZero(g)) {
            setUp((Vector3f) capsule.readSavable(tagUpDirection,
                    new Vector3f(0f, 1f, 0f)));
        }
    }

    /**
     * Serialize this controller to the specified exporter, for example when
     * saving to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(pco, tagPco, null);

        capsule.write(getAngularDamping(), tagAngularDamping, 0f);
        capsule.write(getAngularVelocity(null), tagAngularVelocity, null);
        capsule.write(getFallSpeed(), tagFallSpeed, 55f);
        capsule.write(isUsingGhostSweepTest(), tagGhostSweepTest, true);
        Vector3f g = getGravity(null);
        capsule.write(g, tagGravityVector, new Vector3f(0f, -9.81f, 0f));
        capsule.write(getJumpSpeed(), tagJumpSpeed, 10f);
        capsule.write(getLinearDamping(), tagLinearDamping, 0f);

        capsule.write(getWalkDirection(null), tagWalkDirection, null);
        capsule.write(getLinearVelocity(null), tagLinearVelocity, null);

        capsule.write(getMaxPenetrationDepth(), tagMaxPenetrationDepth, 0.2f);
        capsule.write(getMaxSlope(), tagMaxSlope, FastMath.QUARTER_PI);
        capsule.write(getStepHeight(), tagStepHeight, 1f);
        if (MyVector3f.isZero(g)) {
            capsule.write(getUpDirection(null), tagUpDirection,
                    new Vector3f(0f, 1f, 0f));
        }
    }
    // *************************************************************************
    // Java private methods

    /**
     * Create a btKinematicCharacterController.
     */
    private void createController() {
        long ghostId = pco.nativeId();
        long controllerId = create(ghostId);
        setNativeId(controllerId);
    }

    /**
     * Free the identified tracked native object. Invoked by reflection.
     *
     * @param controllerId the native identifier (not zero)
     */
    private static void freeNativeObject(long controllerId) {
        assert controllerId != 0L;
        finalizeNative(controllerId);
    }
    // *************************************************************************
    // native private methods

    native private static long create(long ghostId);

    native private static void finalizeNative(long controllerId);

    native private static float getAngularDamping(long controllerId);

    native private static void
            getAngularVelocity(long controllerId, Vector3f storeVector);

    native private static float getFallSpeed(long controllerId);

    native private static void
            getGravity(long controllerId, Vector3f storeVector);

    native private static float getJumpSpeed(long controllerId);

    native private static float getLinearDamping(long controllerId);

    native private static void
            getLinearVelocity(long controllerId, Vector3f storeVector);

    native private static float getMaxPenetrationDepth(long controllerId);

    native private static float getMaxSlope(long controllerId);

    native private static float getStepHeight(long controllerId);

    native private static void
            getUpDirection(long controllerId, Vector3f storeVector);

    native private static void
            getWalkOffset(long controllerId, Vector3f storeVector);

    native private static boolean isUsingGhostSweepTest(long controllerId);

    native private static void jump(long controllerId, Vector3f direction);

    native private static boolean onGround(long controllerId);

    native private static void reset(long controllerId, long spaceId);

    native private static void
            setAngularDamping(long controllerId, float damping);

    native private static void
            setAngularVelocity(long controllerId, Vector3f angularVelocity);

    native private static void setFallSpeed(long controllerId, float fallSpeed);

    native private static void setGravity(long controllerId, Vector3f gravity);

    native private static void setJumpSpeed(long controllerId, float jumpSpeed);

    native private static void
            setLinearDamping(long controllerId, float damping);

    native private static void
            setLinearVelocity(long controllerId, Vector3f velocity);

    native private static void
            setMaxPenetrationDepth(long controllerId, float depth);

    native private static void
            setMaxSlope(long controllerId, float slopeRadians);

    native private static void setStepHeight(long controllerId, float height);

    native private static void setUp(long controllerId, Vector3f direction);

    native private static void
            setUseGhostSweepTest(long controllerId, boolean useGhostSweepTest);

    native private static void
            setWalkDirection(long controllerId, Vector3f direction);

    native private static void warp(long controllerId, Vector3f location);

    native private static void warpDp(long controllerId, Vec3d location);
}
