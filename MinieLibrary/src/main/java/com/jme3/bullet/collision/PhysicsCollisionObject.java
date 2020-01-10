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

import com.jme3.bounding.BoundingBox;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.infos.DebugMeshNormals;
import com.jme3.bullet.debug.DebugMeshInitListener;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.material.Material;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import com.jme3.util.clone.JmeCloneable;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * The abstract base class for collision objects based on Bullet's
 * btCollisionObject.
 * <p>
 * Subclasses include PhysicsCharacter, PhysicsRigidBody, and
 * PhysicsGhostObject.
 *
 * @author normenhansen
 */
abstract public class PhysicsCollisionObject
        implements Comparable<PhysicsCollisionObject>, JmeCloneable, Savable {
    // *************************************************************************
    // constants and loggers

    /**
     * collideWithGroups bitmask that represents "no groups"
     */
    final public static int COLLISION_GROUP_NONE = 0x0;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #1
     */
    final public static int COLLISION_GROUP_01 = 0x0001;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #2
     */
    final public static int COLLISION_GROUP_02 = 0x0002;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #3
     */
    final public static int COLLISION_GROUP_03 = 0x0004;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #4
     */
    final public static int COLLISION_GROUP_04 = 0x0008;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #5
     */
    final public static int COLLISION_GROUP_05 = 0x0010;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #6
     */
    final public static int COLLISION_GROUP_06 = 0x0020;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #7
     */
    final public static int COLLISION_GROUP_07 = 0x0040;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #8
     */
    final public static int COLLISION_GROUP_08 = 0x0080;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #9
     */
    final public static int COLLISION_GROUP_09 = 0x0100;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #10
     */
    final public static int COLLISION_GROUP_10 = 0x0200;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #11
     */
    final public static int COLLISION_GROUP_11 = 0x0400;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #12
     */
    final public static int COLLISION_GROUP_12 = 0x0800;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #13
     */
    final public static int COLLISION_GROUP_13 = 0x1000;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #14
     */
    final public static int COLLISION_GROUP_14 = 0x2000;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #15
     */
    final public static int COLLISION_GROUP_15 = 0x4000;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #16
     */
    final public static int COLLISION_GROUP_16 = 0x8000;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(PhysicsCollisionObject.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagAnisotropicFrictionComponents
            = "anisotropicFrictionComponents";
    final private static String tagAnisotropicFrictionMode
            = "anisotropicFrictionMode";
    final private static String tagCcdMotionThreshold
            = "ccdMotionThreshold";
    final private static String tagCcdSweptSphereRadius
            = "ccdSweptSphereRadius";
    final private static String tagCollisionGroup = "collisionGroup";
    final private static String tagCollisionGroupsMask = "collisionGroupsMask";
    final private static String tagCollisionShape = "collisionShape";
    final private static String tagContactDamping = "contactDamping";
    final private static String tagContactProcessingThreshold
            = "contactProcessingThreshold";
    final private static String tagContactStiffness = "contactStiffness";
    final private static String tagDebugMaterial = "debugMaterial";
    final private static String tagDebugMeshNormals = "debugMeshNormals";
    final private static String tagDebugMeshResolution = "debugMeshResolution";
    final private static String tagFriction = "friction";
    final private static String tagRestitution = "restitution";
    final private static String tagRollingFriction = "rollingFriction";
    final private static String tagSpinningFriction = "spinningFriction";
    final private static String tagUserObject = "userObject";
    // *************************************************************************
    // fields

    /**
     * shape of this object, or null if none
     */
    private CollisionShape collisionShape;
    /**
     * listener for new debug meshes, or null for none
     */
    private DebugMeshInitListener debugMeshInitListener = null;
    /**
     * which normals to include in new debug meshes (default=None)
     */
    private DebugMeshNormals debugMeshNormals = DebugMeshNormals.None;
    /**
     * collision groups with which this object can collide (default=only group
     * #1)
     */
    private int collideWithGroups = COLLISION_GROUP_01;
    /**
     * collision group to which this physics object belongs (default=group #1)
     */
    private int collisionGroup = COLLISION_GROUP_01;
    /**
     * resolution for new debug meshes (default=low, effective only for convex
     * shapes)
     */
    private int debugMeshResolution = DebugShapeFactory.lowResolution;
    /**
     * number of visible sides for default debug materials (&ge;0, &le;2,
     * default=1)
     */
    private int debugNumSides = 1;
    /**
     * Unique identifier of the btCollisionObject. Constructors are responsible
     * for setting this to a non-zero value. The ID might change if the object
     * gets rebuilt. TODO privatize and rename nativeId
     */
    protected long objectId = 0L;
    /**
     * custom material for debug shape, or null to use the default material
     */
    private Material debugMaterial = null;
    /**
     * object that uses this collision object, typically a PhysicsControl,
     * PhysicsLink, or Spatial
     */
    private Object userObject;
    // *************************************************************************
    // new methods exposed

    /**
     * Reactivate this object if it has been deactivated due to lack of motion.
     *
     * @param forceFlag true to force activation
     */
    public void activate(boolean forceFlag) {
        activate(objectId, forceFlag);
    }

    /**
     * Add collision groups to the set with which this object can collide.
     *
     * Two objects can collide only if one of them has the collisionGroup of the
     * other in its collideWithGroups set.
     *
     * @param collisionGroup groups to add (bit mask)
     */
    public void addCollideWithGroup(int collisionGroup) {
        collideWithGroups |= collisionGroup;
        if (objectId != 0L) {
            setCollideWithGroups(objectId, collideWithGroups);
        }
    }

    /**
     * Calculate an axis-aligned bounding box for this object, based on its
     * collision shape. Note: the calculated bounds are seldom minimal; they are
     * typically larger than necessary due to centering constraints and
     * collision margins.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a bounding box in physics-space coordinates (either storeResult
     * or a new instance)
     */
    public BoundingBox boundingBox(BoundingBox storeResult) {
        BoundingBox result
                = (storeResult == null) ? new BoundingBox() : storeResult;

        Vector3f translation = getPhysicsLocation(null);
        Matrix3f rotation = getPhysicsRotationMatrix(null);
        collisionShape.boundingBox(translation, rotation, result);

        return result;
    }

    /**
     * Access the listener for new debug meshes.
     *
     * @return the pre-existing instance, or null if none
     */
    public DebugMeshInitListener debugMeshInitListener() {
        return debugMeshInitListener;
    }

    /**
     * Read which normals to include in new debug meshes.
     *
     * @return an enum value (not null)
     */
    public DebugMeshNormals debugMeshNormals() {
        assert debugMeshNormals != null;
        return debugMeshNormals;
    }

    /**
     * Read mesh resolution for new debug meshes.
     *
     * @return 0=low, 1=high
     */
    public int debugMeshResolution() {
        assert debugMeshResolution >= 0 : debugMeshResolution;
        assert debugMeshResolution <= 1 : debugMeshResolution;
        return debugMeshResolution;
    }

    /**
     * Determine how many sides of this object's default debug materials are
     * visible.
     *
     * @return the number of sides (&ge;0, &le;2)
     */
    public int debugNumSides() {
        assert debugNumSides >= 0 : debugNumSides;
        assert debugNumSides <= 2 : debugNumSides;
        return debugNumSides;
    }

    /**
     * Read this object's activation state.
     *
     * @return the state (1=active tag, 2=island sleeping, 3=wants deactivation,
     * 4=disable deactivation, 5=disable simulation)
     * @see Activation
     */
    public int getActivationState() {
        int result = getActivationState(objectId);
        assert result >= Activation.firstValue : result;
        assert result <= Activation.lastValue : result;
        return result;
    }

    /**
     * Copy this object's anisotropic friction components.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the components of the friction (either storeResult or a new
     * vector, not null)
     */
    public Vector3f getAnisotropicFriction(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getAnisotropicFriction(objectId, result);
        return result;
    }

    /**
     * Read the continuous collision detection (CCD) motion threshold for this
     * object.
     *
     * @return the minimum distance per timestep to trigger CCD (in
     * physics-space units, &ge;0)
     */
    public float getCcdMotionThreshold() {
        float distance = getCcdMotionThreshold(objectId);
        assert distance >= 0f : distance;
        return distance;
    }

    /**
     * Read the squared motion threshold for this object.
     *
     * @return the minimum distance squared (in physics-space units, &ge;0)
     */
    public float getCcdSquareMotionThreshold() {
        float distance = getCcdMotionThreshold();
        float dSquared = distance * distance;

        return dSquared;
    }

    /**
     * Read the radius of the sphere used for continuous collision detection
     * (CCD).
     *
     * @return the radius (in physics-space units, &ge;0)
     */
    public float getCcdSweptSphereRadius() {
        float radius = getCcdSweptSphereRadius(objectId);
        assert radius >= 0f : radius;
        return radius;
    }

    /**
     * Read the set of collision groups with which this object can collide.
     *
     * @return bit mask
     */
    public int getCollideWithGroups() {
        return collideWithGroups;
    }

    /**
     * Read the collision group of this object.
     *
     * @return the collision group (bit mask with exactly one bit set)
     */
    public int getCollisionGroup() {
        return collisionGroup;
    }

    /**
     * Access the collision shape of this object.
     *
     * @return the pre-existing instance, or null if none
     */
    public CollisionShape getCollisionShape() {
        return collisionShape;
    }

    /**
     * Read the contact damping.
     *
     * @return the damping
     */
    public float getContactDamping() {
        float damping = getContactDamping(objectId);
        return damping;
    }

    /**
     * Read the contact processing threshold.
     *
     * @return the threshold distance (in physics-space units)
     */
    public float getContactProcessingThreshold() {
        float distance = getContactProcessingThreshold(objectId);
        return distance;
    }

    /**
     * Read the contact stiffness.
     *
     * @return the stiffness
     */
    public float getContactStiffness() {
        float stiffness = getContactStiffness(objectId);
        return stiffness;
    }

    /**
     * Read the deactivation time.
     *
     * @return the time (in seconds)
     */
    public float getDeactivationTime() {
        float time = getDeactivationTime(objectId);
        return time;
    }

    /**
     * Access the custom debug material, if specified.
     *
     * @return the pre-existing instance, or null for default materials
     */
    public Material getDebugMaterial() {
        return debugMaterial;
    }

    /**
     * Read this object's friction coefficient.
     *
     * @return the coefficient (&ge;0)
     */
    public float getFriction() {
        return getFriction(objectId);
    }

    /**
     * Read the ID of the btCollisionObject.
     *
     * @return the unique identifier (not zero)
     */
    final public long getObjectId() {
        assert objectId != 0L;
        return objectId;
    }

    /**
     * For compatibility with the jme3-bullet library.
     *
     * @return a new location vector (in physics-space coordinates, not null)
     */
    public Vector3f getPhysicsLocation() {
        return getPhysicsLocation(null);
    }

    /**
     * Copy the location of this object's center.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in physics-space coordinates, either
     * storeResult or a new vector, not null)
     */
    public Vector3f getPhysicsLocation(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getLocation(objectId, result);

        assert Vector3f.isValidVector(result);
        return result;
    }

    /**
     * Copy the orientation (rotation) of this object to a Quaternion.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a rotation Quaternion (in physics-space coordinates, either
     * storeResult or a new instance, not null)
     */
    public Quaternion getPhysicsRotation(Quaternion storeResult) {
        Quaternion result
                = (storeResult == null) ? new Quaternion() : storeResult;
        getOrientation(objectId, result);
        return result;
    }

    /**
     * Copy the orientation of this object (the basis of its local coordinate
     * system) to a 3x3 matrix.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a rotation matrix (in physics-space coordinates, either
     * storeResult or a new matrix, not null)
     */
    public Matrix3f getPhysicsRotationMatrix(Matrix3f storeResult) {
        Matrix3f result = (storeResult == null) ? new Matrix3f() : storeResult;
        getBasis(objectId, result);
        return result;
    }

    /**
     * Read this object's restitution (bounciness).
     *
     * @return restitution value
     */
    public float getRestitution() {
        return getRestitution(objectId);
    }

    /**
     * Read this object's rolling friction.
     *
     * @return friction value
     */
    public float getRollingFriction() {
        return getRollingFriction(objectId);
    }

    /**
     * Determine the scale of this object.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the scale factor for each local axis (either storeResult or a new
     * vector, not null, no negative component)
     */
    public Vector3f getScale(Vector3f storeResult) {
        Vector3f result = collisionShape.getScale(storeResult);
        return result;
    }

    /**
     * Read this object's spinning friction.
     *
     * @return friction value
     */
    public float getSpinningFriction() {
        return getSpinningFriction(objectId);
    }

    /**
     * Determine the coordinate transform of this object, including the scale of
     * its shape.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a coordinate transform (in physics-space coordinates, either
     * storeResult or a new Transform, not null)
     */
    public Transform getTransform(Transform storeResult) {
        Transform result
                = (storeResult == null) ? new Transform() : storeResult;

        getPhysicsLocation(result.getTranslation());
        getPhysicsRotation(result.getRotation());
        getScale(result.getScale());

        return result;
    }

    /**
     * Access the scene object that uses this collision object.
     *
     * @return the pre-existing instance, or null if none
     */
    public Object getUserObject() {
        return userObject;
    }

    /**
     * Test whether this object has anisotropic friction.
     *
     * @param mode the mode(s) to test for: 1=basic anisotropic friction,
     * 2=anisotropic rolling friction, 3=either one
     * @return true if (one of) the specified mode(s) is active, otherwise false
     */
    public boolean hasAnisotropicFriction(int mode) {
        Validate.inRange(mode, "mode", AfMode.basic, AfMode.either);
        boolean result = hasAnisotropicFriction(objectId, mode);
        return result;
    }

    /**
     * Test whether this object has been deactivated due to lack of motion.
     *
     * @return true if object still active, false if deactivated
     */
    public boolean isActive() {
        return isActive(objectId);
    }

    /**
     * Test whether this object responds to contact with other objects. All
     * ghost objects are non-responsive. Other types are responsive by default.
     *
     * @return true if responsive, otherwise false
     */
    final public boolean isContactResponse() {
        int flags = getCollisionFlags(objectId);
        boolean result = (flags & CollisionFlag.NO_CONTACT_RESPONSE) == 0x0;

        return result;
    }

    /**
     * Test whether this object is added to a PhysicsSpace.
     *
     * @return true&rarr;in a space, false&rarr;not in a space
     */
    final public boolean isInWorld() {
        return isInWorld(objectId);
    }

    /**
     * Test whether this object is static (immobile).
     *
     * @return true if static, otherwise false
     */
    final public boolean isStatic() {
        int flags = getCollisionFlags(objectId);
        boolean result = (flags & CollisionFlag.STATIC_OBJECT) != 0x0;

        return result;
    }

    /**
     * Remove collision groups from the set with which this object can collide.
     *
     * @param collisionGroup groups to remove, ORed together (bit mask)
     */
    public void removeCollideWithGroup(int collisionGroup) {
        collideWithGroups &= ~collisionGroup;
        if (objectId != 0L) {
            setCollideWithGroups(collideWithGroups);
        }
    }

    /**
     * Alter this object's anisotropic friction.
     *
     * @param components the desired friction components (not null, unaffected,
     * default=(1,1,1))
     * @param mode the desired friction mode: 0=isotropic, 1=basic anisotropic
     * friction, 2=anisotropic rolling friction (default=0)
     */
    public void setAnisotropicFriction(Vector3f components, int mode) {
        Validate.nonNull(components, "components");
        Validate.inRange(mode, "mode", AfMode.none, AfMode.rolling);

        setAnisotropicFriction(objectId, components, mode);
    }

    /**
     * Alter the amount of motion required to trigger continuous collision
     * detection (CCD).
     * <p>
     * This addresses the issue of fast objects passing through other objects
     * with no collision detected.
     *
     * @param threshold the desired minimum distance per timestep to trigger CCD
     * (in physics-space units, &gt;0) or zero to disable CCD (default=0)
     */
    public void setCcdMotionThreshold(float threshold) {
        setCcdMotionThreshold(objectId, threshold);
    }

    /**
     * Alter the continuous collision detection (CCD) swept-sphere radius for
     * this object.
     *
     * @param radius (in physics-space units, &ge;0, default=0)
     */
    public void setCcdSweptSphereRadius(float radius) {
        setCcdSweptSphereRadius(objectId, radius);
    }

    /**
     * Directly alter the collision groups with which this object can collide.
     *
     * @param collisionGroups desired groups, ORed together (bit mask,
     * default=COLLISION_GROUP_01)
     */
    public void setCollideWithGroups(int collisionGroups) {
        collideWithGroups = collisionGroups;
        if (objectId != 0L) {
            setCollideWithGroups(objectId, collideWithGroups);
        }
    }

    /**
     * Alter which collision group this object belongs to.
     * <p>
     * Groups are represented by integer bit masks with exactly 1 bit set.
     * Pre-made variables are available in PhysicsCollisionObject.
     * <p>
     * Two objects can collide only if one of them has the collisionGroup of the
     * other in its collideWithGroups set.
     *
     * @param collisionGroup the collisionGroup to apply (bit mask with exactly
     * 1 bit set, default=COLLISION_GROUP_01)
     */
    public void setCollisionGroup(int collisionGroup) {
        assert Integer.bitCount(collisionGroup) == 1 : collisionGroup;

        this.collisionGroup = collisionGroup;
        if (objectId != 0L) {
            setCollisionGroup(objectId, collisionGroup);
        }
    }

    /**
     * Apply the specified CollisionShape to this object.
     *
     * @param collisionShape the shape to apply (not null, alias created)
     */
    public void setCollisionShape(CollisionShape collisionShape) {
        Validate.nonNull(collisionShape, "collision shape");
        this.collisionShape = collisionShape;
    }

    /**
     * Alter the contact damping.
     *
     * @param damping the desired damping (default=0.1)
     */
    public void setContactDamping(float damping) {
        float stiffness = getContactStiffness(objectId);
        setContactStiffnessAndDamping(objectId, stiffness, damping);
    }

    /**
     * Alter the contact-processing threshold.
     *
     * @param distance the desired threshold distance (in physics-space units,
     * default=1e18 with SP library or 1e30 with DP library)
     */
    public void setContactProcessingThreshold(float distance) {
        setContactProcessingThreshold(objectId, distance);
    }

    /**
     * Alter the contact stiffness.
     *
     * @param stiffness the desired stiffness (default=1e18 with SP library or
     * 1e30 with DP library)
     */
    public void setContactStiffness(float stiffness) {
        float damping = getContactDamping(objectId);
        setContactStiffnessAndDamping(objectId, stiffness, damping);
    }

    /**
     * Alter the deactivation time.
     *
     * @param time the desired time (in seconds, default=0)
     */
    public void setDeactivationTime(float time) {
        setDeactivationTime(objectId, time);
    }

    /**
     * Alter or remove the custom debug material.
     *
     * @param material the desired material, or null to use the default debug
     * materials (alias created, default=null)
     */
    public void setDebugMaterial(Material material) {
        debugMaterial = material;
    }

    /**
     * Alter the listener for new debug meshes.
     *
     * @param listener the desired listener, or null for none (default=null)
     */
    public void setDebugMeshInitListener(DebugMeshInitListener listener) {
        debugMeshInitListener = listener;
    }

    /**
     * Alter which normals to include in new debug meshes.
     *
     * @param newSetting an enum value (not null, default=None)
     */
    public void setDebugMeshNormals(DebugMeshNormals newSetting) {
        Validate.nonNull(newSetting, "new setting");
        debugMeshNormals = newSetting;
    }

    /**
     * Alter the mesh resolution for new debug meshes. Effective only for
     * objects with convex shapes.
     *
     * @see com.jme3.bullet.collision.shapes.CollisionShape#isConvex()
     *
     * @param newSetting 0=low, 1=high (default=0)
     */
    public void setDebugMeshResolution(int newSetting) {
        Validate.inRange(newSetting, "new setting",
                DebugShapeFactory.lowResolution,
                DebugShapeFactory.highResolution);
        debugMeshResolution = newSetting;
    }

    /**
     * Alter how many sides of this object's default debug materials are
     * visible. This setting has no effect on custom debug materials.
     *
     * @param numSides the desired number of sides (&ge;0, &le;2, default=1)
     */
    public void setDebugNumSides(int numSides) {
        Validate.inRange(numSides, "number of sides", 0, 2);
        debugNumSides = numSides;
    }

    /**
     * Alter this object's friction.
     *
     * @param friction the desired friction coefficient (&ge;0, default=0.5)
     */
    public void setFriction(float friction) {
        Validate.nonNegative(friction, "friction");
        setFriction(objectId, friction);
    }

    /**
     * Alter this object's restitution (bounciness). For best performance, keep
     * restitution=0.
     *
     * @param restitution the desired value (default=0)
     */
    public void setRestitution(float restitution) {
        setRestitution(objectId, restitution);
    }

    /**
     * Alter this object's rolling friction: torsional friction orthogonal to
     * the contact normal. Use to stop objects from rolling.
     *
     * @param friction the desired friction value (default=0)
     */
    public void setRollingFriction(float friction) {
        setRollingFriction(objectId, friction);
    }

    /**
     * Alter this object's spinning friction: torsional friction around the
     * contact normal. Use for grasping.
     *
     * @param friction the desired friction value (default=0)
     */
    public void setSpinningFriction(float friction) {
        setSpinningFriction(objectId, friction);
    }

    /**
     * Alter which scene object uses this collision object.
     *
     * @param userObject the desired scene object (alias created, default=null)
     */
    public void setUserObject(Object userObject) {
        this.userObject = userObject;
    }
    // *************************************************************************
    // new protected methods

    /**
     * Attach the identified btCollisionShape to the identified
     * btCollisionObject. Native method.
     *
     * @param objectId the identifier of the btCollisionObject (not zero)
     * @param collisionShapeId the identifier of the btCollisionShape (not zero)
     */
    native protected void attachCollisionShape(long objectId,
            long collisionShapeId);

    /**
     * Copy common properties from another PhysicsCollisionObject. Used during
     * cloning.
     *
     * @param old (not null, unaffected)
     */
    final protected void copyPcoProperties(PhysicsCollisionObject old) {
        assert old.objectId != 0L;

        setCcdMotionThreshold(old.getCcdMotionThreshold());
        setCcdSweptSphereRadius(old.getCcdSweptSphereRadius());
        setContactDamping(old.getContactDamping());
        setContactProcessingThreshold(old.getContactProcessingThreshold());
        setContactStiffness(old.getContactStiffness());
        setFriction(old.getFriction());
        setRestitution(old.getRestitution());
        setRollingFriction(old.getRollingFriction());
        setSpinningFriction(old.getSpinningFriction());

        if (old.hasAnisotropicFriction(AfMode.basic)) {
            setAnisotropicFriction(old.getAnisotropicFriction(null),
                    AfMode.basic);
        } else if (old.hasAnisotropicFriction(AfMode.rolling)) {
            setAnisotropicFriction(old.getAnisotropicFriction(null),
                    AfMode.rolling);
        }
    }

    /**
     * Finalize the identified btCollisionObject. Native method.
     *
     * @param objectId the ID of the btCollisionObject (not zero)
     */
    native protected void finalizeNative(long objectId);

    /**
     * Read the collision flags of this object. Subclasses are responsible for
     * cloning/loading/saving these flags. Flags are defined in
     * {@link com.jme3.bullet.collision.CollisionFlag}.
     *
     * @param objectId the ID of the btCollisionObject (not zero)
     * @return the flags that are set, ORed together
     */
    native protected int getCollisionFlags(long objectId);

    /**
     * Read the type of this object.
     *
     * @param objectId the ID of the btCollisionObject (not zero)
     * @return the type value
     */
    final native protected int getInternalType(long objectId);

    /**
     * Initialize the collision-group information of this object.
     */
    protected void initUserPointer() {
        logger.log(Level.FINE, "initUserPointer() for {0}", this);
        initUserPointer(objectId, collisionGroup, collideWithGroups);
    }

    /**
     * Read common properties from a capsule.
     *
     * @param capsule the input capsule (not null, modified)
     * @throws IOException from the importer
     */
    final protected void readPcoProperties(InputCapsule capsule)
            throws IOException {
        setCcdMotionThreshold(capsule.readFloat(tagCcdMotionThreshold, 0f));
        setCcdSweptSphereRadius(
                capsule.readFloat(tagCcdSweptSphereRadius, 0f));
        setContactDamping(capsule.readFloat(tagContactDamping, 0.1f));
        setContactProcessingThreshold(
                capsule.readFloat(tagContactProcessingThreshold, 0f));
        setContactStiffness(capsule.readFloat(tagContactStiffness, 1e30f));
        setFriction(capsule.readFloat(tagFriction, 0.5f));
        setRestitution(capsule.readFloat(tagRestitution, 0f));
        setRollingFriction(capsule.readFloat(tagRollingFriction, 0f));
        setSpinningFriction(capsule.readFloat(tagSpinningFriction, 0f));

        int mode = capsule.readInt(tagAnisotropicFrictionMode, AfMode.none);
        if (mode != AfMode.none) {
            Vector3f components = (Vector3f) capsule.readSavable(
                    tagAnisotropicFrictionComponents, new Vector3f(1f, 1f, 1f));
            setAnisotropicFriction(components, mode);
        }

        userObject = capsule.readSavable(tagUserObject, null);
    }

    /**
     * Alter the activation state of this object.
     *
     * @param objectId the ID of the btCollisionObject (not zero)
     * @param desiredState the desired state
     */
    native protected void setActivationState(long objectId, int desiredState);

    /**
     * Alter the collision flags of this object. Subclasses are responsible for
     * cloning/loading/saving these flags. Flags are defined in
     * {@link com.jme3.bullet.collision.CollisionFlag}.
     *
     * @param objectId the ID of the btCollisionObject (not zero)
     * @param desiredFlags the desired collision flags, ORed together
     */
    native protected void setCollisionFlags(long objectId, int desiredFlags);

    /**
     * Directly alter this object's location and basis.
     *
     * @param centerLocation the desired location for this object's center (in
     * physics-space coordinates, not null, unaffected)
     * @param orientation the desired orientation for this object (rotation
     * matrix in physics-space coordinates, not null, unaffected)
     */
    protected void setLocationAndBasis(Vector3f centerLocation,
            Matrix3f orientation) {
        Validate.finite(centerLocation, "center location");
        Validate.nonNull(orientation, "orientation");

        setLocationAndBasis(objectId, centerLocation, orientation);
    }
    // *************************************************************************
    // Comparable methods

    /**
     * Compare (by ID) with another collision object.
     *
     * @param other (not null, unaffected)
     * @return 0 if the objects have the same ID; negative if this comes before
     * other; positive if this comes after other
     */
    @Override
    public int compareTo(PhysicsCollisionObject other) {
        long otherId = other.getObjectId();
        int result = Long.compare(objectId, otherId);

        return result;
    }
    // *************************************************************************
    // JmeCloneable methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned object into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this object (not null)
     * @param original the instance from which this object was shallow-cloned
     * (unused)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        if (userObject instanceof Cloneable
                || userObject instanceof JmeCloneable) {
            userObject = cloner.clone(userObject);
        }

        collisionShape = cloner.clone(collisionShape);
        debugMaterial = cloner.clone(debugMaterial);

        objectId = 0L;
        /*
         * The subclass should create the btCollisionObject and then
         * invoke copyPcoProperties() .
         */
    }

    /**
     * Create a shallow clone for the JME cloner. Note that the cloned object
     * won't be added to any PhysicsSpace, even if the original was.
     *
     * @return a new instance
     */
    @Override
    public PhysicsCollisionObject jmeClone() {
        try {
            PhysicsCollisionObject clone
                    = (PhysicsCollisionObject) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }
    // *************************************************************************
    // Savable methods

    /**
     * De-serialize this object from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        InputCapsule capsule = importer.getCapsule(this);

        collisionGroup = capsule.readInt(tagCollisionGroup, COLLISION_GROUP_01);
        collideWithGroups = capsule.readInt(tagCollisionGroupsMask,
                COLLISION_GROUP_01);
        debugMeshNormals = capsule.readEnum(tagDebugMeshNormals,
                DebugMeshNormals.class, DebugMeshNormals.None);
        debugMeshResolution = capsule.readInt(tagDebugMeshResolution, 0);
        debugMaterial = (Material) capsule.readSavable(tagDebugMaterial, null);

        Savable shape = capsule.readSavable(tagCollisionShape, null);
        collisionShape = (CollisionShape) shape;
        /*
         * The subclass should create the btCollisionObject and then
         * invoke readPcoProperties() .
         */
    }

    /**
     * Serialize this object to the specified exporter, for example when saving
     * to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(collisionGroup, tagCollisionGroup, COLLISION_GROUP_01);
        capsule.write(collideWithGroups, tagCollisionGroupsMask,
                COLLISION_GROUP_01);
        capsule.write(debugMeshNormals, tagDebugMeshNormals,
                DebugMeshNormals.None);
        capsule.write(debugMeshResolution, tagDebugMeshResolution, 0);
        capsule.write(debugMaterial, tagDebugMaterial, null);
        capsule.write(collisionShape, tagCollisionShape, null);

        if (userObject instanceof Savable) {
            capsule.write((Savable) userObject, "userObject", null);
        }

        // common properties
        capsule.write(getCcdMotionThreshold(), tagCcdMotionThreshold, 0f);
        capsule.write(getCcdSweptSphereRadius(), tagCcdSweptSphereRadius, 0f);
        capsule.write(getContactDamping(), tagContactDamping, 0.1f);
        capsule.write(getContactProcessingThreshold(),
                tagContactProcessingThreshold, 0f);
        capsule.write(getContactStiffness(), tagContactStiffness, 1e30f);
        capsule.write(getFriction(), tagFriction, 0.5f);
        capsule.write(getRestitution(), tagRestitution, 0f);
        capsule.write(getRollingFriction(), tagRollingFriction, 0f);
        capsule.write(getSpinningFriction(), tagSpinningFriction, 0f);

        int mode = AfMode.none;
        if (hasAnisotropicFriction(AfMode.basic)) {
            mode = AfMode.basic;
        } else if (hasAnisotropicFriction(AfMode.rolling)) {
            mode = AfMode.rolling;
        }
        capsule.write(mode, tagAnisotropicFrictionMode, AfMode.none);
        if (mode != AfMode.none) {
            Vector3f components = getAnisotropicFriction(null);
            capsule.write(components, tagAnisotropicFrictionComponents, null);
        }
    }
    // *************************************************************************
    // Object methods

    /**
     * Test for ID equality.
     *
     * @param otherObject (may be null)
     * @return true if the collision objects have the same ID, otherwise false
     */
    @Override
    public boolean equals(Object otherObject) {
        boolean result;
        if (this == otherObject) {
            result = true;
        } else if (otherObject instanceof PhysicsCollisionObject) {
            PhysicsCollisionObject other = (PhysicsCollisionObject) otherObject;
            long otherId = other.getObjectId();
            result = (objectId == otherId);
        } else {
            result = false;
        }

        return result;
    }

    /**
     * Finalize this collision object just before it is destroyed. Should be
     * invoked only by a subclass or by the garbage collector.
     *
     * @throws Throwable ignored by the garbage collector
     */
    @Override
    protected void finalize() throws Throwable {
        super.finalize();
        logger.log(Level.FINE, "Finalizing {0}.", this);
        finalizeNative(objectId);
    }

    /**
     * Generate the hash code for this object.
     *
     * @return value for use in hashing
     */
    @Override
    public int hashCode() {
        int hash = (int) (objectId >> 4);
        return hash;
    }

    /**
     * Represent this collision object as a String.
     *
     * @return a descriptive string of text (not null, not empty)
     */
    @Override
    public String toString() {
        String result = getClass().getSimpleName();
        result = result.replace("Control", "C");
        result = result.replace("Physics", "");
        result = result.replace("Object", "");
        result += "#" + Long.toHexString(objectId);

        return result;
    }
    // *************************************************************************
    // native private methods

    native private void activate(long objectId, boolean forceFlag);

    native private int getActivationState(long objectId);

    native private void getAnisotropicFriction(long objectId,
            Vector3f storeResult);

    native private void getBasis(long objectId, Matrix3f storeResult);

    native private float getCcdMotionThreshold(long objectId);

    native private float getCcdSweptSphereRadius(long objectId);

    native private float getContactDamping(long objectId);

    native private float getContactProcessingThreshold(long objectId);

    native private float getContactStiffness(long objectId);

    native private float getDeactivationTime(long objectId);

    native private float getFriction(long objectId);

    native private void getLocation(long objectId, Vector3f storeResult);

    native private void getOrientation(long objectId, Quaternion storeResult);

    native private float getRestitution(long objectId);

    native private float getRollingFriction(long objectId);

    native private float getSpinningFriction(long objectId);

    native private boolean hasAnisotropicFriction(long objectId, int mode);

    native private void initUserPointer(long objectId, int group, int groups);

    native private boolean isActive(long objectId);

    native private boolean isInWorld(long objectId);

    native private void setAnisotropicFriction(long objectId,
            Vector3f components, int mode);

    native private void setCcdMotionThreshold(long objectId, float threshold);

    native private void setCcdSweptSphereRadius(long objectId, float radius);

    native private void setCollideWithGroups(long objectId,
            int collisionGroups);

    native private void setCollisionGroup(long objectId, int collisionGroup);

    native private void setContactProcessingThreshold(long objectId,
            float thresholdDistance);

    native private void setContactStiffnessAndDamping(long objectId,
            float stiffness, float damping);

    native private void setDeactivationTime(long objectId, float time);

    native private void setFriction(long objectId, float friction);

    native private void setLocationAndBasis(long objectId, Vector3f location,
            Matrix3f basis);

    native private void setRestitution(long objectId, float restitution);

    native private void setRollingFriction(long objectId, float friction);

    native private void setSpinningFriction(long objectId, float friction);
}
