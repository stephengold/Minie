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
import com.jme3.bullet.CollisionSpace;
import com.jme3.bullet.NativePhysicsObject;
import com.jme3.bullet.collision.shapes.CollisionShape;
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
import com.simsilica.mathd.Matrix3d;
import com.simsilica.mathd.Quatd;
import com.simsilica.mathd.Vec3d;
import java.io.IOException;
import java.util.Collection;
import java.util.TreeSet;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.MeshNormals;
import jme3utilities.Validate;

/**
 * The abstract base class for collision objects based on Bullet's
 * {@code btCollisionObject}.
 * <p>
 * Subclasses include MultiBodyCollider, PhysicsBody, PhysicsCharacter, and
 * PhysicsGhostObject.
 *
 * @author normenhansen
 */
abstract public class PhysicsCollisionObject
        extends NativePhysicsObject
        implements JmeCloneable, Savable {
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
    final private static String tagApplicationData
            = "applicationData";
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
    final private static String tagDeactivationTime = "deactivationTime";
    final private static String tagDebugMaterial = "debugMaterial";
    final private static String tagDebugMeshNormals = "debugMeshNormals";
    final private static String tagDebugMeshResolution = "debugMeshResolution";
    final private static String tagHasCsd = "hasCsd";
    final private static String tagFriction = "friction";
    final private static String tagIgnoreList = "ignoreList";
    final private static String tagRestitution = "restitution";
    final private static String tagRollingFriction = "rollingFriction";
    final private static String tagSpinningFriction = "spinningFriction";
    final private static String tagUserObject = "userObject";
    // *************************************************************************
    // fields

    /**
     * make cloneFields() progress visible
     */
    private boolean doneCloningIgnores = false;
    /**
     * copy of the list of specific objects with which collisions are ignored,
     * or null for none
     */
    private Collection<PhysicsCollisionObject> ignoreList;
    /**
     * shape of this object, or null if none
     */
    private CollisionShape collisionShape;
    /**
     * listener for new debug meshes, or null for none
     */
    private DebugMeshInitListener debugMeshInitListener = null;
    /**
     * collision groups with which this object can collide
     */
    private int collideWithGroups = COLLISION_GROUP_01;
    /**
     * collision group to which this object belongs
     */
    private int collisionGroup = COLLISION_GROUP_01;
    /**
     * resolution for new debug meshes (effective only for objects with convex
     * shapes)
     */
    private int debugMeshResolution = DebugShapeFactory.lowResolution;
    /**
     * number of visible sides for default debug materials (&ge;0, &le;2)
     */
    private int debugNumSides = 1;
    /**
     * custom material for the debug shape, or null to use the default material
     */
    private Material debugMaterial = null;
    /**
     * which normals to generate for new debug meshes
     */
    private MeshNormals debugMeshNormals = MeshNormals.None;
    /**
     * application-specific data of this collision object. Untouched unless
     * Cloneable or Savable.
     */
    private Object applicationData = null;
    /**
     * scene object that's using this collision object. The scene object is
     * typically a PhysicsControl, PhysicsLink, or Spatial. Used by physics
     * controls.
     */
    private Object userObject = null;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a collision object with no tracker and no assigned native
     * object.
     */
    protected PhysicsCollisionObject() { // avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Reactivate this object if it has been deactivated due to lack of motion.
     * <p>
     * Deactivation doesn't affect a PhysicsCharacter or PhysicsGhostObject.
     *
     * @param forceFlag true to force activation
     */
    public void activate(boolean forceFlag) {
        long objectId = nativeId();
        activate(objectId, forceFlag);
    }

    /**
     * Add collision groups to the set with which this object can collide.
     * <p>
     * Two objects can collide only if one of them has the collisionGroup of the
     * other in its collideWithGroups set.
     *
     * @param collisionGroup the groups to add, ORed together (bitmask)
     */
    public void addCollideWithGroup(int collisionGroup) {
        this.collideWithGroups |= collisionGroup;
        if (hasAssignedNativeObject()) {
            long objectId = nativeId();
            setCollideWithGroups(objectId, collideWithGroups);
        }
    }

    /**
     * Add another collision object to this object's ignore list and vice versa.
     * <p>
     * Any collisions with objects on the list will be ignored. However, the
     * wheels of a PhysicsVehicle aren't collision objects, so the vehicle's
     * ignore list doesn't affect them.
     *
     * @param otherPco the other collision object (not null, not {@code this},
     * modified)
     */
    public void addToIgnoreList(PhysicsCollisionObject otherPco) {
        Validate.nonNull(otherPco, "other collision object");
        Validate.require(otherPco != this, "2 distinct collision objects");

        if (ignoreList == null) {
            this.ignoreList = new TreeSet<>();
        }
        if (!ignoreList.contains(otherPco)) {
            ignoreList.add(otherPco);

            if (otherPco.ignoreList == null) {
                otherPco.ignoreList = new TreeSet<>();
            }
            assert !otherPco.ignoreList.contains(this);
            otherPco.ignoreList.add(this);

            long thisId = nativeId();
            long otherId = otherPco.nativeId();
            boolean toIgnore = true;
            setIgnoreCollisionCheck(thisId, otherId, toIgnore);
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
     * Clear this object's ignore list.
     */
    public void clearIgnoreList() {
        long thisId = nativeId();
        if (ignoreList != null && !ignoreList.isEmpty()) {
            boolean toIgnore = false;
            for (PhysicsCollisionObject otherPco : ignoreList) {
                long otherId = otherPco.nativeId();
                setIgnoreCollisionCheck(thisId, otherId, toIgnore);

                assert otherPco.ignoreList.contains(this);
                otherPco.ignoreList.remove(this);
            }
            ignoreList.clear();
        }
    }

    /**
     * Return the collision flags. Flag values are defined in
     * {@link com.jme3.bullet.collision.CollisionFlag}.
     *
     * @return the values of all flags that are set, ORed together
     */
    public int collisionFlags() {
        long objectId = nativeId();
        int result = getCollisionFlags(objectId);

        return result;
    }

    /**
     * Copy common properties from another collision object. Used during
     * cloning.
     *
     * @param old (not null, unaffected)
     */
    final public void copyPcoProperties(PhysicsCollisionObject old) {
        assert old.hasAssignedNativeObject();
        assert old.nativeId() != nativeId();

        int flags = old.collisionFlags();
        boolean hasCsd
                = (flags & CollisionFlag.HAS_CONTACT_STIFFNESS_DAMPING) != 0;

        setCcdMotionThreshold(old.getCcdMotionThreshold());
        setCcdSweptSphereRadius(old.getCcdSweptSphereRadius());
        if (hasCsd) {
            setContactDamping(old.getContactDamping());
        }
        setContactProcessingThreshold(old.getContactProcessingThreshold());
        if (hasCsd) {
            setContactStiffness(old.getContactStiffness());
        }
        setDeactivationTime(old.getDeactivationTime());
        setFriction(old.getFriction());
        setRestitution(old.getRestitution());
        setRollingFriction(old.getRollingFriction());
        setSpinningFriction(old.getSpinningFriction());

        if (old.hasAnisotropicFriction(AfMode.basic)) {
            setAnisotropicFriction(
                    old.getAnisotropicFriction(null), AfMode.basic);
        } else if (old.hasAnisotropicFriction(AfMode.rolling)) {
            setAnisotropicFriction(
                    old.getAnisotropicFriction(null), AfMode.rolling);
        }
    }

    /**
     * Count the collision objects in this object's ignore list.
     *
     * @return the count (&ge;0)
     * @see #addToIgnoreList(com.jme3.bullet.collision.PhysicsCollisionObject)
     */
    public int countIgnored() {
        int result = (ignoreList == null) ? 0 : ignoreList.size();

        assert result == getNumObjectsWithoutCollision(nativeId()) :
                result + " != " + getNumObjectsWithoutCollision(nativeId());
        assert result >= 0 : result;
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
     * Determine which normals to include in new debug meshes.
     *
     * @return an enum value (not null)
     */
    public MeshNormals debugMeshNormals() {
        assert debugMeshNormals != null;
        return debugMeshNormals;
    }

    /**
     * Return the resolution for new debug meshes. Effective only for objects
     * with convex shapes.
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
     * Find the collision object with the specified ID. Native method.
     *
     * @param pcoId the native identifier (not zero)
     * @return the pre-existing instance, or null if it's been reclaimed
     */
    native public static PhysicsCollisionObject findInstance(long pcoId);

    /**
     * Return this object's activation state (native field: m_activationState1).
     * <p>
     * Deactivation doesn't affect a PhysicsCharacter or PhysicsGhostObject.
     *
     * @return the state (1=active tag, 2=island sleeping, 3=wants deactivation,
     * 4=disable deactivation, 5=disable simulation)
     * @see Activation
     */
    public int getActivationState() {
        long objectId = nativeId();
        int result = getActivationState(objectId);

        assert result >= Activation.firstValue : result;
        assert result <= Activation.lastValue : result;
        return result;
    }

    /**
     * Copy this object's anisotropic friction components (native field:
     * m_anisotropicFriction).
     * <p>
     * Friction doesn't affect a PhysicsCharacter or PhysicsGhostObject.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the components of the friction (either storeResult or a new
     * vector, not null)
     */
    public Vector3f getAnisotropicFriction(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long objectId = nativeId();
        getAnisotropicFriction(objectId, result);

        return result;
    }

    /**
     * Access any application-specific data associated with this collision
     * object.
     *
     * @return the pre-existing instance, or null if none
     * @see #setApplicationData(java.lang.Object)
     */
    public Object getApplicationData() {
        return applicationData;
    }

    /**
     * Return the continuous collision detection (CCD) motion threshold (native
     * field: m_ccdMotionThreshold).
     * <p>
     * CCD doesn't affect a PhysicsCharacter or PhysicsGhostObject.
     *
     * @return the minimum distance per timestep to trigger CCD (in
     * physics-space units, &ge;0)
     */
    public float getCcdMotionThreshold() {
        long objectId = nativeId();
        float distance = getCcdMotionThreshold(objectId);

        assert distance >= 0f : distance;
        return distance;
    }

    /**
     * Return the squared motion threshold.
     * <p>
     * CCD doesn't affect a PhysicsCharacter or PhysicsGhostObject.
     *
     * @return the minimum distance squared (in physics-space units, &ge;0)
     */
    public float getCcdSquareMotionThreshold() {
        float distance = getCcdMotionThreshold();
        float dSquared = distance * distance;

        return dSquared;
    }

    /**
     * Return the radius of the sphere used for continuous collision detection
     * (CCD) (native field: m_ccdSweptSphereRadius).
     * <p>
     * CCD doesn't affect a PhysicsCharacter or PhysicsGhostObject.
     *
     * @return the radius (in physics-space units, &ge;0)
     */
    public float getCcdSweptSphereRadius() {
        long objectId = nativeId();
        float radius = getCcdSweptSphereRadius(objectId);

        assert radius >= 0f : radius;
        return radius;
    }

    /**
     * Return the set of collision groups with which this object can collide.
     *
     * @return the bitmask
     */
    public int getCollideWithGroups() {
        assert collideWithGroups == getCollideWithGroups(nativeId());
        return collideWithGroups;
    }

    /**
     * Return the collision group of this object.
     *
     * @return the collision group (bitmask with exactly one bit set)
     */
    public int getCollisionGroup() {
        assert collisionGroup == getCollisionGroup(nativeId());
        assert Integer.bitCount(collisionGroup) == 1 : collisionGroup;
        return collisionGroup;
    }

    /**
     * Access the shape of this object.
     *
     * @return the pre-existing instance, or null if none
     */
    public CollisionShape getCollisionShape() {
        return collisionShape;
    }

    /**
     * Access the space where this object is added.
     *
     * @return the pre-existing instance, or null if none
     */
    public CollisionSpace getCollisionSpace() {
        CollisionSpace result = null;
        if (spaceId() != 0L) {
            long objectId = nativeId();
            result = getCollisionSpace(objectId);
        }

        return result;
    }

    /**
     * Return the contact damping (native field: m_contactDamping).
     * <p>
     * Contact damping doesn't affect a PhysicsCharacter or PhysicsGhostObject.
     *
     * @return the damping
     */
    public float getContactDamping() {
        long objectId = nativeId();
        float damping = getContactDamping(objectId);

        return damping;
    }

    /**
     * Return the contact-processing threshold (native field:
     * m_contactProcessingThreshold).
     * <p>
     * Contact processing doesn't affect a PhysicsCharacter or
     * PhysicsGhostObject.
     *
     * @return the threshold distance (in physics-space units)
     */
    public float getContactProcessingThreshold() {
        long objectId = nativeId();
        float distance = getContactProcessingThreshold(objectId);

        return distance;
    }

    /**
     * Return the contact stiffness (native field: m_contactStiffness).
     * <p>
     * Contact stiffness doesn't affect a PhysicsCharacter or
     * PhysicsGhostObject.
     *
     * @return the stiffness
     */
    public float getContactStiffness() {
        long objectId = nativeId();
        float stiffness = getContactStiffness(objectId);

        return stiffness;
    }

    /**
     * Return the deactivation time (native field: m_deactivationTime).
     * <p>
     * Deactivation doesn't affect a PhysicsCharacter or PhysicsGhostObject.
     *
     * @return the time (in seconds)
     */
    public float getDeactivationTime() {
        long objectId = nativeId();
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
     * Return this object's friction parameter (native field: m_friction).
     * <p>
     * Friction doesn't affect a PhysicsCharacter or PhysicsGhostObject.
     *
     * @return the parameter value (&ge;0)
     */
    public float getFriction() {
        long objectId = nativeId();
        float result = getFriction(objectId);

        return result;
    }

    /**
     * Return the ID of the native object ({@code btCollisionObject}).
     *
     * @return the native identifier (not zero)
     * @deprecated use {@link NativePhysicsObject#nativeId()}
     */
    @Deprecated
    final public long getObjectId() {
        return nativeId();
    }

    /**
     * For compatibility with the jme3-jbullet library.
     *
     * @return a new location vector (in physics-space coordinates, not null)
     */
    public Vector3f getPhysicsLocation() {
        return getPhysicsLocation(null);
    }

    /**
     * Copy the location of this object's center to a Vector3f.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in physics-space coordinates, either
     * storeResult or a new vector, finite)
     */
    public Vector3f getPhysicsLocation(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long objectId = nativeId();
        getLocation(objectId, result);

        assert Vector3f.isValidVector(result);
        return result;
    }

    /**
     * Copy the location of this object's center to a Vec3d.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in physics-space coordinates, either
     * storeResult or a new vector, not null, finite)
     */
    public Vec3d getPhysicsLocationDp(Vec3d storeResult) {
        Vec3d result = (storeResult == null) ? new Vec3d() : storeResult;

        long objectId = nativeId();
        getLocationDp(objectId, result);

        assert result.isFinite() : result;
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

        long objectId = nativeId();
        getOrientation(objectId, result);

        return result;
    }

    /**
     * Copy the orientation (rotation) of this object to a Quatd.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a rotation Quatd (in physics-space coordinates, either
     * storeResult or a new instance, not null)
     */
    public Quatd getPhysicsRotationDp(Quatd storeResult) {
        Quatd result = (storeResult == null) ? new Quatd() : storeResult;

        long objectId = nativeId();
        getOrientationDp(objectId, result);

        return result;
    }

    /**
     * Copy the orientation of this object (the basis of its local coordinate
     * system) to a Matrix3f.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a rotation matrix (in physics-space coordinates, either
     * storeResult or a new matrix, not null)
     */
    public Matrix3f getPhysicsRotationMatrix(Matrix3f storeResult) {
        Matrix3f result = (storeResult == null) ? new Matrix3f() : storeResult;

        long objectId = nativeId();
        getBasis(objectId, result);

        return result;
    }

    /**
     * Copy the orientation of this object (the basis of its local coordinate
     * system) to a Matrix3d.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a rotation matrix (in physics-space coordinates, either
     * storeResult or a new matrix, not null)
     */
    public Matrix3d getPhysicsRotationMatrixDp(Matrix3d storeResult) {
        Matrix3d result = (storeResult == null) ? new Matrix3d() : storeResult;

        long objectId = nativeId();
        getBasisDp(objectId, result);

        return result;
    }

    /**
     * Return this object's restitution (bounciness) (native field:
     * m_restitution).
     * <p>
     * Restitution doesn't affect a PhysicsCharacter or PhysicsGhostObject.
     *
     * @return restitution value
     */
    public float getRestitution() {
        long objectId = nativeId();
        float result = getRestitution(objectId);

        return result;
    }

    /**
     * Return this object's rolling friction (native field: m_rollingFriction).
     * <p>
     * Friction doesn't affect a PhysicsCharacter or PhysicsGhostObject.
     *
     * @return friction value
     */
    public float getRollingFriction() {
        long objectId = nativeId();
        float result = getRollingFriction(objectId);

        return result;
    }

    /**
     * Copy the scale of this object.
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
     * Return this object's spinning friction (native field:
     * m_spinningFriction).
     * <p>
     * Friction doesn't affect a PhysicsCharacter or PhysicsGhostObject.
     *
     * @return friction value
     */
    public float getSpinningFriction() {
        long objectId = nativeId();
        float result = getSpinningFriction(objectId);

        return result;
    }

    /**
     * Copy the coordinate transform of this object, including the scale of its
     * shape.
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
     * Access the scene object that's using this collision object, typically a
     * PhysicsControl, PhysicsLink, or Spatial. Used by physics controls.
     *
     * @return the pre-existing instance, or null if none
     * @see #getApplicationData()
     * @see #setUserObject(java.lang.Object)
     */
    public Object getUserObject() {
        return userObject;
    }

    /**
     * Test whether this object has anisotropic friction.
     * <p>
     * Friction doesn't affect a PhysicsCharacter or PhysicsGhostObject.
     *
     * @param mode the mode(s) to test for: 1=basic anisotropic friction,
     * 2=anisotropic rolling friction, 3=either one
     * @return true if (one of) the specified mode(s) is active, otherwise false
     */
    public boolean hasAnisotropicFriction(int mode) {
        Validate.inRange(mode, "mode", AfMode.basic, AfMode.either);

        long objectId = nativeId();
        boolean result = hasAnisotropicFriction(objectId, mode);

        return result;
    }

    /**
     * Test whether the specified collision object is in this object's ignore
     * list.
     *
     * @param other the collision object to search for
     * @return true if found, otherwise false
     * @see #addToIgnoreList(com.jme3.bullet.collision.PhysicsCollisionObject)
     */
    public boolean ignores(PhysicsCollisionObject other) {
        boolean result;
        if (ignoreList == null || other == null) {
            result = false;
        } else {
            result = ignoreList.contains(other);
        }

        return result;
    }

    /**
     * Test whether this object has been deactivated due to lack of motion.
     * <p>
     * Deactivation doesn't affect a PhysicsCharacter or PhysicsGhostObject.
     *
     * @return true if object still active, false if deactivated
     */
    public boolean isActive() {
        long objectId = nativeId();
        boolean result = isActive(objectId);

        return result;
    }

    /**
     * Test whether this object responds to contact with other objects. All
     * ghost objects are non-responsive. Other types are responsive by default.
     *
     * @return true if responsive, otherwise false
     */
    final public boolean isContactResponse() {
        long objectId = nativeId();
        int flags = getCollisionFlags(objectId);
        boolean result = (flags & CollisionFlag.NO_CONTACT_RESPONSE) == 0x0;

        return result;
    }

    /**
     * Test whether this object is added to a space.
     *
     * @return true&rarr;added to a space, false&rarr;not added to a space
     */
    final public boolean isInWorld() {
        long objectId = nativeId();
        boolean result = isInWorld(objectId);

        return result;
    }

    /**
     * Test whether this object is static (immobile).
     *
     * @return true if static, otherwise false
     */
    final public boolean isStatic() {
        long objectId = nativeId();
        int flags = getCollisionFlags(objectId);
        boolean result = (flags & CollisionFlag.STATIC_OBJECT) != 0x0;

        return result;
    }

    /**
     * Enumerate the native IDs of all collision objects in this object's ignore
     * list.
     *
     * @return a new array (not null, may be empty)
     * @see #addToIgnoreList(com.jme3.bullet.collision.PhysicsCollisionObject)
     * @deprecated use {@link #listIgnoredPcos()}
     */
    @Deprecated
    public long[] listIgnoredIds() {
        long objectId = nativeId();
        int numIgnoredObjects = getNumObjectsWithoutCollision(objectId);
        long[] result = new long[numIgnoredObjects];

        for (int listIndex = 0; listIndex < numIgnoredObjects; ++listIndex) {
            long otherId = getObjectWithoutCollision(objectId, listIndex);
            assert otherId != 0L;
            result[listIndex] = otherId;
        }

        return result;
    }

    /**
     * Enumerate all collision objects in this object's ignore list.
     *
     * @return a new array (not null, may be empty)
     * @see #addToIgnoreList(com.jme3.bullet.collision.PhysicsCollisionObject)
     */
    public PhysicsCollisionObject[] listIgnoredPcos() {
        PhysicsCollisionObject[] result;
        if (ignoreList == null) {
            result = new PhysicsCollisionObject[0];

        } else {
            int count = ignoreList.size();
            result = new PhysicsCollisionObject[count];

            int index = 0;
            for (PhysicsCollisionObject otherPco : ignoreList) {
                result[index] = otherPco;
                ++index;
            }
        }

        return result;
    }

    /**
     * Return the collision group of this object's broadphase proxy. A proxy is
     * created when the object is added to a space, and its group is 32 for a
     * PhysicsCharacter, 2 for a static object, or 1 for anything else.
     *
     * @return the proxy's collision group (a bitmask with exactly one bit set)
     * or null if this object has no proxy
     */
    public Integer proxyGroup() {
        Integer result = null;
        long objectId = nativeId();
        if (hasBroadphaseProxy(objectId)) {
            result = getProxyFilterGroup(objectId);
        }

        return result;
    }

    /**
     * Return the collision mask of this object's broadphase proxy. A proxy is
     * created when the object is added to a space, and its mask is -3 for a
     * static object or -1 for anything else.
     *
     * @return the proxy's bitmask, or null if this object has no proxy
     */
    public Integer proxyMask() {
        Integer result = null;
        long objectId = nativeId();
        if (hasBroadphaseProxy(objectId)) {
            result = getProxyFilterMask(objectId);
        }

        return result;
    }

    /**
     * Remove collision groups from the set with which this object can collide.
     *
     * @param collisionGroup the groups to remove, ORed together (bitmask)
     */
    public void removeCollideWithGroup(int collisionGroup) {
        this.collideWithGroups &= ~collisionGroup;
        if (hasAssignedNativeObject()) {
            setCollideWithGroups(collideWithGroups);
        }
    }

    /**
     * Remove a collision object from this object's ignore list and vice versa.
     *
     * @param otherPco the other collision object (not null, not this, modified)
     */
    public void removeFromIgnoreList(PhysicsCollisionObject otherPco) {
        Validate.nonNull(otherPco, "other collision object");
        Validate.require(otherPco != this, "2 distinct collision objects");

        if (ignoreList != null && ignoreList.contains(otherPco)) {
            ignoreList.remove(otherPco);

            assert otherPco.ignoreList != null;
            assert otherPco.ignoreList.contains(this);
            otherPco.ignoreList.remove(this);

            long thisId = nativeId();
            long otherId = otherPco.nativeId();
            boolean toIgnore = false;
            setIgnoreCollisionCheck(thisId, otherId, toIgnore);
        }
    }

    /**
     * Alter this object's anisotropic friction (native field:
     * m_anisotropicFriction).
     * <p>
     * Friction doesn't affect a PhysicsCharacter or PhysicsGhostObject.
     *
     * @param components the desired friction components (not null, unaffected,
     * default=(1,1,1))
     * @param mode the desired friction mode: 0=isotropic, 1=basic anisotropic
     * friction, 2=anisotropic rolling friction (default=0)
     */
    public void setAnisotropicFriction(Vector3f components, int mode) {
        Validate.nonNull(components, "components");
        Validate.inRange(mode, "mode", AfMode.none, AfMode.rolling);

        long objectId = nativeId();
        setAnisotropicFriction(objectId, components, mode);
    }

    /**
     * Associate application-specific data with this collision object. Minie
     * never touches application-specific data, except to clone/load/save it if
     * it is Cloneable or Savable.
     *
     * @param data the desired data object (alias created, default=null)
     * @see #getApplicationData()
     */
    public void setApplicationData(Object data) {
        this.applicationData = data;
    }

    /**
     * Alter the amount of motion required to trigger continuous collision
     * detection (CCD) (native field: m_ccdMotionThreshold).
     * <p>
     * CCD addresses the issue of fast objects passing through other objects
     * with no collision detected.
     * <p>
     * CCD doesn't affect a PhysicsCharacter or PhysicsGhostObject.
     *
     * @param threshold the desired minimum distance per timestep to trigger CCD
     * (in physics-space units, &gt;0) or zero to disable CCD (default=0)
     */
    public void setCcdMotionThreshold(float threshold) {
        long objectId = nativeId();
        setCcdMotionThreshold(objectId, threshold);
    }

    /**
     * Alter the continuous collision detection (CCD) swept-sphere radius for
     * this object (native field: m_ccdSweptSphereRadius).
     * <p>
     * CCD doesn't affect a PhysicsCharacter or PhysicsGhostObject.
     *
     * @param radius (in physics-space units, &ge;0, default=0)
     */
    public void setCcdSweptSphereRadius(float radius) {
        long objectId = nativeId();
        setCcdSweptSphereRadius(objectId, radius);
    }

    /**
     * Directly alter the collision groups with which this object can collide.
     *
     * @param collisionGroups the desired groups, ORed together (bitmask,
     * default=COLLISION_GROUP_01)
     */
    public void setCollideWithGroups(int collisionGroups) {
        long objectId = nativeId();
        this.collideWithGroups = collisionGroups;
        setCollideWithGroups(objectId, collideWithGroups);
    }

    /**
     * Alter which collision group this object belongs to.
     * <p>
     * Groups are represented by bitmasks with exactly one bit set. Manifest
     * constants are defined in PhysicsCollisionObject.
     * <p>
     * Two objects can collide only if one of them has the collisionGroup of the
     * other in its collideWithGroups set.
     *
     * @param collisionGroup the collisionGroup to apply (bitmask with exactly
     * one bit set, default=COLLISION_GROUP_01)
     */
    public void setCollisionGroup(int collisionGroup) {
        Validate.require(
                Integer.bitCount(collisionGroup) == 1, "exactly one bit set");

        this.collisionGroup = collisionGroup;
        long objectId = nativeId();
        setCollisionGroup(objectId, collisionGroup);
    }

    /**
     * Apply the specified shape to this object. Meant to be overridden.
     *
     * @param collisionShape the shape to apply (not null, alias created)
     */
    public void setCollisionShape(CollisionShape collisionShape) {
        Validate.nonNull(collisionShape, "collision shape");
        this.collisionShape = collisionShape;
    }

    /**
     * Alter the contact damping (native field: m_contactDamping). Also affects
     * the collision flags.
     * <p>
     * Contact damping doesn't affect a PhysicsCharacter or PhysicsGhostObject.
     *
     * @param damping the desired damping (default=0.1)
     */
    public void setContactDamping(float damping) {
        long objectId = nativeId();
        float stiffness = getContactStiffness(objectId);
        setContactStiffnessAndDamping(objectId, stiffness, damping);
    }

    /**
     * Alter the contact-processing threshold (native field:
     * m_contactProcessingThreshold).
     * <p>
     * Contact processing doesn't affect a PhysicsCharacter or
     * PhysicsGhostObject.
     *
     * @param distance the desired threshold distance (in physics-space units,
     * default=1e18 with SP library or 1e30 with DP library)
     */
    public void setContactProcessingThreshold(float distance) {
        long objectId = nativeId();
        setContactProcessingThreshold(objectId, distance);
    }

    /**
     * Alter the contact stiffness (native field: m_contactStiffness). Also
     * affects the collision flags.
     * <p>
     * Contact stiffness doesn't affect a PhysicsCharacter or
     * PhysicsGhostObject.
     *
     * @param stiffness the desired stiffness (default=1e18 with SP library or
     * 1e30 with DP library)
     */
    public void setContactStiffness(float stiffness) {
        long objectId = nativeId();
        float damping = getContactDamping(objectId);
        setContactStiffnessAndDamping(objectId, stiffness, damping);
    }

    /**
     * Alter the deactivation time (native field: m_deactivationTime).
     * <p>
     * Deactivation doesn't affect a PhysicsCharacter or PhysicsGhostObject.
     *
     * @param time the desired time (in seconds, default=0)
     */
    public void setDeactivationTime(float time) {
        long objectId = nativeId();
        setDeactivationTime(objectId, time);
    }

    /**
     * Apply or remove a custom debug material.
     *
     * @param material the Material to use (alias created) or null to use the
     * default debug materials (default=null)
     */
    public void setDebugMaterial(Material material) {
        this.debugMaterial = material;
    }

    /**
     * Apply or remove a listener for new debug meshes.
     *
     * @param listener the listener to use (alias created) or null for none
     * (default=null)
     */
    public void setDebugMeshInitListener(DebugMeshInitListener listener) {
        this.debugMeshInitListener = listener;
    }

    /**
     * Alter which normals to include in new debug meshes.
     *
     * @param newSetting an enum value (not null, default=None)
     */
    public void setDebugMeshNormals(MeshNormals newSetting) {
        Validate.nonNull(newSetting, "new setting");
        this.debugMeshNormals = newSetting;
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
        this.debugMeshResolution = newSetting;
    }

    /**
     * Alter how many sides of this object's default debug materials are
     * visible. This setting has no effect on custom debug materials.
     *
     * @param numSides the desired number of sides (&ge;0, &le;2, default=1)
     */
    public void setDebugNumSides(int numSides) {
        Validate.inRange(numSides, "number of sides", 0, 2);
        this.debugNumSides = numSides;
    }

    /**
     * Alter this object's friction (native field: m_friction).
     * <p>
     * Friction doesn't affect a PhysicsCharacter or PhysicsGhostObject.
     *
     * @param friction the desired friction value (&ge;0, default=0.5)
     */
    public void setFriction(float friction) {
        Validate.nonNegative(friction, "friction");

        long objectId = nativeId();
        setFriction(objectId, friction);
    }

    /**
     * Replace the ignore list.
     *
     * @param idList the collision-object IDs to ignore (not null, may be empty,
     * unaffected)
     * @deprecated use {@link #setIgnoreList(
     * com.jme3.bullet.collision.PhysicsCollisionObject[])}
     */
    @Deprecated
    public void setIgnoreList(long[] idList) {
        Validate.nonNull(idList, "ID list");

        clearIgnoreList();
        for (long otherId : idList) {
            PhysicsCollisionObject otherPco = findInstance(otherId);
            addToIgnoreList(otherPco);
        }
    }

    /**
     * Replace the ignore list.
     *
     * @param desiredList collision objects to ignore (not null, may be empty,
     * may contain nulls or duplicates or {@code this}, unaffected)
     */
    public void setIgnoreList(PhysicsCollisionObject[] desiredList) {
        Validate.nonNull(desiredList, "desired list");

        clearIgnoreList();
        if (desiredList.length > 0) {
            if (ignoreList == null) {
                this.ignoreList = new TreeSet<>();
            }

            for (PhysicsCollisionObject otherPco : desiredList) {
                if (otherPco != null && otherPco != this
                        && !ignoreList.contains(otherPco)) {
                    addToIgnoreList(otherPco);
                }
            }
        }
    }

    /**
     * Alter this object's restitution (bounciness) (native field:
     * m_restitution). For perfect elasticity, set restitution=1.
     * <p>
     * Restitution doesn't affect a PhysicsCharacter or PhysicsGhostObject.
     *
     * @param restitution the desired value (default=0)
     */
    public void setRestitution(float restitution) {
        long objectId = nativeId();
        setRestitution(objectId, restitution);
    }

    /**
     * Alter this object's rolling friction: torsional friction orthogonal to
     * the contact normal (native field: m_rollingFriction). Use this to stop
     * bodies from rolling.
     * <p>
     * Friction doesn't affect a PhysicsCharacter or PhysicsGhostObject.
     *
     * @param friction the desired friction value (default=0)
     */
    public void setRollingFriction(float friction) {
        long objectId = nativeId();
        setRollingFriction(objectId, friction);
    }

    /**
     * Alter this object's spinning friction: torsional friction around the
     * contact normal (native field: m_spinningFriction). Use for grasping.
     * <p>
     * Friction doesn't affect a PhysicsCharacter or PhysicsGhostObject.
     *
     * @param friction the desired friction value (default=0)
     */
    public void setSpinningFriction(float friction) {
        long objectId = nativeId();
        setSpinningFriction(objectId, friction);
    }

    /**
     * Associate a "user" with this collision object. Used by physics controls.
     *
     * @param user the desired scene object (alias created, default=null)
     * @see #getUserObject()
     * @see #setApplicationData(java.lang.Object)
     */
    public void setUserObject(Object user) {
        this.userObject = user;
    }

    /**
     * Return the ID of the space where this object is added.
     *
     * @return the ID, or zero if not added to any space
     */
    public long spaceId() {
        long objectId = nativeId();
        long spaceId = getSpaceId(objectId);

        return spaceId;
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
    native protected static void
            attachCollisionShape(long objectId, long collisionShapeId);

    /**
     * Clone an ignore list.
     *
     * @param cloner the Cloner that's cloning this object (not null, modified)
     * @param old the instance from which this object was shallow-cloned (not
     * null, unaffected)
     */
    protected void cloneIgnoreList(Cloner cloner, PhysicsCollisionObject old) {
        assert !doneCloningIgnores;

        if (old.ignoreList != null) {
            for (PhysicsCollisionObject oldPco : old.ignoreList) {
                /*
                 * We want to ignore only new PCOs, not old ones,
                 * so if the other PCO hasn't cloned its list yet,
                 * wait and let *that* PCO invoke addToIgnoreList().
                 */
                if (cloner.isCloned(oldPco)) {
                    PhysicsCollisionObject newPco = cloner.clone(oldPco);
                    if (newPco.doneCloningIgnores) {
                        addToIgnoreList(newPco);
                    }
                }
            }
        }
        this.doneCloningIgnores = true;
    }

    /**
     * Finalize the identified btCollisionObject. Native method.
     *
     * @param objectId the ID of the btCollisionObject (not zero)
     */
    native protected static void finalizeNative(long objectId);

    /**
     * Return the type of this object. Native method.
     *
     * @param objectId the ID of the btCollisionObject (not zero)
     * @return the type value
     * @see PcoType
     */
    final native protected static int getInternalType(long objectId);

    /**
     * (Re-)initialize the native user info of this object, which stores the
     * collision group, collide-with groups, and spaceId (native field:
     * m_userPointer).
     */
    protected void initUserPointer() {
        logger.log(Level.FINE, "initUserPointer() for {0}", this);
        long objectId = nativeId();
        initUserPointer(objectId, collisionGroup, collideWithGroups);

        assert getCollisionGroup(objectId) == collisionGroup;
        assert getCollideWithGroups(objectId) == collideWithGroups;
        assert getSpaceId(objectId) == 0L;
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

        boolean hasCsd = capsule.readBoolean(tagHasCsd, true);
        if (hasCsd) {
            setContactDamping(capsule.readFloat(tagContactDamping, 0.1f));
        }
        setContactProcessingThreshold(
                capsule.readFloat(tagContactProcessingThreshold, 0f));
        if (hasCsd) {
            setContactStiffness(capsule.readFloat(tagContactStiffness, 1e30f));
        }
        setDeactivationTime(capsule.readFloat(tagDeactivationTime, 0f));
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

        Savable[] tmpArray = capsule.readSavableArray(tagIgnoreList, null);
        if (tmpArray != null) {
            for (Savable savable : tmpArray) {
                PhysicsCollisionObject pco = (PhysicsCollisionObject) savable;
                addToIgnoreList(pco);
            }
        }

        this.applicationData = capsule.readSavable(tagApplicationData, null);
        this.userObject = capsule.readSavable(tagUserObject, null);
    }

    /**
     * Alter the activation state of this object. Native method.
     * <p>
     * Deactivation doesn't affect a PhysicsCharacter or PhysicsGhostObject.
     *
     * @param objectId the ID of the btCollisionObject (not zero)
     * @param desiredState the desired state
     */
    native protected static void
            setActivationState(long objectId, int desiredState);

    /**
     * Alter the collision flags of this object (native field:
     * m_collisionFlags). Subclasses are responsible for cloning/loading/saving
     * these flags. Flag values are defined in
     * {@link com.jme3.bullet.collision.CollisionFlag}. Native method.
     *
     * @param objectId the ID of the btCollisionObject (not zero)
     * @param desiredFlags the desired collision flags, ORed together
     */
    native protected static void
            setCollisionFlags(long objectId, int desiredFlags);

    /**
     * Alter the ignore list for collisions. TODO privatize
     *
     * @param object1Id the ID of the first btCollisionObject (not zero)
     * @param object2Id the ID of the 2nd btCollisionObject (not zero)
     * @param setting true to ignore, false to stop ignoring
     */
    native protected static void setIgnoreCollisionCheck(
            long object1Id, long object2Id, boolean setting);

    /**
     * Directly alter this object's location and basis.
     *
     * @param centerLocation the desired location for this object's center (in
     * physics-space coordinates, not null, unaffected)
     * @param orientation the desired orientation for this object (rotation
     * matrix in physics-space coordinates, not null, unaffected)
     */
    protected void
            setLocationAndBasis(Vector3f centerLocation, Matrix3f orientation) {
        Validate.finite(centerLocation, "center location");
        Validate.nonNull(orientation, "orientation");

        long objectId = nativeId();
        setLocationAndBasis(objectId, centerLocation, orientation);
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
        if (applicationData instanceof Cloneable) {
            this.applicationData = cloner.clone(applicationData);
        }
        if (userObject instanceof Cloneable) {
            this.userObject = cloner.clone(userObject);
        }

        this.collisionShape = cloner.clone(collisionShape);
        this.debugMaterial = cloner.clone(debugMaterial);
        this.ignoreList = null;
        /*
         * The caller should unassign the old native object and invoke
         * cloneIgnoreList() and copyPcoProperties().
         */
    }

    /**
     * Create a shallow clone for the JME cloner. Note that the cloned object
     * won't be added to any space, even if the original was.
     *
     * @return a new instance
     */
    @Override
    public PhysicsCollisionObject jmeClone() {
        try {
            PhysicsCollisionObject clone = (PhysicsCollisionObject) clone();
            clone.doneCloningIgnores = false;
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

        this.collisionGroup
                = capsule.readInt(tagCollisionGroup, COLLISION_GROUP_01);
        this.collideWithGroups
                = capsule.readInt(tagCollisionGroupsMask, COLLISION_GROUP_01);
        this.debugMeshNormals = capsule.readEnum(
                tagDebugMeshNormals, MeshNormals.class, MeshNormals.None);
        this.debugMeshResolution = capsule.readInt(tagDebugMeshResolution, 0);
        this.debugMaterial
                = (Material) capsule.readSavable(tagDebugMaterial, null);

        Savable shape = capsule.readSavable(tagCollisionShape, null);
        this.collisionShape = (CollisionShape) shape;
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
        capsule.write(
                collideWithGroups, tagCollisionGroupsMask, COLLISION_GROUP_01);
        capsule.write(
                debugMeshNormals, tagDebugMeshNormals, MeshNormals.None);
        capsule.write(debugMeshResolution, tagDebugMeshResolution, 0);
        capsule.write(debugMaterial, tagDebugMaterial, null);
        capsule.write(collisionShape, tagCollisionShape, null);

        // begin common properties
        if (applicationData instanceof Savable) {
            capsule.write((Savable) applicationData, tagApplicationData, null);
        }
        if (userObject instanceof Savable) {
            capsule.write((Savable) userObject, tagUserObject, null);
        }

        capsule.write(getCcdMotionThreshold(), tagCcdMotionThreshold, 0f);
        capsule.write(getCcdSweptSphereRadius(), tagCcdSweptSphereRadius, 0f);

        int flags = collisionFlags();
        boolean hasCsd
                = (flags & CollisionFlag.HAS_CONTACT_STIFFNESS_DAMPING) != 0;
        capsule.write(hasCsd, tagHasCsd, true);
        capsule.write(getContactDamping(), tagContactDamping, 0.1f);
        capsule.write(getContactProcessingThreshold(),
                tagContactProcessingThreshold, 0f);
        capsule.write(getContactStiffness(), tagContactStiffness, 1e30f);
        capsule.write(getDeactivationTime(), tagDeactivationTime, 0f);
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

        if (ignoreList != null) {
            int numIgnored = ignoreList.size();
            Savable[] tmpArray = new Savable[numIgnored];
            ignoreList.toArray(tmpArray);
            capsule.write(tmpArray, tagIgnoreList, null);
        }
    }
    // *************************************************************************
    // NativePhysicsObject methods

    /**
     * Represent this object as a String.
     *
     * @return a descriptive string of text (not null, not empty)
     */
    @Override
    public String toString() {
        String result = getClass().getSimpleName();
        result = result.replace("Body", "");
        result = result.replace("Control", "C");
        result = result.replace("Physics", "");
        result = result.replace("Object", "");
        if (hasAssignedNativeObject()) {
            long objectId = nativeId();
            result += "#" + Long.toHexString(objectId);
        } else {
            result += "#unassigned";
        }

        return result;
    }
    // *************************************************************************
    // Java private methods

    /**
     * Free the identified tracked native object. Invoked by reflection.
     *
     * @param objectId the native identifier (not zero)
     */
    private static void freeNativeObject(long objectId) {
        assert objectId != 0L;
        finalizeNative(objectId);
    }
    // *************************************************************************
    // native private methods

    native private static void activate(long objectId, boolean forceFlag);

    native private static int getActivationState(long objectId);

    native private static void
            getAnisotropicFriction(long objectId, Vector3f storeVector);

    native private static void getBasis(long objectId, Matrix3f storeMatrix);

    native private static void getBasisDp(long objectId, Matrix3d storeMatrix);

    native private static float getCcdMotionThreshold(long objectId);

    native private static float getCcdSweptSphereRadius(long objectId);

    native private static int getCollideWithGroups(long objectId);

    native private static int getCollisionFlags(long objectId);

    native private static int getCollisionGroup(long objectId);

    native private static CollisionSpace getCollisionSpace(long objectId);

    native private static float getContactDamping(long objectId);

    native private static float getContactProcessingThreshold(long objectId);

    native private static float getContactStiffness(long objectId);

    native private static float getDeactivationTime(long objectId);

    native private static float getFriction(long objectId);

    native private static void getLocation(long objectId, Vector3f storeVector);

    native private static void getLocationDp(long objectId, Vec3d storeVector);

    native private static int getNumObjectsWithoutCollision(long objectId);

    native private static long
            getObjectWithoutCollision(long objectId, int listIndex);

    native private static void
            getOrientation(long objectId, Quaternion storeQuat);

    native private static void
            getOrientationDp(long objectId, Quatd storeQuat);

    native private static int getProxyFilterGroup(long objectId);

    native private static int getProxyFilterMask(long objectId);

    native private static float getRestitution(long objectId);

    native private static float getRollingFriction(long objectId);

    native private static long getSpaceId(long objectId);

    native private static float getSpinningFriction(long objectId);

    native private static boolean
            hasAnisotropicFriction(long objectId, int mode);

    native private static boolean hasBroadphaseProxy(long objectId);

    native private void initUserPointer(long objectId, int group, int groups);

    native private static boolean isActive(long objectId);

    native private static boolean isInWorld(long objectId);

    native private static void setAnisotropicFriction(
            long objectId, Vector3f components, int mode);

    native private static void
            setCcdMotionThreshold(long objectId, float threshold);

    native private static void
            setCcdSweptSphereRadius(long objectId, float radius);

    native private static void
            setCollideWithGroups(long objectId, int collisionGroups);

    native private static void
            setCollisionGroup(long objectId, int collisionGroup);

    native private static void setContactProcessingThreshold(
            long objectId, float thresholdDistance);

    native private static void setContactStiffnessAndDamping(
            long objectId, float stiffness, float damping);

    native private static void setDeactivationTime(long objectId, float time);

    native private static void setFriction(long objectId, float friction);

    native private static void setLocationAndBasis(
            long objectId, Vector3f location, Matrix3f basis);

    native private static void setRestitution(long objectId, float restitution);

    native private static void
            setRollingFriction(long objectId, float friction);

    native private static void
            setSpinningFriction(long objectId, float friction);
}
