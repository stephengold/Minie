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

import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.infos.DebugMeshNormals;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.material.Material;
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
 * Collision objects include PhysicsCharacter, PhysicsRigidBody, and
 * PhysicsGhostObject.
 *
 * @author normenhansen
 */
abstract public class PhysicsCollisionObject
        implements JmeCloneable, Savable {
    // *************************************************************************
    // constants and loggers

    /**
     * collideWithGroups bitmask that represents "no groups"
     */
    public static final int COLLISION_GROUP_NONE = 0x0;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #1
     */
    public static final int COLLISION_GROUP_01 = 0x0001;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #2
     */
    public static final int COLLISION_GROUP_02 = 0x0002;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #3
     */
    public static final int COLLISION_GROUP_03 = 0x0004;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #4
     */
    public static final int COLLISION_GROUP_04 = 0x0008;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #5
     */
    public static final int COLLISION_GROUP_05 = 0x0010;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #6
     */
    public static final int COLLISION_GROUP_06 = 0x0020;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #7
     */
    public static final int COLLISION_GROUP_07 = 0x0040;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #8
     */
    public static final int COLLISION_GROUP_08 = 0x0080;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #9
     */
    public static final int COLLISION_GROUP_09 = 0x0100;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #10
     */
    public static final int COLLISION_GROUP_10 = 0x0200;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #11
     */
    public static final int COLLISION_GROUP_11 = 0x0400;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #12
     */
    public static final int COLLISION_GROUP_12 = 0x0800;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #13
     */
    public static final int COLLISION_GROUP_13 = 0x1000;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #14
     */
    public static final int COLLISION_GROUP_14 = 0x2000;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #15
     */
    public static final int COLLISION_GROUP_15 = 0x4000;
    /**
     * collisionGroup/collideWithGroups bitmask that represents group #16
     */
    public static final int COLLISION_GROUP_16 = 0x8000;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(PhysicsCollisionObject.class.getName());
    // *************************************************************************
    // fields

    /**
     * shape of this object (not null)
     */
    protected CollisionShape collisionShape;
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
     * Unique identifier of the btCollisionObject. Constructors are responsible
     * for setting this to a non-zero value. The id might change if the object
     * gets rebuilt.
     */
    protected long objectId = 0L;
    /**
     * custom material for debug shape, or null to use the default material
     */
    private Material debugMaterial = null;
    /**
     * associated user object
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
     * Read the set of collision groups with which this object can collide.
     *
     * @return bit mask
     */
    public int getCollideWithGroups() {
        return collideWithGroups;
    }

    /**
     * Read the collision group for this physics object.
     *
     * @return the collision group (bit mask with exactly one bit set)
     */
    public int getCollisionGroup() {
        return collisionGroup;
    }

    /**
     * Access the shape of this physics object.
     *
     * @return the pre-existing instance, which can then be applied to other
     * physics objects (sharing improves performance)
     */
    public CollisionShape getCollisionShape() {
        assert collisionShape != null;
        return collisionShape;
    }

    /**
     * Access the custom debug material, if specified.
     *
     * @return the pre-existing instance, or null if default/unspecified
     */
    public Material getDebugMaterial() {
        return debugMaterial;
    }

    /**
     * Read the ID of the btCollisionObject.
     *
     * @return the unique identifier (not zero)
     */
    public long getObjectId() {
        assert objectId != 0L;
        return objectId;
    }

    /**
     * Access the user object associated with this collision object.
     *
     * @return the pre-existing instance, or null if none
     */
    public Object getUserObject() {
        return userObject;
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
     * Directly alter the collision groups with which this object can collide.
     *
     * @param collisionGroups desired groups, ORed together (bit mask)
     */
    public void setCollideWithGroups(int collisionGroups) {
        collideWithGroups = collisionGroups;
        if (objectId != 0L) {
            setCollideWithGroups(objectId, collideWithGroups);
        }
    }

    /**
     * Alter the collision group for this physics object.
     * <p>
     * Groups are represented by integer bit masks with exactly 1 bit set.
     * Pre-made variables are available in PhysicsCollisionObject. By default,
     * physics objects are in COLLISION_GROUP_01.
     * <p>
     * Two objects can collide only if one of them has the collisionGroup of the
     * other in its collideWithGroups set.
     *
     * @param collisionGroup the collisionGroup to apply (bit mask with exactly
     * 1 bit set)
     */
    public void setCollisionGroup(int collisionGroup) {
        assert Integer.bitCount(collisionGroup) == 1 : collisionGroup;

        this.collisionGroup = collisionGroup;
        if (objectId != 0L) {
            setCollisionGroup(objectId, collisionGroup);
        }
    }

    /**
     * Apply the specified CollisionShape to this object. Note that the object
     * should not be in any physics space while changing shape; the object gets
     * rebuilt on the physics side.
     *
     * @param collisionShape the shape to apply (not null, alias created)
     */
    public void setCollisionShape(CollisionShape collisionShape) {
        Validate.nonNull(collisionShape, "collision shape");
        this.collisionShape = collisionShape;
    }

    /**
     * Alter or remove the custom debug material.
     *
     * @param material the desired material, or null for default/unspecified
     * (alias created)
     */
    public void setDebugMaterial(Material material) {
        debugMaterial = material;
    }

    /**
     * Alter which normals to include in new debug meshes.
     *
     * @param newSetting an enum value (not null)
     */
    public void setDebugMeshNormals(DebugMeshNormals newSetting) {
        Validate.nonNull(newSetting, "new setting");
        debugMeshNormals = newSetting;
    }

    /**
     * Alter the mesh resolution for new debug meshes. Effective only for convex
     * shapes.
     *
     * @param newSetting 0=low, 1=high
     */
    public void setDebugMeshResolution(int newSetting) {
        Validate.inRange(newSetting, "new setting", 0, 1);
        debugMeshResolution = newSetting;
    }

    /**
     * Associate a user object (such as a Spatial) with this collision object.
     *
     * @param userObject the object to associate with this collision object
     * (alias created, may be null)
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
     * Finalize the identified btCollisionObject. Native method.
     *
     * @param objectId the identifier of the btCollisionObject (not zero)
     */
    native protected void finalizeNative(long objectId);

    /**
     * Initialize the collision-group information of this object.
     */
    protected void initUserPointer() {
        logger.log(Level.FINE, "initUserPointer() objectId = {0}",
                Long.toHexString(objectId));
        initUserPointer(objectId, collisionGroup, collideWithGroups);
    }
    // *************************************************************************
    // JmeCloneable methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned object into a deep-cloned one, using the specified cloner
     * and original to resolve copied fields.
     *
     * @param cloner the cloner that's cloning this shape (not null)
     * @param original the instance from which this instance was shallow-cloned
     * (unused)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        userObject = cloner.clone(userObject);
        collisionShape = cloner.clone(collisionShape);
        debugMaterial = cloner.clone(debugMaterial);
        objectId = 0L; // subclass must create the btCollisionObject
    }

    /**
     * Create a shallow clone for the JME cloner. Note that the cloned object
     * won't be added to a physics space, even if the original was.
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
     * De-serialize this object, for example when loading from a J3O file.
     *
     * @param im importer (not null)
     * @throws IOException from importer
     */
    @Override
    public void read(JmeImporter im) throws IOException {
        InputCapsule capsule = im.getCapsule(this);

        collisionGroup = capsule.readInt("collisionGroup", COLLISION_GROUP_01);
        collideWithGroups = capsule.readInt("collisionGroupsMask",
                COLLISION_GROUP_01);
        debugMeshNormals = capsule.readEnum("debugMeshNormals",
                DebugMeshNormals.class, DebugMeshNormals.None);
        debugMeshResolution = capsule.readInt("debugMeshResolution", 0);
        debugMaterial = (Material) capsule.readSavable("debugMaterial", null);

        Savable shape = capsule.readSavable("collisionShape", null);
        collisionShape = (CollisionShape) shape;
    }

    /**
     * Serialize this object, for example when saving to a J3O file.
     *
     * @param ex exporter (not null)
     * @throws IOException from exporter
     */
    @Override
    public void write(JmeExporter ex) throws IOException {
        OutputCapsule capsule = ex.getCapsule(this);

        capsule.write(collisionGroup, "collisionGroup", COLLISION_GROUP_01);
        capsule.write(collideWithGroups, "collisionGroupsMask",
                COLLISION_GROUP_01);
        capsule.write(debugMeshNormals, "debugMeshNormals",
                DebugMeshNormals.None);
        capsule.write(debugMeshResolution, "debugMeshResolution", 0);
        capsule.write(debugMaterial, "debugMaterial", null);
        capsule.write(collisionShape, "collisionShape", null);
    }
    // *************************************************************************
    // Object methods

    /**
     * Finalize this collision object just before it is destroyed. Should be
     * invoked only by a subclass or by the garbage collector.
     *
     * @throws Throwable ignored by the garbage collector
     */
    @Override
    protected void finalize() throws Throwable {
        super.finalize();
        logger.log(Level.FINE, "Finalizing CollisionObject {0}",
                Long.toHexString(objectId));
        finalizeNative(objectId);
    }
    // *************************************************************************
    // private methods

    native private void activate(long objectId, boolean forceFlag);

    native private void initUserPointer(long objectId, int group, int groups);

    native private void setCollideWithGroups(long objectId,
            int collisionGroups);

    native private void setCollisionGroup(long objectId, int collisionGroup);
}
