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

import com.jme3.bullet.collision.PcoType;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import com.simsilica.mathd.Matrix3d;
import com.simsilica.mathd.Quatd;
import com.simsilica.mathd.Vec3d;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A collision object for intangibles, based on Bullet's
 * {@code btPairCachingGhostObject}. This is useful for creating a character
 * controller, collision sensors/triggers, explosions etc.
 * <p>
 * Overlap detection skips the narrow-phase collision-detection algorithm and
 * relies entirely on the broad-phase algorithm, which is AABB plus a margin of
 * about 0.06 world units. Precise collision detection is still available via
 * {@code PhysicsSpace.addCollisionListener()}.
 * <p>
 * <i>From Bullet manual:</i><br>
 * btGhostObject is a special btCollisionObject, useful for fast localized
 * collision queries.
 *
 * @author normenhansen
 */
public class PhysicsGhostObject extends PhysicsCollisionObject {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(PhysicsGhostObject.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagPhysicsLocation = "physicsLocation";
    final private static String tagPhysicsRotation = "physicsRotation";
    // *************************************************************************
    // fields

    /**
     * TODO reused list
     */
    private List<PhysicsCollisionObject> overlappingObjects
            = new LinkedList<>();
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected PhysicsGhostObject() {
    }

    /**
     * Instantiate a ghost object with the specified CollisionShape. The new
     * object is not added to any CollisionSpace.
     *
     * @param shape the desired shape (not null, alias created)
     */
    public PhysicsGhostObject(CollisionShape shape) {
        super.setCollisionShape(shape);
        buildObject();

        assert !isContactResponse();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Access an overlapping collision object by its position in the list.
     * Important: {@link #getOverlappingObjects()} must be invoked first!
     *
     * @param index which list position (&ge;0, &lt;count)
     * @return the pre-existing object
     */
    public PhysicsCollisionObject getOverlapping(int index) {
        PhysicsCollisionObject result = overlappingObjects.get(index);
        return result;
    }

    /**
     * Count how many collision objects this object overlaps.
     *
     * @return the count (&ge;0)
     */
    public int getOverlappingCount() {
        long objectId = nativeId();
        int result = getOverlappingCount(objectId);

        return result;
    }

    /**
     * Update and access a list of overlapping objects.
     *
     * @return an internal list which may get reused (not null)
     */
    public List<PhysicsCollisionObject> getOverlappingObjects() {
        overlappingObjects.clear();

        long objectId = nativeId();
        getOverlappingObjects(objectId);

        return overlappingObjects;
    }

    /**
     * Directly alter the location of the ghost's center.
     *
     * @param location the desired location (in physics-space coordinates, not
     * null, finite, unaffected)
     */
    public void setPhysicsLocation(Vector3f location) {
        Validate.finite(location, "location");

        long objectId = nativeId();
        setPhysicsLocation(objectId, location);
    }

    /**
     * Directly alter the location of the ghost's center.
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
     * Directly alter the ghost's orientation.
     *
     * @param orientation the desired orientation (a rotation matrix in
     * physics-space coordinates, not null, unaffected)
     */
    public void setPhysicsRotation(Matrix3f orientation) {
        Validate.nonNull(orientation, "orientation");

        long objectId = nativeId();
        setPhysicsRotation(objectId, orientation);
    }

    /**
     * Directly alter the ghost's orientation.
     *
     * @param orientation the desired orientation (a rotation quaternion in
     * physics-space coordinates, not null, not zero, unaffected)
     */
    public void setPhysicsRotation(Quaternion orientation) {
        Validate.nonZero(orientation, "orientation");

        long objectId = nativeId();
        setPhysicsRotation(objectId, orientation);
    }

    /**
     * Directly alter the ghost's orientation.
     *
     * @param orientation the desired orientation (a rotation matrix in
     * physics-space coordinates, not null, unaffected)
     */
    public void setPhysicsRotationDp(Matrix3d orientation) {
        Validate.nonNull(orientation, "orientation");

        long objectId = nativeId();
        setPhysicsRotationDp(objectId, orientation);
    }

    /**
     * Directly alter the ghost's orientation.
     *
     * @param orientation the desired orientation (a rotation quaternion in
     * physics-space coordinates, not null, unaffected)
     */
    public void setPhysicsRotationDp(Quatd orientation) {
        Validate.nonNull(orientation, "orientation");

        long objectId = nativeId();
        setPhysicsRotationDp(objectId, orientation);
    }
    // *************************************************************************
    // PhysicsCollisionObject methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned object into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this object (not null)
     * @param original the instance from which this object was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);
        unassignNativeObject();
        this.overlappingObjects = cloner.clone(overlappingObjects);
        buildObject();

        PhysicsGhostObject old = (PhysicsGhostObject) original;
        cloneIgnoreList(cloner, old);
        copyPcoProperties(old);

        setPhysicsLocation(old.getPhysicsLocation(null));
        setPhysicsRotation(old.getPhysicsRotationMatrix(null));
    }

    /**
     * Apply the specified CollisionShape to this object. Note that the object
     * should not be in any CollisionSpace while changing shape; the object gets
     * rebuilt on the physics side.
     *
     * @param collisionShape the shape to apply (not null, alias created)
     */
    @Override
    public void setCollisionShape(CollisionShape collisionShape) {
        super.setCollisionShape(collisionShape);
        buildObject();
    }

    /**
     * De-serialize this object from the specified importer, for example when
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

        setPhysicsLocation((Vector3f) capsule.readSavable(
                tagPhysicsLocation, new Vector3f()));
        setPhysicsRotation(((Matrix3f) capsule.readSavable(
                tagPhysicsRotation, new Matrix3f())));
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
        super.write(exporter);
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(getPhysicsLocation(null), tagPhysicsLocation, null);
        capsule.write(getPhysicsRotationMatrix(null), tagPhysicsRotation, null);
    }
    // *************************************************************************
    // Java private methods

    /**
     * This method is invoked by native code.
     *
     * @param co the collision object to add (alias created)
     */
    private void addOverlappingObject(PhysicsCollisionObject co) {
        overlappingObjects.add(co);
    }

    /**
     * Create the configured object in Bullet.
     */
    private void buildObject() {
        if (!hasAssignedNativeObject()) {
            long objectId = createGhostObject();
            setNativeId(objectId);
            assert getInternalType(objectId) == PcoType.ghost :
                    getInternalType(objectId);
            logger2.log(Level.FINE, "Created {0}.", this);

            setGhostFlags(objectId);
            initUserPointer();
        }

        long objectId = nativeId();
        CollisionShape shape = getCollisionShape();
        attachCollisionShape(objectId, shape.nativeId());
    }
    // *************************************************************************
    // native private methods

    native private static long createGhostObject();

    native private static int getOverlappingCount(long objectId);

    native private void getOverlappingObjects(long objectId);

    native private static void setGhostFlags(long objectId);

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
}
