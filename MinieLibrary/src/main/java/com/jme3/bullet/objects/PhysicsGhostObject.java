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
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * A collision object for intangibles, based on Bullet's
 * btPairCachingGhostObject. This is useful for creating a character controller,
 * collision sensors/triggers, explosions etc.
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
     * TODO reused list
     */
    private List<PhysicsCollisionObject> overlappingObjects
            = new LinkedList<>();
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public PhysicsGhostObject() {
    }

    /**
     * Instantiate an object with the specified collision shape.
     *
     * @param shape the desired shape (not null, alias created)
     */
    public PhysicsGhostObject(CollisionShape shape) {
        collisionShape = shape;
        buildObject();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Read the continuous collision detection (CCD) motion threshold for this
     * object.
     *
     * @return threshold value (in physics-space units per second, &ge;0)
     */
    public float getCcdMotionThreshold() {
        return getCcdMotionThreshold(objectId);
    }

    /**
     * Read the CCD square motion threshold for this object.
     *
     * @return threshold value (squared velocity, &ge;0)
     */
    public float getCcdSquareMotionThreshold() {
        return getCcdSquareMotionThreshold(objectId);
    }

    /**
     * Read the radius of the sphere used for continuous collision detection
     * (CCD).
     *
     * @return radius (in physics-space units, &ge;0)
     */
    public float getCcdSweptSphereRadius() {
        return getCcdSweptSphereRadius(objectId);
    }

    /**
     * Access an overlapping collision object by its position in the list.
     *
     * @param index which list position (&ge;0, &lt;count)
     * @return the pre-existing object
     */
    public PhysicsCollisionObject getOverlapping(int index) {
        return overlappingObjects.get(index);
    }

    /**
     * Count how many collision objects this object overlaps.
     *
     * @return count (&ge;0)
     */
    public int getOverlappingCount() {
        return getOverlappingCount(objectId);
    }

    /**
     * Access a list of overlapping objects.
     *
     * @return an internal list which may get reused (not null)
     */
    public List<PhysicsCollisionObject> getOverlappingObjects() {
        overlappingObjects.clear();
        getOverlappingObjects(objectId);

        return overlappingObjects;
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
        getPhysicsLocation(objectId, result);
        return result;
    }

    /**
     * Copy this object's orientation to a quaternion.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return an orientation (in physics-space coordinates, either storeResult
     * or a new quaternion, not null)
     */
    public Quaternion getPhysicsRotation(Quaternion storeResult) {
        Quaternion result
                = (storeResult == null) ? new Quaternion() : storeResult;
        getPhysicsRotation(objectId, result);
        return result;
    }

    /**
     * Copy this object's orientation to a matrix.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return an orientation (in physics-space coordinates, either storeResult
     * or a new matrix, not null)
     */
    public Matrix3f getPhysicsRotationMatrix(Matrix3f storeResult) {
        Matrix3f result = (storeResult == null) ? new Matrix3f() : storeResult;
        getPhysicsRotationMatrix(objectId, result);
        return result;
    }

    /**
     * Alter the amount of motion required to trigger continuous collision
     * detection (CCD).
     * <p>
     * This addresses the issue of fast objects passing through other objects
     * with no collision detected.
     *
     * @param threshold the desired threshold value (in physics-space units per
     * second, &gt;0) or zero to disable CCD (default=0)
     */
    public void setCcdMotionThreshold(float threshold) {
        setCcdMotionThreshold(objectId, threshold);
    }

    /**
     * Alter the continuous collision detection (CCD) swept sphere radius for
     * this object.
     *
     * @param radius (in physics-space units, &ge;0)
     */
    public void setCcdSweptSphereRadius(float radius) {
        setCcdSweptSphereRadius(objectId, radius);
    }

    /**
     * Apply the specified CollisionShape to this object. Note that the object
     * should not be in any physics space while changing shape; the object gets
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
     * Directly alter the location of this object's center.
     *
     * @param location the desired location (in physics-space coordinates, not
     * null, unaffected)
     */
    public void setPhysicsLocation(Vector3f location) {
        setPhysicsLocation(objectId, location);
    }

    /**
     * Directly alter this object's orientation.
     *
     * @param rotation the desired orientation (a rotation matrix in
     * physics-space coordinates, not null, unaffected)
     */
    public void setPhysicsRotation(Matrix3f rotation) {
        setPhysicsRotation(objectId, rotation);
    }

    /**
     * Directly alter this object's orientation.
     *
     * @param rotation the desired orientation (quaternion, not null,
     * unaffected)
     */
    public void setPhysicsRotation(Quaternion rotation) {
        setPhysicsRotation(objectId, rotation);
    }
    // *************************************************************************
    // PhysicsCollisionObject methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned object into a deep-cloned one, using the specified cloner
     * and original to resolve copied fields.
     *
     * @param cloner the cloner that's cloning this object (not null)
     * @param original the instance from which this instance was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);
        overlappingObjects = cloner.clone(overlappingObjects);
        buildObject();

        PhysicsGhostObject old = (PhysicsGhostObject) original;
        setCcdMotionThreshold(old.getCcdMotionThreshold());
        setCcdSweptSphereRadius(old.getCcdSweptSphereRadius());
        setPhysicsLocation(old.getPhysicsLocation(null));
        setPhysicsRotation(old.getPhysicsRotationMatrix(null));
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public PhysicsGhostObject jmeClone() {
        try {
            PhysicsGhostObject clone = (PhysicsGhostObject) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }

    /**
     * Serialize this object, for example when saving to a J3O file.
     *
     * @param ex exporter (not null)
     * @throws IOException from exporter
     */
    @Override
    public void write(JmeExporter ex) throws IOException {
        super.write(ex);
        OutputCapsule capsule = ex.getCapsule(this);
        capsule.write(getPhysicsLocation(new Vector3f()),
                "physicsLocation", new Vector3f());
        capsule.write(getPhysicsRotationMatrix(new Matrix3f()),
                "physicsRotation", new Matrix3f());
        capsule.write(getCcdMotionThreshold(), "ccdMotionThreshold", 0f);
        capsule.write(getCcdSweptSphereRadius(), "ccdSweptSphereRadius", 0f);
    }

    /**
     * De-serialize this object from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param im importer (not null)
     * @throws IOException from importer
     */
    @Override
    public void read(JmeImporter im) throws IOException {
        super.read(im);
        InputCapsule capsule = im.getCapsule(this);
        buildObject();
        setPhysicsLocation((Vector3f) capsule.readSavable("physicsLocation",
                new Vector3f()));
        setPhysicsRotation(((Matrix3f) capsule.readSavable("physicsRotation",
                new Matrix3f())));
        setCcdMotionThreshold(capsule.readFloat("ccdMotionThreshold", 0f));
        setCcdSweptSphereRadius(capsule.readFloat("ccdSweptSphereRadius", 0f));
    }
    // *************************************************************************
    // private methods

    /**
     * This method is invoked from native code.
     *
     * @param co the collision object to add
     */
    private void addOverlappingObject_native(PhysicsCollisionObject co) {
        overlappingObjects.add(co);
    }

    /**
     * Create the configured object in Bullet.
     */
    private void buildObject() {
        if (objectId == 0L) {
            objectId = createGhostObject();
            logger2.log(Level.FINE, "Created Ghost Object {0}",
                    Long.toHexString(objectId));
            setGhostFlags(objectId);
            initUserPointer();
        }
        attachCollisionShape(objectId, collisionShape.getObjectId());
    }

    native private long createGhostObject();

    native private float getCcdMotionThreshold(long objectId);

    native private float getCcdSquareMotionThreshold(long objectId);

    native private float getCcdSweptSphereRadius(long objectId);

    native private int getOverlappingCount(long objectId);

    native private void getOverlappingObjects(long objectId);

    native private void getPhysicsLocation(long objectId, Vector3f storeResult);

    native private void getPhysicsRotation(long objectId,
            Quaternion storeResult);

    native private void getPhysicsRotationMatrix(long objectId,
            Matrix3f storeResult);

    native private void setCcdMotionThreshold(long objectId, float threshold);

    native private void setCcdSweptSphereRadius(long objectId, float radius);

    native private void setGhostFlags(long objectId);

    native private void setPhysicsLocation(long objectId, Vector3f location);

    native private void setPhysicsRotation(long objectId, Matrix3f rotation);

    native private void setPhysicsRotation(long objectId, Quaternion rotation);
}
