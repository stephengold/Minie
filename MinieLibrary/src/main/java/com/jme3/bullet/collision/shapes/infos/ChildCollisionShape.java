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
package com.jme3.bullet.collision.shapes.infos;

import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.EmptyShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Triangle;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import com.jme3.util.clone.JmeCloneable;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;

/**
 * An element in a CompoundCollisionShape, consisting of a (non-compound) base
 * shape, offset and rotated with respect to its parent.
 * <p>
 * Despite its name, it is not a subtype of CollisionShape!
 *
 * @author normenhansen
 */
public class ChildCollisionShape implements JmeCloneable, Savable {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(ChildCollisionShape.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagOffset = "location";
    final private static String tagRotation = "rotation";
    final private static String tagShape = "shape";
    // *************************************************************************
    // fields

    /**
     * base shape (not null, not a compound shape)
     */
    private CollisionShape shape;
    /**
     * copy of rotation in the parent's coordinate system (not null)
     */
    private Matrix3f rotation;
    /**
     * copy of translation in the parent's coordinate system (not null)
     */
    private Vector3f offset;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected ChildCollisionShape() {
    }

    /**
     * Instantiate a child for use in a compound shape.
     *
     * @param offset the desired translation in the parent's coordinate system
     * (not null, unaffected)
     * @param rotation the desired rotation in the parent's coordinate system
     * (not null, unaffected)
     * @param shape the base shape (not null, not a compound shape, alias
     * created)
     */
    public ChildCollisionShape(
            Vector3f offset, Matrix3f rotation, CollisionShape shape) {
        Validate.nonNull(shape, "shape");
        if (shape instanceof CompoundCollisionShape) {
            throw new IllegalArgumentException(
                    "CompoundCollisionShapes cannot be child shapes!");
        }

        this.offset = offset.clone();
        this.rotation = rotation.clone();
        this.shape = shape;
    }

    /**
     * Instantiate a child without any rotation.
     *
     * @param offset the desired translation in the parent's coordinate system
     * (not null, unaffected)
     * @param shape the base shape (not null, not a compound shape, alias
     * created)
     */
    public ChildCollisionShape(Vector3f offset, CollisionShape shape) {
        Validate.nonNull(shape, "shape");
        if (shape instanceof CompoundCollisionShape) {
            throw new IllegalArgumentException(
                    "CompoundCollisionShapes cannot be child shapes!");
        }

        this.offset = offset.clone();
        this.rotation = new Matrix3f();
        this.shape = shape;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the translation in the parent's coordinate system.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a translation vector (either storeResult or a new vector, not
     * null)
     */
    public Vector3f copyOffset(Vector3f storeResult) {
        Vector3f result;
        if (storeResult == null) {
            result = offset.clone();
        } else {
            result = storeResult.set(offset);
        }

        return result;
    }

    /**
     * Copy the rotation in the parent's coordinate system.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a Quaternion (either storeResult or a new Quaternion, not null)
     */
    public Quaternion copyRotation(Quaternion storeResult) {
        Quaternion result
                = (storeResult == null) ? new Quaternion() : storeResult;
        result.fromRotationMatrix(rotation);
        return result;
    }

    /**
     * Copy the rotation in the parent's coordinate system.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a rotation matrix (either storeResult or a new matrix, not null)
     */
    public Matrix3f copyRotationMatrix(Matrix3f storeResult) {
        Matrix3f result;
        if (storeResult == null) {
            result = rotation.clone();
        } else {
            result = storeResult.set(rotation);
        }

        return result;
    }

    /**
     * Copy the Transform relative to the parent's coordinate system.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a Transform with scale=1 (either storeResult or a new instance,
     * not null)
     */
    public Transform copyTransform(Transform storeResult) {
        Transform result
                = (storeResult == null) ? new Transform() : storeResult;

        result.setTranslation(offset);
        result.getRotation().fromRotationMatrix(rotation);
        result.setScale(1f);

        return result;
    }

    /**
     * Access the base shape.
     *
     * @return the pre-existing shape (not null)
     */
    public CollisionShape getShape() {
        assert shape != null;
        return shape;
    }

    /**
     * Alter the child's coordinate transform copy. For internal use only.
     *
     * @param offset the desired translation relative to the parent (not null,
     * unaffected)
     * @param rotation the desired rotation relative to the parent (not null,
     * unaffected)
     * @see com.jme3.bullet.collision.shapes.CompoundCollisionShape
     * #setChildTransform(com.jme3.bullet.collision.shapes.CollisionShape,
     * com.jme3.math.Transform)
     */
    public void setTransform(Vector3f offset, Matrix3f rotation) {
        this.offset.set(offset);
        this.rotation.set(rotation);
    }

    /**
     * Attempt to divide this child into 2 children. The base shape must be
     * splittable.
     *
     * @param parentTriangle a triangle that defines the splitting plane (in
     * parent's shape coordinates, not null, unaffected)
     * @return a pair of children, the first element on the triangle's minus
     * side and the 2nd element on its plus side; either element may be null,
     * indicating an empty child
     */
    public ChildCollisionShape[] split(Triangle parentTriangle) {
        Validate.nonNull(parentTriangle, "parent triangle");

        ChildCollisionShape[] result = new ChildCollisionShape[2];
        if (shape instanceof EmptyShape) {
            return result;
        }
        HullCollisionShape hull = (HullCollisionShape) shape; // TODO more cases

        Transform c2pTransform = copyTransform(null);
        shape.getScale(c2pTransform.getScale()); // Copy the scale factors.
        Triangle childTriangle
                = MyMath.transformInverse(c2pTransform, parentTriangle, null);
        ChildCollisionShape[] mp = hull.split(childTriangle);

        Transform tmpTransform = new Transform();
        for (int i = 0; i < 2; ++i) {
            if (mp[i] != null) {
                mp[i].copyTransform(tmpTransform);
                tmpTransform.combineWithParent(c2pTransform);
                Vector3f newOffset = tmpTransform.getTranslation(); // alias
                CollisionShape base = mp[i].getShape();
                result[i] = new ChildCollisionShape(newOffset, rotation, base);
            }
        }

        return result;
    }
    // *************************************************************************
    // JmeCloneable methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned element into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this element (not null)
     * @param original the instance from which this element was shallow-cloned
     * (unused)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        this.offset = cloner.clone(offset);
        this.rotation = cloner.clone(rotation);
        this.shape = cloner.clone(shape);
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public ChildCollisionShape jmeClone() {
        try {
            ChildCollisionShape clone = (ChildCollisionShape) clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }
    // *************************************************************************
    // Savable methods

    /**
     * De-serialize this child from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        InputCapsule capsule = importer.getCapsule(this);

        this.offset
                = (Vector3f) capsule.readSavable(tagOffset, new Vector3f());
        this.rotation
                = (Matrix3f) capsule.readSavable(tagRotation, new Matrix3f());
        this.shape = (CollisionShape) capsule
                .readSavable(tagShape, new BoxCollisionShape(1f));
    }

    /**
     * Serialize this child to the specified exporter, for example when saving
     * to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(offset, tagOffset, null);
        capsule.write(rotation, tagRotation, null);
        capsule.write(shape, tagShape, null);
    }
}
