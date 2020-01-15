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
package com.jme3.bullet.collision.shapes;

import com.jme3.bullet.collision.shapes.infos.ChildCollisionShape;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Matrix3f;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A CollisionShape formed by combining child shapes, based on Bullet's
 * btCompoundShape.
 *
 * @author normenhansen
 */
public class CompoundCollisionShape extends CollisionShape {
    // *************************************************************************
    // constants and loggers

    /**
     * default initial allocation for children
     */
    final private static int defaultCapacity = 6;
    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(CompoundCollisionShape.class.getName());
    /**
     * local copy of {@link com.jme3.math.Matrix3f#IDENTITY}
     */
    final private static Matrix3f matrixIdentity = new Matrix3f();
    /**
     * field names for serialization
     */
    final private static String tagChildren = "children";
    /**
     * local copy of {@link com.jme3.math.Vector3f#ZERO}
     */
    final private static Vector3f translateIdentity = new Vector3f(0f, 0f, 0f);
    // *************************************************************************
    // fields

    /**
     * child shapes of this shape
     */
    private ArrayList<ChildCollisionShape> children
            = new ArrayList<>(defaultCapacity);
    // *************************************************************************
    // constructors

    /**
     * Instantiate an empty compound shape (with dynamic AABB and no children).
     */
    public CompoundCollisionShape() {
        createEmpty(defaultCapacity);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Add a child shape without transforming its coordinates.
     *
     * @param childShape the child shape to add (not null, not a compound shape,
     * alias created)
     */
    public void addChildShape(CollisionShape childShape) {
        Validate.nonNull(childShape, "child shape");
        addChildShape(childShape, translateIdentity, matrixIdentity);
    }

    /**
     * Add a child shape with the specified local translation.
     *
     * @param childShape the child shape to add (not null, not a compound shape,
     * alias created)
     * @param offset the local coordinates of the child shape's center (not
     * null, unaffected)
     */
    public void addChildShape(CollisionShape childShape, Vector3f offset) {
        Validate.nonNull(childShape, "child shape");
        Validate.nonNull(offset, "offset");

        addChildShape(childShape, offset, matrixIdentity);
    }

    /**
     * Add a child shape with the specified local translation.
     *
     * @param childShape the child shape to add (not null, not a compound shape,
     * alias created)
     * @param offsetX the local X coordinate of the child shape's center
     * @param offsetY the local Y coordinate of the child shape's center
     * @param offsetZ the local Z coordinate of the child shape's center
     */
    public void addChildShape(CollisionShape childShape, float offsetX,
            float offsetY, float offsetZ) {
        Validate.nonNull(childShape, "child shape");

        Vector3f offset
                = new Vector3f(offsetX, offsetY, offsetZ); // TODO garbage
        addChildShape(childShape, offset, matrixIdentity);
    }

    /**
     * Add a child shape with the specified local translation and orientation.
     *
     * @param childShape the child shape to add (not null, not a compound shape,
     * alias created)
     * @param offset the local coordinates of the child shape's center (not
     * null, unaffected)
     * @param rotation the local orientation of the child shape (not null,
     * unaffected)
     */
    public void addChildShape(CollisionShape childShape, Vector3f offset,
            Matrix3f rotation) {
        if (childShape instanceof CompoundCollisionShape) {
            throw new IllegalArgumentException(
                    "A CompoundCollisionShape cannot have"
                    + " a CompoundCollisionShape child!");
        }
        long childId = childShape.getObjectId();

        ChildCollisionShape child
                = new ChildCollisionShape(offset, rotation, childShape);
        children.add(child);

        long parentId = getObjectId();
        addChildShape(parentId, childId, offset, rotation);
    }

    /**
     * Add a child shape with the specified local transform. The transform's
     * scale is ignored.
     *
     * @param shape the child shape to add (not null, not a compound shape,
     * alias created)
     * @param transform the local transform of the child shape (not null,
     * unaffected)
     */
    public void addChildShape(CollisionShape shape, Transform transform) {
        Vector3f offset = transform.getTranslation();
        Matrix3f rotation = transform.getRotation().toRotationMatrix();
        addChildShape(shape, offset, rotation);
    }

    /**
     * Apply the inverse of the specified Transform to each child shape. Assumes
     * that no 2 children refer to the same shape! TODO fix this
     *
     * @param paTransform the Transform to un-apply, typically one calculated by
     * {@link #principalAxes(java.nio.FloatBuffer, com.jme3.math.Transform, com.jme3.math.Vector3f)}
     * (not null, unaffected, scale=1)
     */
    public void correctAxes(Transform paTransform) {
        Transform invTransform = paTransform.clone().invert();

        Transform tmpTransform = new Transform();
        for (ChildCollisionShape child : children) {
            child.copyTransform(tmpTransform);
            tmpTransform.combineWithParent(invTransform);
            setChildTransform(child.getShape(), tmpTransform);
        }
    }

    /**
     * Count the child shapes.
     *
     * @return the count (&ge;0)
     */
    public int countChildren() {
        int numChildren = children.size();
        assert numChildren == countChildren(getObjectId());

        return numChildren;
    }

    /**
     * Find the first child with the specified shape.
     *
     * @param childShape the shape to search for (unaffected)
     * @return the index of the child if found, otherwise -1
     */
    public int findIndex(CollisionShape childShape) {
        int result = -1;
        for (int index = 0; index < children.size(); ++index) {
            ChildCollisionShape ccs = children.get(index);
            CollisionShape shape = ccs.getShape();
            if (shape == childShape) {
                result = index;
                break;
            }
        }

        return result;
    }

    /**
     * Enumerate the child shapes.
     *
     * @return a new array of pre-existing child shapes (not null)
     */
    public ChildCollisionShape[] listChildren() {
        int numChildren = children.size();
        ChildCollisionShape[] result = new ChildCollisionShape[numChildren];
        children.toArray(result);

        return result;
    }

    /**
     * Calculates the coordinate transform to be applied to a collision object
     * in order for this shape to be centered at the center of mass and its
     * principal axes to coincide with its local axes. Apply the inverse of this
     * transform to each child shape. The resuling moment of inertia is also
     * calculated.
     *
     * @param masses the mass for each child shape (not null, direct, all
     * elements &gt;0)
     * @param storeTransform storage for the transform (modified if not null)
     * @param storeInertia storage for the moment of inertia (not null,
     * modified)
     * @return a coordinate transform to apply to the collision object (either
     * storeTransform or a new instance, not null)
     */
    public Transform principalAxes(FloatBuffer masses, Transform storeTransform,
            Vector3f storeInertia) {
        if (!masses.isDirect()) {
            throw new IllegalArgumentException("The buffer must be direct.");
        }
        Transform result
                = (storeTransform == null) ? new Transform() : storeTransform;
        Validate.nonNull(storeInertia, "storage for inertia");

        long shapeId = getObjectId();
        calculatePrincipalAxisTransform(shapeId, masses, result, storeInertia);

        return result;
    }

    /**
     * Remove a child CollisionShape from this shape.
     *
     * @param childShape the collision shape to remove (not null)
     */
    public void removeChildShape(CollisionShape childShape) {
        long childId = childShape.getObjectId();
        long parentId = getObjectId();
        removeChildShape(parentId, childId);

        for (Iterator<ChildCollisionShape> it = children.iterator();
                it.hasNext();) {
            ChildCollisionShape childCollisionShape = it.next();
            if (childCollisionShape.getShape() == childShape) {
                it.remove();
            }
        }
    }

    /**
     * Alter the local transform of the specified child CollisionShape. The
     * transform's scale is ignored.
     *
     * @param childShape the child's CollisionShape (not null, unaffected)
     * @param transform the desired Transform (not null, unaffected)
     */
    public void setChildTransform(CollisionShape childShape,
            Transform transform) {
        long childId = childShape.getObjectId();
        long parentId = getObjectId();
        Vector3f offset = transform.getTranslation();

        int childIndex = findIndex(childShape);
        assert childIndex >= 0 : childIndex;
        assert childIndex < children.size();

        Matrix3f rotation = transform.getRotation().toRotationMatrix();
        setChildTransform(parentId, childId, offset, rotation);

        ChildCollisionShape child = children.get(childIndex);
        child.setTransform(offset, rotation);
    }
    // *************************************************************************
    // CollisionShape methods

    /**
     * Test whether the specified scale factors can be applied to this shape.
     *
     * @param scale the desired scale factor for each local axis (may be null,
     * unaffected)
     * @return true if applicable, otherwise false
     */
    @Override
    public boolean canScale(Vector3f scale) {
        boolean result = super.canScale(scale);

        if (result) {
            for (ChildCollisionShape child : children) {
                CollisionShape childShape = child.getShape();
                if (!childShape.canScale(scale)) {
                    result = false;
                    break;
                }
            }
        }

        return result;
    }

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned shape into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this shape (not null)
     * @param original the instance from which this shape was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);

        children = cloner.clone(children);
        createEmpty(children.size());
        loadChildren();
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public CompoundCollisionShape jmeClone() {
        try {
            CompoundCollisionShape clone
                    = (CompoundCollisionShape) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }

    /**
     * Estimate how far this shape extends from its center.
     *
     * @return a distance estimate (in physics-space units, &ge;0)
     */
    @Override
    public float maxRadius() {
        float result = 0f;
        Transform tmpTransform = new Transform();

        for (ChildCollisionShape child : children) {
            CollisionShape childShape = child.getShape();
            child.copyOffset(tmpTransform.getTranslation());
            tmpTransform.getTranslation().multLocal(scale);
            child.copyRotation(tmpTransform.getRotation());
            float childRadius = DebugShapeFactory.maxDistance(childShape,
                    tmpTransform, DebugShapeFactory.lowResolution);
            if (childRadius > result) {
                result = childRadius;
            }
        }

        return result;
    }

    /**
     * De-serialize this shape from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    @SuppressWarnings("unchecked")
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);
        InputCapsule capsule = importer.getCapsule(this);

        children = capsule.readSavableArrayList(tagChildren, null);
        loadChildren();
    }

    /**
     * Recalculate this shape's bounding box if necessary.
     */
    @Override
    protected void recalculateAabb() {
        long nativeId = getObjectId();
        recalcAabb(nativeId);
    }

    /**
     * Alter the scale of this shape and its children. CAUTION: Not all shapes
     * can be scaled arbitrarily.
     * <p>
     * Note that if shapes are shared (between collision objects and/or compound
     * shapes) changes can have unintended consequences.
     *
     * @param scale the desired scale factor for each local axis (not null, no
     * negative component, unaffected, default=(1,1,1))
     */
    @Override
    public void setScale(Vector3f scale) {
        super.setScale(scale);
        /*
         * Update the children to keep their copied scale factors
         * in synch with the native ones.
         */
        for (ChildCollisionShape child : children) {
            CollisionShape childShape = child.getShape();
            childShape.updateScale();
        }
    }

    /**
     * Serialize this shape to the specified exporter, for example when saving
     * to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        super.write(exporter);
        OutputCapsule capsule = exporter.getCapsule(this);
        capsule.writeSavableArrayList(children, tagChildren, null);
    }
    // *************************************************************************
    // private methods

    /**
     * Instantiate an empty btCompoundShape.
     */
    private void createEmpty(int initialCapacity) {
        boolean enableAabbTree = true;
        long shapeId = createShape2(enableAabbTree, initialCapacity);
        setNativeId(shapeId);

        setScale(scale);
        setMargin(margin);
    }

    /**
     * Add the configured children to the empty btCompoundShape.
     */
    private void loadChildren() {
        long parentId = getObjectId();

        for (ChildCollisionShape child : children) {
            addChildShape(parentId, child.getShape().getObjectId(),
                    child.copyOffset(null), child.copyRotationMatrix(null));
        }
    }
    // *************************************************************************
    // native methods

    native private void addChildShape(long compoundId, long childShapeId,
            Vector3f offset, Matrix3f rotation);

    native private void calculatePrincipalAxisTransform(long shapeId,
            FloatBuffer massBuffer, Transform storeTransform,
            Vector3f storeInertia);

    native private int countChildren(long shapeId);

    native private long createShape2(boolean dynamicAabbTree,
            int initialChildCapacity);

    native private void recalcAabb(long shapeId);

    native private void removeChildShape(long compoundId, long childShapeId);

    native private void setChildTransform(long compoundId, long childShapeId,
            Vector3f offset, Matrix3f rotation);
}
