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

import com.jme3.bounding.BoundingBox;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import com.jme3.util.clone.JmeCloneable;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * The abstract base class for collision shapes based on Bullet's
 * btCollisionShape.
 * <p>
 * Subclasses include BoxCollisionShape and CapsuleCollisionShape. As suggested
 * in the Bullet manual, a single CollisionShape can be shared among multiple
 * collision objects.
 *
 * @author normenhansen
 */
abstract public class CollisionShape
        implements Comparable<CollisionShape>, JmeCloneable, Savable {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(CollisionShape.class.getName());
    // *************************************************************************
    // fields

    /**
     * default margin for new non-sphere/non-capsule shapes (in physics-space
     * units, &gt;0, default=0.04)
     */
    private static float defaultMargin = 0.04f;
    /**
     * copy of collision margin (in physics-space units, &gt;0, default=0.04)
     */
    protected float margin = defaultMargin;
    /**
     * unique identifier of the btCollisionShape
     * <p>
     * Constructors are responsible for setting this to a non-zero value. After
     * that, the ID never changes.
     */
    protected long objectId = 0L;
    /**
     * copy of scaling factors: one for each local axis (default=(1,1,1))
     */
    protected Vector3f scale = new Vector3f(1f, 1f, 1f);
    // *************************************************************************
    // new methods exposed

    /**
     * Calculate an axis-aligned bounding box for this shape with the specified
     * translation and rotation applied to it. Rotation is applied first.
     *
     * @param translation the translation to apply (not null, unaffected)
     * @param rotation the rotation to apply (not null, unaffected)
     * @param storeResult storage for the result (modified if not null)
     * @return a bounding box (either storeResult or a new instance, not null)
     */
    public BoundingBox boundingBox(Vector3f translation, Matrix3f rotation,
            BoundingBox storeResult) {
        Validate.nonNull(translation, "translation");
        Validate.nonNull(rotation, "rotation");
        BoundingBox result
                = (storeResult == null) ? new BoundingBox() : storeResult;

        Vector3f maxima = new Vector3f();
        Vector3f minima = new Vector3f();
        getAabb(objectId, translation, rotation, minima, maxima);
        result.setMinMax(minima, maxima);

        return result;
    }

    /**
     * Calculate an axis-aligned bounding box for this shape with the specified
     * translation and rotation applied to it. Rotation is applied first.
     *
     * @param translation the translation to apply (not null, unaffected)
     * @param rotation the rotation to apply (not null, unaffected)
     * @param storeResult storage for the result (modified if not null)
     * @return a bounding box (either storeResult or a new instance, not null)
     */
    public BoundingBox boundingBox(Vector3f translation, Quaternion rotation,
            BoundingBox storeResult) {
        Validate.nonNull(translation, "translation");
        Validate.nonNull(rotation, "rotation");
        BoundingBox result
                = (storeResult == null) ? new BoundingBox() : storeResult;

        Matrix3f basisMatrix = new Matrix3f();
        basisMatrix.set(rotation);
        Vector3f maxima = new Vector3f();
        Vector3f minima = new Vector3f();
        getAabb(objectId, translation, basisMatrix, minima, maxima);
        result.setMinMax(minima, maxima);

        return result;
    }

    /**
     * Test whether the specified scaling factors can be applied to this shape.
     *
     * @param scale the desired scaling factor for each local axis (may be null,
     * unaffected)
     * @return true if applicable, otherwise false
     */
    public boolean canScale(Vector3f scale) {
        boolean result;
        if (scale == null) {
            result = false;
        } else {
            result = MyVector3f.isAllNonNegative(scale);
        }

        return result;
    }

    /**
     * Read the default margin for new shapes that are neither capsules nor
     * spheres.
     *
     * @return the margin distance (in physics-space units, &gt;0)
     */
    public static float getDefaultMargin() {
        assert defaultMargin > 0f : defaultMargin;
        return defaultMargin;
    }

    /**
     * Read the collision margin for this shape.
     *
     * @return the margin distance (in physics-space units, &ge;0)
     */
    public float getMargin() {
        assert margin > 0f : margin;
        assert margin == getMargin(objectId);
        return margin;
    }

    /**
     * Read the ID of the btCollisionShape.
     *
     * @return the unique identifier (not zero)
     */
    public long getObjectId() {
        assert objectId != 0L;
        return objectId;
    }

    /**
     * Copy the scaling factors.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the scaling factor for each local axis (either storeResult or a
     * new vector, not null)
     */
    public Vector3f getScale(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        assert checkScale(result);
        result.set(scale);
        return result;
    }

    /**
     * Test whether this shape is concave.
     *
     * @return true if concave, false if convex
     */
    public boolean isConcave() {
        boolean result = isConcave(objectId);
        return result;
    }

    /**
     * Alter the default margin for new shapes that are neither capsules nor
     * spheres.
     *
     * @param margin the desired margin distance (in physics-space units, &gt;0,
     * default=0.04)
     */
    public static void setDefaultMargin(float margin) {
        Validate.positive(margin, "margin");
        defaultMargin = margin;
    }

    /**
     * Alter the collision margin of this shape. CAUTION: Margin is applied
     * differently, depending on the type of shape. Generally the collision
     * margin expands the object, creating a gap. Don't set the collision margin
     * to zero.
     * <p>
     * Note that if the shape is shared (between collision objects and/or
     * compound shapes) changes can have unintended consequences.
     *
     * @param margin the desired margin distance (in physics-space units, &gt;0,
     * default=0.04)
     */
    public void setMargin(float margin) {
        Validate.positive(margin, "margin");
        assert objectId != 0L;

        setMargin(objectId, margin);
        logger.log(Level.FINE, "Margining Shape {0}",
                Long.toHexString(objectId));
        this.margin = margin;
    }

    /**
     * Alter the scaling factors of this shape. CAUTION: Not all shapes can be
     * scaled arbitrarily.
     * <p>
     * Note that if the shape is shared (between collision objects and/or
     * compound shapes) changes can have unintended consequences.
     *
     * @param scale the desired scaling factor for each local axis (not null, no
     * negative component, unaffected, default=(1,1,1))
     */
    public void setScale(Vector3f scale) {
        Validate.nonNull(scale, "scale");
        if (!canScale(scale)) {
            String typeName = this.getClass().getCanonicalName();
            String msg = String.format("%s cannot be scaled to (%s,%s,%s)",
                    typeName, scale.x, scale.y, scale.z);
            throw new IllegalArgumentException(msg);
        }
        assert objectId != 0L;

        setLocalScaling(objectId, scale);
        logger.log(Level.FINE, "Scaling Shape {0}", Long.toHexString(objectId));
        this.scale.set(scale);
    }
    // *************************************************************************
    // Comparable methods

    /**
     * Compare (by ID) with the specified collision shape.
     *
     * @param other the other shape (not null, unaffected)
     * @return 0 if the shapes have the same ID; negative if this comes before
     * other; positive if this comes after other
     */
    @Override
    public int compareTo(CollisionShape other) {
        long otherId = other.getObjectId();
        int result = Long.compare(objectId, otherId);

        return result;
    }
    // *************************************************************************
    // JmeCloneable methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned shape into a deep-cloned one, using the specified cloner
     * and original to resolve copied fields.
     *
     * @param cloner the cloner that's cloning this shape (not null)
     * @param original the instance from which this shape was shallow-cloned
     * (unused)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        scale = cloner.clone(scale);
        objectId = 0L; // subclass must create the btCollisionShape
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public CollisionShape jmeClone() {
        try {
            CollisionShape clone = (CollisionShape) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }
    // *************************************************************************
    // Savable methods

    /**
     * De-serialize this shape from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        InputCapsule capsule = importer.getCapsule(this);

        Savable s = capsule.readSavable("scale", new Vector3f(1f, 1f, 1f));
        scale.set((Vector3f) s);
        margin = capsule.readFloat("margin", 0.04f);
        // subclass must create the btCollisionShape and apply margin and scale
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
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(scale, "scale", new Vector3f(1f, 1f, 1f));
        capsule.write(margin, "margin", 0.04f);
    }
    // *************************************************************************
    // Object methods

    /**
     * Finalize this shape just before it is destroyed. Should be invoked only
     * by a subclass or by the garbage collector.
     *
     * @throws Throwable ignored by the garbage collector
     */
    @Override
    protected void finalize() throws Throwable {
        super.finalize();
        logger.log(Level.FINE, "Finalizing Shape {0}",
                Long.toHexString(objectId));
        DebugShapeFactory.removeShapeFromCache(objectId);
        finalizeNative(objectId);
    }
    // *************************************************************************
    // private methods

    /**
     * Compare Bullet's scaling factors to the local copy.
     *
     * @param storeVector caller-allocated temporary storage (not null)
     * @return true if scaling factors are exactly equal, otherwise false
     */
    private boolean checkScale(Vector3f storeVector) {
        assert storeVector != null;

        getLocalScaling(objectId, storeVector);
        boolean result = scale.equals(storeVector);

        return result;
    }

    native private void finalizeNative(long objectId);

    native private void getAabb(long objectId, Vector3f location,
            Matrix3f basisMatrix, Vector3f storeMinima, Vector3f storeMaxima);

    native private void getLocalScaling(long objectId, Vector3f storeVector);

    native private float getMargin(long objectId);

    native private boolean isConcave(long objectId);

    native private void setLocalScaling(long objectId, Vector3f scale);

    native private void setMargin(long objectId, float margin);
}
