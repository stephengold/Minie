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

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.nio.FloatBuffer;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.MyVolume;

/**
 * A cylindrical collision shape based on Bullet's {@code btCylinderShapeX},
 * {@code btCylinderShape}, or {@code btCylinderShapeZ}.
 *
 * @author normenhansen
 */
public class CylinderCollisionShape extends ConvexShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(CylinderCollisionShape.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagAxis = "axis";
    final private static String tagHalfExtents = "halfExtents";
    // *************************************************************************
    // fields

    /**
     * copy of main (height) axis (0&rarr;X, 1&rarr;Y, 2&rarr;Z)
     */
    private int axis;
    /**
     * copy of the unscaled half extent for each local axis (not null, no
     * negative component)
     */
    private Vector3f halfExtents = new Vector3f(0.5f, 0.5f, 0.5f);
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected CylinderCollisionShape() {
    }

    /**
     * Instantiate a cylinder shape around the specified main (height) axis.
     *
     * @param radius the desired unscaled radius (&ge;0)
     * @param height the desired unscaled height (&ge;0)
     * @param axisIndex which local axis to use for the height: 0&rarr;X,
     * 1&rarr;Y, 2&rarr;Z (default=2)
     */
    public CylinderCollisionShape(float radius, float height, int axisIndex) {
        Validate.nonNegative(radius, "radius");
        Validate.nonNegative(height, "height");
        Validate.axisIndex(axisIndex, "axis index");

        this.axis = axisIndex;
        halfExtents.set(radius, radius, radius);
        halfExtents.set(axisIndex, height / 2f);
        createShape();
    }

    /**
     * Instantiate a cylinder shape that encloses the sample locations in the
     * specified FloatBuffer range.
     *
     * @param buffer the buffer that contains the sample locations (not null,
     * unaffected)
     * @param startPosition the position at which the sample locations start
     * (&ge;0, &le;endPosition)
     * @param endPosition the position at which the sample locations end
     * (&ge;startPosition, &le;capacity)
     * @param axisIndex which local axis to use for the height: 0&rarr;X,
     * 1&rarr;Y, 2&rarr;Z
     */
    public CylinderCollisionShape(FloatBuffer buffer, int startPosition,
            int endPosition, int axisIndex) {
        Validate.nonNull(buffer, "buffer");
        Validate.inRange(startPosition, "start position", 0, endPosition);
        Validate.inRange(
                endPosition, "end position", startPosition, buffer.capacity());
        Validate.axisIndex(axisIndex, "axis index");

        this.axis = axisIndex;
        MyBuffer.maxAbs(buffer, startPosition, endPosition, halfExtents);
        float halfHeight = halfExtents.get(axisIndex);

        float radius = MyBuffer
                .cylinderRadius(buffer, startPosition, endPosition, axisIndex);
        halfExtents.set(radius, radius, radius);
        halfExtents.set(axisIndex, halfHeight);
        createShape();
    }

    /**
     * Instantiate a Z-axis cylinder shape with the specified half extents.
     *
     * @param halfExtents the desired unscaled half extents (not null, no
     * negative component, unaffected)
     */
    public CylinderCollisionShape(Vector3f halfExtents) {
        Validate.nonNegative(halfExtents, "half extents");

        this.halfExtents.set(halfExtents);
        this.axis = PhysicsSpace.AXIS_Z;
        createShape();
    }

    /**
     * Instantiate a cylinder shape around the specified axis.
     *
     * @param halfExtents the desired unscaled half extents (not null, no
     * negative component, unaffected)
     * @param axisIndex which local axis to use for the height: 0&rarr;X,
     * 1&rarr;Y, 2&rarr;Z
     */
    public CylinderCollisionShape(Vector3f halfExtents, int axisIndex) {
        Validate.nonNegative(halfExtents, "half extents");
        Validate.axisIndex(axisIndex, "axis index");

        this.halfExtents.set(halfExtents);
        this.axis = axisIndex;
        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Return the main (height) axis of the cylinder.
     *
     * @return the axis index: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     */
    public int getAxis() {
        assert axis == PhysicsSpace.AXIS_X
                || axis == PhysicsSpace.AXIS_Y
                || axis == PhysicsSpace.AXIS_Z : axis;
        return axis;
    }

    /**
     * Copy the half extents of the cylinder.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the unscaled half extent for each local axis (either storeResult
     * or a new vector, not null, no negative component)
     */
    public Vector3f getHalfExtents(Vector3f storeResult) {
        assert MyVector3f.isAllNonNegative(halfExtents) : halfExtents;

        Vector3f result;
        if (storeResult == null) {
            result = halfExtents.clone();
        } else {
            result = storeResult.set(halfExtents);
        }
        return result;
    }

    /**
     * Return the height of the cylinder.
     *
     * @return the unscaled height (&ge;0)
     */
    public float getHeight() {
        float result = 2f * halfExtents.get(axis);

        assert result >= 0f : result;
        return result;
    }

    /**
     * Return the unscaled volume of the cylinder.
     *
     * @return the volume (in shape units cubed, &ge;0)
     */
    public float unscaledVolume() {
        float result = MyVolume.cylinderVolume(halfExtents);

        assert result >= 0f : result;
        return result;
    }
    // *************************************************************************
    // ConvexShape methods

    /**
     * Test whether the specified scale factors can be applied to this shape.
     * For cylinder shapes, radial scaling must be uniform.
     *
     * @param scale the desired scale factor for each local axis (may be null,
     * unaffected)
     * @return true if applicable, otherwise false
     */
    @Override
    public boolean canScale(Vector3f scale) {
        boolean canScale = super.canScale(scale);
        if (canScale) {
            if (axis == PhysicsSpace.AXIS_X && scale.y != scale.z) {
                canScale = false;
            } else if (axis == PhysicsSpace.AXIS_Y && scale.x != scale.z) {
                canScale = false;
            } else if (axis == PhysicsSpace.AXIS_Z && scale.x != scale.y) {
                canScale = false;
            }
        }

        return canScale;
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
        this.halfExtents = cloner.clone(halfExtents);
        createShape();
    }

    /**
     * Estimate how far the cylinder extends from its center.
     *
     * @return a distance (in physics-space units, &ge;0)
     */
    @Override
    public float maxRadius() {
        double xx = scale.x * halfExtents.x;
        double yy = scale.y * halfExtents.y;
        double zz = scale.z * halfExtents.z;

        double bigRadius;
        double halfHeight;
        switch (axis) {
            case PhysicsSpace.AXIS_X:
                halfHeight = xx;
                bigRadius = Math.max(yy, zz);
                break;
            case PhysicsSpace.AXIS_Y:
                halfHeight = yy;
                bigRadius = Math.max(xx, zz);
                break;
            case PhysicsSpace.AXIS_Z:
                halfHeight = zz;
                bigRadius = Math.max(xx, yy);
                break;
            default:
                throw new IllegalStateException("axis = " + axis);
        }
        float result = (float) Math.hypot(halfHeight, bigRadius);

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
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);
        InputCapsule capsule = importer.getCapsule(this);

        Vector3f he = (Vector3f) capsule.readSavable(
                tagHalfExtents, new Vector3f(0.5f, 0.5f, 0.5f));
        halfExtents.set(he);
        this.axis = capsule.readInt(tagAxis, PhysicsSpace.AXIS_Y);
        createShape();
    }

    /**
     * Approximate this shape with a HullCollisionShape.
     *
     * @return a new shape
     */
    @Override
    public HullCollisionShape toHullShape() {
        Vector3f hes = halfExtents.mult(scale);
        float minHalfExtent = MyMath.min(hes.x, hes.y, hes.z); // in PSU
        float defaultMargin = getDefaultMargin();
        float hullMargin = Math.min(defaultMargin, minHalfExtent);
        /*
         * Construct a copy of this shape with its half extents reduced
         * to compensate for the hull's collision margin.
         */
        hes.subtractLocal(hullMargin, hullMargin, hullMargin);
        MyVector3f.accumulateMaxima(hes, new Vector3f(1e-6f, 1e-6f, 1e-6f));
        CylinderCollisionShape reducedShape
                = new CylinderCollisionShape(hes, axis);
        FloatBuffer buffer = DebugShapeFactory
                .debugVertices(reducedShape, DebugShapeFactory.lowResolution);

        // Flip the buffer.
        buffer.rewind();
        buffer.limit(buffer.capacity());

        HullCollisionShape result = new HullCollisionShape(buffer);
        result.setMargin(hullMargin);

        return result;
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

        capsule.write(halfExtents, tagHalfExtents, null);
        capsule.write(axis, tagAxis, PhysicsSpace.AXIS_Y);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured btCollisionShape.
     */
    private void createShape() {
        assert axis == PhysicsSpace.AXIS_X
                || axis == PhysicsSpace.AXIS_Y
                || axis == PhysicsSpace.AXIS_Z : axis;
        assert MyVector3f.isAllNonNegative(halfExtents) : halfExtents;

        long shapeId = createShape(axis, halfExtents);
        setNativeId(shapeId);

        setContactFilterEnabled(enableContactFilter);
        setScale(scale);
        setMargin(margin);
    }
    // *************************************************************************
    // native private methods

    native private static long createShape(int axisIndex, Vector3f halfExtents);
}
