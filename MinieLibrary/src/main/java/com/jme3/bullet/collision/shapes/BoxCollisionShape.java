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
import jme3utilities.math.RectangularSolid;

/**
 * An axis-aligned, rectangular-solid collision shape based on Bullet's
 * {@code btBoxShape}. For a rectangle, use Box2dShape.
 *
 * @author normenhansen
 */
public class BoxCollisionShape extends ConvexShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(BoxCollisionShape.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagHalfExtents = "halfExtents";
    // *************************************************************************
    // fields

    /**
     * copy of the unscaled half extents (in shape units, not null, no negative
     * component)
     */
    private Vector3f halfExtents = new Vector3f(1f, 1f, 1f);
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected BoxCollisionShape() {
    }

    /**
     * Instantiate a cube-shaped box with the specified half extent.
     *
     * @param halfExtent the desired half extent on each local axis (in shape
     * units, not negative)
     */
    public BoxCollisionShape(float halfExtent) {
        Validate.nonNegative(halfExtent, "half extent");

        halfExtents.set(halfExtent, halfExtent, halfExtent);
        createShape();
    }

    /**
     * Instantiate a box shape with the specified half extents.
     *
     * @param xHalfExtent the desired half extent on the local X axis (in shape
     * units, not negative)
     * @param yHalfExtent the desired half extent on the local Y axis (in shape
     * units, not negative)
     * @param zHalfExtent the desired half extent on the local Z axis (in shape
     * units, not negative)
     */
    public BoxCollisionShape(float xHalfExtent, float yHalfExtent,
            float zHalfExtent) {
        Validate.nonNegative(xHalfExtent, "half extent on X");
        Validate.nonNegative(yHalfExtent, "half extent on Y");
        Validate.nonNegative(zHalfExtent, "half extent on Z");

        halfExtents.set(xHalfExtent, yHalfExtent, zHalfExtent);
        createShape();
    }

    /**
     * Instantiate a box shape that encloses the sample locations in the
     * specified FloatBuffer range.
     *
     * @param buffer the buffer that contains the sample locations (not null,
     * unaffected)
     * @param startPosition the position at which the sample locations start
     * (&ge;0, &le;endPosition)
     * @param endPosition the position at which the sample locations end
     * (&ge;startPosition, &le;capacity)
     */
    public BoxCollisionShape(FloatBuffer buffer, int startPosition,
            int endPosition) {
        Validate.nonNull(buffer, "buffer");
        Validate.inRange(startPosition, "start position", 0, endPosition);
        Validate.inRange(endPosition, "end position", startPosition,
                buffer.capacity());

        MyBuffer.maxAbs(buffer, startPosition, endPosition, halfExtents);
        createShape();
    }

    /**
     * Instantiate a box shape with the specified half extents.
     *
     * @param halfExtents the desired half extents (in shape units, not null, no
     * negative component, unaffected)
     */
    public BoxCollisionShape(Vector3f halfExtents) {
        Validate.nonNegative(halfExtents, "half extents");

        this.halfExtents.set(halfExtents);
        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the half extents of the box.
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
     * Return the unscaled volume of the box.
     *
     * @return the volume (in shape units cubed, &ge;0)
     */
    public float unscaledVolume() {
        float result = MyVolume.boxVolume(halfExtents);

        assert result >= 0f : result;
        return result;
    }
    // *************************************************************************
    // ConvexShape methods

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
     * Calculate how far the box extends from its center.
     *
     * @return the distance (in physics-space units, &ge;0)
     */
    @Override
    public float maxRadius() {
        double xx = scale.x * halfExtents.x;
        double yy = scale.y * halfExtents.y;
        double zz = scale.z * halfExtents.z;
        float result = (float) MyMath.hypotenuseDouble(xx, yy, zz);

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

        Vector3f he = (Vector3f) capsule.readSavable(tagHalfExtents,
                new Vector3f(1f, 1f, 1f));
        halfExtents.set(he);
        createShape();
    }

    /**
     * Estimate the volume of this shape, including scale and margin.
     *
     * @return the volume (in physics-space units cubed, &ge;0)
     */
    @Override
    public float scaledVolume() {
        float result = unscaledVolume() * scale.x * scale.y * scale.z;
        return result;
    }

    /**
     * Approximate this shape with a HullCollisionShape.
     *
     * @return a new shape
     */
    @Override
    public HullCollisionShape toHullShape() {
        Vector3f hes = scale.mult(halfExtents); // in PSU
        float minHalfExtent = MyMath.min(hes.x, hes.y, hes.z);
        float defaultMargin = getDefaultMargin();
        float hullMargin = Math.min(minHalfExtent, defaultMargin);
        if (hullMargin <= 1e-9f) {
            hullMargin = 1e-9f;
        }

        hes.subtractLocal(hullMargin, hullMargin, hullMargin);
        RectangularSolid shrunkenSolid = new RectangularSolid(hes);
        HullCollisionShape result = new HullCollisionShape(shrunkenSolid);
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
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured {@code btBoxShape}.
     */
    private void createShape() {
        assert MyVector3f.isAllNonNegative(halfExtents) : halfExtents;

        long shapeId = createShape(halfExtents);
        setNativeId(shapeId);

        setContactFilterEnabled(enableContactFilter);
        setScale(scale);
        setMargin(margin);
    }
    // *************************************************************************
    // native private methods

    native private static long createShape(Vector3f halfExtents);
}
