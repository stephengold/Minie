/*
 * Copyright (c) 2024 jMonkeyEngine
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
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * A collision shape for a spherical segment with uniform density, based on
 * {@code btConvexInternalShape}. By convention, both bases are orthogonal to
 * the local Y axis.
 * <p>
 * This is an imprecise shape; margin always expands the shape.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class SphericalSegment extends ConvexShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(SphericalSegment.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagUnscaledRadius = "radius";
    final private static String tagUnscaledYMax = "yMax";
    final private static String tagUnscaledYMin = "yMin";
    // *************************************************************************
    // fields

    /**
     * radius of the parent sphere, for scale=(1,1,1) and margin=0
     */
    private float unscaledRadius;
    /**
     * Y offset of the upper base from the center of the parent sphere (for
     * scale=(1,1,1) and margin=0)
     */
    private float unscaledYMax;
    /**
     * Y offset of the lower base from the center of the parent sphere (for
     * scale=(1,1,1) and margin=0)
     */
    private float unscaledYMin;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected SphericalSegment() {
    }

    /**
     * Instantiate a hemisphere with the specified radius.
     *
     * @param radius the desired radius, before scaling and excluding margin
     * (&gt;0)
     */
    public SphericalSegment(float radius) {
        this(radius, radius, 0f);
    }

    /**
     * Instantiate a spherical segment with the specified dimensions.
     *
     * @param radius the desired radius of the parent sphere, before scaling and
     * excluding margin (&gt;0)
     * @param yMax the Y offset of the upper base from the center of the parent
     * sphere, before scaling and excluding margin (&ge;yMin, &le;radius)
     * @param yMin the Y offset of the lower base from the center of the parent
     * sphere, before scaling and excluding margin (&ge;-radius, &le;yMax)
     */
    public SphericalSegment(float radius, float yMax, float yMin) {
        Validate.positive(radius, "radius");
        Validate.inRange(yMax, "yMax", yMin, radius);
        Validate.inRange(yMin, "yMin", -radius, yMax);

        this.unscaledRadius = radius;
        this.unscaledYMax = yMax;
        this.unscaledYMin = yMin;

        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Return the height of the segment.
     *
     * @return the unscaled height (&ge;0)
     */
    public float getHeight() {
        float result = unscaledYMax - unscaledYMin;

        assert result >= 0f : result;
        return result;
    }

    /**
     * Return the radius of the parent sphere.
     *
     * @return the unscaled radius (&gt;0)
     */
    public float sphereRadius() {
        assert unscaledRadius > 0f : unscaledRadius;
        return unscaledRadius;
    }

    /**
     * Return the Y offset of the upper base from the center of the parent
     * sphere.
     *
     * @return the unscaled offset (&ge;-radius, &le;radius)
     */
    public float yMax() {
        assert unscaledYMax >= -unscaledRadius;
        assert unscaledYMax <= unscaledRadius;
        return unscaledYMax;
    }

    /**
     * Return the Y offset of the lower base from the center of the parent
     * sphere.
     *
     * @return the unscaled offset (&ge;-radius, &le;radius)
     */
    public float yMin() {
        assert unscaledYMin >= -unscaledRadius;
        assert unscaledYMin <= unscaledRadius;
        return unscaledYMin;
    }
    // *************************************************************************
    // ConvexShape methods

    /**
     * Test whether the specified scale factors can be applied to the shape. For
     * a spherical segment, scaling must be uniform.
     *
     * @param scale the desired scale factor for each local axis (may be null,
     * unaffected)
     * @return true if applicable, otherwise false
     */
    @Override
    public boolean canScale(Vector3f scale) {
        boolean canScale
                = super.canScale(scale) && MyVector3f.isScaleUniform(scale);
        return canScale;
    }

    /**
     * Calculate how far the scaled shape extends from its center of mass,
     * including collision margin.
     *
     * @return the distance (in physics-space units, &ge;0)
     */
    @Override
    public float maxRadius() {
        long shapeId = nativeId();
        float result = maxRadius(shapeId);
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

        this.unscaledRadius = capsule.readFloat(tagUnscaledRadius, 1f);
        this.unscaledYMax = capsule.readFloat(tagUnscaledYMax, 1f);
        this.unscaledYMin = capsule.readFloat(tagUnscaledYMin, 0f);

        setScale(scale);
    }

    /**
     * Estimate the volume of the collision shape, including scale and margin.
     *
     * @return the estimated volume (in physics-space units cubed, &ge;0)
     */
    @Override
    public float scaledVolume() {
        long shapeId = nativeId();
        float result = scaledVolume(shapeId);
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

        capsule.write(unscaledRadius, tagUnscaledRadius, 1f);
        capsule.write(unscaledYMax, tagUnscaledYMax, 1f);
        capsule.write(unscaledYMin, tagUnscaledYMin, 0f);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured shape in Bullet.
     */
    private void createShape() {
        long shapeId
                = createShapeNative(unscaledRadius, unscaledYMax, unscaledYMin);
        setNativeId(shapeId);

        setContactFilterEnabled(enableContactFilter);
        setScale(scale);
        setMargin(margin);
    }
    // *************************************************************************
    // native private methods

    native private static long createShapeNative(
            float radius, float yMax, float yMin);

    native private static float maxRadius(long shapeId);

    native private static float scaledVolume(long shapeId);
}
