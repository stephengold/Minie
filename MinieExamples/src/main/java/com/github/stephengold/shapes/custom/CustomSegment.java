/*
 Copyright (c) 2024 Stephen Gold
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package com.github.stephengold.shapes.custom;

import com.jme3.bullet.collision.shapes.CustomConvexShape;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;

/**
 * A collision shape for a spherical segment with uniform density. By
 * convention, both bases are orthogonal to the local Y axis.
 * <p>
 * This is an imprecise shape; margin always expands the shape.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class CustomSegment extends CustomConvexShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger loggerZ
            = Logger.getLogger(CustomSegment.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagUnscaledRadius = "radius";
    final private static String tagUnscaledYMax = "yMax";
    final private static String tagUnscaledYMin = "yMin";
    // *************************************************************************
    // fields

    /**
     * square of {@code scaledRadius}
     */
    private float scaledR2;
    /**
     * scaled radius of the parent sphere, excluding margin (in physics-space
     * units)
     */
    private float scaledRadius;
    /**
     * Y offset of the segment's center of mass from the center of the parent
     * sphere (in physics-space units)
     */
    private float scaledY0;
    /**
     * Y offset of the upper base from the center of the parent sphere (in
     * physics-space units)
     */
    private float scaledYMax;
    /**
     * Y offset of the lower base from the center of the parent sphere (in
     * physics-space units)
     */
    private float scaledYMin;
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
    protected CustomSegment() {
    }

    /**
     * Instantiate a hemisphere with the specified radius.
     *
     * @see com.github.stephengold.shapes.custom.CustomHemisphere
     *
     * @param radius the desired radius, before scaling and excluding margin
     * (&gt;0)
     */
    public CustomSegment(float radius) {
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
    public CustomSegment(float radius, float yMax, float yMin) {
        super(halfExtents(radius, yMax, yMin));

        Validate.positive(radius, "radius");
        Validate.inRange(yMax, "yMax", yMin, radius);
        Validate.inRange(yMin, "yMin", -radius, yMax);

        this.unscaledRadius = radius;
        this.unscaledYMax = yMax;
        this.unscaledYMin = yMin;
        setScale(scale);
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
    // CustomConvexShape methods

    /**
     * Test whether the specified scale factors can be applied to this shape.
     * For a spherical segment, scaling must be uniform.
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
     * Locate the shape's supporting vertex for the specified normal direction,
     * excluding collision margin.
     * <p>
     * This method is invoked by native code.
     *
     * @param dirX the X-coordinate of the direction to test (in scaled shape
     * coordinates)
     * @param dirY the Y-coordinate of the direction to test (in scaled shape
     * coordinates)
     * @param dirZ the Z-coordinate of the direction to test (in scaled shape
     * coordinates)
     * @return the location on the shape's surface with the specified normal (in
     * scaled shape coordinates, must lie on or within the shape's bounding box)
     */
    @Override
    protected Vector3f locateSupport(float dirX, float dirY, float dirZ) {
        Vector3f result = threadTmpVector.get();

        float y; // relative to the center of the parent sphere
        float dxyz = MyMath.hypotenuse(dirX, dirY, dirZ);
        if (dxyz == 0f) {
            y = 0.5f * (scaledYMax + scaledYMin);
        } else {
            y = scaledRadius * (dirY / dxyz);
            y = FastMath.clamp(y, scaledYMin, scaledYMax);
        }

        // The distance from the local Y axis, in physics-space units:
        float rxz = FastMath.sqrt(scaledR2 - y * y);

        float dxz = MyMath.hypotenuse(dirX, dirZ);
        if (dxz == 0f) {
            result.set(rxz, y - scaledY0, 0f);
        } else {
            result.x = rxz * (dirX / dxz);
            result.y = y - scaledY0;
            result.z = rxz * (dirZ / dxz);
        }

        return result;
    }

    /**
     * Calculate how far the scaled shape extends from its center of mass,
     * excluding collision margin.
     *
     * @return the distance (in physics-space units, &ge;0)
     */
    @Override
    public float maxRadius() {
        float yyMax = scaledYMax - scaledY0;
        float d2YMax = yyMax * yyMax + scaledR2 - scaledYMax * scaledYMax;

        float yyMin = scaledYMin - scaledY0;
        float d2YMin = yyMin * yyMin + scaledR2 - scaledYMin * scaledYMin;

        float d2 = Math.max(d2YMax, d2YMin);
        if (scaledYMax > 0f && scaledYMin < 0f) {
            float d2Equator = scaledY0 * scaledY0 + scaledR2;
            d2 = Math.max(d2, d2Equator);
        }

        float result = FastMath.sqrt(d2);
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
        float yMin = scaledYMin - margin;
        float yMax = scaledYMax + margin;
        float height = yMax - yMin;
        float h3 = MyMath.cube(yMax) - MyMath.cube(yMin); // difference of cubes
        float radius = scaledRadius + margin;
        float r2 = radius * radius;
        float denom = r2 * height - h3 / 3f; // proportional to volume
        float result = FastMath.PI * denom;

        return result;
    }

    /**
     * Alter the scale of the shape.
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

        // super.setScale() has verified that the scaling is uniform.
        this.scaledRadius = scale.x * unscaledRadius;
        this.scaledR2 = scaledRadius * scaledRadius;
        this.scaledYMax = scale.x * unscaledYMax;
        this.scaledYMin = scale.x * unscaledYMin;
        this.scaledY0 = y0(scaledRadius, scaledYMax, scaledYMin);

        float max2 = scaledYMax * scaledYMax;
        float max3 = max2 * scaledYMax;
        float max5 = max2 * max3;

        float min2 = scaledYMin * scaledYMin;
        float min3 = min2 * scaledYMin;
        float min5 = min2 * min3;

        float height = scaledYMax - scaledYMin;
        float h3 = max3 - min3; // difference of cubes
        float h5 = max5 - min5;

        float r4h = scaledR2 * scaledR2 * height;
        float denom = scaledR2 * height - h3 / 3f; // proportional to volume
        /*
         * the moments of inertia of a uniformly dense spherical segment
         * with mass=1, around its center of mass:
         */
        float ixz = (3f * r4h + 2f * scaledR2 * h3 - 1.8f * h5) / (12f * denom)
                - scaledY0 * scaledY0;
        float iy = (3f * r4h - 2f * scaledR2 * h3 + 0.6f * h5) / (6f * denom);
        setScaledInertia(ixz, iy, ixz);
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
    // private methods

    /**
     * Return the half extents of a spherical segment around its center of mass.
     *
     * @param radius the radius of the parent sphere (&gt;0)
     * @param yMax the Y offset of the upper base (&ge;yMin, &le;radius)
     * @param yMin the Y offset of the lower base (&ge;-radius, &le;yMax)
     * @return a new vector with all components &ge;0
     */
    private static Vector3f halfExtents(float radius, float yMax, float yMin) {
        float y0 = y0(radius, yMax, yMin); // the offset of the center of mass

        float yyMax = yMax - y0;
        assert yyMax >= 0f : yyMax;
        float yyMin = yMin - y0;
        assert yyMin <= 0f : yyMin;
        float maxAbsYy = Math.max(yyMax, -yyMin);

        float maxR;
        if (yMax >= 0f && yMin <= 0f) {
            // The segment includes the parent sphere's equator:
            maxR = radius;

        } else { // The segment doesn't include the parent sphere's equator:
            float max2 = yMax * yMax;
            float min2 = yMin * yMin;
            float r2 = radius * radius;

            float minYSquared = Math.min(max2, min2);
            float maxRSquared = r2 - minYSquared;
            maxR = FastMath.sqrt(maxRSquared);
        }

        Vector3f result = new Vector3f(maxR, maxAbsYy, maxR);
        return result;
    }

    /**
     * Return the Y offset of the segment's center of mass from the center of
     * the parent sphere.
     *
     * @param radius the radius of the parent sphere (&gt;0)
     * @param yMax the Y offset of the upper base (&ge;yMin, &le;radius)
     * @param yMin the Y offset of the lower base (&ge;-radius, &le;yMax)
     * @return the offset (&le;radius, &ge;-radius)
     */
    private static float y0(float radius, float yMax, float yMin) {
        assert radius > 0f : radius;
        assert yMax <= radius : yMax + ">" + radius;
        assert yMin <= yMax : yMin + ">" + yMax;
        assert -radius <= yMin : -radius + ">" + yMin;

        float max2 = yMax * yMax;
        float min2 = yMin * yMin;
        float r2 = radius * radius;

        float squares = max2 + yMax * yMin + min2;
        float denominator = 3f * r2 - squares;
        assert denominator != 0f;

        float sum = yMax + yMin;
        float cubes = (max2 + min2) * sum;
        float numerator = 2f * r2 * sum - cubes;

        float result = 0.75f * numerator / denominator;

        return result;
    }
}
