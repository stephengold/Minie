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
import jme3utilities.math.MyVolume;

/**
 * A collision shape for a hemisphere with uniform density. By convention, the
 * apex lies on the local +Y axis.
 * <p>
 * This is an imprecise shape; margin always expands the shape.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class CustomHemisphere extends CustomConvexShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger loggerZ
            = Logger.getLogger(CustomHemisphere.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagUnscaledRadius = "unscaledRadius";
    // *************************************************************************
    // fields

    /**
     * scaled radius, excluding margin (in physics-space units)
     */
    private float scaledRadius;
    /**
     * scaled distance between the center of the parent sphere and the
     * hemisphere's center of mass (in physics-space units)
     */
    private float scaledY0;
    /**
     * radius, for scale=(1,1,1) and margin=0
     */
    private float unscaledRadius;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected CustomHemisphere() {
    }

    /**
     * Instantiate a hemisphere with the specified radius.
     *
     * @param radius the desired radius, before scaling and excluding margin
     * (&gt;0)
     */
    public CustomHemisphere(float radius) {
        super(halfExtents(radius));
        Validate.positive(radius, "radius");

        this.unscaledRadius = radius;
        setScale(scale);
    }
    // *************************************************************************
    // CustomConvexShape methods

    /**
     * Test whether the specified scale factors can be applied to this shape.
     * For a hemisphere, scaling must be uniform.
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

        // The supporting vertex lies on the curved portion of the surface:
        float dxyz = MyMath.hypotenuse(dirX, dirY, dirZ);
        if (dirY > 0f) {
            result.x = scaledRadius * (dirX / dxyz);
            result.y = scaledRadius * (dirY / dxyz) - scaledY0;
            result.z = scaledRadius * (dirZ / dxyz);

        } else { // More precisely, it lies on the rim:
            float dxz = MyMath.hypotenuse(dirX, dirZ);
            if (dxz == 0f) {
                result.set(scaledRadius, -scaledY0, 0f);
            } else {
                result.x = scaledRadius * (dirX / dxz);
                result.y = -scaledY0;
                result.z = scaledRadius * (dirZ / dxz);
            }
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
        float result = MyMath.hypotenuse(scaledRadius, scaledY0);
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
        setScale(scale);
    }

    /**
     * Estimate the volume of the collision shape, including scale and margin.
     *
     * @return the estimated volume (in physics-space units cubed, &ge;0)
     */
    @Override
    public float scaledVolume() {
        float result = 0.5f * MyVolume.sphereVolume(scaledRadius + margin)
                + margin * FastMath.PI * scaledRadius * scaledRadius;
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
        this.scaledY0 = 0.375f * scaledRadius;
        /*
         * the moments of inertia of a uniformly dense hemisphere
         * with mass=1, around its center of mass:
         */
        float iy = 0.4f * scaledRadius * scaledRadius;
        float ixz = iy - scaledY0 * scaledY0;
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
    }
    // *************************************************************************
    // private methods

    /**
     * Return the half extents of a hemisphere around its center of mass.
     *
     * @param radius the radius (&gt;0)
     * @return a new vector with all components &ge;0
     */
    private static Vector3f halfExtents(float radius) {
        // the distance between the center of curvature and the center of mass:
        float y0 = 0.375f * radius;

        float yHalfExtent = radius - y0;
        Vector3f result = new Vector3f(radius, yHalfExtent, radius);

        return result;
    }
}
