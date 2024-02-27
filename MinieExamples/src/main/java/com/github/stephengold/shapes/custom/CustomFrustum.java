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
import jme3utilities.math.MyMath;

/**
 * A collision shape for a conical frustum with uniform density. By convention,
 * the local Y axis is the height axis, with the "A" base having y&lt;0 and the
 * "B" base having y&gt;0.
 * <p>
 * This is an imprecise shape; margin always expands the shape.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class CustomFrustum extends CustomConvexShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger loggerZ
            = Logger.getLogger(CustomFrustum.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagUnscaledA = "unscaledA";
    final private static String tagUnscaledB = "unscaledB";
    final private static String tagUnscaledHeight = "unscaledHeight";
    // *************************************************************************
    // fields

    /**
     * scaled radius of the "A" base, excluding margin (in physics-space units)
     */
    private float scaledA;
    /**
     * scaled radius of the "B" base, excluding margin (in physics-space units)
     */
    private float scaledB;
    /**
     * scaled height, excluding margin (in physics-space units)
     */
    private float scaledHeight;
    /**
     * scaled distance between the center of "A" base and the frustum's center
     * of mass (in physics-space units)
     */
    private float scaledY0;
    /**
     * radius of the "A" base, for scale=(1,1,1) and margin=0
     */
    private float unscaledA;
    /**
     * radius of the "B" base, for scale=(1,1,1) and margin=0
     */
    private float unscaledB;
    /**
     * height, for scale=(1,1,1) and margin=0
     */
    private float unscaledHeight;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected CustomFrustum() {
    }

    /**
     * Instantiate a conical frustum with the specified dimensions.
     *
     * @param a the desired radius of the "A" base for scale=(1,1,1) (&gt;0)
     * @param b the desired radius of the "B" base for scale=(1,1,1) (&gt;0)
     * @param height the desired height for scale=(1,1,1) (&gt;0)
     */
    public CustomFrustum(float a, float b, float height) {
        super(halfExtents(a, b, height));

        this.unscaledA = a;
        this.unscaledB = b;
        this.unscaledHeight = height;
        setScale(scale);
    }
    // *************************************************************************
    // CustomConvexShape methods

    /**
     * Test whether the specified scale factors can be applied to this shape.
     * For a conical frustum, scaling must preserve the circular cross section.
     *
     * @param scale the desired scale factor for each local axis (may be null,
     * unaffected)
     * @return true if applicable, otherwise false
     */
    @Override
    public boolean canScale(Vector3f scale) {
        boolean result = super.canScale(scale) && scale.x == scale.z;
        return result;
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

        // The supporting vertex lies on the rim of one of the bases:
        float dxz = MyMath.hypotenuse(dirX, dirZ);
        if (scaledHeight * dirY < (scaledA - scaledB) * dxz) {
            // More precisely, the rim of the "A" base:
            result.y = -scaledY0;
            if (dxz == 0f) {
                result.x = scaledA;
                result.z = 0f;
            } else {
                result.x = scaledA * (dirX / dxz);
                result.z = scaledA * (dirZ / dxz);
            }

        } else { // It lies on the rim of the "B" base:
            result.y = scaledHeight - scaledY0;
            if (dxz == 0f) {
                result.x = scaledB;
                result.z = 0f;
            } else {
                result.x = scaledB * (dirX / dxz);
                result.z = scaledB * (dirZ / dxz);
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
        float rimA = MyMath.hypotenuse(scaledA, scaledY0);
        float rimB = MyMath.hypotenuse(scaledB, scaledHeight - scaledY0);
        float result = Math.max(rimA, rimB);

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

        this.unscaledA = capsule.readFloat(tagUnscaledA, 1f);
        this.unscaledB = capsule.readFloat(tagUnscaledB, 1f);
        this.unscaledHeight = capsule.readFloat(tagUnscaledHeight, 1f);

        setScale(scale);
    }

    /**
     * Estimate the volume of the collision shape, including scale and margin.
     *
     * @return the estimated volume (in physics-space units cubed, &ge;0)
     */
    @Override
    public float scaledVolume() {
        float a = scaledA + margin;
        float b = scaledB + margin;
        float h = scaledHeight + 2f * margin;
        float denom = a * a + a * b + b * b;
        float result = FastMath.PI * h * denom / 3f;

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

        // super.setScale() has verified that scale.x == scale.z
        this.scaledHeight = scale.y * unscaledHeight;
        this.scaledA = scale.x * unscaledA;
        this.scaledB = scale.x * unscaledB;

        float a2 = scaledA * scaledA;
        float ab = scaledA * scaledB;
        float b2 = scaledB * scaledB;
        float denom = a2 + ab + b2;
        this.scaledY0 = 0.25f * scaledHeight * (a2 + 2f * ab + 3f * b2) / denom;

        float a3 = a2 * scaledA;
        float a4 = a3 * scaledA;
        float b3 = b2 * scaledB;
        float b4 = b3 * scaledB;
        float h2 = scaledHeight * scaledHeight;
        float mom = 3f * a3 * (scaledA + scaledB)
                + a2 * (3f * b2 + 2f * h2)
                + 3f * ab * (b2 + 2f * h2)
                + 3f * b2 * (b2 + 4f * h2);
        /*
         * the moments of inertia of a uniformly dense conical frustum
         * with mass=1, around its center of mass:
         */
        float ixz = (0.05f / denom) * mom - scaledY0 * scaledY0;
        float iy = (0.3f / denom)
                * (a4 + a3 * scaledB + a2 * b2 + scaledA * b3 + b4);
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

        capsule.write(unscaledA, tagUnscaledA, 1f);
        capsule.write(unscaledB, tagUnscaledB, 1f);
        capsule.write(unscaledHeight, tagUnscaledHeight, 1f);
    }
    // *************************************************************************
    // private methods

    /**
     * Return the half extents of a conical frustum around its center of mass.
     *
     * @param a the radius of the "A" base (&gt;0)
     * @param b the radius of the "B" base (&gt;0)
     * @param height the height of the frustum (&gt;0)
     * @return a new vector with all components &ge;0
     */
    private static Vector3f halfExtents(float a, float b, float height) {
        float a2 = a * a;
        float ab = a * b;
        float b2 = b * b;
        float denom = a2 + ab + b2;
        /*
         * the distance between the center of "A" base
         * and the frustum's center of mass:
         */
        float y0 = 0.25f * height * (a2 + 2f * ab + 3f * b2) / denom;

        float maxR = Math.max(a, b);
        float maxY = Math.max(height - y0, y0);

        Vector3f result = new Vector3f(maxR, maxY, maxR);
        return result;
    }
}
