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
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;

/**
 * A collision shape for an axis-aligned ellipsoid.
 * <p>
 * Like a {@code Multisphere} but unlike a {@code SphereCollisionShape}, these
 * shapes have margins and can be scaled non-uniformly. Margin always expands
 * the shape.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class CustomEllipsoid extends CustomConvexShape {
    // *************************************************************************
    // constants and loggers

    /**
     * 4*Pi/3 in single precision
     */
    final private static float fourThirdsPi = 4.1887902f;
    /**
     * message logger for this class
     */
    final public static Logger loggerZ
            = Logger.getLogger(CustomEllipsoid.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagInertiaFactor = "inertiaFactor";
    final private static String tagUnscaledHe = "unscaledHe";
    // *************************************************************************
    // fields

    /**
     * 0.2 for a uniform-density solid, 1/3 for a hollow shell (&gt;0, &lt;1)
     */
    private float inertiaFactor;
    /**
     * scaled half extents on each local axis, excluding margin (in
     * physics-space units)
     */
    private Vector3f scaledHe = new Vector3f();
    /**
     * squared scaled half extents on each local axis, for margin=0
     */
    private Vector3f squaredScaledHe = new Vector3f();
    /**
     * half extents on each local axis, for scale=(1,1,1) and margin=0
     */
    private Vector3f unscaledHe;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected CustomEllipsoid() {
    }

    /**
     * Instantiate a sphere with the specified radius and uniform density.
     *
     * @param radius the radius, before scaling and excluding margin (&gt;0)
     */
    public CustomEllipsoid(float radius) {
        super(radius, radius, radius);
        Validate.positive(radius, "radius");

        this.unscaledHe = new Vector3f(radius, radius, radius);
        this.inertiaFactor = 0.2f;
        setScale(scale);
    }

    /**
     * Instantiate a triaxial ellipsoid with the specified parameters.
     *
     * @param xHalfExtent the desired half extent on the local X axis, before
     * scaling and excluding margin (&gt;0)
     * @param yHalfExtent the desired half extent on the local Y axis, before
     * scaling and excluding margin (&gt;0)
     * @param zHalfExtent the desired half extent on the local Z axis, before
     * scaling and excluding margin (&gt;0)
     * @param inertiaFactor 0.2 for a uniform-density solid, 1/3 for a hollow
     * shell (&gt;0, &lt;1)
     */
    public CustomEllipsoid(float xHalfExtent, float yHalfExtent,
            float zHalfExtent, float inertiaFactor) {
        super(xHalfExtent, yHalfExtent, zHalfExtent);

        Validate.positive(xHalfExtent, "X half extent");
        Validate.positive(yHalfExtent, "Y half extent");
        Validate.positive(zHalfExtent, "Z half extent");

        this.unscaledHe = new Vector3f(xHalfExtent, yHalfExtent, zHalfExtent);
        this.inertiaFactor = inertiaFactor;
        setScale(scale);
    }

    /**
     * Instantiate a triaxial ellipsoid with the specified parameters.
     *
     * @param halfExtents the desired half extents on each local axis, before
     * scaling and excluding margin (not null, all components &gt;0, unaffected)
     * @param inertiaFactor 0.2 for a uniform-density solid, 1/3 for a hollow
     * shell (&gt;0, &lt;1)
     */
    public CustomEllipsoid(Vector3f halfExtents, float inertiaFactor) {
        super(halfExtents);
        Validate.positive(halfExtents, "half extents");

        this.unscaledHe = halfExtents.clone();
        this.inertiaFactor = inertiaFactor;
        setScale(scale);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the half extents of the ellipsoid.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the unscaled half extent for each local axis (either storeResult
     * or a new vector, not null, all components &gt;0)
     */
    public Vector3f getHalfExtents(Vector3f storeResult) {
        assert MyVector3f.isAllPositive(unscaledHe) : unscaledHe;

        Vector3f result;
        if (storeResult == null) {
            result = unscaledHe.clone();
        } else {
            result = storeResult.set(unscaledHe);
        }

        return result;
    }
    // *************************************************************************
    // CustomConvexShape methods

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

        this.scaledHe = cloner.clone(scaledHe);
        this.squaredScaledHe = cloner.clone(squaredScaledHe);
        this.unscaledHe = cloner.clone(unscaledHe);
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

        float x = dirX * squaredScaledHe.x;
        float y = dirY * squaredScaledHe.y;
        float z = dirZ * squaredScaledHe.z;

        float dxyz = MyMath.hypotenuse(x, y, z);
        if (dxyz == 0f) {
            result.set(scaledHe.x, 0f, 0f);
        } else {
            result.x = scaledHe.x * (x / dxyz);
            result.y = scaledHe.y * (y / dxyz);
            result.z = scaledHe.z * (z / dxyz);
        }

        return result;
    }

    /**
     * Calculate how far the scaled shape extends from its center of mass,
     * including collision margin.
     *
     * @return the distance (in physics-space units, &ge;0)
     */
    @Override
    public float maxRadius() {
        float result = MyMath.max(scaledHe.x, scaledHe.y, scaledHe.z) + margin;
        return result;
    }

    /**
     * De-serialize the shape from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);
        InputCapsule capsule = importer.getCapsule(this);

        this.inertiaFactor = capsule.readFloat(tagInertiaFactor, 0.2f);
        this.unscaledHe = (Vector3f) capsule.readSavable(
                tagUnscaledHe, new Vector3f(1f, 1f, 1f));
        setScale(scale);
    }

    /**
     * Estimate the volume of the collision shape, including scale and margin.
     *
     * @return the estimated volume (in physics-space units cubed, &ge;0)
     */
    @Override
    public float scaledVolume() {
        float a = scaledHe.x + margin;
        float b = scaledHe.y + margin;
        float c = scaledHe.z + margin;
        float result = fourThirdsPi * a * b * c;

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

        unscaledHe.mult(scale, scaledHe);
        scaledHe.mult(scaledHe, squaredScaledHe);
        float a2 = squaredScaledHe.x;
        float b2 = squaredScaledHe.y;
        float c2 = squaredScaledHe.z;
        /*
         * see https://adamheins.com/blog/ellipsoidal-shell-inertia
         *
         * the moments of inertia of a uniformly dense ellipsoid
         * with mass=1, around its center of mass:
         */
        float ix = inertiaFactor * (b2 + c2);
        float iy = inertiaFactor * (a2 + c2);
        float iz = inertiaFactor * (a2 + b2);
        setScaledInertia(ix, iy, iz);
    }

    /**
     * Serialize the shape to the specified exporter, for example when saving to
     * a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        super.write(exporter);
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(inertiaFactor, tagInertiaFactor, 0.2f);
        capsule.write(unscaledHe, tagUnscaledHe, new Vector3f(1f, 1f, 1f));
    }
}
