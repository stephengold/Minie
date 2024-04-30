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
import jme3utilities.math.MyVector3f;
import jme3utilities.math.MyVolume;

/**
 * A collision shape for an axis-aligned box with uniform density. This shape
 * could also be called a cuboid or a rectangular solid.
 * <p>
 * {@code BoxCollisionShape} and {@code HullCollisionShape} are probably more
 * efficient.
 * <p>
 * Like {@code HullCollisionShape} (but unlike {@code BoxCollisionShape}) this
 * is an imprecise shape; margin always expands the shape.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class CustomBox extends CustomConvexShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger loggerZ
            = Logger.getLogger(CustomBox.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagUnscaledHe = "unscaledHe";
    // *************************************************************************
    // fields

    /**
     * scaled half extents on each local axis, excluding margin (in
     * physics-space units)
     */
    private Vector3f scaledHe = new Vector3f();
    /**
     * half extents on each local axis, for scale=(1,1,1) and margin=0
     */
    private Vector3f unscaledHe;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected CustomBox() {
    }

    /**
     * Instantiate a cube with the specified half extent.
     *
     * @param halfExtent the desired half extent on each local axis, before
     * scaling and excluding margin (&gt;0)
     */
    public CustomBox(float halfExtent) {
        super(halfExtent, halfExtent, halfExtent);
        Validate.positive(halfExtent, "half extent");

        this.unscaledHe = new Vector3f(halfExtent, halfExtent, halfExtent);
        setScale(scale);
    }

    /**
     * Instantiate a box with the specified half extents.
     *
     * @param xHalfExtent the desired half extent on the local X axis, before
     * scaling and excluding margin (&gt;0)
     * @param yHalfExtent the desired half extent on the local Y axis, before
     * scaling and excluding margin (&gt;0)
     * @param zHalfExtent the desired half extent on the local Z axis, before
     * scaling and excluding margin (&gt;0)
     */
    public CustomBox(float xHalfExtent, float yHalfExtent, float zHalfExtent) {
        super(xHalfExtent, yHalfExtent, zHalfExtent);

        Validate.positive(xHalfExtent, "X half extent");
        Validate.positive(yHalfExtent, "Y half extent");
        Validate.positive(zHalfExtent, "Z half extent");

        this.unscaledHe = new Vector3f(xHalfExtent, yHalfExtent, zHalfExtent);
        setScale(scale);
    }

    /**
     * Instantiate a box with the specified half extents.
     *
     * @param halfExtents the desired half extents on each local axis, before
     * scaling and excluding margin (not null, all components &gt;0, unaffected)
     */
    public CustomBox(Vector3f halfExtents) {
        super(halfExtents);
        Validate.positive(halfExtents, "half extents");

        this.unscaledHe = halfExtents.clone();
        setScale(scale);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the half extents of the box.
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

        // The supporting vertex is simply the nearest corner:
        result.x = (dirX < 0f) ? -scaledHe.x : +scaledHe.x;
        result.y = (dirY < 0f) ? -scaledHe.y : +scaledHe.y;
        result.z = (dirZ < 0f) ? -scaledHe.z : +scaledHe.z;

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
        float result = scaledHe.length() + margin;
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
        float a = scaledHe.x;
        float b = scaledHe.y;
        float c = scaledHe.z;
        float result = MyVolume.boxVolume(scaledHe)
                + MyVolume.sphereVolume(margin)
                + 2f * (a * b + a * c + b * c);

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
        float a = scaledHe.x;
        float b = scaledHe.y;
        float c = scaledHe.z;
        /*
         * The moments of inertia of a uniformly dense box
         * with mass=1, around its center of mass:
         */
        float ix = (b * b + c * c) / 3f;
        float iy = (a * a + c * c) / 3f;
        float iz = (a * a + b * b) / 3f;
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
        capsule.write(unscaledHe, tagUnscaledHe, new Vector3f(1f, 1f, 1f));
    }
}
