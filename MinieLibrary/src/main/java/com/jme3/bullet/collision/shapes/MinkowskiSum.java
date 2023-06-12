/*
 * Copyright (c) 2023 jMonkeyEngine
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
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * A convex collision shape to represent the Minkowki sum of 2 convex shapes,
 * based on Bullet's {@code btMinkowskiSumShape}.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class MinkowskiSum extends ConvexShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(MinkowskiSum.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagShapeA = "shapeA";
    final private static String tagShapeB = "shapeB";
    // *************************************************************************
    // fields

    /**
     * first shape on which this shape is based
     */
    private ConvexShape shapeA;
    /**
     * 2nd shape on which this shape is based
     */
    private ConvexShape shapeB;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected MinkowskiSum() {
    }

    /**
     * Instantiate the Minkowki sum of the specified shapes.
     *
     * @param shapeA the first base shape (not null, convex, alias created)
     * @param shapeB the 2nd base shape (not null, convex, alias created)
     */
    public MinkowskiSum(ConvexShape shapeA, ConvexShape shapeB) {
        Validate.nonNull(shapeA, "shape A");
        Validate.nonNull(shapeA, "shape B");

        this.shapeA = shapeA;
        this.shapeB = shapeB;
        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Access the first base shape.
     *
     * @return the pre-existing shape (not null)
     */
    public ConvexShape getShapeA() {
        assert shapeA != null;
        return shapeA;
    }

    /**
     * Access the 2nd base shape.
     *
     * @return the pre-existing shape (not null)
     */
    public ConvexShape getShapeB() {
        assert shapeB != null;
        return shapeB;
    }
    // *************************************************************************
    // ConvexShape methods

    /**
     * Test whether the specified scale factors can be applied to this shape.
     * For MinkowskiSum shapes, scaling must be unity.
     *
     * @param scale the desired scale factor for each local axis (may be null,
     * unaffected)
     * @return true if applicable, otherwise false
     */
    @Override
    public boolean canScale(Vector3f scale) {
        boolean canScale = super.canScale(scale)
                && MyVector3f.isScaleIdentity(scale);

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
        this.shapeA = cloner.clone(shapeA);
        this.shapeB = cloner.clone(shapeB);
        createShape();
    }

    /**
     * Return the collision margin for this shape.
     *
     * @return the margin distance (in physics-space units, &ge;0)
     */
    @Override
    public float getMargin() {
        this.margin = shapeA.nativeMargin() + shapeB.nativeMargin();
        float result = super.getMargin();

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

        this.shapeA = (ConvexShape) capsule.readSavable(tagShapeA, null);
        this.shapeB = (ConvexShape) capsule.readSavable(tagShapeB, null);
        createShape();
    }

    /**
     * Alter the collision margin of this shape. This feature is disabled for
     * MinkowskiSum shapes. The margin of a MinkowskiSum is simply the sum of
     * the (native) margins of the base shapes.
     *
     * @param margin the desired margin distance (in physics-space units)
     */
    @Override
    public void setMargin(float margin) {
        logger2.log(Level.WARNING,
                "Cannot directly alter the margin of a MinkowskiSum");
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
        capsule.write(shapeA, tagShapeA, null);
        capsule.write(shapeB, tagShapeB, null);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate a {@code btMinkowskiSumShape}.
     */
    private void createShape() {
        long shapeAId = shapeA.nativeId();
        long shapeBId = shapeB.nativeId();
        long shapeId = createShape(shapeAId, shapeBId);
        setNativeId(shapeId);

        setContactFilterEnabled(enableContactFilter);
        setScale(scale);
        float totalMargin = shapeA.getMargin() + shapeB.getMargin();
        setMargin(totalMargin);
    }
    // *************************************************************************
    // native private methods

    native private static long createShape(long shapeAId, long shapeBId);
}
