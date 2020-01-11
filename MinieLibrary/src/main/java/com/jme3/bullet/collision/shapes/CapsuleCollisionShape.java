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
import jme3utilities.math.MyVolume;

/**
 * A capsule CollisionShape based on Bullet's btCapsuleShapeX, btCapsuleShape,
 * or btCapsuleShapeZ. These shapes have no margin and can only be scaled
 * uniformly.
 *
 * @author normenhansen
 */
public class CapsuleCollisionShape extends CollisionShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(CapsuleCollisionShape.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagAxis = "axis";
    final private static String tagHeight = "height";
    final private static String tagRadius = "radius";
    // *************************************************************************
    // fields

    /**
     * copy of the unscaled height of the cylindrical portion (&ge;0)
     */
    private float height;
    /**
     * copy of the unscaled radius (&ge;0)
     */
    private float radius;
    /**
     * copy of the index of the main (height) axis (0&rarr;X, 1&rarr;Y,
     * 2&rarr;Z)
     */
    private int axis;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public CapsuleCollisionShape() {
    }

    /**
     * Instantiate a Y-axis capsule shape with the specified radius and height.
     *
     * @param radius the desired unscaled radius (&ge;0)
     * @param height the desired unscaled height (of the cylindrical portion)
     * (&ge;0)
     */
    public CapsuleCollisionShape(float radius, float height) {
        Validate.nonNegative(radius, "radius");
        Validate.nonNegative(height, "height");

        this.radius = radius;
        this.height = height;
        this.axis = PhysicsSpace.AXIS_Y;
        createShape();
    }

    /**
     * Instantiate a capsule shape around the specified main (height) axis.
     *
     * @param radius the desired unscaled radius (&ge;0)
     * @param height the desired unscaled height (of the cylindrical portion)
     * (&ge;0)
     * @param axisIndex which local axis for height: 0&rarr;X, 1&rarr;Y,
     * 2&rarr;Z
     */
    public CapsuleCollisionShape(float radius, float height, int axisIndex) {
        Validate.nonNegative(radius, "radius");
        Validate.nonNegative(height, "height");
        Validate.inRange(axisIndex, "axis index", PhysicsSpace.AXIS_X,
                PhysicsSpace.AXIS_Z);

        this.radius = radius;
        this.height = height;
        this.axis = axisIndex;
        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Read the main (height) axis of the capsule.
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
     * Read the height (of the cylindrical portion) of the capsule.
     *
     * @return the unscaled height (&ge;0)
     */
    public float getHeight() {
        assert height >= 0f : height;
        return height;
    }

    /**
     * Read the radius of the capsule.
     *
     * @return the unscaled radius (&ge;0)
     */
    public float getRadius() {
        assert radius >= 0f : radius;
        return radius;
    }

    /**
     * Calculate the unscaled volume of the capsule.
     *
     * @return the volume (&ge;0)
     */
    public float unscaledVolume() {
        float result = MyVolume.capsuleVolume(radius, height);

        assert result >= 0f : result;
        return result;
    }
    // *************************************************************************
    // CollisionShape methods

    /**
     * Test whether the specified scale factors can be applied to this shape.
     * For capsule shapes, scaling must be uniform.
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
        createShape();
    }

    /**
     * Determine the collision margin for this shape.
     *
     * @return the margin distance (in physics-space units, &ge;0)
     */
    @Override
    public float getMargin() {
        return 0f;
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public CapsuleCollisionShape jmeClone() {
        try {
            CapsuleCollisionShape clone = (CapsuleCollisionShape) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }

    /**
     * Calculate how far the capsule extends from its center.
     *
     * @return the distance (in physics-space units, &ge;0)
     */
    @Override
    public float maxRadius() {
        float result = scale.x * (radius + height / 2f);
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

        radius = capsule.readFloat(tagRadius, 0.5f);
        height = capsule.readFloat(tagHeight, 1f);
        axis = capsule.readInt(tagAxis, PhysicsSpace.AXIS_Y);
        createShape();
    }

    /**
     * Alter the collision margin of this shape. This feature is disabled for
     * capsule shapes.
     *
     * @param margin the desired margin distance (in physics-space units)
     */
    @Override
    public void setMargin(float margin) {
        logger2.log(Level.WARNING,
                "Cannot alter the margin of a CapsuleCollisionShape.");
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

        capsule.write(radius, tagRadius, 0.5f);
        capsule.write(height, tagHeight, 1f);
        capsule.write(axis, tagAxis, PhysicsSpace.AXIS_Y);
    }
    // *************************************************************************
    // private methods

    /**
     * Instantiate the configured btCollisionShape.
     */
    private void createShape() {
        assert axis == PhysicsSpace.AXIS_X
                || axis == PhysicsSpace.AXIS_Y
                || axis == PhysicsSpace.AXIS_Z : axis;
        assert radius >= 0f : radius;
        assert height >= 0f : height;

        long shapeId = createShape(axis, radius, height);
        setNativeId(shapeId);

        setScale(scale);
        margin = 0f;
    }
    // *************************************************************************
    // native methods

    native private long createShape(int axis, float radius, float height);
}
