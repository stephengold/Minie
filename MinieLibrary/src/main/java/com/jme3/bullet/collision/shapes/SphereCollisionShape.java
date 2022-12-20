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

import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.nio.FloatBuffer;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.MyVolume;

/**
 * A spherical collision shape based on Bullet's {@code btSphereShape}. These
 * shapes have no margin and can only be scaled uniformly.
 *
 * @author normenhansen
 * @see MultiSphere
 */
public class SphereCollisionShape extends ConvexShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(SphereCollisionShape.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagRadius = "radius";
    // *************************************************************************
    // fields

    /**
     * copy of the unscaled radius (in shape units, &ge;0)
     */
    private float radius;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected SphereCollisionShape() {
    }

    /**
     * Instantiate a sphere shape that encloses the sample locations in the
     * specified FloatBuffer range.
     *
     * @param buffer the buffer that contains the sample locations (not null,
     * unaffected)
     * @param startPosition the position at which the sample locations start
     * (&ge;0, &le;endPosition)
     * @param endPosition the position at which the sample locations end
     * (&ge;startPosition, &le;capacity)
     */
    public SphereCollisionShape(
            FloatBuffer buffer, int startPosition, int endPosition) {
        Validate.nonNull(buffer, "buffer");
        Validate.inRange(startPosition, "start position", 0, endPosition);
        Validate.inRange(endPosition, "end position", startPosition,
                buffer.capacity());

        this.radius = MyBuffer.maxLength(buffer, startPosition, endPosition);
        createShape();
    }

    /**
     * Instantiate a sphere shape with the specified radius.
     *
     * @param radius the desired unscaled radius (in shape units, &ge;0)
     */
    public SphereCollisionShape(float radius) {
        Validate.nonNegative(radius, "radius");

        this.radius = radius;
        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Return the radius of the sphere.
     *
     * @return the unscaled radius (in shape units, &ge;0)
     */
    public float getRadius() {
        assert radius >= 0f : radius;
        return radius;
    }

    /**
     * Return the unscaled volume of the sphere.
     *
     * @return the volume (in shape units cubed, &ge;0)
     */
    public float unscaledVolume() {
        float result = MyVolume.sphereVolume(radius);

        assert result >= 0f : result;
        return result;
    }
    // *************************************************************************
    // ConvexShape methods

    /**
     * Test whether the specified scale factors can be applied to this shape.
     * For sphere shapes, scaling must be uniform.
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
     * Return the collision margin for this shape.
     *
     * @return the margin distance (in physics-space units, &ge;0)
     */
    @Override
    public float getMargin() {
        return 0f;
    }

    /**
     * Calculate how far the sphere extends from its center.
     *
     * @return the distance (in physics-space units, &ge;0)
     */
    @Override
    public float maxRadius() {
        float result = scale.x * radius;
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
        this.radius = capsule.readFloat(tagRadius, 0.5f);
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
     * Alter the collision margin of this shape. This feature is disabled for
     * sphere shapes.
     *
     * @param margin the desired margin distance (in physics-space units)
     */
    @Override
    public void setMargin(float margin) {
        logger2.log(Level.WARNING,
                "Cannot alter the margin of a SphereCollisionShape.");
    }

    /**
     * Approximate this shape with a HullCollisionShape.
     *
     * @return a new shape
     */
    @Override
    public HullCollisionShape toHullShape() {
        float defaultMargin = getDefaultMargin();
        float effectiveRadius = scale.x * radius; // in physics-space units

        HullCollisionShape result;
        if (effectiveRadius > defaultMargin) {
            // Use 42 vertices with the default margin.
            SphereCollisionShape shrunkenSphere
                    = new SphereCollisionShape(effectiveRadius - defaultMargin);
            FloatBuffer buffer = DebugShapeFactory.debugVertices(
                    shrunkenSphere, DebugShapeFactory.lowResolution);

            // Flip the buffer.
            buffer.rewind();
            buffer.limit(buffer.capacity());

            result = new HullCollisionShape(buffer);

        } else { // Use a single vertex with a reduced margin.
            result = new HullCollisionShape(0f, 0f, 0f);
            if (effectiveRadius <= 1e-9f) {
                result.setMargin(1e-9f);
            } else {
                result.setMargin(effectiveRadius);
            }
        }

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
        capsule.write(radius, tagRadius, 0.5f);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured {@code btSphereShape}.
     */
    private void createShape() {
        assert radius >= 0f : radius;

        long shapeId = createShape(radius);
        setNativeId(shapeId);

        setContactFilterEnabled(enableContactFilter);
        setScale(scale);
        this.margin = 0f;
    }
    // *************************************************************************
    // native private methods

    native private static long createShape(float radius);
}
