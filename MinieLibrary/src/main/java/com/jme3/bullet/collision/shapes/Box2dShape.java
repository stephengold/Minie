/*
 * Copyright (c) 2019-2022 jMonkeyEngine
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
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;

/**
 * An axis-aligned, rectangular collision shape based on Bullet's
 * {@code btBox2dShape}.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class Box2dShape extends ConvexShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(Box2dShape.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagX = "halfExtentX";
    final private static String tagY = "halfExtentY";
    // *************************************************************************
    // fields

    /**
     * copy of the unscaled half extent on the local X axis (not negative)
     */
    private float halfExtentX = 1f;
    /**
     * copy of the unscaled half extent on the local Y axis (not negative)
     */
    private float halfExtentY = 1f;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected Box2dShape() {
    }

    /**
     * Instantiate a square shape with the specified half extent.
     *
     * @param halfExtent the desired unscaled half extent on each local axis
     * (not negative)
     */
    public Box2dShape(float halfExtent) {
        Validate.nonNegative(halfExtent, "half extent");

        this.halfExtentX = halfExtent;
        this.halfExtentY = halfExtent;
        createShape();
    }

    /**
     * Instantiate a rectangle shape with the specified half extents.
     *
     * @param xHalfExtent the desired unscaled half extent on the local X axis
     * (not negative)
     * @param yHalfExtent the desired unscaled half extent on the local Y axis
     * (not negative)
     */
    public Box2dShape(float xHalfExtent, float yHalfExtent) {
        Validate.nonNegative(xHalfExtent, "half extent on X");
        Validate.nonNegative(yHalfExtent, "half extent on Y");

        this.halfExtentX = xHalfExtent;
        this.halfExtentY = yHalfExtent;
        createShape();
    }

    /**
     * Instantiate a rectangle shape with the specified half extents.
     *
     * @param halfExtents the desired unscaled half extents (not null, no
     * negative component, unaffected)
     */
    public Box2dShape(Vector2f halfExtents) {
        Validate.nonNegative(halfExtents, "half extents");

        this.halfExtentX = halfExtents.x;
        this.halfExtentY = halfExtents.y;
        createShape();
    }

    /**
     * Instantiate a rectangle shape with the specified half extents.
     *
     * @param halfExtents the desired unscaled half extents (not null, no
     * negative component, unaffected, Z component ignored)
     */
    public Box2dShape(Vector3f halfExtents) {
        Validate.nonNegative(halfExtents, "half extents");

        this.halfExtentX = halfExtents.x;
        this.halfExtentY = halfExtents.y;
        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the half extents of the rectangle.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the unscaled half extent for each local axis (either storeResult
     * or a new vector, not null, no negative component)
     */
    public Vector3f getHalfExtents(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        result.set(halfExtentX, halfExtentY, margin);

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
        createShape();
    }

    /**
     * Calculate how far the box extends from its center.
     *
     * @return a distance (in physics-space units, &ge;0)
     */
    @Override
    public float maxRadius() {
        float x = scale.x * halfExtentX;
        float y = scale.y * halfExtentY;
        float result = MyMath.hypotenuse(x, y);

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

        this.halfExtentX = capsule.readFloat(tagX, 1f);
        this.halfExtentY = capsule.readFloat(tagY, 1f);
        createShape();
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
        capsule.write(halfExtentX, tagX, 1f);
        capsule.write(halfExtentY, tagY, 1f);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured {@code btBox2dShape}.
     */
    private void createShape() {
        assert halfExtentX >= 0f : halfExtentX;
        assert halfExtentY >= 0f : halfExtentY;

        long shapeId = createShape(halfExtentX, halfExtentY, margin);
        setNativeId(shapeId);

        setContactFilterEnabled(enableContactFilter);
        setScale(scale);
        setMargin(margin);
    }
    // *************************************************************************
    // native private methods

    native private static long createShape(float x, float y, float margin);
}
