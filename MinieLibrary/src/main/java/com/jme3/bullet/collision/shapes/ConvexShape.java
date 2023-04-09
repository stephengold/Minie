/*
 * Copyright (c) 2020-2023 jMonkeyEngine
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
import java.nio.FloatBuffer;
import java.util.logging.Logger;

/**
 * The abstract base class for convex collision shapes based on Bullet's
 * {@code btConvexShape}.
 * <p>
 * Subclasses include BoxCollisionShape and CapsuleCollisionShape.
 *
 * @author Stephen Gold sgold@sonic.net
 */
abstract public class ConvexShape extends CollisionShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger loggerX
            = Logger.getLogger(ConvexShape.class.getName());
    // *************************************************************************
    // constructors

    /**
     * Instantiate a collision shape with no tracker and no assigned native
     * object.
     */
    protected ConvexShape() { // explicit to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Approximate this shape with a HullCollisionShape. Meant to be overridden.
     *
     * @return a new shape
     */
    public HullCollisionShape toHullShape() {
        // Generate low-res debug vertices.
        FloatBuffer buffer = DebugShapeFactory
                .debugVertices(this, DebugShapeFactory.lowResolution);

        // Flip the buffer.
        buffer.rewind();
        buffer.limit(buffer.capacity());

        HullCollisionShape result = new HullCollisionShape(buffer);
        return result;
    }
    // *************************************************************************
    // CollisionShape methods

    /**
     * Test whether this shape has concave type. In Bullet, "concave" is a
     * property of <em>types</em> of shapes. Specific <em>instances</em> of
     * those types might actually be "convex" in the mathematical sense of the
     * word.
     *
     * @return true if concave type, false otherwise
     */
    @Override
    public boolean isConcave() {
        assert !super.isConcave();
        return false;
    }

    /**
     * Test whether this shape has convex type. In Bullet, "convex" is a
     * property of <em>types</em> of shapes. Specific <em>instances</em> of
     * non-convex types might still be "convex" in the mathematical sense of the
     * word.
     *
     * @return true if convex type, false otherwise
     */
    @Override
    public boolean isConvex() {
        assert super.isConvex();
        return true;
    }

    /**
     * Estimate the volume of this shape, including scale and margin.
     *
     * @return the volume (in physics-space units cubed, &ge;0)
     */
    @Override
    public float scaledVolume() {
        int meshResolution = DebugShapeFactory.lowResolution;
        float result = DebugShapeFactory.volumeConvex(this, meshResolution);

        assert result >= 0f : result;
        return result;
    }

    /**
     * Approximate this shape with a splittable shape.
     *
     * @return a new splittable shape
     */
    @Override
    public CollisionShape toSplittableShape() {
        CollisionShape result;
        if (canSplit()) {
            result = this;
        } else {
            result = toHullShape();
        }

        return result;
    }
}
