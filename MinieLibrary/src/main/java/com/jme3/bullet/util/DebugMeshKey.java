/*
 * Copyright (c) 2018-2022 jMonkeyEngine
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
package com.jme3.bullet.util;

import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.math.Vector3f;
import java.util.logging.Logger;
import jme3utilities.MeshNormals;

/**
 * Key used to locate cached debug meshes. Note: immutable.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class DebugMeshKey {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(DebugMeshKey.class.getName());
    // *************************************************************************
    // fields

    /**
     * margin of the CollisionShape
     */
    final private float margin;
    /**
     * desired mesh resolution
     */
    final private int resolution;
    /**
     * object ID of the CollisionShape
     */
    final private long shapeId;
    /**
     * option for normals in the debug mesh
     */
    final private MeshNormals normals;
    /**
     * scale factors of the CollisionShape
     */
    final private Vector3f scale;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a new key.
     *
     * @param shape (not null, not compound, unaffected)
     * @param normals (not null)
     * @param resolution 0 or 1
     */
    DebugMeshKey(CollisionShape shape, MeshNormals normals, int resolution) {
        assert normals != null;
        assert !(shape instanceof CompoundCollisionShape);

        this.normals = normals;
        margin = shape.getMargin();
        if (shape.isConvex()) {
            this.resolution = resolution;
        } else {
            this.resolution = DebugShapeFactory.lowResolution;
        }
        shapeId = shape.nativeId();
        scale = shape.getScale(null);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Read the shape ID of this key.
     *
     * @return the ID value
     */
    long shapeId() {
        return shapeId;
    }
    // *************************************************************************
    // Object methods

    /**
     * Test for exact equivalence with another Object.
     *
     * @param otherObject the object to compare to (may be null, unaffected)
     * @return true if the objects are equivalent, otherwise false
     */
    @Override
    public boolean equals(Object otherObject) {
        boolean result;
        if (otherObject == this) {
            result = true;
        } else if (otherObject != null
                && otherObject.getClass() == getClass()) {
            DebugMeshKey otherKey = (DebugMeshKey) otherObject;
            result = (shapeId == otherKey.shapeId)
                    && scale.equals(otherKey.scale)
                    && (Float.compare(margin, otherKey.margin) == 0)
                    && (normals == otherKey.normals)
                    && (resolution == otherKey.resolution);
        } else {
            result = false;
        }

        return result;
    }

    /**
     * Generate the hash code for this key.
     *
     * @return value for use in hashing
     */
    @Override
    public int hashCode() {
        int hash = (int) (shapeId >> 4);
        hash = 7 * hash + scale.hashCode();
        hash = 7 * hash + Float.floatToIntBits(margin);
        hash = 7 * hash + resolution;
        hash = 7 * hash + normals.ordinal();

        return hash;
    }

    /**
     * Represent this key as a text string.
     *
     * @return descriptive string of text (not null, not empty)
     */
    @Override
    public String toString() {
        String desc = String.format(
                "shape=%x scale=%s margin=%f res=%d normals=%s",
                shapeId, scale, margin, resolution, normals);
        return desc;
    }
}
