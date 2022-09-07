/*
 * Copyright (c) 2020 jMonkeyEngine
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
package com.jme3.bullet.debug;

import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.infos.ChildCollisionShape;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import java.util.logging.Logger;

/**
 * A concise summary of a ChildCollisionShape, used to detect changes that might
 * affect debug visualization.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class ChildSummary {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final static Logger logger = Logger.getLogger(ChildSummary.class.getName());
    // *************************************************************************
    // fields

    /**
     * if false, this summarizes a nonexistent child: ignore all other fields
     */
    private boolean isValid = false;
    /**
     * collision margin of the CollisionShape
     */
    private float margin;
    /**
     * native ID of the CollisionShape
     */
    private long shapeId;
    /**
     * rotation of the ChildCollisionShape relative to its parent
     */
    final private Quaternion rotation = new Quaternion();
    /**
     * offset of the ChildCollisionShape relative to its parent
     */
    final private Vector3f offset = new Vector3f();
    /**
     * scale factors of the CollisionShape
     */
    final private Vector3f scale = new Vector3f();
    // *************************************************************************
    // new methods exposed

    /**
     * Test whether this summary is valid.
     *
     * @return true if valid, otherwise false
     */
    boolean isValid() {
        return isValid;
    }

    /**
     * Configure based on the specified ChildCollisionShape.
     *
     * @param child the ChildCollisionShape, or null if not present
     */
    void update(ChildCollisionShape child) {
        if (child == null) {
            this.isValid = false;

        } else {
            CollisionShape baseShape = child.getShape();
            this.margin = baseShape.getMargin();
            this.shapeId = baseShape.nativeId();
            child.copyRotation(rotation);
            child.copyOffset(offset);
            baseShape.getScale(scale);
            this.isValid = true;
        }
    }
    // *************************************************************************
    // Object methods

    /**
     * Test for equivalence with another object.
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
            ChildSummary other = (ChildSummary) otherObject;
            if (isValid || other.isValid) {
                result = isValid
                        && other.isValid
                        && shapeId == other.shapeId
                        && Float.compare(margin, other.margin) == 0
                        && rotation.equals(other.rotation)
                        && offset.equals(other.offset)
                        && scale.equals(other.scale);
            } else {
                result = true;
            }

        } else {
            result = false;
        }

        return result;
    }

    /**
     * Generate the hash code for this summary.
     *
     * @return a 32-bit value for use in hashing
     */
    @Override
    public int hashCode() {
        int hash = 5;
        if (isValid) {
            hash = 97 * hash + (int) (shapeId >> 4);
            hash = 97 * hash + Float.floatToIntBits(margin);
            hash = 97 * hash + rotation.hashCode();
            hash = 97 * hash + offset.hashCode();
            hash = 97 * hash + scale.hashCode();
        }
        return hash;
    }
}
