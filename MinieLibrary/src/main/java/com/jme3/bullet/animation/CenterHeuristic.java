/*
 * Copyright (c) 2018 jMonkeyEngine
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
package com.jme3.bullet.animation;

import com.jme3.bounding.BoundingSphere;
import com.jme3.math.Vector3f;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.Collection;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * Enumerate algorithms to locate the center of mass for a PhysicsLink.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public enum CenterHeuristic {
    // *************************************************************************
    // values

    /**
     * center of the smallest axis-aligned bounding box
     */
    AABB,
    /**
     * unweighted average of vertex locations
     */
    Mean,
    /**
     * for bone links only: center on the joint's pivot
     */
    Joint,
    /**
     * center of the smallest enclosing sphere (using Welzl's algorithm)
     */
    Sphere;
    // *************************************************************************
    // new methods exposed

    /**
     * Calculate a center for the specified collection of location vectors. No
     * implementation for {@link #Joint}.
     *
     * @param locations the collection of location vectors (not null, not empty,
     * unaffected)
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (either storeResult or a new vector, not null)
     */
    public Vector3f center(Collection<Vector3f> locations, Vector3f storeResult) {
        Validate.nonEmpty(locations, "locations");
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        switch (this) {
            case AABB:
                Vector3f maxima = new Vector3f(Float.NEGATIVE_INFINITY,
                        Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY);
                Vector3f minima = new Vector3f(Float.POSITIVE_INFINITY,
                        Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY);
                for (Vector3f location : locations) {
                    MyVector3f.accumulateMaxima(maxima, location);
                    MyVector3f.accumulateMinima(minima, location);
                }
                maxima.add(minima, result);
                result.divideLocal(2f);
                break;

            case Mean:
                MyVector3f.mean(locations, result);
                break;

            case Sphere:
                int numFloats = 3 * locations.size();
                FloatBuffer buf = BufferUtils.createFloatBuffer(numFloats);
                for (Vector3f location : locations) {
                    buf.put(location.x);
                    buf.put(location.y);
                    buf.put(location.z);
                }
                BoundingSphere boundingSphere = new BoundingSphere();
                boundingSphere.computeFromPoints(buf);
                boundingSphere.getCenter(result);
                break;

            default:
                String message = "heuristic = " + toString();
                throw new IllegalArgumentException(message);
        }

        return result;
    }
}
