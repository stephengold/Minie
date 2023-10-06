/*
 * Copyright (c) 2018-2019 jMonkeyEngine
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
import java.nio.FloatBuffer;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.VectorSet;

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
     * for bone links only: center on the joint's pivot
     */
    Joint,
    /**
     * unweighted average of vertex locations
     */
    Mean,
    /**
     * center of the smallest enclosing sphere (using Welzl's algorithm)
     */
    Sphere;
    // *************************************************************************
    // new methods exposed

    /**
     * Calculate a center for the specified set of location vectors. No
     * implementation for {@link #Joint}.
     *
     * @param locations the set of location vectors (not null, not empty,
     * unaffected)
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (either storeResult or a new vector, not null)
     */
    public Vector3f center(VectorSet locations, Vector3f storeResult) {
        int numVectors = locations.numVectors();
        assert numVectors > 0 : numVectors;
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        switch (this) {
            case AABB:
                Vector3f maxima = new Vector3f();
                Vector3f minima = new Vector3f();
                locations.maxMin(maxima, minima);
                MyVector3f.midpoint(maxima, minima, result);
                break;

            case Mean:
                locations.mean(result);
                break;

            case Sphere:
                BoundingSphere boundingSphere = new BoundingSphere();
                FloatBuffer buffer = locations.toBuffer();
                boundingSphere.computeFromPoints(buffer);
                boundingSphere.getCenter(result);
                break;

            default:
                String message = "heuristic = " + toString();
                throw new IllegalStateException(message);
        }

        return result;
    }
}
