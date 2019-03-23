/*
 * Copyright (c) 2019 jMonkeyEngine
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

import com.jme3.math.Matrix3f;
import com.jme3.math.Vector3f;
import java.nio.FloatBuffer;

/**
 * A simplified collection of Vector3f values without duplicates.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public interface VectorSet {
    /**
     * Add the value of the specified Vector3f to this set.
     *
     * @param vector the value to add (not null, unaffected)
     */
    void add(Vector3f vector);

    /**
     * Test whether this set contains the value of the specified Vector3f.
     *
     * @param vector the value to find (not null, unaffected)
     * @return true if found, otherwise false
     */
    boolean contains(Vector3f vector);

    /**
     * Calculate the sample covariance of the Vector3f values in this set.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the unbiased sample covariance (either storeResult or a new
     * matrix, not null)
     */
    Matrix3f covariance(Matrix3f storeResult);

    /**
     * Find the length of the longest Vector3f value in this set.
     *
     * @return the length (&ge;0)
     */
    float maxLength();

    /**
     * Find the maximum and minimum coordinates for each axis among the Vector3f
     * values in this set.
     *
     * @param storeMaxima (not null, modified)
     * @param storeMinima (not null, modified)
     */
    void maxMin(Vector3f storeMaxima, Vector3f storeMinima);

    /**
     * Calculate the sample mean for each axis over the Vector3f values in this
     * set.
     *
     * @param storeResult (modified if not null)
     * @return the sample mean for each axis (either storeResult or a new
     * Vector3f)
     */
    Vector3f mean(Vector3f storeResult);

    /**
     * Calculate the number of Vector3f values in this set.
     *
     * @return the count (&ge;0)
     */
    public int numVectors();

    /**
     * Access the buffer containing all the Vector3f values in this set. No
     * further add() is allowed.
     *
     * @return a buffer, flipped but possibly not rewound
     */
    public FloatBuffer toBuffer();
}
