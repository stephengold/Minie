/*
Copyright (c) 2016, Riccardo Balbo
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package vhacd;

import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;

/**
 * An indexed, 3-D, convex hull based on V-HACD's ConvexHull.
 */
public class VHACDHull {
    // *************************************************************************
    // fields

    /**
     * vertex locations (length a multiple of 3)
     */
    final private float[] positions;
    /**
     * vertex indices for each triangular face (length a multiple of 3)
     */
    final private int[] indexes;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a hull based on the identified ConvexHull.
     *
     * @param hullId the native ID of a ConvexHull (not zero)
     */
    VHACDHull(long hullId) {
        assert hullId != 0L;

        int numFloats = getNumFloats(hullId);
        FloatBuffer floatBuffer = BufferUtils.createFloatBuffer(numFloats);
        getPositions(hullId, floatBuffer);
        positions = new float[numFloats];
        for (int floatIndex = 0; floatIndex < numFloats; ++floatIndex) {
            positions[floatIndex] = floatBuffer.get(floatIndex);
        }

        int numInts = getNumInts(hullId);
        IntBuffer intBuffer = BufferUtils.createIntBuffer(numInts);
        getIndices(hullId, intBuffer);
        indexes = new int[numInts];
        for (int intIndex = 0; intIndex < numInts; ++intIndex) {
            indexes[intIndex] = intBuffer.get(intIndex);
        }
    }

    /**
     * Instantiate a hull with the specfied vertex positions and indices.
     *
     * @param positions (alias created)
     * @param indexes (alias created)
     */
    protected VHACDHull(float[] positions, int[] indexes) {
        this.positions = positions;
        this.indexes = indexes;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the vertex positions to a new array.
     *
     * @return the new array
     */
    public float[] clonePositions() {
        int numFloats = positions.length;
        float[] result = new float[numFloats];
        System.arraycopy(positions, 0, result, 0, numFloats);

        return result;
    }
    // *************************************************************************
    // native private methods

    native private static void getIndices(long hullId, IntBuffer storeBuffer);

    native private static int getNumFloats(long hullId);

    native private static int getNumInts(long hullId);

    native private static void getPositions(long hullId,
            FloatBuffer storeBuffer);
}
