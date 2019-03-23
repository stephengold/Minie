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
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;

/**
 * A simplified collection of Vector3f values without duplicates.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class VectorSet2 implements VectorSet {
    // *************************************************************************
    // constants and loggers

    /**
     * number of axes in a Vector3f
     */
    final private static int numAxes = 3;
    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(VectorSet2.class.getName());
    // *************************************************************************
    // fields

    /**
     * buffer to hold the Vector3f values
     */
    private FloatBuffer buffer;
    /**
     * look up last buffer position for hash index
     */
    private int endPosition[];
    /**
     * number of enlargements since last clearStats()
     */
    static int numEnlargements = 0;
    /**
     * number of reads since last clearStats()
     */
    static int numReads = 0;
    /**
     * number of searches since last clearStats()
     */
    static int numSearches = 0;
    /**
     * look up first buffer position plus 1 for hash index
     */
    private int startPositionPlus1[];
    /**
     * system milliseconds as of last clearStats()
     */
    static long resetMillis = 0L;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an empty set with the specified initial capacity.
     *
     * @param numVectors the number of vectors this set can hold without
     * enlargement (&gt;0)
     */
    public VectorSet2(int numVectors) {
        Validate.positive(numVectors, "number of vectors");
        allocate(numVectors);
        flip();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Reset the hashing statistics.
     */
    public static void clearStats() {
        numEnlargements = 0;
        numReads = 0;
        numSearches = 0;
        resetMillis = System.currentTimeMillis();
    }

    /**
     * Print the hashing statistics.
     *
     * @param tag (not null)
     */
    public static void dumpStats(String tag) {
        long msec = System.currentTimeMillis() - resetMillis;
        String msg = String.format(
                "%s %d enlargement%s, %d search%s, and %d read%s in %d msec",
                tag, numEnlargements, (numEnlargements == 1) ? "" : "s",
                numSearches, (numSearches == 1) ? "" : "es",
                numReads, (numReads == 1) ? "" : "s", msec);
        System.out.println(msg);
    }
    // *************************************************************************
    // VectorSet methods

    /**
     * Add the value of the specified Vector3f to this set.
     *
     * @param vector the value to add (not null, unaffected)
     */
    @Override
    public void add(Vector3f vector) {
        if (startPositionPlus1 == null) {
            throw new IllegalStateException("toBuffer() has been invoked.");
        }

        int hashCode = vector.hashCode();
        if (!contains(vector, hashCode)) {
            unflip();
            if (buffer.remaining() < numAxes) {
                enlarge();
                assert buffer.remaining() >= numAxes;
            }
            add(vector, hashCode);
            flip();
        }
    }

    /**
     * Test whether this set contains the value of the specified Vector3f.
     *
     * @param vector the value to find (not null, unaffected)
     * @return true if found, otherwise false
     */
    @Override
    public boolean contains(Vector3f vector) {
        int hashCode = vector.hashCode();
        boolean result = contains(vector, hashCode);

        return result;
    }

    /**
     * Calculate the sample covariance of the Vector3f values in this set.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the unbiased sample covariance (either storeResult or a new
     * matrix, not null)
     */
    @Override
    public Matrix3f covariance(Matrix3f storeResult) {
        Matrix3f result = (storeResult == null) ? new Matrix3f() : storeResult;
        int numSamples = numVectors();
        assert numSamples > 1 : numSamples;

        Vector3f sampleMean = mean(null);
        /*
         * Accumulate sums in the upper triangle of the matrix.
         */
        result.zero();
        float[] aboveMean = new float[numAxes];
        buffer.rewind();
        while (buffer.hasRemaining()) {
            aboveMean[0] = buffer.get() - sampleMean.x;
            aboveMean[1] = buffer.get() - sampleMean.y;
            aboveMean[2] = buffer.get() - sampleMean.z;
            for (int rowI = 0; rowI < numAxes; ++rowI) {
                for (int columnI = rowI; columnI < numAxes; ++columnI) {
                    float sum = result.get(rowI, columnI);
                    sum += aboveMean[rowI] * aboveMean[columnI];
                    result.set(rowI, columnI, sum);
                }
            }
        }
        /*
         * Multiply sums by 1/(N-1) and fill in the lower triangle.
         */
        float nMinus1 = numSamples - 1;
        for (int rowI = 0; rowI < numAxes; ++rowI) {
            for (int columnI = rowI; columnI < numAxes; ++columnI) {
                float sum = result.get(rowI, columnI);
                float element = sum / nMinus1;
                result.set(rowI, columnI, element);
                result.set(columnI, rowI, element);
            }
        }

        return result;
    }

    /**
     * Find the length of the longest Vector3f value in this set.
     *
     * @return the length (&ge;0)
     */
    @Override
    public float maxLength() {
        double maxLengthSquared = 0.0;
        Vector3f tempVector = new Vector3f();

        buffer.rewind();
        while (buffer.hasRemaining()) {
            tempVector.x = buffer.get();
            tempVector.y = buffer.get();
            tempVector.z = buffer.get();
            double lengthSquared = MyVector3f.lengthSquared(tempVector);
            if (lengthSquared > maxLengthSquared) {
                maxLengthSquared = lengthSquared;
            }
        }

        float length = (float) Math.sqrt(maxLengthSquared);
        assert length >= 0f : length;
        return length;
    }

    /**
     * Find the maximum and minimum coordinates for each axis among the Vector3f
     * values in this set.
     *
     * @param storeMaxima (not null, modified)
     * @param storeMinima (not null, modified)
     */
    @Override
    public void maxMin(Vector3f storeMaxima, Vector3f storeMinima) {
        storeMaxima.set(Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY,
                Float.NEGATIVE_INFINITY);
        storeMinima.set(Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY,
                Float.POSITIVE_INFINITY);
        Vector3f tempVector = new Vector3f();

        buffer.rewind();
        while (buffer.hasRemaining()) {
            tempVector.x = buffer.get();
            tempVector.y = buffer.get();
            tempVector.z = buffer.get();
            MyVector3f.accumulateMinima(storeMinima, tempVector);
            MyVector3f.accumulateMaxima(storeMaxima, tempVector);
        }
    }

    /**
     * Calculate the sample mean for each axis over the Vector3f values in this
     * set.
     *
     * @param storeResult (modified if not null)
     * @return the sample mean for each axis (either storeResult or a new
     * Vector3f)
     */
    @Override
    public Vector3f mean(Vector3f storeResult) {
        int numVectors = numVectors();
        assert numVectors > 0 : numVectors;
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        result.zero();

        buffer.rewind();
        while (buffer.hasRemaining()) {
            float x = buffer.get();
            float y = buffer.get();
            float z = buffer.get();
            result.addLocal(x, y, z);
        }
        result.divideLocal(numVectors);

        return result;
    }

    /**
     * Calculate the number of Vector3f values in this set.
     *
     * @return the count (&ge;0)
     */
    @Override
    public int numVectors() {
        int limit = buffer.limit();
        int numFloats = (limit < buffer.capacity()) ? limit : buffer.position();
        assert numFloats % 3 == 0 : numFloats;
        int numVectors = numFloats / 3;

        assert numVectors >= 0 : numVectors;
        return numVectors;
    }

    /**
     * Access the buffer containing all the Vector3f values in this set. No
     * further add() is allowed.
     *
     * @return the pre-existing buffer, flipped but possibly not rewound
     */
    @Override
    public FloatBuffer toBuffer() {
        startPositionPlus1 = null;
        endPosition = null;

        return buffer;
    }
    // *************************************************************************
    // private methods

    /**
     * Add the specified vector's value to this set without checking for
     * capacity or duplication. The buffer cannot be in a flipped state.
     *
     * @param vector the value to add (not null, unaffected)
     * @param hasCode the hash code of vector
     */
    private void add(Vector3f vector, int hashCode) {
        assert buffer.limit() == buffer.capacity();

        int position = buffer.position();
        int hashIndex = MyMath.modulo(hashCode, startPositionPlus1.length);
        int spp1 = startPositionPlus1[hashIndex];
        if (spp1 == 0) {
            startPositionPlus1[hashIndex] = position + 1;
        }
        endPosition[hashIndex] = position;

        buffer.put(vector.x);
        buffer.put(vector.y);
        buffer.put(vector.z);
    }

    /**
     * Initialize an empty, unflipped set with the specified initial capacity.
     *
     * @param numVectors (&gt;0)
     */
    private void allocate(int numVectors) {
        assert numVectors > 0 : numVectors;

        int numFloats = numAxes * numVectors + 1;
        buffer = BufferUtils.createFloatBuffer(numFloats);
        startPositionPlus1 = new int[numFloats]; // initialized to all 0s
        endPosition = new int[numFloats];
    }

    /**
     * Test whether the specified vector's value is in the set.
     *
     * @param vector the value to find (not null, unaffected)
     * @param hashCode the hash code of vector
     * @return true if found, otherwise false
     */
    private boolean contains(Vector3f vector, int hashCode) {
        boolean result = false;
        int hashIndex = MyMath.modulo(hashCode, startPositionPlus1.length);
        int spp1 = startPositionPlus1[hashIndex];
        if (spp1 != 0) {
            int end = endPosition[hashIndex];
            buffer.position(spp1 - 1);
            while (buffer.position() <= end) {
                float x = buffer.get();
                float y = buffer.get();
                float z = buffer.get();
                if (x == vector.x && y == vector.y && z == vector.z) {
                    result = true;
                    break;
                }
            }
            ++numSearches;
            numReads += (end - spp1 + 1) / 3;
        }

        return result;
    }

    /**
     * Quadruple the capacity of the buffer, which must be unflipped.
     */
    private void enlarge() {
        Vector3f tempVector = new Vector3f();
        int numVectors = numVectors();

        FloatBuffer oldBuffer = toBuffer();
        allocate(4 * numVectors);
        oldBuffer.flip();
        while (oldBuffer.hasRemaining()) {
            tempVector.x = oldBuffer.get();
            tempVector.y = oldBuffer.get();
            tempVector.z = oldBuffer.get();
            int hashCode = tempVector.hashCode();
            add(tempVector, hashCode);
        }
        assert numVectors() == numVectors;
        ++numEnlargements;
    }

    /**
     * Switch from writing to reading.
     */
    private void flip() {
        assert buffer.limit() == buffer.capacity();
        buffer.limit(buffer.position());
        assert buffer.limit() != buffer.capacity();
    }

    /**
     * Switch from reading to writing.
     */
    private void unflip() {
        assert buffer.limit() != buffer.capacity();
        buffer.position(buffer.limit());
        buffer.limit(buffer.capacity());
        assert buffer.limit() == buffer.capacity();
    }
}
