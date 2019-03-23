/*
 Copyright (c) 2019, Stephen Gold
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package jme3utilities.minie.test;

import com.jme3.bullet.animation.VectorSet;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import java.nio.FloatBuffer;
import org.junit.Test;

/**
 * Test the VectorSet class.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestVectorSet {
    // *************************************************************************
    // constants

    final private static Vector3f[] testData = new Vector3f[]{
        new Vector3f(3f, 0f, -4f),
        new Vector3f(1f, 0f, 0f),
        new Vector3f(3f, 0f, -4f),
        new Vector3f(Float.POSITIVE_INFINITY, 0f, 0f)
    };
    // *************************************************************************
    // new methods exposed

    @Test
    public void testVectorSet() {
        float maxLength;
        String string;
        Vector3f max = new Vector3f();
        Vector3f mean = new Vector3f();
        Vector3f min = new Vector3f();

        VectorSet vs = new VectorSet(1);
        for (Vector3f v : testData) {
            assert !vs.contains(v);
        }
        maxLength = vs.maxLength();
        assert maxLength == 0f;
        vs.maxMin(max, min);
        assert max.x == Float.NEGATIVE_INFINITY;
        assert max.y == Float.NEGATIVE_INFINITY;
        assert max.z == Float.NEGATIVE_INFINITY;
        assert min.x == Float.POSITIVE_INFINITY;
        assert min.y == Float.POSITIVE_INFINITY;
        assert min.z == Float.POSITIVE_INFINITY;
        assert vs.numVectors() == 0;
        string = vs.toString();
        assert string != null;
        assert !string.isEmpty();
        /*
         * Add the first vector.
         */
        vs.add(testData[0]);
        for (Vector3f v : testData) {
            if (v.equals(testData[0])) {
                assert vs.contains(v);
            } else {
                assert !vs.contains(v);
            }
        }
        maxLength = vs.maxLength();
        assert FastMath.approximateEquals(maxLength, 5f);
        vs.maxMin(max, min);
        assert max.equals(testData[0]);
        assert min.equals(testData[0]);
        vs.mean(mean);
        assert mean.equals(testData[0]);
        assert vs.numVectors() == 1;
        string = vs.toString();
        assert string != null;
        assert !string.isEmpty();
        /*
         * Add the 2nd vector.
         */
        vs.add(testData[1]);
        for (Vector3f v : testData) {
            if (v.equals(testData[0]) || v.equals(testData[1])) {
                assert vs.contains(v);
            } else {
                assert !vs.contains(v);
            }
        }
        maxLength = vs.maxLength();
        assert FastMath.approximateEquals(maxLength, 5f);
        vs.maxMin(max, min);
        assert max.equals(new Vector3f(3f, 0f, 0f));
        assert min.equals(new Vector3f(1f, 0f, -4f));
        vs.mean(mean);
        assert mean.equals(new Vector3f(2f, 0f, -2f));
        assert vs.numVectors() == 2;
        string = vs.toString();
        assert string != null;
        assert !string.isEmpty();
        /*
         * Add the 3rd vector.
         */
        vs.add(testData[2]);
        for (Vector3f v : testData) {
            if (v.equals(testData[0]) || v.equals(testData[1])) {
                assert vs.contains(v);
            } else {
                assert !vs.contains(v);
            }
        }
        vs.covariance(null);
        maxLength = vs.maxLength();
        assert FastMath.approximateEquals(maxLength, 5f);
        vs.maxMin(max, min);
        assert max.equals(new Vector3f(3f, 0f, 0f));
        assert min.equals(new Vector3f(1f, 0f, -4f));
        vs.mean(mean);
        assert mean.equals(new Vector3f(2f, 0f, -2f));
        assert vs.numVectors() == 2;
        string = vs.toString();
        assert string != null;
        assert !string.isEmpty();
        /*
         * Add the 4th vector.
         */
        vs.add(testData[3]);
        for (Vector3f v : testData) {
            if (v.equals(testData[0]) || v.equals(testData[1])) {
                assert vs.contains(v);
            }
        }
        vs.covariance(null);
        maxLength = vs.maxLength();
        assert maxLength == Float.POSITIVE_INFINITY;
        vs.maxMin(max, min);
        assert max.equals(new Vector3f(Float.POSITIVE_INFINITY, 0f, 0f));
        assert min.equals(new Vector3f(1f, 0f, -4f));
        vs.mean(mean);
        assert mean.x == Float.POSITIVE_INFINITY;
        assert mean.y == 0f;
        assert vs.numVectors() == 3;
        string = vs.toString();
        assert string != null;
        assert !string.isEmpty();
        /*
         * Convert to a FloatBuffer.
         */
        FloatBuffer buf = vs.toBuffer();
        assert buf != null;
        assert buf.limit() == 3 * 3;
    }
}
