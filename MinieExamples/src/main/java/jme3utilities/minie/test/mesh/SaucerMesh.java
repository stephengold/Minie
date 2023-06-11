/*
 Copyright (c) 2019-2023, Stephen Gold
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
package jme3utilities.minie.test.mesh;

import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;

/**
 * A 3-D, static, Triangles-mode Mesh (with normals but no indices or texture
 * coordinates) that renders a flying-saucer shape (2 conical frusta joined
 * base-to-base).
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class SaucerMesh extends Mesh {
    // *************************************************************************
    // constants and loggers

    /**
     * number of axes in a vector
     */
    final private static int numAxes = 3;
    /**
     * number of vertices per triangle
     */
    final private static int vpt = 3;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(SaucerMesh.class.getName());
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected SaucerMesh() {
    }

    /**
     * Instantiate a flying-saucer shape.
     *
     * The center of the shape is at the origin. The rim lies in the X-Z plane.
     * The center of the base is at (0,-height/2,0). The base lies parallel to
     * the X-Z plane. All triangles face outward.
     *
     * @param numSides the desired number of sides for the base polygon (&ge;3)
     * @param baseRadius the desired radius of the base (in mesh units, &gt;0)
     * @param yHeight the desired total height on the Y axis (in mesh units,
     * &gt;0)
     */
    public SaucerMesh(int numSides, float baseRadius, float yHeight) {
        Validate.inRange(numSides, "number of sides", 3, Integer.MAX_VALUE);
        Validate.positive(baseRadius, "base radius");
        Validate.positive(yHeight, "total height");

        int numBaseTriangles = numSides - 2;
        int numTriangles = 2 * numBaseTriangles + 4 * numSides;
        int numFloats = numTriangles * vpt * numAxes;
        FloatBuffer positionBuffer = BufferUtils.createFloatBuffer(numFloats);
        setBuffer(VertexBuffer.Type.Position, numAxes, positionBuffer);
        FloatBuffer normalBuffer = BufferUtils.createFloatBuffer(numFloats);
        setBuffer(VertexBuffer.Type.Normal, numAxes, normalBuffer);

        float yTop = yHeight / 2f;
        float yBase = -yHeight / 2f;
        double interiorAngle = 2. * Math.PI / numSides; // in radians

        // the final vertex, where all the base triangles meet:
        double theta0 = (numSides - 1) * interiorAngle; // in radians
        float x0 = baseRadius * (float) Math.sin(theta0);
        float z0 = baseRadius * (float) Math.cos(theta0);

        // the slope of the (upper) sides:
        float tangent = baseRadius / yHeight;
        float cosine = 1f / MyMath.hypotenuse(1f, tangent);
        float ny = cosine * tangent;

        for (int sideIndex = 0; sideIndex < numSides; ++sideIndex) {
            double theta1 = sideIndex * interiorAngle; // in radians
            float x1 = baseRadius * (float) Math.sin(theta1);
            float z1 = baseRadius * (float) Math.cos(theta1);

            int nextSideIndex = sideIndex + 1;
            double theta2 = nextSideIndex * interiorAngle; // in radians
            float x2 = baseRadius * (float) Math.sin(theta2);
            float z2 = baseRadius * (float) Math.cos(theta2);

            // coordinates of vertices in the X-Z plane:
            float xx1 = 2f * x1;
            float xx2 = 2f * x2;
            float zz1 = 2f * z1;
            float zz2 = 2f * z2;

            // components of the normal vectors:
            float nx1 = cosine * x1 / baseRadius;
            float nz1 = cosine * z1 / baseRadius;
            float nx2 = cosine * x2 / baseRadius;
            float nz2 = cosine * z2 / baseRadius;

            // 2 curved triangles connecting the base to the rim:
            positionBuffer.put(x1).put(yBase).put(z1);
            positionBuffer.put(x2).put(yBase).put(z2);
            positionBuffer.put(xx1).put(0f).put(zz1);

            normalBuffer.put(nx1).put(-ny).put(nz1);
            normalBuffer.put(nx2).put(-ny).put(nz2);
            normalBuffer.put(nx1).put(-ny).put(nz1);

            positionBuffer.put(xx2).put(0f).put(zz2);
            positionBuffer.put(xx1).put(0f).put(zz1);
            positionBuffer.put(x2).put(yBase).put(z2);

            normalBuffer.put(nx2).put(-ny).put(nz2);
            normalBuffer.put(nx1).put(-ny).put(nz1);
            normalBuffer.put(nx2).put(-ny).put(nz2);

            // 2 curved triangles connecting the rim to the top:
            positionBuffer.put(x2).put(yTop).put(z2);
            positionBuffer.put(x1).put(yTop).put(z1);
            positionBuffer.put(xx1).put(0f).put(zz1);

            normalBuffer.put(nx2).put(ny).put(nz2);
            normalBuffer.put(nx1).put(ny).put(nz1);
            normalBuffer.put(nx1).put(ny).put(nz1);

            positionBuffer.put(xx1).put(0f).put(zz1);
            positionBuffer.put(xx2).put(0f).put(zz2);
            positionBuffer.put(x2).put(yTop).put(z2);

            normalBuffer.put(nx1).put(ny).put(nz1);
            normalBuffer.put(nx2).put(ny).put(nz2);
            normalBuffer.put(nx2).put(ny).put(nz2);

            if (sideIndex < numBaseTriangles) {
                // top triangle:
                positionBuffer.put(x1).put(yTop).put(z1);
                positionBuffer.put(x2).put(yTop).put(z2);
                positionBuffer.put(x0).put(yTop).put(z0);

                normalBuffer.put(0f).put(1f).put(0f);
                normalBuffer.put(0f).put(1f).put(0f);
                normalBuffer.put(0f).put(1f).put(0f);

                // base triangle:
                positionBuffer.put(x0).put(yBase).put(z0);
                positionBuffer.put(x2).put(yBase).put(z2);
                positionBuffer.put(x1).put(yBase).put(z1);

                normalBuffer.put(0f).put(-1f).put(0f);
                normalBuffer.put(0f).put(-1f).put(0f);
                normalBuffer.put(0f).put(-1f).put(0f);
            }
        }
        positionBuffer.flip();
        assert positionBuffer.limit() == positionBuffer.capacity();

        normalBuffer.flip();
        assert normalBuffer.limit() == positionBuffer.capacity();

        updateBound();
        setStatic();
    }
}
