/*
 Copyright (c) 2020-2023, Stephen Gold
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

import com.jme3.math.FastMath;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.logging.Logger;
import jme3utilities.MyMesh;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * A 3-D, static, Triangles-mode mesh (with normals but no indices or texture
 * coordinates) that renders half of the cushions of a pool table.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class PoolHalfCushions extends Mesh {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(PoolHalfCushions.class.getName());
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected PoolHalfCushions() {
    }

    /**
     * Instantiate a half-set of cushions with the specified leg length, pocket
     * radius, and height.
     *
     * @param legLength the length of each leg, including the pocket radius (in
     * mesh units, &ge;pocketRadius)
     * @param pocketRadius the radius of the pocket (in mesh units, &ge;0,
     * &le;legLength)
     * @param numArcEdges the number of edges in a 90-degree arc of a pocket
     * (&gt;0)
     * @param yHeight the height of the cushions (in mesh units, &ge;0)
     */
    public PoolHalfCushions(float legLength, float pocketRadius,
            int numArcEdges, float yHeight) {
        Validate.inRange(
                legLength, "leg length", pocketRadius, Float.MAX_VALUE);
        Validate.inRange(pocketRadius, "pocket radius", 0f, legLength);
        Validate.positive(numArcEdges, "number of arc edges");

        int numQuadPairs = 4 * numArcEdges + 5;
        int numTriangles = 4 * numQuadPairs;
        int numVertices = MyMesh.vpt * numTriangles;
        int numFloats = MyVector3f.numAxes * numVertices;

        FloatBuffer posBuffer = BufferUtils.createFloatBuffer(numFloats);
        setBuffer(VertexBuffer.Type.Position, MyVector3f.numAxes, posBuffer);

        float x0 = 0f;
        float y0 = 0f;
        float y1 = y0 + yHeight;
        float z0 = legLength + pocketRadius;

        for (int edgeIndex = 0; edgeIndex <= numArcEdges; ++edgeIndex) {
            float theta = FastMath.HALF_PI * edgeIndex / numArcEdges;
            float x1 = pocketRadius * FastMath.sin(theta);
            float z1 = legLength + pocketRadius * FastMath.cos(theta);

            putQuadPair(posBuffer, x0, x1, y0, y1, z0, z1);

            x0 = x1;
            z0 = z1;
        }

        for (int edgeIndex = 0; edgeIndex <= 3 * numArcEdges; ++edgeIndex) {
            float theta = FastMath.HALF_PI * edgeIndex / numArcEdges;
            float x1 = 2f * legLength - pocketRadius * FastMath.cos(theta);
            float z1 = legLength + pocketRadius * FastMath.sin(theta);

            putQuadPair(posBuffer, x0, x1, y0, y1, z0, z1);

            x0 = x1;
            z0 = z1;
        }

        putQuadPair(posBuffer, x0, x0, y0, y1, z0, 0f);

        MyMesh.generateFacetNormals(this);
        updateBound();
        setStatic();
    }
    // *************************************************************************
    // private methods

    /**
     * Write a pair of quads, mirrored across the X-Y plane.
     *
     * @param buffer the buffer to write to (not null, modified)
     * @param x0 the first X coordinate
     * @param x1 the 2nd X coordinate
     * @param y0 the bottom Y coordinate
     * @param y1 the top Y coordinate
     * @param z0 the first Z coordinate
     * @param z1 the 2nd Z coordinate
     */
    private static void putQuadPair(FloatBuffer buffer, float x0, float x1,
            float y0, float y1, float z0, float z1) {
        buffer.put(x0).put(y0).put(z0);
        buffer.put(x0).put(y1).put(z0);
        buffer.put(x1).put(y0).put(z1);

        buffer.put(x1).put(y0).put(z1);
        buffer.put(x0).put(y1).put(z0);
        buffer.put(x1).put(y1).put(z1);

        buffer.put(x0).put(y1).put(-z0);
        buffer.put(x0).put(y0).put(-z0);
        buffer.put(x1).put(y0).put(-z1);

        buffer.put(x1).put(y1).put(-z1);
        buffer.put(x0).put(y1).put(-z0);
        buffer.put(x1).put(y0).put(-z1);
    }
}
