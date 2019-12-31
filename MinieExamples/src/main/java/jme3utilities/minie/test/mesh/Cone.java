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
package jme3utilities.minie.test.mesh;

import com.jme3.math.FastMath;
import com.jme3.math.Triangle;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;

/**
 * A 3-D, static, Triangles-mode Mesh (with normals but no indices or texture
 * coordinates) that renders a pyramid or cone.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class Cone extends Mesh {
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
            = Logger.getLogger(Cone.class.getName());
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected Cone() {
    }

    /**
     * Instantiate a right circular cone or a regular pyramid.
     *
     * The apex is at (0,2*height/2). The center of the base is at
     * (0,-height/2,0). The base lies parallel to the X-Z plane. All triangles
     * face outward.
     *
     * @param numSides the number of sides for the base polygon (&ge;3)
     * @param radius the radius of the base (in mesh units, &gt;0)
     * @param yHeight the total height of the cone/pyramid on the Y axis (in
     * mesh units, &gt;0)
     * @param pyramidFlag true for a pyramid (with flat triangles), false for a
     * cone (with curved triangles)
     */
    public Cone(int numSides, float radius, float yHeight,
            boolean pyramidFlag) {
        Validate.inRange(numSides, "number of sides", 3, Integer.MAX_VALUE);
        Validate.positive(radius, "radius");
        Validate.positive(yHeight, "height");

        int numBaseTriangles = numSides - 2;
        int numTriangles = numBaseTriangles + numSides;
        int numFloats = numTriangles * vpt * numAxes;
        FloatBuffer positionBuffer = BufferUtils.createFloatBuffer(numFloats);
        setBuffer(VertexBuffer.Type.Position, numAxes, positionBuffer);
        FloatBuffer normalBuffer = BufferUtils.createFloatBuffer(numFloats);
        setBuffer(VertexBuffer.Type.Normal, numAxes, normalBuffer);

        float yApex = yHeight / 2f;
        float yBase = -yHeight / 2f;
        float interiorAngle = FastMath.TWO_PI / numSides; // in radians

        Triangle triangle = new Triangle();
        Vector3f p1 = new Vector3f();
        Vector3f p2 = new Vector3f();
        Vector3f p3 = new Vector3f();

        for (int sideIndex = 0; sideIndex < numSides; ++sideIndex) {
            float theta1 = sideIndex * interiorAngle; // in radians
            float x1 = radius * FastMath.sin(theta1);
            float z1 = radius * FastMath.cos(theta1);

            int nextSideIndex = sideIndex + 1;
            float theta2 = nextSideIndex * interiorAngle; // in radians
            float x2 = radius * FastMath.sin(theta2);
            float z2 = radius * FastMath.cos(theta2);

            p1.set(x1, yBase, z1);
            p2.set(x2, yBase, z2);
            p3.set(0f, yApex, 0f);

            positionBuffer.put(p1.x).put(p1.y).put(p1.z);
            positionBuffer.put(p2.x).put(p2.y).put(p2.z);
            positionBuffer.put(p3.x).put(p3.y).put(p3.z);

            if (pyramidFlag) { // flat triangle for a pyramid
                triangle.set(p1, p2, p3);
                triangle.setNormal(null); // work around JME issue #957
                Vector3f n = triangle.getNormal();
                for (int j = 0; j < vpt; ++j) {
                    normalBuffer.put(n.x).put(n.y).put(n.z);
                }

            } else { // curved triangle for a cone
                float tangent = radius / yHeight;
                float cosine = 1f / MyMath.hypotenuse(1f, tangent);
                float ny = cosine * tangent;

                float nx = cosine * x1 / radius;
                float nz = cosine * z1 / radius;
                normalBuffer.put(nx).put(ny).put(nz);

                nx = cosine * x2 / radius;
                nz = cosine * z2 / radius;
                normalBuffer.put(nx).put(ny).put(nz);

                float theta3 = (theta1 + theta2) / 2f;
                nx = cosine * FastMath.sin(theta3);
                nz = cosine * FastMath.cos(theta3);
                normalBuffer.put(nx).put(ny).put(nz);
            }

            if (sideIndex < numBaseTriangles) {
                float theta3 = (numSides - 1) * interiorAngle; // in radians
                float x3 = radius * FastMath.sin(theta3);
                float z3 = radius * FastMath.cos(theta3);

                positionBuffer.put(x3).put(yBase).put(z3);
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
