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
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;

/**
 * A 3-D, static, Triangles-mode mesh (without indices or texture coordinates)
 * that renders an icosahedron. (An icosahedron has 12 vertices and 20
 * triangular faces.) TODO move to Heart library
 *
 * @author Stephen Gold sgold@sonic.net
 * @see jme3utilities.mesh.Icosphere
 * @see jme3utilities.minie.test.mesh.Octahedron
 */
public class Icosahedron extends Mesh {
    // *************************************************************************
    // constants and loggers

    /**
     * golden ratio = 1.618...
     */
    final public static float phi = (1f + FastMath.sqrt(5f)) / 2f;
    /**
     * number of axes in a vector
     */
    final private static int numAxes = 3;
    /**
     * number of vertices per triangle
     */
    final private static int vpt = 3; // TODO use MyMesh
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(Icosahedron.class.getName());
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected Icosahedron() {
    }

    /**
     * Instantiate a regular icosahedron with the specified radius.
     *
     * The center is at (0,0,0). All triangles face outward.
     *
     * @param radius the desired distance of the vertices from the center (in
     * mesh units, &gt;0)
     * @param generateNormals true &rarr; generate normals, false &rarr; no
     * normals
     */
    public Icosahedron(float radius, boolean generateNormals) {
        Validate.positive(radius, "radius");

        float denom = MyMath.hypotenuse(1f, phi) / radius;
        float a = 1f / denom; // small offset
        float b = phi / denom; // big offset

        FloatBuffer positionBuffer = BufferUtils.createFloatBuffer(
                -a, +b, 0f, // p0
                -b, 0f, +a, // p11
                0f, +a, +b, // p5

                -a, +b, 0f, // p0
                0f, +a, +b, // p5
                +a, +b, 0f, // p1

                -a, +b, 0f, // p0
                +a, +b, 0f, // p1
                0f, +a, -b, // p7

                -a, +b, 0f, // p0
                0f, +a, -b, // p7
                -b, 0f, -a, // p10

                -a, +b, 0f, // p0
                -b, 0f, -a, // p10
                -b, 0f, +a, // p11

                +a, +b, 0f, // p1
                0f, +a, +b, // p5
                +b, 0f, +a, // p9

                0f, +a, +b, // p5
                -b, 0f, +a, // p11
                +0f, -a, +b, // p4

                -b, 0f, +a, // p11
                -b, 0f, -a, // p10
                -a, -b, 0f, // p2

                -b, 0f, -a, // p10
                0f, +a, -b, // p7
                0f, -a, -b, // p6

                0f, +a, -b, // p7
                +a, +b, 0f, // p1
                +b, 0f, -a, // p8

                +a, -b, 0f, // p3
                +b, 0f, +a, // p9
                0f, -a, +b, // p4

                +a, -b, 0f, // p3
                0f, -a, +b, // p4
                -a, -b, 0f, // p2

                +a, -b, 0f, // p3
                -a, -b, 0f, // p2
                0f, -a, -b, // p6

                +a, -b, 0f, // p3
                0f, -a, -b, // p6
                +b, 0f, -a, // p8

                a, -b, 0f, // p3
                b, 0f, -a, // p8
                b, 0f, a, // p9

                0f, -a, b, // p4
                b, 0f, a, // p9
                0f, a, b, // p5

                -a, -b, 0f, // p2
                0f, -a, b, // p4
                -b, 0f, a, // p11

                0f, -a, -b, // p6
                -a, -b, 0f, // p2
                -b, 0f, -a, // p10

                b, 0f, -a, // p8
                0f, -a, -b, // p6
                0f, a, -b, // p7

                b, 0f, a, // p9
                b, 0f, -a, // p8
                a, b, 0f // p1
        );
        setBuffer(VertexBuffer.Type.Position, numAxes, positionBuffer);
        int numFloats = positionBuffer.capacity();
        assert numFloats == 20 * vpt * numAxes;
        positionBuffer.limit(numFloats);

        if (generateNormals) {
            StarSlice.generateNormals(this); // TODO use MyMesh
        }

        updateBound();
        setStatic();
    }
}
