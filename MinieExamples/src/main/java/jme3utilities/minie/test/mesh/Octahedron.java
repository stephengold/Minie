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

import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A 3-D, static, Triangles-mode Mesh (without indices or texture coordinates)
 * that renders an octahedron. (An octahedron has 6 vertices and 8 triangular
 * faces.) TODO move to Heart library
 *
 * @author Stephen Gold sgold@sonic.net
 * @see jme3utilities.minie.test.mesh.Icosahedron
 */
public class Octahedron extends Mesh {
    // *************************************************************************
    // constants and loggers

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
            = Logger.getLogger(Octahedron.class.getName());
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected Octahedron() {
    }

    /**
     * Instantiate a regular octahedron with the specified radius.
     *
     * The center is at (0,0,0). All vertices lie on the axes. All triangles
     * face outward.
     *
     * @param radius the desired distance of the vertices from the center (in
     * mesh units, &gt;0)
     * @param generateNormals true &rarr; generate normals, false &rarr; no
     * normals
     */
    public Octahedron(float radius, boolean generateNormals) {
        Validate.positive(radius, "radius");

        FloatBuffer positionBuffer = BufferUtils.createFloatBuffer(
                -radius, 0f, 0f, // A
                0f, 0f, +radius, // Z
                0f, +radius, 0f, // Y

                +radius, 0f, 0f, // X
                0f, +radius, 0f, // Y
                0f, 0f, +radius, // Z

                0f, 0f, -radius, // C
                0f, +radius, 0f, // Y
                +radius, 0f, 0f, // X

                -radius, 0f, 0f, // A
                0f, +radius, 0f, // Y
                0f, 0f, -radius, // C

                0f, -radius, 0f, // B
                0f, 0f, -radius, // C
                +radius, 0f, 0f, // X

                0f, -radius, 0f, // B
                +radius, 0f, 0f, // X
                0f, 0f, +radius, // Z

                -radius, 0f, 0f, // A
                0f, -radius, 0f, // B
                0f, 0f, +radius, // Z

                -radius, 0f, 0f, // A
                0f, 0f, -radius, // C
                0f, -radius, 0f // B
        );
        setBuffer(VertexBuffer.Type.Position, numAxes, positionBuffer);
        int numFloats = positionBuffer.capacity();
        assert numFloats == 8 * vpt * numAxes;
        positionBuffer.limit(numFloats);

        if (generateNormals) {
            StarSlice.generateNormals(this); // TODO use MyMesh
        }

        updateBound();
        setStatic();
    }
}
