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
import java.nio.IntBuffer;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A static, triangle-mode mesh that renders a subdivided rectangle in the X-Z
 * plane.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class ClothGrid extends Mesh {
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
            = Logger.getLogger(ClothGrid.class.getName());
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public ClothGrid() {
    }

    /**
     * Instantiate a grid in the X-Z plane, centered on the mesh origin.
     *
     * @param xLines the number of grid lines parallel to the X axis (&ge;2)
     * @param zLines the number of grid lines parallel to the Z axis (&ge;2)
     * @param separation the distance between adjacent grid lines (in mesh
     * units, &gt;0)
     */
    public ClothGrid(int xLines, int zLines, float separation) {
        Validate.inRange(xLines, "X lines", 2, Integer.MAX_VALUE);
        Validate.inRange(xLines, "Z lines", 2, Integer.MAX_VALUE);
        Validate.positive(separation, "separation");

        int numVertices = xLines * zLines;
        FloatBuffer posBuffer
                = BufferUtils.createFloatBuffer(numAxes * numVertices);
        setBuffer(VertexBuffer.Type.Position, numAxes, posBuffer);
        /*
         * Write the vertex locations:
         */
        for (int xIndex = 0; xIndex < zLines; ++xIndex) {
            float x = (2 * xIndex - zLines + 1) * separation / 2f;
            for (int zIndex = 0; zIndex < xLines; ++zIndex) {
                float z = (2 * zIndex - xLines + 1) * separation / 2f;
                posBuffer.put(x).put(0f).put(z);
            }
        }
        assert posBuffer.position() == numAxes * numVertices;
        posBuffer.flip();

        FloatBuffer normBuffer
                = BufferUtils.createFloatBuffer(numAxes * numVertices);
        setBuffer(VertexBuffer.Type.Normal, numAxes, normBuffer);
        /*
         * Write the normals:
         */
        for (int xIndex = 0; xIndex < zLines; ++xIndex) {
            for (int zIndex = 0; zIndex < xLines; ++zIndex) {
                normBuffer.put(0f).put(1f).put(0f);
            }
        }
        assert normBuffer.position() == numAxes * numVertices;
        normBuffer.flip();

        int numTriangles = 2 * (xLines - 1) * (zLines - 1);
        IntBuffer indexBuffer = BufferUtils.createIntBuffer(vpt * numTriangles);
        setBuffer(VertexBuffer.Type.Index, vpt, indexBuffer);
        /*
         * Write vertex indices for triangles:
         */
        for (int zIndex = 0; zIndex < xLines - 1; ++zIndex) {
            for (int xIndex = 0; xIndex < zLines - 1; ++xIndex) {
                // four vertices forming a square
                int vi0 = zIndex + xLines * xIndex;
                int vi1 = vi0 + 1;
                int vi2 = vi0 + xLines;
                int vi3 = vi1 + xLines;
                indexBuffer.put(vi0).put(vi1).put(vi2);
                indexBuffer.put(vi3).put(vi2).put(vi1);
            }
        }
        assert indexBuffer.position() == vpt * numTriangles;
        indexBuffer.flip();

        updateBound();
        setStatic();
    }
}
