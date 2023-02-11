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
import com.jme3.scene.mesh.IndexBuffer;
import com.jme3.util.BufferUtils;
import java.nio.Buffer;
import java.nio.FloatBuffer;
import java.util.logging.Logger;
import jme3utilities.MyMesh;
import jme3utilities.Validate;

/**
 * A 2-D, static, Lines-mode Mesh (with indices) that renders a regular
 * rectangular grid. Unlike {@link com.jme3.scene.debug.Grid}, the resulting
 * mesh is centered on the mesh origin and includes a vertex for every crossing.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class NetGrid extends Mesh {
    // *************************************************************************
    // constants and loggers

    /**
     * number of axes in a vector
     */
    final private static int numAxes = 3;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(NetGrid.class.getName());
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected NetGrid() {
    }

    /**
     * Instantiate a grid in the X-Z plane, centered on (0,0,0).
     *
     * @param xLines the desired number of grid lines parallel to the X axis
     * (&ge;2)
     * @param zLines the desired number of grid lines parallel to the Z axis
     * (&ge;2)
     * @param lineSpacing the desired distance between adjacent grid lines (in
     * mesh units, &gt;0)
     */
    public NetGrid(int xLines, int zLines, float lineSpacing) {
        Validate.inRange(xLines, "X lines", 2, Integer.MAX_VALUE);
        Validate.inRange(zLines, "Z lines", 2, Integer.MAX_VALUE);
        Validate.positive(lineSpacing, "line spacing");

        setMode(Mode.Lines);

        int numVertices = xLines * zLines;
        int numFloats = numAxes * numVertices;
        FloatBuffer posBuffer = BufferUtils.createFloatBuffer(numFloats);
        setBuffer(VertexBuffer.Type.Position, numAxes, posBuffer);

        // Write the vertex locations:
        for (int xIndex = 0; xIndex < zLines; ++xIndex) {
            float x = (2 * xIndex - zLines + 1) * lineSpacing / 2f;
            for (int zIndex = 0; zIndex < xLines; ++zIndex) {
                float z = (2 * zIndex - xLines + 1) * lineSpacing / 2f;
                posBuffer.put(x).put(0f).put(z);
            }
        }
        assert posBuffer.position() == numFloats;
        posBuffer.flip();

        int numEdges = xLines * (zLines - 1) + (xLines - 1) * zLines;
        int numIndices = MyMesh.vpe * numEdges;
        IndexBuffer indexBuffer
                = IndexBuffer.createIndexBuffer(numVertices, numIndices);
        VertexBuffer.Format ibFormat = indexBuffer.getFormat();
        Buffer ibData = indexBuffer.getBuffer();
        setBuffer(VertexBuffer.Type.Index, 1, ibFormat, ibData);

        // Write vertex indices for edges that parallel the X axis:
        for (int zIndex = 0; zIndex < xLines; ++zIndex) {
            for (int xIndex = 0; xIndex < zLines - 1; ++xIndex) {
                int vi0 = zIndex + xLines * xIndex;
                int vi1 = vi0 + xLines;
                indexBuffer.put(vi0);
                indexBuffer.put(vi1);
            }
        }

        // Write vertex indices for edges the parallel the Z axis:
        for (int xIndex = 0; xIndex < zLines; ++xIndex) {
            for (int zIndex = 0; zIndex < xLines - 1; ++zIndex) {
                int vi0 = zIndex + xLines * xIndex;
                int vi1 = vi0 + 1;
                indexBuffer.put(vi0);
                indexBuffer.put(vi1);
            }
        }
        ibData.flip();
        assert indexBuffer.size() == numIndices;

        updateBound();
        setStatic();
    }
}
