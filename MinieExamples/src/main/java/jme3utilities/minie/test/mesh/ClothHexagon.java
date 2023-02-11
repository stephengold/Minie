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

import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.mesh.IndexBuffer;
import com.jme3.util.BufferUtils;
import java.nio.Buffer;
import java.nio.FloatBuffer;
import java.util.logging.Logger;
import jme3utilities.MyMesh;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;

/**
 * A dynamic, Triangles-mode Mesh (with indices and normals but no texture
 * coordinates) that renders a subdivided regular hexagon.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class ClothHexagon extends Mesh {
    // *************************************************************************
    // constants and loggers

    /**
     * Pi/3
     */
    final private static double oneThirdPi = Math.PI / 3f;
    /**
     * number of axes in a vector
     */
    final private static int numAxes = 3;
    /**
     * number of sides in a hexagon
     */
    final private static int numSides = 6;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(ClothHexagon.class.getName());
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected ClothHexagon() {
    }

    /**
     * Instantiate a hexagonal lattice in the X-Z plane, centered on a vertex at
     * (0,0,0), with 1/3 of its edges parallel to the X axis.
     *
     * @param numRings the desired number of rings around the central vertex
     * (&gt;0)
     * @param vertexSpacing the desired distance between adjacent vertices (in
     * mesh units, &gt;0)
     */
    public ClothHexagon(int numRings, float vertexSpacing) {
        Validate.positive(numRings, "number of rings");
        Validate.positive(vertexSpacing, "vertex spacing");

        int numVertices = 1 + (numSides / 2) * numRings * (numRings + 1);
        FloatBuffer posBuffer
                = BufferUtils.createFloatBuffer(numAxes * numVertices);
        setBuffer(VertexBuffer.Type.Position, numAxes, posBuffer);

        // Write the vertex locations:
        posBuffer.put(0f).put(0f).put(0f);

        for (int ringIndex = 0; ringIndex < numRings; ++ringIndex) {
            float cornerR = vertexSpacing * (ringIndex + 1);

            for (int sideIndex = 0; sideIndex < numSides; ++sideIndex) {
                double theta0
                        = sideIndex * oneThirdPi; // radians from +X around -Y
                float cornerX = cornerR * (float) Math.cos(theta0);
                float cornerZ = cornerR * (float) Math.sin(theta0);

                double theta2 = (sideIndex + 2)
                        * oneThirdPi; // radians from +X around -Y
                float stepX = vertexSpacing * (float) Math.cos(theta2);
                float stepZ = vertexSpacing * (float) Math.sin(theta2);

                for (int stepIndex = 0; stepIndex <= ringIndex; ++stepIndex) {
                    float x = cornerX + stepIndex * stepX;
                    float z = cornerZ + stepIndex * stepZ;
                    posBuffer.put(x).put(0f).put(z);
                }
            }
        }
        assert posBuffer.position() == numAxes * numVertices;
        posBuffer.flip();

        FloatBuffer normBuffer
                = BufferUtils.createFloatBuffer(numAxes * numVertices);
        setBuffer(VertexBuffer.Type.Normal, numAxes, normBuffer);

        // Write the normals:
        for (int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex) {
            normBuffer.put(0f).put(1f).put(0f);
        }
        assert normBuffer.position() == numAxes * numVertices;
        normBuffer.flip();

        int numTriangles = numSides * numRings * numRings;
        int numIndices = MyMesh.vpt * numTriangles;
        IndexBuffer indexBuffer
                = IndexBuffer.createIndexBuffer(numVertices, numIndices);
        VertexBuffer.Format ibFormat = indexBuffer.getFormat();
        Buffer ibData = indexBuffer.getBuffer();
        setBuffer(VertexBuffer.Type.Index, MyMesh.vpt, ibFormat, ibData);

        // Write vertex indices for triangles:
        int ivi = 0; // index of the current vertex in the inner ring
        int ovi = 1; // index of the current vertex in the outer ring
        for (int ringIndex = 0; ringIndex < numRings; ++ringIndex) {
            int startIvi = ivi;
            int startOvi = ovi;
            for (int sideIndex = 0; sideIndex < numSides; ++sideIndex) {
                for (int stepI = 0; stepI < ringIndex - 1; ++stepI) {
                    indexBuffer.put(ovi);
                    indexBuffer.put(ivi);
                    indexBuffer.put(ovi + 1);

                    indexBuffer.put(ivi);
                    indexBuffer.put(ivi + 1);
                    indexBuffer.put(ovi + 1);
                    ++ivi;
                    ++ovi;
                }
                int nextSide = MyMath.modulo(sideIndex + 1, numSides);
                int nivi = startIvi + nextSide * ringIndex;
                int novi = startOvi + nextSide * (ringIndex + 1);
                if (ringIndex > 0) {
                    indexBuffer.put(ovi);
                    indexBuffer.put(ivi);
                    indexBuffer.put(ovi + 1);

                    indexBuffer.put(ivi);
                    indexBuffer.put(nivi);
                    indexBuffer.put(ovi + 1);
                    ++ivi;
                    ++ovi;
                }
                indexBuffer.put(ovi);
                indexBuffer.put(nivi);
                indexBuffer.put(novi);
                ++ovi;
            }
            if (ringIndex == 0) {
                ++ivi;
            }
        }
        ibData.flip();
        assert indexBuffer.size() == numIndices;

        updateBound();
        setDynamic();

        assert getMode() == Mesh.Mode.Triangles;
        assert MyMesh.hasIndices(this);
        assert MyMesh.hasNormals(this);
        assert !MyMesh.hasUV(this);
    }
}
