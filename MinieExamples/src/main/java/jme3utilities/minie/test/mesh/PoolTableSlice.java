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
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * A 2-D, static, TriangleFan-mode mesh (with normals but no indices or texture
 * coordinates) that renders 1/8th of the playing surface of a pool table in the
 * X-Z plane.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class PoolTableSlice extends Mesh {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(PoolTableSlice.class.getName());
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected PoolTableSlice() {
    }

    /**
     * Instantiate a slice with the specified leg length and pocket radius.
     *
     * @param legLength the length of each leg, including the pocket radius (in
     * mesh units, &ge;pocketRadius)
     * @param pocketRadius the radius of the pocket (in mesh units, &ge;0,
     * &le;legLength)
     * @param numArcEdges the number of edges in a 90-degree arc of a pocket
     * (&gt;0)
     */
    public PoolTableSlice(
            float legLength, float pocketRadius, int numArcEdges) {
        Validate.inRange(legLength, "leg length", pocketRadius,
                Float.MAX_VALUE);
        Validate.inRange(pocketRadius, "pocket radius", 0f, legLength);
        Validate.positive(numArcEdges, "number of arc edges");

        setMode(Mode.TriangleFan);

        int numVertices = numArcEdges + 4;
        int numFloats = MyVector3f.numAxes * numVertices;

        FloatBuffer posBuffer = BufferUtils.createFloatBuffer(numFloats);
        setBuffer(VertexBuffer.Type.Position, MyVector3f.numAxes, posBuffer);

        final float y = 0f;
        posBuffer.put(0f).put(y).put(0f);
        posBuffer.put(0f).put(y).put(legLength);

        for (int edgeIndex = 0; edgeIndex <= numArcEdges; ++edgeIndex) {
            float theta = FastMath.HALF_PI * (numArcEdges - edgeIndex)
                    / numArcEdges;
            float x = legLength - pocketRadius * FastMath.sin(theta);
            float z = legLength - pocketRadius * FastMath.cos(theta);
            posBuffer.put(x).put(y).put(z);
        }

        posBuffer.put(legLength).put(y).put(0f);
        assert posBuffer.position() == posBuffer.capacity();

        FloatBuffer normBuffer = BufferUtils.createFloatBuffer(numFloats);
        setBuffer(VertexBuffer.Type.Normal, MyVector3f.numAxes, normBuffer);
        for (int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex) {
            normBuffer.put(0f).put(1f).put(0f);
        }

        updateBound();
        setStatic();
    }
}
