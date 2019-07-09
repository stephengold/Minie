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

import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * A 3-D, static, line-mode mesh that renders an evenly subdivided line segment
 * between 2 endpoints.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class DividedLine extends Mesh {
    // *************************************************************************
    // constants and loggers

    /**
     * number of axes in a vector
     */
    final private static int numAxes = 3;
    /**
     * number of vertices per edge
     */
    final private static int vpe = 2;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(DividedLine.class.getName());
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public DividedLine() {
    }

    /**
     * Instantiate a subdivided line segment between the specified endpoints.
     *
     * @param endPoint1 the location of the first endpoint (in mesh coordinates,
     * not null, unaffected)
     * @param endPoint2 the location of the 2nd endpoint (in mesh coordinates,
     * not null, unaffected)
     * @param numSegments the number of sub-segments (&ge;1)
     */
    public DividedLine(Vector3f endPoint1, Vector3f endPoint2, int numSegments) {
        Validate.positive(numSegments, "number of segments");

        setMode(Mode.Lines);

        int numVertices = numSegments + 1;
        FloatBuffer posBuffer
                = BufferUtils.createFloatBuffer(numAxes * numVertices);
        setBuffer(VertexBuffer.Type.Position, numAxes, posBuffer);
        /*
         * Write the locations of all vertices:
         */
        Vector3f temp = new Vector3f();
        for (int vIndex = 0; vIndex < numVertices; ++vIndex) {
            float t = vIndex / (float) numSegments;
            MyVector3f.lerp(t, endPoint1, endPoint2, temp);
            posBuffer.put(temp.x).put(temp.y).put(temp.z);
        }
        assert posBuffer.position() == numAxes * numVertices;
        posBuffer.flip();

        IntBuffer indexBuffer = BufferUtils.createIntBuffer(vpe * numSegments);
        setBuffer(VertexBuffer.Type.Index, vpe, indexBuffer);
        /*
         * Write the vertex indices of all edges:
         */
        for (int sIndex = 0; sIndex < numSegments; ++sIndex) {
            indexBuffer.put(sIndex).put(sIndex + 1);
        }
        assert indexBuffer.position() == vpe * numSegments;
        indexBuffer.flip();

        updateBound();
        setStatic();
    }
}
