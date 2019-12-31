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
import jme3utilities.math.MyVector3f;

/**
 * A 3-D, static, Triangles-mode Mesh (with normals but no indices or texture
 * coordinates) that renders one slice of a 3-D star.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class StarSlice extends Mesh {
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
            = Logger.getLogger(StarSlice.class.getName());
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected StarSlice() {
    }

    /**
     * Instantiate a Mesh for rotation around its Y axis.
     *
     * The point of the slice lies on the +X axis. All triangles face outward.
     *
     * @param sliceAngle the angle between adjacent slices (in radians, &gt;0,
     * &le;PI)
     * @param innerRadius the distance of the inner edges from the Y axis (in
     * mesh units, &gt;0, &le;outerRadius)
     * @param outerRadius the distance of the outermost point from the Y axis
     * (in mesh units, &ge;innerRadius)
     * @param thickness the thickness at the center (in mesh units, &gt;0)
     */
    public StarSlice(float sliceAngle, float innerRadius, float outerRadius,
            float thickness) {
        Validate.inRange(sliceAngle, "slice angle", 0, FastMath.PI);
        Validate.positive(innerRadius, "inner radius");
        Validate.inRange(outerRadius, "outer radius", innerRadius,
                Float.MAX_VALUE);
        Validate.positive(thickness, "thickness");

        float centerY = thickness / 2f;
        float theta = sliceAngle / 2f;
        float x = innerRadius * FastMath.cos(theta);
        float z = innerRadius * FastMath.sin(theta);
        float innerY = (1f - x / outerRadius) * centerY;

        FloatBuffer positionBuffer = BufferUtils.createFloatBuffer(
                outerRadius, 0f, 0f, // A
                x, innerY, -z, // Y
                x, innerY, z, // X

                outerRadius, 0f, 0f, // A
                x, -innerY, -z, // Z
                x, innerY, -z, // Y

                outerRadius, 0f, 0f, // A
                x, -innerY, z, // W
                x, -innerY, -z, // Z

                outerRadius, 0f, 0f, // A
                x, innerY, z, // X
                x, -innerY, z, // W

                0f, centerY, 0f, // B
                x, innerY, z, // X
                x, innerY, -z, // Y

                0f, -centerY, 0f, // C
                x, -innerY, -z, // Z
                x, -innerY, z // W
        );
        setBuffer(VertexBuffer.Type.Position, numAxes, positionBuffer);
        int numFloats = positionBuffer.capacity();
        positionBuffer.limit(numFloats);

        generateNormals(this);

        updateBound();
        setStatic();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Generate normals on a per-triangle basis for a Triangles-mode mesh
     * without an index buffer. TODO use MyMesh
     *
     * @param mesh (not null)
     */
    public static void generateNormals(Mesh mesh) {
        assert mesh.getMode() == Mesh.Mode.Triangles;
        assert mesh.getBuffer(VertexBuffer.Type.Index) == null;

        FloatBuffer positionBuffer
                = mesh.getFloatBuffer(VertexBuffer.Type.Position);
        int numFloats = positionBuffer.limit();

        FloatBuffer normalBuffer = BufferUtils.createFloatBuffer(numFloats);
        mesh.setBuffer(VertexBuffer.Type.Normal, numAxes, normalBuffer);

        Triangle triangle = new Triangle();
        Vector3f pos1 = new Vector3f();
        Vector3f pos2 = new Vector3f();
        Vector3f pos3 = new Vector3f();

        int numTriangles = numFloats / vpt / numAxes;
        for (int triIndex = 0; triIndex < numTriangles; ++triIndex) {
            int trianglePosition = triIndex * vpt * numAxes;
            get(positionBuffer, trianglePosition, pos1);
            get(positionBuffer, trianglePosition + numAxes, pos2);
            get(positionBuffer, trianglePosition + 2 * numAxes, pos3);
            triangle.set(pos1, pos2, pos3);

            triangle.setNormal(null); // work around JME issue #957
            Vector3f normal = triangle.getNormal();
            for (int j = 0; j < vpt; ++j) {
                normalBuffer.put(normal.x);
                normalBuffer.put(normal.y);
                normalBuffer.put(normal.z);
            }
        }
        normalBuffer.flip();
    }

    /**
     * Read a Vector3f starting from the given position. Does not alter the
     * buffer's position. TODO use MyBuffer
     *
     * @param buffer the buffer to read from (not null, unaffected)
     * @param startPosition the position at which to start reading (&ge;0)
     * @param storeVector storage for the vector (not null, modified)
     */
    public static void get(FloatBuffer buffer, int startPosition,
            Vector3f storeVector) {
        Validate.nonNull(buffer, "buffer");
        Validate.nonNegative(startPosition, "start position");
        Validate.nonNull(storeVector, "store vector");

        storeVector.x = buffer.get(startPosition + MyVector3f.xAxis);
        storeVector.y = buffer.get(startPosition + MyVector3f.yAxis);
        storeVector.z = buffer.get(startPosition + MyVector3f.zAxis);
    }
}
