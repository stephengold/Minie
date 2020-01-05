/*
 Copyright (c) 2019-2020, Stephen Gold
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
 * A 3-D, static, Triangles-mode Mesh (without indices or texture coordinates)
 * that renders one slice of a 3-D star. TODO tweak for innerY=0
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class StarSlice extends Mesh {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(StarSlice.class.getName());
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public StarSlice() {
    }

    /**
     * Instantiate a Mesh for rotation around its Y axis.
     *
     * The point of the slice lies on its +X axis. All triangles face outward.
     *
     * @param sliceAngle the desired angle between adjacent slices (in radians,
     * &gt;0, &le;PI)
     * @param innerRadius the desired distance of the inner edges from the Y
     * axis (in mesh units, &gt;0, &le;outerRadius)
     * @param outerRadius the desired distance of the outermost point from the Y
     * axis (in mesh units, &ge;innerRadius)
     * @param yThickness the desired total thickness at the Y axis (in mesh
     * units, &gt;0)
     * @param generateNormals true &rarr; generate normals, false &rarr; no
     * normals
     */
    public StarSlice(float sliceAngle, float innerRadius, float outerRadius,
            float yThickness, boolean generateNormals) {
        Validate.inRange(sliceAngle, "slice angle", 0, FastMath.PI);
        Validate.positive(innerRadius, "inner radius");
        Validate.inRange(outerRadius, "outer radius", innerRadius,
                Float.MAX_VALUE);
        Validate.positive(yThickness, "thickness");

        float centerY = yThickness / 2f;
        float theta = sliceAngle / 2f;
        float x = innerRadius * FastMath.cos(theta);
        float z = innerRadius * FastMath.sin(theta);
        float innerY = (1f - x / outerRadius) * centerY;

        FloatBuffer positionBuffer = BufferUtils.createFloatBuffer(
                outerRadius, 0f, 0f, // A
                x, +innerY, -z, // Y
                x, +innerY, z, // X

                outerRadius, 0f, 0f, // A
                x, -innerY, -z, // Z
                x, +innerY, -z, // Y

                outerRadius, 0f, 0f, // A
                x, -innerY, z, // W
                x, -innerY, -z, // Z

                outerRadius, 0f, 0f, // A
                x, +innerY, z, // X
                x, -innerY, z, // W

                0f, +centerY, 0f, // B
                x, +innerY, z, // X
                x, +innerY, -z, // Y

                0f, -centerY, 0f, // C
                x, -innerY, -z, // Z
                x, -innerY, z // W
        );
        setBuffer(VertexBuffer.Type.Position, MyVector3f.numAxes,
                positionBuffer);
        int numFloats = positionBuffer.capacity();
        positionBuffer.limit(numFloats);

        if (generateNormals) {
            MyMesh.generateNormals(this);
        }

        updateBound();
        setStatic();
    }
}
