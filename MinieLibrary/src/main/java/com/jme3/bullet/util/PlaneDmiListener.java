/*
 * Copyright (c) 2020 jMonkeyEngine
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of 'jMonkeyEngine' nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package com.jme3.bullet.util;

import com.jme3.bullet.debug.DebugMeshInitListener;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A DebugMeshInitListener to add texture coordinates to (the debug mesh of) a
 * PlaneCollisionShape object.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class PlaneDmiListener implements DebugMeshInitListener {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(PlaneDmiListener.class.getName());
    // *************************************************************************
    // fields

    /**
     * max value for texture coordinates (&gt;0)
     */
    final private float textureScale;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a listener for the specified texture-square size.
     *
     * @param sideLength the desired side length for each texture square (in
     * physics-space units, &gt;0)
     */
    public PlaneDmiListener(float sideLength) {
        Validate.positive(sideLength, "side length");

        float meshSideLength = DebugShapeFactory.meshSideLength();
        textureScale = meshSideLength / sideLength;
    }
    // *************************************************************************
    // DebugMeshInitListener methods

    /**
     * Callback from DebugShapeFactory, invoked just after the mesh positions
     * and normals are initialized.
     *
     * @param debugMesh the mesh to be used for visualization (not null)
     */
    @Override
    public void debugMeshInit(Mesh debugMesh) {
        int numVertices = 8; // 4 vertices for each size
        int numFloats = 2 * numVertices;
        FloatBuffer uvs = BufferUtils.createFloatBuffer(numFloats);
        debugMesh.setBuffer(VertexBuffer.Type.TexCoord, 2, uvs);

        for (int sideIndex = 0; sideIndex < 2; ++sideIndex) {
            uvs.put(new float[]{
                textureScale, textureScale,
                textureScale, 0f,
                0f, 0f,
                0f, textureScale});
        }
        assert uvs.position() == numFloats;

        uvs.flip();
    }
}
