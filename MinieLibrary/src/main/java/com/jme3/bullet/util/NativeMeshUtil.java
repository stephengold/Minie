/*
 * Copyright (c) 2009-2018 jMonkeyEngine
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

import java.nio.ByteBuffer;
import java.util.logging.Logger;

/**
 * A utility class for interfacing with Native Bullet.
 *
 * @author normenhansen
 */
public class NativeMeshUtil {

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(NativeMeshUtil.class.getName());

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private NativeMeshUtil() {
    }

    /**
     * Instantiate a btTriangleIndexVertexArray. Native method.
     *
     * @param triangleIndexBase index buffer (not null)
     * @param vertexBase vertex buffer (not null)
     * @param numTriangles the number of triangles in the mesh (&ge;0)
     * @param numVertices the number of vertices in the mesh (&ge;0)
     * @param vertexStride (in bytes, &gt;0)
     * @param triangleIndexStride (in bytes, &gt;0)
     * @return the unique identifier of the resulting btTriangleIndexVertexArray
     * (not 0)
     */
    public static native long createTriangleIndexVertexArray(
            ByteBuffer triangleIndexBase, ByteBuffer vertexBase,
            int numTriangles, int numVertices, int vertexStride,
            int triangleIndexStride);
}
