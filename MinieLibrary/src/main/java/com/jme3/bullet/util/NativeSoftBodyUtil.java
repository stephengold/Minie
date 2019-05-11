/*
 * Copyright (c) 2009-2016 jMonkeyEngine
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

import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.mesh.IndexBuffer;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.HashSet;
import java.util.Set;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A utility class for interfacing with Native Bullet, specifically for soft
 * bodies.
 *
 * @author dokthar
 */
public class NativeSoftBodyUtil {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(NativeSoftBodyUtil.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private NativeSoftBodyUtil() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Append the edges in the specified line-mode Mesh to the specified soft
     * body.
     *
     * @param mesh the input mesh (not null, Mode.Lines)
     * @param softBody the soft body to which links will be added (not null,
     * modified)
     */
    public static void appendFromLineMesh(Mesh mesh, PhysicsSoftBody softBody) {
        Mesh.Mode mode = mesh.getMode();
        assert mode == Mesh.Mode.Lines : mode;
        Validate.nonNull(softBody, "soft body");

        FloatBuffer positions = mesh.getFloatBuffer(VertexBuffer.Type.Position);
        IndexBuffer indices = mesh.getIndexBuffer();
        softBody.appendMeshData(positions, indices, null, null);
    }

    /**
     * Add the triangles and unique edges in the specified triangle-mode Mesh to
     * the specified soft body. TODO rename appendFromTriMesh
     *
     * @param mesh the input mesh (not null, Mode.Triangles)
     * @param softBody the soft body to which faces and links will be added (not
     * null, modified)
     */
    public static void createFromTriMesh(Mesh mesh, PhysicsSoftBody softBody) {
        Mesh.Mode mode = mesh.getMode();
        assert mode == Mesh.Mode.Triangles : mode;
        Validate.nonNull(softBody, "soft body");

        IndexBuffer triangleIndices = mesh.getIndexBuffer();
        int size = triangleIndices.size();
        Set<MeshEdge> uniqueEdges = new HashSet<>(3 * size);

        for (int intOffset = 0; intOffset < size; intOffset += 3) {
            int ti0 = triangleIndices.get(intOffset);
            int ti1 = triangleIndices.get(intOffset + 1);
            int ti2 = triangleIndices.get(intOffset + 2);

            uniqueEdges.add(new MeshEdge(ti0, ti1));
            uniqueEdges.add(new MeshEdge(ti1, ti2));
            uniqueEdges.add(new MeshEdge(ti0, ti2));
        }

        FloatBuffer positions = mesh.getFloatBuffer(VertexBuffer.Type.Position);
        int vertexCount = positions.capacity();
        int numUniqueEdges = uniqueEdges.size();
        int indexCount = 2 * numUniqueEdges;
        IndexBuffer links
                = IndexBuffer.createIndexBuffer(vertexCount, indexCount);
        int edgeIndex = 0;
        for (MeshEdge edge : uniqueEdges) {
            links.put(edgeIndex, edge.index1());
            links.put(edgeIndex + 1, edge.index2());
            edgeIndex += 2;
        }

        softBody.appendMeshData(positions, links, triangleIndices, null);
    }

    /**
     * Append tetrahedra to the specified soft body, one per face, connecting
     * its faces with the center of its axis-aligned bounding box.
     *
     * @param softBody the soft body to append to (not null, modified)
     */
    public static void createTetras(PhysicsSoftBody softBody) {
        Validate.nonNull(softBody, "soft body");
        /*
         * Append a new node, located at the center of the AABB.
         */
        int centerIndex = softBody.countNodes();
        Vector3f centerLocation = softBody.getPhysicsLocation(null);
        FloatBuffer buffer = BufferUtils.createFloatBuffer(centerLocation);
        softBody.appendNodes(buffer); // TODO set mass of node
        /*
         * Append tetrahedra, one per face.
         */
        int numNodes = softBody.countNodes();
        assert numNodes == centerIndex + 1;
        int numFaces = softBody.countFaces();
        IndexBuffer newTetras
                = IndexBuffer.createIndexBuffer(numNodes, 4 * numFaces);
        IntBuffer faceIndices = softBody.copyFaces(null);
        for (int faceIndex = 0; faceIndex < numFaces; faceIndex += 3) {
            int fi0 = faceIndices.get(3 * faceIndex);
            int fi1 = faceIndices.get(3 * faceIndex + 1);
            int fi2 = faceIndices.get(3 * faceIndex + 2);
            newTetras.put(4 * faceIndex, fi0);
            newTetras.put(4 * faceIndex + 1, fi1);
            newTetras.put(4 * faceIndex + 2, fi2);
            newTetras.put(4 * faceIndex + 3, centerIndex);
        }
        softBody.appendTetras(newTetras);
    }

    /**
     * Update the vertex/position/normal buffers of a Mesh from the specified
     * soft body. Mesh-vertex indexes may be mapped to Bullet node indexes.
     *
     * @param body the soft body to provide locations and normals (not null,
     * unaffected)
     * @param jmeToBulletMap the index map, or null for a 1:1 map
     * @param store the Mesh to update (not null, modified)
     * @param meshInLocalSpace if true, transform the node locations into the
     * body's local coordinates (relative to its bounding-box center), otherwise
     * use physics-space coordinates
     * @param updateNormals if true, update the normal buffer, otherwise skip
     * the normal buffer
     */
    public static void updateMesh(PhysicsSoftBody body,
            IntBuffer jmeToBulletMap, Mesh store, boolean meshInLocalSpace,
            boolean updateNormals) {
        long bodyId = body.getObjectId();
        FloatBuffer positionBuffer
                = store.getFloatBuffer(VertexBuffer.Type.Position);
        assert positionBuffer != null;
        FloatBuffer normalBuffer = null;
        if (updateNormals) {
            normalBuffer = store.getFloatBuffer(VertexBuffer.Type.Normal);
            assert normalBuffer != null;
        }

        if (jmeToBulletMap != null) {
            updateMesh(bodyId, jmeToBulletMap, positionBuffer,
                    normalBuffer, meshInLocalSpace, updateNormals);
        } else {
            updateMesh(bodyId, positionBuffer, normalBuffer,
                    meshInLocalSpace, updateNormals); // TODO index map
        }

        store.getBuffer(VertexBuffer.Type.Position).setUpdateNeeded();
        if (updateNormals) {
            store.getBuffer(VertexBuffer.Type.Normal).setUpdateNeeded();
        }
    }
    // *************************************************************************
    // private methods

    private static native void updateMesh(long softBodyId,
            IntBuffer inIndexMapping, FloatBuffer outPositionBuffer,
            FloatBuffer outNormalBuffer, boolean meshInLocalSpace,
            boolean updateNormals);

    private static native void updateMesh(long softBodyId,
            FloatBuffer outPositionBuffer, FloatBuffer outNormalBuffer,
            boolean meshInLocalSpace, boolean updateNormals);
}
