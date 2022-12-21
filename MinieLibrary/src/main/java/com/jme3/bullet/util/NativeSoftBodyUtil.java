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

import com.jme3.bullet.collision.shapes.infos.IndexedMesh;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.mesh.IndexBuffer;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.IntPair;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyVector3f;

/**
 * A utility class for interfacing with Native Bullet, specifically for soft
 * bodies.
 *
 * @author dokthar
 */
final public class NativeSoftBodyUtil {
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
     * number of vertices per triangle
     */
    final private static int vpt = 3;
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
     * @param mesh the input JME mesh (not null, unaffected, mode=Lines,
     * position and index buffers must be direct)
     * @param softBody the soft body to which links will be added (not null,
     * modified)
     */
    public static void appendFromLineMesh(Mesh mesh, PhysicsSoftBody softBody) {
        Mesh.Mode mode = mesh.getMode();
        assert mode == Mesh.Mode.Lines : mode;
        Validate.nonNull(softBody, "soft body");

        FloatBuffer positions = mesh.getFloatBuffer(VertexBuffer.Type.Position);
        assert positions.isDirect();
        softBody.appendNodes(positions);

        IndexBuffer lineIndices = mesh.getIndexBuffer();
        assert lineIndices.getBuffer().isDirect();
        softBody.appendLinks(lineIndices);
    }

    /**
     * Add the triangles and unique edges in the specified native mesh to the
     * specified soft body.
     *
     * @param mesh the input native mesh (not null)
     * @param softBody the soft body to which faces and links will be added (not
     * null, modified)
     */
    public static void appendFromNativeMesh(
            IndexedMesh mesh, PhysicsSoftBody softBody) {
        Validate.nonNull(softBody, "soft body");

        FloatBuffer positions = mesh.copyVertexPositions();
        assert positions.isDirect();
        softBody.appendNodes(positions);

        IntBuffer triangleIndices = mesh.copyIndices();
        assert triangleIndices.isDirect();
        IndexBuffer indexBuffer = IndexBuffer.wrapIndexBuffer(triangleIndices);
        softBody.appendFaces(indexBuffer);

        // Enumerate all unique edges among the triangles.
        int size = triangleIndices.capacity();
        Collection<IntPair> uniqueEdges = new HashSet<>(vpt * size);
        for (int intOffset = 0; intOffset < size; intOffset += vpt) {
            int ti0 = triangleIndices.get(intOffset);
            int ti1 = triangleIndices.get(intOffset + 1);
            int ti2 = triangleIndices.get(intOffset + 2);

            uniqueEdges.add(new IntPair(ti0, ti1));
            uniqueEdges.add(new IntPair(ti1, ti2));
            uniqueEdges.add(new IntPair(ti0, ti2));
        }

        int numUniqueEdges = uniqueEdges.size();
        int indexCount = vpe * numUniqueEdges;
        IntBuffer links = BufferUtils.createIntBuffer(indexCount);
        int edgeIndex = 0;
        for (IntPair edge : uniqueEdges) {
            links.put(edgeIndex, edge.smaller());
            links.put(edgeIndex + 1, edge.larger());
            edgeIndex += vpe;
        }
        indexBuffer = IndexBuffer.wrapIndexBuffer(links);
        softBody.appendLinks(indexBuffer);
    }

    /**
     * Add the triangles and unique edges in the specified JME mesh to the
     * specified soft body.
     *
     * @param mesh the input JME mesh (not null, unaffected, mode=Triangles,
     * position and index buffers must be direct)
     * @param softBody the soft body to which faces and links will be added (not
     * null, modified)
     */
    public static void appendFromTriMesh(Mesh mesh, PhysicsSoftBody softBody) {
        Mesh.Mode mode = mesh.getMode();
        assert mode == Mesh.Mode.Triangles : mode;
        Validate.nonNull(softBody, "soft body");

        FloatBuffer positions = mesh.getFloatBuffer(VertexBuffer.Type.Position);
        assert positions.isDirect();
        softBody.appendNodes(positions);

        IndexBuffer triangleIndices = mesh.getIndexBuffer();
        assert triangleIndices.getBuffer().isDirect();
        softBody.appendFaces(triangleIndices);

        // Enumerate all unique edges among the triangles.
        int size = triangleIndices.size();
        Collection<IntPair> uniqueEdges = new HashSet<>(vpt * size);
        for (int intOffset = 0; intOffset < size; intOffset += vpt) {
            int ti0 = triangleIndices.get(intOffset);
            int ti1 = triangleIndices.get(intOffset + 1);
            int ti2 = triangleIndices.get(intOffset + 2);

            uniqueEdges.add(new IntPair(ti0, ti1));
            uniqueEdges.add(new IntPair(ti1, ti2));
            uniqueEdges.add(new IntPair(ti0, ti2));
        }

        int vertexCount = positions.limit();
        int numUniqueEdges = uniqueEdges.size();
        int indexCount = vpe * numUniqueEdges;
        IndexBuffer links
                = IndexBuffer.createIndexBuffer(vertexCount, indexCount);
        int edgeIndex = 0;
        for (IntPair edge : uniqueEdges) {
            links.put(edgeIndex, edge.smaller());
            links.put(edgeIndex + 1, edge.larger());
            edgeIndex += vpe;
        }
        softBody.appendLinks(links);
    }

    /**
     * Append tetrahedra to the specified soft body, one per face, connecting
     * its faces with the center of its axis-aligned bounding box.
     *
     * @param softBody the soft body to append to (not null, modified)
     */
    public static void appendTetras(PhysicsSoftBody softBody) {
        // Append a new node, located at the center of the AABB.
        int centerIndex = softBody.countNodes();
        Vector3f centerLocation = softBody.getPhysicsLocation(null);
        FloatBuffer buffer = BufferUtils.createFloatBuffer(centerLocation);
        softBody.appendNodes(buffer); // TODO set mass of node

        // Append tetrahedra, one per face.
        int numNodes = softBody.countNodes();
        assert numNodes == centerIndex + 1;
        int numFaces = softBody.countFaces();
        IndexBuffer newTetras
                = IndexBuffer.createIndexBuffer(numNodes, 4 * numFaces);
        IntBuffer faceIndices = softBody.copyFaces(null);
        for (int faceIndex = 0; faceIndex < numFaces; ++faceIndex) {
            int fi0 = faceIndices.get(vpt * faceIndex);
            int fi1 = faceIndices.get(vpt * faceIndex + 1);
            int fi2 = faceIndices.get(vpt * faceIndex + 2);
            newTetras.put(4 * faceIndex, fi0);
            newTetras.put(4 * faceIndex + 1, fi1);
            newTetras.put(4 * faceIndex + 2, fi2);
            newTetras.put(4 * faceIndex + 3, centerIndex);
        }
        softBody.appendTetras(newTetras);
    }

    /**
     * Create an index map to merge any mesh vertices that share the same
     * position. Other vertex properties (such as bone weights, normals, and
     * texture coordinates) are ignored.
     *
     * @param positionBuffer the buffer of mesh-vertex positions (not null,
     * limit a multiple of 3, unaffected)
     * @return a new index map (not null)
     */
    public static IntBuffer generateIndexMap(FloatBuffer positionBuffer) {
        int numFloats = positionBuffer.limit();
        Validate.require(numFloats % numAxes == 0, "limit a multiple of 3");
        int numVertices = numFloats / numAxes;

        IntBuffer result = BufferUtils.createIntBuffer(numVertices);
        Map<Vector3f, Integer> tmpHashMap = new HashMap<>(numVertices);
        int nextMappedIndex = 0;

        for (int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex) {
            Vector3f position = new Vector3f();
            MyBuffer.get(positionBuffer, numAxes * vertexIndex, position);
            MyVector3f.standardize(position, position);

            if (!tmpHashMap.containsKey(position)) {
                tmpHashMap.put(position, nextMappedIndex);
                result.put(nextMappedIndex);
                ++nextMappedIndex;
            } else {
                int mappedIndex = tmpHashMap.get(position);
                result.put(mappedIndex);
            }
        }
        result.flip();

        return result;
    }

    /**
     * Map all indices in the specified input buffer using the specified map
     * buffer.
     *
     * @param indexMap the buffer to use to map input indices to result indices
     * (not null, unaffected)
     * @param inputBuffer the input buffer to map (not null, unaffected)
     * @param reuseBuffer the buffer to reuse for output, or null if none
     * (modified)
     * @return a buffer containing mapped indices (either reuseBuffer or a new
     * buffer, not null)
     */
    public static IndexBuffer mapIndices(IntBuffer indexMap,
            IndexBuffer inputBuffer, IndexBuffer reuseBuffer) {
        int numIndices = inputBuffer.size();
        IndexBuffer result;
        if (reuseBuffer == null) {
            int vertexCount = Integer.MAX_VALUE;
            result = IndexBuffer.createIndexBuffer(vertexCount, numIndices);
        } else {
            int reuseBufferSize = reuseBuffer.size();
            if (reuseBufferSize < numIndices) {
                logger.log(Level.SEVERE, "outputSize={0}", reuseBufferSize);
                String message = String.format("Reuse buffer size must be "
                        + "greater than or equal to %d.", numIndices);
                throw new IllegalArgumentException(message);
            }
            result = reuseBuffer;
        }

        for (int offset = 0; offset < numIndices; ++offset) {
            int oldIndex = inputBuffer.get(offset);
            int newIndex = indexMap.get(oldIndex);
            result.put(offset, newIndex);
        }
        result.getBuffer().limit(numIndices);

        return result;
    }

    /**
     * Copy all vertex data in the specified input buffer, using the specified
     * map buffer to map vertex indices.
     *
     * @param indexMap the buffer to use to map input indices to result indices
     * (not null, unaffected)
     * @param inputBuffer the input buffer to map (not null, length a multiple
     * of numFloatsPerVertex, unaffected)
     * @param numFloatsPerVertex the number of float components per vertex
     * (&gt;0)
     * @return a new buffer containing mapped vertex data
     */
    public static FloatBuffer mapVertexData(IntBuffer indexMap,
            FloatBuffer inputBuffer, int numFloatsPerVertex) {
        Validate.nonNull(indexMap, "index map");
        Validate.positive(numFloatsPerVertex, "number of floats per vertex");
        int numFloats = inputBuffer.limit();
        assert (numFloats % numFloatsPerVertex == 0) : numFloats;
        int numVertices = numFloats / numFloatsPerVertex;

        FloatBuffer result = BufferUtils.createFloatBuffer(numFloats);

        int lastNewVIndex = -1;
        for (int oldVIndex = 0; oldVIndex < numVertices; ++oldVIndex) {
            int newVIndex = indexMap.get(oldVIndex);
            for (int i = 0; i < numFloatsPerVertex; ++i) {
                int oldFloatIndex = numFloatsPerVertex * oldVIndex + i;
                float x = inputBuffer.get(oldFloatIndex);

                int newFloatIndex = numFloatsPerVertex * newVIndex + i;
                result.put(newFloatIndex, x);
            }
            if (newVIndex > lastNewVIndex) {
                lastNewVIndex = newVIndex;
            }
        }

        int newLimit = numFloatsPerVertex * (lastNewVIndex + 1);
        result.limit(newLimit);

        return result;
    }

    /**
     * Update the position buffer of a Mesh from the clusters in the specified
     * soft body.
     *
     * @param body the soft body to provide locations (not null, unaffected)
     * @param store the Mesh to update (not null, position buffer must be
     * direct, modified)
     * @param meshInLocalSpace if true, transform the cluster locations into the
     * body's local coordinates (relative to its bounding-box center), otherwise
     * use physics-space coordinates
     */
    public static void updateClusterMesh(
            PhysicsSoftBody body, Mesh store, boolean meshInLocalSpace) {
        long bodyId = body.nativeId();
        FloatBuffer positionBuffer
                = store.getFloatBuffer(VertexBuffer.Type.Position);
        assert positionBuffer != null;

        updateClusterMesh(bodyId, positionBuffer, meshInLocalSpace);
        store.getBuffer(VertexBuffer.Type.Position).setUpdateNeeded();
    }

    /**
     * Update the position/normal buffers of a Mesh from the nodes in the
     * specified soft body. Mesh-vertex indices may be mapped to body-node
     * indices, and physics-space locations may be transformed into mesh
     * positions.
     *
     * @param body the soft body to provide locations and normals (not null,
     * unaffected)
     * @param vertexToNodeMap the index map to apply (must be direct) or null
     * for identity
     * @param store the Mesh to update (not null, position, normal, and index
     * buffers must be direct, modified)
     * @param meshInLocalSpace if true, transform node locations into the body's
     * local coordinates (relative to its bounding-box center), otherwise use
     * physics-space coordinates
     * @param updateNormals if true, update the normal buffer, otherwise ignore
     * the normal buffer
     * @param physicsToMesh the coordinate transform to apply, or null for
     * identity (unaffected)
     */
    public static void updateMesh(PhysicsSoftBody body,
            IntBuffer vertexToNodeMap, Mesh store, boolean meshInLocalSpace,
            boolean updateNormals, Transform physicsToMesh) {
        long bodyId = body.nativeId();
        FloatBuffer positionBuffer
                = store.getFloatBuffer(VertexBuffer.Type.Position);
        assert positionBuffer != null;

        FloatBuffer normalBuffer = null;
        if (updateNormals) {
            normalBuffer = store.getFloatBuffer(VertexBuffer.Type.Normal);
            assert normalBuffer != null;
        }

        if (vertexToNodeMap != null) {
            // map mesh-vertex indices to body-node indices
            updateMesh(bodyId, vertexToNodeMap, positionBuffer, normalBuffer,
                    meshInLocalSpace, updateNormals);
        } else {
            // null map: mesh-vertex indices equal body-node indices
            updateMesh(bodyId, positionBuffer, normalBuffer,
                    meshInLocalSpace, updateNormals);
        }

        if (physicsToMesh != null) {
            Vector3f tempVector = new Vector3f();

            // Transform physics locations to mesh positions.
            positionBuffer.rewind();
            while (positionBuffer.hasRemaining()) {
                positionBuffer.mark();
                tempVector.x = positionBuffer.get();
                tempVector.y = positionBuffer.get();
                tempVector.z = positionBuffer.get();
                physicsToMesh.transformVector(tempVector, tempVector);

                positionBuffer.reset();
                positionBuffer.put(tempVector.x);
                positionBuffer.put(tempVector.y);
                positionBuffer.put(tempVector.z);
            }

            if (normalBuffer != null) { // Rotate the normals.
                normalBuffer.rewind();
                while (normalBuffer.hasRemaining()) {
                    normalBuffer.mark();
                    tempVector.x = normalBuffer.get();
                    tempVector.y = normalBuffer.get();
                    tempVector.z = normalBuffer.get();
                    physicsToMesh.getRotation().mult(tempVector, tempVector);

                    normalBuffer.reset();
                    normalBuffer.put(tempVector.x);
                    normalBuffer.put(tempVector.y);
                    normalBuffer.put(tempVector.z);
                }
            }
        }

        store.getBuffer(VertexBuffer.Type.Position).setUpdateNeeded();
        if (normalBuffer != null) {
            store.getBuffer(VertexBuffer.Type.Normal).setUpdateNeeded();
        }
    }

    /**
     * Update the position buffer of a Mesh from the pinned nodes in the
     * specified soft body.
     *
     * @param body the soft body to provide locations (not null, unaffected)
     * @param store the Mesh to update (not null, position buffer must be
     * direct, modified)
     * @param meshInLocalSpace if true, transform the pin locations into the
     * body's local coordinates (relative to its bounding-box center), otherwise
     * use physics-space coordinates
     */
    public static void updatePinMesh(
            PhysicsSoftBody body, Mesh store, boolean meshInLocalSpace) {
        long bodyId = body.nativeId();
        FloatBuffer positionBuffer
                = store.getFloatBuffer(VertexBuffer.Type.Position);
        assert positionBuffer != null;

        updatePinMesh(bodyId, positionBuffer, meshInLocalSpace);
        store.getBuffer(VertexBuffer.Type.Position).setUpdateNeeded();
    }
    // *************************************************************************
    // native private methods

    native private static void updateClusterMesh(long softBodyId,
            FloatBuffer outPositionBuffer, boolean meshInLocalSpace);

    native private static void updateMesh(
            long softBodyId, IntBuffer inIndexMapping,
            FloatBuffer outPositionBuffer, FloatBuffer outNormalBuffer,
            boolean meshInLocalSpace, boolean updateNormals);

    native private static void updateMesh(long softBodyId,
            FloatBuffer outPositionBuffer, FloatBuffer outNormalBuffer,
            boolean meshInLocalSpace, boolean updateNormals);

    native private static void updatePinMesh(long softBodyId,
            FloatBuffer outPositionBuffer, boolean meshInLocalSpace);
}
