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
package com.jme3.bullet.objects;

import com.jme3.bounding.BoundingBox;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.SoftBodyWorldInfo;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.infos.DebugMeshNormals;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.objects.infos.SoftBodyConfig;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.mesh.IndexBuffer;
import com.jme3.util.BufferUtils;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.nio.Buffer;
import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.nio.ShortBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A collision object to simulate a soft body, based on Bullet's btSoftBody. It
 * is modeled as a mesh of nodes. Unlike other collision objects, the
 * CollisionShape is ignored and is typically null.
 *
 * @author dokthar
 */
public class PhysicsSoftBody extends PhysicsBody {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(PhysicsSoftBody.class.getName());
    // *************************************************************************
    // fields

    /**
     * list of joints that connect to this body: The list isn't populated until
     * the body is added to a PhysicsSpace.
     */
    private List<PhysicsJoint> joints = new ArrayList<>(4);
    /**
     * material properties of this soft body, allocated lazily
     */
    private Material material = null;
    /**
     * configuration properties of this soft body
     */
    private SoftBodyConfig config = new SoftBodyConfig(this);
    // *************************************************************************
    // constructors

    /**
     * Instantiate an empty soft body. The new body is not added to any physics
     * space.
     */
    public PhysicsSoftBody() {
        objectId = createEmptySoftBody();
        assert objectId != 0L;

        config = new SoftBodyConfig(this);
        super.initUserPointer();
        float defaultMargin = CollisionShape.getDefaultMargin();
        setMargin(defaultMargin);

        assert !isInWorld();
        assert countAnchors() == 0;
        assert countClusters() == 0;
        assert countNodes() == 0;
        assert countLinks() == 0;
        assert countFaces() == 0;
        assert countTetras() == 0;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Add velocity to this entire body.
     *
     * @param velocity the velocity to add (in physics-space coordinates, not
     * null, unaffected)
     */
    public void addVelocity(Vector3f velocity) {
        Validate.finite(velocity, "velocity");
        addVelocity(objectId, velocity);
    }

    /**
     * Add velocity to the indexed node of this body.
     *
     * @param velocity the velocity to add (in physics-space coordinates, not
     * null, unaffected)
     * @param nodeIndex which node to add it to (&ge;0, &lt;numNodes)
     */
    public void addVelocity(Vector3f velocity, int nodeIndex) {
        Validate.finite(velocity, "velocity");
        int numNodes = countNodes();
        Validate.inRange(nodeIndex, "node index", 0, numNodes - 1);

        addVelocity(objectId, velocity, nodeIndex);
    }

    /**
     * Read the influence of the indexed anchor.
     *
     * @param anchorIndex which anchor (&ge;0, &lt;numAnchors)
     * @return the amount of influence on this body (0&rarr;no influence,
     * 1&rarr;strong influence)
     */
    public float anchorInfluence(int anchorIndex) {
        int numAnchors = countAnchors();
        Validate.inRange(anchorIndex, "anchor index", 0, numAnchors - 1);

        float result = getAnchorInfluence(objectId, anchorIndex);
        return result;
    }

    /**
     * Read the index of the node connected by the indexed anchor.
     *
     * @param anchorIndex which anchor (&ge;0, &lt;numAnchors)
     * @return the index of the node (&ge;0, &lt;numNodes)
     */
    public int anchorNodeIndex(int anchorIndex) {
        int numAnchors = countAnchors();
        Validate.inRange(anchorIndex, "anchor index", 0, numAnchors - 1);

        int result = getAnchorNodeIndex(objectId, anchorIndex);

        assert result >= 0 : result;
        assert result < countNodes() : result;
        return result;
    }

    /**
     * Copy the pivot offset of the indexed anchor.
     *
     * @param anchorIndex which anchor (&ge;0, &lt;numAnchors)
     * @param storeResult storage for the result (modified if null)
     * @return an offset vector (either storeResult or a new vector)
     */
    public Vector3f anchorPivot(int anchorIndex, Vector3f storeResult) {
        int numAnchors = countAnchors();
        Validate.inRange(anchorIndex, "anchor index", 0, numAnchors - 1);
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        getAnchorPivot(objectId, anchorIndex, result);
        return result;
    }

    /**
     * Read the native ID of the rigid body connected by the indexed anchor.
     *
     * @param anchorIndex which anchor (&ge;0, &lt;numAnchors)
     * @return an offset vector (either storeResult or a new vector)
     */
    public long anchorRigidId(int anchorIndex) {
        int numAnchors = countAnchors();
        Validate.inRange(anchorIndex, "anchor index", 0, numAnchors - 1);

        long result = getAnchorRigidId(objectId, anchorIndex);
        return result;
    }

    /**
     * Add an anchor connecting the indexed node of this body to the specified
     * rigid body.
     *
     * @param nodeIndex which node of this body to connect (&ge;0, &lt;numNodes)
     * @param rigidBody the rigid body to connect (not null)
     * @param localPivot the anchor location in the soft body's local
     * coordinates (not null, unaffected)
     * @param collisionBetweenLinkedBodies true&rarr;allow collisions between
     * this body and the rigid body, false&rarr;don't allow such collisions
     * @param influence how much influence the anchor has on this body
     * (0&rarr;no influence, 1&rarr;strong influence).
     */
    public void appendAnchor(int nodeIndex, PhysicsRigidBody rigidBody,
            Vector3f localPivot, boolean collisionBetweenLinkedBodies,
            float influence) {
        int numNodes = countNodes();
        Validate.inRange(nodeIndex, "node index", 0, numNodes - 1);
        Validate.finite(localPivot, "local pivot");

        long rigidBodyId = rigidBody.getObjectId();
        appendAnchor(objectId, nodeIndex, rigidBodyId, localPivot,
                collisionBetweenLinkedBodies, influence);
    }

    /**
     * Append faces to this body. A Face is a triangle connecting 3 nodes.
     *
     * @param nodeIndices a face is created for every 3 indices in this buffer
     * (not null, size a multiple of 3)
     */
    public void appendFaces(IndexBuffer nodeIndices) {
        if (nodeIndices.size() % 3 != 0) {
            throw new IllegalArgumentException(
                    "The number of indices must be a multiple of 3.");
        }

        Buffer buffer = nodeIndices.getBuffer();
        if (buffer instanceof ByteBuffer) {
            appendFaces(objectId, (ByteBuffer) buffer);
        } else if (buffer instanceof ShortBuffer) {
            appendFaces(objectId, (ShortBuffer) buffer);
        } else if (buffer instanceof IntBuffer) {
            appendFaces(objectId, (IntBuffer) buffer);
        } else {
            throw new IllegalArgumentException(
                    buffer.getClass().getSimpleName());
        }
    }

    /**
     * Append links to this body. A link is a interaction (or force) between 2
     * nodes.
     *
     * @param nodeIndices a link is created for each pair of indices in this
     * buffer (not null, size a multiple of 2)
     */
    public void appendLinks(IndexBuffer nodeIndices) {
        if (nodeIndices.size() % 2 != 0) {
            throw new IllegalArgumentException(
                    "The number of indices must be a multiple of 2.");
        }

        Buffer buffer = nodeIndices.getBuffer();
        if (buffer instanceof ByteBuffer) {
            appendLinks(objectId, (ByteBuffer) buffer);
        } else if (buffer instanceof ShortBuffer) {
            appendLinks(objectId, (ShortBuffer) buffer);
        } else if (buffer instanceof IntBuffer) {
            appendLinks(objectId, (IntBuffer) buffer);
        } else {
            throw new IllegalArgumentException(
                    buffer.getClass().getSimpleName());
        }
    }

    /**
     * Append nodes to this body. Nodes provide the shape of a soft body. Each
     * node has its own location, velocity, mass, etcetera.
     *
     * @param nodeLocations a node is created for every 3 floats in this buffer
     * (not null, capacity=limit, limit a multiple of 3)
     */
    public void appendNodes(FloatBuffer nodeLocations) {
        assert nodeLocations.capacity() == nodeLocations.limit();
        if (nodeLocations.capacity() % 3 != 0) {
            throw new IllegalArgumentException(
                    "The number of floats must be a multiple of 3.");
        }

        appendNodes(objectId, nodeLocations);
    }

    /**
     * Append tetrahedra to this body. A tetrahedron defines a volume between 4
     * nodes.
     *
     * @param tetrahedra a tetrahedron is created for every 4 indices in this
     * buffer (not null, size a multiple of 4)
     */
    public void appendTetras(IndexBuffer tetrahedra) {
        if (tetrahedra.size() % 4 != 0) {
            throw new IllegalArgumentException(
                    "The number of indices must be a multiple of 4.");
        }

        Buffer buffer = tetrahedra.getBuffer();
        if (buffer instanceof ByteBuffer) {
            appendTetras(objectId, (ByteBuffer) buffer);
        } else if (buffer instanceof ShortBuffer) {
            appendTetras(objectId, (ShortBuffer) buffer);
        } else if (buffer instanceof IntBuffer) {
            appendTetras(objectId, (IntBuffer) buffer);
        } else {
            throw new IllegalArgumentException(
                    buffer.getClass().getSimpleName());
        }
    }

    /**
     * Apply a force that acts uniformly across the entire body.
     *
     * @param force the force to add (in physics-space coordinates, not null,
     * unaffected)
     */
    public void applyForce(Vector3f force) {
        Validate.finite(force, "force");
        addForce(objectId, force);
    }

    /**
     * Apply a force to the indexed node of this body.
     *
     * @param force the force to add (in physics-space coordinates, not null,
     * unaffected)
     * @param nodeIndex which node to add it to (&ge;0, &lt;numNodes)
     */
    public void applyForce(Vector3f force, int nodeIndex) {
        Validate.finite(force, "force");
        int numNodes = countNodes();
        Validate.inRange(nodeIndex, "node index", 0, numNodes - 1);

        addForce(objectId, force, nodeIndex);
    }

    /**
     * Rotate this body.
     *
     * @param rotation the rotation to apply (not null, unaffected)
     */
    public void applyRotation(Quaternion rotation) {
        Validate.nonNull(rotation, "rotation");
        applyPhysicsRotation(objectId, rotation);
    }

    /**
     * Scale this body. Use caution!
     *
     * @param factors the scaling factor to apply to each axis (not null,
     * unaffected)
     */
    public void applyScale(Vector3f factors) {
        Validate.finite(factors, "factors");
        applyPhysicsScale(objectId, factors);
    }

    /**
     * Transform this body. Apply non-identity scaling with caution!
     *
     * @param transform the transform to apply (not null, unaffected)
     */
    public void applyTransform(Transform transform) {
        Validate.nonNull(transform, "transform");
        applyPhysicsTransform(objectId, transform);
    }

    /**
     * Translate this body.
     *
     * @param offset the translation to apply (not null, unaffected)
     */
    public void applyTranslation(Vector3f offset) {
        Validate.finite(offset, "offset");
        applyPhysicsTranslate(objectId, offset);
    }

    /**
     * Copy the center location of the indexed cluster.
     *
     * @param clusterIndex which cluster (&ge;0, &lt;numClusters)
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in physics-space coordinates, either
     * storeResult or a new vector)
     */
    public Vector3f clusterCenter(int clusterIndex, Vector3f storeResult) {
        int numClusters = countClusters();
        Validate.inRange(clusterIndex, "cluster index", 0, numClusters - 1);
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        getClusterCenter(objectId, clusterIndex, result);
        return result;
    }

    /**
     * Copy the center-of-mass locations of all clusters in this body.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a buffer containing 3 floats per cluster (in physics-space
     * coordinates, either storeResult or a new buffer)
     */
    public FloatBuffer copyClusterCenters(FloatBuffer storeResult) {
        int numClusters = countClusters();
        FloatBuffer result;
        if (storeResult == null) {
            result = BufferUtils.createFloatBuffer(3 * numClusters);
        } else {
            assert storeResult.capacity() == 3 * numClusters;
            result = storeResult;
        }

        getClustersPositions(objectId, result);
        return result;
    }

    /**
     * Copy the masses of all clusters in this body.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a buffer containing a float value per cluster (either storeResult
     * or a new buffer)
     */
    public FloatBuffer copyClusterMasses(FloatBuffer storeResult) {
        int numClusters = countClusters();
        FloatBuffer result;
        if (storeResult == null) {
            result = BufferUtils.createFloatBuffer(numClusters);
        } else {
            assert storeResult.capacity() == numClusters;
            result = storeResult;
        }

        getClustersMasses(objectId, result);
        return result;
    }

    /**
     * Copy the node indices of all faces in this body.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a buffer containing 3 indices per face (either storeResult or a
     * new buffer)
     */
    public IntBuffer copyFaces(IntBuffer storeResult) {
        int numFaces = countFaces();
        IntBuffer result;
        if (storeResult == null) {
            result = BufferUtils.createIntBuffer(3 * numFaces);
        } else {
            assert storeResult.capacity() == 3 * numFaces;
            result = storeResult;
        }

        getFacesIndexes(objectId, result);
        return result;
    }

    /**
     * Copy the node indices of all links in this body.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a buffer containing 2 indices per link (either storeResult or a
     * new buffer)
     */
    public IntBuffer copyLinks(IntBuffer storeResult) {
        int numLinks = countLinks();
        IntBuffer result;
        if (storeResult == null) {
            result = BufferUtils.createIntBuffer(2 * numLinks);
        } else {
            assert storeResult.capacity() == 2 * numLinks;
            result = storeResult;
        }

        getLinksIndexes(objectId, result);
        return result;
    }

    /**
     * Copy the locations of all nodes in this body.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a buffer containing 3 floats per node (in physics-space
     * coordinates, either storeResult or a new buffer)
     */
    public FloatBuffer copyLocations(FloatBuffer storeResult) {
        int numNodes = countNodes();
        FloatBuffer result;
        if (storeResult == null) {
            result = BufferUtils.createFloatBuffer(3 * numNodes);
        } else {
            assert storeResult.capacity() == 3 * numNodes;
            result = storeResult;
        }

        getNodesPositions(objectId, result);
        return result;
    }

    /**
     * Copy the masses of all nodes in this body.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a buffer containing a float value per node (either storeResult or
     * a new buffer)
     */
    public FloatBuffer copyMasses(FloatBuffer storeResult) {
        int numNodes = countNodes();
        FloatBuffer result;
        if (storeResult == null) {
            result = BufferUtils.createFloatBuffer(numNodes);
        } else {
            assert storeResult.capacity() == numNodes;
            result = storeResult;
        }

        getMasses(objectId, result);
        return result;
    }

    /**
     * Copy the normal vectors of all nodes in this body.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a buffer containing 3 floats per node (in physics-space
     * coordinates, either storeResult or a new buffer)
     */
    public FloatBuffer copyNormals(FloatBuffer storeResult) {
        int numNodes = countNodes();
        FloatBuffer result;
        if (storeResult == null) {
            result = BufferUtils.createFloatBuffer(3 * numNodes);
        } else {
            assert storeResult.capacity() == 3 * numNodes;
            result = storeResult;
        }

        getNodesNormals(objectId, result);
        return result;
    }

    /**
     * Copy the node indices of all tetrahedra in this body.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a buffer containing 4 indices per tetrahedron (either storeResult
     * or a new buffer)
     */
    public IntBuffer copyTetras(IntBuffer storeResult) {
        int numTetras = countTetras();
        IntBuffer result;
        if (storeResult == null) {
            result = BufferUtils.createIntBuffer(4 * numTetras);
        } else {
            assert storeResult.capacity() == 4 * numTetras;
            result = storeResult;
        }

        getTetrasIndexes(objectId, result);
        return result;
    }

    /**
     * Copy the velocities of all nodes in this body.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a buffer containing 3 floats per node (in physics-space
     * coordinates, either storeResult or a new buffer)
     */
    public FloatBuffer copyVelocities(FloatBuffer storeResult) {
        int numNodes = countNodes();
        FloatBuffer result;
        if (storeResult == null) {
            result = BufferUtils.createFloatBuffer(3 * numNodes);
        } else {
            assert storeResult.capacity() == 3 * numNodes;
            result = storeResult;
        }

        getNodesVelocities(objectId, result);
        return result;
    }

    /**
     * Count the anchors in this body.
     *
     * @return the number of anchors (&ge;0)
     */
    final public int countAnchors() {
        return getAnchorCount(objectId);
    }

    /**
     * Count the clusters in this body.
     *
     * @return the number of clusters (&ge;0)
     */
    final public int countClusters() {
        return getClusterCount(objectId);
    }

    /**
     * Count the faces in this body.
     *
     * @return the number of faces (&ge;0)
     */
    final public int countFaces() {
        return getNbFaces(objectId);
    }

    /**
     * Count the links in this body.
     *
     * @return the number of links (&ge;0)
     */
    final public int countLinks() {
        return getNbLinks(objectId);
    }

    /**
     * Count the nodes in this body.
     *
     * @return the number of nodes (&ge;0)
     */
    final public int countNodes() {
        return getNbNodes(objectId);
    }

    /**
     * Count the nodes in the indexed cluster.
     *
     * @param clusterIndex which cluster (&ge;0, &lt;numClusters)
     * @return the number of nodes (&ge;0)
     */
    public int countNodesInCluster(int clusterIndex) {
        int numClusters = countClusters();
        Validate.inRange(clusterIndex, "cluster index", 0, numClusters - 1);

        return countNodesInCluster(objectId, clusterIndex);
    }

    /**
     * Count the tetrahedra in this body.
     *
     * @return the number of tetrahedra (&ge;0)
     */
    final public int countTetras() {
        return getNbTetras(objectId);
    }

    /**
     * Cut a pre-existing link in this body. TODO clarify the semantics
     *
     * @param nodeIndex0 the index of a node in the link (&ge;0,&lt;numNodes)
     * @param nodeIndex1 the index of the other node in the link
     * (&ge;0,&lt;numNodes)
     * @param cutLocation where to cut the link
     * @return true if successful, otherwise false
     */
    public boolean cutLink(int nodeIndex0, int nodeIndex1, float cutLocation) {
        int numNodes = countNodes();
        Validate.inRange(nodeIndex0, "node index 0", 0, numNodes - 1);
        Validate.inRange(nodeIndex1, "node index 1", 0, numNodes - 1);

        return cutLink(objectId, nodeIndex0, nodeIndex1, cutLocation);
    }

    /**
     * Generate bending constraints based on hops in the adjacency graph. This
     * may increase the number of links.
     *
     * @param numHops (in links, &ge;2)
     * @param material the material for appending links (not null)
     */
    public void generateBendingConstraints(int numHops, Material material) {
        Validate.inRange(numHops, "number of hops", 2, Integer.MAX_VALUE);

        long materialId = material.materialId;
        generateBendingConstraints(objectId, numHops, materialId);
    }

    /**
     * Generate one cluster per tetrahedron (or one per face if there are no
     * tetrahedra). Any pre-existing clusters are released.
     */
    public void generateClusters() {
        generateClusters(objectId, 0, 8_192);
    }

    /**
     * Generate clusters (K-mean). Any pre-existing clusters are released.
     *
     * @param k (&ge;1, &lt;numNodes)
     * @param maxIterations the maximum number of iterations (&gt;0,
     * default=8192)
     */
    public void generateClusters(int k, int maxIterations) {
        int numNodes = countNodes();
        Validate.inRange(k, "k", 1, numNodes);
        Validate.positive(maxIterations, "maximum number of iterations");

        generateClusters(objectId, k, maxIterations);
    }

    /**
     * Access the SoftBodyConfig of this body.
     *
     * @return the pre-existing instance (not null)
     */
    public SoftBodyConfig getSoftConfig() {
        return config;
    }

    /**
     * Access the Material of this body.
     *
     * @return the instance associated with this body (not null)
     */
    public Material getSoftMaterial() {
        if (material == null) {
            material = new Material(this);
        }

        return material;
    }

    /**
     * Access the world info used by this body. By default a single native
     * object is shared by all soft bodies.
     *
     * @return a new SoftBodyWorldInfo that references the pre-existing native
     * object (not null)
     */
    public SoftBodyWorldInfo getWorldInfo() {
        long worldInfoId = getSoftBodyWorldInfo(objectId);
        SoftBodyWorldInfo worldInfo = new SoftBodyWorldInfo(worldInfoId);

        return worldInfo;
    }

    /**
     * Test whether collisions are allowed between this body and the identified
     * collision object. Disallowed collisions may result from anchors.
     *
     * @param pcoId the native ID of the other collision object (not zero)
     * @return true if allowed, otherwise false
     */
    public boolean isCollisionAllowed(long pcoId) {
        Validate.nonZero(pcoId, "collision object ID");
        boolean result = isCollisionAllowed(objectId, pcoId);
        return result;
    }

    /**
     * List all nodes in the indexed cluster.
     *
     * @param clusterIndex which cluster (&ge;0, &lt;numClusters)
     * @param storeResult storage for the result (modified if not null)
     * @return a buffer containing node indices (either storeResult or a new
     * buffer)
     */
    public IntBuffer listNodesInCluster(int clusterIndex,
            IntBuffer storeResult) {
        int numClusters = countClusters();
        Validate.inRange(clusterIndex, "cluster index", 0, numClusters - 1);
        int numNodes = countNodesInCluster(clusterIndex);
        IntBuffer resultBuffer;
        if (storeResult == null) {
            resultBuffer = BufferUtils.createIntBuffer(numNodes);
        } else {
            assert storeResult.capacity() == numNodes;
            resultBuffer = storeResult;
        }

        listNodesInCluster(objectId, clusterIndex, resultBuffer);
        return resultBuffer;
    }

    /**
     * Read the collision margin of this body.
     *
     * @return the margin distance (in physics-space units, &gt;0)
     */
    public float margin() {
        return getMargin(objectId);
    }

    /**
     * Copy the location of the indexed node.
     *
     * @param nodeIndex which node (&ge;0, &lt;numNodes)
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in physics-space coordinates, either
     * storeResult or a new vector)
     */
    public Vector3f nodeLocation(int nodeIndex, Vector3f storeResult) {
        int numNodes = countNodes();
        Validate.inRange(nodeIndex, "node index", 0, numNodes - 1);
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        getNodeLocation(objectId, nodeIndex, result);
        return result;
    }

    /**
     * Read the mass of the indexed node.
     *
     * @param nodeIndex which node to read (&ge;0, &lt;numNodes)
     * @return the mass of the node (&gt;0)
     */
    public float nodeMass(int nodeIndex) {
        int numNodes = countNodes();
        Validate.inRange(nodeIndex, "node index", 0, numNodes - 1);

        return getMass(objectId, nodeIndex);
    }

    /**
     * Copy the normal vector of the indexed node.
     *
     * @param nodeIndex which node (&ge;0, &lt;numNodes)
     * @param storeResult storage for the result (modified if not null)
     * @return a normal vector (in physics-space coordinates, either storeResult
     * or a new vector)
     */
    public Vector3f nodeNormal(int nodeIndex, Vector3f storeResult) {
        int numNodes = countNodes();
        Validate.inRange(nodeIndex, "node index", 0, numNodes - 1);
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        getNodeNormal(objectId, nodeIndex, result);
        return result;
    }

    /**
     * Copy the velocity of the indexed node.
     *
     * @param nodeIndex which node (&ge;0, &lt;numNodes)
     * @param storeResult storage for the result (modified if not null)
     * @return a velocity vector (in physics-space coordinates, either
     * storeResult or a new vector)
     */
    public Vector3f nodeVelocity(int nodeIndex, Vector3f storeResult) {
        int numNodes = countNodes();
        Validate.inRange(nodeIndex, "node index", 0, numNodes - 1);
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        getNodeVelocity(objectId, nodeIndex, result);
        return result;
    }

    /**
     * Randomize constraints to reduce solver bias.
     */
    public void randomizeConstraints() {
        randomizeConstraints(objectId);
    }

    /**
     * Release all clusters.
     */
    public void releaseAllClusters() {
        releaseClusters(objectId);
    }

    /**
     * Release the indexed cluster.
     *
     * @param clusterIndex which cluster to release (&ge;0, &lt;numClusters)
     */
    public void releaseCluster(int clusterIndex) {
        int numClusters = countClusters();
        Validate.inRange(clusterIndex, "cluster index", 0, numClusters - 1);

        releaseCluster(objectId, clusterIndex);
    }

    /**
     * Remove the pre-existing anchor connecting the indexed node with the
     * specified rigid body.
     *
     * @param nodeIndex which node to disconnect (&ge;0, &lt;numNodes)
     * @param rigidBody which rigid body to disconnect (not null)
     */
    public void removeAnchor(int nodeIndex, PhysicsRigidBody rigidBody) {
        int numNodes = countNodes();
        Validate.inRange(nodeIndex, "node index", 0, numNodes - 1);
        Validate.nonNull(rigidBody, "rigid body");

        long rigidBodyId = rigidBody.getObjectId();
        removeAnchor(objectId, nodeIndex, rigidBodyId);
    }

    /**
     * Set the resting lengths of all links to their current lengths.
     */
    public void resetRestingLengths() {
        resetLinkRestLengths(objectId);
    }

    /**
     * Read the scale factor for resting lengths.
     *
     * @return the scale factor
     */
    public float restingLengthsScale() {
        return getRestLengthScale(objectId);
    }

    /**
     * Alter the collision margin of this body.
     *
     * @param margin the desired margin distance (in physics-space units, &gt;0)
     */
    final public void setMargin(float margin) {
        Validate.positive(margin, "margin");
        setMargin(objectId, margin);
    }

    /**
     * Alter the total mass for this body, distributing it based on the surface
     * area of each face.
     *
     * @param totalMass the desired total mass (&gt;0)
     */
    public void setMassByArea(float totalMass) {
        Validate.positive(totalMass, "total mass");
        setTotalMass(objectId, totalMass, true);
    }

    /**
     * Alter the total mass for this body, distributing it based on the current
     * mass of each node.
     *
     * @param totalMass the desired total mass (&gt;0, default=numNodes)
     */
    public void setMassByCurrent(float totalMass) {
        Validate.positive(totalMass, "total mass");
        setTotalMass(objectId, totalMass, false);
    }

    /**
     * Alter the masses of all nodes.
     *
     * @param masses a buffer containing the desired masses (not null, all
     * elements &gt;0)
     */
    public void setMasses(FloatBuffer masses) {
        Validate.nonNull(masses, "masses");
        setMasses(objectId, masses);
    }

    /**
     * Alter the total mass of this body, weighted by volume.
     *
     * @param density the desired density (&gt;0)
     */
    public void setMassFromDensity(float density) {
        Validate.positive(density, "density");
        setTotalDensity(objectId, density);
    }

    /**
     * Alter the mass of the indexed node.
     *
     * @param nodeIndex which node to modify (&ge;0, &lt;numNodes)
     * @param mass the desired mass (&gt;0)
     */
    public void setNodeMass(int nodeIndex, float mass) {
        int numNodes = countNodes();
        Validate.inRange(nodeIndex, "node index", 0, numNodes - 1);
        Validate.positive(mass, "mass");

        setMass(objectId, nodeIndex, mass);
    }

    /**
     * Alter the velocity of the indexed node.
     *
     * @param nodeIndex which node to modify (&ge;0, &lt;numNodes)
     * @param velocity the desired velocity vector (not null, unaffected)
     */
    public void setNodeVelocity(int nodeIndex, Vector3f velocity) {
        int numNodes = countNodes();
        Validate.inRange(nodeIndex, "node index", 0, numNodes - 1);
        Validate.finite(velocity, "velocity");

        setNodeVelocity(objectId, nodeIndex, velocity);
    }

    /**
     * Alter the normal vectors of all nodes.
     *
     * @param normals a buffer containing the desired normals (in physics-space
     * coordinates, not null, unaffected)
     */
    public void setNormals(FloatBuffer normals) {
        Validate.nonNull(normals, "normals");
        setNormals(objectId, normals);
    }

    /**
     * Set the "default pose" or "lowest energy state" of this body to its
     * current pose.
     *
     * @param setVolumePose true&rarr;alter the volume pose, false&rarr; don't
     * alter it
     * @param setFramePose true&rarr;alter the frame pose, false&rarr; don't
     * alter it
     */
    public void setPose(boolean setVolumePose, boolean setFramePose) {
        setPose(objectId, setVolumePose, setFramePose);
    }

    /**
     * Alter the scale factor for resting lengths.
     *
     * @param scale the desired scale factor
     */
    public void setRestingLengthScale(float scale) {
        setRestLengthScale(objectId, scale);
    }

    /**
     * Alter the velocities of all nodes.
     *
     * @param velocities a buffer containing the desired velocities (in
     * physics-space coordinates, not null, unaffected)
     */
    public void setVelocities(FloatBuffer velocities) {
        Validate.nonNull(velocities, "velocities");
        setVelocities(objectId, velocities);
    }

    /**
     * Alter the velocities of all nodes to make them identical.
     *
     * @param velocity the desired velocity vector (in physics-space
     * coordinates, not null, unaffected)
     */
    public void setVelocity(Vector3f velocity) {
        Validate.finite(velocity, "velocity");
        setVelocity(objectId, velocity);
    }

    /**
     * Set volume density (using tetrahedra) TODO clarify semantics
     *
     * @param density the desired density
     */
    public void setVolumeDensity(float density) {
        setVolumeDensity(objectId, density);
    }

    /**
     * Set volume mass (using tetrahedra) TODO clarify semantics
     *
     * @param mass the desired mass
     */
    public void setVolumeMass(float mass) {
        setVolumeMass(objectId, mass);
    }

    /**
     * Alter the wind velocity.
     *
     * @param velocity the desired velocity vector (in physics-space
     * coordinates, not null, unaffected)
     */
    public void setWindVelocity(Vector3f velocity) {
        Validate.finite(velocity, "velocity");
        setWindVelocity(objectId, velocity);
    }

    /**
     * Alter the world info of this body. Automatically invoked when this body
     * is added to a SoftPhysicsSpace.
     *
     * @param worldInfo the desired SoftBodyWorldInfo (not null)
     */
    public void setWorldInfo(SoftBodyWorldInfo worldInfo) {
        long worldInfoId = worldInfo.nativeId();
        setSoftBodyWorldInfo(objectId, worldInfoId);
    }

    /**
     * Calculate the volume of this body.
     *
     * @return the total volume (in cubic physics-space units, &ge;0)
     */
    public float volume() {
        return getVolume(objectId);
    }

    /**
     * Copy the wind velocity.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a velocity vector (in physics-space coordinates, either
     * storeResult or a new vector)
     */
    public Vector3f windVelocity(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getWindVelocity(objectId, result);
        return result;
    }
    // *************************************************************************
    // new protected methods

    /**
     * Destroy the pre-existing btSoftBody (if any).
     */
    protected void destroySoftBody() {
        if (objectId != 0L) {
            logger2.log(Level.FINE, "Destroying SoftBody {0}",
                    Long.toHexString(objectId));
            finalizeNative(objectId);
            objectId = 0L;
        }
    }

    native protected long getMaterial(long bodyId);

    /**
     * Reinitialize the btSoftBody to the default values.
     */
    protected void initDefault() {
        initDefault(objectId);
    }

    /**
     * Create a new, empty btSoftBody for this PhysicsSoftBody. The pre-existing
     * btSoftBody (if any) will be destroyed.
     */
    protected void newEmptySoftBody() {
        destroySoftBody();
        objectId = createEmptySoftBody();
        assert objectId != 0L;
        initUserPointer();
    }

    /**
     * Build/rebuild this body.
     */
    protected void rebuildSoftBody() {
        boolean removed = false;
        if (objectId != 0L) {
            if (isInWorld()) {
                PhysicsSpace.getPhysicsSpace().remove(this);
                removed = true;
            }
            logger2.log(Level.FINE, "Finalizing SoftBody {0}",
                    Long.toHexString(objectId));
            finalizeNative(objectId);
        }

        objectId = createEmptySoftBody();
        assert objectId != 0L;
        logger2.log(Level.FINE, "Created SoftBody {0}",
                Long.toHexString(objectId));

        if (removed) {
            PhysicsSpace.getPhysicsSpace().add(this);
        }
    }
    // *************************************************************************
    // PhysicsBody methods

    /**
     * Do not invoke directly! Joints are added automatically when created.
     *
     * @param joint the joint to add (not null)
     */
    @Override
    public void addJoint(PhysicsJoint joint) {
        Validate.nonNull(joint, "joint");

        if (!joints.contains(joint)) {
            joints.add(joint);
        }
    }

    /**
     * Calculate the axis-aligned bounding box for this body.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a bounding box (in physics-space coordinates, either storeResult
     * or a new instance)
     */
    @Override
    public BoundingBox boundingBox(BoundingBox storeResult) {
        BoundingBox result
                = (storeResult == null) ? new BoundingBox() : storeResult;

        Vector3f minima = new Vector3f(); // TODO reuse
        Vector3f maxima = new Vector3f();
        getBounds(objectId, minima, maxima);
        result.setMinMax(minima, maxima);

        return result;
    }

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned body into a deep-cloned one, using the specified cloner
     * and original to resolve copied fields.
     *
     * @param cloner the cloner that's cloning this body (not null)
     * @param original the instance from which this instance was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);
        rebuildSoftBody();

        PhysicsSoftBody old = (PhysicsSoftBody) original;
        copyPcoProperties(old);

        config = cloner.clone(config);
        joints = cloner.clone(joints);

        Material oldMaterial = old.getSoftMaterial();
        material = new Material(this);
        material.setAngularStiffness(oldMaterial.angularStiffness());
        material.setLinearStiffness(oldMaterial.linearStiffness());
        material.setVolumeStiffness(oldMaterial.volumeStiffness());

        SoftBodyWorldInfo oldInfo = old.getWorldInfo();
        SoftBodyWorldInfo newInfo = new SoftBodyWorldInfo();
        setWorldInfo(newInfo);
        newInfo.copyAll(oldInfo);

        FloatBuffer floats = old.copyLocations(null);
        appendNodes(floats);
        old.copyNormals(floats);
        setNormals(floats);
        old.copyVelocities(floats);
        setVelocities(floats);

        FloatBuffer masses = old.copyMasses(null);
        setMasses(masses);

        IntBuffer faces = old.copyFaces(null);
        IndexBuffer faceIndices = IndexBuffer.wrapIndexBuffer(faces);
        appendFaces(faceIndices);

        IntBuffer links = old.copyLinks(null);
        IndexBuffer linkIndices = IndexBuffer.wrapIndexBuffer(links);
        appendLinks(linkIndices);

        IntBuffer tetras = old.copyTetras(null);
        IndexBuffer tetraIndices = IndexBuffer.wrapIndexBuffer(tetras);
        appendLinks(tetraIndices);

        setDeactivationTime(old.getDeactivationTime());
    }

    /**
     * Count how many joints connect to this body.
     *
     * @return the count (&ge;0) or 0 if this body isn't added to any
     * PhysicsSpace
     */
    @Override
    public int countJoints() {
        int result = 0;
        if (isInWorld()) {
            result = joints.size();
        }

        return result;
    }

    /**
     * Copy this body's gravitational acceleration.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return an acceleration vector (in physics-space coordinates, either
     * storeResult or a new vector, not null)
     */
    @Override
    public Vector3f getGravity(Vector3f storeResult) {
        SoftBodyWorldInfo info = getWorldInfo();
        Vector3f result = info.copyGravity(storeResult);

        return result;
    }

    /**
     * Determine the total mass of this body.
     *
     * @return the total mass (&gt;0)
     */
    @Override
    public float getMass() {
        return getTotalMass(objectId);
    }

    /**
     * Locate the center of this body's bounding box. The bounding box isn't
     * updated on every frame.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in physics-space coordinates, either
     * storeResult or a new instance, not null)
     */
    @Override
    public Vector3f getPhysicsLocation(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getPhysicsLocation(objectId, result);

        assert Vector3f.isValidVector(result) : result;
        return result;
    }

    /**
     * Copy the orientation (rotation) of this body to a Quaternion.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a rotation quaternion (in physics-space coordinates, either
     * storeResult or a new instance, not null)
     */
    @Override
    public Quaternion getPhysicsRotation(Quaternion storeResult) {
        Quaternion result
                = (storeResult == null) ? new Quaternion() : storeResult;
        result.loadIdentity();
        return result;
    }

    /**
     * Copy the orientation of this body (the basis of its local coordinate
     * system) to a 3x3 matrix.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a rotation matrix (in physics-space coordinates, either
     * storeResult or a new matrix, not null)
     */
    @Override
    public Matrix3f getPhysicsRotationMatrix(Matrix3f storeResult) {
        Matrix3f result = (storeResult == null) ? new Matrix3f() : storeResult;
        result.loadIdentity();
        return result;
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public PhysicsSoftBody jmeClone() {
        try {
            PhysicsSoftBody clone = (PhysicsSoftBody) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }

    /**
     * Enumerate the joints connected to this body.
     *
     * @return a new array of pre-existing joints, or null if this body is not
     * added to any PhysicsSpace
     */
    @Override
    public PhysicsJoint[] listJoints() {
        PhysicsJoint[] result;
        if (isInWorld()) {
            int numJoints = joints.size();
            result = new PhysicsJoint[numJoints];
            joints.toArray(result);
        } else {
            result = null;
        }

        return result;
    }

    /**
     * De-serialize this body, for example when loading from a J3O file.
     *
     * @param importer the importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);
        InputCapsule capsule = importer.getCapsule(this);

        rebuildSoftBody();
        readPcoProperties(capsule);
        config = (SoftBodyConfig) capsule.readSavable("config", null);
        assert config != null;

        setRestingLengthScale(capsule.readFloat("RestLengthScale", 0f));
        setPhysicsLocation((Vector3f) capsule.readSavable("PhysicsLocation",
                new Vector3f()));
        setWorldInfo((SoftBodyWorldInfo) capsule.readSavable(
                "WorldInfo", new SoftBodyWorldInfo()));

        getSoftMaterial().read(capsule);
    }

    /**
     * Do not invoke directly! Joints are removed automatically when destroyed.
     *
     * @param joint the joint to remove (not null)
     */
    @Override
    public void removeJoint(PhysicsJoint joint) {
        Validate.nonNull(joint, "joint");
        joints.remove(joint);
    }

    /**
     * Alter which normals to include in new debug meshes.
     *
     * @param newSetting an enum value (either None or Smooth)
     */
    @Override
    public void setDebugMeshNormals(DebugMeshNormals newSetting) {
        Validate.nonNull(newSetting, "new setting");
        switch (newSetting) {
            case None:
            case Smooth:
                super.setDebugMeshNormals(newSetting);
                break;
            default:
                throw new IllegalArgumentException(newSetting.toString());
        }
    }

    /**
     * Alter this body's gravitational acceleration.
     * <p>
     * Invoke this method <em>after</em> adding this body to a PhysicsSpace.
     * Adding a body to a PhysicsSpace alters its gravity.
     *
     * @param acceleration the desired acceleration vector (in physics-space
     * coordinates, not null, unaffected)
     */
    @Override
    public void setGravity(Vector3f acceleration) {
        Validate.finite(acceleration, "acceleration");

        SoftBodyWorldInfo oldInfo = getWorldInfo();

        SoftBodyWorldInfo newInfo = new SoftBodyWorldInfo();
        newInfo.copyAll(oldInfo);
        newInfo.setGravity(acceleration);

        setWorldInfo(newInfo);
    }

    /**
     * Alter the total mass for this body, distributing it based on the current
     * mass of each node.
     *
     * @param totalMass the desired total mass (&gt;0, default=numNodes)
     */
    @Override
    public void setMass(float totalMass) {
        Validate.positive(totalMass, "total mass");
        setMassByCurrent(totalMass);
    }

    /**
     * Directly relocate the center of this body's bounding box. The bounding
     * box isn't updated on every frame.
     *
     * @param location the desired location (in physics-space coordinates, not
     * null, unaffected)
     */
    @Override
    public void setPhysicsLocation(Vector3f location) {
        Validate.finite(location, "location");
        setPhysicsLocation(objectId, location);
    }

    /**
     * Serialize this object, for example when saving to a J3O file.
     *
     * @param exporter the exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        super.write(exporter);
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(restingLengthsScale(), "RestLengthScale", 0f);
        capsule.write(getPhysicsLocation(), "PhysicsLocation", null);
        capsule.write(getWorldInfo(), "WorldInfo", null);
        // TODO anchors, joints, nodes, links, faces, tetras

        assert config != null;
        capsule.write(config, "config", null);
        getSoftMaterial().write(capsule);
    }
    // *************************************************************************
    // private methods

    native private void addAeroForceToNode(long bodyId, Vector3f windVector,
            int nodeIndex);

    native private void addForce(long bodyId, Vector3f forceVector);

    native private void addForce(long bodyId, Vector3f forceVector,
            int nodeIndex);

    native private void addVelocity(long bodyId, Vector3f velocityVector);

    native private void addVelocity(long bodyId, Vector3f velocityVector,
            int nodeIndex);

    native private void appendAnchor(long softBodyId, int nodeIndex,
            long rigidBodyId, Vector3f localPivotVector,
            boolean collisionBetweenLinkedBodies, float influenceFraction);

    native private void appendFaces(long bodyId, ByteBuffer byteBuffer);

    native private void appendFaces(long bodyId, IntBuffer intBuffer);

    native private void appendFaces(long bodyId, ShortBuffer shortBuffer);

    native private void appendLinks(long bodyId, ByteBuffer byteBuffer);

    native private void appendLinks(long bodyId, IntBuffer intBuffer);

    native private void appendLinks(long bodyId, ShortBuffer shortBuffer);

    native private void appendNodes(long bodyId, FloatBuffer locationBuffer);

    native private void appendTetras(long bodyId, ByteBuffer byteBuffer);

    native private void appendTetras(long bodyId, IntBuffer intBuffer);

    native private void appendTetras(long bodyId, ShortBuffer shortBuffer);

    native private void applyPhysicsRotation(long bodyId,
            Quaternion quaternion);

    native private void applyPhysicsScale(long bodyId, Vector3f vector);

    native private void applyPhysicsTransform(long bodyId, Transform transform);

    native private void applyPhysicsTranslate(long bodyId,
            Vector3f offsetVector);

    native private int countNodesInCluster(long objectId, int clusterIndex);

    native private long createEmptySoftBody();

    native private boolean cutLink(long bodyId, int nodeIndex0, int nodeIndex1,
            float cutLocation);

    native private void generateBendingConstraints(long bodyId, int distance,
            long materialId);

    native private void generateClusters(long bodyId, int k, int maxIterations);

    native private int getAnchorCount(long bodyId);

    native private float getAnchorInfluence(long bodyId, int anchorIndex);

    native private int getAnchorNodeIndex(long bodyId, int anchorIndex);

    native private void getAnchorPivot(long bodyId, int anchorIndex,
            Vector3f storePivot);

    native private long getAnchorRigidId(long bodyId, int anchorIndex);

    native private void getBounds(long objectId, Vector3f storeMinima,
            Vector3f storeMaxima);

    native private void getClusterCenter(long bodyId, int clusterIndex,
            Vector3f storeVector);

    native private int getClusterCount(long bodyId);

    native private void getClustersMasses(long bodyId, FloatBuffer storeBuffer);

    native private void getClustersPositions(long bodyId,
            FloatBuffer storeBuffer);

    native private void getFacesIndexes(long bodyId, IntBuffer storeBuffer);

    native private void getLinksIndexes(long bodyId, IntBuffer storeBuffer);

    native private float getMargin(long bodyId);

    native private float getMass(long bodyId, int nodeIndex);

    native private void getMasses(long bodyId, FloatBuffer storeBuffer);

    native private int getNbFaces(long bodyId);

    native private int getNbLinks(long bodyId);

    native private int getNbNodes(long bodyId);

    native private int getNbTetras(long bodyId);

    native private void getNodeLocation(long bodyId, int nodeIndex,
            Vector3f storeVector);

    native private void getNodeNormal(long bodyId, int nodeIndex,
            Vector3f storeVector);

    native private void getNodesNormals(long bodyId, FloatBuffer storeBuffer);

    native private void getNodesPositions(long bodyId, FloatBuffer storeBuffer);

    native private void getNodesVelocities(long bodyId,
            FloatBuffer storeBuffer);

    native private void getNodeVelocity(long bodyId, int nodeIndex,
            Vector3f storeVector);

    native private void getPhysicsLocation(long bodyId, Vector3f storeVector);

    native private float getRestLengthScale(long bodyId);

    native private long getSoftBodyWorldInfo(long bodyId);

    native private void getTetrasIndexes(long bodyId, IntBuffer indexBuffer);

    native private float getTotalMass(long bodyId);

    native private float getVolume(long bodyId);

    native private void getWindVelocity(long bodyId, Vector3f storeVector);

    native private void initDefault(long bodyId);

    native private boolean isCollisionAllowed(long softBodyId, long pcoId);

    native private void listNodesInCluster(long bodyId, int clusterIndex,
            IntBuffer indexBuffer);

    native private void randomizeConstraints(long bodyId);

    native private void releaseCluster(long bodyId, int index);

    native private void releaseClusters(long bodyId);

    native private void removeAnchor(long softBodyId, int nodeIndex,
            long rigidBodyId);

    native private void resetLinkRestLengths(long bodyId);

    native private void setMargin(long bodyId, float margin);

    native private void setMass(long bodyId, int nodeIndex, float mass);

    native private void setMasses(long bodyId, FloatBuffer massBuffer);

    native private void setNodeVelocity(long bodyId, int nodeIndex,
            Vector3f velocityVector);

    native private void setNormals(long bodyId, FloatBuffer normalBuffer);

    native private void setPhysicsLocation(long bodyId,
            Vector3f locationVector);

    native private void setPose(long bodyId, boolean setVolumePose,
            boolean setFramePose);

    native private void setRestLengthScale(long bodyId, float scale);

    native private void setSoftBodyWorldInfo(long bodyId, long worldInfoId);

    native private void setTotalDensity(long bodyId, float density);

    native private void setTotalMass(long bodyId, float mass,
            boolean fromFaces);

    native private void setVelocities(long bodyId, FloatBuffer velocityBuffer);

    native private void setVelocity(long bodyId, Vector3f velocityVector);

    native private void setVolumeDensity(long bodyId, float density);

    native private void setVolumeMass(long bodyId, float mass);

    native private void setWindVelocity(long bodyId, Vector3f velocityVector);

    /**
     * Provide access to 3 fields of the native btSoftBody::Material struct.
     * TODO make it an outer class
     */
    public class Material {
        // *********************************************************************
        // fields

        /**
         * unique identifier of this Material (not zero)
         */
        private long materialId;
        // *********************************************************************
        // constructors

        /**
         * Instantiate a Material with the default properties.
         *
         * @param body the body to which this Material will apply (not null)
         */
        private Material(PhysicsSoftBody body) {
            long softBodyId = body.getObjectId();
            materialId = body.getMaterial(softBodyId);
            assert materialId != 0L;
        }
        // *********************************************************************
        // new methods exposed

        /**
         * Read the angular stiffness coefficient (native field: m_kAST).
         *
         * @return the coefficient (&ge;0, &le;1)
         */
        public float angularStiffness() {
            return getAngularStiffnessFactor(materialId);
        }

        /**
         * Read the linear stiffness coefficient (native field: m_kLST).
         *
         * @return the coefficient (&ge;0, &le;1)
         */
        public float linearStiffness() {
            return getLinearStiffnessFactor(materialId);
        }

        /**
         * Alter the angular stiffness coefficient (native field: m_kAST).
         *
         * @param coefficient the desired coefficient (&ge;0, &le;1, default=1)
         */
        public void setAngularStiffness(float coefficient) {
            Validate.fraction(coefficient, "stiffness coefficient");
            setAngularStiffnessFactor(materialId, coefficient);
        }

        /**
         * Alter the linear stiffness coefficient (native field: m_kLST).
         *
         * @param coefficient the desired coefficient (&ge;0, &le;1, default=1)
         */
        public void setLinearStiffness(float coefficient) {
            Validate.fraction(coefficient, "stiffness coefficient");
            setLinearStiffnessFactor(materialId, coefficient);
        }

        /**
         * Alter the volume stiffness coefficient (native field: m_kVST).
         *
         * @param coefficient the desired coefficient (&ge;0, &le;1, default=1)
         */
        public void setVolumeStiffness(float coefficient) {
            Validate.fraction(coefficient, "stiffness coefficient");
            setVolumeStiffnessFactor(materialId, coefficient);
        }

        /**
         * Read the volume stiffness coefficient (native field: m_kVST).
         *
         * @return the coefficient (&ge;0, &le;1)
         */
        public float volumeStiffness() {
            return getVolumeStiffnessFactor(materialId);
        }
        // *********************************************************************
        // Object methods

        /**
         * Test for exact equivalence with another Object.
         *
         * @param otherObject the object to compare to (may be null)
         * @return true if the objects are equivalent, otherwise false
         */
        @Override
        public boolean equals(Object otherObject) {
            boolean result = false;
            if (otherObject == this) {
                result = true;
            } else if (otherObject instanceof Material) {
                Material otherMaterial = (Material) otherObject;
                result = (otherMaterial.materialId == materialId);
            }

            return result;
        }

        /**
         * Generate the hash code for this Material.
         *
         * @return the value to use for hashing
         */
        @Override
        public int hashCode() {
            int result = 313;
            result = 79 * result + (int) (materialId ^ (materialId >>> 32));

            return result;
        }
        // *********************************************************************
        // private methods

        native private float getAngularStiffnessFactor(long materialId);

        native private float getLinearStiffnessFactor(long materialId);

        native private float getVolumeStiffnessFactor(long materialId);

        /**
         * De-serialize this Material from the specified capsule, for example
         * when loading from a J3O file.
         *
         * @param capsule the capsule to read from (not null)
         * @throws IOException from the importer
         */
        private void read(InputCapsule capsule) throws IOException {
            setAngularStiffness(capsule.readFloat("AngularStiffness", 1f));
            setLinearStiffness(capsule.readFloat("LinearStiffness", 1f));
            setVolumeStiffness(capsule.readFloat("VolumeStiffness", 1f));
        }

        native private void setAngularStiffnessFactor(long materialId,
                float stiffness);

        native private void setLinearStiffnessFactor(long materialId,
                float stiffness);

        native private void setVolumeStiffnessFactor(long materialId,
                float stiffness);

        /**
         * Serialize this Material to the specified capsule, for example when
         * saving to a J3O file.
         *
         * @param capsule the capsule to write to (not null)
         * @throws IOException from the exporter
         */
        private void write(OutputCapsule capsule) throws IOException {
            capsule.write(angularStiffness(), "AngularStiffness", 1f);
            capsule.write(linearStiffness(), "LinearStiffness", 1f);
            capsule.write(volumeStiffness(), "VolumeStiffness", 1f);
        }
    }
}
