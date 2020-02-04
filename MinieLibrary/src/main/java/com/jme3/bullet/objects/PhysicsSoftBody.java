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
import com.jme3.bullet.SoftBodyWorldInfo;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.infos.DebugMeshNormals;
import com.jme3.bullet.objects.infos.Cluster;
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
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyBuffer;

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
     * number of axes in the coordinate system
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
    final public static Logger logger2
            = Logger.getLogger(PhysicsSoftBody.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagConfig = "config";
    final private static String tagFaceIndices = "faceIndices";
    final private static String tagIndices = "indices";
    final private static String tagLinkIndices = "linkIndices";
    final private static String tagNodeLocations = "nodeLocations";
    final private static String tagNodeMasses = "nodeMasses";
    final private static String tagNodeNormals = "nodeNormals";
    final private static String tagNodeVelocities = "nodeVelocities";
    final private static String tagNumClusters = "numClusters";
    final private static String tagPhysicsLocation = "physicsLocation";
    final private static String tagRestLengthScale = "restLengthScale";
    final private static String tagTetraIndices = "tetraIndices";
    // *************************************************************************
    // fields

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
        assert getInternalType(objectId) == 8 : getInternalType(objectId);
        logger2.log(Level.FINE, "Created {0}.", this);

        config = new SoftBodyConfig(this);
        super.initUserPointer();

        float defaultMargin = CollisionShape.getDefaultMargin();
        setMargin(defaultMargin);

        assert !isInWorld();
        assert isEmpty();
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
     * Append faces to this body. A Face is a triangle connecting 3 nodes.
     *
     * @param nodeIndices a face is created for every 3 indices in this buffer
     * (not null, direct, size a multiple of 3)
     */
    public void appendFaces(IndexBuffer nodeIndices) {
        if (!nodeIndices.getBuffer().isDirect()) {
            throw new IllegalArgumentException("The buffer must be direct.");
        }
        if (nodeIndices.size() % vpt != 0) {
            throw new IllegalArgumentException(
                    "The number of indices must be a multiple of 3.");
        }

        int numFaces = nodeIndices.size() / vpt;
        Buffer buffer = nodeIndices.getBuffer();
        if (buffer instanceof ByteBuffer) {
            appendFaces(objectId, numFaces, (ByteBuffer) buffer);
        } else if (buffer instanceof ShortBuffer) {
            appendFaces(objectId, numFaces, (ShortBuffer) buffer);
        } else if (buffer instanceof IntBuffer) {
            appendFaces(objectId, numFaces, (IntBuffer) buffer);
        } else {
            throw new IllegalArgumentException(
                    buffer.getClass().getSimpleName());
        }
    }

    /**
     * Append links to this body. A link is an interaction (or force) between 2
     * nodes.
     *
     * @param nodeIndices a link is created for each pair of indices in this
     * buffer (not null, direct, size a multiple of 2)
     */
    public void appendLinks(IndexBuffer nodeIndices) {
        if (!nodeIndices.getBuffer().isDirect()) {
            throw new IllegalArgumentException("The buffer must be direct.");
        }
        if (nodeIndices.size() % vpe != 0) {
            throw new IllegalArgumentException(
                    "The number of indices must be a multiple of 2.");
        }

        int numLinks = nodeIndices.size() / vpe;
        Buffer buffer = nodeIndices.getBuffer();
        if (buffer instanceof ByteBuffer) {
            appendLinks(objectId, numLinks, (ByteBuffer) buffer);
        } else if (buffer instanceof ShortBuffer) {
            appendLinks(objectId, numLinks, (ShortBuffer) buffer);
        } else if (buffer instanceof IntBuffer) {
            appendLinks(objectId, numLinks, (IntBuffer) buffer);
        } else {
            throw new IllegalArgumentException(
                    buffer.getClass().getSimpleName());
        }
    }

    /**
     * Append nodes to this body, each with mass=1. Nodes provide the shape of a
     * soft body. Each node has its own location, velocity, mass, etcetera.
     *
     * @param nodeLocations a node is created for every 3 floats in this buffer
     * (not null, direct, limit a multiple of 3)
     */
    public void appendNodes(FloatBuffer nodeLocations) {
        if (!nodeLocations.isDirect()) {
            throw new IllegalArgumentException("The buffer must be direct.");
        }
        if (nodeLocations.limit() % numAxes != 0) {
            throw new IllegalArgumentException(
                    "The number of floats must be a multiple of 3.");
        }

        int numNodes = nodeLocations.limit() / numAxes;
        appendNodes(objectId, numNodes, nodeLocations);
    }

    /**
     * Append tetrahedra to this body. A tetrahedron defines a volume between 4
     * nodes.
     *
     * @param tetrahedra a tetrahedron is created for every 4 indices in this
     * buffer (not null, direct, size a multiple of 4)
     */
    public void appendTetras(IndexBuffer tetrahedra) {
        if (!tetrahedra.getBuffer().isDirect()) {
            throw new IllegalArgumentException("The buffer must be direct.");
        }
        if (tetrahedra.size() % 4 != 0) {
            throw new IllegalArgumentException(
                    "The number of indices must be a multiple of 4.");
        }

        int numTetras = tetrahedra.size() / 4;
        Buffer buffer = tetrahedra.getBuffer();
        if (buffer instanceof ByteBuffer) {
            appendTetras(objectId, numTetras, (ByteBuffer) buffer);
        } else if (buffer instanceof ShortBuffer) {
            appendTetras(objectId, numTetras, (ShortBuffer) buffer);
        } else if (buffer instanceof IntBuffer) {
            appendTetras(objectId, numTetras, (IntBuffer) buffer);
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
     * Scale this body.
     *
     * @param factors the scale factor to apply to each axis (not null,
     * unaffected)
     */
    public void applyScale(Vector3f factors) {
        Validate.finite(factors, "factors");
        applyPhysicsScale(objectId, factors);
    }

    /**
     * Transform this body.
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
     * @param offset the translation to apply to each axis (not null,
     * unaffected)
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
     * @param storeResult storage for the result (direct, modified) or null
     * @return a direct buffer containing 3 floats per cluster (in physics-space
     * coordinates, either storeResult or a new buffer)
     */
    public FloatBuffer copyClusterCenters(FloatBuffer storeResult) {
        if (storeResult != null && !storeResult.isDirect()) {
            throw new IllegalArgumentException("The buffer must be direct.");
        }
        int numFloats = numAxes * countClusters();
        FloatBuffer result = MyBuffer.ensureCapacity(numFloats, storeResult);

        if (numFloats != 0) {
            getClustersPositions(objectId, result);
        }
        return result;
    }

    /**
     * Copy the masses of all clusters in this body.
     *
     * @param storeResult storage for the result (direct, modified) or null
     * @return a direct buffer containing a float value per cluster (either
     * storeResult or a new buffer)
     */
    public FloatBuffer copyClusterMasses(FloatBuffer storeResult) {
        if (storeResult != null && !storeResult.isDirect()) {
            throw new IllegalArgumentException("The buffer must be direct.");
        }
        int numFloats = countClusters();
        FloatBuffer result = MyBuffer.ensureCapacity(numFloats, storeResult);

        if (numFloats != 0) {
            getClustersMasses(objectId, result);
        }
        return result;
    }

    /**
     * Copy the node indices of all faces in this body.
     *
     * @param storeResult storage for the result (direct, modified) or null
     * @return a direct buffer containing 3 indices per face (either storeResult
     * or a new buffer)
     */
    public IntBuffer copyFaces(IntBuffer storeResult) {
        int numInts = vpt * countFaces();
        IntBuffer result;
        if (storeResult == null) {
            result = BufferUtils.createIntBuffer(numInts);
        } else {
            assert storeResult.isDirect();
            assert storeResult.capacity() == numInts;
            result = storeResult;
        }

        if (numInts != 0) {
            getFacesIndexes(objectId, result);
        }
        return result;
    }

    /**
     * Copy the node indices of all links in this body.
     *
     * @param storeResult storage for the result (direct, modified) or null
     * @return a direct buffer containing 2 indices per link (either storeResult
     * or a new buffer)
     */
    public IntBuffer copyLinks(IntBuffer storeResult) {
        int numInts = vpe * countLinks();
        IntBuffer result;
        if (storeResult == null) {
            result = BufferUtils.createIntBuffer(numInts);
        } else {
            assert storeResult.isDirect();
            assert storeResult.capacity() == numInts;
            result = storeResult;
        }

        if (numInts != 0) {
            getLinksIndexes(objectId, result);
        }
        return result;
    }

    /**
     * Copy the locations of all nodes in this body.
     *
     * @param storeResult storage for the result (direct, modified) or null
     * @return a direct buffer containing 3 floats per node (in physics-space
     * coordinates, either storeResult or a new buffer)
     */
    public FloatBuffer copyLocations(FloatBuffer storeResult) {
        if (storeResult != null && !storeResult.isDirect()) {
            throw new IllegalArgumentException("The buffer must be direct.");
        }
        int numFloats = numAxes * countNodes();
        FloatBuffer result = MyBuffer.ensureCapacity(numFloats, storeResult);

        if (numFloats != 0) {
            getNodesPositions(objectId, result);
        }
        return result;
    }

    /**
     * Copy the masses of all nodes in this body.
     *
     * @param storeResult storage for the result (direct, modified) or null
     * @return a direct buffer containing a float value per node (either
     * storeResult or a new buffer)
     */
    public FloatBuffer copyMasses(FloatBuffer storeResult) {
        if (storeResult != null && !storeResult.isDirect()) {
            throw new IllegalArgumentException("The buffer must be direct.");
        }
        int numFloats = countNodes();
        FloatBuffer result = MyBuffer.ensureCapacity(numFloats, storeResult);

        if (numFloats != 0) {
            getMasses(objectId, result);
        }
        return result;
    }

    /**
     * Copy the normal vectors of all nodes in this body.
     *
     * @param storeResult storage for the result (direct, modified) or null
     * @return a direct buffer containing 3 floats per node (in physics-space
     * coordinates, either storeResult or a new buffer)
     */
    public FloatBuffer copyNormals(FloatBuffer storeResult) {
        if (storeResult != null && !storeResult.isDirect()) {
            throw new IllegalArgumentException("The buffer must be direct.");
        }
        int numFloats = numAxes * countNodes();
        FloatBuffer result = MyBuffer.ensureCapacity(numFloats, storeResult);

        if (numFloats != 0) {
            getNodesNormals(objectId, result);
        }
        return result;
    }

    /**
     * Copy the node indices of all tetrahedra in this body.
     *
     * @param storeResult storage for the result (direct, modified) or null
     * @return a direct buffer containing 4 indices per tetrahedron (either
     * storeResult or a new buffer)
     */
    public IntBuffer copyTetras(IntBuffer storeResult) {
        int numInts = 4 * countTetras();
        IntBuffer result;
        if (storeResult == null) {
            result = BufferUtils.createIntBuffer(numInts);
        } else {
            assert storeResult.isDirect();
            assert storeResult.capacity() == numInts;
            result = storeResult;
        }

        if (numInts != 0) {
            getTetrasIndexes(objectId, result);
        }
        return result;
    }

    /**
     * Copy the velocities of all nodes in this body.
     *
     * @param storeResult storage for the result (direct, modified) or null
     * @return a direct buffer containing 3 floats per node (in physics-space
     * coordinates, either storeResult or a new buffer)
     */
    public FloatBuffer copyVelocities(FloatBuffer storeResult) {
        if (storeResult != null && !storeResult.isDirect()) {
            throw new IllegalArgumentException("The buffer must be direct.");
        }
        int numFloats = numAxes * countNodes();
        FloatBuffer result = MyBuffer.ensureCapacity(numFloats, storeResult);

        if (numFloats != 0) {
            getNodesVelocities(objectId, result);
        }
        return result;
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
     * Count the pinned nodes in this body.
     *
     * @return the number of pinned nodes (&ge;0)
     */
    final public int countPinnedNodes() {
        return getNbPinnedNodes(objectId);
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
     * Read the specified parameter of the indexed cluster.
     *
     * @param parameter which parameter to read (not null)
     * @param clusterIndex which cluster (&ge;0, &lt;numClusters)
     * @return the coefficient value
     */
    public float get(Cluster parameter, int clusterIndex) {
        int numClusters = countClusters();
        Validate.inRange(clusterIndex, "cluster index", 0, numClusters - 1);

        float result;
        switch (parameter) {
            case AngularDamping:
                result = getClusterAngularDamping(objectId, clusterIndex);
                break;
            case LinearDamping:
                result = getClusterLinearDamping(objectId, clusterIndex);
                break;
            case Matching:
                result = getClusterMatching(objectId, clusterIndex);
                break;
            case MaxSelfImpulse:
                result = getClusterMaxSelfImpulse(objectId, clusterIndex);
                break;
            case NodeDamping:
                result = getClusterNodeDamping(objectId, clusterIndex);
                break;
            case SelfImpulse:
                result = getClusterSelfImpulse(objectId, clusterIndex);
                break;
            default:
                throw new IllegalArgumentException(parameter.toString());
        }

        return result;
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
     * Test whether this body is empty.
     *
     * @return true if empty, otherwise false
     */
    final public boolean isEmpty() {
        boolean result = countNodes() == 0
                && countFaces() == 0
                && countLinks() == 0
                && countTetras() == 0
                && countJoints() == 0
                && countClusters() == 0;
        return result;
    }

    /**
     * List all nodes in the indexed cluster.
     *
     * @param clusterIndex which cluster (&ge;0, &lt;numClusters)
     * @param storeResult storage for the result (direct, modified) or null
     * @return a direct buffer containing node indices (either storeResult or a
     * new buffer)
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
            assert storeResult.isDirect();
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
     * @return the mass of the node (&ge;0)
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
     * Alter the specified parameter of the indexed cluster.
     *
     * @param parameter which parameter to alter (not null)
     * @param clusterIndex which cluster (&ge;0, &lt;numClusters)
     * @param value the desired value
     */
    public void set(Cluster parameter, int clusterIndex, float value) {
        int numClusters = countClusters();
        Validate.inRange(clusterIndex, "cluster index", 0, numClusters - 1);

        switch (parameter) {
            case AngularDamping:
                setClusterAngularDamping(objectId, clusterIndex, value);
                break;
            case LinearDamping:
                setClusterLinearDamping(objectId, clusterIndex, value);
                break;
            case Matching:
                setClusterMatching(objectId, clusterIndex, value);
                break;
            case MaxSelfImpulse:
                setClusterMaxSelfImpulse(objectId, clusterIndex, value);
                break;
            case NodeDamping:
                setClusterNodeDamping(objectId, clusterIndex, value);
                break;
            case SelfImpulse:
                setClusterSelfImpulse(objectId, clusterIndex, value);
                break;
            default:
                throw new IllegalArgumentException(parameter.toString());
        }
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
     * Alter the total mass for this body, distributing it based on the area of
     * each face.
     *
     * @param totalMass the desired total mass (&gt;0)
     */
    public void setMassByArea(float totalMass) {
        Validate.positive(totalMass, "total mass");
        setTotalMass(objectId, totalMass, true);
    }

    /**
     * Alter the total mass for this body, distributing it to nodes in
     * proportion to their current masses.
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
     * @param masses a direct buffer containing the desired masses (not null,
     * all elements &ge;0)
     */
    public void setMasses(FloatBuffer masses) {
        Validate.nonNull(masses, "masses");
        if (!masses.isDirect()) {
            throw new IllegalArgumentException("The buffer must be direct.");
        }

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
     * @param mass the desired mass (&ge;0, default=1)
     */
    public void setNodeMass(int nodeIndex, float mass) {
        int numNodes = countNodes();
        Validate.inRange(nodeIndex, "node index", 0, numNodes - 1);
        Validate.nonNegative(mass, "mass");

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
     * @param normals a direct buffer containing the desired normals (in
     * physics-space coordinates, not null, unaffected)
     */
    public void setNormals(FloatBuffer normals) {
        Validate.nonNull(normals, "normals");
        if (!normals.isDirect()) {
            throw new IllegalArgumentException("The buffer must be direct.");
        }
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
     * @param velocities a direct buffer containing the desired velocities (in
     * physics-space coordinates, not null, unaffected)
     */
    public void setVelocities(FloatBuffer velocities) {
        Validate.nonNull(velocities, "velocities");
        if (!velocities.isDirect()) {
            throw new IllegalArgumentException("The buffer must be direct.");
        }
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
     * Alter the world info of this body.
     * <p>
     * Invoke this method <em>after</em> adding the body to a PhysicsSpace.
     * Adding a body to a PhysicsSpace overrides its world info.
     *
     * @param worldInfo the desired SoftBodyWorldInfo (not null)
     */
    public void setWorldInfo(SoftBodyWorldInfo worldInfo) {
        if (!isInWorld()) {
            logger2.warning("The body is not in any PhysicsSpace.");
        }

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
            logger2.log(Level.FINE, "Destroying {0}.", this);
            finalizeNative(objectId);
            objectId = 0L;
        }
    }

    /**
     * Identify the Material for the identified btSoftBody.
     *
     * @param bodyId the ID of the btSoftBody (not 0)
     * @return the ID of the Material (not 0)
     */
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
        logger2.log(Level.FINE, "Created {0}.", this);

        initUserPointer();

        assert !isInWorld();
        assert isEmpty();
    }
    // *************************************************************************
    // PhysicsBody methods

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

        Vector3f minima = new Vector3f(); // TODO garbage
        Vector3f maxima = new Vector3f();
        getBounds(objectId, minima, maxima);
        result.setMinMax(minima, maxima);

        return result;
    }

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned body into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this body (not null)
     * @param original the instance from which this body was shallow-cloned (not
     * null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);
        newEmptySoftBody();

        PhysicsSoftBody old = (PhysicsSoftBody) original;
        copyPcoProperties(old);

        config = cloner.clone(config);

        Material oldMaterial = old.getSoftMaterial();
        material = new Material(this);
        material.setAngularStiffness(oldMaterial.angularStiffness());
        material.setLinearStiffness(oldMaterial.linearStiffness());
        material.setVolumeStiffness(oldMaterial.volumeStiffness());

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

        assert countClusters() == 0 : countClusters();
        int numClusters = old.countClusters();
        for (int clusterIndex = 0; clusterIndex < numClusters; ++clusterIndex) {
            IntBuffer nodeIndices = old.listNodesInCluster(clusterIndex, null);
            int numNodesInCluster = nodeIndices.capacity();
            appendCluster(objectId, numNodesInCluster, nodeIndices);

            for (Cluster clusterParameter : Cluster.values()) {
                float value = old.get(clusterParameter, clusterIndex);
                set(clusterParameter, clusterIndex, value);
            }
        }
        finishClusters(objectId);
        assert countClusters() == numClusters : countClusters();

        setDeactivationTime(old.getDeactivationTime());
        /*
         * Soft joints require clusters; clone them after appending clusters.
         */
        cloneJoints(cloner);
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
     * @return the total mass (&ge;0)
     */
    @Override
    public float getMass() {
        float totalMass = getTotalMass(objectId);
        assert totalMass >= 0f : totalMass;
        return totalMass;
    }

    /**
     * Locate the center of this body's axis-aligned bounding box.
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
     * Copy the scale factors of this object.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the scale factor for each local axis (either storeResult or a new
     * vector, not null, no negative component)
     */
    @Override
    public Vector3f getScale(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        result.set(1f, 1f, 1f);
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
     * De-serialize this body from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);
        InputCapsule capsule = importer.getCapsule(this);

        newEmptySoftBody();
        readPcoProperties(capsule);
        config = (SoftBodyConfig) capsule.readSavable(tagConfig, null);
        assert config != null;

        float[] fArray = capsule.readFloatArray(tagNodeLocations, new float[0]);
        appendNodes(BufferUtils.createFloatBuffer(fArray));

        fArray = capsule.readFloatArray(tagNodeMasses, new float[0]);
        setMasses(BufferUtils.createFloatBuffer(fArray));

        fArray = capsule.readFloatArray(tagNodeNormals, new float[0]);
        setNormals(BufferUtils.createFloatBuffer(fArray));

        fArray = capsule.readFloatArray(tagNodeVelocities, new float[0]);
        setVelocities(BufferUtils.createFloatBuffer(fArray));

        int[] nodeIndices = capsule.readIntArray(tagFaceIndices, new int[0]);
        IndexBuffer indexBuffer = IndexBuffer.wrapIndexBuffer(
                BufferUtils.createIntBuffer(nodeIndices));
        appendFaces(indexBuffer);

        nodeIndices = capsule.readIntArray(tagLinkIndices, new int[0]);
        indexBuffer = IndexBuffer.wrapIndexBuffer(
                BufferUtils.createIntBuffer(nodeIndices));
        appendLinks(indexBuffer);

        nodeIndices = capsule.readIntArray(tagTetraIndices, new int[0]);
        indexBuffer = IndexBuffer.wrapIndexBuffer(
                BufferUtils.createIntBuffer(nodeIndices));
        appendTetras(indexBuffer);

        assert countClusters() == 0 : countClusters();
        int numClusters = capsule.readInt(tagNumClusters, 0);
        for (int clusterIndex = 0; clusterIndex < numClusters; ++clusterIndex) {
            nodeIndices = capsule.readIntArray(tagIndices + clusterIndex,
                    new int[0]);
            int numNodesInCluster = nodeIndices.length;
            IntBuffer intBuffer = BufferUtils.createIntBuffer(nodeIndices);
            appendCluster(objectId, numNodesInCluster, intBuffer);

            for (Cluster clusterParameter : Cluster.values()) {
                String tag = clusterParameter.toString() + clusterIndex;
                float defValue = clusterParameter.defValue();
                float value = capsule.readFloat(tag, defValue);
                set(clusterParameter, clusterIndex, value);
            }
        }
        finishClusters(objectId);
        assert countClusters() == numClusters : countClusters();

        setRestingLengthScale(capsule.readFloat(tagRestLengthScale, 0f));
        setPhysicsLocation((Vector3f) capsule.readSavable(tagPhysicsLocation,
                new Vector3f()));

        getSoftMaterial().read(capsule);
        readJoints(capsule);
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
     * Invoke this method <em>after</em> adding the body to a PhysicsSpace.
     * Adding a body to a PhysicsSpace overrides its gravity.
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
     * Directly relocate the center of this body's bounding box.
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
     * Serialize this body to the specified exporter, for example when saving to
     * a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        super.write(exporter);
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(restingLengthsScale(), tagRestLengthScale, 0f);
        capsule.write(getPhysicsLocation(), tagPhysicsLocation, null);

        FloatBuffer floatBuffer = copyLocations(null);
        int capacity = floatBuffer.capacity();
        float[] floatArray = MyBuffer.toFloatArray(floatBuffer, 0, capacity);
        capsule.write(floatArray, tagNodeLocations, null);

        floatBuffer = copyMasses(null);
        capacity = floatBuffer.capacity();
        floatArray = MyBuffer.toFloatArray(floatBuffer, 0, capacity);
        capsule.write(floatArray, tagNodeMasses, null);

        floatBuffer = copyNormals(null);
        capacity = floatBuffer.capacity();
        floatArray = MyBuffer.toFloatArray(floatBuffer, 0, capacity);
        capsule.write(floatArray, tagNodeNormals, null);

        floatBuffer = copyVelocities(null);
        capacity = floatBuffer.capacity();
        floatArray = MyBuffer.toFloatArray(floatBuffer, 0, capacity);
        capsule.write(floatArray, tagNodeVelocities, null);

        IntBuffer intBuffer = copyFaces(null);
        capacity = intBuffer.capacity();
        int[] intArray = MyBuffer.toIntArray(intBuffer, 0, capacity);
        capsule.write(intArray, tagFaceIndices, null);

        intBuffer = copyLinks(null);
        capacity = intBuffer.capacity();
        intArray = MyBuffer.toIntArray(intBuffer, 0, capacity);
        capsule.write(intArray, tagLinkIndices, null);

        intBuffer = copyTetras(null);
        capacity = intBuffer.capacity();
        intArray = MyBuffer.toIntArray(intBuffer, 0, capacity);
        capsule.write(intArray, tagTetraIndices, null);

        int numClusters = countClusters();
        capsule.write(numClusters, tagNumClusters, 0);
        for (int clusterIndex = 0; clusterIndex < numClusters; ++clusterIndex) {
            intBuffer = listNodesInCluster(clusterIndex, null);
            capacity = intBuffer.capacity();
            intArray = MyBuffer.toIntArray(intBuffer, 0, capacity);
            capsule.write(intArray, tagIndices + clusterIndex, null);

            for (Cluster clusterParameter : Cluster.values()) {
                float value = get(clusterParameter, clusterIndex);
                String tag = clusterParameter.toString() + clusterIndex;
                float defValue = clusterParameter.defValue();
                capsule.write(value, tag, defValue);
            }
        }

        assert config != null;
        capsule.write(config, tagConfig, null);
        getSoftMaterial().write(capsule);

        writeJoints(capsule);
    }
    // *************************************************************************
    // native methods

    native private void addForce(long bodyId, Vector3f forceVector);

    native private void addForce(long bodyId, Vector3f forceVector,
            int nodeIndex);

    native private void addVelocity(long bodyId, Vector3f velocityVector);

    native private void addVelocity(long bodyId, Vector3f velocityVector,
            int nodeIndex);

    native private void appendCluster(long softBodyId, int numNodesInCluster,
            IntBuffer intBuffer);

    native private void appendFaces(long bodyId, int numFaces,
            ByteBuffer byteBuffer);

    native private void appendFaces(long bodyId, int numFaces,
            IntBuffer intBuffer);

    native private void appendFaces(long bodyId, int numFaces,
            ShortBuffer shortBuffer);

    native private void appendLinks(long bodyId, int numLinks,
            ByteBuffer byteBuffer);

    native private void appendLinks(long bodyId, int numLinks,
            IntBuffer intBuffer);

    native private void appendLinks(long bodyId, int numLinks,
            ShortBuffer shortBuffer);

    native private void appendNodes(long bodyId, int numNodes,
            FloatBuffer locationBuffer);

    native private void appendTetras(long bodyId, int numNodes,
            ByteBuffer byteBuffer);

    native private void appendTetras(long bodyId, int numNodes,
            IntBuffer intBuffer);

    native private void appendTetras(long bodyId, int numNodes,
            ShortBuffer shortBuffer);

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

    native private void finishClusters(long softBodyId);

    native private void generateBendingConstraints(long bodyId, int distance,
            long materialId);

    native private void generateClusters(long bodyId, int k, int maxIterations);

    native private void getBounds(long objectId, Vector3f storeMinima,
            Vector3f storeMaxima);

    native private float getClusterAngularDamping(long bodyId,
            int clusterIndex);

    native private void getClusterCenter(long bodyId, int clusterIndex,
            Vector3f storeVector);

    native private int getClusterCount(long bodyId);

    native private float getClusterLinearDamping(long bodyId, int clusterIndex);

    native private float getClusterMatching(long bodyId, int clusterIndex);

    native private float getClusterMaxSelfImpulse(long bodyId,
            int clusterIndex);

    native private float getClusterNodeDamping(long bodyId, int clusterIndex);

    native private float getClusterSelfImpulse(long bodyId, int clusterIndex);

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

    native private int getNbPinnedNodes(long bodyId);

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

    native private void resetLinkRestLengths(long bodyId);

    native private void setClusterAngularDamping(long bodyId, int clusterIndex,
            float damping);

    native private void setClusterLinearDamping(long bodyId, int clusterIndex,
            float damping);

    native private void setClusterMatching(long bodyId, int clusterIndex,
            float coefficient);

    native private void setClusterMaxSelfImpulse(long bodyId, int clusterIndex,
            float impulse);

    native private void setClusterNodeDamping(long bodyId, int clusterIndex,
            float damping);

    native private void setClusterSelfImpulse(long bodyId, int clusterIndex,
            float factor);

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
        // constants and loggers

        /**
         * field names for serialization
         */
        final private static String tagAngularStiffness = "angularStiffness";
        final private static String tagLinearStiffness = "linearStiffness";
        final private static String tagVolumeStiffness = "volumeStiffness";
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

        /**
         * De-serialize this Material from the specified capsule, for example
         * when loading from a J3O file.
         *
         * @param capsule the capsule to read from (not null)
         * @throws IOException from the importer
         */
        private void read(InputCapsule capsule) throws IOException {
            setAngularStiffness(capsule.readFloat(tagAngularStiffness, 1f));
            setLinearStiffness(capsule.readFloat(tagLinearStiffness, 1f));
            setVolumeStiffness(capsule.readFloat(tagVolumeStiffness, 1f));
        }

        /**
         * Serialize this Material to the specified capsule, for example when
         * saving to a J3O file.
         *
         * @param capsule the capsule to write to (not null)
         * @throws IOException from the exporter
         */
        private void write(OutputCapsule capsule) throws IOException {
            capsule.write(angularStiffness(), tagAngularStiffness, 1f);
            capsule.write(linearStiffness(), tagLinearStiffness, 1f);
            capsule.write(volumeStiffness(), tagVolumeStiffness, 1f);
        }
        // *********************************************************************
        // native methods

        native private float getAngularStiffnessFactor(long materialId);

        native private float getLinearStiffnessFactor(long materialId);

        native private float getVolumeStiffnessFactor(long materialId);

        native private void setAngularStiffnessFactor(long materialId,
                float stiffness);

        native private void setLinearStiffnessFactor(long materialId,
                float stiffness);

        native private void setVolumeStiffnessFactor(long materialId,
                float stiffness);
    }
}
