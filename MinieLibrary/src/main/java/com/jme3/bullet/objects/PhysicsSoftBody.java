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
import com.jme3.bullet.collision.PcoType;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.objects.infos.Cluster;
import com.jme3.bullet.objects.infos.SoftBodyConfig;
import com.jme3.bullet.objects.infos.SoftBodyMaterial;
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
import com.simsilica.mathd.Matrix3d;
import com.simsilica.mathd.Quatd;
import com.simsilica.mathd.Vec3d;
import java.io.IOException;
import java.nio.Buffer;
import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.nio.ShortBuffer;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.MeshNormals;
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
    final private static String tagIsWorldInfoProtected
            = "isWorldInfoProtected";
    final private static String tagLinkIndices = "linkIndices";
    final private static String tagMaterial = "material";
    final private static String tagNodeLocations = "nodeLocations";
    final private static String tagNodeMasses = "nodeMasses";
    final private static String tagNodeNormals = "nodeNormals";
    final private static String tagNodeVelocities = "nodeVelocities";
    final private static String tagNumClusters = "numClusters";
    final private static String tagPhysicsLocation = "physicsLocation";
    final private static String tagRestLengthScale = "restLengthScale";
    final private static String tagTetraIndices = "tetraIndices";
    final private static String tagWorldInfo = "worldInfo";
    // *************************************************************************
    // fields

    /**
     * false&rarr; world info should be replaced when this body gets added to a
     * space, true&rarr;world info should be preserved
     */
    private boolean isWorldInfoProtected = false;
    /**
     * configuration properties of this soft body
     */
    private SoftBodyConfig config;
    /**
     * material properties of this soft body, allocated lazily
     */
    private SoftBodyMaterial material = null;
    /**
     * properties (including gravity) that may be replaced when this body gets
     * added to a space
     */
    private SoftBodyWorldInfo worldInfo;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an empty soft body. The new body is not added to any physics
     * space.
     */
    public PhysicsSoftBody() {
        this.worldInfo = new SoftBodyWorldInfo();
        long infoId = worldInfo.nativeId();
        long bodyId = createEmpty(infoId);
        super.setNativeId(bodyId);
        assert getInternalType(bodyId) == PcoType.soft :
                getInternalType(bodyId);
        logger2.log(Level.FINE, "Created {0}.", this);

        this.config = new SoftBodyConfig(this);
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

        long objectId = nativeId();
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

        long objectId = nativeId();
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

        long objectId = nativeId();
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

        long objectId = nativeId();
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
        Validate.nonNull(nodeLocations, "node locations");
        Validate.require(nodeLocations.isDirect(), "direct buffer");
        Validate.require(nodeLocations.limit() % numAxes == 0,
                "limit a multiple of 3");

        long objectId = nativeId();
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

        long objectId = nativeId();
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

        long objectId = nativeId();
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

        long objectId = nativeId();
        addForce(objectId, force, nodeIndex);
    }

    /**
     * Rotate this body.
     *
     * @param rotation the rotation to apply (not null, unaffected)
     */
    public void applyRotation(Quaternion rotation) {
        Validate.nonNull(rotation, "rotation");

        long objectId = nativeId();
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

        long objectId = nativeId();
        applyPhysicsScale(objectId, factors);
    }

    /**
     * Transform this body.
     *
     * @param transform the transform to apply (not null, unaffected)
     */
    public void applyTransform(Transform transform) {
        Validate.nonNull(transform, "transform");

        long objectId = nativeId();
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

        long objectId = nativeId();
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

        long objectId = nativeId();
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
            long objectId = nativeId();
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
            long objectId = nativeId();
            getClustersMasses(objectId, result);
        }

        return result;
    }

    /**
     * Copy the (linear) velocities of all clusters in this body.
     *
     * @param storeResult storage for the result (direct, modified) or null
     * @return a direct buffer containing 3 floats per cluster (in physics-space
     * coordinates, either storeResult or a new buffer)
     */
    public FloatBuffer copyClusterVelocities(FloatBuffer storeResult) {
        if (storeResult != null && !storeResult.isDirect()) {
            throw new IllegalArgumentException("The buffer must be direct.");
        }
        int numFloats = numAxes * countClusters();
        FloatBuffer result = MyBuffer.ensureCapacity(numFloats, storeResult);

        if (numFloats != 0) {
            long objectId = nativeId();
            getClustersLinearVelocities(objectId, result);
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
            long objectId = nativeId();
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
            long objectId = nativeId();
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
            long objectId = nativeId();
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
            long objectId = nativeId();
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
            long objectId = nativeId();
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
            long objectId = nativeId();
            getTetrasIndexes(objectId, result);
        }

        return result;
    }

    /**
     * Copy the (linear) velocities of all nodes in this body.
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
            long objectId = nativeId();
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
        long objectId = nativeId();
        int result = getClusterCount(objectId);

        return result;
    }

    /**
     * Count the faces in this body.
     *
     * @return the number of faces (&ge;0)
     */
    final public int countFaces() {
        long objectId = nativeId();
        int result = getNbFaces(objectId);

        return result;
    }

    /**
     * Count the links in this body.
     *
     * @return the number of links (&ge;0)
     */
    final public int countLinks() {
        long objectId = nativeId();
        int result = getNbLinks(objectId);

        return result;
    }

    /**
     * Count the nodes in this body.
     *
     * @return the number of nodes (&ge;0)
     */
    final public int countNodes() {
        long objectId = nativeId();
        int result = getNbNodes(objectId);

        return result;
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

        long objectId = nativeId();
        int result = countNodesInCluster(objectId, clusterIndex);

        return result;
    }

    /**
     * Count the pinned nodes in this body.
     *
     * @return the number of pinned nodes (&ge;0)
     */
    final public int countPinnedNodes() {
        long objectId = nativeId();
        int result = getNbPinnedNodes(objectId);

        return result;
    }

    /**
     * Count the tetrahedra in this body.
     *
     * @return the number of tetrahedra (&ge;0)
     */
    final public int countTetras() {
        long objectId = nativeId();
        int result = getNbTetras(objectId);

        return result;
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

        long objectId = nativeId();
        boolean result = cutLink(objectId, nodeIndex0, nodeIndex1, cutLocation);

        return result;
    }

    /**
     * Generate bending constraints based on hops in the adjacency graph. This
     * may increase the number of links.
     *
     * @param numHops (in links, &ge;2)
     * @param material the material for appending links (not null)
     */
    public void generateBendingConstraints(int numHops,
            SoftBodyMaterial material) {
        Validate.inRange(numHops, "number of hops", 2, Integer.MAX_VALUE);

        long objectId = nativeId();
        long materialId = material.nativeId();
        generateBendingConstraints(objectId, numHops, materialId);
    }

    /**
     * Generate one cluster per tetrahedron (or one per face if there are no
     * tetrahedra). Any pre-existing clusters are released.
     */
    public void generateClusters() {
        long objectId = nativeId();
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

        long objectId = nativeId();
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
        long objectId = nativeId();
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
    public SoftBodyMaterial getSoftMaterial() {
        if (material == null) {
            this.material = new SoftBodyMaterial(this);
        }

        return material;
    }

    /**
     * Access the world info.
     *
     * @return the pre-existing instance (not null)
     */
    public SoftBodyWorldInfo getWorldInfo() {
        assert worldInfo != null;
        assert worldInfo.nativeId() == getSoftBodyWorldInfo(nativeId());
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

        long objectId = nativeId();
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
     * Test whether this body's world info should be replaced when added to a
     * space.
     *
     * @return false if the info should be replaced, true if it should be
     * preserved
     */
    public boolean isWorldInfoProtected() {
        boolean result = isWorldInfoProtected;
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

        long objectId = nativeId();
        listNodesInCluster(objectId, clusterIndex, resultBuffer);

        return resultBuffer;
    }

    /**
     * Read the collision margin of this body.
     *
     * @return the margin distance (in physics-space units, &gt;0)
     */
    public float margin() {
        long objectId = nativeId();
        float result = getMargin(objectId);

        return result;
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

        long objectId = nativeId();
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

        long objectId = nativeId();
        float result = getMass(objectId, nodeIndex);

        return result;
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

        long objectId = nativeId();
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

        long objectId = nativeId();
        getNodeVelocity(objectId, nodeIndex, result);

        return result;
    }

    /**
     * Randomize constraints to reduce solver bias.
     */
    public void randomizeConstraints() {
        long objectId = nativeId();
        randomizeConstraints(objectId);
    }

    /**
     * Release all clusters.
     */
    public void releaseAllClusters() {
        long objectId = nativeId();
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

        long objectId = nativeId();
        releaseCluster(objectId, clusterIndex);
    }

    /**
     * Set the resting lengths of all links to their current lengths.
     */
    public void resetRestingLengths() {
        long objectId = nativeId();
        resetLinkRestLengths(objectId);
    }

    /**
     * Read the scale factor for resting lengths.
     *
     * @return the scale factor
     */
    public float restingLengthsScale() {
        long objectId = nativeId();
        float result = getRestLengthScale(objectId);

        return result;
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

        long objectId = nativeId();
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

        long objectId = nativeId();
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

        long objectId = nativeId();
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

        long objectId = nativeId();
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

        long objectId = nativeId();
        setMasses(objectId, masses);
    }

    /**
     * Alter the total mass of this body, weighted by volume.
     *
     * @param density the desired density (&gt;0)
     */
    public void setMassFromDensity(float density) {
        Validate.positive(density, "density");

        long objectId = nativeId();
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

        long objectId = nativeId();
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

        long objectId = nativeId();
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

        long objectId = nativeId();
        setNormals(objectId, normals);
    }

    /**
     * Directly relocate the center of this body's bounding box.
     *
     * @param location the desired location (in physics-space coordinates, not
     * null, unaffected)
     */
    public void setPhysicsLocationDp(Vec3d location) {
        Validate.nonNull(location, "location");

        long objectId = nativeId();
        setPhysicsLocationDp(objectId, location);
    }

    /**
     * Set the "default pose" (lowest energy state) of this body based on its
     * current pose.
     *
     * @param setVolumePose true&rarr;alter the volume pose, false&rarr; don't
     * alter it
     * @param setFramePose true&rarr;alter the frame pose, false&rarr; don't
     * alter it
     */
    public void setPose(boolean setVolumePose, boolean setFramePose) {
        long objectId = nativeId();
        setPose(objectId, setVolumePose, setFramePose);
    }

    /**
     * Alter whether this body's world info should be replaced when the body
     * gets added to a space.
     *
     * @param newState true to preserve the world info, false to allow it to be
     * replaced (default=false)
     */
    public void setProtectWorldInfo(boolean newState) {
        this.isWorldInfoProtected = newState;
    }

    /**
     * Alter the scale factor for resting lengths.
     *
     * @param scale the desired scale factor
     */
    public void setRestingLengthScale(float scale) {
        long objectId = nativeId();
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

        long objectId = nativeId();
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

        long objectId = nativeId();
        setVelocity(objectId, velocity);
    }

    /**
     * Set volume density (using tetrahedra) TODO clarify semantics
     *
     * @param density the desired density
     */
    public void setVolumeDensity(float density) {
        long objectId = nativeId();
        setVolumeDensity(objectId, density);
    }

    /**
     * Set volume mass (using tetrahedra) TODO clarify semantics
     *
     * @param mass the desired mass
     */
    public void setVolumeMass(float mass) {
        long objectId = nativeId();
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

        long objectId = nativeId();
        setWindVelocity(objectId, velocity);
    }

    /**
     * Replace the world info of this body.
     * <p>
     * Invoke this method <em>after</em> adding the body to a space. Adding a
     * body to a space may replace its world info.
     *
     * @param worldInfo the desired SoftBodyWorldInfo (not null, alias created)
     */
    public void setWorldInfo(SoftBodyWorldInfo worldInfo) {
        if (!isInWorld()) {
            logger2.warning("The body is not in any space.");
        }

        long objectId = nativeId();
        long worldInfoId = worldInfo.nativeId();
        setSoftBodyWorldInfo(objectId, worldInfoId);

        this.worldInfo = worldInfo;
    }

    /**
     * Calculate the volume of this body.
     *
     * @return the total volume (in cubic physics-space units, &ge;0)
     */
    public float volume() {
        long objectId = nativeId();
        float result = getVolume(objectId);

        return result;
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

        long objectId = nativeId();
        getWindVelocity(objectId, result);

        return result;
    }
    // *************************************************************************
    // new protected methods

    /**
     * Destroy the pre-existing btSoftBody (if any) except for its worldInfo.
     */
    protected void destroySoftBody() {
        if (hasAssignedNativeObject()) {
            logger2.log(Level.FINE, "Destroying {0}.", this);
            unassignNativeObject();
        }

        this.material = null;
        this.config = null;
    }

    /**
     * Reinitialize the btSoftBody to the default values.
     */
    protected void initDefault() {
        long objectId = nativeId();
        initDefault(objectId);
    }

    /**
     * Create an empty {@code btSoftBody} for this PhysicsSoftBody, using the
     * pre-existing worldInfo. The pre-existing btSoftBody (if any) will be
     * destroyed.
     */
    protected void newEmptySoftBody() {
        destroySoftBody();

        long infoId = worldInfo.nativeId();
        long objectId = createEmpty(infoId);
        setNativeId(objectId);
        assert getInternalType(objectId) == PcoType.soft :
                getInternalType(objectId);
        logger2.log(Level.FINE, "Created {0}.", this);

        this.config = new SoftBodyConfig(this);
        initUserPointer();

        assert !isInWorld();
        assert countNodes() == 0;
        assert countFaces() == 0;
        assert countLinks() == 0;
        assert countTetras() == 0;
        assert countClusters() == 0;
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
        long objectId = nativeId();
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
        assert !hasAssignedNativeObject();
        PhysicsSoftBody old = (PhysicsSoftBody) original;
        assert old != this;
        assert old.hasAssignedNativeObject();

        super.cloneFields(cloner, original);
        if (hasAssignedNativeObject()) {
            return;
        }

        this.worldInfo = cloner.clone(old.worldInfo);
        long infoId = worldInfo.nativeId();
        long objectId = createEmpty(infoId);
        setNativeId(objectId);
        assert getInternalType(objectId) == PcoType.soft :
                getInternalType(objectId);
        logger2.log(Level.FINE, "Created {0}.", this);

        this.config = new SoftBodyConfig(this);
        initUserPointer();

        cloneIgnoreList(cloner, old);
        copyPcoProperties(old);
        config.copyAll(old.config);
        this.material = cloner.clone(old.material);

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

        // Soft joints require clusters; clone them after appending clusters.
        cloneJoints(cloner, old);
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
        long objectId = nativeId();
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

        long objectId = nativeId();
        getPhysicsLocation(objectId, result);

        assert Vector3f.isValidVector(result) : result;
        return result;
    }

    /**
     * Locate the center of this body's axis-aligned bounding box.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in physics-space coordinates, either
     * storeResult or a new instance, not null, finite)
     */
    @Override
    public Vec3d getPhysicsLocationDp(Vec3d storeResult) {
        Vec3d result = (storeResult == null) ? new Vec3d() : storeResult;

        long objectId = nativeId();
        getPhysicsLocationDp(objectId, result);

        assert result.isFinite() : result;
        return result;
    }

    /**
     * Copy the orientation (rotation) of this body to a Quaternion.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return an identity quaternion (either storeResult or a new instance, not
     * null)
     */
    @Override
    public Quaternion getPhysicsRotation(Quaternion storeResult) {
        Quaternion result
                = (storeResult == null) ? new Quaternion() : storeResult;
        result.loadIdentity();
        return result;
    }

    /**
     * Copy the orientation (rotation) of this body to a Quatd.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return an identity quaternion (either storeResult or a new instance, not
     * null)
     */
    @Override
    public Quatd getPhysicsRotationDp(Quatd storeResult) {
        Quatd result;
        if (storeResult == null) {
            result = new Quatd();
        } else {
            result = storeResult.set(0., 0., 0., 1.);
        }
        return result;
    }

    /**
     * Copy the orientation of this body (the basis of its local coordinate
     * system) to a Matrix3f.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return an identity matrix (either storeResult or a new instance, not
     * null)
     */
    @Override
    public Matrix3f getPhysicsRotationMatrix(Matrix3f storeResult) {
        Matrix3f result = (storeResult == null) ? new Matrix3f() : storeResult;
        result.loadIdentity();
        return result;
    }

    /**
     * Copy the orientation of this body (the basis of its local coordinate
     * system) to a Matrix3d.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return an identity matrix (either storeResult or a new instance, not
     * null)
     */
    @Override
    public Matrix3d getPhysicsRotationMatrixDp(Matrix3d storeResult) {
        Matrix3d result;
        if (storeResult == null) {
            result = new Matrix3d();
        } else {
            result = storeResult.makeIdentity();
        }
        return result;
    }

    /**
     * Copy the scale factors of this object.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the value (1,1,1) (either storeResult or a new vector)
     */
    @Override
    public Vector3f getScale(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        result.set(1f, 1f, 1f);
        return result;
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
        InputCapsule capsule = importer.getCapsule(this);

        this.worldInfo
                = (SoftBodyWorldInfo) capsule.readSavable(tagWorldInfo, null);
        newEmptySoftBody(); // needs worldInfo!

        super.read(importer);
        readPcoProperties(capsule);
        this.isWorldInfoProtected
                = capsule.readBoolean(tagIsWorldInfoProtected, false);

        this.config = (SoftBodyConfig) capsule.readSavable(tagConfig, null);
        assert config != null;

        this.material
                = (SoftBodyMaterial) capsule.readSavable(tagMaterial, null);

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
        long objectId = nativeId();
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

        readJoints(capsule);
    }

    /**
     * Alter which normals to include in new debug meshes.
     *
     * @param newSetting an enum value (either None or Smooth)
     */
    @Override
    public void setDebugMeshNormals(MeshNormals newSetting) {
        Validate.nonNull(newSetting, "new setting");

        switch (newSetting) {
            case None:
            case Smooth:
                super.setDebugMeshNormals(newSetting);
                break;
            default:
                String message = "normals = " + newSetting;
                throw new IllegalArgumentException(message);
        }
    }

    /**
     * Alter this body's gravitational acceleration.
     * <p>
     * Invoke this method <em>after</em> adding the body to a space. Adding a
     * body to a space may override its gravity.
     *
     * @param acceleration the desired acceleration vector (in physics-space
     * coordinates, not null, unaffected)
     */
    @Override
    public void setGravity(Vector3f acceleration) {
        Validate.finite(acceleration, "acceleration");

        SoftBodyWorldInfo newInfo = new SoftBodyWorldInfo();
        newInfo.copyAll(worldInfo);
        newInfo.setGravity(acceleration);

        setWorldInfo(newInfo);
    }

    /**
     * Alter the total mass for this body, distributing it in proportion to the
     * current mass of each node.
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
     * null, finite, unaffected)
     */
    @Override
    public void setPhysicsLocation(Vector3f location) {
        Validate.finite(location, "location");

        long objectId = nativeId();
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

        capsule.write(isWorldInfoProtected, tagIsWorldInfoProtected, false);
        capsule.write(restingLengthsScale(), tagRestLengthScale, 0f);
        capsule.write(getPhysicsLocation(null), tagPhysicsLocation, null);

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

        assert worldInfo != null;
        capsule.write(worldInfo, tagWorldInfo, null);

        assert config != null;
        capsule.write(config, tagConfig, null);
        capsule.write(material, tagMaterial, null);

        writeJoints(capsule);
    }
    // *************************************************************************
    // native private methods

    native private static void addForce(long bodyId, Vector3f forceVector);

    native private static void
            addForce(long bodyId, Vector3f forceVector, int nodeIndex);

    native private static void
            addVelocity(long bodyId, Vector3f velocityVector);

    native private static void
            addVelocity(long bodyId, Vector3f velocityVector, int nodeIndex);

    native private static void appendCluster(
            long softBodyId, int numNodesInCluster, IntBuffer intBuffer);

    native private static void
            appendFaces(long bodyId, int numFaces, ByteBuffer byteBuffer);

    native private static void
            appendFaces(long bodyId, int numFaces, IntBuffer intBuffer);

    native private static void
            appendFaces(long bodyId, int numFaces, ShortBuffer shortBuffer);

    native private static void
            appendLinks(long bodyId, int numLinks, ByteBuffer byteBuffer);

    native private static void
            appendLinks(long bodyId, int numLinks, IntBuffer intBuffer);

    native private static void
            appendLinks(long bodyId, int numLinks, ShortBuffer shortBuffer);

    native private static void
            appendNodes(long bodyId, int numNodes, FloatBuffer locationBuffer);

    native private static void
            appendTetras(long bodyId, int numNodes, ByteBuffer byteBuffer);

    native private static void
            appendTetras(long bodyId, int numNodes, IntBuffer intBuffer);

    native private static void
            appendTetras(long bodyId, int numNodes, ShortBuffer shortBuffer);

    native private static void
            applyPhysicsRotation(long bodyId, Quaternion quaternion);

    native private static void applyPhysicsScale(long bodyId, Vector3f vector);

    native private static void
            applyPhysicsTransform(long bodyId, Transform transform);

    native private static void
            applyPhysicsTranslate(long bodyId, Vector3f offsetVector);

    native private static int
            countNodesInCluster(long objectId, int clusterIndex);

    native private static long createEmpty(long infoId);

    native private static boolean cutLink(
            long bodyId, int nodeIndex0, int nodeIndex1, float cutLocation);

    native private static void finishClusters(long softBodyId);

    native private static void generateBendingConstraints(
            long bodyId, int distance, long materialId);

    native private static void
            generateClusters(long bodyId, int k, int maxIterations);

    native private static void getBounds(
            long objectId, Vector3f storeMinima, Vector3f storeMaxima);

    native private static float
            getClusterAngularDamping(long bodyId, int clusterIndex);

    native private static void getClusterCenter(
            long bodyId, int clusterIndex, Vector3f storeVector);

    native private static int getClusterCount(long bodyId);

    native private static float
            getClusterLinearDamping(long bodyId, int clusterIndex);

    native private static float
            getClusterMatching(long bodyId, int clusterIndex);

    native private static float
            getClusterMaxSelfImpulse(long bodyId, int clusterIndex);

    native private static float
            getClusterNodeDamping(long bodyId, int clusterIndex);

    native private static float
            getClusterSelfImpulse(long bodyId, int clusterIndex);

    native private static void
            getClustersLinearVelocities(long bodyId, FloatBuffer storeBuffer);

    native private static void
            getClustersMasses(long bodyId, FloatBuffer storeBuffer);

    native private static void
            getClustersPositions(long bodyId, FloatBuffer storeBuffer);

    native private static void
            getFacesIndexes(long bodyId, IntBuffer storeBuffer);

    native private static void
            getLinksIndexes(long bodyId, IntBuffer storeBuffer);

    native private static float getMargin(long bodyId);

    native private static float getMass(long bodyId, int nodeIndex);

    native private static void getMasses(long bodyId, FloatBuffer storeBuffer);

    native private static int getNbFaces(long bodyId);

    native private static int getNbLinks(long bodyId);

    native private static int getNbNodes(long bodyId);

    native private static int getNbPinnedNodes(long bodyId);

    native private static int getNbTetras(long bodyId);

    native private static void
            getNodeLocation(long bodyId, int nodeIndex, Vector3f storeVector);

    native private static void
            getNodeNormal(long bodyId, int nodeIndex, Vector3f storeVector);

    native private static void
            getNodesNormals(long bodyId, FloatBuffer storeBuffer);

    native private static void
            getNodesPositions(long bodyId, FloatBuffer storeBuffer);

    native private static void
            getNodesVelocities(long bodyId, FloatBuffer storeBuffer);

    native private static void
            getNodeVelocity(long bodyId, int nodeIndex, Vector3f storeVector);

    native private static void
            getPhysicsLocation(long bodyId, Vector3f storeVector);

    native private static void
            getPhysicsLocationDp(long bodyId, Vec3d storeVector);

    native private static float getRestLengthScale(long bodyId);

    native private static long getSoftBodyWorldInfo(long bodyId);

    native private static void
            getTetrasIndexes(long bodyId, IntBuffer indexBuffer);

    native private static float getTotalMass(long bodyId);

    native private static float getVolume(long bodyId);

    native private static void
            getWindVelocity(long bodyId, Vector3f storeVector);

    native private static void initDefault(long bodyId);

    native private static boolean
            isCollisionAllowed(long softBodyId, long pcoId);

    native private static void listNodesInCluster(
            long bodyId, int clusterIndex, IntBuffer indexBuffer);

    native private static void randomizeConstraints(long bodyId);

    native private static void releaseCluster(long bodyId, int index);

    native private static void releaseClusters(long bodyId);

    native private static void resetLinkRestLengths(long bodyId);

    native private static void setClusterAngularDamping(
            long bodyId, int clusterIndex, float damping);

    native private static void setClusterLinearDamping(
            long bodyId, int clusterIndex, float damping);

    native private static void setClusterMatching(
            long bodyId, int clusterIndex, float coefficient);

    native private static void setClusterMaxSelfImpulse(
            long bodyId, int clusterIndex, float impulse);

    native private static void setClusterNodeDamping(
            long bodyId, int clusterIndex, float damping);

    native private static void
            setClusterSelfImpulse(long bodyId, int clusterIndex, float factor);

    native private static void setMargin(long bodyId, float margin);

    native private static void setMass(long bodyId, int nodeIndex, float mass);

    native private static void setMasses(long bodyId, FloatBuffer massBuffer);

    native private static void setNodeVelocity(
            long bodyId, int nodeIndex, Vector3f velocityVector);

    native private static void
            setNormals(long bodyId, FloatBuffer normalBuffer);

    native private static void
            setPhysicsLocation(long bodyId, Vector3f locationVector);

    native private static void
            setPhysicsLocationDp(long bodyId, Vec3d locationVector);

    native private static void
            setPose(long bodyId, boolean setVolumePose, boolean setFramePose);

    native private static void setRestLengthScale(long bodyId, float scale);

    native private static void
            setSoftBodyWorldInfo(long bodyId, long worldInfoId);

    native private static void setTotalDensity(long bodyId, float density);

    native private static void
            setTotalMass(long bodyId, float mass, boolean fromFaces);

    native private static void
            setVelocities(long bodyId, FloatBuffer velocityBuffer);

    native private static void
            setVelocity(long bodyId, Vector3f velocityVector);

    native private static void setVolumeDensity(long bodyId, float density);

    native private static void setVolumeMass(long bodyId, float mass);

    native private static void
            setWindVelocity(long bodyId, Vector3f velocityVector);
}
