/*
 * Copyright (c) 2020-2024 jMonkeyEngine
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
package com.jme3.bullet.collision.shapes.infos;

import com.jme3.bounding.BoundingBox;
import com.jme3.bullet.NativePhysicsObject;
import com.jme3.bullet.collision.shapes.MeshCollisionShape;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import com.jme3.util.clone.JmeCloneable;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A Bounding-Value Hierarchy (BVH) generated for a MeshCollisionShape, based on
 * Bullet's {@code btOptimizedBvh}.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class BoundingValueHierarchy
        extends NativePhysicsObject
        implements JmeCloneable, Savable {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(BoundingValueHierarchy.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagBytes = "bytes";
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected BoundingValueHierarchy() {
    }

    /**
     * Instantiate an (untracked) reference to the hierarchy of the specified
     * {@code MeshCollisionShape}.
     *
     * @param meshShape the pre-existing shape (not null)
     */
    public BoundingValueHierarchy(MeshCollisionShape meshShape) {
        Validate.nonNull(meshShape, "mesh shape");

        long shapeId = meshShape.nativeId();
        long bvhId = getOptimizedBvh(shapeId);
        super.setNativeIdNotTracked(bvhId);
    }

    /**
     * Instantiate a (tracked) hierarchy from serialized bytes.
     * <p>
     * If the bytes weren't generated on the current {@code Platform} with the
     * same floating-point precision, the results are undefined (likely a JVM
     * crash).
     *
     * @param bytes the serialized bytes (not null, unaffected)
     */
    public BoundingValueHierarchy(byte[] bytes) {
        Validate.nonNull(bytes, "bytes");

        long bvhId = deSerialize(bytes);
        super.setNativeId(bvhId);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the bounds of the hierarchy.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return an axis-aligned bounding box (either {@code storeResult} or a new
     * instance)
     */
    public BoundingBox copyAabb(BoundingBox storeResult) {
        BoundingBox result
                = (storeResult == null) ? new BoundingBox() : storeResult;

        long bvhId = nativeId();
        Vector3f maxima = new Vector3f(); // TODO garbage
        Vector3f minima = new Vector3f();
        getAabb(bvhId, minima, maxima);
        result.setMinMax(minima, maxima);

        return result;
    }

    /**
     * Copy the quantization vector of the hierarchy.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return an vector (either {@code storeResult} or a new vector)
     */
    public Vector3f copyQuantization(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long bvhId = nativeId();
        getQuantization(bvhId, result);

        return result;
    }

    /**
     * Count the leaf nodes in the hierarchy.
     *
     * @return the count (&ge;0)
     */
    public int countLeafNodes() {
        long bvhId = nativeId();
        int result = getNumLeafNodes(bvhId);

        assert result >= 0 : result;
        return result;
    }

    /**
     * Count all nodes in the hierarchy.
     *
     * @return the count (&ge;0)
     */
    public int countNodes() {
        long bvhId = nativeId();
        int result = getNumNodes(bvhId);

        assert result >= 0 : result;
        return result;
    }

    /**
     * Count the subtree headers in the hierarchy (native field:
     * m_SubtreeHeaders).
     *
     * @return the count (&ge;0)
     */
    public int countSubtreeHeaders() {
        long bvhId = nativeId();
        int result = getNumSubtreeHeaders(bvhId);

        assert result >= 0 : result;
        return result;
    }

    /**
     * Return the escape index of the specified node.
     *
     * @param nodeIndex the index of the node (&ge;0)
     * @return the escape index (&ge;0) or -1 if the node is a leaf
     */
    public int escapeIndex(int nodeIndex) {
        long bvhId = nativeId();
        int lastNode = getNumNodes(bvhId) - 1;
        Validate.inRange(nodeIndex, "node index", 0, lastNode);
        int result = getEscapeIndex(bvhId, nodeIndex);

        assert result >= -1 : result;
        return result;
    }

    /**
     * Test whether the hierarchy uses quantized AABB compression.
     *
     * @return true if compressed, otherwise false
     */
    public boolean isCompressed() {
        long bvhId = nativeId();
        boolean result = isCompressed(bvhId);
        return result;
    }

    /**
     * Test whether the specified node is a leaf.
     *
     * @param nodeIndex the index of the node (&ge;0)
     * @return true if a leaf, false if an internal node
     */
    public boolean isLeafNode(int nodeIndex) {
        long bvhId = nativeId();
        int lastNode = getNumNodes(bvhId) - 1;
        Validate.inRange(nodeIndex, "node index", 0, lastNode);

        boolean result = isLeafNode(bvhId, nodeIndex);
        return result;
    }

    /**
     * Return the part index of the specified node.
     *
     * @param nodeIndex the index of the node (&ge;0)
     * @return the part index (&ge;0) or -1 if the node isn't a leaf
     */
    public int partId(int nodeIndex) {
        long bvhId = nativeId();
        int lastNode = getNumNodes(bvhId) - 1;
        Validate.inRange(nodeIndex, "node index", 0, lastNode);
        int result = getPartId(bvhId, nodeIndex);

        assert result >= -1 : result;
        return result;
    }

    /**
     * Serialize the hierarchy to a byte array.
     * <p>
     * Serialization can be used to avoid re-generating the BVH of a
     * `MeshCollisionShape` each time it is instantiated. The resulting bytes
     * are specific to a particular `MeshCollisionShape`. They are also specific
     * to the current {@code Platform} and floating-point precision.
     *
     * @return a new array containing serialized bytes (not null)
     */
    public byte[] serialize() {
        long bvhId = nativeId();
        byte[] result = serialize(bvhId);

        assert result != null;
        return result;
    }

    /**
     * Alter the traversal mode (native field: m_traversalMode).
     *
     * @param mode 0 for "stackless" or 1 for "stackless cache-friendly" or 2
     * for "recursive" (default=0)
     */
    public void setTraversalMode(int mode) {
        Validate.inRange(mode, "mode", 0, 2);

        long bvhId = nativeId();
        setTraversalMode(bvhId, mode);
    }

    /**
     * Return the traversal mode (native field: m_traversalMode).
     *
     * @return 0 for "stackless" or 1 for "stackless cache-friendly" or 2 for
     * "recursive"
     */
    public int traversalMode() {
        long bvhId = nativeId();
        int result = getTraversalMode(bvhId);

        assert result >= 0 && result <= 2 : result;
        return result;
    }

    /**
     * Return the triangle index of the specified node.
     *
     * @param nodeIndex the index of the node (&ge;0)
     * @return the triangle index (&ge;0) or -1 if the node isn't a leaf
     */
    public int triangleIndex(int nodeIndex) {
        long bvhId = nativeId();
        int lastNode = getNumNodes(bvhId) - 1;
        Validate.inRange(nodeIndex, "node index", 0, lastNode);
        int result = getTriangleIndex(bvhId, nodeIndex);

        assert result >= -1 : result;
        return result;
    }
    // *************************************************************************
    // JmeCloneable methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned hierarchy into a deep-cloned one, using the specified
     * Cloner and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this hierarchy (not null)
     * @param original the instance from which this hierarchy was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        BoundingValueHierarchy originalBvh = (BoundingValueHierarchy) original;
        byte[] bytes = originalBvh.serialize();
        long bvhId = deSerialize(bytes);
        reassignNativeId(bvhId);
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public BoundingValueHierarchy jmeClone() {
        try {
            BoundingValueHierarchy clone = (BoundingValueHierarchy) clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }
    // *************************************************************************
    // Savable methods

    /**
     * De-serialize this mesh from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        InputCapsule capsule = importer.getCapsule(this);

        byte[] bytes = capsule.readByteArray(tagBytes, null);
        long bvhId = deSerialize(bytes);
        setNativeId(bvhId);
    }

    /**
     * Serialize this mesh to the specified exporter, for example when saving to
     * a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        OutputCapsule capsule = exporter.getCapsule(this);

        byte[] bytes = serialize();
        capsule.write(bytes, tagBytes, null);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Free the identified tracked native object. Invoked by reflection.
     *
     * @param bvhId the native identifier (not zero)
     */
    private static void freeNativeObject(long bvhId) {
        assert bvhId != 0L;
        finalizeNative(bvhId);
    }
    // *************************************************************************
    // native private methods

    native private static long deSerialize(byte[] buffer);

    native private static void finalizeNative(long bvhId);

    native private static void getAabb(
            long bvhId, Vector3f storeMinima, Vector3f storeMaxima);

    native private static int getEscapeIndex(long bvhId, int nodeIndex);

    native private static int getNumLeafNodes(long bvhId);

    native private static int getNumNodes(long bvhId);

    native private static int getNumSubtreeHeaders(long bvhId);

    native private static long getOptimizedBvh(long shapeId);

    native private static int getPartId(long bvhId, int nodeIndex);

    native private static void getQuantization(
            long bvhId, Vector3f storeVector);

    native private static int getTraversalMode(long bvhId);

    native private static int getTriangleIndex(long bvhId, int nodeIndex);

    native private static boolean isCompressed(long bvhId);

    native private static boolean isLeafNode(long bvhId, int nodeIndex);

    native private static byte[] serialize(long bvhId);

    native private static void setTraversalMode(long bvhId, int mode);
}
