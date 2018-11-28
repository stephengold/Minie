/*
 * Copyright (c) 2018 jMonkeyEngine
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
package com.jme3.bullet.animation;

import com.jme3.animation.Bone;
import com.jme3.animation.Skeleton;
import com.jme3.export.InputCapsule;
import com.jme3.export.Savable;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer;
import java.io.IOException;
import java.nio.Buffer;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import java.util.logging.Logger;
import jme3utilities.MyMesh;
import jme3utilities.MySkeleton;
import jme3utilities.MySpatial;
import jme3utilities.MyString;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * Utility methods used by DynamicAnimControl and associated classes.
 *
 * @author Stephen Gold sgold@sonic.net
 *
 * Based on KinematicRagdollControl by Normen Hansen and Rémy Bouquet (Nehon).
 */
public class RagUtils {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(RagUtils.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private RagUtils() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Assign each mesh vertex to a link and add its location (mesh coordinates
     * in bind pose) to that link's list.
     *
     * @param meshes array of animated meshes to use (not null, unaffected)
     * @param managerMap a map from bone indices to managing link names (not
     * null, unaffected)
     * @return a new map from link names to lists of coordinates
     */
    public static Map<String, List<Vector3f>> coordsMap(Mesh[] meshes,
            String[] managerMap) {
        float[] wArray = new float[4];
        int[] iArray = new int[4];
        Map<String, List<Vector3f>> coordsMap = new HashMap<>(32);
        for (Mesh mesh : meshes) {
            int numVertices = mesh.getVertexCount();
            for (int vertexI = 0; vertexI < numVertices; vertexI++) {
                MyMesh.vertexBoneIndices(mesh, vertexI, iArray);
                MyMesh.vertexBoneWeights(mesh, vertexI, wArray);

                Map<String, Float> weightMap
                        = weightMap(iArray, wArray, managerMap);

                float bestTotalWeight = Float.NEGATIVE_INFINITY;
                String bestLbName = null;
                for (Map.Entry<String, Float> entry : weightMap.entrySet()) {
                    float totalWeight = entry.getValue();
                    if (totalWeight >= bestTotalWeight) {
                        bestTotalWeight = totalWeight;
                        bestLbName = entry.getKey();
                    }
                }
                /*
                 * Add the bind-pose coordinates of the vertex
                 * to the linked bone's list.
                 */
                List<Vector3f> coordList;
                if (coordsMap.containsKey(bestLbName)) {
                    coordList = coordsMap.get(bestLbName);
                } else {
                    coordList = new ArrayList<>(20);
                    coordsMap.put(bestLbName, coordList);
                }
                Vector3f bindPosition = MyMesh.vertexVector3f(mesh,
                        VertexBuffer.Type.BindPosePosition, vertexI, null);
                coordList.add(bindPosition);
            }
        }

        return coordsMap;
    }

    /**
     * Find the main root bone of a skeleton, based on its total bone weight.
     *
     * @param skeleton the skeleton (not null, unaffected)
     * @param targetMeshes an array of animated meshes to provide bone weights
     * (not null)
     * @return a root bone, or null if none found
     */
    public static Bone findMainBone(Skeleton skeleton, Mesh[] targetMeshes) {
        Validate.nonNull(targetMeshes, "target meshes");

        Bone[] rootBones = skeleton.getRoots();

        Bone result;
        if (rootBones.length == 1) {
            result = rootBones[0];
        } else {
            result = null;
            float[] totalWeights = totalWeights(targetMeshes, skeleton);
            float greatestTotalWeight = Float.NEGATIVE_INFINITY;
            for (Bone rootBone : rootBones) {
                int boneIndex = skeleton.getBoneIndex(rootBone);
                float weight = totalWeights[boneIndex];
                if (weight > greatestTotalWeight) {
                    result = rootBone;
                    greatestTotalWeight = weight;
                }
            }
        }

        return result;
    }

    /**
     * Calculate the distance of the location furthest from the origin.
     *
     * @param locations the collection of location vectors (not null, all
     * elements non-null, unaffected)
     * @return the distance (&ge;0)
     */
    public static float maxDistance(Iterable<Vector3f> locations) {
        double maxRSquared = 0.0;
        for (Vector3f location : locations) {
            double rSquared = MyVector3f.lengthSquared(location);
            if (rSquared > maxRSquared) {
                maxRSquared = rSquared;
            }
        }
        float distance = (float) Math.sqrt(maxRSquared);

        assert distance > 0f : distance;
        return distance;
    }

    /**
     * Convert a transform from the mesh coordinate system to the local
     * coordinate system of the specified bone.
     *
     * @param parentBone (not null)
     * @param transform the transform to convert (not null, modified)
     */
    public static void meshToLocal(Bone parentBone, Transform transform) {
        Vector3f location = transform.getTranslation();
        Quaternion orientation = transform.getRotation();
        Vector3f scale = transform.getScale();

        Vector3f pmTranslate = parentBone.getModelSpacePosition();
        Quaternion pmRotInv = parentBone.getModelSpaceRotation().inverse();
        Vector3f pmScale = parentBone.getModelSpaceScale();

        location.subtractLocal(pmTranslate);
        location.divideLocal(pmScale);
        pmRotInv.mult(location, location);
        scale.divideLocal(pmScale);
        pmRotInv.mult(orientation, orientation);
    }

    /**
     * Read an array of transforms from an input capsule.
     *
     * @param capsule the input capsule (not null)
     * @param fieldName the name of the field to read (not null)
     * @return a new array or null
     * @throws IOException from capsule
     */
    public static Transform[] readTransformArray(InputCapsule capsule,
            String fieldName) throws IOException {
        Validate.nonNull(capsule, "capsule");
        Validate.nonNull(fieldName, "field name");

        Savable[] tmp = capsule.readSavableArray(fieldName, null);
        Transform[] result;
        if (tmp == null) {
            result = null;
        } else {
            result = new Transform[tmp.length];
            for (int i = 0; i < tmp.length; i++) {
                result[i] = (Transform) tmp[i];
            }
        }

        return result;
    }

    /**
     * Calculate a coordinate transform for the specified spatial relative to a
     * specified ancestor node. The result incorporates the transform of the
     * starting spatial, but not that of the ancestor.
     *
     * @param startSpatial the starting spatial (not null, unaffected)
     * @param ancestorNode the ancestor node (not null, unaffected)
     * @param storeResult storage for the result (modified if not null)
     * @return a coordinate transform (either storeResult or a new vector, not
     * null)
     */
    public static Transform relativeTransform(Spatial startSpatial,
            Node ancestorNode, Transform storeResult) {
        Validate.nonNull(startSpatial, "spatial");
        Validate.nonNull(ancestorNode, "ancestor");
        assert startSpatial.hasAncestor(ancestorNode);
        Transform result
                = (storeResult == null) ? new Transform() : storeResult;

        result.loadIdentity();
        Spatial loopSpatial = startSpatial;
        while (loopSpatial != ancestorNode) {
            Transform localTransform = loopSpatial.getLocalTransform();
            result.combineWithParent(localTransform);
            loopSpatial = loopSpatial.getParent();
        }

        return result;
    }

    /**
     * Validate a skeleton for use with DynamicAnimControl.
     *
     * @param skeleton the skeleton to validate (not null, unaffected)
     */
    public static void validate(Skeleton skeleton) {
        int numBones = skeleton.getBoneCount();
        if (numBones < 0) {
            throw new IllegalArgumentException("Bone count is negative!");
        }

        Set<String> nameSet = new TreeSet<>();
        for (int boneIndex = 0; boneIndex < numBones; boneIndex++) {
            Bone bone = skeleton.getBone(boneIndex);
            if (bone == null) {
                String msg = String.format("Bone %d in skeleton is null!",
                        boneIndex);
                throw new IllegalArgumentException(msg);
            }
            String boneName = bone.getName();
            if (boneName == null) {
                String msg = String.format("Bone %d in skeleton has null name!",
                        boneIndex);
                throw new IllegalArgumentException(msg);
            } else if (boneName.equals(ConfigDynamicAnimControl.torsoName)) {
                String msg = String.format(
                        "Bone %d in skeleton has a reserved name!",
                        boneIndex);
                throw new IllegalArgumentException(msg);
            } else if (nameSet.contains(boneName)) {
                String msg = "Duplicate bone name in skeleton: "
                        + MyString.quote(boneName);
                throw new IllegalArgumentException(msg);
            }
            nameSet.add(boneName);
        }
    }

    /**
     * Validate a model for use with DynamicAnimControl.
     *
     * @param model the model to validate (not null, unaffected)
     */
    public static void validate(Spatial model) {
        Validate.nonNull(model, "model");

        List<Geometry> geometries
                = MySpatial.listSpatials(model, Geometry.class, null);
        if (geometries.isEmpty()) {
            throw new IllegalArgumentException("No meshes in the model.");
        }
        for (Geometry geometry : geometries) {
            if (geometry.isIgnoreTransform()) {
                throw new IllegalArgumentException(
                        "A model geometry ignores transforms.");
            }
        }
    }

    /**
     * Enumerate the world locations of all vertices in the specified subtree of
     * the scene graph. Note: recursive!
     *
     * @param subtree (may be null)
     * @param storeResult (added to if not null)
     * @return an expanded list (either storeResult or a new instance)
     */
    public static List<Vector3f> vertexLocations(Spatial subtree,
            List<Vector3f> storeResult) {
        List<Vector3f> result = (storeResult == null)
                ? new ArrayList<Vector3f>(100) : storeResult;

        if (subtree instanceof Geometry) {
            Geometry geometry = (Geometry) subtree;
            Mesh mesh = geometry.getMesh();
            int numVertices = mesh.getVertexCount();
            Vector3f meshLocation = new Vector3f();
            for (int vertexI = 0; vertexI < numVertices; ++vertexI) {
                MyMesh.vertexVector3f(mesh, VertexBuffer.Type.Position, vertexI,
                        meshLocation);
                Vector3f worldLocation
                        = geometry.localToWorld(meshLocation, null);
                result.add(worldLocation);
            }

        } else if (subtree instanceof Node) {
            Node node = (Node) subtree;
            List<Spatial> children = node.getChildren();
            for (Spatial child : children) {
                vertexLocations(child, result);
            }
        }

        return result;
    }
    // *************************************************************************
    // private methods

    /**
     * Add the vertex weights of each bone in the specified mesh to an array of
     * total weights.
     *
     * @param mesh animated mesh to analyze (not null, unaffected)
     * @param totalWeights (not null, modified)
     */
    private static void addWeights(Mesh mesh, float[] totalWeights) {
        assert totalWeights != null;

        int maxWeightsPerVert = mesh.getMaxNumWeights();
        if (maxWeightsPerVert <= 0) {
            maxWeightsPerVert = 1;
        }
        assert maxWeightsPerVert > 0 : maxWeightsPerVert;
        assert maxWeightsPerVert <= 4 : maxWeightsPerVert;

        VertexBuffer biBuf = mesh.getBuffer(VertexBuffer.Type.BoneIndex);
        Buffer boneIndexBuffer = biBuf.getDataReadOnly();
        boneIndexBuffer.rewind();
        int numBoneIndices = boneIndexBuffer.remaining();
        assert numBoneIndices % 4 == 0 : numBoneIndices;
        int numVertices = boneIndexBuffer.remaining() / 4;

        VertexBuffer wBuf = mesh.getBuffer(VertexBuffer.Type.BoneWeight);
        FloatBuffer weightBuffer = (FloatBuffer) wBuf.getDataReadOnly();
        weightBuffer.rewind();
        int numWeights = weightBuffer.remaining();
        assert numWeights == numVertices * 4 : numWeights;

        for (int vIndex = 0; vIndex < numVertices; vIndex++) {
            for (int wIndex = 0; wIndex < 4; wIndex++) {
                float weight = weightBuffer.get();
                int boneIndex = MyMesh.readIndex(boneIndexBuffer);
                if (wIndex < maxWeightsPerVert) {
                    totalWeights[boneIndex] += FastMath.abs(weight);
                }
            }
        }
    }

    /**
     * Calculate the total bone weight animated by each bone in the specified
     * meshes.
     *
     * @param meshes the animated meshes to analyze (not null, unaffected)
     * @param skeleton (not null, unaffected)
     * @return a map from bone indices to total bone weight
     */
    private static float[] totalWeights(Mesh[] meshes, Skeleton skeleton) {
        Validate.nonNull(meshes, "meshes");

        int numBones = skeleton.getBoneCount();
        float[] result = new float[numBones];
        for (Mesh mesh : meshes) {
            RagUtils.addWeights(mesh, result);
        }

        List<Bone> bones = MySkeleton.preOrderBones(skeleton);
        Collections.reverse(bones);
        for (Bone childBone : bones) {
            int childIndex = skeleton.getBoneIndex(childBone);
            Bone parent = childBone.getParent();
            if (parent != null) {
                int parentIndex = skeleton.getBoneIndex(parent);
                result[parentIndex] += result[childIndex];
            }
        }

        return result;
    }

    /**
     * Tabulate the total bone weight associated with each bone/torso link in a
     * ragdoll.
     *
     * @param biArray the array of bone indices (not null, unaffected)
     * @param bwArray the array of bone weights (not null, unaffected)
     * @param managerMap a map from bone indices to managing link names (not
     * null, unaffected)
     * @return a new map from link names to total weight
     */
    private static Map<String, Float> weightMap(int[] biArray,
            float[] bwArray, String[] managerMap) {
        assert biArray.length == 4;
        assert bwArray.length == 4;

        Map<String, Float> weightMap = new HashMap<>(4);
        for (int j = 0; j < 4; j++) {
            int boneIndex = biArray[j];
            if (boneIndex != -1) {
                String managerName = managerMap[boneIndex];
                if (weightMap.containsKey(managerName)) {
                    float oldWeight = weightMap.get(managerName);
                    float newWeight = oldWeight + bwArray[j];
                    weightMap.put(managerName, newWeight);
                } else {
                    weightMap.put(managerName, bwArray[j]);
                }
            }
        }

        return weightMap;
    }
}
