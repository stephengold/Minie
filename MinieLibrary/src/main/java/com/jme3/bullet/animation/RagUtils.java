/*
 * Copyright (c) 2018-2023 jMonkeyEngine
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

import com.jme3.anim.Armature;
import com.jme3.anim.Joint;
import com.jme3.anim.SkinningControl;
import com.jme3.animation.Bone;
import com.jme3.animation.Skeleton;
import com.jme3.animation.SkeletonControl;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.export.InputCapsule;
import com.jme3.export.Savable;
import com.jme3.math.Eigen3f;
import com.jme3.math.FastMath;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.UserData;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.control.AbstractControl;
import java.io.IOException;
import java.nio.Buffer;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Collection;
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
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.RectangularSolid;
import jme3utilities.math.VectorSet;
import jme3utilities.math.VectorSetUsingBuffer;

/**
 * Utility methods used by DynamicAnimControl and associated classes.
 *
 * @author Stephen Gold sgold@sonic.net
 *
 * Based on KinematicRagdollControl by Normen Hansen and Rémy Bouquet (Nehon).
 */
final public class RagUtils {
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
     * Assign each mesh vertex to a bone/torso link and add its location (mesh
     * coordinates in bind pose) to that link's list.
     * <p>
     * A software skin update must precede any request for vertex locations.
     * TODO use the Wes library to avoid this limitation?
     *
     * @param meshes array of animated meshes to use (not null, unaffected)
     * @param managerMap a map from bone indices to managing link names (not
     * null, unaffected)
     * @return a new map from bone/torso names to sets of vertex coordinates
     */
    public static Map<String, VectorSet>
            coordsMap(Mesh[] meshes, String[] managerMap) {
        Validate.nonNull(managerMap, "manager map");

        float[] wArray = new float[4];
        int[] iArray = new int[4];
        Vector3f bindPosition = new Vector3f();
        Map<String, VectorSet> coordsMap = new HashMap<>(32);
        for (Mesh mesh : meshes) {
            int numVertices = mesh.getVertexCount();
            for (int vertexI = 0; vertexI < numVertices; ++vertexI) {
                String managerName = findManager(
                        mesh, vertexI, iArray, wArray, managerMap);
                VectorSet set = coordsMap.get(managerName);
                if (set == null) {
                    set = new VectorSetUsingBuffer(1, false);
                    coordsMap.put(managerName, set);
                }
                MyMesh.vertexVector3f(mesh, VertexBuffer.Type.BindPosePosition,
                        vertexI, bindPosition);
                set.add(bindPosition);
            }
        }

        return coordsMap;
    }

    /**
     * Find the main root bone of a Skeleton, based on its total mesh weight.
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
     * Find the main root joint of an Armature, based on its total mesh weight.
     *
     * @param armature the Armature (not null, unaffected)
     * @param targetMeshes an array of animated meshes to provide weights (not
     * null)
     * @return a root joint, or null if none found
     */
    public static Joint findMainJoint(Armature armature, Mesh[] targetMeshes) {
        Validate.nonNull(targetMeshes, "target meshes");

        Joint[] roots = armature.getRoots();

        Joint result;
        if (roots.length == 1) {
            result = roots[0];
        } else {
            result = null;
            float[] totalWeights = totalWeights(targetMeshes, armature);
            float greatestTotalWeight = Float.NEGATIVE_INFINITY;
            for (Joint root : roots) {
                int jointIndex = root.getId();
                float weight = totalWeights[jointIndex];
                if (weight > greatestTotalWeight) {
                    result = root;
                    greatestTotalWeight = weight;
                }
            }
        }

        return result;
    }

    /**
     * Determine which physics link should manage the specified mesh vertex.
     *
     * @param mesh the mesh containing the vertex (not null, unaffected)
     * @param vertexIndex the vertex index in the mesh (&ge;0)
     * @param iArray temporary storage for bone indices (not null, modified)
     * @param wArray temporary storage for bone weights (not null, modified)
     * @param managerMap a map from bone indices to bone/torso names (not null,
     * unaffected)
     * @return a bone/torso name
     */
    public static String findManager(Mesh mesh, int vertexIndex, int[] iArray,
            float[] wArray, String[] managerMap) {
        Validate.nonNull(mesh, "mesh");
        Validate.nonNegative(vertexIndex, "vertex index");
        Validate.nonNull(iArray, "index array");
        Validate.nonNull(wArray, "weight array");
        Validate.nonNull(managerMap, "manager map");

        MyMesh.vertexBoneIndices(mesh, vertexIndex, iArray);
        MyMesh.vertexBoneWeights(mesh, vertexIndex, wArray);
        Map<String, Float> weightMap = weightMap(iArray, wArray, managerMap);

        float bestTotalWeight = Float.NEGATIVE_INFINITY;
        String bestName = null;
        for (Map.Entry<String, Float> entry : weightMap.entrySet()) {
            float totalWeight = entry.getValue();
            if (totalWeight >= bestTotalWeight) {
                bestTotalWeight = totalWeight;
                bestName = entry.getKey();
            }
        }

        return bestName;
    }

    /**
     * Access the SkeletonControl or SkinningControl in the specified subtree,
     * assuming it doesn't contain more than one.
     *
     * @param subtree a subtree of a scene graph (may be null, unaffected)
     * @return the pre-existing instance, or null if none or multiple
     */
    public static AbstractControl findSControl(Spatial subtree) {
        AbstractControl result = null;
        if (subtree != null) {
            List<SkinningControl> skinners = MySpatial
                    .listControls(subtree, SkinningControl.class, null);
            List<SkeletonControl> skellers = MySpatial
                    .listControls(subtree, SkeletonControl.class, null);
            if (skellers.isEmpty() && skinners.size() == 1) {
                result = skinners.get(0);
            } else if (skinners.isEmpty() && skellers.size() == 1) {
                result = skellers.get(0);
            }
        }

        return result;
    }

    /**
     * Traverse a group of bodies joined by physics joints, adding bodies to the
     * ignore list of the start body. Note: recursive!
     *
     * @param start the body where the traversal began (not null, unaffected)
     * @param current the body to traverse (not null, unaffected)
     * @param hopsRemaining the number of hops remaining (&ge;0)
     * @param visited map bodies visited during the current traversal to max
     * remaining hops (not null, modified)
     */
    static void ignoreCollisions(PhysicsBody start, PhysicsBody current,
            int hopsRemaining, Map<PhysicsBody, Integer> visited) {
        if (hopsRemaining <= 0) {
            return;
        }
        int newRemainingHops = hopsRemaining - 1;

        // Consider each neighboring body that isn't the starting body.
        PhysicsJoint[] joints = current.listJoints();
        for (PhysicsJoint joint : joints) {
            PhysicsBody neighbor = joint.findOtherBody(current);
            if (neighbor != null && neighbor != start) {
                // Decide whether to visit (or re-visit) the neighbor.
                boolean visit = true;
                if (visited.containsKey(neighbor)) { // previously visited
                    int mostRemainingHops = visited.get(neighbor);
                    if (newRemainingHops <= mostRemainingHops) {
                        // don't revisit
                        visit = false;
                    }
                }
                if (visit) {
                    start.addToIgnoreList(neighbor);
                    visited.put(neighbor, newRemainingHops);
                    ignoreCollisions(
                            start, neighbor, newRemainingHops, visited);
                }
            }
        }
    }

    /**
     * Enumerate all animated meshes in the specified subtree of a scene graph,
     * skipping spatials tagged with "JmePhysicsIgnore". Note: recursive!
     *
     * @param subtree the subtree to analyze (may be null, aliases created)
     * @param storeResult storage for results (added to if not null)
     * @return an expanded List (either storeResult or a new List)
     */
    public static List<Mesh>
            listDacMeshes(Spatial subtree, List<Mesh> storeResult) {
        List<Mesh> result = (storeResult == null)
                ? new ArrayList<Mesh>(10) : storeResult;

        if (subtree != null) {
            Boolean ignore = subtree.getUserData(UserData.JME_PHYSICSIGNORE);
            if (ignore != null && ignore) {
                return result;
            }
        }

        if (subtree instanceof Geometry) {
            Geometry geometry = (Geometry) subtree;
            Mesh mesh = geometry.getMesh();
            if (MyMesh.isAnimated(mesh) && !result.contains(mesh)) {
                result.add(mesh);
            }

        } else if (subtree instanceof Node) {
            Node node = (Node) subtree;
            List<Spatial> children = node.getChildren();
            for (Spatial child : children) {
                listDacMeshes(child, result);
            }
        }

        return result;
    }

    /**
     * Enumerate all physics joints that connect bodies in the specified array.
     *
     * @param bodies the array to search (not null, unaffected)
     * @return a new Set of pre-existing instances
     */
    public static Set<PhysicsJoint> listInternalJoints(PhysicsBody... bodies) {
        Set<PhysicsJoint> result = new TreeSet<>();

        for (PhysicsBody body : bodies) {
            PhysicsJoint[] joints = body.listJoints();
            for (PhysicsJoint joint : joints) {
                PhysicsBody otherBody = joint.findOtherBody(body);

                // Is otherBody found in the array?
                for (Object b : bodies) {
                    if (b == otherBody) {
                        result.add(joint);
                        break;
                    }
                }
            }
        }

        return result;
    }

    /**
     * Create a transformed CylinderCollisionShape that bounds the locations in
     * the specified VectorSet.
     *
     * @param vectorSet the set of locations (not null, numVectors&gt;1,
     * unaffected)
     * @param scaleFactors scale factors to apply to local coordinates (not
     * null, unaffected)
     * @return a new shape
     */
    public static CompoundCollisionShape
            makeCylinder(VectorSet vectorSet, Vector3f scaleFactors) {
        Validate.nonNull(scaleFactors, "scale factors");
        int numVectors = vectorSet.numVectors();
        Validate.require(numVectors > 1, "multiple vectors");

        RectangularSolid solid = makeRectangularSolid(vectorSet, scaleFactors);

        Vector3f halfExtents = solid.halfExtents(null); // in local coordinates
        float max = MyMath.max(halfExtents.x, halfExtents.y, halfExtents.z);
        float mid = MyMath.mid(halfExtents.x, halfExtents.y, halfExtents.z);
        float min = MyMath.min(halfExtents.x, halfExtents.y, halfExtents.z);
        float halfHeight;
        if (max - mid > mid - min) { // prolate
            halfHeight = max;
        } else { // oblate
            halfHeight = min;
        }
        Vector3f heightDirection = new Vector3f(); // in local coordinates
        if (halfHeight == halfExtents.x) {
            heightDirection.set(1f, 0f, 0f);
        } else if (halfHeight == halfExtents.y) {
            heightDirection.set(0f, 1f, 0f);
        } else {
            assert halfHeight == halfExtents.z;
            heightDirection.set(0f, 0f, 1f);
        }

        Quaternion localToWorld = solid.localToWorld(null);
        Quaternion worldToLocal = localToWorld.inverse();
        assert worldToLocal != null;

        // Convert heightDirection to world coordinates:
        localToWorld.mult(heightDirection, heightDirection);

        // Calculate minimum half height and squared radius for the cylinder.
        halfHeight = 0f;
        double squaredRadius = 0.;
        FloatBuffer buffer = vectorSet.toBuffer();
        Vector3f worldCenter = vectorSet.mean(null);
        Vector3f tempVector = new Vector3f();
        buffer.rewind();
        while (buffer.hasRemaining()) {
            tempVector.x = buffer.get();
            tempVector.y = buffer.get();
            tempVector.z = buffer.get();
            tempVector.subtractLocal(worldCenter);

            // update halfHeight
            float h = tempVector.dot(heightDirection);
            float absH = FastMath.abs(h);
            if (absH > halfHeight) {
                halfHeight = absH;
            }

            // update squaredRadius
            MyVector3f.accumulateScaled(tempVector, heightDirection, -h);
            double r2 = MyVector3f.lengthSquared(tempVector);
            if (r2 > squaredRadius) {
                squaredRadius = r2;
            }
        }

        // Generate the cylinder shape.
        float height = 2f * halfHeight;
        float radius = (float) Math.sqrt(squaredRadius);
        CylinderCollisionShape cylinder
                = new CylinderCollisionShape(radius, height, MyVector3f.xAxis);

        // Choose an orientation for the cylinder.
        Vector3f yAxis = new Vector3f();
        Vector3f zAxis = new Vector3f();
        MyVector3f.generateBasis(heightDirection, yAxis, zAxis);
        Matrix3f cylinderOrientation = new Matrix3f();
        cylinderOrientation.fromAxes(heightDirection, yAxis, zAxis);

        CompoundCollisionShape result = new CompoundCollisionShape();
        result.addChildShape(cylinder, worldCenter, cylinderOrientation);

        return result;
    }

    /**
     * Instantiate a compact RectangularSolid that bounds the sample locations
     * contained in a VectorSet.
     *
     * @param vectorSet the set of locations (not null, numVectors&gt;1,
     * contents unaffected)
     * @param scaleFactors scale factors to apply to local coordinates (not
     * null, unaffected)
     * @return a new RectangularSolid
     */
    public static RectangularSolid
            makeRectangularSolid(VectorSet vectorSet, Vector3f scaleFactors) {
        Validate.nonNull(scaleFactors, "scale factors");
        int numVectors = vectorSet.numVectors();
        Validate.require(numVectors > 1, "multiple vectors");

        // Orient local axes based on the eigenvectors of the covariance matrix.
        Matrix3f covariance = vectorSet.covariance(null);
        Eigen3f eigen = new Eigen3f(covariance);
        Vector3f[] basis = eigen.getEigenVectors();
        Quaternion localToWorld = new Quaternion().fromAxes(basis);
        Quaternion worldToLocal = localToWorld.inverse();

        // Calculate the min and max for each local axis.
        Vector3f maxima = new Vector3f(Float.NEGATIVE_INFINITY,
                Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY);
        Vector3f minima = new Vector3f(Float.POSITIVE_INFINITY,
                Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY);
        Vector3f tempVector = new Vector3f();
        FloatBuffer buffer = vectorSet.toBuffer();
        buffer.rewind();
        while (buffer.hasRemaining()) {
            tempVector.x = buffer.get();
            tempVector.y = buffer.get();
            tempVector.z = buffer.get();
            worldToLocal.mult(tempVector, tempVector);
            MyVector3f.accumulateMaxima(maxima, tempVector);
            MyVector3f.accumulateMinima(minima, tempVector);
        }

        // Apply scale factors to local coordinates of extrema.
        Vector3f center = MyVector3f.midpoint(minima, maxima, null);

        maxima.subtractLocal(center);
        maxima.multLocal(scaleFactors);
        maxima.addLocal(center);

        minima.subtractLocal(center);
        minima.multLocal(scaleFactors);
        minima.addLocal(center);

        RectangularSolid result
                = new RectangularSolid(minima, maxima, localToWorld);

        return result;
    }

    /**
     * Convert a transform from the mesh coordinate system to the local
     * coordinate system of the specified bone.
     *
     * @param parentBone (not null)
     * @param transform the transform to convert (not null, modified)
     */
    public static void meshToLocal(Bone parentBone, Transform transform) {
        Quaternion msr = parentBone.getModelSpaceRotation();
        Validate.require(msr.norm() > 0f, "non-zero parent rotation");

        Vector3f location = transform.getTranslation(); // alias
        Quaternion orientation = transform.getRotation(); // alias
        Vector3f scale = transform.getScale(); // alias

        Vector3f pmTranslate = parentBone.getModelSpacePosition();
        Quaternion pmRotInv = msr.inverse(); // TODO garbage
        Vector3f pmScale = parentBone.getModelSpaceScale();

        location.subtractLocal(pmTranslate);
        location.divideLocal(pmScale);
        pmRotInv.mult(location, location);
        scale.divideLocal(pmScale);
        pmRotInv.mult(orientation, orientation);
    }

    /**
     * Convert a Transform from the mesh coordinate system to the local
     * coordinate system of the specified armature joint.
     *
     * @param parent (not null)
     * @param transform the transform to convert (not null, modified)
     */
    static void meshToLocal(Joint parent, Transform transform) {
        Transform pm = parent.getModelTransform();
        Validate.require(
                pm.getRotation().norm() > 0f, "non-zero parent rotation");

        Vector3f location = transform.getTranslation(); // alias
        Quaternion orientation = transform.getRotation(); // alias
        Vector3f scale = transform.getScale(); // alias

        Vector3f pmTranslate = pm.getTranslation(); // alias
        Quaternion pmRotInv = pm.getRotation().inverse(); // TODO garbage
        Vector3f pmScale = pm.getScale(); // alias

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
    public static Transform[] readTransformArray(
            InputCapsule capsule, String fieldName) throws IOException {
        Validate.nonNull(capsule, "capsule");
        Validate.nonNull(fieldName, "field name");

        Savable[] tmp = capsule.readSavableArray(fieldName, null);
        Transform[] result;
        if (tmp == null) {
            result = null;
        } else {
            result = new Transform[tmp.length];
            for (int i = 0; i < tmp.length; ++i) {
                result[i] = (Transform) tmp[i];
            }
        }

        return result;
    }

    /**
     * Calculate a coordinate transform for the specified Spatial relative to a
     * specified ancestor node. The result incorporates the transform of the
     * starting Spatial, but not that of the ancestor.
     *
     * @param startSpatial the starting Spatial (not null, unaffected)
     * @param ancestorNode the ancestor node (not null, unaffected)
     * @param storeResult storage for the result (modified if not null)
     * @return a coordinate transform (either storeResult or a new vector, not
     * null)
     */
    public static Transform relativeTransform(
            Spatial startSpatial, Node ancestorNode, Transform storeResult) {
        Validate.nonNull(startSpatial, "spatial");
        Validate.nonNull(ancestorNode, "ancestor");
        assert startSpatial.hasAncestor(ancestorNode);
        Transform result
                = (storeResult == null) ? new Transform() : storeResult;

        result.loadIdentity();
        Spatial loopSpatial = startSpatial;
        while (loopSpatial != ancestorNode) {
            Transform localTransform = loopSpatial.getLocalTransform(); // alias
            result.combineWithParent(localTransform);
            loopSpatial = loopSpatial.getParent();
        }

        return result;
    }

    /**
     * Validate an Armature for use with DynamicAnimControl.
     *
     * @param armature the Armature to validate (not null, unaffected)
     */
    public static void validate(Armature armature) {
        int numJoints = armature.getJointCount();
        if (numJoints < 0) {
            throw new IllegalArgumentException("Joint count is negative!");
        }

        Collection<String> nameSet = new TreeSet<>();
        for (int jointIndex = 0; jointIndex < numJoints; ++jointIndex) {
            Joint joint = armature.getJoint(jointIndex);
            if (joint == null) {
                String msg = String
                        .format("Joint %d in armature is null!", jointIndex);
                throw new IllegalArgumentException(msg);
            }
            String jointName = joint.getName();
            if (jointName == null) {
                String message = String.format(
                        "Joint %d in armature has null name!", jointIndex);
                throw new IllegalArgumentException(message);
            } else if (jointName.equals(DacConfiguration.torsoName)) {
                String message = String.format(
                        "Joint %d in armature has a reserved name!",
                        jointIndex);
                throw new IllegalArgumentException(message);
            } else if (nameSet.contains(jointName)) {
                String message
                        = "Duplicate joint name in skeleton: " + jointName;
                throw new IllegalArgumentException(message);
            }
            nameSet.add(jointName);
        }
    }

    /**
     * Validate a Skeleton for use with DynamicAnimControl.
     *
     * @param skeleton the Skeleton to validate (not null, unaffected)
     */
    public static void validate(Skeleton skeleton) {
        int numBones = skeleton.getBoneCount();
        if (numBones < 0) {
            throw new IllegalArgumentException("Bone count is negative!");
        }

        Collection<String> nameSet = new TreeSet<>();
        for (int boneIndex = 0; boneIndex < numBones; ++boneIndex) {
            Bone bone = skeleton.getBone(boneIndex);
            if (bone == null) {
                String msg = String
                        .format("Bone %d in skeleton is null!", boneIndex);
                throw new IllegalArgumentException(msg);
            }
            String boneName = bone.getName();
            if (boneName == null) {
                String msg = String.format(
                        "Bone %d in skeleton has null name!", boneIndex);
                throw new IllegalArgumentException(msg);
            } else if (boneName.equals(DacConfiguration.torsoName)) {
                String msg = String.format(
                        "Bone %d in skeleton has a reserved name!", boneIndex);
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
     * @throws IllegalArgumentException for an invalid model
     */
    public static void validate(Spatial model) {
        Validate.nonNull(model, "model");

        List<Geometry> geometries = MySpatial.listGeometries(model);
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

        FloatBuffer weightBuffer
                = mesh.getFloatBuffer(VertexBuffer.Type.BoneWeight);
        weightBuffer.rewind();
        int numWeights = weightBuffer.remaining();
        assert numWeights == numVertices * 4 : numWeights;

        for (int vIndex = 0; vIndex < numVertices; ++vIndex) {
            for (int wIndex = 0; wIndex < 4; ++wIndex) {
                float weight = weightBuffer.get();
                int boneIndex = MyBuffer.readIndex(boneIndexBuffer);
                if (wIndex < maxWeightsPerVert) {
                    totalWeights[boneIndex] += FastMath.abs(weight);
                }
            }
        }
    }

    /**
     * Calculate the total mesh weight animated by each Joint in the specified
     * meshes.
     *
     * @param meshes the animated meshes to analyze (not null, unaffected)
     * @param armature (not null, unaffected)
     * @return a map from joint indices to total mesh weight
     */
    private static float[] totalWeights(Mesh[] meshes, Armature armature) {
        Validate.nonNull(meshes, "meshes");

        int numBones = armature.getJointCount();
        float[] result = new float[numBones];
        for (Mesh mesh : meshes) {
            addWeights(mesh, result);
        }

        List<Joint> joints = MySkeleton.preOrderJoints(armature);
        Collections.reverse(joints);
        for (Joint childJoint : joints) {
            int childIndex = childJoint.getId();
            Joint parent = childJoint.getParent();
            if (parent != null) {
                int parentIndex = parent.getId();
                result[parentIndex] += result[childIndex];
            }
        }

        return result;
    }

    /**
     * Calculate the total mesh weight animated by each Bone in the specified
     * meshes.
     *
     * @param meshes the animated meshes to analyze (not null, unaffected)
     * @param skeleton (not null, unaffected)
     * @return a map from bone indices to total mesh weight
     */
    private static float[] totalWeights(Mesh[] meshes, Skeleton skeleton) {
        Validate.nonNull(meshes, "meshes");

        int numBones = skeleton.getBoneCount();
        float[] result = new float[numBones];
        for (Mesh mesh : meshes) {
            addWeights(mesh, result);
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
    private static Map<String, Float>
            weightMap(int[] biArray, float[] bwArray, String[] managerMap) {
        assert biArray.length == 4;
        assert bwArray.length == 4;

        Map<String, Float> weightMap = new HashMap<>(4);
        for (int j = 0; j < 4; ++j) {
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
