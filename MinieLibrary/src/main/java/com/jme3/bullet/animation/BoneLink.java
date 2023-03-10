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

import com.jme3.anim.Joint;
import com.jme3.animation.Bone;
import com.jme3.animation.Skeleton;
import com.jme3.bullet.RotationOrder;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.joints.Constraint;
import com.jme3.bullet.joints.New6Dof;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.joints.SixDofJoint;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Spatial;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.MySkeleton;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyQuaternion;

/**
 * Link an animated bone in an Armature/Skeleton to a jointed rigid body in a
 * ragdoll.
 *
 * @author Stephen Gold sgold@sonic.net
 *
 * Based on KinematicRagdollControl by Normen Hansen and Rémy Bouquet (Nehon).
 */
public class BoneLink extends PhysicsLink {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(BoneLink.class.getName());
    /**
     * local copy of {@link com.jme3.math.Matrix3f#IDENTITY}
     */
    final private static Matrix3f matrixIdentity = new Matrix3f();
    /**
     * field names for serialization
     */
    final private static String tagEndBoneTransforms = "endBoneTransforms";
    final private static String tagManagedArmatureJoints
            = "managedArmatureJoints";
    final private static String tagManagedBones = "managedBones";
    final private static String tagPrevBoneTransforms = "prevBoneTransforms";
    final private static String tagStartBoneTransforms = "startBoneTransforms";
    final private static String tagSubmode = "submode";
    /**
     * local copy of {@link com.jme3.math.Vector3f#ZERO}
     */
    final private static Vector3f translateIdentity = new Vector3f(0f, 0f, 0f);
    // *************************************************************************
    // fields

    /**
     * skeleton bones managed by this link, in a pre-order, depth-first
     * traversal of the Skeleton, starting with the linked bone, or null for an
     * Armature
     */
    private Bone[] managedBones = null;
    /**
     * armature joints managed by this link, in a pre-order, depth-first
     * traversal of the Armature, starting with the linked bone, or null for a
     * Skeleton
     */
    private Joint[] managedArmatureJoints = null;
    /**
     * submode when kinematic
     */
    private KinematicSubmode submode = KinematicSubmode.Animated;
    /**
     * reusable temporary storage for a 3x3 matrix
     */
    private Matrix3f tmpMatrix = new Matrix3f();
    /**
     * local transform for each managed bone at the end of a blend to
     * {@code Reset} submode, or null if not specified
     */
    private Transform[] endBoneTransforms = null;
    /**
     * local transform of each managed bone from the previous update
     */
    private Transform[] prevBoneTransforms = null;
    /**
     * local transform of each managed bone at the start of the most recent
     * blend interval
     */
    private Transform[] startBoneTransforms = null;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected BoneLink() {
        // do nothing
    }

    /**
     * Instantiate a purely kinematic link between the specified skeleton bone
     * and the specified rigid body.
     *
     * @param control the Control that will manage this link (not null, alias
     * created)
     * @param bone the linked skeleton bone (not null, alias created)
     * @param collisionShape the desired shape (not null, alias created)
     * @param linkConfig the link configuration (not null)
     * @param localOffset the location of the body's center (in the bone's local
     * coordinates, not null, unaffected)
     */
    BoneLink(DacLinks control, Bone bone, CollisionShape collisionShape,
            LinkConfig linkConfig, Vector3f localOffset) {
        super(control, bone, collisionShape, linkConfig, localOffset);
    }

    /**
     * Instantiate a purely kinematic link between the specified armature joint
     * and the specified rigid body.
     *
     * @param control the Control that will manage this link (not null, alias
     * created)
     * @param joint the linked armature joint (not null, alias created)
     * @param collisionShape the desired shape (not null, alias created)
     * @param linkConfig the link configuration (not null)
     * @param localOffset the location of the body's center (in the joint's
     * local coordinates, not null, unaffected)
     */
    BoneLink(DacLinks control, Joint joint, CollisionShape collisionShape,
            LinkConfig linkConfig, Vector3f localOffset) {
        super(control, joint, collisionShape, linkConfig, localOffset);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Add a PhysicsJoint to this link and configure its range of motion. Also
     * initialize the link's parent and its array of managed bones (or armature
     * joints).
     * <p>
     * The new joint will be either a SixDofJoint or a New6Dof, depending on how
     * the bone is configured. Its "A" end will be the parent's rigid body, and
     * its "B" end will be this link's rigid body.
     *
     * @param parentLink (not null, alias created)
     */
    void addJoint(PhysicsLink parentLink) {
        assert parentLink != null;
        assert getJoint() == null;

        setParent(parentLink);

        Transform parentToWorld = parentLink.physicsTransform(null);
        parentToWorld.setScale(1f);
        Transform worldToParent = parentToWorld.invert();

        Transform childToWorld = physicsTransform(null);
        childToWorld.setScale(1f);

        Transform childToParent = childToWorld.clone();
        childToParent.combineWithParent(worldToParent);

        Spatial transformer = getControl().getTransformer();
        Vector3f pivotMesh;
        Bone bone = getBone();
        if (bone != null) { // old animation system
            pivotMesh = bone.getModelSpacePosition();
        } else { // new animation system
            Joint armatureJoint = getArmatureJoint();
            Transform t = armatureJoint.getModelTransform();
            pivotMesh = t.getTranslation(); // alias
        }
        Vector3f pivotWorld = transformer.localToWorld(pivotMesh, null);

        PhysicsRigidBody parentBody = parentLink.getRigidBody();
        PhysicsRigidBody childBody = getRigidBody();
        Vector3f pivotParent
                = parentToWorld.transformInverseVector(pivotWorld, null);
        Vector3f pivotChild
                = childToWorld.transformInverseVector(pivotWorld, null);
        childToParent.getRotation().toRotationMatrix(tmpMatrix);
        Matrix3f rotParent = tmpMatrix; // alias
        Matrix3f rotChild = matrixIdentity; // alias

        Constraint constraint;
        String name = boneName();
        RotationOrder rotationOrder = getControl().config(name).rotationOrder();
        if (rotationOrder == null) {
            // TODO try HingeJoint or ConeJoint
            constraint = new SixDofJoint(parentBody, childBody, pivotParent,
                    pivotChild, rotParent, rotChild, true);
        } else {
            constraint = new New6Dof(parentBody, childBody, pivotParent,
                    pivotChild, rotParent, rotChild, rotationOrder);
        }
        setJoint(constraint);

        RangeOfMotion rangeOfMotion = getControl().getJointLimits(name);
        rangeOfMotion.setup(constraint, false, false, false);

        assert managedBones == null;
        assert managedArmatureJoints == null;
        int numManaged;
        if (bone != null) { // old animation system
            this.managedBones = getControl().listManagedBones(name);
            numManaged = managedBones.length;
        } else { // new animation system
            this.managedArmatureJoints
                    = getControl().listManagedArmatureJoints(name);
            numManaged = managedArmatureJoints.length;
        }

        this.startBoneTransforms = new Transform[numManaged];
        for (int managedIndex = 0; managedIndex < numManaged; ++managedIndex) {
            this.startBoneTransforms[managedIndex] = new Transform();
        }
    }

    /**
     * Begin blending this link to a purely kinematic mode.
     *
     * @param submode enum value (not null)
     * @param blendInterval the duration of the blend interval (in seconds,
     * &ge;0)
     */
    public void blendToKinematicMode(
            KinematicSubmode submode, float blendInterval) {
        Validate.nonNull(submode, "submode");
        Validate.nonNegative(blendInterval, "blend interval");

        blendToKinematicMode(blendInterval);
        this.submode = submode;

        // Save initial bone transforms for blending.
        int numManaged = countManaged();
        for (int managedIndex = 0; managedIndex < numManaged; ++managedIndex) {
            Transform transform;
            if (prevBoneTransforms == null) { // this link not updated yet
                transform = copyManagedTransform(managedIndex, null);
            } else {
                transform = prevBoneTransforms[managedIndex];
            }
            startBoneTransforms[managedIndex].set(transform);
        }

        // Take or release control of the managed bones.
        if (submode == KinematicSubmode.Animated) {
            setUserControl(false);
        } else {
            setUserControl(true);
        }
    }

    /**
     * Determine the index in the Armature/Skeleton of the indexed managed bone.
     *
     * @param managedIndex which managed bone (0 = the linked bone, &ge;0,
     * &lt;numManaged)
     * @return the index in the Armature or Skeleton (&ge;0)
     */
    public int boneIndex(int managedIndex) {
        int numManaged = countManaged();
        Validate.inRange(managedIndex, "managed index", 0, numManaged - 1);

        int result;
        if (managedBones != null) { // old animation system
            Bone managed = managedBones[managedIndex];
            Skeleton skeleton = getControl().getSkeleton();
            result = skeleton.getBoneIndex(managed);
        } else { // new animation system
            Joint managed = managedArmatureJoints[managedIndex];
            result = managed.getId();
        }

        assert result >= 0 : result;
        return result;
    }

    /**
     * Determine the number of managed skeleton bones or armature joints.
     *
     * @return the count (&ge;1)
     */
    public int countManaged() {
        int result;
        if (managedBones != null) {
            result = managedBones.length;
        } else {
            result = managedArmatureJoints.length;
        }

        assert result >= 1 : result;
        return result;
    }

    /**
     * Estimate the footprint of this link.
     *
     * @return the corner locations of a rectangle (in world coordinates)
     */
    public Vector3f[] footprint() {
        CollisionShape shape = getRigidBody().getCollisionShape();
        assert shape.isConvex();

        Transform localToWorld = physicsTransform(null);
        localToWorld.setScale(1f);

        Vector3f[] result = DebugShapeFactory.footprint(
                shape, localToWorld, DebugShapeFactory.lowResolution);
        return result;
    }

    /**
     * Copy animation data from the specified link, which must have the same
     * name and the same managed bones.
     *
     * @param oldLink the link to copy from (not null, unaffected)
     */
    void postRebuild(BoneLink oldLink) {
        int numManaged = countManaged();
        assert oldLink.countManaged() == numManaged;

        postRebuildLink(oldLink);
        if (oldLink.isKinematic()) {
            this.submode = oldLink.submode;
        } else {
            this.submode = KinematicSubmode.Frozen;
        }

        if (prevBoneTransforms == null) {
            this.prevBoneTransforms = new Transform[numManaged];
            for (int managedI = 0; managedI < numManaged; ++managedI) {
                prevBoneTransforms[managedI] = new Transform();
            }
        }
        for (int managedIndex = 0; managedIndex < numManaged; ++managedIndex) {
            Transform transform = oldLink.prevBoneTransforms[managedIndex];
            prevBoneTransforms[managedIndex].set(transform);

            transform = oldLink.startBoneTransforms[managedIndex];
            startBoneTransforms[managedIndex].set(transform);
        }
    }

    /**
     * Immediately put this link into dynamic mode and update the range of
     * motion of its joint.
     *
     * @param uniformAcceleration the uniform acceleration vector (in
     * physics-space coordinates, not null, unaffected)
     * @param lockX true to lock the joint's X-axis
     * @param lockY true to lock the joint's Y-axis
     * @param lockZ true to lock the joint's Z-axis
     */
    public void setDynamic(Vector3f uniformAcceleration, boolean lockX,
            boolean lockY, boolean lockZ) {
        Validate.finite(uniformAcceleration, "uniform acceleration");
        String desiredAction = "put " + name() + " into dynamic mode";
        getControl().verifyReadyForDynamicMode(desiredAction);

        super.setDynamic(uniformAcceleration);

        String name = boneName();
        RangeOfMotion preset = getControl().getJointLimits(name);
        preset.setup(getJoint(), lockX, lockY, lockZ);
        setUserControl(true);
    }

    /**
     * Immediately put this link into dynamic mode and lock its PhysicsJoint at
     * the specified rotation.
     * <p>
     * The control must be "ready" for dynamic mode.
     *
     * @param uniformAcceleration the uniform acceleration vector (in
     * physics-space coordinates, not null, unaffected)
     * @param userRotation the desired rotation relative to the bind rotation of
     * the linked bone (not null, unaffected)
     */
    public void setDynamic(
            Vector3f uniformAcceleration, Quaternion userRotation) {
        Validate.finite(uniformAcceleration, "uniform acceleration");
        String desiredAction = "put " + name() + " into dynamic mode";
        getControl().verifyReadyForDynamicMode(desiredAction);

        super.setDynamic(uniformAcceleration);
        PhysicsJoint joint = getJoint();

        RotationOrder rotOrder;
        if (joint instanceof SixDofJoint) {
            rotOrder = RotationOrder.XYZ;
        } else {
            rotOrder = ((New6Dof) joint).getRotationOrder();
        }

        userRotation.toRotationMatrix(tmpMatrix);
        Vector3f eulerAngles
                = rotOrder.matrixToEuler(tmpMatrix, null); // garbage
        RangeOfMotion rom = new RangeOfMotion(eulerAngles);
        rom.setup(joint, false, false, false);

        setUserControl(true);
    }

    /**
     * Alter the local transform for each managed bone at the end of a blend to
     * {@code Reset} submode.
     *
     * @param transforms (not null, one element for each managed bone, no null
     * elements, alias created)
     */
    public void setEndBoneTransforms(Transform[] transforms) {
        Validate.nonNull(transforms, "transforms");
        int numManaged = countManaged();
        Validate.require(transforms.length == numManaged,
                "one element for each managed bone");

        this.endBoneTransforms = transforms;
    }

    /**
     * Alter the local transform of the indexed managed bone. Use this method to
     * animate managed bones other than the linked one. Effective only once the
     * link has been updated.
     *
     * @param mbIndex the index of the managed bone (&ge;1, &lt;numManaged)
     * @param localTransform the desired Transform (not null, unaffected)
     */
    public void setLocalTransform(int mbIndex, Transform localTransform) {
        int numManaged = countManaged();
        Validate.inRange(mbIndex, "index", 1, numManaged - 1);

        if (prevBoneTransforms != null) {
            prevBoneTransforms[mbIndex].set(localTransform);
        }
    }
    // *************************************************************************
    // PhysicsLink methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned link into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this link (not null)
     * @param original the instance from which this link was shallow-cloned (not
     * null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);

        this.managedBones = cloner.clone(managedBones);
        this.managedArmatureJoints = cloner.clone(managedArmatureJoints);
        this.tmpMatrix = cloner.clone(tmpMatrix);
        this.endBoneTransforms = cloner.clone(endBoneTransforms);
        this.prevBoneTransforms = cloner.clone(prevBoneTransforms);
        this.startBoneTransforms = cloner.clone(startBoneTransforms);
    }

    /**
     * Update this link in Dynamic mode, setting the linked bone's transform
     * based on the transform of the rigid body.
     */
    @Override
    protected void dynamicUpdate() {
        assert !getRigidBody().isKinematic();

        // Disable bone animations, if any.
        int numManaged = countManaged();
        for (int managedIndex = 1; managedIndex < numManaged; ++managedIndex) {
            Transform t = prevBoneTransforms[managedIndex]; // alias
            setManagedTransform(managedIndex, t);
        }

        // Override the local transform of the linked bone and update.
        Transform transform = localBoneTransform(null); // TODO garbage
        if (managedBones != null) {
            MySkeleton.setLocalTransform(getBone(), transform);
            for (Bone managed : managedBones) {
                managed.updateModelTransforms();
            }
        } else {
            getArmatureJoint().setLocalTransform(transform);
            for (Joint managed : managedArmatureJoints) {
                managed.updateModelTransforms();
            }
        }
    }

    /**
     * Immediately freeze this link.
     *
     * @param forceKinematic true&rarr;force to kinematic mode,
     * false&rarr;preserve mode
     */
    @Override
    public void freeze(boolean forceKinematic) {
        if (forceKinematic || isKinematic()) {
            blendToKinematicMode(KinematicSubmode.Frozen, 0f);
        } else {
            setDynamic(translateIdentity, true, true, true);
        }
    }

    /**
     * Update this link in blended Kinematic mode.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    protected void kinematicUpdate(float tpf) {
        assert tpf >= 0f : tpf;
        assert getRigidBody().isKinematic();

        Transform transform = new Transform();
        int numManaged = countManaged();
        for (int managedIndex = 0; managedIndex < numManaged; ++managedIndex) {
            switch (submode) {
                case Amputated:
                    int boneIndex = boneIndex(managedIndex);
                    getControl().copyBindTransform(boneIndex, transform);
                    transform.setScale(0.001f);
                    break;
                case Animated:
                    copyManagedTransform(managedIndex, transform);
                    break;
                case Bound:
                    boneIndex = boneIndex(managedIndex);
                    getControl().copyBindTransform(boneIndex, transform);
                    break;
                case Frozen:
                    transform.set(prevBoneTransforms[managedIndex]);
                    break;
                case Reset:
                    transform.set(endBoneTransforms[managedIndex]);
                    break;
                default:
                    throw new IllegalStateException(submode.toString());
            }

            if (kinematicWeight() < 1f) { // not purely kinematic yet
                /*
                 * For a smooth transition, blend the saved bone transform
                 * (from the start of the blend interval)
                 * into the goal transform.
                 */
                Transform start = startBoneTransforms[managedIndex]; // alias
                Quaternion startQuat = start.getRotation(); // alias
                MyQuaternion.normalizeLocal(startQuat);
                Quaternion endQuat = transform.getRotation(); // alias
                if (startQuat.dot(endQuat) < 0f) {
                    endQuat.multLocal(-1f);
                } // TODO smarter sign flipping
                MyQuaternion.normalizeLocal(endQuat);
                MyMath.slerp(kinematicWeight(), start, transform, transform);
            }

            // Update the managed bone.
            setManagedTransform(managedIndex, transform);
        }

        super.kinematicUpdate(tpf);
    }

    /**
     * Unambiguously identify this link by name, within its DynamicAnimControl.
     *
     * @return a brief textual description (not null, not empty)
     */
    @Override
    public String name() {
        String result = "Bone:" + boneName();
        return result;
    }

    /**
     * De-serialize this link from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);
        InputCapsule capsule = importer.getCapsule(this);

        Savable[] tmp
                = capsule.readSavableArray(tagManagedArmatureJoints, null);
        if (tmp != null) {
            this.managedArmatureJoints = new Joint[tmp.length];
            for (int managedI = 0; managedI < tmp.length; ++managedI) {
                this.managedArmatureJoints[managedI] = (Joint) tmp[managedI];
            }
        }

        tmp = capsule.readSavableArray(tagManagedBones, null);
        if (tmp != null) {
            this.managedBones = new Bone[tmp.length];
            for (int managedI = 0; managedI < tmp.length; ++managedI) {
                this.managedBones[managedI] = (Bone) tmp[managedI];
            }
        }

        this.submode = capsule.readEnum(
                tagSubmode, KinematicSubmode.class, KinematicSubmode.Animated);
        this.endBoneTransforms
                = RagUtils.readTransformArray(capsule, tagEndBoneTransforms);
        this.prevBoneTransforms
                = RagUtils.readTransformArray(capsule, tagPrevBoneTransforms);
        this.startBoneTransforms
                = RagUtils.readTransformArray(capsule, tagStartBoneTransforms);
    }

    /**
     * Immediately put this link into dynamic mode. The control must be "ready".
     *
     * @param uniformAcceleration the uniform acceleration vector to apply (in
     * physics-space coordinates, not null, unaffected)
     */
    @Override
    public void setDynamic(Vector3f uniformAcceleration) {
        Validate.finite(uniformAcceleration, "uniform acceleration");
        String desiredAction = "put " + name() + " into dynamic mode";
        getControl().verifyReadyForDynamicMode(desiredAction);

        super.setDynamic(uniformAcceleration);
        setUserControl(true);
    }

    /**
     * Immediately put this link into ragdoll mode. The control must be ready
     * for dynamic mode.
     */
    @Override
    public void setRagdollMode() {
        String desiredAction = "put " + name() + " into ragdoll mode";
        getControl().verifyReadyForDynamicMode(desiredAction);

        Vector3f gravity = getControl().gravity(null);
        setDynamic(gravity, false, false, false);

        super.setRagdollMode();
    }

    /**
     * Internal callback, invoked once per frame during the logical-state
     * update, provided the control is added to a scene.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    void update(float tpf) {
        assert tpf >= 0f : tpf;

        int numManaged = countManaged();
        if (prevBoneTransforms == null) {
            /*
             * On the first update, allocate and initialize
             * the array of previous bone transforms, if it wasn't
             * allocated in blendToKinematicMode().
             */
            this.prevBoneTransforms = new Transform[numManaged];
            for (int managedI = 0; managedI < numManaged; ++managedI) {
                Transform boneTransform = copyManagedTransform(managedI, null);
                this.prevBoneTransforms[managedI] = boneTransform;
            }
        }

        super.update(tpf);

        // Save copies of the latest managed-bone transforms.
        for (int managedIndex = 0; managedIndex < numManaged; ++managedIndex) {
            Transform lastTransform = prevBoneTransforms[managedIndex]; // alias
            copyManagedTransform(managedIndex, lastTransform);
        }
    }

    /**
     * Serialize this link to the specified exporter, for example when saving to
     * a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        super.write(exporter);
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(managedArmatureJoints, tagManagedArmatureJoints, null);
        capsule.write(managedBones, tagManagedBones, null);
        capsule.write(submode, tagSubmode, KinematicSubmode.Animated);
        // tmpMatrix is never written.
        capsule.write(endBoneTransforms, tagEndBoneTransforms, null);
        capsule.write(
                prevBoneTransforms, tagPrevBoneTransforms, new Transform[0]);
        capsule.write(
                startBoneTransforms, tagStartBoneTransforms, new Transform[0]);
    }
    // *************************************************************************
    // private methods

    /**
     * Copy the local transform of the indexed managed bone in this link.
     *
     * @param managedIndex which managed bone (&ge;0, &lt;numManaged)
     * @param storeResult storage for the result (modified if not null)
     * @return the Transform (either storeResult or a new instance, not null)
     */
    private Transform copyManagedTransform(
            int managedIndex, Transform storeResult) {
        Transform result
                = (storeResult == null) ? new Transform() : storeResult;

        if (managedBones != null) { // old animation system
            Bone managed = managedBones[managedIndex];
            MySkeleton.copyLocalTransform(managed, result);
        } else { // new animation system
            Joint managed = managedArmatureJoints[managedIndex];
            Transform local = managed.getLocalTransform(); // alias
            result.set(local);
        }

        return result;
    }

    /**
     * Calculate the local bone transform to match the physics transform of the
     * rigid body.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the calculated bone transform (in local coordinates, either
     * storeResult or a new transform, not null)
     */
    private Transform localBoneTransform(Transform storeResult) {
        Transform result
                = (storeResult == null) ? new Transform() : storeResult;
        Vector3f location = result.getTranslation(); // alias
        Quaternion orientation = result.getRotation(); // alias
        Vector3f scale = result.getScale(); // alias

        // Start with the rigid body's transform in physics/world coordinates.
        getRigidBody().getTransform(result);

        // Convert to mesh coordinates.
        Transform worldToMesh = getControl().meshTransform(null).invert();
        result.combineWithParent(worldToMesh);
        /*
         * Convert to the bone's local coordinate system by factoring out the
         * parent bone's transform.
         */
        if (managedBones != null) { // old animation system
            Bone parent = getBone().getParent();
            RagUtils.meshToLocal(parent, result);
        } else { // new animation system
            Joint parent = getArmatureJoint().getParent();
            RagUtils.meshToLocal(parent, result);
        }

        // Subtract the body's local offset, rotated and scaled.
        Vector3f parentOffset = localOffset(null);
        parentOffset.multLocal(scale);
        orientation.mult(parentOffset, parentOffset);
        location.subtractLocal(parentOffset);

        return result;
    }

    /**
     * Alter the local transform of the indexed managed bone in this link.
     *
     * @param managedIndex which managed bone (&ge;0, &lt;numManaged)
     * @param transform the desired Transform (not null, unaffected)
     */
    private void setManagedTransform(int managedIndex, Transform transform) {
        if (managedBones != null) { // old animation system
            Bone managed = managedBones[managedIndex];
            MySkeleton.setLocalTransform(managed, transform);
            managed.updateModelTransforms();
        } else { // new animation system
            Joint managed = managedArmatureJoints[managedIndex];
            managed.setLocalTransform(transform);
            managed.updateModelTransforms();
        }
    }

    /**
     * Alter the user-control flags of all skeleton bones managed by this link.
     *
     * @param wantUserControl the desired setting
     */
    private void setUserControl(boolean wantUserControl) {
        if (managedBones != null) { // old animation system
            for (Bone managed : managedBones) {
                managed.setUserControl(wantUserControl);
            }
        }
    }
}
