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
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MySkeleton;
import jme3utilities.MySpatial;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyQuaternion;

/**
 * Link the torso of an animated model to a rigid body in a ragdoll.
 *
 * @author Stephen Gold sgold@sonic.net
 *
 * Based on KinematicRagdollControl by Normen Hansen and Rémy Bouquet (Nehon).
 */
public class TorsoLink extends PhysicsLink {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(TorsoLink.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagEndBoneTransforms = "endBoneTransforms";
    final private static String tagEndModelTransform = "endModelTransform";
    final private static String tagManagedArmatureJoints
            = "managedArmatureJoints";
    final private static String tagManagedBones = "managedBones";
    final private static String tagMeshToModel = "meshToModel";
    final private static String tagPrevBoneTransforms = "prevBoneTransforms";
    final private static String tagStartBoneTransforms = "startBoneTransforms";
    final private static String tagStartModelTransform = "startModelTransform";
    final private static String tagSubmode = "submode";
    // *************************************************************************
    // fields

    /**
     * skeleton bones managed by this link, in a pre-order, depth-first
     * traversal of the Skeleton, or null for an Armature
     */
    private Bone[] managedBones = null;
    /**
     * armature joints managed by this link, in a pre-order, depth-first
     * traversal of the Armature, or null for a Skeleton
     */
    private Joint[] managedArmatureJoints = null;
    /**
     * submode when kinematic
     */
    private KinematicSubmode submode = KinematicSubmode.Animated;
    /**
     * local transform for the controlled spatial at the end of this link's most
     * recent blend interval, or null for no spatial blending
     */
    private Transform endModelTransform = null;
    /**
     * transform from mesh coordinates to model coordinates
     */
    private Transform meshToModel = null;
    /**
     * local transform of the controlled spatial at the start of this link's
     * most recent blend interval
     */
    private Transform startModelTransform = new Transform();
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
    protected TorsoLink() {
        // do nothing
    }

    /**
     * Instantiate a purely kinematic link between the torso of the specified
     * Control and the specified rigid body.
     *
     * @param control the Control that will manage this link (not null, alias
     * created)
     * @param mainRootBone the main bone, or if not configured then then root
     * bone with the most animation weight (not null, alias created)
     * @param collisionShape the desired shape (not null, alias created)
     * @param linkConfig the link configuration (not null)
     * @param meshToModel the transform from mesh coordinates to model
     * coordinates (not null, unaffected)
     * @param localOffset the location of the body's center (in the bone's local
     * coordinates, not null, unaffected)
     */
    TorsoLink(DacLinks control, Bone mainRootBone,
            CollisionShape collisionShape, LinkConfig linkConfig,
            Transform meshToModel, Vector3f localOffset) {
        super(control, mainRootBone, collisionShape, linkConfig, localOffset);
        this.meshToModel = meshToModel.clone();
        this.managedBones
                = control.listManagedBones(DacConfiguration.torsoName);

        int numManaged = countManaged();
        this.startBoneTransforms = new Transform[numManaged];
        for (int i = 0; i < numManaged; ++i) {
            this.startBoneTransforms[i] = new Transform();
        }
    }

    /**
     * Instantiate a purely kinematic link between the torso of the specified
     * Control and the specified rigid body.
     *
     * @param control the Control that will manage this link (not null, alias
     * created)
     * @param mainRootJoint the main armature joint, or if not configured then
     * the root armature joint with the most animation weight (not null, alias
     * created)
     * @param collisionShape the desired shape (not null, alias created)
     * @param linkConfig the link configuration (not null)
     * @param meshToModel the transform from mesh coordinates to model
     * coordinates (not null, unaffected)
     * @param localOffset the location of the body's center (in the joint's
     * local coordinates, not null, unaffected)
     */
    TorsoLink(DacLinks control, Joint mainRootJoint,
            CollisionShape collisionShape, LinkConfig linkConfig,
            Transform meshToModel, Vector3f localOffset) {
        super(control, mainRootJoint, collisionShape, linkConfig, localOffset);
        this.meshToModel = meshToModel.clone();
        this.managedArmatureJoints
                = control.listManagedArmatureJoints(DacConfiguration.torsoName);

        int numManagedJoints = managedArmatureJoints.length;
        this.startBoneTransforms = new Transform[numManagedJoints];
        for (int i = 0; i < numManagedJoints; ++i) {
            this.startBoneTransforms[i] = new Transform();
        }
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Begin blending this link to a purely kinematic mode.
     *
     * @param submode enum value (not null)
     * @param blendInterval the duration of the blend interval (in seconds,
     * &ge;0)
     * @param endModelTransform the desired local transform for the controlled
     * spatial when the blend completes (alias created) or null for no change to
     * the local transform
     */
    public void blendToKinematicMode(KinematicSubmode submode,
            float blendInterval, Transform endModelTransform) {
        Validate.nonNull(submode, "submode");
        Validate.nonNegative(blendInterval, "blend interval");

        blendToKinematicMode(blendInterval);

        this.submode = submode;
        this.endModelTransform = endModelTransform;

        // Save initial transforms for blending.
        if (endModelTransform != null) {
            Transform current
                    = getControl().getSpatial().getLocalTransform(); // alias
            startModelTransform.set(current);
        }
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
    final public int countManaged() {
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
     * Alter the local transform for each managed bone at the end of a blend
     * interval, for use with the {@code Reset} kinematic submode.
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
     * animate managed bones. Effective only once the link has been updated.
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
        this.endModelTransform = cloner.clone(endModelTransform);
        this.meshToModel = cloner.clone(meshToModel);
        this.endBoneTransforms = cloner.clone(endBoneTransforms);
        this.prevBoneTransforms = cloner.clone(prevBoneTransforms);
        this.startBoneTransforms = cloner.clone(startBoneTransforms);
        this.startModelTransform = cloner.clone(startModelTransform);
    }

    /**
     * Update this link in Dynamic mode, setting the local transform of the
     * model's root spatial based on the transform of the linked rigid body.
     */
    @Override
    protected void dynamicUpdate() {
        assert !getRigidBody().isKinematic();

        DacLinks control = getControl();
        Spatial spatial = control.getSpatial();
        Node parent = spatial.getParent();

        // Calculate the inverse world transform of the model's parent node.
        Transform worldToParent;
        if (parent == null) {
            worldToParent = new Transform();
        } else {
            Transform parentToWorld = MySpatial.worldTransform(parent, null);
            worldToParent = parentToWorld.invert();
        }

        Transform transform = meshToModel.clone(); // TODO garbage
        Transform shapeToWorld = getRigidBody().getTransform(null);
        transform.combineWithParent(shapeToWorld);
        transform.combineWithParent(worldToParent);
        spatial.setLocalTransform(transform);

        // Disable bone animations, if any.
        int numManaged = countManaged();
        for (int managedIndex = 0; managedIndex < numManaged; ++managedIndex) {
            Transform t = prevBoneTransforms[managedIndex]; // alias
            setManagedTransform(managedIndex, t);
        }

        // Override the local transform of the main bone and update.
        localBoneTransform(transform);
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
     * @param forceKinematic ignored
     */
    @Override
    public void freeze(boolean forceKinematic) {
        blendToKinematicMode(KinematicSubmode.Frozen, 0f, null);
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
        if (endModelTransform != null) {
            /*
             * For a smooth transition, blend the saved model transform
             * (from the start of the blend interval) into the goal transform.
             */
            Quaternion startQuat = startModelTransform.getRotation(); // alias
            MyQuaternion.normalizeLocal(startQuat);
            Quaternion endQuat = endModelTransform.getRotation(); // alias
            if (startQuat.dot(endQuat) < 0f) {
                endQuat.multLocal(-1f);
            }
            MyQuaternion.normalizeLocal(endQuat);
            MyMath.slerp(kinematicWeight(), startModelTransform,
                    endModelTransform, transform);
            getControl().getSpatial().setLocalTransform(transform);
        }

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

            } else { // purely kinematic --- stop blending the model transform
                this.endModelTransform = null;
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
        return "Torso:";
    }

    /**
     * Copy animation data from the specified link, which must have the same
     * main bone.
     *
     * @param oldLink the link to copy from (not null, unaffected)
     */
    void postRebuild(TorsoLink oldLink) {
        int numManaged = countManaged();
        assert oldLink.countManaged() == numManaged;

        postRebuildLink(oldLink);
        if (oldLink.isKinematic()) {
            this.submode = oldLink.submode;
        } else {
            this.submode = KinematicSubmode.Frozen;
        }

        this.endModelTransform = Heart.deepCopy(oldLink.endModelTransform);
        startModelTransform.set(oldLink.startModelTransform);

        if (prevBoneTransforms == null) {
            this.prevBoneTransforms = new Transform[numManaged];
            for (int managedI = 0; managedI < numManaged; ++managedI) {
                this.prevBoneTransforms[managedI] = new Transform();
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
        this.endModelTransform = (Transform) capsule
                .readSavable(tagEndModelTransform, new Transform());
        this.meshToModel = (Transform) capsule
                .readSavable(tagMeshToModel, new Transform());
        this.startModelTransform = (Transform) capsule
                .readSavable(tagStartModelTransform, new Transform());
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
        setDynamic(gravity);

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
        capsule.write(endModelTransform, tagEndModelTransform, new Transform());
        capsule.write(meshToModel, tagMeshToModel, new Transform());
        capsule.write(
                startModelTransform, tagStartModelTransform, new Transform());
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
         * parent bone's transform, if any.
         */
        if (managedBones != null) { // old animation system
            Bone parent = getBone().getParent();
            if (parent != null) {
                RagUtils.meshToLocal(parent, result);
            }
        } else { // new animation system
            Joint parent = getArmatureJoint().getParent();
            if (parent != null) {
                RagUtils.meshToLocal(parent, result);
            }
        }

        // Subtract the body's local offset, rotated and scaled.
        Vector3f meshOffset = localOffset(null);
        meshOffset.multLocal(scale);
        orientation.mult(meshOffset, meshOffset);
        location.subtractLocal(meshOffset);

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
