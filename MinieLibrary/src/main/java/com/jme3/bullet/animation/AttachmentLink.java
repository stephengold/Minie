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
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.RotationOrder;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.joints.Constraint;
import com.jme3.bullet.joints.New6Dof;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.joints.SixDofJoint;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Spatial;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MySkeleton;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;

/**
 * Link an attachments node to a jointed rigid body in a ragdoll.
 *
 * @author Stephen Gold sgold@sonic.net
 *
 * Based on KinematicRagdollControl by Normen Hansen and Rémy Bouquet (Nehon).
 */
public class AttachmentLink extends PhysicsLink {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(AttachmentLink.class.getName());
    /**
     * local copy of {@link com.jme3.math.Matrix3f#IDENTITY}
     */
    final private static Matrix3f matrixIdentity = new Matrix3f();
    /**
     * local copy of {@link com.jme3.math.Quaternion#IDENTITY}
     */
    final private static Quaternion rotateIdentity = new Quaternion();
    /**
     * field names for serialization
     */
    final private static String tagAttachedModel = "attachedModel";
    final private static String tagEndModelTransform = "endModelTransform";
    final private static String tagStartModelTransform = "startModelTransform";
    /**
     * local copy of {@link com.jme3.math.Vector3f#ZERO}
     */
    final private static Vector3f translateIdentity = new Vector3f(0f, 0f, 0f);
    // *************************************************************************
    // fields

    /**
     * attached model (not null)
     */
    private Spatial attachedModel;
    /**
     * local transform for the attached model at the end of this link's most
     * recent blend interval, or null for no spatial blending
     */
    private Transform endModelTransform = null;
    /**
     * local transform of the attached model at the start of this link's most
     * recent blend interval
     */
    private Transform startModelTransform = new Transform();
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected AttachmentLink() {
    }

    /**
     * Instantiate a purely kinematic link between the specified model and the
     * specified rigid body.
     *
     * @param control the Control that will manage this link (not null, alias
     * created)
     * @param associatedBone the Bone associated with the attachment node (not
     * null, alias created)
     * @param manager the bone/torso link that manages the associated Bone (not
     * null, alias created)
     * @param attachModel the attached model to link (not null, alias created)
     * @param collisionShape the desired shape (not null, alias created)
     * @param linkConfig the link configuration (not null)
     * @param localOffset the location of the body's center (in the attached
     * model's local coordinates, not null, unaffected)
     */
    AttachmentLink(DacLinks control, Bone associatedBone, PhysicsLink manager,
            Spatial attachModel, CollisionShape collisionShape,
            LinkConfig linkConfig, Vector3f localOffset) {
        super(control, associatedBone, collisionShape, linkConfig, localOffset);
        assert manager != null;
        assert attachModel != null;

        this.attachedModel = attachModel;
        setParent(manager);

        PhysicsRigidBody managerBody = manager.getRigidBody();
        Transform managerToWorld = manager.physicsTransform(null);
        Transform worldToManager = managerToWorld.invert();

        Transform attachToWorld = physicsTransform(null);
        Transform attachToManager = attachToWorld.clone();
        attachToManager.combineWithParent(worldToManager);

        Vector3f pivotMesh = associatedBone.getModelSpacePosition();
        Spatial transformer = control.getTransformer();
        Vector3f pivotWorld = transformer.localToWorld(pivotMesh, null);
        managerToWorld.setScale(1f);
        Vector3f pivotManager
                = managerToWorld.transformInverseVector(pivotWorld, null);
        attachToWorld.setScale(1f);
        Vector3f pivot = attachToWorld.transformInverseVector(pivotWorld, null);

        Matrix3f rotManager = attachToManager.getRotation().toRotationMatrix();
        Matrix3f rot = matrixIdentity;

        Constraint constraint;
        RotationOrder rotationOrder = linkConfig.rotationOrder();
        if (rotationOrder == null) {
            constraint = new SixDofJoint(managerBody, getRigidBody(),
                    pivotManager, pivot, rotManager, rot, true);
        } else {
            constraint = new New6Dof(managerBody, getRigidBody(),
                    pivotManager, pivot, rotManager, rot, rotationOrder);
        }
        setJoint(constraint);

        RangeOfMotion rangeOfMotion = new RangeOfMotion();
        rangeOfMotion.setup(constraint, false, false, false);
    }

    /**
     * Instantiate a purely kinematic link between the specified model and the
     * specified rigid body.
     *
     * @param control the Control that will manage this link (not null, alias
     * created)
     * @param associatedJoint the armature joint associated with the attachment
     * node (not null, alias created)
     * @param manager the bone/torso link that manages the associated armature
     * joint (not null, alias created)
     * @param attachModel the attached model to link (not null, alias created)
     * @param collisionShape the desired shape (not null, alias created)
     * @param linkConfig the link configuration (not null)
     * @param localOffset the location of the body's center (in the attached
     * model's local coordinates, not null, unaffected)
     */
    AttachmentLink(DacLinks control, Joint associatedJoint, PhysicsLink manager,
            Spatial attachModel, CollisionShape collisionShape,
            LinkConfig linkConfig, Vector3f localOffset) {
        super(control, associatedJoint, collisionShape, linkConfig,
                localOffset);
        assert manager != null;
        assert attachModel != null;

        this.attachedModel = attachModel;
        setParent(manager);

        PhysicsRigidBody managerBody = manager.getRigidBody();
        Transform managerToWorld = manager.physicsTransform(null);
        Transform worldToManager = managerToWorld.invert();

        Transform attachToWorld = physicsTransform(null);
        Transform attachToManager = attachToWorld.clone();
        attachToManager.combineWithParent(worldToManager);

        Vector3f pivotMesh
                = associatedJoint.getModelTransform().getTranslation();
        Spatial transformer = control.getTransformer();
        Vector3f pivotWorld = transformer.localToWorld(pivotMesh, null);
        managerToWorld.setScale(1f);
        Vector3f pivotManager
                = managerToWorld.transformInverseVector(pivotWorld, null);
        attachToWorld.setScale(1f);
        Vector3f pivot = attachToWorld.transformInverseVector(pivotWorld, null);

        Matrix3f rotManager = attachToManager.getRotation().toRotationMatrix();
        Matrix3f rot = matrixIdentity;

        Constraint constraint;
        RotationOrder rotationOrder = linkConfig.rotationOrder();
        if (rotationOrder == null) {
            constraint = new SixDofJoint(managerBody, getRigidBody(),
                    pivotManager, pivot, rotManager, rot, true);
        } else {
            constraint = new New6Dof(managerBody, getRigidBody(),
                    pivotManager, pivot, rotManager, rot, rotationOrder);
        }
        setJoint(constraint);

        RangeOfMotion rangeOfMotion = new RangeOfMotion();
        rangeOfMotion.setup(constraint, false, false, false);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Begin blending this link to a purely kinematic mode.
     *
     * @param blendInterval the duration of the blend interval (in seconds,
     * &ge;0)
     * @param endModelTransform the desired local transform for the attached
     * model the blend completes or null for no change to local transform
     * (unaffected)
     */
    public void blendToKinematicMode(
            float blendInterval, Transform endModelTransform) {
        Validate.nonNegative(blendInterval, "blend interval");
        if (isReleased()) {
            throw new IllegalStateException(
                    "Cannot change modes once released.");
        }

        blendToKinematicMode(blendInterval);
        this.endModelTransform = endModelTransform;

        // Save initial transform for blending.
        if (endModelTransform != null) {
            Transform current = attachedModel.getLocalTransform(); // alias
            startModelTransform.set(current);
        }
    }

    /**
     * Access the attached model (not the attachment node).
     *
     * @return the pre-existing instance (not null)
     */
    public Spatial getAttachedModel() {
        assert attachedModel != null;
        return attachedModel;
    }

    /**
     * Release the attached model. The link must already be in dynamic mode and
     * cannot have been previously released.
     */
    public void release() {
        if (isKinematic()) {
            throw new IllegalStateException(
                    "Cannot release an attachment in kinematic mode.");
        }
        if (isReleased()) {
            throw new IllegalStateException(
                    "Cannot release the same attachment twice.");
        }

        PhysicsJoint joint = getJoint();
        joint.destroy();

        PhysicsSpace space = getControl().getPhysicsSpace();
        space.removeJoint(joint);
        setJoint(null);
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

        this.attachedModel = cloner.clone(attachedModel);
        this.endModelTransform = cloner.clone(endModelTransform);
        this.startModelTransform = cloner.clone(startModelTransform);
    }

    /**
     * Update this link in Dynamic mode, setting the local transform of the
     * attached model based on the transform of the linked rigid body.
     */
    @Override
    protected void dynamicUpdate() {
        assert !getRigidBody().isKinematic();

        Transform transform = localModelTransform(null);
        attachedModel.setLocalTransform(transform);
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
            blendToKinematicMode(0f, null);
        } else {
            setDynamic(translateIdentity);
        }
    }

    /**
     * Test whether the attached model has been released.
     *
     * @return true if released, otherwise false
     */
    @Override
    public boolean isReleased() {
        PhysicsJoint joint = getJoint();
        if (joint == null) {
            return true;
        } else {
            return false;
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

        if (endModelTransform != null) {
            /*
             * For a smooth transition, blend the saved model transform
             * (from the start of the blend interval) into the goal transform.
             */
            Quaternion startQuat = startModelTransform.getRotation(); // alias
            Quaternion endQuat = endModelTransform.getRotation(); // alias
            if (startQuat.dot(endQuat) < 0f) {
                endQuat.multLocal(-1f);
            }
            MyMath.slerp(kinematicWeight(), startModelTransform,
                    endModelTransform, transform);
            attachedModel.setLocalTransform(transform);
        }
        // The rigid-body transform gets updated by prePhysicsTick().

        super.kinematicUpdate(tpf);
    }

    /**
     * Unambiguously identify this link by name, within its DynamicAnimControl.
     *
     * @return a brief textual description (not null, not empty)
     */
    @Override
    public String name() {
        String result = "Attachment:" + boneName();
        return result;
    }

    /**
     * Calculate a physics transform for the rigid body.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the calculated transform (in physics-space coordinates, either
     * storeResult or a new transform, not null)
     */
    @Override
    final public Transform physicsTransform(Transform storeResult) {
        Transform result
                = (storeResult == null) ? new Transform() : storeResult;
        /*
         * Start with the rigid body's transform in the attached model's
         * local coordinates.
         */
        result.setTranslation(localOffset(null));
        result.setRotation(rotateIdentity);
        result.setScale(1f);

        // Convert to bone local coordinates.
        Transform tmp = attachedModel.getLocalTransform(); // alias
        result.combineWithParent(tmp);

        // Convert to mesh coordinates.
        Bone bone = getBone();
        if (bone != null) { // old animation system
            tmp = MySkeleton.copyMeshTransform(bone, null);
        } else { // new animation system
            Joint armatureJoint = getArmatureJoint();
            tmp = armatureJoint.getModelTransform().clone();
        }
        result.combineWithParent(tmp);

        // Convert to physics/world coordinates.
        getControl().meshTransform(tmp);
        result.combineWithParent(tmp);

        return result;
    }

    /**
     * Copy animation data from the specified link, which must correspond to the
     * same bone.
     *
     * @param oldLink the link to copy from (not null, unaffected)
     */
    void postRebuild(AttachmentLink oldLink) {
        assert oldLink != null;

        postRebuildLink(oldLink);
        if (oldLink.isReleased()) {
            setDynamic(translateIdentity);
            release();
            PhysicsRigidBody newBody = getRigidBody();
            if (newBody.isInWorld()) {
                PhysicsSpace physicsSpace = getControl().getPhysicsSpace();
                physicsSpace.removeCollisionObject(newBody);
            }
            PhysicsRigidBody oldBody = oldLink.getRigidBody();
            setRigidBody(oldBody);
        }
        this.endModelTransform = Heart.deepCopy(oldLink.endModelTransform);
        this.startModelTransform.set(oldLink.startModelTransform);
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

        this.attachedModel
                = (Spatial) capsule.readSavable(tagAttachedModel, null);
        this.endModelTransform = (Transform) capsule
                .readSavable(tagEndModelTransform, new Transform());
        this.startModelTransform = (Transform) capsule
                .readSavable(tagStartModelTransform, new Transform());
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

        capsule.write(attachedModel, tagAttachedModel, null);
        capsule.write(endModelTransform, tagEndModelTransform, null);
        capsule.write(startModelTransform, tagStartModelTransform, null);
    }
    // *************************************************************************
    // private methods

    /**
     * Calculate the local transform for the attached model to match the physics
     * transform of the rigid body.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the calculated model transform (in local coordinates, either
     * storeResult or a new Transform, not null)
     */
    private Transform localModelTransform(Transform storeResult) {
        Transform result
                = (storeResult == null) ? new Transform() : storeResult;
        Vector3f location = result.getTranslation(); // alias
        Quaternion orientation = result.getRotation(); // alias
        Vector3f scale = result.getScale(); // alias

        // Start with the rigid body's Transform in physics/world coordinates.
        getRigidBody().getTransform(result);

        // Convert to mesh coordinates.
        Transform worldToMesh = getControl().meshTransform(null).invert();
        result.combineWithParent(worldToMesh);

        // Convert to bone local coordinates.
        Transform boneToMesh;
        Bone bone = getBone();
        if (bone != null) { // old animation system
            boneToMesh = MySkeleton.copyMeshTransform(bone, null);
        } else { // new animation system
            Joint armatureJoint = getArmatureJoint();
            boneToMesh = armatureJoint.getModelTransform();
        }
        Transform meshToBone = boneToMesh.invert();
        result.combineWithParent(meshToBone);

        // Subtract the body's local offset, rotated and scaled.
        Vector3f modelOffset = localOffset(null);
        modelOffset.multLocal(scale);
        orientation.mult(modelOffset, modelOffset);
        location.subtractLocal(modelOffset);

        return result;
    }
}
