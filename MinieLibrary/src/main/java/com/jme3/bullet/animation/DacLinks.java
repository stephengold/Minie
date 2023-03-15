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

import com.jme3.anim.AnimComposer;
import com.jme3.anim.Armature;
import com.jme3.anim.Joint;
import com.jme3.anim.SkinningControl;
import com.jme3.animation.Bone;
import com.jme3.animation.Skeleton;
import com.jme3.animation.SkeletonControl;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.control.Control;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MyControl;
import jme3utilities.MyMesh;
import jme3utilities.MySkeleton;
import jme3utilities.MySpatial;
import jme3utilities.MyString;
import jme3utilities.Validate;
import jme3utilities.math.VectorSet;

/**
 * Access a DynamicAnimControl at the PhysicsLink level once it's been added to
 * a Spatial.
 *
 * @author Stephen Gold sgold@sonic.net
 *
 * Based on KinematicRagdollControl by Normen Hansen and Rémy Bouquet (Nehon).
 */
public class DacLinks
        extends DacConfiguration
        implements PhysicsTickListener {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger3
            = Logger.getLogger(DacLinks.class.getName());
    /**
     * local copy of {@link com.jme3.math.Quaternion#IDENTITY}
     */
    final private static Quaternion rotateIdentity = new Quaternion();
    /**
     * field names for serialization
     */
    final private static String tagArmature = "armature";
    final private static String tagAttachmentLinks = "attachmentLinks";
    final private static String tagBindTransforms = "bindTransforms";
    final private static String tagBoneLinkList = "boneLinkList";
    final private static String tagPreComposer = "preComposer";
    final private static String tagSkeleton = "skeleton";
    final private static String tagTorsoLink = "torsoLink";
    final private static String tagTransformer = "transformer";
    /**
     * local copy of {@link com.jme3.math.Transform#IDENTITY}
     */
    final private static Transform transformIdentity = new Transform();
    /**
     * local copy of {@link com.jme3.math.Vector3f#ZERO}
     */
    final private static Vector3f translateIdentity = new Vector3f(0f, 0f, 0f);
    // *************************************************************************
    // fields

    /**
     * Armature being controlled, or null for a Skeleton
     */
    private Armature armature = null;
    /**
     * false until the first simulation step, true thereafter, indicating that
     * all links are ready for dynamic mode
     */
    private boolean isReady = false;
    /**
     * bone links in a pre-order, depth-first traversal of the link hierarchy
     */
    private List<BoneLink> boneLinkList = null;
    /**
     * map bone names to attachment links
     */
    private Map<String, AttachmentLink> attachmentLinks = new HashMap<>(8);
    /**
     * map bone names to bone links
     */
    private Map<String, BoneLink> boneLinks = new HashMap<>(32);
    /**
     * helper control, or null if none
     */
    private PreComposer preComposer = null;
    /**
     * Skeleton being controlled, or null for an Armature
     */
    private Skeleton skeleton = null;
    /**
     * Spatial that provides the mesh-coordinate transform
     */
    private Spatial transformer = null;
    /**
     * torso link for this control
     */
    private TorsoLink torsoLink = null;
    /**
     * saved bind transform for each armature joint or skeleton bone
     */
    private Transform[] bindTransforms = null;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled control without any linked bones or attachments
     * (torso only).
     */
    protected DacLinks() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the saved bind transform of the indexed armature joint.
     *
     * @param jointIndex which joint in the Armature
     * @param storeResult storage for the result (modified if not null)
     * @return the bone's bind transform (in its parent's coordinates, either
     * storeResult or a new instance)
     */
    Transform copyBindTransform(int jointIndex, Transform storeResult) {
        Transform alias = bindTransforms[jointIndex];
        Transform result;
        if (storeResult == null) {
            result = alias.clone();
        } else {
            result = storeResult.set(alias);
        }

        return result;
    }

    /**
     * Access the named armature joint.
     * <p>
     * Allowed only when the Control IS added to a Spatial.
     *
     * @param jointName the name of the armature joint to access
     * @return the pre-existing instance, or null if not found
     */
    public Joint findArmatureJoint(String jointName) {
        verifyAddedToSpatial("access an armature joint");
        Joint result = armature.getJoint(jointName);
        return result;
    }

    /**
     * Access the AttachmentLink for the named bone. Returns null if the bone is
     * not associated with an attachment, or if the control is not added to a
     * Spatial.
     *
     * @param boneName the name of the bone (not null, not empty)
     * @return the pre-existing link, or null if not found
     */
    public AttachmentLink findAttachmentLink(String boneName) {
        Validate.nonEmpty(boneName, "bone name");
        AttachmentLink link = attachmentLinks.get(boneName);
        return link;
    }

    /**
     * Access the named Bone.
     * <p>
     * Allowed only when the Control IS added to a Spatial.
     *
     * @param boneName the name of the skeleton bone to access
     * @return the pre-existing instance, or null if not found
     */
    public Bone findBone(String boneName) {
        verifyAddedToSpatial("access a bone");
        Bone result = skeleton.getBone(boneName);
        return result;
    }

    /**
     * Access the BoneLink for the named bone. Returns null if bone is not
     * linked, or if the control is not added to a Spatial.
     *
     * @param boneName the name of the bone (not null, not empty)
     * @return the pre-existing BoneLink, or null if not found
     */
    public BoneLink findBoneLink(String boneName) {
        Validate.nonEmpty(boneName, "bone name");
        BoneLink boneLink = boneLinks.get(boneName);
        return boneLink;
    }

    /**
     * Access the named link. Returns null if the name is invalid, or if the
     * control is not added to a Spatial.
     *
     * @param linkName the name of the link (not null, not empty)
     * @return the pre-existing link, or null if not found
     */
    public PhysicsLink findLink(String linkName) {
        Validate.nonEmpty(linkName, "link name");

        PhysicsLink link;
        if (linkName.startsWith("Bone:")) {
            String boneName = MyString.remainder(linkName, "Bone:");
            link = findBoneLink(boneName);
        } else if (linkName.equals("Torso:")) {
            link = torsoLink;
        } else {
            String boneName = MyString.remainder(linkName, "Attachment:");
            link = findAttachmentLink(boneName);
        }

        return link;
    }

    /**
     * Access the Armature. Returns null if the Control is not added to a
     * Spatial.
     *
     * @return the pre-existing Armature, or null
     */
    public Armature getArmature() {
        return armature;
    }

    /**
     * Access the Skeleton. Returns null if the Control is not added to a
     * Spatial.
     *
     * @return the pre-existing Skeleton, or null
     */
    public Skeleton getSkeleton() {
        return skeleton;
    }

    /**
     * Access the TorsoLink. Returns null if the control is not added to a
     * Spatial.
     *
     * @return the pre-existing TorsoLink, or null
     */
    public TorsoLink getTorsoLink() {
        return torsoLink;
    }

    /**
     * Access the Spatial with the mesh-coordinate transform. Returns null if
     * the control is not added to a Spatial.
     *
     * @return the pre-existing spatial, or null
     */
    public Spatial getTransformer() {
        return transformer;
    }

    /**
     * Test whether this control is ready for dynamic mode.
     *
     * @return true if ready, otherwise false
     */
    public boolean isReady() {
        return isReady;
    }

    /**
     * Enumerate physics links of the specified type managed by this control.
     *
     * @param <T> subclass of PhysicsLink
     * @param linkType the subclass of PhysicsLink to search for (not null)
     * @return a new array of links (not null, not empty)
     */
    @SuppressWarnings("unchecked")
    public <T extends PhysicsLink> List<T> listLinks(Class<T> linkType) {
        int numLinks = countLinks();
        List<T> result = new ArrayList<>(numLinks);

        if (torsoLink != null
                && linkType.isAssignableFrom(torsoLink.getClass())) {
            result.add((T) torsoLink);
        }
        for (BoneLink link : boneLinkList) {
            if (linkType.isAssignableFrom(link.getClass())) {
                result.add((T) link);
            }
        }
        for (AttachmentLink link : attachmentLinks.values()) {
            if (linkType.isAssignableFrom(link.getClass())) {
                result.add((T) link);
            }
        }

        return result;
    }

    /**
     * Enumerate managed armature joints of the named link, in a pre-order,
     * depth-first traversal of the Armature, such that child joints never
     * precede their ancestors.
     *
     * @param managerName the name of the managing link (not null)
     * @return a new array of managed joints, including the manager if it is not
     * the torso
     */
    Joint[] listManagedArmatureJoints(String managerName) {
        Collection<Joint> list = new ArrayList<>(8);

        if (torsoName.equals(managerName)) {
            Joint[] roots = armature.getRoots();
            for (Joint rootJoint : roots) {
                list.add(rootJoint);
                addUnlinkedDescendants(rootJoint, list);
            }

        } else {
            BoneLink manager = findBoneLink(managerName);
            if (manager == null) {
                String message = "No link named " + MyString.quote(managerName);
                throw new IllegalArgumentException(message);
            }
            Joint managerJoint = manager.getArmatureJoint();
            list.add(managerJoint);
            addUnlinkedDescendants(managerJoint, list);
        }

        // Convert the list to an array.
        int numManagedJoints = list.size();
        Joint[] array = new Joint[numManagedJoints];
        list.toArray(array);

        return array;
    }

    /**
     * Enumerate managed bones of the named link, in a pre-order, depth-first
     * traversal of the skeleton, such that child bones never precede their
     * ancestors.
     *
     * @param managerName the name of the managing link (not null)
     * @return a new array of managed bones, including the manager if it is not
     * the torso
     */
    Bone[] listManagedBones(String managerName) {
        Collection<Bone> list = new ArrayList<>(8);

        if (torsoName.equals(managerName)) {
            Bone[] roots = skeleton.getRoots();
            for (Bone rootBone : roots) {
                list.add(rootBone);
                addUnlinkedDescendants(rootBone, list);
            }

        } else {
            BoneLink manager = findBoneLink(managerName);
            if (manager == null) {
                String msg = "No link named " + MyString.quote(managerName);
                throw new IllegalArgumentException(msg);
            }
            Bone managerBone = manager.getBone();
            list.add(managerBone);
            addUnlinkedDescendants(managerBone, list);
        }

        // Convert the list to an array.
        int numManaged = list.size();
        Bone[] array = new Bone[numManaged];
        list.toArray(array);

        return array;
    }

    /**
     * Enumerate all rigid bodies managed by this control.
     * <p>
     * Allowed only when the control IS added to a Spatial.
     *
     * @return a new array of pre-existing rigid bodies (not null, not empty)
     */
    public PhysicsRigidBody[] listRigidBodies() {
        verifyAddedToSpatial("enumerate rigid bodies");

        int numLinks = countLinks();
        PhysicsRigidBody[] result = new PhysicsRigidBody[numLinks];

        int linkIndex = 0;
        if (torsoLink != null) {
            result[0] = torsoLink.getRigidBody();
            ++linkIndex;
        }
        for (BoneLink boneLink : boneLinkList) {
            result[linkIndex] = boneLink.getRigidBody();
            ++linkIndex;
        }
        for (AttachmentLink link : attachmentLinks.values()) {
            result[linkIndex] = link.getRigidBody();
            ++linkIndex;
        }
        assert linkIndex == numLinks;

        return result;
    }

    /**
     * Copy the model's mesh-to-world transform.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the model's mesh transform (in world coordinates, either
     * storeResult or a new transform, not null)
     */
    Transform meshTransform(Transform storeResult) {
        Transform result = MySpatial.worldTransform(transformer, storeResult);
        return result;
    }

    /**
     * Calculate the physics transform to match the specified skeleton bone.
     *
     * @param bone the skeleton bone to match (not null, unaffected)
     * @param localOffset the location of the body's center (in the bone's local
     * coordinates, not null, unaffected)
     * @param storeResult storage for the result (modified if not null)
     * @return the calculated physics transform (either storeResult or a new
     * transform, not null)
     */
    Transform physicsTransform(
            Bone bone, Vector3f localOffset, Transform storeResult) {
        Transform result
                = (storeResult == null) ? new Transform() : storeResult;

        // Start with the body's transform in the bone's local coordinates.
        result.setTranslation(localOffset);
        result.setRotation(rotateIdentity);
        result.setScale(1f);

        // Convert to mesh coordinates.
        Transform localToMesh = MySkeleton.copyMeshTransform(bone, null);
        result.combineWithParent(localToMesh);

        // Convert to world (physics-space) coordinates.
        Transform meshToWorld = meshTransform(null);
        result.combineWithParent(meshToWorld);

        return result;
    }

    /**
     * Calculate the physics transform to match the specified armature joint.
     *
     * @param joint the armature joint to match (not null, unaffected)
     * @param localOffset the location of the body's center (in the bone's local
     * coordinates, not null, unaffected)
     * @param storeResult storage for the result (modified if not null)
     * @return the calculated physics transform (either storeResult or a new
     * transform, not null)
     */
    Transform physicsTransform(
            Joint joint, Vector3f localOffset, Transform storeResult) {
        Transform result
                = (storeResult == null) ? new Transform() : storeResult;
        /*
         * Start with the body's transform in the armature joint's
         * local coordinates.
         */
        result.setTranslation(localOffset);
        result.setRotation(rotateIdentity);
        result.setScale(1f);

        // Convert to mesh coordinates.
        Transform localToMesh = joint.getModelTransform();
        result.combineWithParent(localToMesh);

        // Convert to world (physics-space) coordinates.
        Transform meshToWorld = meshTransform(null);
        result.combineWithParent(meshToWorld);

        return result;
    }

    /**
     * Rebuild the ragdoll. This is useful if you applied scale to the model
     * after it was initialized.
     * <p>
     * Allowed only when the control IS added to a Spatial.
     */
    public void rebuild() {
        verifyAddedToSpatial("rebuild the ragdoll");

        Map<String, AttachmentLink> saveAttach = new HashMap<>(attachmentLinks);
        Map<String, BoneLink> saveBones = new HashMap<>(boneLinks);
        TorsoLink saveTorso = torsoLink;

        Spatial controlledSpatial = getSpatial();
        removeSpatialData(controlledSpatial);
        createSpatialData(controlledSpatial);

        for (Map.Entry<String, AttachmentLink> entry
                : attachmentLinks.entrySet()) {
            String name = entry.getKey();
            AttachmentLink newLink = entry.getValue();
            AttachmentLink oldLink = saveAttach.get(name);
            newLink.postRebuild(oldLink);
        }
        for (Map.Entry<String, BoneLink> entry : boneLinks.entrySet()) {
            String name = entry.getKey();
            BoneLink newLink = entry.getValue();
            BoneLink oldLink = saveBones.get(name);
            newLink.postRebuild(oldLink);
        }
        if (torsoLink != null) {
            torsoLink.postRebuild(saveTorso);
        }
    }

    /**
     * Alter the mass of the specified link.
     *
     * @param link the link to modify (not null)
     * @param mass the desired mass (&gt;0)
     */
    public void setMass(PhysicsLink link, float mass) {
        Validate.nonNull(link, "link");
        Validate.positive(mass, "mass");

        if (link instanceof BoneLink) {
            String boneName = link.boneName();
            setMass(boneName, mass);
        } else if (link instanceof TorsoLink) {
            setMass(torsoName, mass);
        } else {
            assert link instanceof AttachmentLink;
            String boneName = link.boneName();
            setAttachmentMass(boneName, mass);
        }
    }

    /**
     * Verify that this control is ready for dynamic mode, which implies that it
     * is added to a Spatial, added to a PhysicsSpace, and the physics has been
     * stepped.
     *
     * @param desiredAction (not null, not empty)
     */
    public void verifyReadyForDynamicMode(String desiredAction) {
        assert desiredAction != null;

        verifyAddedToSpatial(desiredAction);

        if (!added) {
            String message = "Cannot " + desiredAction
                    + " unless the control is added to a PhysicsSpace.";
            throw new IllegalStateException(message);
        }

        if (!isReady) {
            String message = "Cannot " + desiredAction
                    + " until the physics has been stepped.";
            throw new IllegalStateException(message);
        }
    }
    // *************************************************************************
    // new protected methods

    /**
     * Access the list of bone links in a pre-order, depth-first traversal of
     * the link hierarchy.
     *
     * @return the pre-existing list (not null)
     */
    protected List<BoneLink> getBoneLinks() {
        assert boneLinkList != null;
        return boneLinkList;
    }

    /**
     * Enumerate attachment links.
     *
     * @return a collection view of values in the internal map (not null)
     */
    protected Collection<AttachmentLink> listAttachmentLinks() {
        Collection<AttachmentLink> result = attachmentLinks.values();
        return result;
    }

    /**
     * Verify that this control is added to a Spatial.
     *
     * @param desiredAction (not null, not empty)
     */
    protected void verifyAddedToSpatial(String desiredAction) {
        assert desiredAction != null;

        Spatial controlledSpatial = getSpatial();
        if (controlledSpatial == null) {
            String message = "Cannot " + desiredAction
                    + " unless the Control is added to a Spatial.";
            throw new IllegalStateException(message);
        }
    }
    // *************************************************************************
    // DacConfiguration methods

    /**
     * Add all managed physics objects to the PhysicsSpace.
     */
    @Override
    protected void addPhysics() {
        PhysicsSpace space = getPhysicsSpace();
        Vector3f gravity = gravity(null);

        PhysicsRigidBody rigidBody;
        if (torsoLink != null) {
            rigidBody = torsoLink.getRigidBody();
            space.addCollisionObject(rigidBody);
            rigidBody.setGravity(gravity);
        }

        for (BoneLink boneLink : boneLinkList) {
            rigidBody = boneLink.getRigidBody();
            space.addCollisionObject(rigidBody);
            rigidBody.setGravity(gravity);

            PhysicsJoint joint = boneLink.getJoint();
            space.addJoint(joint);
        }

        for (AttachmentLink link : attachmentLinks.values()) {
            rigidBody = link.getRigidBody();
            space.addCollisionObject(rigidBody);
            rigidBody.setGravity(gravity);

            PhysicsJoint joint = link.getJoint();
            space.addJoint(joint);
        }
    }

    /**
     * Read the mass of the attachment associated with the named bone.
     *
     * @param boneName the name of the associated bone (not null, not empty)
     * @return the mass (in physics units, &gt;0) or NaN if undetermined
     */
    @Override
    public float attachmentMass(String boneName) {
        Validate.nonNull(boneName, "bone name");

        float mass;
        if (getSpatial() == null) {
            mass = super.attachmentMass(boneName);
        } else if (attachmentLinks.containsKey(boneName)) {
            AttachmentLink link = attachmentLinks.get(boneName);
            PhysicsRigidBody rigidBody = link.getRigidBody();
            mass = rigidBody.getMass();
        } else {
            String msg = "No attachment for " + MyString.quote(boneName);
            throw new IllegalArgumentException(msg);
        }

        return mass;
    }

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned Control into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this Control (not null, modified)
     * @param original the instance from which this Control was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);
        DacLinks originalDac = (DacLinks) original;

        this.boneLinkList = cloner.clone(boneLinkList);

        this.attachmentLinks = new HashMap<>(8);
        for (Map.Entry<String, AttachmentLink> entry
                : originalDac.attachmentLinks.entrySet()) {
            String boneName = entry.getKey();
            AttachmentLink link = entry.getValue();
            AttachmentLink copyLink = cloner.clone(link);
            attachmentLinks.put(boneName, copyLink);
        }

        this.boneLinks = new HashMap<>(32);
        for (Map.Entry<String, BoneLink> entry
                : originalDac.boneLinks.entrySet()) {
            String boneName = entry.getKey();
            BoneLink link = entry.getValue();
            BoneLink copyLink = cloner.clone(link);
            boneLinks.put(boneName, copyLink);
        }

        this.armature = cloner.clone(armature);
        this.preComposer = cloner.clone(preComposer);
        this.skeleton = cloner.clone(skeleton);
        this.transformer = cloner.clone(transformer);
        this.bindTransforms = cloner.clone(bindTransforms);
        this.torsoLink = cloner.clone(torsoLink);
    }

    /**
     * Create spatial-dependent data. Invoked each time the Control is added to
     * a Spatial. Also invoked by {@link #rebuild()}.
     *
     * @param spatial the controlled spatial (not null)
     */
    @Override
    protected void createSpatialData(Spatial spatial) {
        RagUtils.validate(spatial);
        int numDacs = MySpatial.countControls(spatial, DacLinks.class);
        if (numDacs > 1) {
            logger3.log(Level.WARNING, "Added a DynamicAnimControl to a model "
                    + "that already contains {0}.", numDacs - 1);
        }

        boolean saveHwSkinning;
        SkinningControl skinningControl
                = spatial.getControl(SkinningControl.class);
        String[] tempManagerMap;
        Transform[] savedTransforms;
        SkeletonControl skeletonControl = null;
        if (skinningControl == null) {
            skeletonControl = spatial.getControl(SkeletonControl.class);
            if (skeletonControl == null) {
                throw new IllegalArgumentException(
                        "The controlled spatial must have a SkinningControl or "
                        + "a SkeletonControl. Make sure the Control is there "
                        + "and not on some other Spatial.");
            }

            // spatial has a SkeletonControl
            sortControls(skeletonControl);

            // Temporarily disable hardware skinning.
            saveHwSkinning = skeletonControl.isHardwareSkinningPreferred();
            skeletonControl.setHardwareSkinningPreferred(false);

            // Analyze the model's Skeleton.
            this.skeleton = skeletonControl.getSkeleton();
            validateSkeleton();
            tempManagerMap = managerMap(skeleton);
            int numBones = skeleton.getBoneCount();

            // Temporarily put the skeleton into bind pose.
            MySkeleton.setUserControl(skeleton, true);
            savedTransforms = new Transform[numBones];
            Vector3f userScale = new Vector3f();
            for (int boneIndex = 0; boneIndex < numBones; ++boneIndex) {
                Bone bone = skeleton.getBone(boneIndex);
                savedTransforms[boneIndex]
                        = MySkeleton.copyLocalTransform(bone, null);

                userScale.set(bone.getLocalScale());
                userScale.divideLocal(bone.getBindScale()); // multiply?
                bone.setUserTransforms(
                        translateIdentity, rotateIdentity, userScale);
            }
            MySkeleton.setUserControl(skeleton, false);
            skeleton.updateWorldVectors();

            // Save the bind transform of each skeleton bone.
            this.bindTransforms = new Transform[numBones];
            for (int jointIndex = 0; jointIndex < numBones; ++jointIndex) {
                Bone bone = skeleton.getBone(jointIndex);
                this.bindTransforms[jointIndex]
                        = MySkeleton.copyBindTransform(bone, null);
            }

        } else { // spatial has a SkinningControl
            this.armature = skinningControl.getArmature();

            int numArmatureJoints = armature.getJointCount();
            savedTransforms = new Transform[numArmatureJoints];
            for (int jointI = 0; jointI < numArmatureJoints; ++jointI) {
                Joint armatureJoint = armature.getJoint(jointI);
                savedTransforms[jointI]
                        = armatureJoint.getLocalTransform().clone();
            }

            sortControls(skinningControl); // This resets the armature!

            // Temporarily disable hardware skinning.
            saveHwSkinning = skinningControl.isHardwareSkinningPreferred();
            skinningControl.setHardwareSkinningPreferred(false);

            // Analyze the model's Armature.
            validateArmature();
            tempManagerMap = managerMap(armature);

            // Temporarily put the armature into bind pose.
            for (int jointI = 0; jointI < numArmatureJoints; ++jointI) {
                Joint armatureJoint = armature.getJoint(jointI);
                armatureJoint.applyBindPose(); // TODO adjust the scale?
            }
            armature.update();

            // Save the bind transform of each armature joint.
            this.bindTransforms = new Transform[numArmatureJoints];
            for (int jointI = 0; jointI < numArmatureJoints; ++jointI) {
                Joint armatureJoint = armature.getJoint(jointI);
                this.bindTransforms[jointI]
                        = armatureJoint.getLocalTransform().clone();
            }
            /*
             * If there's an AnimComposer, insert a PreComposer to hide
             * our Armature modifications.
             */
            AnimComposer composer = spatial.getControl(AnimComposer.class);
            if (composer == null) {
                logger3.log(Level.WARNING, "Didn't find an AnimComposer.");
            } else {
                int composerIndex = MyControl.findIndex(composer, spatial);
                this.preComposer = new PreComposer(this);
                spatial.addControlAt(composerIndex, preComposer);
            }
        }
        /*
         * Find the target meshes and choose the transform spatial.
         * Don't invoke getTargets() here, since the SkinningControl or
         * SkeletonControl might not be initialized yet.
         */
        List<Mesh> targetList = RagUtils.listDacMeshes(spatial, null);
        Mesh[] targets = new Mesh[targetList.size()];
        targetList.toArray(targets);
        this.transformer = MySpatial.findAnimatedGeometry(spatial);
        if (transformer == null) {
            this.transformer = spatial;
        }

        // Enumerate mesh-vertex coordinates and assign them to managers.
        Map<String, VectorSet> coordsMap
                = RagUtils.coordsMap(targets, tempManagerMap);

        // Create the TorsoLink.
        VectorSet vertexLocations = coordsMap.get(torsoName);
        createTorsoLink(vertexLocations, targets);

        // Create bone links without physics joints.
        String[] linkedBoneNames = listLinkedBoneNames();
        for (String boneName : linkedBoneNames) {
            vertexLocations = coordsMap.get(boneName);
            createBoneLink(boneName, vertexLocations);
        }
        int numLinkedBones = countLinkedBones();
        assert boneLinks.size() == numLinkedBones;
        /*
         * Add physics joints to connect each BoneLink rigid body
         * with its parent in the link hierarchy.
         * Also initialize the boneLinkList.
         */
        this.boneLinkList = new ArrayList<>(numLinkedBones);
        addJoints(torsoLink);
        assert boneLinkList.size() == numLinkedBones : boneLinkList.size();

        // Create attachment links with physics joints.
        String[] attachBoneNames = listAttachmentBoneNames();
        for (String boneName : attachBoneNames) {
            if (skinningControl == null) {
                createAttachmentLink(boneName, skeletonControl, tempManagerMap);
            } else {
                createAttachmentLink(boneName, skinningControl, tempManagerMap);
            }
        }

        if (skinningControl == null) { // old animation system
            skeletonControl.setHardwareSkinningPreferred(saveHwSkinning);

            // Restore the skeleton's pose.
            int numBones = skeleton.getBoneCount();
            for (int boneIndex = 0; boneIndex < numBones; ++boneIndex) {
                Bone bone = skeleton.getBone(boneIndex);
                MySkeleton.setLocalTransform(bone, savedTransforms[boneIndex]);
            }
            skeleton.updateWorldVectors();

        } else { // new animation system
            skinningControl.setHardwareSkinningPreferred(saveHwSkinning);

            // Restore the armature's pose.
            int numArmatureJoints = armature.getJointCount();
            for (int jointI = 0; jointI < numArmatureJoints; ++jointI) {
                Joint armatureJoint = armature.getJoint(jointI);
                armatureJoint.setLocalTransform(savedTransforms[jointI]);
            }
            armature.update();
        }

        int maxHops = ignoredHops();
        ignoreCollisions(maxHops);

        if (added) {
            addPhysics();
        }

        logger3.log(Level.FINE, "Created ragdoll.");
    }

    /**
     * Return the mass of the named bone/torso.
     *
     * @param boneName the name of the bone/torso (not null)
     * @return the mass (&gt;0) or NaN if undetermined
     */
    @Override
    public float mass(String boneName) {
        Validate.nonNull(boneName, "bone name");

        float mass;
        if (getSpatial() == null) {
            mass = super.mass(boneName);
        } else if (torsoName.equals(boneName)) {
            PhysicsRigidBody rigidBody = torsoLink.getRigidBody();
            mass = rigidBody.getMass();
        } else if (boneLinks.containsKey(boneName)) {
            BoneLink link = boneLinks.get(boneName);
            PhysicsRigidBody rigidBody = link.getRigidBody();
            mass = rigidBody.getMass();
        } else {
            String msg = "No bone/torso named " + MyString.quote(boneName);
            throw new IllegalArgumentException(msg);
        }

        return mass;
    }

    /**
     * De-serialize this Control from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    @SuppressWarnings("unchecked")
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);
        InputCapsule capsule = importer.getCapsule(this);

        this.boneLinkList = capsule.readSavableArrayList(tagBoneLinkList, null);
        if (boneLinkList != null) {
            for (BoneLink link : boneLinkList) {
                String name = link.boneName();
                boneLinks.put(name, link);
            }
        }

        Savable[] savableArray = capsule.readSavableArray(
                tagAttachmentLinks, new AttachmentLink[0]);
        for (Savable savable : savableArray) {
            AttachmentLink link = (AttachmentLink) savable;
            String name = link.boneName();
            attachmentLinks.put(name, link);
        }

        this.armature = (Armature) capsule.readSavable(tagArmature, null);
        this.preComposer
                = (PreComposer) capsule.readSavable(tagPreComposer, null);
        this.skeleton = (Skeleton) capsule.readSavable(tagSkeleton, null);
        this.transformer = (Spatial) capsule.readSavable(tagTransformer, null);
        this.bindTransforms
                = RagUtils.readTransformArray(capsule, tagBindTransforms);
        this.torsoLink = (TorsoLink) capsule.readSavable(tagTorsoLink, null);
    }

    /**
     * Remove all managed physics objects from the PhysicsSpace.
     */
    @Override
    protected void removePhysics() {
        assert added;
        PhysicsSpace space = getPhysicsSpace();

        PhysicsRigidBody rigidBody;
        if (torsoLink != null) {
            rigidBody = torsoLink.getRigidBody();
            space.removeCollisionObject(rigidBody);
        }

        for (BoneLink boneLink : boneLinks.values()) {
            rigidBody = boneLink.getRigidBody();
            space.removeCollisionObject(rigidBody);

            PhysicsJoint joint = boneLink.getJoint();
            space.removeJoint(joint);
        }

        for (AttachmentLink link : attachmentLinks.values()) {
            if (!link.isReleased()) {
                rigidBody = link.getRigidBody();
                space.removeCollisionObject(rigidBody);

                PhysicsJoint joint = link.getJoint();
                space.removeJoint(joint);
            }
        }
        this.isReady = false;
    }

    /**
     * Remove spatial-dependent data. Invoked each time this control is rebuilt
     * or removed from a Spatial.
     *
     * @param spatial the Spatial to which this Control was added (unused)
     */
    @Override
    protected void removeSpatialData(Spatial spatial) {
        if (added) {
            removePhysics();
        }

        for (AttachmentLink attachmentLink : attachmentLinks.values()) {
            Node node;
            Joint armatureJoint = attachmentLink.getArmatureJoint();
            if (armatureJoint == null) {
                Bone bone = attachmentLink.getBone();
                node = MySkeleton.getAttachments(bone);
                MySkeleton.cancelAttachments(bone);
            } else {
                node = MySkeleton.getAttachments(armatureJoint);
                MySkeleton.cancelAttachments(armatureJoint);
            }
            node.removeFromParent();
        }
        attachmentLinks.clear();

        if (preComposer != null) {
            Spatial controlledSpatial = getSpatial();
            controlledSpatial.removeControl(preComposer);
            this.preComposer = null;
        }
        this.armature = null;
        if (skeleton != null) { // old animation system
            MySkeleton.setUserControl(skeleton, false);
            this.skeleton = null;
        }

        boneLinks.clear();
        this.boneLinkList = null;
        this.torsoLink = null;
        this.transformer = null;
    }

    /**
     * Alter the configuration of the attachment associated with the named bone.
     *
     * @param boneName the name of the associated bone (not null, not empty)
     * @param config the desired configuration (not null)
     */
    @Override
    public void setAttachmentConfig(String boneName, LinkConfig config) {
        Validate.nonNull(config, "configuration");

        super.setAttachmentConfig(boneName, config);

        AttachmentLink link = attachmentLinks.get(boneName);
        if (link != null) {
            Spatial spatial = getSpatial();
            if (skeleton != null) { // old animation system
                SkeletonControl skeletonControl
                        = spatial.getControl(SkeletonControl.class);
                String[] managerMap = managerMap(skeleton);
                createAttachmentLink(boneName, skeletonControl, managerMap);

            } else { // new animation system
                SkinningControl skinningControl
                        = spatial.getControl(SkinningControl.class);
                String[] managerMap = managerMap(armature);
                createAttachmentLink(boneName, skinningControl, managerMap);
            }
        }
    }

    /**
     * Alter the mass of the attachment associated with the named bone.
     *
     * @param boneName the name of the associated bone (not null, not empty)
     * @param mass the desired mass (&gt;0)
     */
    @Override
    public void setAttachmentMass(String boneName, float mass) {
        Validate.positive(mass, "mass");
        super.setAttachmentMass(boneName, mass);

        AttachmentLink link = attachmentLinks.get(boneName);
        if (link != null) {
            link.getRigidBody().setMass(mass);
        }
    }

    /**
     * Alter the viscous damping ratio for all rigid bodies, including new ones.
     *
     * @param dampingRatio the desired damping ratio (non-negative, 0&rarr;no
     * damping, 1&rarr;critically damped, default=0.6)
     */
    @Override
    public void setDamping(float dampingRatio) {
        Validate.nonNegative(dampingRatio, "damping ratio");
        super.setDamping(dampingRatio);

        if (getSpatial() != null) {
            PhysicsRigidBody[] bodies = listRigidBodies();
            for (PhysicsRigidBody rigidBody : bodies) {
                rigidBody.setDamping(dampingRatio, dampingRatio);
            }
        }
    }

    /**
     * Alter this control's gravitational acceleration for Ragdoll mode.
     *
     * @param gravity the desired acceleration vector (in physics-space
     * coordinates, not null, unaffected, default=(0,-9.8,0))
     */
    @Override
    public void setGravity(Vector3f gravity) {
        Validate.finite(gravity, "gravity");
        super.setGravity(gravity);

        if (getSpatial() != null) { // TODO make sure it's in ragdoll mode
            PhysicsRigidBody[] bodies = listRigidBodies();
            for (PhysicsRigidBody rigidBody : bodies) {
                if (rigidBody.isDynamic() && rigidBody.isInWorld()) {
                    rigidBody.setGravity(gravity);
                }
            }
        }
    }

    /**
     * Alter the range of motion of the joint connecting the named BoneLink to
     * its parent in the link hierarchy.
     *
     * @param boneName the name of the BoneLink (not null, not empty)
     * @param rom the desired range of motion (not null)
     */
    @Override
    public void setJointLimits(String boneName, RangeOfMotion rom) {
        if (!hasBoneLink(boneName)) {
            String msg = "No linked bone named " + MyString.quote(boneName);
            throw new IllegalArgumentException(msg);
        }
        Validate.nonNull(rom, "range of motion");

        super.setJointLimits(boneName, rom);

        if (getSpatial() != null) {
            BoneLink boneLink = findBoneLink(boneName);
            PhysicsJoint joint = boneLink.getJoint();
            rom.setup(joint, false, false, false);
        }
    }

    /**
     * Specify the main bone.
     *
     * @param boneName the name of the desired bone, or null to determine the
     * main bone heuristically when the control is added to a spatial
     */
    @Override
    public void setMainBoneName(String boneName) {
        Spatial controlledSpatial = getSpatial();
        if (controlledSpatial != null) {
            throw new IllegalStateException("Cannot change the main bone once "
                    + "the Control is added to a Spatial.");
        }

        super.setMainBoneName(boneName);
    }

    /**
     * Alter the mass of the named bone/torso.
     *
     * @param boneName the name of the bone, or torsoName (not null)
     * @param mass the desired mass (&gt;0)
     */
    @Override
    public void setMass(String boneName, float mass) {
        Validate.positive(mass, "mass");
        super.setMass(boneName, mass);

        if (getSpatial() != null) {
            PhysicsRigidBody rigidBody;
            if (torsoName.equals(boneName)) {
                rigidBody = torsoLink.getRigidBody();
            } else {
                BoneLink link = findBoneLink(boneName);
                rigidBody = link.getRigidBody();
            }
            rigidBody.setMass(mass);
        }
    }

    /**
     * Translate the torso to the specified location.
     *
     * @param vec desired location (not null, finite, unaffected)
     */
    @Override
    protected void setPhysicsLocation(Vector3f vec) {
        Validate.finite(vec, "vec");
        torsoLink.getRigidBody().setPhysicsLocation(vec);
    }

    /**
     * Rotate the torso to the specified orientation.
     *
     * @param quat desired orientation (not null, not zero, unaffected)
     */
    @Override
    protected void setPhysicsRotation(Quaternion quat) {
        Validate.nonZero(quat, "quat");
        torsoLink.getRigidBody().setPhysicsRotation(quat);
    }

    /**
     * Return the ragdoll's total mass, including attachments.
     *
     * @return the total mass (&gt;0) or NaN if undetermined
     */
    @Override
    public float totalMass() {
        float result;
        Spatial controlledSpatial = getSpatial();
        if (controlledSpatial == null) {
            result = super.totalMass();
        } else {
            PhysicsRigidBody body = torsoLink.getRigidBody();
            result = body.getMass();
            for (BoneLink boneLink : boneLinkList) {
                body = boneLink.getRigidBody();
                result += body.getMass();
            }
            for (AttachmentLink attachmentLink : attachmentLinks.values()) {
                body = attachmentLink.getRigidBody();
                result += body.getMass();
            }
        }

        return result;
    }

    /**
     * Update this Control. Invoked once per frame during the logical-state
     * update, provided the control is added to a scene. Do not invoke directly
     * from user code.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void update(float tpf) {
        verifyAddedToSpatial("update the control");
        if (!isEnabled()) {
            return;
        }

        if (preComposer != null) {
            preComposer.saveArmature();
        }

        if (torsoLink != null) {
            torsoLink.update(tpf);
        }
        for (BoneLink boneLink : boneLinkList) {
            boneLink.update(tpf);
        }
        for (AttachmentLink link : attachmentLinks.values()) {
            link.update(tpf);
        }
    }

    /**
     * Serialize this Control to the specified exporter, for example when saving
     * to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        super.write(exporter);
        OutputCapsule capsule = exporter.getCapsule(this);

        if (boneLinkList != null) {
            int count = countLinkedBones();
            Savable[] savableArray = new Savable[count];
            boneLinkList.toArray(savableArray);
            capsule.write(savableArray, tagBoneLinkList, null);
        }

        int count = countAttachments();
        AttachmentLink[] links = new AttachmentLink[count];
        attachmentLinks.values().toArray(links);
        capsule.write(links, tagAttachmentLinks, new AttachmentLink[0]);

        capsule.write(armature, tagArmature, null);
        // isReady and boneLinks are never written.
        capsule.write(preComposer, tagPreComposer, null);
        capsule.write(skeleton, tagSkeleton, null);
        capsule.write(transformer, tagTransformer, null);
        capsule.write(bindTransforms, tagBindTransforms, null);
        capsule.write(torsoLink, tagTorsoLink, null);
    }
    // *************************************************************************
    // PhysicsTickListener methods

    /**
     * Callback from Bullet, invoked just after the physics has been stepped.
     * Used to re-activate any deactivated rigid bodies.
     *
     * @param space the space that was just stepped (not null)
     * @param timeStep the time per simulation step (in seconds, &ge;0)
     */
    @Override
    public void physicsTick(PhysicsSpace space, float timeStep) {
        assert space == getPhysicsSpace();
        Validate.nonNegative(timeStep, "time step");

        torsoLink.postTick();
        for (BoneLink boneLink : boneLinkList) {
            boneLink.postTick();
        }
        for (AttachmentLink link : attachmentLinks.values()) {
            link.postTick();
        }

        this.isReady = true;
    }

    /**
     * Callback from Bullet, invoked just before the physics is stepped. A good
     * time to clear/apply forces.
     *
     * @param space the space that is about to be stepped (not null)
     * @param timeStep the time per simulation step (in seconds, &ge;0)
     */
    @Override
    public void prePhysicsTick(PhysicsSpace space, float timeStep) {
        assert space == getPhysicsSpace();
        Validate.nonNegative(timeStep, "time step");

        torsoLink.preTick(timeStep);
        for (BoneLink boneLink : boneLinkList) {
            boneLink.preTick(timeStep);
        }
        for (AttachmentLink link : attachmentLinks.values()) {
            link.preTick(timeStep);
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Add physics joints to connect the named bone/torso link with each of its
     * children. Also fill in the boneLinkList. Note: recursive!
     *
     * @param parentLink the parent bone/torso link (not null)
     */
    private void addJoints(PhysicsLink parentLink) {
        List<String> childNames = childNames(parentLink);
        for (String childName : childNames) {
            /*
             * Add the PhysicsJoint and configure its range of motion.
             * Also initialize the BoneLink's parent and its array
             * of managed bones.
             */
            BoneLink childLink = findBoneLink(childName);
            childLink.addJoint(parentLink);

            // Add the BoneLink to the pre-order list.
            boneLinkList.add(childLink);

            addJoints(childLink);
        }
    }

    /**
     * Enumerate immediate child BoneLinks of the specified bone/torso link.
     *
     * @param link the bone/torso link (not null)
     * @return a new list of bone names
     */
    private List<String> childNames(PhysicsLink link) {
        assert link != null;

        String linkName;
        if (link == torsoLink) {
            linkName = torsoName;
        } else {
            linkName = link.boneName();
        }

        List<String> result = new ArrayList<>(8);
        for (String childName : listLinkedBoneNames()) {
            if (armature == null) {
                Bone bone = findBone(childName);
                Bone parent = bone.getParent();
                if (parent != null && findManager(parent).equals(linkName)) {
                    result.add(childName);
                }
            } else {
                Joint armatureJoint = findArmatureJoint(childName);
                Joint parent = armatureJoint.getParent();
                if (parent != null && findManager(parent).equals(linkName)) {
                    result.add(childName);
                }
            }
        }

        return result;
    }

    /**
     * Create a jointed AttachmentLink for the named bone and add it to the
     * attachmentLinks map.
     *
     * @param boneName the name of the attachment bone to be linked (not null)
     * @param skeletonControl (not null)
     * @param managerMap a map from bone indices to managing link names (not
     * null, unaffected)
     */
    private void createAttachmentLink(String boneName,
            SkeletonControl skeletonControl, String[] managerMap) {
        assert boneName != null;
        assert skeletonControl != null;
        assert managerMap != null;

        // Collect the location of every mesh vertex in the attached model.
        Spatial attachModel = getAttachmentModel(boneName);
        attachModel = Heart.deepCopy(attachModel);
        VectorSet vertexLocations
                = MyMesh.listVertexLocations(attachModel, null);

        // Attach the model to the attachments node.
        Node node = skeletonControl.getAttachmentsNode(boneName);
        node.attachChild(attachModel);

        // Determine which link will manage the new AttachmentLink.
        Bone bone = skeleton.getBone(boneName);
        int boneIndex = skeleton.getBoneIndex(bone);
        String managerName = managerMap[boneIndex];
        PhysicsLink manager;
        if (managerName.equals(torsoName)) {
            manager = torsoLink;
        } else {
            manager = boneLinks.get(managerName);
        }

        // Locate the attached model's center of mass.
        LinkConfig linkConfig = attachmentConfig(boneName);
        CenterHeuristic centerHeuristic = linkConfig.centerHeuristic();
        assert centerHeuristic != CenterHeuristic.Joint;
        Vector3f center = centerHeuristic.center(vertexLocations, null);

        // Create the CollisionShape.
        CollisionShape shape = linkConfig
                .createShape(transformIdentity, center, vertexLocations);

        AttachmentLink link = new AttachmentLink(
                this, bone, manager, attachModel, shape, linkConfig, center);
        attachmentLinks.put(boneName, link);
    }

    /**
     * Create a jointed AttachmentLink for the named armature joint and add it
     * to the attachmentLinks map.
     *
     * @param jointName the name of the attachment joint to be linked (not null)
     * @param skinningControl (not null)
     * @param managerMap a map from joint indices to managing link names (not
     * null, unaffected)
     */
    private void createAttachmentLink(String jointName,
            SkinningControl skinningControl, String[] managerMap) {
        assert jointName != null;
        assert skinningControl != null;
        assert managerMap != null;

        // Collect the location of every mesh vertex in the attached model.
        Spatial attachModel = getAttachmentModel(jointName);
        attachModel = Heart.deepCopy(attachModel);
        VectorSet vertexLocations
                = MyMesh.listVertexLocations(attachModel, null);

        // Attach the model to the attachments node.
        Node node = skinningControl.getAttachmentsNode(jointName);
        node.attachChild(attachModel);

        // Determine which link will manage the new AttachmentLink.
        Joint joint = armature.getJoint(jointName);
        int jointIndex = armature.getJointIndex(joint);
        String managerName = managerMap[jointIndex];
        PhysicsLink manager;
        if (managerName.equals(torsoName)) {
            manager = torsoLink;
        } else {
            manager = boneLinks.get(managerName);
        }

        // Locate the attached model's center of mass.
        LinkConfig linkConfig = attachmentConfig(jointName);
        CenterHeuristic centerHeuristic = linkConfig.centerHeuristic();
        assert centerHeuristic != CenterHeuristic.Joint;
        Vector3f center = centerHeuristic.center(vertexLocations, null);

        // Create the CollisionShape.
        CollisionShape shape = linkConfig
                .createShape(transformIdentity, center, vertexLocations);

        AttachmentLink link = new AttachmentLink(
                this, joint, manager, attachModel, shape, linkConfig, center);
        attachmentLinks.put(jointName, link);
    }

    /**
     * Create a jointless BoneLink for the named bone/joint, and add it to the
     * boneLinks map.
     *
     * @param boneName the name of the bone/joint to be linked (not null)
     * @param vertexLocations the set of vertex locations (not null, not empty)
     */
    private void createBoneLink(String boneName, VectorSet vertexLocations) {
        if (vertexLocations == null || vertexLocations.numVectors() == 0) {
            String msg = String.format("No mesh vertices for linked bone %s.",
                    MyString.quote(boneName));
            throw new IllegalArgumentException(msg);
        }

        Bone bone = null;
        Joint armatureJoint = null;
        Transform boneToMesh;
        if (skeleton != null) { // old animation system
            bone = findBone(boneName);
            boneToMesh = MySkeleton.copyMeshTransform(bone, null);
        } else { // new animation system
            armatureJoint = findArmatureJoint(boneName);
            boneToMesh = armatureJoint.getModelTransform();
        }
        Transform meshToBone = boneToMesh.invert();
        LinkConfig linkConfig = config(boneName);

        // Create the CollisionShape and locate the center of mass.
        Vector3f center;
        CenterHeuristic centerHeuristic = linkConfig.centerHeuristic();
        if (centerHeuristic == CenterHeuristic.Joint) {
            center = translateIdentity;
        } else {
            center = centerHeuristic.center(vertexLocations, null);
            center.subtractLocal(boneToMesh.getTranslation());
        }
        CollisionShape shape
                = linkConfig.createShape(meshToBone, center, vertexLocations);

        meshToBone.getTranslation().zero();
        Vector3f offset = meshToBone.transformVector(center, null);

        BoneLink link;
        if (skeleton != null) { // old animation system
            link = new BoneLink(this, bone, shape, linkConfig, offset);
        } else { // new animation system
            link = new BoneLink(this, armatureJoint, shape, linkConfig, offset);
        }
        boneLinks.put(boneName, link);
    }

    /**
     * Create the TorsoLink.
     *
     * @param vertexLocations the set of vertex locations (not null, not empty)
     * @param meshes array of animated meshes to use (not null, unaffected)
     */
    private void createTorsoLink(VectorSet vertexLocations, Mesh[] meshes) {
        if (vertexLocations == null || vertexLocations.numVectors() == 0) {
            throw new IllegalArgumentException(
                    "No mesh vertices for the torso."
                    + " Make sure the root bone is not linked.");
        }

        // Create the CollisionShape.
        Bone bone = null;
        Joint armatureJoint = null;
        String mainBoneName = mainBoneName();
        Transform boneToMesh;
        if (skeleton != null) { // old animation system
            if (mainBoneName == null) { // default main bone
                bone = RagUtils.findMainBone(skeleton, meshes);
                assert bone.getParent() == null;
                mainBoneName = bone.getName();
                super.setMainBoneName(mainBoneName);
            } else {
                bone = skeleton.getBone(mainBoneName);
                if (bone == null) {
                    String q = MyString.quote(mainBoneName);
                    throw new IllegalStateException("Bone not found: " + q);
                }
            }
            boneToMesh = MySkeleton.copyMeshTransform(bone, null);

        } else { // new animation system
            if (mainBoneName == null) { // default main bone
                armatureJoint = RagUtils.findMainJoint(armature, meshes);
                assert armatureJoint.getParent() == null;
                mainBoneName = armatureJoint.getName();
                super.setMainBoneName(mainBoneName);
            } else {
                armatureJoint = armature.getJoint(mainBoneName);
                if (armatureJoint == null) {
                    String q = MyString.quote(mainBoneName);
                    throw new IllegalStateException("Joint not found: " + q);
                }
            }
            boneToMesh = armatureJoint.getModelTransform();
        }
        Transform meshToBone = boneToMesh.invert();
        LinkConfig linkConfig = config(torsoName);

        // Create the CollisionShape and locate the center of mass.
        CenterHeuristic centerHeuristic = linkConfig.centerHeuristic();
        assert centerHeuristic != CenterHeuristic.Joint;
        Vector3f center = centerHeuristic.center(vertexLocations, null);
        center.subtractLocal(boneToMesh.getTranslation());
        CollisionShape shape
                = linkConfig.createShape(meshToBone, center, vertexLocations);

        meshToBone.getTranslation().zero();
        Vector3f offset = meshToBone.transformVector(center, null);

        Transform meshToModel;
        Spatial cgm = getSpatial();
        if (cgm instanceof Node) {
            Transform modelToMesh
                    = RagUtils.relativeTransform(transformer, (Node) cgm, null);
            meshToModel = modelToMesh.invert();
        } else { // cgm instanceof Geometry
            meshToModel = transformIdentity;
        }

        if (skeleton != null) { // old animation system
            this.torsoLink = new TorsoLink(
                    this, bone, shape, linkConfig, meshToModel, offset);
        } else { // new animation system
            this.torsoLink = new TorsoLink(this, armatureJoint, shape,
                    linkConfig, meshToModel, offset);
        }
    }

    /**
     * Ignore collisions between rigid bodies connected by at most maxHops
     * physics joints, but don't ignore any other pairs.
     *
     * @param maxHops the maximum number of hops (&ge;0)
     */
    private void ignoreCollisions(int maxHops) {
        // Clear the ignore lists of all bodies.
        PhysicsRigidBody[] bodies = listRigidBodies();
        for (PhysicsRigidBody body : bodies) {
            body.clearIgnoreList();
        }

        // Rebuild the ignore lists using recursion.
        Map<PhysicsBody, Integer> visited = new HashMap<>(bodies.length);
        for (PhysicsRigidBody body : bodies) {
            visited.clear();
            visited.put(body, maxHops);
            RagUtils.ignoreCollisions(body, body, maxHops, visited);
        }
    }

    /**
     * Sort the controls of the controlled spatial, such that this Control will
     * come BEFORE the specified Control.
     *
     * @param otherSgc (not null)
     */
    private void sortControls(Control otherSgc) {
        assert otherSgc != null;

        Spatial spatial = getSpatial();
        int dacIndex = MyControl.findIndex(this, spatial);
        assert dacIndex != -1;
        int otherIndex = MyControl.findIndex(otherSgc, spatial);
        assert otherIndex != -1;
        assert dacIndex != otherIndex;

        if (dacIndex > otherIndex) {
            /*
             * Remove the other Control and re-add it to make sure it will get
             * updated AFTER this control.
             */
            spatial.removeControl(otherSgc);
            spatial.addControl(otherSgc);

            dacIndex = MyControl.findIndex(this, spatial);
            assert dacIndex != -1;
            otherIndex = MyControl.findIndex(otherSgc, spatial);
            assert otherIndex != -1;
            assert dacIndex < otherIndex;
        }
    }

    /**
     * Validate the model's armature.
     */
    private void validateArmature() {
        RagUtils.validate(armature);

        for (String jointName : listLinkedBoneNames()) {
            Joint joint = findArmatureJoint(jointName);
            if (joint == null) {
                String message = String.format(
                        "Linked bone %s not found in armature.",
                        MyString.quote(jointName));
                throw new IllegalArgumentException(message);
            }
            if (joint.getParent() == null) {
                logger3.log(Level.WARNING, "Linked bone {0} is a root joint.",
                        MyString.quote(jointName));
            }
        }
        for (String jointName : listAttachmentBoneNames()) {
            Joint joint = findArmatureJoint(jointName);
            if (joint == null) {
                String message = String.format(
                        "Attachment joint %s not found in armature.",
                        MyString.quote(jointName));
                throw new IllegalArgumentException(message);
            }
        }
    }

    /**
     * Validate the model's skeleton.
     */
    private void validateSkeleton() {
        RagUtils.validate(skeleton);

        for (String boneName : listLinkedBoneNames()) {
            Bone bone = findBone(boneName);
            if (bone == null) {
                String msg = String.format(
                        "Linked bone %s not found in skeleton.",
                        MyString.quote(boneName));
                throw new IllegalArgumentException(msg);
            }
            if (bone.getParent() == null) {
                logger3.log(Level.WARNING, "Linked bone {0} is a root bone.",
                        MyString.quote(boneName));
            }
        }
        for (String boneName : listAttachmentBoneNames()) {
            Bone bone = findBone(boneName);
            if (bone == null) {
                String msg = String.format(
                        "Attachment bone %s not found in skeleton.",
                        MyString.quote(boneName));
                throw new IllegalArgumentException(msg);
            }
        }
    }
}
