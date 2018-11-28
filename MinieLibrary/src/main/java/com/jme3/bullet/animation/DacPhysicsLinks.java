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
import com.jme3.animation.SkeletonControl;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.joints.SixDofJoint;
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
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Misc;
import jme3utilities.MySkeleton;
import jme3utilities.MySpatial;
import jme3utilities.MyString;
import jme3utilities.Validate;

/**
 * Access a DynamicAnimControl at the PhysicsLink level once it's been added to
 * a Spatial.
 *
 * @author Stephen Gold sgold@sonic.net
 *
 * Based on KinematicRagdollControl by Normen Hansen and Rémy Bouquet (Nehon).
 */
public class DacPhysicsLinks extends ConfigDynamicAnimControl {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger3
            = Logger.getLogger(DacPhysicsLinks.class.getName());
    /**
     * local copy of {@link com.jme3.math.Quaternion#IDENTITY}
     */
    final private static Quaternion rotateIdentity = new Quaternion();
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
     * skeleton being controlled
     */
    private Skeleton skeleton = null;
    /**
     * spatial that provides the mesh-coordinate transform
     */
    private Spatial transformer = null;
    /**
     * torso link for this control
     */
    private TorsoLink torsoLink = null;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled control without any linked bones or attachments
     * (torso only).
     */
    DacPhysicsLinks() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Access the AttachmentLink for the named bone. Returns null if the bone is
     * not associated with an attachment, or if the control is not added to a
     * spatial.
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
     * Access the named bone.
     * <p>
     * Allowed only when the control IS added to a spatial.
     *
     * @param boneName the name of the skeleton bone to access
     * @return the pre-existing instance, or null if not found
     */
    public Bone findBone(String boneName) {
        if (getSpatial() == null) {
            throw new IllegalStateException(
                    "Cannot access bones unless added to a spatial.");
        }

        Bone result = skeleton.getBone(boneName);
        return result;
    }

    /**
     * Access the BoneLink for the named bone. Returns null if bone is not
     * linked, or if the control is not added to a spatial.
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
     * control is not added to a spatial.
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
     * Access the skeleton. Returns null if the control is not added to a
     * spatial.
     *
     * @return the pre-existing skeleton, or null
     */
    Skeleton getSkeleton() {
        return skeleton;
    }

    /**
     * Access the TorsoLink. Returns null if the control is not added to a
     * spatial.
     *
     * @return the pre-existing TorsoLink, or null
     */
    public TorsoLink getTorsoLink() {
        return torsoLink;
    }

    /**
     * Access the spatial with the mesh-coordinate transform. Returns null if
     * the control is not added to a spatial.
     *
     * @return the pre-existing spatial, or null
     */
    Spatial getTransformer() {
        return transformer;
    }

    /**
     * Enumerate all managed bones of the named link, in a pre-order,
     * depth-first traversal of the skeleton, such that child bones never
     * precede their ancestors.
     *
     * @param managerName the name of the managing link (not null)
     * @return a new array of managed bones, including the manager if it is not
     * the torso
     */
    Bone[] listManagedBones(String managerName) {
        List<Bone> list = new ArrayList<>(8);

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
        /*
         * Convert the list to an array.
         */
        int numManagedBones = list.size();
        Bone[] array = new Bone[numManagedBones];
        list.toArray(array);

        return array;
    }

    /**
     * Enumerate all physics links of the specified type managed by this
     * control.
     *
     * @param <T> subclass of PhysicsLink
     * @param linkType the subclass of PhysicsLink to search for (not null)
     * @return a new array of links (not null, not empty)
     */
    @SuppressWarnings("unchecked")
    public <T extends PhysicsLink> List<T> listLinks(Class<T> linkType) {
        int numLinks = countLinks();
        List<T> result = new ArrayList<>(numLinks);

        if (linkType.isAssignableFrom(torsoLink.getClass())) {
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
     * Enumerate all the rigid bodies managed by this control.
     *
     * @return a new array of rigid bodies (not null, not empty)
     */
    public PhysicsRigidBody[] listRigidBodies() {
        int numLinks = countLinks();
        PhysicsRigidBody[] result = new PhysicsRigidBody[numLinks];

        result[0] = torsoLink.getRigidBody();
        int linkIndex = 1;
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
    Transform physicsTransform(Bone bone, Vector3f localOffset,
            Transform storeResult) {
        Transform result
                = (storeResult == null) ? new Transform() : storeResult;
        /*
         * Start with the body's transform in the bone's local coordinates.
         */
        result.setTranslation(localOffset);
        result.setRotation(rotateIdentity);
        result.setScale(1f);
        /*
         * Convert to mesh coordinates.
         */
        Transform localToMesh = MySkeleton.copyMeshTransform(bone, null);
        result.combineWithParent(localToMesh);
        /*
         * Convert to world (physics-space) coordinates.
         */
        Transform meshToWorld = meshTransform(null);
        result.combineWithParent(meshToWorld);

        return result;
    }

    /**
     * Rebuild the ragdoll. This is useful if you applied scale to the model
     * after it was initialized.
     * <p>
     * Allowed only when the control IS added to a spatial.
     */
    public void rebuild() {
        Spatial controlledSpatial = getSpatial();
        if (controlledSpatial == null) {
            throw new IllegalStateException(
                    "Cannot rebuild unless added to a spatial.");
        }

        Map<String, AttachmentLink> saveAttach = new HashMap<>(attachmentLinks);
        Map<String, BoneLink> saveBones = new HashMap<>(boneLinks);
        TorsoLink saveTorso = torsoLink;

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
        torsoLink.postRebuild(saveTorso);
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
     * Enumerate all attachment links.
     *
     * @return a collection view of values in the internal map (not null)
     */
    protected Collection<AttachmentLink> listAttachmentLinks() {
        Collection<AttachmentLink> result = attachmentLinks.values();
        return result;
    }
    // *************************************************************************
    // DacPhysicsLinks methods

    /**
     * Add all managed physics objects to the physics space.
     */
    @Override
    protected void addPhysics() {
        PhysicsSpace space = getPhysicsSpace();
        Vector3f gravity = gravity(null);

        PhysicsRigidBody rigidBody = torsoLink.getRigidBody();
        space.add(rigidBody);
        rigidBody.setGravity(gravity);

        for (BoneLink boneLink : boneLinkList) {
            rigidBody = boneLink.getRigidBody();
            space.add(rigidBody);
            rigidBody.setGravity(gravity);

            PhysicsJoint joint = boneLink.getJoint();
            space.add(joint);
        }

        for (AttachmentLink link : attachmentLinks.values()) {
            rigidBody = link.getRigidBody();
            space.add(rigidBody);
            rigidBody.setGravity(gravity);

            PhysicsJoint joint = link.getJoint();
            space.add(joint);
        }
    }

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned control into a deep-cloned one, using the specified cloner
     * and original to resolve copied fields.
     *
     * @param cloner the cloner that's cloning this control (not null, modified)
     * @param original the control from which this control was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);
        DacPhysicsLinks originalDac = (DacPhysicsLinks) original;

        boneLinkList = cloner.clone(boneLinkList);

        attachmentLinks = new HashMap<>(8);
        for (Map.Entry<String, AttachmentLink> entry
                : originalDac.attachmentLinks.entrySet()) {
            String boneName = entry.getKey();
            AttachmentLink link = entry.getValue();
            AttachmentLink copyLink = cloner.clone(link);
            attachmentLinks.put(boneName, copyLink);
        }

        boneLinks = new HashMap<>(32);
        for (Map.Entry<String, BoneLink> entry
                : originalDac.boneLinks.entrySet()) {
            String boneName = entry.getKey();
            BoneLink link = entry.getValue();
            BoneLink copyLink = cloner.clone(link);
            boneLinks.put(boneName, copyLink);
        }

        skeleton = cloner.clone(skeleton);
        transformer = cloner.clone(transformer);
        torsoLink = cloner.clone(torsoLink);
    }

    /**
     * Create spatial-dependent data. Invoked each time the control is added to
     * a spatial.
     *
     * @param spatial the controlled spatial (not null)
     */
    @Override
    protected void createSpatialData(Spatial spatial) {
        RagUtils.validate(spatial);

        SkeletonControl skeletonControl
                = spatial.getControl(SkeletonControl.class);
        if (skeletonControl == null) {
            throw new IllegalArgumentException(
                    "The controlled spatial must have a SkeletonControl. "
                    + "Make sure the control is there and not on a subnode.");
        }
        sortControls(skeletonControl);
        /*
         * Analyze the model's skeleton.
         */
        skeleton = skeletonControl.getSkeleton();
        validateSkeleton();
        String[] tempManagerMap = managerMap(skeleton);
        int numBones = skeleton.getBoneCount();
        /*
         * Temporarily set all bones' local translations and rotations to bind.
         */
        MySkeleton.setUserControl(skeleton, true);
        Transform[] savedTransforms = new Transform[numBones];
        Vector3f userScale = new Vector3f();
        for (int boneIndex = 0; boneIndex < numBones; boneIndex++) {
            Bone bone = skeleton.getBone(boneIndex);
            savedTransforms[boneIndex]
                    = MySkeleton.copyLocalTransform(bone, null);

            userScale.set(bone.getLocalScale());
            userScale.divideLocal(bone.getBindScale());
            bone.setUserTransforms(translateIdentity, rotateIdentity,
                    userScale);
        }
        MySkeleton.setUserControl(skeleton, false);
        skeleton.updateWorldVectors();
        /*
         * Find the target meshes and choose the transform spatial.
         * Don't invoke SkeletonControl.getTargets() here, since the
         * SkeletonControl might not be initialized yet.
         */
        List<Mesh> targetList = MySpatial.listAnimatedMeshes(spatial, null);
        Mesh[] targets = new Mesh[targetList.size()];
        targetList.toArray(targets);
        transformer = MySpatial.findAnimatedGeometry(spatial);
        if (transformer == null) {
            transformer = spatial;
        }
        /*
         * Enumerate mesh-vertex coordinates and assign them to managers.
         */
        Map<String, List<Vector3f>> coordsMap
                = RagUtils.coordsMap(targets, tempManagerMap);
        /*
         * Create the torso link.
         */
        List<Vector3f> vertexLocations = coordsMap.get(torsoName);
        createTorsoLink(vertexLocations, targets);
        /*
         * Create bone links without joints.
         */
        String[] linkedBoneNames = listLinkedBoneNames();
        for (String boneName : linkedBoneNames) {
            vertexLocations = coordsMap.get(boneName);
            createBoneLink(boneName, vertexLocations);
        }
        int numLinkedBones = countLinkedBones();
        assert boneLinks.size() == numLinkedBones;
        /*
         * Add joints to connect each BoneLink rigid body with its parent in the
         * link hierarchy.  Also initialize the boneLinkList.
         */
        boneLinkList = new ArrayList<>(numLinkedBones);
        addJoints(torsoLink);
        assert boneLinkList.size() == numLinkedBones : boneLinkList.size();
        /*
         * Create attachment links with joints.
         */
        String[] attachBoneNames = listAttachmentBoneNames();
        for (String boneName : attachBoneNames) {
            createAttachmentLink(boneName, skeletonControl, tempManagerMap);
        }
        /*
         * Restore the skeleton's pose.
         */
        for (int boneIndex = 0; boneIndex < numBones; boneIndex++) {
            Bone bone = skeleton.getBone(boneIndex);
            MySkeleton.setLocalTransform(bone, savedTransforms[boneIndex]);
        }
        skeleton.updateWorldVectors();

        if (added) {
            addPhysics();
        }

        logger3.log(Level.FINE, "Created ragdoll for skeleton.");
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public DacPhysicsLinks jmeClone() {
        try {
            DacPhysicsLinks clone = (DacPhysicsLinks) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }

    /**
     * Read the mass of the named bone/torso.
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
     * De-serialize this control, for example when loading from a J3O file.
     *
     * @param im importer (not null)
     * @throws IOException from importer
     */
    @Override
    @SuppressWarnings("unchecked")
    public void read(JmeImporter im) throws IOException {
        super.read(im);
        InputCapsule ic = im.getCapsule(this);

        boneLinkList
                = ic.readSavableArrayList("boneLinkList", null);
        for (BoneLink link : boneLinkList) {
            String name = link.boneName();
            boneLinks.put(name, link);
        }

        Savable[] savableArray
                = ic.readSavableArray("attachmentLinks", null);
        for (Savable savable : savableArray) {
            AttachmentLink link = (AttachmentLink) savable;
            String name = link.boneName();
            attachmentLinks.put(name, link);
        }

        skeleton = (Skeleton) ic.readSavable("skeleton", null);
        transformer = (Spatial) ic.readSavable("transformer", null);
        torsoLink = (TorsoLink) ic.readSavable("torsoLink", null);
    }

    /**
     * Remove all managed physics objects from the physics space.
     */
    @Override
    protected void removePhysics() {
        assert added;
        PhysicsSpace space = getPhysicsSpace();

        PhysicsRigidBody rigidBody = torsoLink.getRigidBody();
        space.remove(rigidBody);

        for (BoneLink boneLink : boneLinks.values()) {
            rigidBody = boneLink.getRigidBody();
            space.remove(rigidBody);

            PhysicsJoint joint = boneLink.getJoint();
            space.remove(joint);
        }

        for (AttachmentLink link : attachmentLinks.values()) {
            if (!link.isReleased()) {
                rigidBody = link.getRigidBody();
                space.remove(rigidBody);

                PhysicsJoint joint = link.getJoint();
                space.remove(joint);
            }
        }
    }

    /**
     * Remove spatial-dependent data. Invoked each time this control is rebuilt
     * or removed from a spatial.
     *
     * @param spat the previously controlled spatial (unused)
     */
    @Override
    protected void removeSpatialData(Spatial spat) {
        if (added) {
            removePhysics();
        }

        for (AttachmentLink attachmentLink : attachmentLinks.values()) {
            Bone bone = attachmentLink.getBone();
            Node node = MySkeleton.getAttachments(bone);
            MySkeleton.cancelAttachments(bone);
            node.removeFromParent();
        }
        attachmentLinks.clear();

        MySkeleton.setUserControl(skeleton, false);
        skeleton = null;

        boneLinks.clear();
        boneLinkList = null;
        torsoLink = null;
        transformer = null;
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
     * coordinates, not null, unaffected, default=0,-9.8,0)
     */
    @Override
    public void setGravity(Vector3f gravity) {
        Validate.nonNull(gravity, "gravity");

        super.setGravity(gravity);

        if (getSpatial() != null) { // TODO make sure it's in ragdoll mode
            PhysicsRigidBody[] bodies = listRigidBodies();
            for (PhysicsRigidBody rigidBody : bodies) {
                rigidBody.setGravity(gravity);
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
            SixDofJoint joint = (SixDofJoint) boneLink.getJoint();
            rom.setupJoint(joint, false, false, false);
        }
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
     * @param vec desired location (not null, unaffected)
     */
    @Override
    protected void setPhysicsLocation(Vector3f vec) {
        torsoLink.getRigidBody().setPhysicsLocation(vec);
    }

    /**
     * Rotate the torso to the specified orientation.
     *
     * @param quat desired orientation (not null, unaffected)
     */
    @Override
    protected void setPhysicsRotation(Quaternion quat) {
        torsoLink.getRigidBody().setPhysicsRotation(quat);
    }

    /**
     * Update this control. Invoked once per frame during the logical-state
     * update, provided the control is added to a scene. Do not invoke directly
     * from user code.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void update(float tpf) {
        assert getSpatial() != null;
        if (!isEnabled()) {
            return;
        }

        torsoLink.update(tpf);
        for (BoneLink boneLink : boneLinkList) {
            boneLink.update(tpf);
        }
        for (AttachmentLink link : attachmentLinks.values()) {
            link.update(tpf);
        }
    }

    /**
     * Serialize this control, for example when saving to a J3O file.
     *
     * @param ex exporter (not null)
     * @throws IOException from exporter
     */
    @Override
    public void write(JmeExporter ex) throws IOException {
        super.write(ex);
        OutputCapsule oc = ex.getCapsule(this);

        int count = countLinkedBones();
        Savable[] savableArray = new Savable[count];
        boneLinkList.toArray(savableArray);
        oc.write(savableArray, "boneLinkList", null);

        count = countAttachments();
        AttachmentLink[] links = new AttachmentLink[count];
        attachmentLinks.values().toArray(links);
        oc.write(links, "attachmentLinks", new AttachmentLink[0]);

        oc.write(skeleton, "skeleton", null);
        oc.write(transformer, "transformer", null);
        oc.write(torsoLink, "torsoLink", null);
    }
    // *************************************************************************
    // private methods

    /**
     * Add joints to connect the named bone/torso link with each of its
     * children. Also fill in the boneLinkList. Note: recursive!
     *
     * @param parentName the parent bone/torso link (not null)
     */
    private void addJoints(PhysicsLink parentLink) {
        List<String> childNames = childNames(parentLink);
        for (String childName : childNames) {
            /*
             * Add the joint and configure its range of motion.
             * Also initialize the BoneLink's parent and its array
             * of managed bones.
             */
            BoneLink childLink = findBoneLink(childName);
            childLink.addJoint(parentLink);
            /*
             * Add the BoneLink to the pre-order list.
             */
            boneLinkList.add(childLink);

            addJoints(childLink);
        }
    }

    /**
     * Enumerate all immediate child BoneLinks of the specified bone/torso link.
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
            Bone bone = findBone(childName);
            Bone parent = bone.getParent();
            if (parent != null && findManager(parent).equals(linkName)) {
                result.add(childName);
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
     * @return a attachment link with a joint, added to the boneLinks map
     */
    private void createAttachmentLink(String boneName,
            SkeletonControl skeletonControl, String[] managerMap) {
        assert boneName != null;
        assert skeletonControl != null;
        assert managerMap != null;

        Spatial attachModel = getAttachmentModel(boneName);
        attachModel = (Spatial) Misc.deepCopy(attachModel);
        List<Vector3f> vertexLocations
                = RagUtils.vertexLocations(attachModel, null);

        Node node = skeletonControl.getAttachmentsNode(boneName);
        node.attachChild(attachModel);

        Bone bone = skeleton.getBone(boneName);
        int boneIndex = skeleton.getBoneIndex(bone);
        String managerName = managerMap[boneIndex];
        PhysicsLink manager;
        if (managerName.equals(torsoName)) {
            manager = torsoLink;
        } else {
            manager = boneLinks.get(managerName);
        }
        /*
         * Create the collision shape.
         */
        LinkConfig linkConfig = attachmentConfig(boneName);
        CenterHeuristic centerHeuristic = linkConfig.centerHeuristic();
        assert centerHeuristic != CenterHeuristic.Joint;
        Vector3f center = centerHeuristic.center(vertexLocations, null);
        CollisionShape shape = linkConfig.createShape(transformIdentity,
                center, vertexLocations);
        PhysicsRigidBody rigidBody = createRigidBody(linkConfig, shape);

        AttachmentLink link = new AttachmentLink(this, bone, manager,
                attachModel, rigidBody, center);
        attachmentLinks.put(boneName, link);
    }

    /**
     * Create a jointless BoneLink for the named bone, and add it to the
     * boneLinks map.
     *
     * @param boneName the name of the bone to be linked (not null)
     * @param vertexLocations the collection of vertex locations (not null, not
     * empty)
     */
    private void createBoneLink(String boneName,
            List<Vector3f> vertexLocations) {
        if (vertexLocations == null || vertexLocations.isEmpty()) {
            String msg = String.format("No mesh vertices for linked bone %s.",
                    MyString.quote(boneName));
            throw new IllegalArgumentException(msg);
        }
        /*
         * Create the collision shape.
         */
        Bone bone = findBone(boneName);
        Transform boneToMesh = MySkeleton.copyMeshTransform(bone, null);
        Transform meshToBone = boneToMesh.invert();
        LinkConfig linkConfig = config(boneName);
        CenterHeuristic centerHeuristic = linkConfig.centerHeuristic();
        Vector3f center;
        if (centerHeuristic == CenterHeuristic.Joint) {
            center = translateIdentity;
        } else {
            center = centerHeuristic.center(vertexLocations, null);
            center.subtractLocal(bone.getModelSpacePosition());
        }
        CollisionShape shape = linkConfig.createShape(meshToBone, center,
                vertexLocations);

        PhysicsRigidBody rigidBody = createRigidBody(linkConfig, shape);

        meshToBone.getTranslation().zero();
        Vector3f offset = meshToBone.transformVector(center, null);

        BoneLink link = new BoneLink(this, bone, rigidBody, offset);
        boneLinks.put(boneName, link);
    }

    /**
     * Create and configure a rigid body for a link.
     *
     * @param linkConfig the link configuration (not null)
     * @param collisionShape the desired shape (not null, alias created)
     * @return a new instance, not in any physics space
     */
    private PhysicsRigidBody createRigidBody(LinkConfig linkConfig,
            CollisionShape collisionShape) {
        Validate.nonNull(collisionShape, "collision shape");

        float mass = linkConfig.mass(collisionShape);
        PhysicsRigidBody rigidBody = new PhysicsRigidBody(collisionShape, mass);

        float viscousDamping = damping();
        rigidBody.setDamping(viscousDamping, viscousDamping);

        return rigidBody;
    }

    /**
     * Create the TorsoLink.
     *
     * @param vertexLocations the collection of vertex locations (not null, not
     * empty)
     * @param meshes array of animated meshes to use (not null, unaffected)
     */
    private void createTorsoLink(Collection<Vector3f> vertexLocations,
            Mesh[] meshes) {
        if (vertexLocations == null) {
            throw new IllegalArgumentException(
                    "No mesh vertices for the torso."
                    + " Make sure the root bone is not linked.");
        }
        /*
         * Create the collision shape.
         */
        Bone bone = RagUtils.findMainBone(skeleton, meshes);
        assert bone.getParent() == null;
        Transform boneToMesh = MySkeleton.copyMeshTransform(bone, null);
        Transform meshToBone = boneToMesh.invert();
        LinkConfig linkConfig = config(torsoName);
        CenterHeuristic centerHeuristic = linkConfig.centerHeuristic();
        assert centerHeuristic != CenterHeuristic.Joint;
        Vector3f center = centerHeuristic.center(vertexLocations, null);
        center.subtractLocal(bone.getModelSpacePosition());
        CollisionShape shape = linkConfig.createShape(meshToBone, center,
                vertexLocations);

        meshToBone.getTranslation().zero();
        Vector3f offset = meshToBone.transformVector(center, null);

        PhysicsRigidBody rigidBody = createRigidBody(linkConfig, shape);

        Node modelNode = (Node) getSpatial();
        Transform modelToMesh
                = RagUtils.relativeTransform(transformer, modelNode, null);
        Transform meshToModel = modelToMesh.invert();

        torsoLink = new TorsoLink(this, bone, rigidBody, meshToModel, offset);
    }

    /**
     * Sort the controls of the controlled spatial, such that this control will
     * come BEFORE the specified SkeletonControl.
     *
     * @param skeletonControl (not null)
     */
    private void sortControls(SkeletonControl skeletonControl) {
        assert skeletonControl != null;

        Spatial spatial = getSpatial();
        int dacIndex = MySpatial.findIndex(spatial, this);
        assert dacIndex != -1;
        int scIndex = MySpatial.findIndex(spatial, skeletonControl);
        assert scIndex != -1;
        assert dacIndex != scIndex;

        if (dacIndex > scIndex) {
            /*
             * Remove the SkeletonControl and re-add it to make sure it will get
             * updated *after* this control. TODO also arrange with AnimControl
             */
            spatial.removeControl(skeletonControl);
            spatial.addControl(skeletonControl);

            dacIndex = MySpatial.findIndex(spatial, this);
            assert dacIndex != -1;
            scIndex = MySpatial.findIndex(spatial, skeletonControl);
            assert scIndex != -1;
            assert dacIndex < scIndex;
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
