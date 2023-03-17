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
import com.jme3.animation.Bone;
import com.jme3.animation.Skeleton;
import com.jme3.bullet.control.AbstractPhysicsControl;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.math.Vector3f;
import com.jme3.scene.Spatial;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.MySpatial;
import jme3utilities.MyString;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * Configure a DynamicAnimControl and access its configuration.
 *
 * @author Stephen Gold sgold@sonic.net
 *
 * Based on KinematicRagdollControl by Normen Hansen and Rémy Bouquet (Nehon).
 */
abstract public class DacConfiguration extends AbstractPhysicsControl {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(DacConfiguration.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagAlConfigs = "alConfigs";
    final private static String tagAttachBoneNames = "attachBoneNames";
    final private static String tagAttachModels = "attachModels";
    final private static String tagBlConfigs = "blConfigs";
    final private static String tagDamping = "damping";
    final private static String tagEventDispatchImpulseThreshold
            = "eventDispatchImpulseThreshold";
    final private static String tagGravity = "gravity";
    final private static String tagIgnoredHops = "ignoredHops";
    final private static String tagLinkedBoneJoints = "linkedBoneJoints";
    final private static String tagLinkedBoneNames = "linkedBoneNames";
    final private static String tagMainBoneName = "mainBoneName";
    final private static String tagRelativeTolerance = "relativeTolerance";
    final private static String tagTorsoConfig = "torsoConfig";
    /**
     * name for the ragdoll's torso, must not be used for any bone
     */
    final public static String torsoName = "";
    // *************************************************************************
    // fields

    /**
     * viscous damping ratio for new rigid bodies (0&rarr;no damping,
     * 1&rarr;critically damped)
     */
    private float damping = 0.6f;
    /**
     * minimum applied impulse for a collision event to be dispatched to
     * listeners
     */
    private float eventDispatchImpulseThreshold = 0f;
    /**
     * relative tolerance for comparing scale factors
     */
    private float relativeTolerance = 0.001f;
    /**
     * maximum number of physics-joint hops across which bodies ignore
     * collisions
     */
    private int ignoredHops = 1;
    /**
     * configuration data for the torso
     */
    private LinkConfig torsoConfig = new LinkConfig();
    /**
     * map attachment bone names to configuration data
     */
    private Map<String, LinkConfig> alConfigMap = new HashMap<>(5);
    /**
     * map linked bone names to configuration data
     */
    private Map<String, LinkConfig> blConfigMap = new HashMap<>(50);
    /**
     * map linked bone names to ranges of motion for createSpatialData()
     */
    private Map<String, RangeOfMotion> jointMap = new HashMap<>(50);
    /**
     * map attachment bone names to models for createSpatialData()
     */
    private Map<String, Spatial> attachModelMap = new HashMap<>(5);
    /**
     * name of the torso's main bone, or null if it hasn't been determined yet
     */
    private String mainBoneName = null;
    /**
     * gravitational acceleration vector for ragdolls
     */
    private Vector3f gravityVector = new Vector3f(0f, -9.8f, 0f);
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled Control without any attachments or linked bones
     * (torso only).
     */
    protected DacConfiguration() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Test whether 2 scale vectors are equal within the tolerance set for this
     * control.
     *
     * @param scale1 the first scale vector (not null, unaffected)
     * @param scale2 the 2nd scale vector (not null, unaffected)
     * @return true if within tolerance, otherwise false
     */
    boolean areWithinTolerance(Vector3f scale1, Vector3f scale2) {
        boolean result = MyVector3f.areWithinTolerance(
                scale1, scale2, relativeTolerance);
        return result;
    }

    /**
     * Configure the specified model as an attachment.
     *
     * @param boneName the name of the associated bone (not null, not empty)
     * @param mass the desired mass of the model (&gt;0)
     * @param model the model to attach (not null, orphan, alias created)
     */
    public void attach(String boneName, float mass, Spatial model) {
        Validate.nonEmpty(boneName, "bone name");
        Validate.positive(mass, "mass");
        RagUtils.validate(model);
        assert MySpatial.isOrphan(model);
        verifyNotAddedToSpatial("add an attachment");
        if (hasAttachmentLink(boneName)) {
            logger2.log(Level.WARNING, "Bone {0} already had an attachment.",
                    MyString.quote(boneName));
        }

        attachModelMap.put(boneName, model);
        LinkConfig config = new LinkConfig(mass);
        alConfigMap.put(boneName, config);
    }

    /**
     * Configure the specified model as an attachment.
     *
     * @param boneName the name of the associated bone (not null, not empty)
     * @param config the desired configuration (not null)
     * @param model the model to attach (not null, orphan, alias created)
     */
    public void attach(String boneName, LinkConfig config, Spatial model) {
        Validate.nonEmpty(boneName, "bone name");
        Validate.nonNull(config, "configuration");
        if (config.centerHeuristic() == CenterHeuristic.Joint) {
            throw new IllegalArgumentException(
                    "Cannot center attachment on Joint.");
        }
        RagUtils.validate(model);
        assert MySpatial.isOrphan(model);
        verifyNotAddedToSpatial("add an attachment");
        if (hasAttachmentLink(boneName)) {
            logger2.log(Level.WARNING, "Bone {0} already had an attachment.",
                    MyString.quote(boneName));
        }

        attachModelMap.put(boneName, model);
        alConfigMap.put(boneName, config);
    }

    /**
     * Access the configuration of the attachment link associated with the named
     * bone.
     *
     * @param boneName the name of the associated bone (not null, not empty)
     * @return the pre-existing configuration (not null)
     */
    public LinkConfig attachmentConfig(String boneName) {
        if (alConfigMap.containsKey(boneName)) {
            LinkConfig config = alConfigMap.get(boneName);
            assert config != null;
            return config;
        } else {
            String msg = "No attachment link for " + MyString.quote(boneName);
            throw new IllegalArgumentException(msg);
        }
    }

    /**
     * Read the mass of the attachment associated with the named bone.
     *
     * @param boneName the name of the associated bone (not null, not empty)
     * @return the mass (in physics units, &gt;0) or NaN if undetermined
     */
    public float attachmentMass(String boneName) {
        LinkConfig config = attachmentConfig(boneName);
        float mass = config.mass();
        return mass;
    }

    /**
     * Access the configuration of the named bone/torso.
     *
     * @param boneName the name of the bone/torso (not null)
     * @return the pre-existing configuration (not null)
     */
    public LinkConfig config(String boneName) {
        LinkConfig result;

        if (torsoName.equals(boneName)) {
            result = torsoConfig;
        } else if (hasBoneLink(boneName)) {
            result = blConfigMap.get(boneName);
        } else {
            String msg = "No bone/torso named " + MyString.quote(boneName);
            throw new IllegalArgumentException(msg);
        }

        assert result != null;
        return result;
    }

    /**
     * Count the attachments.
     *
     * @return count (&ge;0)
     */
    public int countAttachments() {
        int count = alConfigMap.size();

        assert count == attachModelMap.size();
        assert count >= 0 : count;
        return count;
    }

    /**
     * Count the linked bones.
     *
     * @return count (&ge;0)
     */
    public int countLinkedBones() {
        int count = blConfigMap.size();

        assert count == jointMap.size();
        assert count >= 0 : count;
        return count;
    }

    /**
     * Count the links.
     *
     * @return count (&ge;0)
     */
    public int countLinks() {
        int result = countLinkedBones() + countAttachments() + 1;
        return result;
    }

    /**
     * Read the damping ratio for new rigid bodies.
     *
     * @return the viscous damping ratio (0&rarr;no damping, 1&rarr;critically
     * damped)
     */
    public float damping() {
        assert damping >= 0f : damping;
        return damping;
    }

    /**
     * Unlink the AttachmentLink associated with the named bone.
     * <p>
     * Allowed only when the control is NOT added to a Spatial.
     *
     * @param boneName the name of the associated bone (not null, not empty)
     */
    public void detach(String boneName) {
        if (!hasAttachmentLink(boneName)) {
            String msg = "No attachment bone named " + MyString.quote(boneName);
            throw new IllegalArgumentException(msg);
        }
        verifyNotAddedToSpatial("unlink an attachment");

        alConfigMap.remove(boneName);
        attachModelMap.remove(boneName);
    }

    /**
     * Read the event-dispatch impulse threshold of this control.
     *
     * @return the threshold value (&ge;0)
     */
    public float eventDispatchImpulseThreshold() {
        assert eventDispatchImpulseThreshold >= 0f;
        return eventDispatchImpulseThreshold;
    }

    /**
     * Access the model attached to the named bone.
     *
     * @param boneName the name of the associated bone (not null, not empty)
     * @return the orphan spatial (not null)
     */
    public Spatial getAttachmentModel(String boneName) {
        Spatial model;

        if (attachModelMap.containsKey(boneName)) {
            model = attachModelMap.get(boneName);
        } else {
            String msg = "No attachment link for " + MyString.quote(boneName);
            throw new IllegalArgumentException(msg);
        }

        assert model != null;
        assert MySpatial.isOrphan(model);
        return model;
    }

    /**
     * Access the nominal range of motion for the joint connecting the named
     * linked bone to its parent in the hierarchy.
     *
     * @param boneName the name of the linked bone (not null, not empty)
     * @return the pre-existing instance (not null)
     */
    public RangeOfMotion getJointLimits(String boneName) {
        if (!hasBoneLink(boneName)) {
            String msg = "No linked bone named " + MyString.quote(boneName);
            throw new IllegalArgumentException(msg);
        }
        RangeOfMotion result = jointMap.get(boneName);

        assert result != null;
        return result;
    }

    /**
     * Copy this control's gravitational acceleration for Ragdoll mode.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return an acceleration vector (in physics-space coordinates, either
     * storeResult or a new vector, not null)
     */
    public Vector3f gravity(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        result.set(gravityVector);
        return result;
    }

    /**
     * Test whether an AttachmentLink exists for the named bone.
     *
     * @param boneName the name of the associated bone (may be null)
     * @return true if found, otherwise false
     */
    public boolean hasAttachmentLink(String boneName) {
        boolean result;
        if (boneName == null) {
            result = false;
        } else {
            result = alConfigMap.containsKey(boneName);
        }

        return result;
    }

    /**
     * Test whether a BoneLink exists for the named bone.
     *
     * @param boneName the name of the bone (may be null)
     * @return true if found, otherwise false
     */
    public boolean hasBoneLink(String boneName) {
        boolean result;
        if (boneName == null) {
            result = false;
        } else {
            result = blConfigMap.containsKey(boneName);
        }

        return result;
    }

    /**
     * Determine the maximum number of physics-joint hops across which bodies
     * ignore collisions.
     *
     * @return the number of hops (&ge;0)
     */
    public int ignoredHops() {
        assert ignoredHops >= 0 : ignoredHops;
        return ignoredHops;
    }

    /**
     * Link the named bone using the specified mass and range of motion.
     * <p>
     * Allowed only when the control is NOT added to a Spatial.
     *
     * @param boneName the name of the bone to link (not null, not empty)
     * @param mass the desired mass of the bone (&gt;0)
     * @param rom the desired range of motion (not null)
     * @see #setJointLimits(java.lang.String,
     * com.jme3.bullet.animation.RangeOfMotion)
     */
    public void link(String boneName, float mass, RangeOfMotion rom) {
        Validate.nonEmpty(boneName, "bone name");
        Validate.positive(mass, "mass");
        Validate.nonNull(rom, "range of motion");
        verifyNotAddedToSpatial("link a bone");
        if (hasBoneLink(boneName)) {
            logger2.log(Level.WARNING, "Bone {0} is already linked.",
                    MyString.quote(boneName));
        }

        jointMap.put(boneName, rom);
        LinkConfig config = new LinkConfig(mass);
        blConfigMap.put(boneName, config);
    }

    /**
     * Link the named bone using the specified configuration and range of
     * motion.
     * <p>
     * Allowed only when the control is NOT added to a Spatial.
     *
     * @param boneName the name of the bone to link (not null, not empty)
     * @param config the desired configuration (not null)
     * @param rom the desired range of motion (not null)
     * @see #setJointLimits(java.lang.String,
     * com.jme3.bullet.animation.RangeOfMotion)
     */
    public void link(String boneName, LinkConfig config, RangeOfMotion rom) {
        Validate.nonEmpty(boneName, "bone name");
        Validate.nonNull(config, "configuration");
        Validate.nonNull(rom, "range of motion");
        verifyNotAddedToSpatial("link a bone");
        if (hasBoneLink(boneName)) {
            logger2.log(Level.WARNING, "Bone {0} is already linked.",
                    MyString.quote(boneName));
        }

        jointMap.put(boneName, rom);
        blConfigMap.put(boneName, config);
    }

    /**
     * Enumerate bones with attachment links.
     *
     * @return a new array of bone names (not null, may be empty)
     */
    public String[] listAttachmentBoneNames() {
        int size = countAttachments();
        String[] result = new String[size];
        Collection<String> names = alConfigMap.keySet();
        names.toArray(result);

        return result;
    }

    /**
     * Enumerate bones with bone links.
     *
     * @return a new array of bone names (not null, may be empty)
     */
    public String[] listLinkedBoneNames() {
        int size = countLinkedBones();
        String[] result = new String[size];
        Collection<String> names = blConfigMap.keySet();
        names.toArray(result);

        return result;
    }

    /**
     * Return the name of the main bone.
     *
     * @return the name of the bone, or null if the main bone hasn't been
     * determined yet
     */
    public String mainBoneName() {
        return mainBoneName;
    }

    /**
     * Read the mass of the named bone/torso.
     *
     * @param boneName the name of the bone/torso (not null)
     * @return the mass (in physics units, &gt;0) or NaN if undetermined
     */
    public float mass(String boneName) {
        LinkConfig config = config(boneName);
        float mass = config.mass();
        return mass;
    }

    /**
     * Return the relative tolerance for comparing scale factors.
     *
     * @return the relative tolerance (&ge;0)
     */
    public float relativeTolerance() {
        assert relativeTolerance >= 0f;
        return relativeTolerance;
    }

    /**
     * Alter the configuration of the attachment associated with the named bone.
     *
     * @param boneName the name of the associated bone (not null, not empty)
     * @param config the desired configuration (not null)
     */
    public void setAttachmentConfig(String boneName, LinkConfig config) {
        Validate.nonNull(config, "configuration");

        if (alConfigMap.containsKey(boneName)) {
            alConfigMap.put(boneName, config);
        } else {
            String msg = "No attachment link for " + MyString.quote(boneName);
            throw new IllegalArgumentException(msg);
        }
    }

    /**
     * Alter the mass of the attachment associated with the named bone.
     *
     * @param boneName the name of the associated bone (not null, not empty)
     * @param mass the desired mass (&gt;0)
     */
    public void setAttachmentMass(String boneName, float mass) {
        Validate.positive(mass, "mass");

        if (alConfigMap.containsKey(boneName)) {
            LinkConfig config = alConfigMap.get(boneName);
            config = new LinkConfig(mass, config);
            alConfigMap.put(boneName, config);
        } else {
            String msg = "No attachment link for " + MyString.quote(boneName);
            throw new IllegalArgumentException(msg);
        }
    }

    /**
     * Alter the configuration of the named bone/torso.
     *
     * @param boneName the name of the bone, or torsoName (not null)
     * @param config the desired configuration (not null)
     */
    public void setConfig(String boneName, LinkConfig config) {
        Validate.nonNull(config, "configuration");

        if (torsoName.equals(boneName)) {
            if (config.centerHeuristic() == CenterHeuristic.Joint) {
                throw new IllegalArgumentException(
                        "Cannot center torso on Joint.");
            }
            torsoConfig = config;
        } else if (hasBoneLink(boneName)) {
            blConfigMap.put(boneName, config);
        } else {
            String msg = "No bone/torso named " + MyString.quote(boneName);
            throw new IllegalArgumentException(msg);
        }
    }

    /**
     * Alter the viscous damping ratio for new rigid bodies.
     *
     * @param dampingRatio the desired damping ratio (non-negative, 0&rarr;no
     * damping, 1&rarr;critically damped, default=0.6)
     */
    public void setDamping(float dampingRatio) {
        Validate.nonNegative(dampingRatio, "damping ratio");
        damping = dampingRatio;
    }

    /**
     * Alter the event-dispatch impulse threshold of this control.
     *
     * @param threshold the desired threshold (&ge;0)
     */
    public void setEventDispatchImpulseThreshold(float threshold) {
        Validate.nonNegative(threshold, "threshold");
        eventDispatchImpulseThreshold = threshold;
    }

    /**
     * Alter this control's gravitational acceleration for Ragdoll mode.
     *
     * @param gravity the desired acceleration vector (in physics-space
     * coordinates, not null, unaffected, default=(0,-9.8,0))
     */
    public void setGravity(Vector3f gravity) {
        Validate.finite(gravity, "gravity");
        gravityVector.set(gravity);
    }

    /**
     * Alter the maximum number of physics-joint hops across which bodies will
     * ignore collisions.
     * <p>
     * Allowed only when the control is NOT added to a Spatial.
     *
     * @param numHops the desired number of hops (&ge;0, default=1)
     */
    public void setIgnoredHops(int numHops) {
        Validate.nonNegative(numHops, "number of hops");
        verifyNotAddedToSpatial("alter ignored hops");

        ignoredHops = numHops;
    }

    /**
     * Alter the range of motion of the joint connecting the named BoneLink to
     * its parent in the link hierarchy.
     *
     * @param boneName the name of the BoneLink (not null, not empty)
     * @param rom the desired range of motion (not null)
     */
    public void setJointLimits(String boneName, RangeOfMotion rom) {
        Validate.nonNull(rom, "range of motion");
        if (!hasBoneLink(boneName)) {
            String msg = "No linked bone named " + MyString.quote(boneName);
            throw new IllegalArgumentException(msg);
        }

        jointMap.put(boneName, rom);
    }

    /**
     * Specify the main bone.
     *
     * @param boneName the name of the desired bone, or null to determine the
     * main bone heuristically when the control is added to a spatial
     */
    public void setMainBoneName(String boneName) {
        this.mainBoneName = boneName;
    }

    /**
     * Alter the mass of the named bone/torso.
     *
     * @param boneName the name of the bone, or torsoName (not null)
     * @param mass the desired mass (&gt;0)
     */
    public void setMass(String boneName, float mass) {
        Validate.positive(mass, "mass");

        if (torsoName.equals(boneName)) {
            torsoConfig = new LinkConfig(mass, torsoConfig);
        } else if (hasBoneLink(boneName)) {
            LinkConfig config = blConfigMap.get(boneName);
            config = new LinkConfig(mass, config);
            blConfigMap.put(boneName, config);
        } else {
            String msg = "No bone/torso named " + MyString.quote(boneName);
            throw new IllegalArgumentException(msg);
        }
    }

    /**
     * Alter the relative tolerance for comparing scale factors.
     *
     * @param newTolerance the desired value (&ge;0, default=0.001)
     */
    public void setRelativeTolerance(float newTolerance) {
        Validate.nonNegative(newTolerance, "new tolerance");
        this.relativeTolerance = newTolerance;
    }

    /**
     * Calculate the ragdoll's total mass, including attachments.
     *
     * @return the total mass (&gt;0) or NaN if undetermined
     */
    public float totalMass() {
        float totalMass = torsoConfig.mass();
        for (LinkConfig config : blConfigMap.values()) {
            totalMass += config.mass();
        }
        for (LinkConfig config : alConfigMap.values()) {
            totalMass += config.mass();
        }

        return totalMass;
    }

    /**
     * Unlink the BoneLink of the named bone.
     * <p>
     * Allowed only when the control is NOT added to a Spatial.
     *
     * @param boneName the name of the bone to unlink (not null, not empty)
     */
    public void unlinkBone(String boneName) {
        if (!hasBoneLink(boneName)) {
            String msg = "No linked bone named " + MyString.quote(boneName);
            throw new IllegalArgumentException(msg);
        }
        verifyNotAddedToSpatial("unlink a bone");

        jointMap.remove(boneName);
        blConfigMap.remove(boneName);
    }
    // *************************************************************************
    // new protected methods

    /**
     * Add unlinked descendants of the specified Bone to the specified
     * collection. Note: recursive!
     *
     * @param startBone the starting Bone (not null, aliases created)
     * @param addResult the collection of skeleton bones to append to (not null,
     * modified)
     */
    protected void addUnlinkedDescendants(
            Bone startBone, Collection<Bone> addResult) {
        for (Bone child : startBone.getChildren()) {
            String childName = child.getName();
            if (!hasBoneLink(childName)) {
                addResult.add(child);
                addUnlinkedDescendants(child, addResult);
            }
        }
    }

    /**
     * Add unlinked descendants of the specified Joint to the specified
     * collection. Note: recursive!
     *
     * @param startJoint the starting Joint (not null, aliases created)
     * @param addResult the collection of armature joints to append to (not
     * null, modified)
     */
    protected void addUnlinkedDescendants(
            Joint startJoint, Collection<Joint> addResult) {
        for (Joint child : startJoint.getChildren()) {
            String childName = child.getName();
            if (!hasBoneLink(childName)) {
                addResult.add(child);
                addUnlinkedDescendants(child, addResult);
            }
        }
    }

    /**
     * Find the manager of the specified Bone.
     *
     * @param startBone the bone (not null, unaffected)
     * @return a bone/torso name (not null)
     */
    protected String findManager(Bone startBone) {
        Validate.nonNull(startBone, "start bone");

        String managerName;
        Bone bone = startBone;
        while (true) {
            String boneName = bone.getName();
            if (hasBoneLink(boneName)) {
                managerName = boneName;
                break;
            }
            bone = bone.getParent();
            if (bone == null) {
                managerName = torsoName;
                break;
            }
        }

        assert managerName != null;
        return managerName;
    }

    /**
     * Find the manager of the specified armature joint.
     *
     * @param startJoint the joint (not null, unaffected)
     * @return a joint/torso name (not null)
     */
    protected String findManager(Joint startJoint) {
        Validate.nonNull(startJoint, "start joint");

        String managerName;
        Joint joint = startJoint;
        while (true) {
            String jointName = joint.getName();
            if (hasBoneLink(jointName)) {
                managerName = jointName;
                break;
            }
            joint = joint.getParent();
            if (joint == null) {
                managerName = torsoName;
                break;
            }
        }

        assert managerName != null;
        return managerName;
    }

    /**
     * Create a map from joint indices to the names of the armature joints that
     * manage them.
     *
     * @param armature (not null, unaffected)
     * @return a new array of joint/torso names (not null)
     */
    protected String[] managerMap(Armature armature) {
        int numJoints = armature.getJointCount();
        String[] managerMap = new String[numJoints];
        for (int jointIndex = 0; jointIndex < numJoints; ++jointIndex) {
            Joint joint = armature.getJoint(jointIndex);
            managerMap[jointIndex] = findManager(joint);
        }

        return managerMap;
    }

    /**
     * Create a map from bone indices to the names of the bones that manage
     * them.
     *
     * @param skeleton (not null, unaffected)
     * @return a new array of bone/torso names (not null)
     */
    protected String[] managerMap(Skeleton skeleton) {
        int numBones = skeleton.getBoneCount();
        String[] managerMap = new String[numBones];
        for (int boneIndex = 0; boneIndex < numBones; ++boneIndex) {
            Bone bone = skeleton.getBone(boneIndex);
            managerMap[boneIndex] = findManager(bone);
        }

        return managerMap;
    }
    // *************************************************************************
    // AbstractPhysicsControl methods

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

        this.alConfigMap = cloner.clone(alConfigMap);
        this.blConfigMap = cloner.clone(blConfigMap);
        this.jointMap = cloner.clone(jointMap);

        this.attachModelMap = new HashMap<>(5);
        DacConfiguration originalDc = (DacConfiguration) original;
        for (Map.Entry<String, Spatial> entry
                : originalDc.attachModelMap.entrySet()) {
            String boneName = entry.getKey();
            Spatial spatial = entry.getValue();
            Spatial copySpatial = cloner.clone(spatial);
            attachModelMap.put(boneName, copySpatial);
        }

        this.gravityVector = cloner.clone(gravityVector);
    }

    /**
     * De-serialize this Control from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);
        InputCapsule capsule = importer.getCapsule(this);

        this.ignoredHops = capsule.readInt(tagIgnoredHops, 1);
        this.damping = capsule.readFloat(tagDamping, 0.6f);
        this.eventDispatchImpulseThreshold
                = capsule.readFloat(tagEventDispatchImpulseThreshold, 0f);

        jointMap.clear();
        blConfigMap.clear();
        String[] linkedBoneNames
                = capsule.readStringArray(tagLinkedBoneNames, null);
        Savable[] linkedBoneJoints
                = capsule.readSavableArray(tagLinkedBoneJoints, null);
        Savable[] blConfigs = capsule.readSavableArray(tagBlConfigs, null);
        for (int i = 0; i < linkedBoneNames.length; ++i) {
            String boneName = linkedBoneNames[i];
            RangeOfMotion rom = (RangeOfMotion) linkedBoneJoints[i];
            jointMap.put(boneName, rom);
            blConfigMap.put(boneName, (LinkConfig) blConfigs[i]);
        }

        this.mainBoneName = capsule.readString(tagMainBoneName, null);

        attachModelMap.clear();
        alConfigMap.clear();
        String[] attachBoneNames
                = capsule.readStringArray(tagAttachBoneNames, null);
        Savable[] attachModels
                = capsule.readSavableArray(tagAttachModels, null);
        Savable[] alConfigs = capsule.readSavableArray(tagAlConfigs, null);
        for (int i = 0; i < attachBoneNames.length; ++i) {
            String boneName = attachBoneNames[i];
            Spatial model = (Spatial) attachModels[i];
            attachModelMap.put(boneName, model);
            alConfigMap.put(boneName, (LinkConfig) alConfigs[i]);
        }

        this.torsoConfig
                = (LinkConfig) capsule.readSavable(tagTorsoConfig, null);
        this.gravityVector = (Vector3f) capsule.readSavable(tagGravity, null);
        this.relativeTolerance
                = capsule.readFloat(tagRelativeTolerance, 0.001f);
    }

    /**
     * Alter whether physics-space coordinates should match the spatial's local
     * coordinates.
     *
     * @param applyPhysicsLocal true&rarr;match local coordinates,
     * false&rarr;match world coordinates (default=false)
     */
    @Override
    public void setApplyPhysicsLocal(boolean applyPhysicsLocal) {
        if (applyPhysicsLocal) {
            throw new UnsupportedOperationException(
                    "DynamicAnimControl does not support local physics.");
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

        capsule.write(ignoredHops, tagIgnoredHops, 1);
        capsule.write(damping, tagDamping, 0.6f);
        capsule.write(eventDispatchImpulseThreshold,
                tagEventDispatchImpulseThreshold, 0f);

        int count = countLinkedBones();
        String[] linkedBoneNames = new String[count];
        RangeOfMotion[] roms = new RangeOfMotion[count];
        LinkConfig[] blConfigs = new LinkConfig[count];
        int i = 0;
        for (Map.Entry<String, LinkConfig> entry : blConfigMap.entrySet()) {
            linkedBoneNames[i] = entry.getKey();
            roms[i] = jointMap.get(entry.getKey());
            blConfigs[i] = entry.getValue();
            ++i;
        }
        capsule.write(linkedBoneNames, tagLinkedBoneNames, null);
        capsule.write(roms, tagLinkedBoneJoints, null);
        capsule.write(blConfigs, tagBlConfigs, null);

        capsule.write(mainBoneName, tagMainBoneName, null);

        count = countAttachments();
        String[] attachBoneNames = new String[count];
        Spatial[] attachModels = new Spatial[count];
        LinkConfig[] alConfigs = new LinkConfig[count];
        i = 0;
        for (Map.Entry<String, LinkConfig> entry : alConfigMap.entrySet()) {
            attachBoneNames[i] = entry.getKey();
            attachModels[i] = attachModelMap.get(entry.getKey());
            alConfigs[i] = entry.getValue();
            ++i;
        }
        capsule.write(attachBoneNames, tagAttachBoneNames, null);
        capsule.write(attachModels, tagAttachModels, null);
        capsule.write(alConfigs, tagAlConfigs, null);

        capsule.write(torsoConfig, tagTorsoConfig, null);
        capsule.write(gravityVector, tagGravity, null);
        capsule.write(relativeTolerance, tagRelativeTolerance, 0.001f);
    }
    // *************************************************************************
    // private methods

    /**
     * Verify that this Control is NOT added to a Spatial.
     *
     * @param desiredAction (not null, not empty)
     */
    private void verifyNotAddedToSpatial(String desiredAction) {
        assert desiredAction != null;

        Spatial controlledSpatial = getSpatial();
        if (controlledSpatial != null) {
            String message = "Cannot " + desiredAction
                    + " while the Control is added to a Spatial.";
            throw new IllegalStateException(message);
        }
    }
}
