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
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import com.jme3.util.clone.JmeCloneable;
import java.io.IOException;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.MyString;
import jme3utilities.Validate;
import jme3utilities.minie.MyShape;

/**
 * The abstract base class used by DynamicAnimControl to link pieces of a JME
 * model to their corresponding collision objects in a ragdoll. Subclasses
 * include: AttachmentLink, BoneLink, and TorsoLink. The links in each
 * DynamicAnimControl form a hierarchy with the TorsoLink at its root.
 *
 * @author Stephen Gold sgold@sonic.net
 *
 * Based on KinematicRagdollControl by Normen Hansen and Rémy Bouquet (Nehon).
 */
abstract public class PhysicsLink implements JmeCloneable, Savable {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(PhysicsLink.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagArmatureJoint = "armatureJoint";
    final private static String tagBlendInterval = "blendInterval";
    final private static String tagBone = "bone";
    final private static String tagChildren = "children";
    final private static String tagControl = "control";
    final private static String tagDensity = "density";
    final private static String tagIkControllers = "ikControllers";
    final private static String tagJoint = "joint";
    final private static String tagKinematicWeight = "kinematicWeight";
    final private static String tagKpTransform = "kpTransform";
    final private static String tagKpVelocity = "kpVelocity";
    final private static String tagLocalOffset = "offset";
    final private static String tagParent = "parent";
    final private static String tagRigidBody = "rigidBody";
    // *************************************************************************
    // fields

    /**
     * list of controllers for this link's inverse kinematics
     */
    private ArrayList<IKController> ikControllers = new ArrayList<>(8);
    /**
     * immediate children in the link hierarchy (not null)
     */
    private ArrayList<PhysicsLink> children = new ArrayList<>(8);
    /**
     * corresponding Bone in the Skeleton, or null for an armature joint
     */
    private Bone bone;
    /**
     * scene-graph control that manages this link (not null)
     */
    private DacLinks control;
    /**
     * duration of the most recent blend interval (in seconds, &ge;0)
     */
    private float blendInterval = 1f;
    /**
     * average density of the rigid body (in pmu/psu^3, &gt;0)
     */
    private float density;
    /**
     * weighting of kinematic movement (&ge;0, &le;1, 0=purely dynamic, 1=purely
     * kinematic, progresses from 0 to 1 during the blend interval)
     */
    private float kinematicWeight = 1f;
    /**
     * corresponding Joint in the Armature, or null for a skeleton bone
     */
    private Joint armatureJoint;
    /**
     * joint between the rigid body and the parent's rigid body, or null if not
     * yet created
     */
    private PhysicsJoint joint = null;
    /**
     * parent/manager in the link hierarchy, or null if none
     */
    private PhysicsLink parent = null;
    /**
     * linked rigid body in the ragdoll (not null)
     */
    private PhysicsRigidBody rigidBody;
    /**
     * Transform of the rigid body as of the most recent update (in
     * physics-space coordinates, updated in kinematic mode only)
     */
    private Transform kpTransform = new Transform();
    /**
     * estimate of the body's linear velocity as of the most recent update
     * (psu/sec in physics-space coordinates, kinematic mode only)
     */
    private Vector3f kpVelocity = new Vector3f();
    /**
     * location of the rigid body's center (in the skeleton bone's local
     * coordinates)
     */
    private Vector3f localOffset;
    /**
     * temporary scale vector
     */
    private Vector3f tmpScale = new Vector3f();
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected PhysicsLink() {
    }

    /**
     * Instantiate a purely kinematic link between the specified skeleton bone
     * and the specified rigid body.
     *
     * @param control the Control that will manage this link (not null, alias
     * created)
     * @param bone the corresponding Bone (not null, alias created)
     * @param collisionShape the desired shape (not null, alias created)
     * @param linkConfig the link configuration (not null)
     * @param localOffset the location of the body's center (in the bone's local
     * coordinates, not null, unaffected)
     */
    PhysicsLink(DacLinks control, Bone bone, CollisionShape collisionShape,
            LinkConfig linkConfig, Vector3f localOffset) {
        assert control != null;
        assert bone != null;
        assert collisionShape != null;
        assert linkConfig != null;
        assert localOffset != null;

        this.control = control;
        this.bone = bone;
        this.rigidBody = createRigidBody(linkConfig, collisionShape);

        if (logger.isLoggable(Level.FINE)) {
            logger.log(Level.FINE, "Creating link for bone {0} with mass={1}",
                    new Object[]{
                        MyString.quote(bone.getName()), rigidBody.getMass()
                    });
        }

        this.localOffset = localOffset.clone();
        updateKPTransform();
    }

    /**
     * Instantiate a purely kinematic link between the specified armature joint
     * and the specified rigid body.
     *
     * @param control the Control that will manage this link (not null, alias
     * created)
     * @param armatureJoint the corresponding Joint (not null, alias created)
     * @param collisionShape the desired shape (not null, alias created)
     * @param linkConfig the link configuration (not null)
     * @param localOffset the location of the body's center (in the armature
     * joint's local coordinates, not null, unaffected)
     */
    PhysicsLink(DacLinks control, Joint armatureJoint,
            CollisionShape collisionShape, LinkConfig linkConfig,
            Vector3f localOffset) {
        assert control != null;
        assert armatureJoint != null;
        assert collisionShape != null;
        assert linkConfig != null;
        assert localOffset != null;

        this.control = control;
        this.armatureJoint = armatureJoint;
        this.rigidBody = createRigidBody(linkConfig, collisionShape);

        if (logger.isLoggable(Level.FINE)) {
            logger.log(Level.FINE, "Creating link for joint {0} with mass={1}",
                    new Object[]{
                        MyString.quote(armatureJoint.getName()),
                        rigidBody.getMass()
                    });
        }

        this.localOffset = localOffset.clone();
        updateKPTransform();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Add an IK controller.
     *
     * @param controller the controller to add (not null, alias created)
     */
    public void addIKController(IKController controller) {
        Validate.nonNull(controller, "controller");
        assert controller.getLink() == this;
        assert !ikControllers.contains(controller);

        ikControllers.add(controller);
    }

    /**
     * Read the name of the corresponding skeleton bone or armature joint.
     *
     * @return the bone/joint name (not null)
     */
    public String boneName() {
        String name;
        if (bone != null) { // old animation system
            name = bone.getName();
        } else { // new animation system
            name = armatureJoint.getName();
        }

        assert name != null;
        return name;
    }

    /**
     * Count this link's immediate children in the link hierarchy.
     *
     * @return the count (&ge;0)
     */
    public int countChildren() {
        int numChildren = children.size();
        return numChildren;
    }

    /**
     * Read the average density of the rigid body.
     *
     * @return the density (&gt;0)
     */
    public float density() {
        assert density > 0f : density;
        return density;
    }

    /**
     * Disable all IK controllers.
     */
    public void disableAllIKControllers() {
        for (IKController controller : ikControllers) {
            controller.setEnabled(false);
        }
    }

    /**
     * Immediately freeze this link.
     *
     * @param forceKinematic true&rarr;force to kinematic mode,
     * false&rarr;preserve mode
     */
    abstract public void freeze(boolean forceKinematic);

    /**
     * Access the corresponding armature joint.
     *
     * @return the pre-existing instance, or null if none
     */
    final public Joint getArmatureJoint() {
        return armatureJoint;
    }

    /**
     * Access the corresponding skeleton bone.
     *
     * @return the pre-existing instance, or null if none
     */
    final public Bone getBone() {
        return bone;
    }

    /**
     * Access the control that manages this link.
     *
     * @return the pre-existing instance (not null)
     */
    public DacLinks getControl() {
        assert control != null;
        return control;
    }

    /**
     * Access the joint between this link's rigid body and that of its parent.
     *
     * @return the pre-existing instance, or null for the torso
     */
    public PhysicsJoint getJoint() {
        return joint;
    }

    /**
     * Access this link's parent/manager in the link hierarchy.
     *
     * @return the link, or null if none
     */
    public PhysicsLink getParent() {
        return parent;
    }

    /**
     * Access the linked rigid body.
     *
     * @return the pre-existing instance (not null)
     */
    final public PhysicsRigidBody getRigidBody() {
        assert rigidBody != null;
        return rigidBody;
    }

    /**
     * Test whether the link is in kinematic mode.
     *
     * @return true if kinematic, or false if purely dynamic
     */
    public boolean isKinematic() {
        if (kinematicWeight > 0f) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Test whether the attached model (if any) has been released.
     *
     * @return false unless this is an AttachmentLink
     */
    public boolean isReleased() {
        return false;
    }

    /**
     * Read the kinematic weight of this link.
     *
     * @return 0 if purely dynamic, 1 if purely kinematic
     */
    public float kinematicWeight() {
        assert kinematicWeight >= 0f : kinematicWeight;
        assert kinematicWeight <= 1f : kinematicWeight;
        return kinematicWeight;
    }

    /**
     * Enumerate this link's immediate children in the link hierarchy.
     *
     * @return a new array (not null)
     */
    public PhysicsLink[] listChildren() {
        int numChildren = children.size();
        PhysicsLink[] result = new PhysicsLink[numChildren];
        children.toArray(result);

        return result;
    }

    /**
     * Enumerate inverse-kinematics controllers for this link.
     *
     * @return a new array of pre-existing instances
     */
    public IKController[] listIKControllers() {
        int numControllers = ikControllers.size();
        IKController[] result = new IKController[numControllers];
        ikControllers.toArray(result);

        return result;
    }

    /**
     * Unambiguously identify this link by name, within its DynamicAnimControl.
     *
     * @return a text string (not null, not empty)
     */
    abstract public String name();

    /**
     * Calculate a physics transform for the rigid body (to match the skeleton
     * bone).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the calculated transform (in physics-space coordinates, either
     * storeResult or a new transform, not null)
     */
    public Transform physicsTransform(Transform storeResult) {
        Transform result
                = (storeResult == null) ? new Transform() : storeResult;
        if (isKinematic()) {
            result.set(kpTransform);
        } else {
            rigidBody.getTransform(result);
        }

        return result;
    }

    /**
     * Copy animation data from the specified link, which must correspond to the
     * same bone.
     *
     * @param oldLink the link to copy from (not null, unaffected)
     */
    void postRebuildLink(PhysicsLink oldLink) {
        assert oldLink != null;
        assert oldLink.getBone() == bone;

        if (oldLink.isKinematic()) {
            this.blendInterval = oldLink.blendInterval;
            this.kinematicWeight = oldLink.kinematicWeight();
        } else {
            this.blendInterval = 0f;
            this.kinematicWeight = 1f;
        }
    }

    /**
     * Internal callback, invoked just AFTER the physics is stepped.
     */
    void postTick() {
        // do nothing
    }

    /**
     * Internal callback, invoked just BEFORE the physics is stepped.
     *
     * @param timeStep the physics time step (in seconds, &ge;0)
     */
    void preTick(float timeStep) {
        if (isKinematic()) {
            updateRigidBodyTransform();
        } else {
            for (IKController controller : ikControllers) {
                controller.preTick(timeStep);
            }
        }
    }

    /**
     * Remove an IK controller.
     *
     * @param controller the controller to remove (not null, unaffected)
     * @return true if successful, otherwise false
     */
    public boolean removeIKController(IKController controller) {
        Validate.nonNull(controller, "controller");

        boolean success = ikControllers.remove(controller);
        return success;
    }

    /**
     * Immediately put this link into dynamic mode. The control must be "ready".
     *
     * @param uniformAcceleration the uniform acceleration vector to apply (in
     * physics-space coordinates, not null, unaffected)
     */
    public void setDynamic(Vector3f uniformAcceleration) {
        Validate.finite(uniformAcceleration, "uniform acceleration");
        String desiredAction = "put " + name() + " into dynamic mode";
        control.verifyReadyForDynamicMode(desiredAction);

        setKinematicWeight(0f);
        rigidBody.setGravity(uniformAcceleration);
        rigidBody.setEnableSleep(false);
    }

    /**
     * Immediately put this link (and all its controllers) into ragdoll mode.
     */
    public void setRagdollMode() {
        IKController[] controllers = listIKControllers();
        for (IKController controller : controllers) {
            controller.setRagdollMode();
        }
    }

    /**
     * Internal callback, invoked once per frame during the logical-state
     * update, provided the control is added to a scene.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    void update(float tpf) {
        assert tpf >= 0f : tpf;

        if (kinematicWeight > 0f) {
            kinematicUpdate(tpf);
        } else {
            dynamicUpdate();
        }
    }

    /**
     * Copy the body's linear velocity, or an estimate thereof.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a new velocity vector (psu/sec in physics-space coordinates)
     */
    public Vector3f velocity(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        if (isKinematic()) {
            result.set(kpVelocity);
        } else {
            assert !rigidBody.isKinematic();
            rigidBody.getLinearVelocity(result);
        }

        return result;
    }
    // *************************************************************************
    // new protected methods

    /**
     * Begin blending this link to a purely kinematic mode.
     *
     * @param blendInterval the duration of the blend interval (in seconds,
     * &ge;0)
     */
    protected void blendToKinematicMode(float blendInterval) {
        assert blendInterval >= 0f : blendInterval;

        this.blendInterval = blendInterval;
        setKinematicWeight(Float.MIN_VALUE); // non-zero to trigger blending
    }

    /**
     * Update this link in Dynamic mode, setting the linked bone's transform
     * based on the transform of the rigid body.
     */
    abstract protected void dynamicUpdate();

    /**
     * Update this link in blended Kinematic mode.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    protected void kinematicUpdate(float tpf) {
        assert tpf >= 0f : tpf;
        assert rigidBody.isKinematic();

        // If blending, increase the kinematic weight.
        if (blendInterval == 0f) {
            setKinematicWeight(1f); // done blending
        } else {
            setKinematicWeight(kinematicWeight + tpf / blendInterval);
        }
        /*
         * If we didn't need kpVelocity, we could defer this
         * calculation until the preTick().
         */
        Vector3f previousLocation = kpTransform.getTranslation(null);
        updateKPTransform();
        if (tpf > 0f) {
            kpTransform.getTranslation().subtract(previousLocation, kpVelocity);
            kpVelocity.divideLocal(tpf);
        }
    }

    /**
     * Copy the local offset of this link.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the offset (in bone local coordinates, either storeResult or a
     * new vector, not null)
     */
    protected Vector3f localOffset(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        result.set(localOffset);
        return result;
    }

    /**
     * Assign a physics joint to this link, or cancel the assigned joint.
     *
     * @param joint (may be null, alias created)
     */
    final protected void setJoint(PhysicsJoint joint) {
        this.joint = joint;
    }

    /**
     * Assign the parent/manager for this link.
     *
     * @param parent the link to use (not null, alias created)
     */
    final protected void setParent(PhysicsLink parent) {
        assert parent != null;
        assert this.parent == null;

        this.parent = parent;
        parent.children.add(this);
    }

    /**
     * Replace the rigid body for this link.
     *
     * @param body the rigid body to use (not null, alias created)
     */
    protected void setRigidBody(PhysicsRigidBody body) {
        assert body != null;
        assert rigidBody != null;
        this.rigidBody = body;
    }
    // *************************************************************************
    // JmeCloneable methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned link into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this link (not null)
     * @param original the instance from which this link was shallow-cloned
     * (unused)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        this.armatureJoint = cloner.clone(armatureJoint);
        this.bone = cloner.clone(bone);
        this.control = cloner.clone(control);
        this.ikControllers = cloner.clone(ikControllers);
        this.children = cloner.clone(children);
        this.joint = cloner.clone(joint);
        this.parent = cloner.clone(parent);
        this.rigidBody = cloner.clone(rigidBody);
        this.kpTransform = cloner.clone(kpTransform);
        this.kpVelocity = cloner.clone(kpVelocity);
        this.localOffset = cloner.clone(localOffset);
        this.tmpScale = cloner.clone(tmpScale);
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public PhysicsLink jmeClone() {
        try {
            PhysicsLink clone = (PhysicsLink) clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }
    // *************************************************************************
    // Savable methods

    /**
     * De-serialize this link from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    @SuppressWarnings("unchecked")
    public void read(JmeImporter importer) throws IOException {
        InputCapsule capsule = importer.getCapsule(this);

        this.ikControllers = capsule.readSavableArrayList(
                tagIkControllers, new ArrayList(1));
        this.children
                = capsule.readSavableArrayList(tagChildren, new ArrayList(1));
        this.armatureJoint
                = (Joint) capsule.readSavable(tagArmatureJoint, null);
        this.bone = (Bone) capsule.readSavable(tagBone, null);
        this.control = (DacLinks) capsule.readSavable(tagControl, null);
        this.blendInterval = capsule.readFloat(tagBlendInterval, 1f);
        this.density = capsule.readFloat(tagDensity, 1f);
        this.kinematicWeight = capsule.readFloat(tagKinematicWeight, 1f);
        this.joint = (PhysicsJoint) capsule.readSavable(tagJoint, null);
        this.parent = (PhysicsLink) capsule.readSavable(tagParent, null);
        this.rigidBody
                = (PhysicsRigidBody) capsule.readSavable(tagRigidBody, null);
        this.kpTransform = (Transform) capsule.readSavable(
                tagKpTransform, new Transform());
        this.kpVelocity
                = (Vector3f) capsule.readSavable(tagKpVelocity, new Vector3f());
        this.localOffset = (Vector3f) capsule.readSavable(
                tagLocalOffset, new Vector3f());

        rigidBody.setUserObject(this);
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
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.writeSavableArrayList(ikControllers, tagIkControllers, null);
        capsule.writeSavableArrayList(children, tagChildren, null);
        capsule.write(armatureJoint, tagArmatureJoint, null);
        capsule.write(bone, tagBone, null);
        capsule.write(control, tagControl, null);
        capsule.write(blendInterval, tagBlendInterval, 1f);
        capsule.write(density, tagDensity, 1f);
        capsule.write(kinematicWeight, tagKinematicWeight, 1f);
        capsule.write(joint, tagJoint, null);
        capsule.write(parent, tagParent, null);
        capsule.write(rigidBody, tagRigidBody, null);
        capsule.write(kpTransform, tagKpTransform, null);
        capsule.write(kpVelocity, tagKpVelocity, null);
        capsule.write(localOffset, tagLocalOffset, null);
        // tmpScale is never written.
    }
    // *************************************************************************
    // private methods

    /**
     * Create and configure a rigid body for this link.
     *
     * @param linkConfig the link configuration (not null)
     * @param collisionShape the desired shape (not null, alias created)
     * @return a new instance, not in any PhysicsSpace
     */
    private PhysicsRigidBody createRigidBody(
            LinkConfig linkConfig, CollisionShape collisionShape) {
        assert collisionShape != null;

        float volume = MyShape.volume(collisionShape);
        float mass = linkConfig.mass(volume);
        this.density = mass / volume;
        PhysicsRigidBody body = new PhysicsRigidBody(collisionShape, mass);

        float viscousDamping = control.damping();
        body.setDamping(viscousDamping, viscousDamping);

        body.setKinematic(true);
        body.setUserObject(this);

        return body;
    }

    /**
     * Alter the kinematic weight and copy the physics transform and velocity
     * info as needed.
     *
     * @param weight (&ge;0)
     */
    private void setKinematicWeight(float weight) {
        assert weight >= 0f : weight;

        boolean wasKinematic = (kinematicWeight > 0f);
        this.kinematicWeight = (weight > 1f) ? 1f : weight;
        boolean isKinematic = (kinematicWeight > 0f);

        if (wasKinematic && !isKinematic) {
            rigidBody.setKinematic(false);
            updateRigidBodyTransform();
            rigidBody.setLinearVelocity(kpVelocity);
        } else if (isKinematic && !wasKinematic) {
            rigidBody.getTransform(kpTransform);
            rigidBody.getLinearVelocity(kpVelocity);
            rigidBody.setKinematic(true);
        }
    }

    /**
     * Update the kinematic physics transform.
     */
    private void updateKPTransform() {
        if (bone != null) { // old animation system
            control.physicsTransform(bone, localOffset, kpTransform);
        } else { // new animation system
            control.physicsTransform(armatureJoint, localOffset, kpTransform);
        }
    }

    /**
     * Update the rigid-body transform based on the kinematic-physics transform.
     */
    private void updateRigidBodyTransform() {
        rigidBody.setPhysicsLocation(kpTransform.getTranslation());
        rigidBody.setPhysicsRotation(kpTransform.getRotation());

        Vector3f kpScale = kpTransform.getScale(); // alias
        rigidBody.getScale(tmpScale);
        if (!control.areWithinTolerance(kpScale, tmpScale)) {
            rigidBody.setPhysicsScale(kpScale);
        }
    }
}
