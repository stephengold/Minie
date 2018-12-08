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
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.bullet.collision.PhysicsCollisionEvent;
import com.jme3.bullet.collision.PhysicsCollisionListener;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.joints.Point2PointJoint;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer;
import com.jme3.util.SafeArrayList;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.List;
import java.util.logging.Logger;
import jme3utilities.MyMesh;
import jme3utilities.MySkeleton;
import jme3utilities.MySpatial;
import jme3utilities.Validate;

/**
 * Before adding this control to a spatial, configure it by invoking
 * {@link #link(java.lang.String, float, com.jme3.bullet.animation.RangeOfMotion)}
 * for each bone that should have its own rigid body. Leave unlinked bones near
 * the root of the skeleton to form the torso of the ragdoll.
 * <p>
 * When you add the control to a spatial, it generates a ragdoll consisting of a
 * rigid body for the torso and another for each linked bone. It also creates a
 * SixDofJoint connecting each rigid body to its parent in the link hierarchy.
 * The mass of each rigid body and the range-of-motion of each joint can be
 * reconfigured on the fly.
 * <p>
 * Each link is either dynamic (driven by forces and collisions) or kinematic
 * (unperturbed by forces and collisions). Transitions from dynamic to kinematic
 * can be immediate or gradual.
 *
 * @author Stephen Gold sgold@sonic.net
 *
 * Based on KinematicRagdollControl by Normen Hansen and Rémy Bouquet (Nehon).
 */
public class DynamicAnimControl
        extends DacPhysicsLinks
        implements PhysicsCollisionListener, PhysicsTickListener {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger35
            = Logger.getLogger(DynamicAnimControl.class.getName());
    // *************************************************************************
    // fields

    /**
     * false until the 1st physics tick, true thereafter, indicating that all
     * links are ready for dynamic mode
     */
    private boolean isReady = false;
    /**
     * calculated total mass, not including released attachments
     */
    private float ragdollMass = -1f;
    /**
     * registered provider for the center-of-mass goals, or null if none
     */
    private ImpulseController comImpulseProvider = null;
    /**
     * list of registered collision listeners
     */
    private List<RagdollCollisionListener> collisionListeners
            = new SafeArrayList<>(RagdollCollisionListener.class);
    /*
     * center-of-mass actual location (in physics-space coordinates)
     */
    private Vector3f centerLocation = new Vector3f();
    /*
     * center-of-mass estimated velocity (psu/second in physics-space coordinates)
     */
    private Vector3f centerVelocity = new Vector3f();
    /**
     * local copy of {@link com.jme3.math.Vector3f#ZERO}
     */
    final private static Vector3f translateIdentity = new Vector3f(0f, 0f, 0f);
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled control without any linked bones or attachments
     * (torso only).
     */
    public DynamicAnimControl() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Add a collision listener to this control.
     *
     * @param listener (not null, alias created)
     */
    public void addCollisionListener(RagdollCollisionListener listener) {
        Validate.nonNull(listener, "listener");
        collisionListeners.add(listener);
    }

    /**
     * Begin blending the specified BoneLink and all its descendants into an
     * amputated state. This has the effect of hiding those links.
     *
     * @param rootLink the root of the subtree to amputate (not null)
     * @param blendInterval the duration of the blend interval (in seconds,
     * &ge;0)
     */
    public void amputateSubtree(BoneLink rootLink, float blendInterval) {
        Validate.nonNull(rootLink, "root link");
        Validate.nonNegative(blendInterval, "blend interval");
        if (getSpatial() == null) {
            throw new IllegalStateException(
                    "Cannot change modes unless added to a spatial.");
        }

        blendDescendants(rootLink, KinematicSubmode.Amputated, blendInterval);
        rootLink.blendToKinematicMode(KinematicSubmode.Amputated,
                blendInterval);
    }

    /**
     * Begin blending the specified link and all its descendants to kinematic
     * animation.
     *
     * @param rootLink the root of the subtree to bind (not null)
     * @param blendInterval the duration of the blend interval (in seconds,
     * &ge;0)
     */
    public void animateSubtree(PhysicsLink rootLink, float blendInterval) {
        Validate.nonNull(rootLink, "root link");
        Validate.nonNegative(blendInterval, "blend interval");
        if (getSpatial() == null) {
            throw new IllegalStateException(
                    "Cannot change modes unless added to a spatial.");
        }

        blendSubtree(rootLink, KinematicSubmode.Animated, blendInterval);
    }

    /**
     * Begin blending the specified link and all its descendants into bind pose.
     *
     * @param rootLink the root of the subtree to bind (not null)
     * @param blendInterval the duration of the blend interval (in seconds,
     * &ge;0)
     */
    public void bindSubtree(PhysicsLink rootLink, float blendInterval) {
        Validate.nonNull(rootLink, "root link");
        Validate.nonNegative(blendInterval, "blend interval");
        if (getSpatial() == null) {
            throw new IllegalStateException(
                    "Cannot change modes unless added to a spatial.");
        }

        blendSubtree(rootLink, KinematicSubmode.Bound, blendInterval);
    }

    /**
     * Begin blending all links to purely kinematic mode, driven by animation.
     * TODO callback when the transition completes
     * <p>
     * Allowed only when the control IS added to a spatial.
     *
     * @param blendInterval the duration of the blend interval (in seconds,
     * &ge;0)
     * @param endModelTransform the desired local transform for the controlled
     * spatial when the transition completes or null for no change to local
     * transform (unaffected)
     */
    public void blendToKinematicMode(float blendInterval,
            Transform endModelTransform) {
        Validate.nonNegative(blendInterval, "blend interval");
        if (getSpatial() == null) {
            throw new IllegalStateException(
                    "Cannot change modes unless added to a spatial.");
        }

        getTorsoLink().blendToKinematicMode(KinematicSubmode.Animated, blendInterval,
                endModelTransform);
        for (BoneLink boneLink : getBoneLinks()) {
            boneLink.blendToKinematicMode(KinematicSubmode.Animated,
                    blendInterval);
        }
        for (AttachmentLink link : listAttachmentLinks()) {
            if (!link.isReleased()) {
                link.blendToKinematicMode(blendInterval, null);
            }
        }
    }

    /**
     * Locate the ragdoll's center of mass, excluding released attachments.
     * <p>
     * Allowed only when the control IS added to a spatial.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the location (in physics-space coordinates, either storeResult or
     * a new vector, not null)
     */
    public Vector3f centerOfMass(Vector3f storeResult) {
        if (getSpatial() == null) {
            throw new IllegalStateException(
                    "Control is not added to a spatial.");
        }
        if (!isReady) {
            throw new IllegalStateException(
                    "Control is not ready.");
        }

        recalculateCenter();
        if (storeResult == null) {
            return centerLocation.clone();
        } else {
            return storeResult.set(centerLocation);
        }
    }

    /**
     * Release all unreleased attachments to gravity.
     */
    public void dropAttachments() {
        for (AttachmentLink link : listAttachmentLinks()) {
            if (!link.isReleased()) {
                Vector3f gravity = gravity(null);
                link.setDynamic(gravity);
                link.release();
            }
        }
    }

    /**
     * Find the link that manages the specified vertex and optionally calculate
     * the location of the vertex.
     *
     * @param vertexSpecifier a String of the form "index/geometry" or
     * "index/geometry/bone" (not null, not empty)
     * @param storeWorldLocation (modified if not null)
     * @return the pre-existing link (not null)
     */
    public PhysicsLink findManagerForVertex(String vertexSpecifier,
            Vector3f storeWorldLocation) {
        Validate.nonEmpty(vertexSpecifier, "vertex specifier");
        /*
         * Parse the vertex index and geometry name from the specifier.
         */
        String[] fields = vertexSpecifier.split("/");
        int numFields = fields.length;
        if (numFields < 2 || numFields > 3) {
            throw new IllegalArgumentException(vertexSpecifier);
        }
        /*
         * Find the mesh that contains the vertex.
         */
        Skeleton skeleton = getSkeleton();
        Spatial subtree;
        if (numFields == 3) { // The vertex is in an attached model.
            Bone attachBone = skeleton.getBone(fields[2]);
            assert attachBone != null;
            subtree = MySkeleton.getAttachments(attachBone);
            assert subtree != null;
        } else { // The vertex is in the controlled model.
            subtree = getSpatial();
            assert subtree != null;
        }
        String geometryName = fields[1];
        Spatial gSpatial = MySpatial.findNamed(subtree, geometryName);
        Geometry geometry = (Geometry) gSpatial;
        Mesh mesh = geometry.getMesh();
        /*
         * Calculate the mesh location (pos) of the vertex.
         */
        int vertexIndex = Integer.parseInt(fields[0]);
        int numVertices = mesh.getVertexCount();
        if (vertexIndex < 0 || vertexIndex >= numVertices) {
            throw new IllegalArgumentException(vertexSpecifier);
        }
        assert vertexIndex < numVertices : vertexIndex;
        Vector3f pos = MyMesh.vertexVector3f(mesh, VertexBuffer.Type.Position,
                vertexIndex, null);
        /*
         * Find the manager and convert the pos to a world location.
         */
        PhysicsLink manager;
        if (numFields == 3) { // The vertex is in an attached model.
            assert !MyMesh.isAnimated(mesh);
            manager = findAttachmentLink(fields[2]);
            if (manager == null) {
                throw new IllegalArgumentException(vertexSpecifier);
            }

            if (storeWorldLocation != null) {
                geometry.localToWorld(pos, storeWorldLocation);
            }

        } else { // The vertex is in the controlled model.
            assert MyMesh.isAnimated(mesh);
            String[] managerMap = managerMap(skeleton);
            String managerName = RagUtils.findManager(mesh, vertexIndex,
                    new int[4], new float[4], managerMap);
            if (managerName.equals(torsoName)) {
                manager = getTorsoLink();
            } else {
                manager = findBoneLink(managerName);
            }
            assert manager != null;

            if (storeWorldLocation != null) {
                Transform meshToWorld = meshTransform(null);
                meshToWorld.transformVector(pos, storeWorldLocation);
            }
        }

        return manager;
    }

    /**
     * Immediately freeze the specified link and all its descendants. Note:
     * recursive!
     * <p>
     * Allowed only when the control IS added to a spatial.
     *
     * @param rootLink the root of the subtree to freeze (not null)
     * @param forceKinematic true&rarr;force link to kinematic mode,
     * false&rarr;preserve link modes
     */
    public void freezeSubtree(PhysicsLink rootLink, boolean forceKinematic) {
        Validate.nonNull(rootLink, "root link");
        if (getSpatial() == null) {
            throw new IllegalStateException(
                    "Cannot change modes unless added to a spatial.");
        }

        rootLink.freeze(forceKinematic);

        PhysicsLink[] children = rootLink.listChildren();
        for (PhysicsLink child : children) {
            freezeSubtree(child, forceKinematic);
        }
    }

    /**
     * Test whether all links are ready for dynamic mode.
     *
     * @return true if ready, otherwise false
     */
    public boolean isReady() {
        return isReady;
    }

    /**
     * Add an IK joint that will cause the specified link to move until the a
     * fixed pivot reaches the goal.
     *
     * @param link which link to move (not null)
     * @param pivotInBody the pivot location (in the rigid body's local
     * coordinates, not null, unaffected)
     * @param goalInWorld the goal location (in physics-space coordinates, not
     * null, unaffected)
     * @return a new joint (not null)
     */
    public PhysicsJoint moveToWorld(PhysicsLink link, Vector3f pivotInBody,
            Vector3f goalInWorld) {
        Validate.nonNull(pivotInBody, "pivot location");
        Validate.nonNull(goalInWorld, "goal location");

        PhysicsRigidBody rigidBody = link.getRigidBody();
        Point2PointJoint moveJoint
                = new Point2PointJoint(rigidBody, pivotInBody, goalInWorld);
        rigidBody.addJoint(moveJoint);
        getPhysicsSpace().add(moveJoint);

        return moveJoint;
    }

    /**
     * Add an IK joint that will restrict the specified link to pivoting around
     * a fixed pivot.
     *
     * @param link which link to pin (not null)
     * @param pivotInWorld the pivot location (in physics-space coordinates, not
     * null, unaffected)
     * @return a new joint (not null)
     */
    public PhysicsJoint pinToWorld(PhysicsLink link, Vector3f pivotInWorld) {
        Validate.nonNull(pivotInWorld, "pivot location");

        PhysicsRigidBody rigidBody = link.getRigidBody();

        Transform localToWorld = link.physicsTransform(null);
        localToWorld.setScale(1f);
        Vector3f pivotInBody
                = localToWorld.transformInverseVector(pivotInWorld, null);

        Point2PointJoint pinJoint
                = new Point2PointJoint(rigidBody, pivotInBody);
        rigidBody.addJoint(pinJoint);
        getPhysicsSpace().add(pinJoint);

        return pinJoint;
    }

    /**
     * Alter the center-of-mass impulse provider for this control.
     *
     * @param provider (alias created if not null)
     */
    public void setCenterGoalProvider(ImpulseController provider) {
        comImpulseProvider = provider;
    }

    /**
     * Alter the contact-response setting of the specified link and all its
     * descendants (excluding released attachments). Note: recursive!
     * <p>
     * Allowed only when the control IS added to a spatial.
     *
     * @param rootLink the root of the subtree to modify (not null)
     * @param desiredResponse true for the usual rigid-body response, false for
     * ghost-like response
     */
    public void setContactResponseSubtree(PhysicsLink rootLink,
            boolean desiredResponse) {
        Validate.nonNull(rootLink, "root link");
        if (getSpatial() == null) {
            throw new IllegalStateException(
                    "Cannot change modes unless added to a spatial.");
        }

        if (!rootLink.isReleased()) {
            PhysicsRigidBody rigidBody = rootLink.getRigidBody();
            rigidBody.setContactResponse(desiredResponse);

            PhysicsLink[] children = rootLink.listChildren();
            for (PhysicsLink child : children) {
                setContactResponseSubtree(child, desiredResponse);
            }
        }
    }

    /**
     * Immediately put the specified link and all its descendants (excluding
     * released attachments) into dynamic mode. Note: recursive!
     * <p>
     * Allowed only when the control IS added to a spatial.
     *
     * @param rootLink the root of the subtree to modify (not null)
     * @param uniformAcceleration the uniform acceleration vector (in
     * physics-space coordinates, not null, unaffected)
     * @param lockAll true to lock all axes of links (except the torso)
     */
    public void setDynamicSubtree(PhysicsLink rootLink,
            Vector3f uniformAcceleration, boolean lockAll) {
        Validate.nonNull(rootLink, "root link");
        Validate.nonNull(uniformAcceleration, "uniform acceleration");
        if (getSpatial() == null) {
            throw new IllegalStateException(
                    "Cannot change modes unless added to a spatial.");
        }

        if (rootLink == getTorsoLink()) {
            getTorsoLink().setDynamic(uniformAcceleration);
        } else if (rootLink instanceof BoneLink) {
            BoneLink boneLink = (BoneLink) rootLink;
            boneLink.setDynamic(uniformAcceleration, lockAll, lockAll, lockAll);
        } else {
            AttachmentLink attachmentLink = (AttachmentLink) rootLink;
            if (!attachmentLink.isReleased()) {
                attachmentLink.setDynamic(uniformAcceleration);
            }
        }

        PhysicsLink[] children = rootLink.listChildren();
        for (PhysicsLink child : children) {
            setDynamicSubtree(child, uniformAcceleration, lockAll);
        }
    }

    /**
     * Immediately put all links into purely kinematic mode.
     * <p>
     * Allowed only when the control IS added to a spatial.
     */
    public void setKinematicMode() {
        if (getSpatial() == null) {
            throw new IllegalStateException(
                    "Cannot change modes unless added to a spatial.");
        }

        Transform localTransform = getSpatial().getLocalTransform();
        blendToKinematicMode(0f, localTransform);
    }

    /**
     * Immediately put all links into dynamic mode with gravity.
     * <p>
     * Allowed only when the control IS added to a spatial.
     */
    public void setRagdollMode() {
        if (getSpatial() == null) {
            throw new IllegalStateException(
                    "Cannot change modes unless added to a spatial.");
        }

        Vector3f ragdollGravity = gravity(null);

        getTorsoLink().setDynamic(ragdollGravity);
        for (BoneLink boneLink : getBoneLinks()) {
            boneLink.setDynamic(ragdollGravity, false, false, false);
        }
        for (AttachmentLink link : listAttachmentLinks()) {
            link.setDynamic(ragdollGravity);
        }
    }
    // *************************************************************************
    // DacPhysicsLinks methods

    /**
     * Add all managed physics objects to the physics space.
     */
    @Override
    protected void addPhysics() {
        super.addPhysics();

        PhysicsSpace space = getPhysicsSpace();
        space.addCollisionListener(this);
        space.addTickListener(this);
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

        collisionListeners = cloner.clone(collisionListeners);
        centerLocation = cloner.clone(centerLocation);
        centerVelocity = cloner.clone(centerVelocity);
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public DynamicAnimControl jmeClone() {
        try {
            DynamicAnimControl clone = (DynamicAnimControl) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }

    /**
     * De-serialize this control, for example when loading from a J3O file.
     *
     * @param im importer (not null)
     * @throws IOException from importer
     */
    @Override
    public void read(JmeImporter im) throws IOException {
        super.read(im);
        InputCapsule ic = im.getCapsule(this);

        ragdollMass = ic.readFloat("ragdollMass", 1f);
        centerLocation
                = (Vector3f) ic.readSavable("centerLocation", new Vector3f());
        centerVelocity
                = (Vector3f) ic.readSavable("centerVelocity", new Vector3f());
    }

    /**
     * Remove all managed physics objects from the physics space.
     */
    @Override
    protected void removePhysics() {
        super.removePhysics();

        PhysicsSpace space = getPhysicsSpace();
        space.removeCollisionListener(this);
        space.removeTickListener(this);
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

        oc.write(ragdollMass, "ragdollMass", 1f);
        oc.write(centerLocation, "centerLocation", null);
        oc.write(centerVelocity, "centerVelocity", null);
    }
    // *************************************************************************
    // PhysicsCollisionListener methods

    /**
     * For internal use only: callback for collision events.
     *
     * @param event (not null)
     */
    @Override
    public void collision(PhysicsCollisionEvent event) {
        if (event.getNodeA() == null && event.getNodeB() == null) {
            return;
        }
        /*
         * Determine which bone was involved (if any) and also the
         * other collision object involved.
         */
        boolean isThisControlInvolved = false;
        PhysicsLink physicsLink = null;
        PhysicsCollisionObject otherPco = null;
        PhysicsCollisionObject pcoA = event.getObjectA();
        PhysicsCollisionObject pcoB = event.getObjectB();

        Object userA = pcoA.getUserObject();
        Object userB = pcoB.getUserObject();
        if (userA instanceof PhysicsLink) {
            physicsLink = (PhysicsLink) userA;
            DacPhysicsLinks control = physicsLink.getControl();
            if (control == this) {
                isThisControlInvolved = true;
            }
            otherPco = pcoB;
        }
        if (userB instanceof PhysicsLink) {
            physicsLink = (PhysicsLink) userB;
            DacPhysicsLinks control = physicsLink.getControl();
            if (control == this) {
                isThisControlInvolved = true;
            }
            otherPco = pcoA;
        }
        /*
         * Discard collisions that don't involve this control.
         */
        if (!isThisControlInvolved) {
            return;
        }
        /*
         * Discard low-impulse collisions.
         */
        float impulseThreshold = eventDispatchImpulseThreshold();
        if (event.getAppliedImpulse() < impulseThreshold) {
            return;
        }
        /*
         * Dispatch an event.
         */
        for (RagdollCollisionListener listener : collisionListeners) {
            listener.collide(physicsLink, otherPco, event);
        }
    }
    // *************************************************************************
    // PhysicsTickListener methods

    /**
     * Callback from Bullet, invoked just after the physics has been stepped.
     * Used to re-activate any deactivated rigid bodies.
     *
     * @param space the space that was just stepped (not null)
     * @param timeStep the time per physics step (in seconds, &ge;0)
     */
    @Override
    public void physicsTick(PhysicsSpace space, float timeStep) {
        assert space == getPhysicsSpace();
        Validate.nonNegative(timeStep, "time step");

        getTorsoLink().postTick();
        for (BoneLink boneLink : getBoneLinks()) {
            boneLink.postTick();
        }
        for (AttachmentLink link : listAttachmentLinks()) {
            link.postTick();
        }

        if (!isReady) {
            isReady = true;
        }
    }

    /**
     * Callback from Bullet, invoked just before the physics is stepped. A good
     * time to clear/apply forces.
     *
     * @param space the space that is about to be stepped (not null)
     * @param timeStep the time per physics step (in seconds, &ge;0)
     */
    @Override
    public void prePhysicsTick(PhysicsSpace space, float timeStep) {
        assert space == getPhysicsSpace();
        Validate.nonNegative(timeStep, "time step");

        Vector3f comImpulse = new Vector3f(0f, 0f, 0f);
        if (comImpulseProvider != null) {
            recalculateCenter();
            comImpulseProvider.impulse(centerLocation, centerVelocity,
                    ragdollMass, timeStep, this, comImpulse);
        }

        TorsoLink torsoLink = getTorsoLink();
        torsoLink.preTick(comImpulse);
        for (BoneLink boneLink : getBoneLinks()) {
            boneLink.preTick(translateIdentity);
        }
        for (AttachmentLink link : listAttachmentLinks()) {
            link.preTick(translateIdentity);
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Begin blending the descendents of the specified link to the specified
     * kinematic submode. Note: recursive!
     *
     * @param rootLink the root of the subtree to blend (not null)
     * @param submode an enum value (not null)
     * @param blendInterval the duration of the blend interval (in seconds,
     * &ge;0)
     */
    private void blendDescendants(PhysicsLink rootLink,
            KinematicSubmode submode, float blendInterval) {
        assert rootLink != null;
        assert submode != null;
        assert blendInterval >= 0f : blendInterval;

        PhysicsLink[] children = rootLink.listChildren();
        for (PhysicsLink child : children) {
            if (child instanceof BoneLink) {
                BoneLink boneLink = (BoneLink) child;
                boneLink.blendToKinematicMode(submode, blendInterval);
            } else {
                AttachmentLink attachmentLink = (AttachmentLink) child;
                if (!attachmentLink.isReleased()) {
                    attachmentLink.blendToKinematicMode(blendInterval, null);
                }
            }
            blendDescendants(child, submode, blendInterval);
        }
    }

    /**
     * Begin blending the specified link and all its descendants to the
     * specified kinematic submode.
     *
     * @param rootLink the root of the subtree to blend (not null)
     * @param submode the desired submode (not null)
     * @param blendInterval the duration of the blend interval (in seconds,
     * &ge;0)
     */
    private void blendSubtree(PhysicsLink rootLink, KinematicSubmode submode,
            float blendInterval) {
        assert rootLink != null;
        assert submode != null;
        assert blendInterval >= 0f : blendInterval;

        blendDescendants(rootLink, submode, blendInterval);

        if (rootLink == getTorsoLink()) {
            getTorsoLink().blendToKinematicMode(submode, blendInterval, null);
        } else if (rootLink instanceof BoneLink) {
            BoneLink boneLink = (BoneLink) rootLink;
            boneLink.blendToKinematicMode(submode, blendInterval);
        } else {
            AttachmentLink attachmentLink = (AttachmentLink) rootLink;
            if (!attachmentLink.isReleased()) {
                attachmentLink.blendToKinematicMode(blendInterval, null);
            }
        }
    }

    /**
     * Recalculate the location, velocity, and total mass for the ragdoll's
     * center of mass, not including released attachments.
     */
    private void recalculateCenter() {
        double massSum = 0.0;
        Vector3f locationSum = new Vector3f();
        Vector3f velocitySum = new Vector3f();
        Vector3f tmpVector = new Vector3f();
        List<PhysicsLink> links = listLinks(PhysicsLink.class);
        for (PhysicsLink link : links) {
            if (!link.isReleased()) {
                PhysicsRigidBody rigidBody = link.getRigidBody();
                float mass = rigidBody.getMass();
                massSum += (double) mass;

                rigidBody.getPhysicsLocation(tmpVector);
                tmpVector.multLocal(mass);
                locationSum.addLocal(tmpVector);

                link.velocity(tmpVector);
                tmpVector.multLocal(mass);
                velocitySum.addLocal(tmpVector);
            }
        }

        float invMass = (float) (1.0 / massSum);
        locationSum.mult(invMass, centerLocation);
        velocitySum.mult(invMass, centerVelocity);
        ragdollMass = (float) massSum;
    }
}
