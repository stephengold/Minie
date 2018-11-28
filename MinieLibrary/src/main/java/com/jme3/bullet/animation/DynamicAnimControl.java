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

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.bullet.collision.PhysicsCollisionEvent;
import com.jme3.bullet.collision.PhysicsCollisionListener;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.util.SafeArrayList;
import com.jme3.util.clone.Cloner;
import java.util.List;
import java.util.logging.Logger;
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
 * <p>
 * TODO ghost mode
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
     * list of registered collision listeners
     */
    private List<RagdollCollisionListener> listeners
            = new SafeArrayList<>(RagdollCollisionListener.class);
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
        listeners.add(listener);
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

        blendDescendants(rootLink, KinematicSubmode.Bound, blendInterval);

        if (rootLink == getTorsoLink()) {
            getTorsoLink().blendToKinematicMode(KinematicSubmode.Bound,
                    blendInterval, null);
        } else if (rootLink instanceof BoneLink) {
            BoneLink boneLink = (BoneLink) rootLink;
            boneLink.blendToKinematicMode(KinematicSubmode.Bound,
                    blendInterval);
        } else {
            AttachmentLink attachmentLink = (AttachmentLink) rootLink;
            if (!attachmentLink.isReleased()) {
                attachmentLink.blendToKinematicMode(blendInterval, null);
            }
        }
    }

    /**
     * Begin blending all links to fully kinematic mode, driven by animation.
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
     * Immediately put all links into fully kinematic mode.
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
     * Immediately put all links into fully dynamic mode with gravity.
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
    // ConfigDynamicAnimControl methods

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
        listeners = cloner.clone(listeners);
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
     * Remove all managed physics objects from the physics space.
     */
    @Override
    protected void removePhysics() {
        super.removePhysics();

        PhysicsSpace space = getPhysicsSpace();
        space.removeCollisionListener(this);
        space.removeTickListener(this);
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
        for (RagdollCollisionListener listener : listeners) {
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
        PhysicsRigidBody prb = getTorsoLink().getRigidBody();
        prb.activate();

        for (BoneLink boneLink : getBoneLinks()) {
            prb = boneLink.getRigidBody();
            prb.activate();
        }

        for (AttachmentLink attachmentLink : listAttachmentLinks()) {
            prb = attachmentLink.getRigidBody();
            prb.activate();
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
        getTorsoLink().prePhysicsTick();
        for (BoneLink boneLink : getBoneLinks()) {
            boneLink.prePhysicsTick();
        }
        for (AttachmentLink link : listAttachmentLinks()) {
            link.prePhysicsTick();
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Begin blending the descendents of the specified BoneLink to the specified
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
}
