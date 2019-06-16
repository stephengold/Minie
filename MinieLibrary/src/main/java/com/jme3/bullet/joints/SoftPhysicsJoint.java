/*
 * Copyright (c) 2009-2016 jMonkeyEngine
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
package com.jme3.bullet.joints;

import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * The abstract base class for joining a PhysicsSoftBody to another body, based
 * on Bullet's btSoftBody::Joint. There are 2 kinds of SoftPhysicsJoint:
 * <ul>
 * <li>soft-soft joints to join 2 distinct soft bodies, and</li>
 * <li>soft-rigid joints to join a soft body and a rigid body.</li>
 * </ul>
 * Subclasses include: SoftLinearJoint and SoftAngularJoint.
 * <p>
 * Another way to join a soft body and a rigid body is to append an anchor to
 * the soft body:
 * {@link com.jme3.bullet.objects.PhysicsSoftBody#appendAnchor(int, com.jme3.bullet.objects.PhysicsRigidBody, com.jme3.math.Vector3f, boolean, float)}
 *
 * @author dokthar
 */
public abstract class SoftPhysicsJoint extends PhysicsJoint {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(SoftPhysicsJoint.class.getName());
    // *************************************************************************
    // fields TODO privatize
    /**
     * true if this joint is added to a soft body A
     */
    private boolean added = false;
    /**
     * local copy (default=1)
     */
    protected float constraintForceMixing = 1f;
    /**
     * local copy (default=1)
     */
    protected float errorReductionParameter = 1f;
    /**
     * local copy (default=1)
     */
    protected float split = 1f;
    /**
     * soft body A specified in the constructor, or null if A is a rigid body
     */
    protected PhysicsSoftBody softA;
    /**
     * soft body B specified in the constructor, or null if B is a rigid body
     */
    protected PhysicsSoftBody softB;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    protected SoftPhysicsJoint() {
    }

    /**
     * Instantiate a SoftPhysicsJoint to join a soft body and a rigid body.
     * <p>
     * To be effective, the joint must be added to the PhysicsSoftSpace of both
     * bodies.
     *
     * @param nodeA the soft body for the A end (not null, alias created)
     * @param nodeB the rigid body for the B end (not null, alias created)
     * @param pivotA the pivot location in A's local coordinates (not null,
     * unaffected)
     * @param pivotB the pivot location in B's local coordinates (not null,
     * unaffected)
     */
    public SoftPhysicsJoint(PhysicsSoftBody nodeA, PhysicsRigidBody nodeB,
            Vector3f pivotA, Vector3f pivotB) {
        this.nodeA = null;
        this.nodeB = nodeB;
        this.softA = nodeA;
        this.softB = null;
        this.pivotA = pivotA;
        this.pivotB = pivotB;

        nodeA.addJoint(this);
        nodeB.addJoint(this);
    }

    /**
     * Instantiate a SoftPhysicsJoint to join 2 soft bodies.
     * <p>
     * To be effective, the joint must be added to the PhysicsSoftSpace of both
     * bodies. Also, the bodies must be distinct.
     *
     * @param nodeA the body for the A end (not null, alias created)
     * @param nodeB the body for the B end (not null, alias created)
     * @param pivotA the pivot location in A's local coordinates (not null,
     * unaffected)
     * @param pivotB the pivot location in B's local coordinates (not null,
     * unaffected)
     */
    public SoftPhysicsJoint(PhysicsSoftBody nodeA, PhysicsSoftBody nodeB,
            Vector3f pivotA, Vector3f pivotB) {
        if (nodeA == nodeB) {
            throw new IllegalArgumentException("The bodies must be distinct.");
        }

        this.softA = nodeA;
        this.softB = nodeB;
        this.nodeA = null;
        this.nodeB = null;
        this.pivotA = pivotA;
        this.pivotB = pivotB;

        nodeA.addJoint(this);
        nodeB.addJoint(this);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Add this constraint to body A. This method should be invoked only by the
     * PhysicsSoftSpace.
     */
    public void addConstraint() {
        if (!added) {
            addConstraint(objectId, softA.getObjectId());
            added = true;
        }
    }

    /**
     * Get the constraint force mixing coefficient (aka CFM).
     * <p>
     * From bullet documentation :</p>
     * <ul>
     * <li>If CFM = 0 then the constraint will be hard.
     * <li>If CFM is set to a positive value, it will be possible to violate the
     * constraint by "pushing on it" (for example, for contact constraints by
     * forcing the two contacting objects together). In other words the
     * constraint will be soft, and the softness will increase as CFM increases.
     * </ul>
     * <p>
     * Note that setting CFM to a negative value can have undesirable bad
     * effects, such as instability. Don't do it.
     * </p>
     * (1 is the default).
     *
     * @return the constraint force mixing value
     */
    public float getConstraintForceMixing() {
        return getConstraintForceMixing(objectId);
    }

    /**
     * Get the error reduction parameter coefficient (aka ERP).
     * <p>
     * From the Bullet documentation:</p>
     * <p>
     * The ERP specifies what proportion of the joint error will be fixed during
     * the next simulation step.
     * </p>
     * <ul>
     * <li>If ERP=0 then no correcting force is applied and the bodies will
     * eventually drift apart as the simulation proceeds.
     * <li>If ERP = 1 then the simulation will attempt to fix all joint error
     * during the next time step. However, setting ERP=1 is not recommended, as
     * the joint error will not be completely fixed due to various internal
     * approximations.
     * </ul>
     * A value of ERP = 0.1 to 0.8 is recommended (1 is the default).
     *
     * @return the error reduction parameter value
     */
    public float getErrorReductionParameter() {
        return getErrorReductionParameter(objectId);
    }

    public PhysicsSoftBody getSoftBodyA() {
        return softA;
    }

    public PhysicsSoftBody getSoftBodyB() {
        return softB;
    }

    public float getSplit() {
        return getSplit(objectId);
    }

    public boolean isSoftRigidJoint() {
        return nodeB != null;
    }

    public boolean isSoftSoftJoint() {
        return nodeB == null; // == (softB != null)
    }

    /**
     * Remove this constraint from body A. This method should be invoked only by
     * the PhysicsSoftSpace.
     */
    public void removeConstraint() {
        if (added) {
            removeConstraint(objectId, softA.getObjectId());
            added = false;
        }
    }

    /**
     * Set the constraint force mixing coefficient (aka CFM).
     * <p>
     * From bullet documentation:</p>
     * <ul>
     * <li>If CFM = 0 then the constraint will be hard.
     * <li>If CFM is set to a positive value, it will be possible to violate the
     * constraint by "pushing on it" (for example, for contact constraints by
     * forcing the two contacting objects together). In other words the
     * constraint will be soft, and the softness will increase as CFM increases.
     * </ul>
     * <p>
     * Note that setting CFM to a negative value can have undesirable bad
     * effects, such as instability. Don't do it.
     * </p>
     * (1 is the default).
     *
     * @param cfm the value to set, between [0,+inf].
     */
    public void setConstraintForceMixing(float cfm) {
        setConstraintForceMixing(objectId, cfm);
    }

    /**
     * Set the error reduction parameter coefficient (aka ERP).
     * <p>
     * From bullet documentation :</p>
     * <p>
     * The ERP specifies what proportion of the joint error will be fixed during
     * the next simulation step.
     * </p>
     * <ul>
     * <li>If ERP=0 then no correcting force is applied and the bodies will
     * eventually drift apart as the simulation proceeds.
     * <li>If ERP = 1 then the simulation will attempt to fix all joint error
     * during the next time step. However, setting ERP=1 is not recommended, as
     * the joint error will not be completely fixed due to various internal
     * approximations.
     * </ul>
     * A value of ERP = 0.1 to 0.8 is recommended (1 is the default).
     *
     * @param erp the value to set, between [0,1].
     */
    public void setErrorReductionParameter(float erp) {
        setErrorReductionParameter(objectId, erp);
    }

    public void setSplit(float split) {
        setSplit(objectId, split);
    }
    // *************************************************************************
    // PhysicsJoint methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned joint into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this joint (not null)
     * @param original the instance from which this joint was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);
        // TODO
    }

    /**
     * Destroy this joint and remove it from its connected Body joint lists
     */
    @Override
    public void destroy() {
        getSoftBodyA().removeJoint(this);
        if (isSoftRigidJoint()) {
            getBodyB().removeJoint(this);
        } else {
            getSoftBodyB().removeJoint(this);
        }
    }

    @Override
    protected void finalize() throws Throwable {
        if (added) {
            // the joint is still attached to a softbody,
            // assuming the joint is Garbage collected at the same time as his softbody
            // (the softbody have a reference to this in his joint list)
            // when deleted the softbody will delete all his joints as well.
            Logger.getLogger(this.getClass().getName()).log(Level.FINE, "SoftPhysicsJoint {0} is still attached, it will be destroyed by the softBody", Long.toHexString(objectId));
        } else {
            // finalizeNative() will be invoked by the superclass finalize
            super.finalize();
        }
    }

    @Override
    native protected void finalizeNative(long jointId);

    /**
     * The bodyA must be a SoftBody.
     *
     * @return null
     */
    @Override
    public PhysicsRigidBody getBodyA() {
        return null;
    }

    @Override
    public boolean isCollisionBetweenLinkedBodies() {
        return false;
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public SoftPhysicsJoint jmeClone() {
        try {
            SoftPhysicsJoint clone = (SoftPhysicsJoint) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }

    /**
     * De-serialize this joint from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);
        InputCapsule capsule = importer.getCapsule(this);

        softA = (PhysicsSoftBody) capsule.readSavable("softA", null);
        softB = (PhysicsSoftBody) capsule.readSavable("softB", null);

        constraintForceMixing = capsule.readFloat("constraintForceMixing", 1f);
        errorReductionParameter = capsule.readFloat("errorReductionParameter", 1f);
        split = capsule.readFloat("split", 1f);

        // TODO create the btCollisionObject
        // TODO read the breaking impulse threshold
        // TODO read the enabled flag
    }

    /**
     * Serialize this joint to the specified exporter, for example when saving
     * to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        super.write(exporter);
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(softA, "softA", null);
        capsule.write(softB, "softB", null);

        capsule.write(getConstraintForceMixing(), "constraintForceMixing", 1);
        capsule.write(getErrorReductionParameter(), "errorReductionParameter", 1);
        capsule.write(getSplit(), "split", 1);
    }
    // *************************************************************************
    // private methods

    native private void addConstraint(long jointId, long bodyId);

    native private float getConstraintForceMixing(long jointId);

    native private float getErrorReductionParameter(long jointId);

    native private float getSplit(long jointId);

    native private void removeConstraint(long jointId, long bodyId);

    native private void setConstraintForceMixing(long jointId, float cfm);

    native private void setErrorReductionParameter(long jointId, float erp);

    native private void setSplit(long jointId, float split);
}
