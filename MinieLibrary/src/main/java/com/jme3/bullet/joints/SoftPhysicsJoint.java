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
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * The abstract base class for joining a PhysicsSoftBody to another body, based
 * on Bullet's btSoftBody::Joint. There are 2 kinds of SoftPhysicsJoint:
 * <ul>
 * <li>a soft-soft joint joins particular clusters of 2 distinct soft
 * bodies</li>
 * <li>a soft-rigid joint joins a particular cluster of a soft body to a rigid
 * body</li>
 * </ul>
 * Subclasses include SoftLinearJoint and SoftAngularJoint.
 * <p>
 * To join a particular node of a soft body to a rigid body, append an anchor to
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
    final public static Logger logger2
            = Logger.getLogger(SoftPhysicsJoint.class.getName());
    /**
     * local copy of {@link com.jme3.math.Vector3f#ZERO}
     */
    final private static Vector3f translateIdentity = new Vector3f(0f, 0f, 0f);
    // *************************************************************************
    // fields TODO privatize

    /**
     * true if this joint is added to soft body A
     */
    private boolean added = false;
    /**
     * local copy of the constraint force mixing parameter (default=1)
     */
    protected float cfm = 1f;
    /**
     * local copy of the error-reduction parameter (default=1)
     */
    protected float erp = 1f;
    /**
     * local copy (default=1)
     */
    protected float split = 1f;
    /**
     * the index of the cluster for the A end (&ge;0)
     */
    private int clusterIndexA = -1;
    /**
     * the index of the cluster for the B end, or -1 for a soft-rigid joint
     */
    private int clusterIndexB = -1;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    protected SoftPhysicsJoint() {
    }

    /**
     * Instantiate a SoftPhysicsJoint to join a soft-body cluster and a rigid
     * body.
     * <p>
     * To be effective, the joint must be added to the PhysicsSoftSpace of both
     * bodies.
     *
     * @param softBodyA the soft body for the A end (not null, alias created)
     * @param clusterIndexA the index of the cluster for the A end (&ge;0)
     * @param rigidBodyB the rigid body for the B end (not null, alias created)
     */
    public SoftPhysicsJoint(PhysicsSoftBody softBodyA, int clusterIndexA,
            PhysicsRigidBody rigidBodyB) {
        super(softBodyA, rigidBodyB, translateIdentity, translateIdentity);

        int numClustersA = softBodyA.countClusters();
        Validate.inRange(clusterIndexA, "cluster index", 0, numClustersA - 1);

        this.clusterIndexA = clusterIndexA;
    }

    /**
     * Instantiate a SoftPhysicsJoint to join 2 soft bodies.
     * <p>
     * To be effective, the joint must be added to the PhysicsSoftSpace of both
     * bodies. Also, the bodies must be distinct.
     *
     * @param softBodyA the body for the A end (not null, alias created)
     * @param clusterIndexA the index of the cluster for the A end (&ge;0)
     * @param softBodyB the body for the B end (not null, alias created)
     * @param clusterIndexB the index of the cluster for the B end (&ge;0)
     */
    public SoftPhysicsJoint(PhysicsSoftBody softBodyA, int clusterIndexA,
            PhysicsSoftBody softBodyB, int clusterIndexB) {
        super(softBodyA, softBodyB, translateIdentity, translateIdentity);

        int numClustersA = softBodyA.countClusters();
        Validate.inRange(clusterIndexA, "cluster index A", 0, numClustersA - 1);
        int numClustersB = softBodyB.countClusters();
        Validate.inRange(clusterIndexB, "cluster index B", 0, numClustersB - 1);

        this.clusterIndexA = clusterIndexA;
        this.clusterIndexB = clusterIndexB;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Add this constraint to body A. This method should be invoked only by the
     * PhysicsSoftSpace.
     */
    public void addConstraint() {
        if (!added) {
            addConstraint(objectId, nodeA.getObjectId());
            added = true;
        }
    }

    public int clusterIndexA() {
        return clusterIndexA;
    }

    public int clusterIndexB() {
        return clusterIndexB;
    }

    /**
     * Get the constraint force mixing coefficient (aka CFM).
     * <p>
     * From the Bullet documentation:</p>
     * <ul>
     * <li>If CFM=0 then the constraint will be hard.
     * <li>If CFM is set to a positive value, it will be possible to violate the
     * constraint by "pushing on it" (for example, for contact constraints by
     * forcing the two contacting objects together). In other words the
     * constraint will be soft, and the softness will increase as CFM increases.
     * </ul>
     *
     * @return the coefficient value (&ge;0)
     */
    public float getCFM() {
        return getConstraintForceMixing(objectId);
    }

    /**
     * Get the error-reduction parameter (aka ERP).
     * <p>
     * From the Bullet documentation:</p>
     * <p>
     * The ERP specifies what proportion of the joint error will be fixed during
     * the next simulation step.
     * </p>
     * <ul>
     * <li>If ERP=0 then no correcting force is applied and the bodies will
     * eventually drift apart as the simulation proceeds.
     * <li>If ERP=1 then the simulation will attempt to fix all joint error
     * during the next time step. However, setting ERP=1 is not recommended, as
     * the joint error will not be completely fixed due to various internal
     * approximations.
     * </ul>
     * Values between 0.1 and 0.8 are recommended.
     *
     * @return the parameter value (&ge;0, &le;1, default=1)
     */
    public float getERP() {
        return getErrorReductionParameter(objectId);
    }

    /**
     * Access the soft body at the A end.
     *
     * @return the pre-existing soft body, or null if none
     */
    public PhysicsSoftBody getSoftBodyA() {
        PhysicsSoftBody result = null;
        if (nodeA instanceof PhysicsSoftBody) {
            result = (PhysicsSoftBody) nodeA;
        }
        return result;
    }

    /**
     * Access the soft body at the B end.
     *
     * @return the pre-existing soft body, or null if none
     */
    public PhysicsSoftBody getSoftBodyB() {
        PhysicsSoftBody result = null;
        if (nodeB instanceof PhysicsSoftBody) {
            result = (PhysicsSoftBody) nodeB;
        }
        return result;
    }

    /**
     * TODO description
     *
     * @return the split value
     */
    public float getSplit() {
        return getSplit(objectId);
    }

    /**
     * Test whether this joint is a soft-rigid joint.
     *
     * @return true if soft-rigid, otherwise false
     */
    public boolean isSoftRigidJoint() {
        return nodeB instanceof PhysicsRigidBody;
    }

    /**
     * Test whether this joint is a soft-soft joint.
     *
     * @return true if soft-soft, otherwise false
     */
    public boolean isSoftSoftJoint() {
        return nodeB instanceof PhysicsSoftBody;
    }

    /**
     * Remove this constraint from body A. This method should be invoked only by
     * the PhysicsSoftSpace.
     */
    public void removeConstraint() {
        if (added) {
            removeConstraint(objectId, nodeA.getObjectId());
            added = false;
        }
    }

    /**
     * Set the constraint force mixing coefficient (aka CFM).
     * <p>
     * From the Bullet documentation:</p>
     * <ul>
     * <li>If CFM=0 then the constraint will be hard.
     * <li>If CFM is set to a positive value, it will be possible to violate the
     * constraint by "pushing on it" (for example, for contact constraints by
     * forcing the two contacting objects together). In other words the
     * constraint will be soft, and the softness will increase as CFM increases.
     * </ul>
     * <p>
     * Note that setting CFM to a negative value can have undesirable bad
     * effects, such as instability. Don't do it.
     *
     * @param cfm the desired coefficient value (&ge;0, default=1)
     */
    public void setCFM(float cfm) {
        Validate.nonNegative(cfm, "CFM coefficient");
        setConstraintForceMixing(objectId, cfm);
    }

    /**
     * Set the error-reduction parameter (aka ERP).
     * <p>
     * From the Bullet documentation:</p>
     * <p>
     * The ERP specifies what proportion of the joint error will be fixed during
     * the next simulation step.
     * </p>
     * <ul>
     * <li>If ERP=0 then no correcting force is applied and the bodies will
     * eventually drift apart as the simulation proceeds.
     * <li>If ERP=1 then the simulation will attempt to fix all joint error
     * during the next time step. However, setting ERP=1 is not recommended, as
     * the joint error will not be completely fixed due to various internal
     * approximations.
     * </ul>
     * Values between 0.1 and 0.8 are recommended.
     *
     * @param erp the desired parameter value (&ge;0, &le;1, default=1)
     */
    public void setERP(float erp) {
        Validate.fraction(erp, "error-reduction parameter");
        setErrorReductionParameter(objectId, erp);
    }

    /**
     * TODO description
     *
     * @param split the desired split value (default=1)
     */
    public void setSplit(float split) {
        setSplit(objectId, split);
    }
    // *************************************************************************
    // PhysicsJoint methods

    /**
     * Finalize this joint just before it is destroyed. Should be invoked only
     * by a subclass or by the garbage collector.
     *
     * @throws Throwable ignored by the garbage collector
     */
    @Override
    protected void finalize() throws Throwable {
        if (added) {
            // the joint is still attached to a softbody,
            // assuming the joint is Garbage collected at the same time as his softbody
            // (the softbody have a reference to this in his joint list)
            // when deleted the softbody will delete all his joints as well.
            logger2.log(Level.FINE,
                    "SoftPhysicsJoint {0} is still attached, it will be destroyed by the softBody",
                    Long.toHexString(objectId));
        } else {
            // finalizeNative() will be invoked by the superclass finalize
            super.finalize();
        }
    }

    @Override
    native protected void finalizeNative(long jointId);

    @Override
    public boolean isCollisionBetweenLinkedBodies() {
        return false; //TODO ??
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

        cfm = capsule.readFloat("constraintForceMixing", 1f);
        erp = capsule.readFloat("errorReductionParameter", 1f);
        split = capsule.readFloat("split", 1f);

        clusterIndexA = capsule.readInt("clusterIndexA", -1);
        clusterIndexB = capsule.readInt("clusterIndexB", -1);
        /*
         * Each subclass must create the btTypedConstraint and then
         * invoke readJointProperties() .
         */
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

        capsule.write(getCFM(), "constraintForceMixing", 1);
        capsule.write(getERP(), "errorReductionParameter", 1);
        capsule.write(getSplit(), "split", 1);

        capsule.write(clusterIndexA(), "clusterIndexA", -1);
        capsule.write(clusterIndexB(), "clusterIndexB", -1);
    }
    // *************************************************************************
    // private methods

    native private void addConstraint(long jointId, long softBodyId);

    native private float getConstraintForceMixing(long jointId);

    native private float getErrorReductionParameter(long jointId);

    native private float getSplit(long jointId);

    native private void removeConstraint(long jointId, long softBodyId);

    native private void setConstraintForceMixing(long jointId, float cfm);

    native private void setErrorReductionParameter(long jointId, float erp);

    native private void setSplit(long jointId, float split);
}
