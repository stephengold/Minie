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

import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * The abstract base class for joining a PhysicsSoftBody to another body, based
 * on Bullet's btSoftBody::Joint. A SoftPhysicsJoint can be soft-soft or
 * soft-rigid:
 * <ul>
 * <li>a soft-soft joint joins particular clusters of 2 distinct soft
 * bodies</li>
 * <li>a soft-rigid joint joins a particular cluster of a soft body to a rigid
 * body</li>
 * </ul>
 * Subclasses include SoftLinearJoint and SoftAngularJoint.
 * <p>
 * To join a particular *node* of a soft body to a rigid body, use an Anchor
 * instead.
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
     * field names for serialization
     */
    final private static String tagClusterIndexA = "clusterIndexA";
    final private static String tagClusterIndexB = "clusterIndexB";
    final private static String tagConstraintForceMixing
            = "constraintForceMixing";
    final private static String tagErrorReductionParameter
            = "errorReductionParameter";
    final private static String tagSplit = "split";
    // *************************************************************************
    // fields

    /**
     * true if this joint is added to soft body A - TODO never set true!
     */
    private boolean added = false;
    /**
     * local copy of the constraint force mixing parameter (default=1)
     */
    private float cfm = 1f;
    /**
     * local copy of the error-reduction parameter (default=1)
     */
    private float erp = 1f;
    /**
     * local copy of the split parameter (default=1)
     */
    private float split = 1f;
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
     * No-argument constructor needed by SavableClassUtil.
     */
    protected SoftPhysicsJoint() {
    }

    /**
     * Instantiate a SoftPhysicsJoint to join a soft-body cluster and a rigid
     * body.
     * <p>
     * To be fully effective, the joint must be added to the PhysicsSoftSpace of
     * both bodies.
     *
     * @param softBodyA the soft body for the A end (not null, alias created)
     * @param clusterIndexA the index of the cluster for the A end (&ge;0)
     * @param rigidBodyB the rigid body for the B end (not null, alias created)
     */
    public SoftPhysicsJoint(PhysicsSoftBody softBodyA, int clusterIndexA,
            PhysicsRigidBody rigidBodyB) {
        int numClustersA = softBodyA.countClusters();
        Validate.inRange(clusterIndexA, "cluster index", 0, numClustersA - 1);
        Validate.nonNull(rigidBodyB, "rigid body B");

        setBodyA(softBodyA);
        softBodyA.addJoint(this);
        this.clusterIndexA = clusterIndexA;

        setBodyB(rigidBodyB);
        rigidBodyB.addJoint(this);
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
        int numClustersA = softBodyA.countClusters();
        Validate.inRange(clusterIndexA, "cluster index A", 0, numClustersA - 1);
        int numClustersB = softBodyB.countClusters();
        Validate.inRange(clusterIndexB, "cluster index B", 0, numClustersB - 1);

        setBodyA(softBodyA);
        softBodyA.addJoint(this);
        this.clusterIndexA = clusterIndexA;

        setBodyB(softBodyB);
        softBodyB.addJoint(this);
        this.clusterIndexB = clusterIndexB;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Compare Bullet parameters against their the local copies.
     *
     * @return true if the local copies are accurate, otherwise false
     */
    public boolean checkParameters() {
        long jointId = getObjectId();
        boolean result = (cfm == getConstraintForceMixing(jointId))
                && (erp == getErrorReductionParameter(jointId))
                && (split == getSplit(jointId));

        return result;
    }

    /**
     * Read the index of the cluster for the A end.
     *
     * @return the index (&ge;0)
     */
    public int clusterIndexA() {
        assert clusterIndexA >= 0 : clusterIndexA;
        return clusterIndexA;
    }

    /**
     * Read the index of the cluster for the B end.
     *
     * @return the index (&ge;0) or -1 for a soft-rigid joint
     */
    public int clusterIndexB() {
        return clusterIndexB;
    }

    /**
     * Read the constraint force mixing coefficient (aka CFM).
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
        return cfm;
    }

    /**
     * Read the error-reduction parameter (aka ERP).
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
        return erp;
    }

    /**
     * Access the soft body at the A end.
     *
     * @return the pre-existing soft body (not null)
     */
    public PhysicsSoftBody getSoftBodyA() {
        PhysicsBody a = getBodyA();
        assert a != null;
        PhysicsSoftBody result = (PhysicsSoftBody) a;

        return result;
    }

    /**
     * Access the soft body at the B end.
     *
     * @return the pre-existing soft body, or null for a soft-rigid joint
     */
    public PhysicsSoftBody getSoftBodyB() {
        PhysicsBody b = getBodyB();
        PhysicsSoftBody result = null;
        if (b instanceof PhysicsSoftBody) {
            result = (PhysicsSoftBody) b;
        }

        return result;
    }

    /**
     * TODO description
     *
     * @return the split value
     */
    public float getSplit() {
        return split;
    }

    /**
     * Test whether this joint is a soft-rigid joint.
     *
     * @return true if soft-rigid, otherwise false
     */
    public boolean isSoftRigid() {
        PhysicsBody b = getBodyB();
        return b instanceof PhysicsRigidBody;
    }

    /**
     * Test whether this joint is a soft-soft joint.
     *
     * @return true if soft-soft, otherwise false
     */
    public boolean isSoftSoft() {
        PhysicsBody b = getBodyB();
        return b instanceof PhysicsSoftBody;
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
     * Setting CFM to a negative value can cause instability.
     *
     * @param cfm the desired coefficient value (&ge;0, default=1)
     */
    public void setCFM(float cfm) {
        Validate.nonNegative(cfm, "CFM coefficient");

        long jointId = getObjectId();
        setConstraintForceMixing(jointId, cfm);
        this.cfm = cfm;
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

        long jointId = getObjectId();
        setErrorReductionParameter(jointId, erp);
        this.erp = erp;
    }

    /**
     * TODO description
     *
     * @param split the desired split value (default=1)
     */
    public void setSplit(float split) {
        long jointId = getObjectId();
        setSplit(jointId, split);
        this.split = split;
    }
    // *************************************************************************
    // new protected methods

    /**
     * Finalize the btTypedConstraint.
     *
     * @param jointId identifier of the btSoftBody::Joint (not 0)
     */
    native protected void finalizeNative(long jointId);
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
                    "{0} is still attached, it will be destroyed by the soft body",
                    this);
        } else {
            // finalizeNative() will be invoked by the superclass finalize
            super.finalize();
        }
    }

    /**
     * Test whether this joint is enabled.
     *
     * @return true
     */
    @Override
    public boolean isEnabled() {
        return true;
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

        cfm = capsule.readFloat(tagConstraintForceMixing, 1f);
        erp = capsule.readFloat(tagErrorReductionParameter, 1f);
        split = capsule.readFloat(tagSplit, 1f);

        clusterIndexA = capsule.readInt(tagClusterIndexA, -1);
        clusterIndexB = capsule.readInt(tagClusterIndexB, -1);
        /*
         * Each subclass must create the btSoftBody::Joint.
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

        capsule.write(getCFM(), tagConstraintForceMixing, 1);
        capsule.write(getERP(), tagErrorReductionParameter, 1);
        capsule.write(getSplit(), tagSplit, 1);

        capsule.write(clusterIndexA(), tagClusterIndexA, -1);
        capsule.write(clusterIndexB(), tagClusterIndexB, -1);
    }
    // *************************************************************************
    // native methods

    native private float getConstraintForceMixing(long jointId);

    native private float getErrorReductionParameter(long jointId);

    native private float getSplit(long jointId);

    native private void setConstraintForceMixing(long jointId, float cfm);

    native private void setErrorReductionParameter(long jointId, float erp);

    native private void setSplit(long jointId, float split);
}
