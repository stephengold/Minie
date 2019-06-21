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
 * A SoftPhysicsJoint that allows rotation around an axis, based on Bullet's
 * btSoftBody::AJoint.
 *
 * @author dokthar
 */
public class SoftAngularJoint extends SoftPhysicsJoint {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger3
            = Logger.getLogger(SoftAngularJoint.class.getName());
    // *************************************************************************
    // fields

    /**
     * copy of the joint axis (in physics-space coordinates)
     */
    private Vector3f axis;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public SoftAngularJoint() {
    }

    /**
     * Instantiate a soft-rigid angular joint. The soft body ({@code nodeA})
     * must contain a cluster.
     *
     * @param axis the axis of the joint (in physics-space coordinates, not
     * null, unaffected)
     * @param nodeA the soft body for the A end (not null, alias created)
     * @param nodeB the rigid body for the B end (not null, alias created)
     * @param pivotA the pivot location in A's scaled local coordinates (not
     * null, unaffected)
     * @param pivotB the pivot location in B's scaled local coordinates (not
     * null, unaffected)
     */
    public SoftAngularJoint(Vector3f axis, PhysicsSoftBody nodeA,
            PhysicsRigidBody nodeB, Vector3f pivotA, Vector3f pivotB) {
        super(nodeA, nodeB, pivotA, pivotB);
        assert nodeA.countClusters() > 0;

        this.axis = axis.clone();
        createJoint();
    }

    /**
     * Instantiate a soft-soft angular joint. Each soft body must contain a
     * cluster.
     *
     * @param axis the axis of the joint (in physics-space coordinates, not
     * null, unaffected)
     * @param nodeA the soft body for the A end (not null, alias created)
     * @param nodeB the soft body for the B end (not null, alias created)
     * @param pivotA the pivot location in A's scaled local coordinates (not
     * null, unaffected)
     * @param pivotB the pivot location in B's scaled local coordinates (not
     * null, unaffected)
     */
    public SoftAngularJoint(Vector3f axis, PhysicsSoftBody nodeA,
            PhysicsSoftBody nodeB, Vector3f pivotA, Vector3f pivotB) {
        super(nodeA, nodeB, pivotA, pivotB);
        assert nodeA.countClusters() > 0;
        assert nodeB.countClusters() > 0;

        this.axis = axis.clone();
        createJoint();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the joint axis.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the axis (in physics-space coordinates, either storeResult or a
     * new vector, not null)
     */
    public Vector3f getAxis(Vector3f storeResult) {
        // TODO verify copy
        if (storeResult == null) {
            return axis.clone();
        } else {
            return storeResult.set(axis);
        }
    }

    /**
     * Alter the joint axis.
     *
     * @param newAxis the desired axis (in physics-space coordinates, not null,
     * unaffected)
     */
    public void setAxis(Vector3f newAxis) {
        axis.set(newAxis);
        setAxis(objectId, newAxis);
    }
    // *************************************************************************
    // SoftPhysicsJoint methods

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

        axis = cloner.clone(axis);
        createJoint();

        SoftAngularJoint old = (SoftAngularJoint) original;

        float bit = old.getBreakingImpulseThreshold();
        setBreakingImpulseThreshold(bit);

        boolean enableJoint = old.isEnabled();
        setEnabled(enableJoint);
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public SoftAngularJoint jmeClone() {
        try {
            SoftAngularJoint clone = (SoftAngularJoint) super.clone();
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
        axis = (Vector3f) capsule.readSavable("axis", new Vector3f());
        createJoint();
        readJointProperties(capsule);
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
        capsule.write(axis, "axis", null);
    }
    // *************************************************************************
    // private methods

    /**
     * Create the configured joint in Bullet.
     */
    private void createJoint() {
        assert objectId == 0L : objectId;
        assert nodeA instanceof PhysicsSoftBody;
        assert ((PhysicsSoftBody)nodeA).countClusters() > 0;

        if (isSoftRigidJoint()) {
            objectId = createJointSoftRigid(nodeA.getObjectId(),
                    nodeB.getObjectId(), pivotA, pivotB,
                    errorReductionParameter, constraintForceMixing, split,
                    axis);
        } else if (isSoftSoftJoint()) {
            assert ((PhysicsSoftBody)nodeB).countClusters() > 0;
            objectId = createJointSoftSoft(nodeA.getObjectId(),
                    nodeB.getObjectId(), pivotA, pivotB,
                    errorReductionParameter, constraintForceMixing, split,
                    axis);
        }

        assert objectId != 0L;
        logger3.log(Level.FINE, "Created Joint {0}",
                Long.toHexString(objectId));
    }

    private native long createJointSoftRigid(long objectIdA, long objectIdB,
            Vector3f pivotA, Vector3f pivotB, float erp, float cfm, float split,
            Vector3f axis);

    private native long createJointSoftSoft(long objectIdA, long objectIdB,
            Vector3f pivotA, Vector3f pivotB, float erp, float cfm, float split,
            Vector3f axis);

    private native void setAxis(long jointId, Vector3f axis);
}
