/*
 * Copyright (c) 2019 jMonkeyEngine
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

import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.util.clone.Cloner;
import com.jme3.util.clone.JmeCloneable;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A PhysicsJoint used for inverse kinematics (as opposed to joining bones
 * and/or attachments).
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class IKJoint implements JmeCloneable, Savable {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(IKJoint.class.getName());
    // *************************************************************************
    // fields

    /**
     * true&rarr;disable the joint when entering ragdoll mode, false&rarr;joint
     * unaffected by ragdoll mode
     */
    private boolean disableForRagdoll;
    /**
     * underlying PhysicsJoint
     */
    private PhysicsJoint joint;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public IKJoint() {
    }

    /**
     * Instantiate a new IK joint.
     *
     * @param joint the underlying PhysicsJoint (not null, alias created)
     * @param disableForRagdoll true&rarr;disable the joint when entering
     * ragdoll mode, false&rarr;unaffected by ragdoll mode
     */
    public IKJoint(PhysicsJoint joint, boolean disableForRagdoll) {
        Validate.nonNull(joint, "joint");

        this.joint = joint;
        this.disableForRagdoll = disableForRagdoll;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Access the underlying PhysicsJoint.
     *
     * @return the pre-existing joint (not null)
     */
    public PhysicsJoint getPhysicsJoint() {
        return joint;
    }

    /**
     * Test whether to disable the joint when entering ragdoll mode.
     *
     * @return true to disable, otherwise false
     */
    public boolean isDisableForRagdoll() {
        return disableForRagdoll;
    }

    /**
     * Alter whether to disable the joint when entering ragdoll mode.
     *
     * @param disableForRagdoll true to disable, false to be unaffected
     */
    public void setDisableForRagdoll(boolean disableForRagdoll) {
        this.disableForRagdoll = disableForRagdoll;
    }

    /**
     * Put this joint into ragdoll mode.
     */
    void setRagdollMode() {
        if (disableForRagdoll) {
            joint.setEnabled(false);
        }
    }
    // *************************************************************************
    // JmeCloneable methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned link into a deep-cloned one, using the specified cloner
     * and original to resolve copied fields.
     *
     * @param cloner the cloner that's cloning this link (not null)
     * @param original the instance from which this link was shallow-cloned
     * (unused)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        joint = cloner.clone(joint);
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public IKJoint jmeClone() {
        try {
            IKJoint clone = (IKJoint) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }
    // *************************************************************************
    // Savable methods

    /**
     * De-serialize this joint, for example when loading from a J3O file.
     *
     * @param importer importer (not null)
     * @throws IOException from importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        InputCapsule capsule = importer.getCapsule(this);

        disableForRagdoll = capsule.readBoolean("disableForRagdoll", true);
        joint = (PhysicsJoint) capsule.readSavable("joint", null);
    }

    /**
     * Serialize this preset, for example when saving to a J3O file.
     *
     * @param exporter exporter (not null)
     * @throws IOException from exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(disableForRagdoll, "disableForRagdoll", true);
        capsule.write(joint, "joint", null);
    }
}
