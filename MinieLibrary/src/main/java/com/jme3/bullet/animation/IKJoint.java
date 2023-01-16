/*
 * Copyright (c) 2019-2023 jMonkeyEngine
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

import com.jme3.bullet.joints.Constraint;
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
 * A Constraint used for inverse kinematics (as opposed to joining bones and/or
 * attachments).
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
    /**
     * field names for serialization
     */
    final private static String tagConstraint = "joint";
    final private static String tagDisableForRagdoll = "disableForRagdoll";
    // *************************************************************************
    // fields

    /**
     * true&rarr;disable the Constraint when entering ragdoll mode,
     * false&rarr;Constraint unaffected by ragdoll mode
     */
    private boolean disableForRagdoll;
    /**
     * underlying Constraint
     */
    private Constraint constraint;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected IKJoint() {
    }

    /**
     * Instantiate a new IK joint.
     *
     * @param constraint the underlying Constraint (not null, alias created)
     * @param disableForRagdoll true&rarr;disable the Constraint when entering
     * ragdoll mode, false&rarr;unaffected by ragdoll mode
     */
    public IKJoint(Constraint constraint, boolean disableForRagdoll) {
        Validate.nonNull(constraint, "constraint");

        this.constraint = constraint;
        this.disableForRagdoll = disableForRagdoll;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Access the underlying Constraint.
     *
     * @return the pre-existing joint (not null)
     */
    public Constraint getPhysicsJoint() {
        return constraint;
    }

    /**
     * Test whether to disable the Constraint when entering ragdoll mode.
     *
     * @return true to disable, otherwise false
     */
    public boolean isDisableForRagdoll() {
        return disableForRagdoll;
    }

    /**
     * Alter whether to disable the Constraint when entering ragdoll mode.
     *
     * @param disableForRagdoll true to disable, false to be unaffected
     */
    public void setDisableForRagdoll(boolean disableForRagdoll) {
        this.disableForRagdoll = disableForRagdoll;
    }

    /**
     * Put the Constraint into ragdoll mode.
     */
    void setRagdollMode() {
        if (disableForRagdoll) {
            constraint.setEnabled(false);
        }
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
        this.constraint = cloner.clone(constraint);
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public IKJoint jmeClone() {
        try {
            IKJoint clone = (IKJoint) clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }
    // *************************************************************************
    // Savable methods

    /**
     * De-serialize this joint from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        InputCapsule capsule = importer.getCapsule(this);

        disableForRagdoll = capsule.readBoolean(tagDisableForRagdoll, true);
        constraint = (Constraint) capsule.readSavable(tagConstraint, null);
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
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(disableForRagdoll, tagDisableForRagdoll, true);
        capsule.write(constraint, tagConstraint, null);
    }
}
