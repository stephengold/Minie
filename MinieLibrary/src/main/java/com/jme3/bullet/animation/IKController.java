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

import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.util.clone.Cloner;
import com.jme3.util.clone.JmeCloneable;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.MyString;

/**
 * An abstract inverse kinematics (IK) controller for a PhysicsLink in dynamic
 * mode.
 *
 * @author Stephen Gold sgold@sonic.net
 */
abstract public class IKController implements JmeCloneable, Savable {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(IKController.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagControlledLink = "controlledLink";
    final private static String tagIsEnabled = "isEnabled";
    // *************************************************************************
    // fields

    /**
     * true &rarr; enabled, false &rarr; disabled
     */
    private boolean isEnabled;
    /**
     * PhysicsLink controlled by this controller
     */
    private PhysicsLink controlledLink;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected IKController() {
    }

    /**
     * Instantiate an enabled controller.
     *
     * @param controlledLink the link to be controlled (not null)
     */
    protected IKController(PhysicsLink controlledLink) {
        assert controlledLink != null;
        if (logger.isLoggable(Level.FINE)) {
            logger.log(Level.FINE, "Creating controller for bone {0}.",
                    MyString.quote(controlledLink.boneName()));
        }

        this.controlledLink = controlledLink;
        isEnabled = true;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Access the controlled link.
     *
     * @return the pre-existing instance (not null)
     */
    public PhysicsLink getLink() {
        return controlledLink;
    }

    /**
     * Test whether this controller is enabled.
     *
     * @return the pre-existing instance (not null)
     */
    public boolean isEnabled() {
        return isEnabled;
    }

    /**
     * Apply forces, impulses, and torques to the rigid body. Invoked just
     * before the physics is stepped.
     *
     * @param timeStep the physics time step (in seconds, &ge;0)
     */
    abstract public void preTick(float timeStep);

    /**
     * Enable or disable this controller.
     *
     * @param desiredSetting true to enable, false to disable
     */
    public void setEnabled(boolean desiredSetting) {
        isEnabled = desiredSetting;
    }

    /**
     * Immediately put this controller into ragdoll mode. Unless overridden,
     * this method simply disables the controller.
     */
    public void setRagdollMode() {
        isEnabled = false;
    }
    // *************************************************************************
    // JmeCloneable methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned controller into a deep-cloned one, using the specified
     * Cloner and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this controller (not null)
     * @param original the instance from which this controller was
     * shallow-cloned (unused)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        this.controlledLink = cloner.clone(controlledLink);
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public IKController jmeClone() {
        try {
            IKController clone = (IKController) clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }
    // *************************************************************************
    // Savable methods

    /**
     * De-serialize this controller from the specified importer, for example
     * when loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        InputCapsule capsule = importer.getCapsule(this);

        isEnabled = capsule.readBoolean(tagIsEnabled, true);
        controlledLink
                = (PhysicsLink) capsule.readSavable(tagControlledLink, null);
    }

    /**
     * Serialize this controller to the specified exporter, for example when
     * saving to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(isEnabled, tagIsEnabled, true);
        capsule.write(controlledLink, tagControlledLink, null);
    }
}
