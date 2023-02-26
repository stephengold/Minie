/*
 * Copyright (c) 2022 jMonkeyEngine
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

import com.jme3.anim.Armature;
import com.jme3.anim.Joint;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Transform;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.control.AbstractControl;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * Helper control for DynamicAnimControl, to hide armature modifications from an
 * AnimComposer.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class PreComposer extends AbstractControl {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(PreComposer.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagDac = "dac";
    final private static String tagHaveSaved = "haveSaved";
    final private static String tagSavedTransforms = "savedTransforms";
    // *************************************************************************
    // fields

    /**
     * true if saveArmature() has been invoked since the last update
     */
    private boolean haveSaved = false;
    /**
     * DynamicAnimControl that created this helper
     */
    private DacLinks dac;
    /**
     * saved joint transforms, allocated lazily
     */
    private Transform[] savedTransforms;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected PreComposer() {
    }

    /**
     * Instantiate an enabled PreComposer to help the specified
     * DynamicAnimControl hide its Armature modifications from AnimComposer.
     *
     * @param dac the DynamicAnimControl to be helped (not null)
     */
    PreComposer(DacLinks dac) {
        Validate.nonNull(dac, "dac");
        this.dac = dac;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Save the Armature data that DynamicAnimControl might modify: currently
     * just the joint transforms.
     */
    void saveArmature() {
        Armature armature = dac.getArmature();
        int numJoints = armature.getJointCount();
        if (savedTransforms == null) {
            this.savedTransforms = new Transform[numJoints];
            for (int jointIndex = 0; jointIndex < numJoints; ++jointIndex) {
                savedTransforms[jointIndex] = new Transform();
            }
        } else {
            assert savedTransforms.length == numJoints : numJoints;
        }

        for (int jointIndex = 0; jointIndex < numJoints; ++jointIndex) {
            Joint joint = armature.getJoint(jointIndex);
            Transform t = joint.getLocalTransform(); // alias
            savedTransforms[jointIndex].set(t);
        }

        this.haveSaved = true;
    }
    // *************************************************************************
    // AbstractControl methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned Control into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this Control (not null)
     * @param original the instance from which this Control was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);

        this.dac = cloner.clone(dac);
        this.savedTransforms = cloner.clone(savedTransforms);
    }

    /**
     * Update this control. Invoked once per frame during the logical-state
     * update, provided the control is enabled and added to a scene. Should be
     * invoked only by a subclass or by AbstractControl.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    protected void controlUpdate(float tpf) {
        if (!haveSaved) {
            return;
        }

        // Restore all joint transforms to their saved values.
        Armature armature = dac.getArmature();
        int numJoints = armature.getJointCount();
        assert savedTransforms.length == numJoints : numJoints;
        for (int jointIndex = 0; jointIndex < numJoints; ++jointIndex) {
            Joint joint = armature.getJoint(jointIndex);
            Transform t = savedTransforms[jointIndex]; // alias
            joint.setLocalTransform(t);
        }

        this.haveSaved = false;
    }

    /**
     * Render this Control. Invoked once per ViewPort per frame, provided the
     * Control is enabled and added to a scene. Should be invoked only by a
     * subclass or by AbstractControl.
     *
     * @param rm the render manager (unused)
     * @param vp the view port to render (unused)
     */
    @Override
    protected void controlRender(RenderManager rm, ViewPort vp) {
        // do nothing
    }

    /**
     * De-serialize this Control from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);
        InputCapsule capsule = importer.getCapsule(this);

        this.haveSaved = capsule.readBoolean(tagHaveSaved, false);
        this.dac = (DacLinks) capsule.readSavable(tagDac, null);
        this.savedTransforms
                = RagUtils.readTransformArray(capsule, tagSavedTransforms);
    }

    /**
     * Serialize this Control to the specified exporter, for example when saving
     * to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        super.write(exporter);
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(haveSaved, tagHaveSaved, false);
        capsule.write(dac, tagDac, null);
        capsule.write(savedTransforms, tagSavedTransforms, null);
    }
}
