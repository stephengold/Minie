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
package com.jme3.bullet.objects.infos;

import com.jme3.bullet.NativePhysicsObject;
import com.jme3.bullet.objects.PhysicsSoftBody;
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
 * Provide access to 3 fields of the native btSoftBody::Material struct.
 */
public class SoftBodyMaterial
        extends NativePhysicsObject
        implements JmeCloneable, Savable {
    // *********************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(SoftBodyMaterial.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagAngularStiffness = "angularStiffness";
    final private static String tagBody = "body";
    final private static String tagLinearStiffness = "linearStiffness";
    final private static String tagVolumeStiffness = "volumeStiffness";
    // *************************************************************************
    // fields

    /**
     * corresponding soft body
     */
    private PhysicsSoftBody body = null;
    // *********************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected SoftBodyMaterial() {
    }

    /**
     * Instantiate a material with the default properties.
     *
     * @param body the body to which this material will apply (not null)
     */
    public SoftBodyMaterial(PhysicsSoftBody body) {
        assert body != null;
        this.body = body;

        long softBodyId = body.nativeId();
        long materialId = getMaterialId(softBodyId);
        super.setNativeIdNotTracked(materialId);
    }
    // *********************************************************************
    // new methods exposed

    /**
     * Read the angular-stiffness coefficient (native field: m_kAST).
     *
     * @return the coefficient (&ge;0, &le;1)
     */
    public float angularStiffness() {
        long materialId = nativeId();
        float result = getAngularStiffnessFactor(materialId);

        return result;
    }

    /**
     * Read the linear-stiffness coefficient (native field: m_kLST).
     *
     * @return the coefficient (&ge;0, &le;1)
     */
    public float linearStiffness() {
        long materialId = nativeId();
        float result = getLinearStiffnessFactor(materialId);

        return result;
    }

    /**
     * Alter the angular-stiffness coefficient (native field: m_kAST).
     *
     * @param coefficient the desired coefficient (&ge;0, &le;1, default=1)
     */
    public void setAngularStiffness(float coefficient) {
        Validate.fraction(coefficient, "stiffness coefficient");

        long materialId = nativeId();
        setAngularStiffnessFactor(materialId, coefficient);
    }

    /**
     * Alter the linear-stiffness coefficient (native field: m_kLST).
     *
     * @param coefficient the desired coefficient (&ge;0, &le;1, default=1)
     */
    public void setLinearStiffness(float coefficient) {
        Validate.fraction(coefficient, "stiffness coefficient");

        long materialId = nativeId();
        setLinearStiffnessFactor(materialId, coefficient);
    }

    /**
     * Alter the volume-stiffness coefficient (native field: m_kVST).
     *
     * @param coefficient the desired coefficient (&ge;0, &le;1, default=1)
     */
    public void setVolumeStiffness(float coefficient) {
        Validate.fraction(coefficient, "stiffness coefficient");

        long materialId = nativeId();
        setVolumeStiffnessFactor(materialId, coefficient);
    }

    /**
     * Read the volume-stiffness coefficient (native field: m_kVST).
     *
     * @return the coefficient (&ge;0, &le;1)
     */
    public float volumeStiffness() {
        long materialId = nativeId();
        float result = getVolumeStiffnessFactor(materialId);

        return result;
    }
    // *********************************************************************
    // JmeCloneable methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned material into a deep-cloned one, using the specified
     * Cloner and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this material (not null)
     * @param original the instance from which this material was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        unassignNativeObject();

        this.body = cloner.clone(body);
        long softBodyId = body.nativeId();
        long materialId = getMaterialId(softBodyId);
        setNativeIdNotTracked(materialId);

        SoftBodyMaterial old = (SoftBodyMaterial) original;
        setAngularStiffness(old.angularStiffness());
        setLinearStiffness(old.linearStiffness());
        setVolumeStiffness(old.volumeStiffness());
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public SoftBodyMaterial jmeClone() {
        try {
            SoftBodyMaterial clone = (SoftBodyMaterial) clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }
    // *********************************************************************
    // Savable methods

    /**
     * De-serialize this material from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        assert body == null;

        InputCapsule capsule = importer.getCapsule(this);
        this.body = (PhysicsSoftBody) capsule.readSavable(tagBody, null);

        long softBodyId = body.nativeId();
        long materialId = getMaterialId(softBodyId);
        setNativeIdNotTracked(materialId);

        setAngularStiffness(capsule.readFloat(tagAngularStiffness, 1f));
        setLinearStiffness(capsule.readFloat(tagLinearStiffness, 1f));
        setVolumeStiffness(capsule.readFloat(tagVolumeStiffness, 1f));
    }

    /**
     * Serialize this material to the specified exporter, for example when
     * saving to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(body, tagBody, null);

        capsule.write(angularStiffness(), tagAngularStiffness, 1f);
        capsule.write(linearStiffness(), tagLinearStiffness, 1f);
        capsule.write(volumeStiffness(), tagVolumeStiffness, 1f);
    }
    // *********************************************************************
    // native private methods

    native private static float getAngularStiffnessFactor(long materialId);

    native private static float getLinearStiffnessFactor(long materialId);

    native private static long getMaterialId(long bodyId);

    native private static float getVolumeStiffnessFactor(long materialId);

    native private static void
            setAngularStiffnessFactor(long materialId, float stiffness);

    native private static void
            setLinearStiffnessFactor(long materialId, float stiffness);

    native private static void
            setVolumeStiffnessFactor(long materialId, float stiffness);
}
