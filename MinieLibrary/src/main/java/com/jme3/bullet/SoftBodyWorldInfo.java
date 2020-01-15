/*
 * Copyright (c) 2009-2015 jMonkeyEngine
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
package com.jme3.bullet;

import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.math.Vector3f;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * Physics-simulation parameters that can be customized for each
 * PhysicsSoftBody, based on Bullet's btSoftBodyWorldInfo.
 * <p>
 * NOTE: When a PhysicsSoftBody is added to a PhysicsSoftSpace, it acquires the
 * SoftBodyWorldInfo of that space. To customize a body, assign it a new info
 * after adding it to the space.
 *
 * @author dokthar
 */
public class SoftBodyWorldInfo implements Savable {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(PhysicsSoftBody.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagAirDensity = "airDensity";
    final private static String tagGravity = "gravity";
    final private static String tagMaxDisplacement = "maxDisplacement";
    final private static String tagWaterDensity = "waterDensity";
    final private static String tagWaterNormal = "waterNormal";
    final private static String tagWaterOffset = "waterOffset";
    // *************************************************************************
    // fields

    /**
     * unique identifier of the btSoftBodyWorldInfo (not zero)
     * <p>
     * Multiple instances can refer to the same btSoftBodyWorldInfo.
     */
    final private long nativeId;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an info that refers to a new native object with the default
     * parameters.
     */
    public SoftBodyWorldInfo() {
        nativeId = createSoftBodyWorldInfo();
        assert nativeId != 0L;
    }

    /**
     * Instantiate an info that refers to the identified native object. Used
     * internally.
     *
     * @param nativeId the pre-existing btSoftBodyWorldInfo to refer to (not
     * zero)
     */
    public SoftBodyWorldInfo(long nativeId) {
        Validate.nonZero(nativeId, "native ID");
        this.nativeId = nativeId;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Read the air density.
     *
     * @return the density
     */
    public float airDensity() {
        return getAirDensity(nativeId);
    }

    /**
     * Copy all parameter values from the specified info.
     *
     * @param source the info to copy from (not null, unaffected)
     */
    public void copyAll(SoftBodyWorldInfo source) {
        long sourceId = source.nativeId();
        setSoftBodyWorldInfo(nativeId, sourceId);
    }

    /**
     * Copy the gravitational acceleration.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return an acceleration vector in physics-space coordinates (either
     * storeResult or a new vector, not null)
     */
    public Vector3f copyGravity(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getGravity(nativeId, result);
        return result;
    }

    /**
     * Copy the normal direction of the water surface.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a direction vector (in physics-space coordinates, either
     * storeResult or a new vector, not null)
     */
    public Vector3f copyWaterNormal(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getWaterNormal(nativeId, result);
        return result;
    }

    /**
     * Read the maximum distance a node can travel per time step.
     *
     * @return the displacement
     */
    public float maxDisplacement() {
        return getMaxDisplacement(nativeId);
    }

    /**
     * Read the unique identifier of the btSoftBodyWorldInfo.
     *
     * @return the identifier (not zero)
     */
    public long nativeId() {
        assert nativeId != 0L;
        return nativeId;
    }

    /**
     * Alter the air density.
     *
     * @param density the desired density (default=1.2)
     */
    public void setAirDensity(float density) {
        setAirDensity(nativeId, density);
    }

    /**
     * Alter the gravitational acceleration.
     *
     * @param acceleration the desired acceleration vector (in physics-space
     * coordinates, not null, unaffected, default=(0,-10,0))
     */
    public void setGravity(Vector3f acceleration) {
        setGravity(nativeId, acceleration);
    }

    /**
     * Alter the maximum distance a node can travel per time step.
     *
     * @param maxDisplacement the desired value (&gt;0, default=1000)
     */
    public void setMaxDisplacement(float maxDisplacement) {
        Validate.positive(maxDisplacement, "max displacement");
        setMaxDisplacement(nativeId, maxDisplacement);
    }

    /**
     * Alter the water density.
     *
     * @param density the desired density (default=0)
     */
    public void setWaterDensity(float density) {
        setWaterDensity(nativeId, density);
    }

    /**
     * Alter the water normal.
     *
     * @param normalDirection the desired normal direction (not null,
     * unaffected, default=(0,0,0))
     */
    public void setWaterNormal(Vector3f normalDirection) {
        setWaterNormal(nativeId, normalDirection);
    }

    /**
     * Alter the water offset.
     *
     * @param offset the desired offset distance (in physics-space units,
     * default=0)
     */
    public void setWaterOffset(float offset) {
        setWaterOffset(nativeId, offset);
    }

    /**
     * Read the water density.
     *
     * @return the density
     */
    public float waterDensity() {
        return getWaterDensity(nativeId);
    }

    /**
     * Read the water offset.
     *
     * @return the offset distance (in physics-space units)
     */
    public float waterOffset() {
        return getWaterOffset(nativeId);
    }
    // *************************************************************************
    // Savable methods

    /**
     * De-serialize this info from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        InputCapsule capsule = importer.getCapsule(this);

        setAirDensity(capsule.readFloat(tagAirDensity, 1.2f));
        setGravity((Vector3f) capsule.readSavable(tagGravity,
                new Vector3f(0f, -10f, 0f)));
        setMaxDisplacement(capsule.readFloat(tagMaxDisplacement, 1000f));
        setWaterDensity(capsule.readFloat(tagWaterDensity, 0f));
        setWaterNormal((Vector3f) capsule.readSavable(tagWaterNormal,
                new Vector3f(0f, 0f, 0f)));
        setWaterOffset(capsule.readFloat(tagWaterOffset, 0f));
    }

    /**
     * Serialize this info to the specified exporter, for example when saving to
     * a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(airDensity(), tagAirDensity, 1.2f);
        capsule.write(copyGravity(null), tagGravity, null);
        capsule.write(maxDisplacement(), tagMaxDisplacement, 1000f);
        capsule.write(waterDensity(), tagWaterDensity, 0f);
        capsule.write(copyWaterNormal(null), tagWaterNormal, null);
        capsule.write(waterOffset(), tagWaterOffset, 0f);
    }
    // *************************************************************************
    // native methods

    native private long createSoftBodyWorldInfo();

    native private float getAirDensity(long infoId);

    native private void getGravity(long infoId, Vector3f storeVector);

    native private float getMaxDisplacement(long infoId);

    native private float getWaterDensity(long infoId);

    native private void getWaterNormal(long infoId, Vector3f storeVector);

    native private float getWaterOffset(long infoId);

    native private void setAirDensity(long infoId, float density);

    native private void setGravity(long infoId, Vector3f gravityVector);

    native private void setMaxDisplacement(long infoId, float displacement);

    native private void setSoftBodyWorldInfo(long targetId, long sourceId);

    native private void setWaterDensity(long infoId, float density);

    native private void setWaterNormal(long infoId, Vector3f normalVector);

    native private void setWaterOffset(long infoId, float offset);
}
