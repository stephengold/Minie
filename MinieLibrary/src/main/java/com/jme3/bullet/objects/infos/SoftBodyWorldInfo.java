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
package com.jme3.bullet.objects.infos;

import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.math.Vector3f;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A set of physics-simulation parameters that can be customized for each
 * PhysicsSoftBody, based on Bullet's btSoftBodyWorldInfo.
 * <p>
 * By default, all soft bodies share a single SoftBodyWorldInfo. However, this
 * is not the case in native Bullet.
 *
 * @author dokthar
 */
public class SoftBodyWorldInfo {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(PhysicsSoftBody.class.getName());
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
     * Read the maximum displacement.
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
     * Alter the maximum displacement.
     *
     * @param maxDisplacement the desired value (default=1000)
     */
    public void setMaxDisplacement(float maxDisplacement) {
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
    // private methods

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

    native private long setSoftBodyWorldInfo(long targetId, long sourceId);

    native private void setWaterDensity(long infoId, float density);

    native private void setWaterNormal(long infoId, Vector3f normalVector);

    native private void setWaterOffset(long infoId, float offset);
}
