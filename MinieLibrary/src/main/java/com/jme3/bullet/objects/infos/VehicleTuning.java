/*
 * Copyright (c) 2009-2018 jMonkeyEngine
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

import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.util.clone.Cloner;
import com.jme3.util.clone.JmeCloneable;
import java.io.IOException;
import java.util.logging.Logger;

/**
 * Tuning parameters for a PhysicsVehicle, based on btVehicleTuning.
 *
 * @author normenhansen
 */
public class VehicleTuning implements JmeCloneable, Savable {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(VehicleTuning.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagFrictionSlip = "frictionSlip";
    final private static String tagMaxSuspensionForce = "maxSuspensionForce";
    final private static String tagMaxSuspensionTravelCm
            = "maxSuspensionTravelCm";
    final private static String tagSuspensionStiffness = "suspensionStiffness";
    final private static String tagWheelsDampingRelaxation
            = "wheelsDampingRelaxation";
    final private static String tagWheelsDampingCompression
            = "wheelsDampingCompression";
    // *************************************************************************
    // fields

    /**
     * coefficient of friction between tires and ground (0.8&rarr;realistic car,
     * 10000&rarr;kart racer, default=10.5)
     */
    private float frictionSlip = 10.5f;
    /**
     * maximum force exerted by each wheel's suspension (default=6000)
     */
    private float maxSuspensionForce = 6000f;
    /**
     * maximum suspension travel distance (in centimeters, default=500)
     */
    private float maxSuspensionTravelCm = 500f;
    /**
     * suspension damping when compressed (0&rarr;no damping, default=0.83)
     */
    private float suspensionCompression = 0.83f;
    /**
     * suspension damping when expanded (0&rarr;no damping, default=0.88)
     */
    private float suspensionDamping = 0.88f;
    /**
     * suspension stiffness constant (10&rarr;off-road buggy, 50&rarr;sports
     * car, 200&rarr;Formula-1 race car, default=5.88)
     */
    private float suspensionStiffness = 5.88f;
    /**
     * unique identifier of the btVehicleTuning
     */
    private long nativeId = 0L;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an instance with the default parameter values.
     */
    public VehicleTuning() {
        create();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Read the friction between tires and ground (native field:
     * m_frictionSlip).
     *
     * @return the coefficient of friction
     */
    public float getFrictionSlip() {
        return frictionSlip;
    }

    /**
     * Read the maximum force exerted by each wheel's suspension (native field:
     * m_maxSuspensionForce).
     *
     * @return the maximum force
     */
    public float getMaxSuspensionForce() {
        return maxSuspensionForce;
    }

    /**
     * Read the travel distance for each wheel's suspension (native field:
     * m_maxSuspensionTravelCm).
     *
     * @return the maximum travel distance (in centimeters)
     */
    public float getMaxSuspensionTravelCm() {
        return maxSuspensionTravelCm;
    }

    /**
     * Read the native ID of the btCollisionShape.
     *
     * @return the unique identifier (not zero)
     */
    final public long getNativeId() {
        assert nativeId != 0L;
        return nativeId;
    }

    /**
     * Read the suspension damping when compressed (native field:
     * m_suspensionCompression).
     *
     * @return the damping amount
     */
    public float getSuspensionCompression() {
        return suspensionCompression;
    }

    /**
     * Read the suspension damping when expanded (native field:
     * m_suspensionDamping).
     *
     * @return the damping amount (0&rarr;no damping, default=0.88)
     */
    public float getSuspensionDamping() {
        return suspensionDamping;
    }

    /**
     * Read the suspension stiffness (native field: m_suspensionStiffness).
     *
     * @return the stiffness constant
     */
    public float getSuspensionStiffness() {
        return suspensionStiffness;
    }

    /**
     * Alter the friction between tires and ground (native field:
     * m_frictionSlip).
     *
     * @param coeff the desired coefficient of friction (0.8&rarr;realistic car,
     * 10000&rarr;kart racer, default=10.5)
     */
    public void setFrictionSlip(float coeff) {
        frictionSlip = coeff;
        setFrictionSlip(nativeId, coeff);
    }

    /**
     * Alter the force exerted by each wheel's suspension (native field:
     * m_maxSuspensionForce).
     *
     * @param maxForce the desired maximum force (default=6000)
     */
    public void setMaxSuspensionForce(float maxForce) {
        maxSuspensionForce = maxForce;
        setMaxSuspensionForce(nativeId, maxForce);
    }

    /**
     * Alter the travel distance for the suspension (native field:
     * m_maxSuspensionTravelCm).
     *
     * @param travelCm the desired maximum travel distance (in centimeters,
     * default=500)
     */
    public void setMaxSuspensionTravelCm(float travelCm) {
        maxSuspensionTravelCm = travelCm;
        setMaxSuspensionTravelCm(nativeId, travelCm);
    }

    /**
     * Alter the suspension damping when compressed (native field:
     * m_suspensionCompression).
     *
     * @param damping the desired damping amount (0&rarr;no damping,
     * default=0.83)
     */
    public void setSuspensionCompression(float damping) {
        suspensionCompression = damping;
        setSuspensionCompression(nativeId, damping);
    }

    /**
     * Alter the suspension damping when expanded (native field:
     * m_suspensionDamping).
     *
     * @param damping the desired damping (0&rarr;no damping, default=0.88)
     */
    public void setSuspensionDamping(float damping) {
        suspensionDamping = damping;
        setSuspensionDamping(nativeId, damping);
    }

    /**
     * Alter the stiffness of the suspension (native field:
     * m_suspensionStiffness).
     *
     * @param stiffness the desired stiffness constant (10&rarr;off-road buggy,
     * 50&rarr;sports car, 200&rarr;Formula-1 race car, default=5.88)
     */
    public void setSuspensionStiffness(float stiffness) {
        suspensionStiffness = stiffness;
        setSuspensionStiffness(nativeId, stiffness);
    }
    // *************************************************************************
    // JmeCloneable methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned tuning into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this tuning (unused)
     * @param original the instance from which this tuning was shallow-cloned
     * (unused)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        create();
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public VehicleTuning jmeClone() {
        try {
            VehicleTuning clone = (VehicleTuning) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }
    // *************************************************************************
    // Savable methods

    /**
     * De-serialize these parameters from the specified importer, for example
     * when loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        InputCapsule capsule = importer.getCapsule(this);

        suspensionStiffness = capsule.readFloat(tagSuspensionStiffness, 5.88f);
        suspensionDamping
                = capsule.readFloat(tagWheelsDampingRelaxation, 0.88f);
        suspensionCompression
                = capsule.readFloat(tagWheelsDampingCompression, 0.83f);
        frictionSlip = capsule.readFloat(tagFrictionSlip, 10.5f);
        maxSuspensionTravelCm
                = capsule.readFloat(tagMaxSuspensionTravelCm, 500f);
        maxSuspensionForce = capsule.readFloat(tagMaxSuspensionForce, 6000f);

        create();
    }

    /**
     * Serialize these parameters, for example when saving to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(suspensionStiffness, tagSuspensionStiffness, 5.88f);
        capsule.write(suspensionDamping, tagWheelsDampingRelaxation, 0.88f);
        capsule.write(suspensionCompression, tagWheelsDampingCompression,
                0.83f);
        capsule.write(frictionSlip, tagFrictionSlip, 10.5f);
        capsule.write(maxSuspensionTravelCm, tagMaxSuspensionTravelCm, 500f);
        capsule.write(maxSuspensionForce, tagMaxSuspensionForce, 6000f);
    }
    // *************************************************************************
    // private methods

    /**
     * Instantiate the configured tuning in Bullet.
     */
    private void create() {
        nativeId = createNative();

        setFrictionSlip(nativeId, frictionSlip);
        setMaxSuspensionForce(nativeId, maxSuspensionForce);
        setMaxSuspensionTravelCm(nativeId, maxSuspensionTravelCm);
        setSuspensionCompression(nativeId, suspensionCompression);
        setSuspensionDamping(nativeId, suspensionDamping);
        setSuspensionStiffness(nativeId, suspensionStiffness);
    }
    // *************************************************************************
    // native methods

    native private long createNative();

    native private void setFrictionSlip(long tuningId, float slip);

    native private void setMaxSuspensionForce(long tuningId, float maxForce);

    native private void setMaxSuspensionTravelCm(long tuningId, float travel);

    native private void setSuspensionCompression(long tuningId, float damping);

    native private void setSuspensionDamping(long tuningId, float damping);

    native private void setSuspensionStiffness(long tuningId, float stiff);
}
