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
 * Typical tuning parameters for a PhysicsVehicle.
 *
 * @author normenhansen
 */
public class VehicleTuning
        implements JmeCloneable, Savable {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(VehicleTuning.class.getName());

    // TODO privatize all fields
    /**
     * suspension stiffness constant (10&rarr;off-road buggy, 50&rarr;sports
     * car, 200&rarr;Formula-1 race car, default=5.88)
     */
    public float suspensionStiffness = 5.88f;
    /**
     * suspension damping when compressed (0&rarr;no damping, default=0.83)
     */
    public float suspensionCompression = 0.83f;
    /**
     * suspension damping when expanded (0&rarr;no damping, default=0.88)
     */
    public float suspensionDamping = 0.88f;
    /**
     * maximum suspension travel distance (in centimeters, default=500)
     */
    public float maxSuspensionTravelCm = 500f;
    /**
     * maximum force exerted by each wheel's suspension (default=6000)
     */
    public float maxSuspensionForce = 6000f;
    /**
     * coefficient of friction between tires and ground (0.8&rarr;realistic car,
     * 10000&rarr;kart racer, default=10.5)
     */
    public float frictionSlip = 10.5f;
    // *************************************************************************
    // JmeCloneable methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned object into a deep-cloned one, using the specified cloner
     * and original to resolve copied fields.
     *
     * @param cloner the cloner that's cloning this object (not null)
     * @param original the control from which this control was shallow-cloned
     * (unused)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
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
     * De-serialize these parameters, for example when loading from a J3O file.
     *
     * @param im importer (not null)
     * @throws IOException from importer
     */
    @Override
    public void read(JmeImporter im) throws IOException {
        InputCapsule capsule = im.getCapsule(this);
        suspensionStiffness = capsule.readFloat("suspensionStiffness", 5.88f);
        suspensionDamping
                = capsule.readFloat("wheelsDampingRelaxation", 0.88f);
        suspensionCompression
                = capsule.readFloat("wheelsDampingCompression", 0.83f);
        frictionSlip = capsule.readFloat("frictionSlip", 10.5f);
        maxSuspensionTravelCm
                = capsule.readFloat("maxSuspensionTravelCm", 500f);
        maxSuspensionForce = capsule.readFloat("maxSuspensionForce", 6000f);
    }

    /**
     * Serialize these parameters, for example when saving to a J3O file.
     *
     * @param ex exporter (not null)
     * @throws IOException from exporter
     */
    @Override
    public void write(JmeExporter ex) throws IOException {
        OutputCapsule capsule = ex.getCapsule(this);
        capsule.write(suspensionStiffness, "suspensionStiffness", 5.88f);
        capsule.write(suspensionDamping, "wheelsDampingRelaxation", 0.88f);
        capsule.write(suspensionCompression, "wheelsDampingCompression",
                0.83f);
        capsule.write(frictionSlip, "frictionSlip", 10.5f);
        capsule.write(maxSuspensionTravelCm, "maxSuspensionTravelCm", 500f);
        capsule.write(maxSuspensionForce, "maxSuspensionForce", 6000f);
    }
}
