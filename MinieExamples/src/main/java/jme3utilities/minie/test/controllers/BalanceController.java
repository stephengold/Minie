/*
 Copyright (c) 2018-2023, Stephen Gold
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package jme3utilities.minie.test.controllers;

import com.jme3.bullet.animation.DynamicAnimControl;
import com.jme3.bullet.animation.IKController;
import com.jme3.bullet.animation.PhysicsLink;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * An IK controller for balancing a model (keeping its center of mass directly
 * above its center of support). If the model oscillates, try reducing the gain
 * factors.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class BalanceController extends IKController {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(BalanceController.class.getName());
    // *************************************************************************
    // fields

    /**
     * factor used to calculate the location gain
     */
    private float locationGainFactor = 0.02f;
    /**
     * factor used to calculate the velocity gain
     */
    private float velocityGainFactor = 0.02f;
    /**
     * location of the center of support (in physics-space coordinates, not
     * null)
     */
    private Vector3f centerOfSupport;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled controller.
     *
     * @param controlledLink the link to be controlled (not null)
     * @param centerOfSupport the initial location of the center of support (in
     * physics-space coordinates, not null, unaffected)
     */
    public BalanceController(
            PhysicsLink controlledLink, Vector3f centerOfSupport) {
        super(controlledLink);
        this.centerOfSupport = centerOfSupport.clone();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the support location.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in physics-space coordinates, either
     * storeResult or a new vector)
     */
    public Vector3f centerOfSupport(Vector3f storeResult) {
        Vector3f result;
        if (storeResult == null) {
            result = centerOfSupport.clone();
        } else {
            result = storeResult.set(centerOfSupport);
        }

        return result;
    }

    /**
     * Read the location gain factor.
     *
     * @return the gain factor
     */
    public float locationGainFactor() {
        return locationGainFactor;
    }

    /**
     * Alter the support location.
     *
     * @param newLocation the desired location (not null, unaffected)
     */
    public void setCenterOfSupport(Vector3f newLocation) {
        centerOfSupport.set(newLocation);
    }

    /**
     * Alter the location gain factor.
     *
     * @param newFactor the desired factor (default = 0.02)
     */
    public void setLocationGainFactor(float newFactor) {
        locationGainFactor = newFactor;
    }

    /**
     * Alter the velocity gain factor.
     *
     * @param newFactor the desired factor (default = 0.02)
     */
    public void setVelocityGainFactor(float newFactor) {
        velocityGainFactor = newFactor;
    }

    /**
     * Read the velocity gain factor.
     *
     * @return the gain factor
     */
    public float velocityGainFactor() {
        return velocityGainFactor;
    }
    // *************************************************************************
    // IKController methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned controller into a deep-cloned one, using the specified
     * cloner and original to resolve copied fields.
     *
     * @param cloner the cloner that's cloning this controller (not null)
     * @param original the instance from which this controller was
     * shallow-cloned (unused)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);
        this.centerOfSupport = cloner.clone(centerOfSupport);
    }

    /**
     * Apply an impulse to the controlled rigid body to keep the model's center
     * of mass located directly above its center of support. Meant to be invoked
     * by the controlled link before each simulation step.
     *
     * @param timeStep the physics timestep (in seconds, &ge;0)
     */
    @Override
    public void preTick(float timeStep) {
        Validate.nonNegative(timeStep, "time step");
        if (!isEnabled()) {
            return;
        }

        PhysicsLink link = getLink();
        assert !link.isKinematic();

        DynamicAnimControl dac = (DynamicAnimControl) link.getControl();
        Vector3f comLocation = new Vector3f();
        Vector3f comVelocity = new Vector3f();
        float ragdollMass = dac.centerOfMass(comLocation, comVelocity);

        // error = setpoint - actual
        Vector3f locationError = centerOfSupport.subtract(comLocation);
        Vector3f velocityError = comVelocity.negate(); // velocity setpoint = 0

        // Calculate an impulse.
        Vector3f sum = new Vector3f(0f, 0f, 0f);
        if (locationError.y < 0f) { // center of mass is ABOVE center of support
            // location term
            locationError.y = 0f;
            float locationGain = locationGainFactor * ragdollMass / timeStep;
            MyVector3f.accumulateScaled(sum, locationError, locationGain);
            // TODO add a lag term
            // velocity term
            float velocityGain = velocityGainFactor * ragdollMass;
            MyVector3f.accumulateScaled(sum, velocityError, velocityGain);
        }

        // Apply the impulse to the center of the controlled link's rigid body.
        PhysicsRigidBody rigidBody = link.getRigidBody();
        rigidBody.applyCentralImpulse(sum);
    }

    /**
     * De-serialize this controller from the specified importer, for example
     * when loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);
        InputCapsule capsule = importer.getCapsule(this);

        this.locationGainFactor
                = capsule.readFloat("locationGainFactor", 0.02f);
        this.velocityGainFactor
                = capsule.readFloat("velocityGainFactor", 0.02f);
        this.centerOfSupport = (Vector3f) capsule.readSavable(
                "centerOfSupport", new Vector3f());
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
        super.write(exporter);
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(locationGainFactor, "locationGainFactor", 0.02f);
        capsule.write(velocityGainFactor, "velocityGainFactor", 0.02f);
        capsule.write(centerOfSupport, "centerOfSupport", null);
    }
}
