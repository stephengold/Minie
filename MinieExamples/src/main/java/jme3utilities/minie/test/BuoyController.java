/*
 Copyright (c) 2019, Stephen Gold
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
package jme3utilities.minie.test;

import com.jme3.bounding.BoundingBox;
import com.jme3.bullet.animation.IKController;
import com.jme3.bullet.animation.PhysicsLink;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A simple IK controller to simulate buoyancy, based on the bounding box of the
 * rigid body.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class BuoyController extends IKController {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(BuoyController.class.getName());
    // *************************************************************************
    // fields

    /**
     * Y coordinate of the surface (in physics-space coordinates)
     */
    private float surfaceY;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled controller.
     *
     * @param controlledLink the link to be controlled (not null)
     * @param surfaceY the Y coordinate of the surface (in physics-space
     * coordinates)
     */
    public BuoyController(PhysicsLink controlledLink, float surfaceY) {
        super(controlledLink);
        this.surfaceY = surfaceY;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Read the Y coordinate of the surface.
     *
     * @return the surface elevation (in physics-space coordinates)
     */
    float surfaceY() {
        return surfaceY;
    }

    /**
     * Alter the Y coordinate of the surface.
     *
     * @param y the desired surface elevation (in physics-space coordinates)
     */
    void setSurfaceY(float y) {
        surfaceY = y;
    }
    // *************************************************************************
    // IKController methods

    /**
     * Apply an impulse to the controlled rigid body to simulate buoyancy.
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

        PhysicsRigidBody rigidBody = link.getRigidBody();
        BoundingBox boundingBox = rigidBody.boundingBox(null);
        float bottomY = boundingBox.getMin(null).y;
        if (surfaceY < bottomY) {
            return;
        }
        /*
         * The bounding box is at least partially submerged.
         */
        float topY = boundingBox.getMax(null).y;
        assert topY > bottomY : topY;
        float height = topY - bottomY;
        float fractionSubmerged = (surfaceY - bottomY) / height;
        fractionSubmerged = FastMath.clamp(fractionSubmerged, 0f, 1f);
        Vector3f gravity = rigidBody.getGravity(null);
        /*
         * Buoyancy will cancel gravity when the bounding box is 50% submerged.
         */
        Vector3f buoyantForce = gravity.mult(-2f * fractionSubmerged);
        float mass = rigidBody.getMass();
        Vector3f impulse = buoyantForce.mult(mass * timeStep);
        /*
         * Apply the (upward) impulse to the rigid body.
         */
        rigidBody.applyCentralImpulse(impulse);
    }

    /**
     * De-serialize this controller, for example when loading from a J3O file.
     *
     * @param importer the importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);
        InputCapsule ic = importer.getCapsule(this);
        surfaceY = ic.readFloat("surfaceY", 0f);
    }

    /**
     * Immediately put this controller into ragdoll mode.
     */
    @Override
    public void setRagdollMode() {
        // Ragdoll mode has no effect on a BuoyController.
    }

    /**
     * Serialize this controller, for example when saving to a J3O file.
     *
     * @param exporter the exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        super.write(exporter);
        OutputCapsule oc = exporter.getCapsule(this);
        oc.write(surfaceY, "surfaceY", 0f);
    }
}
