/*
 Copyright (c) 2020-2023, Stephen Gold
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
package jme3utilities.tutorial;

import com.jme3.app.SimpleApplication;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;

/**
 * A simple example to demonstrate the use of principalAxes() and correctAxes()
 * to improve the plausibility of a compound shape.
 * <p>
 * Builds upon HelloMadMallet.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class HelloMassDistribution extends SimpleApplication {
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the HelloMassDistribution application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        HelloMassDistribution application = new HelloMassDistribution();
        application.start();
    }
    // *************************************************************************
    // SimpleApplication methods

    /**
     * Initialize this application.
     */
    @Override
    public void simpleInitApp() {
        // Set up Bullet physics and create a physics space.
        BulletAppState bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);
        bulletAppState.setDebugEnabled(true); // for debug visualization
        PhysicsSpace physicsSpace = bulletAppState.getPhysicsSpace();

        physicsSpace.setGravity(new Vector3f(0f, -50f, 0f));

        // Visualize the local axes of each collision object.
        bulletAppState.setDebugAxisLength(1f);

        // Construct a compound shape for the mallet.
        float headLength = 1f;
        float headRadius = 0.5f;
        Vector3f hes = new Vector3f(headLength / 2f, headRadius, headRadius);
        CollisionShape headShape
                = new CylinderCollisionShape(hes, PhysicsSpace.AXIS_X);

        float handleLength = 3f;
        float handleRadius = 0.3f;
        hes.set(handleRadius, handleRadius, handleLength / 2f);
        CollisionShape handleShape
                = new CylinderCollisionShape(hes, PhysicsSpace.AXIS_Z);

        CompoundCollisionShape malletShape = new CompoundCollisionShape();
        malletShape.addChildShape(handleShape, 0f, 0f, handleLength / 2f);
        malletShape.addChildShape(headShape, 0f, 0f, handleLength);

        // Calculate a correction to put 75% of the mass in the head.
        float handleMass = 0.5f;
        float headMass = 1.5f;
        FloatBuffer massDistribution = BufferUtils.createFloatBuffer(
                handleMass, headMass);
        Vector3f inertiaVector = new Vector3f();
        Transform correction = malletShape.principalAxes(
                massDistribution, null, inertiaVector);

        // Correct the shape.
        malletShape.correctAxes(correction);

        // Create a dynamic body for the mallet.
        float mass = handleMass + headMass;
        PhysicsRigidBody mallet = new PhysicsRigidBody(malletShape, mass);
        mallet.setPhysicsLocation(new Vector3f(0f, 4f, 0f));

        // Increase the mallet's angular damping to stabilize it.
        mallet.setAngularDamping(0.9f);

        // The mallet's center has changed, so adjust its moment of inertia.
        Vector3f inverseInertia = Vector3f.UNIT_XYZ.divide(inertiaVector);
        mallet.setInverseInertiaLocal(inverseInertia);

        physicsSpace.addCollisionObject(mallet);

        // Create a static disc and add it to the space.
        float discRadius = 5f;
        float discThickness = 0.5f;
        CollisionShape discShape = new CylinderCollisionShape(
                discRadius, discThickness, PhysicsSpace.AXIS_Y);
        PhysicsRigidBody disc
                = new PhysicsRigidBody(discShape, PhysicsBody.massForStatic);
        physicsSpace.addCollisionObject(disc);
        disc.setPhysicsLocation(new Vector3f(0f, -3f, 0f));

        // Re-position the camera for a better view.
        cam.setLocation(new Vector3f(10f, -2.75f, 0f));
        Vector3f targetLocation = new Vector3f(0f, -2.75f, 0f);
        Vector3f upDirection = Vector3f.UNIT_Y;
        cam.lookAt(targetLocation, upDirection);

        // Minie's BulletAppState simulates the dynamics...
    }
}
