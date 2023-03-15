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
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;

/**
 * A simple example combining kinematic and dynamic rigid bodies.
 * <p>
 * Builds upon HelloStaticBody.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class HelloKinematics
        extends SimpleApplication
        implements PhysicsTickListener {
    // *************************************************************************
    // fields

    /**
     * physics-simulation time (in seconds, &ge;0)
     */
    private static float elapsedTime = 0f;
    /**
     * kinematic ball, orbiting the origin
     */
    private PhysicsRigidBody kineBall;
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the HelloKinematics application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        HelloKinematics application = new HelloKinematics();
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
        PhysicsSpace physicsSpace = bulletAppState.getPhysicsSpace();

        // To enable the callbacks, register the application as a tick listener.
        physicsSpace.addTickListener(this);

        // Enable debug visualization to reveal what occurs in physics space.
        bulletAppState.setDebugEnabled(true);

        // Create a CollisionShape for balls.
        float ballRadius = 1f;
        CollisionShape ballShape = new SphereCollisionShape(ballRadius);

        // Create a dynamic body and add it to the space.
        float mass = 2f;
        PhysicsRigidBody dynaBall = new PhysicsRigidBody(ballShape, mass);
        physicsSpace.addCollisionObject(dynaBall);
        dynaBall.setPhysicsLocation(new Vector3f(0f, 4f, 0f));

        // Create a kinematic body and add it to the space.
        kineBall = new PhysicsRigidBody(ballShape);
        physicsSpace.addCollisionObject(kineBall);
        kineBall.setKinematic(true);

        // Minie's BulletAppState simulates the dynamics...
    }
    // *************************************************************************
    // PhysicsTickListener methods

    /**
     * Callback from Bullet, invoked just before each simulation step.
     *
     * @param space the space that's about to be stepped (not null)
     * @param timeStep the time per simulation step (in seconds, &ge;0)
     */
    @Override
    public void prePhysicsTick(PhysicsSpace space, float timeStep) {
        // Make the kinematic ball orbit the origin.
        float orbitalPeriod = 0.8f; // seconds
        float phaseAngle = elapsedTime * FastMath.TWO_PI / orbitalPeriod;

        float orbitRadius = 0.4f; // physics-space units
        float x = orbitRadius * FastMath.sin(phaseAngle);
        float y = orbitRadius * FastMath.cos(phaseAngle);
        Vector3f location = new Vector3f(x, y, 0f);
        kineBall.setPhysicsLocation(location);

        elapsedTime += timeStep;
    }

    /**
     * Callback from Bullet, invoked just after each simulation step.
     *
     * @param space the space that was just stepped (not null)
     * @param timeStep the time per simulation step (in seconds, &ge;0)
     */
    @Override
    public void physicsTick(PhysicsSpace space, float timeStep) {
        // do nothing
    }
}
