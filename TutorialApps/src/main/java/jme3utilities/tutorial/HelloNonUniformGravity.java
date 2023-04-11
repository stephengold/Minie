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
import com.jme3.math.Vector3f;
import jme3utilities.debug.AxesVisualizer;
import jme3utilities.minie.FilterAll;

/**
 * A simple example of non-uniform gravity.
 * <p>
 * Builds upon HelloRigidBody.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class HelloNonUniformGravity
        extends SimpleApplication
        implements PhysicsTickListener {
    // *************************************************************************
    // fields

    private static PhysicsRigidBody planet;
    final private static Vector3f tmpVector = new Vector3f();
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the HelloNonUniformGravity application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        HelloNonUniformGravity application = new HelloNonUniformGravity();
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

        // Reduce the time step for better accuracy.
        physicsSpace.setAccuracy(0.005f);
        /*
         * Enable debug visualization
         * (including gravity-vector visualization)
         * to reveal what occurs in physics space.
         */
        bulletAppState.setDebugEnabled(true);
        bulletAppState.setDebugGravityVectorFilter(new FilterAll(true));

        // Create a CollisionShape for the planet.
        float planetRadius = 0.1f;
        CollisionShape planetShape = new SphereCollisionShape(planetRadius);

        // Create a planet (dynamic rigid body) and add it to the space.
        float planetMass = 1f; // physics mass unit = 10^25 kg
        planet = new PhysicsRigidBody(planetShape, planetMass);
        physicsSpace.addCollisionObject(planet);

        // Prevent deactivation of the planet.
        planet.setEnableSleep(false);

        // Kick the planet into orbit around the central black hole.
        planet.setPhysicsLocation(new Vector3f(2f, 0f, 0f));
        planet.applyCentralImpulse(new Vector3f(0f, -1f, 0f));

        // Visualize axes to indicate the black hole's location.
        float axisLength = 1f;
        AxesVisualizer axes = new AxesVisualizer(assetManager, axisLength);
        axes.setLineWidth(AxesVisualizer.widthForSolid);
        rootNode.addControl(axes);
        axes.setEnabled(true);

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
        // Calculate the gravitational acceleration GM/r^2.
        planet.getPhysicsLocation(tmpVector);
        float r2 = tmpVector.lengthSquared(); //squared distance from black hole
        tmpVector.normalizeLocal();
        tmpVector.multLocal(-3f / r2);
        planet.setGravity(tmpVector);
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
