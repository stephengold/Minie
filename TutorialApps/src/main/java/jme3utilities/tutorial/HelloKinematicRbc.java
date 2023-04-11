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
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.material.Materials;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Sphere;
import com.jme3.system.AppSettings;

/**
 * A simple example combining kinematic and dynamic rigid-body controls.
 * <p>
 * Builds upon HelloKinematics and HelloRbc.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class HelloKinematicRbc extends SimpleApplication {
    // *************************************************************************
    // fields

    /**
     * physics-simulation time (in seconds, &ge;0)
     */
    private static float elapsedTime = 0f;
    /**
     * kinematic ball, orbiting the origin
     */
    private Geometry kine;
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the HelloKinematicRbc application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        HelloKinematicRbc application = new HelloKinematicRbc();

        // Enable gamma correction for accurate lighting.
        boolean loadDefaults = true;
        AppSettings settings = new AppSettings(loadDefaults);
        settings.setGammaCorrection(true);
        application.setSettings(settings);

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
        PhysicsSpace physicsSpace = configurePhysics();

        // Create a material and a mesh for balls.
        float ballRadius = 1f;
        Material ballMaterial = new Material(assetManager, Materials.LIGHTING);
        Mesh ballMesh = new Sphere(16, 32, ballRadius);

        // Create geometries for a dynamic ball and a kinematic ball
        // and add them to the scene graph.
        Geometry dyna = new Geometry("dyna", ballMesh);
        dyna.setMaterial(ballMaterial);
        rootNode.attachChild(dyna);

        kine = new Geometry("kine", ballMesh);
        kine.setMaterial(ballMaterial);
        rootNode.attachChild(kine);

        // Create RBCs for both balls and add them to the geometries.
        float mass = 2f;
        RigidBodyControl dynaRbc = new RigidBodyControl(mass);
        dyna.addControl(dynaRbc);
        RigidBodyControl kineRbc = new RigidBodyControl(mass);
        kine.addControl(kineRbc);

        // Add the controls to the physics space.
        dynaRbc.setPhysicsSpace(physicsSpace);
        kineRbc.setPhysicsSpace(physicsSpace);

        // Position the dynamic ball in physics space.
        dynaRbc.setPhysicsLocation(new Vector3f(0f, 4f, 0f));

        // Set the kinematic flag on the other ball.
        kineRbc.setKinematic(true);

        // Add lighting.
        addLighting(rootNode);

        // Minie's BulletAppState simulates the dynamics...
    }

    /**
     * Callback invoked once per frame.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void simpleUpdate(float tpf) {
        // Cause the kinematic ball to orbit the origin.
        float orbitalPeriod = 0.8f; // seconds
        float phaseAngle = elapsedTime * FastMath.TWO_PI / orbitalPeriod;

        float orbitRadius = 0.4f; // world units
        float x = orbitRadius * FastMath.sin(phaseAngle);
        float y = orbitRadius * FastMath.cos(phaseAngle);
        Vector3f location = new Vector3f(x, y, 0f);
        kine.setLocalTranslation(location);

        elapsedTime += tpf;
    }
    // *************************************************************************
    // private methods

    /**
     * Add lighting to the specified scene.
     *
     * @param scene the scene to augment (not null)
     */
    private static void addLighting(Spatial scene) {
        // Light the scene with ambient and directional lights.
        ColorRGBA ambientColor = new ColorRGBA(0.02f, 0.02f, 0.02f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        scene.addLight(ambient);
        ambient.setName("ambient");

        ColorRGBA directColor = new ColorRGBA(0.2f, 0.2f, 0.2f, 1f);
        Vector3f direction = new Vector3f(-7f, -3f, -5f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction, directColor);
        scene.addLight(sun);
        sun.setName("sun");
    }

    /**
     * Configure physics during startup.
     *
     * @return a new instance (not null)
     */
    private PhysicsSpace configurePhysics() {
        BulletAppState bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);
        //bulletAppState.setDebugEnabled(true); // for debug visualization
        PhysicsSpace result = bulletAppState.getPhysicsSpace();

        // Reduce the time step for better accuracy.
        result.setAccuracy(0.005f);

        return result;
    }
}
