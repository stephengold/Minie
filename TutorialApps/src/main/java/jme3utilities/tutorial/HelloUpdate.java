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
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.material.Materials;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Sphere;
import com.jme3.system.AppSettings;

/**
 * A simple example without an AppState.
 * <p>
 * Builds upon HelloRbc.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class HelloUpdate extends SimpleApplication {
    // *************************************************************************
    // fields

    /**
     * PhysicsSpace for simulation
     */
    private static PhysicsSpace physicsSpace;
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the HelloUpdate application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        HelloUpdate application = new HelloUpdate();

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
        // Create the physics space.
        physicsSpace = new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);

        // Create a material and a mesh for balls.
        float ballRadius = 1f;
        Material ballMaterial = new Material(assetManager, Materials.LIGHTING);
        Mesh ballMesh = new Sphere(16, 32, ballRadius);

        // Create geometries for a dynamic ball and a static ball
        // and add them to the scene graph.
        Geometry dyna = new Geometry("dyna", ballMesh);
        dyna.setMaterial(ballMaterial);
        rootNode.attachChild(dyna);

        Geometry stat = new Geometry("stat", ballMesh);
        stat.setMaterial(ballMaterial);
        rootNode.attachChild(stat);

        // Create RBCs for both balls and add them to the geometries.
        float mass = 2f;
        RigidBodyControl dynaRbc = new RigidBodyControl(mass);
        dyna.addControl(dynaRbc);

        RigidBodyControl statRbc
                = new RigidBodyControl(PhysicsBody.massForStatic);
        stat.addControl(statRbc);

        // Add the controls to the physics space.
        dynaRbc.setPhysicsSpace(physicsSpace);
        statRbc.setPhysicsSpace(physicsSpace);

        // Position the balls in physics space.
        dynaRbc.setPhysicsLocation(new Vector3f(0f, 4f, 0f));
        statRbc.setPhysicsLocation(new Vector3f(0.1f, 0f, 0f));

        // Add lighting.
        addLighting(rootNode);
    }

    /**
     * Callback invoked once per frame to update the PhysicsSpace.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void simpleUpdate(float tpf) {
        physicsSpace.update(tpf);
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
}
