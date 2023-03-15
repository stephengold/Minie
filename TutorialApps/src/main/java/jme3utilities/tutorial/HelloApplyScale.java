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
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Box;
import com.jme3.system.AppSettings;

/**
 * A simple example of a kinematic RigidBodyControl with setApplyScale(true).
 * <p>
 * Builds upon HelloKinematicRbc.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class HelloApplyScale extends SimpleApplication {
    // *************************************************************************
    // fields

    /**
     * physics-simulation time (in seconds, &ge;0)
     */
    private float elapsedTime = 0f;
    /**
     * cube geometry, varying in size
     */
    private Geometry cubeGeometry;
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the HelloApplyScale application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        HelloApplyScale application = new HelloApplyScale();

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
        PhysicsSpace physicsSpace = configurePhysics();

        // Create the cube Geometry and add it to the scene graph.
        Material cubeMaterial = new Material(assetManager, Materials.LIGHTING);
        Mesh cubeMesh = new Box(1f, 1f, 1f);
        cubeGeometry = new Geometry("kine", cubeMesh);
        cubeGeometry.setMaterial(cubeMaterial);
        rootNode.attachChild(cubeGeometry);

        // Create an RBC and add it to the Geometry.
        float mass = 2f;
        RigidBodyControl kineRbc = new RigidBodyControl(mass);
        cubeGeometry.addControl(kineRbc);

        // Add the control to the space.
        kineRbc.setPhysicsSpace(physicsSpace);

        // Set the kinematic and "apply scale" flags on the RBC.
        kineRbc.setKinematic(true);
        kineRbc.setApplyScale(true);

        addLighting(rootNode);
        configureCamera();
    }

    /**
     * Callback invoked once per frame.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void simpleUpdate(float tpf) {
        // Vary the scale of the Geometry with time.
        float cycleTime = 3f; // seconds
        float phaseAngle = elapsedTime * FastMath.TWO_PI / cycleTime;

        float scaleFactor = 1f + 0.5f * FastMath.sin(phaseAngle);
        Vector3f scale = Vector3f.UNIT_XYZ.mult(scaleFactor);
        cubeGeometry.setLocalScale(scale);

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
     * Configure the camera during startup (for a better view).
     */
    private void configureCamera() {
        cam.setLocation(new Vector3f(1f, 1.452773f, 10.1f));
        cam.setRotation(new Quaternion(0f, 0.99891f, -0.043f, -0.017f));
    }

    /**
     * Configure physics during startup.
     *
     * @return a new instance (not null)
     */
    private PhysicsSpace configurePhysics() {
        BulletAppState bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);

        // Enable debug visualization to reveal what occurs in physics space.
        bulletAppState.setDebugEnabled(true);

        // Direct debug visuals to a post ViewPort that clears the depth buffer.
        // This prevents z-fighting between the box and its debug visuals.
        ViewPort overlay = renderManager.createPostView("Overlay", cam);
        overlay.setClearFlags(false, true, false);
        bulletAppState.setDebugViewPorts(overlay);

        PhysicsSpace result = bulletAppState.getPhysicsSpace();

        return result;
    }
}
