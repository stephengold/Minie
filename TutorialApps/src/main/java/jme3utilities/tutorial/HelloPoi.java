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
import com.jme3.bullet.collision.PhysicsRayTestResult;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.HeightfieldCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.debug.DebugInitListener;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.input.KeyInput;
import com.jme3.input.MouseInput;
import com.jme3.input.controls.ActionListener;
import com.jme3.input.controls.KeyTrigger;
import com.jme3.input.controls.MouseButtonTrigger;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.material.Materials;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.shadow.DirectionalLightShadowRenderer;
import com.jme3.shadow.EdgeFilteringMode;
import com.jme3.system.AppSettings;
import com.jme3.terrain.heightmap.HeightMap;
import com.jme3.terrain.heightmap.ImageBasedHeightMap;
import com.jme3.texture.Image;
import com.jme3.texture.Texture;
import java.util.List;
import jme3utilities.MeshNormals;
import jme3utilities.debug.PointVisualizer;
import jme3utilities.math.MyVector3f;

/**
 * A simple example of point-of-impact prediction.
 * <p>
 * Press the spacebar or LMB to launch a missile.
 * <p>
 * Builds upon HelloWalk.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class HelloPoi
        extends SimpleApplication
        implements ActionListener, PhysicsTickListener {
    // *************************************************************************
    // constants

    /**
     * launch speed for missiles (in physics-space units per second)
     */
    final private static float launchSpeed = 15f;
    // *************************************************************************
    // fields

    /**
     * true when a launch has been requested, but it hasn't occurred yet
     */
    private static boolean launchRequested = false;
    /**
     * Material to visualize missiles
     */
    private static Material redMaterial;
    /**
     * body to model the terrain
     */
    private static PhysicsRigidBody terrain;
    /**
     * PhysicsSpace for simulation
     */
    private static PhysicsSpace physicsSpace;
    /**
     * visualize the predicted point-of-impact
     */
    private static PointVisualizer poiIndicator;
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the HelloPoi application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        HelloPoi application = new HelloPoi();

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
        configureCamera();
        configureInput();
        physicsSpace = configurePhysics();

        redMaterial = new Material(assetManager, Materials.UNSHADED);
        redMaterial.setColor("Color", ColorRGBA.Red.clone());

        // Add an indicator for the predicted point of impact.
        int indicatorSize = 15; // in pixels
        poiIndicator = new PointVisualizer(
                assetManager, indicatorSize, ColorRGBA.Yellow, "cross");
        rootNode.attachChild(poiIndicator);
        poiIndicator.setDepthTest(true);

        // Add a static heightmap to represent the ground.
        addTerrain();
    }

    /**
     * Callback invoked once per frame.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void simpleUpdate(float tpf) {
        updateIndicator();
    }
    // *************************************************************************
    // ActionListener methods

    /**
     * Callback to handle keyboard/button input events.
     *
     * @param action the name of the input event
     * @param ongoing true &rarr; pressed, false &rarr; released
     * @param tpf the time per frame (in seconds, &ge;0)
     */
    @Override
    public void onAction(String action, boolean ongoing, float tpf) {
        switch (action) {
            case "launch":
                if (ongoing) {
                    launchRequested = true;
                }
                return;

            default:
                System.out.println("Unknown action: " + action);
        }
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
        if (launchRequested) {
            launchRequested = false;
            launchMissile();
        }
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
    // *************************************************************************
    // private methods

    /**
     * Add lighting and shadows to the specified scene and set the background
     * color.
     *
     * @param scene the scene to augment (not null)
     */
    private void addLighting(Spatial scene) {
        ColorRGBA ambientColor = new ColorRGBA(0.03f, 0.03f, 0.03f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        scene.addLight(ambient);
        ambient.setName("ambient");

        ColorRGBA directColor = new ColorRGBA(0.3f, 0.3f, 0.3f, 1f);
        Vector3f direction = new Vector3f(-7f, -3f, -5f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction, directColor);
        scene.addLight(sun);
        sun.setName("sun");

        // Render shadows based on the directional light.
        viewPort.clearProcessors();
        int shadowMapSize = 2_048; // in pixels
        int numSplits = 3;
        DirectionalLightShadowRenderer dlsr
                = new DirectionalLightShadowRenderer(
                        assetManager, shadowMapSize, numSplits);
        dlsr.setEdgeFilteringMode(EdgeFilteringMode.PCFPOISSON);
        dlsr.setEdgesThickness(5);
        dlsr.setLight(sun);
        dlsr.setShadowIntensity(0.4f);
        viewPort.addProcessor(dlsr);

        // Set the viewport's background color to light blue.
        ColorRGBA skyColor = new ColorRGBA(0.1f, 0.2f, 0.4f, 1f);
        viewPort.setBackgroundColor(skyColor);
    }

    /**
     * Add a heightfield body to the space.
     */
    private void addTerrain() {
        // Generate a HeightMap from jme3-testdata-3.1.0-stable.jar
        String assetPath = "Textures/Terrain/splat/mountains512.png";
        Texture texture = assetManager.loadTexture(assetPath);
        Image image = texture.getImage();
        HeightMap heightMap = new ImageBasedHeightMap(image);
        heightMap.setHeightScale(0.2f);

        // Construct a static rigid body based on the HeightMap.
        CollisionShape shape = new HeightfieldCollisionShape(heightMap);
        terrain = new RigidBodyControl(shape, PhysicsBody.massForStatic);

        physicsSpace.addCollisionObject(terrain);

        // Customize its debug visualization.
        Material greenMaterial = createLitMaterial(0f, 0.5f, 0f);
        terrain.setDebugMaterial(greenMaterial);
        terrain.setDebugMeshNormals(MeshNormals.Smooth);
    }

    /**
     * Configure the Camera during startup.
     */
    private void configureCamera() {
        flyCam.setMoveSpeed(10f);
        flyCam.setZoomSpeed(10f);

        cam.setLocation(new Vector3f(8f, 30f, -44f));
        cam.setRotation(new Quaternion(0f, 1f, 0f, 0f));

        float frustumWidth = cam.getFrustumTop() - cam.getFrustumBottom();
        float frustumHeight = cam.getFrustumRight() - cam.getFrustumLeft();
        float aspectRatio = frustumHeight / frustumWidth;
        float far = cam.getFrustumFar();
        float fieldOfViewDegrees = 100f; // fish-eye view

        // Bring the near plane closer to reduce clipping.
        float near = 0.1f; // default = 1
        cam.setFrustumPerspective(fieldOfViewDegrees, aspectRatio, near, far);
    }

    /**
     * Configure keyboard/button input during startup.
     */
    private void configureInput() {
        inputManager.addMapping("launch",
                new KeyTrigger(KeyInput.KEY_SPACE),
                new MouseButtonTrigger(MouseInput.BUTTON_LEFT));
        inputManager.addListener(this, "launch");
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

        // Add lighting and shadows to the debug scene.
        bulletAppState.setDebugInitListener(new DebugInitListener() {
            @Override
            public void bulletDebugInit(Node physicsDebugRootNode) {
                addLighting(physicsDebugRootNode);
            }
        });
        bulletAppState.setDebugShadowMode(
                RenderQueue.ShadowMode.CastAndReceive);

        PhysicsSpace result = bulletAppState.getPhysicsSpace();

        // To enable the callbacks, register the application as a tick listener.
        result.addTickListener(this);

        return result;
    }

    /**
     * Create a single-sided lit material with the specified reflectivities.
     *
     * @param red the desired reflectivity for red light (&ge;0, &le;1)
     * @param green the desired reflectivity for green light (&ge;0, &le;1)
     * @param blue the desired reflectivity for blue light (&ge;0, &le;1)
     * @return a new instance (not null)
     */
    private Material createLitMaterial(float red, float green, float blue) {
        Material result = new Material(assetManager, Materials.LIGHTING);
        result.setBoolean("UseMaterialColors", true);

        float opacity = 1f;
        result.setColor("Ambient", new ColorRGBA(red, green, blue, opacity));
        result.setColor("Diffuse", new ColorRGBA(red, green, blue, opacity));

        return result;
    }

    /**
     * Launch a red sphere into the PhysicsSpace.
     */
    private void launchMissile() {
        float radius = 0.5f;
        CollisionShape missileShape = new SphereCollisionShape(radius);
        float mass = 1f;
        PhysicsRigidBody missile = new PhysicsRigidBody(missileShape, mass);
        physicsSpace.addCollisionObject(missile);

        missile.setCcdMotionThreshold(radius);
        missile.setCcdSweptSphereRadius(radius);
        missile.setDebugMaterial(redMaterial);

        Vector3f velocity = cam.getDirection().mult(launchSpeed);
        missile.setLinearVelocity(velocity);

        Vector3f location = cam.getLocation();
        missile.setPhysicsLocation(location);
    }

    /**
     * Predict the point of impact for a hypothetical missile.
     *
     * @param launchLocation the missile's launch location (in physics-space
     * coordinates, not null, unaffected)
     * @param launchVelocity the missile's launch velocity (in physics-space
     * coordinates, not null, unaffected)
     * @return a new location vector (in physics-space coordinates) or null for
     * no prediction
     */
    private Vector3f predictPoi(
            Vector3f launchLocation, Vector3f launchVelocity) {
        Vector3f gravity = physicsSpace.getGravity(null);
        Vector3f velocity = launchVelocity.clone();
        Vector3f location = launchLocation.clone();
        Vector3f previousLocation = new Vector3f();
        final float timeStep = 0.02f; // seconds per step

        for (int stepIndex = 0; stepIndex < 150; ++stepIndex) {
            previousLocation.set(location);
            MyVector3f.accumulateScaled(location, velocity, timeStep);
            MyVector3f.accumulateScaled(velocity, gravity, timeStep);
            List<PhysicsRayTestResult> rayTest
                    = physicsSpace.rayTestRaw(previousLocation, location);

            // Find the closest contact with the terrain.
            float closestFraction = 9f;
            for (PhysicsRayTestResult hit : rayTest) {
                if (hit.getCollisionObject() == terrain) {
                    // ignore other missiles!
                    float hitFraction = hit.getHitFraction();
                    if (hitFraction < closestFraction) {
                        closestFraction = hitFraction;
                    }
                }
            }

            if (closestFraction <= 1f) {
                Vector3f result = MyVector3f.lerp(
                        closestFraction, previousLocation, location, null);
                return result;
            }
        }

        // The predicted impact is >3 seconds away.
        return null;
    }

    /**
     * Update the POI indicator.
     */
    private void updateIndicator() {
        // Predict the point-of-impact for a hypothetical missile
        // launched along the camera's line of sight.
        Vector3f launchLocation = cam.getLocation();
        Vector3f launchVelocity = cam.getDirection().mult(launchSpeed);
        Vector3f predictedLocation = predictPoi(launchLocation, launchVelocity);

        // Update the POI indicator.
        if (predictedLocation == null) {
            poiIndicator.setEnabled(false);
        } else {
            poiIndicator.setEnabled(true);
            poiIndicator.setLocalTranslation(predictedLocation);
        }
    }
}
