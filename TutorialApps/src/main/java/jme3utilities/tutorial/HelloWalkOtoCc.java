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

import com.jme3.anim.AnimComposer;
import com.jme3.anim.tween.action.Action;
import com.jme3.app.SimpleApplication;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.HeightfieldCollisionShape;
import com.jme3.bullet.control.CharacterControl;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.input.KeyInput;
import com.jme3.input.controls.ActionListener;
import com.jme3.input.controls.KeyTrigger;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.material.Materials;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Spatial;
import com.jme3.shadow.DirectionalLightShadowRenderer;
import com.jme3.shadow.EdgeFilteringMode;
import com.jme3.system.AppSettings;
import com.jme3.terrain.geomipmap.TerrainQuad;
import com.jme3.terrain.heightmap.HeightMap;
import com.jme3.terrain.heightmap.ImageBasedHeightMap;
import com.jme3.texture.Image;
import com.jme3.texture.Texture;

/**
 * An example of character physics using Oto and CharacterControl.
 * <p>
 * Press the U/H/J/K keys to walk. Press the space bar to jump.
 * <p>
 * Builds upon HelloWalk.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class HelloWalkOtoCc
        extends SimpleApplication
        implements ActionListener {
    // *************************************************************************
    // fields

    private static Action standAction;
    private static Action walkAction;
    private static AnimComposer composer;
    /**
     * true when the spacebar is pressed, otherwise false
     */
    private static volatile boolean jumpRequested;
    /**
     * true when the U key is pressed, otherwise false
     */
    private static volatile boolean walkAway;
    /**
     * true when the H key is pressed, otherwise false
     */
    private static volatile boolean walkLeft;
    /**
     * true when the K key is pressed, otherwise false
     */
    private static volatile boolean walkRight;
    /**
     * true when the J key is pressed, otherwise false
     */
    private static volatile boolean walkToward;

    private static CharacterControl character;
    /**
     * PhysicsSpace for simulation
     */
    private static PhysicsSpace physicsSpace;
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the HelloWalkOtoCc application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        HelloWalkOtoCc application = new HelloWalkOtoCc();

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
        addLighting(rootNode);
        configureCamera();
        configureInput();
        physicsSpace = configurePhysics();

        // Load the Oto model and find its animation actions.
        Spatial oto = assetManager.loadModel("Models/Oto/Oto.mesh.xml");
        composer = oto.getControl(AnimComposer.class);
        standAction = composer.action("stand");
        walkAction = composer.action("Walk");

        rootNode.attachChild(oto);

        // Create the PhysicsControl and add it to the scene and space.
        float capsuleRadius = 3f;
        float capsuleHeight = 4f;
        CapsuleCollisionShape shape
                = new CapsuleCollisionShape(capsuleRadius, capsuleHeight);
        float stepHeight = 0.01f;
        character = new CharacterControl(shape, stepHeight);
        character.setGravity(60f);
        oto.addControl(character);
        physicsSpace.add(character);

        // Teleport the character to its initial location.
        character.setPhysicsLocation(new Vector3f(-73.6f, 19.09f, -45.58f));

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
        // Determine horizontal directions relative to the camera orientation.
        Vector3f away = cam.getDirection();
        away.y = 0;
        away.normalizeLocal();

        Vector3f left = cam.getLeft();
        left.y = 0;
        left.normalizeLocal();

        // Determine the walk velocity from keyboard inputs.
        Vector3f direction = new Vector3f();
        if (walkAway) {
            direction.addLocal(away);
        }
        if (walkLeft) {
            direction.addLocal(left);
        }
        if (walkRight) {
            direction.subtractLocal(left);
        }
        if (walkToward) {
            direction.subtractLocal(away);
        }
        direction.normalizeLocal();
        float walkSpeed = 7f;
        float timeStep = 1 / 60f;
        Vector3f walkOffset = direction.mult(walkSpeed * timeStep);
        character.setWalkDirection(walkOffset);

        // Decide whether to jump.
        if (jumpRequested) {
            character.jump();
        }

        // Update the animation action.
        Action action = composer.getCurrentAction();
        if (walkOffset.length() < 0.0001f) {
            if (action != standAction) {
                composer.setCurrentAction("stand");
            }
        } else {
            character.setViewDirection(direction);
            if (action != walkAction) {
                composer.setCurrentAction("Walk");
            }
        }
    }
    // *************************************************************************
    // ActionListener methods

    /**
     * Callback to handle keyboard input events.
     *
     * @param action the name of the input event
     * @param ongoing true &rarr; pressed, false &rarr; released
     * @param tpf the time per frame (in seconds, &ge;0)
     */
    @Override
    public void onAction(String action, boolean ongoing, float tpf) {
        switch (action) {
            case "jump":
                jumpRequested = ongoing;
                return;

            case "walk away":
                walkAway = ongoing;
                return;

            case "walk left":
                walkLeft = ongoing;
                return;

            case "walk right":
                walkRight = ongoing;
                return;

            case "walk toward":
                walkToward = ongoing;
                return;

            default:
                System.out.println("Unknown action: " + action);
        }
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
        scene.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);

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

        heightMap.load();
        TerrainQuad terrain
                = new TerrainQuad("terrain", 65, 513, heightMap.getHeightMap());
        rootNode.attachChild(terrain);
        Material greenMaterial = createLitMaterial(0f, 0.5f, 0f);
        terrain.setMaterial(greenMaterial);

        // Construct a static RigidBodyControl based on the HeightMap.
        CollisionShape shape = new HeightfieldCollisionShape(heightMap);
        RigidBodyControl rbc
                = new RigidBodyControl(shape, PhysicsBody.massForStatic);
        rbc.setPhysicsSpace(physicsSpace);
        terrain.addControl(rbc);
    }

    /**
     * Configure the Camera during startup.
     */
    private void configureCamera() {
        flyCam.setMoveSpeed(10f);

        cam.setLocation(new Vector3f(-39f, 34f, -47f));
        cam.setRotation(new Quaternion(0.183f, -0.68302f, 0.183f, 0.68302f));
    }

    /**
     * Configure keyboard input during startup.
     */
    private void configureInput() {
        inputManager.addMapping("jump", new KeyTrigger(KeyInput.KEY_SPACE));
        inputManager.addMapping("walk away", new KeyTrigger(KeyInput.KEY_U));
        inputManager.addMapping("walk left", new KeyTrigger(KeyInput.KEY_H));
        inputManager.addMapping("walk right", new KeyTrigger(KeyInput.KEY_K));
        inputManager.addMapping("walk toward", new KeyTrigger(KeyInput.KEY_J));
        inputManager.addListener(this,
                "jump", "walk away", "walk left", "walk right", "walk toward");
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
}
