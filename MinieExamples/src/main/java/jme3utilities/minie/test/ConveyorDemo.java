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
package jme3utilities.minie.test;

import com.jme3.app.Application;
import com.jme3.app.StatsAppState;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.SolverMode;
import com.jme3.bullet.collision.ContactListener;
import com.jme3.bullet.collision.ContactPointFlag;
import com.jme3.bullet.collision.ManifoldPoints;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.debug.DebugInitListener;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.font.BitmapText;
import com.jme3.input.CameraInput;
import com.jme3.input.KeyInput;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.system.AppSettings;
import com.jme3.util.BufferUtils;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.prefs.BackingStoreException;
import jme3utilities.Heart;
import jme3utilities.MeshNormals;
import jme3utilities.MyAsset;
import jme3utilities.MyString;
import jme3utilities.math.MyMath;
import jme3utilities.minie.test.common.PhysicsDemo;
import jme3utilities.ui.InputMode;

/**
 * Simulate conveyor belts using the ContactListener interface.
 *
 * @author qwq
 */
public class ConveyorDemo
        extends PhysicsDemo
        implements ContactListener, DebugInitListener {
    // *************************************************************************
    // constants and loggers

    /**
     * half the length of each conveyor belt (in physics-space units)
     */
    final private static float beltLength = 4f;
    /**
     * half the width of each conveyor belt (in physics-space units)
     */
    final private static float beltWidth = 0.5f;
    /**
     * mass of each dynamic rigid body (in physics mass units)
     */
    final private static float massForDynamic = 1f;
    /**
     * half the thickness of each belt and wall (in physics-space units)
     */
    final private static float thickness = 0.02f;
    /**
     * half the height of each wall (in physics-space units)
     */
    final private static float wallHeight = 0.2f;
    /**
     * physics-space X coordinate for the center of the scene
     */
    final private static float x0 = -1f;
    /**
     * physics-space Y coordinate for the centers of all static bodies
     */
    final private static float y0 = 0.5f;
    /**
     * speeds the conveyor belts (in physics-space units per second)
     */
    final private static float[] beltSpeeds = {5f, 1f, 5f, 1f};
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(ConveyorDemo.class.getName());
    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName
            = ConveyorDemo.class.getSimpleName();
    /**
     * initial location for drops (in physics-space coordinates)
     */
    final private static Vector3f spawnLocation = new Vector3f(0f, 3f, 0f);
    /**
     * directions of motion of the conveyor belts (in physics-space coordinates)
     */
    final private static Vector3f[] beltDirections = {
        new Vector3f(-1f, 0f, 0f),
        new Vector3f(0f, 0f, 1f),
        new Vector3f(1f, 0f, 0f),
        new Vector3f(0f, 0f, -1f)
    };
    // *************************************************************************
    // fields

    /**
     * text displayed in the upper-left corner of the GUI node
     */
    private static BitmapText statusText;
    /**
     * AppState to manage the PhysicsSpace
     */
    private static BulletAppState bulletAppState;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the ConveyorDemo application.
     */
    public ConveyorDemo() { // explicit to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the ConveyorDemo application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        String title = applicationName + " " + MyString.join(arguments);

        // Mute the chatty loggers in certain packages.
        Heart.setLoggingLevels(Level.WARNING);

        // Enable direct-memory tracking.
        BufferUtils.setTrackDirectMemoryEnabled(true);

        boolean loadDefaults = true;
        AppSettings settings = new AppSettings(loadDefaults);
        try {
            settings.load(applicationName);
        } catch (BackingStoreException exception) {
            logger.warning("Failed to load AppSettings.");
        }
        settings.setAudioRenderer(null);
        settings.setResizable(true);
        settings.setSamples(4); // anti-aliasing
        settings.setTitle(title); // Customize the window's title bar.

        Application application = new ConveyorDemo();
        application.setSettings(settings);
        application.start();
    }
    // *************************************************************************
    // ContactListener methods

    /**
     * Invoked immediately after a contact manifold is created.
     *
     * @param manifoldId ignored
     */
    @Override
    public void onContactEnded(long manifoldId) {
        // do nothing
    }

    /**
     * Invoked immediately after a contact point is refreshed without being
     * removed. Skipped for Sphere-Sphere contacts.
     *
     * @param pcoA the first involved object (not null)
     * @param pcoB the 2nd involved object (not null)
     * @param contactPointId the native ID of the btManifoldPoint (not 0)
     */
    @Override
    public void onContactProcessed(PhysicsCollisionObject pcoA,
            PhysicsCollisionObject pcoB, long contactPointId) {
        boolean aIsAConveyorBelt = pcoA.getApplicationData() instanceof Integer;
        boolean bIsAConveyorBelt = pcoB.getApplicationData() instanceof Integer;
        if (!aIsAConveyorBelt && !bIsAConveyorBelt) {
            return;
        }

        // enable lateral friction for the current contact point:
        ManifoldPoints
                .setFlags(contactPointId, ContactPointFlag.LATERAL_FRICTION);

        PhysicsCollisionObject beltPco = aIsAConveyorBelt ? pcoA : pcoB;
        int beltIndex = (Integer) beltPco.getApplicationData();
        Vector3f direction = beltDirections[beltIndex - 1];
        float beltSpeed = beltSpeeds[beltIndex - 1];

        // modify its motion and its friction direction:
        if (MyMath.isOdd(beltIndex)) {
            ManifoldPoints.setContactMotion1(contactPointId, beltSpeed);
            ManifoldPoints.setLateralFrictionDir1(contactPointId, direction);
        } else {
            ManifoldPoints.setContactMotion2(contactPointId, beltSpeed);
            ManifoldPoints.setLateralFrictionDir2(contactPointId, direction);
        }
    }

    /**
     * Invoked immediately after a contact manifold is created.
     *
     * @param manifoldId ignored
     */
    @Override
    public void onContactStarted(long manifoldId) {
        // do nothing
    }
    // *************************************************************************
    // DebugInitListener methods

    /**
     * Callback from BulletDebugAppState, invoked just before the debug scene is
     * added to the debug viewports.
     *
     * @param physicsDebugRootNode the root node of the debug scene (not null)
     */
    @Override
    public void bulletDebugInit(Node physicsDebugRootNode) {
        addLighting(physicsDebugRootNode);
    }
    // *************************************************************************
    // PhysicsDemo methods

    /**
     * Initialize this application.
     */
    @Override
    public void acorusInit() {
        // Add the status text to the GUI.
        statusText = new BitmapText(guiFont);
        guiNode.attachChild(statusText);

        super.acorusInit();
        setPauseOnLostFocus(false);

        configureCamera();
        configurePhysics();
        generateMaterials();
        generateShapes();

        ColorRGBA bgColor = new ColorRGBA(0.1f, 0.1f, 0.1f, 1f);
        viewPort.setBackgroundColor(bgColor);

        // Hide the render-statistics overlay.
        stateManager.getState(StatsAppState.class).toggleStats();

        addOuterWalls();
        addInnerWalls();
        addConveyorBelts();
    }

    /**
     * Initialize the library of named materials during startup.
     */
    @Override
    public void generateMaterials() {
        Material shinyMaterial
                = MyAsset.createShinyMaterial(assetManager, ColorRGBA.White);
        shinyMaterial.setFloat("Shininess", 100f);

        // materials for dropped boxes
        cloneAndRegisterMaterial("box", shinyMaterial, ColorRGBA.Orange);

        // materials for the conveyor belts and walls of each section
        ColorRGBA[] colorArray = {
            new ColorRGBA(0.82f, 0.65f, 0.04f, 1f),
            new ColorRGBA(0.73f, 0.17f, 0.18f, 1f),
            new ColorRGBA(0.20f, 0.62f, 0.28f, 1f),
            new ColorRGBA(0.24f, 0.44f, 0.79f, 1f)
        };
        for (int i = 1; i <= 4; ++i) {
            ColorRGBA color = colorArray[i - 1];
            cloneAndRegisterMaterial("Section" + i, shinyMaterial, color);
        }
    }

    /**
     * Access the active BulletAppState.
     *
     * @return the pre-existing instance (not null)
     */
    @Override
    protected BulletAppState getBulletAppState() {
        assert bulletAppState != null;
        return bulletAppState;
    }

    /**
     * Determine the length of physics-debug arrows (when they're visible).
     *
     * @return the desired length (in physics-space units, &ge;0)
     */
    @Override
    protected float maxArrowLength() {
        return 2f;
    }

    /**
     * Add application-specific hotkey bindings (and override existing ones, if
     * necessary).
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        dim.bind("add", KeyInput.KEY_RETURN, KeyInput.KEY_INSERT,
                KeyInput.KEY_NUMPAD0, KeyInput.KEY_SPACE);

        dim.bind(asCollectGarbage, KeyInput.KEY_G);
        dim.bind(asDumpSpace, KeyInput.KEY_O);
        dim.bind(asDumpViewport, KeyInput.KEY_P);

        dim.bindSignal(CameraInput.FLYCAM_LOWER, KeyInput.KEY_DOWN);
        dim.bindSignal(CameraInput.FLYCAM_RISE, KeyInput.KEY_UP);

        dim.bind(asToggleAabbs, KeyInput.KEY_APOSTROPHE);
        dim.bind(asToggleHelp, KeyInput.KEY_H);
        dim.bind(asTogglePause, KeyInput.KEY_PAUSE, KeyInput.KEY_PERIOD);
        dim.bind(asToggleVArrows, KeyInput.KEY_K);
        dim.bind(asToggleWArrows, KeyInput.KEY_N);
    }

    /**
     * Process an action that wasn't handled by the active input mode.
     *
     * @param actionString textual description of the action (not null)
     * @param ongoing true if the action is ongoing, otherwise false
     * @param tpf time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void onAction(String actionString, boolean ongoing, float tpf) {
        if (ongoing) {
            switch (actionString) {
                case "add":
                    addBoxBody(new Vector3f(0.2f, 0.2f, 0.2f),
                            spawnLocation, massForDynamic, "box", null);
                    return;
                default:
            }
        }
        super.onAction(actionString, ongoing, tpf);
    }

    /**
     * Update the GUI layout and proposed settings after a resize.
     *
     * @param newWidth the new width of the framebuffer (in pixels, &gt;0)
     * @param newHeight the new height of the framebuffer (in pixels, &gt;0)
     */
    @Override
    public void onViewPortResize(int newWidth, int newHeight) {
        statusText.setLocalTranslation(0f, newHeight, 0f);
        super.onViewPortResize(newWidth, newHeight);
    }

    /**
     * Callback invoked once per frame.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void simpleUpdate(float tpf) {
        super.simpleUpdate(tpf);
        updateStatusText();
    }
    // *************************************************************************
    // private methods

    /**
     * Create a box-shaped rigid body and add it to the PhysicsSpace.
     *
     * @param halfExtents the desired unscaled half extents (not null, no
     * negative component, unaffected)
     * @param centerLocation the desired center location (in physics-space
     * coordinates, not null, unaffected)
     * @param mass the desired mass (in physics mass units, &ge;0)
     * @param materialName the name of the desired debug material (not null)
     * @param appData the desired application data, or null for none
     * @return the new body
     */
    private PhysicsRigidBody addBoxBody(
            Vector3f halfExtents, Vector3f centerLocation, float mass,
            String materialName, Integer appData) {
        CollisionShape shape = new BoxCollisionShape(halfExtents);
        PhysicsRigidBody result = new PhysicsRigidBody(shape, mass);
        addCollisionObject(result);

        result.setApplicationData(appData);

        Material material = findMaterial(materialName);
        result.setDebugMaterial(material);

        result.setDebugMeshNormals(MeshNormals.Facet);
        result.setPhysicsLocation(centerLocation);

        return result;
    }

    /**
     * Create 4 static rigid bodies to represent conveyor belts and add them to
     * the PhysicsSpace. Application data is created to indicate the index of
     * each belt, which the ContactListener uses to modify contact points.
     */
    private void addConveyorBelts() {
        // half-extents for the box shapes
        Vector3f he13 = new Vector3f(beltLength, thickness, beltWidth);
        Vector3f he24 = new Vector3f(beltWidth, thickness, beltLength);

        Vector3f center1 = new Vector3f(x0 + beltLength, y0, 0f);
        addBoxBody(he13, center1, PhysicsBody.massForStatic, "Section1", 1);

        float x2 = x0 + 2 * beltLength + beltWidth;
        Vector3f center2 = new Vector3f(x2, y0, beltWidth - beltLength);
        addBoxBody(he24, center2, PhysicsBody.massForStatic, "Section2", 2);

        float x3 = x0 + beltLength + 2 * beltWidth;
        Vector3f center3 = new Vector3f(x3, y0, -2 * beltLength);
        addBoxBody(he13, center3, PhysicsBody.massForStatic, "Section3", 3);

        float x4 = x0 + beltWidth;
        Vector3f center4 = new Vector3f(x4, y0, -(beltLength + beltWidth));
        addBoxBody(he24, center4, PhysicsBody.massForStatic, "Section4", 4);
    }

    /**
     * Create 4 static rigid bodies to represent inner walls and add them to the
     * PhysicsSpace.
     */
    private void addInnerWalls() {
        // half-extents for the box shapes
        float wallLength = beltLength - beltWidth;
        Vector3f he13 = new Vector3f(wallLength, wallHeight, thickness);
        Vector3f he24 = new Vector3f(thickness, wallHeight, wallLength);

        float x1 = x0 + beltLength + beltWidth;
        Vector3f center1 = new Vector3f(x1, y0, -beltWidth);
        addBoxBody(he13, center1, PhysicsBody.massForStatic, "Section1", null);

        Vector3f center2 = new Vector3f(x0 + 2 * beltLength, y0, -beltLength);
        addBoxBody(he24, center2, PhysicsBody.massForStatic, "Section2", null);

        float x3 = x0 + beltLength + beltWidth;
        Vector3f center3 = new Vector3f(x3, y0, beltWidth - 2 * beltLength);
        addBoxBody(he13, center3, PhysicsBody.massForStatic, "Section3", null);

        Vector3f center4 = new Vector3f(x0 + 2 * beltWidth, y0, -beltLength);
        addBoxBody(he24, center4, PhysicsBody.massForStatic, "Section4", null);
    }

    /**
     * Add lighting to the specified scene.
     *
     * @param rootSpatial which scene (not null)
     */
    private static void addLighting(Spatial rootSpatial) {
        ColorRGBA ambientColor = new ColorRGBA(0.5f, 0.5f, 0.5f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        rootSpatial.addLight(ambient);
        ambient.setName("ambient");

        Vector3f direction = new Vector3f(1f, -2f, -1f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction);
        rootSpatial.addLight(sun);
        sun.setName("sun");
    }

    /**
     * Create 4 static rigid bodies to represent outer walls and add them to the
     * PhysicsSpace.
     */
    private void addOuterWalls() {
        // half-extents for the box shapes
        float wallLength = beltLength + beltWidth;
        Vector3f he13 = new Vector3f(wallLength, wallHeight, thickness);
        Vector3f he24 = new Vector3f(thickness, wallHeight, wallLength);

        float x1 = x0 + beltLength + beltWidth;
        Vector3f center1 = new Vector3f(x1, y0, beltWidth);
        addBoxBody(he13, center1, PhysicsBody.massForStatic, "Section1", null);

        float x2 = x0 + 2 * beltLength + 2 * beltWidth;
        Vector3f center2 = new Vector3f(x2, y0, -beltLength);
        addBoxBody(he24, center2, PhysicsBody.massForStatic, "Section2", null);

        float x3 = x0 + beltLength + beltWidth;
        Vector3f center3
                = new Vector3f(x3, y0, -(2 * beltLength + beltWidth));
        addBoxBody(he13, center3, PhysicsBody.massForStatic, "Section3", null);

        Vector3f center4 = new Vector3f(x0, y0, -beltLength);
        addBoxBody(he24, center4, PhysicsBody.massForStatic, "Section4", null);
    }

    /**
     * Clone the specified material, apply the specified color, and register it
     * with the specified name.
     *
     * @param name the desired name for the Material, which is also the key that
     * will be used to find it (not null)
     * @param material the prototype material (not null, unaffected)
     * @param color the desired color (not null, unaffected)
     */
    private void cloneAndRegisterMaterial(
            String name, Material material, ColorRGBA color) {
        Material clone = material.clone();
        clone.setColor("Ambient", color.clone());
        clone.setColor("Diffuse", color.clone());

        registerMaterial(name, clone);
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(5f);
        flyCam.setZoomSpeed(2f);

        cam.setLocation(new Vector3f(12f, 6f, 5f));
        cam.setRotation(new Quaternion(-0.066f, 0.91052f, -0.1912f, -0.36053f));
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        // Set up Bullet physics and create a physics space.
        bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);
        final PhysicsSpace physicsSpace = bulletAppState.getPhysicsSpace();

        // Enable debug visualization to reveal what occurs in physics space.
        bulletAppState.setDebugEnabled(true);
        bulletAppState.setDebugInitListener(this);

        // Enable relevant solver options.
        int defaultSolverMode = physicsSpace.getSolverInfo().mode();
        int solverMode = defaultSolverMode
                | SolverMode.Use2Directions
                | SolverMode.CacheDirection;
        physicsSpace.getSolverInfo().setMode(solverMode);

        // Register a listener for immediate contact notifications.
        physicsSpace.addContactListener(this);
    }

    /**
     * Update the status text in the GUI.
     */
    private void updateStatusText() {
        String message = isPaused() ? "PAUSED" : "";
        statusText.setText(message);
    }
}
