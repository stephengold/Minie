/*
 Copyright (c) 2022-2023, Stephen Gold
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

import com.jme3.app.StatsAppState;
import com.jme3.app.state.AppState;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.debug.DebugInitListener;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.font.BitmapText;
import com.jme3.input.CameraInput;
import com.jme3.input.KeyInput;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.system.AppSettings;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MeshNormals;
import jme3utilities.MyAsset;
import jme3utilities.MyString;
import jme3utilities.math.MyMath;
import jme3utilities.math.RectSizeLimits;
import jme3utilities.math.noise.Generator;
import jme3utilities.minie.test.common.PhysicsDemo;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.DisplaySettings;
import jme3utilities.ui.DsEditOverlay;
import jme3utilities.ui.InputMode;
import jme3utilities.ui.ShowDialog;

/**
 * A physics demo that simulates a simple Pachinko machine.
 * <p>
 * https://en.wikipedia.org/wiki/Pachinko
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class Pachinko
        extends PhysicsDemo
        implements DebugInitListener, PhysicsTickListener {
    // *************************************************************************
    // constants and loggers

    /**
     * time interval between balls (in simulated seconds)
     */
    final private static float addInterval = 3f;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(Pachinko.class.getName());
    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName
            = Pachinko.class.getSimpleName();
    // *************************************************************************
    // fields

    /**
     * text displayed at the bottom of the GUI node
     */
    private static BitmapText statusText;
    /**
     * AppState to manage the PhysicsSpace
     */
    private static BulletAppState bulletAppState;
    /**
     * collision shape for balls
     */
    private static CollisionShape ballShape;
    /**
     * proposed display settings, for the DsEditOverlay
     */
    private static DisplaySettings proposedSettings;
    /**
     * elapsed time since a ball was added (in simulated seconds)
     */
    private static float timeSinceAdded;
    /**
     * randomize initial motion of balls
     */
    final private static Generator generator = new Generator();
    /**
     * rotation matrix for pins
     */
    final private static Matrix3f rot45 = new Matrix3f();
    // *************************************************************************
    // constructors

    /**
     * Instantiate the Pachinko application.
     */
    public Pachinko() { // made explicit to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the Pachinko application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        // Mute the chatty loggers in certain packages.
        Heart.setLoggingLevels(Level.WARNING);

        for (String arg : arguments) {
            switch (arg) {
                case "--deleteOnly":
                    Heart.deleteStoredSettings(applicationName);
                    return;
                default:
            }
        }

        Pachinko application = new Pachinko();
        RectSizeLimits sizeLimits = new RectSizeLimits(
                480, 240, // min width, height
                2_048, 1_080 // max width, height
        );
        final String title = applicationName + " " + MyString.join(arguments);
        proposedSettings = new DisplaySettings(
                application, applicationName, sizeLimits) {
            @Override
            protected void applyOverrides(AppSettings settings) {
                setShowDialog(ShowDialog.Never);
                settings.setAudioRenderer(null);
                settings.setRenderer(AppSettings.LWJGL_OPENGL32);
                if (settings.getSamples() < 1) {
                    settings.setSamples(4); // anti-aliasing
                }
                settings.setResizable(true);
                settings.setTitle(title); // Customize the window's title bar.
            }
        };

        AppSettings appSettings = proposedSettings.initialize();
        if (appSettings != null) {
            application.setSettings(appSettings);
            /*
             * If the settings dialog should be shown,
             * it has already been shown by DisplaySettings.initialize().
             */
            application.setShowSettings(false);

            application.start();
        }
    }
    // *************************************************************************
    // PhysicsDemo methods

    /**
     * Initialize this application.
     */
    @Override
    public void acorusInit() {
        Quaternion q = new Quaternion().fromAngles(0f, 0f, FastMath.PI / 4f);
        rot45.set(q);

        configureCamera();
        configureDumper();
        generateMaterials();
        configurePhysics();

        // Hide the render-statistics overlay.
        stateManager.getState(StatsAppState.class).toggleStats();

        ColorRGBA skyColor = new ColorRGBA(0f, 0.1f, 0f, 1f);
        viewPort.setBackgroundColor(skyColor);

        // Add the status text to the GUI.
        statusText = new BitmapText(guiFont);
        statusText.setLocalTranslation(205f, 25f, 0f);
        guiNode.attachChild(statusText);

        AppState dseOverlay = new DsEditOverlay(proposedSettings);
        boolean success = stateManager.attach(dseOverlay);
        assert success;

        super.acorusInit();

        float ballRadius = 1f;
        ballShape = new SphereCollisionShape(ballRadius);

        restartSimulation(7);
    }

    /**
     * Initialize the library of named materials during startup.
     */
    @Override
    public void generateMaterials() {
        ColorRGBA black = new ColorRGBA(0.01f, 0.01f, 0.01f, 1f);
        Material ball = MyAsset.createShinyMaterial(assetManager, black);
        ball.setFloat("Shininess", 100f);
        registerMaterial("ball", ball);

        Material staticMaterial = MyAsset.createShadedMaterial(
                assetManager, ColorRGBA.White.clone());
        registerMaterial("static", staticMaterial);
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
     * Determine the length of debug axis arrows (when they're visible).
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

        dim.bind(asDumpSpace, KeyInput.KEY_O);
        dim.bind(asDumpViewport, KeyInput.KEY_P);

        dim.bind(asEditDisplaySettings, KeyInput.KEY_TAB);

        dim.bindSignal(CameraInput.FLYCAM_LOWER, KeyInput.KEY_DOWN);
        dim.bindSignal(CameraInput.FLYCAM_RISE, KeyInput.KEY_UP);
        dim.bindSignal("orbitLeft", KeyInput.KEY_LEFT);
        dim.bindSignal("orbitRight", KeyInput.KEY_RIGHT);

        dim.bind("simulate 4", KeyInput.KEY_F4, KeyInput.KEY_4,
                KeyInput.KEY_NUMPAD4);
        dim.bind("simulate 5", KeyInput.KEY_F5, KeyInput.KEY_5,
                KeyInput.KEY_NUMPAD5);
        dim.bind("simulate 6", KeyInput.KEY_F6, KeyInput.KEY_6,
                KeyInput.KEY_NUMPAD6);
        dim.bind("simulate 7", KeyInput.KEY_F7, KeyInput.KEY_7,
                KeyInput.KEY_NUMPAD7);
        dim.bind("simulate 8", KeyInput.KEY_F8, KeyInput.KEY_8,
                KeyInput.KEY_NUMPAD8);
        dim.bind("simulate 9", KeyInput.KEY_F9, KeyInput.KEY_9,
                KeyInput.KEY_NUMPAD9);

        dim.bind(asToggleAabbs, KeyInput.KEY_APOSTROPHE);
        dim.bind(asToggleHelp, KeyInput.KEY_H);
        dim.bind(asTogglePause, KeyInput.KEY_PAUSE, KeyInput.KEY_PERIOD);
        dim.bind(asTogglePcoAxes, KeyInput.KEY_SEMICOLON);
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
                case asEditDisplaySettings:
                    InputMode editor = InputMode.findMode("dsEdit");
                    InputMode.suspendAndActivate(editor);
                    return;

                case "simulate 1":
                    restartSimulation(1);
                    return;
                case "simulate 2":
                    restartSimulation(2);
                    return;
                case "simulate 3":
                    restartSimulation(3);
                    return;
                case "simulate 4":
                    restartSimulation(4);
                    return;

                default:
            }
        }
        super.onAction(actionString, ongoing, tpf);
    }

    /**
     * Callback invoked after the framebuffer is resized.
     *
     * @param newWidth the new width of the framebuffer (in pixels, &gt;0)
     * @param newHeight the new height of the framebuffer (in pixels, &gt;0)
     */
    @Override
    public void onViewPortResize(int newWidth, int newHeight) {
        proposedSettings.resize(newWidth, newHeight);
        DsEditOverlay dseOverlay = stateManager.getState(DsEditOverlay.class);
        dseOverlay.onViewPortResize(newWidth, newHeight);

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
    // PhysicsTickListener methods

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

    /**
     * Callback from Bullet, invoked just before each simulation step.
     *
     * @param space the space that's about to be stepped (not null)
     * @param timeStep the time per simulation step (in seconds, &ge;0)
     */
    @Override
    public void prePhysicsTick(PhysicsSpace space, float timeStep) {
        if (timeSinceAdded >= addInterval) {
            addBall();
            timeSinceAdded = 0;
        }
        timeSinceAdded += timeStep;
    }
    // *************************************************************************
    // private methods

    /**
     * Add a dynamic ball to the PhysicsSpace.
     */
    private void addBall() {
        PhysicsRigidBody result = new PhysicsRigidBody(ballShape);
        Material material = findMaterial("ball");
        result.setDebugMaterial(material);
        result.setDebugMeshNormals(MeshNormals.Sphere);
        result.setDebugMeshResolution(DebugShapeFactory.highResolution);
        addCollisionObject(result);

        result.setAngularDamping(0.9f);
        result.setEnableSleep(false);
        result.setPhysicsLocation(new Vector3f(0f, 4f, 0f));
        result.setRestitution(0.4f);

        // Restrict the ball's motion to the X-Y plane.
        result.setAngularFactor(new Vector3f(0f, 0f, 1f));
        result.setLinearFactor(new Vector3f(1f, 1f, 0f));

        // Apply a random horizontal impulse.
        float xImpulse = (1f - 2f * generator.nextFloat());
        result.applyCentralImpulse(new Vector3f(xImpulse, 0f, 0f));
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
     * Remove all physics objects from the PhysicsSpace.
     */
    private void clearSpace() {
        stateManager.detach(bulletAppState);
        configurePhysics();
    }

    /**
     * Configure the Camera and CIP during startup.
     */
    private void configureCamera() {
        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(40f);
        flyCam.setZoomSpeed(40f);

        cam.setLocation(new Vector3f(0f, -23f, 83f));

        AppState orbitState
                = new CameraOrbitAppState(cam, "orbitLeft", "orbitRight");
        stateManager.attach(orbitState);
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        bulletAppState = new BulletAppState();
        bulletAppState.setDebugEnabled(true);
        bulletAppState.setDebugInitListener(this);
        stateManager.attach(bulletAppState);

        PhysicsSpace space = bulletAppState.getPhysicsSpace();
        space.addTickListener(this);
    }

    /**
     * Restart the simulation with the specified number of rows of pins.
     *
     * @param numRows (&ge;1, &le;10)
     */
    private void restartSimulation(int numRows) {
        assert numRows > 0 && numRows <= 10 : numRows;

        getPhysicsSpace().destroy();

        // Estimate the number of child shapes in the playing field.
        int estNumChildren = numRows * (numRows + 1) + 3;
        CompoundCollisionShape fieldShape
                = new CompoundCollisionShape(estNumChildren);

        float barHalfWidth = 0.3f;
        int lastRow = numRows - 1;
        Vector3f tmpOffset = new Vector3f();

        // Add child shapes for the pins.
        float pinHalfHeight = 1f;
        float pinHalfWidth = MyMath.rootHalf * barHalfWidth;
        BoxCollisionShape pinShape = new BoxCollisionShape(
                pinHalfWidth, pinHalfWidth, pinHalfHeight);

        float ballRadius = ballShape.maxRadius();
        float pinSpacing = 2f * (barHalfWidth + ballRadius);
        float rowSpacing = 2f * pinSpacing;

        for (int rowIndex = 0; rowIndex < numRows; ++rowIndex) {
            float y = -rowSpacing * rowIndex;
            int numPins = numRows - (rowIndex % 2);
            if (rowIndex == lastRow) {
                numPins += 2;
            }
            for (int pinIndex = 0; pinIndex < numPins; ++pinIndex) {
                float x = pinSpacing * (pinIndex - (numPins - 1) / 2f);
                tmpOffset.set(x, y, 0f);
                fieldShape.addChildShape(pinShape, tmpOffset, rot45);
            }
        }

        // Add child shapes for the vertical bars.
        float barHalfLength = 0.5f * rowSpacing * (11 - numRows);
        BoxCollisionShape barShape = new BoxCollisionShape(
                barHalfWidth, barHalfLength, pinHalfHeight);
        int numBars = numRows - (lastRow % 2) + 2;
        float yBar = -rowSpacing * lastRow - barHalfLength;

        for (int barIndex = 0; barIndex < numBars; ++barIndex) {
            float x = pinSpacing * (barIndex - (numBars - 1) / 2f);
            fieldShape.addChildShape(barShape, x, yBar, 0f);
        }

        // Add a child shape for the horizontal stop at the bottom.
        float yStop = yBar - barHalfLength;
        float stopHalfWidth = pinSpacing * (numBars - 1) / 2f + barHalfWidth;
        BoxCollisionShape stopShape = new BoxCollisionShape(
                stopHalfWidth, barHalfWidth, pinHalfHeight);
        fieldShape.addChildShape(stopShape, 0f, yStop, 0f);

        PhysicsRigidBody playingField
                = new PhysicsRigidBody(fieldShape, PhysicsBody.massForStatic);
        playingField.setRestitution(0.7f);
        Material staticMaterial = findMaterial("static");
        playingField.setDebugMaterial(staticMaterial);

        addCollisionObject(playingField);
        timeSinceAdded = addInterval;
    }

    /**
     * Update the status text in the GUI.
     */
    private void updateStatusText() {
        String message = isPaused() ? "  PAUSED" : "";
        statusText.setText(message);
    }
}
