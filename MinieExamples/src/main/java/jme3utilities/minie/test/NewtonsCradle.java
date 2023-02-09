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

import com.jme3.app.state.AppState;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.debug.DebugInitListener;
import com.jme3.bullet.joints.Point2PointJoint;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.util.DebugShapeFactory;
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
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MeshNormals;
import jme3utilities.MyAsset;
import jme3utilities.MyString;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.RectSizeLimits;
import jme3utilities.minie.test.common.PhysicsDemo;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.DisplaySettings;
import jme3utilities.ui.DsEditOverlay;
import jme3utilities.ui.InputMode;
import jme3utilities.ui.ShowDialog;

/**
 * A physics demo that simulates a Newton's cradle.
 * <p>
 * https://en.wikipedia.org/wiki/Newton%27s_cradle
 * <p>
 * Collision objects are rendered entirely by debug visualization.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class NewtonsCradle
        extends PhysicsDemo
        implements DebugInitListener {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(NewtonsCradle.class.getName());
    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName
            = NewtonsCradle.class.getSimpleName();
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
     * proposed display settings, for the DsEditOverlay
     */
    private static DisplaySettings proposedSettings;
    /**
     * temporary storage used in updateStatusText()
     */
    final private static Vector3f tmpVector = new Vector3f();
    // *************************************************************************
    // constructors

    /**
     * Instantiate the NewtonsCradle application.
     */
    public NewtonsCradle() { // explicit to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the NewtonsCradle application.
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

        NewtonsCradle application = new NewtonsCradle();
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
        configureCamera();
        configureDumper();
        generateMaterials();
        configurePhysics();

        ColorRGBA skyColor = new ColorRGBA(0.1f, 0.2f, 0.4f, 1f);
        viewPort.setBackgroundColor(skyColor);

        // Add the status text to the GUI.
        statusText = new BitmapText(guiFont);
        statusText.setLocalTranslation(205f, 25f, 0f);
        guiNode.attachChild(statusText);

        AppState dseOverlay = new DsEditOverlay(proposedSettings);
        boolean success = stateManager.attach(dseOverlay);
        assert success;

        super.acorusInit();

        restartSimulation(5);
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
        return 12f;
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

        dim.bind("simulate 1", KeyInput.KEY_F1, KeyInput.KEY_1,
                KeyInput.KEY_NUMPAD1);
        dim.bind("simulate 2", KeyInput.KEY_F2, KeyInput.KEY_2,
                KeyInput.KEY_NUMPAD2);
        dim.bind("simulate 3", KeyInput.KEY_F3, KeyInput.KEY_3,
                KeyInput.KEY_NUMPAD3);
        dim.bind("simulate 4", KeyInput.KEY_F4, KeyInput.KEY_4,
                KeyInput.KEY_NUMPAD4);

        dim.bind(asToggleAabbs, KeyInput.KEY_APOSTROPHE);
        dim.bind(asToggleHelp, KeyInput.KEY_H);
        dim.bind(asTogglePause, KeyInput.KEY_PAUSE, KeyInput.KEY_PERIOD);
        dim.bind(asTogglePcoAxes, KeyInput.KEY_SEMICOLON);
        dim.bind(asToggleVArrows, KeyInput.KEY_K);
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
    // private methods

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
     * Add a dynamic ball to the PhysicsSpace, suspended between 2 single-ended
     * point-to-point joints.
     *
     * @param xOffset the desired X coordinate for the ball's center
     * @return the new instance (not null)
     */
    private PhysicsRigidBody addSuspendedBall(float xOffset) {
        float radius = 9.9f;
        SphereCollisionShape sphere = new SphereCollisionShape(radius);
        PhysicsRigidBody ball = new PhysicsRigidBody(sphere);
        Vector3f location = new Vector3f(xOffset, 0f, 0f);
        Material material = findMaterial("ball");
        ball.setDebugMaterial(material);
        ball.setDebugMeshNormals(MeshNormals.Sphere);
        ball.setDebugMeshResolution(DebugShapeFactory.highResolution);
        ball.setFriction(0f);
        ball.setPhysicsLocation(location);
        ball.setRestitution(1f);
        addCollisionObject(ball);

        float wireLength = 80f;
        float yOffset = wireLength * MyMath.rootHalf;

        Vector3f offset = new Vector3f(0f, yOffset, +yOffset);
        Point2PointJoint joint1 = new Point2PointJoint(ball, offset);
        addJoint(joint1);

        offset.set(0f, yOffset, -yOffset);
        Point2PointJoint joint2 = new Point2PointJoint(ball, offset);
        addJoint(joint2);

        return ball;
    }

    /**
     * Remove all physics objects from the PhysicsSpace.
     */
    private void clearSpace() {
        stateManager.detach(bulletAppState);
        configurePhysics();
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(40f);
        flyCam.setZoomSpeed(40f);

        cam.setLocation(new Vector3f(72f, 35f, 140f));
        cam.setRotation(new Quaternion(0.001f, 0.96926f, -0.031f, -0.244f));

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

        getPhysicsSpace().setAccuracy(0.01f);
        setGravityAll(150f);
    }

    /**
     * Restart the simulation (paused) with the specified number of balls.
     *
     * @param numBalls (&ge;1)
     */
    private void restartSimulation(int numBalls) {
        clearSpace();
        this.speed = pausedSpeed;

        float xSeparation = 20f;

        // center-to-center separation between the first and last balls
        float xExtent = (numBalls - 1) * xSeparation;

        float x0 = -xExtent / 2;
        PhysicsRigidBody[] balls = new PhysicsRigidBody[numBalls];
        for (int ballIndex = 0; ballIndex < numBalls; ++ballIndex) {
            float x = x0 + ballIndex * xSeparation;
            balls[ballIndex] = addSuspendedBall(x);
        }

        Vector3f kick = new Vector3f(-20f * numBalls, 0f, 0f);
        balls[0].applyCentralImpulse(kick);
    }

    /**
     * Update the status text in the GUI.
     */
    private void updateStatusText() {
        PhysicsSpace physicsSpace = getPhysicsSpace();
        Vector3f gravity = physicsSpace.getGravity(null);
        double totalEnergy = 0.0;
        for (PhysicsRigidBody body : physicsSpace.getRigidBodyList()) {
            totalEnergy += body.kineticEnergy();

            body.getPhysicsLocation(tmpVector);
            double potentialEnergy = -MyVector3f.dot(tmpVector, gravity);
            totalEnergy += potentialEnergy;
        }

        String message = String.format(
                "KE+PE=%f%s", totalEnergy, isPaused() ? "  PAUSED" : "");
        statusText.setText(message);
    }
}
