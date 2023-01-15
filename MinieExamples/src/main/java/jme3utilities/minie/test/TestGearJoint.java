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
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.joints.GearJoint;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.font.BitmapText;
import com.jme3.input.KeyInput;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.system.AppSettings;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MyString;
import jme3utilities.math.RectSizeLimits;
import jme3utilities.minie.test.common.PhysicsDemo;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.DisplaySettings;
import jme3utilities.ui.DsEditOverlay;
import jme3utilities.ui.InputMode;
import jme3utilities.ui.ShowDialog;
import jme3utilities.ui.Signals;

/**
 * Test/demonstrate gear joints.
 * <p>
 * Collision objects are rendered entirely by debug visualization.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestGearJoint
        extends PhysicsDemo
        implements PhysicsTickListener {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(TestGearJoint.class.getName());
    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName
            = TestGearJoint.class.getSimpleName();
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
     * proposed display settings (for editing)
     */
    private static DisplaySettings proposedSettings;
    /**
     * subject body to which torques are applied
     */
    private static PhysicsRigidBody driveshaft;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the TestGearJoint application.
     */
    public TestGearJoint() { // explicit to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the TestGearJoint application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        String title = applicationName + " " + MyString.join(arguments);

        // Mute the chatty loggers in certain packages.
        Heart.setLoggingLevels(Level.WARNING);

        // Process any command-line arguments.
        ShowDialog showDialog = ShowDialog.Never;
        for (String arg : arguments) {
            switch (arg) {
                case "--deleteOnly":
                    Heart.deleteStoredSettings(applicationName);
                    return;

                case "--showSettingsDialog":
                    showDialog = ShowDialog.FirstTime;
                    break;

                case "--verbose":
                    Heart.setLoggingLevels(Level.INFO);
                    break;

                default:
                    logger.log(Level.WARNING,
                            "Ignored unknown command-line argument {0}",
                            MyString.quote(arg));
            }
        }
        mainStartup(showDialog, title);
    }
    // *************************************************************************
    // PhysicsDemo methods

    /**
     * Initialize this application.
     */
    @Override
    public void acorusInit() {
        DsEditOverlay dseOverlay = new DsEditOverlay(proposedSettings);
        dseOverlay.setBackgroundColor(new ColorRGBA(0.05f, 0f, 0f, 1f));
        boolean success = stateManager.attach(dseOverlay);
        assert success;
        super.acorusInit();

        // Hide the render-statistics overlay.
        stateManager.getState(StatsAppState.class).toggleStats();

        // Add the status text to the GUI.
        statusText = new BitmapText(guiFont);
        statusText.setLocalTranslation(205f, 25f, 0f);
        guiNode.attachChild(statusText);

        configureCamera();
        configureDumper();
        configurePhysics();

        ColorRGBA bgColor = new ColorRGBA(0.05f, 0.05f, 0.05f, 1f);
        viewPort.setBackgroundColor(bgColor);

        float length = 0.8f;
        attachWorldAxes(length);

        // Add an elongated dynamic body for the driveshaft.
        float radius = 0.5f;
        float height = 3f;
        CollisionShape driveshaftShape = new CylinderCollisionShape(
                radius, height, PhysicsSpace.AXIS_Y);
        driveshaft = new PhysicsRigidBody(driveshaftShape);
        driveshaft.setPhysicsLocation(new Vector3f(-1f, 0.2f, 0f));
        driveshaft.setEnableSleep(false);
        addCollisionObject(driveshaft);

        // Add a flattened dynamic body for the wheel.
        radius = 2f;
        height = 0.5f;
        CollisionShape wheelShape = new CylinderCollisionShape(
                radius, height, PhysicsSpace.AXIS_X);
        float wheelMass = 2f;
        PhysicsRigidBody wheel = new PhysicsRigidBody(wheelShape, wheelMass);
        wheel.setPhysicsLocation(new Vector3f(1f, 0.2f, 0f));
        wheel.setEnableSleep(false);
        addCollisionObject(wheel);
        /*
         * Join them with a GearJoint.
         * The driveshaft makes 3 revolutions for each revolution of the wheel.
         */
        GearJoint gear = new GearJoint(
                driveshaft, wheel, Vector3f.UNIT_Y, Vector3f.UNIT_X, 3f);
        addJoint(gear);
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

        dim.bind(asDumpScenes, KeyInput.KEY_P);
        dim.bind(asDumpSpace, KeyInput.KEY_O);

        dim.bind(asEditDisplaySettings, KeyInput.KEY_TAB);
        dim.bindSignal("orbitLeft", KeyInput.KEY_LEFT);
        dim.bindSignal("orbitRight", KeyInput.KEY_RIGHT);
        dim.bindSignal("+torque", KeyInput.KEY_F, KeyInput.KEY_DOWN);
        dim.bindSignal("-torque", KeyInput.KEY_R, KeyInput.KEY_UP);

        dim.bind(asToggleHelp, KeyInput.KEY_H);
        dim.bind(asTogglePause, KeyInput.KEY_PAUSE, KeyInput.KEY_PERIOD);
        dim.bind(asTogglePcoAxes, KeyInput.KEY_SEMICOLON);
        dim.bind(asToggleWorldAxes, KeyInput.KEY_SPACE);
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
                    activateInputMode("dsEdit");
                    return;

                default:
            }
        }
        super.onAction(actionString, ongoing, tpf);
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
    // PhysicsTickListener methods

    /**
     * Callback from Bullet, invoked just before the physics is stepped.
     *
     * @param space the space that's about to be stepped (not null)
     * @param timeStep the time per simulation step (in seconds, &ge;0)
     */
    @Override
    public void prePhysicsTick(PhysicsSpace space, float timeStep) {
        // Check UI signals and apply forces/torques accordingly.
        Signals signals = getSignals();

        if (signals.test("+torque")) {
            driveshaft.applyTorque(new Vector3f(0f, 1f, 0f));
        }
        if (signals.test("-torque")) {
            driveshaft.applyTorque(new Vector3f(0f, -1f, 0f));
        }
    }

    /**
     * Callback from Bullet, invoked just after the physics has been stepped.
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
     * Configure the camera during startup.
     */
    private void configureCamera() {
        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(4f);
        flyCam.setZoomSpeed(4f);

        cam.setLocation(new Vector3f(2.2f, 2f, 7.7f));
        cam.setRotation(new Quaternion(-0.007f, 0.984838f, -0.12f, -0.1251f));

        AppState orbitState
                = new CameraOrbitAppState(cam, "orbitLeft", "orbitRight");
        stateManager.attach(orbitState);
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        // Set up Bullet physics and create a physics space.
        bulletAppState = new BulletAppState();
        float axisLength = maxArrowLength();
        bulletAppState.setDebugAxisLength(axisLength);
        bulletAppState.setDebugEnabled(true);
        stateManager.attach(bulletAppState);

        PhysicsSpace space = getPhysicsSpace();
        space.addTickListener(this);
        setGravityAll(0f);
    }

    /**
     * Initialization performed immediately after parsing the command-line
     * arguments.
     *
     * @param showDialog when to show the JME settings dialog (not null)
     * @param title for the title bar of the app's window
     */
    private static void mainStartup(
            final ShowDialog showDialog, final String title) {
        TestGearJoint application = new TestGearJoint();

        RectSizeLimits sizeLimits = new RectSizeLimits(
                530, 480, // min width, height
                2_048, 1_080 // max width, height
        );
        proposedSettings = new DisplaySettings(
                application, applicationName, sizeLimits) {
            @Override
            protected void applyOverrides(AppSettings settings) {
                setShowDialog(showDialog);
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
        if (appSettings == null) {
            return;
        }

        application.setSettings(appSettings);
        /*
         * If the settings dialog should be shown,
         * it has already been shown by DisplaySettings.initialize().
         */
        application.setShowSettings(false);
        application.start();
    }

    /**
     * Update the status text in the GUI.
     */
    private void updateStatusText() {
        String message = isPaused() ? "PAUSED" : "";
        statusText.setText(message);
    }
}
