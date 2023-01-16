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
import com.jme3.bullet.SolverInfo;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.joints.HingeJoint;
import com.jme3.bullet.joints.JointEnd;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.input.CameraInput;
import com.jme3.input.KeyInput;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.system.AppSettings;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.prefs.BackingStoreException;
import jme3utilities.Heart;
import jme3utilities.MyCamera;
import jme3utilities.MyString;
import jme3utilities.Validate;
import jme3utilities.minie.test.common.PhysicsDemo;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.InputMode;

/**
 * Tune the simulation quality of a single-ended HingeJoint.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class JointElasticity extends PhysicsDemo {
    // *************************************************************************
    // constants and loggers

    /**
     * half the width of the door (in physics-space units)
     */
    final private static float doorHalfWidth = 2f;
    /**
     * mass of the door (in physics mass units)
     */
    final private static float doorMass = 1f;
    /**
     * half the thickness of the door (in physics-space units)
     */
    final private static float halfThickness = 0.3f;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(JointElasticity.class.getName());
    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName
            = JointElasticity.class.getSimpleName();
    /**
     * action string to advance to the next field
     */
    final private static String asNextField = "next field";
    /**
     * action string to advance to the next value
     */
    final private static String asNextValue = "next value";
    /**
     * action string to go to the previous field
     */
    final private static String asPreviousField = "previous field";
    /**
     * action string to go to the previous value
     */
    final private static String asPreviousValue = "previous value";
    /**
     * signal names for the CameraOrbitAppState
     */
    final private static String signalOrbitLeft = "cameraOrbitLeft";
    final private static String signalOrbitRight = "cameraOrbitRight";
    // *************************************************************************
    // fields

    /**
     * AppState to manage the PhysicsSpace
     */
    private static BulletAppState bulletAppState;
    /**
     * AppState to manage the status overlay
     */
    private static JointElasticityStatus status;
    /**
     * dynamic ball
     */
    private static PhysicsRigidBody ballBody;
    /**
     * dynamic door
     */
    private static PhysicsRigidBody doorBody;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the JointElasticity application.
     */
    public JointElasticity() { // to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the JointElasticity application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        String title = applicationName + " " + MyString.join(arguments);

        // Mute the chatty loggers in certain packages.
        Heart.setLoggingLevels(Level.WARNING);

        boolean loadDefaults = true;
        AppSettings settings = new AppSettings(loadDefaults);
        try {
            settings.load(applicationName);
        } catch (BackingStoreException exception) {
            logger.warning("Failed to load AppSettings.");
        }
        settings.setAudioRenderer(null);
        settings.setResizable(true);
        settings.setTitle(title); // Customize the window's title bar.

        JointElasticity application = new JointElasticity();
        application.setSettings(settings);
        application.start();
    }

    /**
     * Alter the mass ratio during a scenario.
     *
     * @param massRatio the mass of the ball as a multiple of the door's mass
     * (&gt;0)
     */
    static void setMassRatio(float massRatio) {
        Validate.positive(massRatio, "mass ratio");

        float newMass = massRatio * doorMass;
        ballBody.setMass(newMass);
    }
    // *************************************************************************
    // PhysicsDemo methods

    /**
     * Initialize this application.
     */
    @Override
    public void acorusInit() {
        status = new JointElasticityStatus();
        boolean success = stateManager.attach(status);
        assert success;

        super.acorusInit();

        configureCamera();
        configureDumper();
        configurePhysics();
        restartScenario();

        ColorRGBA bgColor = new ColorRGBA(0.04f, 0.04f, 0.04f, 1f);
        viewPort.setBackgroundColor(bgColor);

        setSpeed(0.2f);
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
        return 1f;
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

        dim.bind(asNextField, KeyInput.KEY_NUMPAD2);
        dim.bind(asNextValue, KeyInput.KEY_EQUALS, KeyInput.KEY_NUMPAD6);

        dim.bindSignal(CameraInput.FLYCAM_LOWER, KeyInput.KEY_DOWN);
        dim.bindSignal(CameraInput.FLYCAM_RISE, KeyInput.KEY_UP);
        dim.bindSignal(signalOrbitLeft, KeyInput.KEY_LEFT);
        dim.bindSignal(signalOrbitRight, KeyInput.KEY_RIGHT);

        dim.bind(asPreviousField, KeyInput.KEY_NUMPAD8);
        dim.bind(asPreviousValue, KeyInput.KEY_MINUS, KeyInput.KEY_NUMPAD4);

        dim.bind("restart", KeyInput.KEY_NUMPAD5, KeyInput.KEY_SPACE,
                KeyInput.KEY_RETURN);

        dim.bind(asToggleAabbs, KeyInput.KEY_APOSTROPHE);
        dim.bind(asToggleHelp, KeyInput.KEY_H);
        dim.bind(asTogglePause, KeyInput.KEY_PAUSE, KeyInput.KEY_PERIOD);
        dim.bind(asTogglePcoAxes, KeyInput.KEY_SEMICOLON);
        dim.bind(asToggleVArrows, KeyInput.KEY_K);
    }

    /**
     * Process an action that wasn't handled by the active InputMode.
     *
     * @param actionString textual description of the action (not null)
     * @param ongoing true if the action is ongoing, otherwise false
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void onAction(String actionString, boolean ongoing, float tpf) {
        if (ongoing) {
            switch (actionString) {
                case asNextField:
                    status.advanceSelectedField(+1);
                    return;
                case asNextValue:
                    status.advanceValue(+1);
                    return;
                case asPreviousField:
                    status.advanceSelectedField(-1);
                    return;
                case asPreviousValue:
                    status.advanceValue(-1);
                    return;

                case "restart":
                    restartScenario();
                    return;

                default:
            }
            String[] words = actionString.split(" ");
            if (words.length == 2 && "load".equals(words[0])) {
                return;
            }
        }

        // The action is not handled: forward it to the superclass.
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
        status.resize(newWidth, newHeight);
        super.onViewPortResize(newWidth, newHeight);
    }
    // *************************************************************************
    // private methods

    /**
     * Create a kinematic rigid body with a sphere shape and add it to the
     * space.
     */
    private void addBall() {
        float radius = 0.4f;
        SphereCollisionShape shape = new SphereCollisionShape(radius);

        float mass = status.massRatio() * doorMass;
        ballBody = new PhysicsRigidBody(shape, mass);
        ballBody.setLinearVelocity(new Vector3f(2f, 0f, -10f));
        ballBody.setPhysicsLocation(new Vector3f(0.8f, 0f, 2f));
        addCollisionObject(ballBody);
    }

    /**
     * Create a dynamic body with a box shape and add it to the specified
     * PhysicsSpace.
     */
    private void addDoor() {
        float halfHeight = 4f;
        BoxCollisionShape shape = new BoxCollisionShape(
                doorHalfWidth, halfHeight, halfThickness);
        doorBody = new PhysicsRigidBody(shape, doorMass);
        addCollisionObject(doorBody);

        // Disable sleep (deactivation).
        doorBody.setEnableSleep(false);
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        float near = 0.1f;
        float far = 500f;
        MyCamera.setNearFar(cam, near, far);

        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(10f);
        flyCam.setZoomSpeed(10f);

        cam.setLocation(new Vector3f(0f, 8.8f, 6.2f));
        cam.setRotation(new Quaternion(0f, 0.9f, -0.43589f, 0f));

        AppState orbitState = new CameraOrbitAppState(
                cam, signalOrbitRight, signalOrbitLeft);
        stateManager.attach(orbitState);
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        bulletAppState = new BulletAppState();
        bulletAppState.setDebugEnabled(true);
        stateManager.attach(bulletAppState);

        PhysicsSpace physicsSpace = getPhysicsSpace();
        SolverInfo info = physicsSpace.getSolverInfo();

        float erp = status.jointErp();
        info.setJointErp(erp);

        int numIterations = status.numIterations();
        info.setNumIterations(numIterations);

        float timestep = status.timeStep();
        physicsSpace.setAccuracy(timestep);
    }

    /**
     * Restart the scenario.
     */
    private void restartScenario() {
        PhysicsSpace physicsSpace = getPhysicsSpace();
        physicsSpace.destroy();

        // Add a dynamic body for the door.
        addDoor();

        // Add a single-ended hinge joint to constrain the door's motion.
        Vector3f pivotInDoor = new Vector3f(doorHalfWidth, 0f, 0f);
        Vector3f pivotInWorld = new Vector3f(doorHalfWidth, 0f, 0f);
        HingeJoint joint = new HingeJoint(doorBody, pivotInDoor, pivotInWorld,
                Vector3f.UNIT_Y, Vector3f.UNIT_Y, JointEnd.B);
        float lowAngle = 0f;
        float highAngle = 0f;
        joint.setLimit(lowAngle, highAngle); // disable rotation
        addJoint(joint);

        // Add a dynamic, yellow ball.
        addBall();
    }
}
