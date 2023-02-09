/*
 Copyright (c) 2019-2023, Stephen Gold
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
import com.jme3.app.state.AppState;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.RotationOrder;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.joints.New6Dof;
import com.jme3.bullet.joints.motors.MotorParam;
import com.jme3.bullet.joints.motors.RotationMotor;
import com.jme3.font.BitmapText;
import com.jme3.input.CameraInput;
import com.jme3.input.KeyInput;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Box;
import com.jme3.shadow.DirectionalLightShadowRenderer;
import com.jme3.system.AppSettings;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MyAsset;
import jme3utilities.MyString;
import jme3utilities.minie.DumpFlags;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.minie.test.common.PhysicsDemo;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.InputMode;
import jme3utilities.ui.Signals;

/**
 * Test/demonstrate double-ended New6Dof joints.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class JointDemo extends PhysicsDemo {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(JointDemo.class.getName());
    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName
            = JointDemo.class.getSimpleName();
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
    /**
     * scene-graph node for visualizing solid objects
     */
    final private static Node meshesNode = new Node("meshes node");
    /**
     * motor to rotate the left-front leg
     */
    private static RotationMotor lfMotor;
    /**
     * motor to rotate the left-rear leg
     */
    private static RotationMotor lrMotor;
    /**
     * motor to rotate the right-front leg
     */
    private static RotationMotor rfMotor;
    /**
     * motor to rotate the right-rear leg
     */
    private static RotationMotor rrMotor;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the JointDemo application.
     */
    public JointDemo() { // to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the JointDemo application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        String title = applicationName + " " + MyString.join(arguments);

        // Mute the chatty loggers in certain packages.
        Heart.setLoggingLevels(Level.WARNING);

        boolean loadDefaults = true;
        AppSettings settings = new AppSettings(loadDefaults);
        settings.setAudioRenderer(null);
        settings.setSamples(4); // anti-aliasing
        settings.setTitle(title); // Customize the window's title bar.

        Application application = new JointDemo();
        application.setSettings(settings);
        application.start();
    }
    // *************************************************************************
    // PhysicsDemo methods

    /**
     * Initialize this application.
     */
    @Override
    public void acorusInit() {
        super.acorusInit();

        configureCamera();
        configureDumper();
        generateMaterials();
        configurePhysics();

        ColorRGBA skyColor = new ColorRGBA(0.1f, 0.2f, 0.4f, 1f);
        viewPort.setBackgroundColor(skyColor);

        addLighting();

        float length = 0.8f;
        attachWorldAxes(length);

        float halfExtent = 50f;
        float topY = 0f;
        attachCubePlatform(halfExtent, topY);

        addRobot();

        rootNode.attachChild(meshesNode);
        meshesNode.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);

        // Add the status text to the GUI.
        statusText = new BitmapText(guiFont);
        statusText.setLocalTranslation(0f, cam.getHeight(), 0f);
        guiNode.attachChild(statusText);
    }

    /**
     * Configure the PhysicsDumper during startup.
     */
    @Override
    public void configureDumper() {
        super.configureDumper();

        PhysicsDumper dumper = getDumper();
        dumper.setEnabled(DumpFlags.JointsInSpaces, true);
        dumper.setEnabled(DumpFlags.Motors, true);
    }

    /**
     * Generate materials during startup.
     */
    @Override
    public void generateMaterials() {
        super.generateMaterials();

        ColorRGBA brown = new ColorRGBA().setAsSrgb(0.3f, 0.3f, 0.1f, 1f);
        Material robotMaterial
                = MyAsset.createShadedMaterial(assetManager, brown);
        registerMaterial("robot", robotMaterial);
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
        return 0.5f;
    }

    /**
     * Add application-specific hotkey bindings (and override existing ones, if
     * necessary).
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        dim.bind(asDumpScene, KeyInput.KEY_P);
        dim.bind(asDumpSpace, KeyInput.KEY_O);

        dim.bind(asCollectGarbage, KeyInput.KEY_G);

        dim.bindSignal(CameraInput.FLYCAM_LOWER, KeyInput.KEY_DOWN);
        dim.bindSignal(CameraInput.FLYCAM_RISE, KeyInput.KEY_UP);
        dim.bindSignal("orbitLeft", KeyInput.KEY_LEFT);
        dim.bindSignal("orbitRight", KeyInput.KEY_RIGHT);
        dim.bind("signal turnLF", KeyInput.KEY_NUMPAD7);
        dim.bind("signal turnRF", KeyInput.KEY_NUMPAD9);
        dim.bind("signal turnLR", KeyInput.KEY_NUMPAD1);
        dim.bind("signal turnRR", KeyInput.KEY_NUMPAD3);

        dim.bind(asToggleAabbs, KeyInput.KEY_APOSTROPHE);
        dim.bind(asToggleHelp, KeyInput.KEY_H);
        dim.bind(asTogglePause, KeyInput.KEY_PAUSE, KeyInput.KEY_PERIOD);
        dim.bind(asTogglePcoAxes, KeyInput.KEY_SEMICOLON);
        dim.bind("toggle view", KeyInput.KEY_SLASH);
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
                case "toggle view":
                    toggleMeshes();
                    togglePhysicsDebug();
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

        // Check UI signals and update motor velocities accordingly.
        Signals signals = getSignals();

        float lfVelocity = signals.test("turnLF") ? 2f : 0f;
        lfMotor.set(MotorParam.TargetVelocity, lfVelocity);

        float rfVelocity = signals.test("turnRF") ? 2f : 0f;
        rfMotor.set(MotorParam.TargetVelocity, rfVelocity);

        float lrVelocity = signals.test("turnLR") ? 2f : 0f;
        lrMotor.set(MotorParam.TargetVelocity, lrVelocity);

        float rrVelocity = signals.test("turnRR") ? 2f : 0f;
        rrMotor.set(MotorParam.TargetVelocity, rrVelocity);

        updateStatusText();
    }
    // *************************************************************************
    // private methods

    /**
     * Add a rectangular leg to the robot chassis.
     *
     * @param legGeom the geometry for the leg (not null)
     * @param legInWorld the location of the leg (in world coordinates, not
     * null)
     * @param pivotInChassis the pivot offset relative to the chassis (not null)
     * @param chassisInWorld the chassis location (in world coordinates, not
     * null)
     * @param chassisRbc the body for the chassis (not null)
     * @return the new instance
     */
    private RotationMotor addLeg(Geometry legGeom, Vector3f legInWorld,
            Vector3f pivotInChassis, Vector3f chassisInWorld,
            RigidBodyControl chassisRbc) {
        meshesNode.attachChild(legGeom);
        legGeom.move(legInWorld);
        Material robotMaterial = findMaterial("robot");
        legGeom.setMaterial(robotMaterial);

        CollisionShape shape = new BoxCollisionShape(0.1f, 0.4f, 0.1f);
        float mass = 0.1f;
        RigidBodyControl legRbc = new RigidBodyControl(shape, mass);
        legGeom.addControl(legRbc);
        PhysicsSpace physicsSpace = getPhysicsSpace();
        legRbc.setPhysicsSpace(physicsSpace);
        legRbc.setEnableSleep(false);

        Vector3f pivotInLeg
                = pivotInChassis.add(chassisInWorld).subtractLocal(legInWorld);
        New6Dof joint = new New6Dof(chassisRbc, legRbc, pivotInChassis,
                pivotInLeg, new Matrix3f(), new Matrix3f(), RotationOrder.ZYX);

        // Inhibit X- and Y-axis rotations.
        RotationMotor xMotor = joint.getRotationMotor(PhysicsSpace.AXIS_X);
        xMotor.set(MotorParam.UpperLimit, 0f);
        xMotor.set(MotorParam.LowerLimit, 0f);
        RotationMotor yMotor = joint.getRotationMotor(PhysicsSpace.AXIS_Y);
        yMotor.set(MotorParam.UpperLimit, 0f);
        yMotor.set(MotorParam.LowerLimit, 0f);

        // Enable the motor for Z-axis rotation and return a reference to it.
        RotationMotor zMotor = joint.getRotationMotor(PhysicsSpace.AXIS_Z);
        zMotor.setMotorEnabled(true);
        zMotor.set(MotorParam.MaxMotorForce, 9e9f);

        physicsSpace.addJoint(joint);

        return zMotor;
    }

    /**
     * Add lighting and shadows to the main scene.
     */
    private void addLighting() {
        ColorRGBA ambientColor = new ColorRGBA(0.5f, 0.5f, 0.5f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        rootNode.addLight(ambient);
        ambient.setName("ambient");

        Vector3f direction = new Vector3f(1f, -2f, -1f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction);
        rootNode.addLight(sun);
        sun.setName("sun");

        int mapSize = 2_048; // in pixels
        int numSplits = 3;
        DirectionalLightShadowRenderer dlsr
                = new DirectionalLightShadowRenderer(
                        assetManager, mapSize, numSplits);
        dlsr.setLight(sun);
        dlsr.setShadowIntensity(0.5f);
        viewPort.addProcessor(dlsr);
    }

    /**
     * Add a 4-legged robot to the scene.
     */
    private void addRobot() {
        Mesh chassisMesh = new Box(1f, 0.3f, 0.3f);
        CollisionShape chassisShape = new BoxCollisionShape(1f, 0.3f, 0.3f);
        Vector3f chassisInWorld = new Vector3f(0f, 1f, 0f);

        Geometry chassisGeom = new Geometry("chassis", chassisMesh);
        meshesNode.attachChild(chassisGeom);
        chassisGeom.move(chassisInWorld);
        Material robotMaterial = findMaterial("robot");
        chassisGeom.setMaterial(robotMaterial);
        chassisGeom.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);

        float chassisMass = 1f;
        RigidBodyControl chassisRbc
                = new RigidBodyControl(chassisShape, chassisMass);
        chassisGeom.addControl(chassisRbc);
        PhysicsSpace physicsSpace = getPhysicsSpace();
        chassisRbc.setPhysicsSpace(physicsSpace);
        chassisRbc.setEnableSleep(false);

        Mesh legMesh = new Box(0.1f, 0.4f, 0.1f);

        Geometry lfGeom = new Geometry("lf leg", legMesh);
        Vector3f lfLegInWorld = new Vector3f(-0.8f, 0.6f, 0.5f);
        Vector3f lfPivotInChassis = new Vector3f(-0.8f, 0f, 0.4f);
        lfMotor = addLeg(lfGeom, lfLegInWorld, lfPivotInChassis,
                chassisInWorld, chassisRbc);

        Geometry rfGeom = new Geometry("rf leg", legMesh);
        Vector3f rfLegInWorld = new Vector3f(-0.8f, 0.6f, -0.5f);
        Vector3f rfPivotInChassis = new Vector3f(-0.8f, 0f, -0.4f);
        rfMotor = addLeg(rfGeom, rfLegInWorld, rfPivotInChassis,
                chassisInWorld, chassisRbc);

        Geometry lrGeom = new Geometry("lr leg", legMesh);
        Vector3f lrLegInWorld = new Vector3f(0.8f, 0.6f, 0.5f);
        Vector3f lrPivotInChassis = new Vector3f(0.8f, 0f, 0.4f);
        lrMotor = addLeg(lrGeom, lrLegInWorld, lrPivotInChassis,
                chassisInWorld, chassisRbc);

        Geometry rrGeom = new Geometry("rr leg", legMesh);
        Vector3f rrLegInWorld = new Vector3f(0.8f, 0.6f, -0.5f);
        Vector3f rrPivotInChassis = new Vector3f(0.8f, 0f, -0.4f);
        rrMotor = addLeg(rrGeom, rrLegInWorld, rrPivotInChassis,
                chassisInWorld, chassisRbc);
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(4f);
        flyCam.setZoomSpeed(4f);

        cam.setLocation(new Vector3f(2.65f, 2.42f, 9.37f));
        cam.setRotation(new Quaternion(0f, 0.9759f, -0.04f, -0.2136f));

        AppState orbitState
                = new CameraOrbitAppState(cam, "orbitLeft", "orbitRight");
        stateManager.attach(orbitState);
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);
    }

    /**
     * Toggle mesh rendering on/off.
     */
    private static void toggleMeshes() {
        Spatial.CullHint hint = meshesNode.getLocalCullHint();
        if (hint == Spatial.CullHint.Inherit
                || hint == Spatial.CullHint.Never) {
            hint = Spatial.CullHint.Always;
        } else if (hint == Spatial.CullHint.Always) {
            hint = Spatial.CullHint.Never;
        }
        meshesNode.setCullHint(hint);
    }

    /**
     * Update the status text in the GUI.
     */
    private void updateStatusText() {
        String message = "View: ";

        Spatial.CullHint cull = meshesNode.getLocalCullHint();
        message += (cull == Spatial.CullHint.Always) ? "NOmeshes" : "Meshes";

        boolean debug = bulletAppState.isDebugEnabled();
        if (debug) {
            message += "+" + describePhysicsDebugOptions();
        }
        message += isPaused() ? "  PAUSED" : "";
        statusText.setText(message);
    }
}
