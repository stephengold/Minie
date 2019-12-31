/*
 Copyright (c) 2019, Stephen Gold
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
import com.jme3.app.state.ScreenshotAppState;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.RotationOrder;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.joints.New6Dof;
import com.jme3.bullet.joints.motors.MotorParam;
import com.jme3.bullet.joints.motors.RotationMotor;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.font.Rectangle;
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
import jme3utilities.Misc;
import jme3utilities.MyAsset;
import jme3utilities.debug.AxesVisualizer;
import jme3utilities.minie.DumpFlags;
import jme3utilities.minie.FilterAll;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.ui.ActionApplication;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.InputMode;
import jme3utilities.ui.Signals;

/**
 * Test/demonstrate double-ended New6Dof joints.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class JointDemo extends ActionApplication {
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
     * AppState to manage the PhysicsSpace
     */
    private BulletAppState bulletAppState;
    /**
     * filter to control visualization of axis-aligned bounding boxes
     */
    private FilterAll bbFilter;
    /**
     * material to visualize the platform box
     */
    private Material boxMaterial;
    /**
     * material to visualize the robot
     */
    private Material robotMaterial;
    /**
     * GUI node for displaying hotkey help/hints
     */
    private Node helpNode;
    /**
     * scene-graph node for visualizing solid objects
     */
    final private Node solidNode = new Node("solid node");
    /**
     * dump debugging information to System.out
     */
    final private PhysicsDumper dumper = new PhysicsDumper();
    /**
     * space for physics simulation
     */
    private PhysicsSpace physicsSpace;
    /**
     * motor to rotate the left-front leg
     */
    private RotationMotor lfMotor;
    /**
     * motor to rotate the left-rear leg
     */
    private RotationMotor lrMotor;
    /**
     * motor to rotate the right-front leg
     */
    private RotationMotor rfMotor;
    /**
     * motor to rotate the right-rear leg
     */
    private RotationMotor rrMotor;
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the JointDemo application.
     *
     * @param ignored array of command-line arguments (not null)
     */
    public static void main(String[] ignored) {
        /*
         * Mute the chatty loggers in certain packages.
         */
        Misc.setLoggingLevels(Level.WARNING);

        Application application = new JointDemo();
        /*
         * Customize the window's title bar.
         */
        AppSettings settings = new AppSettings(true);
        settings.setTitle(applicationName);

        settings.setGammaCorrection(true);
        settings.setSamples(4); // anti-aliasing
        settings.setVSync(true);
        application.setSettings(settings);

        application.start();
    }
    // *************************************************************************
    // ActionApplication methods

    /**
     * Initialize this application.
     */
    @Override
    public void actionInitializeApplication() {
        configureCamera();
        configureDumper();
        configureMaterials();
        configurePhysics();

        ColorRGBA sky = new ColorRGBA(0.1f, 0.2f, 0.4f, 1f);
        viewPort.setBackgroundColor(sky);

        addLighting();
        /*
         * Capture a screenshot each time KEY_SYSRQ (the PrtSc key) is pressed.
         */
        ScreenshotAppState screenshotAppState
                = new ScreenshotAppState("Written Assets/", "screenshot");
        boolean success = stateManager.attach(screenshotAppState);
        assert success;

        addAxes();
        addBox();
        addRobot();

        rootNode.attachChild(solidNode);
        solidNode.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);
    }

    /**
     * Add application-specific hotkey bindings and override existing ones.
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        dim.bind("dump physicsSpace", KeyInput.KEY_O);
        dim.bind("dump scene", KeyInput.KEY_P);

        dim.bind("signal " + CameraInput.FLYCAM_LOWER, KeyInput.KEY_DOWN);
        dim.bind("signal " + CameraInput.FLYCAM_RISE, KeyInput.KEY_UP);
        dim.bind("signal orbitLeft", KeyInput.KEY_LEFT);
        dim.bind("signal orbitRight", KeyInput.KEY_RIGHT);
        dim.bind("signal turnLF", KeyInput.KEY_NUMPAD7);
        dim.bind("signal turnRF", KeyInput.KEY_NUMPAD9);
        dim.bind("signal turnLR", KeyInput.KEY_NUMPAD1);
        dim.bind("signal turnRR", KeyInput.KEY_NUMPAD3);

        dim.bind("toggle aabb", KeyInput.KEY_APOSTROPHE);
        dim.bind("toggle axes", KeyInput.KEY_SEMICOLON);
        dim.bind("toggle help", KeyInput.KEY_H);
        dim.bind("toggle pause", KeyInput.KEY_PERIOD);
        dim.bind("toggle view", KeyInput.KEY_SLASH);

        float x = 10f;
        float y = cam.getHeight() - 10f;
        float width = cam.getWidth() - 20f;
        float height = cam.getHeight() - 20f;
        Rectangle rectangle = new Rectangle(x, y, width, height);

        float space = 20f;
        helpNode = HelpUtils.buildNode(dim, rectangle, guiFont, space);
        guiNode.attachChild(helpNode);
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
                case "dump physicsSpace":
                    dumper.dump(physicsSpace);
                    return;
                case "dump scene":
                    dumper.dump(rootNode);
                    return;

                case "toggle aabb":
                    toggleAabb();
                    return;
                case "toggle axes":
                    toggleAxes();
                    return;
                case "toggle help":
                    toggleHelp();
                    return;
                case "toggle pause":
                    togglePause();
                    return;
                case "toggle view":
                    toggleMeshes();
                    togglePhysicsDebug();
                    return;
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
        /*
         * Activate any bodies that have fallen asleep.
         */
        for (PhysicsRigidBody body : physicsSpace.getRigidBodyList()) {
            body.activate();
        }
        /*
         * Check UI signals and update motor velocities accordingly.
         */
        Signals signals = getSignals();

        float lfVelocity = signals.test("turnLF") ? 2f : 0f;
        lfMotor.set(MotorParam.TargetVelocity, lfVelocity);

        float rfVelocity = signals.test("turnRF") ? 2f : 0f;
        rfMotor.set(MotorParam.TargetVelocity, rfVelocity);

        float lrVelocity = signals.test("turnLR") ? 2f : 0f;
        lrMotor.set(MotorParam.TargetVelocity, lrVelocity);

        float rrVelocity = signals.test("turnRR") ? 2f : 0f;
        rrMotor.set(MotorParam.TargetVelocity, rrVelocity);
    }
    // *************************************************************************
    // private methods

    /**
     * Add a visualizer for the axes of the world coordinate system.
     */
    private void addAxes() {
        float axisLength = 0.8f;
        AxesVisualizer axes = new AxesVisualizer(assetManager, axisLength);
        axes.setLineWidth(0f);

        rootNode.addControl(axes);
        axes.setEnabled(true);
    }

    /**
     * Add a large, static box to the scene, to serve as a platform.
     */
    private void addBox() {
        float halfExtent = 50f; // mesh units
        Mesh mesh = new Box(halfExtent, halfExtent, halfExtent);
        Geometry geometry = new Geometry("box", mesh);
        solidNode.attachChild(geometry);

        geometry.move(0f, -halfExtent, 0f);
        geometry.setMaterial(boxMaterial);
        geometry.setShadowMode(RenderQueue.ShadowMode.Receive);

        BoxCollisionShape shape = new BoxCollisionShape(halfExtent);
        float mass = PhysicsRigidBody.massForStatic;
        RigidBodyControl boxBody = new RigidBodyControl(shape, mass);
        geometry.addControl(boxBody);
        boxBody.setApplyScale(true);
        boxBody.setPhysicsSpace(physicsSpace);
    }

    /**
     * Add a rectangular leg to the robot chassis.
     */
    private RotationMotor addLeg(Geometry legGeom, Vector3f legInWorld,
            Vector3f pivotInChassis, Vector3f chassisInWorld,
            RigidBodyControl chassisRbc) {
        solidNode.attachChild(legGeom);
        legGeom.move(legInWorld);
        legGeom.setMaterial(robotMaterial);

        CollisionShape shape = new BoxCollisionShape(0.1f, 0.4f, 0.1f);
        float mass = 0.1f;
        RigidBodyControl legRbc = new RigidBodyControl(shape, mass);
        legGeom.addControl(legRbc);
        legRbc.setPhysicsSpace(physicsSpace);

        Vector3f pivotInLeg
                = pivotInChassis.add(chassisInWorld).subtractLocal(legInWorld);
        New6Dof joint = new New6Dof(chassisRbc, legRbc, pivotInChassis,
                pivotInLeg, new Matrix3f(), new Matrix3f(), RotationOrder.ZYX);
        /*
         * Inhibit X- and Y-axis rotations.
         */
        RotationMotor xMotor = joint.getRotationMotor(PhysicsSpace.AXIS_X);
        xMotor.set(MotorParam.UpperLimit, 0f);
        xMotor.set(MotorParam.LowerLimit, 0f);
        RotationMotor yMotor = joint.getRotationMotor(PhysicsSpace.AXIS_Y);
        yMotor.set(MotorParam.UpperLimit, 0f);
        yMotor.set(MotorParam.LowerLimit, 0f);
        /*
         * Enable the motor for Z-axis rotation and return a reference to it.
         */
        RotationMotor zMotor = joint.getRotationMotor(PhysicsSpace.AXIS_Z);
        zMotor.setMotorEnabled(true);
        zMotor.set(MotorParam.MaxMotorForce, 9e9f);

        physicsSpace.add(joint);

        return zMotor;
    }

    /**
     * Add lighting to the scene.
     */
    private void addLighting() {
        ColorRGBA ambientColor = new ColorRGBA(0.5f, 0.5f, 0.5f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        rootNode.addLight(ambient);

        Vector3f direction = new Vector3f(1f, -2f, -1f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction);
        rootNode.addLight(sun);

        DirectionalLightShadowRenderer dlsr
                = new DirectionalLightShadowRenderer(assetManager, 2_048, 3);
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
        solidNode.attachChild(chassisGeom);
        chassisGeom.move(chassisInWorld);
        chassisGeom.setMaterial(robotMaterial);
        chassisGeom.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);

        float chassisMass = 1f;
        RigidBodyControl chassisRbc
                = new RigidBodyControl(chassisShape, chassisMass);
        chassisGeom.addControl(chassisRbc);
        chassisRbc.setPhysicsSpace(physicsSpace);

        Mesh legMesh = new Box(0.1f, 0.4f, 0.1f);

        Geometry lfGeom = new Geometry("lf leg", legMesh);
        Vector3f lfLegInWorld = new Vector3f(-0.8f, 0.6f, 0.5f);
        Vector3f lfPivotInChassis = new Vector3f(-0.8f, 0f, 0.4f);
        lfMotor = addLeg(lfGeom, lfLegInWorld, lfPivotInChassis, chassisInWorld,
                chassisRbc);

        Geometry rfGeom = new Geometry("rf leg", legMesh);
        Vector3f rfLegInWorld = new Vector3f(-0.8f, 0.6f, -0.5f);
        Vector3f rfPivotInChassis = new Vector3f(-0.8f, 0f, -0.4f);
        rfMotor = addLeg(rfGeom, rfLegInWorld, rfPivotInChassis, chassisInWorld,
                chassisRbc);

        Geometry lrGeom = new Geometry("lr leg", legMesh);
        Vector3f lrLegInWorld = new Vector3f(0.8f, 0.6f, 0.5f);
        Vector3f lrPivotInChassis = new Vector3f(0.8f, 0f, 0.4f);
        lrMotor = addLeg(lrGeom, lrLegInWorld, lrPivotInChassis, chassisInWorld,
                chassisRbc);

        Geometry rrGeom = new Geometry("rr leg", legMesh);
        Vector3f rrLegInWorld = new Vector3f(0.8f, 0.6f, -0.5f);
        Vector3f rrPivotInChassis = new Vector3f(0.8f, 0f, -0.4f);
        rrMotor = addLeg(rrGeom, rrLegInWorld, rrPivotInChassis, chassisInWorld,
                chassisRbc);
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(4f);
        cam.setLocation(new Vector3f(2.65f, 2.42f, 9.37f));
        cam.setRotation(new Quaternion(0f, 0.9759f, -0.04f, -0.2136f));

        CameraOrbitAppState orbitState
                = new CameraOrbitAppState(cam, "orbitLeft", "orbitRight");
        stateManager.attach(orbitState);
    }

    /**
     * Configure the PhysicsDumper during startup.
     */
    private void configureDumper() {
        dumper.setEnabled(DumpFlags.JointsInBodies, true);
        dumper.setEnabled(DumpFlags.JointsInSpaces, true);
        dumper.setEnabled(DumpFlags.Motors, true);
        dumper.setEnabled(DumpFlags.ShadowModes, true);
        dumper.setEnabled(DumpFlags.Transforms, true);
    }

    /**
     * Configure materials during startup.
     */
    private void configureMaterials() {
        ColorRGBA darkGreen = new ColorRGBA().setAsSrgb(0.1f, 0.4f, 0.1f, 1f);
        boxMaterial = MyAsset.createShadedMaterial(assetManager, darkGreen);
        boxMaterial.setName("darkGreen");

        ColorRGBA brown = new ColorRGBA().setAsSrgb(0.3f, 0.3f, 0.1f, 1f);
        robotMaterial = MyAsset.createShadedMaterial(assetManager, brown);
        robotMaterial.setName("brown");
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);
        physicsSpace = bulletAppState.getPhysicsSpace();
    }

    /**
     * Toggle visualization of collision-object bounding boxes.
     */
    private void toggleAabb() {
        if (bbFilter == null) {
            bbFilter = new FilterAll(true);
        } else {
            bbFilter = null;
        }

        bulletAppState.setDebugBoundingBoxFilter(bbFilter);
    }

    /**
     * Toggle visualization of collision-object axes.
     */
    private void toggleAxes() {
        float length = bulletAppState.debugAxisLength();
        bulletAppState.setDebugAxisLength(0.5f - length);
    }

    /**
     * Toggle visibility of the helpNode.
     */
    private void toggleHelp() {
        if (helpNode.getCullHint() == Spatial.CullHint.Always) {
            helpNode.setCullHint(Spatial.CullHint.Never);
        } else {
            helpNode.setCullHint(Spatial.CullHint.Always);
        }
    }

    /**
     * Toggle solid mesh rendering on/off.
     */
    private void toggleMeshes() {
        Spatial.CullHint hint = solidNode.getLocalCullHint();
        if (hint == Spatial.CullHint.Inherit
                || hint == Spatial.CullHint.Never) {
            hint = Spatial.CullHint.Always;
        } else if (hint == Spatial.CullHint.Always) {
            hint = Spatial.CullHint.Never;
        }
        solidNode.setCullHint(hint);
    }

    /**
     * Toggle the physics simulation: paused/running.
     */
    private void togglePause() {
        float newSpeed = (speed > 1e-12f) ? 1e-12f : 1f;
        setSpeed(newSpeed);
    }

    /**
     * Toggle physics-debug visualization on/off.
     */
    private void togglePhysicsDebug() {
        boolean enabled = bulletAppState.isDebugEnabled();
        bulletAppState.setDebugEnabled(!enabled);
    }
}
