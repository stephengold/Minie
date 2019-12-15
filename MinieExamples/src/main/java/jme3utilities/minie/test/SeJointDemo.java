/*
 Copyright (c) 2018-2019, Stephen Gold
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
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.MultiSphere;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.joints.ConeJoint;
import com.jme3.bullet.joints.HingeJoint;
import com.jme3.bullet.joints.JointEnd;
import com.jme3.bullet.joints.New6Dof;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.joints.Point2PointJoint;
import com.jme3.bullet.joints.SixDofJoint;
import com.jme3.bullet.joints.SixDofSpringJoint;
import com.jme3.bullet.joints.SliderJoint;
import com.jme3.bullet.joints.motors.MotorParam;
import com.jme3.bullet.joints.motors.RotationMotor;
import com.jme3.font.BitmapText;
import com.jme3.font.Rectangle;
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
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Sphere;
import com.jme3.system.AppSettings;
import java.util.Collection;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Misc;
import jme3utilities.MyAsset;
import jme3utilities.MyCamera;
import jme3utilities.debug.AxesVisualizer;
import jme3utilities.math.MyMath;
import jme3utilities.math.noise.Generator;
import jme3utilities.minie.DumpFlags;
import jme3utilities.minie.FilterAll;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.ui.ActionApplication;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.InputMode;
import jme3utilities.ui.Signals;

/**
 * Test/demonstrate single-ended joints.
 * <p>
 * Seen in the November 2018 demo video:
 * https://www.youtube.com/watch?v=Mh9k5AfWzbg
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class SeJointDemo extends ActionApplication {
    // *************************************************************************
    // constants and loggers

    /**
     * seed radius (in mesh units)
     */
    final private static float seedRadius = 0.12f;
    /**
     * upper limit on the number of seed groups
     */
    final private static int maxGroups = 4;
    /**
     * upper limit on the number of seeds
     */
    final private static int maxSeeds = 300;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(SeJointDemo.class.getName());
    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName
            = SeJointDemo.class.getSimpleName();
    /**
     * joint axes in physics-space coordinates (unit vector)
     */
    final private static Vector3f axisInWorld = Vector3f.UNIT_Z;
    // *************************************************************************
    // fields

    /**
     * status displayed in the upper-left corner of the GUI node
     */
    private BitmapText statusText;
    /**
     * AppState to manage the PhysicsSpace
     */
    final private BulletAppState bulletAppState = new BulletAppState();
    private CollisionShape seedShape;
    /**
     * filter to control visualization of axis-aligned bounding boxes
     */
    private FilterAll bbFilter;
    /**
     * enhanced pseudo-random generator
     */
    final private Generator random = new Generator();
    /**
     * material for each type of seed
     */
    final private Material materials[] = new Material[maxGroups];

    final private Matrix3f rotInSeed = new Matrix3f();
    final private Matrix3f rotInWorld = new Matrix3f();
    /**
     * mesh for visualizing seeds
     */
    private Mesh seedMesh;
    /**
     * GUI node for displaying hotkey help/hints
     */
    private Node helpNode;
    /**
     * scene-graph node for visualizing seeds
     */
    final private Node seedNode = new Node("seed node");
    /**
     * dump debugging information to the console
     */
    final private PhysicsDumper dumper = new PhysicsDumper();
    /**
     * space for physics simulation
     */
    private PhysicsSpace physicsSpace;
    /**
     * name of the test that's currently running
     */
    private String testName = "p2p";
    /**
     * joint axis in each seed's local coordinates (unit vector)
     */
    final private Vector3f axisInSeed = new Vector3f();
    /**
     * gravity vector (in physics-space coordinates)
     */
    final private Vector3f gravity = new Vector3f();
    /**
     * offset of pivot relative to each seed (in scaled shape coordinates)
     */
    final private Vector3f pivotInSeed = new Vector3f();
    /**
     * pivot location for each group (in physics-space coordinates)
     */
    final private Vector3f pivotLocations[] = new Vector3f[maxGroups];
    /**
     * scale factors that determine the seed's shape
     */
    final private Vector3f seedScale = new Vector3f();
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the SeJointDemo application.
     *
     * @param ignored array of command-line arguments (not null)
     */
    public static void main(String[] ignored) {
        /*
         * Mute the chatty loggers in certain packages.
         */
        Misc.setLoggingLevels(Level.WARNING);

        Application application = new SeJointDemo();
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
        configurePhysics();
        configureGroups();

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

        seedMesh = new Sphere(16, 32, seedRadius);
        seedShape = new MultiSphere(seedRadius);

        seedNode.setCullHint(Spatial.CullHint.Never);// meshes initially visible
        rootNode.attachChild(seedNode);
        /*
         * Add the status text to the GUI.
         */
        statusText = new BitmapText(guiFont, false);
        statusText.setLocalTranslation(0f, cam.getHeight(), 0f);
        guiNode.attachChild(statusText);
    }

    /**
     * Add application-specific hotkey bindings and override existing ones.
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        dim.bind("add", KeyInput.KEY_INSERT);

        dim.bind("dump physicsSpace", KeyInput.KEY_O);
        dim.bind("dump scene", KeyInput.KEY_P);

        dim.bind("signal " + CameraInput.FLYCAM_LOWER, KeyInput.KEY_DOWN);
        dim.bind("signal " + CameraInput.FLYCAM_RISE, KeyInput.KEY_UP);
        dim.bind("signal orbitLeft", KeyInput.KEY_LEFT);
        dim.bind("signal orbitRight", KeyInput.KEY_RIGHT);
        dim.bind("signal shower", KeyInput.KEY_I);

        dim.bind("test 6dof", KeyInput.KEY_F6);
        dim.bind("test 6dofSpring", KeyInput.KEY_F7);
        dim.bind("test cone", KeyInput.KEY_F3);
        dim.bind("test hinge", KeyInput.KEY_F2);
        dim.bind("test new6dof", KeyInput.KEY_F8);
        dim.bind("test p2p", KeyInput.KEY_F1);
        dim.bind("test slider", KeyInput.KEY_F4);

        dim.bind("toggle axes", KeyInput.KEY_SEMICOLON);
        dim.bind("toggle boxes", KeyInput.KEY_APOSTROPHE);
        dim.bind("toggle help", KeyInput.KEY_H);
        dim.bind("toggle pause", KeyInput.KEY_PERIOD);
        dim.bind("toggle view", KeyInput.KEY_SLASH);

        float x = 10f;
        float y = cam.getHeight() - 40f;
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
                case "add":
                    addSeed();
                    return;

                case "dump physicsSpace":
                    dumper.dump(physicsSpace);
                    return;

                case "dump scene":
                    dumper.dump(rootNode);
                    return;

                case "test 6dof":
                    cleanupAfterTest();
                    testName = "6dof";
                    return;
                case "test 6dofSpring":
                    cleanupAfterTest();
                    testName = "6dofSpring";
                    return;
                case "test cone":
                    cleanupAfterTest();
                    testName = "cone";
                    return;
                case "test hinge":
                    cleanupAfterTest();
                    testName = "hinge";
                    return;
                case "test new6dof":
                    cleanupAfterTest();
                    testName = "new6dof";
                    return;
                case "test p2p":
                    cleanupAfterTest();
                    testName = "p2p";
                    return;
                case "test slider":
                    cleanupAfterTest();
                    testName = "slider";
                    return;

                case "toggle axes":
                    toggleAxes();
                    return;
                case "toggle boxes":
                    toggleBoxes();
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

        Signals signals = getSignals();
        if (signals.test("shower")) {
            addSeed();
        }

        updateStatusText();
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
     * Add lighting to the scene.
     */
    private void addLighting() {
        ColorRGBA ambientColor = new ColorRGBA(2f, 2f, 2f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        rootNode.addLight(ambient);

        Vector3f direction = new Vector3f(1f, -2f, -1f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction);
        rootNode.addLight(sun);
    }

    /**
     * Add a dynamic rigid body to the scene.
     */
    private void addSeed() {
        int numSeeds = seedNode.getChildren().size();
        if (numSeeds >= maxSeeds) {
            return; // too many seeds
        }
        /*
         * All seeds share a single spherical mesh
         * and a single spherical collision shape.
         * Non-uniform scaling gives seeds their shape.
         */
        int numGroups = maxGroups; // for most tests
        switch (testName) {
            case "6dof":
            case "6dofSpring":
            case "new6dof":
                seedScale.set(3f, 1f, 1f);
                break;

            case "cone":
                numGroups = 1;
                seedScale.set(3f, 1f, 1f);
                break;

            case "hinge":
                seedScale.set(1f, 2f, 1.5f);
                break;

            case "p2p":
                seedScale.set(1f, 1f, 1f);
                break;

            case "slider":
                numGroups = 3;
                seedScale.set(1f, 2f, 2f);
                break;

            default:
                throw new IllegalStateException("testName = " + testName);
        }
        seedShape.setScale(seedScale);
        /*
         * Randomize which group the new seed is in.
         */
        int groupIndex = MyMath.modulo(random.nextInt(), numGroups);
        Material material = materials[groupIndex];
        Vector3f pivotInWorld = pivotLocations[groupIndex];
        /*
         * Randomize the new seed's initial location and velocity.
         */
        Vector3f velocity = random.nextVector3f();
        Vector3f location = random.nextVector3f();
        location.multLocal(0.5f, 1f, 0.5f);
        location.addLocal(0f, 4f, 0f);

        Geometry geometry = new Geometry("seed", seedMesh);
        seedNode.attachChild(geometry);
        geometry.setMaterial(material);
        geometry.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);
        geometry.move(location);

        RigidBodyControl rbc = new RigidBodyControl(seedShape);
        rbc.setApplyScale(true);
        rbc.setKinematic(false);
        rbc.setLinearDamping(0.5f);
        rbc.setLinearVelocity(velocity);
        rbc.setPhysicsLocation(location);
        rbc.setSleepingThresholds(0f, 0f); // never sleep

        PhysicsJoint joint;
        switch (testName) {
            case "6dof":
                gravity.zero();
                pivotInSeed.set(1f, 0f, 0f);
                rotInSeed.loadIdentity();
                float angle = (groupIndex - 1) * FastMath.HALF_PI;
                rotInWorld.fromAngleAxis(angle, Vector3f.UNIT_Z);
                JointEnd referenceFrame = JointEnd.B;
                SixDofJoint sixDofJoint = new SixDofJoint(rbc, pivotInSeed,
                        pivotInWorld, rotInSeed, rotInWorld, referenceFrame);
                sixDofJoint.setAngularLowerLimit(new Vector3f(0f, -1f, -1f));
                sixDofJoint.setAngularUpperLimit(new Vector3f(0f, 1f, 1f));
                joint = sixDofJoint;
                break;

            case "6dofSpring":
                gravity.zero();
                pivotInSeed.set(1f, 0f, 0f);
                rotInSeed.loadIdentity();
                angle = (groupIndex - 1) * FastMath.HALF_PI;
                rotInWorld.fromAngleAxis(angle, Vector3f.UNIT_Z);
                referenceFrame = JointEnd.B;
                SixDofSpringJoint springJoint = new SixDofSpringJoint(rbc,
                        pivotInSeed, pivotInWorld, rotInSeed, rotInWorld,
                        referenceFrame);
                springJoint.setAngularLowerLimit(new Vector3f(0f, -1f, -1f));
                springJoint.setAngularUpperLimit(new Vector3f(0f, 1f, 1f));
                joint = springJoint;
                break;

            case "cone":
                gravity.zero();
                pivotInSeed.set(2f, 0f, 0f);
                rotInSeed.fromAngleAxis(FastMath.PI, Vector3f.UNIT_Z);
                ConeJoint coneJoint
                        = new ConeJoint(rbc, pivotInSeed, rotInSeed);
                coneJoint.setLimit(1f, 1f, 1e30f); // radians
                joint = coneJoint;
                break;

            case "hinge":
                axisInSeed.set(1f, 1f, 0f).normalizeLocal();
                gravity.set(0f, -1f, 0f);
                pivotInSeed.set(0f, 1f, 0f);
                referenceFrame = JointEnd.A;
                joint = new HingeJoint(rbc, pivotInSeed, pivotInWorld,
                        axisInSeed, axisInWorld, referenceFrame);
                break;

            case "new6dof":
                gravity.zero();
                pivotInSeed.set(1f, 0f, 0f);
                rotInSeed.loadIdentity();
                angle = (groupIndex - 1) * FastMath.HALF_PI;
                rotInWorld.fromAngleAxis(angle, Vector3f.UNIT_Z);
                New6Dof nJoint = new New6Dof(rbc,
                        pivotInSeed, pivotInWorld, rotInSeed, rotInWorld,
                        RotationOrder.XYZ);
                RotationMotor x = nJoint.getRotationMotor(PhysicsSpace.AXIS_X);
                x.set(MotorParam.LowerLimit, 0f);
                x.set(MotorParam.UpperLimit, 0f);
                RotationMotor y = nJoint.getRotationMotor(PhysicsSpace.AXIS_Y);
                y.set(MotorParam.LowerLimit, -1f);
                y.set(MotorParam.UpperLimit, 1f);
                RotationMotor z = nJoint.getRotationMotor(PhysicsSpace.AXIS_Z);
                z.set(MotorParam.LowerLimit, -1f);
                z.set(MotorParam.UpperLimit, 1f);
                joint = nJoint;
                break;

            case "p2p":
                gravity.set(0f, -1f, 0f);
                pivotInSeed.set(0.5f, 0f, 0f);
                joint = new Point2PointJoint(rbc, pivotInSeed, pivotInWorld);
                break;

            case "slider":
                gravity.set(0f, 0f, 0f); // TODO
                pivotInSeed.set(1f, 0f, 0f);
                referenceFrame = JointEnd.B;
                joint = new SliderJoint(rbc, pivotInSeed, pivotInWorld,
                        referenceFrame);
                break;

            default:
                throw new IllegalStateException("testName = " + testName);
        }

        rbc.setPhysicsSpace(physicsSpace);
        rbc.setGravity(gravity); // must be set *after* setPhysicsSpace!

        physicsSpace.add(joint);

        geometry.addControl(rbc);
    }

    /**
     * Clean up after a test.
     */
    private void cleanupAfterTest() {
        Collection<PhysicsJoint> jointList = physicsSpace.getJointList();
        for (PhysicsJoint joint : jointList) {
            joint.destroy();
            physicsSpace.remove(joint);
        }

        List<Spatial> seeds = seedNode.getChildren();
        for (Spatial geometry : seeds) {
            RigidBodyControl rbc = geometry.getControl(RigidBodyControl.class);
            rbc.setPhysicsSpace(null);
            geometry.removeControl(rbc);
            geometry.removeFromParent();
        }
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        float near = 0.02f;
        float far = 100f;
        MyCamera.setNearFar(cam, near, far);

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
        dumper.setEnabled(DumpFlags.ShadowModes, true);
        dumper.setEnabled(DumpFlags.Transforms, true);
    }

    /**
     * Configure seed groups during startup. Seeds are divided into 4 groups,
     * each with its own material and pivot location.
     */
    private void configureGroups() {
        ColorRGBA seedColors[] = new ColorRGBA[maxGroups];
        seedColors[0] = new ColorRGBA(0.2f, 0f, 0f, 1f);
        seedColors[1] = new ColorRGBA(0f, 0.07f, 0f, 1f);
        seedColors[2] = new ColorRGBA(0f, 0f, 0.3f, 1f);
        seedColors[3] = new ColorRGBA(0.2f, 0.1f, 0f, 1f);
        for (int i = 0; i < maxGroups; ++i) {
            ColorRGBA color = seedColors[i];
            materials[i] = MyAsset.createShinyMaterial(assetManager, color);
            materials[i].setFloat("Shininess", 15f);
        }
        /*
         * The 4 pivot locations are arranged in a square in the X-Y plane.
         */
        pivotLocations[0] = new Vector3f(0f, 1.5f, 0f);
        pivotLocations[1] = new Vector3f(-1.5f, 0f, 0f);
        pivotLocations[2] = new Vector3f(0f, -1.5f, 0f);
        pivotLocations[3] = new Vector3f(1.5f, 0f, 0f);
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        CollisionShape.setDefaultMargin(0.001f); // 1-mm margin
        stateManager.attach(bulletAppState);

        physicsSpace = bulletAppState.getPhysicsSpace();
        physicsSpace.setAccuracy(0.1f); // 100-msec timestep
    }

    /**
     * Toggle visualization of collision-object axes.
     */
    private void toggleAxes() {
        float length = bulletAppState.debugAxisLength();
        bulletAppState.setDebugAxisLength(0.5f - length);
    }

    /**
     * Toggle visualization of collision-object bounding boxes.
     */
    private void toggleBoxes() {
        if (bbFilter == null) {
            bbFilter = new FilterAll(true);
        } else {
            bbFilter = null;
        }

        bulletAppState.setDebugBoundingBoxFilter(bbFilter);
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
     * Toggle seed rendering on/off.
     */
    private void toggleMeshes() {
        Spatial.CullHint hint = seedNode.getLocalCullHint();
        if (hint == Spatial.CullHint.Inherit
                || hint == Spatial.CullHint.Never) {
            hint = Spatial.CullHint.Always;
        } else if (hint == Spatial.CullHint.Always) {
            hint = Spatial.CullHint.Never;
        }
        seedNode.setCullHint(hint);
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

    /**
     * Update the status text in the GUI.
     */
    private void updateStatusText() {
        int numSeeds = seedNode.getChildren().size();
        String message = String.format("test=%s, count=%d", testName, numSeeds);
        statusText.setText(message);
    }
}
