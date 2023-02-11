/*
 Copyright (c) 2018-2023, Stephen Gold
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
import com.jme3.system.AppSettings;
import java.util.Collection;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MyAsset;
import jme3utilities.MyCamera;
import jme3utilities.MyString;
import jme3utilities.math.noise.Generator;
import jme3utilities.mesh.Icosphere;
import jme3utilities.minie.DumpFlags;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.minie.test.common.PhysicsDemo;
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
public class SeJointDemo extends PhysicsDemo {
    // *************************************************************************
    // constants and loggers

    /**
     * seed radius (in mesh units)
     */
    final private static float seedRadius = 4.8f;
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
    private static BitmapText statusText;
    /**
     * AppState to manage the PhysicsSpace
     */
    private static BulletAppState bulletAppState;
    private static CollisionShape seedShape;
    /**
     * material for each type of seed
     */
    final private static Material[] materials = new Material[maxGroups];

    final private static Matrix3f rotInSeed = new Matrix3f();
    final private static Matrix3f rotInWorld = new Matrix3f();
    /**
     * mesh for visualizing seeds
     */
    private static Mesh seedMesh;
    /**
     * scene-graph node for visualizing seeds
     */
    final private static Node meshesNode = new Node("meshes node");
    /**
     * name of the test that's currently running
     */
    private static String testName = "p2p";
    /**
     * joint axis in each seed's local coordinates (unit vector)
     */
    final private static Vector3f axisInSeed = new Vector3f();
    /**
     * gravity vector (in physics-space coordinates)
     */
    final private static Vector3f gravity = new Vector3f();
    /**
     * offset of pivot relative to each seed (in scaled shape coordinates)
     */
    final private static Vector3f pivotInSeed = new Vector3f();
    /**
     * pivot location for each group (in physics-space coordinates)
     */
    final private static Vector3f[] pivotLocations = new Vector3f[maxGroups];
    /**
     * scale factors that determine the seed's shape
     */
    final private static Vector3f seedScale = new Vector3f();
    // *************************************************************************
    // constructors

    /**
     * Instantiate the SeJointDemo application.
     */
    public SeJointDemo() { // explicit to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the SeJointDemo application.
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
        settings.setResizable(true);
        settings.setSamples(4); // anti-aliasing
        settings.setTitle(title); // Customize the window's title bar.

        Application application = new SeJointDemo();
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
        // Add the status text to the GUI.
        statusText = new BitmapText(guiFont);
        guiNode.attachChild(statusText);

        super.acorusInit();

        configureCamera();
        configureDumper();
        configurePhysics();
        configureGroups();

        ColorRGBA skyColor = new ColorRGBA(0.1f, 0.2f, 0.4f, 1f);
        viewPort.setBackgroundColor(skyColor);

        addLighting();

        float length = 32f;
        attachWorldAxes(length);

        int numRefineSteps = 1;
        seedMesh = new Icosphere(numRefineSteps, seedRadius);
        seedShape = new MultiSphere(seedRadius);

        meshesNode.setCullHint(Spatial.CullHint.Never); // with meshes visible
        rootNode.attachChild(meshesNode);
    }

    /**
     * Configure the PhysicsDumper during startup.
     */
    @Override
    public void configureDumper() {
        super.configureDumper();

        PhysicsDumper dumper = getDumper();
        dumper.setEnabled(DumpFlags.JointsInSpaces, true);
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
        return 20f;
    }

    /**
     * Add application-specific hotkey bindings (and override existing ones, if
     * necessary).
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        dim.bind("add", KeyInput.KEY_INSERT);

        dim.bind(asDumpScene, KeyInput.KEY_P);
        dim.bind(asDumpSpace, KeyInput.KEY_O);

        dim.bind(asCollectGarbage, KeyInput.KEY_G);

        dim.bindSignal(CameraInput.FLYCAM_LOWER, KeyInput.KEY_DOWN);
        dim.bindSignal(CameraInput.FLYCAM_RISE, KeyInput.KEY_UP);
        dim.bindSignal("orbitLeft", KeyInput.KEY_LEFT);
        dim.bindSignal("orbitRight", KeyInput.KEY_RIGHT);
        dim.bind("signal shower", KeyInput.KEY_I);

        dim.bind("test 6dof", KeyInput.KEY_F6);
        dim.bind("test 6dofSpring", KeyInput.KEY_F7);
        dim.bind("test cone", KeyInput.KEY_F3);
        dim.bind("test hinge", KeyInput.KEY_F2);
        dim.bind("test new6dof", KeyInput.KEY_F8);
        dim.bind("test p2p", KeyInput.KEY_F1);
        dim.bind("test slider", KeyInput.KEY_F4);

        dim.bind(asToggleAabbs, KeyInput.KEY_APOSTROPHE);
        dim.bind(asToggleHelp, KeyInput.KEY_H);
        dim.bind(asTogglePause, KeyInput.KEY_PAUSE, KeyInput.KEY_PERIOD);
        dim.bind(asTogglePcoAxes, KeyInput.KEY_SEMICOLON);
        dim.bind("toggle view", KeyInput.KEY_SLASH);
    }

    /**
     * Process an action that wasn't handled by the active InputMode.
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

        Signals signals = getSignals();
        if (signals.test("shower")) {
            addSeed();
        }

        updateStatusText();
    }
    // *************************************************************************
    // private methods

    /**
     * Add lighting to the main scene.
     */
    private void addLighting() {
        ColorRGBA ambientColor = new ColorRGBA(2f, 2f, 2f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        rootNode.addLight(ambient);
        ambient.setName("ambient");

        Vector3f direction = new Vector3f(1f, -2f, -1f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction);
        rootNode.addLight(sun);
        sun.setName("sun");
    }

    /**
     * Add a dynamic rigid body to the scene and PhysicsSpace.
     */
    private void addSeed() {
        int numSeeds = meshesNode.getChildren().size();
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

        // Randomize which group the new seed is in.
        Generator random = getGenerator();
        int groupIndex = random.nextInt(0, numGroups - 1);
        Material material = materials[groupIndex];
        Vector3f pivotInWorld = pivotLocations[groupIndex];

        // Randomize the new seed's initial location and velocity.
        Vector3f velocity = random.nextVector3f();
        Vector3f location = random.nextVector3f();
        location.multLocal(20f, 40f, 20f);
        location.addLocal(0f, 160f, 0f);

        Geometry geometry = new Geometry("seed", seedMesh);
        meshesNode.attachChild(geometry);
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
                pivotInSeed.set(40f, 0f, 0f);
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
                pivotInSeed.set(40f, 0f, 0f);
                rotInSeed.loadIdentity();
                angle = (groupIndex - 1) * FastMath.HALF_PI;
                rotInWorld.fromAngleAxis(angle, Vector3f.UNIT_Z);
                referenceFrame = JointEnd.B;
                SixDofSpringJoint springJoint = new SixDofSpringJoint(
                        rbc, pivotInSeed, pivotInWorld, rotInSeed, rotInWorld,
                        referenceFrame);
                springJoint.setAngularLowerLimit(new Vector3f(0f, -1f, -1f));
                springJoint.setAngularUpperLimit(new Vector3f(0f, 1f, 1f));
                joint = springJoint;
                break;

            case "cone":
                gravity.zero();
                pivotInSeed.set(80f, 0f, 0f);
                rotInSeed.fromAngleAxis(FastMath.PI, Vector3f.UNIT_Z);
                ConeJoint coneJoint
                        = new ConeJoint(rbc, pivotInSeed, rotInSeed);
                coneJoint.setLimit(1f, 1f, 1e30f); // radians
                joint = coneJoint;
                break;

            case "hinge":
                axisInSeed.set(40f, 40f, 0f).normalizeLocal();
                gravity.set(0f, -40f, 0f);
                pivotInSeed.set(0f, 40f, 0f);
                referenceFrame = JointEnd.A;
                joint = new HingeJoint(rbc, pivotInSeed, pivotInWorld,
                        axisInSeed, axisInWorld, referenceFrame);
                break;

            case "new6dof":
                gravity.zero();
                pivotInSeed.set(40f, 0f, 0f);
                rotInSeed.loadIdentity();
                angle = (groupIndex - 1) * FastMath.HALF_PI;
                rotInWorld.fromAngleAxis(angle, Vector3f.UNIT_Z);
                New6Dof nJoint = new New6Dof(rbc, pivotInSeed, pivotInWorld,
                        rotInSeed, rotInWorld, RotationOrder.XYZ);
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
                gravity.set(0f, -40f, 0f);
                pivotInSeed.set(20f, 0f, 0f);
                joint = new Point2PointJoint(rbc, pivotInSeed, pivotInWorld);
                break;

            case "slider":
                gravity.set(0f, 0f, 0f); // TODO
                pivotInSeed.set(40f, 0f, 0f);
                referenceFrame = JointEnd.B;
                joint = new SliderJoint(
                        rbc, pivotInSeed, pivotInWorld, referenceFrame);
                break;

            default:
                throw new IllegalStateException("testName = " + testName);
        }

        PhysicsSpace physicsSpace = getPhysicsSpace();
        rbc.setPhysicsSpace(physicsSpace);
        rbc.setGravity(gravity); // must be set *after* setPhysicsSpace!

        physicsSpace.addJoint(joint);

        geometry.addControl(rbc);
    }

    /**
     * Clean up after a test.
     */
    private void cleanupAfterTest() {
        PhysicsSpace physicsSpace = getPhysicsSpace();
        Collection<PhysicsJoint> jointList = physicsSpace.getJointList();
        for (PhysicsJoint joint : jointList) {
            joint.destroy();
            physicsSpace.removeJoint(joint);
        }

        List<Spatial> seeds = meshesNode.getChildren();
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
        float near = 0.8f;
        float far = 4000f;
        MyCamera.setNearFar(cam, near, far);

        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(160f);
        flyCam.setZoomSpeed(160f);

        cam.setLocation(new Vector3f(106f, 96.8f, 374.8f));
        cam.setRotation(new Quaternion(0f, 0.9759f, -0.04f, -0.2136f));

        AppState orbitState
                = new CameraOrbitAppState(cam, "orbitLeft", "orbitRight");
        stateManager.attach(orbitState);
    }

    /**
     * Configure seed groups during startup. Seeds are divided into 4 groups,
     * each with its own material and pivot location.
     */
    private void configureGroups() {
        ColorRGBA[] seedColors = new ColorRGBA[maxGroups];
        seedColors[0] = new ColorRGBA(0.2f, 0f, 0f, 1f);
        seedColors[1] = new ColorRGBA(0f, 0.07f, 0f, 1f);
        seedColors[2] = new ColorRGBA(0f, 0f, 0.3f, 1f);
        seedColors[3] = new ColorRGBA(0.2f, 0.1f, 0f, 1f);
        for (int i = 0; i < maxGroups; ++i) {
            ColorRGBA color = seedColors[i];
            materials[i] = MyAsset.createShinyMaterial(assetManager, color);
            materials[i].setFloat("Shininess", 15f);
        }

        // The 4 pivot locations are arranged in a square in the X-Y plane.
        pivotLocations[0] = new Vector3f(0f, 60f, 0f);
        pivotLocations[1] = new Vector3f(-60f, 0f, 0f);
        pivotLocations[2] = new Vector3f(0f, -60f, 0f);
        pivotLocations[3] = new Vector3f(60f, 0f, 0f);
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);

        PhysicsSpace physicsSpace = getPhysicsSpace();
        physicsSpace.setAccuracy(0.1f); // 100-msec timestep
    }

    /**
     * Toggle seed rendering on/off.
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
        int numSeeds = meshesNode.getChildren().size();
        boolean isPaused = isPaused();
        String message = String.format("test=%s, count=%d%s", testName,
                numSeeds, isPaused ? "  PAUSED" : "");
        statusText.setText(message);
    }
}
