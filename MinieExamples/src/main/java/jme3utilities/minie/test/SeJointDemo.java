/*
 Copyright (c) 2018, Stephen Gold
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

import com.jme3.audio.openal.ALAudioRenderer;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.MultiSphere;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.joints.ConeJoint;
import com.jme3.bullet.joints.HingeJoint;
import com.jme3.bullet.joints.JointEnd;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.joints.Point2PointJoint;
import com.jme3.bullet.joints.SixDofJoint;
import com.jme3.bullet.joints.SixDofSpringJoint;
import com.jme3.bullet.joints.SliderJoint;
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
import jme3utilities.MyString;
import jme3utilities.debug.AxesVisualizer;
import jme3utilities.math.MyMath;
import jme3utilities.math.noise.Generator;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.ui.ActionApplication;
import jme3utilities.ui.InputMode;
import jme3utilities.ui.Signals;

/**
 * Demonstrate single-ended hinge joints.
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
     * application name for its window's title bar
     */
    final private static String applicationName = "SeJointDemo";
    /**
     * joint axes in physics-space coordinates (unit vector)
     */
    final private static Vector3f axisInWorld = Vector3f.UNIT_Z;
    // *************************************************************************
    // fields

    private BulletAppState bulletAppState;
    private CollisionShape seedShape;
    /**
     * random number generator for this application
     */
    final private Generator rng = new Generator();
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
     * mesh for visualizing seeds
     */
    final private Node seedNode = new Node("seed node");
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
     * Main entry point for the application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        /*
         * Mute the chatty loggers found in some imported packages.
         */
        Misc.setLoggingLevels(Level.WARNING);
        Logger.getLogger(ALAudioRenderer.class.getName())
                .setLevel(Level.SEVERE);

        SeJointDemo application = new SeJointDemo();
        /*
         * Customize the window's title bar.
         */
        AppSettings settings = new AppSettings(true);
        String title = applicationName + " " + MyString.join(arguments);
        settings.setTitle(title);
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
        configurePhysics();
        configureGroups();
        viewPort.setBackgroundColor(ColorRGBA.Gray);
        addLighting();
        addAxes();

        seedMesh = new Sphere(16, 32, seedRadius);
        seedShape = new MultiSphere(seedRadius);

        seedNode.setCullHint(Spatial.CullHint.Never);// meshes initially visible
        rootNode.attachChild(seedNode);
    }

    /**
     * Add application-specific hotkey bindings and override existing ones.
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        dim.bind("dump physicsSpace", KeyInput.KEY_O);
        dim.bind("dump scene", KeyInput.KEY_P);
        dim.bind("signal orbitLeft", KeyInput.KEY_LEFT);
        dim.bind("signal orbitRight", KeyInput.KEY_RIGHT);
        dim.bind("signal shower", KeyInput.KEY_I);
        dim.bind("signal shower", KeyInput.KEY_INSERT);
        dim.bind("test 6dof", KeyInput.KEY_F6);
        dim.bind("test 6dofSpring", KeyInput.KEY_F7);
        dim.bind("test cone", KeyInput.KEY_F3);
        dim.bind("test hinge", KeyInput.KEY_F2);
        dim.bind("test p2p", KeyInput.KEY_F1);
        dim.bind("test slider", KeyInput.KEY_F4);
        dim.bind("toggle pause", KeyInput.KEY_PERIOD);
        dim.bind("toggle view", KeyInput.KEY_SLASH);
    }

    /**
     * Process an action that wasn't handled by the active input mode.
     *
     * @param actionString textual description of the action (not null)
     * @param ongoing true if the action is ongoing, otherwise false
     * @param tpf time interval between render passes (in seconds, &ge;0)
     */
    @Override
    public void onAction(String actionString, boolean ongoing, float tpf) {
        if (ongoing) {
            switch (actionString) {
                case "dump physicsSpace":
                    dumpPhysicsSpace();
                    return;
                case "dump scene":
                    dumpScene();
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
                case "test p2p":
                    cleanupAfterTest();
                    testName = "p2p";
                    return;
                case "test slider":
                    cleanupAfterTest();
                    testName = "slider";
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
     * Callback invoked once per render pass.
     *
     * @param tpf time interval between render passes (in seconds, &ge;0)
     */
    @Override
    public void simpleUpdate(float tpf) {
        super.simpleUpdate(tpf);

        Signals signals = getSignals();
        if (signals.test("shower")) {
            addSeed();
        }

        float orbitAngle = 0f;
        if (signals.test("orbitLeft")) {
            orbitAngle += tpf;
        }
        if (signals.test("orbitRight")) {
            orbitAngle -= tpf;
        }
        if (orbitAngle != 0f) {
            orbitCamera(orbitAngle);

        }
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
     * Add a dynamic seed to the scene.
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
        int groupIndex = MyMath.modulo(rng.nextInt(), numGroups);
        Material material = materials[groupIndex];
        Vector3f pivotInWorld = pivotLocations[groupIndex];
        /*
         * Randomize the new seed's initial location and velocity.
         */
        Vector3f velocity = rng.nextVector3f();
        Vector3f location = rng.nextVector3f();
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

        rbc.addJoint(joint);
        physicsSpace.add(joint);

        rbc.setPhysicsSpace(physicsSpace);
        rbc.setGravity(gravity); // must be set *after* setPhysicsSpace!

        geometry.addControl(rbc);
        ++numSeeds;
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
        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(4f);

        cam.setLocation(new Vector3f(2.65f, 2.42f, 9.37f));
        cam.setRotation(new Quaternion(0f, 0.9759f, -0.04f, -0.2136f));
    }

    /**
     * Configure seed groups during startup. Seeds are divided into 4 groups,
     * each with its own color and pivot location.
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
        CollisionShape.setDefaultMargin(0.001f); // 1 mm

        bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);

        physicsSpace = bulletAppState.getPhysicsSpace();
        physicsSpace.setAccuracy(0.1f); // 10 Hz
        physicsSpace.setSolverNumIterations(15);
    }

    /**
     * Process a "dump physicsSpace" action.
     */
    private void dumpPhysicsSpace() {
        PhysicsDumper dumper = new PhysicsDumper();
        dumper.dump(physicsSpace);
    }

    /**
     * Process a "dump scene" action.
     */
    private void dumpScene() {
        PhysicsDumper dumper = new PhysicsDumper();
        //dumper.setDumpBucket(true);
        //dumper.setDumpCull(true);
        //dumper.setDumpOverride(true);
        //dumper.setDumpShadow(true);
        dumper.setDumpTransform(true);
        //dumper.setDumpUser(true);
        dumper.dump(rootNode);
    }

    /**
     * Orbit the camera around the world's Y axis.
     */
    private void orbitCamera(float orbitAngle) {
        Quaternion rotate = new Quaternion();
        rotate.fromAngles(0f, orbitAngle, 0f);

        Vector3f camLocation = cam.getLocation().clone();
        Vector3f centerLocation
                = camLocation.clone().multLocal(Vector3f.UNIT_Y);
        Vector3f xzOffset = camLocation.subtract(centerLocation);
        rotate.mult(xzOffset, xzOffset);
        Vector3f newLocation = centerLocation.add(xzOffset);
        cam.setLocation(newLocation);

        Vector3f camDirection = cam.getDirection();
        rotate.mult(camDirection, camDirection);
        cam.lookAtDirection(camDirection, Vector3f.UNIT_Y);
    }

    /**
     * Toggle seed rendering on/off.
     */
    private void toggleMeshes() {
        Spatial.CullHint hint = seedNode.getCullHint();
        if (hint == Spatial.CullHint.Always) {
            hint = Spatial.CullHint.Dynamic;
        } else {
            hint = Spatial.CullHint.Always;
        }
        seedNode.setCullHint(hint);
    }

    /**
     * Toggle physics simulation paused/running.
     */
    private void togglePause() {
        float newSpeed = (speed > Float.MIN_VALUE) ? Float.MIN_VALUE : 1f;
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
