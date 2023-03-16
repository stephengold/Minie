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

import com.jme3.anim.Armature;
import com.jme3.anim.Joint;
import com.jme3.anim.SkinningControl;
import com.jme3.app.StatsAppState;
import com.jme3.app.state.AppState;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.animation.CenterHeuristic;
import com.jme3.bullet.animation.DacConfiguration;
import com.jme3.bullet.animation.DynamicAnimControl;
import com.jme3.bullet.animation.IKJoint;
import com.jme3.bullet.animation.LinkConfig;
import com.jme3.bullet.animation.MassHeuristic;
import com.jme3.bullet.animation.PhysicsLink;
import com.jme3.bullet.animation.RangeOfMotion;
import com.jme3.bullet.animation.ShapeHeuristic;
import com.jme3.bullet.joints.Constraint;
import com.jme3.font.Rectangle;
import com.jme3.input.CameraInput;
import com.jme3.input.KeyInput;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.shadow.DirectionalLightShadowRenderer;
import com.jme3.system.AppSettings;
import java.util.ArrayDeque;
import java.util.BitSet;
import java.util.Deque;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MyAsset;
import jme3utilities.MyCamera;
import jme3utilities.MySkeleton;
import jme3utilities.MyString;
import jme3utilities.NameGenerator;
import jme3utilities.debug.SkeletonVisualizer;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.RectSizeLimits;
import jme3utilities.math.noise.Generator;
import jme3utilities.minie.DumpFlags;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.minie.test.common.PhysicsDemo;
import jme3utilities.minie.test.mesh.TubeTreeMesh;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.DisplaySettings;
import jme3utilities.ui.DsEditOverlay;
import jme3utilities.ui.InputMode;
import jme3utilities.ui.ShowDialog;

/**
 * Simulate ropes using DynamicAnimControl.
 * <p>
 * Seen in the February 2019 demo video:
 * https://www.youtube.com/watch?v=7PYDAyB5RCE
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class RopeDemo extends PhysicsDemo {
    // *************************************************************************
    // classes and enums

    /**
     * Enumerate the rope shapes used in this demo.
     */
    private enum RopeShape {
        /**
         * a rope with 4 ends
         */
        Cross,
        /**
         * a noose, pinned at the standing end
         */
        Noose,
        /**
         * a noose, pinned at the standing end, kinematically pre-spliced
         */
        NooseSpliced,
        /**
         * a rope ring
         */
        Ring,
        /**
         * a rope ring, kinematically pre-spliced
         */
        RingSpliced,
        /**
         * a plain rope with slack, pinned at both ends
         */
        Slackline
    }
    // *************************************************************************
    // constants and loggers

    /**
     * cross-section radius for ropes (in mesh units)
     */
    final private static float ropeRadius = 0.4f;
    /**
     * minimum curl radius for ropes (in mesh units)
     */
    final private static float minCurlRadius = 4f * ropeRadius;
    /**
     * mass of each link (in physics mass units)
     */
    final private static float linkMass = 1e-6f;
    /**
     * length of each rope segment (in mesh units)
     */
    final private static float stepLength = 1.9f * minCurlRadius;
    /**
     * number of mesh loops in each tube segment
     */
    final private static int loopsPerSegment = 3;
    /**
     * number of sample points per mesh loop
     */
    final private static int samplesPerLoop = 12;
    /**
     * link configuration for leaf joints (shrunken hull shape)
     */
    final private static LinkConfig leafConfig = new LinkConfig(
            linkMass, MassHeuristic.Mass, ShapeHeuristic.VertexHull,
            new Vector3f(0.84f, 0.84f, 0.84f), CenterHeuristic.Mean);
    /**
     * link configuration for non-leaf joints (stretched capsule shape)
     */
    final private static LinkConfig ropeConfig = new LinkConfig(
            linkMass, MassHeuristic.Mass, ShapeHeuristic.TwoSphere,
            new Vector3f(1f, 1f, 2.5f), CenterHeuristic.Mean);
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(RopeDemo.class.getName());
    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName
            = RopeDemo.class.getSimpleName();
    // *************************************************************************
    // fields

    /**
     * true once {@link #initWhenReady()} has been invoked for the latest rope
     */
    private static boolean dacReadyInitDone = false;
    /**
     * AppState to manage the PhysicsSpace
     */
    private static BulletAppState bulletAppState;
    /**
     * physics controls for the ropes, in order of creation
     */
    final private static Deque<DynamicAnimControl> dacs = new ArrayDeque<>(12);
    /**
     * shapes of the ropes, in order of creation
     */
    final private static Deque<RopeShape> shapes = new ArrayDeque<>(12);
    /**
     * proposed display settings (for editing)
     */
    private static DisplaySettings proposedSettings;
    /**
     * AppState to manage the display-settings editor
     */
    private static DsEditOverlay dseOverlay;
    /**
     * generate names for rope geometries
     */
    final private static NameGenerator geometryNamer = new NameGenerator();
    /**
     * parent for rope geometries
     */
    final private static Node meshesNode = new Node("meshes node");
    /**
     * visualizer for the Armature of the most recently added rope
     */
    private static SkeletonVisualizer sv;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the RopeDemo application.
     */
    public RopeDemo() { // made explicit to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the RopeDemo application.
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
        dseOverlay = new DsEditOverlay(proposedSettings);
        dseOverlay.setBackgroundColor(new ColorRGBA(0.05f, 0f, 0f, 1f));
        boolean success = stateManager.attach(dseOverlay);
        assert success;
        super.acorusInit();

        // Hide the render-statistics overlay.
        stateManager.getState(StatsAppState.class).toggleStats();

        configureCamera();
        configureDumper();
        generateMaterials();
        configurePhysics();

        ColorRGBA skyColor = new ColorRGBA(0.1f, 0.2f, 0.4f, 1f);
        viewPort.setBackgroundColor(skyColor);

        addLighting();

        float halfExtent = 650f;
        float topY = 0f;
        attachCubePlatform(halfExtent, topY);
        addSkeleton();

        rootNode.attachChild(meshesNode);
    }

    /**
     * Configure the PhysicsDumper.
     */
    @Override
    public void configureDumper() {
        super.configureDumper();

        PhysicsDumper dumper = getDumper();
        dumper.setEnabled(DumpFlags.JointsInSpaces, true);
    }

    /**
     * Calculate screen bounds for a detailed help node.
     *
     * @param viewPortWidth (in pixels, &gt;0)
     * @param viewPortHeight (in pixels, &gt;0)
     * @return a new instance
     */
    @Override
    public Rectangle detailedHelpBounds(int viewPortWidth, int viewPortHeight) {
        // Position help nodes on the right side of the viewport.
        float margin = 10f; // in pixels
        float height = viewPortHeight - (2f * margin);
        float width = 260f; // in pixels
        float leftX = viewPortWidth - (width + margin);
        float topY = margin + height;
        Rectangle result = new Rectangle(leftX, topY, width, height);

        return result;
    }

    /**
     * Initialize materials during startup.
     */
    @Override
    public void generateMaterials() {
        super.generateMaterials();

        ColorRGBA taupe = new ColorRGBA().setAsSrgb(0.6f, 0.5f, 0.4f, 1f);
        Material ropeMaterial
                = MyAsset.createShadedMaterial(assetManager, taupe);
        registerMaterial("rope", ropeMaterial);
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
     * Determine the length of debug axis arrows when visible.
     *
     * @return the desired length (in physics-space units, &ge;0)
     */
    @Override
    protected float maxArrowLength() {
        return 0.5f;
    }

    /**
     * Add application-specific hotkey bindings and override existing ones.
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        dim.bind("add Cross", KeyInput.KEY_F2);
        dim.bind("add Ring", KeyInput.KEY_F3);
        dim.bind("add RingSpliced", KeyInput.KEY_F4);
        dim.bind("add Noose", KeyInput.KEY_F7);
        dim.bind("add NooseSpliced", KeyInput.KEY_F8);
        dim.bind("add Slackline", KeyInput.KEY_F1);

        dim.bind("delete", KeyInput.KEY_BACK, KeyInput.KEY_DELETE);

        dim.bind(asDumpSpace, KeyInput.KEY_O);
        dim.bind(asDumpViewport, KeyInput.KEY_P);

        dim.bind(asEditDisplaySettings, KeyInput.KEY_TAB);
        dim.bind("go limp", KeyInput.KEY_SPACE);
        dim.bind("pull a pin", KeyInput.KEY_X);
        dim.bind("save", KeyInput.KEY_SEMICOLON);

        dim.bindSignal(CameraInput.FLYCAM_LOWER, KeyInput.KEY_DOWN);
        dim.bindSignal(CameraInput.FLYCAM_RISE, KeyInput.KEY_UP);
        dim.bindSignal("orbitLeft", KeyInput.KEY_LEFT);
        dim.bindSignal("orbitRight", KeyInput.KEY_RIGHT);

        dim.bind(asToggleAabbs, KeyInput.KEY_APOSTROPHE);
        dim.bind(asToggleDebug, KeyInput.KEY_SLASH);
        dim.bind(asToggleHelp, KeyInput.KEY_H);
        dim.bind("toggle meshes", KeyInput.KEY_M);
        dim.bind(asTogglePause, KeyInput.KEY_PAUSE, KeyInput.KEY_PERIOD);
        dim.bind(asTogglePcoAxes, KeyInput.KEY_SEMICOLON);
        dim.bind("toggle skeleton", KeyInput.KEY_V);
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
                case "delete":
                    delete();
                    return;

                case asEditDisplaySettings:
                    activateInputMode("dsEdit");
                    return;

                case "go limp":
                    goLimp();
                    return;

                case "pull a pin":
                    pullAPin();
                    return;

                case "toggle meshes":
                    toggleMeshes();
                    return;
                case "toggle skeleton":
                    toggleSkeleton();
                    return;

                default:
            }

            String[] words = actionString.split(" ");
            if (words.length >= 2 && "add".equals(words[0])) {
                addRope(words[1]);
                return;
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
        dseOverlay.onViewPortResize(newWidth, newHeight);
        proposedSettings.resize(newWidth, newHeight);

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

        DynamicAnimControl latestDac = dacs.peekLast();
        if (!dacReadyInitDone && latestDac != null && latestDac.isReady()) {
            initWhenReady();
            dacReadyInitDone = true;
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Add lighting and shadows to the main scene.
     */
    private void addLighting() {
        ColorRGBA ambientColor = new ColorRGBA(0.2f, 0.2f, 0.2f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        rootNode.addLight(ambient);
        ambient.setName("ambient");

        Vector3f direction = new Vector3f(1f, -2f, -2f).normalizeLocal();
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
     * Add a kinematic rope (with the named shape) to the scene.
     *
     * @param shapeName the name of the shape to add (not null, not empty)
     */
    private void addRope(String shapeName) {
        DynamicAnimControl latestDac = dacs.peekLast();
        if (latestDac != null && latestDac.getTorsoLink().isKinematic()) {
            // only one kinematic rope at a time
            delete();
        }

        RopeShape shape = RopeShape.valueOf(shapeName);
        switch (shape) {
            case Cross:
                addRopeCross();
                break;
            case Noose:
                addRopeNoose(false);
                break;
            case NooseSpliced:
                addRopeNoose(true);
                break;
            case Ring:
                addRopeRing(false);
                break;
            case RingSpliced:
                addRopeRing(true);
                break;
            case Slackline:
                addRopeSlackline();
                break;
            default:
                throw new IllegalArgumentException(shapeName);
        }

        shapes.addLast(shape);
    }

    /**
     * Add a rope cross to the scene.
     */
    private void addRopeCross() {
        // Generate a cross-shaped Armature.
        int[] stepCounts = {5, 5, 5, 5};
        Vector3f[] stepOffsets = {
            new Vector3f(stepLength, 0f, 0f),
            new Vector3f(-stepLength, 0f, 0f),
            new Vector3f(0f, 0f, stepLength),
            new Vector3f(0f, 0f, -stepLength)
        };
        Armature armature = makeArmature(stepCounts, stepOffsets);

        TubeTreeMesh ropeMesh = new TubeTreeMesh(
                armature, ropeRadius, 0f, loopsPerSegment, samplesPerLoop);

        String geometryName = geometryNamer.unique("rope cross");
        Geometry geometry = new Geometry(geometryName, ropeMesh);

        addRopeSpatial(armature, geometry);

        // Curl all branches clockwise to introduce some slack.
        curlBranch(armature, 0, Vector3f.UNIT_Y, 0.2f);
        curlBranch(armature, 1, Vector3f.UNIT_Y, 0.2f);
        curlBranch(armature, 2, Vector3f.UNIT_Y, 0.2f);
        curlBranch(armature, 3, Vector3f.UNIT_Y, 0.2f);
    }

    /**
     * Add a noose to the scene.
     *
     * @param kinematicSplice true to pre-splice the ends kinematically, false
     * to leave it Y-shaped until dynamic mode is set
     */
    private void addRopeNoose(boolean kinematicSplice) {
        // Generate a Y-shaped Armature. Branch0 forms the stem of the Y.
        int[] stepCounts = {6, 6, 6};
        float dx = stepLength * MyMath.rootHalf;
        Vector3f[] stepOffsets = {
            new Vector3f(stepLength, 0f, 0f),
            new Vector3f(-dx, 0f, -dx),
            new Vector3f(-dx, 0f, dx)
        };
        Armature armature = makeArmature(stepCounts, stepOffsets);

        TubeTreeMesh ropeMesh = new TubeTreeMesh(
                armature, ropeRadius, 0f, loopsPerSegment, samplesPerLoop);
        String geometryName = geometryNamer.unique("noose");
        Geometry geometry = new Geometry(geometryName, ropeMesh);

        addRopeSpatial(armature, geometry);

        if (kinematicSplice) {
            // Curl branch1 and branch2 toward one another, 90 degrees each.
            curlBranch(armature, 1, Vector3f.UNIT_Y, FastMath.HALF_PI);
            curlBranch(armature, 2, Vector3f.UNIT_Y, -FastMath.HALF_PI);
        }
    }

    /**
     * Add a rope ring to the scene.
     *
     * @param kinematicSplice true to pre-splice the ends kinematically, false
     * to leave it linear until dynamic mode is set
     */
    private void addRopeRing(boolean kinematicSplice) {
        // Generate a double-ended straight-line Armature.
        int[] stepCounts = {8, 8};
        Vector3f[] stepOffsets = {
            new Vector3f(stepLength, 0f, 0f),
            new Vector3f(-stepLength, 0f, 0f)
        };
        Armature armature = makeArmature(stepCounts, stepOffsets);

        TubeTreeMesh ropeMesh = new TubeTreeMesh(
                armature, ropeRadius, 0f, loopsPerSegment, samplesPerLoop);

        String geometryName = geometryNamer.unique("rope ring");
        Geometry geometry = new Geometry(geometryName, ropeMesh);

        addRopeSpatial(armature, geometry);

        if (kinematicSplice) {
            // Curl branch0 and branch1 toward one another, 180 degrees each.
            curlBranch(armature, 0, Vector3f.UNIT_Y, FastMath.PI);
            curlBranch(armature, 1, Vector3f.UNIT_Y, -FastMath.PI);
        }
    }

    /**
     * Add a plain rope to the scene. The rope begins in the X-Z plane.
     */
    private void addRopeSlackline() {
        // Generate a double-ended straight-line Armature.
        int[] stepCounts = {8, 8};
        Vector3f[] stepOffsets = {
            new Vector3f(0f, 0f, stepLength),
            new Vector3f(0f, 0f, -stepLength)
        };
        Armature armature = makeArmature(stepCounts, stepOffsets);

        float leafOvershoot = 0.8f * ropeRadius;
        TubeTreeMesh ropeMesh = new TubeTreeMesh(armature, ropeRadius,
                leafOvershoot, loopsPerSegment, samplesPerLoop);
        String geometryName = geometryNamer.unique("slackline");
        Geometry geometry = new Geometry(geometryName, ropeMesh);

        addRopeSpatial(armature, geometry);

        // Curl both branches clockwise to introduce some slack.
        curlBranch(armature, 0, Vector3f.UNIT_Y, 0.2f);
        curlBranch(armature, 1, Vector3f.UNIT_Y, 0.2f);
    }

    /**
     * Code for adding a rope that's shared by all shapes.
     *
     * @param armature (not null)
     * @param spatial (not null)
     */
    private void addRopeSpatial(Armature armature, Spatial spatial) {
        // Set a random elevation and Y-axis rotation.
        Generator random = getGenerator();
        float elevation = random.nextFloat(12f, 24f);
        float rotationAngle = random.nextFloat(0f, FastMath.TWO_PI);
        spatial.move(0f, elevation, 0f);
        spatial.rotate(0f, rotationAngle, 0f);

        spatial.setCullHint(Spatial.CullHint.Never);
        Material ropeMaterial = findMaterial("rope");
        spatial.setMaterial(ropeMaterial);
        spatial.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);

        SkinningControl sControl = new SkinningControl(armature);
        spatial.addControl(sControl);
        sControl.setHardwareSkinningPreferred(false);
        sv.setSubject(sControl);

        assert stepLength < 2f * minCurlRadius : stepLength;
        float maxAngle = FastMath.acos(stepLength / (2f * minCurlRadius));
        RangeOfMotion rom = new RangeOfMotion(maxAngle);

        DynamicAnimControl dac = new DynamicAnimControl();
        for (Joint joint : MySkeleton.preOrderJoints(armature)) {
            if (joint.getParent() != null) {
                String jointName = joint.getName();
                boolean isLeafJoint = joint.getChildren().isEmpty();
                LinkConfig linkConfig = isLeafJoint ? leafConfig : ropeConfig;
                dac.link(jointName, linkConfig, rom);
            }
        }
        dac.setConfig(DacConfiguration.torsoName, ropeConfig);
        dac.setDamping(0.9f);
        dac.setGravity(new Vector3f(0f, -120f, 0f));

        spatial.addControl(dac);
        PhysicsSpace physicsSpace = getPhysicsSpace();
        dac.setPhysicsSpace(physicsSpace);
        meshesNode.attachChild(spatial);

        setFrictionAll(9e9f);

        dacs.addLast(dac);
        dacReadyInitDone = false;
    }

    /**
     * Add the skeleton visualizer the scene.
     */
    private void addSkeleton() {
        sv = new SkeletonVisualizer(assetManager, null);
        rootNode.addControl(sv);
        sv.setLineColor(ColorRGBA.Red);
        sv.setHeadColor(0, ColorRGBA.Green);
        sv.setHeadSize(8);
    }

    /**
     * Choose 3 distinct vertices in the end cap of the indexed branch.
     *
     * @param branchIndex which branch (&ge;0)
     * @return a new array of 3 vertex specifiers
     */
    private static String[] capSpecs(int branchIndex) {
        int capVertex0 = capVertex0(branchIndex);
        String geometryName = dacs.peekLast().getSpatial().getName();
        String[] result = new String[3];
        result[0] = String.format("%d/%s", capVertex0, geometryName);

        if (samplesPerLoop == 6) {
            result[1] = String.format("%d/%s", capVertex0 + 1, geometryName);
            result[2] = String.format("%d/%s", capVertex0 + 7, geometryName);
        } else if (samplesPerLoop == 9) {
            result[1] = String.format("%d/%s", capVertex0 + 4, geometryName);
            result[2] = String.format("%d/%s", capVertex0 + 13, geometryName);
        } else if (samplesPerLoop == 12) {
            result[1] = String.format("%d/%s", capVertex0 + 7, geometryName);
            result[2] = String.format("%d/%s", capVertex0 + 19, geometryName);
        } else {
            String message = "samplesPerLoop = " + samplesPerLoop;
            throw new IllegalStateException(message);
        }

        return result;
    }

    /**
     * Find the index of the first vertex in the end cap of the indexed branch.
     *
     * @param branchIndex which branch(&ge;0)
     * @return the vertex index (&ge;0)
     */
    private static int capVertex0(int branchIndex) {
        assert branchIndex >= 0 : branchIndex;

        DynamicAnimControl latestDac = dacs.peekLast();
        Armature armature = latestDac.getArmature();
        int lastStep = countSteps(armature, branchIndex) - 1;
        String endJointName = jointName(branchIndex, lastStep);

        Geometry geometry = (Geometry) latestDac.getSpatial();
        TubeTreeMesh mesh = (TubeTreeMesh) geometry.getMesh();
        BitSet bitSet = mesh.listCapVertices(endJointName);
        assert bitSet.cardinality() == mesh.verticesPerCap();
        int result = bitSet.nextSetBit(0);

        assert result >= 0 : result;
        return result;
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        float near = 0.1f * ropeRadius;
        float far = 250f;
        MyCamera.setNearFar(cam, near, far);

        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(25f);
        flyCam.setZoomSpeed(25f);

        cam.setLocation(new Vector3f(0f, 33f, 55f));
        cam.setRotation(new Quaternion(0f, 0.982f, -0.188f, 0f));

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

        PhysicsSpace physicsSpace = bulletAppState.getPhysicsSpace();
        physicsSpace.getSolverInfo().setNumIterations(20);
    }

    /**
     * Count the steps in the indexed branch of the specified Armature.
     *
     * @param armature which Armature (not null)
     * @param branchIndex the index of the branch (&ge;0)
     * @return the count (&ge;0)
     */
    private static int countSteps(Armature armature, int branchIndex) {
        int numSteps = 0;

        while (true) {
            String jointName = jointName(branchIndex, numSteps);
            if (armature.getJoint(jointName) == null) {
                break;
            } else {
                ++numSteps;
            }
        }

        return numSteps;
    }

    /**
     * Kinematically curl a branch of the specified Armature around the
     * specified axis.
     *
     * @param armature the Armature to pose (not null)
     * @param branchIndex (&ge;0)
     * @param axis (in each joint's local coordinates, not null, not zero)
     * @param totalAngle the angle between the first and last steps (in radians,
     * may be negative)
     */
    private static void curlBranch(Armature armature, int branchIndex,
            Vector3f axis, float totalAngle) {
        assert !MyVector3f.isZero(axis);

        // Count the steps in this branch.
        int numSteps = countSteps(armature, branchIndex);
        assert numSteps > 1 : numSteps;

        // Calculate local rotation.
        float turnAngle = totalAngle / (numSteps - 1);
        Quaternion turn = new Quaternion().fromAngleAxis(turnAngle, axis);

        // Apply the rotation to each Joint.
        for (int stepIndex = 0; stepIndex < numSteps; ++stepIndex) {
            String jointName = jointName(branchIndex, stepIndex);
            Joint joint = armature.getJoint(jointName);
            joint.setLocalRotation(turn);
        }
    }

    /**
     * Delete the most recently added rope.
     */
    private static void delete() {
        DynamicAnimControl latestDac = dacs.peekLast();
        if (latestDac != null) {
            latestDac.setEnabled(false);
            latestDac.setPhysicsSpace(null);
            Spatial spatial = latestDac.getSpatial();
            spatial.removeControl(latestDac);
            meshesNode.detachChild(spatial);
            sv.setSubject(null);

            dacs.removeLast();
            shapes.removeLast();

            latestDac = dacs.peekLast();
            if (latestDac != null) {
                spatial = latestDac.getSpatial();
                SkinningControl sControl
                        = spatial.getControl(SkinningControl.class);
                sv.setSubject(sControl);
            }
        }
    }

    /**
     * Put the most recently added rope into dynamic mode, with gravity.
     */
    private static void goLimp() {
        DynamicAnimControl latestDac = dacs.peekLast();
        if (latestDac != null && latestDac.isReady()) {
            PhysicsLink rootLink = latestDac.getTorsoLink();
            Vector3f gravity = latestDac.gravity(null);
            latestDac.setDynamicSubtree(rootLink, gravity, false);
        }
    }

    /**
     * Initialization of the latest DynamicAnimControl that takes place once all
     * its links are ready for dynamic mode: add pinning joints and splices.
     */
    private void initWhenReady() {
        assert dacs.peekLast().isReady();
        /*
         * Force a software skin update
         * before invoking DacLinks.findManagerForVertex().
         */
        Spatial spatial = dacs.peekLast().getSpatial();
        SkinningControl sControl = spatial.getControl(SkinningControl.class);
        sControl.render(renderManager, viewPort);

        RopeShape latestShape = shapes.peekLast();
        switch (latestShape) {
            case Cross:
                pinEnd(0);
                pinEnd(1);
                pinEnd(2);
                pinEnd(3);
                break;

            case Noose:
            case NooseSpliced:
                pinEnd(0);
                spliceEnds(1, 2);
                break;

            case Ring:
            case RingSpliced:
                spliceEnds(0, 1);
                break;

            case Slackline:
                pinEnd(0);
                pinEnd(1);
                break;

            default:
                throw new IllegalArgumentException(shapes.toString());
        }
    }

    /**
     * Generate the name for the indexed Joint in the indexed branch.
     *
     * @param branchIndex the index of the branch containing the Joint (&ge;0,
     * &lt;numBranches)
     * @param stepIndex the joint's index in the branch (&ge;0)
     * @return the name (not null, not empty)
     */
    private static String jointName(int branchIndex, int stepIndex) {
        assert branchIndex >= 0 : branchIndex;
        assert stepIndex >= 0 : stepIndex;

        String name = String.format("branch%d.bone%d", branchIndex, stepIndex);
        return name;
    }

    /**
     * Initialization performed immediately after parsing the command-line
     * arguments.
     *
     * @param showDialog when to show the JME settings dialog (not null)
     * @param title for the title bar of the app's window
     */
    private static void
            mainStartup(final ShowDialog showDialog, final String title) {
        RopeDemo application = new RopeDemo();

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
     * Generate an Armature composed of straight-line branches originating from
     * the root joint.
     *
     * @param branchToNumSteps the number of joints in each branch (each &ge;1)
     * @param branchToStepOffsets the parent-to-child offset in each branch
     * @return a new Armature in bind pose (not null)
     */
    private static Armature makeArmature(
            int[] branchToNumSteps, Vector3f[] branchToStepOffsets) {
        assert branchToNumSteps.length > 0;
        assert branchToNumSteps.length == branchToStepOffsets.length;
        int numJoints = 1;
        for (int numSteps : branchToNumSteps) {
            assert numSteps >= 1 : numSteps;
            numJoints += numSteps;
        }

        Joint[] joints = new Joint[numJoints];
        Joint root = new Joint("root joint");
        joints[0] = root;

        int nextJointIndex = 1;
        int numBranches = branchToNumSteps.length;
        for (int branchIndex = 0; branchIndex < numBranches; ++branchIndex) {
            Joint parent = root;
            Vector3f stepOffset = branchToStepOffsets[branchIndex];
            int numSteps = branchToNumSteps[branchIndex];
            for (int stepIndex = 0; stepIndex < numSteps; ++stepIndex) {
                String jointName = jointName(branchIndex, stepIndex);
                Joint joint = new Joint(jointName);
                joint.setLocalTranslation(stepOffset);

                joints[nextJointIndex] = joint;
                parent.addChild(joint);
                parent = joint;
                ++nextJointIndex;
            }
        }
        assert nextJointIndex == numJoints;

        Armature armature = new Armature(joints);
        armature.saveBindPose();

        return armature;
    }

    /**
     * Pin the endpoint of the indexed branch to its current location.
     *
     * @param branchIndex which branch to pin (&ge;0)
     */
    private void pinEnd(int branchIndex) {
        String[] capSpecs = capSpecs(branchIndex);

        DynamicAnimControl latestDac = dacs.peekLast();
        Vector3f sum = new Vector3f();
        PhysicsLink link
                = latestDac.findManagerForVertex(capSpecs[0], sum, null);

        Vector3f pivot = new Vector3f(); // world coordinates
        latestDac.findManagerForVertex(capSpecs[1], pivot, null);
        sum.addLocal(pivot);

        latestDac.findManagerForVertex(capSpecs[2], pivot, null);
        sum.addLocal(pivot);

        latestDac.pinToWorld(link, sum.divide(3f));
    }

    /**
     * Disable one single-ended IK constraint of the most recently added rope.
     */
    private static void pullAPin() {
        DynamicAnimControl latestDac = dacs.peekLast();
        if (latestDac != null) {
            IKJoint[] ikJoints = latestDac.listIKJoints();
            for (IKJoint ikJoint : ikJoints) {
                Constraint constraint = ikJoint.getPhysicsJoint();
                if (constraint.isEnabled() && constraint.countEnds() == 1) {
                    constraint.setEnabled(false);
                    break;
                }
            }
        }
    }

    /**
     * Triple-pin the ends of the indexed branches together.
     *
     * @param branchIndex1 (&ge;0)
     * @param branchIndex2 (&ge;0)
     */
    private void spliceEnds(int branchIndex1, int branchIndex2) {
        PhysicsLink linkA;
        PhysicsLink linkB;
        Vector3f pivotA = new Vector3f(); // local coordinates in rigid body
        Vector3f pivotB = new Vector3f();

        String[] capSpecsA = capSpecs(branchIndex1);
        String[] capSpecsB = capSpecs(branchIndex2);

        DynamicAnimControl latestDac = dacs.peekLast();
        linkA = latestDac.findManagerForVertex(capSpecsA[0], null, pivotA);
        linkB = latestDac.findManagerForVertex(capSpecsB[0], null, pivotB);
        latestDac.pinToSelf(linkA, linkB, pivotA, pivotB);

        linkA = latestDac.findManagerForVertex(capSpecsA[1], null, pivotA);
        linkB = latestDac.findManagerForVertex(capSpecsB[2], null, pivotB);
        latestDac.pinToSelf(linkA, linkB, pivotA, pivotB);

        linkA = latestDac.findManagerForVertex(capSpecsA[2], null, pivotA);
        linkB = latestDac.findManagerForVertex(capSpecsB[1], null, pivotB);
        latestDac.pinToSelf(linkA, linkB, pivotA, pivotB);
    }

    /**
     * Toggle mesh rendering of ropes on/off.
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
     * Toggle the skeleton visualizer on/off.
     */
    private static void toggleSkeleton() {
        boolean enabled = sv.isEnabled();
        sv.setEnabled(!enabled);
    }
}
