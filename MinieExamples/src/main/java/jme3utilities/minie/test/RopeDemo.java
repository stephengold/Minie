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

import com.jme3.animation.Bone;
import com.jme3.animation.Skeleton;
import com.jme3.animation.SkeletonControl;
import com.jme3.app.Application;
import com.jme3.app.StatsAppState;
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
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.joints.Constraint;
import com.jme3.bullet.objects.PhysicsRigidBody;
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
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Box;
import com.jme3.shadow.DirectionalLightShadowRenderer;
import com.jme3.system.AppSettings;
import java.util.ArrayDeque;
import java.util.BitSet;
import java.util.Deque;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Misc;
import jme3utilities.MyAsset;
import jme3utilities.MyCamera;
import jme3utilities.MySkeleton;
import jme3utilities.NameGenerator;
import jme3utilities.debug.SkeletonVisualizer;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.noise.Generator;
import jme3utilities.minie.DumpFlags;
import jme3utilities.minie.FilterAll;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.minie.test.mesh.TubeTreeMesh;
import jme3utilities.ui.ActionApplication;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.InputMode;

/**
 * Simulate ropes using DynamicAnimControl. TODO use the new animation system
 * <p>
 * Seen in the February 2019 demo video:
 * https://www.youtube.com/watch?v=7PYDAyB5RCE
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class RopeDemo extends ActionApplication {
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
    final private static float ropeRadius = 0.03f;
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
     * link configuration for leaf bones (shunken hull shape)
     */
    final private static LinkConfig leafConfig = new LinkConfig(linkMass,
            MassHeuristic.Mass, ShapeHeuristic.VertexHull,
            new Vector3f(0.84f, 0.84f, 0.84f), CenterHeuristic.Mean);
    /**
     * link configuration for non-leaf bones (stretched capsule shape)
     */
    final private static LinkConfig ropeConfig = new LinkConfig(linkMass,
            MassHeuristic.Mass, ShapeHeuristic.TwoSphere,
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
    private boolean dacReadyInitDone = false;
    /**
     * AppState to manage the PhysicsSpace
     */
    final private BulletAppState bulletAppState = new BulletAppState();
    /**
     * physics controls for the ropes, in order of creation
     */
    final private Deque<DynamicAnimControl> dacs = new ArrayDeque<>(12);
    /**
     * shapes of the ropes, in order of creation
     */
    final private Deque<RopeShape> shapes = new ArrayDeque<>(12);
    /**
     * filter to control visualization of axis-aligned bounding boxes
     */
    private FilterAll bbFilter;
    /**
     * enhanced pseudo-random generator
     */
    final private Generator random = new Generator();
    /**
     * material to visualize the platform box
     */
    private Material boxMaterial;
    /**
     * material to visualize ropes
     */
    private Material ropeMaterial;
    /**
     * generate names for rope geometries
     */
    final private NameGenerator geometryNamer = new NameGenerator();
    /**
     * GUI node for displaying hotkey help/hints
     */
    private Node helpNode;
    /**
     * parent for rope geometries
     */
    final private Node ropesNode = new Node("ropes");
    /**
     * dump debugging information to System.out
     */
    final private PhysicsDumper dumper = new PhysicsDumper();
    /**
     * space for physics simulation
     */
    private PhysicsSpace physicsSpace;
    /**
     * visualizer for the skeleton of the most recently added rope
     */
    private SkeletonVisualizer sv;
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the RopeDemo application.
     *
     * @param ignored array of command-line arguments (not null)
     */
    public static void main(String[] ignored) {
        /*
         * Mute the chatty loggers in certain packages.
         */
        Misc.setLoggingLevels(Level.WARNING);

        Application application = new RopeDemo();
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

        ColorRGBA bgColor = new ColorRGBA(0.2f, 0.2f, 1f, 1f);
        viewPort.setBackgroundColor(bgColor);

        addLighting();
        stateManager.getState(StatsAppState.class).toggleStats();
        addBox();
        addSkeleton();

        rootNode.attachChild(ropesNode);
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

        dim.bind("delete", KeyInput.KEY_BACK);
        dim.bind("delete", KeyInput.KEY_DELETE);

        dim.bind("dump physicsSpace", KeyInput.KEY_O);
        dim.bind("dump scenes", KeyInput.KEY_P);
        dim.bind("go limp", KeyInput.KEY_SPACE);
        dim.bind("pull a pin", KeyInput.KEY_X);
        dim.bind("save", KeyInput.KEY_SEMICOLON);

        dim.bind("signal " + CameraInput.FLYCAM_LOWER, KeyInput.KEY_DOWN);
        dim.bind("signal " + CameraInput.FLYCAM_RISE, KeyInput.KEY_UP);
        dim.bind("signal orbitLeft", KeyInput.KEY_LEFT);
        dim.bind("signal orbitRight", KeyInput.KEY_RIGHT);

        dim.bind("toggle axes", KeyInput.KEY_SEMICOLON);
        dim.bind("toggle boxes", KeyInput.KEY_APOSTROPHE);
        dim.bind("toggle help", KeyInput.KEY_H);
        dim.bind("toggle meshes", KeyInput.KEY_M);
        dim.bind("toggle pause", KeyInput.KEY_PERIOD);
        dim.bind("toggle physics debug", KeyInput.KEY_SLASH);
        dim.bind("toggle skeleton", KeyInput.KEY_V);

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
                case "delete":
                    delete();
                    return;

                case "dump physicsSpace":
                    dumper.dump(physicsSpace);
                    return;

                case "dump scenes":
                    dumper.dump(renderManager);
                    return;

                case "toggle axes":
                    toggleAxes();
                    return;

                case "toggle boxes":
                    toggleBoxes();
                    return;

                case "go limp":
                    goLimp();
                    return;

                case "pull a pin":
                    pullAPin();
                    return;

                case "toggle help":
                    toggleHelp();
                    return;

                case "toggle meshes":
                    toggleMeshes();
                    return;

                case "toggle pause":
                    togglePause();
                    return;

                case "toggle physics debug":
                    togglePhysicsDebug();
                    return;

                case "toggle skeleton":
                    toggleSkeleton();
                    return;
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
     * Add a large static box to the scene, to serve as a platform.
     */
    private void addBox() {
        float halfExtent = 50f; // mesh units
        Mesh mesh = new Box(halfExtent, halfExtent, halfExtent);
        Geometry geometry = new Geometry("box", mesh);
        rootNode.attachChild(geometry);

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
     * Add lighting and shadows to the scene.
     */
    private void addLighting() {
        ColorRGBA ambientColor = new ColorRGBA(0.2f, 0.2f, 0.2f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        rootNode.addLight(ambient);

        Vector3f direction = new Vector3f(1f, -2f, -2f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction);
        rootNode.addLight(sun);

        DirectionalLightShadowRenderer dlsr
                = new DirectionalLightShadowRenderer(assetManager, 8_192, 3);
        dlsr.setLight(sun);
        dlsr.setShadowIntensity(0.6f);
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
     *
     * @return the new DAC (added to the spatial and physics space, in kinematic
     * mode)
     */
    private void addRopeCross() {
        /*
         * Generate a cross-shaped skeleton.
         */
        int[] stepCounts = {5, 5, 5, 5};
        Vector3f[] stepOffsets = {
            new Vector3f(stepLength, 0f, 0f),
            new Vector3f(-stepLength, 0f, 0f),
            new Vector3f(0f, 0f, stepLength),
            new Vector3f(0f, 0f, -stepLength)
        };
        Skeleton skeleton = makeSkeleton(stepCounts, stepOffsets);

        TubeTreeMesh ropeMesh = new TubeTreeMesh(skeleton, ropeRadius,
                0f, loopsPerSegment, samplesPerLoop);

        String geometryName = geometryNamer.unique("rope cross");
        Geometry geometry = new Geometry(geometryName, ropeMesh);

        addRopeSpatial(skeleton, geometry);
        /*
         * Curl all branches clockwise to introduce some slack.
         */
        curlBranch(skeleton, 0, Vector3f.UNIT_Y, 0.2f);
        curlBranch(skeleton, 1, Vector3f.UNIT_Y, 0.2f);
        curlBranch(skeleton, 2, Vector3f.UNIT_Y, 0.2f);
        curlBranch(skeleton, 3, Vector3f.UNIT_Y, 0.2f);
    }

    /**
     * Add a noose to the scene.
     *
     * @param kinematicSplice true to pre-splice the ends kinematically, false
     * to leave it Y-shaped until dynamic mode is set
     * @return the new DAC (added to the spatial and physics space, in kinematic
     * mode)
     */
    private void addRopeNoose(boolean kinematicSplice) {
        /*
         * Generate a Y-shaped skeleton. Branch0 forms the stem of the Y.
         */
        int[] stepCounts = {6, 6, 6};
        float dx = stepLength * FastMath.sqrt(0.5f);
        Vector3f[] stepOffsets = {
            new Vector3f(stepLength, 0f, 0f),
            new Vector3f(-dx, 0f, -dx),
            new Vector3f(-dx, 0f, dx)
        };
        Skeleton skeleton = makeSkeleton(stepCounts, stepOffsets);

        TubeTreeMesh ropeMesh = new TubeTreeMesh(skeleton, ropeRadius,
                0f, loopsPerSegment, samplesPerLoop);
        String geometryName = geometryNamer.unique("noose");
        Geometry geometry = new Geometry(geometryName, ropeMesh);

        addRopeSpatial(skeleton, geometry);

        if (kinematicSplice) {
            /*
             * Curl branch1 and branch2 toward one another, 90 degrees each.
             */
            curlBranch(skeleton, 1, Vector3f.UNIT_Y, FastMath.HALF_PI);
            curlBranch(skeleton, 2, Vector3f.UNIT_Y, -FastMath.HALF_PI);
        }
    }

    /**
     * Add a rope ring to the scene.
     *
     * @param kinematicSplice true to pre-splice the ends kinematically, false
     * to leave it linear until dynamic mode is set
     * @return the new DAC (added to the spatial and physics space, in kinematic
     * mode)
     */
    private void addRopeRing(boolean kinematicSplice) {
        /*
         * Generate a double-ended straight-line skeleton.
         */
        int[] stepCounts = {8, 8};
        Vector3f[] stepOffsets = {
            new Vector3f(stepLength, 0f, 0f),
            new Vector3f(-stepLength, 0f, 0f)
        };
        Skeleton skeleton = makeSkeleton(stepCounts, stepOffsets);

        TubeTreeMesh ropeMesh = new TubeTreeMesh(skeleton, ropeRadius,
                0f, loopsPerSegment, samplesPerLoop);

        String geometryName = geometryNamer.unique("rope ring");
        Geometry geometry = new Geometry(geometryName, ropeMesh);

        addRopeSpatial(skeleton, geometry);

        if (kinematicSplice) {
            /*
             * Curl branch0 and branch1 toward one another, 180 degrees each.
             */
            curlBranch(skeleton, 0, Vector3f.UNIT_Y, FastMath.PI);
            curlBranch(skeleton, 1, Vector3f.UNIT_Y, -FastMath.PI);
        }
    }

    /**
     * Add a plain rope to the scene. The rope begins in the X-Z plane.
     *
     * @return the new DAC (added to the spatial and physics space, in kinematic
     * mode)
     */
    private void addRopeSlackline() {
        /*
         * Generate a double-ended straight-line skeleton.
         */
        int[] stepCounts = {8, 8};
        Vector3f[] stepOffsets = {
            new Vector3f(0f, 0f, stepLength),
            new Vector3f(0f, 0f, -stepLength)
        };
        Skeleton skeleton = makeSkeleton(stepCounts, stepOffsets);

        float leafOvershoot = 0.8f * ropeRadius;
        TubeTreeMesh ropeMesh = new TubeTreeMesh(skeleton, ropeRadius,
                leafOvershoot, loopsPerSegment, samplesPerLoop);
        String geometryName = geometryNamer.unique("slackline");
        Geometry geometry = new Geometry(geometryName, ropeMesh);

        addRopeSpatial(skeleton, geometry);
        /*
         * Curl both branches clockwise to introduce some slack.
         */
        curlBranch(skeleton, 0, Vector3f.UNIT_Y, 0.2f);
        curlBranch(skeleton, 1, Vector3f.UNIT_Y, 0.2f);
    }

    /**
     * Code for adding a rope that's shared by all shapes.
     *
     * @param skeleton (not null)
     * @param spatial (not null)
     */
    private void addRopeSpatial(Skeleton skeleton, Spatial spatial) {
        /*
         * Set a random elevation and Y-axis rotation.
         */
        float elevation = 1f + random.nextFloat();
        float rotationAngle = FastMath.TWO_PI * random.nextFloat();
        spatial.move(0f, elevation, 0f);
        spatial.rotate(0f, rotationAngle, 0f);

        spatial.setCullHint(Spatial.CullHint.Never);
        spatial.setMaterial(ropeMaterial);
        spatial.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);

        SkeletonControl skeletonControl = new SkeletonControl(skeleton);
        spatial.addControl(skeletonControl);
        skeletonControl.setHardwareSkinningPreferred(false);
        sv.setSubject(skeletonControl);

        assert stepLength < 2f * minCurlRadius : stepLength;
        float maxAngle = FastMath.acos(stepLength / (2f * minCurlRadius));
        RangeOfMotion rom = new RangeOfMotion(maxAngle);

        DynamicAnimControl dac = new DynamicAnimControl();
        for (Bone bone : MySkeleton.preOrderBones(skeleton)) {
            if (bone.getParent() != null) {
                String boneName = bone.getName();
                boolean isLeafBone = bone.getChildren().isEmpty();
                LinkConfig linkConfig = isLeafBone ? leafConfig : ropeConfig;
                dac.link(boneName, linkConfig, rom);
            }
        }
        dac.setConfig(DacConfiguration.torsoName, ropeConfig);
        dac.setDamping(0.9f);

        spatial.addControl(dac);
        dac.setPhysicsSpace(physicsSpace);
        ropesNode.attachChild(spatial);

        for (PhysicsRigidBody body : dac.listRigidBodies()) {
            //body.setDebugMeshResolution(DebugShapeFactory.highResolution);
            body.setFriction(9e9f);
        }

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
     * Generate the name for the indexed bone in the indexed branch.
     *
     * @param branchIndex the index of the branch containing the bone (&ge;0,
     * &lt;numBranches)
     * @param stepIndex the bone's index in the branch (&ge;0)
     * @return the name (not null, not empty)
     */
    private String boneName(int branchIndex, int stepIndex) {
        assert branchIndex >= 0 : branchIndex;
        assert stepIndex >= 0 : stepIndex;

        String name = String.format("branch%d.bone%d", branchIndex, stepIndex);
        return name;
    }

    /**
     * Choose 3 distinct vertices in the end cap of the indexed branch.
     *
     * @param branchIndex which branch (&ge;0)
     * @return a new array of 3 vertex specifiers
     */
    private String[] capSpecs(int branchIndex) {
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
            String msg = String.format("samplesPerLoop = %d", samplesPerLoop);
            throw new IllegalStateException(msg);
        }

        return result;
    }

    /**
     * Find the index of the first vertex in the end cap of the indexed branch.
     *
     * @param branchIndex which branch(&ge;0)
     * @return the vertex index (&ge;0)
     */
    private int capVertex0(int branchIndex) {
        assert branchIndex >= 0 : branchIndex;

        DynamicAnimControl latestDac = dacs.peekLast();
        Skeleton skeleton = latestDac.getSkeleton();
        int lastStep = countSteps(skeleton, branchIndex) - 1;
        String endBoneName = boneName(branchIndex, lastStep);

        Geometry geometry = (Geometry) latestDac.getSpatial();
        TubeTreeMesh mesh = (TubeTreeMesh) geometry.getMesh();
        BitSet bitSet = mesh.listCapVertices(endBoneName);
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
        float far = 20f;
        MyCamera.setNearFar(cam, near, far);

        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(2f);

        cam.setLocation(new Vector3f(0f, 2.5f, 4.2f));
        cam.setRotation(new Quaternion(0f, 0.982f, -0.188f, 0f));

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
        dumper.setEnabled(DumpFlags.Transforms, true);
    }

    /**
     * Configure materials during startup.
     */
    private void configureMaterials() {
        ColorRGBA darkGreen = new ColorRGBA().setAsSrgb(0.1f, 0.4f, 0.1f, 1f);
        boxMaterial = MyAsset.createShadedMaterial(assetManager, darkGreen);

        ColorRGBA taupe = new ColorRGBA().setAsSrgb(0.6f, 0.5f, 0.4f, 1f);
        ropeMaterial = MyAsset.createShadedMaterial(assetManager, taupe);
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        CollisionShape.setDefaultMargin(ropeRadius / 10f);
        stateManager.attach(bulletAppState);

        physicsSpace = bulletAppState.getPhysicsSpace();
        physicsSpace.setSolverNumIterations(8);
    }

    /**
     * Count the steps in the indexed branch of the specified skeleton.
     *
     * @param skeleton which skeleton (not null)
     * @param branchIndex the index of the branch (&ge;0)
     * @return the count (&ge;0)
     */
    private int countSteps(Skeleton skeleton, int branchIndex) {
        int numSteps = 0;

        while (true) {
            String boneName = boneName(branchIndex, numSteps);
            if (skeleton.getBone(boneName) == null) {
                break;
            } else {
                ++numSteps;
            }
        }

        return numSteps;
    }

    /**
     * Kinematically curl a branch of the specified skeleton around the
     * specified axis.
     *
     * @param skeleton the skeleton to pose (not null)
     * @param branchIndex (&ge;0)
     * @param axis (in each bone's local coordinates, not null, not zero)
     * @param totalAngle the angle between the first and last steps (in radians,
     * may be negative)
     */
    private void curlBranch(Skeleton skeleton, int branchIndex, Vector3f axis,
            float totalAngle) {
        assert !MyVector3f.isZero(axis);
        /*
         * Count the steps in this branch.
         */
        int numSteps = countSteps(skeleton, branchIndex);
        assert numSteps > 1 : numSteps;
        /*
         * Calculate local rotation.
         */
        float turnAngle = totalAngle / (numSteps - 1);
        Quaternion turn = new Quaternion().fromAngleAxis(turnAngle, axis);
        /*
         * Apply the rotation to each bone.
         */
        for (int stepIndex = 0; stepIndex < numSteps; ++stepIndex) {
            String boneName = boneName(branchIndex, stepIndex);
            Bone bone = skeleton.getBone(boneName);
            bone.setUserControl(true);
            bone.setLocalRotation(turn);
        }
    }

    /**
     * Delete the most recently added rope.
     */
    private void delete() {
        DynamicAnimControl latestDac = dacs.peekLast();
        if (latestDac != null) {
            latestDac.setEnabled(false);
            latestDac.setPhysicsSpace(null);
            Spatial spatial = latestDac.getSpatial();
            spatial.removeControl(latestDac);
            ropesNode.detachChild(spatial);
            sv.setSubject(null);

            dacs.removeLast();
            shapes.removeLast();

            latestDac = dacs.peekLast();
            if (latestDac != null) {
                spatial = latestDac.getSpatial();
                SkeletonControl skeletonControl
                        = spatial.getControl(SkeletonControl.class);
                sv.setSubject(skeletonControl);
            }
        }
    }

    /**
     * Put the most recently added rope into dynamic mode, with gravity.
     */
    private void goLimp() {
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
        SkeletonControl skeletonControl
                = spatial.getControl(SkeletonControl.class);
        skeletonControl.render(renderManager, viewPort);

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
     * Generate a Skeleton composed of straight-line branches originating from
     * the root bone.
     *
     * @param branchToNumSteps the number of bones in each branch (each &ge;1)
     * @param branchToStepOffsets the parent-to-child offset in each branch
     * @return a new Skeleton in bind pose (not null)
     */
    private Skeleton makeSkeleton(int[] branchToNumSteps,
            Vector3f[] branchToStepOffsets) {
        assert branchToNumSteps.length > 0;
        assert branchToNumSteps.length == branchToStepOffsets.length;
        int numBones = 1;
        for (int numSteps : branchToNumSteps) {
            assert numSteps >= 1 : numSteps;
            numBones += numSteps;
        }

        Bone[] bones = new Bone[numBones];
        Bone root = new Bone("root bone");
        bones[0] = root;

        int nextBoneIndex = 1;
        int numBranches = branchToNumSteps.length;
        for (int branchIndex = 0; branchIndex < numBranches; ++branchIndex) {
            Bone parent = root;
            Vector3f stepOffset = branchToStepOffsets[branchIndex];
            int numSteps = branchToNumSteps[branchIndex];
            for (int stepIndex = 0; stepIndex < numSteps; ++stepIndex) {
                String boneName = boneName(branchIndex, stepIndex);
                Bone bone = new Bone(boneName);
                bone.setUserControl(true);
                bone.setLocalTranslation(stepOffset);

                bones[nextBoneIndex] = bone;
                parent.addChild(bone);
                parent = bone;
                ++nextBoneIndex;
            }
        }
        assert nextBoneIndex == numBones;

        Skeleton skeleton = new Skeleton(bones);
        skeleton.setBindingPose();

        return skeleton;
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
    private void pullAPin() {
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
        PhysicsLink linkA, linkB;
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
     * Toggle mesh rendering of ropes on/off.
     */
    private void toggleMeshes() {
        Spatial.CullHint hint = ropesNode.getLocalCullHint();
        if (hint == Spatial.CullHint.Inherit
                || hint == Spatial.CullHint.Never) {
            hint = Spatial.CullHint.Always;
        } else if (hint == Spatial.CullHint.Always) {
            hint = Spatial.CullHint.Never;
        }
        ropesNode.setCullHint(hint);
    }

    /**
     * Toggle the animation and physics simulation: paused/running.
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
     * Toggle the skeleton visualizer on/off.
     */
    private void toggleSkeleton() {
        boolean enabled = sv.isEnabled();
        sv.setEnabled(!enabled);
    }
}
