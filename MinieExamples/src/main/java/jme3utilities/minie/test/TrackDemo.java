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

import com.jme3.anim.SkinningControl;
import com.jme3.anim.util.AnimMigrationUtils;
import com.jme3.app.Application;
import com.jme3.app.StatsAppState;
import com.jme3.app.state.AppState;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.animation.CenterHeuristic;
import com.jme3.bullet.animation.DynamicAnimControl;
import com.jme3.bullet.animation.IKJoint;
import com.jme3.bullet.animation.LinkConfig;
import com.jme3.bullet.animation.MassHeuristic;
import com.jme3.bullet.animation.PhysicsLink;
import com.jme3.bullet.animation.RagUtils;
import com.jme3.bullet.animation.ShapeHeuristic;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.joints.Constraint;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.input.CameraInput;
import com.jme3.input.KeyInput;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Ray;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.debug.Grid;
import com.jme3.shadow.DirectionalLightShadowRenderer;
import com.jme3.system.AppSettings;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.InfluenceUtil;
import jme3utilities.MyAsset;
import jme3utilities.MyCamera;
import jme3utilities.MySpatial;
import jme3utilities.MyString;
import jme3utilities.debug.SkeletonVisualizer;
import jme3utilities.math.MyVector3f;
import jme3utilities.minie.DumpFlags;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.minie.test.common.PhysicsDemo;
import jme3utilities.minie.test.controllers.TrackController;
import jme3utilities.minie.test.tunings.BaseMeshControl;
import jme3utilities.minie.test.tunings.Binocular;
import jme3utilities.minie.test.tunings.Face;
import jme3utilities.minie.test.tunings.JaimeControl;
import jme3utilities.minie.test.tunings.MhGameControl;
import jme3utilities.minie.test.tunings.NinjaControl;
import jme3utilities.minie.test.tunings.OtoControl;
import jme3utilities.minie.test.tunings.PuppetControl;
import jme3utilities.minie.test.tunings.SinbadControl;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.InputMode;
import jme3utilities.ui.Signals;

/**
 * Demo/testbed for TrackController inverse kinematics.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TrackDemo extends PhysicsDemo {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(TrackDemo.class.getName());
    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName
            = TrackDemo.class.getSimpleName();
    // *************************************************************************
    // fields

    /**
     * true once {@link #initWhenReady()} has been invoked for the latest model
     */
    private static boolean dacReadyInitDone = false;
    /**
     * AppState to manage the PhysicsSpace
     */
    private static BulletAppState bulletAppState;
    /**
     * inverse kinematics joint for the finger/sword tip
     */
    private static Constraint tipJoint = null;
    /**
     * Control being tested
     */
    private static DynamicAnimControl dac;

    private static int chainLength;
    /**
     * root node of the C-G model on which the Control is being tested
     */
    private static Node cgModel;
    private static PhysicsRigidBody targetBody;
    /**
     * visualizer for the skeleton of the C-G model
     */
    private static SkeletonVisualizer sv;
    /**
     * vertex specifier for finger/sword tip
     */
    private static String tipSpec;

    private static TrackController leftWatch = null;
    private static TrackController rightWatch = null;
    private static TrackController watch = null;
    /**
     * locations of grid corners in world coordinates
     */
    private static Vector3f gridBottomLeft;
    private static Vector3f gridBottomRight;
    private static Vector3f gridTopLeft;
    private static Vector3f gridTopRight;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the TrackDemo application.
     */
    public TrackDemo() { // made explicit to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the TrackDemo application.
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

        Application application = new TrackDemo();
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

        // Hide the render-statistics overlay.
        stateManager.getState(StatsAppState.class).toggleStats();

        float length = 0.8f;
        attachWorldAxes(length);

        float halfExtent = 50f;
        float topY = 0f;
        attachCubePlatform(halfExtent, topY);

        //addModel("Sinbad");
        //addModel("MhGame");
        addModel("SinbadWith1Sword");

        // Add a target rigid body, to be moved by dragging RMB.
        CollisionShape shape = new SphereCollisionShape(0.02f);
        targetBody = new PhysicsRigidBody(shape);
        targetBody.setKinematic(true);
        addCollisionObject(targetBody);

        Vector3f targetStartLocation
                = MyVector3f.midpoint(gridTopLeft, gridBottomRight, null);
        targetBody.setPhysicsLocation(targetStartLocation);
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
        return 0.5f;
    }

    /**
     * Add application-specific hotkey bindings (and override existing ones, if
     * necessary).
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        dim.bind(asDumpSpace, KeyInput.KEY_O);
        dim.bind(asDumpScenes, KeyInput.KEY_P);

        dim.bind("load BaseMesh", KeyInput.KEY_F11);
        dim.bind("load Jaime", KeyInput.KEY_F2);
        dim.bind("load MhGame", KeyInput.KEY_F9);
        dim.bind("load Ninja", KeyInput.KEY_F7);
        dim.bind("load Oto", KeyInput.KEY_F6);
        dim.bind("load Puppet", KeyInput.KEY_F8);
        dim.bind("load Sinbad", KeyInput.KEY_F1);
        dim.bind("load SinbadWith1Sword", KeyInput.KEY_F10);
        dim.bind("load SinbadWithSwords", KeyInput.KEY_F4);

        dim.bindSignal(CameraInput.FLYCAM_LOWER, KeyInput.KEY_DOWN);
        dim.bindSignal(CameraInput.FLYCAM_RISE, KeyInput.KEY_UP);
        dim.bindSignal("orbitLeft", KeyInput.KEY_LEFT);
        dim.bindSignal("orbitRight", KeyInput.KEY_RIGHT);
        dim.bind("signal track", "RMB");

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
                case "toggle meshes":
                    toggleMeshes();
                    return;
                case "toggle skeleton":
                    toggleSkeleton();
                    return;
                default:
            }

            String[] words = actionString.split(" ");
            if (words.length == 2 && "load".equals(words[0])) {
                addModel(words[1]);
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
        if (signals.test("track")) {
            track();
        }

        if (!dacReadyInitDone && dac.isReady()) {
            initWhenReady();
            dacReadyInitDone = true;
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Add a 10x10 X-Z square grid to the scene with the specified bottom-left
     * and bottom-right corners, replacing any pre-existing grid.
     */
    private void addGrid() {
        assert gridBottomLeft.x == gridBottomRight.x;
        assert gridBottomLeft.y == gridBottomRight.y;
        assert gridBottomLeft.z < gridBottomRight.z;

        Spatial old = MySpatial.findChild(rootNode, "grid");
        if (old != null) {
            old.removeFromParent();
        }

        int numColumns = 10;
        int numRows = 10;
        float gridSpacing = (gridBottomRight.z - gridBottomLeft.z) / numColumns;
        Mesh mesh = new Grid(numColumns + 1, numRows + 1, gridSpacing);
        Geometry geometry = new Geometry("grid", mesh);
        rootNode.attachChild(geometry);
        geometry.rotate(0f, 0f, FastMath.HALF_PI);

        ColorRGBA gridColor = new ColorRGBA(0f, 0f, 0f, 1f);
        Material mat = MyAsset.createWireframeMaterial(assetManager, gridColor);
        geometry.setMaterial(mat);
        /*
         * Translate the origin of the grid, which
         * becomes its bottom-left corner.
         */
        geometry.move(gridBottomLeft);
        float gridX = gridBottomLeft.x;
        float topY = gridBottomLeft.y + gridSpacing * numRows;
        gridTopLeft = new Vector3f(gridX, topY, gridBottomLeft.z);
        gridTopRight = new Vector3f(gridX, topY, gridBottomRight.z);
    }

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
     * Add an animated model to the scene, removing any previously added model.
     *
     * @param modelName the name of the model to add (not null, not empty)
     */
    private void addModel(String modelName) {
        if (cgModel != null) {
            dac.getSpatial().removeControl(dac);
            rootNode.detachChild(cgModel);
            rootNode.removeControl(sv);
            dacReadyInitDone = false;
        }
        chainLength = 6; // constant?

        switch (modelName) {
            case "BaseMesh":
                loadBaseMesh();
                break;
            case "Jaime":
                loadJaime();
                break;
            case "MhGame":
                loadMhGame();
                break;
            case "Ninja":
                loadNinja();
                break;
            case "Oto":
                loadOto();
                break;
            case "Puppet":
                loadPuppet();
                break;
            case "Sinbad":
                loadSinbad();
                break;
            case "SinbadWith1Sword":
                loadSinbadWith1Sword();
                break;
            case "SinbadWithSwords":
                loadSinbadWithSwords();
                break;
            default:
                throw new IllegalArgumentException(modelName);
        }

        List<Spatial> list = MySpatial.listSpatials(cgModel);
        for (Spatial spatial : list) {
            spatial.setShadowMode(RenderQueue.ShadowMode.Cast);
        }
        cgModel.setCullHint(Spatial.CullHint.Never);

        SkinningControl sc = (SkinningControl) RagUtils.findSControl(cgModel);
        sc.getArmature().applyBindPose(); // see JME issue #1395

        rootNode.attachChild(cgModel);
        setCgmHeight(cgModel, 2f);
        centerCgm(cgModel);

        Spatial controlledSpatial = sc.getSpatial();
        controlledSpatial.addControl(dac);
        PhysicsSpace physicsSpace = getPhysicsSpace();
        dac.setPhysicsSpace(physicsSpace);

        sv = new SkeletonVisualizer(assetManager, sc);
        sv.setLineColor(ColorRGBA.Yellow);
        InfluenceUtil.hideNonInfluencers(sv, sc);
        rootNode.addControl(sv);

        addGrid();
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(4f);
        flyCam.setZoomSpeed(4f);

        cam.setLocation(new Vector3f(-4.3f, 2.1f, -0.2f));
        cam.setRotation(new Quaternion(0.0643f, 0.69194f, -0.0456f, 0.71765f));

        AppState orbitState
                = new CameraOrbitAppState(cam, "orbitLeft", "orbitRight");
        stateManager.attach(orbitState);
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        CollisionShape.setDefaultMargin(0.005f); // 5-mm margin

        bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);

        PhysicsSpace physicsSpace = getPhysicsSpace();
        physicsSpace.setAccuracy(1f / 30); // 33.33-msec timestep
        physicsSpace.getSolverInfo().setNumIterations(15);
    }

    /**
     * Initialization that takes place once all links are ready for dynamic
     * mode.
     */
    private static void initWhenReady() {
        // Touch the tip vertex to the target.
        Vector3f pivot = new Vector3f();
        PhysicsLink tipLink = dac.findManagerForVertex(tipSpec, null, pivot);
        dac.setMass(tipLink, 100f);
        dac.setDynamicChain(tipLink, chainLength, Vector3f.ZERO, false);
        IKJoint ikJoint
                = dac.moveToBody(tipLink, pivot, targetBody, Vector3f.ZERO);
        tipJoint = ikJoint.getPhysicsJoint();
        tipJoint.setEnabled(false);

        // The face and neck track the target.
        Face face = (Face) dac;
        Vector3f faceDirection = face.faceDirection(null);
        String noseSpec = face.faceCenterSpec();
        PhysicsLink noseLink = dac.findManagerForVertex(noseSpec, null, pivot);
        noseLink.setDynamic(Vector3f.ZERO);
        watch = new TrackController(noseLink, pivot, faceDirection, targetBody);
        noseLink.addIKController(watch);
        watch.setDeltaGainFactor(1.2f);
        watch.setEnabled(false);
        watch.setErrorGainFactor(0.3f);

        // If the model's eyes are animated, each eye also tracks the target.
        if (dac instanceof Binocular) {
            Binocular binocular = (Binocular) dac;

            // left eye
            String spec = binocular.leftPupilSpec();
            PhysicsLink link = dac.findManagerForVertex(spec, null, pivot);
            link.setDynamic(Vector3f.ZERO);
            Vector3f lookDirection = binocular.leftEyeLookDirection(null);
            leftWatch = new TrackController(
                    link, pivot, lookDirection, targetBody);
            link.addIKController(leftWatch);
            leftWatch.setDeltaGainFactor(4f);
            leftWatch.setEnabled(false);
            leftWatch.setErrorGainFactor(1f);

            // right eye
            spec = binocular.rightPupilSpec();
            link = dac.findManagerForVertex(spec, null, pivot);
            link.setDynamic(Vector3f.ZERO);
            binocular.leftEyeLookDirection(lookDirection);
            rightWatch = new TrackController(
                    link, pivot, lookDirection, targetBody);
            link.addIKController(rightWatch);
            rightWatch.setDeltaGainFactor(4f);
            rightWatch.setEnabled(false);
            rightWatch.setErrorGainFactor(1f);
        }
    }

    /**
     * Load the BaseMesh model.
     */
    private void loadBaseMesh() {
        cgModel = (Node) assetManager.loadModel("Models/BaseMesh/BaseMesh.j3o");
        cgModel.rotate(0f, -1.6f, 0f);

        dac = new BaseMeshControl();
        tipSpec = "4914/BaseMesh_011"; // tip of right index finger
    }

    /**
     * Load the Jaime model.
     */
    private void loadJaime() {
        cgModel = (Node) assetManager.loadModel("Models/Jaime/Jaime-new.j3o");
        Geometry g = (Geometry) cgModel.getChild(0);
        RenderState rs = g.getMaterial().getAdditionalRenderState();
        rs.setFaceCullMode(RenderState.FaceCullMode.Off);
        cgModel.rotate(0f, -1.6f, 0f);

        dac = new JaimeControl();
        tipSpec = "2704/JaimeGeom-geom-1"; // tip of right index finger
    }

    /**
     * Load the MhGame model.
     */
    private void loadMhGame() {
        cgModel = (Node) assetManager.loadModel("Models/MhGame/MhGame.j3o");
        cgModel.rotate(0f, -1.6f, 0f);

        dac = new MhGameControl();
        tipSpec = "5239/male_generic"; // tip of right index finger
    }

    /**
     * Load the Ninja model.
     */
    private void loadNinja() {
        cgModel = (Node) assetManager.loadModel("Models/Ninja/Ninja.j3o");
        cgModel.rotate(0f, 1.6f, 0f);

        dac = new NinjaControl();
        tipSpec = "55/Ninja-geom-2"; // tip of katana blade
    }

    /**
     * Load the Oto model.
     */
    private void loadOto() {
        cgModel = (Node) assetManager.loadModel("Models/Oto/Oto.j3o");
        cgModel.rotate(0f, -1.6f, 0f);

        dac = new OtoControl();
        tipSpec = "3236/Oto-geom-1"; // right knuckle
    }

    /**
     * Load the Puppet model.
     */
    private void loadPuppet() {
        cgModel = (Node) assetManager.loadModel("Models/Puppet/Puppet.j3o");
        AnimMigrationUtils.migrate(cgModel);
        cgModel.rotate(0f, -1.6f, 0f);

        dac = new PuppetControl();
        tipSpec = "3185/Mesh.009_0"; // tip of right index finger
    }

    /**
     * Load the Sinbad model without attachments.
     */
    private void loadSinbad() {
        cgModel = (Node) assetManager.loadModel("Models/Sinbad/Sinbad.j3o");
        cgModel.rotate(0f, -1.6f, 0f);

        dac = new SinbadControl();
        tipSpec = "223/Sinbad-geom-2"; // tip of right index finger

        gridBottomLeft.set(-1f, 0.5f, -1f);
        gridBottomRight.set(-1f, 0.5f, 1f);
    }

    /**
     * Load the Sinbad model with an attached sword.
     */
    private void loadSinbadWith1Sword() {
        cgModel = (Node) assetManager.loadModel("Models/Sinbad/Sinbad.j3o");
        cgModel.rotate(0f, -1.6f, 0f);

        Node sword = (Node) assetManager.loadModel("Models/Sinbad/Sword.j3o");
        List<Spatial> list = MySpatial.listSpatials(sword);
        for (Spatial spatial : list) {
            spatial.setShadowMode(RenderQueue.ShadowMode.Cast);
        }

        LinkConfig swordConfig = new LinkConfig(5f, MassHeuristic.Density,
                ShapeHeuristic.VertexHull, Vector3f.UNIT_XYZ,
                CenterHeuristic.AABB);
        dac = new SinbadControl();
        dac.attach("Handle.R", swordConfig, sword);

        tipSpec = "4/Sword-geom-3/Handle.R"; // tip of sword blade
        gridBottomLeft = new Vector3f(-1f, 0.5f, -1f);
        gridBottomRight = new Vector3f(-1f, 0.5f, 1f);
    }

    /**
     * Load the Sinbad model with 2 attached swords.
     */
    private void loadSinbadWithSwords() {
        cgModel = (Node) assetManager.loadModel("Models/Sinbad/Sinbad.j3o");
        cgModel.rotate(0f, -1.6f, 0f);

        Node sword = (Node) assetManager.loadModel("Models/Sinbad/Sword.j3o");
        List<Spatial> list = MySpatial.listSpatials(sword);
        for (Spatial spatial : list) {
            spatial.setShadowMode(RenderQueue.ShadowMode.Cast);
        }

        LinkConfig swordConfig = new LinkConfig(5f, MassHeuristic.Density,
                ShapeHeuristic.VertexHull, Vector3f.UNIT_XYZ,
                CenterHeuristic.AABB);
        dac = new SinbadControl();
        dac.attach("Handle.L", swordConfig, sword);
        dac.attach("Handle.R", swordConfig, sword);

        tipSpec = "4/Sword-geom-3/Handle.R"; // tip of right-hand sword blade
    }

    /**
     * Toggle mesh rendering on/off.
     */
    private static void toggleMeshes() {
        Spatial.CullHint hint = cgModel.getLocalCullHint();
        if (hint == Spatial.CullHint.Inherit
                || hint == Spatial.CullHint.Never) {
            hint = Spatial.CullHint.Always;
        } else if (hint == Spatial.CullHint.Always) {
            hint = Spatial.CullHint.Never;
        }
        cgModel.setCullHint(hint);
    }

    /**
     * Toggle the skeleton visualizer on/off.
     */
    private static void toggleSkeleton() {
        boolean enabled = sv.isEnabled();
        sv.setEnabled(!enabled);
    }

    /**
     * If the mouse is picking a grid location, enable all track controllers and
     * move the target there.
     */
    private void track() {
        Ray ray = MyCamera.mouseRay(cam, inputManager);

        Vector3f location = new Vector3f();
        boolean isTopLeft = ray.intersectWhere(
                gridTopLeft, gridTopRight, gridBottomLeft, location);
        boolean isBottomRight = ray.intersectWhere(
                gridBottomRight, gridTopRight, gridBottomLeft, location);
        if (isTopLeft || isBottomRight) {
            if (leftWatch != null) {
                leftWatch.setEnabled(true);
            }
            if (rightWatch != null) {
                rightWatch.setEnabled(true);
            }
            tipJoint.setEnabled(true);
            watch.setEnabled(true);

            targetBody.setPhysicsLocation(location);
        }
    }
}
