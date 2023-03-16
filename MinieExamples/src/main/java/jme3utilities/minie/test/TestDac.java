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

import com.jme3.anim.AnimClip;
import com.jme3.anim.AnimComposer;
import com.jme3.anim.SkinningControl;
import com.jme3.anim.util.AnimMigrationUtils;
import com.jme3.app.Application;
import com.jme3.app.StatsAppState;
import com.jme3.asset.AssetNotFoundException;
import com.jme3.asset.ModelKey;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.animation.AttachmentLink;
import com.jme3.bullet.animation.BoneLink;
import com.jme3.bullet.animation.CenterHeuristic;
import com.jme3.bullet.animation.DynamicAnimControl;
import com.jme3.bullet.animation.LinkConfig;
import com.jme3.bullet.animation.MassHeuristic;
import com.jme3.bullet.animation.RagUtils;
import com.jme3.bullet.animation.ShapeHeuristic;
import com.jme3.bullet.animation.TorsoLink;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.export.JmeExporter;
import com.jme3.export.binary.BinaryExporter;
import com.jme3.export.xml.XMLExporter;
import com.jme3.font.BitmapText;
import com.jme3.font.Rectangle;
import com.jme3.input.CameraInput;
import com.jme3.input.KeyInput;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.shadow.DirectionalLightShadowRenderer;
import com.jme3.system.AppSettings;
import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.InfluenceUtil;
import jme3utilities.MyAsset;
import jme3utilities.MySpatial;
import jme3utilities.MyString;
import jme3utilities.NameGenerator;
import jme3utilities.debug.SkeletonVisualizer;
import jme3utilities.math.noise.Generator;
import jme3utilities.mesh.Icosphere;
import jme3utilities.minie.DumpFlags;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.minie.test.common.PhysicsDemo;
import jme3utilities.minie.test.tunings.BaseMeshControl;
import jme3utilities.minie.test.tunings.Biped;
import jme3utilities.minie.test.tunings.ElephantControl;
import jme3utilities.minie.test.tunings.JaimeControl;
import jme3utilities.minie.test.tunings.MhGameControl;
import jme3utilities.minie.test.tunings.NinjaControl;
import jme3utilities.minie.test.tunings.OtoControl;
import jme3utilities.minie.test.tunings.PuppetControl;
import jme3utilities.minie.test.tunings.SinbadControl;
import jme3utilities.ui.ActionApplication;
import jme3utilities.ui.InputMode;
import jme3utilities.ui.Signals;
import jme3utilities.wes.AnimationEdit;
import jme3utilities.wes.Pose;

/**
 * Test scaling and load/save on a DynamicAnimControl.
 * <p>
 * Seen in the October 2018 demo video:
 * https://www.youtube.com/watch?v=A1Rii99nb3Q
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestDac extends PhysicsDemo {
    // *************************************************************************
    // constants and loggers

    /**
     * radius of each falling ball (in mesh units)
     */
    final private static float ballRadius = 1f;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(TestDac.class.getName());
    /**
     * asset path for saving J3O
     */
    final private static String saveAssetPath = "TestDac.j3o";
    /**
     * first asset path for saving XML
     */
    final private static String saveAssetPath1 = "TestDac_1.xml";
    /**
     * 2nd asset path for saving XML
     */
    final private static String saveAssetPath2 = "TestDac_2.xml";
    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName = TestDac.class.getSimpleName();
    // *************************************************************************
    // fields

    /**
     * text displayed in the upper-left corner of the GUI node
     */
    private static BitmapText statusText;
    /**
     * important linked bones
     */
    private static BoneLink leftClavicle;
    private static BoneLink leftFemur;
    private static BoneLink leftUlna;
    private static BoneLink rightClavicle;
    private static BoneLink rightFemur;
    private static BoneLink upperBody;
    /**
     * AppState to manage the PhysicsSpace
     */
    private static BulletAppState bulletAppState;
    /**
     * Control being tested
     */
    private static DynamicAnimControl dac;
    /**
     * Mesh for falling balls
     */
    final private static Mesh ballMesh = new Icosphere(2, ballRadius);
    /**
     * generate names for falling balls
     */
    final private static NameGenerator nameGenerator = new NameGenerator();
    /**
     * root node of the C-G model on which the Control is being tested
     */
    private static Node cgModel;
    /**
     * animation pose, or null if not in use
     */
    private static Pose animPose;
    /**
     * visualizer for the skeleton of the C-G model
     */
    private static SkeletonVisualizer sv;
    /**
     * SkinningControl of the loaded model
     */
    private static SkinningControl sc;
    /**
     * name of the Animation/Action to play on the C-G model
     */
    private static String animationName = null;
    /**
     * name the important linked bones
     */
    private static String leftClavicleName;
    private static String leftUlnaName;
    private static String rightClavicleName;
    private static String upperBodyName;
    /**
     * name of the test (the model that's loaded)
     */
    private static String testName = "";
    /**
     * C-G model's local transform when initially loaded
     */
    private static Transform resetTransform;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the TestDac application.
     */
    public TestDac() { // made explicit to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the TestDac application.
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

        Application application = new TestDac();
        application.setSettings(settings);
        /*
         * Designate a sandbox directory.
         * This must be done *prior to* initialization.
         */
        try {
            ActionApplication.designateSandbox("Written Assets");
        } catch (IOException exception) {
            // do nothing
        }
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
        generateMaterials();
        configurePhysics();

        ColorRGBA skyColor = new ColorRGBA(0.1f, 0.2f, 0.4f, 1f);
        viewPort.setBackgroundColor(skyColor);

        addLighting();

        // Hide the render-statistics overlay.
        stateManager.getState(StatsAppState.class).toggleStats();

        float halfExtent = 250f;
        float topY = 0f;
        attachCubePlatform(halfExtent, topY);

        ColorRGBA ballColor = new ColorRGBA(0.4f, 0f, 0f, 1f);
        Material ballMaterial
                = MyAsset.createShinyMaterial(assetManager, ballColor);
        ballMaterial.setFloat("Shininess", 5f);
        registerMaterial("ball", ballMaterial);

        CollisionShape ballShape = new SphereCollisionShape(ballRadius);
        registerShape("ball", ballShape);

        addModel("Sinbad");
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
     * Calculate screen bounds for the detailed help node.
     *
     * @param viewPortWidth (in pixels, &gt;0)
     * @param viewPortHeight (in pixels, &gt;0)
     * @return a new instance
     */
    @Override
    public Rectangle detailedHelpBounds(int viewPortWidth, int viewPortHeight) {
        // Position help nodes below the status.
        float margin = 10f; // in pixels
        float leftX = margin;
        float topY = viewPortHeight - 20f - margin;
        float width = viewPortWidth - leftX - margin;
        float height = topY - margin;
        Rectangle result = new Rectangle(leftX, topY, width, height);

        return result;
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
        return 2f;
    }

    /**
     * Add application-specific hotkey bindings (and override existing ones, if
     * necessary).
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        dim.bind("add", KeyInput.KEY_INSERT);
        dim.bind("amputate left elbow", KeyInput.KEY_DELETE);
        dim.bind("blend all to kinematic", KeyInput.KEY_K);
        dim.bind(asCollectGarbage, KeyInput.KEY_G);
        dim.bind("drop attachments", KeyInput.KEY_PGDN);
        dim.bind(asDumpScenes, KeyInput.KEY_P);
        dim.bind(asDumpSpace, KeyInput.KEY_O);

        dim.bind("freeze all", KeyInput.KEY_F);
        dim.bind("freeze upper body", KeyInput.KEY_U);
        dim.bind("ghost upper body", KeyInput.KEY_8);
        dim.bind("go anim pose", KeyInput.KEY_6);
        dim.bind("go bind pose", KeyInput.KEY_B);
        dim.bind("go dynamic bind pose", KeyInput.KEY_7);
        dim.bind("go floating", KeyInput.KEY_0);
        dim.bind("go frozen", KeyInput.KEY_MINUS);
        dim.bind("go limp", KeyInput.KEY_SPACE);
        dim.bind("limp left arm", KeyInput.KEY_LBRACKET);
        dim.bind("limp right arm", KeyInput.KEY_RBRACKET);
        dim.bind("load", KeyInput.KEY_L);

        dim.bind("load BaseMesh", KeyInput.KEY_F11);
        dim.bind("load Elephant", KeyInput.KEY_F3);
        dim.bind("load Jaime", KeyInput.KEY_F2);
        dim.bind("load MhGame", KeyInput.KEY_F9);
        dim.bind("load Ninja", KeyInput.KEY_F7);
        dim.bind("load Oto", KeyInput.KEY_F6);
        dim.bind("load Puppet", KeyInput.KEY_F8);
        dim.bind("load Sinbad", KeyInput.KEY_F1);
        dim.bind("load SinbadWith1Sword", KeyInput.KEY_F10);
        dim.bind("load SinbadWithSwords", KeyInput.KEY_F4);

        dim.bind("pin leftFemur", KeyInput.KEY_9);
        dim.bind("raise leftFoot", KeyInput.KEY_LCONTROL);
        dim.bind("raise leftHand", KeyInput.KEY_LSHIFT);
        dim.bind("raise rightFoot", KeyInput.KEY_RCONTROL);
        dim.bind("raise rightHand", KeyInput.KEY_RSHIFT);

        dim.bind("reset model transform", KeyInput.KEY_HOME);
        dim.bind("save", KeyInput.KEY_COMMA);
        dim.bind("set height 1", KeyInput.KEY_1);
        dim.bind("set height 2", KeyInput.KEY_2);
        dim.bind("set height 3", KeyInput.KEY_3);

        dim.bindSignal(CameraInput.FLYCAM_LOWER, KeyInput.KEY_DOWN);
        dim.bindSignal(CameraInput.FLYCAM_RISE, KeyInput.KEY_UP);
        dim.bindSignal("rotateLeft", KeyInput.KEY_LEFT);
        dim.bindSignal("rotateRight", KeyInput.KEY_RIGHT);
        dim.bindSignal("shower", KeyInput.KEY_I);

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
                case "add":
                    addBall();
                    return;
                case "amputate left elbow":
                    dac.amputateSubtree(leftUlna, 2f);
                    return;
                case "blend all to kinematic":
                    animPose = null;
                    dac.blendToKinematicMode(2f, null);
                    return;
                case "drop attachments":
                    dac.dropAttachments();
                    return;

                case "freeze all":
                    animPose = null;
                    dac.freezeSubtree(dac.getTorsoLink(), false);
                    return;
                case "freeze upper body":
                    dac.freezeSubtree(upperBody, false);
                    return;
                case "ghost upper body":
                    dac.setContactResponseSubtree(upperBody, false);
                    return;

                case "go anim pose":
                    goAnimPose();
                    return;
                case "go bind pose":
                    animPose = null;
                    dac.bindSubtree(dac.getTorsoLink(), 2f);
                    return;
                case "go dynamic bind pose":
                    if (dac.isReady()) {
                        goDynamicBindPose();
                    }
                    return;
                case "go floating":
                    if (dac.isReady()) {
                        animPose = null;
                        dac.setDynamicSubtree(
                                dac.getTorsoLink(), Vector3f.ZERO, false);
                    }
                    return;
                case "go frozen":
                    if (dac.isReady()) {
                        animPose = null;
                        Vector3f gravity = dac.gravity(null);
                        dac.setDynamicSubtree(
                                dac.getTorsoLink(), gravity, true);
                    }
                    return;
                case "go limp":
                    if (dac.isReady()) {
                        animPose = null;
                        dac.setRagdollMode();
                    }
                    return;

                case "limp left arm":
                    if (dac.isReady()) {
                        dac.setDynamicSubtree(leftClavicle,
                                new Vector3f(0f, -150f, 0f), false);
                    }
                    return;
                case "limp right arm":
                    if (dac.isReady()) {
                        dac.setDynamicSubtree(rightClavicle,
                                new Vector3f(0f, -150f, 0f), false);
                    }
                    return;

                case "load":
                    load();
                    return;

                case "pin leftFemur":
                    if (dac.isReady()) {
                        boolean disableForRagdoll = false;
                        dac.pinToWorld(leftFemur, disableForRagdoll);
                    }
                    return;

                case "raise leftFoot":
                    if (dac.isReady()) {
                        dac.setDynamicSubtree(
                                leftFemur, new Vector3f(0f, 100f, 0f), false);
                    }
                    return;
                case "raise leftHand":
                    if (dac.isReady()) {
                        dac.setDynamicSubtree(leftClavicle,
                                new Vector3f(0f, 100f, 0f), false);
                    }
                    return;
                case "raise rightFoot":
                    if (dac.isReady()) {
                        dac.setDynamicSubtree(
                                rightFemur, new Vector3f(0f, 100f, 0f), false);
                    }
                    return;
                case "raise rightHand":
                    if (dac.isReady()) {
                        dac.setDynamicSubtree(rightClavicle,
                                new Vector3f(0f, 100f, 0f), false);
                    }
                    return;

                case "reset model transform":
                    cgModel.setLocalTransform(resetTransform);
                    return;
                case "save":
                    save(cgModel, saveAssetPath);
                    save(cgModel, saveAssetPath1);
                    return;
                case "set height 1":
                    setHeight(5f);
                    return;
                case "set height 2":
                    setHeight(10f);
                    return;
                case "set height 3":
                    setHeight(15f);
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
            if (words.length >= 2 && "load".equals(words[0])) {
                addModel(words[1]);
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
            addBall();
        }

        float rotateAngle = 0f;
        if (signals.test("rotateRight")) {
            rotateAngle += tpf;
        }
        if (signals.test("rotateLeft")) {
            rotateAngle -= tpf;
        }
        if (rotateAngle != 0f) {
            rotateAngle /= speed;
            Quaternion orientation = MySpatial.worldOrientation(cgModel, null);
            Quaternion rotate = new Quaternion();
            rotate.fromAngles(0f, rotateAngle, 0f);
            rotate.mult(orientation, orientation);
            MySpatial.setWorldOrientation(cgModel, orientation);
        }

        if (animPose != null) {
            matchAnimPose();
        }

        updateStatusText();
    }
    // *************************************************************************
    // private methods

    /**
     * Add a falling ball to the scene.
     */
    private void addBall() {
        String name = nameGenerator.unique("ball");
        Geometry geometry = new Geometry(name, ballMesh);
        rootNode.attachChild(geometry);

        Material material = findMaterial("ball");
        geometry.setMaterial(material);
        geometry.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);
        Generator random = getGenerator();
        Vector3f location = random.nextVector3f();
        location.multLocal(2.5f, 5f, 2.5f);
        location.y += 20f;
        geometry.move(location);

        Vector3f worldScale = geometry.getWorldScale(); // alias
        CollisionShape shape = findShape("ball");
        shape.setScale(worldScale);

        float mass = 12f;
        RigidBodyControl rbc = new RigidBodyControl(shape, mass);
        rbc.setApplyScale(true);
        rbc.setLinearVelocity(new Vector3f(0f, -5f, 0f));
        rbc.setKinematic(false);
        rbc.setFriction(10f);

        addCollisionObject(rbc);
        rbc.setGravity(new Vector3f(0f, -1.5f, 0f));

        geometry.addControl(rbc);
    }

    /**
     * Add lighting and shadows to the main scene.
     */
    private void addLighting() {
        ColorRGBA ambientColor = new ColorRGBA(0.2f, 0.2f, 0.2f, 1f);
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
     * Add an animated model to the scene, removing any previously added model.
     *
     * @param modelName the name of the model to add (not null, not empty)
     */
    private void addModel(String modelName) {
        if (cgModel != null) {
            dac.getSpatial().removeControl(dac);
            rootNode.detachChild(cgModel);
            rootNode.removeControl(sv);
            removeAllBalls();
        }

        switch (modelName) {
            case "BaseMesh":
                loadBaseMesh();
                break;
            case "Elephant":
                loadElephant();
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

        testName = modelName;
        animPose = null;

        List<Spatial> list = MySpatial.listSpatials(cgModel);
        for (Spatial spatial : list) {
            spatial.setShadowMode(RenderQueue.ShadowMode.Cast);
        }
        cgModel.setCullHint(Spatial.CullHint.Never);

        rootNode.attachChild(cgModel);
        setCgmHeight(cgModel, 10f);
        centerCgm(cgModel);
        resetTransform = cgModel.getLocalTransform().clone();

        sc = (SkinningControl) RagUtils.findSControl(cgModel);
        Spatial controlledSpatial = sc.getSpatial();

        controlledSpatial.addControl(dac);
        dac.setGravity(new Vector3f(0f, -50f, 0f));
        PhysicsSpace physicsSpace = getPhysicsSpace();
        dac.setPhysicsSpace(physicsSpace);

        leftClavicle = dac.findBoneLink(leftClavicleName);
        if (dac instanceof Biped) {
            BoneLink leftFoot = ((Biped) dac).getLeftFoot();
            leftFemur = leftFoot;
            while (leftFemur.getParent() instanceof BoneLink) {
                leftFemur = (BoneLink) leftFemur.getParent();
            }

            BoneLink rightFoot = ((Biped) dac).getRightFoot();
            rightFemur = rightFoot;
            while (rightFemur.getParent() instanceof BoneLink) {
                rightFemur = (BoneLink) rightFemur.getParent();
            }

        } else if (dac instanceof ElephantControl) {
            leftFemur = dac.findBoneLink("Oberschenkel_B_L");
            rightFemur = dac.findBoneLink("Oberschenkel_B_R");
        }

        leftUlna = dac.findBoneLink(leftUlnaName);
        rightClavicle = dac.findBoneLink(rightClavicleName);
        upperBody = dac.findBoneLink(upperBodyName);

        AnimComposer composer
                = controlledSpatial.getControl(AnimComposer.class);
        composer.setCurrentAction(animationName);

        sv = new SkeletonVisualizer(assetManager, sc);
        sv.setLineColor(ColorRGBA.Yellow);
        InfluenceUtil.hideNonInfluencers(sv, sc);
        rootNode.addControl(sv);
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(20f);
        flyCam.setZoomSpeed(20f);

        cam.setLocation(new Vector3f(0f, 6f, 25f));
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);

        PhysicsSpace physicsSpace = getPhysicsSpace();
        physicsSpace.setAccuracy(0.01f); // 10-msec timestep
        physicsSpace.getSolverInfo().setNumIterations(15);
    }

    /**
     * Put all physics links into zero-g dynamic mode, fix the torso, and enable
     * pose matching.
     */
    private static void goAnimPose() {
        TorsoLink torsoLink = dac.getTorsoLink();
        torsoLink.setDynamic(Vector3f.ZERO);
        boolean disableForRagdoll = true;
        dac.fixToWorld(torsoLink, disableForRagdoll);

        animPose = Pose.newInstance(sc);
        matchAnimPose();
    }

    /**
     * Put all bone/torso links into zero-g dynamic mode and lock all physics
     * joints at bind pose.
     */
    private static void goDynamicBindPose() {
        animPose = null;

        TorsoLink torsoLink = dac.getTorsoLink();
        torsoLink.setDynamic(Vector3f.ZERO);

        for (BoneLink boneLink : dac.listLinks(BoneLink.class)) {
            boneLink.setDynamic(Vector3f.ZERO, Quaternion.IDENTITY);
        }
    }

    /**
     * Load the saved model from the J3O file.
     */
    private void load() {
        ModelKey key = new ModelKey(saveAssetPath);

        // Remove any copy from the asset manager's cache.
        assetManager.deleteFromCache(key);

        Spatial loadedScene;
        try {
            loadedScene = assetManager.loadAsset(key);
        } catch (AssetNotFoundException exception) {
            logger.log(Level.SEVERE, "Didn''t find asset {0}",
                    MyString.quote(saveAssetPath));
            return;
        }
        logger.log(Level.INFO, "Loaded {0} from asset {1}", new Object[]{
            MyString.quote(loadedScene.getName()),
            MyString.quote(saveAssetPath)
        });

        save(loadedScene, saveAssetPath2);
    }

    /**
     * Load the BaseMesh model.
     */
    private void loadBaseMesh() {
        cgModel = (Node) assetManager.loadModel("Models/BaseMesh/BaseMesh.j3o");
        dac = new BaseMeshControl();
        animationName = "run_01";
        leftClavicleName = "spine2_L.001";
        leftUlnaName = "spine2_L.002";
        rightClavicleName = "spine2_R.001";
        upperBodyName = "spine2";
        /*
         * Normalize all quaternions in the animation to resolve some issues
         * with the BaseMesh model.
         */
        Spatial spatial = RagUtils.findSControl(cgModel).getSpatial();
        AnimComposer composer = spatial.getControl(AnimComposer.class);
        AnimClip clip = composer.getAnimClip(animationName);
        AnimationEdit.normalizeQuaternions(clip, 0.0001f);
    }

    /**
     * Load the Elephant model.
     */
    private void loadElephant() {
        cgModel = (Node) assetManager.loadModel("Models/Elephant/Elephant.j3o");
        cgModel.rotate(0f, 1.6f, 0f);
        dac = new ElephantControl();
        animationName = "legUp";
        leftClavicleName = "Oberschenkel_F_L";
        leftUlnaName = "Knee_F_L";
        rightClavicleName = "Oberschenkel_F_R";
        upperBodyName = "joint5";
    }

    /**
     * Load the Jaime model.
     */
    private void loadJaime() {
        cgModel = (Node) assetManager.loadModel("Models/Jaime/Jaime-new.j3o");
        Geometry g = (Geometry) cgModel.getChild(0);
        RenderState rs = g.getMaterial().getAdditionalRenderState();
        rs.setFaceCullMode(RenderState.FaceCullMode.Off);

        dac = new JaimeControl();
        animationName = "Punches";
        leftClavicleName = "shoulder.L";
        leftUlnaName = "forearm.L";
        rightClavicleName = "shoulder.R";
        upperBodyName = "ribs";
    }

    /**
     * Load the MhGame model.
     */
    private void loadMhGame() {
        cgModel = (Node) assetManager.loadModel("Models/MhGame/MhGame.j3o");
        dac = new MhGameControl();
        animationName = "expr-lib-pose";
        leftClavicleName = "upperarm_l";
        leftUlnaName = "lowerarm_l";
        rightClavicleName = "upperarm_r";
        upperBodyName = "spine_01";
    }

    /**
     * Load the Ninja model.
     */
    private void loadNinja() {
        cgModel = (Node) assetManager.loadModel("Models/Ninja/Ninja.j3o");
        cgModel.rotate(0f, 3f, 0f);
        dac = new NinjaControl();
        animationName = "Walk";
        leftClavicleName = "Joint14";
        leftUlnaName = "Joint16";
        rightClavicleName = "Joint9";
        upperBodyName = "Joint4";
    }

    /**
     * Load the Oto model.
     */
    private void loadOto() {
        cgModel = (Node) assetManager.loadModel("Models/Oto/Oto.j3o");
        dac = new OtoControl();
        animationName = "Walk";
        leftClavicleName = "uparm.left";
        leftUlnaName = "arm.left";
        rightClavicleName = "uparm.right";
        upperBodyName = "spinehigh";
    }

    /**
     * Load the Puppet model.
     */
    private void loadPuppet() {
        cgModel = (Node) assetManager.loadModel("Models/Puppet/Puppet.j3o");
        AnimMigrationUtils.migrate(cgModel);
        dac = new PuppetControl();
        animationName = "walk";
        leftClavicleName = "upper_arm.1.L";
        leftUlnaName = "forearm.1.L";
        rightClavicleName = "upper_arm.1.R";
        upperBodyName = "spine";
    }

    /**
     * Load the Sinbad model without attachments.
     */
    private void loadSinbad() {
        cgModel = (Node) assetManager.loadModel("Models/Sinbad/Sinbad.j3o");
        dac = new SinbadControl();
        animationName = "Dance";
        leftClavicleName = "Clavicle.L";
        leftUlnaName = "Ulna.L";
        rightClavicleName = "Clavicle.R";
        upperBodyName = "Waist";
    }

    /**
     * Load the Sinbad model with an attached sword.
     */
    private void loadSinbadWith1Sword() {
        cgModel = (Node) assetManager.loadModel("Models/Sinbad/Sinbad.j3o");
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

        animationName = "IdleTop";
        leftClavicleName = "Clavicle.L";
        leftUlnaName = "Ulna.L";
        rightClavicleName = "Clavicle.R";
        upperBodyName = "Waist";
    }

    /**
     * Load the Sinbad model with 2 attached swords.
     */
    private void loadSinbadWithSwords() {
        cgModel = (Node) assetManager.loadModel("Models/Sinbad/Sinbad.j3o");
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

        animationName = "RunTop";
        leftClavicleName = "Clavicle.L";
        leftUlnaName = "Ulna.L";
        rightClavicleName = "Clavicle.R";
        upperBodyName = "Waist";
    }

    /**
     * Update the animation pose and apply it to all bones except the main root
     * bone.
     */
    private static void matchAnimPose() {
        assert animPose != null;

        // Update the animation pose.
        Spatial controlledSpatial = sc.getSpatial();
        AnimComposer composer
                = controlledSpatial.getControl(AnimComposer.class);
        AnimClip clip = composer.getAnimClip(animationName);
        double time = composer.getTime(AnimComposer.DEFAULT_LAYER);
        animPose.setToClip(clip, time);

        Vector3f acceleration = Vector3f.ZERO;

        // Ensure that all attachment links are dynamic.
        for (AttachmentLink link : dac.listLinks(AttachmentLink.class)) {
            link.setDynamic(acceleration);
        }
        Transform tmpTransform = new Transform();

        // Apply the animation pose to the torso's managed bones.
        TorsoLink torsoLink = dac.getTorsoLink();
        int numManagedBones = torsoLink.countManaged();
        for (int mbIndex = 1; mbIndex < numManagedBones; ++mbIndex) {
            int boneIndex = torsoLink.boneIndex(mbIndex);
            animPose.localTransform(boneIndex, tmpTransform);
            torsoLink.setLocalTransform(mbIndex, tmpTransform);
        }

        // Apply the animation pose to all bone links and their managed bones.
        for (BoneLink boneLink : dac.listLinks(BoneLink.class)) {
            int boneIndex = boneLink.boneIndex(0); // the linked bone
            Quaternion userRotation = tmpTransform.getRotation(); // alias
            animPose.userRotation(boneIndex, userRotation);
            boneLink.setDynamic(acceleration, userRotation);

            numManagedBones = boneLink.countManaged();
            for (int mbIndex = 1; mbIndex < numManagedBones; ++mbIndex) {
                boneIndex = boneLink.boneIndex(mbIndex);
                animPose.localTransform(boneIndex, tmpTransform);
                boneLink.setLocalTransform(mbIndex, tmpTransform);
            }
        }
    }

    /**
     * Delete all balls from the scene.
     */
    private void removeAllBalls() {
        List<Geometry> geometries = rootNode.descendantMatches(Geometry.class);
        for (Geometry geometry : geometries) {
            String name = geometry.getName();
            if (NameGenerator.isFrom(name, "ball")) {
                RigidBodyControl rbc
                        = geometry.getControl(RigidBodyControl.class);
                rbc.setPhysicsSpace(null);
                geometry.removeControl(rbc);
                geometry.removeFromParent();
            }
        }
    }

    /**
     * Save the specified model to a J3O or XML file.
     *
     * @param model the model to save
     * @param assetPath the asset-path portion of the save filename (not null)
     */
    private static void save(Spatial model, String assetPath) {
        String filePath = filePath(assetPath);
        File file = new File(filePath);

        JmeExporter exporter;
        if (assetPath.endsWith(".j3o")) {
            exporter = BinaryExporter.getInstance();
        } else {
            assert assetPath.endsWith(".xml");
            exporter = XMLExporter.getInstance();
        }

        try {
            exporter.save(model, file);
        } catch (IOException exception) {
            System.out.println("Caught IOException: " + exception.getMessage());
            logger.log(Level.SEVERE,
                    "Output exception while saving {0} to file {1}",
                    new Object[]{
                        MyString.quote(model.getName()),
                        MyString.quote(filePath)
                    });
            return;
        }
        logger.log(Level.INFO, "Saved {0} to file {1}", new Object[]{
            MyString.quote(model.getName()),
            MyString.quote(filePath)
        });
    }

    /**
     * Test re-scaling the model.
     *
     * @param height the desired height of the model (in world units, &gt;0)
     */
    private static void setHeight(float height) {
        assert height > 0f : height;

        setCgmHeight(cgModel, height);
        centerCgm(cgModel);
        dac.rebuild();
    }

    /**
     * Toggle mesh rendering on/off.
     */
    private void toggleMeshes() {
        Spatial.CullHint hint = cgModel.getLocalCullHint();
        if (hint == Spatial.CullHint.Inherit
                || hint == Spatial.CullHint.Never) {
            hint = Spatial.CullHint.Always;
        } else if (hint == Spatial.CullHint.Always) {
            hint = Spatial.CullHint.Never;
        }
        cgModel.setCullHint(hint);
        for (Spatial s : rootNode.getChildren()) {
            if (s.getName().startsWith("ball")) {
                s.setCullHint(hint);
            }
        }
    }

    /**
     * Toggle the skeleton visualizer on/off.
     */
    private static void toggleSkeleton() {
        boolean enabled = sv.isEnabled();
        sv.setEnabled(!enabled);
    }

    /**
     * Update the status text in the GUI.
     */
    private void updateStatusText() {
        String message = "Test: " + testName + "  View: ";

        Spatial.CullHint cull = cgModel.getLocalCullHint();
        message += (cull == Spatial.CullHint.Always) ? "NOmeshes" : "Meshes";

        boolean debug = bulletAppState.isDebugEnabled();
        if (debug) {
            message += "+" + describePhysicsDebugOptions();
        }
        message += sv.isEnabled() ? "+Skeleton" : "";
        message += isPaused() ? "  PAUSED" : "";

        double energy = dac.kineticEnergy();
        if (Double.isFinite(energy)) {
            message += String.format("  KE=%f", energy);
        }
        statusText.setText(message);
    }
}
