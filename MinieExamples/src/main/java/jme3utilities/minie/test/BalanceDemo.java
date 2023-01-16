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

import com.jme3.anim.AnimComposer;
import com.jme3.anim.SkinningControl;
import com.jme3.anim.util.AnimMigrationUtils;
import com.jme3.app.Application;
import com.jme3.app.StatsAppState;
import com.jme3.app.state.AppState;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.animation.BoneLink;
import com.jme3.bullet.animation.CenterHeuristic;
import com.jme3.bullet.animation.DynamicAnimControl;
import com.jme3.bullet.animation.LinkConfig;
import com.jme3.bullet.animation.MassHeuristic;
import com.jme3.bullet.animation.RagUtils;
import com.jme3.bullet.animation.ShapeHeuristic;
import com.jme3.bullet.animation.TorsoLink;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.font.Rectangle;
import com.jme3.input.CameraInput;
import com.jme3.input.KeyInput;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.shadow.DirectionalLightShadowRenderer;
import com.jme3.system.AppSettings;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.InfluenceUtil;
import jme3utilities.MyCamera;
import jme3utilities.MySpatial;
import jme3utilities.MyString;
import jme3utilities.debug.PointVisualizer;
import jme3utilities.debug.SkeletonVisualizer;
import jme3utilities.math.MyArray;
import jme3utilities.math.MyVector3f;
import jme3utilities.minie.DumpFlags;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.minie.test.common.PhysicsDemo;
import jme3utilities.minie.test.controllers.BalanceController;
import jme3utilities.minie.test.controllers.UprightController;
import jme3utilities.minie.test.tunings.Biped;
import jme3utilities.minie.test.tunings.JaimeControl;
import jme3utilities.minie.test.tunings.MhGameControl;
import jme3utilities.minie.test.tunings.NinjaControl;
import jme3utilities.minie.test.tunings.OtoControl;
import jme3utilities.minie.test.tunings.PuppetControl;
import jme3utilities.minie.test.tunings.SinbadControl;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.InputMode;

/**
 * Demo/testbed for BalanceController inverse kinematics.
 * <p>
 * Seen in the December 2018 demo video:
 * https://www.youtube.com/watch?v=ZGqN9ZCCu-8
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class BalanceDemo extends PhysicsDemo {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(BalanceDemo.class.getName());
    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName
            = BalanceDemo.class.getSimpleName();
    // *************************************************************************
    // fields

    /**
     * composer for playing canned animations
     */
    private static AnimComposer composer = null;
    /**
     * keeps the model's center of mass directly above its center of support
     */
    private static BalanceController balance = null;
    /**
     * true once {@link #initWhenReady()} has been invoked for the latest model
     */
    private static boolean dacReadyInitDone = false;
    /**
     * AppState to manage the PhysicsSpace
     */
    private static BulletAppState bulletAppState;
    /**
     * Control being tested
     */
    private static DynamicAnimControl dac;
    /**
     * fraction of the model's weight that's on its right foot
     */
    private static float rightSupportFraction = 0.5f;
    /**
     * gain for the UprightController (tuned for each model)
     */
    private static float uprightGain;
    /**
     * parameters that determine the torso's vertical acceleration
     */
    private static float vaBias = 0f; // tuned for each model
    private static float vaMagnitude = 0f; // tuned for each model
    private static float vaSign = +1f;
    /**
     * root node of the C-G model on which the Control is being tested
     */
    private static Node cgModel;
    /**
     * visualizer for the center of mass
     */
    private static PointVisualizer comPoint;
    /**
     * visualizer for the center of support
     */
    private static PointVisualizer supportPoint;
    /**
     * visualizer for the skeleton of the C-G model
     */
    private static SkeletonVisualizer sv;
    /**
     * name of the Animation/Action to play on the C-G model
     */
    private static String animationName = null;
    /**
     * TorsoLink of the model
     */
    private static TorsoLink torso;
    /**
     * location of the left foot's center of support
     */
    private static Vector3f leftSupportLocation;
    /**
     * location of the right foot's center of support
     */
    private static Vector3f rightSupportLocation;
    /**
     * "up" direction of the torso, in its local coordinate system (unit vector,
     * different for each model)
     */
    private static Vector3f torsoUpDirection = null;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the BalanceDemo application.
     */
    public BalanceDemo() { // to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the BalanceDemo application.
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

        Application application = new BalanceDemo();
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

        int indicatorSize = 16; // in pixels
        comPoint = new PointVisualizer(
                assetManager, indicatorSize, ColorRGBA.Cyan, "ring");
        rootNode.attachChild(comPoint);

        supportPoint = new PointVisualizer(
                assetManager, indicatorSize, ColorRGBA.Yellow, "square");
        rootNode.attachChild(supportPoint);

        float halfExtent = 50f;
        float topY = 0f;
        attachCubePlatform(halfExtent, topY);

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
        // Position help nodes along the top edge of the viewport.
        float margin = 10f; // in pixels
        float width = viewPortWidth - 2f * margin;
        float height = viewPortHeight - 2f * margin;
        float leftX = margin;
        float topY = margin + height;
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
        return 0.5f;
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
        dim.bind("go limp", KeyInput.KEY_SPACE);

        dim.bind("load Jaime", KeyInput.KEY_F2);
        dim.bind("load MhGame", KeyInput.KEY_F9);
        dim.bind("load Ninja", KeyInput.KEY_F7);
        dim.bind("load Oto", KeyInput.KEY_F6);
        dim.bind("load Puppet", KeyInput.KEY_F8);
        dim.bind("load Sinbad", KeyInput.KEY_F1);
        dim.bind("load SinbadWith1Sword", KeyInput.KEY_F10);
        dim.bind("load SinbadWithSwords", KeyInput.KEY_F4);

        dim.bind("posture squat center", KeyInput.KEY_NUMPAD2);
        dim.bind("posture squat left", KeyInput.KEY_NUMPAD3);
        dim.bind("posture squat right", KeyInput.KEY_NUMPAD1);
        dim.bind("posture tall center", KeyInput.KEY_NUMPAD8);
        dim.bind("posture tall left", KeyInput.KEY_NUMPAD9);
        dim.bind("posture tall right", KeyInput.KEY_NUMPAD7);

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
                case "go limp":
                    if (dac.isReady()) {
                        dac.setRagdollMode();
                    }
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
            if (words.length == 2 && "load".equals(words[0])) {
                addModel(words[1]);
                return;
            } else if (words.length == 3 && "posture".equals(words[0])) {
                setPosture(words[1], words[2]);
                return;
            }
        }

        // The action is not handled: forward it to the superclass.
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

        comPoint.setEnabled(false);
        supportPoint.setEnabled(false);

        if (dac.isReady()) {
            if (!dacReadyInitDone) {
                initWhenReady();
                dacReadyInitDone = true;
            }

            Vector3f comLocation = new Vector3f();
            dac.centerOfMass(comLocation, null);
            MySpatial.setWorldLocation(comPoint, comLocation);
            comPoint.setEnabled(true);

            if (balance.isEnabled()) {
                Vector3f verticalAcceleration
                        = new Vector3f(0f, vaBias + vaSign * vaMagnitude, 0f);
                torso.setDynamic(verticalAcceleration);

                Vector3f supportLocation = MyVector3f.lerp(rightSupportFraction,
                        leftSupportLocation, rightSupportLocation, null);
                balance.setCenterOfSupport(supportLocation);
                MySpatial.setWorldLocation(supportPoint, supportLocation);
                supportPoint.setEnabled(true);
            }
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

        switch (modelName) {
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

        rootNode.attachChild(cgModel);
        setCgmHeight(cgModel, 2f);
        centerCgm(cgModel);

        SkinningControl sc = (SkinningControl) RagUtils.findSControl(cgModel);
        Spatial controlledSpatial = sc.getSpatial();

        controlledSpatial.addControl(dac);
        PhysicsSpace physicsSpace = getPhysicsSpace();
        dac.setPhysicsSpace(physicsSpace);

        torso = dac.getTorsoLink();
        composer = controlledSpatial.getControl(AnimComposer.class);

        sv = new SkeletonVisualizer(assetManager, sc);
        sv.setLineColor(ColorRGBA.Yellow);
        InfluenceUtil.hideNonInfluencers(sv, sc);
        rootNode.addControl(sv);
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        float near = 0.02f;
        float far = 20f;
        MyCamera.setNearFar(cam, near, far);

        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(2f);
        flyCam.setZoomSpeed(2f);

        cam.setLocation(new Vector3f(-3.3f, 1.5f, -0.1f));
        cam.setRotation(new Quaternion(0.08f, 0.687f, -0.06f, 0.72f));

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
        Biped biped = (Biped) dac;
        BoneLink leftFoot = biped.getLeftFoot();
        BoneLink rightFoot = biped.getRightFoot();

        // Pin the left foot to the ground.
        dac.setDynamicChain(leftFoot, 9, Vector3f.ZERO, false);
        Vector3f[] leftSole = leftFoot.footprint();
        for (Vector3f location : leftSole) {
            dac.pinToWorld(leftFoot, location);
        }

        // Pin the right foot to the ground.
        dac.setDynamicChain(rightFoot, 9, Vector3f.ZERO, false);
        Vector3f[] rightSole = rightFoot.footprint();
        for (Vector3f location : rightSole) {
            dac.pinToWorld(rightFoot, location);
        }

        // Apply a BalanceController to the torso.
        Vector3f acc = new Vector3f(0f, vaBias + vaSign * vaMagnitude, 0f);
        torso.setDynamic(acc);
        leftSupportLocation = MyArray.mean(leftSole, null);
        rightSupportLocation = MyArray.mean(rightSole, null);
        Vector3f supportLocation = MyVector3f.lerp(rightSupportFraction,
                leftSupportLocation, rightSupportLocation, null);
        balance = new BalanceController(torso, supportLocation);
        torso.addIKController(balance);

        // Apply an UprightController to the torso.
        UprightController upright
                = new UprightController(torso, torsoUpDirection);
        torso.addIKController(upright);
        upright.setDeltaGainFactor(uprightGain);
        upright.setErrorGainFactor(uprightGain);

        // Start playing a canned animation.
        composer.setCurrentAction(animationName);
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
        animationName = "Punches";
        uprightGain = 10f;
        vaBias = 5f;
        vaMagnitude = 100f;
        torsoUpDirection = Vector3f.UNIT_Z;
    }

    /**
     * Load the MhGame model.
     */
    private void loadMhGame() {
        cgModel = (Node) assetManager.loadModel("Models/MhGame/MhGame.j3o");
        cgModel.rotate(0f, 0f, 1.6f);

        dac = new MhGameControl();
        animationName = "expr-lib-pose";
        uprightGain = 25f;
        vaBias = 0f;
        vaMagnitude = 40f;
        torsoUpDirection = Vector3f.UNIT_X;
    }

    /**
     * Load the Ninja model.
     */
    private void loadNinja() {
        cgModel = (Node) assetManager.loadModel("Models/Ninja/Ninja.j3o");
        cgModel.rotate(0f, 1.6f, 0f);

        dac = new NinjaControl();
        animationName = "Walk";
        uprightGain = 100f;
        vaBias = -8f;
        vaMagnitude = 12f;
        torsoUpDirection = Vector3f.UNIT_Y;
    }

    /**
     * Load the Oto model.
     */
    private void loadOto() {
        cgModel = (Node) assetManager.loadModel("Models/Oto/Oto.j3o");
        cgModel.rotate(0f, -1.6f, 0f);

        dac = new OtoControl();
        animationName = "pull";
        uprightGain = 8f;
        vaBias = 0f;
        vaMagnitude = 100f;
        torsoUpDirection = Vector3f.UNIT_Y;
    }

    /**
     * Load the Puppet model.
     */
    private void loadPuppet() {
        cgModel = (Node) assetManager.loadModel("Models/Puppet/Puppet.j3o");
        AnimMigrationUtils.migrate(cgModel);
        cgModel.rotate(0f, -1.6f, 0f);

        dac = new PuppetControl();
        animationName = "wave";
        uprightGain = 10f;
        vaBias = 0f;
        vaMagnitude = 20f;
        torsoUpDirection = new Vector3f(0f, 0f, -1f);
    }

    /**
     * Load the Sinbad model without attachments.
     */
    private void loadSinbad() {
        cgModel = (Node) assetManager.loadModel("Models/Sinbad/Sinbad.j3o");
        cgModel.rotate(0f, -1.6f, 0f);

        dac = new SinbadControl();
        animationName = "Dance";
        uprightGain = 40f;
        vaBias = 0f;
        vaMagnitude = 80f;
        torsoUpDirection = Vector3f.UNIT_Y;
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

        LinkConfig swordConfig = new LinkConfig(
                5f, MassHeuristic.Density, ShapeHeuristic.VertexHull,
                Vector3f.UNIT_XYZ, CenterHeuristic.AABB);
        dac = new SinbadControl();
        dac.attach("Handle.R", swordConfig, sword);

        animationName = "RunTop";
        uprightGain = 40f;
        vaBias = 0f;
        vaMagnitude = 80f;
        torsoUpDirection = Vector3f.UNIT_Y;
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

        LinkConfig swordConfig = new LinkConfig(
                5f, MassHeuristic.Density, ShapeHeuristic.VertexHull,
                Vector3f.UNIT_XYZ, CenterHeuristic.AABB);
        dac = new SinbadControl();
        dac.attach("Handle.L", swordConfig, sword);
        dac.attach("Handle.R", swordConfig, sword);

        animationName = "RunTop";
        uprightGain = 40f;
        vaBias = 0f;
        vaMagnitude = 80f;
        torsoUpDirection = Vector3f.UNIT_Y;
    }

    private static void setPosture(String vertical, String side) {
        switch (vertical) {
            case "squat":
                vaSign = -1f;
                break;
            case "tall":
                vaSign = +1f;
                break;
            default:
                throw new IllegalArgumentException(vertical);
        }

        switch (side) {
            case "center":
                rightSupportFraction = 0.5f;
                break;
            case "left":
                rightSupportFraction = 0f;
                break;
            case "right":
                rightSupportFraction = 1f;
                break;
            default:
                throw new IllegalArgumentException(side);
        }
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
}
