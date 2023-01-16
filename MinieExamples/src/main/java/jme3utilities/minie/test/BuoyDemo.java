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

import com.jme3.anim.AnimComposer;
import com.jme3.anim.SkinningControl;
import com.jme3.anim.util.AnimMigrationUtils;
import com.jme3.app.Application;
import com.jme3.app.StatsAppState;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.animation.CenterHeuristic;
import com.jme3.bullet.animation.DynamicAnimControl;
import com.jme3.bullet.animation.LinkConfig;
import com.jme3.bullet.animation.MassHeuristic;
import com.jme3.bullet.animation.PhysicsLink;
import com.jme3.bullet.animation.RagUtils;
import com.jme3.bullet.animation.ShapeHeuristic;
import com.jme3.font.Rectangle;
import com.jme3.input.CameraInput;
import com.jme3.input.KeyInput;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Plane;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Quad;
import com.jme3.system.AppSettings;
import com.jme3.util.SkyFactory;
import com.jme3.water.SimpleWaterProcessor;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.InfluenceUtil;
import jme3utilities.MySpatial;
import jme3utilities.MyString;
import jme3utilities.debug.SkeletonVisualizer;
import jme3utilities.minie.DumpFlags;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.minie.test.common.PhysicsDemo;
import jme3utilities.minie.test.controllers.BuoyController;
import jme3utilities.minie.test.tunings.BaseMeshControl;
import jme3utilities.minie.test.tunings.ElephantControl;
import jme3utilities.minie.test.tunings.JaimeControl;
import jme3utilities.minie.test.tunings.MhGameControl;
import jme3utilities.minie.test.tunings.NinjaControl;
import jme3utilities.minie.test.tunings.OtoControl;
import jme3utilities.minie.test.tunings.PuppetControl;
import jme3utilities.minie.test.tunings.SinbadControl;
import jme3utilities.ui.InputMode;
import jme3utilities.ui.Signals;

/**
 * Demonstrate BuoyController.
 * <p>
 * Seen in the March 2019 demo video:
 * https://www.youtube.com/watch?v=eq09m7pbk5A
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class BuoyDemo extends PhysicsDemo {
    // *************************************************************************
    // constants and loggers

    /**
     * Y coordinate of the water's surface (in world coordinates)
     */
    final public static float surfaceElevation = 0f;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(BuoyDemo.class.getName());
    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName
            = BuoyDemo.class.getSimpleName();
    // *************************************************************************
    // fields

    /**
     * AppState to manage the PhysicsSpace
     */
    private static BulletAppState bulletAppState;
    /**
     * Control being tested
     */
    private static DynamicAnimControl dac;
    /**
     * root node of the C-G model on which the Control is being tested
     */
    private static Node cgModel;
    /**
     * scene-graph subtree containing all geometries visible in reflections
     */
    final private static Node reflectiblesNode = new Node("reflectibles");
    /**
     * scene-graph subtree containing all reflective geometries
     */
    final private static Node reflectorsNode = new Node("reflectors");
    /**
     * scene processor for water effects
     */
    private static SimpleWaterProcessor processor;
    /**
     * visualizer for the skeleton of the C-G model
     */
    private static SkeletonVisualizer sv;
    /**
     * name of the Animation/Action to play on the C-G model
     */
    private static String animationName = null;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the BuoyDemo application.
     */
    public BuoyDemo() { // made explicit to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the BuoyDemo application.
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

        Application application = new BuoyDemo();
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

        rootNode.attachChild(reflectiblesNode);
        rootNode.attachChild(reflectorsNode);

        configureCamera();
        configureDumper();
        configurePhysics();
        addLighting();

        // Hide the render-statistics overlay.
        stateManager.getState(StatsAppState.class).toggleStats();

        addSurface();
        addSky();
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
        return 2f;
    }

    /**
     * Add application-specific hotkey bindings (and override existing ones, if
     * necessary).
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        dim.bind(asCollectGarbage, KeyInput.KEY_G);
        dim.bind(asDumpScenes, KeyInput.KEY_P);
        dim.bind(asDumpSpace, KeyInput.KEY_O);
        dim.bind("go floating", KeyInput.KEY_0, KeyInput.KEY_SPACE);

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

        dim.bindSignal(CameraInput.FLYCAM_LOWER, KeyInput.KEY_DOWN);
        dim.bindSignal(CameraInput.FLYCAM_RISE, KeyInput.KEY_UP);
        dim.bindSignal("rotateLeft", KeyInput.KEY_LEFT);
        dim.bindSignal("rotateRight", KeyInput.KEY_RIGHT);

        dim.bind(asToggleAabbs, KeyInput.KEY_APOSTROPHE);
        dim.bind(asToggleDebug, KeyInput.KEY_SLASH);
        dim.bind(asTogglePcoAxes, KeyInput.KEY_SEMICOLON);
        dim.bind(asToggleHelp, KeyInput.KEY_H);
        dim.bind("toggle meshes", KeyInput.KEY_M);
        dim.bind(asTogglePause, KeyInput.KEY_PAUSE, KeyInput.KEY_PERIOD);
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
                case "go floating":
                    goFloating();
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
    }
    // *************************************************************************
    // private methods

    /**
     * Add lighting and reflections to the main scene.
     */
    private void addLighting() {
        ColorRGBA ambientColor = new ColorRGBA(0.2f, 0.2f, 0.2f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        rootNode.addLight(ambient);
        ambient.setName("ambient");

        Vector3f direction = new Vector3f(-1f, -0.2f, -1f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction);
        rootNode.addLight(sun);
        sun.setName("sun");

        processor = new SimpleWaterProcessor(assetManager);
        viewPort.addProcessor(processor);
        processor.setLightPosition(direction.mult(1000f));
        Plane surface = new Plane(Vector3f.UNIT_Y, surfaceElevation);
        processor.setPlane(surface);
        processor.setReflectionScene(reflectiblesNode);

        // Clip everything below the surface.
        processor.setReflectionClippingOffset(-0.1f);

        // Configure water and wave parameters.
        float waveHeight = 0.2f;
        processor.setDistortionScale(waveHeight);
        float waterTransparency = 0.4f;
        processor.setWaterDepth(waterTransparency);
        float waveSpeed = 0.06f;
        processor.setWaveSpeed(waveSpeed);
    }

    /**
     * Add an animated model to the scene, removing any previously added model.
     *
     * @param modelName the name of the model to add (not null, not empty)
     */
    private void addModel(String modelName) {
        if (cgModel != null) {
            dac.getSpatial().removeControl(dac);
            reflectiblesNode.detachChild(cgModel);
            rootNode.removeControl(sv);
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

        List<Spatial> list = MySpatial.listSpatials(cgModel);
        for (Spatial spatial : list) {
            spatial.setShadowMode(RenderQueue.ShadowMode.Cast);
        }
        cgModel.setCullHint(Spatial.CullHint.Never);

        reflectiblesNode.attachChild(cgModel);
        setCgmHeight(cgModel, 10f);
        centerCgm(cgModel);

        SkinningControl sc = (SkinningControl) RagUtils.findSControl(cgModel);
        Spatial controlledSpatial = sc.getSpatial();

        controlledSpatial.addControl(dac);
        dac.setGravity(new Vector3f(0f, -50f, 0f));
        PhysicsSpace physicsSpace = getPhysicsSpace();
        dac.setPhysicsSpace(physicsSpace);

        // Add buoyancy to each BoneLink.
        List<PhysicsLink> links = dac.listLinks(PhysicsLink.class);
        float density = 1.5f;
        for (PhysicsLink link : links) {
            BuoyController buoy
                    = new BuoyController(link, density, surfaceElevation);
            link.addIKController(buoy);
        }

        AnimComposer composer
                = controlledSpatial.getControl(AnimComposer.class);
        composer.setCurrentAction(animationName);

        sv = new SkeletonVisualizer(assetManager, sc);
        sv.setLineColor(ColorRGBA.Yellow);
        InfluenceUtil.hideNonInfluencers(sv, sc);
        rootNode.addControl(sv);

        if (isPaused()) {
            togglePause();
        }
    }

    /**
     * Add a cube-mapped sky.
     */
    private void addSky() {
        String assetPath = "Textures/Sky/Bright/BrightSky.dds";
        Spatial sky = SkyFactory.createSky(
                assetManager, assetPath, SkyFactory.EnvMapType.CubeMap);
        reflectiblesNode.attachChild(sky);
    }

    /**
     * Add a large Quad to represent the surface of the water.
     */
    private static void addSurface() {
        float diameter = 2000f;
        Mesh mesh = new Quad(diameter, diameter);
        mesh.scaleTextureCoordinates(new Vector2f(80f, 80f));

        Geometry geometry = new Geometry("water surface", mesh);
        reflectorsNode.attachChild(geometry);

        geometry.move(-diameter / 2, 0f, diameter / 2);
        Quaternion rot = new Quaternion();
        rot.fromAngleAxis(-FastMath.HALF_PI, Vector3f.UNIT_X);
        geometry.setLocalRotation(rot);
        Material material = processor.getMaterial();
        geometry.setMaterial(material);
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(20f);
        flyCam.setZoomSpeed(20f);

        cam.setLocation(new Vector3f(-3f, 12f, 20f));
        cam.setRotation(new Quaternion(0.01f, 0.97587f, -0.2125f, 0.049f));
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
     * Put the loaded model into ragdoll mode with buoyancy enabled.
     */
    private static void goFloating() {
        if (dac.isReady()) {
            dac.setRagdollMode();
        }
    }

    /**
     * Load the BaseMesh model.
     */
    private void loadBaseMesh() {
        cgModel = (Node) assetManager.loadModel("Models/BaseMesh/BaseMesh.j3o");
        //cgModel.rotate(0f, -1.6f, 0f);

        dac = new BaseMeshControl();
        animationName = "run_01";
    }

    /**
     * Load the Elephant model.
     */
    private void loadElephant() {
        cgModel = (Node) assetManager.loadModel("Models/Elephant/Elephant.j3o");
        cgModel.rotate(0f, 1.6f, 0f);
        dac = new ElephantControl();
        animationName = "legUp";
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
    }

    /**
     * Load the MhGame model.
     */
    private void loadMhGame() {
        cgModel = (Node) assetManager.loadModel("Models/MhGame/MhGame.j3o");
        dac = new MhGameControl();
        animationName = "expr-lib-pose";
    }

    /**
     * Load the Ninja model.
     */
    private void loadNinja() {
        cgModel = (Node) assetManager.loadModel("Models/Ninja/Ninja.j3o");
        cgModel.rotate(0f, 3f, 0f);
        dac = new NinjaControl();
        animationName = "Walk";
    }

    /**
     * Load the Oto model.
     */
    private void loadOto() {
        cgModel = (Node) assetManager.loadModel("Models/Oto/Oto.j3o");
        dac = new OtoControl();
        animationName = "Walk";
    }

    /**
     * Load the Puppet model.
     */
    private void loadPuppet() {
        cgModel = (Node) assetManager.loadModel("Models/Puppet/Puppet.j3o");
        AnimMigrationUtils.migrate(cgModel);
        dac = new PuppetControl();
        animationName = "walk";
    }

    /**
     * Load the Sinbad model without attachments.
     */
    private void loadSinbad() {
        cgModel = (Node) assetManager.loadModel("Models/Sinbad/Sinbad.j3o");
        dac = new SinbadControl();
        animationName = "Dance";
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

        LinkConfig swordConfig = new LinkConfig(
                5f, MassHeuristic.Density, ShapeHeuristic.VertexHull,
                Vector3f.UNIT_XYZ, CenterHeuristic.AABB);
        dac = new SinbadControl();
        dac.attach("Handle.R", swordConfig, sword);

        animationName = "IdleTop";
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

        LinkConfig swordConfig = new LinkConfig(
                5f, MassHeuristic.Density, ShapeHeuristic.VertexHull,
                Vector3f.UNIT_XYZ, CenterHeuristic.AABB);
        dac = new SinbadControl();
        dac.attach("Handle.L", swordConfig, sword);
        dac.attach("Handle.R", swordConfig, sword);

        animationName = "RunTop";
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
