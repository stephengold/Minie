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

import com.jme3.animation.SkeletonControl;
import com.jme3.app.StatsAppState;
import com.jme3.audio.openal.ALAudioRenderer;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.animation.CenterHeuristic;
import com.jme3.bullet.animation.DynamicAnimControl;
import com.jme3.bullet.animation.LinkConfig;
import com.jme3.bullet.animation.MassHeuristic;
import com.jme3.bullet.animation.PhysicsLink;
import com.jme3.bullet.animation.ShapeHeuristic;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.objects.PhysicsRigidBody;
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
import com.jme3.scene.plugins.ogre.MaterialLoader;
import com.jme3.scene.plugins.ogre.MeshLoader;
import com.jme3.scene.shape.Box;
import com.jme3.shadow.DirectionalLightShadowRenderer;
import com.jme3.system.AppSettings;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Misc;
import jme3utilities.MyAsset;
import jme3utilities.MyCamera;
import jme3utilities.MySpatial;
import jme3utilities.MyString;
import jme3utilities.debug.AxesVisualizer;
import jme3utilities.debug.PointVisualizer;
import jme3utilities.debug.SkeletonVisualizer;
import jme3utilities.math.MyVector3f;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.minie.test.tunings.Binocular;
import jme3utilities.minie.test.tunings.JaimeControl;
import jme3utilities.minie.test.tunings.MhGameControl;
import jme3utilities.minie.test.tunings.NinjaControl;
import jme3utilities.minie.test.tunings.OtoControl;
import jme3utilities.minie.test.tunings.PuppetControl;
import jme3utilities.minie.test.tunings.SinbadControl;
import jme3utilities.ui.ActionApplication;
import jme3utilities.ui.InputMode;
import jme3utilities.ui.Signals;

/**
 * Demo/testbed for TrackController inverse kinematics.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TrackDemo extends ActionApplication {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(TrackDemo.class.getName());
    /**
     * application name for its window's title bar
     */
    final private static String applicationName = "TrackDemo";
    // *************************************************************************
    // fields

    private boolean dacReadyInitDone = false;
    private BulletAppState bulletAppState;
    private DynamicAnimControl dac;
    /*
     * model height in world units
     */
    private float height;
    /*
     * world X coordinate for the model's center
     */
    private float xOffset;
    private int chainLength;

    private Node cgModel;
    /**
     * inverse kinematics joint for the finger/sword tip
     */
    private PhysicsJoint tipJoint = null;
    private PhysicsLink tipLink;
    private PhysicsRigidBody targetBody;
    private PhysicsSpace physicsSpace;
    private SkeletonControl sc;
    private SkeletonVisualizer sv;
    /**
     * vertex specifier for bridge of the nose
     */
    private String noseSpec;
    /**
     * vertex specifier for finger/sword tip
     */
    private String tipSpec;

    private TrackController leftWatch = null;
    private TrackController rightWatch = null;
    private TrackController watch = null;
    /**
     * face direction in the coordinates of the head
     */
    private Vector3f faceDirection;
    /*
     * locations of grid corners in world coordinates
     */
    private Vector3f gridTopLeft;
    private Vector3f gridTopRight;
    private Vector3f gridBottomLeft;
    private Vector3f gridBottomRight;

    public static Vector3f watchActual = new Vector3f(); //
    private PointVisualizer actualPoint;
    public static Vector3f watchDesired = new Vector3f();
    private PointVisualizer desiredPoint;
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
        Logger.getLogger(MaterialLoader.class.getName()).setLevel(Level.SEVERE);
        Logger.getLogger(MeshLoader.class.getName()).setLevel(Level.SEVERE);
        Logger.getLogger(ALAudioRenderer.class.getName())
                .setLevel(Level.SEVERE);

        TrackDemo application = new TrackDemo();
        /*
         * Customize the window's title bar.
         */
        AppSettings settings = new AppSettings(true);
        String title = applicationName + " " + MyString.join(arguments);
        settings.setTitle(title);

        settings.setGammaCorrection(true);
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
        configurePhysics();
        ColorRGBA bgColor = new ColorRGBA(0.2f, 0.2f, 1f, 1f);
        viewPort.setBackgroundColor(bgColor);
        addLighting();

        stateManager.getState(StatsAppState.class).toggleStats();
        addAxes();
        addBox();
        addGrid();

        //addModel("Sinbad");
        //addModel("MhGame");
        addModel("SinbadWith1Sword");

        actualPoint = new PointVisualizer(assetManager, 16, ColorRGBA.Green,
                "saltire"); //
        rootNode.attachChild(actualPoint);
        desiredPoint = new PointVisualizer(assetManager, 16, ColorRGBA.Green,
                "ring"); //
        rootNode.attachChild(desiredPoint);
        /*
         * Add a target rigid body, to be moved by dragging RMB.
         */
        CollisionShape shape = new SphereCollisionShape(0.02f);
        targetBody = new PhysicsRigidBody(shape);
        targetBody.setKinematic(true);
        physicsSpace.add(targetBody);

        Vector3f targetStartLocation
                = MyVector3f.midpoint(gridTopLeft, gridBottomRight);
        targetBody.setPhysicsLocation(targetStartLocation);
    }

    /**
     * Add application-specific hotkey bindings and override existing ones.
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        dim.bind("dump physicsSpace", KeyInput.KEY_O);
        dim.bind("dump scenes", KeyInput.KEY_P);
        dim.bind("load Jaime", KeyInput.KEY_F2);
        dim.bind("load MhGame", KeyInput.KEY_F9);
        dim.bind("load Ninja", KeyInput.KEY_F7);
        dim.bind("load Oto", KeyInput.KEY_F6);
        dim.bind("load Puppet", KeyInput.KEY_F8);
        dim.bind("load Sinbad", KeyInput.KEY_F1);
        dim.bind("load SinbadWith1Sword", KeyInput.KEY_F10);
        dim.bind("load SinbadWithSwords", KeyInput.KEY_F4);
        dim.bind("signal orbitLeft", KeyInput.KEY_LEFT);
        dim.bind("signal orbitRight", KeyInput.KEY_RIGHT);
        dim.bind("signal track", "RMB");
        dim.bind("toggle meshes", KeyInput.KEY_M);
        dim.bind("toggle pause", KeyInput.KEY_PERIOD);
        dim.bind("toggle physics debug", KeyInput.KEY_SLASH);
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
                case "dump physicsSpace":
                    dumpPhysicsSpace();
                    return;
                case "dump scenes":
                    dumpScenes();
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
     * @param tpf time interval between frames (in seconds, &ge;0)
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

        MySpatial.setWorldLocation(actualPoint, watchActual); //
        MySpatial.setWorldLocation(desiredPoint, watchDesired);

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
     * Add a large static box to serve as a platform.
     */
    private void addBox() {
        float halfExtent = 50f; // mesh units
        Mesh mesh = new Box(halfExtent, halfExtent, halfExtent);
        Geometry geometry = new Geometry("box", mesh);
        rootNode.attachChild(geometry);

        geometry.move(0f, -halfExtent, 0f);
        ColorRGBA color = new ColorRGBA(0f, 0.2f, 0f, 1f);
        Material material = MyAsset.createShadedMaterial(assetManager, color);
        geometry.setMaterial(material);
        geometry.setShadowMode(RenderQueue.ShadowMode.Receive);
        geometry.setUserData("touch", true);

        BoxCollisionShape shape = new BoxCollisionShape(halfExtent);
        float mass = PhysicsRigidBody.massForStatic;
        RigidBodyControl rbc = new RigidBodyControl(shape, mass);
        rbc.setApplyScale(true);
        rbc.setPhysicsSpace(physicsSpace);
        geometry.addControl(rbc);
    }

    /**
     * Add an X-Z grid to the scene.
     */
    private void addGrid() {
        int gridColumns = 10;
        int gridRows = 10;
        float gridSpacing = 0.2f; // mesh units
        Mesh mesh = new Grid(gridColumns + 1, gridRows + 1, gridSpacing);
        Geometry geometry = new Geometry("grid", mesh);
        rootNode.attachChild(geometry);
        geometry.rotate(0f, 0f, FastMath.HALF_PI);

        ColorRGBA color = new ColorRGBA(0f, 0f, 0f, 1f);
        Material mat = MyAsset.createWireframeMaterial(assetManager, color);
        geometry.setMaterial(mat);
        /*
         * Translate the origin of the grid.
         */
        float bottomY = 0.5f;
        float gridX = -1f;
        float leftZ = -1f;
        geometry.move(gridX, bottomY, leftZ);

        float rightZ = leftZ + gridSpacing * gridColumns;
        float topY = bottomY + gridSpacing * gridRows;
        gridBottomLeft = new Vector3f(gridX, bottomY, leftZ);
        gridBottomRight = new Vector3f(gridX, bottomY, rightZ);
        gridTopLeft = new Vector3f(gridX, topY, leftZ);
        gridTopRight = new Vector3f(gridX, topY, rightZ);
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
                = new DirectionalLightShadowRenderer(assetManager, 4_096, 3);
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
        }
        dacReadyInitDone = false;
        xOffset = 0f;
        chainLength = 6;

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

        List<Spatial> list
                = MySpatial.listSpatials(cgModel, Spatial.class, null);
        for (Spatial spatial : list) {
            spatial.setShadowMode(RenderQueue.ShadowMode.Cast);
        }
        cgModel.setCullHint(Spatial.CullHint.Never);

        rootNode.attachChild(cgModel);
        setHeight(cgModel, height);
        center(cgModel);
        cgModel.move(xOffset, 0f, 0f);

        List<SkeletonControl> scList
                = MySpatial.listControls(cgModel, SkeletonControl.class, null);
        assert scList.size() == 1;
        sc = scList.get(0);
        Spatial controlledSpatial = sc.getSpatial();

        controlledSpatial.addControl(dac);
        dac.setPhysicsSpace(physicsSpace);

        sv = new SkeletonVisualizer(assetManager, sc);
        sv.setLineColor(ColorRGBA.Yellow); // TODO clean up visualization
        rootNode.addControl(sv);
    }

    /**
     * Translate a model's center so that the model rests on the X-Z plane, and
     * its center lies on the Y axis.
     */
    private void center(Spatial model) {
        Vector3f[] minMax = MySpatial.findMinMaxCoords(model);
        Vector3f center = MyVector3f.midpoint(minMax[0], minMax[1]);
        Vector3f offset = new Vector3f(center.x, minMax[0].y, center.z);

        Vector3f location = model.getWorldTranslation();
        location.subtractLocal(offset);
        MySpatial.setWorldLocation(model, location);
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(4f);

        cam.setLocation(new Vector3f(-3.3f, 1.5f, -0.1f));
        cam.setRotation(new Quaternion(0.08f, 0.687f, -0.06f, 0.72f));

        CameraOrbitAppState orbitState
                = new CameraOrbitAppState(cam, "orbitLeft", "orbitRight");
        stateManager.attach(orbitState);
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        CollisionShape.setDefaultMargin(0.005f); // 5 mm margin

        bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);

        physicsSpace = bulletAppState.getPhysicsSpace();
        physicsSpace.setAccuracy(0.033f); // 33 msec timestep
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
     * Process a "dump scenes" action.
     */
    private void dumpScenes() {
        PhysicsDumper dumper = new PhysicsDumper();
        //dumper.setDumpBucket(true);
        //dumper.setDumpCull(true);
        //dumper.setDumpMatParam(true);
        //dumper.setDumpOverride(true);
        //dumper.setDumpShadow(true);
        dumper.setDumpTransform(true);
        //dumper.setDumpUser(true);
        dumper.dump(renderManager);
    }

    /**
     * Initialization that takes place once all links are ready for dynamic
     * mode.
     */
    private void initWhenReady() {
        /*
         * Touch the tip vertex to the target.
         */
        Vector3f pivot = new Vector3f();
        tipLink = dac.findManagerForVertex(tipSpec, null, pivot);
        dac.setMass(tipLink, 100f);
        dac.setDynamicChain(tipLink, chainLength, Vector3f.ZERO, false);
        tipJoint = dac.moveToBody(tipLink, pivot, targetBody, Vector3f.ZERO);
        tipJoint.setEnabled(false);
        /*
         * The face tracks the target.
         */
        PhysicsLink noseLink = dac.findManagerForVertex(noseSpec, null, pivot);
        noseLink.setDynamic(Vector3f.ZERO);
        watch = new TrackController(noseLink, pivot, faceDirection, targetBody);
        noseLink.addIKController(watch);
        watch.setDeltaGainFactor(0.2f);
        watch.setEnabled(false);
        watch.setErrorGainFactor(0.1f);
        /*
             * If the model's eyes are animated, each eye tracks the target.
         */
        if (dac instanceof Binocular) {
            Binocular binocular = (Binocular) dac;
            /*
             * left eye
             */
            String spec = binocular.leftPupilSpec();
            PhysicsLink link = dac.findManagerForVertex(spec, null, pivot);
            link.setDynamic(Vector3f.ZERO);
            Vector3f lookDirection = binocular.leftEyeLookDirection(null);
            leftWatch = new TrackController(link, pivot, lookDirection,
                    targetBody);
            link.addIKController(leftWatch);
            leftWatch.setDeltaGainFactor(5f);
            leftWatch.setEnabled(false);
            leftWatch.setErrorGainFactor(0.1f);
            /*
             * right eye
             */
            spec = binocular.rightPupilSpec();
            link = dac.findManagerForVertex(spec, null, pivot);
            link.setDynamic(Vector3f.ZERO);
            binocular.leftEyeLookDirection(lookDirection);
            rightWatch = new TrackController(link, pivot, lookDirection,
                    targetBody);
            link.addIKController(rightWatch);
            rightWatch.setDeltaGainFactor(5f);
            rightWatch.setEnabled(false);
            rightWatch.setErrorGainFactor(0.1f);
        }
    }

    /**
     * Load the Jaime model.
     */
    private void loadJaime() {
        cgModel = (Node) assetManager.loadModel("Models/Jaime/Jaime.j3o");
        Geometry g = (Geometry) cgModel.getChild(0);
        RenderState rs = g.getMaterial().getAdditionalRenderState();
        rs.setFaceCullMode(RenderState.FaceCullMode.Off);
        cgModel.rotate(0f, -1.6f, 0f);

        dac = new JaimeControl();
        faceDirection = Vector3f.UNIT_Z;
        height = 2f;
        noseSpec = "122/JaimeGeom-geom-1"; // bridge of nose
        tipSpec = "2704/JaimeGeom-geom-1"; // tip of right index finger
        //xOffset = 1f;
    }

    /**
     * Load the MhGame model.
     */
    private void loadMhGame() {
        cgModel = (Node) assetManager.loadModel(
                "Models/MhGame/MhGame.mesh.xml");
        cgModel.rotate(0f, -1.6f, 0f);
        dac = new MhGameControl();
        faceDirection = Vector3f.UNIT_Z;
        height = 3f;
        noseSpec = "9313/male_generic"; // bridge of nose
        tipSpec = "5239/male_generic"; // tip of right index finger
    }

    /**
     * Load the Ninja model.
     */
    private void loadNinja() {
        cgModel = (Node) assetManager.loadModel("Models/Ninja/Ninja.mesh.xml");
        cgModel.rotate(0f, 1.6f, 0f);
        dac = new NinjaControl();
        faceDirection = new Vector3f(0f, 0f, -1f);
        height = 2f;
        noseSpec = "277/Ninja-geom-1"; // bridge of nose
        tipSpec = "55/Ninja-geom-2"; // tip of katana blade
    }

    /**
     * Load the Oto model.
     */
    private void loadOto() {
        cgModel = (Node) assetManager.loadModel("Models/Oto/Oto.mesh.xml");
        cgModel.rotate(0f, -1.6f, 0f);
        dac = new OtoControl();
        height = 3f;
        noseSpec = "161/Oto-geom-1"; // bridge of nose
        tipSpec = "3236/Oto-geom-1"; // right knuckle
    }

    /**
     * Load the Puppet model.
     */
    private void loadPuppet() {
        cgModel = (Node) assetManager.loadModel("Models/Puppet/Puppet.j3o");
        cgModel.rotate(0f, -1.6f, 0f);
        dac = new PuppetControl();
        faceDirection = Vector3f.UNIT_Y;
        height = 3f;
        noseSpec = "140/Mesh.011_0"; // bridge of nose
        tipSpec = "3185/Mesh.009_0"; // tip of right index finger
    }

    /**
     * Load the Sinbad model without attachments.
     */
    private void loadSinbad() {
        cgModel = (Node) assetManager.loadModel(
                "Models/Sinbad/Sinbad.mesh.xml");
        cgModel.rotate(0f, -1.6f, 0f);
        dac = new SinbadControl();
        faceDirection = new Vector3f(0f, 1f, -3f).normalizeLocal();
        height = 3f;
        noseSpec = "1844/Sinbad-geom-2"; // bridge of nose
        tipSpec = "223/Sinbad-geom-2"; // tip of right index finger
    }

    /**
     * Load the Sinbad model with an attached sword.
     */
    private void loadSinbadWith1Sword() {
        cgModel = (Node) assetManager.loadModel(
                "Models/Sinbad/Sinbad.mesh.xml");
        cgModel.rotate(0f, -1.6f, 0f);

        Node sword = (Node) assetManager.loadModel(
                "Models/Sinbad/Sword.mesh.xml");
        List<Spatial> list
                = MySpatial.listSpatials(sword, Spatial.class, null);
        for (Spatial spatial : list) {
            spatial.setShadowMode(RenderQueue.ShadowMode.Cast);
        }

        LinkConfig swordConfig = new LinkConfig(5f, MassHeuristic.Density,
                ShapeHeuristic.VertexHull, Vector3f.UNIT_XYZ,
                CenterHeuristic.AABB);
        dac = new SinbadControl();
        dac.attach("Handle.R", swordConfig, sword);

        faceDirection = new Vector3f(0f, 1f, -3f).normalizeLocal();
        height = 2f;
        noseSpec = "1844/Sinbad-geom-2"; // bridge of nose
        tipSpec = "4/Sword-geom-3/Handle.R"; // tip of sword blade
    }

    /**
     * Load the Sinbad model with 2 attached swords.
     */
    private void loadSinbadWithSwords() {
        cgModel = (Node) assetManager.loadModel(
                "Models/Sinbad/Sinbad.mesh.xml");
        cgModel.rotate(0f, -1.6f, 0f);

        Node sword = (Node) assetManager.loadModel(
                "Models/Sinbad/Sword.mesh.xml");
        List<Spatial> list
                = MySpatial.listSpatials(sword, Spatial.class, null);
        for (Spatial spatial : list) {
            spatial.setShadowMode(RenderQueue.ShadowMode.Cast);
        }

        LinkConfig swordConfig = new LinkConfig(5f, MassHeuristic.Density,
                ShapeHeuristic.VertexHull, Vector3f.UNIT_XYZ,
                CenterHeuristic.AABB);
        dac = new SinbadControl();
        dac.attach("Handle.L", swordConfig, sword);
        dac.attach("Handle.R", swordConfig, sword);

        faceDirection = new Vector3f(0f, 1f, -3f).normalizeLocal();
        noseSpec = "1844/Sinbad-geom-2"; // bridge of nose
        tipSpec = "4/Sword-geom-3/Handle.R"; // tip of right-hand sword blade
        height = 2f;
    }

    /**
     * Scale the specified model uniformly so that it has the specified height.
     *
     * @param model (not null, modified)
     * @param height (in world units)
     */
    private void setHeight(Spatial model, float height) {
        Vector3f[] minMax = MySpatial.findMinMaxCoords(model);
        float oldHeight = minMax[1].y - minMax[0].y;

        model.scale(height / oldHeight);
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

    /**
     * If the mouse is picking a grid location, enable all track controllers and
     * move the target there.
     */
    private void track() {
        Ray ray = MyCamera.mouseRay(cam, inputManager);

        Vector3f location = new Vector3f();
        boolean isTopLeft = ray.intersectWhere(gridTopLeft,
                gridTopRight, gridBottomLeft, location);
        boolean isBottomRight = ray.intersectWhere(gridBottomRight,
                gridTopRight, gridBottomLeft, location);
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
