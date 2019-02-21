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

import com.jme3.animation.AnimChannel;
import com.jme3.animation.AnimControl;
import com.jme3.animation.SkeletonControl;
import com.jme3.app.StatsAppState;
import com.jme3.audio.openal.ALAudioRenderer;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.animation.BoneLink;
import com.jme3.bullet.animation.CenterHeuristic;
import com.jme3.bullet.animation.DynamicAnimControl;
import com.jme3.bullet.animation.LinkConfig;
import com.jme3.bullet.animation.MassHeuristic;
import com.jme3.bullet.animation.ShapeHeuristic;
import com.jme3.bullet.animation.TorsoLink;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.input.KeyInput;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
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
import jme3utilities.MySpatial;
import jme3utilities.debug.PointVisualizer;
import jme3utilities.debug.SkeletonVisualizer;
import jme3utilities.math.MyArray;
import jme3utilities.math.MyVector3f;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.minie.test.tunings.Biped;
import jme3utilities.minie.test.tunings.JaimeControl;
import jme3utilities.minie.test.tunings.MhGameControl;
import jme3utilities.minie.test.tunings.NinjaControl;
import jme3utilities.minie.test.tunings.OtoControl;
import jme3utilities.minie.test.tunings.PuppetControl;
import jme3utilities.minie.test.tunings.SinbadControl;
import jme3utilities.ui.ActionApplication;
import jme3utilities.ui.InputMode;

/**
 * Demo/testbed for BalanceController inverse kinematics.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class BalanceDemo extends ActionApplication {
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

    private AnimChannel animChannel = null;
    private BalanceController balance = null;
    private boolean dacReadyInitDone = false;
    private BulletAppState bulletAppState;
    /**
     * control being tested
     */
    private DynamicAnimControl dac;
    private float rightSupportFraction = 0.5f;
    /**
     * parameters that determine the torso's vertical acceleration
     */
    private float vaBias = 0f;
    private float vaMagnitude = 0f;
    private float vaSign = +1f;
    /**
     * C-G model on which the control is being tested
     */
    private Node cgModel;
    /**
     * space for physics simulation
     */
    private PhysicsSpace physicsSpace;
    /**
     * visualizer for the center of mass
     */
    private PointVisualizer comPoint;
    /**
     * visualizer for the center of support
     */
    private PointVisualizer supportPoint;

    private SkeletonControl sc;
    /**
     * visualizer for the skeleton of the C-G model
     */
    private SkeletonVisualizer sv;
    private String animationName = null;
    private TorsoLink torso;
    private Vector3f leftSupportLocation;
    private Vector3f rightSupportLocation;
    private Vector3f torsoUpDirection = null;
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

        BalanceDemo application = new BalanceDemo();
        /*
         * Customize the window's title bar.
         */
        AppSettings settings = new AppSettings(true);
        settings.setTitle(applicationName);

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

        comPoint = new PointVisualizer(assetManager, 16, ColorRGBA.Cyan,
                "ring");
        rootNode.attachChild(comPoint);

        supportPoint = new PointVisualizer(assetManager, 16, ColorRGBA.Yellow,
                "square");
        rootNode.attachChild(supportPoint);

        addBox();
        addModel("Sinbad");
    }

    /**
     * Add application-specific hotkey bindings and override existing ones.
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        dim.bind("dump physicsSpace", KeyInput.KEY_O);
        dim.bind("dump scenes", KeyInput.KEY_P);
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
        dim.bind("signal orbitLeft", KeyInput.KEY_LEFT);
        dim.bind("signal orbitRight", KeyInput.KEY_RIGHT);

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

                case "go limp":
                    if (dac.isReady()) {
                        dac.setRagdollMode();
                    }
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
            } else if (words.length == 3 && "posture".equals(words[0])) {
                setPosture(words[1], words[2]);
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
     * Add a large static box to the scene, to serve as a platform.
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

        Vector3f hes = new Vector3f(halfExtent, halfExtent, halfExtent);
        BoxCollisionShape shape = new BoxCollisionShape(hes);
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

        List<Spatial> list
                = MySpatial.listSpatials(cgModel, Spatial.class, null);
        for (Spatial spatial : list) {
            spatial.setShadowMode(RenderQueue.ShadowMode.Cast);
        }
        cgModel.setCullHint(Spatial.CullHint.Never);

        rootNode.attachChild(cgModel);
        setHeight(cgModel, 2f);
        center(cgModel);

        List<SkeletonControl> scList
                = MySpatial.listControls(cgModel, SkeletonControl.class, null);
        assert scList.size() == 1;
        sc = scList.get(0);
        Spatial controlledSpatial = sc.getSpatial();

        controlledSpatial.addControl(dac);
        dac.setPhysicsSpace(physicsSpace);

        torso = dac.getTorsoLink();

        AnimControl animControl
                = controlledSpatial.getControl(AnimControl.class);
        animChannel = animControl.createChannel();

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
        CollisionShape.setDefaultMargin(0.005f); // 5-mm margin

        bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);

        physicsSpace = bulletAppState.getPhysicsSpace();
        physicsSpace.setAccuracy(1f / 30f); // 33.33-msec timestep
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
        Biped biped = (Biped) dac;
        BoneLink leftFoot = biped.getLeftFoot();
        BoneLink rightFoot = biped.getRightFoot();
        /*
         * Pin the left foot to the ground.
         */
        dac.setDynamicChain(leftFoot, 9, Vector3f.ZERO, false);
        Vector3f[] leftSole = leftFoot.footprint();
        for (Vector3f location : leftSole) {
            dac.pinToWorld(leftFoot, location);
        }
        /*
         * Pin the right foot to the ground.
         */
        dac.setDynamicChain(rightFoot, 9, Vector3f.ZERO, false);
        Vector3f[] rightSole = rightFoot.footprint();
        for (Vector3f location : rightSole) {
            dac.pinToWorld(rightFoot, location);
        }
        /*
         * Apply a BalanceController to the torso.
         */
        Vector3f acc = new Vector3f(0f, vaBias + vaSign * vaMagnitude, 0f);
        torso.setDynamic(acc);
        leftSupportLocation = MyArray.mean(leftSole, null);
        rightSupportLocation = MyArray.mean(rightSole, null);
        Vector3f supportLocation = MyVector3f.lerp(rightSupportFraction,
                leftSupportLocation, rightSupportLocation, null);
        balance = new BalanceController(torso, supportLocation);
        torso.addIKController(balance);
        /*
         * Apply an UprightController to the torso.
         */
        UprightController upright
                = new UprightController(torso, torsoUpDirection);
        torso.addIKController(upright);
        if (dac instanceof MhGameControl || dac instanceof PuppetControl) {
            upright.setDeltaGainFactor(0.3f);
            upright.setErrorGainFactor(0.3f);
        } else if (dac instanceof NinjaControl || dac instanceof SinbadControl) {
            upright.setDeltaGainFactor(1f);
            upright.setErrorGainFactor(1f);
        }
        /*
         * Start playing a canned animation.
         */
        animChannel.setAnim(animationName);
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
        animationName = "Punches";
        vaBias = 5f;
        vaMagnitude = 15f;
        torsoUpDirection = Vector3f.UNIT_Z;
    }

    /**
     * Load the MhGame model.
     */
    private void loadMhGame() {
        cgModel = (Node) assetManager.loadModel(
                "Models/MhGame/MhGame.mesh.xml");
        cgModel.rotate(0f, -1.6f, 0f);
        dac = new MhGameControl();
        animationName = "expr-lib-pose";
        vaMagnitude = 40f;
        torsoUpDirection = Vector3f.UNIT_X;
    }

    /**
     * Load the Ninja model.
     */
    private void loadNinja() {
        cgModel = (Node) assetManager.loadModel("Models/Ninja/Ninja.mesh.xml");
        cgModel.rotate(0f, 1.6f, 0f);
        dac = new NinjaControl();
        animationName = "Walk";
        vaBias = -8f;
        vaMagnitude = 12f;
        torsoUpDirection = Vector3f.UNIT_Y;
    }

    /**
     * Load the Oto model.
     */
    private void loadOto() {
        cgModel = (Node) assetManager.loadModel("Models/Oto/Oto.mesh.xml");
        cgModel.rotate(0f, -1.6f, 0f);
        dac = new OtoControl();
        animationName = "pull";
        vaMagnitude = 80f;
    }

    /**
     * Load the Puppet model.
     */
    private void loadPuppet() {
        cgModel = (Node) assetManager.loadModel("Models/Puppet/Puppet.j3o");
        cgModel.rotate(0f, -1.6f, 0f);
        dac = new PuppetControl();
        animationName = "wave";
        vaBias = 10f;
        vaMagnitude = 30f;
        torsoUpDirection = new Vector3f(0f, 0f, -1f);
    }

    /**
     * Load the Sinbad model without attachments.
     */
    private void loadSinbad() {
        cgModel = (Node) assetManager.loadModel(
                "Models/Sinbad/Sinbad.mesh.xml");
        cgModel.rotate(0f, -1.6f, 0f);
        dac = new SinbadControl();
        animationName = "Dance";
        vaMagnitude = 40f;
        torsoUpDirection = Vector3f.UNIT_Y;
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

        animationName = "RunTop";
        vaMagnitude = 40f;
        torsoUpDirection = Vector3f.UNIT_Y;
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

        animationName = "RunTop";
        vaMagnitude = 40f;
        torsoUpDirection = Vector3f.UNIT_Y;
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

    private void setPosture(String vertical, String side) {
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
}
