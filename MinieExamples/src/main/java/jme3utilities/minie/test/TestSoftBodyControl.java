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

import com.jme3.app.Application;
import com.jme3.app.state.AppState;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.SoftPhysicsAppState;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.control.SoftBodyControl;
import com.jme3.bullet.debug.DebugInitListener;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.bullet.objects.infos.Sbcp;
import com.jme3.bullet.objects.infos.SoftBodyConfig;
import com.jme3.input.CameraInput;
import com.jme3.input.KeyInput;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.shadow.DirectionalLightShadowRenderer;
import com.jme3.system.AppSettings;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MyCamera;
import jme3utilities.MyString;
import jme3utilities.minie.DumpFlags;
import jme3utilities.minie.FilterAll;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.minie.test.common.PhysicsDemo;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.InputMode;

/**
 * Test/demonstrate the SoftBodyControl class.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestSoftBodyControl
        extends PhysicsDemo
        implements DebugInitListener {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(TestSoftBodyControl.class.getName());
    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName
            = TestSoftBodyControl.class.getSimpleName();
    // *************************************************************************
    // fields

    /**
     * physics objects that are not to be visualized
     */
    final private static FilterAll hiddenObjects = new FilterAll(true);
    /**
     * AppState to manage the PhysicsSpace
     */
    private static SoftPhysicsAppState bulletAppState;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the TestSoftBodyControl application.
     */
    public TestSoftBodyControl() { // to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the TestSoftBodyControl application.
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
        settings.setVSync(false);

        Application application = new TestSoftBodyControl();
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
        addLighting(rootNode, false);

        float halfExtent = 4f;
        float topY = 0f;
        attachCubePlatform(halfExtent, topY);

        addRubberDuck();
    }

    /**
     * Configure the PhysicsDumper during startup.
     */
    @Override
    public void configureDumper() {
        PhysicsDumper dumper = getDumper();
        dumper.setEnabled(DumpFlags.MatParams, true);
        dumper.setEnabled(DumpFlags.ShadowModes, true);
        dumper.setEnabled(DumpFlags.Transforms, true);
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
        return 1f;
    }

    /**
     * Add application-specific hotkey bindings (and override existing ones, if
     * necessary).
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        dim.bind(asDumpSpace, KeyInput.KEY_O);
        dim.bind(asDumpViewport, KeyInput.KEY_P);

        dim.bindSignal(CameraInput.FLYCAM_LOWER, KeyInput.KEY_DOWN);
        dim.bindSignal(CameraInput.FLYCAM_RISE, KeyInput.KEY_UP);
        dim.bindSignal("orbitLeft", KeyInput.KEY_LEFT);
        dim.bindSignal("orbitRight", KeyInput.KEY_RIGHT);

        dim.bind("test monkeyHead", KeyInput.KEY_F1);
        dim.bind("test rubberDuck", KeyInput.KEY_F2);
        dim.bind(asToggleHelp, KeyInput.KEY_H);
        dim.bind(asTogglePause, KeyInput.KEY_PAUSE, KeyInput.KEY_PERIOD);
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
        float halfExtent = 4f;
        float topY = 0f;
        if (ongoing) {
            switch (actionString) {
                case "test monkeyHead":
                    cleanupAfterTest();
                    attachCubePlatform(halfExtent, topY);
                    addMonkeyHead();
                    return;

                case "test rubberDuck":
                    cleanupAfterTest();
                    attachCubePlatform(halfExtent, topY);
                    addRubberDuck();
                    return;

                default:
            }
        }
        super.onAction(actionString, ongoing, tpf);
    }
    // *************************************************************************
    // DebugInitListener methods

    /**
     * Callback from BulletDebugAppState, invoked just before the debug scene is
     * added to the debug viewports.
     *
     * @param physicsDebugRootNode the root node of the debug scene (not null)
     */
    @Override
    public void bulletDebugInit(Node physicsDebugRootNode) {
        addLighting(physicsDebugRootNode, true);
    }
    // *************************************************************************
    // private methods

    /**
     * Add lighting and shadows to the specified scene.
     *
     * @param rootSpatial which scene (not null)
     * @param shadowFlag if true, add a shadow renderer to the default viewport
     */
    private void addLighting(Spatial rootSpatial, boolean shadowFlag) {
        ColorRGBA ambientColor = new ColorRGBA(0.1f, 0.1f, 0.1f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        rootSpatial.addLight(ambient);
        ambient.setName("ambient");

        ColorRGBA directColor = new ColorRGBA(0.7f, 0.7f, 0.7f, 1f);
        Vector3f direction = new Vector3f(1f, -2f, -1f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction, directColor);
        rootSpatial.addLight(sun);
        sun.setName("sun");

        rootSpatial.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);
        if (shadowFlag) {
            int mapSize = 2_048; // in pixels
            int numSplits = 3;
            DirectionalLightShadowRenderer dlsr
                    = new DirectionalLightShadowRenderer(
                            assetManager, mapSize, numSplits);
            dlsr.setLight(sun);
            dlsr.setShadowIntensity(0.5f);
            viewPort.addProcessor(dlsr);
        }
    }

    /**
     * Add a monkey head to the scene.
     */
    private void addMonkeyHead() {
        Spatial cgModel
                = assetManager.loadModel("Models/MonkeyHead/MonkeyHead.j3o");
        cgModel = Heart.deepCopy(cgModel); // clone vertex buffers
        rootNode.attachChild(cgModel);

        SoftBodyControl sbc = new SoftBodyControl();
        cgModel.addControl(sbc);
        PhysicsSoftBody psb = sbc.getBody();

        psb.applyScale(new Vector3f(0.6f, 0.6f, 0.6f));
        psb.applyTranslation(new Vector3f(0f, 1.4f, 0f));

        float totalMass = 1f;
        psb.setMassByArea(totalMass);

        SoftBodyConfig config = psb.getSoftConfig();
        config.set(Sbcp.KineticHardness, 1f);
        config.set(Sbcp.PoseMatching, 0.03f);

        boolean setVolumePose = false;
        boolean setFramePose = true;
        psb.setPose(setVolumePose, setFramePose);

        PhysicsSpace space = getPhysicsSpace();
        sbc.setPhysicsSpace(space);
        hiddenObjects.addException(sbc);
    }

    /**
     * Add a rubber duck to the scene.
     */
    private void addRubberDuck() {
        Spatial cgModel = assetManager.loadModel("Models/Duck/Duck.j3o");
        rootNode.attachChild(cgModel);

        SoftBodyControl sbc = new SoftBodyControl();
        cgModel.addControl(sbc);
        PhysicsSoftBody psb = sbc.getBody();

        float totalMass = 1f;
        psb.setMassByArea(totalMass);

        SoftBodyConfig config = psb.getSoftConfig();
        config.set(Sbcp.KineticHardness, 1f);
        config.set(Sbcp.PoseMatching, 0.03f);
        //config.setPositionIterations(15);

        boolean setVolumePose = false;
        boolean setFramePose = true;
        psb.setPose(setVolumePose, setFramePose);

        psb.applyRotation(new Quaternion().fromAngles(0.4f, 0f, 1f));
        psb.applyTranslation(new Vector3f(0f, 1.2f, 0f));

        PhysicsSpace space = getPhysicsSpace();
        sbc.setPhysicsSpace(space);
        hiddenObjects.addException(sbc);
    }

    /**
     * Clean up after a test.
     */
    private void cleanupAfterTest() {
        // Remove any scenery. Debug meshes are under a different root node.
        rootNode.detachAllChildren();

        // Remove physics objects, which also removes their debug meshes.
        PhysicsSpace physicsSpace = getPhysicsSpace();
        physicsSpace.destroy();
        assert physicsSpace.isEmpty();

        // Clear the hidden-object list.
        hiddenObjects.clearExceptions();
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        float yDegrees = MyCamera.yDegrees(cam);
        float aspectRatio = MyCamera.viewAspectRatio(cam);
        float near = 0.02f;
        float far = 20f;
        cam.setFrustumPerspective(yDegrees, aspectRatio, near, far);

        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(2f);
        flyCam.setZoomSpeed(2f);

        cam.setLocation(new Vector3f(-1.7f, 0.5f, 4.4f));
        cam.setRotation(new Quaternion(-0.0065f, 0.977669f, 0.0283f, 0.20814f));

        AppState orbitState
                = new CameraOrbitAppState(cam, "orbitLeft", "orbitRight");
        stateManager.attach(orbitState);
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        CollisionShape.setDefaultMargin(0.005f); // 5-mm margin

        bulletAppState = new SoftPhysicsAppState();
        bulletAppState.setDebugEnabled(true);
        bulletAppState.setDebugFilter(hiddenObjects);
        bulletAppState.setDebugInitListener(this);
        stateManager.attach(bulletAppState);

        setGravityAll(1f);
    }
}
