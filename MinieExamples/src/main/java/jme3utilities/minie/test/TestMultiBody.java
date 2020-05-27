/*
 Copyright (c) 2020, Stephen Gold
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
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.MultiBody;
import com.jme3.bullet.MultiBodyAppState;
import com.jme3.bullet.MultiBodyLink;
import com.jme3.bullet.MultiBodySpace;
import com.jme3.bullet.SolverType;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.font.BitmapText;
import com.jme3.font.Rectangle;
import com.jme3.input.CameraInput;
import com.jme3.input.KeyInput;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Spatial;
import com.jme3.shadow.DirectionalLightShadowRenderer;
import com.jme3.system.AppSettings;
import com.jme3.util.BufferUtils;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MyAsset;
import jme3utilities.MyCamera;
import jme3utilities.minie.test.common.AbstractDemo;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.InputMode;

/**
 * Demo/testbed for multi-body physics.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestMultiBody extends AbstractDemo {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(TestMultiBody.class.getName());
    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName
            = TestMultiBody.class.getSimpleName();
    // *************************************************************************
    // fields

    /**
     * text displayed in the upper-left corner of the GUI node
     */
    final private BitmapText[] statusLines = new BitmapText[1];
    /**
     * AppState to manage the MultiBodySpace
     */
    private MultiBodyAppState bulletAppState;
    /**
     * name of the test being run
     */
    private String testName = "test1";
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the TestSoftBody application.
     *
     * @param ignored array of command-line arguments (not null)
     */
    public static void main(String[] ignored) {
        /*
         * Mute the chatty loggers in certain packages.
         */
        Heart.setLoggingLevels(Level.WARNING);
        /*
         * Enable direct-memory tracking.
         */
        BufferUtils.setTrackDirectMemoryEnabled(true);

        Application application = new TestMultiBody();
        /*
         * Customize the window's title bar.
         */
        boolean loadDefaults = true;
        AppSettings settings = new AppSettings(loadDefaults);
        settings.setTitle(applicationName);

        settings.setAudioRenderer(null);
        settings.setGammaCorrection(true);
        settings.setSamples(16); // anti-aliasing
        settings.setVSync(false);
        application.setSettings(settings);

        application.start();
    }
    // *************************************************************************
    // AbstractDemo methods

    /**
     * Initialize this application.
     */
    @Override
    public void actionInitializeApplication() {
        configureCamera();
        configureDumper();
        generateMaterials();
        configurePhysics();

        ColorRGBA skyColor = new ColorRGBA(0.1f, 0.2f, 0.4f, 1f);
        viewPort.setBackgroundColor(skyColor);

        addLighting(rootNode, false);
        addStatusLines();
        speed = pausedSpeed;

        attachCubePlatform(4f, -1f);
        addMultiBody();
    }

    /**
     * Configure materials during startup.
     */
    @Override
    public void generateMaterials() {
        ColorRGBA gray = new ColorRGBA(0.05f, 0.05f, 0.05f, 1f);
        Material platform = MyAsset.createUnshadedMaterial(assetManager, gray);
        registerMaterial("platform", platform);
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

        dim.bind(AbstractDemo.asDumpScenes, KeyInput.KEY_P);
        dim.bind(AbstractDemo.asDumpSpace, KeyInput.KEY_O);

        dim.bind("signal " + CameraInput.FLYCAM_LOWER, KeyInput.KEY_DOWN);
        dim.bind("signal " + CameraInput.FLYCAM_RISE, KeyInput.KEY_UP);
        dim.bind("signal orbitLeft", KeyInput.KEY_LEFT);
        dim.bind("signal orbitRight", KeyInput.KEY_RIGHT);

        dim.bind("test test1", KeyInput.KEY_F1);

        dim.bind(AbstractDemo.asToggleAabbs, KeyInput.KEY_APOSTROPHE);
        dim.bind(AbstractDemo.asToggleHelp, KeyInput.KEY_H);
        dim.bind(AbstractDemo.asTogglePause, KeyInput.KEY_PAUSE);
        dim.bind(AbstractDemo.asTogglePause, KeyInput.KEY_PERIOD);
        dim.bind(AbstractDemo.asTogglePcoAxes, KeyInput.KEY_SEMICOLON);

        float x = 10f;
        float y = cam.getHeight() - 30f;
        float width = cam.getWidth() - 20f;
        float height = cam.getHeight() - 20f;
        Rectangle rectangle = new Rectangle(x, y, width, height);

        attachHelpNode(rectangle);
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
                case "test test1":
                    testName = "test1";
                    cleanupAfterTest();
                    attachCubePlatform(4f, -1f);
                    addMultiBody();
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
        updateStatusLines();
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
        Vector3f direction = new Vector3f(1f, -2f, -2f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction, directColor);
        rootSpatial.addLight(sun);
        sun.setName("sun");

        rootSpatial.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);
        if (shadowFlag) {
            DirectionalLightShadowRenderer dlsr
                    = new DirectionalLightShadowRenderer(assetManager, 2048, 3);
            dlsr.setLight(sun);
            dlsr.setShadowIntensity(0.5f);
            viewPort.addProcessor(dlsr);
        }
    }

    /**
     * Add a MultiBody to the scene.
     */
    private void addMultiBody() {
        int numLinks = 1;
        float linkMass = 1f;
        Vector3f inertia = new Vector3f(1f, 1f, 1f);
        boolean fixedBase = false;
        boolean canSleep = false;
        MultiBody multiBody = new MultiBody(numLinks, linkMass, inertia,
                fixedBase, canSleep);

        CollisionShape baseShape = new SphereCollisionShape(0.3f);
        multiBody.addBaseCollider(baseShape);

        MultiBodyLink parent = null;
        boolean disableCollision = false;
        Vector3f offset = new Vector3f(0f, -0.1f, 1f);
        MultiBodyLink link = multiBody.configureSphericalLink(linkMass, inertia,
                parent, Quaternion.IDENTITY, offset, offset, disableCollision);
        CollisionShape linkShape = new BoxCollisionShape(0.3f);
        link.addCollider(linkShape);

        getPhysicsSpace().add(multiBody);
    }

    /**
     * Add status lines to the GUI.
     */
    private void addStatusLines() {
        for (int lineIndex = 0; lineIndex < statusLines.length; ++lineIndex) {
            statusLines[lineIndex] = new BitmapText(guiFont, false);
            float y = cam.getHeight() - 20f * lineIndex;
            statusLines[lineIndex].setLocalTranslation(0f, y, 0f);
            guiNode.attachChild(statusLines[lineIndex]);
        }
    }

    /**
     * Clean up after a test.
     */
    private void cleanupAfterTest() {
        /*
         * Remove any scenery. Debug meshes are under a different root node.
         */
        rootNode.detachAllChildren();

        stateManager.detach(bulletAppState);
        configurePhysics();
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

        cam.setLocation(new Vector3f(3.778f, 2.2f, 0.971f));
        cam.setRotation(new Quaternion(0.2181f, -0.68631f, 0.2285f, 0.65513f));

        CameraOrbitAppState orbitState
                = new CameraOrbitAppState(cam, "orbitLeft", "orbitRight");
        stateManager.attach(orbitState);
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        bulletAppState = new MultiBodyAppState();
        bulletAppState.setDebugEnabled(true);
        bulletAppState.setSolverType(SolverType.Lemke);
        float axisLength = maxArrowLength();
        bulletAppState.setDebugAxisLength(axisLength);
        stateManager.attach(bulletAppState);

        MultiBodySpace physicsSpace = bulletAppState.getMultiBodySpace();
        physicsSpace.getSolverInfo().setGlobalCfm(0.1f); // for the Lemke solver
        setGravityAll(2f);
    }

    /**
     * Update the status lines in the GUI.
     */
    private void updateStatusLines() {
        String message = String.format(
                "Test: %s%s", testName, isPaused() ? "  PAUSED" : "");
        statusLines[0].setText(message);
    }
}
