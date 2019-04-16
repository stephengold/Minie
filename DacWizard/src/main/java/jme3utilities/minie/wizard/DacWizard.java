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
package jme3utilities.minie.wizard;

import com.jme3.app.DebugKeysAppState;
import com.jme3.app.state.AppState;
import com.jme3.app.state.AppStateManager;
import com.jme3.audio.openal.ALAudioRenderer;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.animation.DynamicAnimControl;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Spatial;
import com.jme3.system.AppSettings;
import com.jme3.system.JmeVersion;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Misc;
import jme3utilities.MyCamera;
import jme3utilities.MySpatial;
import jme3utilities.MyString;
import jme3utilities.math.MyVector3f;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.nifty.GuiApplication;
import jme3utilities.nifty.bind.BindScreen;
import jme3utilities.nifty.displaysettings.DsScreen;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.DisplaySettings;
import jme3utilities.ui.DisplaySizeLimits;
import jme3utilities.ui.InputMode;

/**
 * A GuiApplication to configure a DynamicAnimControl for a new C-G model. The
 * application's main entry point is in this class.
 * <p>
 * As seen in the April 2019 walkthru video:
 * https://www.youtube.com/watch?v=iWyrzZe45jA
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class DacWizard extends GuiApplication {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final static Logger logger = Logger.getLogger(DacWizard.class.getName());
    // *************************************************************************
    // fields

    /**
     * true once {@link #startup1()} has completed, until then false
     */
    private boolean didStartup1 = false;
    /**
     * application instance
     */
    private static DacWizard application;
    /**
     * Nifty screen for editing display settings
     */
    private static DsScreen displaySettingsScreen;
    /**
     * subject computer-graphics model
     */
    final private static Model model = new Model();
    /**
     * dumper for scene dumps
     */
    final static PhysicsDumper dumper = new PhysicsDumper();
    // *************************************************************************
    // new methods exposed

    /**
     * Clear the scene.
     */
    void clearScene() {
        BulletAppState bulletAppState
                = DacWizard.findAppState(BulletAppState.class);
        PhysicsSpace physicsSpace = bulletAppState.getPhysicsSpace();
        DynamicAnimControl ragdoll = findDac();
        if (ragdoll != null) {
            physicsSpace.remove(ragdoll);
        }
        assert physicsSpace.isEmpty();

        rootNode.setCullHint(Spatial.CullHint.Never);
        List<Spatial> children = rootNode.getChildren();
        for (Spatial child : children) {
            child.removeFromParent();
        }
    }

    /**
     * Find the 1st attached AppState that's an instance of the specified class.
     *
     * @param <T>
     * @param subclass the kind of AppState to search for (not null)
     * @return the pre-existing instance (not null)
     */
    static <T extends AppState> T findAppState(Class<T> subclass) {
        AppStateManager manager = application.getStateManager();
        T appState = manager.getState(subclass);

        assert appState != null;
        return appState;
    }

    /**
     * Find the DynamicAnimControl in the scene.
     *
     * @return the pre-existing control, or null if none/multiple
     */
    DynamicAnimControl findDac() {
        DynamicAnimControl result = null;
        if (rootNode != null) {
            List<DynamicAnimControl> controls = MySpatial.listControls(
                    rootNode, DynamicAnimControl.class, null);
            if (controls.size() == 1) {
                result = controls.get(0);
            }
        }

        return result;
    }

    /**
     * Access the application instance from a static context.
     *
     * @return the pre-existing instance (not null)
     */
    static DacWizard getApplication() {
        assert application != null;
        return application;
    }

    /**
     * Access the subject C-G model.
     *
     * @return the pre-existing instance (not null)
     */
    static Model getModel() {
        assert model != null;
        return model;
    }

    /**
     * Main entry point for the DacWizard application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        /*
         * Mute the chatty loggers found in certain packages.
         */
        Misc.setLoggingLevels(Level.WARNING);
        Logger.getLogger(ALAudioRenderer.class.getName())
                .setLevel(Level.SEVERE);

        String renderer = AppSettings.LWJGL_OPENGL2;
        boolean forceDialog = false;
        /*
         * Process any command-line arguments.
         */
        for (String arg : arguments) {
            switch (arg) {
                case "-3":
                case "--openGL3":
                    renderer = AppSettings.LWJGL_OPENGL3;
                    break;

                case "-f":
                case "--forceDialog":
                    forceDialog = true;
                    break;

                case "-v":
                case "--verbose":
                    Misc.setLoggingLevels(Level.INFO);
                    break;

                default:
                    logger.log(Level.WARNING,
                            "Unknown command-line argument {0}",
                            MyString.quote(arg));
            }
        }

        mainStartup(forceDialog, renderer);
    }

    /**
     * Add a C-G model to the scene and reset the camera.
     *
     * @param cgModel (not null, alias created)
     */
    void makeScene(Spatial cgModel) {
        assert cgModel != null;

        rootNode.attachChild(cgModel);
        float height = 2f;
        setHeight(cgModel, height);
        center(cgModel);
        resetCamera();
    }
    // *************************************************************************
    // GuiApplication methods

    /**
     * Initialize this application.
     */
    @Override
    public void guiInitializeApplication() {
        logger.info("");

        if (!Misc.areAssertionsEnabled()) {
            String message = "Assertions are disabled!";
            logger.warning(message);
        }
        /*
         * Log the jMonkeyEngine version string.
         */
        logger.log(Level.INFO, "jme3-core version is {0}",
                MyString.quote(JmeVersion.FULL_NAME));
        /*
         * Log the jme3-utilities-heart version string.
         */
        logger.log(Level.INFO, "jme3-utilities-heart version is {0}",
                MyString.quote(Misc.versionShort()));
        /*
         * Detach an app state created by SimpleApplication.
         */
        DebugKeysAppState debugKeys = findAppState(DebugKeysAppState.class);
        stateManager.detach(debugKeys);

        addLighting();
        getSignals().add("orbitLeft");
        getSignals().add("orbitRight");

        attachAppStates();
    }

    /**
     * Process an action from the GUI or keyboard that wasn't handled by the
     * active InputMode.
     *
     * @param actionString textual description of the action (not null)
     * @param ongoing true if the action is ongoing, otherwise false
     * @param tpf time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void onAction(String actionString, boolean ongoing, float tpf) {
        logger.log(Level.INFO, "Got action {0} ongoing={1}", new Object[]{
            MyString.quote(actionString), ongoing
        });

        boolean handled = false;
        if (ongoing) {
            handled = Action.processOngoing(actionString);
        }
        if (!handled) {
            super.onAction(actionString, ongoing, tpf);
        }
    }

    /**
     * Callback invoked once per frame.
     *
     * @param tpf time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void simpleUpdate(float tpf) {
        super.simpleUpdate(tpf);

        if (!didStartup1) {
            startup1();
            didStartup1 = true;
        }
        model.pollForTaskCompletion();
    }
    // *************************************************************************
    // private methods

    /**
     * Add lighting and background color to the scene.
     */
    private void addLighting() {
        ColorRGBA ambientColor = new ColorRGBA(1f, 1f, 1f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        rootNode.addLight(ambient);
        ambient.setName("ambient");

        Vector3f direction = new Vector3f(1f, -2f, -1f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction);
        rootNode.addLight(sun);
        sun.setName("sun");

        ColorRGBA backgroundColor = new ColorRGBA(0.3f, 0.3f, 0.3f, 1f);
        viewPort.setBackgroundColor(backgroundColor);
    }

    /**
     * Attach app states during initialization.
     */
    private void attachAppStates() {
        boolean success;

        success = stateManager.attach(new BindScreen());
        assert success;

        success = stateManager.attach(new BulletAppState());
        assert success;
        CollisionShape.setDefaultMargin(0.001f); // 1-mm margin

        success = stateManager.attach(displaySettingsScreen);
        assert success;

        CameraOrbitAppState cameraOrbitAppState
                = new CameraOrbitAppState(cam, "orbitLeft", "orbitRight");
        stateManager.attach(cameraOrbitAppState);
        /*
         * Create and attach an input mode and screen controller
         * for the "filePath" screen.
         */
        success = stateManager.attach(new FilePathMode());
        assert success;
        success = stateManager.attach(new FilePathScreen());
        assert success;
        /*
         * Create and attach an input mode and screen controller
         * for the "load" screen.
         */
        success = stateManager.attach(new LoadMode());
        assert success;
        success = stateManager.attach(new LoadScreen());
        assert success;
        /*
         * Create and attach an input mode and screen controller
         * for the "bones" screen.
         */
        success = stateManager.attach(new BonesMode());
        assert success;
        success = stateManager.attach(new BonesScreen());
        assert success;
        /*
         * Create and attach an input mode and screen controller
         * for the "links" screen.
         */
        success = stateManager.attach(new LinksMode());
        assert success;
        success = stateManager.attach(new LinksScreen());
        assert success;
        /*
         * Create and attach an input mode and screen controller
         * for the "test" screen.
         */
        success = stateManager.attach(new TestMode());
        assert success;
        success = stateManager.attach(new TestScreen());
        assert success;
    }

    /**
     * Translate a model's center so that the model rests on the X-Z plane, and
     * its center lies on the Y axis.
     */
    private void center(Spatial model) {
        Vector3f[] minMax = MySpatial.findMinMaxCoords(model);
        Vector3f center = MyVector3f.midpoint(minMax[0], minMax[1], null);
        Vector3f offset = new Vector3f(center.x, minMax[0].y, center.z);

        Vector3f location = model.getWorldTranslation();
        location.subtractLocal(offset);
        MySpatial.setWorldLocation(model, location);
    }

    /**
     * Initialization performed immediately after parsing the command-line
     * arguments.
     *
     * @param forceDialog true&rarr;force startup to show the JME settings
     * dialog, false&rarr; show the dialog only if persistent settings are
     * missing
     * @param renderer the value passed to
     * {@link com.jme3.system.AppSettings#setRenderer(java.lang.String)}
     */
    private static void mainStartup(boolean forceDialog, final String renderer) {
        /*
         * Instantiate the application.
         */
        application = new DacWizard();
        /*
         * Instantiate the display-settings screen.
         */
        String applicationName = "DacWizard";
        DisplaySizeLimits dsl = new DisplaySizeLimits(
                640, 480, // min width, height
                2_048, 1_080 // max width, height
        );
        DisplaySettings displaySettings
                = new DisplaySettings(application, applicationName, dsl) {
            @Override
            protected void applyOverrides(AppSettings settings) {
                super.applyOverrides(settings);

                setForceDialog(forceDialog);
                settings.setRenderer(renderer);
                settings.setSamples(4);
                settings.setVSync(true);
            }
        };
        displaySettingsScreen = new DsScreen(displaySettings);

        AppSettings appSettings = displaySettings.initialize();
        if (appSettings != null) {
            application.setSettings(appSettings);
            /*
             * Don't pause on lost focus.  This simplifies debugging by
             * permitting the application to run while minimized.
             */
            application.setPauseOnLostFocus(false);
            /*
             * If the settings dialog should be shown, it has already been shown
             * by DisplaySettings.initialize().
             */
            application.setShowSettings(false);
            application.start();
            /*
             * ... and onward to DacWizard.guiInitializeApplication()!
             */
        }
    }

    /**
     * Reset the camera.
     */
    private void resetCamera() {
        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(4f);

        cam.setLocation(new Vector3f(0f, 1.8f, 5f));
        cam.setName("cam");
        cam.setRotation(new Quaternion(0f, 0.9985813f, -0.05f, 0.0175f));
        MyCamera.setNearFar(cam, 0.02f, 50f);
    }

    /**
     * Scale the specified model uniformly so that it has the specified height.
     *
     * @param model (not null, modified)
     * @param height (in world units)
     */
    private void setHeight(Spatial model, float height) {
        Vector3f[] minMax = MySpatial.findMinMaxCoords(model);
        Vector3f min = minMax[0];
        Vector3f max = minMax[1];
        float oldHeight = max.y - min.y;

        model.scale(height / oldHeight);
    }

    /**
     * Initialization performed during the 1st invocation of
     * {@link #simpleUpdate(float)}.
     */
    private void startup1() {
        logger.info("");
        /*
         * Disable the JME statistic displays.
         */
        setDisplayFps(false);
        setDisplayStatView(false);
        /*
         * Enable the initial InputMode.
         */
        InputMode.getActiveMode().setEnabled(false);
        InputMode initialInputMode = InputMode.findMode("filePath");
        initialInputMode.setEnabled(true);
    }
}
