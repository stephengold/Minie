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
package jme3utilities.minie.tuner;

import com.jme3.app.DebugKeysAppState;
import com.jme3.app.state.AppState;
import com.jme3.app.state.AppStateManager;
import com.jme3.asset.AssetManager;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.system.AppSettings;
import com.jme3.system.JmeVersion;
import java.io.IOException;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MyCamera;
import jme3utilities.MySpatial;
import jme3utilities.MyString;
import jme3utilities.ViewPortAppState;
import jme3utilities.debug.AxesVisualizer;
import jme3utilities.math.RectSizeLimits;
import jme3utilities.minie.DumpFlags;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.nifty.GuiApplication;
import jme3utilities.nifty.LibraryVersion;
import jme3utilities.nifty.bind.BindScreen;
import jme3utilities.nifty.displaysettings.DsScreen;
import jme3utilities.ui.ActionApplication;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.DisplaySettings;
import jme3utilities.ui.InputMode;
import jme3utilities.ui.ShowDialog;

/**
 * A GuiApplication to configure V-HACD for a C-G model. The application's main
 * entry point is in this class. The default scene graph and the physics spaces
 * are also managed here.
 * <p>
 * Seen in the August 2022 walkthru video:
 * https://www.youtube.com/watch?v=iEWJtPujmM8
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class VhacdTuner extends GuiApplication {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final static Logger logger = Logger.getLogger(VhacdTuner.class.getName());
    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName
            = VhacdTuner.class.getSimpleName();
    // *************************************************************************
    // fields

    /**
     * true once {@link #startup1()} has completed, until then false
     */
    private static boolean didStartup1 = false;
    /**
     * manage the physics space on the left side
     */
    private static BulletAppState leftBas;
    /**
     * manage the physics space on the right side
     */
    private static BulletAppState rightBas;
    /**
     * Nifty screen for editing display settings
     */
    private static DsScreen displaySettingsScreen;
    /**
     * state information
     */
    final private static Model model = new Model();
    /**
     * parent of the loaded C-G model in the scene
     */
    private Node cgmParent = null;
    /**
     * dump debugging information to {@code System.out}
     */
    final static PhysicsDumper dumper = new PhysicsDumper();
    /**
     * application instance
     */
    private static VhacdTuner application;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the VhacdTuner application.
     */
    public VhacdTuner() { // explicit to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Disable physics debug visualization.
     */
    static void clearPhysicsDebug() {
        leftBas.setDebugEnabled(false);
        rightBas.setDebugEnabled(false);
    }

    /**
     * Clear the default scene and both physics spaces.
     */
    void clearScene() {
        MySpatial.removeAllControls(rootNode);

        List<Spatial> children = rootNode.getChildren();
        for (Spatial child : children) {
            child.removeFromParent();
        }

        this.cgmParent = null;

        PhysicsSpace leftSpace = getLeftSpace();
        leftSpace.destroy();

        PhysicsSpace rightSpace = getRightSpace();
        rightSpace.destroy();
    }

    /**
     * Find the first attached AppState that's an instance of the specified
     * class.
     *
     * @param <T> type of subclass
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
     * Access the application instance from a static context.
     *
     * @return the pre-existing instance (not null)
     */
    static VhacdTuner getApplication() {
        assert application != null;
        return application;
    }

    /**
     * Access the left-side space for physics simulation.
     *
     * @return the pre-existing instance (not null)
     */
    static PhysicsSpace getLeftSpace() {
        PhysicsSpace result = leftBas.getPhysicsSpace();
        assert result != null;
        return result;
    }

    /**
     * Access the state information.
     *
     * @return the pre-existing instance (not null)
     */
    static Model getModel() {
        assert model != null;
        return model;
    }

    /**
     * Access the right-side space for physics simulation.
     *
     * @return the pre-existing instance (not null)
     */
    static PhysicsSpace getRightSpace() {
        PhysicsSpace result = rightBas.getPhysicsSpace();
        assert result != null;
        return result;
    }

    /**
     * Test whether physics debug visualization is enabled.
     *
     * @return true if enabled, otherwise false
     */
    static boolean isDebugEnabled() {
        boolean result = rightBas.isDebugEnabled();
        assert leftBas.isDebugEnabled() == result;
        return result;
    }

    /**
     * Main entry point for the VhacdTuner application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        // Mute the chatty loggers found in certain packages.
        Heart.setLoggingLevels(Level.WARNING);

        String renderer = AppSettings.LWJGL_OPENGL2;
        ShowDialog showDialog = ShowDialog.Never;

        // Process any command-line arguments.
        for (String arg : arguments) {
            switch (arg) {
                case "-3":
                case "--openGL3":
                    renderer = AppSettings.LWJGL_OPENGL33;
                    break;

                case "-f":
                case "--forceDialog":
                    showDialog = ShowDialog.Always;
                    break;

                case "--showSettingsDialog":
                    showDialog = ShowDialog.FirstTime;
                    break;

                case "-v":
                case "--verbose":
                    Heart.setLoggingLevels(Level.INFO);
                    break;

                default:
                    logger.log(
                            Level.WARNING, "Unknown command-line argument {0}",
                            MyString.quote(arg));
            }
        }

        String title = applicationName + " " + MyString.join(arguments);
        mainStartup(showDialog, renderer, title);
    }

    /**
     * Add a C-G model to the (cleared) scene and reset the camera.
     *
     * @param cgModel (not null, alias created)
     */
    void makeScene(Spatial cgModel) {
        assert cgModel != null;
        assert cgmParent == null;

        // Add the C-G model, with its own parent Node.
        this.cgmParent = new Node("cgmParent");
        rootNode.attachChild(cgmParent);
        cgmParent.attachChild(cgModel);

        resetCamera();
        updateAxes(rootNode);
    }

    /**
     * Alter the physics debug visualization.
     *
     * @param viewPorts the left and right viewports to use for debug
     * visualization (not null)
     */
    static void setPhysicsDebug(ViewPort... viewPorts) {
        assert viewPorts.length == 2 : viewPorts.length;

        leftBas.setDebugEnabled(true);
        ViewPort left = viewPorts[0];
        leftBas.setDebugViewPorts(left);

        rightBas.setDebugEnabled(true);
        ViewPort right = viewPorts[1];
        rightBas.setDebugViewPorts(right);
    }

    /**
     * Update the world-axes visualization.
     *
     * @param parent where to attach the axes (not null)
     */
    static void updateAxes(Node parent) {
        float length = model.radius();
        boolean enable = model.isShowingAxes();

        AxesVisualizer axes = parent.getControl(AxesVisualizer.class);
        if (axes == null) {
            AssetManager am = application.getAssetManager();
            axes = new AxesVisualizer(am, length);
            parent.addControl(axes);
        } else {
            axes.setAxisLength(length);
        }
        axes.setEnabled(enable);
    }
    // *************************************************************************
    // GuiApplication methods

    /**
     * Initialize the VhacdTuner application.
     */
    @Override
    public void guiInitializeApplication() {
        logger.info("");
        logger.setLevel(Level.INFO);

        if (!Heart.areAssertionsEnabled()) {
            logger.warning("Assertions are disabled!");
        }

        // Log version strings.
        logger.log(Level.INFO, "jme3-core version is {0}",
                MyString.quote(JmeVersion.FULL_NAME));
        logger.log(Level.INFO, "Heart version is {0}",
                MyString.quote(Heart.versionShort()));
        logger.log(Level.INFO, "jme3-utilities-nifty version is {0}",
                MyString.quote(LibraryVersion.versionShort()));

        // Detach an app state created by SimpleApplication.
        DebugKeysAppState debugKeys = findAppState(DebugKeysAppState.class);
        stateManager.detach(debugKeys);

        configureDumper();
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
        if (logger.isLoggable(Level.INFO)) {
            logger.log(Level.INFO, "Got action {0} ongoing={1}",
                    new Object[]{MyString.quote(actionString), ongoing});
        }

        boolean handled = false;
        if (ongoing) {
            handled = Action.processOngoing(actionString);
        }
        if (!handled) { // Forward unhandled action to the superclass.
            super.onAction(actionString, ongoing, tpf);
        }
    }

    /**
     * Callback invoked once per frame.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void simpleUpdate(float tpf) {
        super.simpleUpdate(tpf);

        if (!didStartup1) {
            startup1();
            didStartup1 = true;
        }
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

        ColorRGBA skyColor = new ColorRGBA(0.1f, 0.2f, 0.4f, 1f);
        viewPort.setBackgroundColor(skyColor);
    }

    /**
     * Attach app states during initialization.
     */
    private void attachAppStates() {
        boolean success;

        leftBas = new BulletAppState();
        success = stateManager.attach(leftBas);
        assert success;

        rightBas = new BulletAppState();
        success = stateManager.attach(rightBas);
        assert success;

        success = stateManager.attach(new BindScreen());
        assert success;

        success = stateManager.attach(new BulletAppState());
        assert success;

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
         * for the "test" screen.
         */
        success = stateManager.attach(new TestMode());
        assert success;
        success = stateManager.attach(new TestScreen());
        assert success;
        /*
         * Create and attach an input mode and screen controller
         * for the "save" screen.
         */
        success = stateManager.attach(new SaveMode());
        assert success;
        success = stateManager.attach(new SaveScreen());
        assert success;

        success = stateManager.attach(new ViewPortAppState());
        assert success;
    }

    /**
     * Configure the PhysicsDumper during startup.
     */
    private static void configureDumper() {
        dumper.setEnabled(DumpFlags.CullHints, true);
        dumper.setEnabled(DumpFlags.Transforms, true);
    }

    /**
     * Initialization performed immediately after parsing the command-line
     * arguments.
     *
     * @param showDialog when to show the JME settings dialog (not null)
     * @param renderer the value passed to
     * {@link com.jme3.system.AppSettings#setRenderer(java.lang.String)}
     * @param title for the title bar of the app's window
     */
    private static void mainStartup(final ShowDialog showDialog,
            final String renderer, final String title) {
        // Instantiate the application.
        application = new VhacdTuner();

        // Instantiate the display-settings screen.
        RectSizeLimits dsl = new RectSizeLimits(
                640, 700, // minimum width and height
                2_048, 1_080 // maximum width and height
        );
        DisplaySettings displaySettings
                = new DisplaySettings(application, applicationName, dsl) {
            @Override
            protected void applyOverrides(AppSettings settings) {
                super.applyOverrides(settings);

                setShowDialog(showDialog);
                settings.setAudioRenderer(null);
                settings.setRenderer(renderer);
                settings.setResizable(true);
                settings.setSamples(4);
                settings.setTitle(title);
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
            // ... and onward to VhacdTuner.guiInitializeApplication()!
        }
    }

    /**
     * Reset the default camera.
     */
    private void resetCamera() {
        float radius = model.radius();
        float distance = 4f * radius;

        flyCam.setMoveSpeed(distance);

        cam.setLocation(new Vector3f(0f, 0f, distance));
        MyCamera.setNearFar(cam, 0.01f * distance, 100f * distance);
        cam.setRotation(new Quaternion(0f, 1f, 0f, 0f));
    }

    /**
     * Initialization performed during the first invocation of
     * {@link #simpleUpdate(float)}.
     */
    private void startup1() {
        logger.info("");
        /*
         * Disable the render statistics.
         * These can be re-enabled by pressing the F5 hotkey.
         */
        setDisplayFps(false);
        setDisplayStatView(false);

        // Enable the initial InputMode.
        InputMode.getActiveMode().setEnabled(false);
        InputMode initialInputMode = InputMode.findMode("filePath");
        initialInputMode.setEnabled(true);
    }
}
