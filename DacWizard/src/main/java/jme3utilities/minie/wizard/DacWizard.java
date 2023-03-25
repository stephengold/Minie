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
package jme3utilities.minie.wizard;

import com.jme3.anim.AnimClip;
import com.jme3.anim.AnimComposer;
import com.jme3.anim.Armature;
import com.jme3.anim.SkinningControl;
import com.jme3.animation.AnimControl;
import com.jme3.animation.Animation;
import com.jme3.animation.Skeleton;
import com.jme3.animation.SkeletonControl;
import com.jme3.app.DebugKeysAppState;
import com.jme3.app.state.AppState;
import com.jme3.app.state.AppStateManager;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.animation.DynamicAnimControl;
import com.jme3.bullet.animation.RagUtils;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.control.AbstractControl;
import com.jme3.scene.control.Control;
import com.jme3.system.AppSettings;
import com.jme3.system.JmeVersion;
import java.io.IOException;
import java.util.Collection;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.InfluenceUtil;
import jme3utilities.MyCamera;
import jme3utilities.MySkeleton;
import jme3utilities.MySpatial;
import jme3utilities.MyString;
import jme3utilities.debug.AxesVisualizer;
import jme3utilities.debug.SkeletonVisualizer;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;
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
import jme3utilities.wes.AnimationEdit;
import jme3utilities.wes.Pose;
import jme3utilities.wes.TweenTransforms;

/**
 * A GuiApplication to configure a DynamicAnimControl for a C-G model. The
 * application's main entry point is in this class. The scene graph is also
 * managed here.
 * <p>
 * Seen in the April 2019 walkthru video:
 * https://www.youtube.com/watch?v=iWyrzZe45jA
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class DacWizard extends GuiApplication {
    // *************************************************************************
    // constants and loggers

    /**
     * desired height of the C-G model (for visualization, in world units)
     */
    final private static float cgmHeight = 10f;
    /**
     * message logger for this class
     */
    final static Logger logger = Logger.getLogger(DacWizard.class.getName());
    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName
            = DacWizard.class.getSimpleName();
    /**
     * magic clip/animation name used to denote a C-G model's bind pose
     */
    final public static String bindPoseName = "( bind pose )";
    // *************************************************************************
    // fields

    /**
     * true once {@link #startup1()} has completed, until then false
     */
    private static boolean didStartup1 = false;
    /**
     * application instance
     */
    private static DacWizard application;
    /**
     * Nifty screen for editing display settings
     */
    private static DsScreen displaySettingsScreen;
    /**
     * state information
     */
    final private static Model model = new Model();
    /**
     * node controlled by the AxesVisualizer
     */
    private static Node axesNode = null;
    /**
     * parent of the loaded C-G model in the scene
     */
    private static Node cgmParent = null;
    /**
     * dump debugging information to {@code System.out}
     */
    final static PhysicsDumper dumper = new PhysicsDumper();
    // *************************************************************************
    // constructors

    /**
     * Instantiate the DacWizard application.
     */
    public DacWizard() { // explicit to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Clear the default scene.
     */
    void clearScene() {
        BulletAppState bulletAppState = findAppState(BulletAppState.class);
        PhysicsSpace physicsSpace = bulletAppState.getPhysicsSpace();
        DynamicAnimControl ragdoll = findDac();
        if (ragdoll != null) {
            ragdoll.setPhysicsSpace(null);
        }
        if (!physicsSpace.isEmpty()) {
            dumper.dump(physicsSpace);
        }
        assert physicsSpace.isEmpty();

        int numControls = rootNode.getNumControls();
        for (int controlI = numControls - 1; controlI >= 0; --controlI) {
            Control control = rootNode.getControl(controlI);
            rootNode.removeControl(control);
        }

        List<Spatial> children = rootNode.getChildren();
        for (Spatial child : children) {
            child.removeFromParent();
        }

        if (axesNode != null) {
            axesNode = null;
        }
        if (cgmParent != null) {
            cgmParent = null;
        }
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
     * Find the AxesVisualizer in the scene.
     *
     * @return the pre-existing Control, or null if none/multiple
     */
    static AxesVisualizer findAxesVisualizer() {
        AxesVisualizer result = null;
        if (axesNode != null) {
            List<AxesVisualizer> controls = MySpatial.listControls(
                    axesNode, AxesVisualizer.class, null);
            if (controls.size() == 1) {
                result = controls.get(0);
            }
        }

        return result;
    }

    /**
     * Find the DynamicAnimControl in the scene.
     *
     * @return the pre-existing Control, or null if none/multiple
     */
    static DynamicAnimControl findDac() {
        DynamicAnimControl result = null;
        if (cgmParent != null) {
            List<DynamicAnimControl> controls = MySpatial.listControls(
                    cgmParent, DynamicAnimControl.class, null);
            if (controls.size() == 1) {
                result = controls.get(0);
            }
        }

        return result;
    }

    /**
     * Find the SkeletonVisualizer in the scene.
     *
     * @return the pre-existing control, or null if none/multiple
     */
    SkeletonVisualizer findSkeletonVisualizer() {
        SkeletonVisualizer result = null;
        List<SkeletonVisualizer> controls = MySpatial.listControls(
                rootNode, SkeletonVisualizer.class, null);
        if (controls.size() == 1) {
            result = controls.get(0);
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
     * Access the state information.
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
     * Add a C-G model to the (cleared) scene, pose the model, transform the
     * model, and reset the camera.
     *
     * @param cgModel the root spatial of model to add (not null, alias created)
     * @param animationName the clip/animation name for the desired pose or
     * {@link #bindPoseName} (not null)
     * @param animationTime the animation time for the desired pose (in seconds,
     * &ge;0)
     */
    void makeScene(Spatial cgModel, String animationName, float animationTime) {
        assert cgModel != null;
        assert animationName != null;
        assert animationTime >= 0f : animationTime;
        assert axesNode == null;
        assert cgmParent == null;

        // Add a disabled visualizer for axes, with its own controlled Node.
        axesNode = new Node("axesNode");
        rootNode.attachChild(axesNode);
        float arrowLength = 0.2f * cgmHeight;
        AxesVisualizer axes = new AxesVisualizer(assetManager, arrowLength);
        axes.setLineWidth(0f); // solid arrows
        axesNode.addControl(axes);

        // Add the C-G model, with its own parent Node.
        cgmParent = new Node("cgmParent");
        if (model.isShowingMeshes()) {
            cgmParent.setCullHint(Spatial.CullHint.Never);
        } else {
            cgmParent.setCullHint(Spatial.CullHint.Always);
        }
        rootNode.attachChild(cgmParent);
        cgmParent.attachChild(cgModel);
        /*
         * Normalize all quaternions in the model's animations,
         * since Pose.applyTo() is sensitive to such flaws.
         */
        List<AnimControl> animControls = MySpatial
                .listControls(cgmParent, AnimControl.class, null);
        normalizeAnimControls(animControls);

        List<AnimComposer> composers = MySpatial
                .listControls(cgmParent, AnimComposer.class, null);
        normalizeComposers(composers);

        // Apply the specified animation pose.
        AbstractControl sc = RagUtils.findSControl(cgModel);
        if (sc instanceof SkeletonControl && animControls.size() == 1) {
            // old animation system
            SkeletonControl skeletonControl = (SkeletonControl) sc;
            Skeleton skeleton = skeletonControl.getSkeleton();
            AnimControl animControl = animControls.get(0);
            poseSkeleton(skeleton, animControl, animationName, animationTime);

        } else if (sc instanceof SkinningControl && composers.size() == 1) {
            // new animation system
            SkinningControl skinningControl = (SkinningControl) sc;
            Armature armature = skinningControl.getArmature();
            AnimComposer composer = composers.get(0);
            poseArmature(armature, composer, animationName, animationTime);
        }

        // Translate and scale the C-G model.
        setParentTransform(cgmHeight);

        if (sc != null && findSkeletonVisualizer() == null) {
            // Add a SkeletonVisualizer.
            SkeletonVisualizer sv = new SkeletonVisualizer(assetManager, sc);
            sv.setLineColor(ColorRGBA.Yellow);
            if (sc instanceof SkeletonControl) { // old animation system
                InfluenceUtil.hideNonInfluencers(sv, (SkeletonControl) sc);
            } else { // new animation system
                InfluenceUtil.hideNonInfluencers(sv, (SkinningControl) sc);
            }
            rootNode.addControl(sv);
        }

        DynamicAnimControl dac = findDac();
        if (dac != null) { // Configure the DAC.
            float gravity = 6f * cgmHeight;
            Vector3f gravityVector = new Vector3f(0f, -gravity, 0f);
            dac.setGravity(gravityVector);
        }

        resetCamera();
    }

    /**
     * Toggle rendering of C-G model meshes on/off.
     */
    static void toggleMesh() {
        boolean oldShow = model.isShowingMeshes();
        boolean newShow = !oldShow;
        model.setShowingMeshes(newShow);

        if (cgmParent != null) {
            Spatial.CullHint cull;
            if (newShow) {
                cull = Spatial.CullHint.Never;
            } else {
                cull = Spatial.CullHint.Always;
            }
            cgmParent.setCullHint(cull);
        }
    }

    /**
     * Toggle the skeleton visualizer on/off.
     */
    static void toggleSkeletonVisualizer() {
        DacWizard app = getApplication();
        SkeletonVisualizer sv = app.findSkeletonVisualizer();
        if (sv != null) {
            boolean newState = !sv.isEnabled();
            sv.setEnabled(newState);
        }
    }
    // *************************************************************************
    // GuiApplication methods

    /**
     * Initialize the DacWizard application.
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

        ColorRGBA bgColor = new ColorRGBA(0.2f, 0.2f, 0.2f, 1f);
        viewPort.setBackgroundColor(bgColor);
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
         * for the "torso" screen.
         */
        success = stateManager.attach(new TorsoMode());
        assert success;
        success = stateManager.attach(new TorsoScreen());
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
     * Configure the PhysicsDumper during startup.
     */
    private static void configureDumper() {
        dumper.setEnabled(DumpFlags.CullHints, true);
        dumper.setEnabled(DumpFlags.JointsInBodies, true);
        dumper.setEnabled(DumpFlags.JointsInSpaces, true);
        dumper.setEnabled(DumpFlags.Transforms, true);
    }

    /**
     * Normalize all quaternions in the specified animation controls.
     *
     * @param animControls the controls to modify (not null)
     */
    private static void normalizeAnimControls(List<AnimControl> animControls) {
        for (AnimControl animControl : animControls) {
            Collection<String> names = animControl.getAnimationNames();
            for (String name : names) {
                Animation animation = animControl.getAnim(name);
                AnimationEdit.normalizeQuaternions(animation, 0.00005f);
            }
        }
    }

    /**
     * Normalize all quaternions in the specified anim composers.
     *
     * @param composers the anim composers to modify (not null)
     */
    private static void normalizeComposers(List<AnimComposer> composers) {
        for (AnimComposer composer : composers) {
            Collection<String> names = composer.getAnimClipsNames();
            for (String clipName : names) {
                AnimClip animClip = composer.getAnimClip(clipName);
                AnimationEdit.normalizeQuaternions(animClip, 0.00005f);
            }
        }
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
        application = new DacWizard();

        // Instantiate the display-settings screen.
        RectSizeLimits dsl = new RectSizeLimits(
                640, 480, // minimum width and height
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
            // ... and onward to DacWizard.guiInitializeApplication()!
        }
    }

    /**
     * Apply the specified animation pose to the specified Armature.
     *
     * @param armature the armature to modify (not null)
     * @param composer the composer containing the named clip, unless it's
     * {@link #bindPoseName}
     * @param clipName the name of the clip containing the desired pose, or
     * {@link #bindPoseName}
     * @param animationTime the animation time of the desired pose in the clip
     * (in seconds, &ge;0)
     */
    private static void poseArmature(Armature armature, AnimComposer composer,
            String clipName, float animationTime) {
        Pose pose = new Pose(armature);
        if (!clipName.equals(bindPoseName)) {
            AnimClip clip = composer.getAnimClip(clipName);
            pose.setToClip(clip, animationTime);
        }
        pose.applyTo(armature);
        armature.update();
    }

    /**
     * Apply the specified animation pose to the specified Skeleton.
     *
     * @param skeleton the skeleton to modify (not null)
     * @param animControl the control containing the named animation, unless
     * it's {@link #bindPoseName}
     * @param animationName the name of the animation containing the desired
     * pose, or {@link #bindPoseName}
     * @param time the animation time of the desired pose in the animation (in
     * seconds, &ge;0)
     */
    private static void poseSkeleton(Skeleton skeleton, AnimControl animControl,
            String animationName, float time) {
        MySkeleton.setUserControl(skeleton, true);
        Pose pose = new Pose(skeleton);
        if (!animationName.equals(bindPoseName)) {
            Animation animation = animControl.getAnim(animationName);
            pose.setToAnimation(animation, time, new TweenTransforms());
        }
        pose.applyTo(skeleton);
        skeleton.updateWorldVectors();
    }

    /**
     * Reset the default camera.
     */
    private void resetCamera() {
        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(20f);

        Vector3f location = new Vector3f(0f, 0.93f, 2.7f);
        location.multLocal(cgmHeight);
        cam.setLocation(location);

        cam.setName("cam");
        cam.setRotation(new Quaternion(0f, 0.995268f, -0.094f, 0.0253f));

        float near = 0.01f * cgmHeight;
        float far = 25f * cgmHeight;
        MyCamera.setNearFar(cam, near, far);
    }

    /**
     * Configure the parent transform so that the C-G model has the specified
     * size and the center of its base is located at the world origin.
     *
     * @param desiredSize the desired size (in world units, &gt;0)
     */
    private static void setParentTransform(float desiredSize) {
        assert desiredSize > 0f : desiredSize;

        // Scale the model uniformly to the desired size.
        Vector3f[] minMax = MySpatial.findMinMaxCoords(cgmParent);
        Vector3f min = minMax[0]; // alias
        Vector3f max = minMax[1]; // alias
        Vector3f dimensions = max.subtract(min);
        float oldSize = MyMath.max(dimensions.x, dimensions.y, dimensions.z);
        if (oldSize > 0f) {
            cgmParent.scale(desiredSize / oldSize);
        }
        /*
         * Translate the model so that its base rests on the X-Z plane and its
         * center lies on the Y axis.
         */
        minMax = MySpatial.findMinMaxCoords(cgmParent);
        min = minMax[0]; // alias
        max = minMax[1]; // alias
        Vector3f center = MyVector3f.midpoint(min, max, null);
        cgmParent.move(-center.x, -min.y, -center.z);
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
