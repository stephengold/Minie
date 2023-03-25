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

import com.jme3.app.Application;
import com.jme3.app.SimpleApplication;
import com.jme3.app.state.AppStateManager;
import com.jme3.asset.AssetManager;
import com.jme3.cursors.plugins.JmeCursor;
import com.jme3.input.CameraInput;
import com.jme3.input.KeyInput;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.MyString;
import jme3utilities.Validate;
import jme3utilities.nifty.dialog.AllowNull;
import jme3utilities.nifty.dialog.DoubleDialog;
import jme3utilities.nifty.dialog.IntegerDialog;
import jme3utilities.ui.InputMode;

/**
 * Input mode for the "test" screen of VhacdTuner.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class TestMode extends InputMode {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final static Logger logger = Logger.getLogger(TestMode.class.getName());
    /**
     * asset path to the cursor for this input mode
     */
    final private static String assetPath = "Textures/cursors/default.cur";
    /**
     * action-string prefixes specific to this input mode:
     */
    final private static String apSetAlphaLeft = "set alpha left ";
    final private static String apSetAlphaRight = "set alpha right ";
    final private static String apSetBetaLeft = "set beta left ";
    final private static String apSetBetaRight = "set beta right ";
    final private static String apSetHullDSLeft = "set hullDS left ";
    final private static String apSetHullDSRight = "set hullDS right ";
    final private static String apSetMaxConcavityLeft
            = "set maxConcavity left ";
    final private static String apSetMaxConcavityRight
            = "set maxConcavity right ";
    final private static String apSetMaxHullsLeft = "set maxHulls left ";
    final private static String apSetMaxHullsRight = "set maxHulls right ";
    final private static String apSetMaxRecursionLeft
            = "set maxRecursion left ";
    final private static String apSetMaxRecursionRight
            = "set maxRecursion right ";
    final private static String apSetMaxVerticesPHLeft = "set maxVph left ";
    final private static String apSetMaxVerticesPHRight = "set maxVph right ";
    final private static String apSetMinEdgeLengthLeft
            = "set minEdgeLength left ";
    final private static String apSetMinEdgeLengthRight
            = "set minEdgeLength right ";
    final private static String apSetMinVolumePHLeft
            = "set minVolumePH left ";
    final private static String apSetMinVolumePHRight
            = "set minVolumePH right ";
    final private static String apSetPlaneDSLeft = "set planeDS left ";
    final private static String apSetPlaneDSRight = "set planeDS right ";
    final private static String apSetResolutionLeft = "set resolution left ";
    final private static String apSetResolutionRight = "set resolution right ";
    final private static String apSetVolumePercentErrorLeft
            = "set volumePercentError left ";
    final private static String apSetVolumePercentErrorRight
            = "set volumePercentError right ";
    /**
     * action strings specific to this input mode:
     */
    final private static String asLeft = "prefer left";
    final private static String asNextFillModeLeft = "next fillMode left";
    final private static String asNextFillModeRight = "next fillMode right";
    final private static String asRight = "prefer right";
    final private static String asSetAlphaLeft = "set alpha left";
    final private static String asSetAlphaRight = "set alpha right";
    final private static String asSetBetaLeft = "set beta left";
    final private static String asSetBetaRight = "set beta right";
    final private static String asSetHullDSLeft = "set hullDS left";
    final private static String asSetHullDSRight = "set hullDS right";
    final private static String asSetMaxConcavityLeft
            = "set maxConcavity left";
    final private static String asSetMaxConcavityRight
            = "set maxConcavity right";
    final private static String asSetMaxHullsLeft = "set maxHulls left";
    final private static String asSetMaxHullsRight = "set maxHulls right";
    final private static String asSetMaxRecursionLeft
            = "set maxRecursion left";
    final private static String asSetMaxRecursionRight
            = "set maxRecursion right";
    final private static String asSetMaxVerticesPHLeft
            = "set maxVerticesPH left";
    final private static String asSetMaxVerticesPHRight
            = "set maxVerticesPH right";
    final private static String asSetMinEdgeLengthLeft
            = "set minEdgeLength left";
    final private static String asSetMinEdgeLengthRight
            = "set minEdgeLength right";
    final private static String asSetMinVolumePHLeft
            = "set minVolumePH left";
    final private static String asSetMinVolumePHRight
            = "set minVolumePH right";
    final private static String asSetPlaneDSLeft = "set planeDS left";
    final private static String asSetPlaneDSRight = "set planeDS right";
    final private static String asSetResolutionLeft = "set resolution left";
    final private static String asSetResolutionRight = "set resolution right";
    final private static String asSetVolumePercentErrorLeft
            = "set volumePercentError left";
    final private static String asSetVolumePercentErrorRight
            = "set volumePercentError right";
    final private static String asStopRanking = "stop ranking";
    final private static String asToggleAsyncLeft = "toggle async left";
    final private static String asToggleAsyncRight = "toggle async right";
    final private static String asToggleFindBestPlaneLeft
            = "toggle findBestPlane left";
    final private static String asToggleFindBestPlaneRight
            = "toggle findBestPlane right";
    final private static String asTogglePcaLeft = "toggle pca left";
    final private static String asTogglePcaRight = "toggle pca right";
    final private static String asToggleShrinkLeft = "toggle shrink left";
    final private static String asToggleShrinkRight = "toggle shrink right";
    final private static String asToggleVersionLeft = "toggle version left";
    final private static String asToggleVersionRight = "toggle version right";
    // *************************************************************************
    // constructors

    /**
     * Instantiate a disabled, uninitialized mode.
     */
    TestMode() {
        super("test");
    }
    // *************************************************************************
    // InputMode methods

    /**
     * Add default hotkey bindings. These bindings will be used if no custom
     * bindings are found.
     */
    @Override
    protected void defaultBindings() {
        bind(SimpleApplication.INPUT_MAPPING_EXIT, KeyInput.KEY_ESCAPE);
        bind(Action.editBindings, KeyInput.KEY_F1);
        bind(Action.editDisplaySettings, KeyInput.KEY_F2);

        bind(Action.nextScreen, KeyInput.KEY_PGDN, KeyInput.KEY_N);
        bind(Action.previousScreen, KeyInput.KEY_PGUP, KeyInput.KEY_B);

        bind(Action.dumpPhysicsSpace, KeyInput.KEY_O);
        bind(Action.dumpRenderer, KeyInput.KEY_P);

        bindSignal(CameraInput.FLYCAM_BACKWARD, KeyInput.KEY_S);
        bindSignal(CameraInput.FLYCAM_FORWARD, KeyInput.KEY_W);
        bindSignal(CameraInput.FLYCAM_LOWER, KeyInput.KEY_Z);
        bindSignal(CameraInput.FLYCAM_RISE, KeyInput.KEY_Q);
        bindSignal("orbitLeft", KeyInput.KEY_A);
        bindSignal("orbitRight", KeyInput.KEY_D);

        bind(SimpleApplication.INPUT_MAPPING_CAMERA_POS, KeyInput.KEY_C);
        bind(Action.togglePhysicsDebug, KeyInput.KEY_SLASH);
    }

    /**
     * Initialize this (disabled) mode prior to its first update.
     *
     * @param stateManager (not null)
     * @param application (not null)
     */
    @Override
    public void initialize(
            AppStateManager stateManager, Application application) {
        // Configure the mouse cursor for this mode.
        AssetManager manager = application.getAssetManager();
        JmeCursor cursor = (JmeCursor) manager.loadAsset(assetPath);
        setCursor(cursor);

        super.initialize(stateManager, application);
    }

    /**
     * Process an action from the keyboard or mouse.
     *
     * @param actionString textual description of the action (not null)
     * @param ongoing true if the action is ongoing, otherwise false
     * @param tpf time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void onAction(String actionString, boolean ongoing, float tpf) {
        Validate.nonNull(actionString, "action string");
        if (logger.isLoggable(Level.INFO)) {
            logger.log(Level.INFO, "Got action {0} ongoing={1}",
                    new Object[]{MyString.quote(actionString), ongoing});
        }

        boolean handled = false;
        if (ongoing) {
            Model model = VhacdTuner.getModel();
            DecompositionTest leftTest = model.getLeftTest();
            DecompositionTest rightTest = model.getRightTest();
            TestScreen screen = VhacdTuner.findAppState(TestScreen.class);
            assert screen.isEnabled();

            handled = true;
            switch (actionString) {
                case asLeft:
                    if (model.isRanking()) {
                        model.prefer(leftTest, rightTest);
                    } else {
                        model.startTest(leftTest);
                    }
                    break;

                case Action.nextScreen:
                    nextScreen();
                    break;

                case asNextFillModeLeft:
                    model.nextFillModeLeft();
                    break;

                case asNextFillModeRight:
                    model.nextFillModeRight();
                    break;

                case Action.previousScreen:
                    previousScreen();
                    break;

                case asRight:
                    if (model.isRanking()) {
                        model.prefer(rightTest, leftTest);
                    } else {
                        model.startTest(rightTest);
                    }
                    break;

                case asSetAlphaLeft:
                    setAlpha(leftTest, apSetAlphaLeft);
                    break;

                case asSetAlphaRight:
                    setAlpha(rightTest, apSetAlphaRight);
                    break;

                case asSetBetaLeft:
                    setBeta(leftTest, apSetBetaLeft);
                    break;

                case asSetBetaRight:
                    setBeta(rightTest, apSetBetaRight);
                    break;

                case asSetHullDSLeft:
                    setHullDS(leftTest, apSetHullDSLeft);
                    break;

                case asSetHullDSRight:
                    setHullDS(rightTest, apSetHullDSRight);
                    break;

                case asSetMaxConcavityLeft:
                    setMaxConcavity(leftTest, apSetMaxConcavityLeft);
                    break;

                case asSetMaxConcavityRight:
                    setMaxConcavity(rightTest, apSetMaxConcavityRight);
                    break;

                case asSetMaxHullsLeft:
                    setMaxHulls(leftTest, apSetMaxHullsLeft);
                    break;

                case asSetMaxHullsRight:
                    setMaxHulls(rightTest, apSetMaxHullsRight);
                    break;

                case asSetMaxRecursionLeft:
                    setMaxRecursion(leftTest, apSetMaxRecursionLeft);
                    break;

                case asSetMaxRecursionRight:
                    setMaxRecursion(rightTest, apSetMaxRecursionRight);
                    break;

                case asSetMaxVerticesPHLeft:
                    setMaxVerticesPH(leftTest, apSetMaxVerticesPHLeft);
                    break;

                case asSetMaxVerticesPHRight:
                    setMaxVerticesPH(rightTest, apSetMaxVerticesPHRight);
                    break;

                case asSetMinEdgeLengthLeft:
                    setMinEdgeLength(leftTest, apSetMinEdgeLengthLeft);
                    break;

                case asSetMinEdgeLengthRight:
                    setMinEdgeLength(rightTest, apSetMinEdgeLengthRight);
                    break;

                case asSetMinVolumePHLeft:
                    setMinVolumePH(leftTest, apSetMinVolumePHLeft);
                    break;

                case asSetMinVolumePHRight:
                    setMinVolumePH(rightTest, apSetMinVolumePHRight);
                    break;

                case asSetPlaneDSLeft:
                    setPlaneDS(leftTest, apSetPlaneDSLeft);
                    break;

                case asSetPlaneDSRight:
                    setPlaneDS(rightTest, apSetPlaneDSRight);
                    break;

                case asSetResolutionLeft:
                    setResolution(leftTest, apSetResolutionLeft);
                    break;

                case asSetResolutionRight:
                    setResolution(rightTest, apSetResolutionRight);
                    break;

                case asSetVolumePercentErrorLeft:
                    setVolumePercentError(
                            leftTest, apSetVolumePercentErrorLeft);
                    break;

                case asSetVolumePercentErrorRight:
                    setVolumePercentError(
                            rightTest, apSetVolumePercentErrorRight);
                    break;

                case asStopRanking:
                    model.stopRanking();
                    break;

                case asToggleAsyncLeft:
                    model.toggleAsyncLeft();
                    break;

                case asToggleAsyncRight:
                    model.toggleAsyncRight();
                    break;

                case asToggleFindBestPlaneLeft:
                    model.toggleFindBestPlaneLeft();
                    break;

                case asToggleFindBestPlaneRight:
                    model.toggleFindBestPlaneRight();
                    break;

                case asTogglePcaLeft:
                    model.togglePcaLeft();
                    break;

                case asTogglePcaRight:
                    model.togglePcaRight();
                    break;

                case asToggleShrinkLeft:
                    model.toggleShrinkLeft();
                    break;

                case asToggleShrinkRight:
                    model.toggleShrinkRight();
                    break;

                case asToggleVersionLeft:
                    model.toggleVersionLeft();
                    break;

                case asToggleVersionRight:
                    model.toggleVersionRight();
                    break;

                case Action.toggleAxes:
                    screen.toggleShowingAxes();
                    break;

                case Action.toggleMesh:
                    screen.toggleMesh();
                    break;

                case Action.togglePhysicsDebug:
                    screen.togglePhysicsDebug();
                    break;

                default:
                    handled = false;
            }
            if (!handled) {
                handled = testForPrefixes(actionString);
            }
        }
        if (!handled) {
            getActionApplication().onAction(actionString, ongoing, tpf);
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Advance to the SaveScreen if possible.
     */
    private void nextScreen() {
        setEnabled(false);
        InputMode save = InputMode.findMode("save");
        save.setEnabled(true);
    }

    /**
     * Go back to the LoadScreen.
     */
    private void previousScreen() {
        setEnabled(false);
        InputMode load = InputMode.findMode("load");
        load.setEnabled(true);
    }

    /**
     * Handle a "set alpha" action.
     *
     * @param test the test to use as the base (not null, unaffected)
     * @param actionPrefix the action prefix for the dialog box
     */
    private static void setAlpha(DecompositionTest test, String actionPrefix) {
        Model model = VhacdTuner.getModel();
        if (model.isRanking()) {
            return;
        }

        double oldValue = test.copyClassic().getAlpha();
        String defaultValue = Double.toString(oldValue);

        DoubleDialog controller
                = new DoubleDialog("Set", 0.0, 1.0, AllowNull.No);

        TestScreen screen = VhacdTuner.findAppState(TestScreen.class);
        assert screen.isEnabled();
        screen.closeAllPopups();
        screen.showTextEntryDialog(
                "Enter alpha:", defaultValue, actionPrefix, controller);
    }

    /**
     * Handle a "set beta" action.
     *
     * @param test the test to use as the base (not null, unaffected)
     * @param actionPrefix the action prefix for the dialog box
     */
    private static void setBeta(DecompositionTest test, String actionPrefix) {
        Model model = VhacdTuner.getModel();
        if (model.isRanking()) {
            return;
        }

        double oldValue = test.copyClassic().getBeta();
        String defaultValue = Double.toString(oldValue);

        DoubleDialog controller
                = new DoubleDialog("Set", 0.0, 1.0, AllowNull.No);

        TestScreen screen = VhacdTuner.findAppState(TestScreen.class);
        assert screen.isEnabled();
        screen.closeAllPopups();
        screen.showTextEntryDialog(
                "Enter beta:", defaultValue, actionPrefix, controller);
    }

    /**
     * Handle a "set hullDS" action.
     *
     * @param test the test to use as the base (not null, unaffected)
     * @param actionPrefix the action prefix for the dialog box
     */
    private static void setHullDS(DecompositionTest test, String actionPrefix) {
        Model model = VhacdTuner.getModel();
        if (model.isRanking()) {
            return;
        }

        int oldValue = test.copyClassic().getConvexHullDownSampling();
        String defaultValue = Integer.toString(oldValue);

        IntegerDialog controller
                = new IntegerDialog("Set", 1, 16, AllowNull.No);

        TestScreen screen = VhacdTuner.findAppState(TestScreen.class);
        assert screen.isEnabled();
        screen.closeAllPopups();
        screen.showTextEntryDialog("Enter convex-hull downsampling:",
                defaultValue, actionPrefix, controller);
    }

    /**
     * Handle a "set maxConcavity" action.
     *
     * @param test the test to use as the base (not null, unaffected)
     * @param actionPrefix the action prefix for the dialog box
     */
    private static void setMaxConcavity(
            DecompositionTest test, String actionPrefix) {
        Model model = VhacdTuner.getModel();
        if (model.isRanking()) {
            return;
        }

        double oldValue = test.copyClassic().getMaxConcavity();
        String defaultValue = Double.toString(oldValue);

        DoubleDialog controller
                = new DoubleDialog("Set", 0.0, 1.0, AllowNull.No);

        TestScreen screen = VhacdTuner.findAppState(TestScreen.class);
        assert screen.isEnabled();
        screen.closeAllPopups();
        screen.showTextEntryDialog("Enter maximum concavity:", defaultValue,
                actionPrefix, controller);
    }

    /**
     * Handle a "set maxHulls" action.
     *
     * @param test the test to use as the base (not null, unaffected)
     * @param actionPrefix the action prefix for the dialog box
     */
    private static void setMaxHulls(
            DecompositionTest test, String actionPrefix) {
        Model model = VhacdTuner.getModel();
        if (model.isRanking()) {
            return;
        }

        int oldValue = test.copyV4().getMaxHulls();
        String defaultValue = Integer.toString(oldValue);

        IntegerDialog controller
                = new IntegerDialog("Set", 1, 64, AllowNull.No);

        TestScreen screen = VhacdTuner.findAppState(TestScreen.class);
        assert screen.isEnabled();
        screen.closeAllPopups();
        screen.showTextEntryDialog("Enter maximum number of hulls:",
                defaultValue, actionPrefix, controller);
    }

    /**
     * Handle a "set maxRecursion" action.
     *
     * @param test the test to use as the base (not null, unaffected)
     * @param actionPrefix the action prefix for the dialog box
     */
    private static void setMaxRecursion(
            DecompositionTest test, String actionPrefix) {
        Model model = VhacdTuner.getModel();
        if (model.isRanking()) {
            return;
        }

        int oldValue = test.copyV4().getMaxRecursion();
        String defaultValue = Integer.toString(oldValue);

        IntegerDialog controller
                = new IntegerDialog("Set", 2, 64, AllowNull.No);

        TestScreen screen = VhacdTuner.findAppState(TestScreen.class);
        assert screen.isEnabled();
        screen.closeAllPopups();
        screen.showTextEntryDialog("Enter maximum recursion:", defaultValue,
                actionPrefix, controller);
    }

    /**
     * Handle a "set maxVerticesPH" action.
     *
     * @param test the test to use as the base (not null, unaffected)
     * @param actionPrefix the action prefix for the dialog box
     */
    private static void setMaxVerticesPH(
            DecompositionTest test, String actionPrefix) {
        Model model = VhacdTuner.getModel();
        if (model.isRanking()) {
            return;
        }

        int oldValue = test.maxVerticesPerHull();
        String defaultValue = Integer.toString(oldValue);

        IntegerDialog controller
                = new IntegerDialog("Set", 4, 2_048, AllowNull.No);

        TestScreen screen = VhacdTuner.findAppState(TestScreen.class);
        assert screen.isEnabled();
        screen.closeAllPopups();
        screen.showTextEntryDialog("Enter maximum vertices per hull:",
                defaultValue, actionPrefix, controller);
    }

    /**
     * Handle a "set minEdgeLength" action.
     *
     * @param test the test to use as the base (not null, unaffected)
     * @param actionPrefix the action prefix for the dialog box
     */
    private static void setMinEdgeLength(
            DecompositionTest test, String actionPrefix) {
        Model model = VhacdTuner.getModel();
        if (model.isRanking()) {
            return;
        }

        int oldValue = test.copyV4().getMinEdgeLength();
        String defaultValue = Integer.toString(oldValue);

        IntegerDialog controller
                = new IntegerDialog("Set", 1, 32, AllowNull.No);

        TestScreen screen = VhacdTuner.findAppState(TestScreen.class);
        assert screen.isEnabled();
        screen.closeAllPopups();
        screen.showTextEntryDialog("Enter minimum edge length:", defaultValue,
                actionPrefix, controller);
    }

    /**
     * Handle a "set minVolumePH" action.
     *
     * @param test the test to use as the base (not null, unaffected)
     * @param actionPrefix the action prefix for the dialog box
     */
    private static void setMinVolumePH(
            DecompositionTest test, String actionPrefix) {
        Model model = VhacdTuner.getModel();
        if (model.isRanking()) {
            return;
        }

        double oldValue = test.copyClassic().getMinVolumePerHull();
        String defaultValue = Double.toString(oldValue);

        DoubleDialog controller
                = new DoubleDialog("Set", 0.0, 0.1, AllowNull.No);

        TestScreen screen = VhacdTuner.findAppState(TestScreen.class);
        assert screen.isEnabled();
        screen.closeAllPopups();
        screen.showTextEntryDialog("Enter minimum volume per hull:",
                defaultValue, actionPrefix, controller);
    }

    /**
     * Handle a "set planeDS" action.
     *
     * @param test the test to use as the base (not null, unaffected)
     * @param actionPrefix the action prefix for the dialog box
     */
    private static void setPlaneDS(
            DecompositionTest test, String actionPrefix) {
        Model model = VhacdTuner.getModel();
        if (model.isRanking()) {
            return;
        }

        int oldValue = test.copyClassic().getPlaneDownSampling();
        String defaultValue = Integer.toString(oldValue);

        IntegerDialog controller
                = new IntegerDialog("Set", 1, 16, AllowNull.No);

        TestScreen screen = VhacdTuner.findAppState(TestScreen.class);
        assert screen.isEnabled();
        screen.closeAllPopups();
        screen.showTextEntryDialog("Enter plane downsampling:", defaultValue,
                actionPrefix, controller);
    }

    /**
     * Handle a "set resolution" action.
     *
     * @param test the test to use as the base (not null, unaffected)
     * @param actionPrefix the action prefix for the dialog box
     */
    private static void setResolution(
            DecompositionTest test, String actionPrefix) {
        Model model = VhacdTuner.getModel();
        if (model.isRanking()) {
            return;
        }

        int oldValue = test.resolution();
        String defaultValue = Integer.toString(oldValue);

        IntegerDialog controller
                = new IntegerDialog("Set", 10_000, 64_000_000, AllowNull.No);

        TestScreen screen = VhacdTuner.findAppState(TestScreen.class);
        assert screen.isEnabled();
        screen.closeAllPopups();
        screen.showTextEntryDialog("Enter voxel resolution:", defaultValue,
                actionPrefix, controller);
    }

    /**
     * Handle a "set volumePercentError" action.
     *
     * @param test the test to use as the base (not null, unaffected)
     * @param actionPrefix the action prefix for the dialog box
     */
    private static void setVolumePercentError(
            DecompositionTest test, String actionPrefix) {
        Model model = VhacdTuner.getModel();
        if (model.isRanking()) {
            return;
        }

        double oldValue = test.copyV4().getVolumePercentError();
        String defaultValue = Double.toString(oldValue);

        DoubleDialog controller
                = new DoubleDialog("Set", 0.0, 100.0, AllowNull.No);

        TestScreen screen = VhacdTuner.findAppState(TestScreen.class);
        assert screen.isEnabled();
        screen.closeAllPopups();
        screen.showTextEntryDialog("Enter volume percent error:", defaultValue,
                actionPrefix, controller);
    }

    /**
     * Process an ongoing action against prefixes.
     *
     * @param actionString textual description of the action (not null)
     * @return true if the action is handled, otherwise false
     */
    private static boolean testForPrefixes(String actionString) {
        Model model = VhacdTuner.getModel();
        DecompositionTest leftTest = model.getLeftTest();
        DecompositionTest rightTest = model.getRightTest();

        boolean handled = true;
        String arg;
        if (actionString.startsWith(apSetAlphaLeft)) {
            arg = MyString.remainder(actionString, apSetAlphaLeft);
            double alpha = Double.parseDouble(arg);
            DecompositionTest test = leftTest.setAlpha(alpha);
            model.setLeftTest(test);

        } else if (actionString.startsWith(apSetAlphaRight)) {
            arg = MyString.remainder(actionString, apSetAlphaRight);
            double alpha = Double.parseDouble(arg);
            DecompositionTest test = rightTest.setAlpha(alpha);
            model.setRightTest(test);

        } else if (actionString.startsWith(apSetBetaLeft)) {
            arg = MyString.remainder(actionString, apSetBetaLeft);
            double beta = Double.parseDouble(arg);
            DecompositionTest test = leftTest.setBeta(beta);
            model.setLeftTest(test);

        } else if (actionString.startsWith(apSetBetaRight)) {
            arg = MyString.remainder(actionString, apSetBetaRight);
            double beta = Double.parseDouble(arg);
            DecompositionTest test = rightTest.setBeta(beta);
            model.setRightTest(test);

        } else if (actionString.startsWith(apSetHullDSLeft)) {
            arg = MyString.remainder(actionString, apSetHullDSLeft);
            int hullDS = Integer.parseInt(arg);
            DecompositionTest test = leftTest.setHullDS(hullDS);
            model.setLeftTest(test);

        } else if (actionString.startsWith(apSetHullDSRight)) {
            arg = MyString.remainder(actionString, apSetHullDSRight);
            int hullDS = Integer.parseInt(arg);
            DecompositionTest test = rightTest.setHullDS(hullDS);
            model.setRightTest(test);

        } else if (actionString.startsWith(apSetMaxConcavityLeft)) {
            arg = MyString.remainder(actionString, apSetMaxConcavityLeft);
            double maxConcavity = Double.parseDouble(arg);
            DecompositionTest test = leftTest.setMaxConcavity(maxConcavity);
            model.setLeftTest(test);

        } else if (actionString.startsWith(apSetMaxConcavityRight)) {
            arg = MyString.remainder(actionString, apSetMaxConcavityRight);
            double maxConcavity = Double.parseDouble(arg);
            DecompositionTest test = rightTest.setMaxConcavity(maxConcavity);
            model.setRightTest(test);

        } else if (actionString.startsWith(apSetMaxHullsLeft)) {
            arg = MyString.remainder(actionString, apSetMaxHullsLeft);
            int maxHulls = Integer.parseInt(arg);
            DecompositionTest test = leftTest.setMaxHulls(maxHulls);
            model.setLeftTest(test);

        } else if (actionString.startsWith(apSetMaxHullsRight)) {
            arg = MyString.remainder(actionString, apSetMaxHullsRight);
            int maxHulls = Integer.parseInt(arg);
            DecompositionTest test = rightTest.setMaxHulls(maxHulls);
            model.setRightTest(test);

        } else if (actionString.startsWith(apSetMaxRecursionLeft)) {
            arg = MyString.remainder(actionString, apSetMaxRecursionLeft);
            int depth = Integer.parseInt(arg);
            DecompositionTest test = leftTest.setMaxRecursion(depth);
            model.setLeftTest(test);

        } else if (actionString.startsWith(apSetMaxRecursionRight)) {
            arg = MyString.remainder(actionString, apSetMaxRecursionRight);
            int depth = Integer.parseInt(arg);
            DecompositionTest test = rightTest.setMaxRecursion(depth);
            model.setRightTest(test);

        } else if (actionString.startsWith(apSetMaxVerticesPHLeft)) {
            arg = MyString.remainder(actionString, apSetMaxVerticesPHLeft);
            int limit = Integer.parseInt(arg);
            DecompositionTest test = leftTest.setMaxVerticesPerHull(limit);
            model.setLeftTest(test);

        } else if (actionString.startsWith(apSetMaxVerticesPHRight)) {
            arg = MyString.remainder(actionString, apSetMaxVerticesPHRight);
            int limit = Integer.parseInt(arg);
            DecompositionTest test = rightTest.setMaxVerticesPerHull(limit);
            model.setRightTest(test);

        } else if (actionString.startsWith(apSetMinEdgeLengthLeft)) {
            arg = MyString.remainder(actionString, apSetMinEdgeLengthLeft);
            int length = Integer.parseInt(arg);
            DecompositionTest test = leftTest.setMinEdgeLength(length);
            model.setLeftTest(test);

        } else if (actionString.startsWith(apSetMinEdgeLengthRight)) {
            arg = MyString.remainder(actionString, apSetMinEdgeLengthRight);
            int length = Integer.parseInt(arg);
            DecompositionTest test = rightTest.setMinEdgeLength(length);
            model.setRightTest(test);

        } else if (actionString.startsWith(apSetMinVolumePHLeft)) {
            arg = MyString.remainder(actionString, apSetMinVolumePHLeft);
            double volume = Double.parseDouble(arg);
            DecompositionTest test = leftTest.setMinVolumePH(volume);
            model.setLeftTest(test);

        } else if (actionString.startsWith(apSetMinVolumePHRight)) {
            arg = MyString.remainder(actionString, apSetMinVolumePHRight);
            double volume = Double.parseDouble(arg);
            DecompositionTest test = rightTest.setMinVolumePH(volume);
            model.setRightTest(test);

        } else if (actionString.startsWith(apSetPlaneDSLeft)) {
            arg = MyString.remainder(actionString, apSetPlaneDSLeft);
            int planeDS = Integer.parseInt(arg);
            DecompositionTest test = leftTest.setPlaneDS(planeDS);
            model.setLeftTest(test);

        } else if (actionString.startsWith(apSetPlaneDSRight)) {
            arg = MyString.remainder(actionString, apSetPlaneDSRight);
            int hullDS = Integer.parseInt(arg);
            DecompositionTest test = rightTest.setPlaneDS(hullDS);
            model.setRightTest(test);

        } else if (actionString.startsWith(apSetResolutionLeft)) {
            arg = MyString.remainder(actionString, apSetResolutionLeft);
            int resolution = Integer.parseInt(arg);
            DecompositionTest test = leftTest.setResolution(resolution);
            model.setLeftTest(test);

        } else if (actionString.startsWith(apSetResolutionRight)) {
            arg = MyString.remainder(actionString, apSetResolutionRight);
            int resolution = Integer.parseInt(arg);
            DecompositionTest test = rightTest.setResolution(resolution);
            model.setRightTest(test);

        } else if (actionString.startsWith(apSetVolumePercentErrorLeft)) {
            arg = MyString.remainder(actionString, apSetVolumePercentErrorLeft);
            double percent = Double.parseDouble(arg);
            DecompositionTest test = leftTest.setVolumePercentError(percent);
            model.setLeftTest(test);

        } else if (actionString.startsWith(apSetVolumePercentErrorRight)) {
            arg = MyString.remainder(
                    actionString, apSetVolumePercentErrorRight);
            double percent = Double.parseDouble(arg);
            DecompositionTest test = rightTest.setVolumePercentError(percent);
            model.setRightTest(test);

        } else {
            handled = false;
        }

        return handled;
    }
}
