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
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.cursors.plugins.JmeCursor;
import com.jme3.export.JmeExporter;
import com.jme3.export.binary.BinaryExporter;
import com.jme3.input.KeyInput;
import com.jme3.scene.Spatial;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintStream;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MyString;
import jme3utilities.Validate;
import jme3utilities.ui.ActionApplication;
import jme3utilities.ui.InputMode;

/**
 * Input mode for the "save" screen of VhacdTuner.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class SaveMode extends InputMode {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final static Logger logger = Logger.getLogger(SaveMode.class.getName());
    /**
     * asset path to the cursor for this mode
     */
    final private static String assetPath = "Textures/cursors/default.cur";
    /**
     * action strings specific to this input mode:
     */
    final private static String asSaveJ3o = "save j3o";
    final private static String asSaveJava = "save java";
    // *************************************************************************
    // constructors

    /**
     * Instantiate a disabled, uninitialized mode.
     */
    SaveMode() {
        super("save");
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

        bind(Action.dumpRenderer, KeyInput.KEY_P);
        bind(Action.previousScreen, KeyInput.KEY_PGUP, KeyInput.KEY_B);
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
            handled = true;
            switch (actionString) {
                case Action.previousScreen:
                    previousScreen();
                    break;

                case asSaveJava:
                    saveJava();
                    break;

                case asSaveJ3o:
                    saveJ3o();
                    break;

                default:
                    handled = false;
            }
        }
        if (!handled) {
            getActionApplication().onAction(actionString, ongoing, tpf);
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Go back to the TestScreen.
     */
    private void previousScreen() {
        setEnabled(false);
        InputMode test = InputMode.findMode("test");
        test.setEnabled(true);
    }

    /**
     * Write the best decomposition to a file, along with the C-G model.
     */
    private static void saveJ3o() {
        Model model = VhacdTuner.getModel();
        DecompositionTest best = model.findRankedTest(0);
        CollisionShape bestShape = best.getShape();
        RigidBodyControl rbc = new RigidBodyControl(bestShape);

        String originalPath = model.filePath();
        File originalFile = new File(originalPath);
        String modelName = originalFile.getName();
        if (modelName.endsWith(".j3o")) {
            modelName = MyString.removeSuffix(modelName, ".j3o");
        } else if (modelName.endsWith(".glb")) {
            modelName = MyString.removeSuffix(modelName, ".glb");
        } else if (modelName.endsWith(".gltf")) {
            modelName = MyString.removeSuffix(modelName, ".gltf");
        }

        String hhmmss = ActionApplication.hhmmss();
        String outputFileName = String.format("%s-%s.j3o", modelName, hhmmss);
        String outputFilePath = ActionApplication.filePath(outputFileName);

        Spatial modelRoot = model.getRootSpatial();
        modelRoot = Heart.deepCopy(modelRoot);
        modelRoot.addControl(rbc);

        JmeExporter exporter = BinaryExporter.getInstance();
        File outputFile = new File(outputFilePath);
        SaveScreen screen = VhacdTuner.findAppState(SaveScreen.class);
        assert screen.isEnabled();

        try {
            exporter.save(modelRoot, outputFile);
        } catch (IOException exception) {
            screen.showInfoDialog("Exception", exception.toString());
            return;
        }

        // Display a confirmation dialog.
        String message = String.format(
                "The model and configured control have been written to%n%s.",
                MyString.quote(outputFilePath));
        screen.showInfoDialog("Success", message);
    }

    /**
     * Write the best parameters to a file.
     */
    private static void saveJava() {
        Model model = VhacdTuner.getModel();
        DecompositionTest best = model.findRankedTest(0);

        String hhmmss = ActionApplication.hhmmss();
        String fileName = String.format("configure%s.java", hhmmss);

        SaveScreen screen = VhacdTuner.findAppState(SaveScreen.class);
        assert screen.isEnabled();

        String path = ActionApplication.filePath(fileName);
        File file = new File(path);
        try (PrintStream stream = new PrintStream(file)) {
            best.write(stream);
        } catch (FileNotFoundException exception) {
            screen.showInfoDialog("Exception", exception.toString());
            return;
        }

        // Display a confirmation dialog.
        String message = String.format(
                "The parameters have been written to%n%s.",
                MyString.quote(path));
        screen.showInfoDialog("Success", message);
    }
}
