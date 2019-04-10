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

import com.jme3.app.Application;
import com.jme3.app.SimpleApplication;
import com.jme3.app.state.AppStateManager;
import com.jme3.asset.AssetManager;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.animation.DacConfiguration;
import com.jme3.bullet.animation.DynamicAnimControl;
import com.jme3.bullet.animation.LinkConfig;
import com.jme3.bullet.animation.RangeOfMotion;
import com.jme3.bullet.animation.TorsoLink;
import com.jme3.cursors.plugins.JmeCursor;
import com.jme3.input.CameraInput;
import com.jme3.input.KeyInput;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.system.JmeVersion;
import de.lessvoid.nifty.Nifty;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Misc;
import jme3utilities.MyString;
import jme3utilities.Validate;
import jme3utilities.minie.MinieVersion;
import jme3utilities.nifty.LibraryVersion;
import jme3utilities.ui.InputMode;
import jme3utilities.ui.UiVersion;

/**
 * Input mode for the "test" screen of DacWizard.
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
     * asset path to the cursor for this mode
     */
    final private static String assetPath = "Textures/cursors/default.cur";
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

        bind("signal " + CameraInput.FLYCAM_RISE, KeyInput.KEY_Q);
        bind("signal " + CameraInput.FLYCAM_FORWARD, KeyInput.KEY_W);
        bind(Action.dumpPhysicsSpace, KeyInput.KEY_O);
        bind(Action.dumpRenderer, KeyInput.KEY_P);

        bind("signal orbitLeft", KeyInput.KEY_A);
        bind("signal " + CameraInput.FLYCAM_BACKWARD, KeyInput.KEY_S);
        bind("signal orbitRight", KeyInput.KEY_D);

        bind("signal " + CameraInput.FLYCAM_LOWER, KeyInput.KEY_Z);
        bind(SimpleApplication.INPUT_MAPPING_CAMERA_POS, KeyInput.KEY_C);
        bind(Action.togglePhysicsDebug, KeyInput.KEY_SLASH);
    }

    /**
     * Initialize this (disabled) mode prior to its 1st update.
     *
     * @param stateManager (not null)
     * @param application (not null)
     */
    @Override
    public void initialize(AppStateManager stateManager,
            Application application) {
        /*
         * Set the mouse cursor for this mode.
         */
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
        logger.log(Level.INFO, "Got action {0} ongoing={1}", new Object[]{
            MyString.quote(actionString), ongoing
        });

        boolean handled = false;
        if (ongoing) {
            switch (actionString) {
                case Action.previousScreen:
                    previousScreen();
                    handled = true;
                    break;

                case Action.save:
                    save();
                    handled = true;
                    break;

                case Action.togglePhysicsDebug:
                    togglePhysicsDebug();
                    handled = true;
                    break;

                case Action.toggleRagdoll:
                    toggleRagdoll();
                    handled = true;
                    break;
            }
        }
        if (!handled) {
            actionApplication.onAction(actionString, ongoing, tpf);
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Format a LinkConfig as Java source code.
     *
     * @param config (not null, unaffected)
     */
    private String format(LinkConfig config) {
        Vector3f scale = config.shapeScale(null);
        String scaleXString = MyString.describe(scale.x);
        String scaleYString = MyString.describe(scale.y);
        String scaleZString = MyString.describe(scale.z);

        float massP = config.massParameter();
        String massPString = MyString.describe(massP);

        String code = String.format(
                "new LinkConfig(%sf, MassHeuristic.%s, "
                + "ShapeHeuristic.%s, new Vector3f(%sf, %sf, %sf), "
                + "CenterHeuristic.%s)",
                massPString, config.massHeuristic(),
                config.shapeHeuristic(),
                scaleXString, scaleYString, scaleZString,
                config.centerHeuristic());
        return code;
    }

    /**
     * Go back to the previous screen.
     */
    private void previousScreen() {
        setEnabled(false);
        InputMode bones = InputMode.findMode("links");
        bones.setEnabled(true);
    }

    /**
     * Save the control configuration to a file.
     */
    private void save() {
        DacWizard wizard = DacWizard.getApplication();
        DynamicAnimControl dac = wizard.findRagdoll();
        String path = DacWizard.filePath("configure.java");
        File file = new File(path);
        try {
            PrintStream stream = new PrintStream(file);
            write(dac, stream);
        } catch (FileNotFoundException exception) {
            TestScreen screen = DacWizard.findAppState(TestScreen.class);
            screen.showInfoDialog("Exception", exception.toString());
        }
    }

    /**
     * Toggle physics-debug visualization on/off.
     */
    private void togglePhysicsDebug() {
        BulletAppState bulletAppState
                = DacWizard.findAppState(BulletAppState.class);
        boolean enabled = bulletAppState.isDebugEnabled();
        bulletAppState.setDebugEnabled(!enabled);
    }

    /**
     * Toggle ragdoll mode.
     */
    private void toggleRagdoll() {
        DacWizard wizard = DacWizard.getApplication();
        DynamicAnimControl dac = wizard.findRagdoll();
        TorsoLink torso = dac.getTorsoLink();
        if (torso.isKinematic()) {
            dac.setRagdollMode();
        } else { // reset to bind pose
            dac.blendToKinematicMode(1f, Transform.IDENTITY);
        }
    }

    /**
     * Write the control configuration to a stream.
     */
    private void write(DynamicAnimControl dac, PrintStream stream) {

        String torsoName = DacConfiguration.torsoName;
        LinkConfig config = dac.config(torsoName);
        String newConfig = format(config);
        String code = String.format("        setConfig(%s, %s);%n",
                MyString.quote(torsoName), newConfig);
        stream.print(code);

        String[] lbNames = dac.listLinkedBoneNames();
        for (String lbName : lbNames) {
            config = dac.config(lbName);
            newConfig = format(config);

            RangeOfMotion range = dac.getJointLimits(lbName);

            float maxX = range.getMaxRotation(PhysicsSpace.AXIS_X);
            String maxXString = MyString.describe(maxX);
            float minX = range.getMinRotation(PhysicsSpace.AXIS_X);
            String minXString = MyString.describe(minX);

            float maxY = range.getMaxRotation(PhysicsSpace.AXIS_Y);
            String maxYString = MyString.describe(maxY);
            float minY = range.getMinRotation(PhysicsSpace.AXIS_Y);
            String minYString = MyString.describe(minY);

            float maxZ = range.getMaxRotation(PhysicsSpace.AXIS_Z);
            String maxZString = MyString.describe(maxZ);
            float minZ = range.getMinRotation(PhysicsSpace.AXIS_Z);
            String minZString = MyString.describe(minZ);

            String newRange = String.format("new RangeOfMotion("
                    + "%sf, %sf, %sf, %sf, %sf, %sf)",
                    maxXString, minXString,
                    maxYString, minYString,
                    maxZString, minZString);

            code = String.format("        setConfig(%s, %s, %s);%n",
                    MyString.quote(lbName), newConfig, newRange);
            stream.print(code);
        }
    }
}
