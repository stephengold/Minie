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
import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.Calendar;
import java.util.Map;
import java.util.TreeMap;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.MyString;
import jme3utilities.Validate;
import jme3utilities.ui.InputMode;

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

                case Action.toggleMesh:
                    TestScreen screen
                            = DacWizard.findAppState(TestScreen.class);
                    screen.toggleMesh();
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
     * Generate a textual description of a single-precision floating-point value
     * using at most 2 decimal places.
     *
     * @param fValue the value to describe
     * @return a description (not null, not empty)
     */
    private static String describeAngle(float fValue) {
        String raw = String.format("%.2f", fValue);
        String result = MyString.trimFloat(raw);

        assert result != null;
        assert !result.isEmpty();
        return result;
    }

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
                "new LinkConfig(%sf, MassHeuristic.%s,%n"
                + "                ShapeHeuristic.%s, "
                + "new Vector3f(%sf, %sf, %sf),%n"
                + "                CenterHeuristic.%s)",
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
     * Write the control configuration to a file.
     */
    private void save() {
        Calendar rightNow = Calendar.getInstance();
        int hours = rightNow.get(Calendar.HOUR_OF_DAY);
        int minutes = rightNow.get(Calendar.MINUTE);
        int seconds = rightNow.get(Calendar.SECOND);
        String hhmmss = String.format("%02d%02d%02d", hours, minutes, seconds);
        String fileName = String.format("configure%s.java", hhmmss);

        DacWizard wizard = DacWizard.getApplication();
        DynamicAnimControl dac = wizard.findDac();
        TestScreen screen = DacWizard.findAppState(TestScreen.class);

        String path = DacWizard.filePath(fileName);
        File file = new File(path);
        try (PrintStream stream = new PrintStream(file)) {
            write(dac, stream);
        } catch (FileNotFoundException exception) {
            screen.showInfoDialog("Exception", exception.toString());
            return;
        }
        /*
         * Display a confirmation dialog.
         */
        String message = String.format(
                "The configuration has been written to%n%s.",
                MyString.quote(path));
        screen.showInfoDialog("Success", message);
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
        DynamicAnimControl dac = wizard.findDac();
        TorsoLink torso = dac.getTorsoLink();
        if (torso.isKinematic()) {
            dac.setRagdollMode();
        } else { // reset to bind pose
            dac.blendToKinematicMode(1f, Transform.IDENTITY);
        }
    }

    /**
     * Write the control configuration to a stream, as Java source code.
     *
     * @param dac a configured control to use as a model (not null, unaffected)
     * @param stream the output stream (not null)
     */
    private void write(DynamicAnimControl dac, PrintStream stream) {
        stream.printf("import com.jme3.bullet.animation.CenterHeuristic;%n"
                + "import com.jme3.bullet.animation.DynamicAnimControl;%n"
                + "import com.jme3.bullet.animation.LinkConfig;%n"
                + "import com.jme3.bullet.animation.MassHeuristic;%n"
                + "import com.jme3.bullet.animation.RangeOfMotion;%n"
                + "import com.jme3.bullet.animation.ShapeHeuristic;%n"
                + "import com.jme3.math.Vector3f;%n%n");
        stream.printf("public class WControl extends DynamicAnimControl {%n%n"
                + "    public WControl() {%n"
                + "        super();%n");

        String[] lbNames = dac.listLinkedBoneNames();
        String torsoName = DacConfiguration.torsoName;
        /*
         * Write each unique LinkConfig.
         */
        int nextConfigIndex = 1;
        Map<LinkConfig, Integer> configs = new TreeMap<>();

        LinkConfig config = dac.config(torsoName);
        if (!configs.containsKey(config)) {
            configs.put(config, nextConfigIndex);
            writeConfig(config, nextConfigIndex, stream);
            ++nextConfigIndex;
        }

        for (String lbName : lbNames) {
            config = dac.config(lbName);
            if (!configs.containsKey(config)) {
                configs.put(config, nextConfigIndex);
                writeConfig(config, nextConfigIndex, stream);
                ++nextConfigIndex;
            }
        }
        /*
         * Configure the torso of the ragdoll.
         */
        config = dac.config(torsoName);
        int configIndex = configs.get(config);
        String code = String.format("        super.setConfig(%s, config%d);%n",
                MyString.quote(torsoName), configIndex);
        stream.print(code);
        /*
         * Configure each linked bone in the ragdoll.
         */
        for (String lbName : lbNames) {
            config = dac.config(lbName);
            configIndex = configs.get(config);

            RangeOfMotion range = dac.getJointLimits(lbName);

            float maxX = range.getMaxRotation(PhysicsSpace.AXIS_X);
            String maxXString = describeAngle(maxX);
            float minX = range.getMinRotation(PhysicsSpace.AXIS_X);
            String minXString = describeAngle(minX);

            float maxY = range.getMaxRotation(PhysicsSpace.AXIS_Y);
            String maxYString = describeAngle(maxY);
            float minY = range.getMinRotation(PhysicsSpace.AXIS_Y);
            String minYString = describeAngle(minY);

            float maxZ = range.getMaxRotation(PhysicsSpace.AXIS_Z);
            String maxZString = describeAngle(maxZ);
            float minZ = range.getMinRotation(PhysicsSpace.AXIS_Z);
            String minZString = describeAngle(minZ);

            String newRange = String.format("new RangeOfMotion("
                    + "%sf, %sf, %sf, %sf, %sf, %sf)",
                    maxXString, minXString,
                    maxYString, minYString,
                    maxZString, minZString);

            code = String.format("        super.link(%s, config%d,%n"
                    + "                %s);%n",
                    MyString.quote(lbName), configIndex, newRange);
            stream.print(code);
        }

        stream.printf("    }%n}%n");
    }

    /**
     * Write a LinkConfig definition to a stream, as Java source code.
     *
     * @param config a LinkConfig to use as a model (not null)
     * @param configIndex the index into unique link configurations (&gt;0)
     * @param stream the output stream (not null)
     */
    private void writeConfig(LinkConfig config, int configIndex,
            PrintStream stream) {
        assert config != null;
        assert configIndex > 0 : configIndex;

        String newValue = format(config);
        stream.printf("        LinkConfig config%d = %s;%n",
                configIndex, newValue);
    }
}
