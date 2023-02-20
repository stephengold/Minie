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

import com.jme3.app.Application;
import com.jme3.app.SimpleApplication;
import com.jme3.app.state.AppStateManager;
import com.jme3.asset.AssetManager;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.RotationOrder;
import com.jme3.bullet.animation.BoneLink;
import com.jme3.bullet.animation.DacConfiguration;
import com.jme3.bullet.animation.DynamicAnimControl;
import com.jme3.bullet.animation.KinematicSubmode;
import com.jme3.bullet.animation.LinkConfig;
import com.jme3.bullet.animation.PhysicsLink;
import com.jme3.bullet.animation.RangeOfMotion;
import com.jme3.bullet.animation.TorsoLink;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.PhysicsRayTestResult;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.cursors.plugins.JmeCursor;
import com.jme3.export.JmeExporter;
import com.jme3.export.binary.BinaryExporter;
import com.jme3.input.CameraInput;
import com.jme3.input.KeyInput;
import com.jme3.math.Transform;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.scene.Spatial;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintStream;
import java.util.Calendar;
import java.util.Collection;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.TreeMap;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MySpatial;
import jme3utilities.MyString;
import jme3utilities.Validate;
import jme3utilities.nifty.dialog.AllowNull;
import jme3utilities.nifty.dialog.FloatDialog;
import jme3utilities.ui.ActionApplication;
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
    /**
     * local transform of the controlled spatial when entering/exiting ragdoll
     * mode
     */
    final private Transform resetTransform = new Transform();
    // *************************************************************************
    // constructors

    /**
     * Instantiate a disabled, uninitialized input mode.
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
        bind(Action.toggleSkeleton, KeyInput.KEY_V);
        bind(Action.togglePhysicsDebug, KeyInput.KEY_SLASH);

        bind(Action.pickLink, "RMB");
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
            logger.log(Level.INFO, "Got action {0} ongoing={1}", new Object[]{
                MyString.quote(actionString), ongoing
            });
        }

        boolean handled = false;
        if (ongoing) {
            handled = true;
            DacWizard app = DacWizard.getApplication();
            switch (actionString) {
                case Action.pickLink:
                    pickLink();
                    break;

                case Action.previousScreen:
                    previousScreen();
                    break;

                case Action.save:
                    saveJava();
                    break;

                case Action.saveJ3o:
                    saveJ3o();
                    break;

                case Action.setMargin:
                    setMargin();
                    break;

                case Action.toggleAxes:
                    Model model = DacWizard.getModel();
                    model.toggleShowingAxes();
                    break;

                case Action.toggleMesh:
                    app.toggleMesh();
                    break;

                case Action.togglePhysicsDebug:
                    togglePhysicsDebug();
                    break;

                case Action.toggleRagdoll:
                    toggleRagdoll();
                    break;

                case Action.toggleSkeleton:
                    DacWizard.toggleSkeletonVisualizer();
                    break;

                default:
                    handled = false;
            }
        }
        if (!handled) {
            if (actionString.startsWith(Action.setMargin + " ")) {
                String arg = MyString
                        .remainder(actionString, Action.setMargin + " ");
                float newMargin = Float.parseFloat(arg);
                setMargin(newMargin);
                handled = true;
            }
        }
        if (!handled) {
            getActionApplication().onAction(actionString, ongoing, tpf);
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
    private static String describeAngle(float fValue) { // TODO move to MyString
        String raw = String.format(Locale.US, "%.2f", fValue);
        String result = MyString.trimFloat(raw);

        assert result != null;
        assert !result.isEmpty();
        return result;
    }

    /**
     * Format a LinkConfig as Java source code.
     *
     * @param config (not null, unaffected)
     * @return the formatted text (not null, not empty)
     */
    private static String format(LinkConfig config) {
        Vector3f scale = config.shapeScale(null);
        String scaleXString = MyString.describe(scale.x);
        String scaleYString = MyString.describe(scale.y);
        String scaleZString = MyString.describe(scale.z);

        float massP = config.massParameter();
        String massPString = MyString.describe(massP);

        RotationOrder ro = config.rotationOrder();
        String orderString = (ro == null) ? "null" : "RotationOrder." + ro;

        String code = String.format(
                "new LinkConfig(%sf, MassHeuristic.%s,%n"
                + "                ShapeHeuristic.%s, "
                + "new Vector3f(%sf, %sf, %sf),%n"
                + "                CenterHeuristic.%s, %s)",
                massPString, config.massHeuristic(),
                config.shapeHeuristic(),
                scaleXString, scaleYString, scaleZString,
                config.centerHeuristic(), orderString);

        return code;
    }

    /**
     * Generate a timestamp.
     *
     * @return the timestamp value
     */
    private static String hhmmss() {
        Calendar rightNow = Calendar.getInstance();
        int hours = rightNow.get(Calendar.HOUR_OF_DAY);
        int minutes = rightNow.get(Calendar.MINUTE);
        int seconds = rightNow.get(Calendar.SECOND);
        String result = String.format("%02d%02d%02d", hours, minutes, seconds);

        return result;
    }

    /**
     * Cast a physics ray from the mouse pointer. If the nearest hit is a
     * PhysicsLink, select that link.
     */
    private void pickLink() {
        Vector2f screenXY = inputManager.getCursorPosition();
        Vector3f from = cam.getWorldCoordinates(screenXY, 0f);
        Vector3f to = cam.getWorldCoordinates(screenXY, 1f);

        BulletAppState bulletAppState
                = DacWizard.findAppState(BulletAppState.class);
        PhysicsSpace physicsSpace = bulletAppState.getPhysicsSpace();

        List<PhysicsRayTestResult> rayTest = physicsSpace.rayTest(from, to);
        if (!rayTest.isEmpty()) {
            PhysicsRayTestResult nearestHit = rayTest.get(0);
            PhysicsCollisionObject pco = nearestHit.getCollisionObject();
            Object user = pco.getUserObject();
            if (user instanceof PhysicsLink) {
                PhysicsLink link = (PhysicsLink) user;
                Model model = DacWizard.getModel();

                if (link instanceof BoneLink) {
                    String boneName = link.boneName();
                    model.selectLink(boneName);
                } else {
                    assert link instanceof TorsoLink;
                    model.selectLink(DacConfiguration.torsoName);
                }
            }
        }
    }

    /**
     * Go back to the "links" screen.
     */
    private void previousScreen() {
        setEnabled(false);
        InputMode links = InputMode.findMode("links");
        links.setEnabled(true);
    }

    /**
     * Write the model to a file, along with a configured control.
     */
    private static void saveJ3o() {
        Model model = DacWizard.getModel();

        String originalPath = model.filePath();
        File originalFile = new File(originalPath);
        String modelName = originalFile.getName();
        if (modelName.endsWith(".j3o")) {
            modelName = MyString.removeSuffix(modelName, ".j3o");
        }
        String hhmmss = hhmmss();
        String outputFileName = String.format("%s-%s.j3o", modelName, hhmmss);
        String outputFilePath = ActionApplication.filePath(outputFileName);

        Spatial modelRoot = model.getRootSpatial();
        modelRoot = Heart.deepCopy(modelRoot);
        List<Spatial> list = MySpatial.listAnimationSpatials(modelRoot, null);
        assert list.size() == 1 : list.size();
        Spatial controlledSpatial = list.get(0);
        DynamicAnimControl dac = model.copyRagdoll();
        controlledSpatial.addControl(dac);

        JmeExporter exporter = BinaryExporter.getInstance();
        File outputFile = new File(outputFilePath);
        TestScreen screen = DacWizard.findAppState(TestScreen.class);
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
     * Write the control configuration to a file.
     */
    private void saveJava() {
        String hhmmss = hhmmss();
        String className = "Dac" + hhmmss;
        String fileName = className + ".java";

        DacWizard wizard = DacWizard.getApplication();
        DynamicAnimControl dac = wizard.findDac();
        TestScreen screen = DacWizard.findAppState(TestScreen.class);

        String path = ActionApplication.filePath(fileName);
        File file = new File(path);
        try (PrintStream stream = new PrintStream(file)) {
            write(dac, className, stream);
        } catch (FileNotFoundException exception) {
            screen.showInfoDialog("Exception", exception.toString());
            return;
        }

        // Display a confirmation dialog.
        String message = String.format(
                "The configuration has been written to%n%s.",
                MyString.quote(path));
        screen.showInfoDialog("Success", message);
    }

    /**
     * Open a dialog box to change the collision margin.
     */
    private static void setMargin() {
        float oldValue = CollisionShape.getDefaultMargin();
        String defaultValue = Float.toString(oldValue);
        FloatDialog controller = new FloatDialog(
                "Set", Float.MIN_VALUE, Float.MAX_VALUE, AllowNull.No);

        TestScreen screen = DacWizard.findAppState(TestScreen.class);
        screen.closeAllPopups();
        screen.showTextEntryDialog("Enter new margin:", defaultValue,
                Action.setMargin + " ", controller);
    }

    /**
     * Alter the default collision margin and also the margin of every shape.
     *
     * @param newMargin the desired value (&gt;0)
     */
    private static void setMargin(float newMargin) {
        CollisionShape.setDefaultMargin(newMargin);

        BulletAppState bulletAppState
                = DacWizard.findAppState(BulletAppState.class);
        PhysicsSpace physicsSpace = bulletAppState.getPhysicsSpace();
        Collection<PhysicsCollisionObject> list
                = physicsSpace.getPcoList();
        for (PhysicsCollisionObject pco : list) {
            pco.getCollisionShape().setMargin(newMargin);
        }
    }

    /**
     * Toggle physics-debug visualization on/off.
     */
    private static void togglePhysicsDebug() {
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
            Spatial controlledSpatial = dac.getSpatial();
            Transform local = controlledSpatial.getLocalTransform();
            resetTransform.set(local);
            dac.setRagdollMode();

        } else { // reset to bind pose
            KinematicSubmode bindPose = KinematicSubmode.Bound;
            float blendInterval = 1f; // in seconds
            torso.blendToKinematicMode(bindPose, blendInterval, resetTransform);
            Collection<BoneLink> boneLinks = dac.listLinks(BoneLink.class);
            for (BoneLink boneLink : boneLinks) {
                boneLink.blendToKinematicMode(bindPose, blendInterval);
            }
        }
    }

    /**
     * Write the control configuration to a stream, as Java source code.
     *
     * @param dac a configured control to reproduce (not null, unaffected)
     * @param className name for the Java class (not null, not empty)
     * @param stream the output stream (not null)
     */
    private static void write(
            DacConfiguration dac, String className, PrintStream stream) {
        stream.printf("import com.jme3.bullet.RotationOrder;%n"
                + "import com.jme3.bullet.animation.CenterHeuristic;%n"
                + "import com.jme3.bullet.animation.DacConfiguration;%n"
                + "import com.jme3.bullet.animation.DynamicAnimControl;%n"
                + "import com.jme3.bullet.animation.LinkConfig;%n"
                + "import com.jme3.bullet.animation.MassHeuristic;%n"
                + "import com.jme3.bullet.animation.RangeOfMotion;%n"
                + "import com.jme3.bullet.animation.ShapeHeuristic;%n"
                + "import com.jme3.math.Vector3f;%n%n");
        stream.printf("public class %s extends DynamicAnimControl ", className);
        stream.printf("{%n%n    public %s() {%n", className);
        stream.println("        super();");

        // Write each unique LinkConfig.
        Map<LinkConfig, Integer> configs = writeLinkConfigs(dac, stream);

        // Configure the torso of the ragdoll.
        String torsoName = DacConfiguration.torsoName;
        LinkConfig config = dac.config(torsoName);
        int configIndex = configs.get(config);
        String code = String.format("        super.setConfig(%s, config%d);%n",
                MyString.quote(torsoName), configIndex);
        stream.print(code);

        writeConfigureBoneLinks(dac, "super", configs, stream);
        stream.printf("    }%n%n");

        stream.println(
                "    public static void configure(DacConfiguration dac) {");

        // Write each unique LinkConfig.
        configs = writeLinkConfigs(dac, stream);

        // Configure the torso of the ragdoll.
        config = dac.config(torsoName);
        configIndex = configs.get(config);
        code = String.format("        dac.setConfig(%s, config%d);%n",
                MyString.quote(torsoName), configIndex);
        stream.print(code);

        writeConfigureBoneLinks(dac, "dac", configs, stream);
        stream.printf("    }%n}%n");
    }

    /**
     * Write a LinkConfig definition to a stream, as Java source code.
     *
     * @param config a LinkConfig to use as a model (not null)
     * @param configIndex the index into unique link configurations (&gt;0)
     * @param stream the output stream (not null)
     */
    private static void writeConfig(
            LinkConfig config, int configIndex, PrintStream stream) {
        assert config != null;
        assert configIndex > 0 : configIndex;

        String newValue = format(config);
        stream.printf(
                "        LinkConfig config%d = %s;%n", configIndex, newValue);
    }

    /**
     * Write code to configure each linked bone in the ragdoll.
     *
     * @param dac a configured control to reproduce (not null, unaffected)
     * @param varName the name of the Java variable to configure (not null, not
     * empty)
     * @param configs map from unique link configurations to indices (not null,
     * unaffected)
     * @param stream the output stream (not null)
     */
    private static void writeConfigureBoneLinks(
            DacConfiguration dac, String varName,
            Map<LinkConfig, Integer> configs, PrintStream stream) {
        String[] lbNames = dac.listLinkedBoneNames();

        for (String lbName : lbNames) {
            LinkConfig config = dac.config(lbName);
            int configIndex = configs.get(config);
            stream.printf("        %s.link(%s, config%d,%n",
                    varName, MyString.quote(lbName), configIndex);

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
            stream.printf("                %s);%n", newRange);
        }
    }

    /**
     * Assign an index to each unique link configuration in the specified
     * control and write generative Java code to the specified stream.
     *
     * @param dac the control to analyze
     * @param stream the output stream (not null)
     * @return a new map from link configurations to indices (not null)
     */
    private static Map<LinkConfig, Integer> writeLinkConfigs(
            DacConfiguration dac, PrintStream stream) {
        int nextConfigIndex = 1;
        Map<LinkConfig, Integer> result = new TreeMap<>();

        LinkConfig config = dac.config(DacConfiguration.torsoName);
        if (!result.containsKey(config)) {
            result.put(config, nextConfigIndex);
            writeConfig(config, nextConfigIndex, stream);
            ++nextConfigIndex;
        }

        String[] lbNames = dac.listLinkedBoneNames();
        for (String lbName : lbNames) {
            config = dac.config(lbName);
            if (!result.containsKey(config)) {
                result.put(config, nextConfigIndex);
                writeConfig(config, nextConfigIndex, stream);
                ++nextConfigIndex;
            }
        }

        return result;
    }
}
