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
import com.jme3.app.state.AppStateManager;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.RotationOrder;
import com.jme3.bullet.animation.CenterHeuristic;
import com.jme3.bullet.animation.DacConfiguration;
import com.jme3.bullet.animation.LinkConfig;
import com.jme3.bullet.animation.MassHeuristic;
import com.jme3.bullet.animation.RangeOfMotion;
import com.jme3.bullet.animation.ShapeHeuristic;
import com.jme3.math.Vector3f;
import de.lessvoid.nifty.NiftyEventSubscriber;
import de.lessvoid.nifty.controls.Button;
import de.lessvoid.nifty.controls.SliderChangedEvent;
import de.lessvoid.nifty.controls.TreeBox;
import de.lessvoid.nifty.controls.TreeItem;
import de.lessvoid.nifty.elements.Element;
import java.util.List;
import java.util.logging.Logger;
import jme3utilities.InitialState;
import jme3utilities.MyString;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;
import jme3utilities.nifty.GuiScreenController;
import jme3utilities.nifty.PopupMenuBuilder;
import jme3utilities.nifty.SliderTransform;
import jme3utilities.nifty.dialog.AllowNull;
import jme3utilities.nifty.dialog.DialogController;
import jme3utilities.nifty.dialog.FloatDialog;
import jme3utilities.nifty.dialog.VectorDialog;
import jme3utilities.ui.InputMode;

/**
 * The screen controller for the "links" screen of DacWizard.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class LinksScreen extends GuiScreenController {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final static Logger logger = Logger.getLogger(LinksScreen.class.getName());
    // *************************************************************************
    // fields

    /**
     * element of GUI button to proceed to the "test" screen
     */
    private Element nextElement;
    /**
     * TreeBox to display links in the hierarchy
     */
    private TreeBox<LinkValue> treeBox;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an uninitialized, disabled screen that will not be enabled
     * during initialization.
     */
    LinksScreen() {
        super("links", "Interface/Nifty/screens/wizard/links.xml",
                InitialState.Disabled);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Determine user feedback (if any) regarding the "next screen" action.
     *
     * @return "" if ready to proceed, otherwise an explanatory message
     */
    static String feedback() {
        return "";
    }

    /**
     * Callback handler that Nifty invokes after a slider changes.
     *
     * @param sliderId Nifty element ID of the slider (not null)
     * @param event details of the event (not null, ignored)
     */
    @NiftyEventSubscriber(pattern = ".*Slider")
    public void linksScreenSliderChanged(
            final String sliderId, final SliderChangedEvent event) {
        Validate.nonNull(sliderId, "slider ID");
        assert sliderId.endsWith("Slider") : sliderId;
        Validate.nonNull(event, "event");

        if (!isIgnoreGuiChanges() && hasStarted()) {
            readSliders();
        }
    }

    /**
     * Handle a "next massHeuristic" action.
     */
    void nextMassHeuristic() {
        LinkConfig config = config();
        float massParameter = config.massParameter();
        MassHeuristic massHeuristic = config.massHeuristic();
        if (massHeuristic == MassHeuristic.Mass) {
            massHeuristic = MassHeuristic.Density;
        } else {
            massHeuristic = MassHeuristic.Mass;
        }
        ShapeHeuristic shapeHeuristic = config.shapeHeuristic();
        Vector3f shapeScale = config.shapeScale(null);
        CenterHeuristic centerHeuristic = config.centerHeuristic();
        RotationOrder axisOrder = config.rotationOrder();

        config = new LinkConfig(massParameter, massHeuristic, shapeHeuristic,
                shapeScale, centerHeuristic, axisOrder);
        setConfig(config);
    }

    /**
     * Handle a "select centerHeuristic" action with no argument.
     */
    void selectCenterHeuristic() {
        PopupMenuBuilder builder = new PopupMenuBuilder();

        LinkConfig config = config();
        String boneName = selectedLink();
        CenterHeuristic selected = config.centerHeuristic();
        for (CenterHeuristic heuristic : CenterHeuristic.values()) {
            if (boneName.equals(DacConfiguration.torsoName)
                    && heuristic == CenterHeuristic.Joint) {
                continue;
            }
            if (heuristic != selected) {
                String name = heuristic.toString();
                builder.add(name);
            }
        }

        showPopupMenu("select centerHeuristic ", builder);
    }

    /**
     * Handle a "select centerHeuristic" action with an argument.
     *
     * @param heuristic the desired heuristic (not null)
     */
    void selectCenterHeuristic(CenterHeuristic heuristic) {
        LinkConfig config = config();
        float massParameter = config.massParameter();
        MassHeuristic massHeuristic = config.massHeuristic();
        ShapeHeuristic shapeHeuristic = config.shapeHeuristic();
        Vector3f shapeScale = config.shapeScale(null);
        RotationOrder axisOrder = config.rotationOrder();

        config = new LinkConfig(massParameter, massHeuristic, shapeHeuristic,
                shapeScale, heuristic, axisOrder);
        setConfig(config);
    }

    /**
     * Determine which PhysicsLink is currently selected.
     *
     * @return the bone/torso name, or null if none selected
     */
    String selectedLink() {
        String result = null;
        List<TreeItem<LinkValue>> selectedLinks = treeBox.getSelection();
        if (!selectedLinks.isEmpty()) {
            assert selectedLinks.size() == 1;
            TreeItem<LinkValue> selectedItem = selectedLinks.get(0);
            LinkValue value = selectedItem.getValue();
            result = value.boneName();
        }

        return result;
    }

    /**
     * Handle a "select rotationOrder" action without an argument.
     */
    void selectRotationOrder() {
        PopupMenuBuilder builder = new PopupMenuBuilder();

        LinkConfig config = config();
        RotationOrder selected = config.rotationOrder();
        if (selected != null) {
            builder.add("sixdof");
        }
        for (RotationOrder axisOrder : RotationOrder.values()) {
            if (axisOrder != selected) {
                String name = axisOrder.toString();
                builder.add(name);
            }
        }

        showPopupMenu("select rotationOrder ", builder);
    }

    /**
     * Handle a "select rotationOrder" action with an argument.
     *
     * @param axisOrder may be null
     */
    void selectRotationOrder(RotationOrder axisOrder) {
        LinkConfig config = config();
        float massParameter = config.massParameter();
        MassHeuristic massHeuristic = config.massHeuristic();
        ShapeHeuristic shapeHeuristic = config.shapeHeuristic();
        Vector3f shapeScale = config.shapeScale(null);
        CenterHeuristic centerHeuristic = config.centerHeuristic();

        config = new LinkConfig(massParameter, massHeuristic, shapeHeuristic,
                shapeScale, centerHeuristic, axisOrder);
        setConfig(config);
    }

    /**
     * Handle a "select shapeHeuristic" action without an argument.
     */
    void selectShapeHeuristic() {
        PopupMenuBuilder builder = new PopupMenuBuilder();

        LinkConfig config = config();
        ShapeHeuristic selected = config.shapeHeuristic();
        for (ShapeHeuristic heuristic : ShapeHeuristic.values()) {
            if (heuristic != selected) {
                String name = heuristic.toString();
                builder.add(name);
            }
        }

        showPopupMenu("select shapeHeuristic ", builder);
    }

    /**
     * Handle a "select shapeHeuristic" action with an argument.
     *
     * @param heuristic the desired heuristic (not null)
     */
    void selectShapeHeuristic(ShapeHeuristic heuristic) {
        LinkConfig config = config();
        float massParameter = config.massParameter();
        MassHeuristic massHeuristic = config.massHeuristic();
        Vector3f shapeScale = config.shapeScale(null);
        CenterHeuristic centerHeuristic = config.centerHeuristic();
        RotationOrder axisOrder = config.rotationOrder();

        config = new LinkConfig(massParameter, massHeuristic, heuristic,
                shapeScale, centerHeuristic, axisOrder);
        setConfig(config);
    }

    /**
     * Handle a "set massParameter" action without an argument.
     */
    void setMassParameter() {
        LinkConfig config = config();
        float oldParameter = config.massParameter();
        String defaultText = Float.toString(oldParameter);
        String actionPrefix = "set massParameter ";
        DialogController controller = new FloatDialog(
                "Set", Float.MIN_VALUE, Float.MAX_VALUE, AllowNull.No);
        showTextEntryDialog("Enter the mass-parameter value:", defaultText,
                actionPrefix, controller);
    }

    /**
     * Handle a "set massParameter" action with an argument.
     *
     * @param value the desired parameter value (&gt;0)
     */
    void setMassParameter(float value) {
        LinkConfig config = config();
        MassHeuristic massHeuristic = config.massHeuristic();
        ShapeHeuristic shapeHeuristic = config.shapeHeuristic();
        Vector3f shapeScale = config.shapeScale(null);
        CenterHeuristic centerHeuristic = config.centerHeuristic();
        RotationOrder axisOrder = config.rotationOrder();

        config = new LinkConfig(value, massHeuristic, shapeHeuristic,
                shapeScale, centerHeuristic, axisOrder);
        setConfig(config);
    }

    /**
     * Handle a "set shapeScale" action without an argument.
     */
    void setShapeScale() {
        LinkConfig config = config();
        Vector3f oldParameter = config.shapeScale(null);
        String defaultText = oldParameter.toString();
        String actionPrefix = "set shapeScale ";
        DialogController controller = new VectorDialog("Set", 3, AllowNull.No);
        showTextEntryDialog("Enter the shapeScale value:", defaultText,
                actionPrefix, controller);
    }

    /**
     * Handle a "set shapeScale" action with an argument.
     *
     * @param scaleFactors the desired scale factors (not null, no negative
     * component, unaffected)
     */
    void setShapeScale(Vector3f scaleFactors) {
        LinkConfig config = config();
        MassHeuristic massHeuristic = config.massHeuristic();
        float massParameter = config.massParameter();
        ShapeHeuristic shapeHeuristic = config.shapeHeuristic();
        CenterHeuristic centerHeuristic = config.centerHeuristic();
        RotationOrder axisOrder = config.rotationOrder();

        config = new LinkConfig(massParameter, massHeuristic, shapeHeuristic,
                scaleFactors, centerHeuristic, axisOrder);
        setConfig(config);
    }
    // *************************************************************************
    // GuiScreenController methods

    /**
     * Initialize this (disabled) screen prior to its first update.
     *
     * @param stateManager (not null)
     * @param application (not null)
     */
    @Override
    public void initialize(
            AppStateManager stateManager, Application application) {
        super.initialize(stateManager, application);

        InputMode inputMode = InputMode.findMode("links");
        assert inputMode != null;
        setListener(inputMode);
        inputMode.influence(this);
    }

    /**
     * A callback from Nifty, invoked each time this screen starts up.
     */
    @Override
    @SuppressWarnings("unchecked")
    public void onStartScreen() {
        super.onStartScreen();

        Button nextButton = getButton("next");
        if (nextButton == null) {
            throw new RuntimeException("missing GUI control: nextButton");
        }
        this.nextElement = nextButton.getElement();

        this.treeBox = getScreen().findNiftyControl("hierarchy", TreeBox.class);
        if (treeBox == null) {
            throw new RuntimeException("missing GUI control: hierarchy");
        }

        TreeItem<LinkValue> rootItem = new TreeItem<>();
        rootItem.setExpanded(true);

        // Create an item for the torso.
        LinkValue linkItem = new LinkValue(DacConfiguration.torsoName);
        TreeItem<LinkValue> torsoItem = new TreeItem<>(linkItem);
        torsoItem.setExpanded(true);

        // Create an item for each linked bone in the hierarchy.
        Model model = DacWizard.getModel();
        int[] linkedBoneIndices = model.listLinkedBones();
        int numLinkedBones = linkedBoneIndices.length;
        TreeItem<LinkValue>[] lbItems = new TreeItem[numLinkedBones];
        for (int lbIndex = 0; lbIndex < numLinkedBones; ++lbIndex) {
            int boneIndex = linkedBoneIndices[lbIndex];
            String boneName = model.boneName(boneIndex);
            linkItem = new LinkValue(boneName);
            lbItems[lbIndex] = new TreeItem<>(linkItem);
            lbItems[lbIndex].setExpanded(true);
        }

        // Parent each item.
        for (int childLbi = 0; childLbi < numLinkedBones; ++childLbi) {
            TreeItem<LinkValue> childItem = lbItems[childLbi];
            LinkValue childValue = childItem.getValue();
            String childName = childValue.boneName();
            String parentName = model.linkedBoneParentName(childName);
            if (parentName.equals(DacConfiguration.torsoName)) {
                torsoItem.addTreeItem(childItem);
            } else { // parent is a BoneLink
                TreeItem<LinkValue> parentItem
                        = findLinkedBoneItem(parentName, lbItems);
                parentItem.addTreeItem(childItem);
            }
        }
        rootItem.addTreeItem(torsoItem);
        treeBox.setTree(rootItem);

        // Initialize the selection.
        String selected = model.selectedLink();
        if (selected.equals(DacConfiguration.torsoName)) {
            treeBox.selectItem(torsoItem);
        } else { // a BoneLink is selected
            TreeItem<LinkValue> item = findLinkedBoneItem(selected, lbItems);
            treeBox.selectItem(item);
        }
    }

    /**
     * Update this ScreenController prior to rendering. (Invoked once per
     * frame.)
     *
     * @param tpf time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void update(float tpf) {
        super.update(tpf);

        if (!hasStarted()) {
            return;
        }

        Model model = DacWizard.getModel();
        List<TreeItem<LinkValue>> selectedLinks = treeBox.getSelection();
        String boneName;
        if (selectedLinks.isEmpty()) {
            boneName = model.selectedLink();
        } else { // an item is selected
            assert selectedLinks.size() == 1 : selectedLinks.size();
            TreeItem<LinkValue> selectedItem = selectedLinks.get(0);
            LinkValue value = selectedItem.getValue();
            boneName = value.boneName();
            model.selectLink(boneName);
        }

        String xRangeStatus = "";
        String yRangeStatus = "";
        String zRangeStatus = "";
        if (boneName.equals(DacConfiguration.torsoName)) {
            setSliderEnabled("minX", false);
            setSliderEnabled("maxX", false);
            setSliderEnabled("minY", false);
            setSliderEnabled("maxY", false);
            setSliderEnabled("minZ", false);
            setSliderEnabled("maxZ", false);
        } else { // a BoneLink is selected
            RangeOfMotion rom = model.rom(boneName);
            xRangeStatus = describe(rom, PhysicsSpace.AXIS_X);
            yRangeStatus = describe(rom, PhysicsSpace.AXIS_Y);
            zRangeStatus = describe(rom, PhysicsSpace.AXIS_Z);
            setAxisSliders("minX", "maxX", rom, PhysicsSpace.AXIS_X);
            setAxisSliders("minY", "maxY", rom, PhysicsSpace.AXIS_Y);
            setAxisSliders("minZ", "maxZ", rom, PhysicsSpace.AXIS_Z);
        }
        setStatusText("xRangeStatus", xRangeStatus);
        setStatusText("yRangeStatus", yRangeStatus);
        setStatusText("zRangeStatus", zRangeStatus);

        LinkConfig config = model.config(boneName);
        String centerHeuristicButton = config.centerHeuristic().toString();
        setButtonText("centerHeuristic", centerHeuristicButton);

        MassHeuristic massHeuristic = config.massHeuristic();
        String massHeuristicButton = massHeuristic.toString().toLowerCase();
        setButtonText("massHeuristic", massHeuristicButton);

        float massParameter = config.massParameter();
        String massParameterButton = MyString.describe(massParameter);
        setButtonText("massParameter", massParameterButton);

        String shapeHeuristicButton = config.shapeHeuristic().toString();
        setButtonText("shapeHeuristic", shapeHeuristicButton);

        Vector3f shapeScale = config.shapeScale(null);
        String shapeScaleButton = MyVector3f.describe(shapeScale);
        setButtonText("shapeScale", shapeScaleButton);

        RotationOrder order = config.rotationOrder();
        String rotationOrderButtonText;
        if (order == null) {
            rotationOrderButtonText = "sixdof";
        } else {
            rotationOrderButtonText = order.toString();
        }
        setButtonText("rotationOrder", rotationOrderButtonText);

        String feedback = feedback();
        setStatusText("feedback", feedback);
        if (feedback.isEmpty()) {
            nextElement.show();
        } else {
            nextElement.hide();
        }

        AngleMode angleMode = model.angleMode();
        String angleModeText = angleMode.toString();
        setButtonText("angleMode", angleModeText);
    }
    // *************************************************************************
    // private methods

    /**
     * Read the configuration of the selected link.
     *
     * @return the configuration (not null)
     */
    private LinkConfig config() {
        Model model = DacWizard.getModel();
        String boneName = selectedLink();
        LinkConfig result = model.config(boneName);

        return result;
    }

    /**
     * Describe one axis of a joint's range of motion.
     *
     * @param rom (not null)
     * @param axisIndex axisIndex which axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     * @return descriptive text (not null, not empty)
     */
    private static String describe(RangeOfMotion rom, int axisIndex) {
        float maxRadians = rom.getMaxRotation(axisIndex);
        float minRadians = rom.getMinRotation(axisIndex);

        String text;
        AngleMode angleMode = DacWizard.getModel().angleMode();
        switch (angleMode) {
            case Degrees:
                float maxDegrees = MyMath.toDegrees(maxRadians);
                float minDegrees = MyMath.toDegrees(minRadians);
                int max = Math.round(maxDegrees);
                int min = Math.round(minDegrees);
                text = String.format("%+d to %+d degrees", min, max);
                break;

            case Radians:
                text = String.format(
                        "%+.2f to %+.2f rad", minRadians, maxRadians);
                break;

            default:
                throw new IllegalStateException("angleMode = " + angleMode);
        }

        return text;
    }

    /**
     * Find the item for the named BoneLink.
     *
     * @param boneName the bone name of the link (not null, not empty)
     * @param linkedBoneItems the array of items (not null, not empty,
     * unaffected)
     * @return the pre-existing item, or null if not found
     */
    private static TreeItem<LinkValue> findLinkedBoneItem(
            String boneName, TreeItem<LinkValue>[] linkedBoneItems) {
        assert boneName != null;
        assert !boneName.isEmpty();

        TreeItem<LinkValue> result;
        for (TreeItem<LinkValue> linkedBoneItem : linkedBoneItems) {
            result = linkedBoneItem;
            LinkValue value = result.getValue();
            if (boneName.equals(value.boneName())) {
                return result;
            }
        }

        return null;
    }

    /**
     * Update the RangeOfMotion of the selected BoneLink based on slider
     * positions.
     */
    private void readSliders() {
        float maxX = readSlider("maxX", SliderTransform.None);
        maxX = MyMath.toRadians(maxX);

        float minX = readSlider("minX", SliderTransform.None);
        minX = MyMath.toRadians(minX);

        float maxY = readSlider("maxY", SliderTransform.None);
        maxY = MyMath.toRadians(maxY);

        float minY = readSlider("minY", SliderTransform.None);
        minY = MyMath.toRadians(minY);

        float maxZ = readSlider("maxZ", SliderTransform.None);
        maxZ = MyMath.toRadians(maxZ);

        float minZ = readSlider("minZ", SliderTransform.None);
        minZ = MyMath.toRadians(minZ);

        RangeOfMotion rom
                = new RangeOfMotion(maxX, minX, maxY, minY, maxZ, minZ);

        Model model = DacWizard.getModel();
        String boneName = model.selectedLink();
        model.setRom(boneName, rom);
    }

    /**
     * Reposition and enable the named sliders to reflect the indexed axis of
     * the specified RangeOfMotion.
     *
     * @param minSliderName the name of the slider for the minimum rotation (not
     * null, not empty)
     * @param maxSliderName the name of the slider for the maximum rotation (not
     * null, not empty)
     * @param rom (not null)
     * @param axisIndex which axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     */
    private void setAxisSliders(String minSliderName, String maxSliderName,
            RangeOfMotion rom, int axisIndex) {
        float maxRadians = rom.getMaxRotation(axisIndex);
        float maxDegrees = MyMath.toDegrees(maxRadians);
        setSliderEnabled(maxSliderName, true);
        setSlider(maxSliderName, SliderTransform.None, maxDegrees);

        float minRadians = rom.getMinRotation(axisIndex);
        float minDegrees = MyMath.toDegrees(minRadians);
        setSliderEnabled(minSliderName, true);
        setSlider(minSliderName, SliderTransform.None, minDegrees);
    }

    /**
     * Alter the configuration of the selected link.
     *
     * @param config the desired configuration (not null)
     */
    private void setConfig(LinkConfig config) {
        Model model = DacWizard.getModel();
        String boneName = selectedLink();
        model.setConfig(boneName, config);
    }
}
