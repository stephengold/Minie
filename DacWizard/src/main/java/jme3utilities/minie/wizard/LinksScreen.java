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
import com.jme3.app.state.AppStateManager;
import com.jme3.bullet.animation.CenterHeuristic;
import com.jme3.bullet.animation.DacConfiguration;
import com.jme3.bullet.animation.LinkConfig;
import com.jme3.bullet.animation.MassHeuristic;
import com.jme3.bullet.animation.ShapeHeuristic;
import com.jme3.math.Vector3f;
import de.lessvoid.nifty.controls.Button;
import de.lessvoid.nifty.controls.TreeBox;
import de.lessvoid.nifty.controls.TreeItem;
import de.lessvoid.nifty.elements.Element;
import java.util.List;
import java.util.logging.Logger;
import jme3utilities.MyString;
import jme3utilities.math.MyVector3f;
import jme3utilities.nifty.GuiScreenController;
import jme3utilities.nifty.PopupMenuBuilder;
import jme3utilities.nifty.dialog.DialogController;
import jme3utilities.nifty.dialog.FloatDialog;
import jme3utilities.nifty.dialog.VectorDialog;
import jme3utilities.ui.InputMode;

/**
 * The screen controller for the "links" screen of DacWizard.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class LinksScreen extends GuiScreenController {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final static Logger logger = Logger.getLogger(LinksScreen.class.getName());
    // *************************************************************************
    // fields

    /**
     * element of GUI button to proceed to the next Screen
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
        super("links", "Interface/Nifty/screens/wizard/links.xml", false);
    }
    // *************************************************************************
    // new methods exposed

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

        config = new LinkConfig(massParameter, massHeuristic, shapeHeuristic,
                shapeScale, centerHeuristic);
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
     */
    void selectCenterHeuristic(CenterHeuristic heuristic) {
        LinkConfig config = config();
        float massParameter = config.massParameter();
        MassHeuristic massHeuristic = config.massHeuristic();
        ShapeHeuristic shapeHeuristic = config.shapeHeuristic();
        Vector3f shapeScale = config.shapeScale(null);

        config = new LinkConfig(massParameter, massHeuristic, shapeHeuristic,
                shapeScale, heuristic);
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
     */
    void selectShapeHeuristic(ShapeHeuristic heuristic) {
        LinkConfig config = config();
        float massParameter = config.massParameter();
        MassHeuristic massHeuristic = config.massHeuristic();
        Vector3f shapeScale = config.shapeScale(null);
        CenterHeuristic centerHeuristic = config.centerHeuristic();

        config = new LinkConfig(massParameter, massHeuristic, heuristic,
                shapeScale, centerHeuristic);
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
        boolean allowNull = false;
        DialogController controller = new FloatDialog("Set", Float.MIN_VALUE,
                Float.MAX_VALUE, allowNull);
        showTextEntryDialog("Enter the mass-parameter value:", defaultText,
                actionPrefix, controller);
    }

    /**
     * Handle a "set massParameter" action with an argument.
     */
    void setMassParameter(float value) {
        LinkConfig config = config();
        MassHeuristic massHeuristic = config.massHeuristic();
        ShapeHeuristic shapeHeuristic = config.shapeHeuristic();
        Vector3f shapeScale = config.shapeScale(null);
        CenterHeuristic centerHeuristic = config.centerHeuristic();

        config = new LinkConfig(value, massHeuristic, shapeHeuristic,
                shapeScale, centerHeuristic);
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
        boolean allowNull = false;
        DialogController controller = new VectorDialog("Set", 3, allowNull);
        showTextEntryDialog("Enter the shapeScale value:", defaultText,
                actionPrefix, controller);
    }

    /**
     * Handle a "set shapeScale" action with an argument.
     */
    void setShapeScale(Vector3f scaleFactors) {
        LinkConfig config = config();
        MassHeuristic massHeuristic = config.massHeuristic();
        float massParameter = config.massParameter();
        ShapeHeuristic shapeHeuristic = config.shapeHeuristic();
        CenterHeuristic centerHeuristic = config.centerHeuristic();

        config = new LinkConfig(massParameter, massHeuristic, shapeHeuristic,
                scaleFactors, centerHeuristic);
        setConfig(config);
    }
    // *************************************************************************
    // GuiScreenController methods

    /**
     * Initialize this (disabled) mode prior to its 1st update.
     *
     * @param stateManager (not null)
     * @param application (not null)
     */
    @Override
    public void initialize(AppStateManager stateManager,
            Application application) {
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
        nextElement = nextButton.getElement();

        treeBox = getScreen().findNiftyControl("hierarchy", TreeBox.class);
        if (treeBox == null) {
            throw new RuntimeException("missing GUI control: hierarchy");
        }

        TreeItem<LinkValue> rootItem = new TreeItem<>();
        rootItem.setExpanded(true);
        Model model = DacWizard.getModel();
        model.makeRagdoll();
        int[] linkedBoneIndices = model.listLinkedBones();
        int numLinkedBones = linkedBoneIndices.length;
        /*
         * Create an item for the torso.
         */
        LinkValue linkItem = new LinkValue(DacConfiguration.torsoName);
        TreeItem<LinkValue> torsoItem = new TreeItem<>(linkItem);
        torsoItem.setExpanded(true);
        /*
         * Create an item for each linked bone in the hierarchy.
         */
        TreeItem<LinkValue>[] lbItems = new TreeItem[numLinkedBones];
        for (int lbIndex = 0; lbIndex < numLinkedBones; ++lbIndex) {
            int boneIndex = linkedBoneIndices[lbIndex];
            String boneName = model.boneName(boneIndex);
            linkItem = new LinkValue(boneName);
            lbItems[lbIndex] = new TreeItem<>(linkItem);
            lbItems[lbIndex].setExpanded(true);
        }
        /*
         * Parent each item.
         */
        for (int childLbi = 0; childLbi < numLinkedBones; ++childLbi) {
            TreeItem<LinkValue> childItem = lbItems[childLbi];
            LinkValue childValue = childItem.getValue();
            String childName = childValue.boneName();
            String parentName = model.linkedBoneParentName(childName);
            if (parentName.equals(DacConfiguration.torsoName)) {
                torsoItem.addTreeItem(childItem);
            } else {
                TreeItem<LinkValue> parentItem = null;
                for (int parentLbi = 0; parentLbi < numLinkedBones; ++parentLbi) {
                    parentItem = lbItems[parentLbi];
                    LinkValue parentValue = parentItem.getValue();
                    if (parentName.equals(parentValue.boneName())) {
                        break;
                    }
                }
                parentItem.addTreeItem(childItem);
            }
        }
        rootItem.addTreeItem(torsoItem);
        treeBox.setTree(rootItem);
        /*
         * Pre-select the torso.
         */
        treeBox.selectItem(torsoItem);
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

        List<TreeItem<LinkValue>> selectedLinks = treeBox.getSelection();
        boolean isSelected = !selectedLinks.isEmpty();

        String centerHeuristicButton = "";
        String massHeuristicButton = "";
        String massParameterButton = "";
        String shapeHeuristicButton = "";
        String shapeScaleButton = "";

        if (isSelected) {
            assert selectedLinks.size() == 1;
            TreeItem<LinkValue> selectedItem = selectedLinks.get(0);
            LinkValue value = selectedItem.getValue();
            String boneName = value.boneName();

            Model model = DacWizard.getModel();
            LinkConfig config = model.config(boneName);
            centerHeuristicButton = config.centerHeuristic().toString();
            massHeuristicButton = config.massHeuristic().toString();
            float massParameter = config.massParameter();
            massParameterButton = MyString.describe(massParameter);
            shapeHeuristicButton = config.shapeHeuristic().toString();
            Vector3f shapeScale = config.shapeScale(null);
            shapeScaleButton = MyVector3f.describe(shapeScale);
        }

        setButtonText("centerHeuristic", centerHeuristicButton);
        setButtonText("massHeuristic", massHeuristicButton);
        setButtonText("massParameter", massParameterButton);
        setButtonText("shapeHeuristic", shapeHeuristicButton);
        setButtonText("shapeScale", shapeScaleButton);

        String feedback = "";
        setStatusText("feedback", feedback);
        if (feedback.isEmpty()) {
            nextElement.show();
        } else {
            nextElement.hide();
        }
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
