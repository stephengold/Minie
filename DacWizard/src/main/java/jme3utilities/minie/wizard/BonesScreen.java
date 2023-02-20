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
import com.jme3.bullet.animation.DacConfiguration;
import de.lessvoid.nifty.controls.Button;
import de.lessvoid.nifty.controls.TreeBox;
import de.lessvoid.nifty.controls.TreeItem;
import de.lessvoid.nifty.elements.Element;
import java.util.BitSet;
import java.util.List;
import java.util.logging.Logger;
import jme3utilities.InitialState;
import jme3utilities.MyString;
import jme3utilities.nifty.GuiScreenController;
import jme3utilities.ui.InputMode;

/**
 * The screen controller for the "bones" screen of DacWizard.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class BonesScreen extends GuiScreenController {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final static Logger logger = Logger.getLogger(BonesScreen.class.getName());
    // *************************************************************************
    // fields

    /**
     * element of the GUI button to proceed to the "torso" screen
     */
    private Element nextElement;
    /**
     * TreeBox to display all bones in the skeleton
     */
    private TreeBox<BoneValue> treeBox;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an uninitialized, disabled screen that will not be enabled
     * during initialization.
     */
    BonesScreen() {
        super("bones", "Interface/Nifty/screens/wizard/bones.xml",
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
        Model model = DacWizard.getModel();
        int numBones = model.countBones();

        String result = "";
        if (model.listLinkedBones().length == 0) {
            result = "No bones are linked.";
        } else if (model.countVertices(DacConfiguration.torsoName) == 0) {
            result = "No mesh vertices for the torso.";
        } else {
            for (int i = 0; i < numBones; ++i) {
                if (model.isBoneLinked(i)) {
                    String name = model.boneName(i);
                    if (model.countVertices(name) == 0) {
                        result = "No mesh vertices for " + MyString.quote(name);
                    }
                }
            }
        }

        return result;
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

        InputMode inputMode = InputMode.findMode("bones");
        assert inputMode != null;
        setListener(inputMode);
        inputMode.influence(this);
    }

    /**
     * A callback from Nifty, invoked each time the screen shuts down.
     */
    @Override
    public void onEndScreen() {
        treeBox.clear();
        super.onEndScreen();
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

        this.treeBox = getScreen().findNiftyControl("skeleton", TreeBox.class);
        if (treeBox == null) {
            throw new RuntimeException("missing GUI control: skeleton");
        }

        TreeItem<BoneValue> rootItem = new TreeItem<>();
        rootItem.setExpanded(true);
        Model model = DacWizard.getModel();
        int numBones = model.countBones();

        // Create an item for each bone in the model's skeleton.
        TreeItem<BoneValue>[] boneItems = new TreeItem[numBones];
        for (int boneIndex = 0; boneIndex < numBones; ++boneIndex) {
            BoneValue value = new BoneValue(boneIndex);
            boneItems[boneIndex] = new TreeItem<>(value);
            boneItems[boneIndex].setExpanded(true);
        }

        // Parent each item.
        for (int childIndex = 0; childIndex < numBones; ++childIndex) {
            TreeItem<BoneValue> childItem = boneItems[childIndex];
            int parentIndex = model.parentIndex(childIndex);
            if (parentIndex == -1) {
                rootItem.addTreeItem(childItem);
            } else {
                boneItems[parentIndex].addTreeItem(childItem);
            }
        }
        treeBox.setTree(rootItem);

        // Pre-select items.
        for (int boneIndex = 0; boneIndex < numBones; ++boneIndex) {
            TreeItem<BoneValue> item = boneItems[boneIndex];
            if (model.isBoneLinked(boneIndex)) {
                treeBox.selectItem(item);
            }
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

        List<TreeItem<BoneValue>> selectedBones = treeBox.getSelection();
        int numSelected = selectedBones.size();

        Model model = DacWizard.getModel();
        int numBones = model.countBones();

        String numText = String.format("Selected %d of %d bone%s",
                numSelected, numBones, numBones == 1 ? "" : "s");
        setStatusText("numSelected", numText);

        BitSet linkedBones = new BitSet(numBones);
        for (TreeItem<BoneValue> treeItem : selectedBones) {
            BoneValue value = treeItem.getValue();
            int boneIndex = value.boneIndex();
            linkedBones.set(boneIndex);
        }
        model.setLinkedBones(linkedBones);

        String feedback = feedback();
        setStatusText("feedback", feedback);
        if (feedback.isEmpty()) {
            nextElement.show();
        } else {
            nextElement.hide();
        }
    }
}
