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
import de.lessvoid.nifty.controls.Button;
import de.lessvoid.nifty.controls.TreeBox;
import de.lessvoid.nifty.controls.TreeItem;
import de.lessvoid.nifty.elements.Element;
import java.util.List;
import java.util.logging.Logger;
import jme3utilities.InitialState;
import jme3utilities.nifty.GuiScreenController;
import jme3utilities.ui.InputMode;

/**
 * The screen controller for the "torso" screen of DacWizard.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class TorsoScreen extends GuiScreenController {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final static Logger logger = Logger.getLogger(TorsoScreen.class.getName());
    // *************************************************************************
    // fields

    /**
     * element of the GUI button to proceed to the "links" screen
     */
    private Element linksElement;
    /**
     * TreeBox to display bones managed by the torso
     */
    private TreeBox<BoneValue> treeBox;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an uninitialized, disabled screen that will not be enabled
     * during initialization.
     */
    TorsoScreen() {
        super("torso", "Interface/Nifty/screens/wizard/torso.xml",
                InitialState.Disabled);
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

        InputMode inputMode = InputMode.findMode("torso");
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

        Button linksButton = getButton("links");
        if (linksButton == null) {
            throw new RuntimeException("missing GUI control: linksButton");
        }
        this.linksElement = linksButton.getElement();

        this.treeBox = getScreen().findNiftyControl("skeleton", TreeBox.class);
        if (treeBox == null) {
            throw new RuntimeException("missing GUI control: skeleton");
        }

        TreeItem<BoneValue> rootItem = new TreeItem<>();
        rootItem.setExpanded(true);
        Model model = DacWizard.getModel();
        int[] managedBones = model.listTorsoManagedBones();
        int numManaged = managedBones.length;

        // Create an item for each bone managed by the torso.
        TreeItem<BoneValue>[] boneItems = new TreeItem[numManaged];
        for (int managedIndex = 0; managedIndex < numManaged; ++managedIndex) {
            int boneIndex = managedBones[managedIndex];
            BoneValue value = new BoneValue(boneIndex);
            boneItems[managedIndex] = new TreeItem<>(value);
            boneItems[managedIndex].setExpanded(true);
        }

        // Generate a map from bone indices to managed-bone indices.
        int numBones = model.countBones();
        int[] mbiArray = new int[numBones];
        for (int boneIndex = 0; boneIndex < numBones; ++boneIndex) {
            mbiArray[boneIndex] = -1;
        }
        for (int managedIndex = 0; managedIndex < numManaged; ++managedIndex) {
            int boneIndex = managedBones[managedIndex];
            mbiArray[boneIndex] = managedIndex;
        }

        // Parent each item.
        for (int managedIndex = 0; managedIndex < numManaged; ++managedIndex) {
            TreeItem<BoneValue> childItem = boneItems[managedIndex];
            int boneIndex = managedBones[managedIndex];
            int parentIndex = model.parentIndex(boneIndex);
            if (parentIndex == -1) {
                rootItem.addTreeItem(childItem);
            } else {
                int parentMbi = mbiArray[parentIndex];
                boneItems[parentMbi].addTreeItem(childItem);
            }
        }
        treeBox.setTree(rootItem);

        // Pre-select the item for the main bone.
        int mainBoneIndex = model.mainBoneIndex();
        selectItem(mainBoneIndex);
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
        List<TreeItem<BoneValue>> selectedItems = treeBox.getSelection();
        int numSelected = selectedItems.size();
        if (numSelected == 1) { // Update the main-bone index in the model.
            TreeItem<BoneValue> mbItem = selectedItems.get(0);
            BoneValue mbValue = mbItem.getValue();
            int mbIndex = mbValue.boneIndex();
            model.setMainBoneIndex(mbIndex);

        } else { // empty selection:  re-select the main bone
            int mainBoneIndex = model.mainBoneIndex();
            selectItem(mainBoneIndex);
        }
        /*
         * If there's a ragdoll already configured, allow the user to
         * bypass range-of-motion estimation.
         */
        boolean hasRagdoll = model.hasConfiguredRagdoll();
        if (hasRagdoll) {
            linksElement.show();
        } else {
            linksElement.hide();
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Select the tree item corresponding to the indexed bone.
     *
     * @param boneIndex the index of the bone to select (&ge;0)
     */
    private void selectItem(int boneIndex) {
        assert boneIndex >= 0 : boneIndex;

        List<TreeItem<BoneValue>> itemList = treeBox.getItems();
        for (TreeItem<BoneValue> item : itemList) {
            BoneValue value = item.getValue();
            if (value.boneIndex() == boneIndex) {
                treeBox.selectItem(item);
                break;
            }
        }
    }
}
