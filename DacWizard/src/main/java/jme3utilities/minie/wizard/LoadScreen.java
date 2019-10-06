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
import com.jme3.scene.Spatial;
import de.lessvoid.nifty.controls.Button;
import de.lessvoid.nifty.elements.Element;
import java.util.logging.Logger;
import jme3utilities.Misc;
import jme3utilities.nifty.GuiScreenController;
import jme3utilities.ui.InputMode;

/**
 * The screen controller for the "load" screen of DacWizard.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class LoadScreen extends GuiScreenController {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final static Logger logger = Logger.getLogger(LoadScreen.class.getName());
    // *************************************************************************
    // fields

    /**
     * element of GUI button to proceed to the next Screen
     */
    private Element nextElement;
    /**
     * root spatial of the C-G model being previewed
     */
    private Spatial viewedSpatial;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an uninitialized, disabled screen that will not be enabled
     * during initialization.
     */
    LoadScreen() {
        super("load", "Interface/Nifty/screens/wizard/load.xml", false);
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
    public void initialize(AppStateManager stateManager,
            Application application) {
        super.initialize(stateManager, application);

        InputMode inputMode = InputMode.findMode("load");
        assert inputMode != null;
        setListener(inputMode);
        inputMode.influence(this);
    }

    /**
     * A callback from Nifty, invoked each time this screen starts up.
     */
    @Override
    public void onStartScreen() {
        super.onStartScreen();

        Button nextButton = getButton("next");
        if (nextButton == null) {
            throw new RuntimeException("missing GUI control: nextButton");
        }
        nextElement = nextButton.getElement();

        DacWizard wizard = DacWizard.getApplication();
        wizard.clearScene();
        viewedSpatial = null;
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

        updateFeedback();
        updatePath();

        Model model = DacWizard.getModel();
        Spatial nextSpatial = model.getRootSpatial();
        if (nextSpatial != viewedSpatial) {
            DacWizard wizard = DacWizard.getApplication();
            wizard.clearScene();
            viewedSpatial = nextSpatial;
            if (nextSpatial != null) {
                Spatial cgModel = (Spatial) Misc.deepCopy(nextSpatial);
                wizard.makeScene(cgModel);
            }
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Update the feedback line and the load/next buttons.
     */
    private void updateFeedback() {
        Model model = DacWizard.getModel();
        int numSkeletonControls = model.countSkeletonControls();
        Spatial nextSpatial = model.getRootSpatial();
        String loadException = model.loadExceptionString();

        String feedback = "";
        String loadButton = "";
        if (!loadException.isEmpty()) {
            feedback = loadException;
        } else if (nextSpatial == null) {
            feedback = "The model hasn't been loaded yet.";
            loadButton = "Load and preview";
        } else if (numSkeletonControls == 0) {
            feedback = "The model lacks a skeleton control.";
        } else if (numSkeletonControls > 1) {
            feedback = String.format("The model has %d skeleton controls.",
                    numSkeletonControls);
        } else if (model.countBones() < 1) {
            feedback = "The model's skeleton lacks bones.";
        }

        setStatusText("feedback", feedback);
        setButtonText("load", loadButton);
        if (feedback.isEmpty()) {
            nextElement.show();
        } else {
            nextElement.hide();
        }
    }

    /**
     * Update the path status and "+" buttons.
     */
    private void updatePath() {
        Model model = DacWizard.getModel();

        String assetPath = model.assetPath();
        String assetRoot = model.assetRoot();
        String[] pathComponents = assetPath.split("/");
        String[] rootComponents = assetRoot.split("/");

        String morePathButton = "";
        String moreRootButton = "";
        if (pathComponents.length > 2) {
            moreRootButton = "+";
        }
        if (rootComponents.length > 1) {
            morePathButton = "+";
        }

        setButtonText("morePath", morePathButton);
        setButtonText("moreRoot", moreRootButton);

        setStatusText("assetPath", " " + assetPath);
        setStatusText("assetRoot", " " + assetRoot);
    }
}
