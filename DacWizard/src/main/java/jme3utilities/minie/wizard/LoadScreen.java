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

import com.jme3.animation.Skeleton;
import com.jme3.app.Application;
import com.jme3.app.state.AppStateManager;
import com.jme3.scene.Spatial;
import de.lessvoid.nifty.controls.Button;
import de.lessvoid.nifty.elements.Element;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.InitialState;
import jme3utilities.MyString;
import jme3utilities.debug.SkeletonVisualizer;
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
     * element of the GUI button to proceed to the "bones" screen
     */
    private Element nextElement;
    /**
     * animation time of the pose being viewed (in seconds)
     */
    private float viewedAnimationTime;
    /**
     * root spatial of the C-G model being viewed, or null for none
     */
    private Spatial viewedSpatial;
    /**
     * clip/animation name of the pose being viewed
     */
    private String viewedAnimationName;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an uninitialized, disabled screen that will not be enabled
     * during initialization.
     */
    LoadScreen() {
        super("load", "Interface/Nifty/screens/wizard/load.xml",
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
        int numDacs = model.countDacs();
        int numSkeletonControls = model.countSControls();
        Spatial nextSpatial = model.getRootSpatial();
        String loadException = model.loadExceptionString();

        String result;
        if (!loadException.isEmpty()) {
            result = loadException;
        } else if (nextSpatial == null) {
            result = "The model hasn't been loaded yet.";
        } else if (numSkeletonControls == 0) {
            result = "The model lacks a skinning/skeleton control.";
        } else if (numSkeletonControls > 1) {
            result = String.format(
                    "The model has %d skinning/skeleton controls.",
                    numSkeletonControls);

        } else if (model.countBones() < 1) {
            if (model.findSkeleton() == null) { // new animation system
                result = "The model's Armature lacks joints.";
            } else { // old animation system
                result = "The model's Skeleton lacks bones.";
            }

        } else if (numDacs > 1) {
            result = String.format("The model has %d DACs.", numDacs);
        } else {
            result = model.validationFeedback();
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
        this.nextElement = nextButton.getElement();

        DacWizard wizard = DacWizard.getApplication();
        wizard.clearScene();
        this.viewedSpatial = null;
        this.viewedAnimationName = null;
        this.viewedAnimationTime = Float.NaN;

        Model model = DacWizard.getModel();
        model.setShowingMeshes(true);
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

        updateFeedback();
        updatePath();
        updateToggleButton();

        // Update the 3-D scene.
        Model model = DacWizard.getModel();
        Spatial nextSpatial = model.getRootSpatial();
        String nextAnimationName = model.animationName();
        float nextAnimationTime = model.animationTime();
        if (nextSpatial != viewedSpatial
                || !nextAnimationName.equals(viewedAnimationName)
                || nextAnimationTime != viewedAnimationTime) {
            DacWizard wizard = DacWizard.getApplication();
            wizard.clearScene();

            this.viewedSpatial = nextSpatial;
            this.viewedAnimationName = nextAnimationName;
            this.viewedAnimationTime = nextAnimationTime;

            if (nextSpatial != null) {
                Spatial cgModel = Heart.deepCopy(nextSpatial);
                wizard.makeScene(cgModel, nextAnimationName, nextAnimationTime);
            }
        }

        updatePosingControls();
    }
    // *************************************************************************
    // private methods

    /**
     * Update the feedback line and the load/next buttons.
     */
    private void updateFeedback() {
        Model model = DacWizard.getModel();
        Spatial nextSpatial = model.getRootSpatial();
        String loadException = model.loadExceptionString();

        String loadButton = "";
        if (loadException.isEmpty() && nextSpatial == null) {
            loadButton = "Load and preview";
        }
        setButtonText("load", loadButton);

        String feedback = feedback();
        setStatusText("feedback", feedback);
        if (feedback.isEmpty()) {
            nextElement.show();
        } else {
            nextElement.hide();
        }
    }

    /**
     * Update the posing controls.
     */
    private void updatePosingControls() {
        String anText = "";
        String atText = "";
        String naText = "";
        String paText = "";

        if (viewedSpatial != null) {
            Model model = DacWizard.getModel();
            int numAnimations = model.countAnimations();
            if (numAnimations > 0) {
                paText = "-";
                naText = "+";
            }

            float duration = model.animationDuration();
            if (duration > 0f) {
                atText = Float.toString(viewedAnimationTime) + " seconds";
            }

            anText = viewedAnimationName;
            if (!anText.equals(DacWizard.bindPoseName)) {
                anText = MyString.quote(anText);
            }
        }

        setStatusText("animationName", anText);
        setButtonText("animationTime", atText);
        setButtonText("nextAnimation", naText);
        setButtonText("previousAnimation", paText);
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

    /**
     * Update the button to toggle skeleton visualization.
     */
    private void updateToggleButton() {
        String buttonText = "";

        DacWizard app = DacWizard.getApplication();
        SkeletonVisualizer sv = app.findSkeletonVisualizer();
        Model model = DacWizard.getModel();
        Spatial root = model.getRootSpatial();
        if (sv != null && root != null) {
            boolean isShown = model.isShowingSkeleton();
            sv.setEnabled(isShown);

            Skeleton skeleton = model.findSkeleton();
            String armature = (skeleton == null) ? "armature" : "skeleton";
            if (isShown) {
                buttonText = "Hide " + armature;
            } else {
                buttonText = "Show " + armature;
            }
        }

        setButtonText("skeleton", buttonText);
    }
}
