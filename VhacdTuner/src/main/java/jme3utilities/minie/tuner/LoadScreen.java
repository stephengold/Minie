/*
 Copyright (c) 2019-2022, Stephen Gold
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
import com.jme3.app.state.AppStateManager;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Spatial;
import de.lessvoid.nifty.controls.Button;
import de.lessvoid.nifty.elements.Element;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.InitialState;
import jme3utilities.MyCamera;
import jme3utilities.nifty.GuiScreenController;
import jme3utilities.ui.InputMode;

/**
 * The screen controller for the "load" screen of VhacdTuner.
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
        super("load", "Interface/Nifty/screens/tuner/load.xml",
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
        Model model = VhacdTuner.getModel();
        Spatial nextSpatial = model.getRootSpatial();
        String loadException = model.loadExceptionString();

        String result;
        if (!loadException.isEmpty()) {
            result = loadException;
        } else if (nextSpatial == null) {
            result = "The model hasn't been loaded yet.";
        } else {
            result = "";
        }

        return result;
    }

    /**
     * Toggle the visibility of the world axes.
     */
    void toggleShowingAxes() {
        Model model = VhacdTuner.getModel();
        model.toggleAxes();
        VhacdTuner.updateAxes(rootNode);
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

        cam.setName("default camera");
        cam.setLocation(new Vector3f(0f, 0f, 10f));
        MyCamera.setNearFar(cam, 0.1f, 1000f);
        cam.setRotation(new Quaternion(0f, 1f, 0f, 0f));

        flyCam.setDragToRotate(true);
        viewPort.setEnabled(true);

        Button nextButton = getButton("next");
        if (nextButton == null) {
            throw new RuntimeException("missing GUI control: nextButton");
        }
        this.nextElement = nextButton.getElement();

        VhacdTuner tuner = VhacdTuner.getApplication();
        tuner.clearScene();
        this.viewedSpatial = null;

        VhacdTuner.updateAxes(rootNode);
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
        updateViewButtons();

        Model model = VhacdTuner.getModel();
        Spatial nextSpatial = model.getRootSpatial();
        if (nextSpatial != this.viewedSpatial) {
            VhacdTuner tuner = VhacdTuner.getApplication();
            tuner.clearScene();
            this.viewedSpatial = nextSpatial;
            if (nextSpatial != null) {
                Spatial cgModel = Heart.deepCopy(nextSpatial);
                tuner.makeScene(cgModel);
            }
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Update the feedback line and the load/next buttons.
     */
    private void updateFeedback() {
        Model model = VhacdTuner.getModel();
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
     * Update the path status and "+" buttons.
     */
    private void updatePath() {
        Model model = VhacdTuner.getModel();

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
     * Update the buttons that toggle view elements.
     */
    private void updateViewButtons() {
        Model model = VhacdTuner.getModel();
        String axesText = model.isShowingAxes() ? "Hide axes" : "Show axes";
        setButtonText("axes", axesText);
    }
}
