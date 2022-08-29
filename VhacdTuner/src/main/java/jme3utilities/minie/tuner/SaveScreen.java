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
import de.lessvoid.nifty.NiftyEventSubscriber;
import de.lessvoid.nifty.controls.RadioButton;
import de.lessvoid.nifty.controls.RadioButtonStateChangedEvent;
import de.lessvoid.nifty.screen.Screen;
import java.util.Map;
import java.util.logging.Logger;
import jme3utilities.InitialState;
import jme3utilities.MyString;
import jme3utilities.nifty.GuiScreenController;
import jme3utilities.ui.InputMode;

/**
 * The screen controller for the "save" screen of VhacdTuner.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class SaveScreen extends GuiScreenController {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final static Logger logger = Logger.getLogger(SaveScreen.class.getName());
    // *************************************************************************
    // constructors

    /**
     * Instantiate an uninitialized, disabled screen that will not be enabled
     * during initialization.
     */
    SaveScreen() {
        super("save", "Interface/Nifty/screens/tuner/save.xml",
                InitialState.Disabled);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Callback that Nifty invokes after a left-side radio button changes.
     *
     * @param buttonId Nifty element id of the radio button (not null,
     * "RadioButton" suffix)
     * @param event details of the event (not null)
     */
    @NiftyEventSubscriber(pattern = "left.*RadioButton")
    public void onLeftRadioButtonChanged(final String buttonId,
            final RadioButtonStateChangedEvent event) {
        if (!isIgnoreGuiChanges() && hasStarted()) {
            String buttonName = MyString.removeSuffix(buttonId, "RadioButton");
            String digits = MyString.remainder(buttonName, "left");
            if (digits.equals("Other")) {
                return;
            }
            int rank = Integer.parseInt(digits) - 1;

            Model model = VhacdTuner.getModel();
            DecompositionTest test = model.findRankedTest(rank);
            if (test == null) {
                test = model.getLeftTest();
                updateGroup("left", test);
            } else {
                model.setLeftTest(test);
            }
        }
    }

    /**
     * Callback that Nifty invokes after a right-side radio button changes.
     *
     * @param buttonId Nifty element id of the radio button (not null,
     * "RadioButton" suffix)
     * @param event details of the event (not null)
     */
    @NiftyEventSubscriber(pattern = "right.*RadioButton")
    public void onRightRadioButtonChanged(final String buttonId,
            final RadioButtonStateChangedEvent event) {
        if (!isIgnoreGuiChanges() && hasStarted()) {
            String buttonName = MyString.removeSuffix(buttonId, "RadioButton");
            String digits = MyString.remainder(buttonName, "right");
            if (digits.equals("Other")) {
                return;
            }
            int rank = Integer.parseInt(digits) - 1;

            Model model = VhacdTuner.getModel();
            DecompositionTest test = model.findRankedTest(rank);
            if (test == null) {
                test = model.getRightTest();
                updateGroup("right", test);
            } else {
                model.setRightTest(test);
            }
        }
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

        InputMode inputMode = InputMode.findMode("save");
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

        Model model = VhacdTuner.getModel();
        DecompositionTest bestTest = model.findRankedTest(0);
        Map<String, Object> bestParameters = bestTest.toMap();
        String bestString = bestTest.toString();
        setStatusText("best", bestString);

        StringBuilder builder = new StringBuilder(99);
        int numRanked = model.countRankedTests();
        for (int rank = 1; rank < numRanked; ++rank) {
            builder.setLength(0);
            DecompositionTest test = model.findRankedTest(rank);
            Map<String, Object> parameters = test.toMap();
            for (Map.Entry<String, Object> entry : parameters.entrySet()) {
                String key = entry.getKey();
                Object value = entry.getValue();
                Object bestValue = bestParameters.get(key);
                if (value != null && !value.equals(bestValue)) {
                    builder.append(" ").append(key).append("=").append(value);
                }
            }
            int cardinal = rank + 1;
            setStatusText("test" + cardinal, builder.toString());
        }

        DecompositionTest leftTest = model.getLeftTest();
        updateGroup("left", leftTest);

        DecompositionTest rightTest = model.getRightTest();
        updateGroup("right", rightTest);
    }
    // *************************************************************************
    // private methods

    /**
     * Update the left/right radio buttons.
     *
     * @param sideName "left" or "right"
     * @param sideTest the test parameters and results for that side (not null,
     * unaffected)
     */
    private void updateGroup(String sideName, DecompositionTest sideTest) {
        setIgnoreGuiChanges(true);
        Model model = VhacdTuner.getModel();
        Screen screen = getScreen();
        boolean found = false;

        for (int rank = 0; rank < 13; ++rank) {
            int cardinal = rank + 1;
            String id = sideName + cardinal + "RadioButton";
            RadioButton radioButton
                    = screen.findNiftyControl(id, RadioButton.class);

            DecompositionTest test = model.findRankedTest(rank);
            if (test == null) {
                radioButton.disable();
            } else {
                radioButton.enable();
                if (test == sideTest) {
                    radioButton.select();
                    found = true;
                }
            }
        }

        String id = sideName + "OtherRadioButton";
        RadioButton radioButton
                = screen.findNiftyControl(id, RadioButton.class);
        if (!found) {
            radioButton.select();
        }
        setIgnoreGuiChanges(false);
    }
}
