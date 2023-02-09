/*
 Copyright (c) 2020-2023, Stephen Gold
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
package jme3utilities.minie.test;

import com.jme3.app.Application;
import com.jme3.app.state.AppStateManager;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.font.BitmapFont;
import com.jme3.font.BitmapText;
import com.jme3.math.ColorRGBA;
import java.util.Arrays;
import java.util.logging.Logger;
import jme3utilities.SimpleAppState;
import jme3utilities.math.MyArray;
import jme3utilities.math.MyMath;
import jme3utilities.ui.AcorusDemo;

/**
 * AppState to display the status of the JointElasticity application in an
 * overlay. The overlay consists of status lines, one of which is selected for
 * editing. The overlay is located in the upper-left portion of the display.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final class JointElasticityStatus extends SimpleAppState {
    // *************************************************************************
    // constants and loggers

    /**
     * list of error-reduction parameter values, in ascending order
     */
    final private static float[] erpValues
            = {0.01f, 0.1f, 0.2f, 0.5f, 0.8f, 0.9f, 1f};
    /**
     * list of mass ratios, in ascending order
     */
    final private static float[] ratioValues = {1f, 10f, 100f, 1000f};
    /**
     * list of physics timesteps, in ascending order
     */
    final private static float[] timestepValues
            = {0.002f, 0.003f, 0.005f, 0.01f, 1f / 60, 0.04f};
    /**
     * list of constraint-solver iteration counts, in ascending order
     */
    final private static int[] iterationsValues
            = {10, 250, 500, 1000, 2000, 4000};
    /**
     * index of the status line for the joint ERP value
     */
    final private static int erpStatusLine = 0;
    /**
     * index of the status line for the constraint-solver iteration count
     */
    final private static int iterationsStatusLine = 1;
    /**
     * index of the status line for the mass ratio
     */
    final private static int ratioStatusLine = 2;
    /**
     * index of the status line for physics timestep
     */
    final private static int timestepStatusLine = 3;
    /**
     * number of lines of text in the overlay
     */
    final private static int numStatusLines = 4;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(JointElasticityStatus.class.getName());
    // *************************************************************************
    // fields

    /**
     * lines of text displayed in the upper-left corner of the GUI node ([0] is
     * the top line)
     */
    final private BitmapText[] statusLines = new BitmapText[numStatusLines];
    /**
     * error-reduction parameter value for joints
     */
    private float jointErp = 0.2f;
    /**
     * ball's mass as a multiple of the door
     */
    private float massRatio = 1f;
    /**
     * physics timestep (in seconds)
     */
    private float timestep = 1f / 60;
    /**
     * maximum number of solver iterations per physics timestep
     */
    private int numIterations = 10;
    /**
     * index of the line being edited (&ge;1)
     */
    private int selectedLine = timestepStatusLine;
    /**
     * reference to the application instance
     */
    private JointElasticity appInstance;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an uninitialized enabled state.
     */
    JointElasticityStatus() {
        super(true);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Advance the field selection by the specified amount.
     *
     * @param amount the number of fields to move downward
     */
    void advanceSelectedField(int amount) {
        int firstField = 0;
        int numFields = numStatusLines - firstField;

        int selectedField = selectedLine - firstField;
        int sum = selectedField + amount;
        selectedField = MyMath.modulo(sum, numFields);
        this.selectedLine = selectedField + firstField;
    }

    /**
     * Advance the value of the selected field by the specified amount.
     *
     * @param amount the number of values to advance (may be negative)
     */
    void advanceValue(int amount) {
        switch (selectedLine) {
            case erpStatusLine:
                advanceJointErp(amount);
                break;

            case iterationsStatusLine:
                advanceIterations(amount);
                break;

            case ratioStatusLine:
                advanceRatio(amount);
                break;

            case timestepStatusLine:
                advanceTimestep(amount);
                break;

            default:
                throw new IllegalStateException("line = " + selectedLine);
        }
    }

    float jointErp() {
        assert jointErp >= 0f : jointErp;
        assert jointErp <= 1f : jointErp;
        return jointErp;
    }

    float massRatio() {
        assert massRatio > 0f : massRatio;
        return massRatio;
    }

    int numIterations() {
        assert numIterations > 0 : numIterations;
        return numIterations;
    }

    /**
     * Update the GUI layout and proposed settings after a resize.
     *
     * @param newWidth the new width of the framebuffer (in pixels, &gt;0)
     * @param newHeight the new height of the framebuffer (in pixels, &gt;0)
     */
    void resize(int newWidth, int newHeight) {
        if (isInitialized()) {
            for (int lineIndex = 0; lineIndex < numStatusLines; ++lineIndex) {
                float y = newHeight - 20f * lineIndex;
                statusLines[lineIndex].setLocalTranslation(0f, y, 0f);
            }
        }
    }

    float timeStep() {
        assert timestep > 0f : timestep;
        return timestep;
    }
    // *************************************************************************
    // ActionAppState methods

    /**
     * Clean up this AppState during the first update after it gets detached.
     * Should be invoked only by a subclass or by the AppStateManager.
     */
    @Override
    public void cleanup() {
        super.cleanup();

        // Remove the status lines from the guiNode.
        for (int i = 0; i < numStatusLines; ++i) {
            statusLines[i].removeFromParent();
        }
    }

    /**
     * Initialize this AppState on the first update after it gets attached.
     *
     * @param sm application's state manager (not null)
     * @param app application which owns this state (not null)
     */
    @Override
    public void initialize(AppStateManager sm, Application app) {
        super.initialize(sm, app);

        this.appInstance = (JointElasticity) app;
        BitmapFont guiFont
                = assetManager.loadFont("Interface/Fonts/Default.fnt");

        // Add status lines to the guiNode.
        for (int lineIndex = 0; lineIndex < numStatusLines; ++lineIndex) {
            statusLines[lineIndex] = new BitmapText(guiFont);
            float y = cam.getHeight() - 20f * lineIndex;
            statusLines[lineIndex].setLocalTranslation(0f, y, 0f);
            guiNode.attachChild(statusLines[lineIndex]);
        }

        assert MyArray.isSorted(iterationsValues);
        assert MyArray.isSorted(ratioValues);
        assert MyArray.isSorted(timestepValues);
    }

    /**
     * Callback to update this AppState prior to rendering. (Invoked once per
     * frame while the state is attached and enabled.)
     *
     * @param tpf time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void update(float tpf) {
        super.update(tpf);

        int index = 1 + Arrays.binarySearch(erpValues, jointErp);
        String message = String.format("Joint ERP (#%d of %d):   %.2f",
                index, erpValues.length, jointErp);
        updateStatusLine(erpStatusLine, message);

        index = 1 + Arrays.binarySearch(iterationsValues, numIterations);
        message = String.format("Max solver iterations (#%d of %d):   %d",
                index, iterationsValues.length, numIterations);
        updateStatusLine(iterationsStatusLine, message);

        index = 1 + Arrays.binarySearch(ratioValues, massRatio);
        message = String.format("Mass ratio (#%d of %d):   %.0f : 1",
                index, ratioValues.length, massRatio);
        updateStatusLine(ratioStatusLine, message);

        index = 1 + Arrays.binarySearch(timestepValues, timestep);
        message = String.format("Timestep (#%d of %d):   %.4f second",
                index, timestepValues.length, timestep);
        updateStatusLine(timestepStatusLine, message);
    }
    // *************************************************************************
    // private methods

    /**
     * Advance the iteration-count selection by the specified amount.
     *
     * @param amount the number of values to advance (may be negative)
     */
    private void advanceIterations(int amount) {
        numIterations = AcorusDemo.advanceInt(
                iterationsValues, numIterations, amount);
        PhysicsSpace physicsSpace = appInstance.getPhysicsSpace();
        physicsSpace.getSolverInfo().setNumIterations(numIterations);
    }

    /**
     * Advance the joint ERP value by the specified amount.
     *
     * @param amount the number of values to advance (may be negative)
     */
    private void advanceJointErp(int amount) {
        jointErp = AcorusDemo.advanceFloat(erpValues, jointErp, amount);
        PhysicsSpace physicsSpace = appInstance.getPhysicsSpace();
        physicsSpace.getSolverInfo().setJointErp(jointErp);
    }

    /**
     * Advance the mass ratio by the specified amount.
     *
     * @param amount the number of values to advance (may be negative)
     */
    private void advanceRatio(int amount) {
        this.massRatio = AcorusDemo
                .advanceFloat(ratioValues, massRatio, amount);
        JointElasticity.setMassRatio(massRatio);
    }

    /**
     * Advance the timestep selection by the specified amount.
     *
     * @param amount the number of values to advance (may be negative)
     */
    private void advanceTimestep(int amount) {
        this.timestep = AcorusDemo
                .advanceFloat(timestepValues, timestep, amount);
        PhysicsSpace physicsSpace = appInstance.getPhysicsSpace();
        physicsSpace.setAccuracy(timestep);
    }

    /**
     * Update the indexed status line.
     *
     * @param lineIndex which status line (&ge;0)
     * @param text the text to display, not including the arrow, if any
     */
    private void updateStatusLine(int lineIndex, String text) {
        BitmapText spatial = statusLines[lineIndex];

        if (lineIndex == selectedLine) {
            spatial.setColor(ColorRGBA.Yellow.clone());
            spatial.setText("--> " + text);
        } else {
            spatial.setColor(ColorRGBA.White.clone());
            spatial.setText(" " + text);
        }
    }
}
