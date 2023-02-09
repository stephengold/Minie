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
import com.jme3.bullet.collision.shapes.CollisionShape;
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
 * AppState to display the status of the SplitDemo application in an overlay.
 * The overlay consists of status lines, one of which is selected for editing.
 * The overlay is located in the upper-left portion of the display.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class SplitDemoStatus extends SimpleAppState {
    // *************************************************************************
    // constants and loggers

    /**
     * list of collision margins, in ascending order
     */
    final private static float[] marginValues = {0.008f, 0.04f, 0.2f, 1f};
    /**
     * index of the status line for the collision margin
     */
    final private static int marginStatusLine = 3;
    /**
     * number of lines of text in the overlay
     */
    final private static int numStatusLines = 4;
    /**
     * index of the status line for the type of the next shape
     */
    final private static int shapeStatusLine = 2;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(SplitDemoStatus.class.getName());
    /**
     * names of all shape types, in ascending lexicographic order
     * <p>
     * "bowlingPin" is omitted because its hull shapes are too complex for
     * real-time splitting.
     * <p>
     * "sieve" is omitted because plane shapes are not splittable.
     */
    final private static String[] shapeNames = {
        "ankh", "banana", "barbell", "barrel", "bedOfNails", "box", "candyDish",
        "capsule", "cone", "corner", "cylinder", "digit", "dimples", "dome",
        "duck", "football", "frame", "halfPipe", "heart", "horseshoe", "hull",
        "iBeam", "knucklebone", "ladder", "letter", "lidlessBox", "link",
        "multiSphere", "platonic", "prism", "pyramid", "roundedRectangle",
        "smooth", "snowman", "sphere", "star", "sword", "table", "teapot",
        "teapotGi", "tetrahedron", "thumbTack", "top", "torus", "tray",
        "triangle", "triangularFrame", "trident", "washer"
    };
    // *************************************************************************
    // fields

    /**
     * lines of text displayed in the upper-left corner of the GUI node ([0] is
     * the top line)
     */
    final private BitmapText[] statusLines = new BitmapText[numStatusLines];
    /**
     * flag to enable child coloring for PCOs with compound shapes
     */
    private boolean isChildColoring = false;
    /**
     * flag to force wireframe materials for all PCOs (overrides child coloring)
     */
    private boolean isWireframe = false;
    /**
     * reference to the application instance
     */
    private SplitDemo appInstance;
    /**
     * index of the line being edited (&ge;1)
     */
    private int selectedLine = shapeStatusLine;
    /**
     * name of the type selected for the next shape
     */
    private String nextShapeType = "prism";
    // *************************************************************************
    // constructors

    /**
     * Instantiate an uninitialized enabled state.
     */
    SplitDemoStatus() {
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
        int firstField = 2;
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
            case marginStatusLine:
                advanceMargin(amount);
                break;
            case shapeStatusLine:
                advanceShape(amount);
                break;
            default:
                throw new IllegalStateException("line = " + selectedLine);
        }
    }

    /**
     * Test whether child coloring is enabled.
     *
     * @return true if enabled, otherwise false
     */
    boolean isChildColoring() {
        return isChildColoring;
    }

    /**
     * Test whether wireframe materials are enabled.
     *
     * @return true if enabled, otherwise false
     */
    boolean isWireframe() {
        return isWireframe;
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

    /**
     * Return the name of the selected shape type.
     *
     * @return the shape name (not null, not empty)
     */
    String shapeName() {
        assert nextShapeType != null;
        assert !nextShapeType.isEmpty();
        return nextShapeType;
    }

    /**
     * Toggle child coloring disabled/enabled.
     */
    void toggleChildColor() {
        this.isChildColoring = !isChildColoring;
    }

    /**
     * Toggle wireframe disabled/enabled.
     */
    void toggleWireframe() {
        this.isWireframe = !isWireframe;
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

        this.appInstance = (SplitDemo) app;
        BitmapFont guiFont
                = assetManager.loadFont("Interface/Fonts/Default.fnt");

        // Add status lines to the guiNode.
        for (int lineIndex = 0; lineIndex < numStatusLines; ++lineIndex) {
            statusLines[lineIndex] = new BitmapText(guiFont);
            float y = cam.getHeight() - 20f * lineIndex;
            statusLines[lineIndex].setLocalTranslation(0f, y, 0f);
            guiNode.attachChild(statusLines[lineIndex]);
        }

        assert MyArray.isSorted(marginValues);
        assert MyArray.isSorted(shapeNames);
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

        updateStatusText();

        float margin = CollisionShape.getDefaultMargin();
        int index = 1 + Arrays.binarySearch(marginValues, margin);
        int count = marginValues.length;
        String message = String.format(
                "Margin #%d of %d:  %.3f", index, count, margin);
        updateStatusLine(marginStatusLine, message);

        index = 1 + Arrays.binarySearch(shapeNames, nextShapeType);
        count = shapeNames.length;
        message = String.format(
                "Shape #%d of %d:  %s", index, count, nextShapeType);
        updateStatusLine(shapeStatusLine, message);
    }
    // *************************************************************************
    // private methods

    /**
     * Advance the collision-margin selection by the specified amount.
     *
     * @param amount the number of values to advance (may be negative)
     */
    private static void advanceMargin(int amount) {
        float margin = CollisionShape.getDefaultMargin();
        margin = AcorusDemo.advanceFloat(marginValues, margin, amount);
        CollisionShape.setDefaultMargin(margin);
    }

    /**
     * Advance the next-shape selection by the specified amount.
     *
     * @param amount the number of values to advance (may be negative)
     */
    private void advanceShape(int amount) {
        this.nextShapeType
                = AcorusDemo.advanceString(shapeNames, nextShapeType, amount);
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
            spatial.setColor(ColorRGBA.Yellow);
            spatial.setText("-> " + text);
        } else {
            spatial.setColor(ColorRGBA.White);
            spatial.setText(" " + text);
        }
    }

    /**
     * Update the status text (top 2 lines).
     */
    private void updateStatusText() {
        String message = " View: ";
        if (isWireframe) {
            message += "Wireframe ";
        } else if (isChildColoring) {
            message += "Lit/ChildColored ";
        } else {
            message += "Lit ";
        }
        String viewOptions = appInstance.describePhysicsDebugOptions();
        message += viewOptions;
        statusLines[0].setText(message);

        int numActiveBodies = appInstance.countActive();
        float splitRadians = SplitDemo.splitAngle();
        float splitDegrees = MyMath.toDegrees(splitRadians);
        boolean isPaused = appInstance.isPaused();
        message = String.format(" activeBodies=%d splitAngle=%.0f deg%s",
                numActiveBodies, splitDegrees, isPaused ? "  PAUSED" : "");
        statusLines[1].setText(message);
    }
}
