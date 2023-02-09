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
import com.jme3.bullet.util.DebugShapeFactory;
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
 * AppState to display the status of the TargetDemo application in an overlay.
 * The overlay consists of status lines, one of which is selected for editing.
 * The overlay is located in the upper-left portion of the display.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class TargetDemoStatus extends SimpleAppState {
    // *************************************************************************
    // constants and loggers

    /**
     * list of damping fractions, in ascending order
     */
    final private static float[] dampingValues
            = {0f, 0.1f, 0.3f, 0.6f, 0.9f, 0.99f};
    /**
     * list of friction coefficients, in ascending order
     */
    final private static float[] frictionValues
            = {0f, 0.1f, 0.2f, 0.5f, 1f, 2f, 4f};
    /**
     * list of gravity magnitudes, in ascending order
     */
    final private static float[] gravityValues
            = {1f, 2f, 5f, 10f, 20f, 30f, 50f};
    /**
     * list of missile initial speeds, in ascending order
     */
    final private static float[] missileInitialSpeedValues
            = {10f, 50f, 100f, 200f, 400f, 1000f};
    /**
     * list of missile masses, in ascending order
     */
    final private static float[] missileMassValues
            = {0.1f, 0.3f, 0.5f, 1f, 3f, 10f};
    /**
     * list of missile radii, in ascending order
     */
    final private static float[] missileRadiusValues
            = {0.1f, 0.3f, 0.5f, 1f, 3f, 10f};
    /**
     * list of restitution fractions, in ascending order
     */
    final private static float[] restitutionValues
            = {0f, 0.1f, 0.3f, 0.6f, 0.9f, 0.99f};
    /**
     * index of the status line for the damping fraction
     */
    final private static int dampingStatusLine = 7;
    /**
     * index of the status line for the friction coefficient
     */
    final private static int frictionStatusLine = 8;
    /**
     * index of the status line for the gravity magnitude
     */
    final private static int gravityStatusLine = 9;
    /**
     * index of the status line for the missile initial speed
     */
    final private static int missileInitialSpeedStatusLine = 6;
    /**
     * index of the status line for the missile mass
     */
    final private static int missileMassStatusLine = 4;
    /**
     * index of the status line for the missile radius
     */
    final private static int missileRadiusStatusLine = 5;
    /**
     * number of lines of text in the overlay
     */
    final private static int numStatusLines = 11;
    /**
     * index of the status line for the platform name
     */
    final private static int platformStatusLine = 2;
    /**
     * index of the status line for the restitution fraction
     */
    final private static int restitutionStatusLine = 10;
    /**
     * index of the status line for the scenario
     */
    final private static int scenarioStatusLine = 3;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(TargetDemoStatus.class.getName());
    /**
     * names of all platforms, in ascending lexicographic order
     */
    final private static String[] platformNames = {
        "box", "cone", "cylinder", "hull", "plane", "roundedRectangle",
        "square", "triangle"
    };
    /**
     * list of scenario names, in ascending lexicographic order
     */
    final private static String[] scenarioNames = {
        "brick tower", "brick wall", "can pyramid", "domino row", "empty",
        "pool", "tenpin"
    };
    // *************************************************************************
    // fields

    /**
     * lines of text displayed in the upper-left corner of the GUI node ([0] is
     * the top line)
     */
    final private BitmapText[] statusLines = new BitmapText[numStatusLines];
    /**
     * flag to enable child coloring for unselected PCOs with compound shapes
     */
    private boolean isChildColoring = false;
    /**
     * flag to force wireframe materials for all unselected PCOs (overrides
     * child coloring)
     */
    private boolean isWireframe = false;
    /**
     * damping fraction for all dynamic bodies (&ge;0, &le;1)
     */
    private float damping = 0.6f;
    /**
     * friction coefficient for all rigid bodies (&ge;0)
     */
    private float friction = 0.5f;
    /**
     * gravity magnitude for all dynamic bodies (in physics-space units per
     * second squared, &ge;0)
     */
    private float gravity = 30f;
    /**
     * initial speed selected for the next missile (in physics-space units per
     * second, &gt;0)
     */
    private float missileInitialSpeed = 200f;
    /**
     * mass selected for the next missile (&gt;0)
     */
    private float missileMass = 0.5f;
    /**
     * radius selected for the next missile (in physics-space units, &gt;0)
     */
    private float missileRadius = 0.5f;
    /**
     * restitution all all rigid bodies (&ge;0, &le;1)
     */
    private float restitution = 0.3f;
    /**
     * index of the line being edited (&ge;1)
     */
    private int selectedLine = platformStatusLine;
    /**
     * name of the platform
     */
    private String platformName = "plane";
    /**
     * name of the selected scenario
     */
    private String scenarioName = "brick wall";
    /**
     * reference to the application instance
     */
    private TargetDemo appInstance;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an uninitialized enabled state.
     */
    TargetDemoStatus() {
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
            case dampingStatusLine:
                advanceDamping(amount);
                break;

            case frictionStatusLine:
                advanceFriction(amount);
                break;

            case gravityStatusLine:
                advanceGravity(amount);
                break;

            case missileInitialSpeedStatusLine:
                advanceMissileInitialSpeed(amount);
                break;

            case missileMassStatusLine:
                advanceMissileMass(amount);
                break;

            case missileRadiusStatusLine:
                advanceMissileRadius(amount);
                break;

            case platformStatusLine:
                advancePlatform(amount);
                break;

            case restitutionStatusLine:
                advanceRestitution(amount);
                break;

            case scenarioStatusLine:
                advanceScenario(amount);
                break;

            default:
                throw new IllegalStateException("line = " + selectedLine);
        }
    }

    /**
     * Determine the selected damping fraction for all dynamic bodies.
     *
     * @return the fraction (&ge;0, &le;1)
     */
    float damping() {
        assert damping >= 0f : damping;
        assert damping <= 1f : damping;
        return damping;
    }

    /**
     * Determine the selected friction coefficient for all rigid bodies.
     *
     * @return the coefficient (&ge;0)
     */
    float friction() {
        assert friction >= 0f : friction;
        return friction;
    }

    /**
     * Determine the selected gravity magnitude for all dynamic bodies.
     *
     * @return the acceleration (in world units per second squared, &ge;0)
     */
    float gravity() {
        assert gravity >= 0f : gravity;
        return gravity;
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
     * Determine the initial speed for a new missile.
     *
     * @return the speed (in physics-space units per second, &gt;0)
     */
    float missileInitialSpeed() {
        assert missileInitialSpeed > 0f : missileInitialSpeed;
        return missileInitialSpeed;
    }

    /**
     * Determine the mass for a new missile.
     *
     * @return the mass (&gt;0)
     */
    float missileMass() {
        assert missileMass > 0f : missileMass;
        return missileMass;
    }

    /**
     * Determine the radius for a new missile.
     *
     * @return the radius (in physics-space units, &gt;0)
     */
    float missileRadius() {
        assert missileRadius > 0f : missileRadius;
        return missileRadius;
    }

    /**
     * Determine the selected type of platform.
     *
     * @return the name (not null, not empty)
     */
    String platformType() {
        assert platformName != null;
        assert !platformName.isEmpty();
        return platformName;
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
     * Determine the selected restitution fraction for all rigid bodies.
     *
     * @return the fraction (&ge;0, &le;1)
     */
    float restitution() {
        assert restitution >= 0f : restitution;
        assert restitution <= 1f : restitution;
        return restitution;
    }

    /**
     * Determine the selected scenario.
     *
     * @return the scenario name (not null, not empty)
     */
    String scenarioName() {
        assert scenarioName != null;
        assert !scenarioName.isEmpty();
        return scenarioName;
    }

    /**
     * Toggle child coloring disabled/enabled.
     */
    void toggleChildColor() {
        this.isChildColoring = !isChildColoring;
        appInstance.setDebugMaterialsAll();
    }

    /**
     * Toggle wireframe disabled/enabled.
     */
    void toggleWireframe() {
        this.isWireframe = !isWireframe;
        appInstance.setDebugMaterialsAll();
        TargetDemo.setDebugShadowMode();
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

        this.appInstance = (TargetDemo) app;
        BitmapFont guiFont
                = assetManager.loadFont("Interface/Fonts/Default.fnt");

        // Add status lines to the guiNode.
        for (int lineIndex = 0; lineIndex < numStatusLines; ++lineIndex) {
            statusLines[lineIndex] = new BitmapText(guiFont);
            float y = cam.getHeight() - 20f * lineIndex;
            statusLines[lineIndex].setLocalTranslation(0f, y, 0f);
            guiNode.attachChild(statusLines[lineIndex]);
        }

        assert MyArray.isSorted(dampingValues);
        assert MyArray.isSorted(frictionValues);
        assert MyArray.isSorted(gravityValues);
        assert MyArray.isSorted(missileInitialSpeedValues);
        assert MyArray.isSorted(missileMassValues);
        assert MyArray.isSorted(missileRadiusValues);
        assert MyArray.isSorted(restitutionValues);

        assert MyArray.isSorted(platformNames);
        assert MyArray.isSorted(scenarioNames);
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

        int index = 1 + Arrays.binarySearch(dampingValues, damping);
        int count = dampingValues.length;
        String message = String.format(
                "Damping #%d of %d:  %.2f", index, count, damping);
        updateStatusLine(dampingStatusLine, message);

        index = 1 + Arrays.binarySearch(frictionValues, friction);
        count = frictionValues.length;
        message = String.format(
                "Friction #%d of %d:  %.1f", index, count, friction);
        updateStatusLine(frictionStatusLine, message);

        index = 1 + Arrays.binarySearch(gravityValues, gravity);
        count = gravityValues.length;
        message = String.format(
                "Gravity #%d of %d:  %.1f", index, count, gravity);
        updateStatusLine(gravityStatusLine, message);

        index = 1 + Arrays.binarySearch(scenarioNames, scenarioName);
        count = scenarioNames.length;
        message = String.format(
                "Scenario #%d of %d:  %s", index, count, scenarioName);
        updateStatusLine(scenarioStatusLine, message);

        index = 1 + Arrays.binarySearch(platformNames, platformName);
        count = platformNames.length;
        message = String.format(
                "Platform #%d of %d:  %s", index, count, platformName);
        updateStatusLine(platformStatusLine, message);

        index = 1 + Arrays.binarySearch(restitutionValues, restitution);
        count = restitutionValues.length;
        message = String.format(
                "Restitution #%d of %d:  %.2f", index, count, restitution);
        updateStatusLine(restitutionStatusLine, message);

        index = 1 + Arrays.binarySearch(
                missileInitialSpeedValues, missileInitialSpeed);
        count = missileInitialSpeedValues.length;
        message = String.format("Missile speed #%d of %d:  %.0f", index, count,
                missileInitialSpeed);
        updateStatusLine(missileInitialSpeedStatusLine, message);

        index = 1 + Arrays.binarySearch(missileMassValues, missileMass);
        count = missileMassValues.length;
        message = String.format(
                "Missile mass #%d of %d:  %.2f", index, count, missileMass);
        updateStatusLine(missileMassStatusLine, message);

        index = 1 + Arrays.binarySearch(missileRadiusValues, missileRadius);
        count = missileRadiusValues.length;
        message = String.format(
                "Missile radius #%d of %d:  %.2f", index, count, missileRadius);
        updateStatusLine(missileRadiusStatusLine, message);
    }
    // *************************************************************************
    // private methods

    /**
     * Advance the damping selection by the specified amount.
     *
     * @param amount the number of values to advance (may be negative)
     */
    private void advanceDamping(int amount) {
        this.damping = AcorusDemo.advanceFloat(dampingValues, damping, amount);
        appInstance.setDampingAll(damping);
    }

    /**
     * Advance the friction selection by the specified amount.
     *
     * @param amount the number of values to advance (may be negative)
     */
    private void advanceFriction(int amount) {
        this.friction
                = AcorusDemo.advanceFloat(frictionValues, friction, amount);
        appInstance.setFrictionAll(friction);
    }

    /**
     * Advance the gravity selection by the specified amount.
     *
     * @param amount the number of values to advance (may be negative)
     */
    private void advanceGravity(int amount) {
        this.gravity = AcorusDemo.advanceFloat(gravityValues, gravity, amount);
        appInstance.setGravityAll(gravity);
    }

    /**
     * Advance the initial speed selection by the specified amount.
     *
     * @param amount the number of values to advance (may be negative)
     */
    private void advanceMissileInitialSpeed(int amount) {
        this.missileInitialSpeed = AcorusDemo.advanceFloat(
                missileInitialSpeedValues, missileInitialSpeed, amount);
    }

    /**
     * Advance the missile mass selection by the specified amount.
     *
     * @param amount the number of values to advance (may be negative)
     */
    private void advanceMissileMass(int amount) {
        this.missileMass = AcorusDemo
                .advanceFloat(missileMassValues, missileMass, amount);
    }

    /**
     * Advance the missile radius selection by the specified amount.
     *
     * @param amount the number of values to advance (may be negative)
     */
    private void advanceMissileRadius(int amount) {
        this.missileRadius = AcorusDemo
                .advanceFloat(missileRadiusValues, missileRadius, amount);
    }

    /**
     * Advance the platform selection by the specified amount.
     *
     * @param amount the number of values to advance (may be negative)
     */
    private void advancePlatform(int amount) {
        this.platformName
                = AcorusDemo.advanceString(platformNames, platformName, amount);
        appInstance.restartScenario();
    }

    /**
     * Advance the restitution selection by the specified amount.
     *
     * @param amount the number of values to advance (may be negative)
     */
    private void advanceRestitution(int amount) {
        this.restitution = AcorusDemo
                .advanceFloat(restitutionValues, restitution, amount);
        appInstance.setRestitutionAll(restitution);
    }

    /**
     * Advance the scenario selection by the specified amount.
     *
     * @param amount the number of values to advance (may be negative)
     */
    private void advanceScenario(int amount) {
        this.scenarioName
                = AcorusDemo.advanceString(scenarioNames, scenarioName, amount);
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
            message += "Lit/ChildColoring ";
        } else {
            message += "Lit ";
        }
        String viewOptions = appInstance.describePhysicsDebugOptions();
        message += viewOptions;
        statusLines[0].setText(message);

        int numActiveBodies = appInstance.countActive();
        int numCachedMeshes = DebugShapeFactory.countCachedMeshes();
        boolean isPaused = appInstance.isPaused();
        message = String.format("activeBodies=%d  cachedMeshes=%d%s",
                numActiveBodies, numCachedMeshes, isPaused ? "  PAUSED" : "");
        statusLines[1].setText(message);
    }
}
