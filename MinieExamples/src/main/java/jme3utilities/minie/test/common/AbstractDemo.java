/*
 Copyright (c) 2018-2021, Stephen Gold
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
package jme3utilities.minie.test.common;

import com.jme3.font.Rectangle;
import com.jme3.input.KeyInput;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.texture.Texture;
import java.util.Arrays;
import java.util.Map;
import java.util.TreeMap;
import java.util.logging.Logger;
import jme3utilities.MyAsset;
import jme3utilities.MySpatial;
import jme3utilities.Validate;
import jme3utilities.debug.AxesVisualizer;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;
import jme3utilities.ui.ActionApplication;
import jme3utilities.ui.HelpUtils;
import jme3utilities.ui.InputMode;

/**
 * An abstract ActionApplication with additional data and methods for use in
 * demos. TODO use jme3-utilities-ui
 *
 * @author Stephen Gold sgold@sonic.net
 */
abstract public class AbstractDemo extends ActionApplication {
    // *************************************************************************
    // constants and loggers

    /**
     * animation/physics speed when paused
     */
    final public static float pausedSpeed = 1e-12f;
    /**
     * message logger for this class
     */
    final public static Logger loggerA
            = Logger.getLogger(AbstractDemo.class.getName());
    /**
     * action strings that onAction() recognizes
     */
    final public static String asCollectGarbage = "collect garbage";
    final public static String asToggleHelp = "toggle help";
    final public static String asTogglePause = "toggle pause";
    final public static String asToggleWorldAxes = "toggle worldAxes";
    // *************************************************************************
    // fields

    /**
     * visualizer for the world axes
     */
    private AxesVisualizer worldAxes = null;
    /**
     * library of named geometry materials
     */
    final private Map<String, Material> namedMaterials = new TreeMap<>();
    /**
     * Node for displaying hotkey help in the GUI scene
     */
    private Node helpNode;
    /**
     * Node for displaying "toggle help: H" in the GUI scene
     */
    private Node minHelpNode;
    // *************************************************************************
    // new methods exposed

    /**
     * Advance a float selection by the specified (cyclic) amount.
     *
     * @param valuesArray an array of values in ascending order (not null,
     * unaffected)
     * @param startValue the starting value (found in values[])
     * @param amount the number of values to advance (may be negative)
     * @return the new (advanced) value
     */
    public static float advanceFloat(float[] valuesArray, float startValue,
            int amount) {
        int index = Arrays.binarySearch(valuesArray, startValue);

        float result;
        if (index < 0) {
            result = valuesArray[0];
        } else {
            assert valuesArray[index] == startValue;
            index = MyMath.modulo(index + amount, valuesArray.length);
            result = valuesArray[index];
        }

        return result;
    }

    /**
     * Advance a String selection by the specified (cyclic) amount.
     *
     * @param valuesArray an array of values in ascending order (not null,
     * unaffected)
     * @param startValue the starting value (found in values[])
     * @param amount the number of values to advance (may be negative)
     * @return the new (advanced) value
     */
    public static String advanceString(String[] valuesArray, String startValue,
            int amount) {
        int index = Arrays.binarySearch(valuesArray, startValue);

        String result;
        if (index < 0) {
            result = valuesArray[0];
        } else {
            assert valuesArray[index].equals(startValue);
            index = MyMath.modulo(index + amount, valuesArray.length);
            result = valuesArray[index];
        }

        return result;
    }

    /**
     * Test whether the world axes are enabled.
     *
     * @return true if enabled, otherwise false
     */
    public boolean areWorldAxesEnabled() {
        boolean result;
        if (worldAxes == null) {
            result = false;
        } else {
            result = worldAxes.isEnabled();
        }

        return result;
    }

    /**
     * Generate full and minimal versions of the hotkey help. Attach the minimal
     * one to the GUI scene.
     *
     * @param bounds the desired screen coordinates (not null, unaffected)
     */
    public void attachHelpNode(Rectangle bounds) {
        Validate.nonNull(bounds, "bounds");

        InputMode inputMode = getDefaultInputMode();
        float extraSpace = 20f;
        helpNode = HelpUtils.buildNode(inputMode, bounds, guiFont, extraSpace);
        helpNode.move(0f, 0f, 1f); // move (slightly) to the front

        InputMode dummyMode = new InputMode("dummy") {
            @Override
            protected void defaultBindings() {
            }

            @Override
            public void onAction(String s, boolean b, float f) {
            }
        };
        dummyMode.bind(asToggleHelp, KeyInput.KEY_H);

        float width = 100f; // in pixels
        float height = bounds.height;
        float x = bounds.x + bounds.width - width;
        float y = bounds.y;
        Rectangle dummyBounds = new Rectangle(x, y, width, height);

        minHelpNode = HelpUtils.buildNode(dummyMode, dummyBounds, guiFont, 0f);
        guiNode.attachChild(minHelpNode);
    }

    /**
     * Add a visualizer for the axes of the world coordinate system.
     *
     * @param axisLength the desired length for each axis arrow (in world units,
     * &gt;0)
     */
    public void attachWorldAxes(float axisLength) {
        Validate.positive(axisLength, "axis length");

        if (worldAxes != null) {
            rootNode.removeControl(worldAxes);
        }

        worldAxes = new AxesVisualizer(assetManager, axisLength);
        worldAxes.setLineWidth(AxesVisualizer.widthForSolid);

        rootNode.addControl(worldAxes);
        worldAxes.setEnabled(true);
    }

    /**
     * Translate a model's center so that the model rests on the X-Z plane, and
     * its center lies on the Y axis.
     *
     * @param cgModel (not null, modified)
     */
    public void centerCgm(Spatial cgModel) {
        Validate.nonNull(cgModel, "model");

        Vector3f[] minMax = MySpatial.findMinMaxCoords(cgModel);
        Vector3f min = minMax[0];
        Vector3f max = minMax[1];
        Vector3f center = MyVector3f.midpoint(min, max, null);
        Vector3f offset = new Vector3f(center.x, min.y, center.z);

        Vector3f location = cgModel.getWorldTranslation();
        location.subtractLocal(offset);
        MySpatial.setWorldLocation(cgModel, location);
    }

    /**
     * Find the named Material in the library.
     *
     * @param name the name of the Material to find (not null)
     * @return the pre-existing instance, or null if not found
     */
    public Material findMaterial(String name) {
        Validate.nonNull(name, "name");

        Material result = namedMaterials.get(name);
        return result;
    }

    /**
     * Initialize the library of named materials. Invoke during startup.
     */
    public void generateMaterials() {
        ColorRGBA green = new ColorRGBA(0f, 0.12f, 0f, 1f);
        Material platform = MyAsset.createShadedMaterial(assetManager, green);
        registerMaterial("platform", platform);

        Texture texture = MyAsset.loadTexture(assetManager,
                "Textures/greenTile.png", true);
        texture.setMinFilter(Texture.MinFilter.Trilinear);
        texture.setWrap(Texture.WrapMode.Repeat);
        Material greenTile
                = MyAsset.createShadedMaterial(assetManager, texture);
        registerMaterial("greenTile", greenTile);
    }

    /**
     * Test whether animation is paused.
     *
     * @return true if paused, otherwise false
     */
    public boolean isPaused() {
        if (speed <= pausedSpeed) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Add a Material to the library.
     *
     * @param name the desired name for the Material, which is also the key that
     * will be used to find it (not null)
     * @param material (not null, alias created)
     */
    public void registerMaterial(String name, Material material) {
        Validate.nonNull(name, "name");
        Validate.nonNull(material, "material");
        assert !namedMaterials.containsKey(name);

        material.setName(name);
        namedMaterials.put(name, material);
    }

    /**
     * Scale the specified C-G model uniformly so that it has the specified
     * height, assuming Y-up orientation.
     *
     * @param cgModel (not null, modified)
     * @param height the desired height (in world units, &gt;0)
     */
    public void setCgmHeight(Spatial cgModel, float height) {
        Validate.nonNull(cgModel, "model");
        Validate.positive(height, "height");

        Vector3f[] minMax = MySpatial.findMinMaxCoords(cgModel);
        Vector3f min = minMax[0];
        Vector3f max = minMax[1];
        float oldHeight = max.y - min.y;
        if (oldHeight > 0f) {
            cgModel.scale(height / oldHeight);
        }
    }

    /**
     * Toggle the animation and physics simulation: paused/running.
     */
    public void togglePause() {
        float newSpeed = isPaused() ? 1f : pausedSpeed;
        setSpeed(newSpeed);
    }
    // *************************************************************************
    // ActionApplication methods

    /**
     * Process an action that wasn't handled by the active InputMode.
     *
     * @param actionString textual description of the action (not null)
     * @param ongoing true if the action is ongoing, otherwise false
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void onAction(String actionString, boolean ongoing, float tpf) {
        if (ongoing) {
            switch (actionString) {
                case asCollectGarbage:
                    System.gc();
                    return;
                case asToggleHelp:
                    toggleHelp();
                    return;
                case asTogglePause:
                    togglePause();
                    return;
                case asToggleWorldAxes:
                    toggleWorldAxes();
                    return;
            }
        }
        super.onAction(actionString, ongoing, tpf);
    }
    // *************************************************************************
    // private methods

    /**
     * Toggle between the full help node and the minimal one.
     */
    private void toggleHelp() {
        if (helpNode.getParent() == null) {
            minHelpNode.removeFromParent();
            guiNode.attachChild(helpNode);
        } else {
            helpNode.removeFromParent();
            guiNode.attachChild(minHelpNode);
        }
    }

    /**
     * Toggle visualization of world axes.
     */
    private void toggleWorldAxes() {
        boolean enabled = worldAxes.isEnabled();
        worldAxes.setEnabled(!enabled);
    }
}
