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
package jme3utilities.minie.test;

import com.jme3.font.BitmapFont;
import com.jme3.font.BitmapText;
import com.jme3.font.Rectangle;
import com.jme3.scene.Node;
import java.util.Collection;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.TreeMap;
import java.util.logging.Logger;
import jme3utilities.MyString;
import jme3utilities.Validate;
import jme3utilities.ui.InputMode;

/**
 * Utility methods to generate hotkey clues for action-oriented applications.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class HelpUtils {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(HelpUtils.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private HelpUtils() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Build a Node to describe the hotkey mappings of the specified InputMode
     * within the specified bounds.
     *
     * @param inputMode (not null, unaffected)
     * @param bounds (not null, unaffected)
     * @param font (not null, unaffected)
     * @param space amount of extra space between hotkey descriptions (in
     * pixels)
     * @return a new Node, suitable for attachment to the GUI node
     */
    static Node buildNode(InputMode inputMode, Rectangle bounds,
            BitmapFont font, float space) {
        Validate.nonNull(inputMode, "input mode");
        Validate.nonNull(bounds, "bounds");
        Validate.nonNull(font, "font");

        Map<String, String> actionToList = HelpUtils.mapActions(inputMode);
        Map<String, String> listToAction = HelpUtils.invert(actionToList);

        Node result = new Node("help node");
        float x = bounds.x;
        float y = bounds.y;
        for (int group = 1; group <= 3; ++group) {
            for (Map.Entry<String, String> entry : listToAction.entrySet()) {
                String hotkeyList = entry.getKey();
                int listGroup;
                if (hotkeyList.length() == 1) {
                    listGroup = 1; // single letter key
                } else if (hotkeyList.matches("^f1?[0-9]$")) {
                    listGroup = 2; // single function key
                } else {
                    listGroup = 3; // miscellaneous
                }
                if (listGroup == group) {
                    BitmapText spatial = new BitmapText(font);
                    result.attachChild(spatial);
                    spatial.setSize(font.getCharSet().getRenderedSize());

                    String actionName = entry.getValue();
                    String string = hotkeyList + ": " + actionName;
                    spatial.setText(string);
                    float textWidth = spatial.getLineWidth();
                    if (x > bounds.x
                            && x + textWidth > bounds.x + bounds.width) {
                        // start a new line of text
                        y -= spatial.getHeight();
                        x = bounds.x;
                    }
                    spatial.setLocalTranslation(x, y, 0f);
                    x += textWidth + space;
                }
            }
        }

        return result;
    }

    /**
     * Invert the specified String-to-String map. TODO add to heart lib
     *
     * @param input (not null, unaffected)
     * @return a new String-to-String map
     */
    static Map<String, String> invert(Map<String, String> input) {
        Map<String, String> result = new TreeMap<>();
        for (Map.Entry<String, String> entry : input.entrySet()) {
            String key = entry.getKey();
            String value = entry.getValue();
            if (result.containsKey(key)) {
                throw new IllegalArgumentException("Non-invertible map.");
            }
            result.put(value, key);
        }

        return result;
    }

    /**
     * For the specified InputMode, construct a Map from beautified action names
     * to comma-separated, compressed hotkey names.
     *
     * @param inputMode (not null, unaffected)
     * @return a new String-to-String Map
     */
    static Map<String, String> mapActions(InputMode inputMode) {
        List<String> actionNames = inputMode.listActionNames();
        Map<String, String> actionsToHots = new TreeMap<>();
        for (String actionName : actionNames) {
            Collection<String> hotkeyNames = inputMode.findHotkeys(actionName);
            for (String hotkeyName : hotkeyNames) {
                String action = beautify(actionName);
                String hot = compress(hotkeyName);
                if (actionsToHots.containsKey(action)) {
                    String oldList = actionsToHots.get(action);
                    String newList = oldList + "," + hot;
                    actionsToHots.put(action, newList);
                }
                actionsToHots.put(action, hot);
            }
        }

        return actionsToHots;
    }
    // *************************************************************************
    // private methods

    /**
     * Beautify the specified action name.
     *
     * @param actionName the action name (not null)
     * @return the beautified name (not null)
     */
    private static String beautify(String actionName) {
        Validate.nonNull(actionName, "action name");

        String result = actionName;
        if (result.startsWith("signal ")) {
            result = MyString.remainder(result, "signal ");
        }
        if (result.startsWith("SIMPLEAPP_")) {
            String suffix = MyString.remainder(result, "SIMPLEAPP_");
            result = firstToLower(suffix);
        } else if (result.startsWith("FLYCAM_")) {
            String suffix = MyString.remainder(result, "FLYCAM_");
            result = "camera " + firstToLower(suffix);
        }

        return result;
    }

    /**
     * Compress the specified hotkey name.
     *
     * @param hotkeyName the hotkey name (not null)
     * @return the compressed name (not null)
     */
    private static String compress(String hotkeyName) {
        Validate.nonNull(hotkeyName, "hotkey name");

        String result = hotkeyName;
        if (result.endsWith(" arrow")) {
            result = MyString.removeSuffix(result, " arrow");
        }

        return result;
    }

    /**
     * Convert the first character of the specified String to lower case.
     *
     * @param input the input string (not null)
     * @return the converted String (not null)
     */
    private static String firstToLower(String input) {
        Validate.nonNull(input, "input");

        String result = input;
        if (!input.isEmpty()) {
            String first = input.substring(0, 1);
            first = first.toLowerCase(Locale.ROOT);
            String rest = input.substring(1);
            result = first + rest;
        }

        return result;
    }
}
