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
import com.jme3.math.ColorRGBA;
import com.jme3.scene.Node;
import java.util.Collection;
import java.util.List;
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

        Map<String, String> actionToList = mapActions(inputMode);

        Node result = new Node("help node");
        float x = bounds.x;
        float y = bounds.y;

        for (Map.Entry<String, String> entry : actionToList.entrySet()) {
            BitmapText spatial = new BitmapText(font);
            result.attachChild(spatial);
            spatial.setSize(font.getCharSet().getRenderedSize());

            String actionName = entry.getKey();
            String hotkeyList = entry.getValue();
            String string = actionName + ": " + hotkeyList;
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

            if (actionName.equals("toggle help")) {
                spatial.setColor(ColorRGBA.Yellow);
            }
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
                    String newList = oldList + "/" + hot;
                    actionsToHots.put(action, newList);
                } else {
                    actionsToHots.put(action, hot);
                }
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
            result = MyString.firstToLower(suffix);
        } else if (result.startsWith("FLYCAM_")) {
            String suffix = MyString.remainder(result, "FLYCAM_");
            result = "camera " + MyString.firstToLower(suffix);
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
        result = result.replace("numpad ", "num");

        return result;
    }
}
