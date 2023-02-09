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
package jme3utilities.minie.tuner;

import com.jme3.app.Application;
import com.jme3.app.state.AppStateManager;
import de.lessvoid.nifty.controls.Button;
import de.lessvoid.nifty.elements.Element;
import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.InitialState;
import jme3utilities.MyString;
import jme3utilities.nifty.GuiScreenController;
import jme3utilities.nifty.PopupMenuBuilder;
import jme3utilities.ui.InputMode;

/**
 * The screen controller for the "filePath" screen of VhacdTuner.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class FilePathScreen extends GuiScreenController {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final static Logger logger
            = Logger.getLogger(FilePathScreen.class.getName());
    // *************************************************************************
    // fields

    /**
     * element of GUI button to proceed to the next Screen
     */
    private Element nextElement;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an uninitialized, disabled screen that will not be enabled
     * during initialization.
     */
    FilePathScreen() {
        super("filePath", "Interface/Nifty/screens/tuner/filePath.xml",
                InitialState.Disabled);
        setSubmenuWarp(0.5f, 0.5f);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Handle a "browse" action to begin browsing the file system.
     */
    void browse() {
        Map<String, File> fileMap = Heart.driveMap();

        // Add the current working directory to the file map.
        String workPath = System.getProperty("user.dir");
        File work = new File(workPath);
        if (work.isDirectory()) {
            String absolutePath = Heart.fixPath(workPath);
            fileMap.put(absolutePath, work);
        }

        // Add the user's home directory to the file map.
        String homePath = System.getProperty("user.home");
        File home = new File(homePath);
        if (home.isDirectory()) {
            String absolutePath = Heart.fixPath(homePath);
            fileMap.put(absolutePath, home);
        }
        /*
         * If a filesystem path is selected, add its parent directory
         * to the file map.
         */
        Model model = VhacdTuner.getModel();
        String filePath = model.filePath();
        if (!filePath.isEmpty()) {
            File file = new File(filePath);
            File parent = file.getParentFile();
            String parentPath = parent.getPath();
            String absolutePath = Heart.fixPath(parentPath);
            fileMap.put(absolutePath, parent);
        }

        // Build and show a popup menu.
        String actionPrefix = "set pathPrefix ";
        PopupMenuBuilder builder = buildFileMenu(fileMap);
        showPopupMenu(actionPrefix, builder);
    }

    /**
     * Determine user feedback (if any) regarding the "next screen" action.
     *
     * @return "" if ready to proceed, otherwise an explanatory message
     */
    static String feedback() {
        Model model = VhacdTuner.getModel();
        String filePath = model.filePath();

        String result = "";
        if (!filePath.contains("/")) {
            result = "No model is selected yet.";
        }

        return result;
    }

    /**
     * Handle a "set pathPrefix" action.
     *
     * @param pathPrefix the user-selected filesystem-path prefix (not null, not
     * empty)
     */
    void setPathPrefix(String pathPrefix) {
        assert pathPrefix != null;

        String absPathPrefix = Heart.fixPath(pathPrefix);
        File file = new File(absPathPrefix);

        boolean isDirectory = file.isDirectory();
        if (!isDirectory && file.canRead()) { // complete path to readable file
            Model model = VhacdTuner.getModel();
            model.setFilePath(absPathPrefix);

        } else {
            Map<String, File> fileMap;
            String actionPrefix;
            if (isDirectory) {
                fileMap = directoryMap(absPathPrefix, "");
                actionPrefix = "set pathPrefix " + absPathPrefix;

            } else { // an incomplete path
                File parent = file.getParentFile();
                String parentPath = parent.getPath();
                parentPath = Heart.fixPath(parentPath);

                String name = file.getName();
                fileMap = directoryMap(parentPath, name);
                actionPrefix = "set pathPrefix " + parentPath;
            }
            if (!actionPrefix.endsWith("/")) {
                actionPrefix += "/";
            }

            // Build and show a popup menu.
            PopupMenuBuilder builder = buildFileMenu(fileMap);
            showPopupMenu(actionPrefix, builder);
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
    public void initialize(
            AppStateManager stateManager, Application application) {
        super.initialize(stateManager, application);

        InputMode inputMode = InputMode.findMode("filePath");
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

        Model model = VhacdTuner.getModel();
        String filePath = model.filePath();
        setStatusText("filePath", " " + filePath);

        String feedback = feedback();
        setStatusText("feedback", feedback);
        if (feedback.isEmpty()) {
            nextElement.show();
        } else {
            nextElement.hide();
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Build a file-selection popup menu based on the specified file map.
     *
     * @param fileMap the map of files to include (not null)
     * @return a new instance (not null)
     */
    private PopupMenuBuilder buildFileMenu(Map<String, File> fileMap) {
        assert fileMap != null;

        // Generate a list of file names (and prefixes) to display in the menu.
        Set<String> nameSet = fileMap.keySet();
        assert !nameSet.contains(".");
        List<String> nameList = new ArrayList<>(nameSet);

        // Reduce the list as necessary to fit on the screen.
        int height = cam.getHeight();
        int maxMenuItems = height / 26;
        MyString.reduce(nameList, maxMenuItems);

        // Sort the list and build the menu.
        Collections.sort(nameList);
        PopupMenuBuilder result = new PopupMenuBuilder();
        for (String name : nameList) {
            if (fileMap.containsKey(name)) {
                File file = fileMap.get(name);
                if (file.isDirectory()) {
                    result.add(name, "Textures/icons/folder.png");
                } else if (name.endsWith(".j3o")) {
                    result.add(name, "Textures/icons/jme.png");
                } else if (name.endsWith(".glb")) {
                    result.add(name, "Textures/icons/jme.png");
                } else if (name.endsWith(".gltf")) {
                    result.add(name, "Textures/icons/jme.png");
                }
            } else { // prefix
                result.add(name, "Textures/icons/ellipsis.png");
            }
        }

        return result;
    }

    /**
     * Build a map of files, in the specified directory, whose names have the
     * specified prefix.
     *
     * @param dirPath the filesystem path to the directory (not null)
     * @param namePrefix required name prefix (not null)
     * @return a new instance (not null)
     */
    private static Map<String, File> directoryMap(
            String dirPath, String namePrefix) {
        assert dirPath != null;
        assert namePrefix != null;

        Map<String, File> fileMap = new TreeMap<>();
        /*
         * Initialize the map with subdirectories and readable files.
         * Exclude names that start with ".".
         */
        File directory = new File(dirPath);
        File[] files = directory.listFiles();
        if (files != null) {
            for (File file : files) {
                if (file.isDirectory() || file.canRead()) {
                    String name = file.getName();
                    if (name.startsWith(namePrefix) && !name.startsWith(".")) {
                        fileMap.put(name, file);
                    }
                }
            }
        }

        // Add ".." if a parent directory exists.
        File parent = directory.getParentFile();
        if (parent != null) {
            if ("..".startsWith(namePrefix)) {
                fileMap.put("..", parent);
            }
        }

        return fileMap;
    }
}
