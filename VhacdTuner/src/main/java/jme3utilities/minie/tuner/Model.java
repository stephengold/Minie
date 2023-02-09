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

import com.jme3.asset.AssetManager;
import com.jme3.scene.Spatial;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.logging.Logger;
import jme3utilities.MyMesh;
import jme3utilities.math.VectorSet;
import jme3utilities.ui.Locators;
import vhacd.ACDMode;
import vhacd.VHACD;
import vhacd.VHACDParameters;
import vhacd4.Vhacd4;
import vhacd4.Vhacd4Parameters;

/**
 * The state information (MVC model) in the VhacdTuner application.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class Model {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final static Logger logger
            = Logger.getLogger(Model.class.getName());
    // *************************************************************************
    // fields

    /**
     * whether to display axes in the TestScreen
     */
    private boolean isShowingAxes = false;
    /**
     * parameters and results for the left side
     */
    private DecompositionTest leftTest;
    /**
     * parameters and results for the right side
     */
    private DecompositionTest rightTest;
    /**
     * parameters and results for the test currently running, or null if none
     */
    private DecompositionTest runningTest;
    /**
     * parameters and results for the test being ranked, or null if none
     */
    private DecompositionTest testBeingRanked;
    /**
     * Exception that occurred during load
     */
    private Exception loadException;
    /**
     * distance of the furthest vertex from the model origin, or 2.5 if no
     * vertices
     */
    private float radius = 2.5f;
    /**
     * maximum rank of the test being ranked (0 &rarr; best, 1 &rarr;
     * second-best, and so on), or -1 if no test being ranked
     */
    private int maxRank = -1;
    /**
     * minimum rank of the test being ranked (0 &rarr; best, 1 &rarr;
     * second-best, and so on), or -1 if no test being ranked
     */
    private int minRank = -1;
    /**
     * number of components in the filesystem path to the asset root
     */
    private int numComponentsInRoot;
    /**
     * ranked test results, ordered from best to worst
     */
    final private List<DecompositionTest> rankedTests = new LinkedList<>();
    /**
     * map classic parameters to test results
     */
    final private Map<VHACDParameters, DecompositionTest> classicMap
            = new HashMap<>(16);
    /**
     * map v4 parameters to test results
     */
    final private Map<Vhacd4Parameters, DecompositionTest> v4Map
            = new HashMap<>(16);
    /**
     * root spatial of the loaded C-G model
     */
    private Spatial rootSpatial;
    /**
     * components of the filesystem path to the C-G model (not null)
     */
    private String[] filePathComponents = new String[0];
    // *************************************************************************
    // new methods exposed

    /**
     * Determine the asset path to the J3O/glTF asset. The filesystem path must
     * be set.
     *
     * @return the path (not null, not empty)
     */
    String assetPath() {
        int numComponents = filePathComponents.length;
        if (numComponents == 0) {
            throw new RuntimeException("Filesystem path not set.");
        }
        assert numComponentsInRoot < numComponents : numComponents;
        String[] resultComponents = Arrays.copyOfRange(
                filePathComponents, numComponentsInRoot, numComponents);
        String result = String.join("/", resultComponents);
        result = "/" + result;

        return result;
    }

    /**
     * Determine the filesystem path to the asset root. The filesystem path must
     * be set.
     *
     * @return the path (not null, not empty)
     */
    String assetRoot() {
        int numComponents = filePathComponents.length;
        if (numComponents == 0) {
            throw new RuntimeException("Filesystem path not set.");
        }
        assert numComponentsInRoot < numComponents : numComponents;
        String[] resultComponents = Arrays.copyOfRange(
                filePathComponents, 0, numComponentsInRoot);
        String result = String.join("/", resultComponents);
        result += "/";

        assert result != null;
        assert !result.isEmpty();
        return result;
    }

    /**
     * Count how many tests have been ranked.
     *
     * @return the count (&ge;0)
     */
    int countRankedTests() {
        int result = rankedTests.size();
        assert result >= 0 : result;
        return result;
    }

    /**
     * Return the filesystem path to the J3O/glTF file.
     *
     * @return the path (not null, may be empty)
     */
    String filePath() {
        String result = String.join("/", filePathComponents);
        assert result != null;
        return result;
    }

    /**
     * Return the rank of the specified test result.
     *
     * @param test the test result to rank (not null, unaffected)
     * @return 0 &rarr; best, 1 &rarr; second-best, and so on, or -1 if test is
     * unranked
     */
    int findRank(DecompositionTest test) {
        int result = rankedTests.indexOf(test);
        return result;
    }

    /**
     * Return the test result with the specified rank.
     *
     * @param rank 0 &rarr; best, 1 &rarr; second-best, and so on
     * @return the pre-existing instance, or null if no such test
     */
    DecompositionTest findRankedTest(int rank) {
        DecompositionTest result = null;
        if (rankedTests.size() > rank) {
            result = rankedTests.get(rank);
        }

        return result;
    }

    /**
     * Access the parameters and results for the left side.
     *
     * @return the pre-existing instance, or null if none
     */
    DecompositionTest getLeftTest() {
        return leftTest;
    }

    /**
     * Access the parameters and results for the right side.
     *
     * @return the pre-existing instance, or null if none
     */
    DecompositionTest getRightTest() {
        return rightTest;
    }

    /**
     * Access the root spatial of the loaded C-G model.
     *
     * @return the pre-existing Spatial, or null if no model loaded
     */
    Spatial getRootSpatial() {
        return rootSpatial;
    }

    /**
     * Test whether the specified test has been ranked.
     *
     * @param test (unaffected)
     * @return true if ranked, otherwise false
     */
    boolean isRanked(DecompositionTest test) {
        if (rankedTests.contains(test)) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Test whether a test is currently being ranked.
     *
     * @return true if being ranked, otherwise false
     */
    boolean isRanking() {
        if (testBeingRanked == null) {
            return false;
        } else {
            return true;
        }
    }

    /**
     * Test whether the world axes will be rendered.
     *
     * @return true if rendered, otherwise false
     */
    boolean isShowingAxes() {
        return isShowingAxes;
    }

    /**
     * Attempt to load a C-G model. The filesystem path must have been
     * previously set. If successful, {@code rootSpatial} is initialized.
     * Otherwise, {@code rootSpatial == null} and loadException is set.
     */
    void load() {
        int numComponents = filePathComponents.length;
        if (numComponents == 0) {
            throw new RuntimeException("Filesystem path not set.");
        }

        unload();
        String assetRoot = assetRoot();
        String assetPath = assetPath();

        Locators.save();
        Locators.unregisterAll();
        Locators.registerFilesystem(assetRoot);
        Locators.registerDefault();
        AssetManager assetManager = Locators.getAssetManager();
        assetManager.clearCache();
        try {
            this.rootSpatial = assetManager.loadModel(assetPath);
            this.loadException = null;
        } catch (RuntimeException exception) {
            this.rootSpatial = null;
            this.loadException = exception;
        }
        Locators.restore();

        if (rootSpatial != null) {
            VectorSet set = MyMesh.listVertexLocations(rootSpatial, null);
            if (set.numVectors() == 0) {
                this.radius = 1f;
            } else {
                this.radius = set.maxLength();
            }
        } else {
            this.radius = 1f;
        }

        // Invalidate all tests of the prior C-G model, if any.
        rankedTests.clear();
        classicMap.clear();
        v4Map.clear();

        Vhacd4Parameters v4 = new Vhacd4Parameters();
        DecompositionTest left = new DecompositionTest(v4);
        setLeftTest(left);

        VHACDParameters classic = new VHACDParameters();
        DecompositionTest right = new DecompositionTest(classic);
        setRightTest(right);
    }

    /**
     * Read the exception that occurred during the most recent load attempt.
     *
     * @return the exception message, or "" if none
     */
    String loadExceptionString() {
        String result = "";
        if (loadException != null) {
            result = loadException.toString();
        }

        return result;
    }

    /**
     * Shift one component of the filesystem path from the asset root to the
     * asset path.
     */
    void morePath() {
        unload();
        --numComponentsInRoot;
    }

    /**
     * Shift one component of the filesystem path from the asset path to the
     * asset root.
     */
    void moreRoot() {
        unload();
        ++numComponentsInRoot;
    }

    /**
     * Advance the fill mode of the left-side test to the next value.
     */
    void nextFillModeLeft() {
        if (isRanking()) {
            return;
        }
        Vhacd4Parameters copy = leftTest.copyV4();
        copy.nextFillMode();
        this.leftTest = getTest(copy);
    }

    /**
     * Advance the fill mode of the right-side test to the next value.
     */
    void nextFillModeRight() {
        if (isRanking()) {
            return;
        }
        Vhacd4Parameters copy = rightTest.copyV4();
        copy.nextFillMode();
        this.rightTest = getTest(copy);
    }

    /**
     * Check for completion of a running test.
     */
    void pollForTaskCompletion() {
        if (runningTest == null || !runningTest.hasBeenRun()) {
            return;
        }
        assert !isRanked(runningTest);

        TestScreen screen = VhacdTuner.findAppState(TestScreen.class);
        screen.closeAllPopups();

        if (rankedTests.isEmpty()) {
            /*
             * There are no ranked tests to compare with,
             * so the test that just was run is #1 for now.
             */
            rankedTests.add(runningTest);

        } else { // Begin ranking the test that was just run.
            this.minRank = 0;
            this.maxRank = rankedTests.size();
            int compareIndex = (minRank + maxRank) / 2;
            DecompositionTest compareTest = rankedTests.get(compareIndex);
            this.testBeingRanked = runningTest;

            if (testBeingRanked == rightTest) {
                assert isRanked(leftTest) || !leftTest.hasBeenRun();
                this.leftTest = compareTest;
            } else {
                assert testBeingRanked == leftTest;
                assert isRanked(rightTest) || !rightTest.hasBeenRun();
                this.rightTest = compareTest;
            }
        }

        this.runningTest = null;
    }

    /**
     * Record a preference for the first argument over the second argument.
     *
     * @param better a test preferred over {@code worse} (not null, unaffected)
     * @param worse a test to be ranked lower than {@code better} (not null,
     * unaffected)
     */
    void prefer(DecompositionTest better, DecompositionTest worse) {
        assert isRanking();
        assert maxRank >= 0 : maxRank;
        assert minRank >= 0 : minRank;
        assert minRank < maxRank;

        if (testBeingRanked == better) {
            int compareIndex = rankedTests.indexOf(worse);
            assert compareIndex >= 0 : compareIndex;
            this.maxRank = compareIndex;

        } else {
            assert testBeingRanked == worse;
            int compareIndex = rankedTests.indexOf(better);
            assert compareIndex >= 0 : compareIndex;
            this.minRank = compareIndex + 1;
        }

        if (minRank == maxRank) { // The test's rank has been determined.
            rankedTests.add(minRank, testBeingRanked);
            this.maxRank = -1;
            this.minRank = -1;
            this.testBeingRanked = null;

        } else {
            int compareIndex = (minRank + maxRank) / 2;
            DecompositionTest compareTest = rankedTests.get(compareIndex);
            if (testBeingRanked == rightTest) {
                assert isRanked(leftTest);
                this.leftTest = compareTest;
            } else {
                assert testBeingRanked == leftTest;
                assert isRanked(rightTest);
                this.rightTest = compareTest;
            }
        }
    }

    /**
     * Return the overall size of the model.
     *
     * @return the distance of the furthest vertex from the model origin, or 2.5
     * if no vertices
     */
    float radius() {
        return radius;
    }

    /**
     * Alter the model's filesystem path.
     *
     * @param path the desired filesystem path (not null, contains a "/")
     */
    void setFilePath(String path) {
        assert path != null;
        assert path.contains("/");

        this.filePathComponents = path.split("/");
        this.numComponentsInRoot = 1;
        this.loadException = null;
        unload();
        /*
         * Use heuristics to guess how many components there are
         * in the filesystem path to the asset root.
         */
        int numComponents = filePathComponents.length;
        assert numComponents > 0 : numComponents;
        for (int componentI = 0; componentI < numComponents; ++componentI) {
            String component = filePathComponents[componentI];
            switch (component) {
                case "assets":
                case "resources":
                case "Written Assets":
                    if (componentI > 1) {
                        numComponentsInRoot = componentI - 1;
                    }
                    break;
                case "Models":
                    if (componentI > 0 && componentI < numComponents) {
                        numComponentsInRoot = componentI;
                    }
                    break;
                default:
            }
        }
    }

    /**
     * Select the test for the left side.
     *
     * @param test a test with the desired parameters (not null)
     */
    void setLeftTest(DecompositionTest test) {
        if (test.isClassic()) {
            VHACDParameters parameters = test.copyClassic();
            this.leftTest = getTest(parameters);
        } else {
            Vhacd4Parameters parameters = test.copyV4();
            this.leftTest = getTest(parameters);
        }
    }

    /**
     * Select the test for the right side.
     *
     * @param test a test with the desired parameters (not null)
     */
    void setRightTest(DecompositionTest test) {
        if (test.isClassic()) {
            VHACDParameters parameters = test.copyClassic();
            this.rightTest = getTest(parameters);
        } else {
            Vhacd4Parameters parameters = test.copyV4();
            this.rightTest = getTest(parameters);
        }
    }

    /**
     * Start a thread to run the specified test.
     *
     * @param test the test to run (not null, not running)
     */
    void startTest(DecompositionTest test) {
        assert !test.hasBeenRun();
        assert !test.isRunning();
        assert runningTest == null;

        ProgressDialog progressListener = new ProgressDialog();
        Vhacd4.addProgressListener(progressListener);
        VHACD.addProgressListener(progressListener);

        TestScreen screen = VhacdTuner.findAppState(TestScreen.class);
        screen.closeAllPopups();
        screen.showConfirmDialog("", "", "", progressListener);

        this.runningTest = test;
        Thread runThread = new Thread(test);
        runThread.start();
    }

    /**
     * Abort the current ranking sequence, if any.
     */
    void stopRanking() {
        this.maxRank = -1;
        this.minRank = -1;
        this.testBeingRanked = null;
    }

    /**
     * Toggle the ACD mode of the left-side test.
     */
    void toggleAcdModeLeft() {
        if (isRanking()) {
            return;
        }
        VHACDParameters copy = leftTest.copyClassic();
        ACDMode oldMode = copy.getACDMode();
        switch (oldMode) {
            case TETRAHEDRON:
                copy.setACDMode(ACDMode.VOXEL);
                break;
            case VOXEL:
                copy.setACDMode(ACDMode.TETRAHEDRON);
                break;
            default:
                throw new IllegalStateException("oldMode = " + oldMode);
        }
        this.leftTest = getTest(copy);
    }

    /**
     * Toggle the ACD mode of the right-side test.
     */
    void toggleAcdModeRight() {
        if (isRanking()) {
            return;
        }
        VHACDParameters copy = rightTest.copyClassic();
        ACDMode oldMode = copy.getACDMode();
        switch (oldMode) {
            case TETRAHEDRON:
                copy.setACDMode(ACDMode.VOXEL);
                break;
            case VOXEL:
                copy.setACDMode(ACDMode.TETRAHEDRON);
                break;
            default:
                throw new IllegalStateException("oldMode = " + oldMode);
        }
        this.rightTest = getTest(copy);
    }

    /**
     * Toggle the async flag of the left-side test.
     */
    void toggleAsyncLeft() {
        if (isRanking()) {
            return;
        }
        Vhacd4Parameters copy = leftTest.copyV4();
        boolean oldSetting = copy.isAsync();
        copy.setAsync(!oldSetting);
        this.leftTest = getTest(copy);
    }

    /**
     * Toggle the async flag of the right-side test.
     */
    void toggleAsyncRight() {
        if (isRanking()) {
            return;
        }
        Vhacd4Parameters copy = rightTest.copyV4();
        boolean oldSetting = copy.isAsync();
        copy.setAsync(!oldSetting);
        this.rightTest = getTest(copy);
    }

    /**
     * Toggle the visibility of world axes in LoadScreen and TestScreen.
     */
    void toggleAxes() {
        this.isShowingAxes = !isShowingAxes;
    }

    /**
     * Toggle the "find best plane" option of the left-side test.
     */
    void toggleFindBestPlaneLeft() {
        if (isRanking()) {
            return;
        }
        Vhacd4Parameters copy = leftTest.copyV4();
        boolean oldSetting = copy.isFindBestPlane();
        copy.setFindBestPlane(!oldSetting);
        this.leftTest = getTest(copy);
    }

    /**
     * Toggle the "find best plane" option of the right-side test.
     */
    void toggleFindBestPlaneRight() {
        if (isRanking()) {
            return;
        }
        Vhacd4Parameters copy = rightTest.copyV4();
        boolean oldSetting = copy.isFindBestPlane();
        copy.setFindBestPlane(!oldSetting);
        this.rightTest = getTest(copy);
    }

    /**
     * Toggle the PCA setting of the left-side test.
     */
    void togglePcaLeft() {
        if (isRanking()) {
            return;
        }
        VHACDParameters copy = leftTest.copyClassic();
        boolean oldSetting = copy.getPCA();
        copy.setPCA(!oldSetting);
        this.leftTest = getTest(copy);
    }

    /**
     * Toggle the PCA setting of the right-side test.
     */
    void togglePcaRight() {
        if (isRanking()) {
            return;
        }
        VHACDParameters copy = rightTest.copyClassic();
        boolean oldSetting = copy.getPCA();
        copy.setPCA(!oldSetting);
        this.rightTest = getTest(copy);
    }

    /**
     * Toggle the shrink-wrap setting of the left-side test.
     */
    void toggleShrinkLeft() {
        if (isRanking()) {
            return;
        }
        Vhacd4Parameters copy = leftTest.copyV4();
        boolean oldSetting = copy.isShrinkWrap();
        copy.setShrinkWrap(!oldSetting);
        this.leftTest = getTest(copy);
    }

    /**
     * Toggle the shrink-wrap setting of the right-side test.
     */
    void toggleShrinkRight() {
        if (isRanking()) {
            return;
        }
        Vhacd4Parameters copy = rightTest.copyV4();
        boolean oldSetting = copy.isShrinkWrap();
        copy.setShrinkWrap(!oldSetting);
        this.rightTest = getTest(copy);
    }

    /**
     * Toggle the V-HACD version of the left-side test.
     */
    void toggleVersionLeft() {
        if (isRanking()) {
            return;
        }
        this.leftTest = getToggledVersion(leftTest);
    }

    /**
     * Toggle the V-HACD version of the right-side test.
     */
    void toggleVersionRight() {
        if (isRanking()) {
            return;
        }
        this.rightTest = getToggledVersion(rightTest);
    }

    /**
     * Unload the loaded C-G model, if any.
     */
    void unload() {
        this.rootSpatial = null;
    }
    // *************************************************************************
    // private methods

    /**
     * Access the test results for the specified classic parameters.
     *
     * @param parameters the parameters (not null)
     * @return a new or pre-existing test with the specified parameters
     */
    DecompositionTest getTest(VHACDParameters parameters) {
        DecompositionTest result = classicMap.get(parameters);
        if (result == null) {
            // Create a new test result.
            result = new DecompositionTest(parameters);
            classicMap.put(parameters, result);
        }

        return result;
    }

    /**
     * Access the test results for the specified v4 parameters.
     *
     * @param parameters the parameters (not null)
     * @return a new or pre-existing test with the specified parameters
     */
    DecompositionTest getTest(Vhacd4Parameters parameters) {
        DecompositionTest result = v4Map.get(parameters);
        if (result == null) {
            // Create a new test result.
            result = new DecompositionTest(parameters);
            v4Map.put(parameters, result);
        }

        return result;
    }
    // *************************************************************************
    // private methods

    /**
     * Construct a test with the opposite version from the specified test.
     *
     * @param test a test to use as a basis (not null, unaffected)
     * @return a new or pre-existing test
     */
    private DecompositionTest getToggledVersion(DecompositionTest test) {
        DecompositionTest result;

        if (test.isClassic()) {
            VHACDParameters classic = test.copyClassic();
            boolean debug = classic.getDebugEnabled();
            int maxVph = classic.getMaxVerticesPerHull();
            int resolution = classic.getVoxelResolution();

            Vhacd4Parameters v4 = new Vhacd4Parameters();
            v4.setDebugEnabled(debug);
            v4.setMaxVerticesPerHull(maxVph);
            v4.setVoxelResolution(resolution);

            result = getTest(v4);

        } else {
            Vhacd4Parameters v4 = test.copyV4();
            boolean debug = v4.getDebugEnabled();
            int maxVph = v4.getMaxVerticesPerHull();
            int resolution = v4.getVoxelResolution();

            VHACDParameters classic = new VHACDParameters();
            classic.setDebugEnabled(debug);
            classic.setMaxVerticesPerHull(maxVph);
            classic.setVoxelResolution(resolution);

            result = getTest(classic);
        }

        return result;
    }
}
