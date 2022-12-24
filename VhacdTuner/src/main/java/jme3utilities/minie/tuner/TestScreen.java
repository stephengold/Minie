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
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.debug.BulletDebugAppState;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import de.lessvoid.nifty.controls.Button;
import de.lessvoid.nifty.elements.Element;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.InitialState;
import jme3utilities.MyAsset;
import jme3utilities.MyCamera;
import jme3utilities.MySpatial;
import jme3utilities.nifty.GuiScreenController;
import jme3utilities.ui.InputMode;

/**
 * The screen controller for the "test" screen of VhacdTuner.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class TestScreen extends GuiScreenController {
    // *************************************************************************
    // constants and loggers

    /**
     * width of the GUI (in pixels)
     */
    final private static float guiWidth = 230f;
    /**
     * message logger for this class
     */
    final static Logger logger = Logger.getLogger(TestScreen.class.getName());
    // *************************************************************************
    // fields

    /**
     * element of GUI button to proceed to the next Screen
     */
    private Element nextElement;
    /**
     * parent of the C-G model on the left side of the screen
     */
    final private Node leftScene = new Node("left scene");
    /**
     * parent of the C-G model on the right side of the screen
     */
    final private Node rightScene = new Node("right scene");
    /**
     * visualize the mesh of the C-G model on the left side of the screen
     */
    private Spatial leftMeshCgm;
    /**
     * visualize the mesh of the C-G model on the right side of the screen
     */
    private Spatial rightMeshCgm;
    /**
     * pre viewport for the left side of the screen
     */
    private ViewPort leftView;
    /**
     * pre viewport for the right side of the screen
     */
    private ViewPort rightView;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an uninitialized, disabled screen that will not be enabled
     * during initialization.
     */
    TestScreen() {
        super("test", "Interface/Nifty/screens/tuner/test.xml",
                InitialState.Disabled);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Test whether mesh rendering is disabled.
     *
     * @return true if rendering is disabled, otherwise false
     */
    boolean areMeshesHidden() {
        Spatial.CullHint cull = leftMeshCgm.getLocalCullHint();
        if (cull == Spatial.CullHint.Always) {
            assert rightMeshCgm.getLocalCullHint() == Spatial.CullHint.Always;
            return true;
        } else {
            assert rightMeshCgm.getLocalCullHint() != Spatial.CullHint.Always;
            return false;
        }
    }

    /**
     * Toggle mesh rendering off/on.
     */
    void toggleMesh() {
        boolean hidden = areMeshesHidden();
        if (hidden) {
            leftMeshCgm.setCullHint(Spatial.CullHint.Never);
            rightMeshCgm.setCullHint(Spatial.CullHint.Never);
            assert !areMeshesHidden();
        } else {
            leftMeshCgm.setCullHint(Spatial.CullHint.Always);
            rightMeshCgm.setCullHint(Spatial.CullHint.Always);
            assert areMeshesHidden();
        }
    }

    /**
     * Toggle physics debug visualization off/on.
     */
    void togglePhysicsDebug() {
        boolean enabled = VhacdTuner.isDebugEnabled();
        if (enabled) {
            VhacdTuner.clearPhysicsDebug();
        } else {
            VhacdTuner.setPhysicsDebug(leftView, rightView);
        }
    }

    /**
     * Toggle world axes visualization off/on.
     */
    void toggleShowingAxes() {
        Model model = VhacdTuner.getModel();
        model.toggleAxes();

        VhacdTuner.updateAxes(leftScene);
        VhacdTuner.updateAxes(rightScene);
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

        InputMode inputMode = InputMode.findMode("test");
        assert inputMode != null;
        setListener(inputMode);
        inputMode.influence(this);
    }

    /**
     * A callback from Nifty, invoked each time this screen shuts down.
     */
    @Override
    public void onEndScreen() {
        super.onEndScreen();

        VhacdTuner.clearPhysicsDebug();
        boolean success = renderManager.removeMainView(leftView);
        assert success;

        success = renderManager.removeMainView(rightView);
        assert success;
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

        // Disable the default viewport.
        viewPort.setEnabled(false);

        // Clear both scenes.
        MySpatial.removeAllControls(leftScene);
        MySpatial.removeAllControls(rightScene);

        List<Spatial> children = leftScene.getChildren();
        for (Spatial child : children) {
            child.removeFromParent();
        }
        children = rightScene.getChildren();
        for (Spatial child : children) {
            child.removeFromParent();
        }

        // Create wireframe copies of the C-G model.
        Model model = VhacdTuner.getModel();
        this.leftMeshCgm = model.getRootSpatial();
        leftMeshCgm = Heart.deepCopy(leftMeshCgm);
        List<Geometry> geometries = MySpatial.listGeometries(leftMeshCgm);
        Material wireframe = MyAsset.createWireframeMaterial(
                assetManager, ColorRGBA.Yellow);
        wireframe.getAdditionalRenderState()
                .setWireframe(true); // TODO Why is this necessary?
        wireframe.setName("wireframe");
        for (Geometry geometry : geometries) {
            geometry.setMaterial(wireframe);
        }
        this.rightMeshCgm = Heart.deepCopy(leftMeshCgm);

        ColorRGBA skyColor = new ColorRGBA(0.1f, 0.2f, 0.4f, 1f);

        float screenWidth = cam.getWidth(); // pixels
        float viewPortWidth = (screenWidth - guiWidth) / (2f * screenWidth);

        Camera leftCamera = createSideCamera("left", 0f, viewPortWidth);
        this.leftView = renderManager.createMainView("left", leftCamera);
        leftView.attachScene(leftScene);
        leftView.setBackgroundColor(skyColor);
        leftView.setClearFlags(true, true, true);

        Camera rightCamera = createSideCamera("right", 1f - viewPortWidth, 1f);
        this.rightView = renderManager.createMainView("right", rightCamera);
        rightView.attachScene(rightScene);
        rightView.setBackgroundColor(skyColor);
        rightView.setClearFlags(true, true, true);

        leftScene.attachChild(leftMeshCgm);
        rightScene.attachChild(rightMeshCgm);

        VhacdTuner.updateAxes(leftScene);
        VhacdTuner.updateAxes(rightScene);

        VhacdTuner.setPhysicsDebug(leftView, rightView);
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

        Model model = VhacdTuner.getModel();
        model.pollForTaskCompletion();

        updateCameras();
        updateFeedback();
        updateViewButtons();

        DecompositionTest leftTest = model.getLeftTest();
        DecompositionTest rightTest = model.getRightTest();

        PhysicsSpace leftSpace = VhacdTuner.getLeftSpace();
        updatePhysics(leftTest, leftSpace);
        updateSideButtons("Left", leftTest, rightTest);
        updateSideLabels("Left", leftTest);

        PhysicsSpace rightSpace = VhacdTuner.getRightSpace();
        updatePhysics(rightTest, rightSpace);
        updateSideButtons("Right", rightTest, leftTest);
        updateSideLabels("Right", rightTest);

        // Update the "stop ranking" button.
        String stop = model.isRanking() ? "Stop ranking" : "";
        setButtonText("stopRanking", stop);
    }
    // *************************************************************************
    // private methods

    /**
     * Instantiate a new camera for a partial-width view port.
     *
     * @param name the name for the camera
     * @param leftEdge the left edge of the view port (&ge;0, &lt;rightEdge)
     * @param rightEdge the right edge of the view port (&gt;leftEdge, &le;1)
     * @return a new instance with perspective projection
     */
    private Camera createSideCamera(
            String name, float leftEdge, float rightEdge) {
        Camera result = cam.clone();
        result.setName(name);

        float bottomEdge = 0f;
        float topEdge = 1f;
        result.setViewPort(leftEdge, rightEdge, bottomEdge, topEdge);

        // The frustum should have the same aspect ratio as the view port.
        float yDegrees = MyCamera.yDegrees(result);
        float aspectRatio = MyCamera.viewAspectRatio(result);
        float near = result.getFrustumNear();
        float far = result.getFrustumFar();
        result.setFrustumPerspective(yDegrees, aspectRatio, near, far);

        return result;
    }

    /**
     * Move both side cameras to mimic the default camera (which is the one
     * controlled by the mouse and keyboard).
     */
    private void synchronizeCameras() {
        Quaternion orientation = cam.getRotation(); // alias
        Vector3f location = cam.getLocation(); // alias

        Camera leftCamera = leftView.getCamera();
        leftCamera.setLocation(location);
        leftCamera.setRotation(orientation);

        Camera rightCamera = rightView.getCamera();
        rightCamera.setLocation(location);
        rightCamera.setRotation(orientation);
    }

    private void updateCameras() {
        float screenWidth = cam.getWidth(); // pixels
        float viewPortWidth = (screenWidth - guiWidth) / (2f * screenWidth);

        Camera leftCamera = leftView.getCamera();
        leftCamera.setViewPortRight(viewPortWidth);

        Camera rightCamera = rightView.getCamera();
        rightCamera.setViewPortLeft(1f - viewPortWidth);

        synchronizeCameras();
    }

    /**
     * Update the feedback line and the "Next>" button.
     */
    private void updateFeedback() {
        Model model = VhacdTuner.getModel();
        boolean isRanking = model.isRanking();
        DecompositionTest leftTest = model.getLeftTest();
        boolean leftRun = leftTest.hasBeenRun();
        DecompositionTest rightTest = model.getRightTest();
        boolean rightRun = rightTest.hasBeenRun();

        String feedback = "";
        if (model.countRankedTests() == 0) {
            feedback = "You haven't run any tests yet.";
        } else if (isRanking && leftRun && !model.isRanked(leftTest)) {
            feedback = "You haven't ranked the left test yet.";
        } else if (isRanking && rightRun && !model.isRanked(rightTest)) {
            feedback = "You haven't ranked the right test yet.";
        }

        setStatusText("feedback", feedback);
        if (feedback.isEmpty()) {
            nextElement.show();
        } else {
            nextElement.hide();
        }
    }

    /**
     * If the specified test has generated a shape, update the specified space
     * to contain a single rigid body with that shape. Otherwise, empty the
     * space.
     *
     * @param test (not null)
     * @param space (not null, may be modified)
     */
    private static void updatePhysics(
            DecompositionTest test, PhysicsSpace space) {
        boolean haveShape = test.hasBeenRun();
        boolean isEmpty = space.isEmpty();
        if (haveShape) {
            CollisionShape shape = test.getShape();

            Collection<PhysicsRigidBody> bodies = space.getRigidBodyList();
            PhysicsRigidBody body = Heart.first(bodies);
            if (body == null || body.getCollisionShape() != shape) {
                space.destroy();

                body = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);
                body.setApplicationData(test);
                body.setDebugMaterial(BulletDebugAppState.enableChildColoring);
                space.addCollisionObject(body);
            }

        } else if (!haveShape && !isEmpty) {
            space.destroy();
        }
    }

    /**
     * Update the left/right buttons.
     *
     * @param side the side being updated: "Left" or "Right"
     * @param test parameters and results for the side being updated (not null)
     * @param otherTest parameters and results for the other side (not null)
     */
    private void updateSideButtons(
            String side, DecompositionTest test, DecompositionTest otherTest) {
        Model model = VhacdTuner.getModel();

        String prefer = "";
        boolean isLeft = side.equals("Left");
        if (model.isRanking()) {
            prefer = isLeft ? "<-- Prefer" : "Prefer -->";
        } else if (!test.hasBeenRun()) {
            boolean otherRanked = model.isRanked(otherTest);
            boolean otherRun = otherTest.hasBeenRun();
            if (otherRanked || !otherRun) {
                prefer = isLeft ? "<-- Run" : "Run -->";
            }
        }
        setButtonText("prefer" + side, prefer);

        Map<String, Object> map = test.toMap();

        String alpha = "";
        if (map.containsKey("alpha")) {
            alpha = map.get("alpha").toString();
        }
        setButtonText("alpha" + side, alpha);

        String async = "";
        if (map.containsKey("async")) {
            boolean setting = (Boolean) map.get("async");
            async = setting ? "yes" : "no";
        }
        setButtonText("async" + side, async);

        String beta = "";
        if (map.containsKey("beta")) {
            beta = map.get("beta").toString();
        }
        setButtonText("beta" + side, beta);

        String fillMode = "";
        if (map.containsKey("fillMode")) {
            fillMode = map.get("fillMode").toString();
        }
        setButtonText("fillMode" + side, fillMode);

        String findBestPlane = "";
        if (map.containsKey("findBest")) {
            boolean fbp = (Boolean) map.get("findBest");
            findBestPlane = fbp ? "yes" : "no";
        }
        setButtonText("findBestPlane" + side, findBestPlane);

        String hullDS = "";
        if (map.containsKey("hullDS")) {
            hullDS = map.get("hullDS").toString();
        }
        setButtonText("hullDS" + side, hullDS);

        String maxConcavity = "";
        if (map.containsKey("maxConcavity")) {
            maxConcavity = map.get("maxConcavity").toString();
        }
        setButtonText("maxConcavity" + side, maxConcavity);

        String maxHulls = "";
        if (map.containsKey("maxHulls")) {
            maxHulls = map.get("maxHulls").toString();
        }
        setButtonText("maxHulls" + side, maxHulls);

        String maxRecursion = "";
        if (map.containsKey("maxRecursion")) {
            maxRecursion = map.get("maxRecursion").toString();
        }
        setButtonText("maxRecursion" + side, maxRecursion);

        String maxVerticesPH = map.get("maxVerticesPH").toString();
        setButtonText("maxVerticesPH" + side, maxVerticesPH);

        String minEdgeLength = "";
        if (map.containsKey("minEdge")) {
            minEdgeLength = map.get("minEdge").toString();
        }
        setButtonText("minEdgeLength" + side, minEdgeLength);

        String minVolumePH = "";
        if (map.containsKey("minVolumePH")) {
            minVolumePH = map.get("minVolumePH").toString();
        }
        setButtonText("minVolumePH" + side, minVolumePH);

        String pca = "";
        if (map.containsKey("PCA")) {
            boolean setting = (Boolean) map.get("PCA");
            pca = setting ? "yes" : "no";
        }
        setButtonText("pca" + side, pca);

        String planeDS = "";
        if (map.containsKey("planeDS")) {
            planeDS = map.get("planeDS").toString();
        }
        setButtonText("planeDS" + side, planeDS);

        String resolution = map.get("resolution").toString();
        setButtonText("resolution" + side, resolution);

        String shrinkWrap = "";
        if (map.containsKey("shrink")) {
            boolean shrink = (Boolean) map.get("shrink");
            shrinkWrap = shrink ? "yes" : "no";
        }
        setButtonText("shrink" + side, shrinkWrap);

        boolean isClassic = (Boolean) map.get("classic");
        String version = isClassic ? "classic" : "v4";
        setButtonText("version" + side, version);

        String volumeError = "";
        if (map.containsKey("volumeErr")) {
            volumeError = map.get("volumeErr").toString() + "%";
        }
        setButtonText("volumeError" + side, volumeError);
    }

    /**
     * Update the left/right status labels.
     *
     * @param sideName "Left" or "Right"
     * @param test the parameters and results for that side (not null,
     * unaffected)
     */
    private void updateSideLabels(String sideName, DecompositionTest test) {
        String hulls = "?";
        String rank = "?";
        String seconds = "?";
        String vertices = "?";

        if (test.hasBeenRun()) {
            int h = test.getShape().countChildren();
            hulls = Integer.toString(h);

            float latency = test.latency();
            seconds = String.format("%.2f", latency);

            int numVertices = test.countVertices();
            vertices = Integer.toString(numVertices);

            Model model = VhacdTuner.getModel();
            int r = model.findRank(test);
            if (r >= 0) {
                int cardinal = r + 1;
                rank = "#" + cardinal;
            }
        }

        setStatusText("hulls" + sideName, hulls);
        setStatusText("rank" + sideName, rank);
        setStatusText("seconds" + sideName, seconds);
        setStatusText("vertices" + sideName, vertices);
    }

    /**
     * Update the buttons that toggle view elements.
     */
    private void updateViewButtons() {
        String debugButton = "";
        Model model = VhacdTuner.getModel();
        boolean leftRun = model.getLeftTest().hasBeenRun();
        boolean rightRun = model.getRightTest().hasBeenRun();
        if (leftRun || rightRun) {
            boolean isDebug = VhacdTuner.isDebugEnabled();
            debugButton = isDebug ? "Hide hulls" : "Show hulls";
        }
        setButtonText("debug", debugButton);

        String meshButton = areMeshesHidden() ? "Show mesh" : "Hide mesh";
        setButtonText("mesh", meshButton);

        String axesText = model.isShowingAxes() ? "Hide axes" : "Show axes";
        setButtonText("axes", axesText);
    }
}
