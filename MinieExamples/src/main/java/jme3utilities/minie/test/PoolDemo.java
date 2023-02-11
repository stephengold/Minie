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
import com.jme3.app.StatsAppState;
import com.jme3.app.state.AppState;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.PhysicsRayTestResult;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.util.CollisionShapeFactory;
import com.jme3.font.BitmapText;
import com.jme3.input.CameraInput;
import com.jme3.input.KeyInput;
import com.jme3.light.AmbientLight;
import com.jme3.light.PointLight;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Sphere;
import com.jme3.shadow.EdgeFilteringMode;
import com.jme3.shadow.PointLightShadowRenderer;
import com.jme3.system.AppSettings;
import com.jme3.texture.Texture;
import java.util.List;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MyAsset;
import jme3utilities.MyCamera;
import jme3utilities.MyString;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.noise.Generator;
import jme3utilities.math.noise.Permutation;
import jme3utilities.minie.DumpFlags;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.minie.test.common.PhysicsDemo;
import jme3utilities.minie.test.mesh.PoolHalfCushions;
import jme3utilities.minie.test.mesh.PoolTableSlice;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.InputMode;

/**
 * Test/demonstrate 3 kinds of friction.
 *
 * TODO set cue ball after a scratch, provide aiming hints
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class PoolDemo extends PhysicsDemo {
    // *************************************************************************
    // constants and loggers

    /**
     * radius of each ball (in physics-space units)
     */
    final private static float ballRadius = 11.2f;
    /**
     * physics-space Y coordinate for the top surface of the platform
     */
    final private static float platformTopY = 0f;
    /**
     * ID of the cue ball
     */
    final private static int cueBallId = 0;
    /**
     * how many numbered balls
     */
    final private static int numberedBalls = 15;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(PoolDemo.class.getName());
    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName
            = PoolDemo.class.getSimpleName();
    // *************************************************************************
    // fields

    /**
     * text displayed in the upper-left corner of the GUI node
     */
    final private static BitmapText[] statusLines = new BitmapText[1];
    /**
     * AppState to manage the PhysicsSpace
     */
    private static BulletAppState bulletAppState;
    /**
     * Mesh shared by all balls in the scene
     */
    private static Mesh ballMesh;
    /**
     * scene-graph node to parent all spatials with physics controls
     */
    final private static Node rbcNode = new Node("rbc node");
    // *************************************************************************
    // constructors

    /**
     * Instantiate the PoolDemo application.
     */
    public PoolDemo() { // made explicit to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the PoolDemo application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        String title = applicationName + " " + MyString.join(arguments);

        // Mute the chatty loggers in certain packages.
        Heart.setLoggingLevels(Level.WARNING);

        boolean loadDefaults = true;
        AppSettings settings = new AppSettings(loadDefaults);
        settings.setAudioRenderer(null);
        settings.setResizable(true);
        settings.setSamples(4); // anti-aliasing
        settings.setTitle(title); // Customize the window's title bar.

        Application application = new PoolDemo();
        application.setSettings(settings);
        application.start();
    }
    // *************************************************************************
    // PhysicsDemo methods

    /**
     * Initialize this application.
     */
    @Override
    public void acorusInit() {
        addStatusLines();
        super.acorusInit();

        rootNode.attachChild(rbcNode);
        addLighting(rootNode);
        configureCamera();
        configureDumper();
        configurePhysics();
        generateMaterials();
        generateShapes();

        // Hide the render-statistics overlay.
        stateManager.getState(StatsAppState.class).toggleStats();

        ColorRGBA bgColor = new ColorRGBA(0.02f, 0.02f, 0.02f, 1f);
        viewPort.setBackgroundColor(bgColor);

        restartScenario();
    }

    /**
     * Configure the PhysicsDumper during startup.
     */
    @Override
    public void configureDumper() {
        PhysicsDumper dumper = getDumper();

        dumper.setEnabled(DumpFlags.MatParams, true);
        //dumper.setEnabled(DumpFlags.ShadowModes, true);
        //dumper.setEnabled(DumpFlags.Transforms, true);
    }

    /**
     * Initialize the library of named materials during startup.
     */
    @Override
    public void generateMaterials() {
        super.generateMaterials();

        // Change the platform material to double-sided.
        Material platformMaterial = findMaterial("platform");
        RenderState ars = platformMaterial.getAdditionalRenderState();
        ars.setFaceCullMode(RenderState.FaceCullMode.Off);
        /*
         * Register a material for each numbered ball, based on textures
         * generated by the MakePoolBalls application.
         */
        final float shininess = 99f;
        for (int ballId = 1; ballId <= numberedBalls; ++ballId) {
            String ballName = ballName(ballId);
            String assetPath = "Textures/poolBalls/" + ballName + ".png";
            final boolean generateMips = false;
            Texture texture = MyAsset
                    .loadTexture(assetManager, assetPath, generateMips);

            Material material
                    = MyAsset.createShadedMaterial(assetManager, texture);
            material.setColor("Ambient", new ColorRGBA(1f, 1f, 1f, 1f));
            material.setColor("Diffuse", new ColorRGBA(1f, 1f, 1f, 1f));
            material.setFloat("Shininess", shininess);
            material.setColor("Specular", new ColorRGBA(1f, 1f, 1f, 1f));
            material.setBoolean("UseMaterialColors", true);

            registerMaterial(ballName, material);
        }

        Material material
                = MyAsset.createShinyMaterial(assetManager, ColorRGBA.White);
        material.setFloat("Shininess", shininess);
        registerMaterial(ballName(0), material);
    }

    /**
     * Initialize the library of named collision shapes during startup.
     */
    @Override
    public void generateShapes() {
        CollisionShape ballShape = new SphereCollisionShape(ballRadius);
        registerShape("ball", ballShape);

        final int zSamples = 20;
        final int radialSamples = 2 * zSamples;
        ballMesh = new Sphere(zSamples, radialSamples, ballRadius);
    }

    /**
     * Access the active BulletAppState.
     *
     * @return the pre-existing instance (not null)
     */
    @Override
    protected BulletAppState getBulletAppState() {
        assert bulletAppState != null;
        return bulletAppState;
    }

    /**
     * Determine the length of debug axis arrows (when they're visible).
     *
     * @return the desired length (in physics-space units, &ge;0)
     */
    @Override
    protected float maxArrowLength() {
        return 20f;
    }

    /**
     * Add application-specific hotkey bindings (and override existing ones, if
     * necessary).
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        dim.bind(asCollectGarbage, KeyInput.KEY_G);

        dim.bind(asDumpSpace, KeyInput.KEY_O);
        dim.bind(asDumpViewport, KeyInput.KEY_P);

        dim.bind("restart", KeyInput.KEY_NUMPAD5);

        dim.bindSignal(CameraInput.FLYCAM_LOWER, KeyInput.KEY_DOWN);
        dim.bindSignal(CameraInput.FLYCAM_RISE, KeyInput.KEY_UP);
        dim.bindSignal("orbitLeft", KeyInput.KEY_LEFT);
        dim.bindSignal("orbitRight", KeyInput.KEY_RIGHT);

        dim.bind("strike", "RMB");
        dim.bind("strike", KeyInput.KEY_RETURN, KeyInput.KEY_SPACE);

        dim.bind(asToggleAabbs, KeyInput.KEY_APOSTROPHE);
        dim.bind(asToggleDebug, KeyInput.KEY_SLASH);
        dim.bind(asToggleHelp, KeyInput.KEY_H);
        dim.bind(asTogglePause, KeyInput.KEY_PAUSE, KeyInput.KEY_PERIOD);
        dim.bind(asTogglePcoAxes, KeyInput.KEY_SEMICOLON);
        dim.bind("toggle scene", KeyInput.KEY_M);
        dim.bind(asToggleVArrows, KeyInput.KEY_K);
        dim.bind(asToggleWArrows, KeyInput.KEY_N);
    }

    /**
     * Process an action that wasn't handled by the active input mode.
     *
     * @param actionString textual description of the action (not null)
     * @param ongoing true if the action is ongoing, otherwise false
     * @param tpf time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void onAction(String actionString, boolean ongoing, float tpf) {
        if (ongoing) {
            switch (actionString) {
                case "restart":
                    restartScenario();
                    return;

                case "strike":
                    strikeBall();
                    return;

                case "toggle scene":
                    toggleScene();
                    return;

                default:
            }
        }
        super.onAction(actionString, ongoing, tpf);
    }

    /**
     * Update the GUI layout and proposed settings after a resize.
     *
     * @param newWidth the new width of the framebuffer (in pixels, &gt;0)
     * @param newHeight the new height of the framebuffer (in pixels, &gt;0)
     */
    @Override
    public void onViewPortResize(int newWidth, int newHeight) {
        for (int lineIndex = 0; lineIndex < statusLines.length; ++lineIndex) {
            float y = newHeight - 20f * lineIndex;
            statusLines[lineIndex].setLocalTranslation(0f, y, 0f);
        }
        super.onViewPortResize(newWidth, newHeight);
    }

    /**
     * Callback invoked once per frame.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void simpleUpdate(float tpf) {
        super.simpleUpdate(tpf);
        updateStatusLines();
    }
    // *************************************************************************
    // private methods

    /**
     * Add lighting and shadows to the specified scene.
     *
     * @param rootSpatial which scene (not null)
     */
    private void addLighting(Spatial rootSpatial) {
        ColorRGBA ambientColor = new ColorRGBA(0.02f, 0.02f, 0.02f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        rootSpatial.addLight(ambient);
        ambient.setName("ambient");

        Vector3f lightPosition = new Vector3f(0f, platformTopY + 600f, 0f);
        ColorRGBA pointColor = new ColorRGBA(90f, 90f, 90f, 1f);
        PointLight lamp = new PointLight(lightPosition, pointColor);
        rootSpatial.addLight(lamp);
        lamp.setName("lamp");
        lamp.setRadius(3_000f);

        viewPort.clearProcessors();
        int mapSize = 2_048; // in pixels
        PointLightShadowRenderer plsr
                = new PointLightShadowRenderer(assetManager, mapSize);
        viewPort.addProcessor(plsr);

        plsr.setEdgeFilteringMode(EdgeFilteringMode.PCFPOISSON);
        plsr.setEdgesThickness(7);
        plsr.setLight(lamp);
        plsr.setShadowIntensity(0.7f);
    }

    /**
     * Add status lines to the GUI.
     */
    private void addStatusLines() {
        for (int lineIndex = 0; lineIndex < statusLines.length; ++lineIndex) {
            statusLines[lineIndex] = new BitmapText(guiFont);
            guiNode.attachChild(statusLines[lineIndex]);
        }
    }

    /**
     * Generate the name of the identified pool ball.
     *
     * @param ballId which ball (0=cue ball, 8=eight ball, &ge;0,
     * &le;numberedBalls)
     * @return the name
     */
    private static String ballName(int ballId) {
        String result;
        if (ballId == cueBallId) {
            result = "cue";
        } else {
            result = Integer.toString(ballId);
        }
        result += "Ball";

        return result;
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        float near = 1f;
        float far = 10_000f;
        MyCamera.setNearFar(cam, near, far);

        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(200f);

        cam.setLocation(new Vector3f(-44f, platformTopY + 189f, 425f));
        cam.setRotation(new Quaternion(0.003f, 0.971321f, -0.2374f, 0.0123f));

        AppState orbitState
                = new CameraOrbitAppState(cam, "orbitLeft", "orbitRight");
        stateManager.attach(orbitState);
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);

        PhysicsSpace physicsSpace = getPhysicsSpace();
        physicsSpace.setAccuracy(0.001f);
        physicsSpace.setGravity(new Vector3f(0f, -3_800f, 0f));
        physicsSpace.setMaxSubSteps(25);
    }

    /**
     * Restart the scenario.
     */
    private void restartScenario() {
        List<Spatial> children = rbcNode.getChildren();
        for (Spatial child : children) {
            RigidBodyControl rbc = child.getControl(RigidBodyControl.class);
            rbc.setPhysicsSpace(null);
        }
        rbcNode.detachAllChildren();

        PhysicsSpace physicsSpace = getPhysicsSpace();
        assert physicsSpace.isEmpty();

        setUpTable();
        Vector3f location = new Vector3f(0f, platformTopY + ballRadius, 99f);
        setUpBall(location, cueBallId);
        setUpBallWedge();
    }

    /**
     * Set up a single billiard ball.
     *
     * @param location the desired location (in physics-space coordinates, not
     * null, unaffected)
     * @param ballId which ball (0=cue ball, 8=eight ball, &ge;0,
     * &le;numberedBalls)
     */
    private void setUpBall(Vector3f location, int ballId) {
        String ballName = ballName(ballId);
        Geometry geometry = new Geometry(ballName, ballMesh);
        rbcNode.attachChild(geometry);

        geometry.setShadowMode(RenderQueue.ShadowMode.Cast);

        Material material = findMaterial(ballName);
        assert material != null;
        geometry.setMaterial(material);

        CollisionShape shape = findShape("ball");
        final float mass = 1f;
        RigidBodyControl rbc = new RigidBodyControl(shape, mass);
        geometry.addControl(rbc);

        rbc.setApplicationData(ballId);
        rbc.setCcdMotionThreshold(ballRadius);
        rbc.setCcdSweptSphereRadius(ballRadius);
        rbc.setFriction(0.03f);
        rbc.setPhysicsLocation(location);

        Generator random = getGenerator();
        Quaternion rotation = random.nextQuaternion(null);
        rbc.setPhysicsRotation(rotation);

        PhysicsSpace space = getPhysicsSpace();
        rbc.setPhysicsSpace(space);

        rbc.setRestitution(0.9f);
    }

    /**
     * Set up 15 numbered balls in a wedge, as if for a game of Pool.
     */
    private void setUpBallWedge() {
        final float xSpacing = 2.09f * ballRadius; // center-to-center
        float zSpacing = xSpacing / FastMath.sqrt(1.5f); // center-to-center
        final float z0 = -60f;
        final int numRows = 5;

        Random random = getGenerator();
        Permutation permutation;
        while (true) {
            long seed = random.nextLong();
            permutation = new Permutation(numberedBalls, seed);

            boolean eightBallInCenter = (1 + permutation.permute(4) == 8);
            boolean leftBackCornerIsStripe = (1 + permutation.permute(10) > 8);
            boolean rightBackCornerIsStripe = (1 + permutation.permute(14) > 8);
            boolean legalRack = eightBallInCenter
                    && (leftBackCornerIsStripe ^ rightBackCornerIsStripe);
            if (legalRack) {
                break;
            }
        }

        int ballIndex = 0;
        Vector3f location = new Vector3f(0f, platformTopY + ballRadius, z0);
        for (int rowIndex = 0; rowIndex < numRows; ++rowIndex) {
            int numBallsInRow = rowIndex + 1;
            location.x = -(numBallsInRow - 1) * xSpacing / 2f;
            for (int j = 0; j < numBallsInRow; ++j) {
                int ballId = 1 + permutation.permute(ballIndex);
                setUpBall(location, ballId);
                ++ballIndex;
                location.x += xSpacing;
            }
            location.z -= zSpacing;
        }
    }

    /**
     * Set up a pool table.
     */
    private void setUpTable() {
        final float legLength = 190f;
        final float pocketRadius = 22f;
        final int numSliceArcEdges = 4;
        Mesh tableSlice
                = new PoolTableSlice(legLength, pocketRadius, numSliceArcEdges);

        Material platformMaterial = findMaterial("platform");
        assert platformMaterial != null;

        Node platformNode = new Node("platform node");
        rbcNode.attachChild(platformNode);

        Transform tmpTransform = new Transform();
        Quaternion rotation = tmpTransform.getRotation(); // alias

        for (int offsetIndex = 0; offsetIndex < 2; ++offsetIndex) {
            float zOffset = (2 * offsetIndex - 1) * legLength;
            tmpTransform.setTranslation(0f, 0f, zOffset);

            for (int yRotIndex = 0; yRotIndex < 4; ++yRotIndex) {
                int sliceIndex = yRotIndex + 4 * offsetIndex;
                String name = "tableSlice." + sliceIndex;
                float yAngle = yRotIndex * FastMath.HALF_PI;
                rotation.fromAngles(0f, yAngle, 0f);
                Geometry geometry = new Geometry(name, tableSlice);

                platformNode.attachChild(geometry);
                geometry.setLocalTransform(tmpTransform);
                geometry.setMaterial(platformMaterial);
                geometry.setShadowMode(RenderQueue.ShadowMode.Receive);
            }
        }

        int numCushionArcEdges = 20;
        final float bumperHeight = 20f;
        Mesh halfCushions = new PoolHalfCushions(legLength, pocketRadius,
                numCushionArcEdges, bumperHeight);

        for (int yRotIndex = 0; yRotIndex < 2; ++yRotIndex) {
            String name = "halfCushions." + yRotIndex;
            float yAngle = (0.5f + yRotIndex) * FastMath.PI;
            rotation.fromAngles(0f, yAngle, 0f);
            Geometry geometry = new Geometry(name, halfCushions);

            platformNode.attachChild(geometry);
            geometry.setLocalRotation(rotation);
            geometry.setMaterial(platformMaterial);
            geometry.setShadowMode(RenderQueue.ShadowMode.Receive);
        }

        CollisionShape shape
                = CollisionShapeFactory.createMergedMeshShape(platformNode);
        RigidBodyControl rbc
                = new RigidBodyControl(shape, PhysicsBody.massForStatic);
        platformNode.addControl(rbc);
        rbc.setDebugNumSides(2);
        rbc.setFriction(5f);

        PhysicsSpace space = getPhysicsSpace();
        rbc.setPhysicsSpace(space);

        rbc.setRestitution(0.6f);
        rbc.setRollingFriction(10f);
        rbc.setSpinningFriction(50f);
    }

    /**
     * Strike a ball, determining the location and direction from the camera and
     * mouse cursor.
     */
    private void strikeBall() {
        Vector2f screenXY = inputManager.getCursorPosition();
        Vector3f nearLocation
                = cam.getWorldCoordinates(screenXY, MyCamera.nearZ);
        Vector3f farLocation = cam.getWorldCoordinates(screenXY, MyCamera.farZ);
        PhysicsSpace space = getPhysicsSpace();
        List<PhysicsRayTestResult> hits
                = space.rayTest(nearLocation, farLocation);

        for (PhysicsRayTestResult hit : hits) {
            PhysicsCollisionObject pco = hit.getCollisionObject();
            Object appObject = pco.getApplicationData();
            if (appObject instanceof Integer && cueBallId == (int) appObject) {
                Vector3f impulseVector = farLocation.subtract(nearLocation);
                float impulseMagnitude = 1_000f; // TODO player-controlled
                float factor = impulseMagnitude / impulseVector.length();
                impulseVector.multLocal(factor);

                float hitFraction = hit.getHitFraction();
                Vector3f location = MyVector3f
                        .lerp(hitFraction, nearLocation, farLocation, null);
                Vector3f ballCenter = pco.getPhysicsLocation(null);
                location.subtractLocal(ballCenter);

                PhysicsRigidBody body = (PhysicsRigidBody) pco;
                body.applyImpulse(impulseVector, location);

                if (isPaused()) {
                    togglePause();
                }
                return;
            }
        }
    }

    /**
     * Toggle mesh rendering on/off.
     */
    private static void toggleScene() {
        Spatial.CullHint hint = rbcNode.getLocalCullHint();
        if (hint == Spatial.CullHint.Inherit
                || hint == Spatial.CullHint.Never) {
            hint = Spatial.CullHint.Always;
        } else if (hint == Spatial.CullHint.Always) {
            hint = Spatial.CullHint.Never;
        }
        rbcNode.setCullHint(hint);
    }

    /**
     * Update the status lines in the GUI.
     */
    private void updateStatusLines() {
        String message = "View: ";

        Spatial.CullHint cull = rbcNode.getLocalCullHint();
        message += (cull == Spatial.CullHint.Always) ? "NOscene" : "Scene";

        boolean debug = bulletAppState.isDebugEnabled();
        if (debug) {
            message += "+" + describePhysicsDebugOptions();
        }
        message += isPaused() ? "  PAUSED" : "";
        statusLines[0].setText(message);
    }
}
