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
import com.jme3.bounding.BoundingBox;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.SoftPhysicsAppState;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.PhysicsRayTestResult;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.MultiSphere;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.debug.BulletDebugAppState;
import com.jme3.bullet.debug.DebugInitListener;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.font.Rectangle;
import com.jme3.input.CameraInput;
import com.jme3.input.KeyInput;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Limits;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.shadow.DirectionalLightShadowRenderer;
import com.jme3.shadow.EdgeFilteringMode;
import com.jme3.system.AppSettings;
import com.jme3.util.BufferUtils;
import java.util.Collection;
import java.util.List;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MeshNormals;
import jme3utilities.MyAsset;
import jme3utilities.MyCamera;
import jme3utilities.MyString;
import jme3utilities.math.MyMath;
import jme3utilities.math.noise.Generator;
import jme3utilities.minie.test.common.PhysicsDemo;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.InputMode;

/**
 * Test/demonstrate dynamic physics by launching missiles (small/dynamic/rigid
 * bodies) at various targets.
 * <p>
 * Collision objects are rendered entirely by debug visualization.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TargetDemo
        extends PhysicsDemo
        implements DebugInitListener {
    // *************************************************************************
    // constants and loggers

    /**
     * Y coordinate for the top surface of the platform (in physics-space
     * coordinates)
     */
    final private static float platformTopY = 0f;
    /**
     * number of colors/materials for targets
     */
    final private static int numTargetColors = 4;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(TargetDemo.class.getName());
    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName
            = TargetDemo.class.getSimpleName();
    // *************************************************************************
    // fields

    /**
     * AppState to manage the PhysicsSpace
     */
    private static BulletAppState bulletAppState;
    /**
     * selected rigid body, or null if none
     */
    private static PhysicsRigidBody selectedBody = null;
    /**
     * AppState to manage the status overlay
     */
    private static TargetDemoStatus status;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the TargetDemo application.
     */
    public TargetDemo() { // explicit to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Count how many rigid bodies are active.
     *
     * @return the count (&ge;0)
     */
    int countActive() {
        int result = 0;
        Collection<PhysicsRigidBody> rigidBodies
                = getPhysicsSpace().getRigidBodyList();
        for (PhysicsRigidBody rigidBody : rigidBodies) {
            if (rigidBody.isActive()) {
                ++result;
            }
        }

        return result;
    }

    /**
     * Main entry point for the TargetDemo application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        String title = applicationName + " " + MyString.join(arguments);

        // Mute the chatty loggers in certain packages.
        Heart.setLoggingLevels(Level.WARNING);

        // Enable direct-memory tracking.
        BufferUtils.setTrackDirectMemoryEnabled(true);

        boolean loadDefaults = true;
        AppSettings settings = new AppSettings(loadDefaults);
        settings.setAudioRenderer(null);
        settings.setResizable(true);
        settings.setSamples(4); // anti-aliasing
        settings.setTitle(title); // Customize the window's title bar.

        Application application = new TargetDemo();
        application.setSettings(settings);
        application.start();
    }

    /**
     * Restart the current scenario.
     */
    void restartScenario() {
        selectBody(null);

        PhysicsSpace physicsSpace = getPhysicsSpace();
        physicsSpace.destroy();
        assert physicsSpace.isEmpty();

        String platformName = status.platformType();
        addPlatform(platformName, platformTopY);

        setUpScenario();
        setDebugMaterialsAll();
    }

    /**
     * Update the debug materials of all collision objects.
     */
    void setDebugMaterialsAll() {
        PhysicsSpace physicsSpace = getPhysicsSpace();
        for (PhysicsCollisionObject pco : physicsSpace.getPcoList()) {
            setDebugMaterial(pco);
        }
    }

    /**
     * Update the ShadowMode of the debug scene.
     */
    static void setDebugShadowMode() {
        RenderQueue.ShadowMode mode;
        if (status.isWireframe()) {
            mode = RenderQueue.ShadowMode.Off;
        } else {
            mode = RenderQueue.ShadowMode.CastAndReceive;
        }
        bulletAppState.setDebugShadowMode(mode);
    }
    // *************************************************************************
    // PhysicsDemo methods

    /**
     * Initialize this application.
     */
    @Override
    public void acorusInit() {
        status = new TargetDemoStatus();
        boolean success = stateManager.attach(status);
        assert success;

        super.acorusInit();

        configureCamera();
        configureDumper();
        generateMaterials();
        configurePhysics();
        generateShapes();

        // Hide the render-statistics overlay.
        stateManager.getState(StatsAppState.class).toggleStats();

        ColorRGBA skyColor = new ColorRGBA(0.1f, 0.2f, 0.4f, 1f);
        viewPort.setBackgroundColor(skyColor);

        String platformName = status.platformType();
        addPlatform(platformName, platformTopY);

        Integer maxDegree = renderer.getLimits().get(Limits.TextureAnisotropy);
        int degree = (maxDegree == null) ? 1 : Math.min(8, maxDegree);
        renderer.setDefaultAnisotropicFilter(degree);

        setUpScenario();
    }

    /**
     * Calculate screen bounds for the detailed help node.
     *
     * @param viewPortWidth (in pixels, &gt;0)
     * @param viewPortHeight (in pixels, &gt;0)
     * @return a new instance
     */
    @Override
    public Rectangle detailedHelpBounds(int viewPortWidth, int viewPortHeight) {
        // Position help nodes below the status.
        float margin = 10f; // in pixels
        float leftX = margin;
        float topY = viewPortHeight - 220f - margin;
        float width = viewPortWidth - leftX - margin;
        float height = topY - margin;
        Rectangle result = new Rectangle(leftX, topY, width, height);

        return result;
    }

    /**
     * Initialize the library of named materials during startup.
     */
    @Override
    public void generateMaterials() {
        super.generateMaterials();

        ColorRGBA red = new ColorRGBA(0.5f, 0f, 0f, 1f);
        Material missile = MyAsset.createShinyMaterial(assetManager, red);
        missile.setFloat("Shininess", 15f);
        registerMaterial("missile", missile);

        ColorRGBA lightGray = new ColorRGBA(0.6f, 0.6f, 0.6f, 1f);
        Material selected
                = MyAsset.createShinyMaterial(assetManager, lightGray);
        selected.setFloat("Shininess", 15f);
        registerMaterial("selected", selected);

        // shiny, lit materials for targets
        ColorRGBA[] targetColors = new ColorRGBA[numTargetColors];
        targetColors[0] = new ColorRGBA(0.2f, 0f, 0f, 1f); // ruby
        targetColors[1] = new ColorRGBA(0f, 0.07f, 0f, 1f); // emerald
        targetColors[2] = new ColorRGBA(0f, 0f, 0.3f, 1f); // sapphire
        targetColors[3] = new ColorRGBA(0.2f, 0.1f, 0f, 1f); // topaz

        for (int index = 0; index < targetColors.length; ++index) {
            ColorRGBA color = targetColors[index];
            Material material
                    = MyAsset.createShinyMaterial(assetManager, color);
            material.setFloat("Shininess", 15f);

            registerMaterial("target" + index, material);
        }
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
     * Determine the length of physics-debug arrows (when they're visible).
     *
     * @return the desired length (in physics-space units, &ge;0)
     */
    @Override
    protected float maxArrowLength() {
        return 2f;
    }

    /**
     * Add application-specific hotkey bindings (and override existing ones, if
     * necessary).
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        dim.bind(asCollectGarbage, KeyInput.KEY_G);
        dim.bind("delete selected", KeyInput.KEY_DECIMAL, KeyInput.KEY_DELETE);

        dim.bind("dump selected", KeyInput.KEY_LBRACKET);
        dim.bind(asDumpSpace, KeyInput.KEY_O);
        dim.bind(asDumpViewport, KeyInput.KEY_P);

        dim.bind("launch", KeyInput.KEY_RETURN, KeyInput.KEY_INSERT,
                KeyInput.KEY_NUMPAD0);

        dim.bind("next field", KeyInput.KEY_NUMPAD2);
        dim.bind("next value", KeyInput.KEY_EQUALS, KeyInput.KEY_NUMPAD6);

        dim.bind("pick", "RMB");
        dim.bind("pick", KeyInput.KEY_R);

        dim.bind("pop selected", KeyInput.KEY_PGUP);

        dim.bind("previous field", KeyInput.KEY_NUMPAD8);
        dim.bind("previous value", KeyInput.KEY_MINUS, KeyInput.KEY_NUMPAD4);

        dim.bind("restart", KeyInput.KEY_NUMPAD5);

        dim.bindSignal(CameraInput.FLYCAM_LOWER, KeyInput.KEY_DOWN);
        dim.bindSignal(CameraInput.FLYCAM_RISE, KeyInput.KEY_UP);
        dim.bindSignal("orbitLeft", KeyInput.KEY_LEFT);
        dim.bindSignal("orbitRight", KeyInput.KEY_RIGHT);

        dim.bind(asToggleAabbs, KeyInput.KEY_APOSTROPHE);
        dim.bind(asToggleCcdSpheres, KeyInput.KEY_L);
        dim.bind("toggle childColor", KeyInput.KEY_COMMA);
        dim.bind(asToggleGArrows, KeyInput.KEY_J);
        dim.bind(asToggleHelp, KeyInput.KEY_H);
        dim.bind(asTogglePause, KeyInput.KEY_PAUSE, KeyInput.KEY_PERIOD);
        dim.bind(asTogglePcoAxes, KeyInput.KEY_SEMICOLON);
        dim.bind(asToggleVArrows, KeyInput.KEY_K);
        dim.bind(asToggleWArrows, KeyInput.KEY_N);
        dim.bind("toggle wireframe", KeyInput.KEY_SLASH);
    }

    /**
     * Process an action that wasn't handled by the active InputMode.
     *
     * @param actionString textual description of the action (not null)
     * @param ongoing true if the action is ongoing, otherwise false
     * @param tpf time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void onAction(String actionString, boolean ongoing, float tpf) {
        if (ongoing) {
            switch (actionString) {
                case "delete selected":
                    deleteSelected();
                    return;
                case "dump selected":
                    dumpSelected();
                    return;
                case "launch":
                    launchMissile();
                    return;

                case "next field":
                    status.advanceSelectedField(+1);
                    return;
                case "next value":
                    status.advanceValue(+1);
                    return;

                case "pick":
                    pick();
                    return;
                case "pop selected":
                    popSelected();
                    return;

                case "previous field":
                    status.advanceSelectedField(-1);
                    return;
                case "previous value":
                    status.advanceValue(-1);
                    return;

                case "restart":
                    restartScenario();
                    return;

                case "toggle childColor":
                    status.toggleChildColor();
                    return;
                case "toggle wireframe":
                    status.toggleWireframe();
                    return;

                default:
            }
        }

        // The action is not handled: forward it to the superclass.
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
        status.resize(newWidth, newHeight);
        super.onViewPortResize(newWidth, newHeight);
    }

    /**
     * Callback invoked after adding a collision object to the PhysicsSpace.
     *
     * @param pco the object that was added (not null)
     */
    @Override
    public void postAdd(PhysicsCollisionObject pco) {
        if (pco instanceof PhysicsRigidBody) {
            PhysicsRigidBody rigidBody = (PhysicsRigidBody) pco;

            float damping = status.damping();
            rigidBody.setDamping(damping, damping);

            float linearThreshold = 1f;
            float angularThreshold = 1f;
            rigidBody.setSleepingThresholds(linearThreshold, angularThreshold);
        }

        float friction = status.friction();
        pco.setFriction(friction);

        float restitution = status.restitution();
        pco.setRestitution(restitution);

        setDebugMaterial(pco);
        pco.setDebugMeshResolution(DebugShapeFactory.highResolution);
    }
    // *************************************************************************
    // DebugInitListener methods

    /**
     * Callback from BulletDebugAppState, invoked just before the debug scene is
     * added to the debug viewports.
     *
     * @param physicsDebugRootNode the root node of the debug scene (not null)
     */
    @Override
    public void bulletDebugInit(Node physicsDebugRootNode) {
        addLighting(physicsDebugRootNode);
    }
    // *************************************************************************
    // private methods

    /**
     * Add lighting and shadows to the specified scene.
     *
     * @param rootSpatial which scene (not null)
     */
    private void addLighting(Spatial rootSpatial) {
        ColorRGBA ambientColor = new ColorRGBA(0.5f, 0.5f, 0.5f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        rootSpatial.addLight(ambient);
        ambient.setName("ambient");

        ColorRGBA directColor = new ColorRGBA(0.7f, 0.7f, 0.7f, 1f);
        Vector3f direction = new Vector3f(1f, -3f, -1f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction, directColor);
        rootSpatial.addLight(sun);
        sun.setName("sun");

        viewPort.clearProcessors();
        int mapSize = 2_048; // in pixels
        int numSplits = 3;
        DirectionalLightShadowRenderer dlsr
                = new DirectionalLightShadowRenderer(
                        assetManager, mapSize, numSplits);
        dlsr.setEdgeFilteringMode(EdgeFilteringMode.PCFPOISSON);
        dlsr.setEdgesThickness(5);
        dlsr.setLight(sun);
        dlsr.setShadowIntensity(0.7f);
        viewPort.addProcessor(dlsr);
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        float near = 0.1f;
        float far = 500f;
        MyCamera.setNearFar(cam, near, far);

        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(10f);
        flyCam.setZoomSpeed(10f);

        cam.setLocation(new Vector3f(0f, platformTopY + 20f, 40f));
        cam.setRotation(new Quaternion(-0.002f, 0.991408f, -0.1295f, 0.0184f));

        AppState orbitState
                = new CameraOrbitAppState(cam, "orbitLeft", "orbitRight");
        stateManager.attach(orbitState);
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        DebugShapeFactory.setIndexBuffers(200);

        bulletAppState = new SoftPhysicsAppState();
        bulletAppState.setDebugEnabled(true);
        bulletAppState.setDebugInitListener(this);
        bulletAppState.setDebugShadowMode(
                RenderQueue.ShadowMode.CastAndReceive);
        stateManager.attach(bulletAppState);

        float gravity = status.gravity();
        setGravityAll(gravity);
    }

    /**
     * Delete the selected rigid body, if any.
     */
    private void deleteSelected() {
        if (selectedBody != null) {
            getPhysicsSpace().removeCollisionObject(selectedBody);
            selectBody(null);
            activateAll();
        }
    }

    /**
     * Dump the selected rigid body, if any.
     */
    private void dumpSelected() {
        if (selectedBody == null) {
            System.out.printf("%nNo body selected.");
        } else {
            getDumper().dump(selectedBody, "");
        }
    }

    /**
     * Launch a new missile, its starting position and velocity determined by
     * the camera and mouse cursor.
     */
    private void launchMissile() {
        Vector2f screenXY = inputManager.getCursorPosition();
        Vector3f nearLocation
                = cam.getWorldCoordinates(screenXY, MyCamera.nearZ);
        Vector3f farLocation = cam.getWorldCoordinates(screenXY, MyCamera.farZ);

        Vector3f direction
                = farLocation.subtract(nearLocation).normalizeLocal();
        float initialSpeed = status.missileInitialSpeed(); // psu per second
        Vector3f initialVelocity = direction.mult(initialSpeed);

        float radius = status.missileRadius(); // psu
        CollisionShape shape = new MultiSphere(radius);
        float mass = status.missileMass(); // pmu
        PhysicsRigidBody body = new PhysicsRigidBody(shape, mass);

        Material debugMaterial = findMaterial("missile");
        body.setApplicationData(debugMaterial);
        body.setCcdMotionThreshold(radius);
        body.setCcdSweptSphereRadius(radius);
        body.setDebugMaterial(debugMaterial);
        body.setDebugMeshNormals(MeshNormals.Sphere);
        body.setDebugMeshResolution(DebugShapeFactory.highResolution);
        body.setLinearVelocity(initialVelocity);
        body.setPhysicsLocation(nearLocation);

        addCollisionObject(body);
    }

    /**
     * Cast a physics ray from the cursor and select the nearest rigid body in
     * the result.
     */
    private void pick() {
        List<PhysicsRayTestResult> hits = rayTestCursor();
        for (PhysicsRayTestResult hit : hits) {
            PhysicsCollisionObject pco = hit.getCollisionObject();
            if (pco instanceof PhysicsRigidBody) {
                selectBody((PhysicsRigidBody) pco);
                return;
            }
        }
        selectBody(null);
    }

    /**
     * Apply an upward impulse to the selected rigid body.
     */
    private void popSelected() {
        if (selectedBody instanceof PhysicsRigidBody) {
            float gravity = status.gravity();
            float deltaV = FastMath.sqrt(30f * gravity);
            float impulse = selectedBody.getMass() * deltaV;
            Vector3f impulseVector = new Vector3f(0f, impulse, 0f);
            Generator random = getGenerator();
            Vector3f offset = random.nextVector3f().multLocal(0.2f);
            PhysicsRigidBody rigidBody = selectedBody;
            rigidBody.applyImpulse(impulseVector, offset);
        }
    }

    /**
     * Register a spherical shape with the specified radius.
     *
     * @param radius the desired radius (in physics-space units, &gt;0)
     */
    private void registerBallShape(float radius) {
        unregisterShape("ball");
        CollisionShape shape = new SphereCollisionShape(radius);
        registerShape("ball", shape);
    }

    /**
     * Register a bowling-pin shape with the specified radius.
     *
     * @param radius the desired radius (in physics-space units, &gt;0)
     */
    private void registerBowlingPinShape(float radius) {
        unregisterShape("bowlingPin");

        String bowlingPinPath = "CollisionShapes/bowlingPin.j3o";
        CollisionShape shape
                = (CollisionShape) assetManager.loadAsset(bowlingPinPath);
        shape = Heart.deepCopy(shape);

        BoundingBox bounds
                = shape.boundingBox(Vector3f.ZERO, Matrix3f.IDENTITY, null);
        float xHalfExtent = bounds.getXExtent();
        float yHalfExtent = bounds.getYExtent();
        float unscaledRadius = (xHalfExtent + yHalfExtent) / 2f;
        float scale = radius / unscaledRadius;
        shape.setScale(scale);

        registerShape("bowlingPin", shape);
    }

    /**
     * Register a brick shape with the specified name, height, and length.
     *
     * @param shapeName (Z axis, not null)
     * @param height the total height (Y axis, in physics-space units, &gt;0)
     * @param length the total length (X axis, in physics-space units, &gt;0)
     * @param depth the total depth (Z axis, in physics-space units, &gt;0)
     */
    private void registerBrickShape(
            String shapeName, float height, float length, float depth) {
        unregisterShape(shapeName);

        float halfHeight = height / 2f;
        float halfLength = length / 2f;
        float halfDepth = depth / 2f;
        CollisionShape shape
                = new BoxCollisionShape(halfLength, halfHeight, halfDepth);

        registerShape(shapeName, shape);
    }

    /**
     * Register a can shape with the specified radius and height.
     *
     * @param radius the radius (in physics-space units, &gt;0)
     * @param height the total height (Y axis, in physics-space units, &gt;0)
     */
    private void registerCanShape(float radius, float height) {
        unregisterShape("can");
        CollisionShape shape = new CylinderCollisionShape(
                radius, height, PhysicsSpace.AXIS_Y);
        registerShape("can", shape);
    }

    /**
     * Register a domino shape with the specified length.
     *
     * @param length the total length (Y axis, in physics-space units, &gt;0)
     */
    private void registerDominoShape(float length) {
        unregisterShape("domino");

        float halfLength = length / 2f;
        float halfThickness = halfLength / 5f;
        float halfWidth = halfLength / 2f;
        CollisionShape shape
                = new BoxCollisionShape(halfThickness, halfLength, halfWidth);

        registerShape("domino", shape);
    }

    /**
     * Alter which rigid body is selected.
     *
     * @param rigidBody the body to select (alias created) or null for none
     */
    private void selectBody(PhysicsRigidBody rigidBody) {
        if (rigidBody != selectedBody) {
            selectedBody = rigidBody;
            setDebugMaterialsAll();
        }
    }

    /**
     * Update the debug materials of the specified collision object.
     *
     * @param pco the collision object to modify (not null)
     */
    private void setDebugMaterial(PhysicsCollisionObject pco) {
        CollisionShape shape = pco.getCollisionShape();

        Material debugMaterial;
        if (selectedBody == pco) {
            debugMaterial = findMaterial("selected");

        } else if (status.isWireframe()) {
            debugMaterial = null;

        } else if (status.isChildColoring()
                && shape instanceof CompoundCollisionShape) {
            debugMaterial = BulletDebugAppState.enableChildColoring;

        } else {
            // Use the previously set lit material.
            debugMaterial = (Material) pco.getApplicationData();
        }

        pco.setDebugMaterial(debugMaterial);
    }

    /**
     * Set up a single ball as a target.
     *
     * @param location the desired location (in physics-space coordinates, not
     * null, unaffected)
     */
    private void setUpBall(Vector3f location) {
        CollisionShape shape = findShape("ball");
        float mass = 0.2f;
        PhysicsRigidBody body = new PhysicsRigidBody(shape, mass);
        body.setDebugMeshNormals(MeshNormals.Sphere);
        body.setPhysicsLocation(location);

        setUpTarget(body);
    }

    /**
     * Set up a single bowling pin as a target.
     *
     * @param location the desired location (in physics-space coordinates, not
     * null, unaffected)
     */
    private void setUpBowlingPin(Vector3f location) {
        CollisionShape shape = findShape("bowlingPin");
        float mass = 0.2f;
        PhysicsRigidBody body = new PhysicsRigidBody(shape, mass);
        body.setDebugMeshNormals(MeshNormals.Smooth);
        body.setPhysicsLocation(location);

        Quaternion rotation = new Quaternion();
        rotation.fromAngles(FastMath.HALF_PI, 0f, 0f);
        body.setPhysicsRotation(rotation);

        setUpTarget(body);
    }

    /**
     * Set up a single brick as a target.
     *
     * @param shapeName the key to the shapes library
     * @param location the desired world location (not null, unaffected)
     * @param orientation the desired world orientation (not null, unaffected)
     */
    private void setUpBrick(
            String shapeName, Vector3f location, Quaternion orientation) {
        CollisionShape shape = findShape(shapeName);

        float mass = 3f;
        PhysicsRigidBody body = new PhysicsRigidBody(shape, mass);
        body.setDebugMeshNormals(MeshNormals.Facet);
        body.setPhysicsLocation(location);
        body.setPhysicsRotation(orientation);

        setUpTarget(body);
    }

    /**
     * Set up a round tower of bricks.
     *
     * @param numRings the desired number of rings/layers of bricks (&ge;0)
     * @param numBricksPerRing the desired number of bricks per ring (&gt;0)
     * @param thickness the thickness of the tower wall (&gt;0)
     */
    private void setUpBrickTower(
            int numRings, int numBricksPerRing, float thickness) {
        float innerDiameter = 32f - 2f * thickness;
        float innerCircumference = FastMath.PI * innerDiameter;
        float insideSpacing = innerCircumference / numBricksPerRing;
        float insideGap = 0.05f * insideSpacing;
        float length = insideSpacing - insideGap;
        float height = Math.min(length, thickness) / MyMath.phi;
        registerBrickShape("tower", height, length, thickness);

        float angleStep = FastMath.TWO_PI / numBricksPerRing;
        float midRadius = (innerDiameter + thickness) / 2f;
        float y0 = platformTopY + height / 2f;
        Quaternion orientation = new Quaternion();
        Vector3f location = new Vector3f(0f, y0, midRadius);
        for (int ringIndex = 0; ringIndex < numRings; ++ringIndex) {
            float theta0;
            if (MyMath.isOdd(ringIndex)) {
                theta0 = angleStep / 2f;
            } else {
                theta0 = 0f;
            }
            for (int j = 0; j < numBricksPerRing; ++j) {
                float theta = theta0 + j * angleStep;
                location.x = midRadius * FastMath.sin(theta);
                location.z = midRadius * FastMath.cos(theta);
                orientation.fromAngles(0f, theta, 0f);
                setUpBrick("tower", location, orientation);
            }
            location.y += height;
        }
    }

    /**
     * Erect a brick wall along the X axis.
     *
     * @param numRows the desired number of rows of bricks (&ge;0)
     * @param numBricksPerRow the desired number of bricks per row (&ge;1)
     */
    private void setUpBrickWall(int numRows, int numBricksPerRow) {
        float xSpacing = 32f / numBricksPerRow; // center-to-center
        float xGap = 0.1f * xSpacing;
        float length = xSpacing - xGap;
        float shortLength = (length - xGap) / 2f;
        float thickness = length / MyMath.phi;
        float height = thickness / MyMath.phi;
        registerBrickShape("short", height, shortLength, thickness);
        registerBrickShape("long", height, length, thickness);

        float x0even = -xSpacing * (numBricksPerRow - 1) / 2f;
        float endSpacing = xGap + (length + shortLength) / 2f;
        float x1odd = x0even + xSpacing / 2f - endSpacing;
        float y0 = platformTopY + height / 2f;
        float ySpacing = 1f * height; // center-to-center
        Vector3f location = new Vector3f(x0even, y0, 0f);
        for (int rowIndex = 0; rowIndex < numRows; ++rowIndex) {
            if (MyMath.isOdd(rowIndex)) {
                location.x = x1odd;
                setUpBrick("short", location, Quaternion.IDENTITY);
                location.x += endSpacing;

                for (int j = 1; j < numBricksPerRow; ++j) {
                    setUpBrick("long", location, Quaternion.IDENTITY);
                    location.x += xSpacing;
                }

                location.x += endSpacing - xSpacing;
                setUpBrick("short", location, Quaternion.IDENTITY);

            } else {
                location.x = x0even;
                for (int j = 0; j < numBricksPerRow; ++j) {
                    setUpBrick("long", location, Quaternion.IDENTITY);
                    location.x += xSpacing;
                }
            }
            location.y += ySpacing;
        }
    }

    /**
     * Set up a single can as a target.
     *
     * @param location the desired location (in physics-space coordinates, not
     * null, unaffected)
     */
    private void setUpCan(Vector3f location) {
        CollisionShape shape = findShape("can");
        float mass = 10f;
        PhysicsRigidBody body = new PhysicsRigidBody(shape, mass);
        body.setDebugMeshNormals(MeshNormals.Smooth);
        body.setPhysicsLocation(location);

        setUpTarget(body);
    }

    /**
     * Erect a pyramid of cans along the X axis.
     *
     * @param numRows the desired number of rows in the pyramid (&ge;1)
     */
    private void setUpCanPyramid(int numRows) {
        float xSpacing = 32f / numRows; // center-to-center
        float xGap = 0.1f * xSpacing;
        float radius = (xSpacing - xGap) / 2f;
        float height = 2.65f * radius;
        registerCanShape(radius, height);

        float ySpacing = 1f * height; // center-to-center
        float y0 = platformTopY + height / 2f;
        Vector3f location = new Vector3f(0, y0, 0f);
        for (int rowIndex = 0; rowIndex < numRows; ++rowIndex) {
            int numCansInRow = numRows - rowIndex;
            location.x = -(numCansInRow - 1) * xSpacing / 2f;
            for (int j = 0; j < numCansInRow; ++j) {
                setUpCan(location);
                location.x += xSpacing;
            }
            location.y += ySpacing;
        }
    }

    /**
     * Set up a single domino as a target.
     *
     * @param location the desired location (in physics-space coordinates, not
     * null, unaffected)
     * @param orientation the desired orientation (in physics-space coordinates,
     * not null, unaffected)
     */
    private void setUpDomino(Vector3f location, Quaternion orientation) {
        CollisionShape shape = findShape("domino");

        float mass = 10f;
        PhysicsRigidBody body = new PhysicsRigidBody(shape, mass);
        body.setDebugMeshNormals(MeshNormals.Facet);
        body.setPhysicsLocation(location);
        body.setPhysicsRotation(orientation);

        setUpTarget(body);
    }

    /**
     * Set up a row of dominoes, evenly-spaced along the X axis.
     *
     * @param numDominoes the desired number of dominoes (&ge;1)
     */
    private void setUpDominoRow(int numDominoes) {
        float xSpacing = 32f / numDominoes; // center-to-center
        float length = 1.6f * xSpacing;
        registerDominoShape(length);

        float x0 = -xSpacing * (numDominoes - 1) / 2f;
        float y = platformTopY + length / 2;
        Vector3f location = new Vector3f(x0, y, 0f);
        for (int j = 0; j < numDominoes; ++j) {
            setUpDomino(location, Quaternion.IDENTITY);
            location.x += xSpacing;
        }
    }

    /**
     * Set up 15 balls in a wedge, as if for a game of Pool.
     */
    private void setUpPool() {
        int numRows = 5;
        float xSpacing = 32f / numRows; // center-to-center
        float radius = 0.48f * xSpacing;
        registerBallShape(radius);

        float zSpacing = xSpacing / FastMath.sqrt(1.5f); // center-to-center
        float z0 = (numRows - 1) * zSpacing / 2f;
        Vector3f location = new Vector3f(0, radius, z0);
        for (int rowIndex = 0; rowIndex < numRows; ++rowIndex) {
            int numBallsInRow = rowIndex + 1;
            location.x = -(numBallsInRow - 1) * xSpacing / 2f;
            for (int j = 0; j < numBallsInRow; ++j) {
                setUpBall(location);
                location.x += xSpacing;
            }
            location.z -= zSpacing;
        }
    }

    /**
     * Set up all targets for selected scenario.
     */
    private void setUpScenario() {
        String scenarioName = status.scenarioName();
        switch (scenarioName) {
            case "brick tower": {
                int numRings = 10;
                int numBricksPerRing = 16;
                float thickness = 2f;
                setUpBrickTower(numRings, numBricksPerRing, thickness);
                break;
            }

            case "brick wall": {
                int numRows = 14;
                int numBricksPerRow = 10;
                setUpBrickWall(numRows, numBricksPerRow);
                break;
            }

            case "can pyramid": {
                int numRows = 15;
                setUpCanPyramid(numRows);
                break;
            }

            case "domino row": {
                int numDominoes = 25;
                setUpDominoRow(numDominoes);
                break;
            }

            case "empty":
                break;

            case "pool":
                setUpPool();
                break;

            case "tenpin":
                setUpTenpin();
                break;

            default:
                String msg = "scenarioName = " + MyString.quote(scenarioName);
                throw new RuntimeException(msg);
        }
    }

    /**
     * Set up a single target body whose position and debug normals have already
     * been configured.
     *
     * @param body the body to set up (not null, not in world)
     */
    private void setUpTarget(PhysicsRigidBody body) {
        assert body != null;
        assert !body.isInWorld();

        Random random = getGenerator();
        String materialName = "target" + random.nextInt(numTargetColors);
        Material debugMaterial = findMaterial(materialName);
        body.setApplicationData(debugMaterial);

        addCollisionObject(body);
    }

    /**
     * Set up 10 bowling pins in a wedge, as if for a game of (ten-pin) bowling.
     */
    private void setUpTenpin() {
        int numRows = 4;
        float xSpacing = 32f / numRows; // center-to-center
        float radius = 0.2f * xSpacing;
        registerBowlingPinShape(radius);

        float y0 = 2.50f * radius;
        float zSpacing = xSpacing / FastMath.sqrt(1.5f); // center-to-center
        float z0 = (numRows - 1) * zSpacing / 2f;
        Vector3f location = new Vector3f(0, y0, z0);
        for (int rowIndex = 0; rowIndex < numRows; ++rowIndex) {
            int numPinsInRow = rowIndex + 1;
            location.x = -(numPinsInRow - 1) * xSpacing / 2f;
            for (int j = 0; j < numPinsInRow; ++j) {
                setUpBowlingPin(location);
                location.x += xSpacing;
            }
            location.z -= zSpacing;
        }
    }
}
