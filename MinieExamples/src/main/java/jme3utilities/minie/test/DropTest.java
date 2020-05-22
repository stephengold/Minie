/*
 Copyright (c) 2018-2020, Stephen Gold
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
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.SoftPhysicsAppState;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.PhysicsRayTestResult;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.MeshCollisionShape;
import com.jme3.bullet.collision.shapes.infos.DebugMeshNormals;
import com.jme3.bullet.debug.BulletDebugAppState;
import com.jme3.bullet.debug.DebugInitListener;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.bullet.objects.infos.SoftBodyConfig;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.bullet.util.NativeSoftBodyUtil;
import com.jme3.font.Rectangle;
import com.jme3.input.CameraInput;
import com.jme3.input.KeyInput;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.shadow.DirectionalLightShadowRenderer;
import com.jme3.shadow.EdgeFilteringMode;
import com.jme3.system.AppSettings;
import com.jme3.util.BufferUtils;
import java.util.ArrayDeque;
import java.util.Collection;
import java.util.Deque;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MyAsset;
import jme3utilities.MyCamera;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.minie.test.common.AbstractDemo;
import jme3utilities.minie.test.mesh.ClothHexagon;
import jme3utilities.minie.test.shape.ShapeGenerator;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.InputMode;
import jme3utilities.ui.Signals;

/**
 * Test/demonstrate various collision shapes by dropping drops
 * (small/dynamic/rigid bodies) onto a platform (large/fixed/horizontal body).
 * <p>
 * Collision objects are rendered entirely by debug visualization.
 * <p>
 * Seen in the November 2018 demo video:
 * https://www.youtube.com/watch?v=OS2zjB01c6E
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class DropTest
        extends AbstractDemo
        implements DebugInitListener {
    // *************************************************************************
    // constants and loggers

    /**
     * approximate Y coordinate for the main surface of the platform (in
     * physics-space coordinates)
     */
    final private static float platformSurfaceY = 0f;
    /**
     * upper limit on the number of drops
     */
    final private static int maxNumDrops = 80;
    /**
     * number of colors/materials for drops
     */
    final private static int numDropColors = 4;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(DropTest.class.getName());
    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName
            = DropTest.class.getSimpleName();
    // *************************************************************************
    // fields

    /**
     * current drops, in order of creation
     */
    final private Deque<Drop> drops = new ArrayDeque<>(maxNumDrops);
    /**
     * selected drop, or null if none
     */
    private Drop selectedDrop = null;
    /**
     * AppState to manage the status overlay
     */
    private DropTestStatus status;
    /**
     * AppState to manage the PhysicsSpace
     */
    private SoftPhysicsAppState bulletAppState;
    // *************************************************************************
    // new methods exposed

    /**
     * Count how many rigid bodies are active.
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
     * Count how many drops are in the PhysicsSpace.
     */
    int countDrops() {
        int result = drops.size();
        return result;
    }

    /**
     * Main entry point for the DropTest application.
     *
     * @param ignored array of command-line arguments (not null)
     */
    public static void main(String[] ignored) {
        /*
         * Mute the chatty loggers in certain packages.
         */
        Heart.setLoggingLevels(Level.WARNING);
        /*
         * Enable direct-memory tracking.
         */
        BufferUtils.setTrackDirectMemoryEnabled(true);

        Application application = new DropTest();
        /*
         * Customize the window's title bar.
         */
        boolean loadDefaults = true;
        AppSettings settings = new AppSettings(loadDefaults);
        settings.setTitle(applicationName);

        settings.setGammaCorrection(true);
        settings.setSamples(4); // anti-aliasing
        settings.setVSync(true);
        application.setSettings(settings);

        application.start();
    }

    /**
     * Restart the current scenario.
     */
    void restartScenario() {
        for (Drop drop : drops) {
            drop.removeFromSpace();
        }
        selectDrop(null);
        drops.clear();

        PhysicsSpace physicsSpace = getPhysicsSpace();
        Collection<PhysicsCollisionObject> pcos = physicsSpace.getPcoList();
        for (PhysicsCollisionObject pco : pcos) {
            physicsSpace.removeCollisionObject(pco);
        }

        assert physicsSpace.isEmpty();

        String platformName = status.platformType();
        addPlatform(platformName, platformSurfaceY);
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
    void setDebugShadowMode() {
        RenderQueue.ShadowMode mode;
        if (status.isWireframe()) {
            mode = RenderQueue.ShadowMode.Off;
        } else {
            mode = RenderQueue.ShadowMode.CastAndReceive;
        }
        bulletAppState.setDebugShadowMode(mode);
    }
    // *************************************************************************
    // AbstractDemo methods

    /**
     * Initialize this application.
     */
    @Override
    public void actionInitializeApplication() {
        status = new DropTestStatus();
        boolean success = stateManager.attach(status);
        assert success;

        configureCamera();
        configureDumper();
        generateMaterials();
        configurePhysics();
        generateShapes();

        ColorRGBA skyColor = new ColorRGBA(0.1f, 0.2f, 0.4f, 1f);
        viewPort.setBackgroundColor(skyColor);

        String platformName = status.platformType();
        addPlatform(platformName, platformSurfaceY);

        renderer.setDefaultAnisotropicFilter(8);
        addADrop();
    }

    /**
     * Add a platform to the PhysicsSpace.
     *
     * @param platformName the name of the desired platform type (not null)
     * @param topY the desired Y coordinate of the top surface (in physics-space
     * coordinates)
     */
    @Override
    public void addPlatform(String platformName, float topY) {
        switch (platformName) {
            case "bedOfNails":
            case "corner":
            case "sieve":
            case "tray":
                addPlatform(platformName, DebugMeshNormals.Facet, topY);
                break;

            case "candyDish":
            case "dimples":
            case "smooth":
                addPlatform(platformName, DebugMeshNormals.Smooth, topY);
                break;

            case "trampoline":
                addTrampoline(topY);
                break;

            default:
                super.addPlatform(platformName, topY);
        }
    }

    /**
     * Initialize the library of named materials during startup.
     */
    @Override
    public void generateMaterials() {
        super.generateMaterials();
        /*
         * shiny, lit material for the selected drop
         */
        ColorRGBA lightGray = new ColorRGBA(0.6f, 0.6f, 0.6f, 1f);
        Material selected
                = MyAsset.createShinyMaterial(assetManager, lightGray);
        selected.setFloat("Shininess", 15f);
        registerMaterial("selected", selected);
        /*
         * shiny, lit materials for dynamic bodies in drops
         */
        ColorRGBA dropColors[] = new ColorRGBA[numDropColors];
        dropColors[0] = new ColorRGBA(0.2f, 0f, 0f, 1f); // ruby
        dropColors[1] = new ColorRGBA(0f, 0.07f, 0f, 1f); // emerald
        dropColors[2] = new ColorRGBA(0f, 0f, 0.3f, 1f); // sapphire
        dropColors[3] = new ColorRGBA(0.2f, 0.1f, 0f, 1f); // topaz

        for (int index = 0; index < dropColors.length; ++index) {
            ColorRGBA color = dropColors[index];
            Material material
                    = MyAsset.createShinyMaterial(assetManager, color);
            material.setFloat("Shininess", 15f);
            RenderState additional = material.getAdditionalRenderState();
            additional.setFaceCullMode(RenderState.FaceCullMode.Off);

            registerMaterial("drop" + index, material);
        }
    }

    /**
     * Initialize the library of named collision shapes during startup.
     */
    @Override
    public void generateShapes() {
        super.generateShapes();
        CollisionShape shape;
        /*
         * "ankh" using manual decomposition
         */
        String ankhPath = "CollisionShapes/ankh.j3o";
        shape = (CollisionShape) assetManager.loadAsset(ankhPath);
        registerShape("ankh", shape);
        /*
         * "banana" using manual decomposition
         */
        String bananaPath = "CollisionShapes/banana.j3o";
        shape = (CollisionShape) assetManager.loadAsset(bananaPath);
        registerShape("banana", shape);
        /*
         * "barrel"
         */
        String barrelPath = "CollisionShapes/barrel.j3o";
        shape = (CollisionShape) assetManager.loadAsset(barrelPath);
        shape.setScale(3f);
        registerShape("barrel", shape);
        /*
         * "bowlingPin" using manual decomposition
         */
        String bowlingPinPath = "CollisionShapes/bowlingPin.j3o";
        shape = (CollisionShape) assetManager.loadAsset(bowlingPinPath);
        registerShape("bowlingPin", shape);
        /*
         * "candyDish"
         */
        String candyDishPath = "Models/CandyDish/CandyDish.j3o";
        Node candyDishNode = (Node) assetManager.loadModel(candyDishPath);
        Geometry candyDishGeometry = (Geometry) candyDishNode.getChild(0);
        Mesh candyDishMesh = candyDishGeometry.getMesh();
        shape = new MeshCollisionShape(candyDishMesh);
        shape.setScale(5f);
        registerShape("candyDish", shape);
        /*
         * "duck" using V-HACD
         */
        String duckPath = "CollisionShapes/duck.j3o";
        shape = (CollisionShape) assetManager.loadAsset(duckPath);
        shape.setScale(2f);
        registerShape("duck", shape);
        /*
         * "heart"
         */
        String heartPath = "CollisionShapes/heart.j3o";
        shape = (CollisionShape) assetManager.loadAsset(heartPath);
        shape.setScale(1.2f);
        registerShape("heart", shape);
        /*
         * "horseshoe" using manual decomposition
         */
        String horseshoePath = "CollisionShapes/horseshoe.j3o";
        shape = (CollisionShape) assetManager.loadAsset(horseshoePath);
        registerShape("horseshoe", shape);
        /*
         * "sword" using V-HACD
         */
        String swordPath = "CollisionShapes/sword.j3o";
        shape = (CollisionShape) assetManager.loadAsset(swordPath);
        shape.setScale(5f);
        registerShape("sword", shape);
        /*
         * "teapot" using V-HACD
         */
        String teapotPath = "CollisionShapes/teapot.j3o";
        shape = (CollisionShape) assetManager.loadAsset(teapotPath);
        shape.setScale(3f);
        registerShape("teapot", shape);
        /*
         * letter shapes
         */
        for (char character = 'A'; character <= 'Z'; ++character) {
            char[] array = new char[]{character};
            String glyphString = new String(array);
            String assetPath = String.format("CollisionShapes/glyphs/%s.j3o",
                    glyphString);
            shape = (CollisionShape) assetManager.loadAsset(assetPath);
            registerShape(glyphString, shape);
        }
        /*
         * digit shapes
         */
        for (char character = '0'; character <= '9'; ++character) {
            char[] array = new char[]{character};
            String glyphString = new String(array);
            String assetPath = String.format("CollisionShapes/glyphs/%s.j3o",
                    glyphString);
            shape = (CollisionShape) assetManager.loadAsset(assetPath);
            registerShape(glyphString, shape);
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
     * Determine the length of debug axis arrows (when they're visible).
     *
     * @return the desired length (in physics-space units, &ge;0)
     */
    @Override
    protected float maxArrowLength() {
        return 2f;
    }

    /**
     * Add application-specific hotkey/button bindings and override existing
     * ones.
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        dim.bind("add", KeyInput.KEY_RETURN);
        dim.bind("add", KeyInput.KEY_INSERT);
        dim.bind("add", KeyInput.KEY_NUMPAD0);

        dim.bind("advance value +7", KeyInput.KEY_NUMPAD9);
        dim.bind("advance value -7", KeyInput.KEY_NUMPAD7);

        dim.bind(AbstractDemo.asCollectGarbage, KeyInput.KEY_G);

        dim.bind("delete last", KeyInput.KEY_BACK);
        dim.bind("delete last", KeyInput.KEY_SUBTRACT);
        dim.bind("delete selected", KeyInput.KEY_DECIMAL);
        dim.bind("delete selected", KeyInput.KEY_DELETE);

        dim.bind(AbstractDemo.asDumpPhysicsSpace, KeyInput.KEY_O);
        dim.bind("dump selected", KeyInput.KEY_LBRACKET);
        dim.bind(AbstractDemo.asDumpViewport, KeyInput.KEY_P);

        dim.bind("next statusLine", KeyInput.KEY_NUMPAD2);
        dim.bind("next value", KeyInput.KEY_EQUALS);
        dim.bind("next value", KeyInput.KEY_NUMPAD6);

        dim.bind("pick", "RMB");
        dim.bind("pick", KeyInput.KEY_R);

        dim.bind("pop selected", KeyInput.KEY_PGUP);

        dim.bind("previous statusLine", KeyInput.KEY_NUMPAD8);
        dim.bind("previous value", KeyInput.KEY_MINUS);
        dim.bind("previous value", KeyInput.KEY_NUMPAD4);

        dim.bind("restart scenario", KeyInput.KEY_NUMPAD5);

        dim.bind("signal " + CameraInput.FLYCAM_LOWER, KeyInput.KEY_DOWN);
        dim.bind("signal " + CameraInput.FLYCAM_RISE, KeyInput.KEY_UP);
        dim.bind("signal orbitLeft", KeyInput.KEY_LEFT);
        dim.bind("signal orbitRight", KeyInput.KEY_RIGHT);
        dim.bind("signal shower", KeyInput.KEY_ADD);
        dim.bind("signal shower", KeyInput.KEY_I);

        dim.bind(AbstractDemo.asToggleAabbs, KeyInput.KEY_APOSTROPHE);
        dim.bind(AbstractDemo.asToggleCcdSpheres, KeyInput.KEY_L);
        dim.bind("toggle childColoring", KeyInput.KEY_COMMA);
        dim.bind(AbstractDemo.asToggleGravities, KeyInput.KEY_J);
        dim.bind(AbstractDemo.asToggleHelp, KeyInput.KEY_H);
        dim.bind(AbstractDemo.asTogglePause, KeyInput.KEY_PAUSE);
        dim.bind(AbstractDemo.asTogglePause, KeyInput.KEY_PERIOD);
        dim.bind(AbstractDemo.asTogglePcoAxes, KeyInput.KEY_SEMICOLON);
        dim.bind(AbstractDemo.asToggleVelocities, KeyInput.KEY_K);
        dim.bind("toggle wireframe", KeyInput.KEY_SLASH);
        /*
         * The help node can't be created until all hotkeys are bound.
         */
        addHelp();
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
                case "add":
                    addADrop();
                    return;

                case "advance value +7":
                    status.advanceValue(+7);
                    return;
                case "advance value -7":
                    status.advanceValue(-7);
                    return;

                case "delete last":
                    deleteLastDrop();
                    return;
                case "delete selected":
                    deleteSelected();
                    return;

                case "dump selected":
                    dumpSelected();
                    return;

                case "next statusLine":
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

                case "previous statusLine":
                    status.advanceSelectedField(-1);
                    return;
                case "previous value":
                    status.advanceValue(-1);
                    return;

                case "restart scenario":
                    restartScenario();
                    return;

                case "toggle childColoring":
                    status.toggleChildColoring();
                    return;
                case "toggle wireframe":
                    status.toggleWireframe();
                    return;
            }
        }
        super.onAction(actionString, ongoing, tpf);
    }

    /**
     * Callback invoked after adding a collision object to the PhysicsSpace.
     *
     * @param pco the object that was added (not null)
     */
    @Override
    public void postAdd(PhysicsCollisionObject pco) {
        Object appData = pco.getApplicationData();
        if (appData == null) {
            ShapeGenerator random = getGenerator();
            String materialName = "drop" + random.nextInt(numDropColors);
            Material debugMaterial = findMaterial(materialName);
            assert debugMaterial != null : materialName;
            pco.setApplicationData(debugMaterial);
        }

        CollisionShape shape = pco.getCollisionShape();
        if (!pco.isStatic() && shape != null) {
            pco.setCcdMotionThreshold(5f);

            float sweptSphereRadius = shape.maxRadius();
            pco.setCcdSweptSphereRadius(sweptSphereRadius);
        }

        if (pco instanceof PhysicsRigidBody) {
            PhysicsRigidBody rigidBody = (PhysicsRigidBody) pco;

            float damping = status.damping();
            rigidBody.setDamping(damping, damping);

            rigidBody.setSleepingThresholds(0.1f, 0.1f);
        }

        pco.setDebugMeshResolution(DebugShapeFactory.highResolution);

        float friction = status.friction();
        pco.setFriction(friction);

        float restitution = status.restitution();
        pco.setRestitution(restitution);

        setDebugMaterial(pco);
    }

    /**
     * Callback invoked once per frame.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void simpleUpdate(float tpf) {
        super.simpleUpdate(tpf);

        for (Drop drop : drops) {
            drop.update();
        }

        Signals signals = getSignals();
        if (signals.test("shower")) {
            addADrop();
        }
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
     * Add a drop (dynamic rigid body) to the PhysicsSpace.
     */
    private void addADrop() {
        if (countDrops() >= maxNumDrops) {
            return; // too many drops
        }

        ShapeGenerator random = getGenerator();
        Vector3f startLocation = random.nextVector3f(); //TODO garbage
        startLocation.multLocal(2.5f, 5f, 2.5f);
        startLocation.y += 20f;
        Quaternion startOrientation = random.nextQuaternion(); //TODO garbage
        Transform startPosition
                = new Transform(startLocation, startOrientation); //TODO garbage

        String typeName = status.nextDropType();
        float totalMass = 1f;
        Drop drop = new Drop(this, typeName, totalMass, startPosition);

        drop.addToSpace();
        drops.addLast(drop);
    }

    /**
     * Attach a Node to display hotkey help/hints.
     */
    private void addHelp() {
        float margin = 10f; // in pixels
        float width = 370f; // in pixels
        float height = cam.getHeight() - (2 * margin + 2 * 20f);
        float leftX = cam.getWidth() - (width + margin);
        float topY = height + margin;
        Rectangle rectangle = new Rectangle(leftX, topY, width, height);

        attachHelpNode(rectangle);
    }

    /**
     * Add lighting and shadows to the specified scene.
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

        rootSpatial.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);

        viewPort.clearProcessors();
        DirectionalLightShadowRenderer dlsr
                = new DirectionalLightShadowRenderer(assetManager, 2_048, 3);
        dlsr.setEdgeFilteringMode(EdgeFilteringMode.PCFPOISSON);
        dlsr.setEdgesThickness(5);
        dlsr.setLight(sun);
        dlsr.setShadowIntensity(0.7f);
        viewPort.addProcessor(dlsr);
    }

    /**
     * Add a hexagonal trampoline to the PhysicsSpace, to serve as a platform.
     *
     * @param y the initial Y coordinate
     */
    private void addTrampoline(float y) {
        int numRings = 9;
        float vertexSpacing = 2f;
        Mesh mesh = new ClothHexagon(numRings, vertexSpacing);
        PhysicsSoftBody softBody = new PhysicsSoftBody();
        NativeSoftBodyUtil.appendFromTriMesh(mesh, softBody);
        softBody.applyTranslation(new Vector3f(0f, y, 0f));
        /*
         * Pin every node on the perimeter.
         */
        int numNodes = mesh.getVertexCount();
        int numInteriorNodes = 1 + 3 * numRings * (numRings - 1);
        for (int nodeI = numInteriorNodes; nodeI < numNodes; ++nodeI) {
            softBody.setNodeMass(nodeI, PhysicsBody.massForStatic);
        }

        softBody.setDebugMeshNormals(DebugMeshNormals.Smooth);
        softBody.setMargin(1f);
        softBody.setMass(100f);

        SoftBodyConfig config = softBody.getSoftConfig();
        config.setPositionIterations(3);

        addPlatform(softBody);
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

        cam.setLocation(new Vector3f(0f, platformSurfaceY + 20f, 40f));
        cam.setRotation(new Quaternion(0f, 0.9649f, -0.263f, 0f));

        CameraOrbitAppState orbitState
                = new CameraOrbitAppState(cam, "orbitLeft", "orbitRight");
        stateManager.attach(orbitState);
    }

    /**
     * Create and configure a new PhysicsSpace.
     */
    private void configurePhysics() {
        DebugShapeFactory.setIndexBuffers(200);

        bulletAppState = new SoftPhysicsAppState();
        bulletAppState.setDebugEnabled(true);
        bulletAppState.setDebugInitListener(this);
        stateManager.attach(bulletAppState);

        float gravity = status.gravity();
        setGravityAll(gravity);
    }

    /**
     * Delete the most recently added drop.
     */
    private void deleteLastDrop() {
        Drop lastDrop = drops.peekLast();
        if (lastDrop != null) {
            lastDrop.removeFromSpace();
            if (lastDrop == selectedDrop) {
                selectDrop(null);
            }
            drops.removeLast();
            activateAll();
        }
    }

    /**
     * Delete the selected drop, if any.
     */
    private void deleteSelected() {
        if (selectedDrop != null) {
            selectedDrop.removeFromSpace();
            boolean success = drops.remove(selectedDrop);
            assert success;
            selectDrop(null);
            activateAll();
        }
    }

    /**
     * Dump the selected drop, if any.
     */
    private void dumpSelected() {
        if (selectedDrop == null) {
            System.out.printf("%nNo drop selected.");
        } else {
            PhysicsDumper dumper = getDumper();
            PhysicsSpace space = getPhysicsSpace();
            dumper.dump(space, "", selectedDrop);
        }
    }

    /**
     * Cast a physics ray from the cursor and select the nearest drop in the
     * result.
     */
    private void pick() {
        List<PhysicsRayTestResult> hits = rayTestCursor();
        for (PhysicsRayTestResult hit : hits) {
            PhysicsCollisionObject pco = hit.getCollisionObject();
            for (Drop drop : drops) {
                if (drop.displayObject(pco)) {
                    selectDrop(drop);
                    return;
                }
            }
        }
        selectDrop(null);
    }

    /**
     * Apply an upward impulse to the selected drop.
     */
    private void popSelected() {
        if (selectedDrop != null) {
            float gravity = status.gravity();
            float deltaV = FastMath.sqrt(30f * gravity);
            selectedDrop.pop(deltaV);
        }
    }

    /**
     * Alter which drop is selected.
     *
     * @param newDrop the drop to select (or null for none)
     */
    private void selectDrop(Drop newDrop) {
        if (newDrop != selectedDrop) {
            selectedDrop = newDrop;
            setDebugMaterialsAll();
        }
    }

    /**
     * Update the debug materials of the specified collision object.
     */
    private void setDebugMaterial(PhysicsCollisionObject pco) {
        CollisionShape shape = pco.getCollisionShape();

        Material debugMaterial;
        if (selectedDrop != null && selectedDrop.displayObject(pco)) {
            debugMaterial = findMaterial("selected");

        } else if (status.isWireframe()) {
            debugMaterial = null;

        } else if (status.isChildColoring()
                && shape instanceof CompoundCollisionShape) {
            debugMaterial = BulletDebugAppState.enableChildColoring;

        } else {
            // Use the pre-selected lit material.
            debugMaterial = (Material) pco.getApplicationData();
        }

        pco.setDebugMaterial(debugMaterial);
    }
}
