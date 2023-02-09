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
import com.jme3.bullet.CollisionSpace;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.SoftPhysicsAppState;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.EmptyShape;
import com.jme3.bullet.collision.shapes.GImpactCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.MeshCollisionShape;
import com.jme3.bullet.collision.shapes.infos.ChildCollisionShape;
import com.jme3.bullet.debug.BulletDebugAppState;
import com.jme3.bullet.debug.DebugInitListener;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.font.Rectangle;
import com.jme3.input.CameraInput;
import com.jme3.input.KeyInput;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Triangle;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Limits;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Line;
import com.jme3.system.AppSettings;
import com.jme3.util.BufferUtils;
import java.util.ArrayList;
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
import jme3utilities.math.MyVector3f;
import jme3utilities.math.noise.Generator;
import jme3utilities.minie.test.common.PhysicsDemo;
import jme3utilities.minie.test.shape.ShapeGenerator;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.InputMode;
import jme3utilities.ui.Signals;

/**
 * Test/demonstrate splitting of rigid bodies.
 * <p>
 * Collision objects are rendered entirely by debug visualization.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class SplitDemo
        extends PhysicsDemo
        implements DebugInitListener {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(SplitDemo.class.getName());
    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName
            = SplitDemo.class.getSimpleName();
    // *************************************************************************
    // fields

    /**
     * AppState to manage the PhysicsSpace
     */
    private static BulletAppState bulletAppState;
    /**
     * angle between the normal of the splitting plane and default camera's "up"
     * vector (in radians, &ge;0, &lt;Pi)
     */
    private static float splitAngle = 0f;
    /**
     * visualize the splitting plane
     */
    private static Geometry splitterGeometry;
    /**
     * temporary storage for a 3x3 matrix
     */
    final private static Matrix3f tmpMatrix = new Matrix3f();
    /**
     * temporary storage for a Quaternion
     */
    final private static Quaternion tmpRotation = new Quaternion();
    /**
     * AppState to manage the status overlay
     */
    private static SplitDemoStatus status;
    /**
     * first screen location used to define the splitting plane (measured from
     * the lower left corner)
     */
    final private static Vector2f screen1 = new Vector2f();
    /**
     * 2nd screen location used to define the splitting plane (measured from the
     * lower left corner)
     */
    final private static Vector2f screen2 = new Vector2f();
    /**
     * temporary storage for a vector
     */
    final private static Vector3f tmpLocation = new Vector3f();
    /**
     * first world location used to define the splitting plane
     */
    final private static Vector3f world1 = new Vector3f();
    /**
     * 2nd world location used to define the splitting plane
     */
    final private static Vector3f world2 = new Vector3f();
    // *************************************************************************
    // constructors

    /**
     * Instantiate the SplitDemo application.
     */
    public SplitDemo() { // made explicit to avoid a warning from JDK 18 javadoc
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
     * Main entry point for the SplitDemo application.
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

        Application application = new SplitDemo();
        application.setSettings(settings);
        application.start();
    }

    /**
     * Return the inclination angle of the splitting plane.
     *
     * @return the angle between the plane normal and the default camera's "up"
     * vector (in radians, &ge;0, &lt;Pi)
     */
    static float splitAngle() {
        assert splitAngle >= 0f : splitAngle;
        assert splitAngle < FastMath.PI : splitAngle;

        return splitAngle;
    }
    // *************************************************************************
    // PhysicsDemo methods

    /**
     * Initialize this application.
     */
    @Override
    public void acorusInit() {
        status = new SplitDemoStatus();
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

        Integer maxDegree = renderer.getLimits().get(Limits.TextureAnisotropy);
        int degree = (maxDegree == null) ? 1 : Math.min(8, maxDegree);
        renderer.setDefaultAnisotropicFilter(degree);

        Line lineMesh = new Line(Vector3f.ZERO, Vector3f.ZERO);
        splitterGeometry = new Geometry("plane", lineMesh);
        Material splitter = MyAsset.createWireframeMaterial(
                assetManager, ColorRGBA.White);
        splitterGeometry.setMaterial(splitter);
        rootNode.attachChild(splitterGeometry);

        restartScenario();
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
        float topY = viewPortHeight - 88f - margin;
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

        ColorRGBA color = new ColorRGBA(0.2f, 0f, 0f, 1f);
        Material solid = MyAsset.createShinyMaterial(assetManager, color);
        solid.setFloat("Shininess", 15f);
        RenderState additional = solid.getAdditionalRenderState();
        additional.setFaceCullMode(RenderState.FaceCullMode.Off);
        registerMaterial("solid", solid);

        ColorRGBA gray = new ColorRGBA(0.2f, 0.2f, 0.2f, 1f);
        Material stat = MyAsset.createShinyMaterial(assetManager, gray);
        stat.setFloat("Shininess", 15f);
        additional = stat.getAdditionalRenderState();
        additional.setFaceCullMode(RenderState.FaceCullMode.Off);
        registerMaterial("stat", stat);
    }

    /**
     * Initialize the library of named collision shapes during startup.
     */
    @Override
    public void generateShapes() {
        super.generateShapes();
        CollisionShape shape;

        // "ankh" using manual decomposition
        String ankhPath = "CollisionShapes/ankh.j3o";
        shape = (CollisionShape) assetManager.loadAsset(ankhPath);
        registerShape("ankh", shape);

        // "banana" using manual decomposition
        String bananaPath = "CollisionShapes/banana.j3o";
        shape = (CollisionShape) assetManager.loadAsset(bananaPath);
        registerShape("banana", shape);

        // "barrel"
        String barrelPath = "CollisionShapes/barrel.j3o";
        shape = (CollisionShape) assetManager.loadAsset(barrelPath);
        shape.setScale(3f);
        registerShape("barrel", shape);

        // "candyDish"
        String candyDishPath = "Models/CandyDish/CandyDish.j3o";
        Node candyDishNode = (Node) assetManager.loadModel(candyDishPath);
        Geometry candyDishGeometry = (Geometry) candyDishNode.getChild(0);
        Mesh candyDishMesh = candyDishGeometry.getMesh();
        shape = new MeshCollisionShape(candyDishMesh);
        shape.setScale(0.5f);
        registerShape("candyDish", shape);

        // "duck" using V-HACD
        String duckPath = "CollisionShapes/duck.j3o";
        shape = (CollisionShape) assetManager.loadAsset(duckPath);
        shape.setScale(2f);
        registerShape("duck", shape);

        // "heart"
        String heartPath = "CollisionShapes/heart.j3o";
        shape = (CollisionShape) assetManager.loadAsset(heartPath);
        shape.setScale(1.2f);
        registerShape("heart", shape);

        // "horseshoe" using manual decomposition
        String horseshoePath = "CollisionShapes/horseshoe.j3o";
        shape = (CollisionShape) assetManager.loadAsset(horseshoePath);
        registerShape("horseshoe", shape);

        // "sword" using V-HACD
        String swordPath = "CollisionShapes/sword.j3o";
        shape = (CollisionShape) assetManager.loadAsset(swordPath);
        shape.setScale(5f);
        registerShape("sword", shape);

        // "teapot" using V-HACD and GImpact
        String teapotPath = "CollisionShapes/teapot.j3o";
        shape = (CollisionShape) assetManager.loadAsset(teapotPath);
        shape.setScale(3f);
        registerShape("teapot", shape);

        String teapotGiPath = "CollisionShapes/teapotGi.j3o";
        shape = (CollisionShape) assetManager.loadAsset(teapotGiPath);
        shape.setScale(3f);
        registerShape("teapotGi", shape);

        // letter shapes
        for (char character = 'A'; character <= 'Z'; ++character) {
            char[] array = {character};
            String glyphString = new String(array);
            String assetPath = String.format(
                    "CollisionShapes/glyphs/%s.j3o", glyphString);
            shape = (CollisionShape) assetManager.loadAsset(assetPath);
            registerShape(glyphString, shape);
        }

        // digit shapes
        for (char character = '0'; character <= '9'; ++character) {
            char[] array = {character};
            String glyphString = new String(array);
            String assetPath = String.format(
                    "CollisionShapes/glyphs/%s.j3o", glyphString);
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
        dim.bind(asDumpSpace, KeyInput.KEY_O);
        dim.bind(asDumpViewport, KeyInput.KEY_P);

        dim.bind("make splittable", KeyInput.KEY_TAB);

        dim.bind("next field", KeyInput.KEY_NUMPAD2);
        dim.bind("next value", KeyInput.KEY_EQUALS, KeyInput.KEY_NUMPAD6);

        dim.bind("previous field", KeyInput.KEY_NUMPAD8);
        dim.bind("previous value", KeyInput.KEY_MINUS, KeyInput.KEY_NUMPAD4);

        dim.bind("restart", KeyInput.KEY_NUMPAD5);

        dim.bindSignal(CameraInput.FLYCAM_LOWER, KeyInput.KEY_DOWN);
        dim.bindSignal(CameraInput.FLYCAM_RISE, KeyInput.KEY_UP);
        dim.bindSignal("orbitLeft", KeyInput.KEY_LEFT);
        dim.bindSignal("orbitRight", KeyInput.KEY_RIGHT);
        dim.bindSignal("rotatePlaneCcw", KeyInput.KEY_LBRACKET);
        dim.bindSignal("rotatePlaneCw", KeyInput.KEY_RBRACKET);

        dim.bind("split", KeyInput.KEY_RETURN, KeyInput.KEY_INSERT,
                KeyInput.KEY_NUMPAD0, KeyInput.KEY_SPACE);

        dim.bind(asToggleAabbs, KeyInput.KEY_APOSTROPHE);
        dim.bind("toggle childColor", KeyInput.KEY_COMMA);
        dim.bind(asToggleHelp, KeyInput.KEY_H);
        dim.bind(asTogglePause, KeyInput.KEY_PAUSE, KeyInput.KEY_PERIOD);
        dim.bind(asTogglePcoAxes, KeyInput.KEY_SEMICOLON);
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
                case "make splittable":
                    makeSplittableAll();
                    return;

                case "next field":
                    status.advanceSelectedField(+1);
                    return;
                case "next value":
                    status.advanceValue(+1);
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
                case "split":
                    splitAll(); // TODO do this on the physics thread
                    return;

                case "toggle childColor":
                    status.toggleChildColor();
                    setDebugMaterialsAll();
                    return;
                case "toggle wireframe":
                    status.toggleWireframe();
                    setDebugMaterialsAll();
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
        pco.setDebugMeshResolution(DebugShapeFactory.highResolution);
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

        Signals signals = getSignals();
        if (signals.test("rotatePlaneCcw")) {
            splitAngle += tpf;
        }
        if (signals.test("rotatePlaneCw")) {
            splitAngle -= tpf;
        }
        splitAngle = MyMath.modulo(splitAngle, FastMath.PI);

        float w = cam.getWidth();
        float h = cam.getHeight();
        if (Math.abs(splitAngle - FastMath.HALF_PI) < FastMath.QUARTER_PI) {
            // The plane is more vertical than horizontal.
            float cotangent = FastMath.tan(FastMath.HALF_PI - splitAngle);
            screen1.x = 0.5f * (w + h * cotangent);
            screen1.y = h;
            screen2.x = 0.5f * (w - h * cotangent);
            screen2.y = 0f;
        } else { // The plane is more horizontal than vertical.
            float tangent = FastMath.tan(splitAngle);
            screen1.x = w;
            screen1.y = 0.5f * (h + w * tangent);
            screen2.x = 0f;
            screen2.y = 0.5f * (h - w * tangent);
        }
        cam.getWorldCoordinates(screen1, 0.1f, world1);
        cam.getWorldCoordinates(screen2, 0.1f, world2);

        Line planeMesh = (Line) splitterGeometry.getMesh();
        planeMesh.updatePoints(world1, world2);
        splitterGeometry.setMesh(planeMesh); // to update the geometry's bounds
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
     * Add lighting to the specified scene.
     *
     * @param rootSpatial which scene (not null)
     */
    private static void addLighting(Spatial rootSpatial) {
        ColorRGBA ambientColor = new ColorRGBA(0.1f, 0.1f, 0.1f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        rootSpatial.addLight(ambient);
        ambient.setName("ambient");

        ColorRGBA directColor = new ColorRGBA(0.7f, 0.7f, 0.7f, 1f);
        Vector3f direction = new Vector3f(1f, -3f, -1f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction, directColor);
        rootSpatial.addLight(sun);
        sun.setName("sun");
    }

    /**
     * Create a rigid body with the specified shape and debug normals and add it
     * to the PhysicsSpace at the origin, with random rotation.
     *
     * @param shape the collision shape to use (not null)
     * @param debugMeshNormals how to generate normals for debug visualization
     * (not null)
     * @param mass the desired mass (0 or 1)
     */
    private void addRigidBody(
            CollisionShape shape, MeshNormals debugMeshNormals, float mass) {
        PhysicsRigidBody body = new PhysicsRigidBody(shape, mass);
        body.setDebugMeshNormals(debugMeshNormals);

        Generator random = getGenerator();
        random.nextVector3f(tmpLocation);
        body.setPhysicsLocation(tmpLocation);
        random.nextQuaternion(tmpRotation);
        body.setPhysicsRotation(tmpRotation);

        addCollisionObject(body);
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

        cam.setLocation(new Vector3f(0f, 0f, 6.8f));
        cam.setRotation(new Quaternion(0f, 1f, 0f, 0f));

        AppState orbitState
                = new CameraOrbitAppState(cam, "orbitLeft", "orbitRight");
        stateManager.attach(orbitState);
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        PhysicsBody.setDeactivationEnabled(false); // avoid a distraction

        bulletAppState = new SoftPhysicsAppState();
        bulletAppState.setDebugEnabled(true);
        bulletAppState.setDebugInitListener(this);
        stateManager.attach(bulletAppState);

        PhysicsSpace physicsSpace = getPhysicsSpace();
        physicsSpace.setGravity(Vector3f.ZERO);
    }

    /**
     * Ensure that all rigid bodies in the PhysicsSpace have splittable shapes.
     */
    private void makeSplittableAll() {
        PhysicsSpace space = getPhysicsSpace();
        Collection<PhysicsCollisionObject> allPcos = space.getPcoList();
        for (PhysicsCollisionObject pco : allPcos) {
            PhysicsRigidBody body = (PhysicsRigidBody) pco;
            makeSplittable(body);
        }
    }

    /**
     * Ensure that the specified rigid body has a splittable shape.
     *
     * @param body (not null)
     */
    private static void makeSplittable(PhysicsRigidBody body) {
        CollisionShape oldShape = body.getCollisionShape();
        CollisionShape splittableShape = oldShape.toSplittableShape();
        assert splittableShape.canSplit();
        body.setCollisionShape(splittableShape);
    }

    /**
     * Pseudo-randomly select the shape of a decimal digit.
     *
     * @return the pre-existing instance (not null)
     */
    private CollisionShape randomDigit() {
        Random random = getGenerator();
        char glyphChar = (char) ('0' + random.nextInt(10));
        String glyphString = Character.toString(glyphChar);
        CollisionShape result = findShape(glyphString);
        assert result != null : glyphChar;

        return result;
    }

    /**
     * Pseudo-randomly select the shape of an uppercase letter.
     *
     * @return the pre-existing instance (not null)
     */
    private CollisionShape randomLetter() {
        Random random = getGenerator();
        char glyphChar = (char) ('A' + random.nextInt(26));
        String glyphString = Character.toString(glyphChar);
        CollisionShape result = findShape(glyphString);
        assert result != null : glyphChar;

        return result;
    }

    /**
     * Restart the current scenario.
     */
    private void restartScenario() {
        PhysicsSpace physicsSpace = getPhysicsSpace();
        physicsSpace.destroy();
        assert physicsSpace.isEmpty();

        setUpShape();
    }

    /**
     * Update the debug materials of the specified collision object.
     *
     * @param pco the object to update (not null, modified)
     */
    private void setDebugMaterial(PhysicsCollisionObject pco) {
        CollisionShape shape = pco.getCollisionShape();

        Material debugMaterial;
        if (status.isWireframe()) {
            debugMaterial = null;

        } else if (status.isChildColoring()
                && shape instanceof CompoundCollisionShape) {
            debugMaterial = BulletDebugAppState.enableChildColoring;

        } else if (pco instanceof PhysicsRigidBody && pco.isStatic()) {
            // Use the shiny gray lit/shaded material for static bodies.
            debugMaterial = findMaterial("stat");

        } else { // Use the shiny red lit/shaded material.
            debugMaterial = findMaterial("solid");
        }

        pco.setDebugMaterial(debugMaterial);
    }

    /**
     * Update the debug materials of all collision objects.
     */
    private void setDebugMaterialsAll() {
        PhysicsSpace physicsSpace = getPhysicsSpace();
        for (PhysicsCollisionObject pco : physicsSpace.getPcoList()) {
            setDebugMaterial(pco);
        }
    }

    /**
     * Add a dynamic rigid body with the selected shape.
     */
    private void setUpShape() {
        ShapeGenerator random = getGenerator();
        float randomMass = random.nextInt(2);
        String shapeName = status.shapeName();

        CollisionShape shape;
        switch (shapeName) {
            case "ankh":
            case "duck":
            case "heart":
            case "horseshoe":
            case "sword":
            case "table":
            case "teapot":
            case "thumbTack":
            case "tray":
                shape = findShape(shapeName);
                addRigidBody(shape, MeshNormals.Facet, randomMass);
                break;

            case "banana":
            case "barbell":
            case "barrel":
            case "knucklebone":
            case "ladder":
            case "link":
            case "teapotGi":
            case "top":
                shape = findShape(shapeName);
                addRigidBody(shape, MeshNormals.Smooth, randomMass);
                break;

            case "bedOfNails":
            case "corner":
            case "roundedRectangle":
            case "triangle":
                shape = findShape(shapeName);
                addRigidBody(
                        shape, MeshNormals.Facet, PhysicsBody.massForStatic);
                break;

            case "box":
            case "frame":
            case "halfPipe":
            case "hull":
            case "iBeam":
            case "lidlessBox":
            case "platonic":
            case "prism":
            case "pyramid":
            case "star":
            case "tetrahedron":
            case "triangularFrame":
            case "trident":
            case "washer":
                shape = random.nextShape(shapeName);
                addRigidBody(shape, MeshNormals.Facet, randomMass);
                break;

            case "candyDish":
            case "dimples":
            case "smooth":
                shape = findShape(shapeName);
                addRigidBody(
                        shape, MeshNormals.Smooth, PhysicsBody.massForStatic);
                break;

            case "capsule":
            case "cone":
            case "cylinder":
            case "dome":
            case "football":
            case "multiSphere":
            case "snowman":
            case "torus":
                shape = random.nextShape(shapeName);
                addRigidBody(shape, MeshNormals.Smooth, randomMass);
                break;

            case "digit":
                shape = randomDigit();
                shape.setScale(0.5f);
                addRigidBody(shape, MeshNormals.Facet, randomMass);
                break;

            case "letter":
                shape = randomLetter();
                shape.setScale(0.5f);
                addRigidBody(shape, MeshNormals.Facet, randomMass);
                break;

            case "sphere":
                shape = random.nextShape(shapeName);
                addRigidBody(shape, MeshNormals.Sphere, randomMass);
                break;

            default:
                String message = "shapeName = " + MyString.quote(shapeName);
                throw new RuntimeException(message);
        }
    }

    /**
     * Split all rigid bodies in the PhysicsSpace.
     */
    private void splitAll() {
        Vector3f world3 = cam.getLocation(); // alias
        Triangle triangle = new Triangle(world1, world2, world3);

        PhysicsSpace space = getPhysicsSpace();
        Collection<PhysicsCollisionObject> allPcos = space.getPcoList();
        for (PhysicsCollisionObject pco : allPcos) {
            PhysicsRigidBody body = (PhysicsRigidBody) pco;
            splitBody(body, triangle);
        }
    }

    /**
     * Attempt to split the specified rigid body using the plane of the
     * specified triangle.
     *
     * @param oldBody (not null, added to the PhysicsSpace)
     * @param worldTriangle a triangle that defines the splitting plane (in
     * world coordinates, not null, unaffected)
     */
    private void splitBody(PhysicsRigidBody oldBody, Triangle worldTriangle) {
        CollisionShape originalShape = oldBody.getCollisionShape();
        CollisionShape splittableShape = originalShape.toSplittableShape();
        assert splittableShape.canSplit();
        if (splittableShape instanceof EmptyShape) {
            return; // Splitting an empty shape has no effect.
        }

        // Transform the triangle to the shape coordinate system.
        Transform shapeToWorld = oldBody.getTransform(null);
        if (splittableShape instanceof CompoundCollisionShape) {
            shapeToWorld.setScale(1f);
        } else {
            splittableShape.getScale(shapeToWorld.getScale());
        }
        Triangle shapeTriangle
                = MyMath.transformInverse(shapeToWorld, worldTriangle, null);

        CollisionShape[] shapes;
        float[] volumes;
        int[] signs;
        Vector3f[] locations;
        Vector3f worldNormal = worldTriangle.getNormal(); // alias

        if (splittableShape instanceof HullCollisionShape) {
            HullCollisionShape hullShape = (HullCollisionShape) splittableShape;
            ChildCollisionShape[] children = hullShape.split(shapeTriangle);
            assert children.length == 2 : children.length;
            if (children[0] == null || children[1] == null) {
                return; // The split plane didn't intersect the hull.
            }

            shapes = new CollisionShape[2];
            volumes = new float[2];
            signs = new int[2];
            locations = new Vector3f[2];
            for (int sideI = 0; sideI < 2; ++sideI) {
                shapes[sideI] = children[sideI].getShape();
                volumes[sideI] = shapes[sideI].scaledVolume();
                signs[sideI] = 2 * sideI - 1;

                Vector3f location = children[sideI].copyOffset(null);
                shapeToWorld.transformVector(location, location);
                locations[sideI] = location;
            }

        } else if (splittableShape instanceof CompoundCollisionShape) {
            CompoundCollisionShape compound
                    = (CompoundCollisionShape) splittableShape;
            CompoundCollisionShape[] compounds = compound.split(shapeTriangle);
            assert compounds.length == 2 : compounds.length;
            if (compounds[0] == null || compounds[1] == null) {
                return; // The split plane didn't intersect the compound shape.
            }
            /*
             * Enumerate the groups of connected children
             * on each side of the split plane.
             */
            List<CompoundCollisionShape> groupList = new ArrayList<>(4);
            List<Integer> signList = new ArrayList<>(4);
            CollisionSpace testSpace = getPhysicsSpace();
            for (int sideI = 0; sideI < 2; ++sideI) {
                ChildCollisionShape[] children
                        = compounds[sideI].listChildren();
                int numChildren = children.length;
                int[] map = new int[numChildren];
                int numGroups = compounds[sideI].countGroups(testSpace, map);
                int sign = 2 * sideI - 1;

                for (int groupI = 0; groupI < numGroups; ++groupI) {
                    CompoundCollisionShape newGroup
                            = new CompoundCollisionShape(numChildren);
                    for (int childI = 0; childI < numChildren; ++childI) {
                        if (map[childI] == groupI) {
                            ChildCollisionShape child = children[childI];
                            CollisionShape baseShape = child.getShape();
                            child.copyOffset(tmpLocation);
                            child.copyRotationMatrix(tmpMatrix);
                            newGroup.addChildShape(
                                    baseShape, tmpLocation, tmpMatrix);
                        }
                    }
                    groupList.add(newGroup);
                    signList.add(sign);
                }
            }

            int numGroups = groupList.size();
            assert signList.size() == numGroups;
            shapes = new CollisionShape[numGroups];
            volumes = new float[numGroups];
            signs = new int[numGroups];
            locations = new Vector3f[numGroups];

            for (int groupI = 0; groupI < numGroups; ++groupI) {
                CompoundCollisionShape shape = groupList.get(groupI);
                shapes[groupI] = shape;
                volumes[groupI] = shape.scaledVolume();
                signs[groupI] = signList.get(groupI);
                /*
                 * Translate each compound so its AABB is centered at (0,0,0)
                 * in its shape coordinates.
                 */
                Vector3f location = shape.aabbCenter(null);
                Vector3f offset = location.negate();
                shape.translate(offset);
                shapeToWorld.setScale(1f);
                shapeToWorld.transformVector(location, location);
                locations[groupI] = location;
            }

        } else if (splittableShape instanceof GImpactCollisionShape) {
            GImpactCollisionShape gi = (GImpactCollisionShape) splittableShape;
            ChildCollisionShape[] children = gi.split(shapeTriangle);
            assert children.length == 2 : children.length;
            if (children[0] == null || children[1] == null) {
                return; // The split plane didn't intersect the GImpact shape.
            }

            shapes = new CollisionShape[2];
            volumes = new float[2];
            signs = new int[2];
            locations = new Vector3f[2];
            for (int sideI = 0; sideI < 2; ++sideI) {
                shapes[sideI] = children[sideI].getShape();
                volumes[sideI] = 1f; // TODO calculate area
                signs[sideI] = 2 * sideI - 1;

                Vector3f location = children[sideI].copyOffset(null);
                shapeToWorld.transformVector(location, location);
                locations[sideI] = location;
            }

        } else if (splittableShape instanceof MeshCollisionShape) {
            assert oldBody.isStatic();
            MeshCollisionShape mesh = (MeshCollisionShape) splittableShape;
            shapes = mesh.split(shapeTriangle);
            assert shapes.length == 2 : shapes.length;
            if (shapes[0] == null || shapes[1] == null) {
                return; // The split plane didn't intersect the mesh shape.
            }

            volumes = new float[2];
            signs = new int[2];
            locations = new Vector3f[2];
            for (int sideI = 0; sideI < 2; ++sideI) {
                volumes[sideI] = 0f; // unused
                signs[sideI] = 2 * sideI - 1;
                locations[sideI] = oldBody.getPhysicsLocation(null);
            }

        } else { // TODO handle simplex n<=2
            logger.log(Level.WARNING, "Shape not split:  {0}", originalShape);
            return;
        }

        splitBody(oldBody, worldNormal, shapes, volumes, signs, locations);
    }

    /**
     * Split the specified rigid body using the specified shapes.
     *
     * @param oldBody (not null, added to the PhysicsSpace)
     * @param worldNormal the normal of the splitting plane (in world
     * coordinates, not null, unaffected)
     * @param shapes the shapes to use (length&ge;2, all not null)
     * @param sizes the estimated relative size of each shape (all &gt;0)
     * @param signs -1 &rarr; shape is on the negative side of the splitting
     * plane, +1 &rarr; on the positive side
     * @param locations the center location of each shape (in physics-space
     * coordinates, all not null)
     */
    private void splitBody(PhysicsRigidBody oldBody, Vector3f worldNormal,
            CollisionShape[] shapes, float[] sizes, int[] signs,
            Vector3f[] locations) {
        int numShapes = shapes.length;
        assert numShapes >= 2 : numShapes;
        assert sizes.length == numShapes : sizes.length;
        assert signs.length == numShapes : signs.length;
        assert locations.length == numShapes : locations.length;

        // Tweak the locations to create some separation.
        boolean isDynamic = oldBody.isDynamic();
        float deltaX = isDynamic ? 0.04f : 0.1f;
        for (int shapeI = 0; shapeI < numShapes; ++shapeI) {
            float factor = signs[shapeI] * deltaX;
            MyVector3f.accumulateScaled(locations[shapeI], worldNormal, factor);
        }

        float[] masses = new float[numShapes];
        Vector3f v;
        Vector3f w;
        Vector3f[] velocities = new Vector3f[numShapes];
        if (isDynamic) {
            // Tweak the linear velocities to enhance the separation.
            float deltaV = 0.04f;
            v = oldBody.getLinearVelocity(null);
            for (int shapeI = 0; shapeI < numShapes; ++shapeI) {
                velocities[shapeI] = v.clone();
                float multiplier = signs[shapeI] * deltaV;
                MyVector3f.accumulateScaled(
                        velocities[shapeI], worldNormal, multiplier);
            }

            // Calculate the masses.
            float totalSize = 0f;
            for (int shapeI = 0; shapeI < numShapes; ++shapeI) {
                totalSize += sizes[shapeI];
            }
            assert totalSize > 0f : totalSize;
            float totalMass = oldBody.getMass();
            for (int shapeI = 0; shapeI < numShapes; ++shapeI) {
                masses[shapeI] = totalMass * sizes[shapeI] / totalSize;
                assert masses[shapeI] > 0f : masses[shapeI];
            }

            w = oldBody.getAngularVelocity(null);

        } else {
            for (int shapeI = 0; shapeI < numShapes; ++shapeI) {
                masses[shapeI] = PhysicsBody.massForStatic;
            }
            w = null;
        }

        MeshNormals debugMeshNormals = oldBody.debugMeshNormals();
        Quaternion orientation = oldBody.getPhysicsRotation(null);

        PhysicsSpace space = getPhysicsSpace();
        space.removeCollisionObject(oldBody);

        for (int shapeI = 0; shapeI < numShapes; ++shapeI) {
            PhysicsRigidBody body
                    = new PhysicsRigidBody(shapes[shapeI], masses[shapeI]);
            body.setDebugMeshNormals(debugMeshNormals);
            body.setPhysicsLocation(locations[shapeI]);
            body.setPhysicsRotation(orientation);
            if (isDynamic) {
                body.setAngularVelocity(w);
                body.setLinearVelocity(velocities[shapeI]);
            }
            addCollisionObject(body);
        }
    }
}
