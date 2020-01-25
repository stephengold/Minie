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
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.PhysicsRayTestResult;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.ConeCollisionShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.MeshCollisionShape;
import com.jme3.bullet.collision.shapes.PlaneCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.collision.shapes.infos.DebugMeshNormals;
import com.jme3.bullet.debug.DebugInitListener;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.font.BitmapText;
import com.jme3.font.Rectangle;
import com.jme3.input.CameraInput;
import com.jme3.input.KeyInput;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Matrix3f;
import com.jme3.math.Plane;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector2f;
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
import java.nio.FloatBuffer;
import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Collection;
import java.util.Deque;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Misc;
import jme3utilities.MyAsset;
import jme3utilities.MyCamera;
import jme3utilities.MyString;
import jme3utilities.math.MyArray;
import jme3utilities.math.MyMath;
import jme3utilities.math.noise.Generator;
import jme3utilities.mesh.Prism;
import jme3utilities.minie.DumpFlags;
import jme3utilities.minie.FilterAll;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.minie.test.shape.MinieTestShapes;
import jme3utilities.ui.ActionApplication;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.HelpUtils;
import jme3utilities.ui.InputMode;
import jme3utilities.ui.Signals;

/**
 * Test/demonstrate various collision shapes by dropping gems (dynamic rigid
 * bodies) onto a platform (static rigid body).
 * <p>
 * Collision objects are rendered entirely by debug visualization.
 * <p>
 * Seen in the November 2018 demo video:
 * https://www.youtube.com/watch?v=OS2zjB01c6E
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class DropTest
        extends ActionApplication
        implements DebugInitListener {
    // *************************************************************************
    // constants and loggers

    /**
     * upper limit on the number of gems
     */
    final private static int maxNumGems = 80;
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
    /**
     * list of gem-shape names, in ascending lexicographic order
     */
    final private static String[] gemShapeNames = {
        "barbell", "box", "capsule", "chair", "cone", "cylinder", "duck",
        "football", "funnyHammer", "hammer", "heart", "hull", "knucklebone",
        "ladder", "letter", "multiSphere", "platonic", "sphere", "star",
        "sword", "teapot", "tetrahedron", "top", "torus"
    };
    // *************************************************************************
    // fields

    /**
     * text displayed in the upper-left corner of the GUI node
     */
    final private BitmapText[] statusLines = new BitmapText[3];
    /**
     * AppState to manage the PhysicsSpace
     */
    final private BulletAppState bulletAppState = new BulletAppState();
    /**
     * shape for the next gem
     */
    private CollisionShape gemShape;
    /**
     * current gems, in order of creation
     */
    final private Deque<PhysicsRigidBody> gems = new ArrayDeque<>(maxNumGems);
    /**
     * filter to control visualization of axis-aligned bounding boxes
     */
    private FilterAll bbFilter;
    /**
     * filter to control visualization of swept spheres
     */
    private FilterAll ssFilter;
    /**
     * damping fraction for all gems (&ge;0, &le;1)
     */
    private float damping = 0.6f;
    /**
     * friction coefficient for all rigid bodies (&ge;0)
     */
    private float friction = 0.5f;
    /**
     * enhanced pseudo-random generator
     */
    final private Generator random = new Generator();
    /**
     * map names to collision shapes
     */
    final private Map<String, CollisionShape> namedShapes = new TreeMap<>();
    /**
     * materials to visualize gems
     */
    final private Material gemMaterials[] = new Material[4];
    /**
     * single-sided green material to visualize the platform
     */
    private Material greenMaterial;
    /**
     * GUI node for displaying hotkey help/hints
     */
    private Node helpNode;
    /**
     * dump debugging information to System.out
     */
    final private PhysicsDumper dumper = new PhysicsDumper();
    /**
     * selected gem, or null if none
     */
    private PhysicsRigidBody selectedGem = null;
    /**
     * space for physics simulation
     */
    private PhysicsSpace physicsSpace;
    /**
     * name of the shape of the platform
     */
    private String platformName = "box";
    /**
     * name of the shape for the next gem
     */
    private String shapeName = "multiSphere";
    /**
     * local inverse inertial vector for new gems (or null)
     */
    private Vector3f gemInverseInertia = null;
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the DropTest application.
     *
     * @param ignored array of command-line arguments (not null)
     */
    public static void main(String[] ignored) {
        /*
         * Mute the chatty loggers in certain packages.
         */
        Misc.setLoggingLevels(Level.WARNING);
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
    // *************************************************************************
    // ActionApplication methods

    /**
     * Initialize this application.
     */
    @Override
    public void actionInitializeApplication() {
        configureCamera();
        configureDumper();
        generateMaterials();
        configurePhysics();
        assert MyArray.isSorted(gemShapeNames);

        ColorRGBA bgColor = new ColorRGBA(0.1f, 0.2f, 0.4f, 1f);
        viewPort.setBackgroundColor(bgColor);

        addStatusLines();
        addAPlatform();
        addAGem();
    }

    /**
     * Add application-specific hotkey bindings and override existing ones.
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        dim.bind("add", KeyInput.KEY_INSERT);

        dim.bind("delete last", KeyInput.KEY_BACK);
        dim.bind("delete selected", KeyInput.KEY_DELETE);

        dim.bind("dump physicsSpace", KeyInput.KEY_O);
        dim.bind("dump viewport", KeyInput.KEY_P);

        dim.bind("less damping", KeyInput.KEY_B);
        dim.bind("less friction", KeyInput.KEY_V);

        dim.bind("more damping", KeyInput.KEY_G);
        dim.bind("more friction", KeyInput.KEY_F);

        dim.bind("next value", KeyInput.KEY_EQUALS);
        dim.bind("next value", KeyInput.KEY_NUMPAD6);

        dim.bind("platform bedOfNails", KeyInput.KEY_0);
        dim.bind("platform box", KeyInput.KEY_3);
        dim.bind("platform candyDish", KeyInput.KEY_5);
        dim.bind("platform cone", KeyInput.KEY_4);
        dim.bind("platform cylinder", KeyInput.KEY_6);
        dim.bind("platform hull", KeyInput.KEY_2);
        dim.bind("platform plane", KeyInput.KEY_8);
        dim.bind("platform roundedRectangle", KeyInput.KEY_F2);
        dim.bind("platform smooth", KeyInput.KEY_1);
        dim.bind("platform tray", KeyInput.KEY_9);
        dim.bind("platform triangle", KeyInput.KEY_7);

        dim.bind("previous value", KeyInput.KEY_MINUS);
        dim.bind("previous value", KeyInput.KEY_NUMPAD4);

        dim.bind("select gem", "RMB");
        dim.bind("select gem", KeyInput.KEY_R);

        dim.bind("signal " + CameraInput.FLYCAM_LOWER, KeyInput.KEY_DOWN);
        dim.bind("signal " + CameraInput.FLYCAM_RISE, KeyInput.KEY_UP);
        dim.bind("signal orbitLeft", KeyInput.KEY_LEFT);
        dim.bind("signal orbitRight", KeyInput.KEY_RIGHT);
        dim.bind("signal shower", KeyInput.KEY_I);

        dim.bind("toggle aabb", KeyInput.KEY_APOSTROPHE);
        dim.bind("toggle axes", KeyInput.KEY_SEMICOLON);
        dim.bind("toggle help", KeyInput.KEY_H);
        dim.bind("toggle pause", KeyInput.KEY_PAUSE);
        dim.bind("toggle pause", KeyInput.KEY_PERIOD);
        dim.bind("toggle spheres", KeyInput.KEY_L);

        float x = 10f;
        float y = cam.getHeight() - 80f;
        float width = cam.getWidth() - 20f;
        float height = cam.getHeight() - 20f;
        Rectangle rectangle = new Rectangle(x, y, width, height);

        float space = 20f;
        helpNode = HelpUtils.buildNode(dim, rectangle, guiFont, space);
        guiNode.attachChild(helpNode);
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
                    addAGem();
                    return;

                case "delete last":
                    deleteLast();
                    return;
                case "delete selected":
                    deleteSelected();
                    return;

                case "dump physicsSpace":
                    dumper.dump(physicsSpace);
                    return;
                case "dump scenes":
                    dumper.dump(renderManager);
                    return;
                case "dump viewport":
                    dumper.dump(viewPort);
                    return;

                case "less damping":
                    incrementDamping(-0.1f);
                    return;

                case "less friction":
                    multiplyFriction(0.5f);
                    return;

                case "more damping":
                    incrementDamping(0.1f);
                    return;

                case "more friction":
                    multiplyFriction(2f);
                    return;

                case "next value":
                    advanceGemShape(+1);
                    return;
                case "previous value":
                    advanceGemShape(-1);
                    return;

                case "select gem":
                    pickGem();
                    return;

                case "toggle aabb":
                    toggleAabb();
                    return;
                case "toggle axes":
                    toggleAxes();
                    return;
                case "toggle help":
                    toggleHelp();
                    return;
                case "toggle pause":
                    togglePause();
                    return;
                case "toggle spheres":
                    toggleSpheres();
                    return;
            }

            String[] words = actionString.split(" ");
            if (words.length >= 2 && "platform".equals(words[0])) {
                platformName = words[1];
                restartTest();
                return;
            } else if (words.length >= 2 && "shape".equals(words[0])) {
                shapeName = words[1];
                return;
            }
        }
        super.onAction(actionString, ongoing, tpf);
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
        if (signals.test("shower")) {
            addAGem();
        }

        updateStatusLines();
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
     * Add a gem (dynamic rigid body) to the PhysicsSpace.
     */
    private void addAGem() {
        if (gems.size() >= maxNumGems) {
            return; // too many gems
        }

        gemInverseInertia = null;
        DebugMeshNormals debugMeshNormals;
        switch (shapeName) {
            case "barbell":
            case "football":
            case "knucklebone":
            case "ladder":
            case "top":
            case "torus":
                gemShape = namedShapes.get(shapeName);
                debugMeshNormals = DebugMeshNormals.Smooth;
                break;

            case "box":
                randomBox();
                debugMeshNormals = DebugMeshNormals.Facet;
                break;

            case "capsule":
                randomCapsule();
                debugMeshNormals = DebugMeshNormals.Smooth;
                break;

            case "chair":
                gemShape = namedShapes.get("chair");
                gemInverseInertia = MinieTestShapes.chairInverseInertia;
                debugMeshNormals = DebugMeshNormals.Facet;
                break;

            case "cone":
                randomCone();
                debugMeshNormals = DebugMeshNormals.Smooth;
                break;

            case "cylinder":
                randomCylinder();
                debugMeshNormals = DebugMeshNormals.Smooth;
                break;

            case "duck":
            case "heart":
            case "sword":
            case "teapot":
                gemShape = namedShapes.get(shapeName);
                debugMeshNormals = DebugMeshNormals.Facet;
                break;

            case "funnyHammer":
                randomHammer(false);
                debugMeshNormals = DebugMeshNormals.Smooth;
                break;

            case "hammer":
                randomHammer(true);
                debugMeshNormals = DebugMeshNormals.Smooth;
                break;

            case "hull":
                gemShape = MinieTestShapes.randomHull(random);
                debugMeshNormals = DebugMeshNormals.Facet;
                break;

            case "letter":
                randomLetter();
                debugMeshNormals = DebugMeshNormals.Facet;
                break;

            case "multiSphere":
                gemShape = MinieTestShapes.randomMultiSphere(random);
                debugMeshNormals = DebugMeshNormals.Smooth;
                break;

            case "platonic":
                gemShape = MinieTestShapes.randomPlatonic(random);
                debugMeshNormals = DebugMeshNormals.Facet;
                break;

            case "sphere":
                randomSphere();
                debugMeshNormals = DebugMeshNormals.Smooth;
                break;

            case "star":
                gemShape = MinieTestShapes.randomStar(random);
                debugMeshNormals = DebugMeshNormals.Facet;
                break;

            case "tetrahedron":
                gemShape = MinieTestShapes.randomTetrahedron(random);
                debugMeshNormals = DebugMeshNormals.Facet;
                break;

            default:
                String message = "shapeName = " + MyString.quote(shapeName);
                throw new RuntimeException(message);
        }

        Vector3f startLocation = random.nextVector3f();
        startLocation.multLocal(2.5f, 5f, 2.5f);
        startLocation.y += 20f;

        Quaternion startOrientation = random.nextQuaternion();

        Material debugMaterial = (Material) random.pick(gemMaterials);

        float mass = 1f;
        PhysicsRigidBody body = new PhysicsRigidBody(gemShape, mass);
        body.setCcdMotionThreshold(5f);
        float gemRadius = gemShape.maxRadius();
        body.setCcdSweptSphereRadius(gemRadius);
        body.setDamping(damping, damping);
        body.setDebugMaterial(debugMaterial);
        body.setDebugMeshNormals(debugMeshNormals);
        body.setDebugMeshResolution(DebugShapeFactory.highResolution);
        body.setFriction(friction);
        body.setPhysicsLocation(startLocation);
        body.setPhysicsRotation(startOrientation);
        if (gemInverseInertia != null) {
            body.setInverseInertiaLocal(gemInverseInertia);
        }
        body.setUserObject(debugMaterial);

        physicsSpace.add(body);
        gems.addLast(body);
    }

    /**
     * Add a platform (large, static rigid body) to the PhysicsSpace.
     */
    private void addAPlatform() {
        switch (platformName) {
            case "bedOfNails":
            case "tray":
            case "triangle":
                addNamedPlatform(DebugMeshNormals.Facet);
                break;

            case "box":
                addBoxPlatform();
                break;

            case "candyDish":
            case "smooth":
                addNamedPlatform(DebugMeshNormals.Smooth);
                break;

            case "cone":
                addConePlatform();
                break;

            case "cylinder":
                addCylinderPlatform();
                break;

            case "hull":
                addHullPlatform();
                break;

            case "plane":
                addPlanePlatform();
                break;

            case "roundedRectangle":
                addRoundedRectangle();
                break;

            default:
                String message
                        = "platformName = " + MyString.quote(platformName);
                throw new RuntimeException(message);
        }
    }

    /**
     * Add a large, static box to the PhysicsSpace, to serve as a platform.
     */
    private void addBoxPlatform() {
        float halfExtent = 20f;
        CollisionShape shape = new BoxCollisionShape(halfExtent);
        float mass = PhysicsRigidBody.massForStatic;
        PhysicsRigidBody body = new PhysicsRigidBody(shape, mass);

        body.setDebugMeshNormals(DebugMeshNormals.Facet);
        body.setPhysicsLocation(new Vector3f(0f, -halfExtent, 0f));
        makePlatform(body);
    }

    /**
     * Add a large, downward-pointing, static cone to the PhysicsSpace, to serve
     * as a platform.
     */
    private void addConePlatform() {
        float radius = 20f;
        float height = 2f * radius;
        ConeCollisionShape shape = new ConeCollisionShape(radius, height);

        float mass = PhysicsRigidBody.massForStatic;
        PhysicsRigidBody body = new PhysicsRigidBody(shape, mass);

        body.setDebugMeshNormals(DebugMeshNormals.Smooth);
        body.setDebugMeshResolution(DebugShapeFactory.highResolution);
        body.setPhysicsLocation(new Vector3f(0f, -radius, 0f));
        /*
         * Rotate the cone 180 degrees around the X axis
         * so that it points downward instead of upward.
         */
        Quaternion orientation = new Quaternion();
        orientation.fromAngles(FastMath.PI, 0f, 0f);
        body.setPhysicsRotation(orientation);

        makePlatform(body);
    }

    /**
     * Add a large, static cylinder to the PhysicsSpace, to serve as a platform.
     */
    private void addCylinderPlatform() {
        float radius = 20f;
        float thickness = 50f;
        CylinderCollisionShape shape = new CylinderCollisionShape(radius,
                thickness, PhysicsSpace.AXIS_Y);

        float mass = PhysicsRigidBody.massForStatic;
        PhysicsRigidBody body = new PhysicsRigidBody(shape, mass);

        body.setDebugMeshNormals(DebugMeshNormals.Smooth);
        body.setDebugMeshResolution(DebugShapeFactory.highResolution);
        body.setPhysicsLocation(new Vector3f(0f, -0.5f * thickness, 0f));

        makePlatform(body);
    }

    /**
     * Add a large, static petagonal prism shape to the PhysicsSpace, to serve
     * as a platform.
     */
    private void addHullPlatform() {
        float radius = 20f;
        float thickness = 5f;
        boolean normals = false;
        Mesh mesh = new Prism(5, radius, thickness, normals);
        HullCollisionShape shape = new HullCollisionShape(mesh);

        float mass = PhysicsRigidBody.massForStatic;
        PhysicsRigidBody body = new PhysicsRigidBody(shape, mass);

        body.setDebugMeshNormals(DebugMeshNormals.Facet);
        body.setPhysicsLocation(new Vector3f(0f, -0.5f * thickness, 0f));
        makePlatform(body);
    }

    /**
     * Add lighting and shadows to the specified scene.
     */
    private void addLighting(Spatial rootSpatial) {
        ColorRGBA ambientColor = new ColorRGBA(0.05f, 0.05f, 0.05f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        rootSpatial.addLight(ambient);

        ColorRGBA directColor = new ColorRGBA(0.7f, 0.7f, 0.7f, 1f);
        Vector3f direction = new Vector3f(1f, -3f, -1f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction, directColor);
        rootSpatial.addLight(sun);

        rootSpatial.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);

        DirectionalLightShadowRenderer dlsr
                = new DirectionalLightShadowRenderer(assetManager, 2_048, 3);
        dlsr.setEdgeFilteringMode(EdgeFilteringMode.PCFPOISSON);
        dlsr.setEdgesThickness(5);
        dlsr.setLight(sun);
        dlsr.setShadowIntensity(0.5f);
        viewPort.addProcessor(dlsr);
    }

    /**
     * Add a named static shape to the PhysicsSpace, to serve as a platform.
     */
    private void addNamedPlatform(DebugMeshNormals normals) {
        CollisionShape shape = namedShapes.get(platformName);
        float mass = PhysicsRigidBody.massForStatic;
        PhysicsRigidBody body = new PhysicsRigidBody(shape, mass);

        body.setDebugMeshNormals(normals);
        makePlatform(body);
    }

    /**
     * Add a static plane to the PhysicsSpace, to serve as a platform.
     */
    private void addPlanePlatform() {
        Plane plane = new Plane(Vector3f.UNIT_Y, 0f);
        PlaneCollisionShape shape = new PlaneCollisionShape(plane);

        float mass = PhysicsRigidBody.massForStatic;
        PhysicsRigidBody body = new PhysicsRigidBody(shape, mass);

        body.setDebugMeshNormals(DebugMeshNormals.Facet);
        makePlatform(body);
    }

    /**
     * Add a rounded rectangle to the PhysicsSpace, to serve as a platform.
     */
    private void addRoundedRectangle() {
        CollisionShape shape = namedShapes.get(platformName);
        float mass = PhysicsRigidBody.massForStatic;
        PhysicsRigidBody body = new PhysicsRigidBody(shape, mass);

        body.setDebugMeshNormals(DebugMeshNormals.Facet);
        Quaternion rotation = new Quaternion();
        rotation.fromAngles(FastMath.HALF_PI, 0f, 0f);
        body.setPhysicsRotation(rotation);
        makePlatform(body);
    }

    /**
     * Add status lines to the GUI.
     */
    private void addStatusLines() {
        for (int lineIndex = 0; lineIndex < statusLines.length; ++lineIndex) {
            statusLines[lineIndex] = new BitmapText(guiFont, false);
            float y = cam.getHeight() - 20f * lineIndex;
            statusLines[lineIndex].setLocalTranslation(0f, y, 0f);
            guiNode.attachChild(statusLines[lineIndex]);
        }
    }

    /**
     * Advance the gem-shape selection by the specified amount.
     *
     * @param amount the number of shapes to advance the selection
     */
    private void advanceGemShape(int amount) {
        int index = Arrays.binarySearch(gemShapeNames, shapeName);
        if (index < 0) {
            shapeName = gemShapeNames[0];
        } else {
            assert gemShapeNames[index].equals(shapeName);
            index = MyMath.modulo(index + amount, gemShapeNames.length);
            shapeName = gemShapeNames[index];
        }
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

        cam.setLocation(new Vector3f(0f, 20f, 40f));
        cam.setRotation(new Quaternion(0f, 0.9649f, -0.263f, 0f));

        CameraOrbitAppState orbitState
                = new CameraOrbitAppState(cam, "orbitLeft", "orbitRight");
        stateManager.attach(orbitState);
    }

    /**
     * Configure the PhysicsDumper during startup.
     */
    private void configureDumper() {
        dumper.setEnabled(DumpFlags.MatParams, true);
        dumper.setEnabled(DumpFlags.ShadowModes, true);
        dumper.setEnabled(DumpFlags.Transforms, true);
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        bulletAppState.setDebugEnabled(true);
        bulletAppState.setDebugInitListener(this);
        stateManager.attach(bulletAppState);

        physicsSpace = bulletAppState.getPhysicsSpace();
        physicsSpace.setGravity(new Vector3f(0f, -30f, 0f));

        DebugShapeFactory.setIndexBuffers(true);
        generateShapes();
    }

    /**
     * Delete the most recently added gem.
     */
    private void deleteLast() {
        PhysicsRigidBody latestGem = gems.peekLast();
        if (latestGem != null) {
            physicsSpace.remove(latestGem);
            if (latestGem == selectedGem) {
                selectGem(null);
            }
            gems.removeLast();
            for (PhysicsRigidBody gem : gems) {
                gem.activate();
            }
        }
    }

    /**
     * Delete the selected gem, if any.
     */
    private void deleteSelected() {
        if (selectedGem != null) {
            physicsSpace.remove(selectedGem);
            selectGem(null);
            gems.remove(selectedGem);
            for (PhysicsRigidBody gem : gems) {
                gem.activate();
            }
        }
    }

    /**
     * Initialize materials during startup.
     */
    private void generateMaterials() {
        ColorRGBA green = new ColorRGBA(0f, 0.12f, 0f, 1f);
        greenMaterial = MyAsset.createShadedMaterial(assetManager, green);
        greenMaterial.setName("green");

        ColorRGBA gemColors[] = new ColorRGBA[gemMaterials.length];
        gemColors[0] = new ColorRGBA(0.2f, 0f, 0f, 1f); // ruby
        gemColors[1] = new ColorRGBA(0f, 0.07f, 0f, 1f); // emerald
        gemColors[2] = new ColorRGBA(0f, 0f, 0.3f, 1f); // sapphire
        gemColors[3] = new ColorRGBA(0.2f, 0.1f, 0f, 1f); // topaz

        for (int i = 0; i < gemMaterials.length; ++i) {
            ColorRGBA color = gemColors[i];
            gemMaterials[i]
                    = MyAsset.createShinyMaterial(assetManager, color);
            gemMaterials[i].setFloat("Shininess", 15f);
        }
    }

    /**
     * Initialize the collection of named collision shapes during startup.
     */
    private void generateShapes() {
        /*
         * "barbell", "bedOfNails", "chair", "football", "knucklebone",
         * "ladder", "roundedRectangle", "smooth", "top", "torus", "tray",
         * and "triangle" are generated at runtime
         */
        MinieTestShapes.addShapes(namedShapes);
        /*
         * letter shapes
         */
        CollisionShape shape;
        for (char character = 'A'; character <= 'Z'; ++character) {
            char[] array = new char[]{character};
            String glyphString = new String(array);
            String assetPath = String.format("CollisionShapes/glyphs/%s.j3o",
                    glyphString);
            shape = (CollisionShape) assetManager.loadAsset(assetPath);
            namedShapes.put(glyphString, shape);
        }
        /*
         * "candyDish"
         */
        String candyDishPath = "Models/CandyDish/CandyDish.j3o";
        Node candyDishNode = (Node) assetManager.loadModel(candyDishPath);
        Geometry candyDishGeometry = (Geometry) candyDishNode.getChild(0);
        Mesh candyDishMesh = candyDishGeometry.getMesh();
        shape = new MeshCollisionShape(candyDishMesh);
        shape.setScale(5f);
        namedShapes.put("candyDish", shape);
        /*
         * "duck" using V-HACD
         */
        String duckPath = "CollisionShapes/duck.j3o";
        shape = (CollisionShape) assetManager.loadAsset(duckPath);
        shape.setScale(2f);
        namedShapes.put("duck", shape);
        /*
         * "heart"
         */
        String heartPath = "CollisionShapes/heart.j3o";
        shape = (CollisionShape) assetManager.loadAsset(heartPath);
        shape.setScale(1.5f);
        namedShapes.put("heart", shape);
        /*
         * "sword" using V-HACD
         */
        String swordPath = "CollisionShapes/sword.j3o";
        shape = (CollisionShape) assetManager.loadAsset(swordPath);
        shape.setScale(5f);
        namedShapes.put("sword", shape);
        /*
         * "teapot" using V-HACD
         */
        String teapotPath = "CollisionShapes/teapot.j3o";
        shape = (CollisionShape) assetManager.loadAsset(teapotPath);
        shape.setScale(3f);
        namedShapes.put("teapot", shape);
    }

    /**
     * Alter the damping fractions for all gems.
     *
     * @param increment the amount to increase the fraction (may be negative)
     */
    private void incrementDamping(float increment) {
        float newDamping = FastMath.clamp(damping + increment, 0f, 1f);
        if (newDamping != damping) {
            damping = newDamping;
            for (PhysicsRigidBody gem : gems) {
                gem.setDamping(damping, damping);
            }
        }
    }

    /**
     * Add the specified platform body to the PhysicsSpace.
     */
    private void makePlatform(PhysicsBody body) {
        body.setDebugMaterial(greenMaterial);
        body.setFriction(friction);
        physicsSpace.add(body);
    }

    /**
     * Alter the friction coefficients for all rigid bodies.
     *
     * @param factor the factor to increase the coefficient (&gt;0)
     */
    private void multiplyFriction(float factor) {
        assert factor > 0f : factor;

        friction *= factor;
        for (PhysicsRigidBody body : physicsSpace.getRigidBodyList()) {
            body.setFriction(friction);
        }
    }

    /**
     * Cast a physics ray from the cursor and select the nearest gem in the
     * result.
     */
    private void pickGem() {
        Vector2f screenXY = inputManager.getCursorPosition();
        Vector3f from = cam.getWorldCoordinates(screenXY, 0f);
        Vector3f to = cam.getWorldCoordinates(screenXY, 1f);
        List<PhysicsRayTestResult> hits = physicsSpace.rayTest(from, to);

        for (PhysicsRayTestResult hit : hits) {
            PhysicsCollisionObject pco = hit.getCollisionObject();
            if (pco instanceof PhysicsRigidBody) {
                PhysicsRigidBody body = (PhysicsRigidBody) pco;
                if (gems.contains(body)) {
                    selectGem(body);
                    return;
                }
            }
        }
        selectGem(null);
    }

    /**
     * Generate a box shape with random extents.
     */
    private void randomBox() {
        float rx = random.nextFloat(0.5f, 1.5f);
        float ry = random.nextFloat(0.5f, 1.5f);
        float rz = random.nextFloat(0.5f, 1.5f);
        Vector3f halfExtents = new Vector3f(rx, ry, rz);
        gemShape = new BoxCollisionShape(halfExtents);
    }

    /**
     * Randomly generate a capsule shape.
     */
    private void randomCapsule() {
        float radius = random.nextFloat(0.2f, 1.2f);
        float height = random.nextFloat(0.5f, 1.5f);
        gemShape = new CapsuleCollisionShape(radius, height);
    }

    /**
     * Randomly generate a Y-axis cone shape.
     */
    private void randomCone() {
        float baseRadius = random.nextFloat(0.5f, 1.5f);
        float height = random.nextFloat(0.5f, 2.5f);
        gemShape = new ConeCollisionShape(baseRadius, height);
    }

    /**
     * Generate a Z-axis cylinder shape with random extents.
     */
    private void randomCylinder() {
        float baseRadius = random.nextFloat(0.5f, 1.5f);
        float halfHeight = random.nextFloat(0.5f, 2f);
        Vector3f halfExtents = new Vector3f(baseRadius, baseRadius, halfHeight);
        gemShape = new CylinderCollisionShape(halfExtents);
    }

    /**
     * Randomly generate an asymmetrical compound shape with 2 cylinders.
     *
     * @param correctAxes if true, correct the shape's principal axes
     */
    private void randomHammer(boolean correctAxes) {
        float handleR = 0.5f;
        float headR = handleR + random.nextFloat();
        float headHalfLength = headR + random.nextFloat();
        float handleHalfLength = headHalfLength + random.nextFloat(0f, 2.5f);
        Vector3f hes = new Vector3f(headR, headR, headHalfLength);
        CollisionShape head = new CylinderCollisionShape(hes);

        hes.set(handleR, handleR, handleHalfLength);
        CollisionShape handle = new CylinderCollisionShape(hes);

        CompoundCollisionShape compound = new CompoundCollisionShape();
        gemShape = compound;

        compound.addChildShape(handle, 0f, 0f, handleHalfLength);

        Vector3f offset = new Vector3f(0f, 0f, 2f * handleHalfLength);
        Matrix3f rotation = new Matrix3f();
        rotation.fromAngleAxis(FastMath.HALF_PI, Vector3f.UNIT_X);
        compound.addChildShape(head, offset, rotation);

        if (correctAxes) {
            float handleMass = 0.15f;
            float headMass = 1f - handleMass;
            FloatBuffer masses
                    = BufferUtils.createFloatBuffer(handleMass, headMass);

            Vector3f inertia = new Vector3f();
            Transform transform = compound.principalAxes(masses, null, inertia);
            gemInverseInertia = Vector3f.UNIT_XYZ.divide(inertia);
            compound.correctAxes(transform);
        }
    }

    /**
     * Randomly select the shape of an uppercase letter.
     */
    private void randomLetter() {
        char glyphChar = (char) ('A' + random.nextInt(26));
        String glyphString = Character.toString(glyphChar);
        gemShape = namedShapes.get(glyphString);
    }

    /**
     * Randomly generate a sphere shape.
     */
    private void randomSphere() {
        float radius = random.nextFloat(0.5f, 1.5f);
        gemShape = new SphereCollisionShape(radius);
    }

    /**
     * Start a new test using the named platform.
     */
    private void restartTest() {
        selectGem(null);
        gems.clear();

        Collection<PhysicsRigidBody> bodies = physicsSpace.getRigidBodyList();
        for (PhysicsRigidBody body : bodies) {
            physicsSpace.remove(body);
        }

        addAPlatform();
    }

    /**
     * Alter which gem is selected.
     *
     * @param gem the gem to select (or null)
     */
    private void selectGem(PhysicsRigidBody gem) {
        if (gem != selectedGem) {
            if (selectedGem != null) {
                Material material = (Material) selectedGem.getUserObject();
                selectedGem.setDebugMaterial(material);
            }
            selectedGem = gem;
            if (selectedGem != null) {
                selectedGem.setDebugMaterial(null);
            }
        }
    }

    /**
     * Toggle visualization of collision-object bounding boxes.
     */
    private void toggleAabb() {
        if (bbFilter == null) {
            bbFilter = new FilterAll(true);
        } else {
            bbFilter = null;
        }

        bulletAppState.setDebugBoundingBoxFilter(bbFilter);
    }

    /**
     * Toggle visualization of collision-object axes.
     */
    private void toggleAxes() {
        float length = bulletAppState.debugAxisLength();
        bulletAppState.setDebugAxisLength(2.5f - length);
    }

    /**
     * Toggle visibility of the helpNode.
     */
    private void toggleHelp() {
        if (helpNode.getCullHint() == Spatial.CullHint.Always) {
            helpNode.setCullHint(Spatial.CullHint.Never);
        } else {
            helpNode.setCullHint(Spatial.CullHint.Always);
        }
    }

    /**
     * Toggle the physics simulation: paused/running.
     */
    private void togglePause() {
        float newSpeed = (speed > 1e-12f) ? 1e-12f : 1f;
        setSpeed(newSpeed);
    }

    /**
     * Toggle visualization of collision-object swept spheres.
     */
    private void toggleSpheres() {
        if (ssFilter == null) {
            ssFilter = new FilterAll(true);
        } else {
            ssFilter = null;
        }

        bulletAppState.setDebugSweptSphereFilter(ssFilter);
    }

    /**
     * Update the status lines in the GUI.
     */
    private void updateStatusLines() {
        String message = String.format("Platform: %s", platformName);
        statusLines[0].setText(message);

        int index = 1 + Arrays.binarySearch(gemShapeNames, shapeName);
        int count = gemShapeNames.length;
        message = String.format("Next gem shape #%d of %d: %s", index, count,
                shapeName);
        statusLines[1].setText(message);

        int numActive = 0;
        for (PhysicsRigidBody gem : gems) {
            if (gem.isActive()) {
                ++numActive;
            }
        }

        int numGems = gems.size();
        boolean isPaused = (speed <= 1e-12f);
        message = String.format(
                "numGems=%d  numActive=%d  friction=%.2f  damping=%.2f%s",
                numGems, numActive, friction, damping,
                isPaused ? "  PAUSED" : "");
        statusLines[2].setText(message);
    }
}
