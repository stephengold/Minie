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
import com.jme3.bullet.PhysicsSoftSpace;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.SoftPhysicsAppState;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.PhysicsRayTestResult;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.ConeCollisionShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.MeshCollisionShape;
import com.jme3.bullet.collision.shapes.PlaneCollisionShape;
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
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
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
import jme3utilities.mesh.Prism;
import jme3utilities.minie.DumpFlags;
import jme3utilities.minie.FilterAll;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.minie.test.mesh.ClothHexagon;
import jme3utilities.minie.test.shape.MinieTestShapes;
import jme3utilities.minie.test.shape.ShapeGenerator;
import jme3utilities.ui.ActionApplication;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.HelpUtils;
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
        extends ActionApplication
        implements DebugInitListener {
    // *************************************************************************
    // constants and loggers

    /**
     * upper limit on the number of drops
     */
    final private static int maxNumDrops = 80;
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
     * AppState to manage the PhysicsSpace
     */
    final private SoftPhysicsAppState bulletAppState
            = new SoftPhysicsAppState();
    /**
     * shape for the next drop
     */
    private CollisionShape dropShape;
    /**
     * current drops, in order of creation
     */
    final private Deque<PhysicsRigidBody> drops = new ArrayDeque<>(maxNumDrops);
    /**
     * AppState to manage the status overlay
     */
    private DropTestStatus status;
    /**
     * filter to control visualization of axis-aligned bounding boxes
     */
    private FilterAll bbFilter;
    /**
     * filter to control visualization of swept spheres
     */
    private FilterAll ssFilter;
    /**
     * enhanced pseudo-random generator
     */
    final private ShapeGenerator random = new ShapeGenerator();
    /**
     * map names to collision shapes
     */
    final private Map<String, CollisionShape> namedShapes = new TreeMap<>();
    /**
     * shiny, lit materials to visualize drops
     */
    final private Material dropMaterials[] = new Material[4];
    /**
     * single-sided lit green material to visualize platforms
     */
    private Material platformMaterial;
    /**
     * GUI node for displaying hotkey help/hints
     */
    private Node helpNode;
    /**
     * dump debugging information to System.out
     */
    final private PhysicsDumper dumper = new PhysicsDumper();
    /**
     * selected drop, or null if none
     */
    private PhysicsRigidBody selectedDrop = null;
    /**
     * space for physics simulation
     */
    private PhysicsSoftSpace physicsSpace;
    /**
     * local inverse inertial vector for the current drop (or null)
     */
    private Vector3f inverseInertia = null;
    // *************************************************************************
    // new methods exposed

    /**
     * Count how many drops are active.
     */
    int countActive() {
        int result = 0;
        for (PhysicsRigidBody drop : drops) {
            if (drop.isActive()) {
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
     * Read the rate of physics simulation.
     *
     * @return the speed (1&rarr;real time)
     */
    float getSpeed() {
        return speed;
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

    /**
     * Start a new test.
     */
    void restartTest() {
        selectDrop(null);
        drops.clear();

        Collection<PhysicsRigidBody> rigids = physicsSpace.getRigidBodyList();
        for (PhysicsRigidBody body : rigids) {
            physicsSpace.remove(body);
        }

        Collection<PhysicsSoftBody> softs = physicsSpace.getSoftBodyList();
        for (PhysicsSoftBody body : softs) {
            physicsSpace.remove(body);
        }

        addAPlatform();
    }

    /**
     * Alter the damping fractions for all drops.
     *
     * @param fraction the desired fraction (&ge;0, &le;1)
     */
    void setDamping(float fraction) {
        assert fraction >= 0f : fraction;
        assert fraction <= 1f : fraction;

        for (PhysicsRigidBody drop : drops) {
            drop.setDamping(fraction, fraction);
        }
    }

    /**
     * Alter the friction coefficients for all rigid bodies.
     *
     * @param coefficient the desired coefficient (&ge;0)
     */
    void setFriction(float coefficient) {
        assert coefficient >= 0f : coefficient;

        for (PhysicsRigidBody body : physicsSpace.getRigidBodyList()) {
            body.setFriction(coefficient);
        }
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

        ColorRGBA bgColor = new ColorRGBA(0.1f, 0.2f, 0.4f, 1f);
        viewPort.setBackgroundColor(bgColor);

        status = new DropTestStatus();
        boolean success = stateManager.attach(status);
        assert success;

        addAPlatform();
        addADrop();
    }

    /**
     * Add application-specific hotkey bindings and override existing ones.
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        dim.bind("add", KeyInput.KEY_RETURN);
        dim.bind("add", KeyInput.KEY_INSERT);
        dim.bind("add", KeyInput.KEY_NUMPAD0);

        dim.bind("collect garbage", KeyInput.KEY_G);

        dim.bind("delete last", KeyInput.KEY_BACK);
        dim.bind("delete last", KeyInput.KEY_SUBTRACT);
        dim.bind("delete selected", KeyInput.KEY_DECIMAL);
        dim.bind("delete selected", KeyInput.KEY_DELETE);

        dim.bind("dump physicsSpace", KeyInput.KEY_O);
        dim.bind("dump selectedDrop", KeyInput.KEY_LBRACKET);
        dim.bind("dump viewport", KeyInput.KEY_P);

        dim.bind("next statusLine", KeyInput.KEY_NUMPAD2);
        dim.bind("next value", KeyInput.KEY_EQUALS);
        dim.bind("next value", KeyInput.KEY_NUMPAD6);

        dim.bind("pick", "RMB");
        dim.bind("pick", KeyInput.KEY_R);

        dim.bind("previous statusLine", KeyInput.KEY_NUMPAD8);
        dim.bind("previous value", KeyInput.KEY_MINUS);
        dim.bind("previous value", KeyInput.KEY_NUMPAD4);

        dim.bind("restart test", KeyInput.KEY_NUMPAD5);

        dim.bind("signal " + CameraInput.FLYCAM_LOWER, KeyInput.KEY_DOWN);
        dim.bind("signal " + CameraInput.FLYCAM_RISE, KeyInput.KEY_UP);
        dim.bind("signal orbitLeft", KeyInput.KEY_LEFT);
        dim.bind("signal orbitRight", KeyInput.KEY_RIGHT);
        dim.bind("signal shower", KeyInput.KEY_ADD);
        dim.bind("signal shower", KeyInput.KEY_I);

        dim.bind("toggle aabb", KeyInput.KEY_APOSTROPHE);
        dim.bind("toggle axes", KeyInput.KEY_SEMICOLON);
        dim.bind("toggle childColoring", KeyInput.KEY_COMMA);
        dim.bind("toggle help", KeyInput.KEY_H);
        dim.bind("toggle pause", KeyInput.KEY_PAUSE);
        dim.bind("toggle pause", KeyInput.KEY_PERIOD);
        dim.bind("toggle spheres", KeyInput.KEY_L);
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

                case "collect garbage":
                    System.gc();
                    return;

                case "delete last":
                    deleteLastDrop();
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
                case "dump selectedDrop":
                    dumpSelectedDrop();
                    return;
                case "dump viewport":
                    dumper.dump(viewPort);
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

                case "previous statusLine":
                    status.advanceSelectedField(-1);
                    return;
                case "previous value":
                    status.advanceValue(-1);
                    return;

                case "restart test":
                    restartTest();
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

        inverseInertia = null;
        String dropName = status.nextDropType();
        DebugMeshNormals debugMeshNormals;
        switch (dropName) {
            case "barbell":
            case "knucklebone":
            case "ladder":
            case "top":
                dropShape = namedShapes.get(dropName);
                debugMeshNormals = DebugMeshNormals.Smooth;
                break;

            case "box":
            case "frame":
            case "halfPipe":
            case "hull":
            case "platonic":
            case "prism":
            case "pyramid":
            case "star":
            case "tetrahedron":
                dropShape = random.nextShape(dropName);
                debugMeshNormals = DebugMeshNormals.Facet;
                break;

            case "capsule":
            case "cone":
            case "cylinder":
            case "dome":
            case "football":
            case "multiSphere":
            case "sphere":
            case "torus":
                dropShape = random.nextShape(dropName);
                debugMeshNormals = DebugMeshNormals.Smooth;
                break;

            case "chair":
                dropShape = namedShapes.get("chair");
                inverseInertia = MinieTestShapes.chairInverseInertia;
                debugMeshNormals = DebugMeshNormals.Facet;
                break;

            case "duck":
            case "heart":
            case "sword":
            case "teapot":
                dropShape = namedShapes.get(dropName);
                debugMeshNormals = DebugMeshNormals.Facet;
                break;

            case "letter":
                randomLetter();
                debugMeshNormals = DebugMeshNormals.Facet;
                break;

            case "madMallet":
                randomMallet(false);
                debugMeshNormals = DebugMeshNormals.Smooth;
                break;

            case "mallet":
                randomMallet(true);
                debugMeshNormals = DebugMeshNormals.Smooth;
                break;

            default:
                String message = "dropName = " + MyString.quote(dropName);
                throw new RuntimeException(message);
        }

        Vector3f startLocation = random.nextVector3f();
        startLocation.multLocal(2.5f, 5f, 2.5f);
        startLocation.y += 20f;

        Quaternion startOrientation = random.nextQuaternion();

        Material debugMaterial;
        if (status.isChildColoring()
                && dropShape instanceof CompoundCollisionShape) {
            debugMaterial = BulletDebugAppState.enableChildColoring;
        } else {
            debugMaterial = (Material) random.pick(dropMaterials);
        }

        float mass = 1f;
        PhysicsRigidBody body = new PhysicsRigidBody(dropShape, mass);
        body.setCcdMotionThreshold(5f);
        float sweptSphereRadius = dropShape.maxRadius();
        body.setCcdSweptSphereRadius(sweptSphereRadius);
        float damping = status.damping();
        body.setDamping(damping, damping);
        body.setDebugMaterial(debugMaterial);
        body.setDebugMeshNormals(debugMeshNormals);
        body.setDebugMeshResolution(DebugShapeFactory.highResolution);
        float friction = status.friction();
        body.setFriction(friction);
        body.setPhysicsLocation(startLocation);
        body.setPhysicsRotation(startOrientation);
        if (inverseInertia != null) {
            body.setInverseInertiaLocal(inverseInertia);
        }
        body.setUserObject(debugMaterial);

        physicsSpace.add(body);
        drops.addLast(body);
    }

    /**
     * Add a platform to the PhysicsSpace.
     */
    private void addAPlatform() {
        String platformName = status.platformType();
        switch (platformName) {
            case "bedOfNails":
            case "sieve":
            case "tray":
            case "triangle":
                addNamedPlatform(platformName, DebugMeshNormals.Facet);
                break;

            case "box":
                addBoxPlatform();
                break;

            case "candyDish":
            case "dimples":
            case "smooth":
                addNamedPlatform(platformName, DebugMeshNormals.Smooth);
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

            case "trampoline":
                addTrampoline();
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
     * Attach a Node to display hotkey help/hints.
     */
    private void addHelp() {
        float width = 360f;
        float y = cam.getHeight() - 30f;
        float x = cam.getWidth() - width - 10f;
        float height = cam.getHeight() - 20f;
        Rectangle rectangle = new Rectangle(x, y, width, height);

        InputMode dim = getDefaultInputMode();
        float space = 20f;
        helpNode = HelpUtils.buildNode(dim, rectangle, guiFont, space);
        guiNode.attachChild(helpNode);
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
     * Add a static rigid body with the named shape to the PhysicsSpace, to
     * serve as a platform.
     */
    private void addNamedPlatform(String shapeName, DebugMeshNormals normals) {
        CollisionShape shape = namedShapes.get(shapeName);
        assert shape != null : shapeName;
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
        CollisionShape shape = namedShapes.get("roundedRectangle");
        float mass = PhysicsRigidBody.massForStatic;
        PhysicsRigidBody body = new PhysicsRigidBody(shape, mass);

        body.setDebugMeshNormals(DebugMeshNormals.Facet);
        Quaternion rotation = new Quaternion();
        rotation.fromAngles(FastMath.HALF_PI, 0f, 0f);
        body.setPhysicsRotation(rotation);
        makePlatform(body);
    }

    /**
     * Add a hexagonal trampoline to the PhysicsSpace, to serve as a platform.
     */
    private void addTrampoline() {
        int numRings = 9;
        float vertexSpacing = 2f;
        Mesh mesh = new ClothHexagon(numRings, vertexSpacing);
        PhysicsSoftBody softBody = new PhysicsSoftBody();
        NativeSoftBodyUtil.appendFromTriMesh(mesh, softBody);
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

        makePlatform(softBody);
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

        physicsSpace = bulletAppState.getPhysicsSoftSpace();
        physicsSpace.setGravity(new Vector3f(0f, -30f, 0f));

        generateShapes();
    }

    /**
     * Delete the most recently added drop.
     */
    private void deleteLastDrop() {
        PhysicsRigidBody lastDrop = drops.peekLast();
        if (lastDrop != null) {
            physicsSpace.remove(lastDrop);
            if (lastDrop == selectedDrop) {
                selectDrop(null);
            }
            drops.removeLast();
            for (PhysicsRigidBody drop : drops) {
                drop.activate();
            }
        }
    }

    /**
     * Delete the selected drop, if any.
     */
    private void deleteSelected() {
        if (selectedDrop != null) {
            physicsSpace.remove(selectedDrop);
            selectDrop(null);
            drops.remove(selectedDrop);
            for (PhysicsRigidBody drop : drops) {
                drop.activate();
            }
        }
    }

    /**
     * Dump the selected drop.
     */
    private void dumpSelectedDrop() {
        if (selectedDrop == null) {
            System.out.printf("%nNo drop selected.");
        } else {
            dumper.dump(selectedDrop, "");
        }
    }

    /**
     * Initialize materials during startup.
     */
    private void generateMaterials() {
        ColorRGBA green = new ColorRGBA(0f, 0.12f, 0f, 1f);
        platformMaterial = MyAsset.createShadedMaterial(assetManager, green);
        platformMaterial.setName("green");

        ColorRGBA dropColors[] = new ColorRGBA[dropMaterials.length];
        dropColors[0] = new ColorRGBA(0.2f, 0f, 0f, 1f); // ruby
        dropColors[1] = new ColorRGBA(0f, 0.07f, 0f, 1f); // emerald
        dropColors[2] = new ColorRGBA(0f, 0f, 0.3f, 1f); // sapphire
        dropColors[3] = new ColorRGBA(0.2f, 0.1f, 0f, 1f); // topaz

        for (int i = 0; i < dropMaterials.length; ++i) {
            ColorRGBA color = dropColors[i];
            dropMaterials[i]
                    = MyAsset.createShinyMaterial(assetManager, color);
            dropMaterials[i].setFloat("Shininess", 15f);
        }
    }

    /**
     * Initialize the collection of named collision shapes during startup.
     */
    private void generateShapes() {
        /*
         * "barbell", "bedOfNails", "chair", "dimples", "knucklebone", "ladder",
         * "roundedRectangle", "sieve", "smooth", "top", "tray", and "triangle"
         * are each generated once, during startup
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
     * Add the specified platform body to the PhysicsSpace.
     */
    private void makePlatform(PhysicsBody body) {
        body.setDebugMaterial(platformMaterial);
        float friction = status.friction();
        body.setFriction(friction);
        physicsSpace.add(body);
    }

    /**
     * Cast a physics ray from the cursor and select the nearest drop in the
     * result.
     */
    private void pick() {
        Vector2f screenXY = inputManager.getCursorPosition();
        Vector3f from = cam.getWorldCoordinates(screenXY, 0f);
        Vector3f to = cam.getWorldCoordinates(screenXY, 1f);
        List<PhysicsRayTestResult> hits = physicsSpace.rayTest(from, to);

        for (PhysicsRayTestResult hit : hits) {
            PhysicsCollisionObject pco = hit.getCollisionObject();
            if (pco instanceof PhysicsRigidBody) {
                PhysicsRigidBody body = (PhysicsRigidBody) pco;
                if (drops.contains(body)) {
                    selectDrop(body);
                    return;
                }
            }
        }
        selectDrop(null);
    }

    /**
     * Randomly select the shape of an uppercase letter.
     */
    private void randomLetter() {
        char glyphChar = (char) ('A' + random.nextInt(26));
        String glyphString = Character.toString(glyphChar);
        dropShape = namedShapes.get(glyphString);
    }

    /**
     * Randomly generate an asymmetrical compound shape consisting of 2
     * cylinders.
     *
     * @param correctAxes if true, correct the shape's center of mass and
     * principal axes
     */
    private void randomMallet(boolean correctAxes) {
        float handleR = 0.5f;
        float headR = handleR + random.nextFloat();
        float headHalfLength = headR + random.nextFloat();
        float handleHalfLength = headHalfLength + random.nextFloat(0f, 2.5f);
        CompoundCollisionShape compound = MinieTestShapes.makeMadMallet(handleR,
                headR, handleHalfLength, headHalfLength);
        dropShape = compound;
        /*
         * At this point, the shape's center of mass lies at the bare end
         * of the handle:  a "mad" mallet that prefers to stand upright.
         */
        if (correctAxes) {
            float handleMass = 0.15f;
            float headMass = 1f - handleMass; // Put 85% of mass in the head.
            FloatBuffer masses
                    = BufferUtils.createFloatBuffer(handleMass, headMass);

            Vector3f inertia = new Vector3f();
            Transform transform = compound.principalAxes(masses, null, inertia);
            inverseInertia = Vector3f.UNIT_XYZ.divide(inertia);
            compound.correctAxes(transform);
        }
    }

    /**
     * Alter which drop is selected.
     *
     * @param drop the drop to select (or null)
     */
    private void selectDrop(PhysicsRigidBody drop) {
        if (drop != selectedDrop) {
            if (selectedDrop != null) {
                Material material = (Material) selectedDrop.getUserObject();
                selectedDrop.setDebugMaterial(material);
            }
            selectedDrop = drop;
            if (selectedDrop != null) {
                selectedDrop.setDebugMaterial(null);
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
}
