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
import com.jme3.app.StatsAppState;
import com.jme3.app.state.ScreenshotAppState;
import com.jme3.asset.TextureKey;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.PhysicsRayTestResult;
import com.jme3.bullet.collision.shapes.Box2dShape;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.ConeCollisionShape;
import com.jme3.bullet.collision.shapes.Convex2dShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.GImpactCollisionShape;
import com.jme3.bullet.collision.shapes.HeightfieldCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.MeshCollisionShape;
import com.jme3.bullet.collision.shapes.MultiSphere;
import com.jme3.bullet.collision.shapes.SimplexCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.util.CollisionShapeFactory;
import com.jme3.cursors.plugins.JmeCursor;
import com.jme3.font.BitmapText;
import com.jme3.font.Rectangle;
import com.jme3.input.CameraInput;
import com.jme3.input.KeyInput;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.shape.AbstractBox;
import com.jme3.scene.shape.Box;
import com.jme3.scene.shape.Cylinder;
import com.jme3.scene.shape.Sphere;
import com.jme3.system.AppSettings;
import com.jme3.terrain.geomipmap.TerrainQuad;
import com.jme3.terrain.heightmap.AbstractHeightMap;
import com.jme3.terrain.heightmap.ImageBasedHeightMap;
import com.jme3.texture.Image;
import com.jme3.texture.Texture;
import java.nio.FloatBuffer;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Misc;
import jme3utilities.MyAsset;
import jme3utilities.MyCamera;
import jme3utilities.debug.AxesVisualizer;
import jme3utilities.debug.PointVisualizer;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.RectangularSolid;
import jme3utilities.math.VectorSet;
import jme3utilities.mesh.Cone;
import jme3utilities.mesh.Prism;
import jme3utilities.mesh.RectangleMesh;
import jme3utilities.mesh.Tetrahedron;
import jme3utilities.minie.FilterAll;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.ui.ActionApplication;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.HelpUtils;
import jme3utilities.ui.InputMode;
import vhacd.VHACDParameters;

/**
 * Test various shapes, scales, and collision margins on a kinematic
 * RigidBodyControl. Features tested include bounding boxes, debug
 * visualization, and ray casting.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestRbc extends ActionApplication {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(TestRbc.class.getName());
    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName = TestRbc.class.getSimpleName();
    /**
     * list of test names, in ascending lexicographic order
     */
    final private static String[] testNames = {
        "Box", "BoxGImpact", "BoxHull", "BoxMesh", "Cone", "ConeGImpact",
        "ConeHull", "ConeMesh", "Cylinder", "CylinderGImpact", "CylinderHull",
        "CylinderMesh", "FourSphere", "LargeTerrain", "OneSphere", "Prism",
        "Simplex", "SmallTerrain", "SmallTerrainVhacd", "Sphere",
        "SphereCapsule", "SphereGImpact", "SphereHull", "SphereMesh", "Square",
        "SquareBox", "SquareConvex2d", "SquareGImpact", "SquareHeightfield",
        "SquareHull", "SquareMesh", "TetraGImpact", "TetraHull", "TetraMesh"};
    // *************************************************************************
    // fields

    /**
     * height map for a large heightfield
     */
    private AbstractHeightMap heightMap;
    /**
     * visualizer for the world axes
     */
    private AxesVisualizer axes;
    /**
     * text displayed in the upper-left corner of the GUI node
     */
    final private BitmapText[] statusLines = new BitmapText[2];
    /**
     * AppState to manage the PhysicsSpace
     */
    final private BulletAppState bulletAppState = new BulletAppState();
    /**
     * pre-calculated shape for the SmallTerrainVhacd test
     */
    private CollisionShape smallTerrainVhacdShape;
    /**
     * shape being tested
     */
    private CollisionShape testShape;
    /**
     * height array for a small heightfield
     */
    final private float[] nineHeights = new float[9];
    /**
     * filter to control visualization of axis-aligned bounding boxes
     */
    private FilterAll bbFilter;
    /**
     * cursor shown when the raytest finds a collision object
     */
    private JmeCursor hitCursor;
    /**
     * cursor shown when the raytest doesn't find a collision object
     */
    private JmeCursor missCursor;
    /**
     * double-sided, green, shaded material for test spatials
     */
    private Material greenMaterial;
    /**
     * double-sided, green, wireframe material for the SmallTerrain Spatial
     */
    private Material wireMaterial;
    /**
     * parent for added spatials
     */
    final private Node addNode = new Node("add");
    /**
     * GUI node for displaying hotkey help/hints
     */
    private Node helpNode;
    /**
     * dump debugging information to System.out
     */
    final private PhysicsDumper dumper = new PhysicsDumper();
    /**
     * space for physics simulation
     */
    private PhysicsSpace physicsSpace;
    /**
     * visualizer for ray intersections
     */
    private PointVisualizer rayPoint;
    /**
     * geometry/terrainQuad(s) being tested
     */
    private Spatial testSpatial;
    /**
     * name of the test being run
     */
    private String testName = "SmallTerrain";
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the TestRbc application.
     *
     * @param ignored array of command-line arguments (not null)
     */
    public static void main(String[] ignored) {
        /*
         * Mute the chatty loggers in certain packages.
         */
        Misc.setLoggingLevels(Level.WARNING);

        Application application = new TestRbc();
        /*
         * Customize the window's title bar.
         */
        AppSettings settings = new AppSettings(true);
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
        configurePhysics();
        generateCursors();
        generateMaterials();
        initializeHeightData();

        ColorRGBA bgColor = new ColorRGBA(0.2f, 0.2f, 1f, 1f);
        viewPort.setBackgroundColor(bgColor);

        addAxes();
        addLighting();
        addStatusLines();
        /*
         * Capture a screenshot each time KEY_SYSRQ (the PrtSc key) is pressed.
         */
        ScreenshotAppState screenshotAppState
                = new ScreenshotAppState("Written Assets/", "screenshot");
        boolean success = stateManager.attach(screenshotAppState);
        assert success;
        /*
         * Hide the render-statistics overlay initially.
         */
        stateManager.getState(StatsAppState.class).toggleStats();

        rayPoint = new PointVisualizer(assetManager, 16, ColorRGBA.Red,
                "saltire");
        rootNode.attachChild(rayPoint);
        rayPoint.setEnabled(false);

        rootNode.attachChild(addNode);

        restartTest();
    }

    /**
     * Add application-specific hotkey bindings and override existing ones.
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        dim.bind("cast ray", "RMB");
        dim.bind("clear shapes", KeyInput.KEY_BACK);
        dim.bind("clear shapes", KeyInput.KEY_DELETE);
        dim.bind("dump physicsSpace", KeyInput.KEY_O);
        dim.bind("dump viewport", KeyInput.KEY_P);

        dim.bind("less margin", KeyInput.KEY_V);
        dim.bind("more margin", KeyInput.KEY_F);

        dim.bind("next test", KeyInput.KEY_EQUALS);
        dim.bind("next test", KeyInput.KEY_NUMPAD6);
        dim.bind("previous test", KeyInput.KEY_MINUS);
        dim.bind("previous test", KeyInput.KEY_NUMPAD4);

        dim.bind("scale identity", KeyInput.KEY_I);
        dim.bind("scale nonuniform", KeyInput.KEY_N);
        dim.bind("scale uniform", KeyInput.KEY_U);

        dim.bind("signal " + CameraInput.FLYCAM_LOWER, KeyInput.KEY_DOWN);
        dim.bind("signal " + CameraInput.FLYCAM_RISE, KeyInput.KEY_UP);
        dim.bind("signal orbitLeft", KeyInput.KEY_LEFT);
        dim.bind("signal orbitRight", KeyInput.KEY_RIGHT);

        dim.bind("toggle aabb", KeyInput.KEY_APOSTROPHE);
        dim.bind("toggle axes", KeyInput.KEY_SEMICOLON);
        dim.bind("toggle help", KeyInput.KEY_H);
        dim.bind("toggle view", KeyInput.KEY_SLASH);

        float x = 10f;
        float y = cam.getHeight() - 60f;
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
                case "cast ray":
                    castRay();
                    return;

                case "clear shapes":
                    clearShapes();
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

                case "next test":
                    advanceTest(+1);
                    return;
                case "previous test":
                    advanceTest(-1);
                    return;

                case "less margin":
                    multiplyMargin(0.5f);
                    return;

                case "more margin":
                    multiplyMargin(2f);
                    return;

                case "scale identity":
                    setScale(1f, 1f, 1f);
                    return;
                case "scale nonuniform":
                    setScale(0.8f, 1f, 1.3f);
                    return;
                case "scale uniform":
                    setScale(0.8f, 0.8f, 0.8f);
                    return;

                case "toggle aabb":
                    toggleAabb();
                    return;
                case "toggle axes":
                    axes.toggleEnabled();
                    return;
                case "toggle help":
                    toggleHelp();
                    return;
                case "toggle view":
                    toggleMeshes();
                    togglePhysicsDebug();
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
        /*
         * Set mouse-cursor shape based on a raytest result.
         */
        Vector2f screenXY = inputManager.getCursorPosition();
        Vector3f from = cam.getWorldCoordinates(screenXY, 0f);
        Vector3f to = cam.getWorldCoordinates(screenXY, 1f);
        List rayTest = physicsSpace.rayTestRaw(from, to);
        if (rayTest.size() > 0) {
            inputManager.setMouseCursor(hitCursor);
        } else {
            inputManager.setMouseCursor(missCursor);
        }

        updateStatusLines();
    }
    // *************************************************************************
    // private methods

    /**
     * Add a shape to the scene and PhysicsSpace.
     */
    private void addAShape() {
        switch (testName) {
            case "Box":
            case "BoxGImpact":
            case "BoxHull":
            case "BoxMesh":
            case "FourSphere":
                addBox();
                break;

            case "Cone":
            case "ConeGImpact":
            case "ConeHull":
            case "ConeMesh":
                addCone();
                break;

            case "Cylinder":
            case "CylinderGImpact":
            case "CylinderHull":
            case "CylinderMesh":
                addCylinder();
                break;

            case "LargeTerrain":
                addLargeTerrain();
                break;

            case "Prism":
                addPrism();
                break;

            case "Simplex":
            case "TetraGImpact":
            case "TetraHull":
            case "TetraMesh":
                addTetrahedron();
                break;

            case "SmallTerrain":
            case "SmallTerrainVhacd":
                addSmallTerrain();
                break;

            case "OneSphere":
            case "Sphere":
            case "SphereCapsule":
            case "SphereGImpact":
            case "SphereHull":
            case "SphereMesh":
                addSphere();
                break;

            case "Square":
            case "SquareBox":
            case "SquareConvex2d":
            case "SquareGImpact":
            case "SquareHeightfield":
            case "SquareHull":
            case "SquareMesh":
                addSquare();
                break;

            default:
                throw new IllegalArgumentException(testName);
        }
    }

    /**
     * Add a visualizer for the axes of the world coordinate system.
     */
    private void addAxes() {
        float axisLength = 0.8f;
        axes = new AxesVisualizer(assetManager, axisLength);
        axes.setLineWidth(0f);

        rootNode.addControl(axes);
        axes.setEnabled(true);
    }

    /**
     * Add a box (rectangular solid) to the scene and PhysicsSpace.
     */
    private void addBox() {
        float halfThickness = 0.5f;
        float radius = 1.9f;
        AbstractBox mesh = new Box(radius, halfThickness, radius);
        testSpatial = new Geometry("box", mesh);

        switch (testName) {
            case "Box":
                testShape
                        = new BoxCollisionShape(radius, halfThickness, radius);
                break;

            case "BoxGImpact":
                testShape = new GImpactCollisionShape(mesh);
                break;

            case "BoxHull":
                testShape = new HullCollisionShape(mesh);
                break;

            case "BoxMesh":
                testShape = new MeshCollisionShape(mesh);
                break;

            case "FourSphere":
                RectangularSolid rs = new RectangularSolid(mesh);
                testShape = new MultiSphere(rs);
                break;

            default:
                throw new IllegalArgumentException(testName);
        }

        makeTestShape();
    }

    /**
     * Add a right circular cone to the scene and PhysicsSpace.
     */
    private void addCone() {
        boolean makePyramid = false;
        float height = 3f;
        float radius = 2f;
        int circularSamples = 64;
        Mesh mesh = new Cone(circularSamples, radius, height, makePyramid);
        testSpatial = new Geometry("cone", mesh);

        switch (testName) {
            case "Cone":
                testShape = new ConeCollisionShape(radius, height,
                        PhysicsSpace.AXIS_Y);
                break;

            case "ConeGImpact":
                testShape = new GImpactCollisionShape(mesh);
                break;

            case "ConeHull":
                testShape = new HullCollisionShape(mesh);
                break;

            case "ConeMesh":
                testShape = new MeshCollisionShape(mesh);
                break;

            default:
                throw new IllegalArgumentException(testName);
        }

        makeTestShape();
    }

    /**
     * Add a right circular cylinder to the scene and PhysicsSpace.
     */
    private void addCylinder() {
        boolean closedFlag = true;
        float height = 1f;
        float radius = 2f;
        int heightSample = 2;
        int circSample = 18;
        Mesh mesh = new Cylinder(heightSample, circSample, radius, height,
                closedFlag);
        testSpatial = new Geometry("cylinder", mesh);

        switch (testName) {
            case "Cylinder":
                testShape = new CylinderCollisionShape(radius, height,
                        PhysicsSpace.AXIS_Z);
                break;

            case "CylinderGImpact":
                testShape = new GImpactCollisionShape(mesh);
                break;

            case "CylinderHull":
                testShape = new HullCollisionShape(mesh);
                break;

            case "CylinderMesh":
                testShape = new MeshCollisionShape(mesh);
                break;

            default:
                throw new IllegalArgumentException(testName);
        }

        makeTestShape();
    }

    /**
     * Add a 513x513 terrain to the scene and PhysicsSpace.
     */
    private void addLargeTerrain() {
        int patchSize = 33; // in pixels
        int terrainDiameter = heightMap.getSize(); // in pixels
        int mapSize = terrainDiameter + 1; // number of samples on a side
        float[] heightArray = heightMap.getHeightMap();
        testSpatial = new TerrainQuad("large terrain", patchSize, mapSize,
                heightArray);
        testSpatial.setMaterial(greenMaterial);

        testShape = CollisionShapeFactory.createMeshShape(testSpatial);
        makeTestShape();
    }

    /**
     * Add lighting to the scene.
     */
    private void addLighting() {
        ColorRGBA ambientColor = new ColorRGBA(0.2f, 0.2f, 0.2f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        rootNode.addLight(ambient);
        ambient.setName("ambient");

        Vector3f direction = new Vector3f(1f, -2f, -2f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction);
        rootNode.addLight(sun);
        sun.setName("sun");
    }

    /**
     * Add a petagonal prism to the scene and PhysicsSpace.
     */
    private void addPrism() {
        int numSides = 5;
        float radius = 2.5f;
        float thickness = 1f;
        boolean normals = true;
        Mesh mesh = new Prism(numSides, radius, thickness, normals);
        testSpatial = new Geometry("prism", mesh);

        testShape = CollisionShapeFactory.createDynamicMeshShape(testSpatial);
        makeTestShape();
    }

    /**
     * Add a 3x3 terrain to the scene and PhysicsSpace.
     */
    private void addSmallTerrain() {
        int patchSize = 3;
        int mapSize = 3;
        testSpatial = new TerrainQuad("small terrain", patchSize, mapSize,
                nineHeights);
        testSpatial.setMaterial(wireMaterial);

        switch (testName) {
            case "SmallTerrain":
                testShape = CollisionShapeFactory.createMeshShape(testSpatial);
                break;

            case "SmallTerrainVhacd":
                testShape = (CollisionShape) Misc.deepCopy(
                        smallTerrainVhacdShape);
                break;

            default:
                throw new IllegalArgumentException(testName);
        }

        makeTestShape();
    }

    /**
     * Add a sphere to the scene and PhysicsSpace.
     */
    private void addSphere() {
        float radius = 2.5f;
        Mesh mesh = new Sphere(16, 32, radius);
        testSpatial = new Geometry("sphere", mesh);

        switch (testName) {
            case "OneSphere":
                testShape = new MultiSphere(radius);
                break;

            case "Sphere":
                testShape = new SphereCollisionShape(radius);
                break;

            case "SphereCapsule":
                float height = 0f;
                testShape = new CapsuleCollisionShape(radius, height);
                break;

            case "SphereGImpact":
                testShape = new GImpactCollisionShape(mesh);
                break;

            case "SphereHull":
                testShape = new HullCollisionShape(mesh);
                break;

            case "SphereMesh":
                testShape = new MeshCollisionShape(mesh);
                break;

            default:
                throw new IllegalArgumentException(testName);
        }

        makeTestShape();
    }

    /**
     * Add a square in the X-Y plane to the scene and PhysicsSpace.
     */
    private void addSquare() {
        float he = 1f;
        Mesh mesh = new RectangleMesh(-he, +he, -he, +he, 1f);
        testSpatial = new Geometry("square", mesh);

        switch (testName) {
            case "Square":
                testShape = new Box2dShape(he);
                break;

            case "SquareBox":
                testShape = new BoxCollisionShape(he, he, 0.04f);
                break;

            case "SquareConvex2d":
                FloatBuffer buffer
                        = mesh.getFloatBuffer(VertexBuffer.Type.Position);
                testShape = new Convex2dShape(buffer);
                break;

            case "SquareGImpact":
                testShape = new GImpactCollisionShape(mesh);
                break;

            case "SquareHeightfield":
                int stickLength = 3;
                int stickWidth = 3;
                float[] heightmap = new float[stickLength * stickWidth];
                Vector3f scale = Vector3f.UNIT_XYZ;
                int upAxis = PhysicsSpace.AXIS_Z;
                boolean flipQuadEdges = false;
                boolean flipTriangleWinding = false;
                boolean useDiamond = false;
                boolean useZigzag = false;
                testShape = new HeightfieldCollisionShape(stickLength,
                        stickWidth, heightmap, scale, upAxis, flipQuadEdges,
                        flipTriangleWinding, useDiamond, useZigzag);
                break;

            case "SquareHull":
                testShape = new HullCollisionShape(mesh);
                break;

            case "SquareMesh":
                testShape = new MeshCollisionShape(mesh);
                break;

            default:
                throw new IllegalArgumentException(testName);
        }

        makeTestShape();
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
     * Add a regular tetrahedron to the scene and PhysicsSpace.
     */
    private void addTetrahedron() {
        float radius = 2.5f;
        Mesh mesh = new Tetrahedron(radius, true);
        testSpatial = new Geometry("tetrahedron", mesh);

        switch (testName) {
            case "Simplex":
                FloatBuffer buffer
                        = mesh.getFloatBuffer(VertexBuffer.Type.Position);
                VectorSet vs = MyBuffer.distinct(buffer, 0, buffer.limit());
                buffer = vs.toBuffer();
                testShape
                        = new SimplexCollisionShape(buffer, 0, buffer.limit());
                break;

            case "TetraGImpact":
                testShape = new GImpactCollisionShape(mesh);
                break;

            case "TetraHull":
                testShape = new HullCollisionShape(mesh);
                break;

            case "TetraMesh":
                testShape = new MeshCollisionShape(mesh);
                break;

            default:
                throw new IllegalArgumentException(testName);
        }

        makeTestShape();
    }

    /**
     * Advance the test selection by the specified amount and start the selected
     * test.
     *
     * @param amount the number of tests to advance the selection
     */
    private void advanceTest(int amount) {
        int index = Arrays.binarySearch(testNames, testName);
        if (index < 0) {
            testName = testNames[0];
        } else {
            assert testNames[index].equals(testName);
            index = MyMath.modulo(index + amount, testNames.length);
            testName = testNames[index];
        }

        restartTest();
    }

    /**
     * Cast a physics ray from the mouse pointer and move the PointVisualizer to
     * the nearest intersection.
     */
    private void castRay() {
        Vector2f screenXY = inputManager.getCursorPosition();
        Vector3f from = cam.getWorldCoordinates(screenXY, 0f);
        Vector3f to = cam.getWorldCoordinates(screenXY, 1f);
        List<PhysicsRayTestResult> rayTest = physicsSpace.rayTest(from, to);

        if (rayTest.size() > 0) {
            PhysicsRayTestResult intersect = rayTest.get(0);
            float fraction = intersect.getHitFraction();
            Vector3f location = MyVector3f.lerp(fraction, from, to, null);
            rayPoint.setLocalTranslation(location);
            rayPoint.setEnabled(true);
        } else {
            rayPoint.setEnabled(false);
        }
    }

    /**
     * Delete all added shapes from the scene and physics space. Also disable
     * the visualizer for ray intersections.
     */
    private void clearShapes() {
        rayPoint.setEnabled(false);
        /*
         * Remove any added spatials from the scene.
         */
        addNode.detachAllChildren();
        /*
         * Remove all collision objects from the physics space,
         * which also removes their debug meshes.
         */
        Collection<PhysicsCollisionObject> pcos = physicsSpace.getPcoList();
        for (PhysicsCollisionObject pco : pcos) {
            physicsSpace.remove(pco);
        }
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        float near = 0.02f;
        float far = 20f;
        MyCamera.setNearFar(cam, near, far);

        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(5f);

        cam.setLocation(new Vector3f(5.9f, 5.4f, 2.3f));
        cam.setRotation(new Quaternion(0.1825f, -0.76803f, 0.2501f, 0.56058f));

        CameraOrbitAppState orbitState
                = new CameraOrbitAppState(cam, "orbitLeft", "orbitRight");
        stateManager.attach(orbitState);
    }

    /**
     * Configure the PhysicsDumper during startup.
     */
    private void configureDumper() {
        //dumper.setMaxChildren(3);
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        stateManager.attach(bulletAppState);
        physicsSpace = bulletAppState.getPhysicsSpace();
    }

    /**
     * Initialize cursors during startup.
     */
    private void generateCursors() {
        String hitPath = "Textures/cursors/menu.cur";
        hitCursor = (JmeCursor) assetManager.loadAsset(hitPath);

        String missPath = "Textures/cursors/default.cur";
        missCursor = (JmeCursor) assetManager.loadAsset(missPath);
    }

    /**
     * Initialize materials during startup.
     */
    private void generateMaterials() {
        ColorRGBA green = new ColorRGBA(0f, 0.12f, 0f, 1f);
        greenMaterial = MyAsset.createShadedMaterial(assetManager, green);
        greenMaterial.setName("green");
        RenderState ars = greenMaterial.getAdditionalRenderState();
        ars.setFaceCullMode(RenderState.FaceCullMode.Off);

        wireMaterial = MyAsset.createWireframeMaterial(assetManager, green);
        wireMaterial.setName("green wire");
        ars = wireMaterial.getAdditionalRenderState();
        ars.setFaceCullMode(RenderState.FaceCullMode.Off);
    }

    /**
     * Initialize the height data during startup.
     */
    private void initializeHeightData() {
        heightMap = loadHeightMap();

        nineHeights[0] = 1f;
        nineHeights[1] = 0f;
        nineHeights[2] = 1f;
        nineHeights[3] = 0f;
        nineHeights[4] = 0.5f;
        nineHeights[5] = 0f;
        nineHeights[6] = 1f;
        nineHeights[7] = 0f;
        nineHeights[8] = 1f;

        int patchSize = 3;
        int mapSize = 3;
        Spatial spatial
                = new TerrainQuad("terrain", patchSize, mapSize, nineHeights);
        smallTerrainVhacdShape = CollisionShapeFactory.createVhacdShape(
                spatial, new VHACDParameters(), null);
    }

    /**
     * Load a simple height map from a texture asset.
     *
     * @return a new instance (not null)
     */
    private AbstractHeightMap loadHeightMap() {
        boolean flipY = false;
        TextureKey key = new TextureKey(
                "Textures/BumpMapTest/Simple_height.png", flipY);
        Texture texture = assetManager.loadTexture(key);
        Image image = texture.getImage();

        float heightScale = 1f;
        AbstractHeightMap result = new ImageBasedHeightMap(image, heightScale);
        result.load();

        return result;
    }

    /**
     * Convert the test shape into a kinematic RigidBodyControl and add it to
     * the main scene and the PhysicsSpace.
     */
    private void makeTestShape() {
        addNode.attachChild(testSpatial);
        setScale(1f, 1f, 1f);
        if (testSpatial instanceof Geometry) {
            testSpatial.setMaterial(greenMaterial);
        }

        RigidBodyControl rbc = new RigidBodyControl(testShape);
        rbc.setApplyScale(true);
        rbc.setKinematic(true);
        rbc.setPhysicsSpace(physicsSpace);
        rbc.setDebugNumSides(2);
        testSpatial.addControl(rbc);
    }

    /**
     * Alter the collision margin for the test shape.
     *
     * @param factor the factor to increase the margin (&gt;0)
     */
    private void multiplyMargin(float factor) {
        assert factor > 0f : factor;

        float margin = testShape.getMargin();
        margin *= factor;
        testShape.setMargin(margin);
    }

    /**
     * Start a new test using the named shape.
     */
    private void restartTest() {
        clearShapes();
        addAShape();
    }

    /**
     * Alter the scale of the Spatial controlled by the RigidBodyControl.
     * Because the control is in kinematic mode with setApplyScale(true), it
     * will attempt to scale its collision shape accordingly.
     *
     * @param xScale desired X-axis scale factor (default=1)
     * @param yScale desired Y-axis scale factor (default=1)
     * @param zScale desired Z-axis scale factor (default=1)
     */
    private void setScale(float xScale, float yScale, float zScale) {
        if (testName.equals("LargeTerrain")) {
            testSpatial.setLocalScale(
                    xScale / 100f, yScale / 100f, zScale / 100f);
        } else {
            testSpatial.setLocalScale(xScale, yScale, zScale);
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
     * Toggle rendering of the test spatial(s).
     */
    private void toggleMeshes() {
        Spatial.CullHint hint = addNode.getLocalCullHint();
        if (hint == Spatial.CullHint.Inherit
                || hint == Spatial.CullHint.Never) {
            hint = Spatial.CullHint.Always;
        } else if (hint == Spatial.CullHint.Always) {
            hint = Spatial.CullHint.Never;
        }
        addNode.setCullHint(hint);
    }

    /**
     * Toggle physics-debug visualization on/off.
     */
    private void togglePhysicsDebug() {
        boolean enabled = bulletAppState.isDebugEnabled();
        bulletAppState.setDebugEnabled(!enabled);
    }

    /**
     * Update the status lines in the GUI.
     */
    private void updateStatusLines() {
        int index = 1 + Arrays.binarySearch(testNames, testName);
        int count = testNames.length;
        String message
                = String.format("Test #%d of %d: %s", index, count, testName);
        statusLines[0].setText(message);

        String viewName;
        boolean debug = bulletAppState.isDebugEnabled();
        if (debug) {
            viewName = "physics";
            if (bbFilter != null) {
                viewName += "+aabb";
            }
        } else {
            viewName = "mesh";
        }

        float margin = testShape.getMargin();
        Vector3f scale = testShape.getScale(null);
        message = String.format("view=%s, margin=%.3f, scale=%s",
                viewName, margin, scale);
        statusLines[1].setText(message);
    }
}
