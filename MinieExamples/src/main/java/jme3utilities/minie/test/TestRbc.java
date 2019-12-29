/*
 Copyright (c) 2018-2019, Stephen Gold
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
import com.jme3.asset.TextureKey;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.PhysicsRayTestResult;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.MultiSphere;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.util.CollisionShapeFactory;
import com.jme3.font.BitmapText;
import com.jme3.font.Rectangle;
import com.jme3.input.CameraInput;
import com.jme3.input.KeyInput;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Ray;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.AbstractBox;
import com.jme3.scene.shape.Box;
import com.jme3.system.AppSettings;
import com.jme3.terrain.geomipmap.TerrainQuad;
import com.jme3.terrain.heightmap.AbstractHeightMap;
import com.jme3.terrain.heightmap.ImageBasedHeightMap;
import com.jme3.texture.Image;
import com.jme3.texture.Texture;
import java.util.Collection;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Misc;
import jme3utilities.MyAsset;
import jme3utilities.MyCamera;
import jme3utilities.debug.AxesVisualizer;
import jme3utilities.debug.PointVisualizer;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.RectangularSolid;
import jme3utilities.minie.FilterAll;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.minie.test.mesh.Prism;
import jme3utilities.ui.ActionApplication;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.InputMode;

/**
 * Test rigid-body controls with various shapes and collision margins. Features
 * tested include bounding boxes, debug visualization, and ray casting.
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
    final private static String applicationName
            = TestRbc.class.getSimpleName();
    // *************************************************************************
    // fields

    /**
     * height map for a large heightfield
     */
    private AbstractHeightMap heightMap;
    /**
     * status displayed in the upper-left corner of the GUI node
     */
    private BitmapText statusText;
    /**
     * AppState to manage the PhysicsSpace
     */
    final private BulletAppState bulletAppState = new BulletAppState();
    /**
     * shape currently being tested
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
     * lit green material for scene objects
     */
    private Material greenMaterial;
    /**
     * green wireframe material for scene objects
     */
    private Material wireMaterial;
    /**
     * parent for added geometries
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
     * name of shape currently being tested
     */
    private String shapeName = "SmallTerrain";
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
        generateMaterials();
        configurePhysics();
        initializeHeightData();

        ColorRGBA bgColor = new ColorRGBA(0.2f, 0.2f, 1f, 1f);
        viewPort.setBackgroundColor(bgColor);
        addAxes();
        addLighting();

        rayPoint = new PointVisualizer(assetManager, 16, ColorRGBA.Red,
                "saltire");
        rootNode.attachChild(rayPoint);
        rayPoint.setEnabled(false);

        rootNode.attachChild(addNode);

        restartTest();
        /*
         * Add the status text to the GUI.
         */
        statusText = new BitmapText(guiFont, false);
        statusText.setLocalTranslation(0f, cam.getHeight(), 0f);
        guiNode.attachChild(statusText);
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
        dim.bind("dump scenes", KeyInput.KEY_P);

        dim.bind("less margin", KeyInput.KEY_V);
        dim.bind("more margin", KeyInput.KEY_F);

        dim.bind("signal " + CameraInput.FLYCAM_LOWER, KeyInput.KEY_DOWN);
        dim.bind("signal " + CameraInput.FLYCAM_RISE, KeyInput.KEY_UP);
        dim.bind("signal orbitLeft", KeyInput.KEY_LEFT);
        dim.bind("signal orbitRight", KeyInput.KEY_RIGHT);

        dim.bind("test Box", KeyInput.KEY_F6);
        dim.bind("test BoxHull", KeyInput.KEY_F7);
        dim.bind("test FourSphere", KeyInput.KEY_F8);
        dim.bind("test LargeTerrain", KeyInput.KEY_F1);
        dim.bind("test Prism", KeyInput.KEY_F3);
        dim.bind("test SmallTerrain", KeyInput.KEY_F2);

        dim.bind("toggle boxes", KeyInput.KEY_APOSTROPHE);
        dim.bind("toggle help", KeyInput.KEY_H);
        dim.bind("toggle meshes", KeyInput.KEY_M);
        dim.bind("toggle physics debug", KeyInput.KEY_SLASH);

        float x = 10f;
        float y = cam.getHeight() - 40f;
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

                case "less margin":
                    multiplyMargin(0.5f);
                    return;

                case "more margin":
                    multiplyMargin(2f);
                    return;

                case "toggle boxes":
                    toggleBoxes();
                    return;
                case "toggle help":
                    toggleHelp();
                    return;
                case "toggle meshes":
                    toggleMeshes();
                    return;
                case "toggle physics debug":
                    togglePhysicsDebug();
                    return;
            }

            String[] words = actionString.split(" ");
            if (words.length >= 2 && "test".equals(words[0])) {
                shapeName = words[1];
                restartTest();
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
        updateStatusText();
    }
    // *************************************************************************
    // private methods

    /**
     * Add a shape to the scene and PhysicsSpace.
     */
    private void addAShape() {
        switch (shapeName) {
            case "Box":
            case "BoxHull":
            case "FourSphere":
                addBox();
                break;

            case "LargeTerrain":
                addLargeTerrain();
                break;

            case "Prism":
                addPrism();
                break;

            case "SmallTerrain":
                addSmallTerrain();
                break;

            default:
                throw new IllegalArgumentException(shapeName);
        }
    }

    /**
     * Add a visualizer for the axes of the world coordinate system.
     */
    private void addAxes() {
        float axisLength = 0.8f;
        AxesVisualizer axes = new AxesVisualizer(assetManager, axisLength);
        axes.setLineWidth(0f);

        rootNode.addControl(axes);
        axes.setEnabled(true);
    }

    /**
     * Add a box to the scene and PhysicsSpace.
     */
    private void addBox() {
        float radius = 3f;
        float halfThickness = 0.5f;
        AbstractBox mesh = new Box(radius, halfThickness, radius);
        Geometry geometry = new Geometry("box", mesh);
        addNode.attachChild(geometry);
        geometry.setMaterial(wireMaterial);

        switch (shapeName) {
            case "Box":
                testShape
                        = new BoxCollisionShape(radius, halfThickness, radius);
                break;

            case "BoxHull":
                RectangularSolid rs = new RectangularSolid(mesh);
                testShape = new HullCollisionShape(rs);
                break;

            case "FourSphere":
                rs = new RectangularSolid(mesh);
                testShape = new MultiSphere(rs);
                break;

            default:
                throw new IllegalArgumentException(shapeName);
        }

        float massForStatic = 0f;
        RigidBodyControl rbc = new RigidBodyControl(testShape, massForStatic);
        rbc.setPhysicsSpace(physicsSpace);
        geometry.addControl(rbc);
    }

    /**
     * Add 513x513 terrain to the scene and PhysicsSpace.
     */
    private void addLargeTerrain() {
        int patchSize = 33; // in pixels
        int terrainDiameter = heightMap.getSize(); // in pixels
        int mapSize = terrainDiameter + 1; // number of samples on a side
        float[] heightArray = heightMap.getHeightMap();
        TerrainQuad quad
                = new TerrainQuad("terrain", patchSize, mapSize, heightArray);
        addNode.attachChild(quad);
        quad.setLocalScale(0.01f);
        quad.setMaterial(greenMaterial);

        testShape = CollisionShapeFactory.createMeshShape(quad);
        float massForStatic = 0f;
        RigidBodyControl rbc = new RigidBodyControl(testShape, massForStatic);
        rbc.setPhysicsSpace(physicsSpace);
        quad.addControl(rbc);
    }

    /**
     * Add lighting to the scene.
     */
    private void addLighting() {
        ColorRGBA ambientColor = new ColorRGBA(0.2f, 0.2f, 0.2f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        rootNode.addLight(ambient);

        Vector3f direction = new Vector3f(1f, -2f, -2f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction);
        rootNode.addLight(sun);
    }

    /**
     * Add a petagonal prism to the scene and PhysicsSpace.
     */
    private void addPrism() {
        float radius = 3f;
        float thickness = 1f;
        Mesh mesh = new Prism(5, radius, thickness);
        Geometry geometry = new Geometry("prism", mesh);
        addNode.attachChild(geometry);
        geometry.setMaterial(wireMaterial);

        testShape = CollisionShapeFactory.createDynamicMeshShape(geometry);
        float massForStatic = 0f;
        RigidBodyControl rbc = new RigidBodyControl(testShape, massForStatic);
        rbc.setPhysicsSpace(physicsSpace);
        geometry.addControl(rbc);
    }

    /**
     * Add 3x3 terrain to the scene and PhysicsSpace.
     */
    private void addSmallTerrain() {
        int patchSize = 3;
        int mapSize = 3;
        TerrainQuad quad
                = new TerrainQuad("terrain", patchSize, mapSize, nineHeights);
        addNode.attachChild(quad);
        quad.setMaterial(wireMaterial);

        testShape = CollisionShapeFactory.createMeshShape(quad);
        float massForStatic = 0f;
        RigidBodyControl rbc = new RigidBodyControl(testShape, massForStatic);
        rbc.setPhysicsSpace(physicsSpace);
        quad.addControl(rbc);
    }

    /**
     * Cast a physics ray from mouse pointer and move the PointVisualizer to the
     * nearest intersection.
     */
    private void castRay() {
        Ray mouseRay = MyCamera.mouseRay(cam, inputManager);
        Vector3f from = mouseRay.origin;
        float rayLength = cam.getFrustumFar();
        Vector3f to = mouseRay.direction.clone().scaleAdd(rayLength, from);
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
         * Remove all added shapes from the scene.
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
        dumper.setMaxChildren(3);
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        stateManager.attach(bulletAppState);
        physicsSpace = bulletAppState.getPhysicsSpace();
    }

    /**
     * Initialize materials during startup.
     */
    private void generateMaterials() {
        ColorRGBA green = new ColorRGBA(0f, 0.12f, 0f, 1f);
        greenMaterial = MyAsset.createShadedMaterial(assetManager, green);
        greenMaterial.setName("green");

        wireMaterial = MyAsset.createWireframeMaterial(assetManager, green);
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
     * Toggle visualization of collision-object bounding boxes.
     */
    private void toggleBoxes() {
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
     * Toggle rendering of added geometries on/off.
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
     * Update the status text in the GUI.
     */
    private void updateStatusText() {
        float margin = testShape.getMargin();
        String message
                = String.format("shape=%s, margin=%.3f", shapeName, margin);
        statusText.setText(message);
    }
}
