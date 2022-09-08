/*
 Copyright (c) 2020-2022, Stephen Gold
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
import com.jme3.bullet.SoftPhysicsAppState;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
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
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Triangle;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Limits;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Line;
import com.jme3.system.AppSettings;
import com.jme3.util.BufferUtils;
import java.util.Collection;
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
    private BulletAppState bulletAppState;
    /**
     * angle between the normal of the splitting plane and default camera's "up"
     * vector (in radians, &ge;0, &lt;Pi)
     */
    private float splitAngle = 0f;
    /**
     * visualize the splitting plane
     */
    private Geometry splitterGeometry;
    /**
     * AppState to manage the status overlay
     */
    private SplitDemoStatus status;
    /**
     * first screen location used to define the splitting plane (measured from
     * the lower left corner)
     */
    final private Vector2f screen1 = new Vector2f();
    /**
     * 2nd screen location used to define the splitting plane (measured from the
     * lower left corner)
     */
    final private Vector2f screen2 = new Vector2f();
    /**
     * first world location used to define the splitting plane
     */
    final private Vector3f world1 = new Vector3f();
    /**
     * 2nd world location used to define the splitting plane
     */
    final private Vector3f world2 = new Vector3f();
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
     * Restart the current scenario.
     */
    void restartScenario() {
        PhysicsSpace physicsSpace = getPhysicsSpace();
        physicsSpace.destroy();
        assert physicsSpace.isEmpty();

        setUpShape();
    }

    /**
     * Return the inclination angle of the splitting plane.
     *
     * @return the angle between the plane normal and the default camera's "up"
     * vector (in radians, &ge;0, &lt;Pi)
     */
    float splitAngle() {
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
        this.status = new SplitDemoStatus();
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
        this.splitterGeometry = new Geometry("plane", lineMesh);
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

        ColorRGBA red = new ColorRGBA(0.5f, 0f, 0f, 1f);
        Material missile = MyAsset.createShinyMaterial(assetManager, red);
        missile.setFloat("Shininess", 15f);
        registerMaterial("missile", missile); // TODO used?

        ColorRGBA color = new ColorRGBA(0.2f, 0f, 0f, 1f);
        Material solid = MyAsset.createShinyMaterial(assetManager, color);
        solid.setFloat("Shininess", 15f);
        RenderState additional = solid.getAdditionalRenderState();
        additional.setFaceCullMode(RenderState.FaceCullMode.Off);
        registerMaterial("solid", solid);
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
                KeyInput.KEY_NUMPAD0);

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
            this.splitAngle += tpf;
        }
        if (signals.test("rotatePlaneCw")) {
            this.splitAngle -= tpf;
        }
        this.splitAngle = MyMath.modulo(splitAngle, FastMath.PI);

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
     */
    private void addLighting(Spatial rootSpatial) {
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
     */
    private void addRigidBody(
            CollisionShape shape, MeshNormals debugMeshNormals) {
        PhysicsRigidBody body = new PhysicsRigidBody(shape);
        body.setDebugMeshNormals(debugMeshNormals);

        Generator random = getGenerator();
        Quaternion rotation = random.nextQuaternion(); // TODO garbage
        body.setPhysicsRotation(rotation);

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

        cam.setLocation(new Vector3f(0f, 1.5f, 6.8f));
        cam.setRotation(new Quaternion(-0.002f, 0.991408f, -0.1295f, 0.0184f));

        AppState orbitState
                = new CameraOrbitAppState(cam, "orbitLeft", "orbitRight");
        stateManager.attach(orbitState);
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        PhysicsBody.setDeactivationEnabled(false); // avoid a distraction

        this.bulletAppState = new SoftPhysicsAppState();
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
    private void makeSplittable(PhysicsRigidBody body) {
        CollisionShape oldShape = body.getCollisionShape();
        CollisionShape splittableShape = oldShape.toSplittableShape();
        assert splittableShape.canSplit();
        body.setCollisionShape(splittableShape);
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

        } else { // use the yellow lit/shaded material
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
        String shapeName = status.shapeName();
        CollisionShape shape;
        switch (shapeName) {
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
                addRigidBody(shape, MeshNormals.Facet);
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
                addRigidBody(shape, MeshNormals.Smooth);
                break;

            case "sphere":
                shape = random.nextShape(shapeName);
                addRigidBody(shape, MeshNormals.Sphere);
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
     * Split the specified rigid body using the plane of the specified triangle.
     *
     * @param oldBody (not null, added to the PhysicsSpace)
     * @param worldTriangle a triangle that defines the splitting plane (in
     * world coordinates, not null, unaffected)
     */
    private void splitBody(PhysicsRigidBody oldBody, Triangle worldTriangle) {
        // Transform the triangle to the shape coordinate system.
        Transform shapeToWorld = oldBody.getTransform(null);
        Triangle shapeTriangle
                = MyMath.transformInverse(shapeToWorld, worldTriangle, null);

        CollisionShape originalShape = oldBody.getCollisionShape();
        CollisionShape splittableShape = originalShape.toSplittableShape();
        assert splittableShape.canSplit();

        if (splittableShape instanceof HullCollisionShape) {
            HullCollisionShape hullShape = (HullCollisionShape) splittableShape;
            ChildCollisionShape[] children = hullShape.split(shapeTriangle);
            if (children[0] == null || children[1] == null) {
                // The split plane didn't intersect the hull.
                return;
            }

            CollisionShape[] shapes = new CollisionShape[2];
            float[] volumes = new float[2];
            Vector3f[] locations = new Vector3f[2];
            for (int i = 0; i < 2; ++i) {
                ChildCollisionShape child = children[i];
                shapes[i] = child.getShape();
                HullCollisionShape hull = (HullCollisionShape) shapes[i];
                volumes[i] = hull.scaledVolume();

                locations[i] = child.copyOffset(null);
            }

            Vector3f shapeNormal = shapeTriangle.getNormal(); // alias
            Vector3f worldNormal = worldTriangle.getNormal(); // alias
            splitBody(oldBody, worldNormal, shapeToWorld, shapeNormal,
                    shapes, volumes, locations);

        } else {
            String className = splittableShape.getClass().getSimpleName();
            System.out.println("Shape not split:  class=" + className);
            System.out.flush();
        }
    }

    /**
     * Split the specified rigid body into 2 using the specified shapes.
     *
     * @param oldBody (not null, added to the PhysicsSpace)
     * @param worldTriangle a triangle that defines the splitting plane (in
     * world coordinates, not null, unaffected)
     * @param shapeToWorld the body's shape-to-world coordinate transform (not
     * null, unaffected)
     * @param shapeNormal the normal of the splitting plane (in shape
     * coordinates, not null, unaffected)
     * @param shapes the shapes to apply (length=2, both not null)
     * @param volumes the estimated volumes of the shapes (length=2, both &gt;0)
     * @param locations the center locations (in physics-space coordinates,
     * length=2, both not null)
     */
    private void splitBody(PhysicsRigidBody oldBody, Vector3f worldNormal,
            Transform shapeToWorld, Vector3f shapeNormal,
            CollisionShape[] shapes, float[] volumes, Vector3f[] locations) {
        assert shapes.length == 2 : shapes.length;
        assert volumes.length == 2 : volumes.length;
        assert locations.length == 2 : locations.length;

        // Tweak the locations to create some separation.
        for (int i = 0; i < 2; ++i) {
            Vector3f location = locations[i];
            if (i == 0) {
                MyVector3f.accumulateScaled(location, shapeNormal, -0.04f);
            } else {
                MyVector3f.accumulateScaled(location, shapeNormal, +0.04f);
            }
            shapeToWorld.transformVector(location, location);
        }

        Vector3f[] velocities = new Vector3f[2];
        velocities[0] = oldBody.getLinearVelocity(null);
        velocities[1] = velocities[0].clone();

        // Tweak the linear velocities to enhance the separation.
        float deltaV = 0.04f;
        MyVector3f.accumulateScaled(velocities[0], worldNormal, -deltaV);
        MyVector3f.accumulateScaled(velocities[1], worldNormal, +deltaV);

        float totalVolume = volumes[0] + volumes[1];
        assert totalVolume > 0f : totalVolume;
        float totalMass = oldBody.getMass();

        Vector3f w = oldBody.getAngularVelocity(null);

        PhysicsSpace space = getPhysicsSpace();
        space.removeCollisionObject(oldBody);

        for (int i = 0; i < 2; ++i) {
            float mass = totalMass * volumes[i] / totalVolume;
            PhysicsRigidBody body = new PhysicsRigidBody(shapes[i], mass);
            body.setPhysicsLocation(locations[i]);
            body.setLinearVelocity(velocities[i]);
            body.setPhysicsRotation(shapeToWorld.getRotation());
            body.setAngularVelocity(w);
            body.setDebugMeshNormals(MeshNormals.Facet);
            addCollisionObject(body);
        }
    }
}
