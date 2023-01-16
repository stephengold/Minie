/*
 Copyright (c) 2022-2023, Stephen Gold
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

import com.jme3.anim.AnimComposer;
import com.jme3.anim.SkinningControl;
import com.jme3.anim.tween.Tween;
import com.jme3.anim.tween.Tweens;
import com.jme3.anim.tween.action.Action;
import com.jme3.app.Application;
import com.jme3.app.StatsAppState;
import com.jme3.app.state.AppState;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.PhysicsSweepTestResult;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.ConvexShape;
import com.jme3.bullet.debug.DebugInitListener;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.font.BitmapText;
import com.jme3.input.CameraInput;
import com.jme3.input.KeyInput;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Box;
import com.jme3.system.AppSettings;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.prefs.BackingStoreException;
import jme3utilities.Heart;
import jme3utilities.MeshNormals;
import jme3utilities.MyAsset;
import jme3utilities.MyCamera;
import jme3utilities.MyString;
import jme3utilities.math.noise.Generator;
import jme3utilities.minie.test.common.PhysicsDemo;
import jme3utilities.minie.test.shape.ShapeGenerator;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.InputMode;

/**
 * Demonstrate the use of a sweep test with an animated model.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class SweepDemo
        extends PhysicsDemo
        implements DebugInitListener, PhysicsTickListener {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(SweepDemo.class.getName());
    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName
            = SweepDemo.class.getSimpleName();
    /**
     * action string to initiate the attack animation
     */
    final private static String asAttack = "attack";
    /**
     * location of the spawn point (in physics-space coordinates)
     */
    final private static Vector3f spawnLocation = new Vector3f(0f, 10f, 20f);
    /**
     * initial velocity for new projectiles (in physics-space coordinates)
     */
    final private static Vector3f spawnVelocity = new Vector3f(0f, 0.2f, -2f);
    // *************************************************************************
    // fields

    /**
     * play the "Punches" animation for one cycle before returning to idle
     */
    private static Action punchesOnce;
    /**
     * for playing canned animations
     */
    private static AnimComposer composer;
    /**
     * text displayed in the upper-left corner of the GUI node
     */
    private static BitmapText statusText;
    /**
     * to manage the PhysicsSpace
     */
    private static BulletAppState bulletAppState;
    /**
     * collision shape for sweep tests
     */
    private static ConvexShape saberShape;
    /**
     * results of the latest sweep test
     */
    final private static List<PhysicsSweepTestResult> sweepResults
            = new ArrayList<>(10);
    /**
     * visualize the blade of Jaime's saber
     */
    private static Spatial saberSpatial;
    /**
     * previous position of the saber (in physics-space coordinates) or null if
     * not initialized
     */
    private static Transform previousPosition;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the SweepDemo application.
     */
    public SweepDemo() { // made explicit to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the SweepDemo application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        String title = applicationName + " " + MyString.join(arguments);

        // Mute the chatty loggers in certain packages.
        Heart.setLoggingLevels(Level.WARNING);

        boolean loadDefaults = true;
        AppSettings settings = new AppSettings(loadDefaults);
        try {
            settings.load(applicationName);
        } catch (BackingStoreException exception) {
            logger.warning("Failed to load AppSettings.");
        }
        settings.setAudioRenderer(null);
        settings.setResizable(true);
        settings.setSamples(4); // anti-aliasing
        settings.setTitle(title); // Customize the window's title bar.

        Application application = new SweepDemo();
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
        // Add the status text to the GUI.
        statusText = new BitmapText(guiFont);
        guiNode.attachChild(statusText);

        super.acorusInit();

        addLighting(rootNode);
        configureCamera();
        configurePhysics();
        generateMaterials();

        ColorRGBA skyColor = new ColorRGBA(0.1f, 0.2f, 0.4f, 1f);
        viewPort.setBackgroundColor(skyColor);

        // Hide the render-statistics overlay.
        stateManager.getState(StatsAppState.class).toggleStats();

        float halfExtent = 10f;
        float topY = 0f;
        attachCubePlatform(halfExtent, topY);

        Node model = (Node) assetManager
                .loadModel("Models/Jaime/Jaime-new.j3o");
        Geometry g = (Geometry) model.getChild(0);
        RenderState rs = g.getMaterial().getAdditionalRenderState();
        rs.setFaceCullMode(RenderState.FaceCullMode.Off);

        model.setLocalScale(10f);
        centerCgm(model);
        rootNode.attachChild(model);

        Vector3f halfExtents = new Vector3f(0.3f, 0.02f, 0.02f);
        Box box = new Box(halfExtents.x, halfExtents.y, halfExtents.z);
        saberSpatial = new Geometry("saber", box);
        saberSpatial.move(0.4f, 0.05f, 0.01f);
        Material idleMaterial = findMaterial("idleSaber");
        saberSpatial.setMaterial(idleMaterial);
        /*
         * Create an attachments node for Jaime's right hand,
         * and attach the saber to that Node.
         */
        SkinningControl sControl = model.getControl(SkinningControl.class);
        Node rightHandNode = sControl.getAttachmentsNode("hand.R");
        rightHandNode.attachChild(saberSpatial);
        Vector3f scale = saberSpatial.getWorldScale(); // alias
        /*
         * Define a "PunchesOnce" animation sequence
         * to play one cycle of the "Punches" clip and then return to idle.
         */
        composer = model.getControl(AnimComposer.class);
        Action punches = composer.action("Punches");
        Tween doneTween
                = Tweens.callMethod(composer, "setCurrentAction", "Idle");
        punchesOnce
                = composer.actionSequence("PunchesOnce", punches, doneTween);

        // Begin playing the "Idle" animation at half speed.
        composer.setCurrentAction("Idle");
        composer.setGlobalSpeed(0.5f);

        saberShape = new BoxCollisionShape(halfExtents);
        saberShape.setScale(scale); // Scale the shape to match the Spatial!

        launchProjectile();
    }

    /**
     * Initialize the library of named materials during startup.
     */
    @Override
    public void generateMaterials() {
        super.generateMaterials();

        ColorRGBA gray = new ColorRGBA(0.7f, 0.7f, 0.7f, 1f);
        Material projectile
                = MyAsset.createShadedMaterial(assetManager, gray);
        registerMaterial("projectile", projectile);

        Material busySaber
                = assetManager.loadMaterial("Common/Materials/RedColor.j3m");
        registerMaterial("busySaber", busySaber);

        Material idleSaber
                = assetManager.loadMaterial("Common/Materials/WhiteColor.j3m");
        registerMaterial("idleSaber", idleSaber);
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
        return 4f;
    }

    /**
     * Add application-specific hotkey bindings (and override existing ones, if
     * necessary).
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        dim.bind(asAttack, KeyInput.KEY_SPACE, KeyInput.KEY_RETURN);
        dim.bind(asDumpSpace, KeyInput.KEY_O);
        dim.bind(asDumpViewport, KeyInput.KEY_P);

        dim.bindSignal(CameraInput.FLYCAM_LOWER, KeyInput.KEY_DOWN);
        dim.bindSignal(CameraInput.FLYCAM_RISE, KeyInput.KEY_UP);
        dim.bindSignal("orbitLeft", KeyInput.KEY_LEFT);
        dim.bindSignal("orbitRight", KeyInput.KEY_RIGHT);

        dim.bind(asToggleAabbs, KeyInput.KEY_APOSTROPHE);
        dim.bind(asToggleHelp, KeyInput.KEY_H);
        dim.bind(asTogglePause, KeyInput.KEY_PAUSE, KeyInput.KEY_PERIOD);
        dim.bind(asTogglePcoAxes, KeyInput.KEY_SEMICOLON);
        dim.bind(asToggleVArrows, KeyInput.KEY_K);
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
        if (ongoing && actionString.equals(asAttack)) {
            if (isSaberReady()) {
                composer.setCurrentAction("PunchesOnce");
            }
            return;
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
        statusText.setLocalTranslation(0f, newHeight, 0f);
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

        updateSaberColor();
        updateStatusText();

        Transform currentPosition = saberSpatial.getWorldTransform(); // alias
        if (previousPosition == null) {
            previousPosition = new Transform();

        } else {
            /*
             * Use a sweep test to check for contact
             * between any projectile and the saber.
             */
            PhysicsSpace space = getPhysicsSpace();
            sweepResults.clear();
            space.sweepTest(saberShape, previousPosition, currentPosition,
                    sweepResults);
            for (PhysicsSweepTestResult result : sweepResults) {
                PhysicsCollisionObject pco = result.getCollisionObject();
                if (pco.isInWorld()) {
                    space.removeCollisionObject(pco);
                }
            }
        }
        previousPosition.set(currentPosition);
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
    // PhysicsTickListener methods

    /**
     * Callback from Bullet, invoked just before the physics is stepped.
     *
     * @param space the space that's about to be stepped (not null)
     * @param timeStep the time per simulation step (in seconds, &ge;0)
     */
    @Override
    public void prePhysicsTick(PhysicsSpace space, float timeStep) {
        Random random = getGenerator();
        if (random.nextFloat() < 0.2f * timeStep) {
            launchProjectile();
        }
    }

    /**
     * Callback from Bullet, invoked just after the physics has been stepped.
     *
     * @param space the space that was just stepped (not null)
     * @param timeStep the time per simulation step (in seconds, &ge;0)
     */
    @Override
    public void physicsTick(PhysicsSpace space, float timeStep) {
        // do nothing
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
        Vector3f direction = new Vector3f(-1f, -1f, -2f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction, directColor);
        rootSpatial.addLight(sun);
        sun.setName("sun");
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        float near = 1f;
        float far = 900f;
        MyCamera.setNearFar(cam, near, far);

        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(50f); // default = 3

        cam.setLocation(new Vector3f(41f, 3.6f, 10.5f));
        cam.setRotation(new Quaternion(0.002f, -0.76744f, 0.0022f, 0.6411f));

        AppState orbitState
                = new CameraOrbitAppState(cam, "orbitLeft", "orbitRight");
        stateManager.attach(orbitState);
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        PhysicsBody.setDeactivationEnabled(false); // default = true

        bulletAppState = new BulletAppState();
        bulletAppState.setDebugEnabled(true); // default = false

        // Add lighting to the debug scene.
        bulletAppState.setDebugInitListener((Node physicsDebugRootNode)
                -> addLighting(physicsDebugRootNode)
        );

        stateManager.attach(bulletAppState);
        PhysicsSpace space = getPhysicsSpace();
        space.setGravity(new Vector3f(0f, -0.2f, 0f));
        space.addTickListener(this);
    }

    /**
     * Test whether the saber is ready to attack.
     *
     * @return true if ready, otherwise false
     */
    private static boolean isSaberReady() {
        boolean isReady = composer.getCurrentAction() != punchesOnce;
        return isReady;
    }

    /**
     * Launch a rigid body toward Jaime.
     */
    private void launchProjectile() {
        // Generate a pseudo-random shape.
        CollisionShape shape;
        ShapeGenerator generator = getGenerator();
        int shapeIndex = generator.nextInt(5);
        switch (shapeIndex) {
            case 0:
                shape = generator.nextBox();
                break;
            case 1:
                shape = generator.nextCone();
                break;
            case 2:
                shape = generator.nextFootball();
                break;
            case 3:
                shape = generator.nextPlatonic();
                break;
            case 4:
                shape = generator.nextStar();
                break;
            default:
                throw new IllegalStateException();
        }

        PhysicsRigidBody body = new PhysicsRigidBody(shape);
        Material material = findMaterial("projectile");
        body.setDebugMaterial(material);

        body.setDebugMeshNormals(MeshNormals.Facet);
        body.setPhysicsLocation(spawnLocation);
        body.setLinearVelocity(spawnVelocity);

        Generator random = getGenerator();
        Quaternion rotation = random.nextQuaternion();
        body.setPhysicsRotation(rotation);

        addCollisionObject(body);
    }

    /**
     * Update the color of the saber to indicate whether it is ready or not.
     */
    private void updateSaberColor() {
        Material material;
        if (isSaberReady()) {
            material = findMaterial("idleSaber");
        } else {
            material = findMaterial("busySaber");
        }
        saberSpatial.setMaterial(material);
    }

    /**
     * Update the status text in the GUI.
     */
    private void updateStatusText() {
        String message = isPaused() ? "PAUSED" : "";
        statusText.setText(message);
    }
}
