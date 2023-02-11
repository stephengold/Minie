/*
 Copyright (c) 2019-2023, Stephen Gold
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
import com.jme3.anim.tween.action.Action;
import com.jme3.anim.util.AnimMigrationUtils;
import com.jme3.app.Application;
import com.jme3.app.state.AppState;
import com.jme3.bounding.BoundingBox;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSoftSpace;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.SoftPhysicsAppState;
import com.jme3.bullet.animation.DynamicAnimControl;
import com.jme3.bullet.animation.PhysicsLink;
import com.jme3.bullet.animation.RagUtils;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.debug.DebugInitListener;
import com.jme3.bullet.debug.DebugMeshInitListener;
import com.jme3.bullet.joints.Anchor;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.bullet.objects.infos.Aero;
import com.jme3.bullet.objects.infos.Sbcp;
import com.jme3.bullet.objects.infos.SoftBodyConfig;
import com.jme3.bullet.objects.infos.SoftBodyMaterial;
import com.jme3.bullet.util.NativeSoftBodyUtil;
import com.jme3.font.BitmapText;
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
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer;
import com.jme3.shadow.DirectionalLightShadowRenderer;
import com.jme3.system.AppSettings;
import com.jme3.texture.Texture;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.Collection;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MeshNormals;
import jme3utilities.MyAsset;
import jme3utilities.MyCamera;
import jme3utilities.MySpatial;
import jme3utilities.MyString;
import jme3utilities.math.MyArray;
import jme3utilities.math.MyVector3f;
import jme3utilities.mesh.ClothGrid;
import jme3utilities.mesh.Icosphere;
import jme3utilities.minie.DumpFlags;
import jme3utilities.minie.FilterAll;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.minie.test.common.PhysicsDemo;
import jme3utilities.minie.test.tunings.PuppetControl;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.InputMode;

/**
 * Test/demonstrate soft-body physics.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestSoftBody
        extends PhysicsDemo
        implements DebugInitListener {
    // *************************************************************************
    // constants and loggers

    /**
     * number of axes in a vector
     */
    final private static int numAxes = 3;
    /**
     * number of lines of text in the overlay
     */
    final private static int numStatusLines = 2;
    /**
     * add UVs to the debug mesh of a flag
     */
    final private static DebugMeshInitListener flagDmiListener
            = new DebugMeshInitListener() {
        @Override
        public void debugMeshInit(Mesh debugMesh) {
            FloatBuffer positions
                    = debugMesh.getFloatBuffer(VertexBuffer.Type.Position);
            int numVertices = positions.limit() / numAxes;
            FloatBuffer uvs = BufferUtils.createFloatBuffer(2 * numVertices);
            debugMesh.setBuffer(VertexBuffer.Type.TexCoord, 2, uvs);
            for (int vertexI = 0; vertexI < numVertices; ++vertexI) {
                float x = positions.get(numAxes * vertexI);
                float y = positions.get(numAxes * vertexI + 1);
                float u = x - 0.5f;
                float v = 2f - y;
                uvs.put(u).put(v);
            }
            uvs.flip();
        }
    };
    /**
     * add UVs to the debug mesh of a tablecloth
     */
    final private static DebugMeshInitListener tableclothDmiListener
            = new DebugMeshInitListener() {
        @Override
        public void debugMeshInit(Mesh debugMesh) {
            FloatBuffer positions
                    = debugMesh.getFloatBuffer(VertexBuffer.Type.Position);
            int numVertices = positions.limit() / numAxes;
            FloatBuffer uvs = BufferUtils.createFloatBuffer(2 * numVertices);
            debugMesh.setBuffer(VertexBuffer.Type.TexCoord, 2, uvs);
            for (int vertexI = 0; vertexI < numVertices; ++vertexI) {
                float x = positions.get(numAxes * vertexI);
                float z = positions.get(numAxes * vertexI + 2);
                float u = 12f * x;
                float v = 12f * z;
                uvs.put(u).put(v);
            }
            uvs.flip();
        }
    };
    /**
     * mass of each soft body (&gt;0)
     */
    final private static float mass = 1f;
    /**
     * indices of the waistline vertices in the Puppet model, arranged clockwise
     * as seen from above, starting at her right hip
     */
    final private static int[] waistlineVertices = {
        2396, 2394, 569, 545, 553, 554, 562, 2401};
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(TestSoftBody.class.getName());
    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName
            = TestSoftBody.class.getSimpleName();
    // *************************************************************************
    // fields

    /**
     * lines of text displayed in the upper-left corner of the GUI node ([0] is
     * the top line)
     */
    final private static BitmapText[] statusLines
            = new BitmapText[numStatusLines];
    /**
     * invisible physics objects
     */
    final private static FilterAll hiddenObjects = new FilterAll(true);
    /**
     * space for physics simulation
     */
    private static PhysicsSoftSpace physicsSpace;
    /**
     * AppState to manage the PhysicsSpace
     */
    private static SoftPhysicsAppState bulletAppState;
    /**
     * name of the test being run
     */
    private static String testName = "puppetInSkirt";
    // *************************************************************************
    // constructors

    /**
     * Instantiate the TestSoftBody application.
     */
    public TestSoftBody() { // explicit to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the TestSoftBody application.
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
        settings.setVSync(false);

        Application application = new TestSoftBody();
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

        configureCamera();
        configureDumper();
        generateMaterials();
        configurePhysics();

        ColorRGBA skyColor = new ColorRGBA(0.1f, 0.2f, 0.4f, 1f);
        viewPort.setBackgroundColor(skyColor);

        addLighting(rootNode, false);

        float halfExtent = 4f;
        float topY = 0f;
        attachCubePlatform(halfExtent, topY);

        DynamicAnimControl dac = addPuppet();
        addSkirt(dac);
    }

    /**
     * Configure the PhysicsDumper during startup.
     */
    @Override
    public void configureDumper() {
        PhysicsDumper dumper = getDumper();
        dumper.setEnabled(DumpFlags.MatParams, true);
        //dumper.setEnabled(DumpFlags.NativeIDs, true);
        //dumper.setEnabled(DumpFlags.NodesInSofts, true);
        dumper.setEnabled(DumpFlags.ShadowModes, true);
        dumper.setEnabled(DumpFlags.Transforms, true);
    }

    /**
     * Calculate screen bounds for a detailed help node.
     *
     * @param viewPortWidth (in pixels, &gt;0)
     * @param viewPortHeight (in pixels, &gt;0)
     * @return a new instance
     */
    @Override
    public Rectangle detailedHelpBounds(int viewPortWidth, int viewPortHeight) {
        // Position help nodes just below the status lines.
        float margin = 10f; // in pixels
        float width = viewPortWidth - 2f * margin;
        float height = viewPortHeight - (2f * margin + numStatusLines * 20f);
        float leftX = margin;
        float topY = margin + height;
        Rectangle result = new Rectangle(leftX, topY, width, height);

        return result;
    }

    /**
     * Initialize the library of named materials during startup.
     */
    @Override
    public void generateMaterials() {
        super.generateMaterials();

        Texture plaid
                = MyAsset.loadTexture(assetManager, "Textures/plaid.png", true);
        plaid.setAnisotropicFilter(8);
        plaid.setWrap(Texture.WrapMode.Repeat);
        Material plaidMaterial
                = MyAsset.createShadedMaterial(assetManager, plaid);
        RenderState renderState = plaidMaterial.getAdditionalRenderState();
        renderState.setFaceCullMode(RenderState.FaceCullMode.Off);
        registerMaterial("plaid", plaidMaterial);

        Texture texture = MyAsset.loadTexture(
                assetManager, "Interface/Logo/Monkey.png", true);
        Material logoMaterial
                = MyAsset.createShadedMaterial(assetManager, texture);
        renderState = logoMaterial.getAdditionalRenderState();
        renderState.setFaceCullMode(RenderState.FaceCullMode.Off);
        registerMaterial("logo", logoMaterial);

        ColorRGBA pink = new ColorRGBA(1.2f, 0.2f, 0.1f, 1f);
        Material pinkMaterial = MyAsset.createShinyMaterial(assetManager, pink);
        pinkMaterial.setFloat("Shininess", 4f);
        renderState = pinkMaterial.getAdditionalRenderState();
        renderState.setFaceCullMode(RenderState.FaceCullMode.Off);
        registerMaterial("pink", pinkMaterial);

        ColorRGBA red = new ColorRGBA(0.7f, 0.01f, 0.01f, 1f);
        Material redMaterial = MyAsset.createShadedMaterial(assetManager, red);
        redMaterial.setColor(
                "Specular", new ColorRGBA(0.05f, 0.05f, 0.05f, 1f));
        redMaterial.setFloat("Shininess", 4f);
        renderState = redMaterial.getAdditionalRenderState();
        renderState.setFaceCullMode(RenderState.FaceCullMode.Off);
        registerMaterial("red", redMaterial);
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
        return 0.5f;
    }

    /**
     * Add application-specific hotkey bindings (and override existing ones, if
     * necessary).
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        dim.bind(asDumpSpace, KeyInput.KEY_O);
        dim.bind(asDumpViewport, KeyInput.KEY_P);

        dim.bind(asCollectGarbage, KeyInput.KEY_G);
        dim.bind("go limp", KeyInput.KEY_SPACE);
        dim.bind("next", KeyInput.KEY_N);

        dim.bindSignal(CameraInput.FLYCAM_LOWER, KeyInput.KEY_DOWN);
        dim.bindSignal(CameraInput.FLYCAM_RISE, KeyInput.KEY_UP);
        dim.bindSignal("orbitLeft", KeyInput.KEY_LEFT);
        dim.bindSignal("orbitRight", KeyInput.KEY_RIGHT);

        dim.bind("test poleAndFlag", KeyInput.KEY_F3);
        dim.bind("test puppetInSkirt", KeyInput.KEY_F4);
        dim.bind("test squishyBall", KeyInput.KEY_F1);
        dim.bind("test tablecloth", KeyInput.KEY_F2);

        dim.bind(asToggleAabbs, KeyInput.KEY_APOSTROPHE);
        dim.bind(asToggleHelp, KeyInput.KEY_H);
        dim.bind(asTogglePause, KeyInput.KEY_PAUSE, KeyInput.KEY_PERIOD);
        dim.bind(asTogglePcoAxes, KeyInput.KEY_SEMICOLON);
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
        final float halfExtent = 4f;
        float topY;

        if (ongoing) {
            switch (actionString) {
                case "go limp":
                    goLimp();
                    return;

                case "next":
                    nextPuppetAnimation();
                    nextWindVelocity();
                    return;

                case "test poleAndFlag":
                    testName = "poleAndFlag";
                    cleanupAfterTest();

                    float length = 0.4f;
                    attachWorldAxes(length);

                    topY = -2f;
                    attachCubePlatform(halfExtent, topY);

                    addPoleAndFlag();
                    return;

                case "test puppetInSkirt":
                    testName = "puppetInSkirt";
                    cleanupAfterTest();

                    topY = 0f;
                    attachCubePlatform(halfExtent, topY);

                    DynamicAnimControl dac = addPuppet();
                    addSkirt(dac);
                    return;

                case "test squishyBall":
                    testName = "squishyBall";
                    cleanupAfterTest();

                    topY = 0f;
                    attachCubePlatform(halfExtent, topY);

                    addSquishyBall(1.5f);
                    return;

                case "test tablecloth":
                    testName = "tablecloth";
                    cleanupAfterTest();

                    topY = -1f;
                    attachCubePlatform(halfExtent, topY);

                    topY = 1.7f;
                    addCylinder(topY);

                    float startY = 2f;
                    addTablecloth(startY);
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
    // DebugInitListener methods

    /**
     * Callback from BulletDebugAppState, invoked just before the debug scene is
     * added to the debug viewports.
     *
     * @param physicsDebugRootNode the root node of the debug scene (not null)
     */
    @Override
    public void bulletDebugInit(Node physicsDebugRootNode) {
        boolean shadowFlag = true;
        addLighting(physicsDebugRootNode, shadowFlag);
    }
    // *************************************************************************
    // private methods

    /**
     * Add a static cylinder to the scene, to serve as an obstacle.
     *
     * @param topY the Y coordinate of the top surface (in physics-space
     * coordinates)
     */
    private static void addCylinder(float topY) {
        float radius = 1f;
        float height = 0.4f;
        CollisionShape shape = new CylinderCollisionShape(
                radius, height, PhysicsSpace.AXIS_Y);
        PhysicsRigidBody cylinderBody
                = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);

        Vector3f translation = new Vector3f(0f, topY - height / 2f, 0f);
        cylinderBody.setPhysicsLocation(translation);

        physicsSpace.addCollisionObject(cylinderBody);
        hiddenObjects.addException(cylinderBody);
    }

    /**
     * Add lighting and shadows to the specified scene.
     *
     * @param rootSpatial which scene (not null)
     * @param shadowFlag if true, add a shadow renderer to the default viewport
     */
    private void addLighting(Spatial rootSpatial, boolean shadowFlag) {
        ColorRGBA ambientColor = new ColorRGBA(0.1f, 0.1f, 0.1f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        rootSpatial.addLight(ambient);
        ambient.setName("ambient");

        ColorRGBA directColor = new ColorRGBA(0.7f, 0.7f, 0.7f, 1f);
        Vector3f direction = new Vector3f(1f, -2f, -2f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction, directColor);
        rootSpatial.addLight(sun);
        sun.setName("sun");

        rootSpatial.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);
        if (shadowFlag) {
            int mapSize = 2_048; // in pixels
            int numSplits = 3;
            DirectionalLightShadowRenderer dlsr
                    = new DirectionalLightShadowRenderer(
                            assetManager, mapSize, numSplits);
            dlsr.setLight(sun);
            dlsr.setShadowIntensity(0.5f);
            viewPort.addProcessor(dlsr);
        }
    }

    /**
     * Add a static pole with a rectangular flag.
     */
    private void addPoleAndFlag() {
        float radius = 0.06f;
        float height = 4f;
        CollisionShape shape = new CylinderCollisionShape(
                radius, height, PhysicsSpace.AXIS_Y);
        PhysicsRigidBody polePrb
                = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);

        ColorRGBA color = new ColorRGBA(0.7f, 0.7f, 1f, 1f);
        Material material = MyAsset.createShadedMaterial(assetManager, color);
        polePrb.setDebugMaterial(material);
        polePrb.setDebugMeshNormals(MeshNormals.Smooth);

        physicsSpace.addCollisionObject(polePrb);

        int xLines = 20;
        int zLines = 2 * xLines; // 2x as wide as it is tall
        float width = 2f;
        float spacing = width / zLines;
        Mesh mesh = new ClothGrid(xLines, zLines, spacing);
        PhysicsSoftBody flagPsb = new PhysicsSoftBody();

        NativeSoftBodyUtil.appendFromTriMesh(mesh, flagPsb);
        flagPsb.setMargin(0.1f);
        flagPsb.setMass(mass);

        Vector3f wind = new Vector3f(2.5f, 0f, -0.5f);
        flagPsb.setWindVelocity(wind);

        SoftBodyConfig config = flagPsb.getSoftConfig();
        config.set(Sbcp.Damping, 0.01f);
        config.set(Sbcp.Drag, 0.5f);
        config.set(Sbcp.Lift, 1f);
        config.setAerodynamics(Aero.F_TwoSidedLiftDrag);
        config.setPositionIterations(3);

        SoftBodyMaterial softMaterial = flagPsb.getSoftMaterial();
        softMaterial.setAngularStiffness(0f);

        Material logoMaterial = findMaterial("logo");
        flagPsb.setDebugMaterial(logoMaterial);
        flagPsb.setDebugMeshInitListener(flagDmiListener);
        flagPsb.setDebugMeshNormals(MeshNormals.Smooth);

        Quaternion rotation = new Quaternion();
        rotation.fromAngles(FastMath.HALF_PI, 0f, 0f);
        flagPsb.applyRotation(rotation);
        flagPsb.setPhysicsLocation(new Vector3f(1f, 1.5f, 0f));

        physicsSpace.addCollisionObject(flagPsb);

        // Add 2 anchors that join the flag to the pole.
        boolean allowCollisions = true;
        int nodeIndex = 0; // upper left corner of flag
        Vector3f initialLocation = flagPsb.nodeLocation(nodeIndex, null);
        Anchor anchor0 = new Anchor(
                flagPsb, nodeIndex, polePrb, initialLocation, allowCollisions);
        physicsSpace.addJoint(anchor0);

        nodeIndex = xLines - 1; // lower left corner of flag
        flagPsb.nodeLocation(nodeIndex, initialLocation);
        Anchor anchor1 = new Anchor(
                flagPsb, nodeIndex, polePrb, initialLocation, allowCollisions);
        physicsSpace.addJoint(anchor1);
    }

    /**
     * Add a Puppet model.
     *
     * @return the new instance (not null)
     */
    private DynamicAnimControl addPuppet() {
        // Load the model in "T" pose.
        Spatial cgModel = assetManager.loadModel("Models/Puppet/Puppet.j3o");
        AnimMigrationUtils.migrate(cgModel);
        rootNode.attachChild(cgModel);
        SkinningControl sc = (SkinningControl) RagUtils.findSControl(cgModel);
        Spatial controlledSpatial = sc.getSpatial();

        // Configure and add her physics control.
        DynamicAnimControl dac = new PuppetControl();
        controlledSpatial.addControl(dac);
        dac.setPhysicsSpace(physicsSpace);

        // Don't visualize her rigid bodies.
        PhysicsRigidBody[] rigids = dac.listRigidBodies();
        for (PhysicsRigidBody rigid : rigids) {
            hiddenObjects.addException(rigid);
        }

        return dac;
    }

    /**
     * Add a wraparound skirt to the specified Puppet model.
     *
     * @param puppetDac the model's physics control (not null)
     */
    private void addSkirt(DynamicAnimControl puppetDac) {
        int numDivisions = 5;
        int numAnchors = 51;
        float length = 0.57f;
        Vector3f[] anchorLocs = new Vector3f[numAnchors];
        for (int zIndex = 0; zIndex < numAnchors; ++zIndex) {
            anchorLocs[zIndex] = new Vector3f();
        }
        Mesh mesh
                = createSkirtMesh(puppetDac, numDivisions, length, anchorLocs);

        // Create and configure the soft body.
        PhysicsSoftBody skirtPsb = new PhysicsSoftBody();
        NativeSoftBodyUtil.appendFromTriMesh(mesh, skirtPsb);
        skirtPsb.setMargin(0.1f);
        skirtPsb.setMass(0.02f);

        SoftBodyConfig config = skirtPsb.getSoftConfig();
        config.set(Sbcp.AnchorHardness, 1f);
        config.set(Sbcp.KineticHardness, 1f);
        config.setPositionIterations(6);

        SoftBodyMaterial material = skirtPsb.getSoftMaterial();
        material.setAngularStiffness(0f);
        material.setLinearStiffness(0.5f);

        Material redMaterial = findMaterial("red");
        skirtPsb.setDebugMaterial(redMaterial);
        skirtPsb.setDebugMeshNormals(MeshNormals.Smooth);

        String vSpec0 = puppetVSpec(0);
        PhysicsLink link = puppetDac.findManagerForVertex(vSpec0, null, null);
        Transform localToWorld = link.physicsTransform(null);
        skirtPsb.applyTransform(localToWorld);

        physicsSpace.addCollisionObject(skirtPsb);
        skirtPsb.setGravity(new Vector3f(0f, -10f, 0f));

        // Add anchors that join Puppet to her skirt.
        PhysicsRigidBody rigid = link.getRigidBody();
        boolean allowCollisions = true;
        for (int anchorIndex = 0; anchorIndex < numAnchors; ++anchorIndex) {
            Vector3f location = anchorLocs[anchorIndex];
            Anchor anchor = new Anchor(
                    skirtPsb, anchorIndex, rigid, location, allowCollisions);
            physicsSpace.addJoint(anchor);
        }
    }

    /**
     * Add a squishy ball to the scene.
     *
     * @param startY the desired initial Y coordinate of the center
     */
    private void addSquishyBall(float startY) {
        int numRefinementIterations = 3;
        float radius = 0.5f;
        Mesh mesh = new Icosphere(numRefinementIterations, radius);
        PhysicsSoftBody ballPsb = new PhysicsSoftBody();
        NativeSoftBodyUtil.appendFromTriMesh(mesh, ballPsb);
        ballPsb.setMass(mass);

        SoftBodyConfig config = ballPsb.getSoftConfig();
        config.set(Sbcp.PoseMatching, 0.02f);
        config.set(Sbcp.KineticHardness, 1f);

        boolean setVolumePose = false;
        boolean setFramePose = true;
        ballPsb.setPose(setVolumePose, setFramePose);

        Material pinkMaterial = findMaterial("pink");
        ballPsb.setDebugMaterial(pinkMaterial);
        ballPsb.setDebugMeshNormals(MeshNormals.Smooth);

        Vector3f translation = new Vector3f(0f, startY, 0f);
        ballPsb.applyTranslation(translation);

        physicsSpace.addCollisionObject(ballPsb);
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
     * Add a square tablecloth to the scene.
     *
     * @param startY the desired initial Y coordinate of the cloth
     */
    private void addTablecloth(float startY) {
        int numLines = 40;
        float spacing = 0.08f;
        Mesh mesh = new ClothGrid(numLines, numLines, spacing);
        PhysicsSoftBody softBody = new PhysicsSoftBody();
        NativeSoftBodyUtil.appendFromTriMesh(mesh, softBody);
        softBody.setMass(mass);

        SoftBodyConfig config = softBody.getSoftConfig();
        config.set(Sbcp.Damping, 0.02f);
        config.setPositionIterations(3);

        SoftBodyMaterial material = softBody.getSoftMaterial();
        material.setAngularStiffness(0f);

        Material plaidMaterial = findMaterial("plaid");
        softBody.setDebugMaterial(plaidMaterial);
        softBody.setDebugMeshInitListener(tableclothDmiListener);
        softBody.setDebugMeshNormals(MeshNormals.Smooth);

        Vector3f translation = new Vector3f(0f, startY, 0f);
        softBody.applyTranslation(translation);

        physicsSpace.addCollisionObject(softBody);
    }

    /**
     * Clean up after a test.
     */
    private void cleanupAfterTest() {
        // Remove any scenery. Debug meshes are under a different root node.
        rootNode.detachAllChildren();

        // Remove physics objects, which also removes their debug meshes.
        physicsSpace.destroy();
        assert physicsSpace.isEmpty();

        // Clear the hidden-object list.
        hiddenObjects.clearExceptions();
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        float near = 0.02f;
        float far = 20f;
        MyCamera.setNearFar(cam, near, far);

        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(2f);
        flyCam.setZoomSpeed(2f);

        cam.setLocation(new Vector3f(0f, 2.2f, 3.9f));
        cam.setRotation(new Quaternion(0f, 0.98525f, -0.172f, 0f));

        AppState orbitState
                = new CameraOrbitAppState(cam, "orbitLeft", "orbitRight");
        stateManager.attach(orbitState);
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        CollisionShape.setDefaultMargin(0.005f); // 5-mm margin

        bulletAppState = new SoftPhysicsAppState();
        bulletAppState.setDebugEnabled(true);
        bulletAppState.setDebugFilter(hiddenObjects);
        bulletAppState.setDebugInitListener(this);
        stateManager.attach(bulletAppState);

        physicsSpace = bulletAppState.getPhysicsSoftSpace();
        physicsSpace.setAccuracy(0.01f); // 10-msec timestep
        setGravityAll(1f);
    }

    /**
     * Generate a Mesh for Puppet's skirt.
     *
     * @param puppetDac the model's physics control (not null)
     * @param numSubdiv the number of mesh squares between successive anchors
     * (&ge;1)
     * @param skirtLength the desired length (in physics-space units, &get;0)
     * @param local storage for waist locations in local coordinates (not null,
     * modified)
     * @return a new Mesh, fitted to the model in her bone's local coordinates
     */
    private static Mesh createSkirtMesh(DynamicAnimControl puppetDac,
            int numSubdiv, float skirtLength, Vector3f[] local) {
        int numWaistVerts = waistlineVertices.length;
        int numXLines = local.length;

        for (Vector3f vector3f : local) {
            vector3f.zero();
        }
        /*
         * Calculate locations (in local coordinates of the
         * rigid body) of the mesh vertices on Puppet's waistline.
         */
        float raiseWaistline = 0.04f;
        String vSpec = puppetVSpec(0);
        PhysicsLink link0 = puppetDac.findManagerForVertex(vSpec, null, null);
        for (int zIndex = 0; zIndex < numXLines; zIndex += numSubdiv) {
            int waistVertI = (zIndex / numSubdiv) % numWaistVerts;
            vSpec = puppetVSpec(waistVertI);
            Vector3f location = local[zIndex];
            PhysicsLink link
                    = puppetDac.findManagerForVertex(vSpec, null, location);
            assert link == link0 : link;
            location.z -= raiseWaistline;
        }
        /*
         * Expand the waistline outward (from the apex of the cone)
         * to provide a margin.  Also, compute maxRadius.
         */
        float margin = 0.02f;
        BoundingBox aabb = MyArray.aabb(local, null);
        Vector3f apex = aabb.getCenter(); // alias
        float raiseApex = 0.5f; // 0 = full-circle skirt, 0.5 = not very full
        apex.z -= raiseApex;
        float maxRadius = 0f;
        for (int zIndex = 0; zIndex < numXLines; zIndex += numSubdiv) {
            float radius = local[zIndex].distance(apex);
            float expandedRadius = radius + margin;
            local[zIndex].multLocal(expandedRadius / radius);

            maxRadius = Math.max(expandedRadius, maxRadius);
        }
        /*
         * Subdivide the waistline, interpolating numSubdiv-1 soft-body nodes
         * between each pair of successive Puppet vertices.
         * Also, calculate its circumference.
         */
        float circumference = 0f;
        for (int zIndex = numSubdiv; zIndex < numXLines; zIndex += numSubdiv) {
            int prevZi = zIndex - numSubdiv;
            float vSpacing = local[zIndex].distance(local[prevZi]);
            circumference += vSpacing;

            for (int zi = prevZi + 1; zi < zIndex; ++zi) {
                float t = ((float) (zi - prevZi)) / numSubdiv;
                MyVector3f.lerp(t, local[prevZi], local[zIndex], local[zi]);
            }
        }

        // Generate the mesh topology.
        float averageSpacing = circumference / (numXLines - 1);
        int numZLines = Math.round(0.3f * skirtLength / averageSpacing);
        ClothGrid mesh = new ClothGrid(numXLines, numZLines, averageSpacing);
        /*
         * Scale the waistline outward from the apex.
         * Apply the resulting locations to deform the ClothGrid
         * into a roughly conical shape.
         */
        float hemRadius = maxRadius + skirtLength;
        Vector3f offset = new Vector3f();
        Vector3f hemLocation = new Vector3f();
        Vector3f tmpLocation = new Vector3f();
        for (int zIndex = 0; zIndex < numXLines; ++zIndex) {
            local[zIndex].subtract(apex, offset);
            float waistRadius = offset.length();
            offset.multLocal(hemRadius / waistRadius);
            offset.add(apex, hemLocation);

            for (int xIndex = 0; xIndex < numZLines; ++xIndex) {
                float t = ((float) xIndex) / (numZLines - 1);
                MyVector3f.lerp(t, local[zIndex], hemLocation, tmpLocation);
                mesh.reposition(xIndex, zIndex, tmpLocation);
            }
        }

        return mesh;
    }

    /**
     * If the scene contains exactly one DynamicAnimControl, and it's ready to
     * go dynamic, put it into ragdoll mode.
     */
    private void goLimp() {
        List<DynamicAnimControl> dacs = MySpatial
                .listControls(rootNode, DynamicAnimControl.class, null);
        if (dacs.size() == 1) {
            DynamicAnimControl dac = dacs.get(0);
            if (dac.isReady()) {
                dac.setRagdollMode();
            }
        }
    }

    /**
     * Cycle through animations of the Puppet model.
     */
    private void nextPuppetAnimation() {
        SkinningControl sc = (SkinningControl) RagUtils.findSControl(rootNode);
        if (sc == null) {
            return;
        }

        Spatial controlledSpatial = sc.getSpatial();
        AnimComposer composer
                = controlledSpatial.getControl(AnimComposer.class);

        Action jog = composer.action("jog");
        Action walk = composer.action("walk");

        Action action = composer.getCurrentAction();
        if (action == null) { // first time
            composer.setCurrentAction("jog");
        } else if (action == jog) {
            composer.setCurrentAction("walk");
        } else if (action == walk) {
            composer.setCurrentAction("jog");
        }
    }

    /**
     * Cycle through wind velocities.
     */
    private static void nextWindVelocity() {
        Collection<PhysicsSoftBody> softBodies = physicsSpace.getSoftBodyList();
        for (PhysicsSoftBody softBody : softBodies) {
            Vector3f windVelocity = softBody.windVelocity(null);
            float windSpeed = windVelocity.length();
            if (windSpeed > 1f) {
                windVelocity.divideLocal(4f);
            } else {
                windVelocity.multLocal(4f);
            }
            softBody.setWindVelocity(windVelocity);
        }
    }

    /**
     * Generate a specifier for the indexed vertex on Puppet's waistline.
     *
     * @param waistIndex (&ge;0, &lt;numWaistVertices)
     * @return the specifier string
     */
    private static String puppetVSpec(int waistIndex) {
        int puppetVertex = waistlineVertices[waistIndex];
        String vertexSpecifier = String.format("%d/Mesh.009_0", puppetVertex);

        return vertexSpecifier;
    }

    /**
     * Update the status lines in the GUI.
     */
    private void updateStatusLines() {
        String message = String.format(
                "Test: %s%s", testName, isPaused() ? "  PAUSED" : "");
        statusLines[0].setText(message);

        String viewOptions = describePhysicsDebugOptions();
        message = "View: " + viewOptions;
        statusLines[1].setText(message);
    }
}
