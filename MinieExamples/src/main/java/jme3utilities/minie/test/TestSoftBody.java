/*
 Copyright (c) 2019, Stephen Gold
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

import com.jme3.animation.AnimChannel;
import com.jme3.animation.AnimControl;
import com.jme3.animation.SkeletonControl;
import com.jme3.app.Application;
import com.jme3.bounding.BoundingBox;
import com.jme3.bullet.PhysicsSoftSpace;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.SoftPhysicsAppState;
import com.jme3.bullet.animation.DynamicAnimControl;
import com.jme3.bullet.animation.PhysicsLink;
import com.jme3.bullet.animation.RagUtils;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.infos.DebugMeshNormals;
import com.jme3.bullet.debug.BulletDebugAppState;
import com.jme3.bullet.debug.DebugInitListener;
import com.jme3.bullet.debug.DebugMeshInitListener;
import com.jme3.bullet.joints.Anchor;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.bullet.objects.infos.Aero;
import com.jme3.bullet.objects.infos.Sbcp;
import com.jme3.bullet.objects.infos.SoftBodyConfig;
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
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Misc;
import jme3utilities.MyAsset;
import jme3utilities.MyCamera;
import jme3utilities.debug.AxesVisualizer;
import jme3utilities.math.MyArray;
import jme3utilities.math.MyVector3f;
import jme3utilities.minie.DumpFlags;
import jme3utilities.minie.FilterAll;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.minie.test.mesh.ClothGrid;
import jme3utilities.minie.test.mesh.Icosphere;
import jme3utilities.minie.test.tunings.PuppetControl;
import jme3utilities.ui.ActionApplication;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.InputMode;

/**
 * Demo/testbed for soft-body physics.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestSoftBody
        extends ActionApplication
        implements DebugInitListener {
    // *************************************************************************
    // constants and loggers

    /**
     * add UVs to the debug mesh of a flag
     */
    final private static DebugMeshInitListener flagDmiListener
            = new DebugMeshInitListener() {
        @Override
        public void debugMeshInit(Mesh debugMesh) {
            VertexBuffer pos = debugMesh.getBuffer(VertexBuffer.Type.Position);
            int numVertices = pos.getNumElements();
            FloatBuffer positions = (FloatBuffer) pos.getDataReadOnly();
            FloatBuffer uvs = BufferUtils.createFloatBuffer(2 * numVertices);
            debugMesh.setBuffer(VertexBuffer.Type.TexCoord, 2, uvs);
            for (int vertexI = 0; vertexI < numVertices; ++vertexI) {
                float x = positions.get(3 * vertexI);
                float y = positions.get(3 * vertexI + 1);
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
            VertexBuffer pos = debugMesh.getBuffer(VertexBuffer.Type.Position);
            int numVertices = pos.getNumElements();
            FloatBuffer positions = (FloatBuffer) pos.getDataReadOnly();
            FloatBuffer uvs = BufferUtils.createFloatBuffer(2 * numVertices);
            debugMesh.setBuffer(VertexBuffer.Type.TexCoord, 2, uvs);
            for (int vertexI = 0; vertexI < numVertices; ++vertexI) {
                float x = positions.get(3 * vertexI);
                float z = positions.get(3 * vertexI + 2);
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
    final private static int[] waistlineVertices = new int[]{2396, 2394, 569,
        545, 553, 554, 562, 2401};
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
     * AppState to manage the PhysicsSpace
     */
    final private SoftPhysicsAppState bulletAppState
            = new SoftPhysicsAppState();
    /**
     * filter to control visualization of axis-aligned bounding boxes
     */
    private BulletDebugAppState.DebugAppStateFilter bbFilter;
    /**
     * physics objects that are not to be visualized
     */
    final private FilterAll hiddenObjects = new FilterAll(true);
    /**
     * single-sided green material to visualize the platform
     */
    private Material greenMaterial;
    /**
     * double-sided logo material to visualize flags
     */
    private Material logoMaterial;
    /**
     * double-sided pink material to visualize 3-D soft bodies
     */
    private Material pinkMaterial;
    /**
     * double-sided plaid material to visualize tablecloths
     */
    private Material plaidMaterial;
    /**
     * double-sided red material to visualize clothing
     */
    private Material redMaterial;
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
    private PhysicsSoftSpace physicsSpace;
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the TestSoftBody application.
     *
     * @param ignored array of command-line arguments (not null)
     */
    public static void main(String[] ignored) {
        /*
         * Mute the chatty loggers in certain packages.
         */
        Misc.setLoggingLevels(Level.WARNING);

        Application application = new TestSoftBody();
        /*
         * Customize the window's title bar.
         */
        AppSettings settings = new AppSettings(true);
        settings.setTitle(applicationName);

        settings.setGammaCorrection(true);
        settings.setSamples(4); // anti-aliasing
        settings.setVSync(false);
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
        configureMaterials();
        configurePhysics();

        ColorRGBA bgColor = new ColorRGBA(0.2f, 0.2f, 1f, 1f);
        viewPort.setBackgroundColor(bgColor);
        addLighting(rootNode, false);

        addBox(0f);
        DynamicAnimControl dac = addPuppet();
        addSkirt(dac);
    }

    /**
     * Add application-specific hotkey bindings and override existing ones.
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        dim.bind("dump physicsSpace", KeyInput.KEY_O);
        dim.bind("dump scenes", KeyInput.KEY_P);
        dim.bind("next", KeyInput.KEY_N);

        dim.bind("signal " + CameraInput.FLYCAM_LOWER, KeyInput.KEY_DOWN);
        dim.bind("signal " + CameraInput.FLYCAM_RISE, KeyInput.KEY_UP);
        dim.bind("signal orbitLeft", KeyInput.KEY_LEFT);
        dim.bind("signal orbitRight", KeyInput.KEY_RIGHT);

        dim.bind("test poleAndFlag", KeyInput.KEY_F3);
        dim.bind("test puppetInSkirt", KeyInput.KEY_F4);
        dim.bind("test squishyBall", KeyInput.KEY_F1);
        dim.bind("test tablecloth", KeyInput.KEY_F2);

        dim.bind("toggle axes", KeyInput.KEY_SEMICOLON);
        dim.bind("toggle boxes", KeyInput.KEY_APOSTROPHE);
        dim.bind("toggle help", KeyInput.KEY_H);
        dim.bind("toggle pause", KeyInput.KEY_PERIOD);

        float x = 10f;
        float y = cam.getHeight() - 10f;
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
                case "dump physicsSpace":
                    dumper.dump(physicsSpace);
                    return;

                case "dump scenes":
                    dumper.dump(renderManager);
                    return;

                case "next":
                    nextPuppetAnimation();
                    nextWindVelocity();
                    return;

                case "test poleAndFlag":
                    cleanupAfterTest();
                    addAxes();
                    addBox(-2f);
                    addPoleAndFlag();
                    return;

                case "test puppetInSkirt":
                    cleanupAfterTest();
                    addBox(0f);
                    DynamicAnimControl dac = addPuppet();
                    addSkirt(dac);
                    return;

                case "test squishyBall":
                    cleanupAfterTest();
                    addBox(0f);
                    addSquishyBall(1.5f);
                    return;

                case "test tablecloth":
                    cleanupAfterTest();
                    addBox(-1f);
                    addCylinder(1.7f);
                    addTablecloth(2f);
                    return;

                case "toggle axes":
                    toggleAxes();
                    return;

                case "toggle boxes":
                    toggleBoxes();
                    return;

                case "toggle help":
                    toggleHelp();
                    return;

                case "toggle pause":
                    togglePause();
                    return;
            }
        }
        super.onAction(actionString, ongoing, tpf);
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
        addLighting(physicsDebugRootNode, true);
    }
    // *************************************************************************
    // private methods

    /**
     * Add a visualizer for the axes of the world coordinate system.
     */
    private void addAxes() {
        float axisLength = 0.4f;
        AxesVisualizer axes = new AxesVisualizer(assetManager, axisLength);
        axes.setLineWidth(0f);

        rootNode.addControl(axes);
        axes.setEnabled(true);
    }

    /**
     * Add a large static box to the scene, to serve as a platform.
     *
     * @param topY the Y coordinate of the top surface (in physics-space
     * coordinates)
     */
    private void addBox(float topY) {
        float halfExtent = 4f;
        BoxCollisionShape shape = new BoxCollisionShape(halfExtent);
        float boxMass = PhysicsRigidBody.massForStatic;
        PhysicsRigidBody boxBody = new PhysicsRigidBody(shape, boxMass);

        boxBody.setDebugMaterial(greenMaterial);
        boxBody.setDebugMeshNormals(DebugMeshNormals.Facet);
        boxBody.setPhysicsLocation(new Vector3f(0f, topY - halfExtent, 0f));
        physicsSpace.add(boxBody);
    }

    /**
     * Add a static cylinder to the scene, to serve as an obstacle.
     *
     * @param topY the Y coordinate of the top surface (in physics-space
     * coordinates)
     */
    private void addCylinder(float topY) {
        float halfHeight = 0.2f;
        float radius = 1f;
        Vector3f halfExtents = new Vector3f(radius, halfHeight, radius);
        CollisionShape shape
                = new CylinderCollisionShape(halfExtents, PhysicsSpace.AXIS_Y);
        float cylMass = PhysicsRigidBody.massForStatic;
        PhysicsRigidBody cylinderBody = new PhysicsRigidBody(shape, cylMass);

        Vector3f translation = new Vector3f(0f, topY, 0f);
        cylinderBody.setPhysicsLocation(translation);

        physicsSpace.add(cylinderBody);
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

        ColorRGBA directColor = new ColorRGBA(0.7f, 0.7f, 0.7f, 1f);
        Vector3f direction = new Vector3f(1f, -2f, -2f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction, directColor);
        rootSpatial.addLight(sun);

        rootSpatial.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);
        if (shadowFlag) {
            DirectionalLightShadowRenderer dlsr
                    = new DirectionalLightShadowRenderer(assetManager, 2048, 3);
            dlsr.setLight(sun);
            dlsr.setShadowIntensity(0.5f);
            viewPort.addProcessor(dlsr);
        }
    }

    /**
     * Add a static pole with a rectangular flag.
     */
    private void addPoleAndFlag() {
        float halfHeight = 2f;
        float radius = 0.06f;
        Vector3f halfExtents = new Vector3f(radius, halfHeight, radius);
        CollisionShape shape
                = new CylinderCollisionShape(halfExtents, PhysicsSpace.AXIS_Y);
        float poleMass = PhysicsRigidBody.massForStatic;
        PhysicsRigidBody polePrb = new PhysicsRigidBody(shape, poleMass);

        ColorRGBA color = new ColorRGBA(0.7f, 0.7f, 1f, 1f);
        Material material = MyAsset.createShadedMaterial(assetManager, color);
        polePrb.setDebugMaterial(material);
        polePrb.setDebugMeshNormals(DebugMeshNormals.Smooth);

        physicsSpace.add(polePrb);

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
        config.set(Sbcp.Lift, 1);
        config.setAerodynamics(Aero.F_TwoSidedLiftDrag);
        config.setPositionIterations(3);

        PhysicsSoftBody.Material softMaterial = flagPsb.getSoftMaterial();
        softMaterial.setAngularStiffness(0f);

        flagPsb.setDebugMaterial(logoMaterial);
        flagPsb.setDebugMeshInitListener(flagDmiListener);
        flagPsb.setDebugMeshNormals(DebugMeshNormals.Smooth);

        Quaternion rotation = new Quaternion();
        rotation.fromAngles(FastMath.HALF_PI, 0f, 0f);
        flagPsb.applyRotation(rotation);
        flagPsb.setPhysicsLocation(new Vector3f(1f, 1.5f, 0f));

        physicsSpace.add(flagPsb);
        /*
         * Add 2 anchors that join the flag to the pole.
         */
        boolean allowCollisions = true;
        int nodeIndex = 0; // upper left corner of flag
        Vector3f initialLocation = flagPsb.nodeLocation(nodeIndex, null);
        Anchor anchor0 = new Anchor(flagPsb, nodeIndex, polePrb,
                initialLocation, allowCollisions);
        physicsSpace.add(anchor0);

        nodeIndex = xLines - 1; // lower left corner of flag
        flagPsb.nodeLocation(nodeIndex, initialLocation);
        Anchor anchor1 = new Anchor(flagPsb, nodeIndex, polePrb,
                initialLocation, allowCollisions);
        physicsSpace.add(anchor1);
    }

    /**
     * Add a Puppet model. TODO add "go limp" action
     */
    private DynamicAnimControl addPuppet() {
        /*
         * Load the model in "T" pose.
         */
        Spatial cgModel = assetManager.loadModel("Models/Puppet/Puppet.j3o");
        rootNode.attachChild(cgModel);
        /*
         * Create the animation channel now
         * so that animation blending will work later.
         */
        SkeletonControl skeletonControl = RagUtils.findSkeletonControl(cgModel);
        Spatial controlledSpatial = skeletonControl.getSpatial();
        AnimControl animControl
                = controlledSpatial.getControl(AnimControl.class);
        animControl.createChannel(); // Channel[0] includes all bones.
        /*
         * Configure and add her physics control.
         */
        DynamicAnimControl dac = new PuppetControl();
        controlledSpatial.addControl(dac);
        dac.setPhysicsSpace(physicsSpace);
        /*
         * Don't visualize her rigid bodies.
         */
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
        int numSubdiv = 5;
        int numAnchors = 51;
        float length = 0.57f;
        Vector3f[] anchorLocs = new Vector3f[numAnchors];
        for (int zIndex = 0; zIndex < numAnchors; ++zIndex) {
            anchorLocs[zIndex] = new Vector3f();
        }
        Mesh mesh = createSkirtMesh(puppetDac, numSubdiv, length, anchorLocs);
        /*
         * Create and configure the soft body.
         */
        PhysicsSoftBody skirtPsb = new PhysicsSoftBody();
        NativeSoftBodyUtil.appendFromTriMesh(mesh, skirtPsb);
        skirtPsb.setMargin(0.1f);
        skirtPsb.setMass(0.02f);

        SoftBodyConfig config = skirtPsb.getSoftConfig();
        config.set(Sbcp.AnchorHardness, 1f);
        config.set(Sbcp.KineticHardness, 1f);
        config.setPositionIterations(6);

        PhysicsSoftBody.Material material = skirtPsb.getSoftMaterial();
        material.setAngularStiffness(0f);
        material.setLinearStiffness(0.5f);

        skirtPsb.setDebugMaterial(redMaterial);
        skirtPsb.setDebugMeshNormals(DebugMeshNormals.Smooth);

        String vSpec0 = puppetVSpec(0);
        PhysicsLink link = puppetDac.findManagerForVertex(vSpec0, null, null);
        Transform localToWorld = link.physicsTransform(null);
        skirtPsb.applyTransform(localToWorld);

        physicsSpace.add(skirtPsb);
        skirtPsb.setGravity(new Vector3f(0f, -10f, 0f));
        /*
         * Add anchors that join Puppet to her skirt.
         */
        PhysicsRigidBody rigid = link.getRigidBody();
        boolean allowCollisions = true;
        for (int anchorIndex = 0; anchorIndex < numAnchors; ++anchorIndex) {
            Vector3f location = anchorLocs[anchorIndex];
            Anchor anchor = new Anchor(skirtPsb, anchorIndex, rigid, location,
                    allowCollisions);
            physicsSpace.add(anchor);
        }
    }

    /**
     * Add a squishy ball to the scene.
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

        ballPsb.setDebugMaterial(pinkMaterial);
        ballPsb.setDebugMeshNormals(DebugMeshNormals.Smooth);

        Vector3f translation = new Vector3f(0f, startY, 0f);
        ballPsb.applyTranslation(translation);

        physicsSpace.add(ballPsb);
    }

    /**
     * Add a square tablecloth to the scene.
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

        PhysicsSoftBody.Material material = softBody.getSoftMaterial();
        material.setAngularStiffness(0f);

        softBody.setDebugMaterial(plaidMaterial);
        softBody.setDebugMeshInitListener(tableclothDmiListener);
        softBody.setDebugMeshNormals(DebugMeshNormals.Smooth);

        Vector3f translation = new Vector3f(0f, startY, 0f);
        softBody.applyTranslation(translation);

        physicsSpace.add(softBody);
    }

    /**
     * Clean up after a test.
     */
    private void cleanupAfterTest() {
        /*
         * Remove any scenery. Debug meshes are under a different root node.
         */
        rootNode.detachAllChildren();
        /*
         * Remove physics objects, which also removes their debug meshes.
         */
        Collection<PhysicsJoint> joints = physicsSpace.getJointList();
        for (PhysicsJoint joint : joints) {
            physicsSpace.remove(joint);
        }
        Collection<PhysicsCollisionObject> pcos = physicsSpace.getPcoList();
        for (PhysicsCollisionObject pco : pcos) {
            physicsSpace.remove(pco);
        }
        /*
         * Clear the hidden-object list.
         */
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

        cam.setLocation(new Vector3f(0f, 2.2f, 3.9f));
        cam.setRotation(new Quaternion(0f, 0.98525f, -0.172f, 0f));

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
     * Configure materials during startup.
     */
    private void configureMaterials() {
        ColorRGBA green = new ColorRGBA(0f, 0.12f, 0f, 1f);
        greenMaterial = MyAsset.createShadedMaterial(assetManager, green);
        greenMaterial.setName("green");

        Texture plaid
                = MyAsset.loadTexture(assetManager, "Textures/plaid.png", true);
        plaid.setAnisotropicFilter(8);
        plaid.setWrap(Texture.WrapMode.Repeat);
        plaidMaterial = MyAsset.createShadedMaterial(assetManager, plaid);
        plaidMaterial.setName("plaid");
        RenderState renderState = plaidMaterial.getAdditionalRenderState();
        renderState.setFaceCullMode(RenderState.FaceCullMode.Off);

        Texture texture = MyAsset.loadTexture(assetManager,
                "Interface/Logo/Monkey.png", true);
        logoMaterial = MyAsset.createShadedMaterial(assetManager, texture);
        logoMaterial.setName("logo");
        renderState = logoMaterial.getAdditionalRenderState();
        renderState.setFaceCullMode(RenderState.FaceCullMode.Off);

        ColorRGBA pink = new ColorRGBA(1.2f, 0.2f, 0.1f, 1f);
        pinkMaterial = MyAsset.createShinyMaterial(assetManager, pink);
        pinkMaterial.setFloat("Shininess", 4f);
        pinkMaterial.setName("pink");
        renderState = pinkMaterial.getAdditionalRenderState();
        renderState.setFaceCullMode(RenderState.FaceCullMode.Off);

        ColorRGBA red = new ColorRGBA(0.7f, 0.01f, 0.01f, 1f);
        redMaterial = MyAsset.createShadedMaterial(assetManager, red);
        redMaterial.setColor("Specular", new ColorRGBA(0.05f, 0.05f, 0.05f, 1f));
        redMaterial.setFloat("Shininess", 4f);
        redMaterial.setName("red");
        renderState = redMaterial.getAdditionalRenderState();
        renderState.setFaceCullMode(RenderState.FaceCullMode.Off);
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        CollisionShape.setDefaultMargin(0.005f); // 5-mm margin

        bulletAppState.setDebugEnabled(true);
        bulletAppState.setDebugFilter(hiddenObjects);
        bulletAppState.setDebugInitListener(this);
        stateManager.attach(bulletAppState);

        physicsSpace = bulletAppState.getPhysicsSoftSpace();
        physicsSpace.setAccuracy(0.01f); // 10-msec timestep
        physicsSpace.setGravity(new Vector3f(0f, -1f, 0f));
    }

    /**
     * Generate a Mesh for Puppet's skirt.
     *
     * @param puppetDac the model's physics control (not null)
     * @param numSubDiv the number of mesh squares between successive anchors
     * (&ge;1)
     * @param skirtLength the desired length (in physics-space units, &get;0)
     * @param local storage for waist locations in local coordinates (not null,
     * modified)
     * @return a new Mesh, fitted to the model in her bone's local coordinates
     */
    private Mesh createSkirtMesh(DynamicAnimControl puppetDac, int numSubdiv,
            float skirtLength, Vector3f[] local) {
        int numWaistVerts = waistlineVertices.length;
        int numXLines = local.length;

        for (int zIndex = 0; zIndex < numXLines; ++zIndex) {
            local[zIndex].zero();
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
        Vector3f apex = aabb.getCenter();
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
        /*
         * Generate the mesh topology.
         */
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
     * Cycle through animations of the Puppet model.
     */
    private void nextPuppetAnimation() {
        SkeletonControl skeletonControl
                = RagUtils.findSkeletonControl(rootNode);
        if (skeletonControl == null) {
            return;
        }

        Spatial controlledSpatial = skeletonControl.getSpatial();
        AnimControl animControl
                = controlledSpatial.getControl(AnimControl.class);
        AnimChannel channel = animControl.getChannel(0);

        String animationName = channel.getAnimationName();
        if (animationName == null) { // first time
            channel.setAnim("jog");
        } else if (animationName.equals("jog")) {
            float blendTime = 1f; // seconds
            channel.setAnim("walk", blendTime);
        } else if (animationName.equals("walk")) {
            float blendTime = 1f; // seconds
            channel.setAnim("jog", blendTime);
        }
    }

    /**
     * Cycle through wind velocities.
     */
    private void nextWindVelocity() {
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
     */
    private static String puppetVSpec(int waistIndex) {
        int puppetVertex = waistlineVertices[waistIndex];
        String vertexSpecifier = String.format("%d/Mesh.009_0", puppetVertex);
        return vertexSpecifier;
    }

    /**
     * Toggle visualization of collision-object axes.
     */
    private void toggleAxes() {
        float length = bulletAppState.debugAxisLength();
        bulletAppState.setDebugAxisLength(0.5f - length);
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
     * Toggle the animation and physics simulation: paused/running.
     */
    private void togglePause() {
        float newSpeed = (speed > 1e-12f) ? 1e-12f : 1f;
        setSpeed(newSpeed);
    }
}
