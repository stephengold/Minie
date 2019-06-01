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
import com.jme3.bullet.PhysicsSoftSpace;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.SoftPhysicsAppState;
import com.jme3.bullet.animation.DynamicAnimControl;
import com.jme3.bullet.animation.PhysicsLink;
import com.jme3.bullet.animation.RagUtils;
import com.jme3.bullet.animation.TorsoLink;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.infos.DebugMeshNormals;
import com.jme3.bullet.debug.BulletDebugAppState;
import com.jme3.bullet.debug.DebugInitListener;
import com.jme3.bullet.debug.DebugMeshInitListener;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.bullet.objects.infos.Aero;
import com.jme3.bullet.objects.infos.Sbcp;
import com.jme3.bullet.objects.infos.SoftBodyConfig;
import com.jme3.bullet.util.NativeSoftBodyUtil;
import com.jme3.export.Savable;
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
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer;
import com.jme3.system.AppSettings;
import com.jme3.texture.Texture;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Misc;
import jme3utilities.MyAsset;
import jme3utilities.MyCamera;
import jme3utilities.debug.AxesVisualizer;
import jme3utilities.math.MyVector3f;
import jme3utilities.minie.DumpFlags;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.minie.test.mesh.ClothGrid;
import jme3utilities.minie.test.mesh.Icosphere;
import jme3utilities.minie.test.tunings.PuppetControl;
import jme3utilities.ui.ActionApplication;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.InputMode;

/**
 * Test/demonstrate soft-body physics.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestSoftBody
        extends ActionApplication
        implements BulletDebugAppState.DebugAppStateFilter, DebugInitListener {
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
     * as seen from above, starting and ending at her right hip - TODO overlap?
     */
    final private static int[] waistlineVertices = new int[]{2311, 2312, 2345,
        499, 498, 502, 464, 509, 505, 2351, 2354, 2319, 2311};
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
     * physics objects that are not to be visualized
     */
    final private List<Savable> hiddenObjects = new ArrayList<>(9);
    /**
     * double-sided blue material to visualize clothing
     */
    private Material blueMaterial;
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
        configureMaterials();
        configurePhysics();
        ColorRGBA gray = new ColorRGBA(0.1f, 0.1f, 0.1f, 1f);
        viewPort.setBackgroundColor(gray);

        addBox(0f);
        addSquishyBall(1.5f);
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
        dim.bind("toggle pause", KeyInput.KEY_PERIOD);
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
                    return;

                case "test poleAndFlag":
                    cleanupAfterTest();
                    addBox(-2f);
                    addPoleAndFlag();
                    return;

                case "test puppetInSkirt":
                    cleanupAfterTest();
                    addAxes();
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

                case "toggle pause":
                    togglePause();
                    return;
            }
        }
        super.onAction(actionString, ongoing, tpf);
    }
    // *************************************************************************
    // BulletDebugAppState.DebugAppStateFilter methods

    /**
     * Test whether the specified physics object should be displayed in the
     * debug scene.
     *
     * @param physicsObject the joint or collision object to test (unaffected)
     * @return return true if the object should be displayed, false if not
     */
    @Override
    public boolean displayObject(Savable physicsObject) {
        return !hiddenObjects.contains(physicsObject);
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
     */
    private void addBox(float topY) {
        float halfExtent = 4f;
        BoxCollisionShape shape = new BoxCollisionShape(halfExtent);
        float boxMass = PhysicsRigidBody.massForStatic;
        PhysicsRigidBody boxBody = new PhysicsRigidBody(shape, boxMass);

        Material material = MyAsset.createDebugMaterial(assetManager);
        boxBody.setDebugMaterial(material);
        boxBody.setDebugMeshNormals(DebugMeshNormals.Facet);
        boxBody.setPhysicsLocation(new Vector3f(0f, topY - halfExtent, 0f));
        physicsSpace.add(boxBody);
    }

    /**
     * Add a static cylinder to the scene, to serve as an obstacle.
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
        hiddenObjects.add(cylinderBody);
    }

    /**
     * Add lighting to the specified scene.
     */
    private void addLighting(Spatial rootSpatial) {
        ColorRGBA ambientColor = new ColorRGBA(0.1f, 0.1f, 0.1f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        rootSpatial.addLight(ambient);

        ColorRGBA directColor = new ColorRGBA(0.7f, 0.7f, 0.7f, 1f);
        Vector3f direction = new Vector3f(1f, -2f, -1f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction, directColor);
        rootSpatial.addLight(sun);
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
        flagPsb.setMass(mass);

        Vector3f wind = new Vector3f(2.5f, 0f, -0.5f);
        flagPsb.setWindVelocity(wind); // TODO vary the wind velocity

        SoftBodyConfig config = flagPsb.getSoftConfig();
        config.set(Sbcp.Damping, 0.05f);
        config.set(Sbcp.Drag, 0.5f);
        config.set(Sbcp.Lift, 0.5f);
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
         * Append 2 anchors connecting the flag to the pole.
         */
        boolean collideFlag = true;
        float influence = 1f;
        int nodeIndex = 0; // upper left corner of flag
        Vector3f localPivot = flagPsb.nodeLocation(nodeIndex, null);
        flagPsb.appendAnchor(nodeIndex, polePrb, localPivot, collideFlag,
                influence);
        nodeIndex = xLines - 1; // lower left corner of flag
        flagPsb.nodeLocation(nodeIndex, localPivot);
        flagPsb.appendAnchor(nodeIndex, polePrb, localPivot, collideFlag,
                influence);
    }

    /**
     * Add a Puppet model.
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
         * Configure and add the physics control.
         */
        DynamicAnimControl dac = new PuppetControl();
        controlledSpatial.addControl(dac);
        dac.setPhysicsSpace(physicsSpace);

        PhysicsRigidBody[] rigids = dac.listRigidBodies();
        for (PhysicsRigidBody rigid : rigids) {
            hiddenObjects.add(rigid);
        }

        float thighMargin = 0.12f;
        PhysicsLink leftThigh = dac.findBoneLink("thigh.L");
        leftThigh.getRigidBody().getCollisionShape().setMargin(thighMargin);
        PhysicsLink rightThigh = dac.findBoneLink("thigh.R");
        rightThigh.getRigidBody().getCollisionShape().setMargin(thighMargin);

        return dac;
    }

    /**
     * Anchor a skirt to the specified Puppet model.
     *
     * @param puppetDac the model's physics control (not null)
     */
    private void addSkirt(DynamicAnimControl puppetDac) {
        int numSubdiv = 8;
        int numAnchors = waistlineVertices.length;
        Vector3f[] anchorLocal = new Vector3f[numAnchors];
        Mesh mesh = createSkirtMesh(puppetDac, numSubdiv, anchorLocal);
        /*
         * Create and configure the soft body.
         */
        PhysicsSoftBody skirtPsb = new PhysicsSoftBody();
        NativeSoftBodyUtil.appendFromTriMesh(mesh, skirtPsb);
        skirtPsb.setMass(0.01f);

        SoftBodyConfig config = skirtPsb.getSoftConfig();
        config.set(Sbcp.AnchorHardness, 1f);
        config.set(Sbcp.KineticHardness, 1f);
        config.setPositionIterations(6);

        PhysicsSoftBody.Material material = skirtPsb.getSoftMaterial();
        material.setAngularStiffness(0f);

        skirtPsb.setDebugMaterial(blueMaterial);
        skirtPsb.setDebugMeshNormals(DebugMeshNormals.Smooth);

        TorsoLink torsoLink = puppetDac.getTorsoLink();
        Transform localToWorld = torsoLink.physicsTransform(null);
        skirtPsb.applyTransform(localToWorld);

        physicsSpace.add(skirtPsb);
        skirtPsb.setGravity(new Vector3f(0f, -10f, 0f));
        /*
         * Append anchors connecting Puppet to her skirt.
         */
        boolean collide = true;
        float influence = 1f;
        PhysicsRigidBody rigid = torsoLink.getRigidBody();
        for (int anchorIndex = 0; anchorIndex < numAnchors; ++anchorIndex) {
            int nodeIndex = numSubdiv * anchorIndex;
            Vector3f inner = anchorLocal[anchorIndex];
            skirtPsb.appendAnchor(nodeIndex, rigid, inner, collide, influence);
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

        SoftBodyConfig clothPsb = softBody.getSoftConfig();
        clothPsb.set(Sbcp.Damping, 0.02f);
        clothPsb.setPositionIterations(3);

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
        hiddenObjects.clear();
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        float yDegrees = MyCamera.yDegrees(cam);
        float aspectRatio = MyCamera.viewAspectRatio(cam);
        float near = 0.02f;
        float far = 20f;
        cam.setFrustumPerspective(yDegrees, aspectRatio, near, far);

        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(4f);
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
        ColorRGBA blue = new ColorRGBA(0.5f, 0.5f, 1f, 1f);
        blueMaterial = MyAsset.createShadedMaterial(assetManager, blue);
        blueMaterial.setName("blue");
        RenderState renderState = blueMaterial.getAdditionalRenderState();
        renderState.setFaceCullMode(RenderState.FaceCullMode.Off);

        Texture plaid
                = MyAsset.loadTexture(assetManager, "Textures/plaid.png", true);
        plaid.setAnisotropicFilter(8);
        plaid.setWrap(Texture.WrapMode.Repeat);
        plaidMaterial = MyAsset.createShadedMaterial(assetManager, plaid);
        plaidMaterial.setName("plaid");
        renderState = plaidMaterial.getAdditionalRenderState();
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
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        CollisionShape.setDefaultMargin(0.005f);

        SoftPhysicsAppState bulletAppState = new SoftPhysicsAppState();
        bulletAppState.setDebugEnabled(true);
        bulletAppState.setDebugFilter(this);
        bulletAppState.setDebugInitListener(this);
        stateManager.attach(bulletAppState);

        physicsSpace = bulletAppState.getPhysicsSoftSpace();
        physicsSpace.setAccuracy(0.01f);
        physicsSpace.setGravity(new Vector3f(0f, -1f, 0f));
    }

    /**
     * Generate a Mesh for Puppet's skirt.
     *
     * @param puppetDac the model's physics control (not null)
     * @param numSubDiv the number of mesh squares between successive anchors
     * (&ge;1)
     * @param anchorLocal storage for anchor locations (not null,
     * length=numAnchors)
     * @return a new Mesh
     */
    private Mesh createSkirtMesh(DynamicAnimControl puppetDac, int numSubdiv,
            Vector3f[] anchorLocal) {
        int numAnchors = waistlineVertices.length;
        assert anchorLocal.length == numAnchors : numAnchors;

        int numXLines = numAnchors * numSubdiv - numSubdiv + 1;
        Vector3f[] nodeLocal = new Vector3f[numXLines];
        for (int zIndex = 0; zIndex < numXLines; ++zIndex) {
            nodeLocal[zIndex] = new Vector3f();
        }
        /*
         * Calculate anchor locations (in the local coordinates of the
         * rigid body) along Puppet's waistline. Later, these will be
         * anchored to the xIndex=0 edge of the ClothGrid.
         */
        for (int anchorIndex = 0; anchorIndex < numAnchors; ++anchorIndex) {
            int zIndex = numSubdiv * anchorIndex;
            Vector3f local = nodeLocal[zIndex]; // alias
            anchorLocal[anchorIndex] = local; // alias
            String vSpec = puppetVSpec(anchorIndex);
            PhysicsLink link = puppetDac.findManagerForVertex(vSpec, null, local);
            assert link == puppetDac.getTorsoLink();
        }
        /*
         * Expand the waistline outward (from the origin of the rigid body)
         * to provide a margin.  Also, compute its maxRadius.
         */
        float margin = 0.001f;
        float maxRadius = 0f;
        for (int anchorIndex = 0; anchorIndex < numAnchors; ++anchorIndex) {
            Vector3f local = anchorLocal[anchorIndex]; // alias
            float radius = local.length();
            float expandedRadius = radius + margin;
            local.multLocal(expandedRadius / radius);

            maxRadius = Math.max(expandedRadius, maxRadius);
        }
        /*
         * Subdivide the waistline, interpolating numSubdiv-1 nodes
         * between each pair of successive anchors.
         * Also, calculate its circumference.
         */
        float circumference = 0f;
        for (int anchorIndex = 1; anchorIndex < numAnchors; ++anchorIndex) {
            int zIndex = numSubdiv * anchorIndex;
            Vector3f local = nodeLocal[zIndex]; // alias
            int prevAnchorZIndex = numSubdiv * (anchorIndex - 1);
            Vector3f prevAnchorLocal = nodeLocal[prevAnchorZIndex]; // alias
            float anchorSpacing = local.distance(prevAnchorLocal);
            circumference += anchorSpacing;

            for (int zi = prevAnchorZIndex + 1; zi < zIndex; ++zi) {
                float t = ((float) (zi - prevAnchorZIndex)) / numSubdiv;
                MyVector3f.lerp(t, prevAnchorLocal, local, nodeLocal[zi]);
            }
        }
        /*
         * Generate the mesh topology.
         */
        float averageSpacing = circumference / (numAnchors - 1);
        float skirtLength = 0.8f;
        float outerRadius = maxRadius + skirtLength;
        int numZLines = Math.round(skirtLength / averageSpacing);
        ClothGrid mesh = new ClothGrid(numXLines, numZLines, averageSpacing);
        /*
         * Scale the waistline outward from a chosen point.
         * Use those locations to deform the mesh into a roughly conical shape.
         */
        Vector3f scaleCenter = new Vector3f(0f, 0f, -0.3f);
        Vector3f offset = new Vector3f();
        Vector3f outer = new Vector3f();
        Vector3f tmpLocation = new Vector3f();
        for (int zIndex = 0; zIndex < numXLines; ++zIndex) {
            Vector3f inner = nodeLocal[zIndex]; // alias
            inner.subtract(scaleCenter, offset);
            float innerRadius = offset.length();
            offset.multLocal(outerRadius / innerRadius);
            offset.add(scaleCenter, outer);

            for (int xIndex = 0; xIndex < numZLines; ++xIndex) {
                float t = ((float) xIndex) / (numZLines - 1);
                MyVector3f.lerp(t, inner, outer, tmpLocation);
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
            animationName = "idle";
        } else {
            switch (animationName) {
                case "walk":
                    animationName = "idle";
                    break;
                case "idle":
                    animationName = "walk";
                    break;
            }
        }

        float blendTime = 1f; // seconds
        channel.setAnim(animationName, blendTime);
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
     * Toggle the animation and physics simulation: paused/running.
     */
    private void togglePause() {
        float newSpeed = (speed > 1e-12f) ? 1e-12f : 1f;
        setSpeed(newSpeed);
    }
}
