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

import com.jme3.app.Application;
import com.jme3.bullet.PhysicsSoftSpace;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.SoftPhysicsAppState;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.infos.DebugMeshNormals;
import com.jme3.bullet.debug.BulletDebugAppState;
import com.jme3.bullet.debug.DebugInitListener;
import com.jme3.bullet.debug.DebugMeshInitListener;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.bullet.objects.infos.Aero;
import com.jme3.bullet.objects.infos.Sbcp;
import com.jme3.bullet.objects.infos.SoftBodyConfig;
import com.jme3.bullet.util.NativeSoftBodyUtil;
import com.jme3.export.Savable;
import com.jme3.input.KeyInput;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
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
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Misc;
import jme3utilities.MyAsset;
import jme3utilities.minie.DumpFlags;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.minie.test.mesh.ClothGrid;
import jme3utilities.minie.test.mesh.Icosphere;
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
                uvs.put(x - 0.5f).put(2f - y);
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
                uvs.put(12f * x).put(12f * z);
            }
            uvs.flip();
        }
    };
    /**
     * mass of each soft body (&gt;0)
     */
    final private static float mass = 1f;
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
     * physics objects not visualized
     */
    final private List<Savable> hiddenObjects = new ArrayList<>(9);
    /**
     * logo material to visualize flags
     */
    private Material logoMaterial;
    /**
     * pink material to visualize soft bodies
     */
    private Material pinkMaterial;
    /**
     * plaid material to visualize tablecloths
     */
    private Material plaidMaterial;
    /**
     * dump debugging information to the console
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
        addSquishyBall();
    }

    /**
     * Add application-specific hotkey bindings and override existing ones.
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        dim.bind("dump physicsSpace", KeyInput.KEY_O);
        dim.bind("dump scenes", KeyInput.KEY_P);
        dim.bind("signal orbitLeft", KeyInput.KEY_LEFT);
        dim.bind("signal orbitRight", KeyInput.KEY_RIGHT);
        dim.bind("test poleAndFlag", KeyInput.KEY_F3);
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

                case "test poleAndFlag":
                    cleanupAfterTest();
                    addBox(-2f);
                    addPoleAndFlag();
                    return;

                case "test squishyBall":
                    cleanupAfterTest();
                    addBox(0f);
                    addSquishyBall();
                    return;

                case "test tablecloth":
                    cleanupAfterTest();
                    addBox(-2f);
                    addCylinder();
                    addTablecloth();
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
    private void addCylinder() {
        float halfHeight = 0.2f;
        float radius = 1f;
        Vector3f halfExtents = new Vector3f(radius, halfHeight, radius);
        CollisionShape shape
                = new CylinderCollisionShape(halfExtents, PhysicsSpace.AXIS_Y);
        float cylMass = PhysicsRigidBody.massForStatic;
        PhysicsRigidBody cylinderBody = new PhysicsRigidBody(shape, cylMass);

        cylinderBody.setPhysicsLocation(new Vector3f(0f, 0.7f, 0f));
        physicsSpace.add(cylinderBody);
        hiddenObjects.add(cylinderBody);
    }

    /**
     * Add lighting and shadows to the specified scene.
     */
    private void addLighting(Spatial rootSpatial) {
        ColorRGBA ambientColor = new ColorRGBA(0.1f, 0.1f, 0.1f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        rootSpatial.addLight(ambient);

        ColorRGBA directColor = new ColorRGBA(0.7f, 0.7f, 0.7f, 1f);
        Vector3f direction = new Vector3f(1f, -2f, -1f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction, directColor);
        rootSpatial.addLight(sun);

        rootSpatial.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);

        DirectionalLightShadowRenderer dlsr
                = new DirectionalLightShadowRenderer(assetManager, 2_048, 3);
        dlsr.setLight(sun);
        dlsr.setShadowIntensity(0.6f);
        viewPort.addProcessor(dlsr);
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
        PhysicsRigidBody poleBody = new PhysicsRigidBody(shape, poleMass);

        ColorRGBA color = new ColorRGBA(0.7f, 0.7f, 1f, 1f);
        Material material = MyAsset.createShadedMaterial(assetManager, color);
        poleBody.setDebugMaterial(material);
        poleBody.setDebugMeshNormals(DebugMeshNormals.Smooth);
        physicsSpace.add(poleBody);

        int xLines = 12;
        int zLines = 24;
        float separation = 0.08f;
        Mesh mesh = new ClothGrid(xLines, zLines, separation);
        PhysicsSoftBody flagBody = new PhysicsSoftBody();

        NativeSoftBodyUtil.appendFromTriMesh(mesh, flagBody);
        flagBody.setMass(mass);
        Vector3f wind = new Vector3f(2.5f, 0f, -0.5f);
        flagBody.setWindVelocity(wind);
        SoftBodyConfig config = flagBody.getSoftConfig();
        config.set(Sbcp.Damping, 0.05f);
        config.set(Sbcp.Drag, 0.5f);
        config.set(Sbcp.Lift, 0.5f);
        config.setAerodynamics(Aero.F_TwoSidedLiftDrag);
        config.setPositionIterations(6);
        PhysicsSoftBody.Material softMaterial = flagBody.getSoftMaterial();
        softMaterial.setAngularStiffness(0f);

        flagBody.setDebugMaterial(logoMaterial);
        flagBody.setDebugMeshInitListener(flagDmiListener);
        flagBody.setDebugMeshNormals(DebugMeshNormals.Smooth);

        Quaternion rotation = new Quaternion();
        rotation.fromAngles(FastMath.HALF_PI, 0f, 0f);
        flagBody.applyRotation(rotation);
        flagBody.setPhysicsLocation(new Vector3f(1f, 1.5f, 0f));
        physicsSpace.add(flagBody);
        /*
         * Add 2 anchors connecting the flag to the pole.
         */
        boolean collideFlag = true;
        float influence = 1f; // Bullet issue #2269 when set to 4
        int nodeIndex = 0; // upper left corner of flag
        Vector3f localPivot = flagBody.nodeLocation(nodeIndex, null);
        flagBody.appendAnchor(nodeIndex, poleBody, localPivot, collideFlag,
                influence);
        nodeIndex = xLines - 1; // lower left corner of flag
        flagBody.nodeLocation(nodeIndex, localPivot);
        flagBody.appendAnchor(nodeIndex, poleBody, localPivot, collideFlag,
                influence);
    }

    /**
     * Add a squishy ball to the scene.
     */
    private void addSquishyBall() {
        int numRefinementIterations = 3;
        float radius = 0.5f;
        Mesh mesh = new Icosphere(numRefinementIterations, radius);
        PhysicsSoftBody softBody = new PhysicsSoftBody();
        NativeSoftBodyUtil.appendFromTriMesh(mesh, softBody);
        softBody.setMass(mass);

        SoftBodyConfig config = softBody.getSoftConfig();
        config.set(Sbcp.PoseMatching, 0.02f);

        boolean setVolumePose = false;
        boolean setFramePose = true;
        softBody.setPose(setVolumePose, setFramePose);

        softBody.setDebugMaterial(pinkMaterial);
        softBody.setDebugMeshNormals(DebugMeshNormals.Smooth);
        softBody.setPhysicsLocation(new Vector3f(0f, 1.5f, 0f));
        physicsSpace.add(softBody);
    }

    /**
     * Add a square tablecloth to the scene.
     */
    private void addTablecloth() {
        int numLines = 40;
        float separation = 0.08f;
        Mesh mesh = new ClothGrid(numLines, numLines, separation);
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
        softBody.setPhysicsLocation(new Vector3f(0f, 1.5f, 0f));
        physicsSpace.add(softBody);
    }

    /**
     * Clean up after a test.
     */
    private void cleanupAfterTest() {
        Collection<PhysicsCollisionObject> pcos = physicsSpace.getPcoList();
        for (PhysicsCollisionObject pco : pcos) {
            physicsSpace.remove(pco);
        }

        hiddenObjects.clear();
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(4f);
        cam.setLocation(new Vector3f(0f, 2.3f, 5f));
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
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        SoftPhysicsAppState bulletAppState = new SoftPhysicsAppState();
        bulletAppState.setDebugEnabled(true);
        bulletAppState.setDebugFilter(this);
        bulletAppState.setDebugInitListener(this);
        stateManager.attach(bulletAppState);

        physicsSpace = bulletAppState.getPhysicsSoftSpace();
        physicsSpace.setGravity(new Vector3f(0f, -1f, 0f));
    }

    /**
     * Toggle the animation and physics simulation: paused/running.
     */
    private void togglePause() {
        float newSpeed = (speed > 1e-12f) ? 1e-12f : 1f;
        setSpeed(newSpeed);
    }
}
