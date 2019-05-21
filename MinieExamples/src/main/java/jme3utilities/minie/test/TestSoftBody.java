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
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.debug.DebugInitListener;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.bullet.objects.infos.Sbcp;
import com.jme3.bullet.util.NativeSoftBodyUtil;
import com.jme3.input.KeyInput;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Box;
import com.jme3.system.AppSettings;
import java.util.ArrayList;
import java.util.Collection;
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
 * Test soft-body physics.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestSoftBody
        extends ActionApplication
        implements DebugInitListener {
    // *************************************************************************
    // constants and loggers

    /**
     * magnitude of the gravitational acceleration (&ge;0)
     */
    final private float gravity = 1f;
    /**
     * mass of the soft body (&gt;0)
     */
    final private float mass = 1f;
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
     * material to visualize soft bodies
     */
    private Material debugMaterial;
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
        configurePhysics();
        viewPort.setBackgroundColor(ColorRGBA.Gray);

        ColorRGBA pink = new ColorRGBA(1.2f, 0.2f, 0.1f, 1f);
        debugMaterial = MyAsset.createShinyMaterial(assetManager, pink);
        debugMaterial.setFloat("Shininess", 4f);
        debugMaterial.setName("pink");

        addBox();
        addFatBall();
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
        dim.bind("test fatball", KeyInput.KEY_F1);
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
                    dumpPhysicsSpace();
                    return;

                case "dump scenes":
                    dumpScenes();
                    return;

                case "test fatball":
                    cleanupAfterTest();
                    addBox();
                    addFatBall();
                    return;

                case "test tablecloth":
                    cleanupAfterTest();
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
    // DebugInitListener methods

    /**
     * Callback from BulletDebugAppState, invoked just before the debug scene is
     * added to the debug viewports.
     *
     * @param physicsDebugRootNode the root node of the debug scene (not null)
     */
    @Override
    public void bulletDebugInit(Node physicsDebugRootNode) {
        ColorRGBA ambientColor = new ColorRGBA(0.03f, 0.03f, 0.03f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        physicsDebugRootNode.addLight(ambient);

        ColorRGBA directColor = new ColorRGBA(0.7f, 0.7f, 0.7f, 1f);
        Vector3f direction = new Vector3f(1f, -2f, -1f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction, directColor);
        physicsDebugRootNode.addLight(sun);
    }
    // *************************************************************************
    // private methods

    /**
     * Add a large static box to the scene, to serve as a platform.
     */
    private void addBox() {
        float halfExtent = 50f; // mesh units
        Mesh mesh = new Box(halfExtent, halfExtent, halfExtent);
        Geometry boxGeometry = new Geometry("box", mesh);
        rootNode.attachChild(boxGeometry);

        boxGeometry.move(0f, -halfExtent, 0f);
        Material material = MyAsset.createDebugMaterial(assetManager);
        boxGeometry.setMaterial(material);

        BoxCollisionShape shape = new BoxCollisionShape(halfExtent);
        float boxMass = PhysicsRigidBody.massForStatic;
        RigidBodyControl boxBody = new RigidBodyControl(shape, boxMass);
        boxGeometry.addControl(boxBody);
        boxBody.setApplyScale(true);
        boxBody.setPhysicsSpace(physicsSpace);
    }

    /**
     * Add a static cylinder to the scene, to serve as an obstacle.
     */
    private void addCylinder() {
        float radius = 1f;
        Vector3f halfExtents = new Vector3f(radius, 0.2f, radius);
        CollisionShape shape
                = new CylinderCollisionShape(halfExtents, PhysicsSpace.AXIS_Y);
        float cylMass = PhysicsRigidBody.massForStatic;
        PhysicsRigidBody cylinderBody = new PhysicsRigidBody(shape, cylMass);
        cylinderBody.setPhysicsLocation(new Vector3f(0f, 0.7f, 0f));
        physicsSpace.add(cylinderBody);
    }

    /**
     * Add a squishy ball to the scene.
     */
    private void addFatBall() {
        int numSteps = 3;
        float radius = 0.5f;
        Mesh mesh = new Icosphere(numSteps, radius);
        PhysicsSoftBody softBody = new PhysicsSoftBody();
        NativeSoftBodyUtil.appendFromTriMesh(mesh, softBody);
        softBody.setMass(mass);

        PhysicsSoftBody.Config config = softBody.getSoftConfig();
        config.set(Sbcp.PoseMatching, 0.02f);

        boolean setVolumePose = false;
        boolean setFramePose = true;
        softBody.setPose(setVolumePose, setFramePose);

        softBody.setDebugMaterial(debugMaterial);
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

        PhysicsSoftBody.Config config = softBody.getSoftConfig();
        config.set(Sbcp.Damping, 0.02f);
        config.setPositionIterations(3);

        PhysicsSoftBody.Material material = softBody.getSoftMaterial();
        material.setAngularStiffness(0f);

        softBody.setDebugMaterial(debugMaterial);
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

        Collection<Spatial> spatials = rootNode.getChildren(); // alias
        Collection<Spatial> copyList = new ArrayList<>(spatials);
        for (Spatial spatial : copyList) {
            spatial.removeFromParent();
        }
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
     * Configure physics during startup.
     */
    private void configurePhysics() {
        SoftPhysicsAppState bulletAppState = new SoftPhysicsAppState();
        bulletAppState.setDebugEnabled(true);
        bulletAppState.setDebugInitListener(this);
        stateManager.attach(bulletAppState);

        physicsSpace = bulletAppState.getPhysicsSoftSpace();
        physicsSpace.setGravity(new Vector3f(0f, -gravity, 0f));
    }

    /**
     * Process a "dump physicsSpace" action.
     */
    private void dumpPhysicsSpace() {
        PhysicsDumper dumper = new PhysicsDumper();
        dumper.setEnabled(DumpFlags.JointsInBodies, true);
        dumper.setEnabled(DumpFlags.JointsInSpaces, true);
        dumper.dump(physicsSpace);
    }

    /**
     * Process a "dump scenes" action.
     */
    private void dumpScenes() {
        PhysicsDumper dumper = new PhysicsDumper();
        dumper.setEnabled(DumpFlags.Transforms, true);
        dumper.setEnabled(DumpFlags.MatParams, true);
        dumper.dump(renderManager);
    }

    /**
     * Toggle the animation and physics simulation: paused/running.
     */
    private void togglePause() {
        float newSpeed = (speed > 1e-12f) ? 1e-12f : 1f;
        setSpeed(newSpeed);
    }
}
