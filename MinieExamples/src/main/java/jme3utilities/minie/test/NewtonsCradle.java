/*
 Copyright (c) 2020, Stephen Gold
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
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.collision.shapes.infos.DebugMeshNormals;
import com.jme3.bullet.debug.DebugInitListener;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.joints.Point2PointJoint;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.font.BitmapText;
import com.jme3.font.Rectangle;
import com.jme3.input.CameraInput;
import com.jme3.input.KeyInput;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.system.AppSettings;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MyAsset;
import jme3utilities.math.MyVector3f;
import jme3utilities.minie.DumpFlags;
import jme3utilities.minie.FilterAll;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.ui.ActionApplication;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.HelpUtils;
import jme3utilities.ui.InputMode;

/**
 * An ActionApplication to simulate Newton's cradle.
 * <p>
 * Collision objects are rendered entirely by debug visualization.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class NewtonsCradle
        extends ActionApplication
        implements DebugInitListener {
    // *************************************************************************
    // constants and loggers

    /**
     * speed when paused
     */
    final private static float pausedSpeed = 1e-12f;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(NewtonsCradle.class.getName());
    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName
            = NewtonsCradle.class.getSimpleName();
    // *************************************************************************
    // fields

    /**
     * status displayed in the upper-left corner of the GUI node
     */
    private BitmapText statusText;
    /**
     * AppState to manage the PhysicsSpace
     */
    final private BulletAppState bulletAppState = new BulletAppState();
    /**
     * filter to control visualization of axis-aligned bounding boxes
     */
    private FilterAll bbFilter;
    /**
     * Material for the balls
     */
    private Material ballMaterial;
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
     * temporary storage used in updateStatusText()
     */
    final private static Vector3f tmpVector = new Vector3f();
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the NewtonsCradle application.
     *
     * @param ignored array of command-line arguments (not null)
     */
    public static void main(String[] ignored) {
        /*
         * Mute the chatty loggers in certain packages.
         */
        Heart.setLoggingLevels(Level.WARNING);

        Application application = new NewtonsCradle();
        AppSettings settings = new AppSettings(true);

        settings.setGammaCorrection(true);
        settings.setSamples(4); // anti-aliasing
        settings.setTitle(applicationName); // Customize the window's title bar.
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

        ColorRGBA sky = new ColorRGBA(0.1f, 0.2f, 0.4f, 1f);
        viewPort.setBackgroundColor(sky);
        /*
         * Add the status text to the GUI.
         */
        statusText = new BitmapText(guiFont, false);
        statusText.setLocalTranslation(0f, cam.getHeight(), 0f);
        guiNode.attachChild(statusText);

        restartSimulation(5);
    }

    /**
     * Add application-specific hotkey bindings and override existing ones.
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        dim.bind("dump physicsSpace", KeyInput.KEY_O);
        dim.bind("dump scene", KeyInput.KEY_P);

        dim.bind("signal " + CameraInput.FLYCAM_LOWER, KeyInput.KEY_DOWN);
        dim.bind("signal " + CameraInput.FLYCAM_RISE, KeyInput.KEY_UP);
        dim.bind("signal orbitLeft", KeyInput.KEY_LEFT);
        dim.bind("signal orbitRight", KeyInput.KEY_RIGHT);

        dim.bind("simulate 1", KeyInput.KEY_F1);
        dim.bind("simulate 1", KeyInput.KEY_1);
        dim.bind("simulate 1", KeyInput.KEY_NUMPAD1);
        dim.bind("simulate 2", KeyInput.KEY_F2);
        dim.bind("simulate 2", KeyInput.KEY_2);
        dim.bind("simulate 2", KeyInput.KEY_NUMPAD2);
        dim.bind("simulate 3", KeyInput.KEY_F3);
        dim.bind("simulate 3", KeyInput.KEY_3);
        dim.bind("simulate 3", KeyInput.KEY_NUMPAD3);
        dim.bind("simulate 4", KeyInput.KEY_F4);
        dim.bind("simulate 4", KeyInput.KEY_4);
        dim.bind("simulate 4", KeyInput.KEY_NUMPAD4);

        dim.bind("toggle aabb", KeyInput.KEY_APOSTROPHE);
        dim.bind("toggle axes", KeyInput.KEY_SEMICOLON);
        dim.bind("toggle help", KeyInput.KEY_H);
        dim.bind("toggle pause", KeyInput.KEY_PERIOD);

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
                case "dump physicsSpace":
                    dumper.dump(physicsSpace);
                    return;
                case "dump scene":
                    dumper.dump(rootNode);
                    return;

                case "simulate 1":
                    restartSimulation(1);
                    return;
                case "simulate 2":
                    restartSimulation(2);
                    return;
                case "simulate 3":
                    restartSimulation(3);
                    return;
                case "simulate 4":
                    restartSimulation(4);
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
     * Add lighting and shadows to the specified scene.
     */
    private void addLighting(Spatial rootSpatial) {
        ColorRGBA ambientColor = new ColorRGBA(0.5f, 0.5f, 0.5f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        rootSpatial.addLight(ambient);

        Vector3f direction = new Vector3f(1f, -2f, -1f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction);
        rootSpatial.addLight(sun);
    }

    /**
     * Add a dynamic ball to the PhysicsSpace, suspended between 2 single-ended
     * point-to-point joints.
     */
    private PhysicsRigidBody addSuspendedBall(float xOffset) {
        float radius = 9.9f;
        SphereCollisionShape sphere = new SphereCollisionShape(radius);
        PhysicsRigidBody ball = new PhysicsRigidBody(sphere);
        Vector3f location = new Vector3f(xOffset, 0f, 0f);
        ball.setDebugMaterial(ballMaterial);
        ball.setDebugMeshNormals(DebugMeshNormals.Sphere);
        ball.setDebugMeshResolution(DebugShapeFactory.highResolution);
        ball.setPhysicsLocation(location);
        ball.setRestitution(1f);
        physicsSpace.add(ball);

        float wireLength = 80f;
        float yOffset = wireLength / FastMath.sqrt(2f);

        Vector3f offset = new Vector3f(0f, yOffset, +yOffset);
        Point2PointJoint joint1 = new Point2PointJoint(ball, offset);
        physicsSpace.add(joint1);

        offset.set(0f, yOffset, -yOffset);
        Point2PointJoint joint2 = new Point2PointJoint(ball, offset);
        physicsSpace.add(joint2);

        return ball;
    }

    /**
     * Remove all physics objects from the PhysicsSpace.
     */
    private void clearSpace() {
        for (PhysicsJoint joint : physicsSpace.getJointList()) {
            physicsSpace.remove(joint);
        }
        for (PhysicsCollisionObject pco : physicsSpace.getPcoList()) {
            physicsSpace.remove(pco);
        }
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(40f);
        cam.setLocation(new Vector3f(72f, 35f, 140f));
        cam.setRotation(new Quaternion(0.001f, 0.96926f, -0.031f, -0.244f));

        CameraOrbitAppState orbitState
                = new CameraOrbitAppState(cam, "orbitLeft", "orbitRight");
        stateManager.attach(orbitState);
    }

    /**
     * Configure the PhysicsDumper during startup.
     */
    private void configureDumper() {
        dumper.setEnabled(DumpFlags.JointsInBodies, true);
        dumper.setEnabled(DumpFlags.ShadowModes, true);
        dumper.setEnabled(DumpFlags.Transforms, true);
    }

    /**
     * Configure materials during startup.
     */
    private void configureMaterials() {
        ColorRGBA ballColor = new ColorRGBA(0.01f, 0.01f, 0.01f, 1f);
        ballMaterial = MyAsset.createShinyMaterial(assetManager, ballColor);
        ballMaterial.setFloat("Shininess", 100f);
        ballMaterial.setName("ball");
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        bulletAppState.setDebugEnabled(true);
        bulletAppState.setDebugInitListener(this);
        stateManager.attach(bulletAppState);

        physicsSpace = bulletAppState.getPhysicsSpace();
        physicsSpace.setAccuracy(0.01f);
        physicsSpace.setGravity(new Vector3f(0f, -250f, 0f));
    }

    /**
     * Test whether the physics simulation is paused.
     */
    private boolean isPaused() {
        if (speed <= pausedSpeed) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Restart the simulation (paused) with the specified number of balls.
     *
     * @param numBalls (&ge;1)
     */
    private void restartSimulation(int numBalls) {
        clearSpace();
        speed = pausedSpeed;

        float xSeparation = 20f;

        // center-to-center separation between the first and last balls
        float xEntent = (numBalls - 1) * xSeparation;

        float x0 = -xEntent / 2;
        PhysicsRigidBody[] balls = new PhysicsRigidBody[numBalls];
        for (int ballIndex = 0; ballIndex < numBalls; ++ballIndex) {
            float x = x0 + ballIndex * xSeparation;
            balls[ballIndex] = addSuspendedBall(x);
        }

        Vector3f kick = new Vector3f(-20f * numBalls, 0f, 0f);
        balls[0].applyCentralImpulse(kick);
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
        bulletAppState.setDebugAxisLength(2f - length);
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
        float newSpeed = isPaused() ? 1f : pausedSpeed;
        setSpeed(newSpeed);
    }

    /**
     * Update the status text in the GUI.
     */
    private void updateStatusText() {
        Vector3f gravity = physicsSpace.getGravity(null);
        double totalEnergy = 0.0;
        for (PhysicsRigidBody body : physicsSpace.getRigidBodyList()) {
            totalEnergy += body.kineticEnergy();

            body.getPhysicsLocation(tmpVector);
            double potentialEnergy = -MyVector3f.dot(tmpVector, gravity);
            totalEnergy += potentialEnergy;
        }

        String message = String.format("KE+PE=%f%s", totalEnergy,
                isPaused() ? "  PAUSED" : "");
        statusText.setText(message);
    }
}
