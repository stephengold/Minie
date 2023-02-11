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

import com.jme3.app.state.AppState;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.bullet.RotationOrder;
import com.jme3.bullet.collision.AfMode;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.MultiSphere;
import com.jme3.bullet.debug.BulletDebugAppState;
import com.jme3.bullet.debug.DebugInitListener;
import com.jme3.bullet.joints.New6Dof;
import com.jme3.bullet.joints.motors.MotorParam;
import com.jme3.bullet.joints.motors.RotationMotor;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.font.BitmapText;
import com.jme3.input.KeyInput;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.system.AppSettings;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MeshNormals;
import jme3utilities.MyAsset;
import jme3utilities.MyString;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.RectSizeLimits;
import jme3utilities.minie.test.common.PhysicsDemo;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.DisplaySettings;
import jme3utilities.ui.DsEditOverlay;
import jme3utilities.ui.InputMode;
import jme3utilities.ui.ShowDialog;
import jme3utilities.ui.Signals;

/**
 * A physics demo that simulates a cable coiled around a horizontal barrel. The
 * cable is composed of capsule-shaped rigid segments. A hook is attached to the
 * free end of the cable.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class Windlass
        extends PhysicsDemo
        implements DebugInitListener, PhysicsTickListener {
    // *************************************************************************
    // constants and loggers

    /**
     * simulation time step (in seconds)
     */
    final private static float timeStep = 0.0015f;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(Windlass.class.getName());
    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName
            = Windlass.class.getSimpleName();
    // *************************************************************************
    // fields

    /**
     * text displayed at the bottom of the GUI node
     */
    private static BitmapText statusText;
    /**
     * AppState to manage the PhysicsSpace
     */
    private static BulletAppState bulletAppState;
    /**
     * All cable segments have exactly the same shape.
     */
    private static CollisionShape segmentShape;
    /**
     * proposed display settings, for the DsEditOverlay
     */
    private static DisplaySettings proposedSettings;
    /**
     * rotation of the barrel (in radians)
     */
    private static float barrelXRotation;
    /**
     * input signal: 1&rarr;turn counter-clockwise (initially lowers the hook)
     */
    private static int signalCcw;
    /**
     * input signal: 1&rarr;turn clockwise (initially raises the hook)
     */
    private static int signalCw;
    /**
     * body that represents the barrel
     */
    private static PhysicsRigidBody barrel;
    /**
     * orientation of the barrel
     */
    final private static Quaternion barrelOrientation = new Quaternion();
    /**
     * location of the forward pivot in a segment's local coordinates
     */
    final private static Vector3f localPivot = new Vector3f();
    // *************************************************************************
    // constructors

    /**
     * Instantiate the Windlass application.
     */
    public Windlass() { // made explicit to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the Windlass application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        // Mute the chatty loggers in certain packages.
        Heart.setLoggingLevels(Level.WARNING);

        for (String arg : arguments) {
            switch (arg) {
                case "--deleteOnly":
                    Heart.deleteStoredSettings(applicationName);
                    return;
                default:
            }
        }

        Windlass application = new Windlass();
        RectSizeLimits sizeLimits = new RectSizeLimits(
                480, 240, // min width, height
                2_048, 1_080 // max width, height
        );
        final String title = applicationName + " " + MyString.join(arguments);
        proposedSettings = new DisplaySettings(
                application, applicationName, sizeLimits) {
            @Override
            protected void applyOverrides(AppSettings settings) {
                setShowDialog(ShowDialog.Never);
                settings.setAudioRenderer(null);
                settings.setRenderer(AppSettings.LWJGL_OPENGL32);
                if (settings.getSamples() < 1) {
                    settings.setSamples(4); // anti-aliasing
                }
                settings.setResizable(true);
                settings.setTitle(title); // Customize the window's title bar.
            }
        };

        AppSettings appSettings = proposedSettings.initialize();
        if (appSettings != null) {
            application.setSettings(appSettings);
            /*
             * If the settings dialog should be shown,
             * it has already been shown by DisplaySettings.initialize().
             */
            application.setShowSettings(false);

            application.start();
        }
    }
    // *************************************************************************
    // PhysicsDemo methods

    /**
     * Initialize this application.
     */
    @Override
    public void acorusInit() {
        configureCamera();
        configureDumper();
        generateMaterials();
        configurePhysics();

        ColorRGBA skyColor = new ColorRGBA(0.1f, 0.2f, 0.4f, 1f);
        viewPort.setBackgroundColor(skyColor);

        // Add the status text to the GUI.
        statusText = new BitmapText(guiFont);
        statusText.setLocalTranslation(205f, 25f, 0f);
        guiNode.attachChild(statusText);

        AppState dseOverlay = new DsEditOverlay(proposedSettings);
        boolean success = stateManager.attach(dseOverlay);
        assert success;

        super.acorusInit();

        configureCamera();
        viewPort.setBackgroundColor(skyColor);

        float cableRadius = 1f; // should be much larger than collision margin
        Vector3f attachPoint = addBarrel(cableRadius);
        /*
         * Determine the segment length, which is also the distance between
         * successive pivots.
         */
        int numSegmentsPerCoil = 12;
        float deltaPhi = FastMath.TWO_PI / numSegmentsPerCoil;
        float z0 = attachPoint.z;
        float deltaX = 2.1f * cableRadius / numSegmentsPerCoil;
        float deltaY = 2f * z0 * FastMath.tan(deltaPhi / 2f);
        float segmentLength = MyMath.hypotenuse(deltaX, deltaY);

        // The segment shape is a Z-axis capsule.
        assert segmentLength > 2f * cableRadius; // alternate segments collide!
        segmentShape = new CapsuleCollisionShape(
                cableRadius, segmentLength, PhysicsSpace.AXIS_Z);
        localPivot.set(0f, 0f, segmentLength / 2f);
        /*
         * Make the first cable segment tangent to the +Z side of the barrel
         * and attach it with a fixed joint (all DOFs locked).
         */
        float zRotation = FastMath.atan2(deltaX, deltaY);
        Quaternion orientation = new Quaternion().fromAngles(0f, zRotation, 0f);
        new Quaternion().fromAngles(FastMath.HALF_PI, 0f, 0f)
                .mult(orientation, orientation);

        PhysicsRigidBody segment = addCableSegment(attachPoint, orientation);
        New6Dof fixed = New6Dof.newInstance(
                segment, barrel, attachPoint, orientation, RotationOrder.XYZ);
        for (int axisIndex = 0; axisIndex < MyVector3f.numAxes; ++axisIndex) {
            RotationMotor motor = fixed.getRotationMotor(axisIndex);
            motor.set(MotorParam.LowerLimit, 0f);
            motor.set(MotorParam.UpperLimit, 0f);
        }
        addJoint(fixed);

        Quaternion rotatePhi = new Quaternion().fromAngles(deltaPhi, 0f, 0f);
        int numCoils = 4;
        int numCoiledSegments = numCoils * numSegmentsPerCoil;

        // Attach successive segments a spiral coiling around the barrel.
        float phi = FastMath.HALF_PI;
        PhysicsRigidBody endSegment = segment;
        Vector3f center = attachPoint.clone();
        for (int segmentI = 0; segmentI < numCoiledSegments; ++segmentI) {
            // Calculate the position of the next segment.
            center.x += deltaX;
            phi += deltaPhi;
            center.y = z0 * FastMath.cos(phi);
            center.z = z0 * FastMath.sin(phi);
            rotatePhi.mult(orientation, orientation);

            // Create a new segment and splice it to the existing cable.
            PhysicsRigidBody newSegment = addCableSegment(center, orientation);
            spliceCableSegments(newSegment, endSegment);

            endSegment = newSegment;
        }

        orientation.fromAngles(FastMath.HALF_PI, 0f, 0f);
        int numPendantSegments = 4;

        // Attach successive segments in vertical drop.
        for (int segmentI = 0; segmentI < numPendantSegments; ++segmentI) {
            // Calculate the location of the next segment.
            center.y -= segmentLength;

            // Create a new segment and splice it to the existing cable.
            PhysicsRigidBody newSegment = addCableSegment(center, orientation);
            spliceCableSegments(newSegment, endSegment);

            endSegment = newSegment;
        }

        addHook(endSegment, cableRadius);
    }

    /**
     * Initialize the library of named materials during startup.
     */
    @Override
    public void generateMaterials() {
        ColorRGBA gray = new ColorRGBA(0.2f, 0.2f, 0.2f, 1f);
        Material shiny = MyAsset.createShinyMaterial(assetManager, gray);
        shiny.setFloat("Shininess", 100f);
        registerMaterial("shiny", shiny);

        ColorRGBA brown = new ColorRGBA(0.15f, 0.1f, 0f, 1f);
        Material drab = MyAsset.createShadedMaterial(assetManager, brown);
        registerMaterial("drab", drab);
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
        return 8f;
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

        dim.bind(asEditDisplaySettings, KeyInput.KEY_TAB);

        dim.bindSignal("orbitLeft", KeyInput.KEY_LEFT);
        dim.bindSignal("orbitRight", KeyInput.KEY_RIGHT);

        dim.bind(asToggleAabbs, KeyInput.KEY_APOSTROPHE);
        dim.bind(asToggleHelp, KeyInput.KEY_H);
        dim.bind(asTogglePause, KeyInput.KEY_PAUSE, KeyInput.KEY_PERIOD);
        dim.bind(asTogglePcoAxes, KeyInput.KEY_SEMICOLON);
        dim.bind(asToggleVArrows, KeyInput.KEY_K);
        dim.bind(asToggleWArrows, KeyInput.KEY_N);

        dim.bindSignal("wind ccw", KeyInput.KEY_DOWN);
        dim.bindSignal("wind cw", KeyInput.KEY_UP);
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
        signalCcw = signals.test("wind ccw") ? 1 : 0;
        signalCw = signals.test("wind cw") ? 1 : 0;

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
    // PhysicsTickListener methods

    /**
     * Callback from Bullet, invoked just after each simulation step.
     *
     * @param space the space that was just stepped (not null)
     * @param timeStep the time per simulation step (in seconds, &ge;0)
     */
    @Override
    public void physicsTick(PhysicsSpace space, float timeStep) {
        // do nothing
    }

    /**
     * Callback from Bullet, invoked just before each simulation step.
     *
     * @param space the space that's about to be stepped (not null)
     * @param timeStep the time per simulation step (in seconds, &ge;0)
     */
    @Override
    public void prePhysicsTick(PhysicsSpace space, float timeStep) {
        // Turn the barrel based on user-input signals.
        float turnRate = 4f; // radians per second
        barrelXRotation += (signalCcw - signalCw) * turnRate * timeStep;
        barrelOrientation.fromAngles(barrelXRotation, 0f, 0f);
        barrel.setPhysicsRotation(barrelOrientation);
    }
    // *************************************************************************
    // private methods

    /**
     * Add the barrel, which is a kinematic rigid body shaped like a horizontal
     * cylinder, with flanges and handles at both ends.
     *
     * @param cableRadius the radius of the cable (in physics-space units,
     * &gt;0)
     * @return the attachment point for the cable (a location vector in physics
     * space)
     */
    private Vector3f addBarrel(float cableRadius) {
        int axis = PhysicsSpace.AXIS_X;

        float drumLength = 12f * cableRadius;
        float drumRadius = 0.6f * drumLength;
        CollisionShape cylinderShape = new CylinderCollisionShape(
                drumRadius, drumLength, axis);

        float flangeRadius = drumRadius + 3.5f * cableRadius;
        float flangeWidth = 0.1f * drumLength;
        CollisionShape flangeShape = new CylinderCollisionShape(
                flangeRadius, flangeWidth, axis);

        float handleRadius = 0.8f * cableRadius;
        float handleLength = 8f * cableRadius;
        CollisionShape handleShape = new CylinderCollisionShape(
                handleRadius, handleLength, axis);

        CompoundCollisionShape barrelShape = new CompoundCollisionShape(5);
        barrelShape.addChildShape(cylinderShape);

        float flangeX = (drumLength + flangeWidth) / 2f;
        barrelShape.addChildShape(flangeShape, +flangeX, 0f, 0f);
        barrelShape.addChildShape(flangeShape, -flangeX, 0f, 0f);

        float handleX = drumLength / 2f + flangeWidth + handleLength / 2f;
        float handleY = flangeRadius - handleRadius;
        barrelShape.addChildShape(handleShape, +handleX, +handleY, 0f);
        barrelShape.addChildShape(handleShape, -handleX, -handleY, 0f);

        float barrelMass = 100f;
        barrel = new PhysicsRigidBody(barrelShape, barrelMass);
        barrel.setKinematic(true);
        barrel.setAnisotropicFriction(
                new Vector3f(900f, 10f, 10f), AfMode.basic);
        barrel.setFriction(0f); // disable normal friction

        Material material = findMaterial("drab");
        barrel.setDebugMaterial(material);
        barrel.setDebugMeshNormals(MeshNormals.Smooth);
        barrel.setDebugMeshResolution(DebugShapeFactory.highResolution);

        addCollisionObject(barrel);

        // Calculate an attachment point on the +Z side of the drum;
        float x0 = -0.49f * drumLength + cableRadius;
        float z0 = drumRadius + cableRadius;
        Vector3f result = new Vector3f(x0, 0f, z0);

        return result;
    }

    /**
     * Add a single segment of cable.
     *
     * @param center the desired center location in physics space (not null,
     * unaffected)
     * @param orientation the desired orientation in physics space (not null,
     * unaffected)
     * @return a new instance
     */
    private PhysicsRigidBody addCableSegment(
            Vector3f center, Quaternion orientation) {
        float mass = 0.2f;
        PhysicsRigidBody result = new PhysicsRigidBody(segmentShape, mass);

        result.setPhysicsLocation(center);
        result.setPhysicsRotation(orientation);

        Material material = findMaterial("shiny");
        result.setDebugMaterial(material);
        result.setDebugMeshNormals(MeshNormals.Smooth);
        result.setDebugMeshResolution(DebugShapeFactory.highResolution);

        addCollisionObject(result);

        return result;
    }

    /**
     * Attach a hook to the end of the cable.
     *
     * @param endSegment the final segment of the cable (not null)
     * @param cableRadius the radius of the cable (&gt;0)
     */
    private void addHook(PhysicsRigidBody endSegment, float cableRadius) {
        // Locate the final pivot.
        Transform endTransform = endSegment.getTransform(null);
        Vector3f pivotLocation = endTransform.transformVector(localPivot, null);
        /*
         * Collision shape is composed of 11 overlapping 2-sphere shapes,
         * arranged in a circular arc.
         */
        int numChildren = 11;
        int numSpheres = numChildren + 1;
        float hookRadius = 4f * cableRadius;
        float maxThick = 2.1f * cableRadius; // max thickness
        float minThick = 0.5f * cableRadius; // min thickness

        float[] radius = new float[numSpheres];
        float[] y = new float[numSpheres];
        float[] z = new float[numSpheres];
        float xAngle = 0f; // in radians
        for (int sphereI = 0; sphereI < numSpheres; ++sphereI) {
            float p = sphereI / (float) (numSpheres - 1); // goes from 0 to 1
            float p3 = FastMath.pow(p, 3f);
            float thickness = maxThick - p3 * (maxThick - minThick);
            radius[sphereI] = thickness / 2f;
            if (sphereI > 0) {
                xAngle += radius[sphereI] / hookRadius;
            }
            y[sphereI] = hookRadius * FastMath.cos(xAngle);
            z[sphereI] = -hookRadius * FastMath.sin(xAngle);
            xAngle += radius[sphereI] / hookRadius;
        }

        List<Vector3f> centers = new ArrayList<>(2);
        centers.add(new Vector3f());
        centers.add(new Vector3f());

        List<Float> radii = new ArrayList<>(2);
        radii.add(0f);
        radii.add(0f);

        CompoundCollisionShape shape = new CompoundCollisionShape(numChildren);
        for (int childI = 0; childI < numChildren; ++childI) {
            centers.get(0).set(0f, y[childI], z[childI]);
            radii.set(0, radius[childI]);

            int nextI = childI + 1;
            centers.get(1).set(0f, y[nextI], z[nextI]);
            radii.set(1, radius[nextI]);

            MultiSphere twoSphere = new MultiSphere(centers, radii);
            shape.addChildShape(twoSphere);
        }

        float hookMass = 3f;
        PhysicsRigidBody hook = new PhysicsRigidBody(shape, hookMass);
        hook.setAngularDamping(0.7f);
        hook.setLinearDamping(0.4f);

        float pivotY = hookRadius + maxThick / 2f;
        Vector3f center = pivotLocation.subtract(0f, pivotY, 0f);
        hook.setPhysicsLocation(center);

        Material material = findMaterial("shiny");
        hook.setDebugMaterial(material);
        hook.setDebugMeshNormals(MeshNormals.Smooth);
        hook.setDebugMeshResolution(DebugShapeFactory.highResolution);

        addCollisionObject(hook);

        Quaternion orientation = endTransform.getRotation(); // alias
        New6Dof joint = New6Dof.newInstance(hook, endSegment,
                pivotLocation, orientation, RotationOrder.XYZ);
        joint.setCollisionBetweenLinkedBodies(false);
        addJoint(joint);
    }

    /**
     * Add lighting and shadows to the specified scene.
     *
     * @param rootSpatial which scene (not null)
     */
    private static void addLighting(Spatial rootSpatial) {
        ColorRGBA ambientColor = new ColorRGBA(0.5f, 0.5f, 0.5f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        rootSpatial.addLight(ambient);
        ambient.setName("ambient");

        Vector3f direction = new Vector3f(1f, -2f, -1f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction);
        rootSpatial.addLight(sun);
        sun.setName("sun");
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(20f);
        flyCam.setZoomSpeed(20f);

        cam.setLocation(new Vector3f(30f, 25f, 135f));
        cam.setRotation(new Quaternion(-0.02f, 0.975877f, -0.19204f, -0.1019f));

        AppState orbitState
                = new CameraOrbitAppState(cam, "orbitLeft", "orbitRight");
        stateManager.attach(orbitState);
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        bulletAppState = new BulletAppState();
        bulletAppState.setDebugEnabled(true);

        // Visualize only the rigid bodies, not the joints.
        bulletAppState.setDebugFilter(
                new BulletDebugAppState.DebugAppStateFilter() {
            @Override
            public boolean displayObject(Object object) {
                return object instanceof PhysicsRigidBody;
            }
        });
        bulletAppState.setDebugInitListener(this);

        stateManager.attach(bulletAppState);

        PhysicsSpace space = getPhysicsSpace();
        space.addTickListener(this);
        space.setAccuracy(timeStep);
        space.setMaxSubSteps(99); // default=4
        setGravityAll(981f); // 1 psu = 1 cm
    }

    /**
     * Connect the specified cable segments with a New6Dof joint.
     *
     * @param newSegment the new cable segment (not null)
     * @param endSegment the final segment of the cable so far (not null)
     */
    private void spliceCableSegments(
            PhysicsRigidBody newSegment, PhysicsRigidBody endSegment) {
        // Position the pivot.
        Transform endTransform = endSegment.getTransform(null);
        Vector3f pivotLocation = endTransform.transformVector(localPivot, null);

        Quaternion pivotOrientation = endSegment.getPhysicsRotation(null);
        New6Dof joint = New6Dof.newInstance(newSegment, endSegment,
                pivotLocation, pivotOrientation, RotationOrder.XYZ);
        joint.setCollisionBetweenLinkedBodies(false);

        RotationMotor zrMotor = joint.getRotationMotor(PhysicsSpace.AXIS_Z);
        zrMotor.set(MotorParam.Damping, 0.25f / timeStep);
        zrMotor.set(MotorParam.LowerLimit, 0f);
        zrMotor.set(MotorParam.UpperLimit, 0f);
        zrMotor.setSpringEnabled(true);

        addJoint(joint);
    }

    /**
     * Update the status text in the GUI.
     */
    private void updateStatusText() {
        String message = isPaused() ? "  PAUSED" : "";
        statusText.setText(message);
    }
}
