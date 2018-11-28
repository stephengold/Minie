/*
 Copyright (c) 2018, Stephen Gold
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

import com.jme3.audio.openal.ALAudioRenderer;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.MultiSphere;
import com.jme3.bullet.collision.shapes.infos.DebugMeshNormals;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.debug.BulletDebugAppState;
import com.jme3.bullet.debug.DebugInitListener;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.export.Savable;
import com.jme3.input.KeyInput;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Box;
import com.jme3.system.AppSettings;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Misc;
import jme3utilities.MyAsset;
import jme3utilities.MyString;
import jme3utilities.math.noise.Generator;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.ui.ActionApplication;
import jme3utilities.ui.InputMode;
import jme3utilities.ui.Signals;

/**
 * Demo for multi-sphere collision shapes.
 */
public class MultiSphereDemo
        extends ActionApplication
        implements BulletDebugAppState.DebugAppStateFilter, DebugInitListener {
    // *************************************************************************
    // constants and loggers

    /**
     * upper limit on the number of gems
     */
    final private static int maxNumGems = 80;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(MultiSphereDemo.class.getName());
    /**
     * application name for the window's title bar
     */
    final private static String applicationName = "MultiSphereDemo";
    // *************************************************************************
    // fields

    private BulletAppState bulletAppState;
    final private Generator random = new Generator();
    private int numGems = 0;
    final private Material gemMaterials[] = new Material[4];
    private PhysicsSpace physicsSpace;
    private RigidBodyControl boxBody;
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        /*
         * Mute the chatty loggers found in some imported packages.
         */
        Misc.setLoggingLevels(Level.WARNING);
        Logger.getLogger(ALAudioRenderer.class.getName())
                .setLevel(Level.SEVERE);

        MultiSphereDemo application = new MultiSphereDemo();
        /*
         * Customize the window's title bar.
         */
        AppSettings settings = new AppSettings(true);
        String title = applicationName + " " + MyString.join(arguments);
        settings.setTitle(title);
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
        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(4f);
        cam.setLocation(new Vector3f(0f, 1.2f, 5f));

        ColorRGBA gemColors[] = new ColorRGBA[gemMaterials.length];
        gemColors[0] = new ColorRGBA(0.2f, 0f, 0f, 1f); // ruby
        gemColors[1] = new ColorRGBA(0f, 0.07f, 0f, 1f); // emerald
        gemColors[2] = new ColorRGBA(0f, 0f, 0.3f, 1f); // sapphire
        gemColors[3] = new ColorRGBA(0.2f, 0.1f, 0f, 1f); // topaz

        for (int i = 0; i < gemMaterials.length; ++i) {
            ColorRGBA color = gemColors[i];
            gemMaterials[i]
                    = MyAsset.createShinyMaterial(assetManager, color);
            gemMaterials[i].setFloat("Shininess", 15f);
        }

        viewPort.setBackgroundColor(ColorRGBA.Gray);
        addLighting(rootNode);

        CollisionShape.setDefaultMargin(0.005f); // 5 mm
        bulletAppState = new BulletAppState();
        bulletAppState.setDebugEnabled(true);
        bulletAppState.setDebugFilter(this);
        bulletAppState.setDebugInitListener(this);
        stateManager.attach(bulletAppState);
        physicsSpace = bulletAppState.getPhysicsSpace();
        physicsSpace.setSolverNumIterations(30);

        addBox();
    }

    /**
     * Add application-specific hotkey bindings and override existing ones.
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        dim.bind("dump physicsSpace", KeyInput.KEY_O);
        dim.bind("dump scenes", KeyInput.KEY_P);
        dim.bind("signal shower", KeyInput.KEY_INSERT);
        dim.bind("toggle pause", KeyInput.KEY_PERIOD);
    }

    /**
     * Process an action that wasn't handled by the active input mode.
     *
     * @param actionString textual description of the action (not null)
     * @param ongoing true if the action is ongoing, otherwise false
     * @param tpf time interval between render passes (in seconds, &ge;0)
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
                case "toggle pause":
                    togglePause();
                    return;
            }
        }
        super.onAction(actionString, ongoing, tpf);
    }

    /**
     * Callback invoked once per render pass.
     *
     * @param tpf time interval between render passes (in seconds, &ge;0)
     */
    @Override
    public void simpleUpdate(float tpf) {
        super.simpleUpdate(tpf);

        Signals signals = getSignals();
        if (signals.test("shower")) {
            addAGem();
        }
    }
    // *************************************************************************
    // DebugAppStateFilter methods

    /**
     * Test whether the specified physics object should be displayed in the
     * debug scene.
     *
     * @param object the joint or collision object to test (unaffected)
     * @return return true if the object should be displayed, false if not
     */
    @Override
    public boolean displayObject(Savable object) {
        return object != boxBody;
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
     * Add a dynamic object to the scene.
     */
    private void addAGem() {
        if (numGems >= maxNumGems) {
            return;
        }

        int numSpheres = 1 + random.nextInt(4);
        List<Vector3f> centers = new ArrayList<>(numSpheres);
        List<Float> radii = new ArrayList<>(numSpheres);
        centers.add(new Vector3f(0f, 0f, 0f));
        float mainRadius = 0.1f + 0.2f * random.nextFloat();
        radii.add(mainRadius);

        for (int sphereIndex = 1; sphereIndex < numSpheres; ++sphereIndex) {
            Vector3f center = random.nextUnitVector3f();
            center.multLocal(mainRadius);
            centers.add(center);

            float radius = mainRadius * (0.2f + 0.8f * random.nextFloat());
            radii.add(radius);
        }
        CollisionShape shape = new MultiSphere(centers, radii);

        Vector3f startLocation = random.nextVector3f();
        startLocation.multLocal(0.5f, 1f, 0.5f);
        startLocation.y += 4f;

        Material debugMaterial = (Material) random.pick(gemMaterials);

        float mass = 1f;
        PhysicsRigidBody body = new PhysicsRigidBody(shape, mass);
        body.setDamping(0.6f, 0.6f);
        body.setDebugMaterial(debugMaterial);
        body.setDebugMeshNormals(DebugMeshNormals.Smooth);
        body.setDebugMeshResolution(DebugShapeFactory.highResolution);
        body.setFriction(1f);
        body.setKinematic(false);
        body.setPhysicsLocation(startLocation);

        physicsSpace.add(body);
        body.setGravity(new Vector3f(0f, -9f, 0f));

        ++numGems;
    }

    /**
     * Add a large static box to serve as a platform.
     */
    private void addBox() {
        float halfExtent = 50f; // mesh units
        Mesh mesh = new Box(halfExtent, halfExtent, halfExtent);
        Geometry geometry = new Geometry("box", mesh);
        rootNode.attachChild(geometry);

        geometry.move(0f, -halfExtent, 0f);
        ColorRGBA color = new ColorRGBA(0.1f, 0.3f, 0.1f, 1f);
        Material material = MyAsset.createShadedMaterial(assetManager, color);
        geometry.setMaterial(material);
        geometry.setShadowMode(RenderQueue.ShadowMode.Receive);

        Vector3f hes = new Vector3f(halfExtent, halfExtent, halfExtent);
        BoxCollisionShape shape = new BoxCollisionShape(hes);
        float mass = PhysicsRigidBody.massForStatic;
        boxBody = new RigidBodyControl(shape, mass);
        boxBody.setApplyScale(true);
        boxBody.setKinematic(true);
        boxBody.setPhysicsSpace(physicsSpace);

        geometry.addControl(boxBody);
    }

    /**
     * Add lighting to the specified scene.
     */
    private void addLighting(Spatial rootSpatial) {
        ColorRGBA ambientColor = new ColorRGBA(0.7f, 0.7f, 0.7f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        rootSpatial.addLight(ambient);

        Vector3f direction = new Vector3f(1f, -2f, -1f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction);
        rootSpatial.addLight(sun);
    }

    /**
     * Process a "dump physicsSpace" action.
     */
    private void dumpPhysicsSpace() {
        PhysicsDumper dumper = new PhysicsDumper();
        dumper.dump(physicsSpace);
    }

    /**
     * Process a "dump scenes" action.
     */
    private void dumpScenes() {
        PhysicsDumper dumper = new PhysicsDumper();
        //dumper.setDumpBucket(true);
        //dumper.setDumpCull(true);
        //dumper.setDumpOverride(true);
        //dumper.setDumpShadow(true);
        dumper.setDumpTransform(true);
        //dumper.setDumpUser(true);
        dumper.dump(renderManager);
    }

    /**
     * Toggle the animation and physics simulation: paused/running.
     */
    private void togglePause() {
        float newSpeed = (speed > Float.MIN_VALUE) ? Float.MIN_VALUE : 1f;
        setSpeed(newSpeed);
    }
}
