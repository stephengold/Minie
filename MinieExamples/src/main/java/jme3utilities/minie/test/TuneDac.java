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

import jme3utilities.minie.test.tunings.SinbadControl;
import jme3utilities.minie.test.tunings.PuppetControl;
import jme3utilities.minie.test.tunings.OtoControl;
import jme3utilities.minie.test.tunings.NinjaControl;
import jme3utilities.minie.test.tunings.JaimeControl;
import jme3utilities.minie.test.tunings.ElephantControl;
import jme3utilities.minie.test.tunings.MhGameControl;
import com.jme3.animation.Skeleton;
import com.jme3.animation.SkeletonControl;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.animation.BoneLink;
import com.jme3.bullet.animation.DynamicAnimControl;
import com.jme3.bullet.animation.PhysicsLink;
import com.jme3.bullet.animation.RangeOfMotion;
import com.jme3.bullet.animation.TorsoLink;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.joints.SixDofJoint;
import com.jme3.bullet.joints.motors.RotationalLimitMotor;
import com.jme3.input.KeyInput;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import java.util.Arrays;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.MyAsset;
import jme3utilities.MySpatial;
import jme3utilities.MyString;
import jme3utilities.debug.SkeletonVisualizer;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.ui.ActionApplication;
import jme3utilities.ui.InputMode;
import jme3utilities.ui.Signals;

/**
 * Interim tuning tool for DynamicAnimControl.
 */
public class TuneDac extends ActionApplication {
    // *************************************************************************
    // constants and loggers

    /**
     * number of axes
     */
    final private static int numAxes = 3;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(TuneDac.class.getName());
    /**
     * name of each axis
     */
    final private static String[] axisNames = new String[]{"X", "Y", "Z"};
    // *************************************************************************
    // fields

    private BulletAppState bulletAppState;
    private DynamicAnimControl dac;
    private int motorVelocitySign = 0;
    private int wiggleAxis = -1;
    private Material magenta;
    private Node model;
    private PhysicsLink wiggleLink;
    private RotationalLimitMotor motor = null;
    private Skeleton skeleton;
    private SkeletonVisualizer sv;
    private Transform resetTransform;
    // *************************************************************************
    // new methods exposed

    public static void main(String[] args) {
        TuneDac app = new TuneDac();
        app.start();
    }
    // *************************************************************************
    // ActionApplication methods

    /**
     * Initialize this application.
     */
    @Override
    public void actionInitializeApplication() {
        flyCam.setEnabled(false);

        cam.setLocation(new Vector3f(0f, 1f, 4f));
        viewPort.setBackgroundColor(ColorRGBA.Gray);
        addLighting();

        bulletAppState = new BulletAppState();
        bulletAppState.setDebugEnabled(true);
        stateManager.attach(bulletAppState);
        magenta = MyAsset.createWireframeMaterial(assetManager,
                ColorRGBA.Magenta);

        CollisionShape.setDefaultMargin(0.005f); // 5 mm
        PhysicsSpace ps = bulletAppState.getPhysicsSpace();
        ps.setSolverNumIterations(30);
        addModel();

        List<SkeletonControl> scList
                = MySpatial.listControls(model, SkeletonControl.class, null);
        SkeletonControl sc = scList.get(0);
        sv = new SkeletonVisualizer(assetManager, sc);
        sv.setLineColor(ColorRGBA.Yellow);
        rootNode.addControl(sv);

        dac.setPhysicsSpace(ps);
        TorsoLink torsoLink = dac.getTorsoLink();
        dac.bindSubtree(torsoLink, 0f);
        setWiggleLink(torsoLink);
    }

    /**
     * Add new hotkey bindings and override existing ones.
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        dim.bind("dump scene", KeyInput.KEY_P);
        dim.bind("dump skeleton", KeyInput.KEY_LBRACKET);
        dim.bind("reset model transform", KeyInput.KEY_DOWN);
        dim.bind("signal rotateLeft", KeyInput.KEY_LEFT);
        dim.bind("signal rotateRight", KeyInput.KEY_RIGHT);
        dim.bind("toggle meshes", KeyInput.KEY_M);
        dim.bind("toggle physics debug", KeyInput.KEY_SLASH);
        dim.bind("toggle skeleton", KeyInput.KEY_V);
        dim.bind("wiggle bone first child", KeyInput.KEY_NUMPAD2);
        dim.bind("wiggle bone next sibling", KeyInput.KEY_NUMPAD6);
        dim.bind("wiggle bone parent", KeyInput.KEY_NUMPAD8);
        dim.bind("wiggle bone prev sibling", KeyInput.KEY_NUMPAD4);
        dim.bind("wiggle bone x", KeyInput.KEY_X);
        dim.bind("wiggle bone y", KeyInput.KEY_Y);
        dim.bind("wiggle bone z", KeyInput.KEY_Z);
    }

    @Override
    public void onAction(String actionString, boolean ongoing, float tpf) {
        if (ongoing) {
            switch (actionString) {
                case "dump scene":
                    new PhysicsDumper().dump(renderManager);
                    return;
                case "dump skeleton":
                    new PhysicsDumper().dump(skeleton, "");
                    return;
                case "reset model transform":
                    model.setLocalTransform(resetTransform);
                    return;
                case "toggle meshes":
                    toggleMeshes();
                    return;
                case "toggle skeleton":
                    toggleSkeleton();
                    return;
                case "toggle physics debug":
                    togglePhysicsDebug();
                    return;

                case "wiggle bone first child":
                    PhysicsLink[] children = wiggleLink.listChildren();
                    if (children.length > 0) {
                        PhysicsLink firstChild = children[0];
                        setWiggleLink(firstChild);
                    }
                    return;

                case "wiggle bone next sibling":
                    PhysicsLink parent = wiggleLink.getParent();
                    if (parent != null) {
                        PhysicsLink[] siblings = parent.listChildren();
                        int i = Arrays.asList(siblings).indexOf(wiggleLink);
                        i = MyMath.modulo(i + 1, siblings.length);
                        PhysicsLink nc = siblings[i];
                        setWiggleLink(nc);
                    }
                    return;

                case "wiggle bone parent":
                    parent = wiggleLink.getParent();
                    if (parent != null) {
                        setWiggleLink(parent);
                    }
                    return;

                case "wiggle bone prev sibling":
                    parent = wiggleLink.getParent();
                    if (parent != null) {
                        PhysicsLink[] siblings = parent.listChildren();
                        int i = Arrays.asList(siblings).indexOf(wiggleLink);
                        i = MyMath.modulo(i - 1, siblings.length);
                        PhysicsLink nc = siblings[i];
                        setWiggleLink(nc);
                    }
                    return;

                case "wiggle bone x":
                    setWiggleAxis(PhysicsSpace.AXIS_X);
                    return;
                case "wiggle bone y":
                    setWiggleAxis(PhysicsSpace.AXIS_Y);
                    return;
                case "wiggle bone z":
                    setWiggleAxis(PhysicsSpace.AXIS_Z);
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
        if (signals.test("rotateRight")) {
            model.rotate(0f, tpf, 0f);
        }
        if (signals.test("rotateLeft")) {
            model.rotate(0f, -tpf, 0f);
        }

        String text = wiggleLink.boneName();
        if (wiggleLink.getParent() == null) {
            text = "TORSO";
        }
        if (motor != null) {
            motor.setTargetVelocity(motorVelocitySign * 2f);

            float angle = motor.getAngle();
            String axisSign = (motorVelocitySign < 0) ? "-" : "+";
            String axisName = axisNames[wiggleAxis];
            text += String.format(" %s%s angle=%4.2f", axisSign, axisName,
                    angle);
            fpsText.setText(text);
        } else {
            fpsText.setText(text);
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Add lighting to the scene.
     */
    private void addLighting() {
        ColorRGBA lightColor = new ColorRGBA(0.7f, 0.7f, 0.7f, 1f);
        AmbientLight ambient = new AmbientLight(lightColor);
        rootNode.addLight(ambient);

        Vector3f direction = new Vector3f(1f, -2f, -1f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction, lightColor);
        rootNode.addLight(sun);
    }

    /**
     * Add an animated model to the scene.
     */
    private void addModel() {
        //loadElephant();
        //loadJaime();
        //loadMhGame();
        loadNinja();
        //loadOto();
        //loadPuppet();
        //loadSinbad();

        rootNode.attachChild(model);
        setHeight(model, 2f);
        center(model);
        resetTransform = model.getLocalTransform().clone();

        List<SkeletonControl> scList
                = MySpatial.listControls(model, SkeletonControl.class, null);
        assert scList.size() == 1;
        SkeletonControl sc = scList.get(0);
        skeleton = sc.getSkeleton();

        Spatial controlledSpatial = sc.getSpatial();
        controlledSpatial.addControl(dac);
    }

    /**
     * Translate a model's center so that the model rests on the X-Z plane, and
     * its center lies on the Y axis.
     */
    private void center(Spatial model) {
        Vector3f[] minMax = MySpatial.findMinMaxCoords(model);
        Vector3f center = MyVector3f.midpoint(minMax[0], minMax[1]);
        Vector3f offset = new Vector3f(center.x, minMax[0].y, center.z);

        Vector3f location = model.getWorldTranslation();
        location.subtractLocal(offset);
        MySpatial.setWorldLocation(model, location);
    }

    /**
     * Load the Elephant model.
     */
    private void loadElephant() {
        model = (Node) assetManager.loadModel(
                "Models/Elephant/Elephant.mesh.xml");
        model.setCullHint(Spatial.CullHint.Never);
        model.rotate(0f, 1.6f, 0f);
        dac = new ElephantControl();
    }

    /**
     * Load the Jaime model.
     */
    private void loadJaime() {
        model = (Node) assetManager.loadModel("Models/Jaime/Jaime.j3o");
        dac = new JaimeControl();
    }

    /**
     * Load the MhGame model.
     */
    private void loadMhGame() {
        model = (Node) assetManager.loadModel("Models/MhGame/MhGame.j3o");
        dac = new MhGameControl();
    }

    /**
     * Load the Ninja model.
     */
    private void loadNinja() {
        model = (Node) assetManager.loadModel("Models/Ninja/Ninja.mesh.xml");
        model.rotate(0f, 3f, 0f);
        dac = new NinjaControl();
    }

    /**
     * Load the Oto model.
     */
    private void loadOto() {
        model = (Node) assetManager.loadModel("Models/Oto/Oto.mesh.xml");
        dac = new OtoControl();
    }

    /**
     * Load the Puppet model.
     */
    private void loadPuppet() {
        model = (Node) assetManager.loadModel("Models/Puppet/Puppet.j3o");
        dac = new PuppetControl();
    }

    /**
     * Load the Sinbad model.
     */
    private void loadSinbad() {
        model = (Node) assetManager.loadModel("Models/Sinbad/Sinbad.mesh.xml");
        dac = new SinbadControl();
    }

    /**
     * Scale the specified model uniformly so that it has the specified height.
     *
     * @param model (not null, modified)
     * @param height (in world units)
     */
    private void setHeight(Spatial model, float height) {
        Vector3f[] minMax = MySpatial.findMinMaxCoords(model);
        float oldHeight = minMax[1].y - minMax[0].y;

        model.scale(height / oldHeight);
    }

    /**
     * Start wiggling the indexed rotational axis of the current joint.
     *
     * @param axisIndex the axis index: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     */
    private void setWiggleAxis(int axisIndex) {
        assert axisIndex >= 0 : axisIndex;
        assert axisIndex < numAxes : axisIndex;

        SixDofJoint joint = (SixDofJoint) wiggleLink.getJoint();
        if (joint == null) {
            return;
        }
        if (wiggleAxis != axisIndex) {
            if (motor != null) {
                motor.setEnableMotor(false);
            }
            logger.log(Level.SEVERE, "change rotation axis");
            wiggleAxis = axisIndex;
            motorVelocitySign = 1;
            RangeOfMotion newRom = new RangeOfMotion(axisIndex);
            newRom.setupJoint(joint, false, false, false);
            motor = joint.getRotationalLimitMotor(axisIndex);
            motor.setEnableMotor(true);
            motor.setMaxMotorForce(100f);

        } else {
            logger.log(Level.SEVERE, "reverse rotation");
            motorVelocitySign *= -1;
        }
    }

    private void setWiggleLink(PhysicsLink link) {
        if (wiggleLink != link) {
            String name = link.boneName();
            logger.log(Level.SEVERE, "change link to {0}",
                    MyString.quote(name));
            wiggleLink = link;
            wiggleAxis = -1;
            motor = null;
            motorVelocitySign = 0;

            TorsoLink torsoLink = dac.getTorsoLink();
            dac.bindSubtree(torsoLink, 0.5f);
            if (wiggleLink == torsoLink) {
                torsoLink.getRigidBody().setDebugMaterial(magenta);
            } else {
                torsoLink.getRigidBody().setDebugMaterial(null);
                BoneLink boneLink = (BoneLink) wiggleLink;
                boneLink.setDynamic(Vector3f.ZERO, false, false, false);
            }
        }
    }

    /**
     * Toggle mesh rendering on/off.
     */
    private void toggleMeshes() {
        Spatial.CullHint hint = model.getLocalCullHint();
        if (hint == Spatial.CullHint.Inherit
                || hint == Spatial.CullHint.Never) {
            hint = Spatial.CullHint.Always;
        } else if (hint == Spatial.CullHint.Always) {
            hint = Spatial.CullHint.Never;
        }
        model.setCullHint(hint);
    }

    /**
     * Toggle the physics-debug visualization on/off.
     */
    private void togglePhysicsDebug() {
        boolean enabled = bulletAppState.isDebugEnabled();
        bulletAppState.setDebugEnabled(!enabled);
    }

    /**
     * Toggle the skeleton visualizer on/off.
     */
    private void toggleSkeleton() {
        boolean enabled = sv.isEnabled();
        sv.setEnabled(!enabled);
    }
}
