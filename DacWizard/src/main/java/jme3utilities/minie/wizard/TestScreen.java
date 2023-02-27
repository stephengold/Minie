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
package jme3utilities.minie.wizard;

import com.jme3.animation.Skeleton;
import com.jme3.app.Application;
import com.jme3.app.state.AppStateManager;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.animation.BoneLink;
import com.jme3.bullet.animation.DacConfiguration;
import com.jme3.bullet.animation.DynamicAnimControl;
import com.jme3.bullet.animation.PhysicsLink;
import com.jme3.bullet.animation.RagUtils;
import com.jme3.bullet.animation.TorsoLink;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.PlaneCollisionShape;
import com.jme3.bullet.joints.Constraint;
import com.jme3.bullet.joints.JointEnd;
import com.jme3.bullet.joints.New6Dof;
import com.jme3.bullet.joints.SixDofJoint;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Plane;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Spatial;
import com.jme3.scene.control.AbstractControl;
import java.util.List;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.InitialState;
import jme3utilities.MyAsset;
import jme3utilities.MyString;
import jme3utilities.debug.AxesVisualizer;
import jme3utilities.debug.SkeletonVisualizer;
import jme3utilities.nifty.GuiScreenController;
import jme3utilities.ui.InputMode;

/**
 * The screen controller for the "test" screen of DacWizard.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class TestScreen extends GuiScreenController {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final static Logger logger = Logger.getLogger(TestScreen.class.getName());
    // *************************************************************************
    // fields

    /**
     * animation time of the pose being viewed (in seconds)
     */
    private float viewedAnimationTime;
    /**
     * debug material for the selected PhysicsLink
     */
    private Material selectMaterial;
    /**
     * horizontal plane added to physics space, or null if not added
     */
    private PhysicsRigidBody groundPlane = null;
    /**
     * root spatial of the C-G model being viewed, or null for none
     */
    private Spatial viewedSpatial = null;
    /**
     * clip/animation name of the pose being viewed
     */
    private String viewedAnimationName;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an uninitialized, disabled screen that will not be enabled
     * during initialization.
     */
    TestScreen() {
        super("test", "Interface/Nifty/screens/wizard/test.xml",
                InitialState.Disabled);
    }
    // *************************************************************************
    // GuiScreenController methods

    /**
     * Initialize this (disabled) screen prior to its first update.
     *
     * @param stateManager (not null)
     * @param application (not null)
     */
    @Override
    public void initialize(
            AppStateManager stateManager, Application application) {
        super.initialize(stateManager, application);

        InputMode inputMode = InputMode.findMode("test");
        assert inputMode != null;
        setListener(inputMode);
        inputMode.influence(this);

        if (selectMaterial == null) {
            this.selectMaterial = MyAsset.createWireframeMaterial(
                    assetManager, ColorRGBA.White);
        }
    }

    /**
     * A callback from Nifty, invoked each time this screen shuts down.
     */
    @Override
    public void onEndScreen() {
        super.onEndScreen();
        removeGroundPlane();
    }

    /**
     * A callback from Nifty, invoked each time this screen starts up.
     */
    @Override
    public void onStartScreen() {
        super.onStartScreen();

        removeGroundPlane();
        DacWizard wizard = DacWizard.getApplication();
        wizard.clearScene();
        this.viewedSpatial = null;
        this.viewedAnimationName = null;
        this.viewedAnimationTime = Float.NaN;

        BulletAppState bulletAppState
                = DacWizard.findAppState(BulletAppState.class);
        bulletAppState.setDebugEnabled(true);
    }

    /**
     * Update this ScreenController prior to rendering. (Invoked once per
     * frame.)
     *
     * @param tpf time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void update(float tpf) {
        super.update(tpf);

        if (!hasStarted()) {
            return;
        }

        updateMarginButton();
        updateRagdollButton();
        updateViewButtons();

        // Update the 3-D scene.
        Model model = DacWizard.getModel();
        Spatial nextSpatial = model.getRootSpatial();
        String nextAnimationName = model.animationName();
        float nextAnimationTime = model.animationTime();
        if (nextSpatial != viewedSpatial
                || !nextAnimationName.equals(viewedAnimationName)
                || nextAnimationTime != viewedAnimationTime) {
            DacWizard wizard = DacWizard.getApplication();

            removeGroundPlane();
            wizard.clearScene();

            this.viewedSpatial = nextSpatial;
            this.viewedAnimationName = nextAnimationName;
            this.viewedAnimationTime = nextAnimationTime;

            if (nextSpatial != null) {
                Spatial cgModel = Heart.deepCopy(nextSpatial);
                wizard.makeScene(cgModel, nextAnimationName, nextAnimationTime);
                addGroundPlane();

                AbstractControl sControl = RagUtils.findSControl(cgModel);
                Spatial controlledSpatial = sControl.getSpatial();
                DynamicAnimControl dac = model.copyRagdoll();
                controlledSpatial.addControl(dac);

                BulletAppState bulletAppState
                        = DacWizard.findAppState(BulletAppState.class);
                PhysicsSpace physicsSpace = bulletAppState.getPhysicsSpace();
                dac.setPhysicsSpace(physicsSpace);
            }
        }

        updateAxes();
        updatePosingControls();
        updateSelectedLink();
    }
    // *************************************************************************
    // private methods

    /**
     * If there isn't a ground plane, create one and add it to the PhysicsSpace.
     */
    private void addGroundPlane() {
        if (groundPlane == null) {
            Plane xzPlane = new Plane(Vector3f.UNIT_Y, 0f);
            PlaneCollisionShape shape = new PlaneCollisionShape(xzPlane);
            this.groundPlane
                    = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);

            BulletAppState bulletAppState
                    = DacWizard.findAppState(BulletAppState.class);
            PhysicsSpace physicsSpace = bulletAppState.getPhysicsSpace();
            physicsSpace.addCollisionObject(groundPlane);
        }
    }

    /**
     * Apply the pivot-to-PhysicsSpace transform of the specified Constraint to
     * the specified Spatial.
     *
     * @param constraint the constraint to analyze (not null)
     * @param spatial where to apply the transform (not null)
     */
    private static void applyTransform(Constraint constraint, Spatial spatial) {
        Transform frame = new Transform(); // TODO garbage
        if (constraint instanceof New6Dof) {
            New6Dof new6dof = (New6Dof) constraint;
            new6dof.getFrameTransform(JointEnd.A, frame);
        } else {
            SixDofJoint sixDof = (SixDofJoint) constraint;
            sixDof.getFrameTransform(JointEnd.A, frame);
        }

        PhysicsRigidBody bodyA = constraint.getBodyA();
        Transform bodyTransform = bodyA.getTransform(null);
        bodyTransform.setScale(1f);
        frame.combineWithParent(bodyTransform);

        spatial.setLocalTransform(frame);
    }

    /**
     * Remove the ground plane (if any) from the PhysicsSpace.
     */
    private void removeGroundPlane() {
        if (groundPlane != null) {
            BulletAppState bulletAppState
                    = DacWizard.findAppState(BulletAppState.class);
            PhysicsSpace physicsSpace = bulletAppState.getPhysicsSpace();
            physicsSpace.removeCollisionObject(groundPlane);
            this.groundPlane = null;
        }
    }

    /**
     * Update the AxesVisualizer.
     */
    private void updateAxes() {
        DacWizard wizard = DacWizard.getApplication();
        AxesVisualizer axesVisualizer = wizard.findAxesVisualizer();

        Model model = DacWizard.getModel();
        boolean showingAxes = model.isShowingAxes();
        String btName = model.selectedLink();

        if (!showingAxes
                || viewedSpatial == null
                || btName.equals(DacConfiguration.torsoName)) {
            axesVisualizer.setEnabled(false);

        } else {
            // Align the visualizer axes with the PhysicsJoint.
            DynamicAnimControl dac = wizard.findDac();
            PhysicsLink selectedLink = dac.findBoneLink(btName);
            Constraint constraint = (Constraint) selectedLink.getJoint();
            Spatial axesNode = axesVisualizer.getSpatial();
            applyTransform(constraint, axesNode);

            axesVisualizer.setEnabled(true);
        }
    }

    /**
     * Update the collision-margin button.
     */
    private void updateMarginButton() {
        float margin = CollisionShape.getDefaultMargin();
        String marginButton = Float.toString(margin);
        setButtonText("margin", marginButton);
    }

    /**
     * Update the posing controls.
     */
    private void updatePosingControls() {
        String anText = "";
        String atText = "";
        String naText = "";
        String paText = "";

        if (viewedSpatial != null) {
            Model model = DacWizard.getModel();
            int numAnimations = model.countAnimations();
            if (numAnimations > 0) {
                paText = "-";
                naText = "+";
            }

            float duration = model.animationDuration();
            if (duration > 0f) {
                atText = Float.toString(viewedAnimationTime) + " seconds";
            }

            anText = viewedAnimationName;
            if (!anText.equals(DacWizard.bindPoseName)) {
                anText = MyString.quote(anText);
            }
        }

        setStatusText("animationName", anText);
        setButtonText("animationTime", atText);
        setButtonText("nextAnimation", naText);
        setButtonText("previousAnimation", paText);
    }

    /**
     * Update the "Go limp" button.
     */
    private void updateRagdollButton() {
        DacWizard wizard = DacWizard.getApplication();
        DynamicAnimControl dac = wizard.findDac();

        String ragdollButton = "";
        if (dac != null && dac.isReady()) {
            TorsoLink torso = dac.getTorsoLink();
            if (torso.isKinematic()) {
                ragdollButton = "Go limp";
            } else {
                ragdollButton = "Reset model";
            }
        }
        setButtonText("ragdoll", ragdollButton);
    }

    /**
     * Update the linkNameStatus and the custom materials of the DAC's bodies.
     */
    private void updateSelectedLink() {
        DacWizard wizard = DacWizard.getApplication();
        DynamicAnimControl dac = wizard.findDac();
        String linkName = "";
        if (dac != null) {
            Model model = DacWizard.getModel();
            String selectedBtName = model.selectedLink();

            if (selectedBtName.equals(DacConfiguration.torsoName)) {
                linkName = "Torso:";
            } else {
                linkName = "Bone:" + selectedBtName;
            }

            List<BoneLink> boneLinks = dac.listLinks(BoneLink.class);
            for (BoneLink link : boneLinks) {
                String boneName = link.boneName();
                PhysicsRigidBody body = link.getRigidBody();
                if (boneName.equals(selectedBtName)) {
                    body.setDebugMaterial(selectMaterial);
                } else {
                    body.setDebugMaterial(null);
                }
            }
            TorsoLink link = dac.getTorsoLink();
            PhysicsRigidBody body = link.getRigidBody();
            if (selectedBtName.equals(DacConfiguration.torsoName)) {
                body.setDebugMaterial(selectMaterial);
            } else {
                body.setDebugMaterial(null);
            }
        }
        setStatusText("linkNameStatus", linkName);
    }

    /**
     * Update the buttons that toggle view elements.
     */
    private void updateViewButtons() {
        BulletAppState bulletAppState
                = DacWizard.findAppState(BulletAppState.class);

        String debugButton;
        if (bulletAppState.isDebugEnabled()) {
            debugButton = "Hide physics";
        } else {
            debugButton = "Show physics";
        }
        setButtonText("debug", debugButton);

        Model model = DacWizard.getModel();
        String meshButton;
        if (model.isShowingMeshes()) {
            meshButton = "Hide meshes";
        } else {
            meshButton = "Show meshes";
        }
        setButtonText("mesh", meshButton);

        String skeletonText = "";
        DacWizard app = DacWizard.getApplication();
        SkeletonVisualizer sv = app.findSkeletonVisualizer();
        Spatial root = model.getRootSpatial();
        if (sv != null && root != null) {
            boolean isShown = model.isShowingSkeleton();
            sv.setEnabled(isShown);

            Skeleton skeleton = model.findSkeleton();
            String armature = (skeleton == null) ? "armature" : "skeleton";
            if (isShown) {
                skeletonText = "Hide " + armature;
            } else {
                skeletonText = "Show " + armature;
            }
        }
        setButtonText("skeleton", skeletonText);

        String axesText = model.isShowingAxes() ? "Hide axes" : "Show axes";
        setButtonText("axes", axesText);
    }
}
