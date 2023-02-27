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

import com.jme3.anim.AnimClip;
import com.jme3.anim.AnimComposer;
import com.jme3.anim.Armature;
import com.jme3.anim.Joint;
import com.jme3.anim.SkinningControl;
import com.jme3.animation.AnimControl;
import com.jme3.animation.Animation;
import com.jme3.animation.Bone;
import com.jme3.animation.Skeleton;
import com.jme3.animation.SkeletonControl;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.bullet.animation.BoneLink;
import com.jme3.bullet.animation.DynamicAnimControl;
import com.jme3.bullet.animation.RagUtils;
import com.jme3.bullet.animation.RangeOfMotion;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.joints.SixDofJoint;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.scene.Spatial;
import com.jme3.scene.control.AbstractControl;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MySpatial;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.noise.Generator;
import jme3utilities.wes.Pose;
import jme3utilities.wes.TweenTransforms;

/**
 * A Callable for asynchronously estimating the ranges of motion of the loaded
 * C-G model, based on a pseudo-random sample its animations.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class RomCallable implements Callable<RangeOfMotion[]>, PhysicsTickListener {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final static Logger logger
            = Logger.getLogger(RomCallable.class.getName());
    // *************************************************************************
    // fields

    /**
     * initial state of physics debug visualization
     */
    final private boolean wasDebugEnabled;
    /**
     * temporary DynamicAnimationControl for use with the temporary model
     */
    final private DynamicAnimControl tempDac;
    /**
     * pseudo-random generator for skeleton poses
     */
    final private Generator generator = new Generator();
    /**
     * information about the subject C-G model
     */
    final private Model model;
    /**
     * root of the temporary C-G model
     */
    final private Spatial tempModelRoot;
    /**
     * interpolation techniques used when calculating skeleton poses
     */
    final private TweenTransforms techniques = new TweenTransforms();
    /**
     * maximum angles seen for each rotational axis of each physics joint
     */
    final private Vector3f[] maxima;
    /**
     * minimum angles seen for each rotational axis of each physics joint
     */
    final private Vector3f[] minima;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a callable to analyze the specified model.
     *
     * @param subjectModel the model to analyze (not null)
     */
    RomCallable(Model subjectModel) {
        this.model = subjectModel;

        // Initialize accumulators for maximum and minimum rotation angles.
        int numBones = model.countBones();
        this.maxima = new Vector3f[numBones];
        this.minima = new Vector3f[numBones];
        for (int boneIndex = 0; boneIndex < numBones; ++boneIndex) {
            if (model.isBoneLinked(boneIndex)) {
                maxima[boneIndex] = new Vector3f(0f, 0f, 0f);
                minima[boneIndex] = new Vector3f(0f, 0f, 0f);
            }
        }

        // Temporarily enable physics-debug visualization.
        BulletAppState bulletAppState
                = DacWizard.findAppState(BulletAppState.class);
        this.wasDebugEnabled = bulletAppState.isDebugEnabled();
        if (!wasDebugEnabled) {
            bulletAppState.setDebugEnabled(true);
        }
        /*
         * Create a temporary copy of the C-G model and attach it
         * to the scene graph.
         */
        Spatial cgModel = model.getRootSpatial();

        DacWizard wizard = DacWizard.getApplication();
        wizard.clearScene();

        this.tempModelRoot = Heart.deepCopy(cgModel);
        String animationName = model.animationName();
        float animationTime = model.animationTime();
        wizard.makeScene(tempModelRoot, animationName, animationTime);
        /*
         * Add a DynamicAnimControl to the copy.  Since the control will
         * stay in kinematic mode, its masses and ranges of motion
         * don't matter.
         */
        float mass = 1f;
        RangeOfMotion stiffRom = new RangeOfMotion();
        AbstractControl sControl = RagUtils.findSControl(tempModelRoot);
        if (sControl instanceof SkinningControl) { // new animation system
            Armature armature = ((SkinningControl) sControl).getArmature();
            this.tempDac = new DynamicAnimControl() {
                @Override
                public void update(float tpf) {
                    applyRandomPose();
                    super.update(tpf);
                }
            };
            for (int jointIndex = 0; jointIndex < numBones; ++jointIndex) {
                if (model.isBoneLinked(jointIndex)) {
                    Joint joint = armature.getJoint(jointIndex);
                    String boneName = joint.getName();
                    tempDac.link(boneName, mass, stiffRom);
                }
            }

        } else { // old animation system
            Skeleton skeleton = ((SkeletonControl) sControl).getSkeleton();
            this.tempDac = new DynamicAnimControl() {
                @Override
                public void update(float tpf) {
                    applyRandomPose();
                    super.update(tpf);
                }
            };
            for (int boneIndex = 0; boneIndex < numBones; ++boneIndex) {
                if (model.isBoneLinked(boneIndex)) {
                    Bone bone = skeleton.getBone(boneIndex);
                    String boneName = bone.getName();
                    tempDac.link(boneName, mass, stiffRom);
                }
            }
        }

        int mbIndex = model.mainBoneIndex();
        String mbName = model.boneName(mbIndex);
        tempDac.setMainBoneName(mbName);

        Spatial controlledSpatial = sControl.getSpatial();
        controlledSpatial.addControl(tempDac);

        // Disable contact response for all rigid bodies in the ragdoll.
        PhysicsRigidBody[] bodies = tempDac.listRigidBodies();
        for (PhysicsRigidBody body : bodies) {
            body.setContactResponse(false);
        }

        // Add the ragdoll to physics space.
        PhysicsSpace physicsSpace = bulletAppState.getPhysicsSpace();
        assert physicsSpace.isEmpty();
        tempDac.setPhysicsSpace(physicsSpace);
        physicsSpace.addTickListener(this);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Clean up the scene graph and the physics.
     */
    void cleanup() {
        BulletAppState bulletAppState
                = DacWizard.findAppState(BulletAppState.class);
        PhysicsSpace physicsSpace = bulletAppState.getPhysicsSpace();

        tempModelRoot.removeFromParent();
        physicsSpace.removeTickListener(this);
        tempDac.setPhysicsSpace(null);
        assert physicsSpace.isEmpty();

        if (!wasDebugEnabled) {
            bulletAppState.setDebugEnabled(false);
        }
    }
    // *************************************************************************
    // Callable methods

    /**
     * Estimate the range of motion of each linked bone in the loaded C-G model.
     *
     * @return a new map from bone indices to ranges of motion
     */
    @Override
    public RangeOfMotion[] call() throws InterruptedException {
        // Accumulate joint-angle statistics for 10 seconds.
        Thread.sleep(10_000);

        // Convert the statistics into ranges of motion.
        int numBones = model.countBones();
        RangeOfMotion[] roms = new RangeOfMotion[numBones];
        for (int boneIndex = 0; boneIndex < numBones; ++boneIndex) {
            if (model.isBoneLinked(boneIndex)) {
                Vector3f max = maxima[boneIndex];
                Vector3f min = minima[boneIndex];
                roms[boneIndex] = new RangeOfMotion(
                        max.x, min.x, max.y, min.y, max.z, min.z);
            }
        }

        return roms;
    }
    // *************************************************************************
    // PhysicsTickListener methods

    /**
     * Callback from Bullet, invoked just after the simulation has been stepped.
     *
     * @param space the space that was just stepped (not null)
     * @param timeStep the time per simulation step (in seconds, &ge;0)
     */
    @Override
    public void physicsTick(PhysicsSpace space, float timeStep) {
        // Read joint angles from the ragdoll and update statistics.
        Vector3f angles = new Vector3f();
        int numBones = model.countBones();
        for (int boneIndex = 0; boneIndex < numBones; ++boneIndex) {
            String boneName = model.boneName(boneIndex);
            BoneLink link = tempDac.findBoneLink(boneName);
            if (link != null) {
                PhysicsJoint joint = link.getJoint();
                SixDofJoint sixDof = (SixDofJoint) joint;
                sixDof.getAngles(angles);
                assert angles.x >= -FastMath.PI : angles;
                assert angles.x <= FastMath.PI : angles;
                assert angles.y >= -FastMath.PI : angles;
                assert angles.y <= FastMath.PI : angles;
                assert angles.z >= -FastMath.PI : angles;
                assert angles.z <= FastMath.PI : angles;
                MyVector3f.accumulateMaxima(maxima[boneIndex], angles);
                MyVector3f.accumulateMinima(minima[boneIndex], angles);
            }
        }
    }

    /**
     * Callback from Bullet, invoked just before the simulation is stepped.
     *
     * @param space the space that's about to be stepped (not null)
     * @param timeStep the time per simulation step (in seconds, &ge;0)
     */
    @Override
    public void prePhysicsTick(PhysicsSpace space, float timeStep) {
        // do nothing
    }
    // *************************************************************************
    // private methods

    /**
     * Apply a pseudo-random Pose to the Armature or Skeleton of the temporary
     * C-G model.
     */
    @SuppressWarnings("unchecked")
    private void applyRandomPose() {
        // Choose an AnimComposer or AnimControl.
        List controls = MySpatial.listControls(
                tempModelRoot, AnimControl.class, null);
        MySpatial.listControls(tempModelRoot, AnimComposer.class, controls);
        AbstractControl control = (AbstractControl) generator.pick(controls);

        if (control instanceof AnimControl) {
            AnimControl animControl = (AnimControl) control;

            // Choose an Animation.
            Collection<String> nameCollection = animControl.getAnimationNames();
            int numAnimations = nameCollection.size();
            String[] nameArray = new String[numAnimations];
            nameCollection.toArray(nameArray);
            String animationName = generator.pick(nameArray);
            if (animationName == null) {
                return;
            }
            Animation animation = animControl.getAnim(animationName);

            // Choose an animation time.
            float duration = animation.getLength();
            float animationTime = generator.nextFloat(0f, duration);

            Skeleton skeleton = tempDac.getSkeleton();
            Pose pose = new Pose(skeleton);
            pose.setToAnimation(animation, animationTime, techniques);
            pose.applyTo(skeleton);

        } else if (control instanceof AnimComposer) {
            AnimComposer composer = (AnimComposer) control;

            // Choose an AnimClip.
            Collection<AnimClip> clips = composer.getAnimClips();
            int numClips = clips.size();
            AnimClip[] clipArray = new AnimClip[numClips];
            clips.toArray(clipArray);
            AnimClip clip = generator.pick(clipArray);
            if (clip == null) {
                return;
            }

            // Choose an animation time.
            double duration = clip.getLength();
            double animationTime = duration * generator.nextDouble();

            Armature armature = tempDac.getArmature();
            Pose pose = new Pose(armature);
            pose.setToClip(clip, animationTime);
            pose.applyTo(armature);
        }
    }
}
