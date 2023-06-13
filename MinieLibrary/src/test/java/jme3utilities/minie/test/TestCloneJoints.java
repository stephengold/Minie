/*
 Copyright (c) 2018-2023, Stephen Gold
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

import com.jme3.asset.AssetManager;
import com.jme3.asset.DesktopAssetManager;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.RotationOrder;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.joints.ConeJoint;
import com.jme3.bullet.joints.Constraint;
import com.jme3.bullet.joints.GearJoint;
import com.jme3.bullet.joints.HingeJoint;
import com.jme3.bullet.joints.JointEnd;
import com.jme3.bullet.joints.New6Dof;
import com.jme3.bullet.joints.NewHinge;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.joints.Point2PointJoint;
import com.jme3.bullet.joints.SixDofJoint;
import com.jme3.bullet.joints.SixDofSpringJoint;
import com.jme3.bullet.joints.SliderJoint;
import com.jme3.bullet.joints.SoftAngularJoint;
import com.jme3.bullet.joints.SoftLinearJoint;
import com.jme3.bullet.joints.motors.MotorParam;
import com.jme3.bullet.joints.motors.RotationMotor;
import com.jme3.bullet.joints.motors.RotationalLimitMotor;
import com.jme3.bullet.joints.motors.TranslationMotor;
import com.jme3.bullet.joints.motors.TranslationalLimitMotor;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.bullet.util.NativeSoftBodyUtil;
import com.jme3.export.binary.BinaryExporter;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.debug.WireBox;
import com.jme3.system.NativeLibraryLoader;
import jme3utilities.Heart;
import org.junit.Assert;
import org.junit.Test;

/**
 * Test cloning/saving/loading physics joints of all types.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestCloneJoints {
    // *************************************************************************
    // constants and loggers

    final private static Quaternion qa = new Quaternion(1f, 0f, 0f, 0f);
    final private static Quaternion qb = new Quaternion(0.5f, 0.5f, 0.5f, 0.5f);
    final private static Vector3f va  = new Vector3f(-1f, -2f, -3f);
    final private static Vector3f vaNorm = va.normalize();
    final private static Vector3f vb = new Vector3f(-4f, -5f, -6f);
    final private static Vector3f vbNorm = vb.normalize();
    // *************************************************************************
    // fields

    /**
     * AssetManager required by the BinaryImporter
     */
    final private static AssetManager assetManager = new DesktopAssetManager();

    private static PhysicsRigidBody rigidA;
    private static PhysicsRigidBody rigidB;
    private static PhysicsSoftBody softA;
    private static PhysicsSoftBody softB;
    // *************************************************************************
    // new methods exposed

    /**
     * Test cloning/saving/loading physics joints of all types.
     */
    @Test
    public void testCloneJoints() {
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);

        for (int iteration = 0; iteration < 99; ++iteration) {
            newBodies();

            cloneCone();
            cloneGear();
            cloneHinge();
            cloneNew6Dof();
            cloneP2P();
            cloneSix();
            cloneSlide();
            cloneSoftAngular();
            cloneSoftLinear();
            cloneSpring();
        }
    }
    // *************************************************************************
    // private methods

    /**
     * ConeJoint: single- and double-ended
     */
    private static void cloneCone() {
        ConeJoint cone = new ConeJoint(rigidA, rigidB, va, vb,
                qa.toRotationMatrix(), qb.toRotationMatrix());
        setParameters(cone, 0f);
        verifyParameters(cone, 0f);
        ConeJoint coneClone = Heart.deepCopy(cone);
        cloneTest(cone, coneClone);

        ConeJoint seCone = new ConeJoint(rigidA, va, qa.toRotationMatrix());
        setParameters(seCone, 0f);
        verifyParameters(seCone, 0f);
        ConeJoint seConeClone = Heart.deepCopy(seCone);
        cloneTest(seCone, seConeClone);
    }

    /**
     * GearJoint
     */
    private static void cloneGear() {
        GearJoint gear = new GearJoint(rigidA, rigidB, va, vb);
        setParameters(gear, 0f);
        verifyParameters(gear, 0f);
        GearJoint gearClone = Heart.deepCopy(gear);
        cloneTest(gear, gearClone);
    }

    /**
     * HingeJoint: single- and double-ended
     */
    private static void cloneHinge() {
        HingeJoint hinge = new HingeJoint(rigidA, rigidB, va, vb,
                new Vector3f(1f, 0f, 0f), new Vector3f(1f, 0f, 0f));
        setParameters(hinge, 0f);
        verifyParameters(hinge, 0f);
        HingeJoint hingeClone = Heart.deepCopy(hinge);
        cloneTest(hinge, hingeClone);

        HingeJoint seHinge = new HingeJoint(rigidA, va, vb,
                new Vector3f(1f, 0f, 0f), new Vector3f(1f, 0f, 0f), JointEnd.A);
        setParameters(seHinge, 0f);
        verifyParameters(seHinge, 0f);
        HingeJoint seHingeClone = Heart.deepCopy(seHinge);
        cloneTest(seHinge, seHingeClone);
    }

    /**
     * New6Dof: single- and double-ended and NewHinge
     */
    private static void cloneNew6Dof() {
        New6Dof deNew6 = new New6Dof(
                rigidA, rigidB, va, vb, qa.toRotationMatrix(),
                qb.toRotationMatrix(), RotationOrder.XYZ);
        setParameters(deNew6, 0f);
        verifyParameters(deNew6, 0f);
        New6Dof deNew6Clone = Heart.deepCopy(deNew6);
        cloneTest(deNew6, deNew6Clone);

        New6Dof seNew6 = new New6Dof(rigidA, vb, va, qb.toRotationMatrix(),
                qa.toRotationMatrix(), RotationOrder.XYZ);
        setParameters(seNew6, 0f);
        verifyParameters(seNew6, 0f);
        New6Dof seNew6Clone = Heart.deepCopy(seNew6);
        cloneTest(seNew6, seNew6Clone);

        NewHinge newH = new NewHinge(rigidA, rigidB, Vector3f.ZERO,
                Vector3f.UNIT_Y, Vector3f.UNIT_Z);
        setParameters(newH, 0f);
        verifyParameters(newH, 0f);
        NewHinge newHClone = Heart.deepCopy(newH);
        cloneTest(newH, newHClone);
    }

    /**
     * Point2PointJoint: single- and double-ended
     */
    private static void cloneP2P() {
        Point2PointJoint p2p = new Point2PointJoint(rigidA, rigidB, va, vb);
        setParameters(p2p, 0f);
        verifyParameters(p2p, 0f);
        Point2PointJoint p2pClone = Heart.deepCopy(p2p);
        cloneTest(p2p, p2pClone);

        Point2PointJoint sep2p
                = new Point2PointJoint(rigidA, va, vb);
        setParameters(sep2p, 0f);
        verifyParameters(sep2p, 0f);
        Point2PointJoint sep2pClone = Heart.deepCopy(sep2p);
        cloneTest(sep2p, sep2pClone);
    }

    /**
     * SixDofJoint: single- and double-ended
     */
    private static void cloneSix() {
        SixDofJoint six = new SixDofJoint(rigidA, rigidB, va, vb,
                qa.toRotationMatrix(), qb.toRotationMatrix(), false);
        setParameters(six, 0f);
        verifyParameters(six, 0f);
        SixDofJoint sixClone = Heart.deepCopy(six);
        cloneTest(six, sixClone);

        SixDofJoint seSix = new SixDofJoint(rigidA, vb, va,
                qb.toRotationMatrix(), qa.toRotationMatrix(), JointEnd.A);
        setParameters(seSix, 0f);
        verifyParameters(seSix, 0f);
        SixDofJoint seSixClone = Heart.deepCopy(seSix);
        cloneTest(seSix, seSixClone);
    }

    /**
     * SliderJoint: single- and double-ended
     */
    private static void cloneSlide() {
        SliderJoint slide = new SliderJoint(rigidA, rigidB, va, vb,
                qa.toRotationMatrix(), qb.toRotationMatrix(), false);
        setParameters(slide, 0f);
        verifyParameters(slide, 0f);
        SliderJoint slideClone = Heart.deepCopy(slide);
        cloneTest(slide, slideClone);

        SliderJoint seSlide = new SliderJoint(rigidB, vb, va, JointEnd.B);
        setParameters(seSlide, 0f);
        verifyParameters(seSlide, 0f);
        SliderJoint seSlideClone = Heart.deepCopy(seSlide);
        cloneTest(seSlide, seSlideClone);
    }

    /**
     * SoftAngularJoint: soft-soft and soft-rigid
     */
    private static void cloneSoftAngular() {
        SoftAngularJoint softSoft
                = new SoftAngularJoint(va, softA, 0, softB, 0);
        setParameters(softSoft, 0f);
        verifyParameters(softSoft, 0f);
        SoftAngularJoint ssClone = Heart.deepCopy(softSoft);
        cloneTest(softSoft, ssClone);

        SoftAngularJoint softRigid = new SoftAngularJoint(va, softA, 0, rigidB);
        setParameters(softRigid, 0f);
        verifyParameters(softRigid, 0f);
        SoftAngularJoint srClone = Heart.deepCopy(softRigid);
        cloneTest(softRigid, srClone);
    }

    /**
     * Clone SoftLinearJoint: soft-soft and soft-rigid
     */
    private static void cloneSoftLinear() {
        SoftLinearJoint softSoft = new SoftLinearJoint(va, softA, 0, softB, 0);
        setParameters(softSoft, 0f);
        verifyParameters(softSoft, 0f);
        SoftLinearJoint ssClone = Heart.deepCopy(softSoft);
        cloneTest(softSoft, ssClone);

        SoftLinearJoint softRigid = new SoftLinearJoint(va, softA, 0, rigidB);
        setParameters(softRigid, 0f);
        verifyParameters(softRigid, 0f);
        SoftLinearJoint srClone = Heart.deepCopy(softRigid);
        cloneTest(softRigid, srClone);
    }

    /**
     * SixDofSpringJoint: single- and double-ended
     */
    private static void cloneSpring() {
        SixDofSpringJoint spring = new SixDofSpringJoint(rigidA, rigidB, va, vb,
                qa.toRotationMatrix(), qb.toRotationMatrix(), false);
        setParameters(spring, 0f);
        verifyParameters(spring, 0f);
        SixDofSpringJoint springClone = Heart.deepCopy(spring);
        cloneTest(spring, springClone);

        SixDofSpringJoint seSpring = new SixDofSpringJoint(rigidA, vb, va,
                qb.toRotationMatrix(), qa.toRotationMatrix(), JointEnd.A);
        setParameters(seSpring, 0f);
        verifyParameters(seSpring, 0f);
        SixDofSpringJoint seSpringClone = Heart.deepCopy(seSpring);
        cloneTest(seSpring, seSpringClone);
    }

    private static void cloneTest(PhysicsJoint joint, PhysicsJoint jointClone) {
        Assert.assertNotSame(joint, jointClone);
        Assert.assertNotEquals(joint.nativeId(), jointClone.nativeId());

        PhysicsBody a = joint.getBody(JointEnd.A);
        PhysicsBody aClone = jointClone.getBody(JointEnd.A);
        if (a == null) {
            Assert.assertNull(aClone);
        } else {
            Assert.assertNotNull(aClone);
            Assert.assertNotEquals(a.nativeId(), aClone.nativeId());
        }

        PhysicsBody b = joint.getBody(JointEnd.B);
        PhysicsBody bClone = jointClone.getBody(JointEnd.B);
        if (b == null) {
            Assert.assertNull(bClone);
        } else {
            Assert.assertNotNull(bClone);
            Assert.assertNotEquals(b.nativeId(), bClone.nativeId());
        }

        verifyParameters(joint, 0f);
        verifyParameters(jointClone, 0f);

        setParameters(joint, 0.3f);
        verifyParameters(joint, 0.3f);
        verifyParameters(jointClone, 0f);

        setParameters(jointClone, 0.6f);
        verifyParameters(joint, 0.3f);
        verifyParameters(jointClone, 0.6f);

        PhysicsJoint jointCopy
                = BinaryExporter.saveAndLoad(assetManager, joint);
        verifyParameters(jointCopy, 0.3f);

        PhysicsJoint jointCloneCopy
                = BinaryExporter.saveAndLoad(assetManager, jointClone);
        verifyParameters(jointCloneCopy, 0.6f);
    }

    private static void newBodies() {
        float mass = 1f;
        CollisionShape box = new BoxCollisionShape(1f);
        rigidA = new PhysicsRigidBody(box, mass);
        rigidB = new PhysicsRigidBody(box, mass);

        Mesh wireBox = new WireBox();
        softA = new PhysicsSoftBody();
        NativeSoftBodyUtil.appendFromLineMesh(wireBox, softA);
        softA.generateClusters(2, 999);
        softA.setMargin(0.1f);
        softA.setMass(mass);

        softB = new PhysicsSoftBody();
        NativeSoftBodyUtil.appendFromLineMesh(wireBox, softB);
        softB.generateClusters(2, 999);
        softB.setMargin(0.1f);
        softB.setMass(mass);
    }

    /**
     * Modify joint parameters based on the specified key value.
     *
     * @param joint the joint to modify (not null)
     * @param b the key value
     */
    private static void setParameters(PhysicsJoint joint, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);
        int index = Math.round(b / 0.3f);

        if (joint instanceof Constraint) {
            Constraint constraint = (Constraint) joint;
            constraint.setEnabled(flag);
            constraint.setFeedback(flag);
            constraint.setBreakingImpulseThreshold(b + 0.505f);
            constraint.overrideIterations(index);
            if (joint.countEnds() == 2) {
                constraint.setCollisionBetweenLinkedBodies(!flag);
            }
        }

        if (joint instanceof ConeJoint) {
            setCone((ConeJoint) joint, b);
        } else if (joint instanceof GearJoint) {
            setGear((GearJoint) joint, b);
        } else if (joint instanceof HingeJoint) {
            setHinge((HingeJoint) joint, b);
        } else if (joint instanceof New6Dof) {
            setNew6Dof((New6Dof) joint, b);
        } else if (joint instanceof Point2PointJoint) {
            setP2P((Point2PointJoint) joint, b);
        } else if (joint instanceof SixDofJoint) {
            setSix((SixDofJoint) joint, b);
        } else if (joint instanceof SixDofSpringJoint) {
            setSpring((SixDofSpringJoint) joint, b);
        } else if (joint instanceof SliderJoint) {
            setSlide((SliderJoint) joint, b);
        } else if (joint instanceof SoftAngularJoint) {
            setSoftAngular((SoftAngularJoint) joint, b);
        } else if (joint instanceof SoftLinearJoint) {
            setSoftLinear((SoftLinearJoint) joint, b);
        } else {
            throw new IllegalArgumentException(joint.getClass().getName());
        }
    }

    private static void setCone(ConeJoint cone, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);
        cone.setAngularOnly(!flag);

        cone.setLimit(b + 0.01f, b + 0.02f, b + 0.03f);
    }

    private static void setGear(GearJoint gear, float b) {
        gear.setRatio(b + 0.01f);
    }

    private static void setHinge(HingeJoint hinge, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);
        hinge.setAngularOnly(!flag);

        hinge.enableMotor(flag, b + 0.01f, b + 0.02f);
        hinge.setLimit(b + 0.03f, b + 0.04f, b + 0.05f, b + 0.06f, b + 0.07f);
    }

    private static void setNew6Dof(New6Dof constraint, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);

        RotationMotor rMotor
                = constraint.getRotationMotor(PhysicsSpace.AXIS_Z);
        rMotor.setDampingLimited(flag);
        rMotor.setMotorEnabled(flag);
        rMotor.setServoEnabled(!flag);
        rMotor.setSpringEnabled(flag);
        rMotor.setStiffnessLimited(!flag);

        TranslationMotor tMotor = constraint.getTranslationMotor();
        tMotor.setDampingLimited(PhysicsSpace.AXIS_Z, flag);
        tMotor.setMotorEnabled(PhysicsSpace.AXIS_Y, flag);
        tMotor.setServoEnabled(PhysicsSpace.AXIS_X, !flag);
        tMotor.setSpringEnabled(PhysicsSpace.AXIS_Y, !flag);
        tMotor.setStiffnessLimited(PhysicsSpace.AXIS_Z, !flag);

        for (MotorParam parameter : MotorParam.values()) {
            int parameterIndex = parameter.ordinal();
            for (int dofIndex = 0; dofIndex < 6; ++dofIndex) {
                float value = b + dofIndex * 0.001f + parameterIndex * 0.007f;
                constraint.set(parameter, dofIndex, value);
            }
        }
    }

    private static void setP2P(Point2PointJoint p2p, float b) {
        p2p.setDamping(b + 0.01f);
        p2p.setImpulseClamp(b + 0.02f);
        p2p.setTau(b + 0.03f);
    }

    private static void setSix(SixDofJoint six, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);

        RotationalLimitMotor rot
                = six.getRotationalLimitMotor(PhysicsSpace.AXIS_Z);
        rot.setEnableMotor(!flag);
        rot.setAccumulatedImpulse(b + 0.003f);
        rot.setRestitution(b + 0.01f);
        rot.setDamping(b + 0.02f);
        rot.setERP(b + 0.03f);
        rot.setLowerLimit(b + 0.04f);
        rot.setUpperLimit(b + 0.05f);
        rot.setLimitSoftness(b + 0.06f);
        rot.setMaxLimitForce(b + 0.07f);
        rot.setMaxMotorForce(b + 0.08f);
        rot.setTargetVelocity(b + 0.09f);
        rot.setNormalCFM(b + 0.091f);
        rot.setStopCFM(b + 0.092f);

        TranslationalLimitMotor tra = six.getTranslationalLimitMotor();
        tra.setEnabled(0, flag);
        tra.setEnabled(1, flag);
        tra.setEnabled(2, !flag);
        tra.setAccumulatedImpulse(
                new Vector3f(b + 0.101f, b + 0.102f, b + 0.103f));
        tra.setDamping(b + 0.10f);
        tra.setLimitSoftness(b + 0.11f);
        tra.setLowerLimit(new Vector3f(b + 0.12f, b + 0.13f, b + 0.14f));
        tra.setRestitution(b + 0.15f);
        tra.setUpperLimit(new Vector3f(b + 0.16f, b + 0.17f, b + 0.18f));
        tra.setERP(new Vector3f(0f, 0f, b + 0.19f));
        tra.setMaxMotorForce(new Vector3f(0f, 0f, b + 0.20f));
        tra.setNormalCFM(new Vector3f(0f, 0f, b + 0.22f));
        tra.setStopCFM(new Vector3f(0f, 0f, b + 0.23f));
        tra.setTargetVelocity(new Vector3f(0f, 0f, b + 0.24f));
    }

    private static void setSlide(SliderJoint slide, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);

        slide.setDampingDirAng(b + 0.01f);
        slide.setDampingDirLin(b + 0.02f);
        slide.setDampingLimAng(b + 0.03f);
        slide.setDampingLimLin(b + 0.04f);
        slide.setDampingOrthoAng(b + 0.05f);
        slide.setDampingOrthoLin(b + 0.06f);

        slide.setLowerAngLimit(b + 0.07f);
        slide.setLowerLinLimit(b + 0.08f);

        slide.setMaxAngMotorForce(b + 0.09f);
        slide.setMaxLinMotorForce(b + 0.10f);

        slide.setPoweredAngMotor(!flag);
        slide.setPoweredLinMotor(flag);

        slide.setRestitutionDirAng(b + 0.11f);
        slide.setRestitutionDirLin(b + 0.12f);
        slide.setRestitutionLimAng(b + 0.13f);
        slide.setRestitutionLimLin(b + 0.14f);
        slide.setRestitutionOrthoAng(b + 0.15f);
        slide.setRestitutionOrthoLin(b + 0.16f);

        slide.setSoftnessDirAng(b + 0.17f);
        slide.setSoftnessDirLin(b + 0.18f);
        slide.setSoftnessLimAng(b + 0.19f);
        slide.setSoftnessLimLin(b + 0.20f);
        slide.setSoftnessOrthoAng(b + 0.21f);
        slide.setSoftnessOrthoLin(b + 0.22f);

        slide.setTargetAngMotorVelocity(b + 0.23f);
        slide.setTargetLinMotorVelocity(b + 0.24f);

        slide.setUpperAngLimit(b + 0.25f);
        slide.setUpperLinLimit(b + 0.26f);
    }

    private static void setSoftAngular(SoftAngularJoint saj, float b) {
        saj.setCFM(b + 0.01f);
        saj.setERP(b + 0.02f);
        saj.setSplit(b + 0.03f);
    }

    private static void setSoftLinear(SoftLinearJoint slj, float b) {
        slj.setCFM(b + 0.02f);
        slj.setERP(b + 0.03f);
        slj.setSplit(b + 0.04f);
    }

    private static void setSpring(SixDofSpringJoint spring, float b) {
        setSix(spring, b);

        spring.setDamping(0, b + 0.251f);
        spring.setDamping(1, b + 0.252f);
        spring.setDamping(2, b + 0.253f);
        spring.setDamping(3, b + 0.254f);
        spring.setDamping(4, b + 0.255f);
        spring.setDamping(5, b + 0.256f);

        spring.setStiffness(0, b + 0.261f);
        spring.setStiffness(1, b + 0.262f);
        spring.setStiffness(2, b + 0.263f);
        spring.setStiffness(3, b + 0.264f);
        spring.setStiffness(4, b + 0.265f);
        spring.setStiffness(5, b + 0.266f);
    }

    /**
     * Verify that all joint parameters have their expected values for the
     * specified key value.
     *
     * @param joint the joint to verify (not null, unaffected)
     * @param b the key value
     */
    private static void verifyParameters(PhysicsJoint joint, float b) {
        Assert.assertNotNull(joint);
        boolean flag = (b > 0.15f && b < 0.45f);
        int index = Math.round(b / 0.3f);

        if (joint instanceof Constraint) {
            Constraint constraint = (Constraint) joint;
            Assert.assertEquals(flag, constraint.isEnabled());
            Assert.assertEquals(flag, constraint.isFeedback());
            Assert.assertEquals(
                    b + 0.505f, constraint.getBreakingImpulseThreshold(), 0f);
            Assert.assertEquals(index, constraint.getOverrideIterations());
            if (joint.countEnds() == 2) {
                Assert.assertEquals(
                        !flag, constraint.isCollisionBetweenLinkedBodies());
            }
        }

        if (joint instanceof ConeJoint) {
            verifyCone((ConeJoint) joint, b);
        } else if (joint instanceof GearJoint) {
            verifyGear((GearJoint) joint, b);
        } else if (joint instanceof HingeJoint) {
            verifyHinge((HingeJoint) joint, b);
        } else if (joint instanceof New6Dof) {
            verifyNew6Dof((New6Dof) joint, b);
        } else if (joint instanceof Point2PointJoint) {
            verifyP2P((Point2PointJoint) joint, b);
        } else if (joint instanceof SixDofJoint) {
            verifySix((SixDofJoint) joint, b);
        } else if (joint instanceof SixDofSpringJoint) {
            verifySpring((SixDofSpringJoint) joint, b);
        } else if (joint instanceof SliderJoint) {
            verifySlide((SliderJoint) joint, b);
        } else if (joint instanceof SoftAngularJoint) {
            verifySoftAngular((SoftAngularJoint) joint, b);
        } else if (joint instanceof SoftLinearJoint) {
            verifySoftLinear((SoftLinearJoint) joint, b);
        } else {
            throw new IllegalArgumentException(joint.getClass().getName());
        }
    }

    private static void verifyCone(ConeJoint cone, float b) {
        Transform ta = cone.getFrameTransform(JointEnd.A, null);
        Utils.assertEquals(va, ta.getTranslation(), 0f);
        Assert.assertEquals(qa, ta.getRotation());

        if (cone.countEnds() == 2) {
            Transform tb = cone.getFrameTransform(JointEnd.B, null);
            Utils.assertEquals(vb, tb.getTranslation(), 0f);
            Assert.assertEquals(qb, tb.getRotation());
        }

        boolean flag = (b > 0.15f && b < 0.45f);
        Assert.assertEquals(!flag, cone.isAngularOnly());

        Assert.assertEquals(b + 0.01f, cone.getSwingSpan1(), 0f);
        Assert.assertEquals(b + 0.02f, cone.getSwingSpan2(), 0f);
        Assert.assertEquals(b + 0.03f, cone.getTwistSpan(), 0f);
    }

    private static void verifyGear(GearJoint gear, float b) {
        Vector3f axisA = gear.getAxisA(null);
        Utils.assertEquals(vaNorm, axisA, 0f);

        Vector3f axisB = gear.getAxisB(null);
        Utils.assertEquals(vbNorm, axisB, 0f);

        Assert.assertEquals(b + 0.01f, gear.getRatio(), 0f);
    }

    private static void verifyHinge(HingeJoint hinge, float b) {
        Transform ta = hinge.getFrameTransform(JointEnd.A, null);
        Utils.assertEquals(va, ta.getTranslation(), 0f);

        Transform tb = hinge.getFrameTransform(JointEnd.B, null);
        Utils.assertEquals(vb, tb.getTranslation(), 0f);

        boolean flag = (b > 0.15f && b < 0.45f);
        Assert.assertEquals(!flag, hinge.isAngularOnly());

        Assert.assertEquals(b + 0.01f, hinge.getMotorTargetVelocity(), 1e-6f);
        Assert.assertEquals(b + 0.02f, hinge.getMaxMotorImpulse(), 1e-6f);
        Assert.assertEquals(b + 0.03f, hinge.getLowerLimit(), 1e-6f);
        Assert.assertEquals(b + 0.04f, hinge.getUpperLimit(), 1e-6f);
        Assert.assertEquals(b + 0.05f, hinge.getLimitSoftness(), 1e-6f);
        Assert.assertEquals(b + 0.06f, hinge.getBiasFactor(), 1e-6f);
        Assert.assertEquals(b + 0.07f, hinge.getRelaxationFactor(), 1e-6f);
    }

    private static void verifyNew6Dof(New6Dof constraint, float b) {
        if (!(constraint instanceof NewHinge)) {
            Transform ta = constraint.getFrameTransform(JointEnd.A, null);
            Utils.assertEquals(va, ta.getTranslation(), 0f);
            Assert.assertEquals(qa, ta.getRotation());

            Transform tb = constraint.getFrameTransform(JointEnd.B, null);
            Utils.assertEquals(vb, tb.getTranslation(), 0f);
            Assert.assertEquals(qb, tb.getRotation());
        }

        boolean flag = (b > 0.15f && b < 0.45f);

        RotationMotor rMotor
                = constraint.getRotationMotor(PhysicsSpace.AXIS_Z);
        Assert.assertEquals(flag, rMotor.isDampingLimited());
        Assert.assertEquals(flag, rMotor.isMotorEnabled());
        Assert.assertEquals(!flag, rMotor.isServoEnabled());
        Assert.assertEquals(flag, rMotor.isSpringEnabled());
        Assert.assertEquals(!flag, rMotor.isStiffnessLimited());

        TranslationMotor tMotor = constraint.getTranslationMotor();
        Assert.assertEquals(flag, tMotor.isDampingLimited(PhysicsSpace.AXIS_Z));
        Assert.assertEquals(flag, tMotor.isMotorEnabled(PhysicsSpace.AXIS_Y));
        Assert.assertEquals(!flag, tMotor.isServoEnabled(PhysicsSpace.AXIS_X));
        Assert.assertEquals(!flag, tMotor.isSpringEnabled(PhysicsSpace.AXIS_Y));
        Assert.assertEquals(
                !flag, tMotor.isStiffnessLimited(PhysicsSpace.AXIS_Z));

        for (MotorParam parameter : MotorParam.values()) {
            int parameterIndex = parameter.ordinal();
            for (int dofIndex = 0; dofIndex < 6; ++dofIndex) {
                float value = b + dofIndex * 0.001f + parameterIndex * 0.007f;
                Assert.assertEquals(
                        value, constraint.get(parameter, dofIndex), 0f);
            }
        }
    }

    private static void verifyP2P(Point2PointJoint p2p, float b) {
        Assert.assertEquals(b + 0.01f, p2p.getDamping(), 0f);
        Assert.assertEquals(b + 0.02f, p2p.getImpulseClamp(), 0f);
        Assert.assertEquals(b + 0.03f, p2p.getTau(), 0f);
    }

    private static void verifySix(SixDofJoint six, float b) {
        Transform ta = six.getFrameTransform(JointEnd.A, null);
        Utils.assertEquals(va, ta.getTranslation(), 0f);
        Assert.assertEquals(qa, ta.getRotation());

        Transform tb = six.getFrameTransform(JointEnd.B, null);
        Utils.assertEquals(vb, tb.getTranslation(), 0f);
        Assert.assertEquals(qb, tb.getRotation());

        boolean flag = (b > 0.15f && b < 0.45f);

        RotationalLimitMotor rot
                = six.getRotationalLimitMotor(PhysicsSpace.AXIS_Z);
        Assert.assertEquals(!flag, rot.isEnableMotor());
        Assert.assertEquals(b + 0.003f, rot.getAccumulatedImpulse(), 0f);
        Assert.assertEquals(b + 0.01f, rot.getRestitution(), 0f);
        Assert.assertEquals(b + 0.02f, rot.getDamping(), 0f);
        Assert.assertEquals(b + 0.03f, rot.getERP(), 0f);
        Assert.assertEquals(b + 0.04f, rot.getLowerLimit(), 0f);
        Assert.assertEquals(b + 0.05f, rot.getUpperLimit(), 0f);
        Assert.assertEquals(b + 0.06f, rot.getLimitSoftness(), 0f);
        Assert.assertEquals(b + 0.07f, rot.getMaxLimitForce(), 0f);
        Assert.assertEquals(b + 0.08f, rot.getMaxMotorForce(), 0f);
        Assert.assertEquals(b + 0.09f, rot.getTargetVelocity(), 0f);
        Assert.assertEquals(b + 0.091f, rot.getNormalCFM(), 0f);
        Assert.assertEquals(b + 0.092f, rot.getStopCFM(), 0f);

        TranslationalLimitMotor tra = six.getTranslationalLimitMotor();
        Assert.assertEquals(flag, tra.isEnabled(0));
        Assert.assertEquals(flag, tra.isEnabled(1));
        Assert.assertEquals(!flag, tra.isEnabled(2));
        Utils.assertEquals(b + 0.101f, b + 0.102f, b + 0.103f,
                tra.getAccumulatedImpulse(null), 0f);
        Assert.assertEquals(b + 0.10f, tra.getDamping(), 0f);
        Assert.assertEquals(b + 0.11f, tra.getLimitSoftness(), 0f);
        Utils.assertEquals(
                b + 0.12f, b + 0.13f, b + 0.14f, tra.getLowerLimit(null), 0f);
        Assert.assertEquals(b + 0.15f, tra.getRestitution(), 0f);
        Utils.assertEquals(
                b + 0.16f, b + 0.17f, b + 0.18f, tra.getUpperLimit(null), 0f);
        Assert.assertEquals(b + 0.19f, tra.getERP(null).z, 0f);
        Assert.assertEquals(b + 0.20f, tra.getMaxMotorForce(null).z, 0f);
        Assert.assertEquals(b + 0.22f, tra.getNormalCFM(null).z, 0f);
        Assert.assertEquals(b + 0.23f, tra.getStopCFM(null).z, 0f);
        Assert.assertEquals(b + 0.24f, tra.getTargetVelocity(null).z, 0f);
    }

    private static void verifySlide(SliderJoint slide, float b) {
        Transform ta = slide.getFrameTransform(JointEnd.A, null);
        Utils.assertEquals(va, ta.getTranslation(), 0f);

        Transform tb = slide.getFrameTransform(JointEnd.B, null);
        Utils.assertEquals(vb, tb.getTranslation(), 0f);

        boolean flag = (b > 0.15f && b < 0.45f);

        Assert.assertEquals(b + 0.01f, slide.getDampingDirAng(), 0f);
        Assert.assertEquals(b + 0.02f, slide.getDampingDirLin(), 0f);
        Assert.assertEquals(b + 0.03f, slide.getDampingLimAng(), 0f);
        Assert.assertEquals(b + 0.04f, slide.getDampingLimLin(), 0f);
        Assert.assertEquals(b + 0.05f, slide.getDampingOrthoAng(), 0f);
        Assert.assertEquals(b + 0.06f, slide.getDampingOrthoLin(), 0f);

        Assert.assertEquals(b + 0.07f, slide.getLowerAngLimit(), 0f);
        Assert.assertEquals(b + 0.08f, slide.getLowerLinLimit(), 0f);

        Assert.assertEquals(b + 0.09f, slide.getMaxAngMotorForce(), 0f);
        Assert.assertEquals(b + 0.10f, slide.getMaxLinMotorForce(), 0f);

        Assert.assertEquals(!flag, slide.isPoweredAngMotor());
        Assert.assertEquals(flag, slide.isPoweredLinMotor());

        Assert.assertEquals(b + 0.11f, slide.getRestitutionDirAng(), 0f);
        Assert.assertEquals(b + 0.12f, slide.getRestitutionDirLin(), 0f);
        Assert.assertEquals(b + 0.13f, slide.getRestitutionLimAng(), 0f);
        Assert.assertEquals(b + 0.14f, slide.getRestitutionLimLin(), 0f);
        Assert.assertEquals(b + 0.15f, slide.getRestitutionOrthoAng(), 0f);
        Assert.assertEquals(b + 0.16f, slide.getRestitutionOrthoLin(), 0f);

        Assert.assertEquals(b + 0.17f, slide.getSoftnessDirAng(), 0f);
        Assert.assertEquals(b + 0.18f, slide.getSoftnessDirLin(), 0f);
        Assert.assertEquals(b + 0.19f, slide.getSoftnessLimAng(), 0f);
        Assert.assertEquals(b + 0.20f, slide.getSoftnessLimLin(), 0f);
        Assert.assertEquals(b + 0.21f, slide.getSoftnessOrthoAng(), 0f);
        Assert.assertEquals(b + 0.22f, slide.getSoftnessOrthoLin(), 0f);

        Assert.assertEquals(b + 0.23f, slide.getTargetAngMotorVelocity(), 0f);
        Assert.assertEquals(b + 0.24f, slide.getTargetLinMotorVelocity(), 0f);

        Assert.assertEquals(b + 0.25f, slide.getUpperAngLimit(), 0f);
        Assert.assertEquals(b + 0.26f, slide.getUpperLinLimit(), 0f);
    }

    private static void verifySoftAngular(SoftAngularJoint saj, float b) {
        Assert.assertEquals(b + 0.01f, saj.getCFM(), 0f);
        Assert.assertEquals(b + 0.02f, saj.getERP(), 0f);
        Assert.assertEquals(b + 0.03f, saj.getSplit(), 0f);
    }

    private static void verifySoftLinear(SoftLinearJoint slj, float b) {
        Assert.assertEquals(b + 0.02f, slj.getCFM(), 0f);
        Assert.assertEquals(b + 0.03f, slj.getERP(), 0f);
        Assert.assertEquals(b + 0.04f, slj.getSplit(), 0f);
    }

    private static void verifySpring(SixDofSpringJoint spring, float b) {
        verifySix(spring, b);

        Assert.assertEquals(b + 0.251f, spring.getDamping(0), 0f);
        Assert.assertEquals(b + 0.252f, spring.getDamping(1), 0f);
        Assert.assertEquals(b + 0.253f, spring.getDamping(2), 0f);
        Assert.assertEquals(b + 0.254f, spring.getDamping(3), 0f);
        Assert.assertEquals(b + 0.255f, spring.getDamping(4), 0f);
        Assert.assertEquals(b + 0.256f, spring.getDamping(5), 0f);

        Assert.assertEquals(b + 0.261f, spring.getStiffness(0), 0f);
        Assert.assertEquals(b + 0.262f, spring.getStiffness(1), 0f);
        Assert.assertEquals(b + 0.263f, spring.getStiffness(2), 0f);
        Assert.assertEquals(b + 0.264f, spring.getStiffness(3), 0f);
        Assert.assertEquals(b + 0.265f, spring.getStiffness(4), 0f);
        Assert.assertEquals(b + 0.266f, spring.getStiffness(5), 0f);
    }
}
