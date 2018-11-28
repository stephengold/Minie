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

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.joints.ConeJoint;
import com.jme3.bullet.joints.HingeJoint;
import com.jme3.bullet.joints.JointEnd;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.joints.Point2PointJoint;
import com.jme3.bullet.joints.SixDofJoint;
import com.jme3.bullet.joints.SixDofSpringJoint;
import com.jme3.bullet.joints.SliderJoint;
import com.jme3.bullet.joints.motors.RotationalLimitMotor;
import com.jme3.bullet.joints.motors.TranslationalLimitMotor;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.FastMath;
import com.jme3.math.Matrix3f;
import com.jme3.math.Vector3f;
import com.jme3.system.NativeLibraryLoader;
import jme3utilities.Misc;
import org.junit.Test;

/**
 * Test cloning physics joints of all types.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestCloneJoints {
    // *************************************************************************
    // new methods exposed

    @Test
    public void testCloneJoints() {
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);

        CollisionShape box = new BoxCollisionShape(new Vector3f(1f, 1f, 1f));
        PhysicsRigidBody bodyA = new PhysicsRigidBody(box, 1f);
        PhysicsRigidBody bodyB = new PhysicsRigidBody(box, 1f);
        /*
         * ConeJoint: single- and double-ended
         */
        ConeJoint cone
                = new ConeJoint(bodyA, bodyB, new Vector3f(), new Vector3f());
        set(cone, 0f);
        verify(cone, 0f);
        ConeJoint coneClone = (ConeJoint) Misc.deepCopy(cone);
        cloneTest(cone, coneClone);

        ConeJoint seCone = new ConeJoint(bodyA, new Vector3f(), new Matrix3f());
        set(seCone, 0f);
        verify(seCone, 0f);
        ConeJoint seConeClone = (ConeJoint) Misc.deepCopy(seCone);
        cloneTest(seCone, seConeClone);
        /*
         * HingeJoint: single- and double-ended
         */
        HingeJoint hinge = new HingeJoint(bodyA, bodyB, new Vector3f(),
                new Vector3f(), new Vector3f(1f, 0f, 0f),
                new Vector3f(1f, 0f, 0f));
        set(hinge, 0f);
        verify(hinge, 0f);
        HingeJoint hingeClone = (HingeJoint) Misc.deepCopy(hinge);
        cloneTest(hinge, hingeClone);

        HingeJoint seHinge = new HingeJoint(bodyA, new Vector3f(),
                new Vector3f(), new Vector3f(1f, 0f, 0f),
                new Vector3f(1f, 0f, 0f), JointEnd.A);
        set(seHinge, 0f);
        verify(seHinge, 0f);
        HingeJoint seHingeClone = (HingeJoint) Misc.deepCopy(seHinge);
        cloneTest(seHinge, seHingeClone);
        /*
         * Point2PointJoint: single- and double-ended
         */
        Point2PointJoint p2p = new Point2PointJoint(bodyA, bodyB,
                new Vector3f(), new Vector3f());
        set(p2p, 0f);
        verify(p2p, 0f);
        Point2PointJoint p2pClone = (Point2PointJoint) Misc.deepCopy(p2p);
        cloneTest(p2p, p2pClone);

        Point2PointJoint sep2p
                = new Point2PointJoint(bodyA, new Vector3f(), new Vector3f());
        set(sep2p, 0f);
        verify(sep2p, 0f);
        Point2PointJoint sep2pClone = (Point2PointJoint) Misc.deepCopy(sep2p);
        cloneTest(sep2p, sep2pClone);
        /*
         * SixDofJoint: single- and double-ended
         */
        SixDofJoint six = new SixDofJoint(bodyA, bodyB, new Vector3f(),
                new Vector3f(), new Matrix3f(), new Matrix3f(), false);
        set(six, 0f);
        verify(six, 0f);
        SixDofJoint sixClone = (SixDofJoint) Misc.deepCopy(six);
        cloneTest(six, sixClone);

        SixDofJoint seSix = new SixDofJoint(bodyA, new Vector3f(),
                new Vector3f(), new Matrix3f(), new Matrix3f(), JointEnd.A);
        set(seSix, 0f);
        verify(seSix, 0f);
        SixDofJoint seSixClone = (SixDofJoint) Misc.deepCopy(seSix);
        cloneTest(seSix, seSixClone);
        /*
         * SixDofSpringJoint: single- and double-ended
         */
        SixDofSpringJoint spring = new SixDofSpringJoint(bodyA, bodyB,
                new Vector3f(), new Vector3f(), new Matrix3f(), new Matrix3f(),
                false);
        set(spring, 0f);
        verify(spring, 0f);
        SixDofSpringJoint springClone
                = (SixDofSpringJoint) Misc.deepCopy(spring);
        cloneTest(spring, springClone);

        SixDofSpringJoint seSpring = new SixDofSpringJoint(bodyA, bodyB,
                new Vector3f(), new Vector3f(), new Matrix3f(), new Matrix3f(),
                false);
        set(seSpring, 0f);
        verify(seSpring, 0f);
        SixDofSpringJoint seSpringClone
                = (SixDofSpringJoint) Misc.deepCopy(seSpring);
        cloneTest(seSpring, seSpringClone);
        /*
         * SliderJoint: single- and double-ended
         */
        SliderJoint slide = new SliderJoint(bodyA, bodyB, new Vector3f(),
                new Vector3f(), new Matrix3f(), new Matrix3f(), false);
        set(slide, 0f);
        verify(slide, 0f);
        SliderJoint slideClone = (SliderJoint) Misc.deepCopy(slide);
        cloneTest(slide, slideClone);

        SliderJoint seSlide = new SliderJoint(bodyB, new Vector3f(),
                new Vector3f(), JointEnd.B);
        set(seSlide, 0f);
        verify(seSlide, 0f);
        SliderJoint seSlideClone = (SliderJoint) Misc.deepCopy(seSlide);
        cloneTest(seSlide, seSlideClone);
    }
    // *************************************************************************
    // private methods

    private static void cloneTest(PhysicsJoint joint, PhysicsJoint jointClone) {
        assert jointClone.getObjectId() != joint.getObjectId();

        PhysicsRigidBody a = joint.getBodyA();
        PhysicsRigidBody aClone = jointClone.getBodyA();
        if (a == null) {
            assert aClone == null;
        } else {
            assert aClone != null;
            assert a.getObjectId() != aClone.getObjectId();
        }

        PhysicsRigidBody b = joint.getBodyB();
        PhysicsRigidBody bClone = jointClone.getBodyB();
        if (b == null) {
            assert bClone == null;
        } else {
            assert bClone != null;
            assert b.getObjectId() != bClone.getObjectId();
        }

        verify(joint, 0f);
        verify(jointClone, 0f);

        set(joint, 0.3f);
        verify(joint, 0.3f);
        verify(jointClone, 0f);

        set(jointClone, 0.6f);
        verify(joint, 0.3f);
        verify(jointClone, 0.6f);
    }

    private static void set(PhysicsJoint joint, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);
        joint.setEnabled(flag);
        joint.setBreakingImpulseThreshold(b + 0.505f);

        if (joint instanceof ConeJoint) {
            setCone((ConeJoint) joint, b);
        } else if (joint instanceof HingeJoint) {
            setHinge((HingeJoint) joint, b);
        } else if (joint instanceof Point2PointJoint) {
            setP2P((Point2PointJoint) joint, b);
        } else if (joint instanceof SixDofJoint) {
            setSix((SixDofJoint) joint, b);
        } else if (joint instanceof SliderJoint) {
            setSlide((SliderJoint) joint, b);
        } else {
            throw new IllegalArgumentException(joint.getClass().getName());
        }
    }

    private static void setCone(ConeJoint cone, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);
        cone.setCollisionBetweenLinkedBodies(flag);
        cone.setAngularOnly(!flag);

        cone.setLimit(b + 0.01f, b + 0.02f, b + 0.03f);
    }

    private static void setHinge(HingeJoint hinge, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);
        hinge.setCollisionBetweenLinkedBodies(flag);
        hinge.setAngularOnly(!flag);

        hinge.enableMotor(flag, b + 0.01f, b + 0.02f);
        hinge.setLimit(b + 0.03f, b + 0.04f, b + 0.05f, b + 0.06f, b + 0.07f);
    }

    private static void setP2P(Point2PointJoint p2p, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);
        p2p.setCollisionBetweenLinkedBodies(flag);

        p2p.setDamping(b + 0.01f);
        p2p.setImpulseClamp(b + 0.02f);
        p2p.setTau(b + 0.03f);
    }

    private static void setSix(SixDofJoint six, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);
        six.setCollisionBetweenLinkedBodies(flag);

        RotationalLimitMotor rot
                = six.getRotationalLimitMotor(PhysicsSpace.AXIS_Z);
        rot.setEnableMotor(!flag);
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
        slide.setCollisionBetweenLinkedBodies(flag);

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

    private static void verify(PhysicsJoint joint, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);
        assert joint.isEnabled() == flag;
        assert joint.getBreakingImpulseThreshold() == b + 0.505f;

        if (joint instanceof ConeJoint) {
            verifyCone((ConeJoint) joint, b);
        } else if (joint instanceof HingeJoint) {
            verifyHinge((HingeJoint) joint, b);
        } else if (joint instanceof Point2PointJoint) {
            verifyP2P((Point2PointJoint) joint, b);
        } else if (joint instanceof SixDofJoint) {
            verifySix((SixDofJoint) joint, b);
        } else if (joint instanceof SliderJoint) {
            verifySlide((SliderJoint) joint, b);
        } else {
            throw new IllegalArgumentException(joint.getClass().getName());
        }
    }

    private static void verifyCone(ConeJoint cone, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);
        assert cone.isCollisionBetweenLinkedBodies() == flag;
        assert cone.isAngularOnly() == !flag;

        assert cone.getSwingSpan1() == b + 0.01f;
        assert cone.getSwingSpan2() == b + 0.02f;
        assert cone.getTwistSpan() == b + 0.03f;
    }

    private static void verifyHinge(HingeJoint hinge, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);
        assert hinge.isCollisionBetweenLinkedBodies() == flag;
        assert hinge.isAngularOnly() == !flag;

        assert FastMath.approximateEquals(
                hinge.getMotorTargetVelocity(), b + 0.01f);
        assert FastMath.approximateEquals(
                hinge.getMaxMotorImpulse(), b + 0.02f);

        assert FastMath.approximateEquals(
                hinge.getLowerLimit(), b + 0.03f);
        assert FastMath.approximateEquals(
                hinge.getUpperLimit(), b + 0.04f);
        assert FastMath.approximateEquals(
                hinge.getLimitSoftness(), b + 0.05f);
        assert FastMath.approximateEquals(
                hinge.getBiasFactor(), b + 0.06f);
        assert FastMath.approximateEquals(
                hinge.getRelaxationFactor(), b + 0.07f);
    }

    private static void verifyP2P(Point2PointJoint p2p, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);
        assert p2p.isCollisionBetweenLinkedBodies() == flag;

        assert p2p.getDamping() == b + 0.01f;
        assert p2p.getImpulseClamp() == b + 0.02f;
        assert p2p.getTau() == b + 0.03f;
    }

    private static void verifySix(SixDofJoint six, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);
        assert six.isCollisionBetweenLinkedBodies() == flag;

        RotationalLimitMotor rot
                = six.getRotationalLimitMotor(PhysicsSpace.AXIS_Z);
        assert rot.isEnableMotor() == !flag;
        assert rot.getRestitution() == b + 0.01f;
        assert rot.getDamping() == b + 0.02f;
        assert rot.getERP() == b + 0.03f;
        assert rot.getLowerLimit() == b + 0.04f;
        assert rot.getUpperLimit() == b + 0.05f;
        assert rot.getLimitSoftness() == b + 0.06f;
        assert rot.getMaxLimitForce() == b + 0.07f;
        assert rot.getMaxMotorForce() == b + 0.08f;
        assert rot.getTargetVelocity() == b + 0.09f;
        assert rot.getNormalCFM() == b + 0.091f;
        assert rot.getStopCFM() == b + 0.092f;

        TranslationalLimitMotor tra = six.getTranslationalLimitMotor();
        assert tra.getDamping() == b + 0.10f;
        assert tra.getLimitSoftness() == b + 0.11f;
        assert tra.getLowerLimit(null).x == b + 0.12f;
        assert tra.getLowerLimit(null).y == b + 0.13f;
        assert tra.getLowerLimit(null).z == b + 0.14f;
        assert tra.getRestitution() == b + 0.15f;
        assert tra.getUpperLimit(null).x == b + 0.16f;
        assert tra.getUpperLimit(null).y == b + 0.17f;
        assert tra.getUpperLimit(null).z == b + 0.18f;
        assert tra.getERP(null).z == b + 0.19f;
        assert tra.getMaxMotorForce(null).z == b + 0.20f;
        assert tra.getNormalCFM(null).z == b + 0.22f;
        assert tra.getStopCFM(null).z == b + 0.23f;
        assert tra.getTargetVelocity(null).z == b + 0.24f;
    }

    private static void verifySlide(SliderJoint slide, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);
        assert slide.isCollisionBetweenLinkedBodies() == flag;

        assert slide.getDampingDirAng() == b + 0.01f;
        assert slide.getDampingDirLin() == b + 0.02f;
        assert slide.getDampingLimAng() == b + 0.03f;
        assert slide.getDampingLimLin() == b + 0.04f;
        assert slide.getDampingOrthoAng() == b + 0.05f;
        assert slide.getDampingOrthoLin() == b + 0.06f;

        assert slide.getLowerAngLimit() == b + 0.07f;
        assert slide.getLowerLinLimit() == b + 0.08f;

        assert slide.getMaxAngMotorForce() == b + 0.09f;
        assert slide.getMaxLinMotorForce() == b + 0.10f;

        assert slide.isPoweredAngMotor() == !flag;
        assert slide.isPoweredLinMotor() == flag;

        assert slide.getRestitutionDirAng() == b + 0.11f;
        assert slide.getRestitutionDirLin() == b + 0.12f;
        assert slide.getRestitutionLimAng() == b + 0.13f;
        assert slide.getRestitutionLimLin() == b + 0.14f;
        assert slide.getRestitutionOrthoAng() == b + 0.15f;
        assert slide.getRestitutionOrthoLin() == b + 0.16f;

        assert slide.getSoftnessDirAng() == b + 0.17f;
        assert slide.getSoftnessDirLin() == b + 0.18f;
        assert slide.getSoftnessLimAng() == b + 0.19f;
        assert slide.getSoftnessLimLin() == b + 0.20f;
        assert slide.getSoftnessOrthoAng() == b + 0.21f;
        assert slide.getSoftnessOrthoLin() == b + 0.22f;

        assert slide.getTargetAngMotorVelocity() == b + 0.23f;
        assert slide.getTargetLinMotorVelocity() == b + 0.24f;

        assert slide.getUpperAngLimit() == b + 0.25f;
        assert slide.getUpperLinLimit() == b + 0.26f;
    }
}
