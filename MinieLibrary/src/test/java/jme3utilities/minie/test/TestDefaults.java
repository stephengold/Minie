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

import com.jme3.bullet.PhysicsSoftSpace;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.RayTestFlag;
import com.jme3.bullet.SoftBodyWorldInfo;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.collision.shapes.infos.DebugMeshNormals;
import com.jme3.bullet.joints.SixDofJoint;
import com.jme3.bullet.joints.motors.RotationalLimitMotor;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.math.Vector3f;
import com.jme3.system.NativeLibraryLoader;
import org.junit.Assert;
import org.junit.Test;

/**
 * Verify the defaults for physics objects.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestDefaults {
    // *************************************************************************
    // new methods exposed

    /**
     * Verify the defaults for physics objects.
     */
    @Test
    public void testDefaults() {
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);

        PhysicsSoftSpace space = new PhysicsSoftSpace(
                new Vector3f(-10000f, -10000f, -10000f),
                new Vector3f(10000f, 10000f, 10000f),
                PhysicsSpace.BroadphaseType.DBVT);
        Assert.assertEquals(0, space.countJoints());
        Assert.assertEquals(0, space.countRigidBodies());
        Assert.assertEquals(0, space.countSoftBodies());
        Assert.assertEquals(1 / 60f, space.getAccuracy(), 0f);
        assertEquals(0f, -9.81f, 0f, space.getGravity(null), 0f);
        Assert.assertEquals(4, space.maxSubSteps());
        Assert.assertEquals(0.1f, space.maxTimeStep(), 0f);
        Assert.assertEquals(RayTestFlag.SubSimplexRaytest,
                space.getRayTestFlags());
        Assert.assertEquals(10, space.getSolverNumIterations());

        SoftBodyWorldInfo info = new SoftBodyWorldInfo();
        Assert.assertEquals(1.2f, info.airDensity(), 0f);
        assertEquals(0f, -10f, 0f, info.copyGravity(null), 0f);
        Assert.assertEquals(1000f, info.maxDisplacement(), 0f);
        Assert.assertEquals(0f, info.waterDensity(), 0f);
        assertEquals(0f, 0f, 0f, info.copyWaterNormal(null), 0f);
        Assert.assertEquals(0f, info.waterOffset(), 0f);

        CollisionShape box = new BoxCollisionShape(1f);
        Assert.assertEquals(0.04f, box.getMargin(), 0f);
        assertEquals(1f, 1f, 1f, box.getScale(null), 0f);

        CollisionShape capsule = new CapsuleCollisionShape(1f, 1f);
        Assert.assertEquals(0f, capsule.getMargin(), 0f);
        assertEquals(1f, 1f, 1f, capsule.getScale(null), 0f);

        CollisionShape sphere = new SphereCollisionShape(1f);
        Assert.assertEquals(0f, sphere.getMargin(), 0f);
        assertEquals(1f, 1f, 1f, sphere.getScale(null), 0f);

        PhysicsRigidBody rigidA = new PhysicsRigidBody(box);
        Assert.assertFalse(rigidA.isInWorld());
        Assert.assertFalse(rigidA.isStatic());
        Assert.assertEquals(0, rigidA.countJoints());
        Assert.assertTrue(rigidA.isActive());
        Assert.assertEquals(0f, rigidA.getAngularDamping(), 0f);
        Assert.assertEquals(1f, rigidA.getAngularFactor(), 0f);
        Assert.assertEquals(1f, rigidA.getAngularSleepingThreshold(), 0f);
        assertEquals(0f, 0f, 0f, rigidA.getAngularVelocity(null), 0);
        assertEquals(1f, 1f, 1f, rigidA.getAnisotropicFriction(null), 0f);
        Assert.assertFalse(rigidA.hasAnisotropicFriction(1));
        Assert.assertFalse(rigidA.hasAnisotropicFriction(2));
        Assert.assertEquals(0f, rigidA.getCcdMotionThreshold(), 0f);
        Assert.assertEquals(0f, rigidA.getCcdSweptSphereRadius(), 0f);
        Assert.assertEquals(PhysicsCollisionObject.COLLISION_GROUP_01,
                rigidA.getCollideWithGroups());
        Assert.assertEquals(PhysicsCollisionObject.COLLISION_GROUP_01,
                rigidA.getCollisionGroup());
        Assert.assertEquals(0.1f, rigidA.getContactDamping(), 0f);
        Assert.assertEquals(1e18f, rigidA.getContactProcessingThreshold(), 0f);
        Assert.assertTrue(rigidA.isContactResponse());
        Assert.assertEquals(1e18f, rigidA.getContactStiffness(), 0f);
        Assert.assertEquals(0f, rigidA.getDeactivationTime(), 0f);
        Assert.assertNull(rigidA.getDebugMaterial());
        Assert.assertNull(rigidA.debugMeshInitListener());
        Assert.assertSame(DebugMeshNormals.None, rigidA.debugMeshNormals());
        Assert.assertEquals(DebugShapeFactory.lowResolution,
                rigidA.debugMeshResolution());
        Assert.assertEquals(1, rigidA.debugNumSides());
        Assert.assertEquals(0.5f, rigidA.getFriction(), 0f);
        Assert.assertFalse(rigidA.isKinematic());
        Assert.assertEquals(1f, rigidA.getMass(), 0f);
        Assert.assertEquals(0f, rigidA.getRestitution(), 0f);
        Assert.assertEquals(0f, rigidA.getRollingFriction(), 0f);
        Assert.assertEquals(0f, rigidA.getSpinningFriction(), 0f);
        Assert.assertNull(rigidA.getUserObject());

        PhysicsRigidBody rigidB = new PhysicsRigidBody(box);
        SixDofJoint six = new SixDofJoint(rigidA, rigidB, new Vector3f(),
                new Vector3f(), false);
        Assert.assertTrue(six.isCollisionBetweenLinkedBodies());
        Assert.assertTrue(six.isEnabled());
        Assert.assertEquals(Float.MAX_VALUE, six.getBreakingImpulseThreshold(),
                0f);

        RotationalLimitMotor rlm
                = six.getRotationalLimitMotor(PhysicsSpace.AXIS_X);
        Assert.assertEquals(1f, rlm.getDamping(), 0f);
        Assert.assertFalse(rlm.isEnableMotor());
        Assert.assertEquals(0.2f, rlm.getERP(), 0f);
        Assert.assertEquals(0.5f, rlm.getLimitSoftness(), 0f);
        Assert.assertEquals(1f, rlm.getLowerLimit(), 0f);
        Assert.assertEquals(300f, rlm.getMaxLimitForce(), 0f);
        Assert.assertEquals(6f, rlm.getMaxMotorForce(), 0f);
        Assert.assertEquals(0f, rlm.getNormalCFM(), 0f);
        Assert.assertEquals(0f, rlm.getRestitution(), 0f);
        Assert.assertEquals(0f, rlm.getStopCFM(), 0f);
        Assert.assertEquals(0f, rlm.getTargetVelocity(), 0f);
        Assert.assertEquals(-1f, rlm.getUpperLimit(), 0f);
    }

    void assertEquals(float x, float y, float z, Vector3f vector,
            float tolerance) {
        Assert.assertEquals(x, vector.x, tolerance);
        Assert.assertEquals(y, vector.y, tolerance);
        Assert.assertEquals(z, vector.z, tolerance);
    }
}
