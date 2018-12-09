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

import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.control.VehicleControl;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsVehicle;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.system.NativeLibraryLoader;
import jme3utilities.Misc;
import org.junit.Test;

/**
 * Test cloning on PhysicsRigidBody and all its subclasses.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestCloneBody {
    // *************************************************************************
    // new methods exposed

    @Test
    public void testCloneBody() {
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);

        CollisionShape shape = new SphereCollisionShape(1f);
        /*
         * PhysicsRigidBody
         */
        PhysicsRigidBody body = new PhysicsRigidBody(shape, 1f);
        setParameters(body, 0f);
        verifyParameters(body, 0f);
        PhysicsRigidBody bodyClone = (PhysicsRigidBody) Misc.deepCopy(body);
        cloneTest(body, bodyClone);
        /*
         * RigidBodyControl
         */
        RigidBodyControl rbc = new RigidBodyControl(shape, 1f);
        setParameters(rbc, 0f);
        verifyParameters(rbc, 0f);
        RigidBodyControl rbcClone = (RigidBodyControl) Misc.deepCopy(rbc);
        cloneTest(rbc, rbcClone);
        /*
         * PhysicsVehicle
         */
        PhysicsVehicle vehicle = new PhysicsVehicle(shape, 1f);
        setParameters(vehicle, 0f);
        verifyParameters(vehicle, 0f);
        PhysicsRigidBody vehicleClone = (PhysicsRigidBody) Misc.deepCopy(vehicle);
        cloneTest(vehicle, vehicleClone);
        /*
         * VehicleControl
         */
        VehicleControl vc = new VehicleControl(shape, 1f);
        setParameters(vc, 0f);
        verifyParameters(vc, 0f);
        VehicleControl vcClone = (VehicleControl) Misc.deepCopy(vc);
        cloneTest(vc, vcClone);
    }
    // *************************************************************************
    // private methods

    private void cloneTest(PhysicsRigidBody body, PhysicsRigidBody bodyClone) {
        assert bodyClone.getObjectId() != body.getObjectId();

        verifyParameters(body, 0f);
        verifyParameters(bodyClone, 0f);

        setParameters(body, 0.3f);
        verifyParameters(body, 0.3f);
        verifyParameters(bodyClone, 0f);

        setParameters(bodyClone, 0.6f);
        verifyParameters(body, 0.3f);
        verifyParameters(bodyClone, 0.6f);
    }

    private void setParameters(PhysicsRigidBody body, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);
        body.setContactResponse(flag);
        body.setKinematic(!flag);

        body.setAngularDamping(b + 0.01f);
        body.setAngularFactor(b + 0.02f);
        body.setSleepingThresholds(b + 0.17f, b + 0.03f);
        body.setAngularVelocity(new Vector3f(b + 0.04f, b + 0.05f, b + 0.06f));
        body.setCcdMotionThreshold(b + 0.07f);
        body.setCcdSweptSphereRadius(b + 0.08f);
        body.setFriction(b + 0.09f);
        body.setGravity(new Vector3f(b + 0.10f, b + 0.11f, b + 0.12f));
        body.setLinearDamping(b + 0.13f);
        body.setLinearFactor(new Vector3f(b + 0.14f, b + 0.15f, b + 0.16f));
        body.setPhysicsLocation(new Vector3f(b + 0.18f, b + 0.19f, b + 0.20f));

        Quaternion orient
                = new Quaternion(b + 0.21f, b + 0.22f, b + 0.23f, b + 0.24f);
        orient.normalizeLocal();
        Matrix3f matrix = orient.toRotationMatrix();
        body.setPhysicsRotation(matrix);

        body.setRestitution(b + 0.25f);
        body.setLinearVelocity(new Vector3f(b + 0.26f, b + 0.27f, b + 0.28f));
    }

    private void verifyParameters(PhysicsRigidBody body, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);
        assert body.isContactResponse() == flag;
        assert body.isKinematic() == !flag;

        assert body.getAngularDamping() == b + 0.01f;
        assert body.getAngularFactor() == b + 0.02f;
        assert body.getAngularSleepingThreshold() == b + 0.03f;

        Vector3f w = body.getAngularVelocity(null);
        assert w.x == b + 0.04f : w;
        assert w.y == b + 0.05f : w;
        assert w.z == b + 0.06f : w;

        assert body.getCcdMotionThreshold() == b + 0.07f;
        assert body.getCcdSweptSphereRadius() == b + 0.08f;
        assert body.getFriction() == b + 0.09f;

        Vector3f g = body.getGravity(null);
        assert g.x == b + 0.10f : g;
        assert g.y == b + 0.11f : g;
        assert g.z == b + 0.12f : g;

        assert body.getLinearDamping() == b + 0.13f;

        Vector3f f = body.getLinearFactor(null);
        assert f.x == b + 0.14f : f;
        assert f.y == b + 0.15f : f;
        assert f.z == b + 0.16f : f;

        assert body.getLinearSleepingThreshold() == b + 0.17f;

        Vector3f x = body.getPhysicsLocation(null);
        assert x.x == b + 0.18f : x;
        assert x.y == b + 0.19f : x;
        assert x.z == b + 0.20f : x;

        Quaternion orient
                = new Quaternion(b + 0.21f, b + 0.22f, b + 0.23f, b + 0.24f);
        orient.normalizeLocal();
        Matrix3f matrix = orient.toRotationMatrix();
        Matrix3f m = body.getPhysicsRotationMatrix(null);
        assert m.equals(matrix);

        assert body.getRestitution() == b + 0.25f;

        Vector3f v = body.getLinearVelocity(null);
        assert v.x == b + 0.26f : v;
        assert v.y == b + 0.27f : v;
        assert v.z == b + 0.28f : v;
    }
}
