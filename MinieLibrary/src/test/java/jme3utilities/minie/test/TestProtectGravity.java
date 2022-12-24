/*
 Copyright (c) 2020-2022, Stephen Gold
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

import com.jme3.bullet.MultiBodySpace;
import com.jme3.bullet.PhysicsSoftSpace;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.math.Vector3f;
import com.jme3.system.NativeLibraryLoader;
import org.junit.Assert;
import org.junit.Test;

/**
 * Test the setProtectGravity() method.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestProtectGravity {
    // *************************************************************************
    // constants

    final private static Vector3f bodyGravity = new Vector3f(0f, 1f, 2f);
    final private static Vector3f spaceGravity1 = new Vector3f(3f, 4f, 5f);
    final private static Vector3f spaceGravity2 = new Vector3f(6f, 7f, 8f);
    // *************************************************************************
    // fields

    private static CollisionShape shape;
    // *************************************************************************
    // new methods exposed

    /**
     * Test the setProtectGravity() method.
     */
    @Test
    public void testProtectGravity() {
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);

        float radius = 1f;
        shape = new SphereCollisionShape(radius);

        MultiBodySpace mbSpace = new MultiBodySpace(
                new Vector3f(-10000f, -10000f, -10000f),
                new Vector3f(10000f, 10000f, 10000f),
                PhysicsSpace.BroadphaseType.AXIS_SWEEP_3);
        mbSpace.setGravity(spaceGravity1);
        testPhysicsSpace(mbSpace);

        PhysicsSoftSpace space
                = new PhysicsSoftSpace(PhysicsSpace.BroadphaseType.DBVT);
        space.setGravity(spaceGravity1);
        testPhysicsSpace(space);

        PhysicsSpace pSpace
                = new PhysicsSpace(PhysicsSpace.BroadphaseType.AXIS_SWEEP_3);
        pSpace.setGravity(spaceGravity1);
        testPhysicsSpace(pSpace);
    }
    // *************************************************************************
    // private methods

    private static void testPhysicsSpace(PhysicsSpace space) {
        // a protected dynamic body
        float mass = 1f;
        PhysicsRigidBody pdb = new PhysicsRigidBody(shape, mass);
        pdb.setGravity(bodyGravity);
        pdb.setProtectGravity(true);
        Assert.assertEquals(bodyGravity, pdb.getGravity(null));
        Assert.assertTrue(pdb.isGravityProtected());
        Assert.assertTrue(pdb.isDynamic());

        // an unprotected dynamic body
        PhysicsRigidBody udb = new PhysicsRigidBody(shape, mass);
        udb.setGravity(bodyGravity);
        Assert.assertEquals(bodyGravity, udb.getGravity(null));
        Assert.assertFalse(udb.isGravityProtected());
        Assert.assertTrue(udb.isDynamic());

        // an unprotected static body
        PhysicsRigidBody usb
                = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);
        usb.setGravity(bodyGravity);
        Assert.assertEquals(bodyGravity, usb.getGravity(null));
        Assert.assertFalse(usb.isGravityProtected());
        Assert.assertTrue(usb.isStatic());

        PhysicsSoftBody pSoft = null;
        PhysicsSoftBody uSoft = null;
        if (space instanceof PhysicsSoftSpace) {
            pSoft = new PhysicsSoftBody();
            pSoft.setGravity(bodyGravity);
            pSoft.setProtectWorldInfo(true);
            Assert.assertEquals(bodyGravity, pSoft.getGravity(null));
            Assert.assertTrue(pSoft.isWorldInfoProtected());
            Assert.assertFalse(pSoft.isStatic());

            uSoft = new PhysicsSoftBody();
            uSoft.setGravity(bodyGravity);
            Assert.assertEquals(bodyGravity, uSoft.getGravity(null));
            Assert.assertFalse(uSoft.isWorldInfoProtected());
            Assert.assertFalse(uSoft.isStatic());
        }

        // Add all bodies to the PhysicsSpace.
        space.addCollisionObject(pdb);
        space.addCollisionObject(udb);
        space.addCollisionObject(usb);
        if (pSoft != null) {
            space.addCollisionObject(pSoft);
            space.addCollisionObject(uSoft);
        }

        Assert.assertEquals(bodyGravity, pdb.getGravity(null));
        Assert.assertEquals(spaceGravity1, udb.getGravity(null));
        Assert.assertEquals(bodyGravity, usb.getGravity(null));
        if (pSoft != null) {
            Assert.assertEquals(bodyGravity, pSoft.getGravity(null));
            Assert.assertEquals(spaceGravity1, uSoft.getGravity(null));
        }

        // Alter the gravity of the PhysicsSpace.
        space.setGravity(spaceGravity2);

        Assert.assertEquals(bodyGravity, pdb.getGravity(null));
        Assert.assertEquals(spaceGravity2, udb.getGravity(null));
        Assert.assertEquals(bodyGravity, usb.getGravity(null));
        if (pSoft != null) {
            Assert.assertEquals(bodyGravity, pSoft.getGravity(null));
            Assert.assertEquals(spaceGravity2, uSoft.getGravity(null));
        }

        // TODO test sleeping rigid bodies
    }
}
