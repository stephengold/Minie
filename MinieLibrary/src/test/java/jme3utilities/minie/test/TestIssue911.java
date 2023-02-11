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

import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.system.NativeLibraryLoader;
import org.junit.Assert;
import org.junit.Test;

/**
 * Test case for JME issue #911: PhysicsRigidBody sleeping threshold setters
 * have unexpected side effects.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestIssue911 {
    // *************************************************************************
    // new methods exposed

    /**
     * Test to verify that the issue remains solved.
     */
    @Test
    public void testIssue911() {
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);

        CollisionShape capsule = new SphereCollisionShape(1f);
        PhysicsRigidBody body = new PhysicsRigidBody(capsule, 1f);
        Assert.assertEquals(1f, body.getAngularSleepingThreshold(), 0f);
        Assert.assertEquals(0.8f, body.getLinearSleepingThreshold(), 0f);

        body.setAngularSleepingThreshold(0.03f);

        assert body.getAngularSleepingThreshold() == 0.03f;
        float lst = body.getLinearSleepingThreshold();
        Assert.assertEquals(0.8f, lst, 0f); // fails, actual value is 1f

        body.setLinearSleepingThreshold(0.17f);

        float ast = body.getAngularSleepingThreshold();
        Assert.assertEquals(0.03f, ast, 0f); // fails, actual value is 1f
    }
}
