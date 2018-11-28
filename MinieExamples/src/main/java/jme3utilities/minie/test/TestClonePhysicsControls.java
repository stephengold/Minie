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

import com.jme3.app.SimpleApplication;
import com.jme3.bullet.animation.DynamicAnimControl;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.control.BetterCharacterControl;
import com.jme3.bullet.control.GhostControl;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.control.VehicleControl;
import com.jme3.math.Vector3f;
import jme3utilities.Misc;

/**
 * Test cloning physics controls.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestClonePhysicsControls extends SimpleApplication {
    // *************************************************************************
    // new methods exposed

    public static void main(String[] args) {
        TestClonePhysicsControls app = new TestClonePhysicsControls();
        app.start();
    }
    // *************************************************************************
    // SimpleApplication methods

    @Override
    public void simpleInitApp() {
        CollisionShape capsule = new SphereCollisionShape(1f);

        RigidBodyControl rbc = new RigidBodyControl(capsule, 1f);
        RigidBodyControl rbcClone = (RigidBodyControl) Misc.deepCopy(rbc);

        GhostControl gc = new GhostControl(capsule);
        GhostControl gcClone = (GhostControl) Misc.deepCopy(gc);

        VehicleControl vc = new VehicleControl(capsule);
        Vector3f wheelDirection = new Vector3f(0, -1, 0);
        Vector3f wheelAxle = new Vector3f(-1, 0, 0);
        float radius = 0.5f;
        float restLength = 0.3f;
        Vector3f connectionPoint = new Vector3f(0f, 0f, 0f);
        vc.addWheel(null, connectionPoint,
                wheelDirection, wheelAxle, restLength, radius, true);

        VehicleControl vcClone = (VehicleControl) Misc.deepCopy(vc);

        float height = 3f;
        BetterCharacterControl bcc
                = new BetterCharacterControl(radius, height, 1f);
        BetterCharacterControl bccClone
                = (BetterCharacterControl) Misc.deepCopy(bcc);

        DynamicAnimControl dac = new DynamicAnimControl();
        DynamicAnimControl dacClone
                = (DynamicAnimControl) Misc.deepCopy(dac);

        stop();
        // TODO test cloning controls added to a scene graph
    }
}
