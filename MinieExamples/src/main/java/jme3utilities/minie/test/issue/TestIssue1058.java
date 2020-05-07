/*
 Copyright (c) 2019-2020, Stephen Gold
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
package jme3utilities.minie.test.issue;

import com.jme3.app.Application;
import com.jme3.app.SimpleApplication;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.objects.PhysicsRigidBody;

/**
 * Test case for JME issue #1058: native Bullet crash while removing a rigid
 * body from a BroadphaseType.SIMPLE PhysicsSpace.
 * <p>
 * If successful, the app will complete normally.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestIssue1058
        extends SimpleApplication
        implements PhysicsTickListener {

    private volatile boolean hasStepped = false;
    private PhysicsRigidBody body1;
    private PhysicsRigidBody body2;
    private PhysicsSpace physicsSpace;

    public static void main(String[] arguments) {
        Application application = new TestIssue1058();
        application.start();
    }

    @Override
    public void simpleInitApp() {
        BulletAppState bulletAppState = new BulletAppState();
        bulletAppState.setBroadphaseType(PhysicsSpace.BroadphaseType.SIMPLE);
        stateManager.attach(bulletAppState);

        physicsSpace = bulletAppState.getPhysicsSpace();
        physicsSpace.addTickListener(this);

        CollisionShape shape = new SphereCollisionShape(1f);

        body1 = new PhysicsRigidBody(shape, 1f);
        physicsSpace.addCollisionObject(body1);

        body2 = new PhysicsRigidBody(shape, 2f);
        physicsSpace.addCollisionObject(body2);
    }

    @Override
    public void simpleUpdate(float tpf) {
        super.simpleUpdate(tpf);
        if (hasStepped) {
            physicsSpace.removeCollisionObject(body1); // native Bullet crash occurs here
            physicsSpace = null;
            stop();
        }
    }

    @Override
    public void prePhysicsTick(PhysicsSpace space, float tpf) {
        // do nothing
    }

    @Override
    public void physicsTick(PhysicsSpace space, float tpf) {
        hasStepped = true;
    }
}
