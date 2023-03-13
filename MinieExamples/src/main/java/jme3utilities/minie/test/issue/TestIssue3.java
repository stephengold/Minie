/*
 Copyright (c) 2020-2023, Stephen Gold
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

import com.jme3.app.SimpleApplication;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.MeshCollisionShape;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsCharacter;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.util.NativeLibrary;
import com.jme3.input.KeyInput;
import com.jme3.input.controls.ActionListener;
import com.jme3.input.controls.KeyTrigger;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import java.nio.FloatBuffer;
import java.util.logging.Logger;
import jme3utilities.math.MyBuffer;
import jme3utilities.mesh.RectangleMesh;
import jme3utilities.minie.PhysicsDumper;

/**
 * Test case for Minie issue #3: btAssert() at the peak of a character's jump.
 * <p>
 * Press the spacebar. If successful, the jump will complete with no JVM crash.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class TestIssue3 extends SimpleApplication
        implements ActionListener, PhysicsTickListener {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(TestIssue3.class.getName());
    // *************************************************************************
    // fields

    /**
     * count of physics timesteps simulated
     */
    private static int tickCount = 0;
    /**
     * jumping character to be tested
     */
    private static PhysicsCharacter character;
    /**
     * space for physics simulation
     */
    private static PhysicsSpace physicsSpace;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the TestIssue3 application.
     */
    public TestIssue3() { // explicit to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the TestIssue3 application.
     *
     * @param arguments unused
     */
    public static void main(String[] arguments) {
        TestIssue3 app = new TestIssue3();
        app.start();
    }
    // *************************************************************************
    // SimpleApplication methods

    /**
     * Initialize this application.
     */
    @Override
    public void simpleInitApp() {
        inputManager.addMapping("Jump", new KeyTrigger(KeyInput.KEY_SPACE));
        inputManager.addListener(this, "Jump");

        BulletAppState bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);
        bulletAppState.setDebugEnabled(true);
        physicsSpace = bulletAppState.getPhysicsSpace();

        assert NativeLibrary.isDebug() : "This test requires a Debug library.";

        Mesh mesh = new RectangleMesh(-999f, 999f, -999f, 999f, 1f);
        FloatBuffer buffer = mesh.getFloatBuffer(VertexBuffer.Type.Position);
        Quaternion rot = new Quaternion().fromAngles(1.5f, 0f, 0f);
        MyBuffer.rotate(buffer, 0, buffer.limit(), rot);
        CollisionShape meshShape = new MeshCollisionShape(mesh);
        PhysicsRigidBody staticBody
                = new PhysicsRigidBody(meshShape, PhysicsBody.massForStatic);
        physicsSpace.addCollisionObject(staticBody);

        CapsuleCollisionShape capsuleShape = new CapsuleCollisionShape(1f, 3f);
        character = new PhysicsCharacter(capsuleShape, 0.05f);
        character.setGravity(120f);
        character.setPhysicsLocation(new Vector3f(0f, 2.5f, 0f));
        physicsSpace.addCollisionObject(character);

        new PhysicsDumper().dump(bulletAppState);
    }

    /**
     * Process an input action.
     *
     * @param binding textual description of the action (not null)
     * @param ongoing true if the action is ongoing, otherwise false
     * @param tpf time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void onAction(String binding, boolean ongoing, float tpf) {
        if (ongoing && binding.equals("Jump")) {
            if (physicsSpace.countTickListeners() == 0) {
                physicsSpace.addTickListener(this);
            }
            character.jump();
        }
    }

    /**
     * Callback from Bullet, invoked just before the physics is stepped.
     *
     * @param space the space that's about to be stepped (not null)
     * @param timeStep the time per simulation step (in seconds, &ge;0)
     */
    @Override
    public void prePhysicsTick(PhysicsSpace space, float timeStep) {
        ++tickCount;
        System.out.println("tick #" + tickCount);
        if (tickCount >= 5) {
            System.out.flush();
        }
    }

    /**
     * Callback from Bullet, invoked just after the physics has been stepped.
     *
     * @param space the space that was just stepped (not null)
     * @param timeStep the time per simulation step (in seconds, &ge;0)
     */
    @Override
    public void physicsTick(PhysicsSpace space, float timeStep) {
        // do nothing
    }
}
