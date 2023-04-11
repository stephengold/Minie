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
package jme3utilities.tutorial;

import com.jme3.app.SimpleApplication;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.input.KeyInput;
import com.jme3.input.controls.ActionListener;
import com.jme3.input.controls.InputListener;
import com.jme3.input.controls.KeyTrigger;
import com.jme3.math.Vector3f;

/**
 * A simple demonstration of contact response.
 * <p>
 * Press the spacebar to disable the ball's contact response. Once this happens,
 * the blue (static) box no longer exerts any contact force on the ball. Gravity
 * takes over, and the ball falls through.
 * <p>
 * Builds upon HelloStaticBody.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class HelloContactResponse extends SimpleApplication {
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the HelloContactResponse application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        HelloContactResponse application = new HelloContactResponse();
        application.start();
    }
    // *************************************************************************
    // SimpleApplication methods

    /**
     * Initialize this application.
     */
    @Override
    public void simpleInitApp() {
        // Set up Bullet physics (with debug enabled).
        BulletAppState bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);
        bulletAppState.setDebugEnabled(true); // for debug visualization
        PhysicsSpace physicsSpace = bulletAppState.getPhysicsSpace();

        // Add a static box to the space, to serve as a horizontal platform.
        float boxHalfExtent = 3f;
        CollisionShape boxShape = new BoxCollisionShape(boxHalfExtent);
        PhysicsRigidBody box
                = new PhysicsRigidBody(boxShape, PhysicsBody.massForStatic);
        physicsSpace.addCollisionObject(box);
        box.setPhysicsLocation(new Vector3f(0f, -4f, 0f));

        // Add a dynamic ball to the space.
        float ballRadius = 1f;
        CollisionShape ballShape = new SphereCollisionShape(ballRadius);
        float ballMass = 2f;
        final PhysicsRigidBody ball = new PhysicsRigidBody(ballShape, ballMass);
        physicsSpace.addCollisionObject(ball);
        assert ball.isContactResponse();

        // Position the ball directly above the box.
        ball.setPhysicsLocation(new Vector3f(0f, 4f, 0f));

        // Configure the InputManager to respond to the spacebar.
        inputManager.addMapping("freefall", new KeyTrigger(KeyInput.KEY_SPACE));
        InputListener actionListener = new ActionListener() {
            @Override
            public void onAction(String action, boolean ongoing, float tpf) {
                if (action.equals("freefall") && ongoing) {
                    // Disable the ball's contact response.
                    ball.setContactResponse(false);

                    // Activate the ball in case it got deactivated.
                    ball.activate();
                }
            }
        };
        inputManager.addListener(actionListener, "freefall");
    }
}
