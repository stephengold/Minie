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
package jme3utilities.tutorial;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Vector3f;
import com.jme3.system.NativeLibraryLoader;

/**
 * A simple example of a console application without a scene graph.
 * <p>
 * Builds upon HelloStaticBody.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class HelloConsole {
    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private HelloConsole() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the HelloConsole application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        // Load the native library for this platform.
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);

        // Create a physics space.
        PhysicsSpace physicsSpace
                = new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);

        // Create a CollisionShape for balls.
        float ballRadius = 1f;
        CollisionShape ballShape = new SphereCollisionShape(ballRadius);

        // Create a dynamic body and add it to the space.
        float mass = 2f;
        PhysicsRigidBody dynaBall = new PhysicsRigidBody(ballShape, mass);
        physicsSpace.addCollisionObject(dynaBall);
        dynaBall.setPhysicsLocation(new Vector3f(0f, 4f, 0f));

        // Create a static body and add it to the space.
        PhysicsRigidBody statBall
                = new PhysicsRigidBody(ballShape, PhysicsBody.massForStatic);
        physicsSpace.addCollisionObject(statBall);
        statBall.setPhysicsLocation(new Vector3f(0.1f, 0f, 0f));

        // 50 iterations with a 20-msec timestep
        float timeStep = 0.02f;
        for (int iteration = 0; iteration < 50; ++iteration) {
            physicsSpace.update(timeStep, 0);
            Vector3f location = dynaBall.getPhysicsLocation();
            float t = timeStep * iteration;
            System.out.printf("t = %.2f sec  location = %s%n", t, location);
        }
    }
}
