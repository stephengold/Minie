/*
 Copyright (c) 2024 Stephen Gold
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
import com.jme3.bullet.collision.ContactListener;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.HeightfieldCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.material.Material;
import com.jme3.material.Materials;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.shape.Sphere;
import java.util.logging.Logger;
import jme3utilities.minie.PhysicsDumper;

/**
 * Test for Minie issue #40 (certain shapes fall through a flat terrain with
 * contact filtering).
 * <p>
 * Collision objects are rendered entirely by debug visualization.
 * <p>
 * If successful, the ball will land on the square heightmap. If unsuccessful,
 * it will pass through and keep going.
 * <p>
 * Based on "TerrainTest.java" provided by ndebruyn.
 */
final public class TestIssue40 extends SimpleApplication
        implements ContactListener, PhysicsTickListener {
    // *************************************************************************
    // constants and loggers

    /**
     * simulation step to start printing debug info
     */
    final private static int startTick = 51;
    /**
     * simulation step to stop printing debug info
     */
    final private static int stopTick = 70;
    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(TestIssue40.class.getName());
    // *************************************************************************
    // fields

    /**
     * count simulation steps
     */
    private static int tickCount = 0;
    /**
     * dynamic ball
     */
    private static PhysicsRigidBody ballBody;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the TestIssue40 application.
     * <p>
     * This no-arg constructor was made explicit to avoid javadoc warnings from
     * JDK 18+.
     */
    public TestIssue40() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the TestIssue40 application.
     *
     * @param arguments unused
     */
    public static void main(String[] arguments) {
        TestIssue40 application = new TestIssue40();
        application.start();
    }
    // *************************************************************************
    // SimpleApplication methods

    /**
     * Initialize the application.
     */
    @Override
    public void simpleInitApp() {
        // Configure the camera:
        flyCam.setEnabled(false);
        cam.setLocation(new Vector3f(-10f, 5f, 10f));
        cam.setRotation(new Quaternion(0.064f, 0.9106f, -0.156f, 0.377f));

        // Configure physics:
        BulletAppState bulletAppState = new BulletAppState();
        bulletAppState.setDebugEnabled(true);
        stateManager.attach(bulletAppState);
        PhysicsSpace physicsSpace = bulletAppState.getPhysicsSpace();
        physicsSpace.addContactListener(this);
        physicsSpace.addTickListener(this);

        // Add a dynamic ball:
        Sphere sphere = new Sphere(5, 7, 1f);
        HullCollisionShape hullShape = new HullCollisionShape(sphere);
        ballBody = new PhysicsRigidBody(hullShape);
        ballBody.setPhysicsLocation(new Vector3f(0f, 5f, 0f));
        physicsSpace.add(ballBody);

        // Add a small static terrain with all heights=0:
        CollisionShape heightShape = new HeightfieldCollisionShape(
                new float[9], new Vector3f(2f, 1f, 2f));
        PhysicsRigidBody terrainBody
                = new PhysicsRigidBody(heightShape, PhysicsBody.massForStatic);
        //heightShape.setContactFilterEnabled(false); // default=true
        physicsSpace.add(terrainBody);

        // For clarity, make the terrain opaque:
        Material solidGray = new Material(assetManager, Materials.UNSHADED);
        solidGray.setColor("Color", ColorRGBA.DarkGray);
        terrainBody.setDebugMaterial(solidGray);

        new PhysicsDumper().dump(bulletAppState);
    }
    // *************************************************************************
    // ContactListener methods

    /**
     * Invoked immediately after a contact manifold is removed.
     *
     * @param manifoldId the native ID of the {@code btPersistentManifold} (not
     * zero)
     */
    @Override
    public void onContactEnded(long manifoldId) {
        if (tickCount >= startTick && tickCount < stopTick) {
            System.out.println("  removed manifold " + manifoldId);
            System.out.flush();
        }
    }

    /**
     * Invoked immediately after a contact point is refreshed without being
     * removed. Skipped for Sphere-Sphere contacts.
     *
     * @param pcoA the first involved object (not null)
     * @param pcoB the 2nd involved object (not null)
     * @param manifoldPointId the native ID of the {@code btManifoldPoint} (not
     * zero)
     */
    @Override
    public void onContactProcessed(PhysicsCollisionObject pcoA,
            PhysicsCollisionObject pcoB, long manifoldPointId) {
        if (tickCount >= startTick && tickCount < stopTick) {
            System.out.println("  processed point " + manifoldPointId);
            System.out.flush();
        }
    }

    /**
     * Invoked immediately after a contact manifold is created.
     *
     * @param manifoldId the native ID of the {@code btPersistentManifold} (not
     * zero)
     */
    @Override
    public void onContactStarted(long manifoldId) {
        if (tickCount >= startTick && tickCount < stopTick) {
            System.out.println("  created manifold " + manifoldId);
            System.out.flush();
        }
    }
    // *************************************************************************
    // PhysicsTickListener methods

    /**
     * Callback from Bullet, invoked just before the physics is stepped.
     *
     * @param space the space that's about to be stepped (not null)
     * @param timeStep the duration of the simulation step (in seconds, &ge;0)
     */
    @Override
    public void prePhysicsTick(PhysicsSpace space, float timeStep) {
        if (tickCount >= startTick && tickCount < stopTick) {
            Vector3f location = ballBody.getPhysicsLocation(null);
            Vector3f velocity = ballBody.getLinearVelocity(null);
            int numManifolds = space.countManifolds();
            System.out.println("tick " + tickCount + "  y=" + location.y
                    + "  vy=" + velocity.y + " numManifolds=" + numManifolds);
            System.out.flush();
        }
    }

    /**
     * Callback from Bullet, invoked just after the physics has been stepped.
     *
     * @param space the space that was just stepped (not null)
     * @param timeStep the duration of the simulation step (in seconds, &ge;0)
     */
    @Override
    public void physicsTick(PhysicsSpace space, float timeStep) {
        ++tickCount;
    }
}
