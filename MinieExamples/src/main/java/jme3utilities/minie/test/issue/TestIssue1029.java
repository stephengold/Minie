/*
 Copyright (c) 2019-2023, Stephen Gold
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
import com.jme3.bullet.collision.PhysicsCollisionEvent;
import com.jme3.bullet.collision.PhysicsCollisionListener;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.objects.PhysicsGhostObject;
import com.jme3.bullet.objects.PhysicsRigidBody;
import java.util.logging.Logger;
import jme3utilities.minie.MyShape;

/**
 * Test case for JME issue #1029: sphere-sphere collisions not reported.
 * <p>
 * Collision objects are rendered entirely by debug visualization.
 * <p>
 * If successful, the app will terminate normally, without a RuntimeException.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestIssue1029
        extends SimpleApplication
        implements PhysicsCollisionListener {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(TestIssue1029.class.getName());
    // *************************************************************************
    // fields

    /**
     * track elapsed time (in seconds) for the timeout
     */
    private static double elapsedSeconds = 0.0;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the TestIssue1029 application.
     */
    public TestIssue1029() { // explicit to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the TestIssue1029 application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        Application application = new TestIssue1029();
        application.start();
    }
    // *************************************************************************
    // SimpleApplication methods

    /**
     * Initialize this application.
     */
    @Override
    public void simpleInitApp() {
        BulletAppState bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);
        bulletAppState.setDebugEnabled(true);

        PhysicsSpace physicsSpace = bulletAppState.getPhysicsSpace();
        physicsSpace.addCollisionListener(this);
        //physicsSpace.addOngoingCollisionListener(this);

        CollisionShape shape;
        shape = new SphereCollisionShape(1f);
        //shape = new BoxCollisionShape(new Vector3f(1f, 1f, 1f));

        PhysicsRigidBody staticBody = new PhysicsRigidBody(shape, 0f);
        physicsSpace.addCollisionObject(staticBody);

        PhysicsGhostObject ghost = new PhysicsGhostObject(shape);
        physicsSpace.addCollisionObject(ghost);
    }

    /**
     * Callback invoked once per frame.
     *
     * @param tpf time interval between frame (in seconds, &ge;0)
     */
    @Override
    public void simpleUpdate(float tpf) {
        super.simpleUpdate(tpf);

        elapsedSeconds += tpf;
        if (elapsedSeconds > 1.0) {
            throw new RuntimeException("No collisions reported!");
        }
    }
    // *************************************************************************
    // PhysicsCollisionListener methods

    /**
     * The collision listener, invoked multiple times for each collision that
     * occurs in the PhysicsSpace.
     *
     * @param event the event that occurred (not null, reusable)
     */
    @Override
    public void collision(PhysicsCollisionEvent event) {
        CollisionShape a = event.getObjectA().getCollisionShape();
        String aShape = MyShape.describeType(a);
        CollisionShape b = event.getObjectB().getCollisionShape();
        String bShape = MyShape.describeType(b);

        System.out.printf("%s-%s collision reported at t = %f sec%n",
                aShape, bShape, elapsedSeconds);
        stop();
    }
}
