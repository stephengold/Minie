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
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.objects.PhysicsGhostObject;
import java.util.logging.Logger;

/**
 * Test case for JME issue #1351: native Bullet crash during garbage collection.
 *
 * If successful, the application will complete normally.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class TestIssue1351 extends SimpleApplication {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(TestIssue1351.class.getName());
    // *************************************************************************
    // constructors

    /**
     * Instantiate the TestIssue1351 application.
     */
    public TestIssue1351() { // explicit to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the TestIssue1351 application.
     *
     * @param arguments unused
     */
    public static void main(String[] arguments) {
        TestIssue1351 application = new TestIssue1351();
        application.start();
    }

    /**
     * Initialize this application.
     */
    @Override
    public void simpleInitApp() {
        CollisionShape shape = new SphereCollisionShape(1f);
        for (int i = 0; i < 10; ++i) {
            {
                PhysicsGhostObject a = new PhysicsGhostObject(shape);
                PhysicsGhostObject b = new PhysicsGhostObject(shape);
                PhysicsSpace space
                        = new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
                space.addCollisionObject(a);
                space.addCollisionObject(b);
            }
            System.gc();
        }
        stop();
    }
}
