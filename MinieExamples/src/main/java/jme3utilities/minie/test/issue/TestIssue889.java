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
package jme3utilities.minie.test.issue;

import com.jme3.app.SimpleApplication;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.control.BetterCharacterControl;
import com.jme3.bullet.control.GhostControl;
import com.jme3.bullet.control.RigidBodyControl;
import java.util.logging.Logger;

/**
 * Test case for JME issue #889: disabled physics control gets added to a
 * PhysicsSpace.
 * <p>
 * If successful, no debug meshes will be visible.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestIssue889 extends SimpleApplication {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(TestIssue889.class.getName());
    // *************************************************************************
    // constructors

    /**
     * Instantiate the TestIssue889 application.
     */
    public TestIssue889() { // explicit to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the TestIssue889 application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        TestIssue889 app = new TestIssue889();
        app.start();
    }
    // *************************************************************************
    // SimpleApplication methods

    /**
     * Initialize this application.
     */
    @Override
    public void simpleInitApp() {
        flyCam.setEnabled(false);

        BulletAppState bulletAppState = new BulletAppState();
        bulletAppState.setDebugEnabled(true);
        bulletAppState.setSpeed(0f);
        stateManager.attach(bulletAppState);
        PhysicsSpace space = bulletAppState.getPhysicsSpace();

        float radius = 1f;
        CollisionShape sphere = new SphereCollisionShape(radius);
        CollisionShape box = new BoxCollisionShape(1f);

        RigidBodyControl rbc = new RigidBodyControl(box);
        rbc.setEnabled(false);
        rbc.setPhysicsSpace(space);
        rootNode.addControl(rbc);

        BetterCharacterControl bcc = new BetterCharacterControl(radius, 4f, 1f);
        bcc.setEnabled(false);
        bcc.setPhysicsSpace(space);
        rootNode.addControl(bcc);

        GhostControl gc = new GhostControl(sphere);
        gc.setEnabled(false);
        gc.setPhysicsSpace(space);
        rootNode.addControl(gc);
    }
}
