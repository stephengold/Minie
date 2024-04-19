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
import com.jme3.asset.plugins.FileLocator;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.MeshCollisionShape;
import com.jme3.bullet.collision.shapes.infos.BoundingValueHierarchy;
import com.jme3.bullet.collision.shapes.infos.ChildCollisionShape;
import com.jme3.bullet.util.CollisionShapeFactory;
import com.jme3.scene.Spatial;
import java.util.logging.Logger;

/**
 * Test for Minie issue #41 (JVM crash while serializing BVH on Windows).
 * <p>
 * If successful, the test will print "successful" and complete normally. If
 * unsuccessful, it will crash the Java Virtual Machine.
 * <p>
 * Derived from "Main.java" provided by joliver82.
 */
final public class TestIssue41 extends SimpleApplication {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(TestIssue41.class.getName());
    // *************************************************************************
    // constructors

    /**
     * Instantiate the TestIssue41 application.
     * <p>
     * This no-arg constructor was made explicit to avoid javadoc warnings from
     * JDK 18+.
     */
    public TestIssue41() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the TestIssue41 application.
     *
     * @param arguments unused
     */
    public static void main(String[] arguments) {
        TestIssue41 application = new TestIssue41();
        application.start();
    }
    // *************************************************************************
    // SimpleApplication methods

    /**
     * Initialize the application.
     */
    @Override
    public void simpleInitApp() {
        assetManager.registerLocator(".", FileLocator.class);
        Spatial geomPhy = assetManager.loadModel("pc-screen.gltf");

        CollisionShape meshShape;
        meshShape = CollisionShapeFactory.createMeshShape(geomPhy);
        ChildCollisionShape[] children
                = ((CompoundCollisionShape) meshShape).listChildren();
        BoundingValueHierarchy bvh
                = ((MeshCollisionShape) (children[1].getShape())).getBvh();
        bvh.serialize();

        System.out.println("successful");
        stop();
    }
}
