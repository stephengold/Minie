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
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.MeshCollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.logging.Logger;
import jme3utilities.math.noise.Generator;

/**
 * Generate a collision shape for a very large mesh.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestLargeMesh extends SimpleApplication {
    // *************************************************************************
    // constants and loggers

    final private static boolean optimized = false;
    final private static int numTriangles = 10_000_000;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(TestLargeMesh.class.getName());
    // *************************************************************************
    // constructors

    /**
     * Instantiate the TestLargeMesh application.
     */
    public TestLargeMesh() { // explicit to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the TestLargeMesh application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        TestLargeMesh app = new TestLargeMesh();
        app.start();
    }
    // *************************************************************************
    // SimpleApplication methods

    /**
     * Initialize this application.
     */
    @Override
    public void simpleInitApp() {
        Generator generator = new Generator();

        int numVertices = 3 * numTriangles;
        int numFloats = 3 * numVertices;
        FloatBuffer buffer = BufferUtils.createFloatBuffer(numFloats);
        for (int floatIndex = 0; floatIndex < numFloats; ++floatIndex) {
            float coordinate = generator.nextFloat();
            buffer.put(coordinate);
        }

        Mesh mesh = new Mesh();
        mesh.setBuffer(VertexBuffer.Type.Position, 3, buffer);
        CollisionShape shape = new MeshCollisionShape(mesh, optimized);

        RigidBodyControl rbc
                = new RigidBodyControl(shape, PhysicsBody.massForStatic);

        BulletAppState bulletAppState = new BulletAppState();
        //bulletAppState.setDebugEnabled(true);
        stateManager.attach(bulletAppState);
        PhysicsSpace space = bulletAppState.getPhysicsSpace();
        rbc.setPhysicsSpace(space);
        rootNode.addControl(rbc);

        stop();
    }
}
