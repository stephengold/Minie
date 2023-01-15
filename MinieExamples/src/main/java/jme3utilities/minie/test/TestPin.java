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
package jme3utilities.minie.test;

import com.jme3.bullet.PhysicsSoftSpace;
import com.jme3.bullet.SoftPhysicsAppState;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.collision.shapes.infos.IndexedMesh;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.bullet.objects.infos.SoftBodyConfig;
import com.jme3.bullet.objects.infos.SoftBodyMaterial;
import com.jme3.bullet.util.NativeSoftBodyUtil;
import com.jme3.math.Vector3f;
import com.jme3.system.AppSettings;
import jme3utilities.MyMesh;
import jme3utilities.MyString;
import jme3utilities.Validate;
import jme3utilities.ui.AcorusDemo;

/**
 * A simple cloth simulation with a pinned node, using a native mesh.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestPin extends AcorusDemo {
    // *************************************************************************
    // constants and loggers

    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName = TestPin.class.getSimpleName();
    // *************************************************************************
    // constructors

    /**
     * Instantiate the TestPin application.
     */
    public TestPin() { // made explicit to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the TestPin application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        String title = applicationName + " " + MyString.join(arguments);
        boolean loadDefaults = true;
        AppSettings settings = new AppSettings(loadDefaults);
        settings.setTitle(title); // Customize the window's title bar.

        TestPin application = new TestPin();
        application.setSettings(settings);
        application.start();
    }
    // *************************************************************************
    // AcorusDemo methods

    /**
     * Initialize this application.
     */
    @Override
    public void acorusInit() {
        super.acorusInit();

        // Set up Bullet physics (with debug enabled).
        SoftPhysicsAppState bulletAppState = new SoftPhysicsAppState();
        bulletAppState.setDebugEnabled(true);
        stateManager.attach(bulletAppState);
        PhysicsSoftSpace physicsSpace = bulletAppState.getPhysicsSoftSpace();

        // Relocate the camera.
        cam.setLocation(new Vector3f(0f, 1f, 8f));

        // Create a static, rigid sphere and add it to the physics space.
        float radius = 1f;
        SphereCollisionShape shape = new SphereCollisionShape(radius);
        PhysicsRigidBody sphere
                = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);
        physicsSpace.addCollisionObject(sphere);

        // Generate a subdivided square mesh with alternating diagonals.
        int numLines = 41;
        float lineSpacing = 0.1f; // mesh units
        IndexedMesh squareGrid
                = createClothGrid(numLines, numLines, lineSpacing);

        // Create a soft square and add it to the physics space.
        PhysicsSoftBody cloth = new PhysicsSoftBody();
        NativeSoftBodyUtil.appendFromNativeMesh(squareGrid, cloth);
        physicsSpace.addCollisionObject(cloth);

        // Pin one of the corner nodes by setting its mass to zero.
        int nodeIndex = 0;
        cloth.setNodeMass(nodeIndex, PhysicsBody.massForStatic);

        // Make the cloth flexible by altering the angular stiffness
        // of its material.
        SoftBodyMaterial mat = cloth.getSoftMaterial();
        mat.setAngularStiffness(0f); // default=1

        // Improve simulation accuracy by increasing
        // the number of position-solver iterations for the cloth.
        SoftBodyConfig config = cloth.getSoftConfig();
        config.setPositionIterations(9);  // default=1

        // Translate the cloth upward to its starting location.
        cloth.applyTranslation(new Vector3f(0f, 2f, 0f));
    }
    // *************************************************************************
    // private methods

    /**
     * Instantiate a grid in the X-Z plane, centered on (0,0,0).
     *
     * @param xLines the desired number of grid lines parallel to the X axis
     * (&ge;2)
     * @param zLines the desired number of grid lines parallel to the Z axis
     * (&ge;2)
     * @param lineSpacing the desired initial distance between adjacent grid
     * lines (in mesh units, &gt;0)
     * @return a new IndexedMesh
     */
    private static IndexedMesh
         createClothGrid(int xLines, int zLines, float lineSpacing) {
        Validate.inRange(xLines, "X lines", 2, Integer.MAX_VALUE);
        Validate.inRange(zLines, "Z lines", 2, Integer.MAX_VALUE);
        Validate.positive(lineSpacing, "line spacing");

        int numVertices = xLines * zLines;
        Vector3f[] positionArray = new Vector3f[numVertices];

        // Write the vertex locations:
        int vectorIndex = 0;
        for (int xIndex = 0; xIndex < zLines; ++xIndex) {
            float x = (2 * xIndex - zLines + 1) * lineSpacing / 2f;
            for (int zIndex = 0; zIndex < xLines; ++zIndex) {
                float z = (2 * zIndex - xLines + 1) * lineSpacing / 2f;
                positionArray[vectorIndex] = new Vector3f(x, 0f, z);
                ++vectorIndex;
            }
        }
        assert vectorIndex == positionArray.length;

        int numTriangles = 2 * (xLines - 1) * (zLines - 1);
        int numIndices = MyMesh.vpt * numTriangles;
        int[] indexArray = new int[numIndices];

        // Write vertex indices for triangles:
        int intIndex = 0;
        for (int zIndex = 0; zIndex < xLines - 1; ++zIndex) {
            for (int xIndex = 0; xIndex < zLines - 1; ++xIndex) {
                // 4 vertices and 2 triangles forming a square
                int vi0 = zIndex + xLines * xIndex;
                int vi1 = vi0 + 1;
                int vi2 = vi0 + xLines;
                int vi3 = vi1 + xLines;
                if ((xIndex + zIndex) % 2 == 0) {
                    // major diagonal: joins vi1 to vi2
                    indexArray[intIndex] = vi0;
                    indexArray[intIndex + 1] = vi1;
                    indexArray[intIndex + 2] = vi2;

                    indexArray[intIndex + 3] = vi3;
                    indexArray[intIndex + 4] = vi2;
                    indexArray[intIndex + 5] = vi1;
                } else {
                    // minor diagonal: joins vi0 to vi3
                    indexArray[intIndex] = vi0;
                    indexArray[intIndex + 1] = vi1;
                    indexArray[intIndex + 2] = vi3;

                    indexArray[intIndex + 3] = vi3;
                    indexArray[intIndex + 4] = vi2;
                    indexArray[intIndex + 5] = vi0;
                }
                intIndex += 6;
            }
        }
        assert intIndex == indexArray.length;

        IndexedMesh result = new IndexedMesh(positionArray, indexArray);
        return result;
    }
}
