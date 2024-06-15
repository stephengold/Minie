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
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.font.BitmapText;
import com.jme3.material.Material;
import com.jme3.material.Materials;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.shape.Box;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;

/**
 * The "fountain of boxes" stress test to estimate how many active dynamic rigid
 * bodies can be simulated while rendering 30 frames per second.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestManyBoxes extends SimpleApplication {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(TestManyBoxes.class.getName());
    // *************************************************************************
    // fields

    private static BitmapText statusDisplay;
    private static Box smallBox;
    private static BoxCollisionShape smallBoxShape;
    private static float secondsSinceFpsUpdate = 0f;
    private static int framesSinceFpsUpdate = 0;
    private static int numBoxes = 1;
    private static Material solidGray;
    private static PhysicsSpace physicsSpace;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the TestManyBoxes application.
     * <p>
     * This no-arg constructor was made explicit to avoid javadoc warnings from
     * JDK 18+.
     */
    public TestManyBoxes() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the TestManyBoxes application.
     *
     * @param arguments unused
     */
    public static void main(String[] arguments) {
        TestManyBoxes app = new TestManyBoxes();
        app.start();
    }
    // *************************************************************************
    // SimpleApplication methods

    /**
     * Initialize the application.
     */
    @Override
    public void simpleInitApp() {
        if (Heart.areAssertionsEnabled()) {
            System.out.println("Warning: assertions are enabled.");
        }

        // Disable logging of rigid-body creation:
        Logger prbLogger
                = Logger.getLogger("com.jme3.bullet.objects.PhysicsRigidBody");
        prbLogger.setLevel(Level.SEVERE);

        // Pre-position the main camera and disable FlyByCamera:
        cam.setLocation(new Vector3f(0f, 110f, 210f));
        cam.setRotation(new Quaternion(0f, 0.9777315f, -0.20985f, 0.00143f));
        flyCam.setEnabled(false);

        // Attach a simple status display to the GUI node:
        statusDisplay = new BitmapText(loadGuiFont());
        statusDisplay.setLocalTranslation(0f, settings.getHeight(), 0f);
        guiNode.attachChild(statusDisplay);

        // Create a high-gravity PhysicsSpace:
        BulletAppState bulletAppState = new BulletAppState();
        bulletAppState.setBroadphaseType(
                PhysicsSpace.BroadphaseType.AXIS_SWEEP_3); // default = DBVT
        bulletAppState.setWorldMax(new Vector3f(100f, 50f, 100f));
        bulletAppState.setWorldMin(new Vector3f(-100f, -200f, -100f));
        stateManager.attach(bulletAppState);
        physicsSpace = bulletAppState.getPhysicsSpace();
        physicsSpace.setGravity(new Vector3f(0f, -99f, 0f));
        physicsSpace.setMaxSubSteps(1); // IMPORTANT! default = 4
        System.out.printf("maxSubSteps = %d, numSolvers = %d%n",
                physicsSpace.maxSubSteps(), physicsSpace.countSolvers());

        // Add a big blue box to serve as a platform:
        float size = 99f; // half extent in world units
        Box bigBox = new Box(size, size, size);
        BoxCollisionShape bigBoxShape
                = new BoxCollisionShape(new Vector3f(size, size, size));
        Material solidBlue = new Material(assetManager, Materials.UNSHADED);
        solidBlue.setColor("Color", ColorRGBA.Blue);

        RigidBodyControl staticRbc = new RigidBodyControl(bigBoxShape, 0f);
        staticRbc.setPhysicsSpace(physicsSpace);

        Geometry bbGeom = new Geometry("BigBox", bigBox);
        rootNode.attachChild(bbGeom);
        bbGeom.move(0f, -size, 0f);
        bbGeom.addControl(staticRbc);
        bbGeom.setMaterial(solidBlue);

        smallBox = new Box(1f, 1f, 1f);
        smallBoxShape = new BoxCollisionShape(new Vector3f(1f, 1f, 1f));

        solidGray = new Material(assetManager, Materials.UNSHADED);
        solidGray.setColor("Color", ColorRGBA.DarkGray);
    }

    /**
     * Callback invoked once per frame to add a box and update statistics.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void simpleUpdate(float tpf) {
        // Drop a small, gray, dynamic box from (0, 40, 0):
        RigidBodyControl dynamicRbc = new RigidBodyControl(smallBoxShape, 1f);
        dynamicRbc.setPhysicsSpace(physicsSpace);

        Geometry sbGeom = new Geometry("", smallBox);
        rootNode.attachChild(sbGeom);
        sbGeom.move(0f, 40f, 0f);
        sbGeom.addControl(dynamicRbc);
        sbGeom.setMaterial(solidGray);
        ++numBoxes;

        // Update the status:
        String statusText = "numBoxes = " + numBoxes;
        statusDisplay.setText(statusText);

        // Update the FPS statistics:
        secondsSinceFpsUpdate += tpf;
        ++framesSinceFpsUpdate;
        if (secondsSinceFpsUpdate >= 1f) {
            // Calculate frames per second over a 1-second interval:
            float fps = framesSinceFpsUpdate / secondsSinceFpsUpdate;
            if (fps < 30f) {
                // Terminate the application if FPS drops below 30:
                System.out.println(statusText);
                stop();
            }
            secondsSinceFpsUpdate = 0f;
            framesSinceFpsUpdate = 0;
        }
    }
}
