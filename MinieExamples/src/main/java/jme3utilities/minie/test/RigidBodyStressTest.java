/*
 Copyright (c) 2019, Stephen Gold
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

import com.jme3.app.SimpleApplication;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.font.BitmapFont;
import com.jme3.font.BitmapText;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import java.util.Random;
import java.util.logging.Logger;

/**
 * Determine how many rigid bodies Bullet Physics can support at 15 fps.
 *
 * Run with Vsync enabled!
 */
public class RigidBodyStressTest extends SimpleApplication {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(RigidBodyStressTest.class.getName());
    // *************************************************************************
    // fields

    /**
     * true for one frame
     */
    private boolean isFirstFrame = true;
    /**
     * scene-graph node for displaying user-interface text
     */
    private BitmapText uiText;
    /**
     * shape for falling gems
     */
    private CollisionShape gemShape;
    /**
     * accumulate tpf up to 1 second
     */
    private float secondCounter = 0f;
    /**
     * count the frames in one second
     */
    private int frameCounter = 0;
    /**
     * number of falling bodies in the scene
     */
    private int numGems = 0;
    /**
     * physics space
     */
    private PhysicsSpace physicsSpace;
    /**
     * pseudo-random generator
     */
    final private Random random = new Random(1L);
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        RigidBodyStressTest application = new RigidBodyStressTest();
        application.start();
    }
    // *************************************************************************
    // SimpleApplication methods

    /**
     * Initialize this application.
     */
    @Override
    public void simpleInitApp() {
        configureCamera();
        configureUi();
        configurePhysics();
        addBox();

        int shapeId = 3;
        switch (shapeId) {
            case 1:
                gemShape = new SphereCollisionShape(0.1f);
                break;
            case 2:
                gemShape
                        = new BoxCollisionShape(new Vector3f(0.1f, 0.1f, 0.1f));
                break;
            case 3:
                Geometry teapot = (Geometry) assetManager.loadModel(
                        "Models/Teapot/Teapot.obj");
                gemShape = new HullCollisionShape(teapot.getMesh());
                gemShape.setScale(new Vector3f(0.5f, 0.5f, 0.5f));
        }

        gemShape.setMargin(0.005f);
    }

    /**
     * Callback invoked once per frame.
     *
     * @param tpf time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void simpleUpdate(float tpf) {
        super.simpleUpdate(tpf);

        if (isFirstFrame) {
            // The first frame includes startup time, so ignore it.
            isFirstFrame = false;
        } else {
            secondCounter += getTimer().getTimePerFrame();
        }
        /*
         * Calculate the frame rate and abort the test if it's too low.
         */
        frameCounter++;
        if (secondCounter >= 1f) {
            float fps = frameCounter / secondCounter;
            if (fps < 15f) {
                System.out.printf("final numGems = %d%n", numGems);
                stop();
            }
            secondCounter = 0f;
            frameCounter = 0;
        }
        /*
         * Add complex shapes once per second, simple shapes once per frame.
         */
        if (!(gemShape instanceof HullCollisionShape) || frameCounter == 0) {
            addAGem();
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Add a falling PhysicsRigidBody to the scene.
     */
    private void addAGem() {
        float x = 2f * random.nextFloat() - 1f;
        float y = 2f * random.nextFloat() - 1f;
        float z = 2f * random.nextFloat() - 1f;
        Vector3f startLocation = new Vector3f(x, y, z);
        startLocation.multLocal(0.5f, 1f, 0.5f);
        startLocation.y += 4f;

        float mass = 1f;
        PhysicsRigidBody body = new PhysicsRigidBody(gemShape, mass);
        body.setDamping(0.6f, 0.6f);
        body.setFriction(1f);
        body.setKinematic(false);
        body.setPhysicsLocation(startLocation);

        physicsSpace.add(body);
        body.setGravity(new Vector3f(0f, -9f, 0f));

        ++numGems;
        /*
         * Update the user interface.
         */
        String msg = String.format("numGems=%d", numGems);
        uiText.setText(msg);
    }

    /**
     * Add a large static box to serve as a platform.
     */
    private void addBox() {
        Node boxNode = new Node("box");
        rootNode.attachChild(boxNode);

        float halfExtent = 50f;
        boxNode.move(0f, -halfExtent, 0f);

        Vector3f hes = new Vector3f(halfExtent, halfExtent, halfExtent);
        BoxCollisionShape bcs = new BoxCollisionShape(hes);
        float mass = 0f;
        RigidBodyControl boxBody = new RigidBodyControl(bcs, mass);
        boxBody.setPhysicsSpace(physicsSpace);
        boxNode.addControl(boxBody);
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        flyCam.setEnabled(false);
        cam.setLocation(new Vector3f(0f, 1.5f, 7f));
        cam.setRotation(new Quaternion(0f, 0.9935938f, -0.113f, 0f));
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        BulletAppState bulletAppState = new BulletAppState();
        bulletAppState.setDebugEnabled(true);
        stateManager.attach(bulletAppState);

        physicsSpace = bulletAppState.getPhysicsSpace();
        physicsSpace.setAccuracy(1f / 60); // 16.67 msec timestep
        physicsSpace.setSolverNumIterations(10);
    }

    /*
     * Add a BitmapText in the upper-left corner of the display.
     */
    private void configureUi() {
        BitmapFont font = assetManager.loadFont("Interface/Fonts/Default.fnt");
        uiText = new BitmapText(font);
        guiNode.attachChild(uiText);
        float displayHeight = cam.getHeight();
        uiText.move(0f, displayHeight, 0f);
    }
}
