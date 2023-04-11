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
package jme3utilities.tutorial;

import com.jme3.app.Application;
import com.jme3.app.SimpleApplication;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.material.Material;
import com.jme3.material.Materials;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.shape.Box;

/**
 * A simple example of debug visualization in a post ViewPort.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class HelloDebugToPost extends SimpleApplication {
    // *************************************************************************
    // fields

    /**
     * PhysicsSpace for simulation
     */
    private static PhysicsSpace physicsSpace;
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the HelloDebugToPost application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        Application application = new HelloDebugToPost();
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

        // Set up Bullet physics (with debug enabled).
        BulletAppState bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);
        bulletAppState.setDebugEnabled(true); // for debug visualization
        physicsSpace = bulletAppState.getPhysicsSpace();
        /*
         * Direct debug visuals to a post ViewPort that clears the depth buffer.
         * This prevents z-fighting between the box and its debug visuals,
         * but allows debug visuals to overdraw the GUI.
         */
        ViewPort overlay = renderManager.createPostView("Overlay", cam);
        overlay.setClearFlags(false, true, false);
        bulletAppState.setDebugViewPorts(overlay);

        addBox();
    }
    // *************************************************************************
    // private methods

    /**
     * Add a large static cube.
     */
    private void addBox() {
        float halfExtent = 1f; // mesh units
        Mesh mesh = new Box(halfExtent, halfExtent, halfExtent);
        Geometry geometry = new Geometry("box", mesh);
        rootNode.attachChild(geometry);

        Material boxMaterial = new Material(assetManager, Materials.UNSHADED);
        boxMaterial.setColor("Color", ColorRGBA.Gray.clone());
        geometry.setMaterial(boxMaterial);

        BoxCollisionShape shape = new BoxCollisionShape(halfExtent);
        RigidBodyControl boxBody
                = new RigidBodyControl(shape, PhysicsBody.massForStatic);
        geometry.addControl(boxBody);
        boxBody.setPhysicsSpace(physicsSpace);
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        flyCam.setMoveSpeed(10f);
        flyCam.setZoomSpeed(10f);

        cam.setLocation(new Vector3f(4.4f, 4.8f, 14.8f));
        cam.setRotation(new Quaternion(-0.0152f, 0.98352f, -0.15f, -0.09974f));
    }
}
