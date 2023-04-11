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
package jme3utilities.tutorial;

import com.jme3.app.SimpleApplication;
import com.jme3.bullet.PhysicsSoftSpace;
import com.jme3.bullet.SoftPhysicsAppState;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.control.SoftBodyControl;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.bullet.objects.infos.Sbcp;
import com.jme3.bullet.objects.infos.SoftBodyConfig;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.material.Materials;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Box;

/**
 * A simple example of a soft body.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class HelloSoftBody extends SimpleApplication {
    // *************************************************************************
    // fields

    /**
     * PhysicsSpace for simulation
     */
    private static PhysicsSoftSpace physicsSpace;
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the HelloSoftBody application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        HelloSoftBody application = new HelloSoftBody();
        application.start();
    }
    // *************************************************************************
    // SimpleApplication methods

    /**
     * Initialize this application.
     */
    @Override
    public void simpleInitApp() {
        // Set up Bullet physics.
        SoftPhysicsAppState bulletAppState = new SoftPhysicsAppState();
        stateManager.attach(bulletAppState);
        //bulletAppState.setDebugEnabled(true); // for debug visualization
        physicsSpace = bulletAppState.getPhysicsSoftSpace();

        // Add a box to the scene and relocate the camera.
        addBox();
        cam.setLocation(new Vector3f(0f, 1f, 8f));

        // Add a light to the scene.
        Vector3f direction = new Vector3f(1f, -2f, -4f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction);
        rootNode.addLight(sun);

        // Add a model to the scene.
        Spatial cgModel = assetManager.loadModel(
                "Models/MonkeyHead/MonkeyHead.mesh.xml");
        rootNode.attachChild(cgModel);

        // Add a soft-body control to the model.
        SoftBodyControl sbc = new SoftBodyControl();
        cgModel.addControl(sbc);
        /*
         * Translate and rotate the model's physics body.  Since the control
         * is "dynamic", the model will follow its body.
         */
        PhysicsSoftBody body = sbc.getBody();
        body.applyRotation(new Quaternion().fromAngles(0.4f, 0f, 1f));
        body.applyTranslation(new Vector3f(0f, 3f, 0f));
        /*
         * Set the ball's default frame pose:  if deformed,
         * it will tend to return to its current shape.
         */
        boolean setVolumePose = false;
        boolean setFramePose = true;
        body.setPose(setVolumePose, setFramePose);

        // Enable pose matching to make the body bouncy.
        SoftBodyConfig config = body.getSoftConfig();
        config.set(Sbcp.PoseMatching, 0.05f);

        sbc.setPhysicsSpace(physicsSpace);
    }
    // *************************************************************************
    // private methods

    /**
     * Add a large static cube to serve as a platform.
     */
    private void addBox() {
        float halfExtent = 2f; // mesh units
        Mesh mesh = new Box(halfExtent, halfExtent, halfExtent);
        Geometry geometry = new Geometry("cube platform", mesh);
        rootNode.attachChild(geometry);

        geometry.move(0f, -halfExtent, 0f);
        ColorRGBA color = new ColorRGBA(0.1f, 0.4f, 0.1f, 1f);
        Material material = new Material(assetManager, Materials.LIGHTING);
        material.setBoolean("UseMaterialColors", true);
        material.setColor("Diffuse", color);
        geometry.setMaterial(material);
        geometry.setShadowMode(RenderQueue.ShadowMode.Receive);

        BoxCollisionShape shape = new BoxCollisionShape(halfExtent);
        RigidBodyControl boxBody
                = new RigidBodyControl(shape, PhysicsBody.massForStatic);
        geometry.addControl(boxBody);
        boxBody.setPhysicsSpace(physicsSpace);
    }
}
