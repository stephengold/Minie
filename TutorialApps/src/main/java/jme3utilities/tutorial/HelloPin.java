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
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.bullet.objects.infos.SoftBodyConfig;
import com.jme3.bullet.objects.infos.SoftBodyMaterial;
import com.jme3.bullet.util.NativeSoftBodyUtil;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import jme3utilities.mesh.ClothGrid;

/**
 * A simple cloth simulation with a pinned node.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class HelloPin extends SimpleApplication {
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the HelloPin application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        HelloPin application = new HelloPin();
        application.start();
    }
    // *************************************************************************
    // SimpleApplication methods

    /**
     * Initialize this application.
     */
    @Override
    public void simpleInitApp() {
        // Set up Bullet physics (with debug enabled).
        SoftPhysicsAppState bulletAppState = new SoftPhysicsAppState();
        stateManager.attach(bulletAppState);
        bulletAppState.setDebugEnabled(true); // for debug visualization
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
        Mesh squareGrid = new ClothGrid(numLines, numLines, lineSpacing);

        // Create a soft square and add it to the physics space.
        PhysicsSoftBody cloth = new PhysicsSoftBody();
        NativeSoftBodyUtil.appendFromTriMesh(squareGrid, cloth);
        physicsSpace.addCollisionObject(cloth);

        // Pin one of the corner nodes by setting its mass to zero.
        int nodeIndex = 0; // upper left corner
        cloth.setNodeMass(nodeIndex, PhysicsBody.massForStatic);
        /*
         * Make the cloth flexible by reducing the angular stiffness
         * of its material.
         */
        SoftBodyMaterial mat = cloth.getSoftMaterial();
        mat.setAngularStiffness(0f); // default=1

        // Improve simulation accuracy by increasing
        // the number of position-solver iterations for the cloth.
        SoftBodyConfig config = cloth.getSoftConfig();
        config.setPositionIterations(9);  // default=1

        // Translate the cloth upward to its starting location.
        cloth.applyTranslation(new Vector3f(0f, 2f, 0f));
    }
}
