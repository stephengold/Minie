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
import com.jme3.bullet.PhysicsSoftSpace;
import com.jme3.bullet.SoftPhysicsAppState;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.bullet.objects.infos.Sbcp;
import com.jme3.bullet.util.NativeSoftBodyUtil;
import com.jme3.material.Material;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.shape.Box;
import com.jme3.scene.shape.Sphere;
import jme3utilities.MyAsset;
import jme3utilities.minie.DumpFlags;
import jme3utilities.minie.PhysicsDumper;

/**
 * Test soft-body physics.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestSoftBody extends SimpleApplication {

    final private float gravity = 0.05f;
    final private float radius = 1f;
    private PhysicsSoftSpace physicsSpace;
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the TestSoftBody application.
     *
     * @param ignored array of command-line arguments (not null)
     */
    public static void main(String[] ignored) {
        TestSoftBody app = new TestSoftBody();
        app.start();
    }
    // *************************************************************************
    // SimpleApplication methods

    @Override
    public void simpleInitApp() {
        configureCamera();
        configurePhysics();
        addBox();

        Mesh mesh = new Sphere(6, 6, radius);
        PhysicsSoftBody softBody = new PhysicsSoftBody();
        NativeSoftBodyUtil.createFromTriMesh(mesh, softBody);
        softBody.setMassByArea(1f);

        PhysicsSoftBody.Config config = softBody.getSoftConfig();
        config.set(Sbcp.VolumeConservation, 0.99f);

        softBody.setPhysicsLocation(new Vector3f(0f, 0.825f, 0f));
        physicsSpace.add(softBody);
        softBody.setGravity(new Vector3f(0f, -gravity, 0f));

        new PhysicsDumper()
                .setEnabled(DumpFlags.ClustersInSofts, true)
                .setEnabled(DumpFlags.NodesInSofts, true)
                .dump(physicsSpace);
        System.out.println();
        System.out.flush();
    }
    // *************************************************************************
    // private methods

    /**
     * Add a large static box to the scene, to serve as a platform.
     */
    private void addBox() {
        float halfExtent = 50f; // mesh units
        Mesh mesh = new Box(halfExtent, halfExtent, halfExtent);
        Geometry boxGeometry = new Geometry("box", mesh);
        rootNode.attachChild(boxGeometry);

        boxGeometry.move(0f, -halfExtent, 0f);
        Material material = MyAsset.createDebugMaterial(assetManager);
        boxGeometry.setMaterial(material);

        BoxCollisionShape shape = new BoxCollisionShape(halfExtent);
        float mass = PhysicsRigidBody.massForStatic;
        RigidBodyControl boxBody = new RigidBodyControl(shape, mass);
        boxGeometry.addControl(boxBody);
        boxBody.setApplyScale(true);
        boxBody.setPhysicsSpace(physicsSpace);
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(4f);

        cam.setLocation(new Vector3f(0f, 1.2f, 5f));
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        SoftPhysicsAppState bulletAppState = new SoftPhysicsAppState();
        bulletAppState.setDebugEnabled(true);
        stateManager.attach(bulletAppState);

        physicsSpace = bulletAppState.getPhysicsSoftSpace();
        physicsSpace.setAccuracy(0.01f); // 10-msec timestep
    }
}
