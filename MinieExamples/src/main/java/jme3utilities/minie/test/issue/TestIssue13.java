/*
 Copyright (c) 2021-2023, Stephen Gold
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
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.PlaneCollisionShape;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsVehicle;
import com.jme3.bullet.objects.VehicleWheel;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Plane;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.MyAsset;
import jme3utilities.debug.AxesVisualizer;
import jme3utilities.mesh.PointMesh;

/**
 * Test for Minie issue #13 (vehicle acceleration depends on its location).
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class TestIssue13 extends SimpleApplication {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(TestIssue13.class.getName());
    // *************************************************************************
    // fields

    final static private Random random = new Random();
    // *************************************************************************
    // constructors

    /**
     * Instantiate the TestIssue13 application.
     */
    public TestIssue13() { // explicit to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the TestIssue13 application.
     *
     * @param arguments unused
     */
    public static void main(String[] arguments) {
        TestIssue13 application = new TestIssue13();
        application.start();
    }
    // *************************************************************************
    // SimpleApplication methods

    /**
     * Initialize this application.
     */
    @Override
    public void simpleInitApp() {
        PhysicsRigidBody.logger2.setLevel(Level.WARNING);

        cam.setLocation(new Vector3f(39f, 64f, 172f));
        cam.setRotation(new Quaternion(-0.013f, 0.98608f, -0.1254f, -0.1084f));
        flyCam.setMoveSpeed(100f);

        Material hiMat = MyAsset
                .createWireframeMaterial(assetManager, ColorRGBA.Red, 3f);
        Material loMat = MyAsset
                .createWireframeMaterial(assetManager, ColorRGBA.Green, 3f);

        // Add axes
        float axisLength = 30f;
        AxesVisualizer axes = new AxesVisualizer(assetManager, axisLength);
        axes.setLineWidth(AxesVisualizer.widthForSolid);
        rootNode.addControl(axes);
        axes.setEnabled(true);

        for (int i = 0; i < 1000; ++i) {
            float x = -50f + 100f * random.nextFloat();
            float z = -50f + 100f * random.nextFloat();
            float vz = test(x, z);

            PointMesh pointMesh = new PointMesh();
            pointMesh.setLocation(new Vector3f(x, 5f * vz, z));
            Geometry geometry = new Geometry("result", pointMesh);
            if (vz > 1f) {
                geometry.setMaterial(hiMat);
            } else {
                geometry.setMaterial(loMat);
            }
            rootNode.attachChild(geometry);
        }
    }
    // *************************************************************************
    // private methods

    private static float test(float startX, float startZ) {
        PhysicsSpace physicsSpace
                = new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);

        // Add a static plane to represent the ground.
        Plane plane = new Plane(Vector3f.UNIT_Y, 0f);
        PlaneCollisionShape shape = new PlaneCollisionShape(plane);
        PhysicsRigidBody ground
                = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);
        physicsSpace.addCollisionObject(ground);

        // Create a wedge-shaped vehicle with a low center of gravity.
        // The local forward direction is +Z.
        float noseZ = 1.4f;
        float spoilerY = 0.5f;
        float tailZ = -0.7f;
        float undercarriageY = -0.1f;
        float halfWidth = 0.4f;
        Collection<Vector3f> cornerLocations = new ArrayList<>(6);
        cornerLocations.add(new Vector3f(+halfWidth, undercarriageY, noseZ));
        cornerLocations.add(new Vector3f(-halfWidth, undercarriageY, noseZ));
        cornerLocations.add(new Vector3f(+halfWidth, undercarriageY, tailZ));
        cornerLocations.add(new Vector3f(-halfWidth, undercarriageY, tailZ));
        cornerLocations.add(new Vector3f(+halfWidth, spoilerY, tailZ));
        cornerLocations.add(new Vector3f(-halfWidth, spoilerY, tailZ));
        HullCollisionShape wedgeShape
                = new HullCollisionShape(cornerLocations);

        float mass = 1.525f;
        PhysicsVehicle vehicle = new PhysicsVehicle(wedgeShape, mass);
        vehicle.setDamping(0.086f, 0f);
        vehicle.setSuspensionCompression(6f);
        vehicle.setSuspensionDamping(7f);
        vehicle.setSuspensionStiffness(150f);

        // Add 4 wheels, 2 in the front (for steering) and 2 in the rear.
        boolean front = true;
        boolean rear = false;
        float frontAxisZ = 0.7f * noseZ;
        float rearAxisZ = 0.8f * tailZ;
        float radius = 0.425f; // of each tire
        float restLength = 0.2f; // of the suspension
        float xOffset = 0.9f * halfWidth;
        Vector3f axleDirection = new Vector3f(-1f, 0f, 0f);
        Vector3f suspensionDirection = new Vector3f(0f, -1f, 0f);
        vehicle.addWheel(new Vector3f(-xOffset, 0f, frontAxisZ),
                suspensionDirection, axleDirection, restLength, radius, front);
        vehicle.addWheel(new Vector3f(xOffset, 0f, frontAxisZ),
                suspensionDirection, axleDirection, restLength, radius, front);
        vehicle.addWheel(new Vector3f(-xOffset, 0f, rearAxisZ),
                suspensionDirection, axleDirection, restLength, radius, rear);
        vehicle.addWheel(new Vector3f(xOffset, 0f, rearAxisZ),
                suspensionDirection, axleDirection, restLength, radius, rear);

        for (int i = 0; i < 4; ++i) {
            VehicleWheel w = vehicle.getWheel(i);
            w.setFrictionSlip(0.32f);
            w.setMaxSuspensionForce(8f);
            w.setRestLength(0.225f);
            w.setSuspensionStiffness(10f);
            w.setWheelsDampingCompression(2.087f);
            w.setWheelsDampingRelaxation(2.845f);
        }

        Vector3f startLocation = new Vector3f(startX, 0.382268f, startZ);
        vehicle.setPhysicsLocation(startLocation);
        physicsSpace.addCollisionObject(vehicle);

        // Simulate a single timestep.
        vehicle.accelerate(2, 415.71f);
        vehicle.accelerate(3, 415.71f);
        physicsSpace.update(1f / 60, 0);
        Vector3f velocity = vehicle.getLinearVelocity();

        physicsSpace.destroy();

        return velocity.z;
    }
}
