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

import com.jme3.app.SimpleApplication;
import com.jme3.asset.TextureKey;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.PlaneCollisionShape;
import com.jme3.bullet.joints.NewHinge;
import com.jme3.bullet.joints.motors.MotorParam;
import com.jme3.bullet.joints.motors.RotationMotor;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.util.PlaneDmiListener;
import com.jme3.material.Material;
import com.jme3.material.Materials;
import com.jme3.math.FastMath;
import com.jme3.math.Plane;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Limits;
import com.jme3.texture.Texture;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/**
 * An example of vehicle physics using NewHinge.
 * <p>
 * Builds upon HelloVehicle.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class HelloNewHinge
        extends SimpleApplication
        implements PhysicsTickListener {
    // *************************************************************************
    // fields

    /**
     * wheels for steering
     */
    final private static List<RotationMotor> steer = new ArrayList<>(2);
    /**
     * drive wheels
     */
    final private static List<PhysicsRigidBody> drive = new ArrayList<>(2);
    private static PhysicsRigidBody chassis;
    /**
     * PhysicsSpace for simulation
     */
    private static PhysicsSpace physicsSpace;
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the HelloNewHinge application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        HelloNewHinge application = new HelloNewHinge();
        application.start();
    }
    // *************************************************************************
    // SimpleApplication methods

    /**
     * Initialize this application.
     */
    @Override
    public void simpleInitApp() {
        physicsSpace = configurePhysics();

        // Create a wedge-shaped vehicle with a low center of gravity.
        // The local forward direction is +Z.
        float noseZ = 1.4f;           // offset from chassis center
        float spoilerY = 0.5f;        // offset from chassis center
        float tailZ = -0.7f;          // offset from chassis center
        float undercarriageY = -0.1f; // offset from chassis center
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
        float mass = 5f;
        chassis = new PhysicsRigidBody(wedgeShape, mass);
        chassis.setEnableSleep(false);
        physicsSpace.addCollisionObject(chassis);

        // Add 4 wheels, 2 in the front (for steering) and 2 in the rear.
        boolean front = true;
        boolean rear = false;
        float frontAxisZ = 0.7f * noseZ; // offset from chassis center
        float rearAxisZ = 0.8f * tailZ; // offset from chassis center
        float radius = 0.3f; // of each tire
        float restLength = 0.2f; // of the suspension
        float xOffset = 0.9f * halfWidth;
        Vector3f axleDirection = new Vector3f(-1f, 0f, 0f);
        Vector3f suspensionDirection = new Vector3f(0f, -1f, 0f);
        addWheel(new Vector3f(-xOffset, 0f, frontAxisZ),
                suspensionDirection, axleDirection, restLength, radius, front);
        addWheel(new Vector3f(xOffset, 0f, frontAxisZ),
                suspensionDirection, axleDirection, restLength, radius, front);
        addWheel(new Vector3f(-xOffset, 0f, rearAxisZ),
                suspensionDirection, axleDirection, restLength, radius, rear);
        addWheel(new Vector3f(xOffset, 0f, rearAxisZ),
                suspensionDirection, axleDirection, restLength, radius, rear);

        // Apply a steering angle of 6 degrees left (to the front wheels).
        for (RotationMotor motor : steer) {
            motor.set(MotorParam.ServoTarget, FastMath.PI / 30f);
        }

        // Add a static plane to represent the ground.
        float y = -radius - 0.35f;
        addPlane(y);
    }
    // *************************************************************************
    // PhysicsTickListener methods

    /**
     * Callback from Bullet, invoked just before each simulation step.
     *
     * @param space the space that's about to be stepped (not null)
     * @param timeStep the time per simulation step (in seconds, &ge;0)
     */
    @Override
    public void prePhysicsTick(PhysicsSpace space, float timeStep) {
        // Apply a constant torque (to the rear wheels).
        for (PhysicsRigidBody wheel : drive) {
            Vector3f torque = new Vector3f(1f, 0f, 0f);
            wheel.getPhysicsRotation().mult(torque, torque);
            wheel.applyTorque(torque);
        }
    }

    /**
     * Callback from Bullet, invoked just after each simulation step.
     *
     * @param space the space that was just stepped (not null)
     * @param timeStep the time per simulation step (in seconds, &ge;0)
     */
    @Override
    public void physicsTick(PhysicsSpace space, float timeStep) {
        // do nothing
    }
    // *************************************************************************
    // private methods

    /**
     * Add a horizontal plane body to the space.
     *
     * @param y (the desired elevation, in physics-space coordinates)
     */
    private void addPlane(float y) {
        Plane plane = new Plane(Vector3f.UNIT_Y, y);
        PlaneCollisionShape shape = new PlaneCollisionShape(plane);
        PhysicsRigidBody body
                = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);

        // Load a repeating tile texture.
        String assetPath = "Textures/greenTile.png";
        boolean flipY = false;
        TextureKey key = new TextureKey(assetPath, flipY);
        boolean generateMips = true;
        key.setGenerateMips(generateMips);
        Texture texture = assetManager.loadTexture(key);
        texture.setMinFilter(Texture.MinFilter.Trilinear);
        texture.setWrap(Texture.WrapMode.Repeat);

        // Enable anisotropic filtering, to reduce blurring.
        Integer maxDegree = renderer.getLimits().get(Limits.TextureAnisotropy);
        int degree = (maxDegree == null) ? 1 : Math.min(8, maxDegree);
        texture.setAnisotropicFilter(degree);

        // Apply a tiled, unshaded debug material to the body.
        Material material = new Material(assetManager, Materials.UNSHADED);
        material.setTexture("ColorMap", texture);
        body.setDebugMaterial(material);

        // Generate texture coordinates during debug-mesh initialization.
        float tileSize = 1f;
        PlaneDmiListener planeDmiListener = new PlaneDmiListener(tileSize);
        body.setDebugMeshInitListener(planeDmiListener);

        physicsSpace.addCollisionObject(body);
    }

    /**
     * Add a cylindrical wheel, joined to the chassis by a NewHinge.
     *
     * @param connectionPoint the location of the connection point (not null)
     * @param suspensionDirection the direction of suspension motion (not null,
     * not zero, unaffected)
     * @param axle the direction of the axle's axis (not null, not zero,
     * unaffected)
     * @param restLength the rest length
     * @param wheelRadius the desired radius of the wheel
     * @param isFrontWheel true for a front/steer wheel, false for a rear/drive
     * wheel
     */
    private void addWheel(Vector3f connectionPoint,
            Vector3f suspensionDirection, Vector3f axle, float restLength,
            float wheelRadius, boolean isFrontWheel) {
        float thickness = 0.5f * wheelRadius;
        CylinderCollisionShape shape = new CylinderCollisionShape(
                wheelRadius, thickness, PhysicsSpace.AXIS_X);
        float mass = 0.5f;
        PhysicsRigidBody body = new PhysicsRigidBody(shape, mass);
        body.setEnableSleep(false);
        Vector3f center = connectionPoint.add(0f, -restLength, 0f);
        body.setPhysicsLocation(center);
        physicsSpace.addCollisionObject(body);

        NewHinge joint = new NewHinge(
                chassis, body, center, suspensionDirection, axle);
        if (isFrontWheel) {
            RotationMotor motor = joint.getRotationMotor(PhysicsSpace.AXIS_Z);
            motor.setMotorEnabled(true);
            motor.setServoEnabled(true);
            motor.set(MotorParam.TargetVelocity, 1f);
            steer.add(motor);
        } else {
            joint.set(MotorParam.LowerLimit, 3 + PhysicsSpace.AXIS_Z, 0f);
            joint.set(MotorParam.UpperLimit, 3 + PhysicsSpace.AXIS_Z, 0f);
            drive.add(body);
        }
        joint.set(MotorParam.Damping, PhysicsSpace.AXIS_Z, 30f);
        joint.set(MotorParam.Stiffness, PhysicsSpace.AXIS_Z, 90f);
        joint.setCollisionBetweenLinkedBodies(false);
        physicsSpace.addJoint(joint);
    }

    /**
     * Configure physics during startup.
     *
     * @return a new instance (not null)
     */
    private PhysicsSpace configurePhysics() {
        BulletAppState bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);
        bulletAppState.setDebugEnabled(true); // for debug visualization
        PhysicsSpace result = bulletAppState.getPhysicsSpace();

        // To enable the callbacks, register the application as a tick listener.
        result.addTickListener(this);

        return result;
    }
}
