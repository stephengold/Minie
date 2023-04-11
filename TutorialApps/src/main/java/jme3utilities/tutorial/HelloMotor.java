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
package jme3utilities.tutorial;

import com.jme3.app.SimpleApplication;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.RotationOrder;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.debug.DebugInitListener;
import com.jme3.bullet.joints.New6Dof;
import com.jme3.bullet.joints.motors.MotorParam;
import com.jme3.bullet.joints.motors.RotationMotor;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.input.KeyInput;
import com.jme3.input.controls.ActionListener;
import com.jme3.input.controls.InputListener;
import com.jme3.input.controls.KeyTrigger;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.material.Materials;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.shadow.DirectionalLightShadowRenderer;
import com.jme3.shadow.EdgeFilteringMode;
import com.jme3.system.AppSettings;

/**
 * A simple example of a PhysicsJoint with a motor.
 * <p>
 * Builds upon HelloLimit.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class HelloMotor extends SimpleApplication {
    // *************************************************************************
    // fields

    /**
     * PhysicsSpace for simulation
     */
    private static PhysicsSpace physicsSpace;
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the HelloMotor application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        HelloMotor application = new HelloMotor();

        // Enable gamma correction for accurate lighting.
        boolean loadDefaults = true;
        AppSettings settings = new AppSettings(loadDefaults);
        settings.setGammaCorrection(true);
        application.setSettings(settings);

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
        physicsSpace = configurePhysics();

        // Add a dynamic, green frame.
        PhysicsRigidBody frameBody = addFrame();

        // Add a dynamic, yellow box for the door.
        PhysicsRigidBody doorBody = addDoor();

        // Add a double-ended physics joint to join the door to the frame.
        Vector3f pivotLocation = new Vector3f(-1f, 0f, 0f);
        Quaternion pivotOrientation = Quaternion.IDENTITY;
        New6Dof joint = New6Dof.newInstance(frameBody, doorBody,
                pivotLocation, pivotOrientation, RotationOrder.XYZ);
        physicsSpace.addJoint(joint);

        int xRotationDof = 3 + PhysicsSpace.AXIS_X;
        int yRotationDof = 3 + PhysicsSpace.AXIS_Y;
        int zRotationDof = 3 + PhysicsSpace.AXIS_Z;

        // Lock the X and Z rotation DOFs.
        joint.set(MotorParam.LowerLimit, xRotationDof, 0f);
        joint.set(MotorParam.LowerLimit, zRotationDof, 0f);
        joint.set(MotorParam.UpperLimit, xRotationDof, 0f);
        joint.set(MotorParam.UpperLimit, zRotationDof, 0f);

        // Limit the Y rotation DOF.
        joint.set(MotorParam.LowerLimit, yRotationDof, 0f);
        joint.set(MotorParam.UpperLimit, yRotationDof, 1.2f);

        // Enable the motor for Y rotation.
        final RotationMotor motor = joint.getRotationMotor(PhysicsSpace.AXIS_Y);
        motor.set(MotorParam.TargetVelocity, 0.4f);
        motor.setMotorEnabled(true);

        // Configure the InputManager to respond to the spacebar.
        inputManager.addMapping("reverse", new KeyTrigger(KeyInput.KEY_SPACE));
        InputListener actionListener = new ActionListener() {
            @Override
            public void onAction(String action, boolean ongoing, float tpf) {
                if (action.equals("reverse") && ongoing) {
                    // Reverse the motor's direction.
                    float rate = motor.get(MotorParam.TargetVelocity);
                    motor.set(MotorParam.TargetVelocity, -rate);
                }
            }
        };
        inputManager.addListener(actionListener, "reverse");
    }
    // *************************************************************************
    // private methods

    /**
     * Create a dynamic rigid body with a box shape and add it to the space.
     *
     * @return the new body
     */
    private PhysicsRigidBody addDoor() {
        BoxCollisionShape shape = new BoxCollisionShape(0.8f, 0.8f, 0.1f);

        float mass = 0.2f;
        PhysicsRigidBody result = new PhysicsRigidBody(shape, mass);
        physicsSpace.addCollisionObject(result);

        // Disable sleep (deactivation).
        result.setEnableSleep(false);

        Material yellowMaterial = createLitMaterial(1f, 1f, 0f);
        result.setDebugMaterial(yellowMaterial);

        return result;
    }

    /**
     * Create a dynamic body with a square-frame shape and add it to the space.
     *
     * @return the new body
     */
    private PhysicsRigidBody addFrame() {
        CapsuleCollisionShape xShape
                = new CapsuleCollisionShape(0.1f, 2f, PhysicsSpace.AXIS_X);
        CapsuleCollisionShape yShape
                = new CapsuleCollisionShape(0.1f, 2f, PhysicsSpace.AXIS_Y);

        CompoundCollisionShape frameShape = new CompoundCollisionShape();
        frameShape.addChildShape(xShape, 0f, +1f, 0f);
        frameShape.addChildShape(xShape, 0f, -1f, 0f);
        frameShape.addChildShape(yShape, +1f, 0f, 0f);
        frameShape.addChildShape(yShape, -1f, 0f, 0f);

        PhysicsRigidBody result = new PhysicsRigidBody(frameShape);
        physicsSpace.addCollisionObject(result);

        Material greenMaterial = createLitMaterial(0f, 1f, 0f);
        result.setDebugMaterial(greenMaterial);

        return result;
    }

    /**
     * Add lighting and shadows to the specified scene and set the background
     * color.
     *
     * @param scene the scene to augment (not null)
     */
    private void addLighting(Spatial scene) {
        ColorRGBA ambientColor = new ColorRGBA(0.03f, 0.03f, 0.03f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        scene.addLight(ambient);
        ambient.setName("ambient");

        ColorRGBA directColor = new ColorRGBA(0.2f, 0.2f, 0.2f, 1f);
        Vector3f direction = new Vector3f(-7f, -3f, -5f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction, directColor);
        scene.addLight(sun);
        sun.setName("sun");

        // Render shadows based on the directional light.
        viewPort.clearProcessors();
        int shadowMapSize = 2_048; // in pixels
        int numSplits = 3;
        DirectionalLightShadowRenderer dlsr
                = new DirectionalLightShadowRenderer(
                        assetManager, shadowMapSize, numSplits);
        dlsr.setEdgeFilteringMode(EdgeFilteringMode.PCFPOISSON);
        dlsr.setEdgesThickness(5);
        dlsr.setLight(sun);
        dlsr.setShadowIntensity(0.6f);
        viewPort.addProcessor(dlsr);

        // Set the viewport's background color to light blue.
        ColorRGBA skyColor = new ColorRGBA(0.1f, 0.2f, 0.4f, 1f);
        viewPort.setBackgroundColor(skyColor);
    }

    /**
     * Position the camera during startup.
     */
    private void configureCamera() {
        flyCam.setMoveSpeed(5f);

        cam.setLocation(new Vector3f(0f, 1.5f, 4f));
        cam.setRotation(new Quaternion(0.003f, 0.98271f, -0.1846f, 0.014f));
    }

    /**
     * Configure physics during startup.
     *
     * @return a new instance (not null)
     */
    private PhysicsSpace configurePhysics() {
        BulletAppState bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);

        // Enable debug visualization to reveal what occurs in physics space.
        bulletAppState.setDebugEnabled(true);

        // Add lighting and shadows to the debug scene.
        bulletAppState.setDebugInitListener(new DebugInitListener() {
            @Override
            public void bulletDebugInit(Node physicsDebugRootNode) {
                addLighting(physicsDebugRootNode);
            }
        });
        bulletAppState.setDebugShadowMode(
                RenderQueue.ShadowMode.CastAndReceive);

        PhysicsSpace result = bulletAppState.getPhysicsSpace();
        result.setGravity(Vector3f.ZERO);

        return result;
    }

    /**
     * Create a single-sided lit material with the specified reflectivities.
     *
     * @param red the desired reflectivity for red light (&ge;0, &le;1)
     * @param green the desired reflectivity for green light (&ge;0, &le;1)
     * @param blue the desired reflectivity for blue light (&ge;0, &le;1)
     * @return a new instance (not null)
     */
    private Material createLitMaterial(float red, float green, float blue) {
        Material result = new Material(assetManager, Materials.LIGHTING);
        result.setBoolean("UseMaterialColors", true);

        float opacity = 1f;
        result.setColor("Ambient", new ColorRGBA(red, green, blue, opacity));
        result.setColor("Diffuse", new ColorRGBA(red, green, blue, opacity));

        return result;
    }
}
