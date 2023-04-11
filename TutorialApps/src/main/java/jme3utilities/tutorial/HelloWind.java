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
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.bullet.SoftPhysicsAppState;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.bullet.objects.infos.Aero;
import com.jme3.bullet.objects.infos.Sbcp;
import com.jme3.bullet.objects.infos.SoftBodyConfig;
import com.jme3.bullet.objects.infos.SoftBodyMaterial;
import com.jme3.bullet.util.NativeSoftBodyUtil;
import com.jme3.input.KeyInput;
import com.jme3.input.controls.ActionListener;
import com.jme3.input.controls.InputListener;
import com.jme3.input.controls.KeyTrigger;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import jme3utilities.math.MyMath;
import jme3utilities.mesh.ClothGrid;
import jme3utilities.minie.FilterAll;

/**
 * A simple cloth simulation with wind.
 * <p>
 * Builds upon HelloPin.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class HelloWind
        extends SimpleApplication
        implements PhysicsTickListener {
    // *************************************************************************
    // constants

    /**
     * wind speed, in psu per second
     */
    final private static float windSpeed = 3f;
    // *************************************************************************
    // fields

    /**
     * true when the left-arrow key is pressed, otherwise false
     */
    private static volatile boolean turnLeft;
    /**
     * true when the right-arrow key is pressed, otherwise false
     */
    private static volatile boolean turnRight;
    /**
     * wind direction (in radians from +X)
     */
    private static float windAzimuth = -0.8f;
    /**
     * collision object for the flag
     */
    private static PhysicsSoftBody flag;

    final private static Vector3f tmpVelocity = new Vector3f();
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the HelloWind application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        HelloWind application = new HelloWind();
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
        configureInput();

        // Set the viewport's background color to light blue.
        ColorRGBA skyColor = new ColorRGBA(0.1f, 0.2f, 0.4f, 1f);
        viewPort.setBackgroundColor(skyColor);

        // Set up Bullet physics (with debug enabled).
        SoftPhysicsAppState bulletAppState = new SoftPhysicsAppState();
        stateManager.attach(bulletAppState);
        bulletAppState.setDebugEnabled(true); // for debug visualization
        bulletAppState.setWindVelocityFilter(new FilterAll(true));
        PhysicsSoftSpace physicsSpace = bulletAppState.getPhysicsSoftSpace();

        physicsSpace.setAccuracy(0.01f); // 10-msec timestep

        Vector3f gravityVector = new Vector3f(0f, -1f, 0f);
        physicsSpace.setGravity(gravityVector);

        // To enable the callbacks, register the application as a tick listener.
        physicsSpace.addTickListener(this);

        // Generate a subdivided rectangle mesh with alternating diagonals.
        int xLines = 20;
        int zLines = 2 * xLines; // 2x as wide as it is tall
        float width = 2f;
        float lineSpacing = width / zLines;
        Mesh mesh = new ClothGrid(xLines, zLines, lineSpacing);

        // Create a soft rectangle for the flag.
        flag = new PhysicsSoftBody();
        NativeSoftBodyUtil.appendFromTriMesh(mesh, flag);
        flag.setMargin(0.1f);
        flag.setMass(1f);

        // Pin the left edge of the flag.
        int nodeIndex = 0; // upper left corner
        flag.setNodeMass(nodeIndex, PhysicsBody.massForStatic);
        nodeIndex = xLines - 1; // lower left corner
        flag.setNodeMass(nodeIndex, PhysicsBody.massForStatic);
        /*
         * Make the flag flexible by reducing the angular stiffness
         * of its material.
         */
        SoftBodyMaterial softMaterial = flag.getSoftMaterial();
        softMaterial.setAngularStiffness(0f);

        // Configure the flag's aerodynamics.
        SoftBodyConfig config = flag.getSoftConfig();
        config.setAerodynamics(Aero.F_TwoSidedLiftDrag);
        config.set(Sbcp.Damping, 0.01f); // default = 0
        config.set(Sbcp.Drag, 0.5f); // default = 0.2
        config.set(Sbcp.Lift, 1f); // default = 0
        /*
         * Improve simulation accuracy by increasing
         * the number of position-solver iterations for the flag.
         */
        config.setPositionIterations(3);

        Quaternion rotation = new Quaternion();
        rotation.fromAngles(FastMath.HALF_PI, 0f, 0f);
        flag.applyRotation(rotation);

        // Initialize the wind velocity.
        tmpVelocity.x = windSpeed * FastMath.cos(windAzimuth);
        tmpVelocity.z = windSpeed * FastMath.sin(windAzimuth);
        flag.setWindVelocity(tmpVelocity);

        flag.setDebugNumSides(2);
        physicsSpace.addCollisionObject(flag);
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
        // Update the flag's wind velocity.
        if (turnLeft) {
            windAzimuth -= timeStep;
        }
        if (turnRight) {
            windAzimuth += timeStep;
        }
        windAzimuth = MyMath.standardizeAngle(windAzimuth);
        tmpVelocity.x = windSpeed * FastMath.cos(windAzimuth);
        tmpVelocity.z = windSpeed * FastMath.sin(windAzimuth);
        flag.setWindVelocity(tmpVelocity);
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
     * Configure the Camera during startup.
     */
    private void configureCamera() {
        cam.setLocation(new Vector3f(7f, 1.2f, -0.7f));
        cam.setRotation(new Quaternion(0.08619f, -0.68974f, 0.0833f, 0.71407f));
    }

    /**
     * Configure keyboard input during startup.
     */
    private void configureInput() {
        inputManager.addMapping("left", new KeyTrigger(KeyInput.KEY_F1));
        inputManager.addMapping("right", new KeyTrigger(KeyInput.KEY_F2));
        InputListener input = new ActionListener() {
            @Override
            public void onAction(String action, boolean isPressed, float tpf) {
                switch (action) {
                    case "left":
                        turnLeft = isPressed;
                        return;

                    case "right":
                        turnRight = isPressed;
                        return;

                    default:
                        System.out.println("Unknown action: " + action);
                }
            }
        };
        inputManager.addListener(input, "left", "right");
    }
}
