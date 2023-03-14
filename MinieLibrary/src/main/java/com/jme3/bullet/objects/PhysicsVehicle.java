/*
 * Copyright (c) 2009-2018 jMonkeyEngine
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of 'jMonkeyEngine' nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package com.jme3.bullet.objects;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.objects.infos.RigidBodyMotionState;
import com.jme3.bullet.objects.infos.VehicleController;
import com.jme3.bullet.objects.infos.VehicleTuning;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Vector3f;
import com.jme3.scene.Spatial;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A rigid body for simplified vehicle simulation based on Bullet's
 * {@code btRaycastVehicle}.
 * <p>
 * The wheels of a PhysicsVehicle aren't collision objects, so the vehicle's
 * ignore list doesn't affect them.
 * <p>
 * <i>From Bullet manual:</i><br>
 * For arcade style vehicle simulations, it is recommended to use the simplified
 * Bullet vehicle model as provided in btRaycastVehicle. Instead of simulation
 * each wheel and chassis as separate rigid bodies, connected by constraints, it
 * uses a simplified model. This simplified model has many benefits, and is
 * widely used in commercial driving games.
 * <p>
 * The entire vehicle is represented as a single rigid body, the chassis. The
 * collision detection of the wheels is approximated by ray casts, and the tire
 * friction is a basic anisotropic friction model.
 *
 * @author normenhansen
 */
public class PhysicsVehicle extends PhysicsRigidBody {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger3
            = Logger.getLogger(PhysicsVehicle.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagTuning = "tuning";
    final private static String tagWheelsList = "wheelsList";
    // *************************************************************************
    // fields

    /**
     * list of wheels
     */
    private ArrayList<VehicleWheel> wheels = new ArrayList<>(6);
    /**
     * controller or "action" for this vehicle
     */
    private VehicleController controller;
    /**
     * tuning parameters applied when a wheel is created
     */
    private VehicleTuning tuning = new VehicleTuning();
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected PhysicsVehicle() {
    }

    /**
     * Instantiate a responsive vehicle with the specified CollisionShape and
     * mass=1.
     *
     * @param shape the desired shape of the chassis (not null, alias created)
     */
    public PhysicsVehicle(CollisionShape shape) {
        super(shape);
    }

    /**
     * Instantiate a responsive vehicle with the specified CollisionShape and
     * mass.
     *
     * @param shape the desired shape of the chassis (not null, alias created)
     * @param mass the desired mass of the chassis (&gt;0, default=1)
     */
    public PhysicsVehicle(CollisionShape shape, float mass) {
        super(shape, mass);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Apply the specified engine force to each wheel. Works continuously. The
     * vehicle must be added to a PhysicsSpace.
     *
     * @param force the desired amount of force (may be negative)
     */
    public void accelerate(float force) {
        assert isInWorld();

        for (VehicleWheel wheel : wheels) {
            controller.applyEngineForce(wheel, force);
        }
    }

    /**
     * Apply the specified engine force to the indexed wheel. Works
     * continuously. The vehicle must be added to a PhysicsSpace.
     *
     * @param wheelIndex the index of the wheel to apply the force to (&ge;0,
     * &lt;count)
     * @param force the desired amount of force (may be negative)
     */
    public void accelerate(int wheelIndex, float force) {
        Validate.inRange(wheelIndex, "wheel index", 0, wheels.size() - 1);
        assert isInWorld();

        VehicleWheel wheel = wheels.get(wheelIndex);
        controller.applyEngineForce(wheel, force);
    }

    /**
     * Add a wheel to this vehicle.
     *
     * @param subtree the scene-graph subtree for visualization (alias created)
     * or null for none
     * @param connectionPoint the location where the suspension connects to the
     * chassis (in chassis coordinates, not null, unaffected)
     * @param direction the suspension direction (in chassis coordinates, not
     * null, unaffected, typically down/0,-1,0)
     * @param axle the axis direction (in chassis coordinates, not null,
     * unaffected, typically -1,0,0)
     * @param suspensionRestLength the rest length of the suspension (in
     * physics-space units)
     * @param wheelRadius the wheel radius (in physics-space units, &gt;0)
     * @param isFrontWheel true&rarr;front (steering) wheel,
     * false&rarr;non-front wheel
     * @return a new VehicleWheel for access (not null)
     */
    public VehicleWheel addWheel(Spatial subtree, Vector3f connectionPoint,
            Vector3f direction, Vector3f axle, float suspensionRestLength,
            float wheelRadius, boolean isFrontWheel) {
        Validate.positive(wheelRadius, "wheel radius");

        VehicleWheel result = new VehicleWheel(subtree, connectionPoint,
                direction, axle, suspensionRestLength, wheelRadius,
                isFrontWheel);

        result.setFrictionSlip(tuning.getFrictionSlip());
        result.setMaxSuspensionTravelCm(tuning.getMaxSuspensionTravelCm());
        result.setSuspensionStiffness(tuning.getSuspensionStiffness());
        result.setWheelsDampingCompression(tuning.getSuspensionCompression());
        result.setWheelsDampingRelaxation(tuning.getSuspensionDamping());
        result.setMaxSuspensionForce(tuning.getMaxSuspensionForce());
        wheels.add(result);

        if (controller != null) {
            long controllerId = controller.nativeId();
            int wheelIndex = controller.addWheel(result, tuning);
            result.setVehicleId(controllerId, wheelIndex);
            assert result.checkCopies();
        }

        return result;
    }

    /**
     * For compatibility with the jme3-jbullet library.
     *
     * @param connectionPoint the location where the suspension connects to the
     * chassis (in chassis coordinates, not null, unaffected)
     * @param direction the suspension direction (in chassis coordinates, not
     * null, unaffected, typically down/0,-1,0)
     * @param axle the axis direction (in chassis coordinates, not null,
     * unaffected, typically -1,0,0)
     * @param suspensionRestLength the rest length of the suspension (in
     * physics-space units)
     * @param wheelRadius the wheel radius (in physics-space units, &gt;0)
     * @param isFrontWheel true&rarr;front (steering) wheel,
     * false&rarr;non-front wheel
     * @return a new VehicleWheel for access (not null)
     */
    public VehicleWheel addWheel(Vector3f connectionPoint,
            Vector3f direction, Vector3f axle, float suspensionRestLength,
            float wheelRadius, boolean isFrontWheel) {
        Spatial subtree = null;
        VehicleWheel result = addWheel(subtree, connectionPoint, direction,
                axle, suspensionRestLength, wheelRadius, isFrontWheel);

        return result;
    }

    /**
     * Used internally.
     */
    public void applyWheelTransforms() {
        if (wheels != null) {
            for (VehicleWheel wheel : wheels) {
                wheel.applyWheelTransform();
            }
        }
    }

    /**
     * Apply the specified brake impulse to all wheels. Works continuously. The
     * vehicle must be added to a PhysicsSpace.
     *
     * @param impulse the desired impulse
     */
    public void brake(float impulse) {
        assert isInWorld();

        for (VehicleWheel wheel : wheels) {
            controller.brake(wheel, impulse);
            assert wheel.getBrake() == impulse;
        }
    }

    /**
     * Apply the specified brake impulse to the indexed wheel. Works
     * continuously. The vehicle must be added to a PhysicsSpace.
     *
     * @param wheelIndex the index of the wheel to apply the impulse to (&ge;0,
     * &lt;count)
     * @param impulse the desired impulse
     */
    public void brake(int wheelIndex, float impulse) {
        Validate.inRange(wheelIndex, "wheel index", 0, wheels.size() - 1);
        assert isInWorld();

        VehicleWheel wheel = wheels.get(wheelIndex);
        controller.brake(wheel, impulse);
    }

    /**
     * Compute depth for the indexed wheel by raycasting. The vehicle must be
     * added to a PhysicsSpace.
     *
     * @param wheelIndex the index of the wheel to raycast (&ge;0, &lt;count)
     * @return the depth value, or -1 if the raycast finds no result
     */
    public float castRay(int wheelIndex) {
        Validate.inRange(wheelIndex, "wheel index", 0, wheels.size() - 1);
        assert isInWorld();

        VehicleWheel wheel = wheels.get(wheelIndex);
        float result = controller.castRay(wheel);
        return result;
    }

    /**
     * Used internally, creates the controller when the vehicle is added to a
     * PhysicsSpace.
     *
     * @param space the PhysicsSpace to use, or null for none
     */
    public void createVehicle(PhysicsSpace space) {
        if (space == null) {
            return;
        }
        if (isInWorld()) {
            assert getCollisionSpace() == space;
        }

        this.controller = new VehicleController(this, space);
        logger3.log(Level.FINE, "Created {0}", controller);

        controller.setCoordinateSystem(PhysicsSpace.AXIS_X,
                PhysicsSpace.AXIS_Y, PhysicsSpace.AXIS_Z);

        long controllerId = controller.nativeId();
        for (VehicleWheel wheel : wheels) {
            int wheelIndex = controller.addWheel(wheel, tuning);
            wheel.setVehicleId(controllerId, wheelIndex);
            assert wheel.checkCopies();
            wheel.setSuspensionLength(0f);
        }
    }

    /**
     * Determine the index of this vehicle's forward axis. The vehicle must be
     * added to a PhysicsSpace.
     *
     * @return the index of the local axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z,
     * -1&rarr;custom axes
     */
    public int forwardAxisIndex() {
        assert isInWorld();
        int result = controller.forwardAxisIndex();
        return result;
    }

    /**
     * Access this vehicle's controller.
     *
     * @return the pre-existing instance, or null if never added to a
     * PhysicsSpace
     */
    public VehicleController getController() {
        return controller;
    }

    /**
     * Determine this vehicle's forward velocity as of the previous time step.
     * The vehicle must be added to a PhysicsSpace.
     *
     * Note that the units are kilometers per hour ONLY if the physics-space
     * unit is exactly one meter.
     *
     * @return the forward component of the linear velocity (in physics-space
     * units per 3.6 seconds, may be negative)
     */
    public float getCurrentVehicleSpeedKmHour() {
        assert isInWorld();
        float result = controller.getCurrentVehicleSpeedKmHour();
        return result;
    }

    /**
     * Determine this vehicle's forward direction. The vehicle must be added to
     * a PhysicsSpace.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a direction vector (in physics-space coordinates, either
     * storeResult or a new vector, not null)
     */
    public Vector3f getForwardVector(Vector3f storeResult) {
        Vector3f result = controller.getForwardVector(storeResult);
        return result;
    }

    /**
     * Determine the initial friction for new wheels.
     *
     * @return the coefficient of friction between tire and ground
     * (0.8&rarr;realistic car, 10000&rarr;kart racer)
     */
    public float getFrictionSlip() {
        float result = tuning.getFrictionSlip();
        return result;
    }

    /**
     * Determine the initial maximum suspension force for new wheels.
     *
     * @return the maximum force per wheel
     */
    public float getMaxSuspensionForce() {
        float result = tuning.getMaxSuspensionForce();
        return result;
    }

    /**
     * Determine the initial maximum suspension travel distance for new wheels.
     *
     * Note that the units are centimeters ONLY if the physics-space unit is
     * exactly one meter.
     *
     * @return the maximum amount a suspension can be compressed or expanded,
     * relative to its rest length (in hundredths of a physics-space unit)
     */
    public float getMaxSuspensionTravelCm() {
        float result = tuning.getMaxSuspensionTravelCm();
        return result;
    }

    /**
     * Count the number of wheels on this vehicle.
     *
     * @return count (&ge;0)
     */
    public int getNumWheels() {
        assert checkNumWheels();
        int result = wheels.size();
        return result;
    }

    /**
     * Determine the initial damping (when the suspension is compressed) for new
     * wheels.
     *
     * @return the damping coefficient
     */
    public float getSuspensionCompression() {
        float result = tuning.getSuspensionCompression();
        return result;
    }

    /**
     * Determine the initial damping (when the suspension is expanded) for new
     * wheels.
     *
     * @return the damping coefficient
     */
    public float getSuspensionDamping() {
        float result = tuning.getSuspensionDamping();
        return result;
    }

    /**
     * Determine the initial suspension stiffness for new wheels.
     *
     * @return the stiffness constant (10&rarr;off-road buggy, 50&rarr;sports
     * car, 200&rarr;Formula-1 race car)
     */
    public float getSuspensionStiffness() {
        float result = tuning.getSuspensionStiffness();
        return result;
    }

    /**
     * Access the tuning parameters applied when a wheel is created.
     *
     * @return the pre-existing instance (not null)
     */
    public VehicleTuning getTuning() {
        return tuning;
    }

    /**
     * Used internally.
     *
     * @return the unique identifier (not zero)
     */
    public long getVehicleId() {
        long result = controller.nativeId();
        return result;
    }

    /**
     * Access the indexed wheel of this vehicle.
     *
     * @param wheelIndex the index of the wheel to access (&ge;0, &lt;count)
     * @return the pre-existing instance
     */
    public VehicleWheel getWheel(int wheelIndex) {
        VehicleWheel result = wheels.get(wheelIndex);
        return result;
    }

    /**
     * Remove a wheel. TODO test this---with joints!
     *
     * @param wheelIndex the index of the wheel to remove (&ge;0, &lt;count)
     */
    public void removeWheel(int wheelIndex) {
        wheels.remove(wheelIndex);
        rebuildRigidBody();
        //Bullet has no API to remove a wheel.
    }

    /**
     * Reset this vehicle's suspension. The vehicle must be added to a
     * PhysicsSpace.
     */
    public void resetSuspension() {
        controller.resetSuspension();
    }

    /**
     * Determine the index of this vehicle's right-side axis. The vehicle must
     * be added to a PhysicsSpace.
     *
     * @return the index of the local axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z,
     * -1&rarr;custom axes
     */
    public int rightAxisIndex() {
        int result = controller.rightAxisIndex();
        return result;
    }

    /**
     * Alter the initial friction for new wheels. Effective only before adding
     * wheels. After adding a wheel, use {@link #setFrictionSlip(int, float)}.
     * <p>
     * For better handling, increase the friction.
     *
     * @param frictionSlip the desired coefficient of friction between tire and
     * ground (0.8&rarr;realistic car, 10000&rarr;kart racer, default=10.5)
     */
    public void setFrictionSlip(float frictionSlip) {
        tuning.setFrictionSlip(frictionSlip);
    }

    /**
     * Alter the friction of the indexed wheel.
     * <p>
     * For better handling, increase the friction.
     *
     * @param wheelIndex the index of the wheel to modify (&ge;0, &lt;count)
     * @param frictionSlip the desired coefficient of friction between tire and
     * ground (0.8&rarr;realistic car, 10000&rarr;kart racer, default=10.5)
     */
    public void setFrictionSlip(int wheelIndex, float frictionSlip) {
        VehicleWheel wheel = wheels.get(wheelIndex);
        wheel.setFrictionSlip(frictionSlip);
    }

    /**
     * Alter the initial maximum suspension force for new wheels. Effective only
     * before adding wheels. After adding a wheel, use
     * {@link #setMaxSuspensionForce(int, float)}.
     * <p>
     * If the suspension cannot handle the vehicle's weight, increase this
     * limit.
     *
     * @param maxSuspensionForce the desired maximum force per wheel
     * (default=6000)
     */
    public void setMaxSuspensionForce(float maxSuspensionForce) {
        tuning.setMaxSuspensionForce(maxSuspensionForce);
    }

    /**
     * Alter the maximum suspension force for the specified wheel.
     * <p>
     * If the suspension cannot handle the vehicle's weight, increase this
     * limit.
     *
     * @param wheelIndex the index of the wheel to modify (&ge;0, &lt;count)
     * @param maxSuspensionForce the desired maximum force per wheel
     * (default=6000)
     */
    public void setMaxSuspensionForce(
            int wheelIndex, float maxSuspensionForce) {
        VehicleWheel wheel = wheels.get(wheelIndex);
        wheel.setMaxSuspensionForce(maxSuspensionForce);
    }

    /**
     * Alter the initial maximum suspension travel distance for new wheels.
     * Effective only before adding wheels. After adding a wheel, use
     * {@link #setMaxSuspensionTravelCm(int, float)}.
     *
     * Note that the units are centimeters ONLY if the physics-space unit is
     * exactly one meter.
     *
     * @param maxSuspensionTravelCm the desired maximum amount a suspension can
     * be compressed or expanded, relative to its rest length (in hundredths of
     * a physics-space unit, default=500)
     */
    public void setMaxSuspensionTravelCm(float maxSuspensionTravelCm) {
        tuning.setMaxSuspensionTravelCm(maxSuspensionTravelCm);
    }

    /**
     * Alter the maximum suspension travel distance for the indexed wheel.
     *
     * Note that the units are centimeters ONLY if the physics-space unit is
     * exactly one meter.
     *
     * @param wheelIndex the index of the wheel to modify (&ge;0, &lt;count)
     * @param maxSuspensionTravelCm the desired maximum amount the suspension
     * can be compressed or expanded, relative to its rest length (in hundredths
     * of a physics-space unit, default=500)
     */
    public void setMaxSuspensionTravelCm(
            int wheelIndex, float maxSuspensionTravelCm) {
        VehicleWheel wheel = wheels.get(wheelIndex);
        wheel.setMaxSuspensionTravelCm(maxSuspensionTravelCm);
    }

    /**
     * Alter the roll influence of the indexed wheel.
     * <p>
     * The roll-influence factor reduces (or magnifies) any torque contributed
     * by the wheel that would tend to cause the vehicle to roll over. This is a
     * bit of a hack, but it's quite effective.
     * <p>
     * If the friction between the tires and the ground is too high, you may
     * reduce this factor to prevent the vehicle from rolling over. You should
     * also try lowering the vehicle's center of mass.
     *
     * @param wheelIndex the index of the wheel to modify (&ge;0, &lt;count)
     * @param rollInfluence the desired roll-influence factor (0&rarr;no roll
     * torque, 1&rarr;realistic behavior, default=1)
     */
    public void setRollInfluence(int wheelIndex, float rollInfluence) {
        VehicleWheel wheel = wheels.get(wheelIndex);
        wheel.setRollInfluence(rollInfluence);
    }

    /**
     * Alter the initial damping (when the suspension is compressed) for new
     * wheels. Effective only before adding wheels. After adding a wheel, use
     * {@link #setSuspensionCompression(int, float)}.
     * <p>
     * Set to k * 2 * FastMath.sqrt(m_suspensionStiffness) where k is the
     * damping ratio:
     * <p>
     * k = 0.0 undamped and bouncy, k = 1.0 critical damping, k between 0.1 and
     * 0.3 are good values
     *
     * @param coefficient the desired damping coefficient (default=0.83)
     */
    public void setSuspensionCompression(float coefficient) {
        tuning.setSuspensionCompression(coefficient);
    }

    /**
     * Alter the damping (when the suspension is compressed) for the indexed
     * wheel.
     * <p>
     * Set to k * 2 * FastMath.sqrt(m_suspensionStiffness) where k is the
     * damping ratio:
     * <p>
     * k = 0.0 undamped and bouncy, k = 1.0 critical damping, k between 0.1 and
     * 0.3 are good values
     *
     * @param wheelIndex the index of the wheel to modify (&ge;0, &lt;count)
     * @param coefficient the desired damping coefficient (default=0.83)
     */
    public void setSuspensionCompression(int wheelIndex, float coefficient) {
        VehicleWheel wheel = wheels.get(wheelIndex);
        wheel.setWheelsDampingCompression(coefficient);
    }

    /**
     * Alter the initial damping (when the suspension is expanded) for new
     * wheels. Effective only before adding wheels. After adding a wheel, use
     * {@link #setSuspensionCompression(int, float)}.
     * <p>
     * Set to k * 2 * FastMath.sqrt(m_suspensionStiffness) where k is the
     * damping ratio:
     * <p>
     * k = 0.0 undamped and bouncy, k = 1.0 critical damping, k between 0.1 and
     * 0.3 are good values
     *
     * @param coefficient the desired damping coefficient (default=0.88)
     */
    public void setSuspensionDamping(float coefficient) {
        tuning.setSuspensionDamping(coefficient);
    }

    /**
     * Alter the damping (when the suspension is expanded) for the indexed
     * wheel.
     * <p>
     * Set to k * 2 * FastMath.sqrt(m_suspensionStiffness) where k is the
     * damping ratio:
     * <p>
     * k = 0.0 undamped and bouncy, k = 1.0 critical damping, k between 0.1 and
     * 0.3 are good values
     *
     * @param wheelIndex the index of the wheel to modify (&ge;0, &lt;count)
     * @param coefficient the desired damping coefficient (default=0.88)
     */
    public void setSuspensionDamping(int wheelIndex, float coefficient) {
        VehicleWheel wheel = wheels.get(wheelIndex);
        wheel.setWheelsDampingRelaxation(coefficient);
    }

    /**
     * Alter the initial suspension stiffness for new wheels. Effective only
     * before adding wheels. After adding a wheel, use
     * {@link #setSuspensionStiffness(int, float)}.
     *
     * @param coefficient the desired stiffness coefficient (10&rarr;off-road
     * buggy, 50&rarr;sports car, 200&rarr;Formula-1 race car, default=5.88)
     */
    public void setSuspensionStiffness(float coefficient) {
        tuning.setSuspensionStiffness(coefficient);
    }

    /**
     * Alter the suspension stiffness of the indexed wheel.
     *
     * @param wheelIndex the index of the wheel to modify (&ge;0, &lt;count)
     * @param coefficient the desired stiffness coefficient (10&rarr;off-road
     * buggy, 50&rarr;sports car, 200&rarr;Formula-1 race car, default=5.88)
     */
    public void setSuspensionStiffness(int wheelIndex, float coefficient) {
        VehicleWheel wheel = wheels.get(wheelIndex);
        wheel.setSuspensionStiffness(coefficient);
    }

    /**
     * Alter the steering angle of all front wheels. The vehicle must be added
     * to a PhysicsSpace.
     *
     * @param angle the desired angle (in radians, 0=straight, positive=left)
     */
    public void steer(float angle) {
        for (VehicleWheel wheel : wheels) {
            if (wheel.isFrontWheel()) {
                controller.steer(wheel, angle);
            }
        }
    }

    /**
     * Alter the steering angle of the indexed wheel. The vehicle must be added
     * to a PhysicsSpace.
     *
     * @param wheelIndex the index of the wheel to steer (&ge;0, &lt;count)
     * @param angle the desired angle (in radians, 0=straight, positive=left)
     */
    public void steer(int wheelIndex, float angle) {
        Validate.inRange(wheelIndex, "wheel index", 0, wheels.size() - 1);
        assert isInWorld();

        VehicleWheel wheel = wheels.get(wheelIndex);
        controller.steer(wheel, angle);
    }

    /**
     * Determine the index of this vehicle's up axis. The vehicle must be added
     * to a PhysicsSpace.
     *
     * @return the index of the local axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z,
     * -1&rarr;custom axes
     */
    public int upAxisIndex() {
        int result = controller.upAxisIndex();
        return result;
    }

    /**
     * Used internally.
     */
    public void updateWheels() {
        if (controller != null) {
            for (VehicleWheel wheel : wheels) {
                controller.updateWheelTransform(wheel);
            }
        }
    }
    // *************************************************************************
    // PhysicsRigidBody methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned body into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this vehicle (not null)
     * @param original the instance from which this vehicle was shallow-cloned
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        assert !hasAssignedNativeObject();
        PhysicsVehicle old = (PhysicsVehicle) original;
        assert old != this;
        assert old.hasAssignedNativeObject();

        super.cloneFields(cloner, original);
        if (hasAssignedNativeObject()) {
            return;
        }

        RigidBodyMotionState motionState = getMotionState();
        motionState.setVehicle(this);

        this.wheels = cloner.clone(wheels);
        this.tuning = cloner.clone(tuning);
    }

    /**
     * De-serialize this vehicle from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    @SuppressWarnings("unchecked")
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);
        InputCapsule capsule = importer.getCapsule(this);

        this.tuning = (VehicleTuning) capsule.readSavable(tagTuning, null);
        this.wheels = capsule
                .readSavableArrayList(tagWheelsList, new ArrayList<>(6));

        RigidBodyMotionState motionState = getMotionState();
        motionState.setVehicle(this);
    }

    /**
     * Invoked during a rebuild after the native object is created.
     * <p>
     * For use by subclasses.
     */
    @Override
    protected void postRebuild() {
        super.postRebuild();

        RigidBodyMotionState motionState = getMotionState();
        motionState.setVehicle(this);

        PhysicsSpace space = (PhysicsSpace) getCollisionSpace();
        createVehicle(space);
        // TODO re-create any joints
    }

    /**
     * Serialize this vehicle to the specified exporter, for example when saving
     * to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        super.write(exporter);
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(tuning, tagTuning, null);
        capsule.writeSavableArrayList(wheels, tagWheelsList, null);
    }
    // *************************************************************************
    // private methods

    /**
     * Compare Bullet's wheel count to the local copy.
     *
     * @return true if the counts are exactly equal, otherwise false
     */
    private boolean checkNumWheels() {
        boolean result = true;
        if (controller != null) {
            int size = wheels.size();
            int count = controller.countWheels();
            result = (size == count);
        }

        return result;
    }
}
