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
 * A collision object for simplified vehicle simulation based on Bullet's
 * btRaycastVehicle.
 * <p>
 * <i>From Bullet manual:</i><br>
 * For arcade style vehicle simulations, it is recommended to use the simplified
 * Bullet vehicle model as provided in btRaycastVehicle. Instead of simulation
 * each wheel and chassis as separate rigid bodies, connected by constraints, it
 * uses a simplified model. This simplified model has many benefits, and is
 * widely used in commercial driving games.
 * <p>
 * The entire vehicle is represented as a single rigidbody, the chassis. The
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
    final private static String tagFrictionSlip = "frictionSlip";
    final private static String tagMaxSuspensionForce = "maxSuspensionForce";
    final private static String tagMaxSuspensionTravelCm
            = "maxSuspensionTravelCm";
    final private static String tagSuspensionCompression
            = "suspensionCompression";
    final private static String tagSuspensionDamping = "suspensionDamping";
    final private static String tagSuspensionStiffness = "suspensionStiffness";
    final private static String tagWheelsList = "wheelsList";
    // *************************************************************************
    // fields

    /**
     * list of wheels
     */
    protected ArrayList<VehicleWheel> wheels = new ArrayList<>(6);
    /**
     * Unique identifier of the ray caster. createVehicle() sets this to a
     * non-zero value. The ID will change if the object gets rebuilt.
     */
    private long rayCasterId = 0L;
    /**
     * Unique identifier of the btRaycastVehicle. createVehicle() sets this to a
     * non-zero value. The ID will change if the object gets rebuilt.
     */
    private long vehicleId = 0L;
    /**
     * space where this vehicle is added, or null if none
     */
    private PhysicsSpace physicsSpace;
    /**
     * tuning parameters applied when a wheel is created
     */
    protected VehicleTuning tuning = new VehicleTuning();
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public PhysicsVehicle() {
    }

    /**
     * Instantiate a responsive vehicle with the specified CollisionShape and
     * mass=1.
     *
     * @param shape the desired shape (not null, alias created)
     */
    public PhysicsVehicle(CollisionShape shape) {
        super(shape);
    }

    /**
     * Instantiate a responsive vehicle with the specified CollisionShape and
     * mass.
     *
     * @param shape the desired shape (not null, alias created)
     * @param mass (&gt;0)
     */
    public PhysicsVehicle(CollisionShape shape, float mass) {
        super(shape, mass);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Apply the specified engine force to all wheels. Works continuously. The
     * vehicle must be added to a PhysicsSpace.
     *
     * @param force the desired amount of force (may be negative)
     */
    public void accelerate(float force) {
        long vid = getVehicleId();
        for (int wheelIndex = 0; wheelIndex < wheels.size(); ++wheelIndex) {
            applyEngineForce(vid, wheelIndex, force);
        }
    }

    /**
     * Apply the given engine force to the indexed wheel. Works continuously.
     * The vehicle must be added to a PhysicsSpace.
     *
     * @param wheelIndex the index of the wheel to apply the force to (&ge;0,
     * &lt;count)
     * @param force the desired amount of force (may be negative)
     */
    public void accelerate(int wheelIndex, float force) {
        Validate.inRange(wheelIndex, "wheel index", 0, wheels.size());

        long vid = getVehicleId();
        applyEngineForce(vid, wheelIndex, force);
    }

    /**
     * Add a wheel to this vehicle.
     *
     * @param spat the associated spatial, or null if none
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
    public VehicleWheel addWheel(Spatial spat, Vector3f connectionPoint,
            Vector3f direction, Vector3f axle, float suspensionRestLength,
            float wheelRadius, boolean isFrontWheel) {
        Validate.positive(wheelRadius, "wheel radius");

        VehicleWheel wheel = new VehicleWheel(spat, connectionPoint, direction,
                axle, suspensionRestLength, wheelRadius, isFrontWheel);

        wheel.setFrictionSlip(tuning.getFrictionSlip());
        wheel.setMaxSuspensionTravelCm(tuning.getMaxSuspensionTravelCm());
        wheel.setSuspensionStiffness(tuning.getSuspensionStiffness());
        wheel.setWheelsDampingCompression(tuning.getSuspensionCompression());
        wheel.setWheelsDampingRelaxation(tuning.getSuspensionDamping());
        wheel.setMaxSuspensionForce(tuning.getMaxSuspensionForce());
        wheels.add(wheel);

        if (vehicleId != 0L) {
            long tuningId = tuning.getNativeId();
            int index = addWheel(vehicleId, wheel.getLocation(null),
                    wheel.getDirection(null), wheel.getAxle(null),
                    wheel.getRestLength(), wheel.getRadius(), tuningId,
                    wheel.isFrontWheel());
            wheel.setVehicleId(vehicleId, index);
            assert wheel.checkCopies();
        }

        return wheel;
    }

    /**
     * For compatibility with the jme3-bullet library.
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
        return addWheel(null, connectionPoint, direction, axle,
                suspensionRestLength, wheelRadius, isFrontWheel);
    }

    /**
     * used internally
     */
    public void applyWheelTransforms() {
        if (wheels != null) {
            for (VehicleWheel wheel : wheels) {
                wheel.applyWheelTransform();
            }
        }
    }

    /**
     * Apply the given brake impulse to all wheels. Works continuously.
     *
     * @param impulse the desired impulse
     */
    public void brake(float impulse) {
        for (int wheelIndex = 0; wheelIndex < wheels.size(); ++wheelIndex) {
            brake(wheelIndex, impulse);
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
        Validate.inRange(wheelIndex, "wheel index", 0, wheels.size());

        long vid = getVehicleId();
        brake(vid, wheelIndex, impulse);
    }

    /**
     * Compute depth for the indexed wheel by raycasting. The vehicle must be
     * added to a PhysicsSpace.
     *
     * @param wheelIndex the index of the wheel to raycast (&ge;0, &lt;count)
     * @return the depth value, or -1 if the raycast finds no result
     */
    public float castRay(int wheelIndex) {
        Validate.inRange(wheelIndex, "wheel index", 0, wheels.size());

        long vid = getVehicleId();
        return rayCast(vid, wheelIndex);
    }

    /**
     * Used internally, creates the btRaycastVehicle when vehicle is added to a
     * PhysicsSpace.
     *
     * @param space which PhysicsSpace (not zero)
     */
    public void createVehicle(PhysicsSpace space) {
        physicsSpace = space;
        if (space == null) {
            return;
        }
        long spaceId = space.getSpaceId();
        if (spaceId == 0L) {
            throw new IllegalStateException(
                    "Physics space is not initialized!");
        }
        if (rayCasterId != 0L) {
            logger3.log(Level.FINE, "Clearing RayCaster {0}",
                    Long.toHexString(rayCasterId));
            logger3.log(Level.FINE, "Clearing Vehicle {0}",
                    Long.toHexString(vehicleId));
            finalizeNative(rayCasterId, vehicleId);
        }
        rayCasterId = createVehicleRaycaster(spaceId);
        logger3.log(Level.FINE, "Created RayCaster {0}",
                Long.toHexString(rayCasterId));
        vehicleId = createRaycastVehicle(objectId, rayCasterId);
        logger3.log(Level.FINE, "Created Vehicle {0}",
                Long.toHexString(vehicleId));
        setCoordinateSystem(vehicleId, PhysicsSpace.AXIS_X,
                PhysicsSpace.AXIS_Y, PhysicsSpace.AXIS_Z);

        long tuningId = tuning.getNativeId();
        for (VehicleWheel wheel : wheels) {
            wheel.setVehicleId(vehicleId, addWheel(vehicleId,
                    wheel.getLocation(null), wheel.getDirection(null),
                    wheel.getAxle(null), wheel.getRestLength(),
                    wheel.getRadius(), tuningId, wheel.isFrontWheel()));
        }
    }

    /**
     * Read the index of the vehicle's forward axis. The vehicle must be added
     * to a PhysicsSpace.
     *
     * @return the index of the local axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     */
    public int forwardAxisIndex() {
        long vid = getVehicleId();
        int result = getForwardAxisIndex(vid);
        return result;
    }

    /**
     * Read the vehicle's speed in km/h. The vehicle must be added to a
     * PhysicsSpace.
     *
     * @return speed (in kilometers per hour, positive in the forward direction)
     */
    public float getCurrentVehicleSpeedKmHour() {
        long vid = getVehicleId();
        return getCurrentVehicleSpeedKmHour(vid);
    }

    /**
     * Copy the vehicle's forward direction. The vehicle must be added to a
     * PhysicsSpace.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a direction vector (in physics-space coordinates, either
     * storeResult or a new vector, not null)
     */
    public Vector3f getForwardVector(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        long vid = getVehicleId();
        getForwardVector(vid, result);
        return result;
    }

    /**
     * Read the initial friction for new wheels.
     *
     * @return the coefficient of friction between tire and ground
     * (0.8&rarr;realistic car, 10000&rarr;kart racer)
     */
    public float getFrictionSlip() {
        return tuning.getFrictionSlip();
    }

    /**
     * Read the initial maximum suspension force for new wheels.
     *
     * @return the maximum force per wheel
     */
    public float getMaxSuspensionForce() {
        return tuning.getMaxSuspensionForce();
    }

    /**
     * Read the initial maximum suspension travel distance for new wheels.
     *
     * @return the maximum distance the suspension can be compressed (in
     * centimeters)
     */
    public float getMaxSuspensionTravelCm() {
        return tuning.getMaxSuspensionTravelCm();
    }

    /**
     * Read the number of wheels on this vehicle.
     *
     * @return count (&ge;0)
     */
    public int getNumWheels() {
        assert checkNumWheels();
        int count = wheels.size();
        return count;
    }

    /**
     * Read the initial damping (when the suspension is compressed) for new
     * wheels.
     *
     * @return the damping coefficient
     */
    public float getSuspensionCompression() {
        return tuning.getSuspensionCompression();
    }

    /**
     * Read the initial damping (when the suspension is expanded) for new
     * wheels.
     *
     * @return the damping coefficient
     */
    public float getSuspensionDamping() {
        return tuning.getSuspensionDamping();
    }

    /**
     * Read the initial suspension stiffness for new wheels.
     *
     * @return the stiffness constant (10&rarr;off-road buggy, 50&rarr;sports
     * car, 200&rarr;Formula-1 race car)
     */
    public float getSuspensionStiffness() {
        return tuning.getSuspensionStiffness();
    }

    /**
     * used internally
     *
     * @return the unique identifier (not zero)
     */
    public long getVehicleId() {
        assert vehicleId != 0L;
        return vehicleId;
    }

    /**
     * Access the indexed wheel of this vehicle.
     *
     * @param wheelIndex the index of the wheel to access (&ge;0, &lt;count)
     * @return the pre-existing instance
     */
    public VehicleWheel getWheel(int wheelIndex) {
        return wheels.get(wheelIndex);
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
        long vid = getVehicleId();
        resetSuspension(vid);
    }

    /**
     * Read the index of the vehicle's right-side axis. The vehicle must be
     * added to a PhysicsSpace.
     *
     * @return the index of the local axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     */
    public int rightAxisIndex() {
        long vid = getVehicleId();
        int result = getRightAxisIndex(vid);
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
     * ground (0.8&rarr;realistic car, 10000&rarr;kart racer)
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
    public void setMaxSuspensionForce(int wheelIndex,
            float maxSuspensionForce) {
        VehicleWheel wheel = wheels.get(wheelIndex);
        wheel.setMaxSuspensionForce(maxSuspensionForce);
    }

    /**
     * Alter the initial maximum suspension travel distance for new wheels.
     * Effective only before adding wheels. After adding a wheel, use
     * {@link #setMaxSuspensionTravelCm(int, float)}.
     *
     * @param maxSuspensionTravelCm the desired maximum distance the suspension
     * can be compressed (in centimeters, default=500)
     */
    public void setMaxSuspensionTravelCm(float maxSuspensionTravelCm) {
        tuning.setMaxSuspensionTravelCm(maxSuspensionTravelCm);
    }

    /**
     * Alter the maximum suspension travel distance for the indexed wheel.
     *
     * @param wheelIndex the index of the wheel to modify (&ge;0, &lt;count)
     * @param maxSuspensionTravelCm the desired maximum distance the suspension
     * can be compressed (in centimeters)
     */
    public void setMaxSuspensionTravelCm(int wheelIndex,
            float maxSuspensionTravelCm) {
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
     * @param coefficient the desired damping coefficient
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
     * @param coefficient the desired damping coefficient
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
        for (int wheelIndex = 0; wheelIndex < wheels.size(); ++wheelIndex) {
            if (getWheel(wheelIndex).isFrontWheel()) {
                steer(wheelIndex, angle);
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
        Validate.inRange(wheelIndex, "wheel index", 0, wheels.size());
        long vid = getVehicleId();
        steer(vid, wheelIndex, angle);
    }

    /**
     * Read the index of the vehicle's up axis. The vehicle must be added to a
     * PhysicsSpace.
     *
     * @return the index of the local axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     */
    public int upAxisIndex() {
        long vid = getVehicleId();
        int result = getUpAxisIndex(vid);
        return result;
    }

    /**
     * used internally
     */
    public void updateWheels() {
        if (vehicleId != 0L) {
            for (int wheelIndex = 0; wheelIndex < wheels.size(); ++wheelIndex) {
                updateWheelTransform(vehicleId, wheelIndex, true);
                VehicleWheel wheel = wheels.get(wheelIndex);
                wheel.updatePhysicsState();
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
        super.cloneFields(cloner, original);
        RigidBodyMotionState ms = getMotionState();
        ms.setVehicle(this);

        //physicsSpace not cloned
        tuning = cloner.clone(tuning);
        wheels = cloner.clone(wheels);
    }

    /**
     * Finalize this vehicle just before it is destroyed. Should be invoked only
     * by a subclass or by the garbage collector.
     *
     * @throws Throwable ignored by the garbage collector
     */
    @Override
    protected void finalize() throws Throwable {
        super.finalize();

        logger3.log(Level.FINE, "Finalizing RayCaster {0}",
                Long.toHexString(rayCasterId));
        logger3.log(Level.FINE, "Finalizing Vehicle {0}",
                Long.toHexString(vehicleId));
        finalizeNative(rayCasterId, vehicleId);
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public PhysicsVehicle jmeClone() {
        try {
            PhysicsVehicle clone = (PhysicsVehicle) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
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
        InputCapsule capsule = importer.getCapsule(this);
        tuning = new VehicleTuning();

        float readFloat = capsule.readFloat(tagFrictionSlip, 10.5f);
        tuning.setFrictionSlip(readFloat);

        readFloat = capsule.readFloat(tagMaxSuspensionTravelCm, 500f);
        tuning.setMaxSuspensionTravelCm(readFloat);

        readFloat = capsule.readFloat(tagMaxSuspensionForce, 6000f);
        tuning.setMaxSuspensionForce(readFloat);

        readFloat = capsule.readFloat(tagSuspensionCompression, 0.83f);
        tuning.setSuspensionCompression(readFloat);

        readFloat = capsule.readFloat(tagSuspensionDamping, 0.88f);
        tuning.setSuspensionDamping(readFloat);

        readFloat = capsule.readFloat(tagSuspensionStiffness, 5.88f);
        tuning.setSuspensionStiffness(readFloat);

        wheels = capsule.readSavableArrayList(tagWheelsList,
                new ArrayList<VehicleWheel>(6));
        RigidBodyMotionState ms = getMotionState();
        ms.setVehicle(this);

        super.read(importer);
    }

    @Override
    protected void postRebuild() {
        super.postRebuild();
        RigidBodyMotionState ms = getMotionState();
        ms.setVehicle(this);
        createVehicle(physicsSpace);
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
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(tuning.getFrictionSlip(), tagFrictionSlip, 10.5f);
        capsule.write(tuning.getMaxSuspensionTravelCm(),
                tagMaxSuspensionTravelCm, 500f);
        capsule.write(tuning.getMaxSuspensionForce(), tagMaxSuspensionForce,
                6000f);
        capsule.write(tuning.getSuspensionCompression(),
                tagSuspensionCompression, 0.83f);
        capsule.write(tuning.getSuspensionDamping(), tagSuspensionDamping,
                0.88f);
        capsule.write(tuning.getSuspensionStiffness(), tagSuspensionStiffness,
                5.88f);
        capsule.writeSavableArrayList(wheels, tagWheelsList, null);

        super.write(exporter);
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
        if (vehicleId != 0L) {
            int size = wheels.size();
            int count = getNumWheels(vehicleId);
            result = (size == count);
        }

        return result;
    }
    // *************************************************************************
    // native methods

    native private int addWheel(long vehicleId, Vector3f location,
            Vector3f direction, Vector3f axle, float restLength, float radius,
            long tuningId, boolean frontWheel);

    native private void applyEngineForce(long vehicleId, int wheelIndex,
            float force);

    native private void brake(long vehicleId, int wheelIndex, float impulse);

    native private long createRaycastVehicle(long bodyId, long rayCasterId);

    native private long createVehicleRaycaster(long physicsSpaceId);

    native private void finalizeNative(long rayCasterId, long vehicleId);

    native private float getCurrentVehicleSpeedKmHour(long vehicleId);

    native private int getForwardAxisIndex(long vehicleId);

    native private void getForwardVector(long vehicleId, Vector3f storeResult);

    native private int getRightAxisIndex(long vehicleId);

    native private int getNumWheels(long vehicleId);

    native private int getUpAxisIndex(long vehicleId);

    native private float rayCast(long vehicleId, int wheelIndex);

    native private void resetSuspension(long vehicleId);

    native private void setCoordinateSystem(long vehicleId, int rightAxisIndex,
            int upAxisIndex, int forwardAxisIndex);

    native private void steer(long vehicleId, int wheelIndex, float angle);

    native private void updateWheelTransform(long vehicleId, int wheelIndex,
            boolean interpolated);
}
