/*
 * Copyright (c) 2009-2023 jMonkeyEngine
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
package com.jme3.bullet.objects.infos;

import com.jme3.bullet.NativePhysicsObject;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.objects.PhysicsVehicle;
import com.jme3.bullet.objects.VehicleWheel;
import com.jme3.math.Vector3f;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * The "action" (controller) portion of a PhysicsVehicle, based on Bullet's
 * btRaycastVehicle.
 *
 * @author Stephen Gold sgold@sonic.net
 *
 * Based on PhysicsVehicle by normenhansen.
 */
public class VehicleController extends NativePhysicsObject {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(VehicleController.class.getName());
    // *************************************************************************
    // fields

    /**
     * chassis collision object that's being controlled
     */
    final private PhysicsVehicle pco;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a controller for the specified collision object.
     *
     * @param vehicle the collision object to control (not null)
     * @param space the PhysicsSpace where the collision object is or will be
     * added (not null)
     */
    public VehicleController(PhysicsVehicle vehicle, PhysicsSpace space) {
        Validate.nonNull(vehicle, "vehicle");
        Validate.nonNull(space, "space");

        this.pco = vehicle;

        long spaceId = space.nativeId();
        long rigidBodyId = vehicle.nativeId();
        VehicleTuning tuning = vehicle.getTuning();
        long tuningId = tuning.nativeId();
        long controllerId
                = createRaycastVehicle(spaceId, rigidBodyId, tuningId);

        super.setNativeId(controllerId);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Add a wheel to this controller. Used internally.
     *
     * @param wheel (not null, unaffected)
     * @param tuning (not null, unaffected)
     * @return the wheel index to be assigned (&ge;0)
     */
    public int addWheel(VehicleWheel wheel, VehicleTuning tuning) {
        Vector3f location = wheel.getLocation(null);
        Vector3f direction = wheel.getDirection(null);
        Vector3f axle = wheel.getAxle(null);
        float restLength = wheel.getRestLength();
        float radius = wheel.getRadius();
        boolean isFrontWheel = wheel.isFrontWheel();
        long controllerId = nativeId();
        long tuningId = tuning.nativeId();
        int result = addWheel(controllerId, location, direction, axle,
                restLength, radius, tuningId, isFrontWheel);

        return result;
    }

    /**
     * Apply the specified engine force to the specified wheel. Works
     * continuously. The vehicle must be added to a PhysicsSpace.
     *
     * @param wheel which wheel (not null)
     * @param force the desired amount of force (may be negative)
     */
    public void applyEngineForce(VehicleWheel wheel, float force) {
        assert pco.isInWorld();

        long controllerId = nativeId();
        int wheelIndex = wheel.getIndex();
        applyEngineForce(controllerId, wheelIndex, force);

        assert wheel.getEngineForce() == force : wheel.getEngineForce();
    }

    /**
     * Apply the specified brake impulse to the specified wheel. Works
     * continuously. The vehicle must be added to a PhysicsSpace.
     *
     * @param wheel which wheel (not null)
     * @param impulse the desired impulse
     */
    public void brake(VehicleWheel wheel, float impulse) {
        assert pco.isInWorld();

        long controllerId = nativeId();
        int wheelIndex = wheel.getIndex();
        brake(controllerId, wheelIndex, impulse);

        assert wheel.getBrake() == impulse : wheel.getBrake();
    }

    /**
     * Determine the depth for the specified wheel by raycasting. The vehicle
     * must be added to a PhysicsSpace.
     *
     * @param wheel which wheel (not null, unaffected)
     * @return the depth value, or -1 if the raycast finds no result
     */
    public float castRay(VehicleWheel wheel) {
        assert pco.isInWorld();

        long controllerId = nativeId();
        int wheelIndex = wheel.getIndex();
        float result = rayCast(controllerId, wheelIndex);

        return result;
    }

    /**
     * Determine the number of wheels in this controller.
     *
     * @return the count (&ge;0)
     */
    public int countWheels() {
        long controllerId = nativeId();
        int result = getNumWheels(controllerId);

        return result;
    }

    /**
     * Determine the index of the vehicle's forward axis. The vehicle must be
     * added to a PhysicsSpace.
     *
     * @return the index of the local axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z,
     * -1&rarr;custom axes
     */
    public int forwardAxisIndex() {
        assert pco.isInWorld();

        long controllerId = nativeId();
        int result = getForwardAxisIndex(controllerId);

        return result;
    }

    /**
     * Determine the vehicle's speed in km/h. The vehicle must be added to a
     * PhysicsSpace.
     *
     * @return the speed (in kilometers per hour, positive in the forward
     * direction)
     */
    public float getCurrentVehicleSpeedKmHour() {
        assert pco.isInWorld();

        long controllerId = nativeId();
        float result = getCurrentVehicleSpeedKmHour(controllerId);

        return result;
    }

    /**
     * Determine the vehicle's forward direction. The vehicle must be added to a
     * PhysicsSpace.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a direction vector (in physics-space coordinates, either
     * storeResult or a new vector, not null)
     */
    public Vector3f getForwardVector(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        assert pco.isInWorld();

        long controllerId = nativeId();
        getForwardVector(controllerId, result);

        return result;
    }

    /**
     * Reset the vehicle's suspension. The vehicle must be added to a
     * PhysicsSpace.
     */
    public void resetSuspension() {
        assert pco.isInWorld();
        long controllerId = nativeId();
        resetSuspension(controllerId);
    }

    /**
     * Determine the index of the vehicle's right-side axis. The vehicle must be
     * added to a PhysicsSpace.
     *
     * @return the index of the local axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z,
     * -1&rarr;custom axes
     */
    public int rightAxisIndex() {
        assert pco.isInWorld();

        long controllerId = nativeId();
        int result = getRightAxisIndex(controllerId);

        return result;
    }

    /**
     * Alter the coordinate system of the vehicle.
     *
     * Note that the Libbulletjme's default coordinate system is left-handed.
     * That's not required, nor is it even a good idea.
     *
     * @param rightAxisIndex the desired local axis index (0&rarr;X, 1&rarr;Y,
     * 2&rarr;Z, default=0)
     * @param upAxisIndex the desired local axis index (0&rarr;X, 1&rarr;Y,
     * 2&rarr;Z, default=1)
     * @param forwardAxisIndex the desired local axis index (0&rarr;X, 1&rarr;Y,
     * 2&rarr;Z, default=2)
     */
    public void setCoordinateSystem(
            int rightAxisIndex, int upAxisIndex, int forwardAxisIndex) {
        Validate.axisIndex(rightAxisIndex, "right axis");
        Validate.axisIndex(upAxisIndex, "up axis");
        Validate.axisIndex(forwardAxisIndex, "forward axis");

        long controllerId = nativeId();
        setCoordinateSystem(controllerId, rightAxisIndex, upAxisIndex,
                forwardAxisIndex);
    }

    /**
     * Customize the coordinate system of the vehicle. The arguments must form
     * an orthonormal basis.
     *
     * Note that the Libbulletjme's default coordinate system is left-handed.
     * That's not required, nor is it even a good idea.
     *
     * @param right the desired direction (in chassis coordinates, not null,
     * length=1, default=(1,0,0))
     * @param up the desired direction (in chassis coordinates, not null,
     * length=1, default=(0,1,0))
     * @param forward the desired direction (in chassis coordinates, not null,
     * length=1, default=(0,0,1))
     */
    public void
            setCoordinateSystem(Vector3f right, Vector3f up, Vector3f forward) {
        Validate.nonNull(right, "right");
        Validate.nonNull(right, "up");
        Validate.nonNull(right, "forward");

        long controllerId = nativeId();
        setupCoordinateSystem(controllerId, right, up, forward);
    }

    /**
     * Alter the steering angle of the specified wheel. The vehicle must be
     * added to a PhysicsSpace.
     *
     * @param wheel which wheel to steer (not null)
     * @param angle the desired angle (in radians, 0=straight, positive=left)
     */
    public void steer(VehicleWheel wheel, float angle) {
        assert pco.isInWorld();

        long controllerId = nativeId();
        int wheelIndex = wheel.getIndex();
        steer(controllerId, wheelIndex, angle);
    }

    /**
     * Determine the index of the vehicle's up axis. The vehicle must be added
     * to a PhysicsSpace.
     *
     * @return the index of the local axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z,
     * -1&rarr;custom axes
     */
    public int upAxisIndex() {
        assert pco.isInWorld();

        long controllerId = nativeId();
        int result = getUpAxisIndex(controllerId);

        return result;
    }

    /**
     * Used internally.
     *
     * @param wheel (not null)
     */
    public void updateWheelTransform(VehicleWheel wheel) {
        long controllerId = nativeId();
        int wheelIndex = wheel.getIndex();
        boolean interpolate = true;
        updateWheelTransform(controllerId, wheelIndex, interpolate);

        wheel.updatePhysicsState();
    }
    // *************************************************************************
    // Java private methods

    /**
     * Free the identified tracked native object. Invoked by reflection.
     *
     * @param controllerId the native identifier (not zero)
     */
    private static void freeNativeObject(long controllerId) {
        assert controllerId != 0L;
        finalizeNative(controllerId);
    }
    // *************************************************************************
    // native private methods

    native private static int addWheel(long controllerId, Vector3f location,
            Vector3f direction, Vector3f axle, float restLength, float radius,
            long tuningId, boolean frontWheel);

    native private static void
            applyEngineForce(long controllerId, int wheelIndex, float force);

    native private static void
            brake(long controllerId, int wheelIndex, float impulse);

    native private static long
            createRaycastVehicle(long spaceId, long bodyId, long tuningId);

    native private static void finalizeNative(long controllerId);

    native private static float getCurrentVehicleSpeedKmHour(long controllerId);

    native private static int getForwardAxisIndex(long controllerId);

    native private static void
            getForwardVector(long controllerId, Vector3f storeResult);

    native private static int getNumWheels(long controllerId);

    native private static int getRightAxisIndex(long controllerId);

    native private static int getUpAxisIndex(long controllerId);

    native private static float rayCast(long controllerId, int wheelIndex);

    native private static void resetSuspension(long controllerId);

    native private static void setCoordinateSystem(long controllerId,
            int rightAxisIndex, int upAxisIndex, int forwardAxisIndex);

    native private static void setupCoordinateSystem(
            long controllerId, Vector3f right, Vector3f up, Vector3f forward);

    native private static void
            steer(long controllerId, int wheelIndex, float angle);

    native private static void updateWheelTransform(
            long controllerId, int wheelIndex, boolean interpolated);
}
