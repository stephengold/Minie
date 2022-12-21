/*
 * Copyright (c) 2020-2022 jMonkeyEngine
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
package com.jme3.bullet.joints;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.RotationOrder;
import com.jme3.bullet.joints.motors.MotorParam;
import com.jme3.bullet.joints.motors.RotationMotor;
import com.jme3.bullet.joints.motors.TranslationMotor;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.FastMath;
import com.jme3.math.Matrix3f;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Logger;

/**
 * A 3 degree-of-freedom Constraint that mimics ODE's Hinge2 joint, such as
 * might be used to model one of the front wheels on a motor vehicle.
 * <p>
 * Rotation is enabled only for the Z and X axes, with limits on Z-axis
 * rotation. The X axis rotates freely. Translation is enabled only for the Z
 * axis, with a suspension spring.
 * <p>
 * Inspired by Bullet's btHinge2Constraint.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class NewHinge extends New6Dof {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger3
            = Logger.getLogger(NewHinge.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagAxis1 = "axis1";
    final private static String tagAxis2 = "axis2";
    // *************************************************************************
    // fields

    /**
     * initial direction of the constraint's Z axis (in physics-space
     * coordinates)
     */
    private Vector3f axis1;
    /**
     * initial direction of the constraint's X axis (in physics-space
     * coordinates)
     */
    private Vector3f axis2;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected NewHinge() {
    }

    /**
     * Instantiate a double-ended constraint.
     * <p>
     * To be effective, the constraint must be added to the PhysicsSpace of both
     * bodies. Also, the bodies must be distinct and at least one of them must
     * be dynamic.
     *
     * @param rigidBodyA the body for the A end (not null, alias created)
     * @param rigidBodyB the body for the B end (not null, alias created)
     * @param anchor initial anchor location for the constraint frame (in
     * physics-space coordinates, not null, unaffected)
     * @param axis1 initial Z-axis direction for the constraint frame (in
     * physics-space coordinates, not null, not zero, unaffected)
     * @param axis2 initial X-axis direction for the constraint frame (in
     * physics-space coordinates, not null, not zero, unaffected)
     */
    public NewHinge(PhysicsRigidBody rigidBodyA, PhysicsRigidBody rigidBodyB,
            Vector3f anchor, Vector3f axis1, Vector3f axis2) {
        super(rigidBodyA, rigidBodyB,
                pivotInBody(rigidBodyA, anchor),
                pivotInBody(rigidBodyB, anchor),
                rotInBody(rigidBodyA, axis1, axis2),
                rotInBody(rigidBodyB, axis1, axis2),
                RotationOrder.XYZ);
        this.axis1 = axis1.clone();
        this.axis2 = axis2.clone();

        // Configure the limits as in btHinge2Constraint.cpp .
        TranslationMotor translation = super.getTranslationMotor();
        translation.set(MotorParam.LowerLimit, new Vector3f(0f, 0f, -1f));
        translation.set(MotorParam.UpperLimit, new Vector3f(0f, 0f, 1f));
        setLowerLimit(-FastMath.PI / 4f);
        setUpperLimit(FastMath.PI / 4f);

        // Configure the suspension spring as in btHinge2Constraint.cpp .
        super.enableSpring(PhysicsSpace.AXIS_Z, true);
        super.set(MotorParam.Damping, PhysicsSpace.AXIS_Z, 0.01f);
        float stiffness = 4f * FastMath.PI * FastMath.PI;
        super.set(MotorParam.Stiffness, PhysicsSpace.AXIS_Z, stiffness);
        super.setEquilibriumPoint();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Determine the anchor location for body A.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the location (either storeResult or a new vector, not null)
     */
    public Vector3f getAnchor(Vector3f storeResult) {
        Vector3f result = calculatedOriginA(storeResult);
        return result;
    }

    /**
     * Determine the anchor location for body B.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the location (either storeResult or a new vector, not null)
     */
    public Vector3f getAnchor2(Vector3f storeResult) {
        Vector3f result = calculatedOriginB(storeResult);
        return result;
    }

    /**
     * Determine the rotation angle for axis1.
     *
     * @return the rotation angle (in radians)
     */
    public float getAngle1() {
        Vector3f angles = getAngles(null);
        float result = angles.z;

        return result;
    }

    /**
     * Determine the rotation angle for axis2.
     *
     * @return the rotation angle (in radians)
     */
    public float getAngle2() {
        Vector3f angles = getAngles(null);
        float result = angles.x;

        return result;
    }

    /**
     * Determine the direction of axis1.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a direction vector (either storeResult or a new vector, not null)
     */
    public Vector3f getAxis1(Vector3f storeResult) {
        Vector3f result;
        if (storeResult == null) {
            result = axis1.clone();
        } else {
            result = storeResult.set(axis1);
        }

        return result;
    }

    /**
     * Determine the direction of axis2.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a direction vector (either storeResult or a new vector, not null)
     */
    public Vector3f getAxis2(Vector3f storeResult) {
        Vector3f result;
        if (storeResult == null) {
            result = axis2.clone();
        } else {
            result = storeResult.set(axis2);
        }

        return result;
    }

    /**
     * Alter the lower limit for axis1 rotation.
     *
     * @param angle1Min the desired angle (in radians)
     */
    final public void setLowerLimit(float angle1Min) {
        RotationMotor xMotor = getRotationMotor(PhysicsSpace.AXIS_X);
        xMotor.set(MotorParam.LowerLimit, 1f);

        RotationMotor yMotor = getRotationMotor(PhysicsSpace.AXIS_Y);
        yMotor.set(MotorParam.LowerLimit, 0f);

        RotationMotor zMotor = getRotationMotor(PhysicsSpace.AXIS_Z);
        zMotor.set(MotorParam.LowerLimit, angle1Min);
    }

    /**
     * Alter the upper limit for axis1 rotation.
     *
     * @param angle1Max the desired angle (in radians)
     */
    final public void setUpperLimit(float angle1Max) {
        RotationMotor xMotor = getRotationMotor(PhysicsSpace.AXIS_X);
        xMotor.set(MotorParam.UpperLimit, -1f);

        RotationMotor yMotor = getRotationMotor(PhysicsSpace.AXIS_Y);
        yMotor.set(MotorParam.UpperLimit, 0f);

        RotationMotor zMotor = getRotationMotor(PhysicsSpace.AXIS_Z);
        zMotor.set(MotorParam.UpperLimit, angle1Max);
    }
    // *************************************************************************
    // New6Dof methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned constraint into a deep-cloned one, using the specified
     * Cloner and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this constraint (not null)
     * @param original the instance from which this constraint was
     * shallow-cloned (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        assert !hasAssignedNativeObject();
        NewHinge old = (NewHinge) original;
        assert old != this;
        assert old.hasAssignedNativeObject();

        super.cloneFields(cloner, original);
        if (hasAssignedNativeObject()) {
            return;
        }

        this.axis1 = cloner.clone(axis1);
        this.axis2 = cloner.clone(axis2);
    }

    /**
     * De-serialize this Constraint from the specified importer, for example
     * when loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);
        InputCapsule capsule = importer.getCapsule(this);

        this.axis1 = (Vector3f) capsule.readSavable(tagAxis1, null);
        this.axis2 = (Vector3f) capsule.readSavable(tagAxis2, null);
    }

    /**
     * Serialize this Constraint to the specified exporter, for example when
     * saving to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        super.write(exporter);
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(axis1, tagAxis1, null);
        capsule.write(axis2, tagAxis2, null);
    }
    // *************************************************************************
    // private methods

    /**
     * Determine the pivot location in a body's scaled local coordinates.
     *
     * @param body the body to use (not null, unaffected)
     * @param anchor the location in physics-space coordinates (not null,
     * unaffected)
     * @return a new location vector (in scaled local coordinates)
     */
    private static Vector3f
            pivotInBody(PhysicsRigidBody body, Vector3f anchor) {
        Transform bodyToWorld = body.getTransform(null);
        bodyToWorld.setScale(1f);
        Vector3f result = bodyToWorld.transformInverseVector(anchor, null);

        return result;
    }

    /**
     * Determine the orientation of the constraint in a body's local
     * coordinates.
     *
     * @param body the body to use (not null, unaffected)
     * @param axis1 (not null, unaffected)
     * @param axis2 (not null, unaffected)
     * @return a new rotation matrix (in local coordinates)
     */
    private static Matrix3f
            rotInBody(PhysicsRigidBody body, Vector3f axis1, Vector3f axis2) {
        Vector3f zAxis = axis1.normalize();
        Vector3f xAxis = axis2.normalize();
        Vector3f yAxis = zAxis.cross(xAxis);

        Matrix3f frameInW = new Matrix3f();
        frameInW.fromAxes(xAxis, yAxis, zAxis);

        // Calculate constraint frame rotation in the body's local coordinates.
        Matrix3f rotation = body.getPhysicsRotationMatrix(null); // b2w
        rotation.invert(null); // w2b
        Matrix3f result = rotation.mult(frameInW, null);

        return result;
    }
}
