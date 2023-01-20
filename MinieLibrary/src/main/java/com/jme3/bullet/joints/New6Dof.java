/*
 * Copyright (c) 2019-2023 jMonkeyEngine
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

import com.jme3.bullet.RotationOrder;
import com.jme3.bullet.joints.motors.MotorParam;
import com.jme3.bullet.joints.motors.RotationMotor;
import com.jme3.bullet.joints.motors.TranslationMotor;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * A 6 degree-of-freedom Constraint based on Bullet's
 * btGeneric6DofSpring2Constraint.
 * <p>
 * <i>From the Bullet manual:</i><br>
 * This generic constraint can emulate a variety of standard constraints, by
 * configuring each of the 6 degrees of freedom (dof). The first 3 dof axis are
 * linear axis, which represent translation of rigid bodies, and the latter 3
 * dof axis represent the angular motion. Each axis can be either locked, free
 * or limited.
 * <p>
 * For each axis:<ul>
 * <li>Lowerlimit = Upperlimit &rarr; axis is locked</li>
 * <li>Lowerlimit &gt; Upperlimit &rarr; axis is free</li>
 * <li>Lowerlimit &lt; Upperlimit &rarr; axis is limited in that range</li>
 * </ul>
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class New6Dof extends Constraint {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(New6Dof.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagDampingLimited = "_DampingLimited";
    final private static String tagMotorEnabled = "_MotorEnabled";
    final private static String tagRotA = "rotA";
    final private static String tagRotB = "rotB";
    final private static String tagRotOrder = "rotOrder";
    final private static String tagRotMotor = "rm";
    final private static String tagServoEnabled = "_ServoEnabled";
    final private static String tagSpringEnabled = "_SpringEnabled";
    final private static String tagStiffnessLimited = "_StiffnessLimited";
    final private static String tagTransMotor = "tm";
    // *************************************************************************
    // fields

    /**
     * copy of the constraint orientation: in physics-space coordinates if bodyA
     * is null, or else in A's local coordinates (rotation matrix)
     */
    private Matrix3f rotA;
    /**
     * copy of the constraint orientation in B's local coordinates (rotation
     * matrix)
     */
    private Matrix3f rotB;
    /**
     * rotational motor for each axis
     */
    private RotationMotor[] rotationMotor;
    /**
     * order in which to apply axis rotations (not null)
     */
    private RotationOrder rotationOrder;
    /**
     * translational motor
     */
    private TranslationMotor translationMotor;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected New6Dof() {
    }

    /**
     * Instantiate a single-ended New6Dof constraint with all rotation DOFs free
     * and all translation DOFs locked.
     * <p>
     * To be effective, the constraint must be added to the body's PhysicsSpace
     * and the body must be dynamic.
     *
     * @param rigidBodyB the body to constrain (not null, alias created)
     * @param pivotInB the pivot location in B's scaled local coordinates (not
     * null, unaffected)
     * @param pivotInWorld the pivot location in physics-space coordinates (not
     * null, unaffected)
     * @param rotInB the orientation of the constraint in B's local coordinates
     * (rotation matrix, not null, unaffected)
     * @param rotInWorld the orientation of the constraint in physics-space
     * coordinates (rotation matrix, not null, unaffected)
     * @param rotationOrder the order in which to apply axis rotations (not
     * null)
     */
    public New6Dof(PhysicsRigidBody rigidBodyB, Vector3f pivotInB,
            Vector3f pivotInWorld, Matrix3f rotInB, Matrix3f rotInWorld,
            RotationOrder rotationOrder) {
        super(rigidBodyB, JointEnd.B, pivotInB, pivotInWorld);

        this.rotA = rotInWorld.clone();
        this.rotB = rotInB.clone();
        this.rotationOrder = rotationOrder;
        createConstraint();
    }

    /**
     * Instantiate a double-ended New6Dof constraint with all rotation DOFs free
     * and all translation DOFs locked.
     * <p>
     * To be effective, the constraint must be added to the PhysicsSpace of both
     * bodies. Also, the bodies must be distinct and at least one of them must
     * be dynamic.
     *
     * @param rigidBodyA the body for the A end (not null, alias created)
     * @param rigidBodyB the body for the B end (not null, alias created)
     * @param pivotInA the pivot location in A's scaled local coordinates (not
     * null, unaffected)
     * @param pivotInB the pivot location in B's scaled local coordinates (not
     * null, unaffected)
     * @param rotInA the orientation of the constraint in A's local coordinates
     * (not null, unaffected)
     * @param rotInB the orientation of the constraint in B's local coordinates
     * (not null, unaffected)
     * @param rotationOrder the order in which to apply axis rotations (not
     * null)
     */
    public New6Dof(PhysicsRigidBody rigidBodyA, PhysicsRigidBody rigidBodyB,
            Vector3f pivotInA, Vector3f pivotInB,
            Matrix3f rotInA, Matrix3f rotInB, RotationOrder rotationOrder) {
        super(rigidBodyA, rigidBodyB, pivotInA, pivotInB);

        this.rotA = rotInA.clone();
        this.rotB = rotInB.clone();
        this.rotationOrder = rotationOrder;
        createConstraint();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Calculate the pivot orientation of the A end.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the orientation (a rotation matrix in physics-space coordinates,
     * either storeResult or a new matrix, not null)
     */
    public Matrix3f calculatedBasisA(Matrix3f storeResult) {
        Matrix3f result = (storeResult == null) ? new Matrix3f() : storeResult;

        long constraintId = nativeId();
        getCalculatedBasisA(constraintId, result);

        return result;
    }

    /**
     * Calculate the pivot orientation of the B end.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the orientation (a rotation matrix in physics-space coordinates,
     * either storeResult or a new matrix, not null)
     */
    public Matrix3f calculatedBasisB(Matrix3f storeResult) {
        Matrix3f result = (storeResult == null) ? new Matrix3f() : storeResult;

        long constraintId = nativeId();
        getCalculatedBasisB(constraintId, result);

        return result;
    }

    /**
     * Calculate the pivot location of the A end (native field:
     * m_calculatedTransformA.m_origin).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in physics-space coordinates, either
     * storeResult or a new vector, not null)
     */
    public Vector3f calculatedOriginA(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long constraintId = nativeId();
        getCalculatedOriginA(constraintId, result);

        return result;
    }

    /**
     * Calculate the pivot location of the B end (native field:
     * m_calculatedTransformB.m_origin).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in physics-space coordinates, either
     * storeResult or a new vector, not null)
     */
    public Vector3f calculatedOriginB(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long constraintId = nativeId();
        getCalculatedOriginB(constraintId, result);

        return result;
    }

    /**
     * Compare Bullet's rotation order to the local copy.
     *
     * @return true if rotation orders are identical, otherwise false
     */
    public boolean checkRotationOrder() {
        long constraintId = nativeId();
        int rotOrder = getRotationOrder(constraintId);
        boolean result = (rotOrder == rotationOrder.ordinal());

        return result;
    }

    /**
     * Enable or disable the spring for the indexed degree of freedom.
     *
     * @param dofIndex which degree of freedom (0&rarr;X translation, 1&rarr;Y
     * translation, 2&rarr;Z translation, 3&rarr;X rotation, 4&rarr;Y rotation,
     * 5&rarr;Z rotation)
     * @param onOff true &rarr; enable, false &rarr; disable (default=false)
     */
    public void enableSpring(int dofIndex, boolean onOff) {
        Validate.inRange(dofIndex, "DOF index", 0, 5);

        long constraintId = nativeId();
        enableSpring(constraintId, dofIndex, onOff);
    }

    /**
     * Read the specified parameter of the indexed degree of freedom.
     *
     * @param parameter which parameter (not null)
     * @param dofIndex which degree of freedom (0&rarr;X translation, 1&rarr;Y
     * translation, 2&rarr;Z translation, 3&rarr;X rotation, 4&rarr;Y rotation,
     * 5&rarr;Z rotation)
     * @return the parameter value
     */
    public float get(MotorParam parameter, int dofIndex) {
        Validate.inRange(dofIndex, "DOF index", 0, 5);
        float result;

        if (dofIndex >= 3) {
            int axisIndex = dofIndex - 3;
            RotationMotor motor = getRotationMotor(axisIndex);
            result = motor.get(parameter);
        } else {
            TranslationMotor motor = getTranslationMotor();
            Vector3f vector = motor.get(parameter, null); // TODO garbage
            result = vector.get(dofIndex);
        }

        return result;
    }

    /**
     * Calculate the constraint's rotation angles (native field:
     * m_calculatedAxisAngleDiff).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the rotation angle for each local axis (in radians, either
     * storeResult or a new vector, not null)
     */
    public Vector3f getAngles(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long constraintId = nativeId();
        getAngles(constraintId, result);

        return result;
    }

    /**
     * Calculate the indexed axis of the Constraint (native field:
     * m_calculatedAxis).
     *
     * @param axisIndex the axis index of the desired motor: 0&rarr;X, 1&rarr;Y,
     * 2&rarr;Z
     * @param storeResult storage for the result (modified if not null)
     * @return the rotation angle for each local axis (in radians, either
     * storeResult or a new vector, not null)
     */
    public Vector3f getAxis(int axisIndex, Vector3f storeResult) {
        Validate.axisIndex(axisIndex, "axis index");
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long constraintId = nativeId();
        getAxis(constraintId, axisIndex, result);

        return result;
    }

    /**
     * Copy the constraint's frame transform relative to the specified end
     * (native fields: m_frameInA, m_frameInB).
     *
     * @param end which end (not null)
     * @param storeResult storage for the result (modified if not null)
     * @return the transform of the constraint space relative to the end
     */
    public Transform getFrameTransform(JointEnd end, Transform storeResult) {
        Transform result
                = (storeResult == null) ? new Transform() : storeResult;

        long constraintId = nativeId();
        switch (end) {
            case A:
                getFrameOffsetA(constraintId, result);
                break;
            case B:
                getFrameOffsetB(constraintId, result);
                break;
            default:
                String message = "end = " + end;
                throw new IllegalArgumentException(message);
        }

        return result;
    }

    /**
     * Copy the constraint's pivot offset (native field:
     * m_calculatedLinearDiff).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the relative pivot position on each local axis (in physics-space
     * units, either storeResult or a new vector, not null)
     */
    public Vector3f getPivotOffset(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long constraintId = nativeId();
        getPivotOffset(constraintId, result);

        return result;
    }

    /**
     * Copy the the orientation of the constraint in the specified end's
     * coordinate system.
     *
     * @see com.jme3.bullet.joints.Constraint#getPivot(
     * com.jme3.bullet.joints.JointEnd, com.jme3.math.Vector3f)
     *
     * @param end which end (not null)
     * @param storeResult storage for the result (modified if not null)
     * @return a rotation matrix (either storeResult or a new instance)
     */
    public Matrix3f getRotationMatrix(JointEnd end, Matrix3f storeResult) {
        Validate.nonNull(end, "end");
        Matrix3f result = (storeResult == null) ? new Matrix3f() : storeResult;

        if (end == JointEnd.A) {
            result.set(rotA);
        } else {
            assert end == JointEnd.B : end;
            result.set(rotB);
        }

        return result;
    }

    /**
     * Access the indexed RotationMotor of this constraint, the motor which
     * influences rotation around the indexed axis.
     *
     * @param axisIndex the axis index of the desired motor: 0&rarr;X, 1&rarr;Y,
     * 2&rarr;Z
     * @return the pre-existing instance
     */
    public RotationMotor getRotationMotor(int axisIndex) {
        Validate.axisIndex(axisIndex, "axis index");
        return rotationMotor[axisIndex];
    }

    /**
     * Return the order in which axis rotations are applied (native field:
     * m_rotateOrder).
     *
     * @return an enum value (not null)
     */
    public RotationOrder getRotationOrder() {
        checkRotationOrder();
        assert rotationOrder != null;
        return rotationOrder;
    }

    /**
     * Access the TranslationMotor of this Constraint, the motor which
     * influences translation on all 3 axes.
     *
     * @return the pre-existing instance
     */
    public TranslationMotor getTranslationMotor() {
        return translationMotor;
    }

    /**
     * Test whether the motor is enabled for the indexed degree of freedom.
     *
     * @param dofIndex which degree of freedom (0&rarr;X translation, 1&rarr;Y
     * translation, 2&rarr;Z translation, 3&rarr;X rotation, 4&rarr;Y rotation,
     * 5&rarr;Z rotation)
     * @return true if enabled, otherwise false
     */
    public boolean isMotorEnabled(int dofIndex) {
        Validate.inRange(dofIndex, "DOF index", 0, 5);

        boolean result;
        if (dofIndex >= 3) {
            int axisIndex = dofIndex - 3;
            RotationMotor motor = getRotationMotor(axisIndex);
            result = motor.isMotorEnabled();
        } else {
            TranslationMotor motor = getTranslationMotor();
            result = motor.isMotorEnabled(dofIndex);
        }

        return result;
    }

    /**
     * Test whether the servo is enabled for the indexed degree of freedom.
     *
     * @param dofIndex which degree of freedom (0&rarr;X translation, 1&rarr;Y
     * translation, 2&rarr;Z translation, 3&rarr;X rotation, 4&rarr;Y rotation,
     * 5&rarr;Z rotation)
     * @return true if enabled, otherwise false
     */
    public boolean isServoEnabled(int dofIndex) {
        Validate.inRange(dofIndex, "DOF index", 0, 5);

        boolean result;
        if (dofIndex >= 3) {
            int axisIndex = dofIndex - 3;
            RotationMotor motor = getRotationMotor(axisIndex);
            result = motor.isServoEnabled();
        } else {
            TranslationMotor motor = getTranslationMotor();
            result = motor.isServoEnabled(dofIndex);
        }

        return result;
    }

    /**
     * Test whether the spring is enabled for the indexed degree of freedom.
     *
     * @param dofIndex which degree of freedom (0&rarr;X translation, 1&rarr;Y
     * translation, 2&rarr;Z translation, 3&rarr;X rotation, 4&rarr;Y rotation,
     * 5&rarr;Z rotation)
     * @return true if enabled, otherwise false
     */
    public boolean isSpringEnabled(int dofIndex) {
        Validate.inRange(dofIndex, "DOF index", 0, 5);

        boolean result;
        if (dofIndex >= 3) {
            int axisIndex = dofIndex - 3;
            RotationMotor motor = getRotationMotor(axisIndex);
            result = motor.isSpringEnabled();
        } else {
            TranslationMotor motor = getTranslationMotor();
            result = motor.isSpringEnabled(dofIndex);
        }

        return result;
    }

    /**
     * Instantiate a double-ended New6Dof constraint with all rotation DOFs free
     * and all translation DOFs locked at zero. The pivot calculations assume
     * all DOFs are initially at zero.
     * <p>
     * To be effective, the Constraint must be added to the PhysicsSpace of both
     * bodies. Also, the bodies must be distinct and at least one of them must
     * be dynamic.
     *
     * @param rigidBodyA the body for the A end (not null, alias created)
     * @param rigidBodyB the body for the B end (not null, alias created)
     * @param pivotInWorld the pivot location (in physics-space coordinates, not
     * null, unaffected)
     * @param rotInWorld the orientation of the constraint (in physics-space
     * coordinates, not null, not zero, unaffected)
     * @param rotationOrder the order in which to apply axis rotations (not
     * null)
     * @return a new instance, not in any PhysicsSpace
     */
    public static New6Dof newInstance(PhysicsRigidBody rigidBodyA,
            PhysicsRigidBody rigidBodyB, Vector3f pivotInWorld,
            Quaternion rotInWorld, RotationOrder rotationOrder) {
        Validate.nonNull(rigidBodyA, "a");
        Validate.nonNull(rigidBodyB, "b");
        Validate.finite(pivotInWorld, "pivot location");
        Validate.nonZero(rotInWorld, "pivot orientation");
        Validate.nonNull(rotationOrder, "rotation order");

        Transform tmpTransform = new Transform();

        rigidBodyA.getTransform(tmpTransform);
        tmpTransform.setScale(1f); // a2w
        tmpTransform = tmpTransform.invert();  // w2a
        Transform p2a = new Transform(pivotInWorld, rotInWorld);
        p2a.combineWithParent(tmpTransform);
        Vector3f pivotInA = p2a.getTranslation(); // alias
        Matrix3f rotInA = p2a.getRotation().toRotationMatrix();

        rigidBodyB.getTransform(tmpTransform);
        tmpTransform.setScale(1f); // b2w
        tmpTransform = tmpTransform.invert();  // w2b
        Transform p2b = new Transform(pivotInWorld, rotInWorld);
        p2b.combineWithParent(tmpTransform);
        Vector3f pivotInB = p2b.getTranslation(); // alias
        Matrix3f rotInB = p2b.getRotation().toRotationMatrix();

        New6Dof result = new New6Dof(rigidBodyA, rigidBodyB, pivotInA,
                pivotInB, rotInA, rotInB, rotationOrder);

        return result;
    }

    /**
     * Alter the specified parameter for the indexed degree of freedom.
     *
     * @param parameter which parameter (not null)
     * @param dofIndex which degree of freedom (0&rarr;X translation, 1&rarr;Y
     * translation, 2&rarr;Z translation, 3&rarr;X rotation, 4&rarr;Y rotation,
     * 5&rarr;Z rotation)
     * @param value the desired parameter value
     */
    public void set(MotorParam parameter, int dofIndex, float value) {
        Validate.inRange(dofIndex, "DOF index", 0, 5);

        if (dofIndex >= 3) {
            int axisIndex = dofIndex - 3;
            RotationMotor motor = getRotationMotor(axisIndex);
            motor.set(parameter, value);
        } else {
            TranslationMotor motor = getTranslationMotor();
            Vector3f vector = motor.get(parameter, null);
            vector.set(dofIndex, value);
            motor.set(parameter, vector);
        }
    }

    /**
     * Alter the damping for the indexed degree of freedom.
     *
     * @param dofIndex which degree of freedom (0&rarr;X translation, 1&rarr;Y
     * translation, 2&rarr;Z translation, 3&rarr;X rotation, 4&rarr;Y rotation,
     * 5&rarr;Z rotation)
     * @param damping the desired viscous damping ratio (0&rarr;no damping,
     * 1&rarr;critically damped, default=1)
     * @param limitIfNeeded true&rarr;automatically limit damping to prevent the
     * spring from blowing up
     */
    public void setDamping(int dofIndex, float damping, boolean limitIfNeeded) {
        Validate.inRange(dofIndex, "DOF index", 0, 5);

        long constraintId = nativeId();
        setDamping(constraintId, dofIndex, damping, limitIfNeeded);
    }

    /**
     * Alter the equilibrium points for all degrees of freedom, based on the
     * constraint's current location/orientation.
     */
    public void setEquilibriumPoint() {
        long constraintId = nativeId();
        setAllEquilibriumPointsToCurrent(constraintId);
    }

    /**
     * Alter the equilibrium point of the indexed degree of freedom, based on
     * the constraint's current location/orientation.
     *
     * @param dofIndex which degree of freedom (0&rarr;X translation, 1&rarr;Y
     * translation, 2&rarr;Z translation, 3&rarr;X rotation, 4&rarr;Y rotation,
     * 5&rarr;Z rotation)
     */
    public void setEquilibriumPoint(int dofIndex) {
        Validate.inRange(dofIndex, "DOF index", 0, 5);

        long constraintId = nativeId();
        setEquilibriumPointToCurrent(constraintId, dofIndex);
    }

    /**
     * Alter the equilibrium point of the indexed degree of freedom.
     *
     * @param dofIndex which degree of freedom (0&rarr;X translation, 1&rarr;Y
     * translation, 2&rarr;Z translation, 3&rarr;X rotation, 4&rarr;Y rotation,
     * 5&rarr;Z rotation)
     * @param value the desired equilibrium point
     */
    public void setEquilibriumPoint(int dofIndex, float value) {
        Validate.inRange(dofIndex, "DOF index", 0, 5);

        long constraintId = nativeId();
        setEquilibriumPoint(constraintId, dofIndex, value);
    }

    /**
     * Alter the order in which to apply axis rotations.
     *
     * @param rotationOrder the desired order (not null)
     */
    public void setRotationOrder(RotationOrder rotationOrder) {
        long constraintId = nativeId();
        int rotOrder = rotationOrder.ordinal();
        setRotationOrder(constraintId, rotOrder);
    }

    /**
     * Alter the spring stiffness for the indexed degree of freedom.
     *
     * @param dofIndex which degree of freedom (0&rarr;X translation, 1&rarr;Y
     * translation, 2&rarr;Z translation, 3&rarr;X rotation, 4&rarr;Y rotation,
     * 5&rarr;Z rotation)
     * @param stiffness the desired stiffness (default=0)
     * @param limitIfNeeded true&rarr;automatically limit stiffness to prevent
     * the spring from moving too widely
     */
    public void
            setStiffness(int dofIndex, float stiffness, boolean limitIfNeeded) {
        Validate.inRange(dofIndex, "DOF index", 0, 5);

        long constraintId = nativeId();
        setStiffness(constraintId, dofIndex, stiffness, limitIfNeeded);
    }
    // *************************************************************************
    // Constraint methods

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
        New6Dof old = (New6Dof) original;
        assert old != this;
        assert old.hasAssignedNativeObject();

        super.cloneFields(cloner, original);
        if (hasAssignedNativeObject()) {
            return;
        }

        this.rotA = cloner.clone(rotA);
        this.rotB = cloner.clone(rotB);
        this.rotationMotor = null;
        this.translationMotor = null;
        createConstraint();

        copyConstraintProperties(old);

        for (int i = 0; i < MyVector3f.numAxes; ++i) {
            RotationMotor motor = getRotationMotor(i);
            RotationMotor oldMotor = old.getRotationMotor(i);

            motor.setDampingLimited(oldMotor.isDampingLimited());
            motor.setMotorEnabled(oldMotor.isMotorEnabled());
            motor.setServoEnabled(oldMotor.isServoEnabled());
            motor.setSpringEnabled(oldMotor.isSpringEnabled());
            motor.setStiffnessLimited(oldMotor.isStiffnessLimited());

            for (MotorParam p : MotorParam.values()) {
                motor.set(p, oldMotor.get(p));
            }
        }

        TranslationMotor motor = getTranslationMotor();
        TranslationMotor oldMotor = old.getTranslationMotor();

        for (int i = 0; i < MyVector3f.numAxes; ++i) {
            motor.setDampingLimited(i, oldMotor.isDampingLimited(i));
            motor.setMotorEnabled(i, oldMotor.isMotorEnabled(i));
            motor.setServoEnabled(i, oldMotor.isServoEnabled(i));
            motor.setSpringEnabled(i, oldMotor.isSpringEnabled(i));
            motor.setStiffnessLimited(i, oldMotor.isStiffnessLimited(i));
        }

        for (MotorParam p : MotorParam.values()) {
            motor.set(p, oldMotor.get(p, null));
        }
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

        this.rotA = (Matrix3f) capsule.readSavable(tagRotA, null);
        this.rotB = (Matrix3f) capsule.readSavable(tagRotB, null);
        this.rotationOrder = capsule
                .readEnum(tagRotOrder, RotationOrder.class, RotationOrder.XYZ);

        createConstraint();
        readConstraintProperties(capsule);

        for (int axisIndex = 0; axisIndex < MyVector3f.numAxes; ++axisIndex) {
            RotationMotor motor = getRotationMotor(axisIndex);
            String motorTag = tagRotMotor + axisIndex;

            motor.setDampingLimited(capsule.readBoolean(
                    motorTag + tagDampingLimited, false));
            motor.setMotorEnabled(capsule.readBoolean(
                    motorTag + tagMotorEnabled, false));
            motor.setServoEnabled(capsule.readBoolean(
                    motorTag + tagServoEnabled, false));
            motor.setSpringEnabled(capsule.readBoolean(
                    motorTag + tagSpringEnabled, false));
            motor.setStiffnessLimited(capsule.readBoolean(
                    motorTag + tagStiffnessLimited, false));

            for (MotorParam param : MotorParam.values()) {
                String tag = motorTag + param.tagSuffix();
                float defaultValue = param.defaultForRotationMotor();
                float value = capsule.readFloat(tag, defaultValue);
                motor.set(param, value);
            }
        }

        TranslationMotor motor = translationMotor;
        String motorTag = tagTransMotor;

        for (int axisIndex = 0; axisIndex < MyVector3f.numAxes; ++axisIndex) {
            motor.setDampingLimited(axisIndex, capsule.readBoolean(
                    motorTag + tagDampingLimited + axisIndex, false));
            motor.setMotorEnabled(axisIndex, capsule.readBoolean(
                    motorTag + tagMotorEnabled + axisIndex, false));
            motor.setServoEnabled(axisIndex, capsule.readBoolean(
                    motorTag + tagServoEnabled + axisIndex, false));
            motor.setSpringEnabled(axisIndex, capsule.readBoolean(
                    motorTag + tagSpringEnabled + axisIndex, false));
            motor.setStiffnessLimited(axisIndex, capsule.readBoolean(
                    motorTag + tagStiffnessLimited + axisIndex, false));
        }

        for (MotorParam p : MotorParam.values()) {
            String tag = motorTag + p.tagSuffix();
            float def = p.defaultForRotationMotor();
            Vector3f defaultVector = new Vector3f(def, def, def);
            Savable value = capsule.readSavable(tag, defaultVector);
            motor.set(p, (Vector3f) value);
        }
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

        capsule.write(rotA, tagRotA, null);
        capsule.write(rotB, tagRotB, null);
        capsule.write(rotationOrder, tagRotOrder, RotationOrder.XYZ);

        for (int axisIndex = 0; axisIndex < MyVector3f.numAxes; ++axisIndex) {
            RotationMotor motor = rotationMotor[axisIndex];
            String motorTag = tagRotMotor + axisIndex;

            capsule.write(motor.isDampingLimited(),
                    motorTag + tagDampingLimited, false);
            capsule.write(motor.isMotorEnabled(),
                    motorTag + tagMotorEnabled, false);
            capsule.write(motor.isServoEnabled(),
                    motorTag + tagServoEnabled, false);
            capsule.write(motor.isSpringEnabled(),
                    motorTag + tagSpringEnabled, false);
            capsule.write(motor.isStiffnessLimited(),
                    motorTag + tagStiffnessLimited, false);

            for (MotorParam param : MotorParam.values()) {
                float value = motor.get(param);
                String tag = motorTag + param.tagSuffix();
                float defaultValue = param.defaultForRotationMotor();
                capsule.write(value, tag, defaultValue);
            }
        }

        TranslationMotor motor = translationMotor;
        String motorTag = tagTransMotor;

        for (int axisIndex = 0; axisIndex < MyVector3f.numAxes; ++axisIndex) {
            capsule.write(motor.isDampingLimited(axisIndex),
                    motorTag + tagDampingLimited + axisIndex, false);
            capsule.write(motor.isMotorEnabled(axisIndex),
                    motorTag + tagMotorEnabled + axisIndex, false);
            capsule.write(motor.isServoEnabled(axisIndex),
                    motorTag + tagServoEnabled + axisIndex, false);
            capsule.write(motor.isSpringEnabled(axisIndex),
                    motorTag + tagSpringEnabled + axisIndex, false);
            capsule.write(motor.isStiffnessLimited(axisIndex),
                    motorTag + tagStiffnessLimited + axisIndex, false);
        }

        for (MotorParam param : MotorParam.values()) {
            Vector3f values = motor.get(param, null);
            String tag = motorTag + param.tagSuffix();
            capsule.write(values, tag, null);
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Create the configured Constraint in Bullet.
     */
    private void createConstraint() {
        PhysicsRigidBody a = getBodyA();
        assert pivotA != null;
        assert rotA != null;

        PhysicsRigidBody b = getBodyB();
        long bId = b.nativeId();
        assert pivotB != null;
        assert rotB != null;

        int rotOrder = rotationOrder.ordinal();
        long constraintId;
        if (a == null) {
            /*
             * Create a single-ended constraint.  Bullet assumes single-ended
             * btGeneric6DofSpring2Constraints are satisfied at creation, so we
             * temporarily re-position the body to satisfy the constraint.
             */
            Transform jInWorld = new Transform();
            jInWorld.getRotation().fromRotationMatrix(rotA);
            jInWorld.setTranslation(pivotA);

            Transform jInB = new Transform();
            jInB.getRotation().fromRotationMatrix(rotB);
            jInB.setTranslation(pivotB);

            Transform bToWorld = jInB.invert().combineWithParent(jInWorld);

            Vector3f saveLocation = b.getPhysicsLocation(null);
            Quaternion saveRotation = b.getPhysicsRotation(null);

            b.setPhysicsLocation(bToWorld.getTranslation());
            b.setPhysicsRotation(bToWorld.getRotation());

            constraintId = createSingleEnded(bId, pivotB, rotB, rotOrder);

            b.setPhysicsLocation(saveLocation);
            b.setPhysicsRotation(saveRotation);

        } else { // Create a double-ended constraint.
            long aId = a.nativeId();
            constraintId = createDoubleEnded(
                    aId, bId, pivotA, rotA, pivotB, rotB, rotOrder);
            if (logger2.isLoggable(Level.INFO)) {
                logger2.log(Level.INFO, "Created {0} with A={1} B={2}",
                        new Object[]{
                            Long.toHexString(constraintId),
                            Long.toHexString(aId),
                            Long.toHexString(bId)
                        });
            }
        }

        assert getConstraintType(constraintId) == 12;
        setNativeId(constraintId);

        gatherMotors();
    }

    private void gatherMotors() {
        assert rotationMotor == null;
        assert translationMotor == null;

        long constraintId = nativeId();
        this.rotationMotor = new RotationMotor[MyVector3f.numAxes];

        for (int axisIndex = 0; axisIndex < MyVector3f.numAxes; ++axisIndex) {
            long motorId = getRotationalMotor(constraintId, axisIndex);
            this.rotationMotor[axisIndex] = new RotationMotor(motorId);
        }

        long motorId = getTranslationalMotor(constraintId);
        this.translationMotor = new TranslationMotor(motorId);
    }
    // *************************************************************************
    // native private methods

    native private static long createDoubleEnded(long bodyIdA, long bodyIdB,
            Vector3f pivotInA, Matrix3f rotInA, Vector3f pivotInB,
            Matrix3f rotInB, int rotOrder);

    native private static long createSingleEnded(
            long bodyIdB, Vector3f pivotInB, Matrix3f rotInB, int rotOrder);

    native private static void
            enableSpring(long constraintId, int dofIndex, boolean enableFlag);

    native private static void
            getAngles(long constraintId, Vector3f storeVector);

    native private static void
            getAxis(long constraintId, int axisIndex, Vector3f storeVector);

    native private static void
            getCalculatedBasisA(long constraintId, Matrix3f storeMatrix);

    native private static void
            getCalculatedBasisB(long constraintId, Matrix3f storeMatrix);

    native private static void
            getCalculatedOriginA(long constraintId, Vector3f storeVector);

    native private static void
            getCalculatedOriginB(long constraintId, Vector3f storeVector);

    native private static void
            getFrameOffsetA(long constraintId, Transform storeTransform);

    native private static void
            getFrameOffsetB(long constraintId, Transform storeTransform);

    native private static void
            getPivotOffset(long constraintId, Vector3f storeVector);

    native private static long getRotationalMotor(long constraintId, int index);

    native private static int getRotationOrder(long constraintId);

    native private static long getTranslationalMotor(long constraintId);

    native private static void
            setAllEquilibriumPointsToCurrent(long constraintId);

    native private static void setDamping(long constraintId, int dofIndex,
            float damping, boolean limitIfNeeded);

    native private static void
            setEquilibriumPoint(long constraintId, int dofIndex, float value);

    native private static void
            setEquilibriumPointToCurrent(long constraintId, int dofIndex);

    native private static void
            setRotationOrder(long constraintId, int rotOrder);

    native private static void setStiffness(long constraintId, int dofIndex,
            float stiffness, boolean limitIfNeeded);
}
