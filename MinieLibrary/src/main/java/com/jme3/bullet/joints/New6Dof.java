/*
 * Copyright (c) 2019 jMonkeyEngine
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
import com.jme3.export.Savable;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A 6 degree-of-freedom Constraint based on Bullet's
 * btGeneric6DofSpring2Constraint.
 * <p>
 * <i>From the Bullet manual:</i><br>
 * This generic constraint can emulate a variety of standard constraints, by
 * configuring each of the 6 degrees of freedom (dof). The first 3 dof axis are
 * linear axis, which represent translation of rigidbodies, and the latter 3 dof
 * axis represent the angular motion. Each axis can be either locked, free or
 * limited.
 * <p>
 * For each axis:<ul>
 * <li>Lowerlimit = Upperlimit &rarr; axis is locked</li>
 * <li>Lowerlimit &gt; Upperlimit &rarr; axis is free</li>
 * <li>Lowerlimit &lt; Upperlimit &rarr; axis is limited in that range</li>
 * </ul>
 *
 * @author sgold@sonic.net
 */
public class New6Dof extends Constraint {
    // *************************************************************************
    // constants and loggers

    /**
     * number of axes in a vector
     */
    final private static int numAxes = 3;
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
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public New6Dof() {
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

        rotA = rotInWorld.clone();
        rotB = rotInB.clone();
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

        rotA = rotInA.clone();
        rotB = rotInB.clone();
        this.rotationOrder = rotationOrder;
        createConstraint();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Compare Bullet's rotation order to the local copy.
     *
     * @return true if rotation orders are identical, otherwise false
     */
    public boolean checkRotationOrder() {
        long constraintId = getObjectId();
        int rotOrder = getRotationOrder(constraintId);
        boolean result = rotOrder == rotationOrder.ordinal();

        return result;
    }

    /**
     * Calculate the constraint's rotation angles.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the rotation angle for each local axis (in radians, either
     * storeResult or a new vector, not null)
     */
    public Vector3f getAngles(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long constraintId = getObjectId();
        getAngles(constraintId, result);

        return result;
    }

    /**
     * Calculate the indexed axis of the Constraint.
     *
     * @param axisIndex the axis index of the desired motor: 0&rarr;X, 1&rarr;Y,
     * 2&rarr;Z
     * @param storeResult storage for the result (modified if not null)
     * @return the rotation angle for each local axis (in radians, either
     * storeResult or a new vector, not null)
     */
    public Vector3f getAxis(int axisIndex, Vector3f storeResult) {
        Validate.inRange(axisIndex, "index", PhysicsSpace.AXIS_X,
                PhysicsSpace.AXIS_Z);
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long constraintId = getObjectId();
        getAxis(constraintId, axisIndex, result);

        return result;
    }

    /**
     * Copy the constraint's frame transform relative to the specified end.
     *
     * @param end which end (not null)
     * @param storeResult storage for the result (modified if not null)
     * @return the transform of the constraint space relative to the end
     */
    public Transform getFrameTransform(JointEnd end, Transform storeResult) {
        Transform result
                = (storeResult == null) ? new Transform() : storeResult;

        long constraintId = getObjectId();
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
     * Enable or disable the spring for the indexed degree of freedom.
     *
     * @param dofIndex which degree of freedom (0&rarr;X translation, 1&rarr;Y
     * translation, 2&rarr;Z translation, 3&rarr;X rotation, 4&rarr;Y rotation,
     * 5&rarr;Z rotation)
     * @param onOff true &rarr; enable, false &rarr; disable (default=false)
     */
    public void enableSpring(int dofIndex, boolean onOff) {
        Validate.inRange(dofIndex, "DOF index", 0, 5);

        long constraintId = getObjectId();
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
     * Copy the constraint's pivot offset.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the relative pivot position on each local axis (in physics-space
     * units, either storeResult or a new vector, not null)
     */
    public Vector3f getPivotOffset(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long constraintId = getObjectId();
        getPivotOffset(constraintId, result);

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
        Validate.inRange(axisIndex, "index", PhysicsSpace.AXIS_X,
                PhysicsSpace.AXIS_Z);
        return rotationMotor[axisIndex];
    }

    /**
     * Read the order in which axis rotations are applied.
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

        long constraintId = getObjectId();
        setDamping(constraintId, dofIndex, damping, limitIfNeeded);
    }

    /**
     * Alter the equilibrium points for all degrees of freedom, based on the
     * constraint's current location/orientation.
     */
    public void setEquilibriumPoint() {
        long constraintId = getObjectId();
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

        long constraintId = getObjectId();
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

        long constraintId = getObjectId();
        setEquilibriumPoint(constraintId, dofIndex, value);
    }

    /**
     * Alter the order in which to apply axis rotations.
     *
     * @param rotationOrder the desired order (not null)
     */
    public void setRotationOrder(RotationOrder rotationOrder) {
        long constraintId = getObjectId();
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
    public void setStiffness(int dofIndex, float stiffness,
            boolean limitIfNeeded) {
        Validate.inRange(dofIndex, "DOF index", 0, 5);

        long constraintId = getObjectId();
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
        super.cloneFields(cloner, original);

        rotA = cloner.clone(rotA);
        rotB = cloner.clone(rotB);
        rotationMotor = null;
        translationMotor = null;
        createConstraint();

        New6Dof old = (New6Dof) original;

        float bit = old.getBreakingImpulseThreshold();
        setBreakingImpulseThreshold(bit);

        boolean enableConstraint = old.isEnabled();
        setEnabled(enableConstraint);

        int numIterations = old.getOverrideIterations();
        overrideIterations(numIterations);

        for (int i = 0; i < numAxes; ++i) {
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

        for (int i = 0; i < numAxes; ++i) {
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
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public New6Dof jmeClone() {
        try {
            New6Dof clone = (New6Dof) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
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

        rotA = (Matrix3f) capsule.readSavable(tagRotA, null);
        rotB = (Matrix3f) capsule.readSavable(tagRotB, null);
        rotationOrder = capsule.readEnum(tagRotOrder, RotationOrder.class,
                RotationOrder.XYZ);

        createConstraint();
        readConstraintProperties(capsule);

        for (int axisIndex = 0; axisIndex < numAxes; ++axisIndex) {
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

        for (int axisIndex = 0; axisIndex < numAxes; ++axisIndex) {
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

        for (int axisIndex = 0; axisIndex < numAxes; ++axisIndex) {
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

        for (int axisIndex = 0; axisIndex < numAxes; ++axisIndex) {
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
        long bId = b.getObjectId();
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

        } else {
            /*
             * Create a double-ended constraint.
             */
            long aId = a.getObjectId();
            constraintId = createDoubleEnded(aId, bId, pivotA, rotA, pivotB,
                    rotB, rotOrder);
        }
        setNativeId(constraintId);

        gatherMotors();
    }

    private void gatherMotors() {
        assert rotationMotor == null;
        assert translationMotor == null;

        long constraintId = getObjectId();
        rotationMotor = new RotationMotor[numAxes];

        for (int axisIndex = 0; axisIndex < numAxes; ++axisIndex) {
            long motorId = getRotationalMotor(constraintId, axisIndex);
            rotationMotor[axisIndex] = new RotationMotor(motorId);
        }

        long motorId = getTranslationalMotor(constraintId);
        translationMotor = new TranslationMotor(motorId);
    }
    // *************************************************************************
    // native private methods

    native private long createDoubleEnded(long bodyIdA, long bodyIdB,
            Vector3f pivotInA, Matrix3f rotInA, Vector3f pivotInB,
            Matrix3f rotInB, int rotOrder);

    native private long createSingleEnded(long bodyIdB, Vector3f pivotInB,
            Matrix3f rotInB, int rotOrder);

    native private void enableSpring(long constraintId, int dofIndex,
            boolean enableFlag);

    native private void getAngles(long constraintId, Vector3f storeVector);

    native private void getAxis(long constraintId, int axisIndex,
            Vector3f storeVector);

    native private void getFrameOffsetA(long constraintId,
            Transform storeTransform);

    native private void getFrameOffsetB(long constraintId,
            Transform storeTransform);

    native private void getPivotOffset(long constraintId, Vector3f storeVector);

    native private long getRotationalMotor(long constraintId, int index);

    native private int getRotationOrder(long constraintId);

    native private long getTranslationalMotor(long constraintId);

    native private void setAllEquilibriumPointsToCurrent(long constraintId);

    native private void setDamping(long constraintId, int dofIndex,
            float damping, boolean limitIfNeeded);

    native private void setEquilibriumPoint(long constraintId, int dofIndex,
            float value);

    native private void setEquilibriumPointToCurrent(long constraintId,
            int dofIndex);

    native private void setRotationOrder(long constraintId, int rotOrder);

    native private void setStiffness(long constraintId, int dofIndex,
            float stiffness, boolean limitIfNeeded);
}
