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
package com.jme3.bullet.joints;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.joints.motors.RotationalLimitMotor;
import com.jme3.bullet.joints.motors.TranslationalLimitMotor;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A 6 degree-of-freedom joint based on Bullet's btGeneric6DofConstraint.
 * <p>
 * <i>From the Bullet manual:</i><br>
 * This generic constraint can emulate a variety of standard constraints, by
 * configuring each of the 6 degrees of freedom (dof). The first 3 dof axis are
 * linear axis, which represent translation of rigidbodies, and the latter 3 dof
 * axis represent the angular motion. Each axis can be either locked, free or
 * limited. On construction of a new btGeneric6DofSpring2Constraint, all axis
 * are locked. Afterwards the axis can be reconfigured. Note that several
 * combinations that include free and/or limited angular degrees of freedom are
 * undefined.
 * <p>
 * For each axis:<ul>
 * <li>Lowerlimit = Upperlimit &rarr; axis is locked</li>
 * <li>Lowerlimit &gt; Upperlimit &rarr; axis is free</li>
 * <li>Lowerlimit &lt; Upperlimit &rarr; axis is limited in that range</li>
 * </ul>
 *
 * @author normenhansen
 */
public class SixDofJoint extends Constraint {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(SixDofJoint.class.getName());
    /**
     * local copy of {@link com.jme3.math.Vector3f#ZERO}
     */
    final private static Vector3f translateIdentity = new Vector3f(0f, 0f, 0f);
    // *************************************************************************
    // fields

    /**
     * copy of the joint orientation: in physics-space coordinates if bodyA is
     * null, or else in A's local coordinates (rotation matrix)
     */
    private Matrix3f rotA;
    /**
     * copy of the joint orientation in B's local coordinates (rotation matrix)
     */
    private Matrix3f rotB;
    /**
     * true&rarr;limits give the allowable range of movement of frameB in frameA
     * space, false&rarr;limits give the allowable range of movement of frameA
     * in frameB space
     */
    private boolean useLinearReferenceFrameA;
    /**
     * rotational motor for each axis
     */
    private RotationalLimitMotor[] rotationalMotors;
    /**
     * translational motor
     */
    private TranslationalLimitMotor translationalMotor;
    /**
     * upper limits for rotation of all 3 axes
     */
    private Vector3f angularUpperLimit = new Vector3f(0f, 0f, 0f);
    /**
     * lower limits for rotation of all 3 axes
     */
    private Vector3f angularLowerLimit = new Vector3f(0f, 0f, 0f);
    /**
     * upper limit for translation of all 3 axes
     */
    private Vector3f linearUpperLimit = new Vector3f(0f, 0f, 0f);
    /**
     * lower limits for translation of all 3 axes
     */
    private Vector3f linearLowerLimit = new Vector3f(0f, 0f, 0f);
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public SixDofJoint() {
    }

    /**
     * Instantiate a single-ended SixDofJoint.
     * <p>
     * To be effective, the joint must be added to the body's PhysicsSpace and
     * the body must be dynamic.
     *
     * @param rigidBodyB the body to constrain (not null, alias created)
     * @param pivotInB the pivot location in B's scaled local coordinates (not
     * null, unaffected)
     * @param pivotInWorld the pivot location in physics-space coordinates (not
     * null, unaffected)
     * @param rotInB the orientation of the joint in B's local coordinates
     * (rotation matrix, not null, unaffected)
     * @param rotInWorld the orientation of the joint in physics-space
     * coordinates (rotation matrix, not null, unaffected)
     * @param linearReferenceFrame which end to use as the linear reference
     * frame (not null)
     */
    public SixDofJoint(PhysicsRigidBody rigidBodyB, Vector3f pivotInB,
            Vector3f pivotInWorld, Matrix3f rotInB, Matrix3f rotInWorld,
            JointEnd linearReferenceFrame) {
        super(rigidBodyB, JointEnd.B, pivotInB, pivotInWorld);

        useLinearReferenceFrameA = (linearReferenceFrame == JointEnd.A);
        rotA = rotInWorld.clone();
        rotB = rotInB.clone();
        createJoint();
    }

    /**
     * Instantiate a double-ended SixDofJoint.
     * <p>
     * To be effective, the joint must be added to the PhysicsSpace of both
     * bodies. Also, the bodies must be distinct and at least one of them must
     * be dynamic.
     *
     * @param rigidBodyA the body for the A end (not null, alias created)
     * @param rigidBodyB the body for the B end (not null, alias created)
     * @param pivotInA the pivot location in A's scaled local coordinates (not
     * null, unaffected)
     * @param pivotInB the pivot location in B's scaled local coordinates (not
     * null, unaffected)
     * @param rotInA the joint orientation in A's local coordinates (rotation
     * matrix, unaffected)
     * @param rotInB the joint orientation in B's local coordinates (rotation
     * matrix, unaffected)
     * @param useLinearReferenceFrameA true&rarr;use body A, false&rarr;use body
     * B
     */
    public SixDofJoint(PhysicsRigidBody rigidBodyA, PhysicsRigidBody rigidBodyB,
            Vector3f pivotInA, Vector3f pivotInB, Matrix3f rotInA,
            Matrix3f rotInB, boolean useLinearReferenceFrameA) {
        super(rigidBodyA, rigidBodyB, pivotInA, pivotInB);

        this.useLinearReferenceFrameA = useLinearReferenceFrameA;
        rotA = rotInA.clone();
        rotB = rotInB.clone();
        createJoint();
    }

    /**
     * Instantiate a double-ended SixDofJoint.
     * <p>
     * To be effective, the joint must be added to the body's PhysicsSpace and
     * the body must be dynamic.
     *
     * @param rigidBodyA the 1st body to constrain (not null, alias created)
     * @param rigidBodyB the 2nd body to constrain (not null, alias created)
     * @param pivotInA the pivot location in A's scaled local coordinates (not
     * null, unaffected)
     * @param pivotInB the pivot location in B's scaled local coordinates (not
     * null, unaffected)
     * @param useLinearReferenceFrameA true&rarr;use body A, false&rarr;use body
     * B
     */
    public SixDofJoint(PhysicsRigidBody rigidBodyA, PhysicsRigidBody rigidBodyB,
            Vector3f pivotInA, Vector3f pivotInB,
            boolean useLinearReferenceFrameA) {
        super(rigidBodyA, rigidBodyB, pivotInA, pivotInB);

        this.useLinearReferenceFrameA = useLinearReferenceFrameA;
        rotA = new Matrix3f();
        rotB = new Matrix3f();
        createJoint();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the joint's rotation angles.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the rotation angle for each local axis (in radians, either
     * storeResult or a new vector, not null)
     */
    public Vector3f getAngles(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getAngles(objectId, result);
        return result;
    }

    /**
     * Copy the joint's lower limits for rotation on all 3 axes.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the lower limit for each local axis (in radians, either
     * storeResult or a new vector, not null)
     */
    public Vector3f getAngularLowerLimit(Vector3f storeResult) {
        if (storeResult == null) {
            return angularLowerLimit.clone();
        } else {
            return storeResult.set(angularLowerLimit);
        }
    }

    /**
     * Copy the joint's upper limits for rotation on all 3 axes.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the upper limit for each local axis (in radians, either
     * storeResult or a new vector, not null)
     */
    public Vector3f getAngularUpperLimit(Vector3f storeResult) {
        if (storeResult == null) {
            return angularUpperLimit.clone();
        } else {
            return storeResult.set(angularUpperLimit);
        }
    }

    /**
     * Copy the joint's frame transform relative to the specified end.
     *
     * @param end which end (not null)
     * @param storeResult storage for the result (modified if not null)
     * @return the transform of the constraint space relative to the end
     */
    public Transform getFrameTransform(JointEnd end, Transform storeResult) {
        Transform result
                = (storeResult == null) ? new Transform() : storeResult;

        switch (end) {
            case A:
                getFrameOffsetA(objectId, result);
                break;
            case B:
                getFrameOffsetB(objectId, result);
                break;
            default:
                String message = "end = " + end.toString();
                throw new IllegalArgumentException(message);
        }

        return result;
    }

    /**
     * Copy the joint's pivot offset.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the relative pivot position on each local axis (in physics-space
     * units, either storeResult or a new vector, not null)
     */
    public Vector3f getPivotOffset(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getPivotOffset(objectId, result);
        return result;
    }

    /**
     * Copy the joint's lower limits for translation on all 3 axes.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the lower limit for each local axis (in radians, either
     * storeResult or a new vector, not null)
     */
    public Vector3f getLinearLowerLimit(Vector3f storeResult) {
        if (storeResult == null) {
            return linearLowerLimit.clone();
        } else {
            return storeResult.set(linearLowerLimit);
        }
    }

    /**
     * Copy the joint's upper limits for translation on all 3 axes.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the upper limit for each local axis (in radians, either
     * storeResult or a new vector, not null)
     */
    public Vector3f getLinearUpperLimit(Vector3f storeResult) {
        if (storeResult == null) {
            return linearUpperLimit.clone();
        } else {
            return storeResult.set(linearUpperLimit);
        }
    }

    /**
     * Access the indexed RotationalLimitMotor of this joint, the motor which
     * influences rotation around the indexed axis.
     *
     * @param axisIndex the axis index of the desired motor: 0&rarr;X, 1&rarr;Y,
     * 2&rarr;Z
     * @return the pre-existing instance
     */
    public RotationalLimitMotor getRotationalLimitMotor(int axisIndex) {
        Validate.inRange(axisIndex, "index", PhysicsSpace.AXIS_X,
                PhysicsSpace.AXIS_Z);
        return rotationalMotors[axisIndex];
    }

    /**
     * Access the TranslationalLimitMotor of this joint, the motor which
     * influences translation on all 3 axes.
     *
     * @return the pre-existing instance
     */
    public TranslationalLimitMotor getTranslationalLimitMotor() {
        return translationalMotor;
    }

    /**
     * Alter the joint's lower limits for rotation of all 3 axes.
     *
     * @param vector the desired lower limits (in radians, not null, unaffected)
     */
    public void setAngularLowerLimit(Vector3f vector) {
        angularLowerLimit.set(vector);
        setAngularLowerLimit(objectId, vector);
    }

    /**
     * Alter the joint's upper limits for rotation of all 3 axes.
     *
     * @param vector the desired upper limits (in radians, not null, unaffected)
     */
    public void setAngularUpperLimit(Vector3f vector) {
        angularUpperLimit.set(vector);
        setAngularUpperLimit(objectId, vector);
    }

    /**
     * Alter the joint's lower limits for translation of all 3 axes.
     *
     * @param vector the desired lower limits (not null, unaffected)
     */
    public void setLinearLowerLimit(Vector3f vector) {
        linearLowerLimit.set(vector);
        setLinearLowerLimit(objectId, vector);
    }

    /**
     * Alter the joint's upper limits for translation of all 3 axes.
     *
     * @param vector the desired upper limits (not null, unaffected)
     */
    public void setLinearUpperLimit(Vector3f vector) {
        linearUpperLimit.set(vector);
        setLinearUpperLimit(objectId, vector);
    }
    // *************************************************************************
    // new protected methods

    /**
     * Create a new, double-ended btGeneric6DofConstraint.
     *
     * @param bodyIdA the ID of the body for the A end (not 0)
     * @param bodyIdB the ID of the body for the B end (not 0)
     * @param pivotInA the pivot location in A's scaled local coordinates (not
     * null, unaffected)
     * @param rotInA the orientation of the joint in A's local coordinates (not
     * null, unaffected)
     * @param pivotInB the pivot location in B's scaled local coordinates (not
     * null, unaffected)
     * @param rotInB the orientation of the joint in B's local coordinates (not
     * null, unaffected)
     * @param useLinearReferenceFrameA true&rarr;use body A, false&rarr;use body
     * B
     * @return the ID of the new joint
     */
    native protected long createJoint(long bodyIdA, long bodyIdB,
            Vector3f pivotInA, Matrix3f rotInA, Vector3f pivotInB,
            Matrix3f rotInB, boolean useLinearReferenceFrameA);

    /**
     * Create a new, single-ended btGeneric6DofConstraint.
     *
     * @param bodyIdB the ID of the body for the B end (not 0)
     * @param pivotInB the pivot location in B's scaled local coordinates (not
     * null, unaffected)
     * @param rotInB the orientation of the joint in B's local coordinates (not
     * null, unaffected)
     * @param useLinearReferenceFrameB true&rarr;use body A, false&rarr;use body
     * B
     * @return the ID of the new joint
     */
    native protected long createJoint1(long bodyIdB, Vector3f pivotInB,
            Matrix3f rotInB, boolean useLinearReferenceFrameB);
    // *************************************************************************
    // PhysicsJoint methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned joint into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this joint (not null)
     * @param original the instance from which this joint was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);

        rotA = cloner.clone(rotA);
        rotB = cloner.clone(rotB);
        rotationalMotors = null;
        translationalMotor = null;
        createJoint();

        angularLowerLimit = cloner.clone(angularLowerLimit);
        angularUpperLimit = cloner.clone(angularUpperLimit);
        linearLowerLimit = cloner.clone(linearLowerLimit);
        linearUpperLimit = cloner.clone(linearUpperLimit);

        SixDofJoint old = (SixDofJoint) original;

        float bit = old.getBreakingImpulseThreshold();
        setBreakingImpulseThreshold(bit);

        boolean enableJoint = old.isEnabled();
        setEnabled(enableJoint);

        int numIterations = old.getOverrideIterations();
        overrideIterations(numIterations);

        setAngularLowerLimit(old.getAngularLowerLimit(null));
        setAngularUpperLimit(old.getAngularUpperLimit(null));
        setLinearLowerLimit(old.getLinearLowerLimit(null));
        setLinearLowerLimit(old.getLinearUpperLimit(null));

        TranslationalLimitMotor tlm = getTranslationalLimitMotor();
        TranslationalLimitMotor oldTlm = old.getTranslationalLimitMotor();

        tlm.setDamping(oldTlm.getDamping());
        tlm.setERP(oldTlm.getERP(null));
        tlm.setLimitSoftness(oldTlm.getLimitSoftness());
        tlm.setLowerLimit(oldTlm.getLowerLimit(null));
        tlm.setMaxMotorForce(oldTlm.getMaxMotorForce(null));
        tlm.setNormalCFM(oldTlm.getNormalCFM(null));
        tlm.setRestitution(oldTlm.getRestitution());
        tlm.setStopCFM(oldTlm.getStopCFM(null));
        tlm.setTargetVelocity(oldTlm.getTargetVelocity(null));
        tlm.setUpperLimit(oldTlm.getUpperLimit(null));

        for (int i = 0; i < 3; ++i) {
            RotationalLimitMotor rlm = getRotationalLimitMotor(i);
            RotationalLimitMotor oldRlm = old.getRotationalLimitMotor(i);

            rlm.setRestitution(oldRlm.getRestitution());
            rlm.setDamping(oldRlm.getDamping());
            rlm.setEnableMotor(oldRlm.isEnableMotor());
            rlm.setERP(oldRlm.getERP());
            rlm.setUpperLimit(oldRlm.getUpperLimit());
            rlm.setLimitSoftness(oldRlm.getLimitSoftness());
            rlm.setLowerLimit(oldRlm.getLowerLimit());
            rlm.setMaxLimitForce(oldRlm.getMaxLimitForce());
            rlm.setMaxMotorForce(oldRlm.getMaxMotorForce());
            rlm.setNormalCFM(oldRlm.getNormalCFM());
            rlm.setStopCFM(oldRlm.getStopCFM());
            rlm.setTargetVelocity(oldRlm.getTargetVelocity());
        }
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public SixDofJoint jmeClone() {
        try {
            SixDofJoint clone = (SixDofJoint) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }

    /**
     * De-serialize this joint from the specified importer, for example when
     * loading from a J3O file. TODO re-format code
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);
        InputCapsule capsule = importer.getCapsule(this);

        rotA = (Matrix3f) capsule.readSavable("rotA", new Matrix3f());
        rotB = (Matrix3f) capsule.readSavable("rotB", new Matrix3f());
        useLinearReferenceFrameA
                = capsule.readBoolean("useLinearReferenceFrameA", false);

        createJoint();
        readConstraintProperties(capsule);

        setAngularUpperLimit((Vector3f) capsule.readSavable("angularUpperLimit", new Vector3f()));
        setAngularLowerLimit((Vector3f) capsule.readSavable("angularLowerLimit", new Vector3f()));
        setLinearUpperLimit((Vector3f) capsule.readSavable("linearUpperLimit", new Vector3f()));
        setLinearLowerLimit((Vector3f) capsule.readSavable("linearLowerLimit", new Vector3f()));

        for (int i = 0; i < 3; ++i) {
            RotationalLimitMotor rotationalLimitMotor = getRotationalLimitMotor(i);
            rotationalLimitMotor.setRestitution(capsule.readFloat("rotMotor" + i + "_Bounce", 0.0f));
            rotationalLimitMotor.setDamping(capsule.readFloat("rotMotor" + i + "_Damping", 1.0f));
            rotationalLimitMotor.setERP(capsule.readFloat("rotMotor" + i + "_ERP", 0.5f));
            rotationalLimitMotor.setUpperLimit(capsule.readFloat("rotMotor" + i + "_HiLimit", Float.POSITIVE_INFINITY));
            rotationalLimitMotor.setLimitSoftness(capsule.readFloat("rotMotor" + i + "_LimitSoftness", 0.5f));
            rotationalLimitMotor.setLowerLimit(capsule.readFloat("rotMotor" + i + "_LoLimit", Float.NEGATIVE_INFINITY));
            rotationalLimitMotor.setMaxLimitForce(capsule.readFloat("rotMotor" + i + "_MaxLimitForce", 300.0f));
            rotationalLimitMotor.setMaxMotorForce(capsule.readFloat("rotMotor" + i + "_MaxMotorForce", 0.1f));
            rotationalLimitMotor.setNormalCFM(capsule.readFloat("rotMotor" + i + "_NormalCFM", 0f));
            rotationalLimitMotor.setStopCFM(capsule.readFloat("rotMotor" + i + "_StopCFM", 0f));
            rotationalLimitMotor.setTargetVelocity(capsule.readFloat("rotMotor" + i + "_TargetVelocity", 0f));
            rotationalLimitMotor.setEnableMotor(capsule.readBoolean("rotMotor" + i + "_EnableMotor", false));
        }

        getTranslationalLimitMotor().setAccumulatedImpulse((Vector3f) capsule.readSavable("transMotor_AccumulatedImpulse", translateIdentity));
        getTranslationalLimitMotor().setDamping(capsule.readFloat("transMotor_Damping", 1.0f));
        getTranslationalLimitMotor().setERP((Vector3f) capsule.readSavable("transMotor_ERP", translateIdentity));
        getTranslationalLimitMotor().setLimitSoftness(capsule.readFloat("transMotor_LimitSoftness", 0.7f));
        getTranslationalLimitMotor().setLowerLimit((Vector3f) capsule.readSavable("transMotor_LowerLimit", translateIdentity));
        getTranslationalLimitMotor().setMaxMotorForce((Vector3f) capsule.readSavable("transMotor_MaxMotorForce", translateIdentity));
        getTranslationalLimitMotor().setNormalCFM((Vector3f) capsule.readSavable("transMotor_NormalCFM", translateIdentity));
        getTranslationalLimitMotor().setRestitution(capsule.readFloat("transMotor_Restitution", 0.5f));
        getTranslationalLimitMotor().setStopCFM((Vector3f) capsule.readSavable("transMotor_StopCFM", translateIdentity));
        getTranslationalLimitMotor().setTargetVelocity((Vector3f) capsule.readSavable("transMotor_TargetVelocity", translateIdentity));
        getTranslationalLimitMotor().setUpperLimit((Vector3f) capsule.readSavable("transMotor_UpperLimit", translateIdentity));
    }

    /**
     * Serialize this joint to the specified exporter, for example when saving
     * to a J3O file. TODO re-format code
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        super.write(exporter);
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(rotA, "rotA", new Matrix3f());
        capsule.write(rotB, "rotB", new Matrix3f());
        capsule.write(useLinearReferenceFrameA, "useLinearReferenceFrameA",
                false);

        capsule.write(angularUpperLimit, "angularUpperLimit", new Vector3f());
        capsule.write(angularLowerLimit, "angularLowerLimit", new Vector3f());
        capsule.write(linearUpperLimit, "linearUpperLimit", new Vector3f());
        capsule.write(linearLowerLimit, "linearLowerLimit", new Vector3f());

        int i = 0;
        for (RotationalLimitMotor rotationalLimitMotor : rotationalMotors) {
            capsule.write(rotationalLimitMotor.getRestitution(), "rotMotor" + i + "_Bounce", 0.0f);
            capsule.write(rotationalLimitMotor.getDamping(), "rotMotor" + i + "_Damping", 1.0f);
            capsule.write(rotationalLimitMotor.getERP(), "rotMotor" + i + "_ERP", 0.5f);
            capsule.write(rotationalLimitMotor.getUpperLimit(), "rotMotor" + i + "_HiLimit", Float.POSITIVE_INFINITY);
            capsule.write(rotationalLimitMotor.getLimitSoftness(), "rotMotor" + i + "_LimitSoftness", 0.5f);
            capsule.write(rotationalLimitMotor.getLowerLimit(), "rotMotor" + i + "_LoLimit", Float.NEGATIVE_INFINITY);
            capsule.write(rotationalLimitMotor.getMaxLimitForce(), "rotMotor" + i + "_MaxLimitForce", 300.0f);
            capsule.write(rotationalLimitMotor.getMaxMotorForce(), "rotMotor" + i + "_MaxMotorForce", 0.1f);
            capsule.write(rotationalLimitMotor.getNormalCFM(), "rotMotor" + i + "_NormalCFM", 0f);
            capsule.write(rotationalLimitMotor.getStopCFM(), "rotMotor" + i + "_StopCFM", 0f);
            capsule.write(rotationalLimitMotor.getTargetVelocity(), "rotMotor" + i + "_TargetVelocity", 0f);
            capsule.write(rotationalLimitMotor.isEnableMotor(), "rotMotor" + i + "_EnableMotor", false);
            ++i;
        }

        capsule.write(getTranslationalLimitMotor().getAccumulatedImpulse(null), "transMotor_AccumulatedImpulse", null);
        capsule.write(getTranslationalLimitMotor().getDamping(), "transMotor_Damping", 1.0f);
        capsule.write(getTranslationalLimitMotor().getERP(null), "transMotor_ERP", null);
        capsule.write(getTranslationalLimitMotor().getLimitSoftness(), "transMotor_LimitSoftness", 0.7f);
        capsule.write(getTranslationalLimitMotor().getLowerLimit(null), "transMotor_LowerLimit", null);
        capsule.write(getTranslationalLimitMotor().getMaxMotorForce(null), "transMotor_MaxMotorForce", null);
        capsule.write(getTranslationalLimitMotor().getNormalCFM(null), "transMotor_NormalCFM", null);
        capsule.write(getTranslationalLimitMotor().getRestitution(), "transMotor_Restitution", 0.5f);
        capsule.write(getTranslationalLimitMotor().getStopCFM(null), "transMotor_StopCFM", null);
        capsule.write(getTranslationalLimitMotor().getTargetVelocity(null), "transMotor_TargetVelocity", null);
        capsule.write(getTranslationalLimitMotor().getUpperLimit(null), "transMotor_UpperLimit", null);
    }
    // *************************************************************************
    // private methods

    /**
     * Create the configured joint in Bullet.
     */
    private void createJoint() {
        assert objectId == 0L;
        assert pivotA != null;
        assert rotA != null;
        PhysicsRigidBody b = getBodyB();
        assert b != null;
        assert pivotB != null;
        assert rotB != null;

        if (bodyA == null) {
            /*
             * Create a single-ended joint.  Bullet assumes single-ended
             * btGeneric6DofConstraints are satisfied at creation, so we
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
            boolean useLinearReferenceFrameB = !useLinearReferenceFrameA;
            objectId = createJoint1(b.getObjectId(), pivotB, rotB,
                    useLinearReferenceFrameB);

            b.setPhysicsLocation(saveLocation);
            b.setPhysicsRotation(saveRotation);

        } else {
            /*
             * Create a double-ended joint.
             */
            objectId = createJoint(bodyA.getObjectId(), b.getObjectId(),
                    pivotA, rotA, pivotB, rotB, useLinearReferenceFrameA);
        }
        assert objectId != 0L;
        logger2.log(Level.FINE, "Created {0}.", this);

        gatherMotors();
    }

    private void gatherMotors() {
        assert rotationalMotors == null;
        assert translationalMotor == null;

        rotationalMotors = new RotationalLimitMotor[3];
        for (int axisIndex = 0; axisIndex < 3; ++axisIndex) {
            long motorId = getRotationalLimitMotor(objectId, axisIndex);
            rotationalMotors[axisIndex] = new RotationalLimitMotor(motorId);
        }

        long motorId = getTranslationalLimitMotor(objectId);
        translationalMotor = new TranslationalLimitMotor(motorId);
    }
    // *************************************************************************
    // native private methods

    native private void getAngles(long jointId, Vector3f storeVector);

    native private void getFrameOffsetA(long jointId, Transform frameInA);

    native private void getFrameOffsetB(long jointId, Transform frameInB);

    native private void getPivotOffset(long jointId, Vector3f storeVector);

    native private long getRotationalLimitMotor(long jointId, int index);

    native private long getTranslationalLimitMotor(long jointId);

    native private void setAngularLowerLimit(long jointId, Vector3f limits);

    native private void setAngularUpperLimit(long jointId, Vector3f limits);

    native private void setLinearLowerLimit(long jointId, Vector3f limits);

    native private void setLinearUpperLimit(long jointId, Vector3f limits);
}
