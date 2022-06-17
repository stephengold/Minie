/*
 * Copyright (c) 2018-2020 jMonkeyEngine
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
package com.jme3.bullet.animation;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.joints.New6Dof;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.joints.SixDofJoint;
import com.jme3.bullet.joints.motors.MotorParam;
import com.jme3.bullet.joints.motors.RotationMotor;
import com.jme3.bullet.joints.motors.RotationalLimitMotor;
import com.jme3.bullet.joints.motors.TranslationMotor;
import com.jme3.bullet.joints.motors.TranslationalLimitMotor;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * A range-of-motion preset for a ragdoll joint. Immutable except for
 * {@link #read(com.jme3.export.JmeImporter)}.
 *
 * @author Stephen Gold sgold@sonic.net
 *
 * Based on RagdollPreset by Rémy Bouquet (Nehon).
 */
public class RangeOfMotion implements Savable {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(RangeOfMotion.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagMaxX = "maxX";
    final private static String tagMaxY = "maxY";
    final private static String tagMaxZ = "maxZ";
    final private static String tagMinX = "minX";
    final private static String tagMinY = "minY";
    final private static String tagMinZ = "minZ";
    /**
     * local copy of {@link com.jme3.math.Vector3f#ZERO}
     */
    final private static Vector3f translateIdentity = new Vector3f(0f, 0f, 0f);
    /**
     * maximum motor force on each axis
     */
    final private static Vector3f maxMotorForces
            = new Vector3f(1e8f, 1e8f, 1e8f);
    // *************************************************************************
    // fields

    /**
     * maximum rotation angle around the X axis (in radians)
     */
    private float maxX = 0f;
    /**
     * minimum rotation angle around the X axis (in radians)
     */
    private float minX = 0f;
    /**
     * maximum rotation angle around the Y axis (in radians)
     */
    private float maxY = 0f;
    /**
     * minimum rotation angle around the Y axis (in radians)
     */
    private float minY = 0f;
    /**
     * maximum rotation angle around the Z axis (in radians)
     */
    private float maxZ = 0f;
    /**
     * minimum rotation angle around the Z axis (in radians)
     */
    private float minZ = 0f;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a preset with no motion allowed.
     */
    public RangeOfMotion() {
        // do nothing
    }

    /**
     * Instantiate a preset with no motion allowed.
     *
     * @param angles the desired Euler rotation angles (not null, unaffected)
     */
    public RangeOfMotion(Vector3f angles) {
        Validate.nonNull(angles, "angles");
        Validate.inRange(angles.x, "X rotation", -FastMath.PI, FastMath.PI);
        Validate.inRange(angles.y, "Y rotation", -FastMath.PI, FastMath.PI);
        Validate.inRange(angles.z, "Z rotation", -FastMath.PI, FastMath.PI);

        maxX = angles.x;
        minX = angles.x;

        maxY = angles.y;
        minY = angles.y;

        maxZ = angles.z;
        minZ = angles.z;
    }

    /**
     * Instantiate a preset with the specified range of motion.
     *
     * @param maxX the maximum rotation around the X axis (in radians)
     * @param minX the minimum rotation around the X axis (in radians)
     * @param maxY the maximum rotation around the Y axis (in radians)
     * @param minY the minimum rotation around the Y axis (in radians)
     * @param maxZ the maximum rotation around the Z axis (in radians)
     * @param minZ the minimum rotation around the Z axis (in radians)
     */
    public RangeOfMotion(float maxX, float minX, float maxY, float minY,
            float maxZ, float minZ) {
        Validate.inRange(maxX, "max X rotation", minX, FastMath.PI);
        Validate.inRange(minX, "min X rotation", -FastMath.PI, maxX);
        Validate.inRange(maxY, "max Y rotation", minY, FastMath.PI);
        Validate.inRange(minY, "min Y rotation", -FastMath.PI, maxY);
        Validate.inRange(maxZ, "max Z rotation", minZ, FastMath.PI);
        Validate.inRange(minZ, "min Z rotation", -FastMath.PI, maxZ);

        this.maxX = maxX;
        this.minX = minX;
        this.maxY = maxY;
        this.minY = minY;
        this.maxZ = maxZ;
        this.minZ = minZ;
    }

    /**
     * Instantiate a preset with the specified symmetric range of motion.
     *
     * @param maxX the maximum rotation around the X axis (in radians, &ge;0)
     * @param maxY the maximum rotation around the Y axis (in radians, &ge;0)
     * @param maxZ the maximum rotation around the Z axis (in radians, &ge;0)
     */
    public RangeOfMotion(float maxX, float maxY, float maxZ) {
        Validate.inRange(maxX, "max X rotation", 0f, FastMath.PI);
        Validate.inRange(maxY, "max Y rotation", 0f, FastMath.PI);
        Validate.inRange(maxZ, "max Z rotation", 0f, FastMath.PI);

        this.maxX = maxX;
        this.minX = -maxX;
        this.maxY = maxY;
        this.minY = -maxY;
        this.maxZ = maxZ;
        this.minZ = -maxZ;
    }

    /**
     * Instantiate a preset with the specified symmetric range of motion.
     *
     * @param maxAngle the maximum rotation around each axis (in radians, &ge;0)
     */
    public RangeOfMotion(float maxAngle) {
        Validate.inRange(maxX, "max rotation", 0f, FastMath.PI);

        maxX = maxAngle;
        minX = -maxAngle;
        maxY = maxAngle;
        minY = -maxAngle;
        maxZ = maxAngle;
        minZ = -maxAngle;
    }

    /**
     * Instantiate a preset for rotation on a single axis.
     *
     * @param axisIndex which axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     */
    public RangeOfMotion(int axisIndex) {
        switch (axisIndex) {
            case PhysicsSpace.AXIS_X:
                maxX = 1f;
                minX = -1f;
                break;
            case PhysicsSpace.AXIS_Y:
                maxY = 1f;
                minY = -1f;
                break;
            case PhysicsSpace.AXIS_Z:
                maxZ = 1f;
                minZ = -1f;
                break;
            default:
                throw new IllegalArgumentException("axisIndex = " + axisIndex);
        }
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Read the maximum rotation around the indexed axis.
     *
     * @param axisIndex which axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     *
     * @return the rotation angle (in radians)
     */
    public float getMaxRotation(int axisIndex) {
        float result;
        switch (axisIndex) {
            case PhysicsSpace.AXIS_X:
                result = maxX;
                break;
            case PhysicsSpace.AXIS_Y:
                result = maxY;
                break;
            case PhysicsSpace.AXIS_Z:
                result = maxZ;
                break;
            default:
                throw new IllegalArgumentException("axisIndex = " + axisIndex);
        }

        return result;
    }

    /**
     * Read the minimum rotation around the indexed axis.
     *
     * @param axisIndex which axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     *
     * @return the rotation angle (in radians)
     */
    public float getMinRotation(int axisIndex) {
        float result;
        switch (axisIndex) {
            case PhysicsSpace.AXIS_X:
                result = minX;
                break;
            case PhysicsSpace.AXIS_Y:
                result = minY;
                break;
            case PhysicsSpace.AXIS_Z:
                result = minZ;
                break;
            default:
                String message = "axisIndex = " + axisIndex;
                throw new IllegalArgumentException(message);
        }

        return result;
    }

    /**
     * Apply this preset to the specified constraint, locking the specified
     * rotational axes at their current angles.
     *
     * @param constraint where to apply this preset (not null, modified)
     * @param lockX true&rarr;prevent the joint from rotating around its local X
     * axis
     * @param lockY true&rarr;prevent the joint from rotating around its local Y
     * axis
     * @param lockZ true&rarr;prevent the joint from rotating around its local Z
     * axis
     */
    public void setup(PhysicsJoint constraint, boolean lockX, boolean lockY,
            boolean lockZ) {
        if (constraint instanceof New6Dof) {
            setupNew6Dof((New6Dof) constraint, lockX, lockY, lockZ);
        } else {
            SixDofJoint joint = (SixDofJoint) constraint;
            setupJoint(joint, lockX, lockY, lockZ);
        }
    }

    /**
     * Apply this preset to the specified joint, locking the specified
     * rotational axes at their current angles.
     *
     * @param joint where to apply this preset (not null, modified)
     * @param lockX true&rarr;prevent the joint from rotating around its local X
     * axis
     * @param lockY true&rarr;prevent the joint from rotating around its local Y
     * axis
     * @param lockZ true&rarr;prevent the joint from rotating around its local Z
     * axis
     */
    public void setupJoint(SixDofJoint joint, boolean lockX, boolean lockY,
            boolean lockZ) {
        Validate.nonNull(joint, "joint");

        Vector3f lower = new Vector3f(minX, minY, minZ);
        Vector3f upper = new Vector3f(maxX, maxY, maxZ);

        RotationalLimitMotor rotX
                = joint.getRotationalLimitMotor(PhysicsSpace.AXIS_X);
        if (lockX) {
            float angle = rotX.getAngle();
            //angle = FastMath.clamp(angle, minX, maxX);
            lower.x = angle;
            upper.x = angle;
        }
        rotX.setLowerLimit(lower.x);
        rotX.setUpperLimit(upper.x);

        RotationalLimitMotor rotY
                = joint.getRotationalLimitMotor(PhysicsSpace.AXIS_Y);
        if (lockY) {
            float angle = rotY.getAngle();
            //angle = FastMath.clamp(angle, minY, maxY);
            lower.y = angle;
            upper.y = angle;
        }
        rotY.setLowerLimit(lower.y);
        rotY.setUpperLimit(upper.y);

        RotationalLimitMotor rotZ
                = joint.getRotationalLimitMotor(PhysicsSpace.AXIS_Z);
        if (lockZ) {
            float angle = rotZ.getAngle();
            //angle = FastMath.clamp(angle, minZ, maxZ);
            lower.z = angle;
            upper.z = angle;
        }
        rotZ.setLowerLimit(lower.z);
        rotZ.setUpperLimit(upper.z);

        joint.setAngularLowerLimit(lower);
        joint.setAngularUpperLimit(upper);

        for (int i = 0; i < MyVector3f.numAxes; ++i) {
            RotationalLimitMotor rot = joint.getRotationalLimitMotor(i);
            rot.setMaxMotorForce(maxMotorForces.x);
            rot.setMaxLimitForce(10f * maxMotorForces.x);
        }

        // Prevent the joint from translating.
        joint.setLinearLowerLimit(translateIdentity);
        joint.setLinearUpperLimit(translateIdentity);

        TranslationalLimitMotor tra = joint.getTranslationalLimitMotor();
        tra.setLowerLimit(translateIdentity);
        tra.setMaxMotorForce(maxMotorForces);
        tra.setUpperLimit(translateIdentity);
    }

    /**
     * Apply this preset to the specified constraint, locking the specified
     * rotational axes at their current angles.
     *
     * @param constraint where to apply this preset (not null, modified)
     * @param lockX true&rarr;prevent the joint from rotating around its local X
     * axis
     * @param lockY true&rarr;prevent the joint from rotating around its local Y
     * axis
     * @param lockZ true&rarr;prevent the joint from rotating around its local Z
     * axis
     */
    public void setupNew6Dof(New6Dof constraint, boolean lockX, boolean lockY,
            boolean lockZ) {
        Vector3f current = constraint.getAngles(null);
        Vector3f lower = new Vector3f(minX, minY, minZ);
        Vector3f upper = new Vector3f(maxX, maxY, maxZ);

        if (lockX) {
            lower.x = current.x;
            upper.x = current.x;
        }
        RotationMotor rotX = constraint.getRotationMotor(PhysicsSpace.AXIS_X);
        rotX.set(MotorParam.Equilibrium, 0.5f * (lower.x + upper.x));
        rotX.set(MotorParam.LowerLimit, lower.x);
        rotX.set(MotorParam.UpperLimit, upper.x);
        rotX.setSpringEnabled(lockX);

        if (lockY) {
            lower.y = current.y;
            upper.y = current.y;
        }
        RotationMotor rotY = constraint.getRotationMotor(PhysicsSpace.AXIS_Y);
        rotY.set(MotorParam.Equilibrium, 0.5f * (lower.y + upper.y));
        rotY.set(MotorParam.LowerLimit, lower.y);
        rotY.set(MotorParam.UpperLimit, upper.y);
        rotY.setSpringEnabled(lockY);

        if (lockZ) {
            lower.z = current.z;
            upper.z = current.z;
        }
        RotationMotor rotZ = constraint.getRotationMotor(PhysicsSpace.AXIS_Z);
        rotZ.set(MotorParam.Equilibrium, 0.5f * (lower.z + upper.z));
        rotZ.set(MotorParam.LowerLimit, lower.z);
        rotZ.set(MotorParam.UpperLimit, upper.z);
        rotZ.setSpringEnabled(lockZ);

        for (int i = 0; i < MyVector3f.numAxes; ++i) {
            RotationMotor rot = constraint.getRotationMotor(i);
            rot.set(MotorParam.MaxMotorForce, maxMotorForces.x);
        }

        // Inhibit translation.
        TranslationMotor tra = constraint.getTranslationMotor();
        tra.set(MotorParam.LowerLimit, translateIdentity);
        tra.set(MotorParam.MaxMotorForce, maxMotorForces);
        tra.set(MotorParam.UpperLimit, translateIdentity);
    }
    // *************************************************************************
    // Savable methods

    /**
     * De-serialize this preset from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        InputCapsule capsule = importer.getCapsule(this);

        maxX = capsule.readFloat(tagMaxX, 0f);
        minX = capsule.readFloat(tagMinX, 0f);
        maxY = capsule.readFloat(tagMaxY, 0f);
        minY = capsule.readFloat(tagMinY, 0f);
        maxZ = capsule.readFloat(tagMaxZ, 0f);
        minZ = capsule.readFloat(tagMinZ, 0f);
    }

    /**
     * Serialize this preset to the specified exporter, for example when saving
     * to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(maxX, tagMaxX, 0f);
        capsule.write(minX, tagMinX, 0f);
        capsule.write(maxY, tagMaxY, 0f);
        capsule.write(minY, tagMinY, 0f);
        capsule.write(maxZ, tagMaxZ, 0f);
        capsule.write(minZ, tagMinZ, 0f);
    }
}
