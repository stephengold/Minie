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

import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Matrix3f;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A 6 degree-of-freedom joint based on Bullet's btGeneric6DofSpringConstraint.
 * <p>
 * <i>From the Bullet manual:</i><br>
 * <p>
 * For each axis:<ul>
 * <li>Lowerlimit = Upperlimit &rarr; axis is locked</li>
 * <li>Lowerlimit &gt; Upperlimit &rarr; axis is free</li>
 * <li>Lowerlimit &lt; Upperlimit &rarr; axis is limited in that range</li>
 * </ul>
 * <p>
 * It is recommended to use the btGeneric6DofSpring2Constraint, it has some
 * improvements over the original btGeneric6Dof(Spring)Constraint.
 *
 * @see com.jme3.bullet.joints.New6Dof
 * @author normenhansen
 */
public class SixDofSpringJoint extends SixDofJoint {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger3
            = Logger.getLogger(SixDofSpringJoint.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagDof = "Dof";
    final private static String tagSpringDamping = "_SpringDamping";
    final private static String tagStiffness = "_Stiffness";
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected SixDofSpringJoint() {
    }

    /**
     * Instantiate a single-ended SixDofSpringJoint.
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
    public SixDofSpringJoint(PhysicsRigidBody rigidBodyB, Vector3f pivotInB,
            Vector3f pivotInWorld, Matrix3f rotInB, Matrix3f rotInWorld,
            JointEnd linearReferenceFrame) {
        super(rigidBodyB, pivotInB, pivotInWorld, rotInB, rotInWorld,
                linearReferenceFrame);
    }

    /**
     * Instantiate a double-ended SixDofSpringJoint.
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
     * @param rotInA the orientation of the joint in A's local coordinates (not
     * null, unaffected)
     * @param rotInB the orientation of the joint in B's local coordinates (not
     * null, unaffected)
     * @param useLinearReferenceFrameA true&rarr;use body A, false&rarr;use body
     * B
     */
    public SixDofSpringJoint(PhysicsRigidBody rigidBodyA,
            PhysicsRigidBody rigidBodyB, Vector3f pivotInA, Vector3f pivotInB,
            Matrix3f rotInA, Matrix3f rotInB,
            boolean useLinearReferenceFrameA) {
        super(rigidBodyA, rigidBodyB, pivotInA, pivotInB, rotInA, rotInB,
                useLinearReferenceFrameA);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Enable or disable the spring for the indexed degree of freedom. Enabling
     * a spring also enables the corresponding limit motor. Disabling a spring
     * also disables the corresponding limit motor.
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
     * Read the damping of the indexed degree of freedom.
     *
     * @param dofIndex which degree of freedom (0&rarr;X translation, 1&rarr;Y
     * translation, 2&rarr;Z translation, 3&rarr;X rotation, 4&rarr;Y rotation,
     * 5&rarr;Z rotation)
     * @return the viscous damping ratio
     */
    public float getDamping(int dofIndex) {
        Validate.inRange(dofIndex, "DOF index", 0, 5);

        long constraintId = nativeId();
        float result = getDamping(constraintId, dofIndex);

        return result;
    }

    /**
     * Read the equilibrium point of the indexed degree of freedom.
     *
     * @param dofIndex which degree of freedom (0&rarr;X translation, 1&rarr;Y
     * translation, 2&rarr;Z translation, 3&rarr;X rotation, 4&rarr;Y rotation,
     * 5&rarr;Z rotation)
     * @return the equilibrium point
     */
    public float getEquilibriumPoint(int dofIndex) {
        Validate.inRange(dofIndex, "DOF index", 0, 5);

        long constraintId = nativeId();
        float result = getEquilibriumPoint(constraintId, dofIndex);

        return result;
    }

    /**
     * Read the spring stiffness of the indexed degree of freedom.
     *
     * @param dofIndex which degree of freedom (0&rarr;X translation, 1&rarr;Y
     * translation, 2&rarr;Z translation, 3&rarr;X rotation, 4&rarr;Y rotation,
     * 5&rarr;Z rotation)
     * @return the stiffness
     */
    public float getStiffness(int dofIndex) {
        Validate.inRange(dofIndex, "DOF index", 0, 5);

        long constraintId = nativeId();
        float result = getStiffness(constraintId, dofIndex);

        return result;
    }

    /**
     * Test whether the spring for the indexed degree of freedom is enabled.
     *
     * @param dofIndex which degree of freedom (0&rarr;X translation, 1&rarr;Y
     * translation, 2&rarr;Z translation, 3&rarr;X rotation, 4&rarr;Y rotation,
     * 5&rarr;Z rotation)
     * @return true if enabled, otherwise false
     */
    public boolean isSpringEnabled(int dofIndex) {
        Validate.inRange(dofIndex, "DOF index", 0, 5);

        long constraintId = nativeId();
        boolean result = isSpringEnabled(constraintId, dofIndex);

        return result;
    }

    /**
     * Alter the damping for the indexed degree of freedom.
     *
     * @param dofIndex which degree of freedom (0&rarr;X translation, 1&rarr;Y
     * translation, 2&rarr;Z translation, 3&rarr;X rotation, 4&rarr;Y rotation,
     * 5&rarr;Z rotation)
     * @param damping the desired viscous damping ratio (0&rarr;no damping,
     * 1&rarr;critically damped, default=1)
     */
    public void setDamping(int dofIndex, float damping) {
        Validate.inRange(dofIndex, "DOF index", 0, 5);

        long constraintId = nativeId();
        setDamping(constraintId, dofIndex, damping);
    }

    /**
     * Alter the equilibrium points for all degrees of freedom, based on the
     * joint's current location/orientation.
     */
    public void setEquilibriumPoint() {
        long constraintId = nativeId();
        setEquilibriumPoint(constraintId);
    }

    /**
     * Alter the equilibrium point of the indexed degree of freedom, based on
     * the joint's current location/orientation.
     *
     * @param dofIndex which degree of freedom (0&rarr;X translation, 1&rarr;Y
     * translation, 2&rarr;Z translation, 3&rarr;X rotation, 4&rarr;Y rotation,
     * 5&rarr;Z rotation)
     */
    public void setEquilibriumPoint(int dofIndex) {
        Validate.inRange(dofIndex, "DOF index", 0, 5);

        long constraintId = nativeId();
        setEquilibriumPoint(constraintId, dofIndex);
    }

    /**
     * Alter the spring stiffness for the indexed degree of freedom.
     *
     * @param dofIndex which degree of freedom (0&rarr;X translation, 1&rarr;Y
     * translation, 2&rarr;Z translation, 3&rarr;X rotation, 4&rarr;Y rotation,
     * 5&rarr;Z rotation)
     * @param stiffness the desired stiffness (default=0)
     */
    public void setStiffness(int dofIndex, float stiffness) {
        Validate.inRange(dofIndex, "DOF index", 0, 5);

        long constraintId = nativeId();
        setStiffness(constraintId, dofIndex, stiffness);
    }
    // *************************************************************************
    // SixDofJoint methods

    /**
     * Create a double-ended {@code btGeneric6DofSpringConstraint}.
     *
     * @param bodyIdA the ID of the body for the A end (not zero)
     * @param bodyIdB the ID of the body for the B end (not zero)
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
    @Override
    native protected long createJoint(long bodyIdA, long bodyIdB,
            Vector3f pivotInA, Matrix3f rotInA, Vector3f pivotInB,
            Matrix3f rotInB, boolean useLinearReferenceFrameA);

    /**
     * Create a single-ended {@code btGeneric6DofSpringConstraint}.
     *
     * @param bodyIdB the ID of the body for the B end (not zero)
     * @param pivotInB the pivot location in B's scaled local coordinates (not
     * null, unaffected)
     * @param rotInB the orientation of the joint in B's local coordinates (not
     * null, unaffected)
     * @param useLinearReferenceFrameB true&rarr;use body A, false&rarr;use body
     * B
     * @return the ID of the new joint
     */
    @Override
    native protected long createJoint1(long bodyIdB, Vector3f pivotInB,
            Matrix3f rotInB, boolean useLinearReferenceFrameB);

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
        assert !hasAssignedNativeObject();
        SixDofSpringJoint old = (SixDofSpringJoint) original;
        assert old != this;
        assert old.hasAssignedNativeObject();

        super.cloneFields(cloner, original);
        if (hasAssignedNativeObject()) {
            return;
        }

        for (int dofIndex = 0; dofIndex < 6; ++dofIndex) {
            setDamping(dofIndex, old.getDamping(dofIndex));
            setStiffness(dofIndex, old.getStiffness(dofIndex));
            // TODO: enableSpring, equilibrium points
        }
    }

    /**
     * De-serialize this joint from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);
        InputCapsule capsule = importer.getCapsule(this);

        for (int dofIndex = 0; dofIndex < 6; ++dofIndex) {
            String dofTag = tagDof + dofIndex;

            setDamping(dofIndex,
                    capsule.readFloat(dofTag + tagSpringDamping, 1f));
            setStiffness(dofIndex,
                    capsule.readFloat(dofTag + tagStiffness, 0f));
            // TODO: enableSpring, equilibrium points
        }
    }

    /**
     * Serialize this joint to the specified exporter, for example when saving
     * to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        super.write(exporter);
        OutputCapsule capsule = exporter.getCapsule(this);

        for (int dofIndex = 0; dofIndex < 6; ++dofIndex) {
            String dofTag = tagDof + dofIndex;

            capsule.write(getDamping(dofIndex), dofTag + tagSpringDamping, 1f);
            capsule.write(getStiffness(dofIndex), dofTag + tagStiffness, 0f);
        }

    }
    // *************************************************************************
    // native private methods

    native private static void
            enableSpring(long jointId, int dofIndex, boolean onOff);

    native private static float getDamping(long jointId, int dofIndex);

    native private static float getEquilibriumPoint(long jointId, int dofIndex);

    native private static float getStiffness(long jointId, int dofIndex);

    native private static boolean isSpringEnabled(long jointId, int dofIndex);

    native private static void
            setDamping(long jointId, int dofIndex, float damping);

    native private static void setEquilibriumPoint(long jointId);

    native private static void setEquilibriumPoint(long jointId, int dofIndex);

    native private static void
            setStiffness(long jointId, int dofIndex, float stiffness);
}
