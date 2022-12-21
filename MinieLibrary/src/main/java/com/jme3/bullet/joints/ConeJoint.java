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
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Logger;

/**
 * A 3 degree-of-freedom joint based on Bullet's btConeTwistConstraint.
 * <p>
 * <i>From the Bullet manual:</i><br>
 * To create ragdolls, the cone twist constraint is very useful for limbs like
 * the upper arm. It is a special point to point constraint that adds cone and
 * twist axis limits. The x-axis serves as twist axis.
 *
 * @author normenhansen
 */
public class ConeJoint extends Constraint {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(ConeJoint.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagAngularOnly = "angularOnly";
    final private static String tagRotA = "rotA";
    final private static String tagRotB = "rotB";
    final private static String tagSwingSpan1 = "swingSpan1";
    final private static String tagSwingSpan2 = "swingSpan2";
    final private static String tagTwistSpan = "twistSpan";
    /**
     * local copy of {@link com.jme3.math.Vector3f#ZERO}
     */
    final private static Vector3f translateIdentity = new Vector3f(0f, 0f, 0f);
    // *************************************************************************
    // fields

    /**
     * copy of joint orientation in A's local coordinates (rotation matrix)
     */
    private Matrix3f rotA;
    /**
     * copy of joint orientation: in B's local coordinates for a double-ended
     * joint, or in physics-space coordinates for a single-ended joint (rotation
     * matrix)
     */
    private Matrix3f rotB;
    /**
     * copy of span of the first swing axis (in radians)
     */
    private float swingSpan1 = 1e30f;
    /**
     * copy of span of the 2nd swing axis (in radians)
     */
    private float swingSpan2 = 1e30f;
    /**
     * copy of span of the twist (local X) axis (in radians)
     */
    private float twistSpan = 1e30f;
    /**
     * copy of the angular-only flag
     */
    private boolean angularOnly = false;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected ConeJoint() {
    }

    /**
     * Instantiate a single-ended ConeJoint with its pivot at the physics-space
     * origin.
     * <p>
     * To be effective, the joint must be added to the PhysicsSpace of the body
     * and the body must be dynamic.
     *
     * @param rigidBodyA the body to constrain (not null, alias created)
     * @param pivotInA the pivot location in A's scaled local coordinates (not
     * null, unaffected)
     * @param rotInA the joint orientation in A's local coordinates (rotation
     * matrix, unaffected)
     */
    public ConeJoint(
            PhysicsRigidBody rigidBodyA, Vector3f pivotInA, Matrix3f rotInA) {
        super(rigidBodyA, JointEnd.A, pivotInA, translateIdentity);
        this.rotA = rotInA.clone();
        this.rotB = rotA;
        createJoint();
    }

    /**
     * Instantiate a double-ended ConeJoint.
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
     */
    public ConeJoint(PhysicsRigidBody rigidBodyA, PhysicsRigidBody rigidBodyB,
            Vector3f pivotInA, Vector3f pivotInB) {
        super(rigidBodyA, rigidBodyB, pivotInA, pivotInB);
        this.rotA = new Matrix3f();
        this.rotB = new Matrix3f();
        createJoint();
    }

    /**
     * Instantiate a double-ended ConeJoint.
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
     */
    public ConeJoint(PhysicsRigidBody rigidBodyA, PhysicsRigidBody rigidBodyB,
            Vector3f pivotInA, Vector3f pivotInB, Matrix3f rotInA,
            Matrix3f rotInB) {
        super(rigidBodyA, rigidBodyB, pivotInA, pivotInB);
        this.rotA = rotInA.clone();
        this.rotB = rotInB.clone();
        createJoint();
    }
    // *************************************************************************
    // new methods exposed

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
     * Read the span of the first swing axis.
     *
     * @return the span (in radians)
     */
    public float getSwingSpan1() {
        return swingSpan1;
    }

    /**
     * Read the span of the 2nd swing axis.
     *
     * @return the span (in radians)
     */
    public float getSwingSpan2() {
        return swingSpan2;
    }

    /**
     * Read the span of the twist (local X) axis.
     *
     * @return the span (in radians)
     */
    public float getTwistSpan() {
        return twistSpan;
    }

    /**
     * Test whether this joint is angular-only.
     *
     * @return true if angular only, otherwise false
     */
    public boolean isAngularOnly() {
        return angularOnly;
    }

    /**
     * Alter whether this joint is angular-only.
     *
     * @param value the desired setting (default=false)
     */
    public void setAngularOnly(boolean value) {
        long constraintId = nativeId();
        this.angularOnly = value;
        setAngularOnly(constraintId, value);
    }

    /**
     * Alter the angular limits for this joint.
     *
     * @param swingSpan1 the desired span of the first swing axis (in radians)
     * @param swingSpan2 the desired span of the 2nd swing axis (in radians)
     * @param twistSpan the desired span of the twist (local X) axis (in
     * radians)
     */
    public void setLimit(float swingSpan1, float swingSpan2, float twistSpan) {
        long constraintId = nativeId();
        this.swingSpan1 = swingSpan1;
        this.swingSpan2 = swingSpan2;
        this.twistSpan = twistSpan;
        setLimit(constraintId, swingSpan1, swingSpan2, twistSpan);
    }
    // *************************************************************************
    // Constraint methods

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
        ConeJoint old = (ConeJoint) original;
        assert old != this;
        assert old.hasAssignedNativeObject();

        super.cloneFields(cloner, original);
        if (hasAssignedNativeObject()) {
            return;
        }

        this.rotA = cloner.clone(rotA);
        this.rotB = cloner.clone(rotB);
        createJoint();

        copyConstraintProperties(old);
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

        this.rotA = (Matrix3f) capsule.readSavable(tagRotA, new Matrix3f());
        this.rotB = (Matrix3f) capsule.readSavable(tagRotB, new Matrix3f());

        this.angularOnly = capsule.readBoolean(tagAngularOnly, false);
        this.swingSpan1 = capsule.readFloat(tagSwingSpan1, 1e30f);
        this.swingSpan2 = capsule.readFloat(tagSwingSpan2, 1e30f);
        this.twistSpan = capsule.readFloat(tagTwistSpan, 1e30f);

        createJoint();
        readConstraintProperties(capsule);
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

        capsule.write(rotA, tagRotA, null);
        capsule.write(rotB, tagRotB, null);

        capsule.write(angularOnly, tagAngularOnly, false);
        capsule.write(swingSpan1, tagSwingSpan1, 1e30f);
        capsule.write(swingSpan2, tagSwingSpan2, 1e30f);
        capsule.write(twistSpan, tagTwistSpan, 1e30f);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Create the configured joint in Bullet.
     */
    private void createJoint() {
        PhysicsRigidBody a = getBodyA();
        long aId = a.nativeId();
        assert pivotA != null;
        assert rotA != null;
        PhysicsRigidBody b = getBodyB();

        long constraintId;
        if (b == null) {
            /*
             * Create a single-ended joint.
             * Bullet assumes single-ended btConeTwistConstraints have
             * rotInWorld=rotInA and pivotInWorld=(0,0,0).
             */
            constraintId = createJoint1(aId, pivotA, rotA);

        } else {
            assert pivotB != null;
            assert rotB != null;

            // Create a double-ended joint.
            long bId = b.nativeId();
            constraintId = createJoint(aId, bId, pivotA, rotA, pivotB, rotB);
        }

        assert getConstraintType(constraintId) == 5;
        setNativeId(constraintId);

        setLimit(swingSpan1, swingSpan2, twistSpan);
        setAngularOnly(angularOnly);
    }
    // *************************************************************************
    // native private methods

    native private static long createJoint(long bodyIdA, long bodyIdB,
            Vector3f pivotInA, Matrix3f rotInA, Vector3f pivotInB,
            Matrix3f rotInB);

    native private static long
            createJoint1(long bodyIdA, Vector3f pivotInA, Matrix3f rotInA);

    native private static void
            getFrameOffsetA(long jointId, Transform frameInA);

    native private static void
            getFrameOffsetB(long jointId, Transform frameInB);

    native private static void
            setAngularOnly(long jointId, boolean angularOnly);

    native private static void setLimit(
            long jointId, float swingSpan1, float swingSpan2, float twistSpan);
}
