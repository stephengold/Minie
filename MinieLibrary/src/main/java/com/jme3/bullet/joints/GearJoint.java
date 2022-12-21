/*
 * Copyright (c) 2022 jMonkeyEngine
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
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * A joint that couples the angular velocities of two bodies, based on Bullet's
 * btGearConstraint.
 * <p>
 * <i>From the Bullet manual:</i><br>
 * <p>
 * The btGearConstraint will couple the angular velocity for two bodies around
 * given local axis and ratio.
 *
 * @author elmfrain
 */
public class GearJoint extends Constraint {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(GearJoint.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagAxisA = "axisA";
    final private static String tagAxisB = "axisB";
    final private static String tagRatio = "ratio";
    // *************************************************************************
    // fields

    /**
     * copy of the A body's axis of rotation in its local coordinates (unit
     * vector)
     */
    private Vector3f axisA;
    /**
     * copy of the B body's axis of rotation in its local coordinates (unit
     * vector)
     */
    private Vector3f axisB;
    /**
     * copy of the gear ratio
     */
    private float ratio;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected GearJoint() {
    }

    /**
     * Instantiate a double-ended GearJoint with a 1:1 ratio.
     * <p>
     * To be effective, the joint must be added to the PhysicsSpace of both
     * bodies. Also, the bodies must be distinct and at least one of them must
     * be dynamic.
     *
     * @param rigidBodyA the body for the A end (not null, alias created)
     * @param rigidBodyB the body for the B end (not null, alias created)
     * @param axisInA the A body's axis of rotation in its local coordinates
     * (unit vector, not null, unaffected)
     * @param axisInB the B body's axis of rotation in its local coordinates
     * (unit vector, not null, unaffected)
     */
    public GearJoint(PhysicsRigidBody rigidBodyA, PhysicsRigidBody rigidBodyB,
            Vector3f axisInA, Vector3f axisInB) {
        super(rigidBodyA, rigidBodyB, Vector3f.ZERO, Vector3f.ZERO);

        Validate.nonZero(axisInA, "axis in body A");
        Validate.nonZero(axisInB, "axis in body B");

        this.axisA = axisInA.normalize();
        this.axisB = axisInB.normalize();
        this.ratio = 1f;
        createJoint();
    }

    /**
     * Instantiate a double-ended GearJoint with the specified gear ratio.
     * <p>
     * To be effective, the joint must be added to the PhysicsSpace of both
     * bodies. Also, the bodies must be distinct and at least one of them must
     * be dynamic.
     *
     * @param rigidBodyA the body for the A end (not null, alias created)
     * @param rigidBodyB the body for the B end (not null, alias created)
     * @param axisInA the A body's axis of rotation in its local coordinates
     * (unit vector, not null, unaffected)
     * @param axisInB the B body's axis of rotation in its local coordinates
     * (unit vector, not null, unaffected)
     * @param ratio the number of revolutions the A body should make for each
     * revolution of the B body
     */
    public GearJoint(PhysicsRigidBody rigidBodyA, PhysicsRigidBody rigidBodyB,
            Vector3f axisInA, Vector3f axisInB, float ratio) {
        super(rigidBodyA, rigidBodyB, Vector3f.ZERO, Vector3f.ZERO);

        Validate.nonZero(axisInA, "axis in body A");
        Validate.nonZero(axisInB, "axis in body B");

        this.axisA = axisInA.normalize();
        this.axisB = axisInB.normalize();
        this.ratio = ratio;
        createJoint();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy A body's axis of rotation.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the A body's axis of rotation in its local coordinates (either
     * storeResult or new vector)
     */
    public Vector3f getAxisA(Vector3f storeResult) {
        assert checkAxisA();

        Vector3f result;
        if (storeResult == null) {
            result = axisA.clone();
        } else {
            result = storeResult.set(axisA);
        }
        return result;
    }

    /**
     * Copy the B body's axis of rotation.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the B body's axis of rotation in its local coordinates (either
     * storeResult or new vector)
     */
    public Vector3f getAxisB(Vector3f storeResult) {
        assert checkAxisB();

        Vector3f result;
        if (storeResult == null) {
            result = axisB.clone();
        } else {
            result = storeResult.set(axisB);
        }
        return result;
    }

    /**
     * Get the gear ratio.
     *
     * @return the number of revolutions the A body makes for each revolution of
     * the B body
     */
    public float getRatio() {
        assert ratio == getRatio(nativeId()) : ratio;
        return ratio;
    }

    /**
     * Alter the A body's axis of rotation.
     *
     * @param axisInA the desired axis in local coordinates (unit vector, not
     * null, unaffected)
     */
    public void setAxisA(Vector3f axisInA) {
        Validate.nonZero(axisInA, "axis in body A");

        axisA.set(axisInA);
        MyVector3f.normalizeLocal(axisA);

        long constraintId = nativeId();
        setAxisA(constraintId, axisA);
    }

    /**
     * Alter the B body's axis of rotation.
     *
     * @param axisInB the axisInB axis in local coordinates (unit vector, not
     * null, unaffected)
     */
    public void setAxisB(Vector3f axisInB) {
        Validate.nonZero(axisInB, "axis in body B");

        axisB.set(axisInB);
        MyVector3f.normalizeLocal(axisB);

        long constraintId = nativeId();
        setAxisB(constraintId, axisB);
    }

    /**
     * Alter the joint's gear ratio.
     *
     * @param ratio the number of revolutions the A body should make for each
     * revolution of the B body (default=1)
     */
    public void setRatio(float ratio) {
        this.ratio = ratio;

        long constraintId = nativeId();
        setRatio(constraintId, ratio);
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
        GearJoint old = (GearJoint) original;
        assert old != this;
        assert old.hasAssignedNativeObject();

        super.cloneFields(cloner, original);
        if (hasAssignedNativeObject()) {
            return;
        }

        this.axisA = cloner.clone(axisA);
        this.axisB = cloner.clone(axisB);
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

        this.axisA = (Vector3f) capsule.readSavable(tagAxisA, null);
        this.axisB = (Vector3f) capsule.readSavable(tagAxisB, null);
        this.ratio = capsule.readFloat(tagRatio, 1f);

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

        capsule.write(axisA, tagAxisA, null);
        capsule.write(axisB, tagAxisB, null);
        capsule.write(ratio, tagRatio, 1f);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Test whether the local copy of the A body's axis matches the native
     * value.
     *
     * @return true for an exact match, otherwise false
     */
    private boolean checkAxisA() {
        long constraintId = nativeId();
        Vector3f tmpVector = new Vector3f();
        getAxisA(constraintId, tmpVector);
        boolean result = axisA.equals(tmpVector);

        return result;
    }

    /**
     * Test whether the local copy of the A body's axis matches the native
     * value.
     *
     * @return true for an exact match, otherwise false
     */
    private boolean checkAxisB() {
        long constraintId = nativeId();
        Vector3f tmpVector = new Vector3f();
        getAxisB(constraintId, tmpVector);
        boolean result = axisB.equals(tmpVector);

        return result;
    }

    /**
     * Create the configured joint in Bullet.
     */
    private void createJoint() {
        PhysicsRigidBody a = getBodyA();
        long aId = a.nativeId();
        assert !MyVector3f.isZero(axisA);

        PhysicsRigidBody b = getBodyB();
        long bId = b.nativeId();
        assert !MyVector3f.isZero(axisB);

        long constraintId;
        constraintId = createJoint(aId, bId, axisA, axisB, ratio);

        assert getConstraintType(constraintId) == 10;
        setNativeId(constraintId);
    }
    // *************************************************************************
    // native private methods

    native private static long createJoint(long objectIdA, long objectIdB,
            Vector3f axisInA, Vector3f axisInB, float ratio);

    native private static void getAxisA(long jointId, Vector3f storeResult);

    native private static void getAxisB(long jointId, Vector3f storeResult);

    native private static float getRatio(long jointId);

    native private static void setAxisA(long jointId, Vector3f axisA);

    native private static void setAxisB(long jointId, Vector3f axisB);

    native private static void setRatio(long jointId, float ratio);
}
