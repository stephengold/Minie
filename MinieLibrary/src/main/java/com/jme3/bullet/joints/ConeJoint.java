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
import java.util.logging.Level;
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
public class ConeJoint extends PhysicsJoint {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(ConeJoint.class.getName());
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
     * copy of span of the 1st swing axis (in radians)
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
     * copy of the angular-only flag (default=false)
     */
    private boolean angularOnly = false;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public ConeJoint() {
    }

    /**
     * Instantiate a single-ended ConeJoint with its pivot at the physics-space
     * origin.
     * <p>
     * To be effective, the joint must be added to the physics space of the body
     * and the body must be dynamic.
     *
     * @param nodeA the body to constrain (not null, alias created)
     * @param pivotInA the pivot location in A's scaled local coordinates (not
     * null, unaffected)
     * @param rotInA the joint orientation in A's local coordinates (rotation
     * matrix, unaffected)
     */
    public ConeJoint(PhysicsRigidBody nodeA, Vector3f pivotInA,
            Matrix3f rotInA) {
        super(nodeA, JointEnd.A, pivotInA, translateIdentity);
        rotA = rotInA.clone();
        rotB = rotA;
        createJoint();
    }

    /**
     * Instantiate a double-ended ConeJoint.
     * <p>
     * To be effective, the joint must be added to the physics space of the 2
     * bodies. Also, the bodies must be dynamic and distinct.
     *
     * @param nodeA the body for the A end (not null, alias created)
     * @param nodeB the body for the B end (not null, alias created)
     * @param pivotInA the pivot location in A's scaled local coordinates (not
     * null, unaffected)
     * @param pivotInB the pivot location in B's scaled local coordinates (not
     * null, unaffected)
     */
    public ConeJoint(PhysicsRigidBody nodeA, PhysicsRigidBody nodeB,
            Vector3f pivotInA, Vector3f pivotInB) {
        super(nodeA, nodeB, pivotInA, pivotInB);
        rotA = new Matrix3f();
        rotB = new Matrix3f();
        createJoint();
    }

    /**
     * Instantiate a double-ended ConeJoint.
     * <p>
     * To be effective, the joint must be added to the physics space of the 2
     * bodies. Also, the bodies must be dynamic and distinct.
     *
     * @param nodeA the body for the A end (not null, alias created)
     * @param nodeB the body for the B end (not null, alias created)
     * @param pivotInA the pivot location in A's scaled local coordinates (not
     * null, unaffected)
     * @param pivotInB the pivot location in B's scaled local coordinates (not
     * null, unaffected)
     * @param rotInA the joint orientation in A's local coordinates (rotation
     * matrix, unaffected)
     * @param rotInB the joint orientation in B's local coordinates (rotation
     * matrix, unaffected)
     */
    public ConeJoint(PhysicsRigidBody nodeA, PhysicsRigidBody nodeB,
            Vector3f pivotInA, Vector3f pivotInB, Matrix3f rotInA,
            Matrix3f rotInB) {
        super(nodeA, nodeB, pivotInA, pivotInB);
        rotA = rotInA.clone();
        rotB = rotInB.clone();
        createJoint();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Read the span of the 1st swing axis.
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
        angularOnly = value;
        setAngularOnly(objectId, value);
    }

    /**
     * Alter the angular limits for this joint.
     *
     * @param swingSpan1 the desired span of the 1st swing axis (in radians)
     * @param swingSpan2 the desired span of the 2nd swing axis (in radians)
     * @param twistSpan the desired span of the twist (local X) axis (in
     * radians)
     */
    public void setLimit(float swingSpan1, float swingSpan2, float twistSpan) {
        this.swingSpan1 = swingSpan1;
        this.swingSpan2 = swingSpan2;
        this.twistSpan = twistSpan;
        setLimit(objectId, swingSpan1, swingSpan2, twistSpan);
    }
    // *************************************************************************
    // PhysicsJoint methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned object into a deep-cloned one, using the specified cloner
     * and original to resolve copied fields.
     *
     * @param cloner the cloner that's cloning this shape (not null)
     * @param original the instance from which this instance was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);

        rotA = cloner.clone(rotA);
        rotB = cloner.clone(rotB);
        createJoint();

        ConeJoint old = (ConeJoint) original;

        float bit = old.getBreakingImpulseThreshold();
        setBreakingImpulseThreshold(bit);

        boolean enableJoint = old.isEnabled();
        setEnabled(enableJoint);
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public ConeJoint jmeClone() {
        try {
            ConeJoint clone = (ConeJoint) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }

    /**
     * De-serialize this joint, for example when loading from a J3O file.
     *
     * @param im importer (not null)
     * @throws IOException from importer
     */
    @Override
    public void read(JmeImporter im) throws IOException {
        super.read(im);
        InputCapsule capsule = im.getCapsule(this);

        float breakingImpulseThreshold = capsule.readFloat(
                "breakingImpulseThreshold", Float.MAX_VALUE);
        boolean isEnabled = capsule.readBoolean("isEnabled", true);

        rotA = (Matrix3f) capsule.readSavable("rotA", new Matrix3f());
        rotB = (Matrix3f) capsule.readSavable("rotB", new Matrix3f());

        angularOnly = capsule.readBoolean("angularOnly", false);
        swingSpan1 = capsule.readFloat("swingSpan1", 1e30f);
        swingSpan2 = capsule.readFloat("swingSpan2", 1e30f);
        twistSpan = capsule.readFloat("twistSpan", 1e30f);

        createJoint();

        setBreakingImpulseThreshold(breakingImpulseThreshold);
        setEnabled(isEnabled);
    }

    /**
     * Serialize this joint, for example when saving to a J3O file.
     *
     * @param ex exporter (not null)
     * @throws IOException from exporter
     */
    @Override
    public void write(JmeExporter ex) throws IOException {
        super.write(ex);
        OutputCapsule capsule = ex.getCapsule(this);

        capsule.write(rotA, "rotA", null);
        capsule.write(rotB, "rotB", null);

        capsule.write(angularOnly, "angularOnly", false);
        capsule.write(swingSpan1, "swingSpan1", 1e30f);
        capsule.write(swingSpan2, "swingSpan2", 1e30f);
        capsule.write(twistSpan, "twistSpan", 1e30f);
    }
    // *************************************************************************
    // private methods

    /**
     * Create the configured joint in Bullet.
     */
    private void createJoint() {
        assert objectId == 0L;
        assert nodeA != null;
        assert pivotA != null;
        assert rotA != null;

        if (nodeB == null) {
            /*
             * Create a single-ended joint.
             * Bullet assumes single-ended btConeTwistConstraints have
             * rotInWorld=rotInA and pivotInWorld=0,0,0.
             */
            objectId = createJoint1(nodeA.getObjectId(), pivotA, rotA);

        } else {
            assert pivotB != null;
            assert rotB != null;
            /*
             * Create a double-ended joint.
             */
            objectId = createJoint(nodeA.getObjectId(), nodeB.getObjectId(),
                    pivotA, rotA, pivotB, rotB);
        }
        assert objectId != 0L;
        logger2.log(Level.FINE, "Created Joint {0}", Long.toHexString(objectId));

        setLimit(swingSpan1, swingSpan2, twistSpan);
        setAngularOnly(angularOnly);
    }

    native private long createJoint(long bodyIdA, long bodyIdB,
            Vector3f pivotInA, Matrix3f rotInA, Vector3f pivotInB,
            Matrix3f rotInB);

    native private long createJoint1(long bodyIdA, Vector3f pivotInA,
            Matrix3f rotInA);

    native private void setAngularOnly(long jointId, boolean angularOnly);

    native private void setLimit(long jointId, float swingSpan1,
            float swingSpan2, float twistSpan);
}
