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

import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * The abstract base class for rigid-body physics joints based on Bullet's
 * btTypedConstraint. A Constraint can be single-ended or double-ended:
 * <ul>
 * <li>A single-ended Constraint constrains the motion of a dynamic rigid
 * body.</li>
 * <li>A double-ended Conatraint connects 2 rigid bodies together in the same
 * PhysicsSpace. One of the bodies must be dynamic.</li>
 * </ul>
 * Subclasses include ConeJoint, HingeJoint, Point2PointJoint, and SixDofJoint.
 *
 * @author Stephen Gold sgold@sonic.net
 *
 * Based on PhysicsJoint by normenhansen.
 */
abstract public class Constraint extends PhysicsJoint {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger15
            = Logger.getLogger(Constraint.class.getName());
    // *************************************************************************
    // fields TODO privatize

    /**
     * true IFF bodies A and B are allowed to collide
     */
    private boolean collisionBetweenLinkedBodies = true;
    /**
     * copy of the pivot location: in physics-space coordinates if nodeA is
     * null, or else in A's scaled local coordinates
     */
    protected Vector3f pivotA;
    /**
     * copy of the pivot location: in physics-space coordinates if nodeB is
     * null, or else in B's scaled local coordinates
     */
    protected Vector3f pivotB;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected Constraint() {
    }

    /**
     * Instantiate a single-ended Constraint using the specified body at the
     * specified end.
     * <p>
     * To be effective, the Constraint must be added to the body's PhysicsSpace
     * and the body must be dynamic.
     *
     * @param body the body to constrain (not null, alias created)
     * @param bodyEnd at which end to attach the body (not null)
     * @param pivotInBody the pivot location in the body's scaled local
     * coordinates (not null, unaffected)
     */
    protected Constraint(PhysicsRigidBody body, JointEnd bodyEnd,
            Vector3f pivotInBody) {
        Validate.nonNull(body, "body");
        Validate.nonNull(bodyEnd, "body end");
        Validate.nonNull(pivotInBody, "pivot in body");

        switch (bodyEnd) {
            case A:
                nodeA = body;
                nodeB = null;
                pivotA = pivotInBody.clone();
                pivotB = null;
                nodeA.addJoint(this);
                break;

            case B:
                nodeA = null;
                nodeB = body;
                pivotA = null;
                pivotB = pivotInBody.clone();
                nodeB.addJoint(this);
                break;

            default:
                String message = "body end = " + bodyEnd.toString();
                throw new IllegalArgumentException(message);
        }
    }

    /**
     * Instantiate a single-ended Constraint using the specified body at the
     * specified end.
     * <p>
     * To be effective, the Constraint must be added to the body's PhysicsSpace
     * and the body must be dynamic.
     *
     * @param body the body to constrain (not null, alias created)
     * @param bodyEnd at which end to attach the body (not null)
     * @param pivotInBody the pivot location in the body's scaled local
     * coordinates (not null, unaffected)
     * @param pivotInWorld the pivot location in physics-space coordinates (not
     * null, unaffected)
     */
    protected Constraint(PhysicsRigidBody body, JointEnd bodyEnd,
            Vector3f pivotInBody, Vector3f pivotInWorld) {
        Validate.nonNull(body, "body");
        Validate.nonNull(bodyEnd, "body end");
        Validate.nonNull(pivotInBody, "pivot in body");
        Validate.nonNull(pivotInWorld, "pivot in world");

        switch (bodyEnd) {
            case A:
                nodeA = body;
                nodeB = null;
                pivotA = pivotInBody.clone();
                pivotB = pivotInWorld.clone();
                nodeA.addJoint(this);
                break;

            case B:
                nodeA = null;
                nodeB = body;
                pivotA = pivotInWorld.clone();
                pivotB = pivotInBody.clone();
                nodeB.addJoint(this);
                break;

            default:
                String message = "body end = " + bodyEnd.toString();
                throw new IllegalArgumentException(message);
        }
    }

    /**
     * Instantiate a double-ended Constraint.
     * <p>
     * To be effective, the Constraint must be added to the PhysicsSpace of both
     * bodies. Also, the bodies must be distinct and at least one of them must
     * be dynamic.
     *
     * @param nodeA the body for the A end (not null, alias created)
     * @param nodeB the body for the B end (not null, alias created)
     * @param pivotInA the pivot location in A's scaled local coordinates (not
     * null, unaffected)
     * @param pivotInB the pivot location in B's scaled local coordinates (not
     * null, unaffected)
     */
    protected Constraint(PhysicsBody nodeA, PhysicsBody nodeB,
            Vector3f pivotInA, Vector3f pivotInB) {
        if (nodeA == nodeB) {
            throw new IllegalArgumentException(
                    "The jointed bodies must be distinct.");
        }

        this.nodeA = nodeA;
        this.nodeB = nodeB;
        pivotA = pivotInA.clone();
        pivotB = pivotInB.clone();
        nodeA.addJoint(this);
        nodeB.addJoint(this);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Read the magnitude of the applied impulse. Requires feedback.
     *
     * @return impulse magnitude (&ge;0)
     * @throws IllegalStateException if feedback is not enabled
     */
    public float getAppliedImpulse() {
        if (!isFeedback()) {
            throw new IllegalStateException();
        }
        float result = getAppliedImpulse(objectId);

        assert result >= 0f : result;
        return result;
    }

    /**
     * Access the rigid body at the A end.
     *
     * @return the pre-existing rigid body, or null if none
     */
    public PhysicsRigidBody getBodyA() {
        PhysicsRigidBody result = null;
        if (nodeA instanceof PhysicsRigidBody) {
            result = (PhysicsRigidBody) nodeA;
        }
        return result;
    }

    /**
     * Access the rigid body at the B end.
     *
     * @return the pre-existing body, or null if none
     */
    public PhysicsRigidBody getBodyB() {
        PhysicsRigidBody result = null;
        if (nodeB instanceof PhysicsRigidBody) {
            result = (PhysicsRigidBody) nodeB;
        }
        return result;
    }

    /**
     * Read the breaking impulse threshold.
     *
     * @return the value
     */
    public float getBreakingImpulseThreshold() {
        float result = getBreakingImpulseThreshold(objectId);
        return result;
    }

    /**
     * Copy the location of the specified connection point in the specified
     * body.
     *
     * @param end which end of the Constraint to access (not null)
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in scaled local coordinates, either
     * storeResult or a new instance)
     */
    public Vector3f getPivot(JointEnd end, Vector3f storeResult) {
        Validate.nonNull(end, "end");

        switch (end) {
            case A:
                return getPivotA(storeResult);
            case B:
                return getPivotB(storeResult);
            default:
                throw new IllegalArgumentException("end = " + end.toString());
        }
    }

    /**
     * Copy the location of the connection point in the body at the A end.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in scaled local coordinates, either
     * storeResult or a new instance)
     */
    public Vector3f getPivotA(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        if (nodeA == null) {
            throw new IllegalArgumentException("No body at the A end.");
        }

        result.set(pivotA);
        return result;
    }

    /**
     * Copy the location of the connection point in the body at the B end.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in scaled local coordinates, either
     * storeResult or a new instance)
     */
    public Vector3f getPivotB(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        if (nodeB == null) {
            throw new IllegalArgumentException("No body at the B end.");
        }

        result.set(pivotB);
        return result;
    }

    /**
     * Test whether collisions are allowed between the linked bodies.
     *
     * @return true if collision are allowed, otherwise false
     */
    public boolean isCollisionBetweenLinkedBodies() {
        return collisionBetweenLinkedBodies;
    }

    /**
     * Test whether this Constraint has feedback enabled.
     *
     * @return true if enabled, otherwise false
     */
    public boolean isFeedback() {
        boolean result = needsFeedback(objectId);
        return result;
    }

    /**
     * Read the breaking impulse threshold.
     *
     * @param desiredThreshold the desired value (default=MAX_VALUE with SP
     * library or +Infinity with DP library)
     */
    public void setBreakingImpulseThreshold(float desiredThreshold) {
        setBreakingImpulseThreshold(objectId, desiredThreshold);
    }

    /**
     * Enable or disable collisions between the linked bodies. Changes take
     * effect when the Constraint is added to a PhysicsSpace.
     *
     * @param enable true to allow collisions, false to prevent them
     */
    public void setCollisionBetweenLinkedBodies(boolean enable) {
        collisionBetweenLinkedBodies = enable;
    }

    /**
     * Enable or disable this Constraint.
     *
     * @param enable true to enable, false to disable (default=true)
     */
    public void setEnabled(boolean enable) {
        setEnabled(objectId, enable);
    }

    /**
     * Enable or disable feedback for this Constraint.
     *
     * @param enable true to enable, false to disable (default=false)
     */
    public void setFeedback(boolean enable) {
        enableFeedback(objectId, enable);
    }
    // *************************************************************************
    // new protected methods

    /**
     * Finalize the btTypedConstraint.
     *
     * @param constraintId identifier of the btTypedConstraint (not 0)
     */
    native protected void finalizeNative(long constraintId);

    /**
     * Read common properties from a capsule.
     *
     * @param capsule the input capsule (not null, modified)
     * @throws IOException from the importer
     */
    final protected void readConstraintProperties(InputCapsule capsule)
            throws IOException {
        float bit = capsule.readFloat(
                "breakingImpulseThreshold", Float.MAX_VALUE);
        setBreakingImpulseThreshold(bit);

        boolean collisionFlag = capsule.readBoolean(
                "isCollisionBetweenLinkedBodies", true);
        setCollisionBetweenLinkedBodies(collisionFlag);

        boolean enabledFlag = capsule.readBoolean("isEnabled", true);
        setEnabled(enabledFlag);
    }
    // *************************************************************************
    // JmeCloneable methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned Constraint into a deep-cloned one, using the specified
     * Cloner and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this Constraint (not null)
     * @param original the instance from which this Constraint was
     * shallow-cloned (unused)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);

        pivotA = cloner.clone(pivotA);
        pivotB = cloner.clone(pivotB);
        // Each subclass must create the btTypedConstraint.
    }

    /**
     * Test whether this Constraint is enabled.
     *
     * @return true if enabled, otherwise false
     */
    @Override
    public boolean isEnabled() {
        boolean result = isEnabled(objectId);
        return result;
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public Constraint jmeClone() {
        try {
            Constraint clone = (Constraint) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }
    // *************************************************************************
    // Savable methods

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

        pivotA = (Vector3f) capsule.readSavable("pivotA", new Vector3f());
        pivotB = (Vector3f) capsule.readSavable("pivotB", new Vector3f());
        /*
         * Each subclass must create the btTypedConstraint and then
         * invoke readConstraintProperties().
         */
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

        capsule.write(pivotA, "pivotA", null);
        capsule.write(pivotB, "pivotB", null);

        capsule.write(getBreakingImpulseThreshold(),
                "breakingImpulseThreshold", Float.MAX_VALUE);
        capsule.write(isCollisionBetweenLinkedBodies(),
                "isCollisionBetweenLinkedBodies", true);
        capsule.write(isEnabled(), "isEnabled", true);
    }
    // *************************************************************************
    // Object methods

    /**
     * Finalize this Constraint just before it is destroyed. Should be invoked
     * only by a subclass or by the garbage collector.
     *
     * @throws Throwable ignored by the garbage collector
     */
    @Override
    protected void finalize() throws Throwable {
        super.finalize();
        logger.log(Level.FINE, "Finalizing Constraint {0}",
                Long.toHexString(objectId));
        finalizeNative(objectId);
    }
    // *************************************************************************
    // private methods

    native private void enableFeedback(long constraintId, boolean enable);

    native private float getAppliedImpulse(long constraintId);

    native private float getBreakingImpulseThreshold(long constraintId);

    native private boolean isEnabled(long constraintId);

    native private boolean needsFeedback(long constraintId);

    native private void setBreakingImpulseThreshold(long constraintId,
            float desiredThreshold);

    native private void setEnabled(long constraintId, boolean enable);
}
