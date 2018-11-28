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
import com.jme3.export.Savable;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import com.jme3.util.clone.JmeCloneable;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * The abstract base class for physics joints based on Bullet's
 * btTypedConstraint. A physics joint can be single-ended or double-ended:
 * <ul>
 * <li>A single-ended joint constrains the motion of a dynamic rigid body in
 * physics space.</li>
 * <li>A double-ended joint connects 2 rigid bodies together in the same physics
 * space. One of the bodies must be dynamic.</li>
 * </ul>
 * Subclasses include: ConeJoint, HingeJoint, Point2PointJoint, and SixDofJoint.
 *
 * @author normenhansen
 */
abstract public class PhysicsJoint
        implements JmeCloneable, Savable {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(PhysicsJoint.class.getName());
    // *************************************************************************
    // fields TODO re-order

    /**
     * Unique identifier of the btTypedConstraint. Subtype constructors are
     * responsible for setting this to a non-zero value. Once set, the
     * identifier never changes.
     */
    protected long objectId = 0L;
    /**
     * body A specified in the constructor, or null for a single-ended joint
     * with body B
     */
    protected PhysicsRigidBody nodeA;
    /**
     * body B specified in the constructor, or null for a single-ended joint
     * with body A
     */
    protected PhysicsRigidBody nodeB;
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
    /**
     * true IFF bodies A and B are allowed to collide
     */
    private boolean collisionBetweenLinkedBodies = true;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public PhysicsJoint() {
    }

    /**
     * Instantiate a single-ended PhysicsJoint using the specified body at the
     * specified end.
     * <p>
     * To be effective, the joint must be added to the physics space with the
     * body and the body must be dynamic.
     *
     * @param body the body to constrain (not null, alias created)
     * @param bodyEnd at which end to attach the body (not null)
     * @param pivotInBody the pivot location in the body's scaled local
     * coordinates (not null, unaffected)
     * @param pivotInWorld the pivot location in physics-space coordinates (not
     * null, unaffected)
     */
    protected PhysicsJoint(PhysicsRigidBody body, JointEnd bodyEnd,
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
     * Instantiate a double-ended PhysicsJoint.
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
    protected PhysicsJoint(PhysicsRigidBody nodeA, PhysicsRigidBody nodeB,
            Vector3f pivotInA, Vector3f pivotInB) {
        if (nodeA == nodeB) {
            throw new IllegalArgumentException("The bodies must be distinct.");
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
     * Count how many ends this joint has.
     *
     * @return 1 if single-ended, 2 if double-ended
     */
    public int countEnds() {
        if (nodeA == null || nodeB == null) {
            return 1;
        } else {
            return 2;
        }
    }

    /**
     * Remove this joint from the joint lists of both connected bodies.
     */
    public void destroy() {
        if (nodeA != null) {
            nodeA.removeJoint(this);
        }
        if (nodeB != null) {
            nodeB.removeJoint(this);
        }
    }

    /**
     * Read the magnitude of the applied impulse.
     *
     * @return impulse magnitude (&ge;0)
     */
    public float getAppliedImpulse() {
        float result = getAppliedImpulse(objectId);
        assert result >= 0f : result;
        return result;
    }

    /**
     * Access the specified body.
     *
     * @param end which end of the joint to access (not null)
     * @return the pre-existing body, or null if none
     */
    public PhysicsRigidBody getBody(JointEnd end) {
        switch (end) {
            case A:
                return nodeA;
            case B:
                return nodeB;
            default:
                throw new IllegalArgumentException("end = " + end.toString());
        }
    }

    /**
     * Access the body at the A end.
     *
     * @return the pre-existing body, or null if none
     */
    public PhysicsRigidBody getBodyA() {
        return nodeA;
    }

    /**
     * Access the body at the B end.
     *
     * @return the pre-existing body, or null if none
     */
    public PhysicsRigidBody getBodyB() {
        return nodeB;
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
     * Read the id of the btTypedConstraint.
     *
     * @return the unique identifier (not zero)
     */
    public long getObjectId() {
        assert objectId != 0L;
        return objectId;
    }

    /**
     * Copy the location of the specified connection point in the specified
     * body.
     *
     * @param end which end of the joint to access (not null)
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
     * Copy the location of the connection point in physics space (for a
     * single-ended joint).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in scaled local coordinates, either
     * storeResult or a new instance)
     */
    public Vector3f getPivotInWorld(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        if (nodeA == null) {
            result.set(pivotA);
        } else if (nodeB == null) {
            result.set(pivotB);
        } else {
            throw new IllegalStateException("Joint is double-ended.");
        }

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
     * Test whether this joint is enabled.
     *
     * @return true if enabled, otherwise false
     */
    public boolean isEnabled() {
        boolean result = isEnabled(objectId);
        return result;
    }

    /**
     * Read the breaking impulse threshold.
     *
     * @param desiredThreshold the desired value (default=MAX_VALUE)
     */
    public void setBreakingImpulseThreshold(float desiredThreshold) {
        setBreakingImpulseThreshold(objectId, desiredThreshold);
    }

    /**
     * Enable or disable collisions between the linked bodies. Changes take
     * effect when the joint is added to a PhysicsSpace.
     *
     * @param enable true to allow collisions, false to prevent them
     */
    public void setCollisionBetweenLinkedBodies(boolean enable) {
        collisionBetweenLinkedBodies = enable;
    }

    /**
     * Enable or disable this joint.
     *
     * @param enable true to enable, false to disable (default=true)
     */
    public void setEnabled(boolean enable) {
        setEnabled(objectId, enable);
    }
    // *************************************************************************
    // JmeCloneable methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned object into a deep-cloned one, using the specified cloner
     * and original to resolve copied fields.
     *
     * @param cloner the cloner that's cloning this shape (not null)
     * @param original the instance from which this instance was shallow-cloned
     * (unused)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        nodeA = cloner.clone(nodeA);
        nodeB = cloner.clone(nodeB);
        pivotA = cloner.clone(pivotA);
        pivotB = cloner.clone(pivotB);
        objectId = 0L; // subclass must create the btCollisionObject
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public PhysicsJoint jmeClone() {
        try {
            PhysicsJoint clone = (PhysicsJoint) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }
    // *************************************************************************
    // Savable methods

    /**
     * De-serialize this joint, for example when loading from a J3O file.
     *
     * @param im importer (not null)
     * @throws IOException from importer
     */
    @Override
    public void read(JmeImporter im) throws IOException {
        InputCapsule capsule = im.getCapsule(this);

        nodeA = (PhysicsRigidBody) capsule.readSavable("nodeA", null);
        nodeB = (PhysicsRigidBody) capsule.readSavable("nodeB", null);
        pivotA = (Vector3f) capsule.readSavable("pivotA", new Vector3f());
        pivotB = (Vector3f) capsule.readSavable("pivotB", null);
        /*
         * Each subclass must create the btCollisionObject and
         * read the breaking impulse threshold and the enabled flag.
         */
    }

    /**
     * Serialize this joint, for example when saving to a J3O file.
     *
     * @param ex exporter (not null)
     * @throws IOException from exporter
     */
    @Override
    public void write(JmeExporter ex) throws IOException {
        OutputCapsule capsule = ex.getCapsule(this);

        capsule.write(nodeA, "nodeA", null);
        capsule.write(nodeB, "nodeB", null);
        capsule.write(pivotA, "pivotA", null);
        capsule.write(pivotB, "pivotB", null);

        capsule.write(getBreakingImpulseThreshold(), "breakingImpulseThreshold",
                Float.MAX_VALUE);
        capsule.write(isEnabled(), "isEnabled", true);
    }
    // *************************************************************************
    // Object methods

    /**
     * Finalize this physics joint just before it is destroyed. Should be
     * invoked only by a subclass or by the garbage collector.
     *
     * @throws Throwable ignored by the garbage collector
     */
    @Override
    protected void finalize() throws Throwable {
        super.finalize();
        logger.log(Level.FINE, "Finalizing Joint {0}",
                Long.toHexString(objectId));
        finalizeNative(objectId);
    }
    // *************************************************************************
    // private methods

    native private void finalizeNative(long jointId);

    native private float getAppliedImpulse(long jointId);

    native private float getBreakingImpulseThreshold(long jointId);

    native private boolean isEnabled(long jointId);

    native private void setBreakingImpulseThreshold(long jointId,
            float desiredThreshold);

    native private void setEnabled(long jointId, boolean enable);
}
