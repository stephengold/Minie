/*
 * Copyright (c) 2020-2023 jMonkeyEngine
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
package com.jme3.bullet;

import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.objects.MultiBodyCollider;
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
import com.jme3.util.clone.JmeCloneable;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A single link in a MultiBody, based on Bullet's {@code btMultibodyLink}.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class MultiBodyLink
        extends NativePhysicsObject
        implements JmeCloneable, Savable {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(MultiBodyLink.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagCollider = "collider";
    final private static String tagLinkIndex = "linkIndex";
    final private static String tagMultiBody = "multiBody";
    final private static String tagNumDofs = "numDofs";
    final private static String tagParentLink = "parentLink";
    // *************************************************************************
    // fields

    /**
     * index of this link in its MultiBody, which is also the index of the
     * {@code btMultiBodyLink} in its {@code btMultiBody} (&ge;0)
     */
    private int linkIndex;
    /**
     * copy of the number of degrees of freedom in this link's joint
     */
    private int numDofs;
    /**
     * copy of the ID of the {@code btMultiBody}
     */
    private long multiBodyId;
    /**
     * MultiBody that contains this link
     */
    private MultiBody multiBody;
    /**
     * collider for this link, or null if none
     */
    private MultiBodyCollider collider = null;
    /**
     * parent of this link, or null if joined to the base
     */
    private MultiBodyLink parentLink;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected MultiBodyLink() {
    }

    /**
     * Instantiate a MultiBodyLink.
     *
     * @param multiBody the MultiBody that contains this link (not null, alias
     * created)
     * @param index the index of this link in the MultiBody (&ge;0)
     */
    MultiBodyLink(MultiBody multiBody, int index) {
        Validate.nonNull(multiBody, "multibody");
        Validate.nonNegative(index, "index");

        this.linkIndex = index;
        this.multiBody = multiBody;

        multiBodyId = multiBody.nativeId();

        long linkId = getLinkId(multiBodyId, index);
        super.setNativeIdNotTracked(linkId);

        numDofs = getDofCount(linkId);

        int parentIndex = getParentIndex(linkId);
        if (parentIndex == -1) {
            parentLink = null;
        } else {
            parentLink = multiBody.getLink(parentIndex);
        }
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Add a collider for this link.
     *
     * @param shape (not null, alias created)
     * @return a new collider
     */
    public MultiBodyCollider addCollider(CollisionShape shape) {
        Validate.nonNull(shape, "shape");
        assert collider == null : collider;

        collider = new MultiBodyCollider(multiBody, linkIndex);
        long linkId = nativeId();
        long colliderId = collider.nativeId();
        setCollider(linkId, colliderId);
        collider.attachShape(shape);

        return collider;
    }

    /**
     * Add an external force to this link.
     *
     * @param force the force to add (in physics-space coordinates, not null,
     * unaffected)
     */
    public void addConstraintForce(Vector3f force) {
        Validate.finite(force, "force");

        long linkId = nativeId();
        addConstraintForce(linkId, force);
    }

    /**
     * Add an external torque to this link.
     *
     * @param torque the torque to add (in physics-space coordinates, not null,
     * unaffected)
     */
    public void addConstraintTorque(Vector3f torque) {
        Validate.finite(torque, "torque");

        long linkId = nativeId();
        addConstraintTorque(linkId, torque);
    }

    /**
     * Add an external force to this link.
     *
     * @param force the force to add (in physics-space coordinates, not null,
     * unaffected)
     */
    public void addForce(Vector3f force) {
        Validate.finite(force, "force");

        long linkId = nativeId();
        addForce(linkId, force);
    }

    /**
     * Add an external torque to this link's joint.
     *
     * @param dofIndex which degree of freedom (&ge;0, &lt;numDofs)
     * @param torque the torque to add
     */
    public void addJointTorque(int dofIndex, float torque) {
        Validate.inRange(dofIndex, "DOF index", 0, numDofs - 1);

        long linkId = nativeId();
        addJointTorque(linkId, dofIndex, torque);
    }

    /**
     * Add an external torque to this link.
     *
     * @param torque the torque to add (in physics-space coordinates, not null,
     * unaffected)
     */
    public void addTorque(Vector3f torque) {
        Validate.finite(torque, "torque");

        long linkId = nativeId();
        addTorque(linkId, torque);
    }

    /**
     * Determine the net applied force on this link.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the force vector (in physics-space coordinates, either
     * storeResult or a new vector, not null)
     */
    public Vector3f appliedForce(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long linkId = nativeId();
        getAppliedForce(linkId, result);

        return result;
    }

    /**
     * Determine the net applied torque on this link.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the torque vector (in physics-space coordinates, either
     * storeResult or a new vector, not null)
     */
    public Vector3f appliedTorque(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long linkId = nativeId();
        getAppliedTorque(linkId, result);

        return result;
    }

    /**
     * Determine the direction of the joint's axis for a planar, prismatic, or
     * revolute joint.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the direction (a unit vector, either storeResult or a new vector)
     */
    public Vector3f axis(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        MultiBodyJointType jointType = jointType();
        long linkId = nativeId();
        switch (jointType) {
            case Planar:
            case Revolute:
                getAxisTop(linkId, 0, result);
                break;

            case Prismatic:
                getAxisBottom(linkId, 0, result);
                break;

            default:
                throw new IllegalStateException("jointType = " + jointType);
        }

        return result;
    }

    /**
     * Determine the net constraint force on this link.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the force vector (in physics-space coordinates, either
     * storeResult or a new vector, not null)
     */
    public Vector3f constraintForce(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long linkId = nativeId();
        getConstraintForce(linkId, result);

        return result;
    }

    /**
     * Determine the net constraint torque on this link.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the torque vector (in physics-space coordinates, either
     * storeResult or a new vector, not null)
     */
    public Vector3f constraintTorque(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long linkId = nativeId();
        getConstraintTorque(linkId, result);

        return result;
    }

    /**
     * Count the degrees of freedom in the joint.
     *
     * @return the count (&ge;0)
     */
    public int countDofs() {
        assert numDofs == getDofCount(nativeId());
        return numDofs;
    }

    /**
     * Count the position variables in the joint.
     *
     * @return the count (&ge;0)
     */
    public int countPositionVariables() {
        long linkId = nativeId();
        int result = getPosVarCount(linkId);

        return result;
    }

    /**
     * Access the collider for this link.
     *
     * @return the pre-existing instance, or null if none
     */
    public MultiBodyCollider getCollider() {
        if (collider == null) {
            assert getCollider(multiBodyId, linkIndex) == 0L;
        } else {
            assert getCollider(multiBodyId, linkIndex)
                    == collider.nativeId();
        }

        return collider;
    }

    /**
     * Access the MultiBody that contains this link.
     *
     * @return the pre-existing instance (not null)
     */
    public MultiBody getMultiBody() {
        assert multiBody != null;
        return multiBody;
    }

    /**
     * Determine the parent of this link.
     *
     * @return the pre-existing instance, or null if this link's parent is the
     * base
     */
    public MultiBodyLink getParentLink() {
        return parentLink;
    }

    /**
     * Determine the index of this link in its MultiBody.
     *
     * @return the index (&ge;0)
     */
    public int index() {
        assert linkIndex >= 0 : linkIndex;
        return linkIndex;
    }

    /**
     * Determine the rotational inertia of this link.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the principal (diagonal) components of the inertia tensor (in the
     * link's local coordinates, either storeResult or a new vector, not null)
     */
    public Vector3f inertia(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long linkId = nativeId();
        getInertiaLocal(linkId, result);

        return result;
    }

    /**
     * Test whether collisions with the immediate parent link are enabled.
     *
     * @return true if collisions are enabled, otherwise false
     */
    public boolean isCollisionWithParent() {
        long linkId = nativeId();
        int flags = getFlags(linkId);
        int disableCollisionWithParentFlag = 0x1;
        if ((flags & disableCollisionWithParentFlag) != 0x0) {
            return false;
        } else {
            return true;
        }
    }

    /**
     * Determine the position of the indexed DOF.
     *
     * @param dofIndex which degree of freedom (&ge;0, &lt;numDofs)
     * @return the position
     */
    public float jointPosition(int dofIndex) {
        Validate.inRange(dofIndex, "DOF index", 0, numDofs - 1);

        long linkId = nativeId();
        float result = getJointPos(linkId, dofIndex);

        return result;
    }

    /**
     * Determine the torque applied to the indexed DOF using
     * {@link #addJointTorque(int, float)}, which is zeroed after each
     * simulation step.
     *
     * @param dofIndex which degree of freedom (&ge;0, &lt;numDofs)
     * @return the torque
     */
    public float jointTorque(int dofIndex) {
        Validate.inRange(dofIndex, "DOF index", 0, numDofs - 1);

        long linkId = nativeId();
        float result = getJointTorque(linkId, dofIndex);

        return result;
    }

    /**
     * Determine the type of joint between this link and its parent.
     *
     * @return an enum value (not null)
     */
    public MultiBodyJointType jointType() {
        long linkId = nativeId();
        int ordinal = getJointType(linkId);
        MultiBodyJointType result = MultiBodyJointType.values()[ordinal];

        return result;
    }

    /**
     * Determine the velocity of the indexed DOF.
     *
     * @param dofIndex which degree of freedom (&ge;0, &lt;numDofs)
     * @return the velocity
     */
    public float jointVelocity(int dofIndex) {
        Validate.inRange(dofIndex, "DOF index", 0, numDofs - 1);
        float result = getJointVel(multiBodyId, linkIndex, dofIndex);
        return result;
    }

    /**
     * Determine the location of this link's center of mass.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the location vector (in physics-space coordinates, either
     * storeResult or a new vector, not null)
     */
    public Vector3f location(Vector3f storeResult) {
        Vector3f result;
        if (storeResult == null) {
            result = new Vector3f();
        } else {
            result = storeResult.zero();
        }
        localPosToWorld(multiBodyId, linkIndex, result);

        return result;
    }

    /**
     * Determine the mass of this link.
     *
     * @return the mass (in physics-space units, &gt;0)
     */
    public float mass() {
        long linkId = nativeId();
        float result = getMass(linkId);

        return result;
    }

    /**
     * Determine the orientation of the link relative to its parent when Q=0.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the orientation (either storeResult or a new Quaternion)
     */
    public Quaternion orientation(Quaternion storeResult) {
        Quaternion result
                = (storeResult == null) ? new Quaternion() : storeResult;

        long linkId = nativeId();
        getQ0Parent2LinkRotation(linkId, result);

        return result;
    }

    /**
     * Determine the offset of the link's center relative to its parent's center
     * for a planar joint.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the offset (in this link's coordinate system, either storeResult
     * or a new vector)
     */
    public Vector3f parent2Link(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        assert jointType() == MultiBodyJointType.Planar;

        long linkId = nativeId();
        getEVector(linkId, result);

        return result;
    }

    /**
     * Determine the offset from the parent's center to the pivot for a
     * non-planar joint.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the offset (in the parent's coordinate system, either storeResult
     * or a new vector)
     */
    public Vector3f parent2Pivot(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        assert jointType() != MultiBodyJointType.Planar;

        long linkId = nativeId();
        getEVector(linkId, result);

        return result;
    }

    /**
     * Determine the offset from the pivot to this link's center for a
     * non-planar joint.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the offset (in this link's coordinate system, either storeResult
     * or a new vector)
     */
    public Vector3f pivot2Link(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        assert jointType() != MultiBodyJointType.Planar;

        long linkId = nativeId();
        getDVector(linkId, result);

        return result;
    }

    /**
     * Alter the position of the indexed DOF.
     *
     * @param dofIndex which degree of freedom (&ge;0, &lt;numDofs)
     * @param position the desired position
     */
    public void setJointPosition(int dofIndex, float position) {
        Validate.inRange(dofIndex, "DOF index", 0, numDofs - 1);
        setJointPos(multiBodyId, linkIndex, dofIndex, position);
    }

    /**
     * Alter the velocity of the indexed DOF.
     *
     * @param dofIndex which degree of freedom (&ge;0, &lt;numDofs)
     * @param velocity the desired velocity
     */
    public void setJointVelocity(int dofIndex, float velocity) {
        Validate.inRange(dofIndex, "DOF index", 0, numDofs - 1);
        setJointVel(multiBodyId, linkIndex, dofIndex, velocity);
    }

    /**
     * Determine the transform of this link. Assumes the physics simulation has
     * been stepped.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the Transform (either storeResult or a new instance, not null)
     */
    public Transform worldTransform(Transform storeResult) {
        Transform result
                = (storeResult == null) ? new Transform() : storeResult;

        long linkId = nativeId();
        getWorldTransform(linkId, result);

        return result;
    }
    // *************************************************************************
    // JmeCloneable methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned object into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this object (not null)
     * @param original the instance from which this object was shallow-cloned
     * (unused)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        this.multiBody = cloner.clone(multiBody);
        this.multiBodyId = multiBody.nativeId();
        this.parentLink = cloner.clone(parentLink);

        long linkId = getLinkId(multiBodyId, linkIndex);
        reassignNativeId(linkId);

        this.collider = cloner.clone(collider);
    }

    /**
     * Create a shallow clone for the JME cloner. Note that the cloned MultiBody
     * won't be added to any PhysicsSpace, even if the original was.
     *
     * @return a new instance
     */
    @Override
    public MultiBodyLink jmeClone() {
        try {
            MultiBodyLink clone = (MultiBodyLink) clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }
    // *************************************************************************
    // Savable methods

    /**
     * De-serialize this object from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        InputCapsule capsule = importer.getCapsule(this);

        this.collider
                = (MultiBodyCollider) capsule.readSavable(tagCollider, null);
        this.linkIndex = capsule.readInt(tagLinkIndex, -1);
        this.multiBody = (MultiBody) capsule.readSavable(tagMultiBody, null);
        this.numDofs = capsule.readInt(tagNumDofs, 0);
        this.parentLink
                = (MultiBodyLink) capsule.readSavable(tagParentLink, null);
    }

    /**
     * Serialize this object to the specified exporter, for example when saving
     * to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(collider, tagCollider, null);
        capsule.write(linkIndex, tagLinkIndex, -1);
        capsule.write(multiBody, tagMultiBody, null);
        capsule.write(numDofs, tagNumDofs, 0);
        capsule.write(parentLink, tagParentLink, null);
    }
    // *************************************************************************
    // native private methods

    native private static void
            addConstraintForce(long linkId, Vector3f forceVector);

    native private static void
            addConstraintTorque(long linkId, Vector3f torqueVector);

    native private static void addForce(long linkId, Vector3f forceVector);

    native private static void
            addJointTorque(long linkId, int dofIndex, float torque);

    native private static void addTorque(long linkId, Vector3f torqueVector);

    native private static void
            getAppliedForce(long linkId, Vector3f storeVector);

    native private static void
            getAppliedTorque(long linkId, Vector3f storeVector);

    native private static void
            getAxisBottom(long linkId, int dofIndex, Vector3f storeVector);

    native private static void
            getAxisTop(long linkId, int dofIndex, Vector3f storeVector);

    native private static long getCollider(long multiBodyId, int linkIndex);

    native private static void
            getConstraintForce(long linkId, Vector3f storeVector);

    native private static void
            getConstraintTorque(long linkId, Vector3f storeVector);

    native private static int getDofCount(long linkId);

    native private static void getDVector(long linkId, Vector3f storeVector);

    native private static void getEVector(long linkId, Vector3f storeVector);

    native private static int getFlags(long multiBodyId);

    native private static void
            getInertiaLocal(long linkId, Vector3f storeVector);

    native private static float getJointPos(long linkId, int dofIndex);

    native private static float getJointTorque(long linkId, int dofIndex);

    native private static int getJointType(long linkId);

    native private static float
            getJointVel(long multiBodyId, int linkIndex, int dofIndex);

    native private static long getLinkId(long multiBodyId, int linkIndex);

    native private static float getMass(long linkId);

    native private static void
            getParent2LinkRotation(long linkId, Quaternion storeQuaternion);

    native private static int getParentIndex(long linkId);

    native private static int getPosVarCount(long linkId);

    native private static void
            getQ0Parent2LinkRotation(long linkId, Quaternion storeQuaternion);

    native private static void
            getWorldTransform(long linkId, Transform storeTransform);

    native private static void localFrameToWorld(
            long multiBodyId, int linkIndex, Matrix3f rotationMatrix);

    native private static void localPosToWorld(
            long multiBodyId, int linkIndex, Vector3f locationVector);

    native private static void setCollider(long linkId, long colliderId);

    native private static void setJointPos(long multiBodyId, int linkIndex,
            int dofIndex, float positionVector);

    native private static void setJointVel(long multiBodyId, int linkIndex,
            int dofIndex, float velocityVector);

    native private static void worldPosToLocal(
            long multiBodyId, int linkIndex, Vector3f locationVector);
}
