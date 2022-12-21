/*
 * Copyright (c) 2019-2022 jMonkeyEngine
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
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A PhysicsJoint to join a particular node of a soft body (A) to a rigid body
 * (B), based on Bullet's btSoftBody::Anchor.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class Anchor extends PhysicsJoint {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(Anchor.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagAllowCollisions = "allowCollisions";
    final private static String tagInfluence = "influence";
    final private static String tagNodeIndexA = "nodeIndexA";
    final private static String tagPivotInB = "pivotInB";
    // *************************************************************************
    // fields

    /**
     * true&rarr;allow collisions between the bodies, false&rarr;disallow such
     * collisions
     */
    private boolean allowCollisions;
    /**
     * how much influence the anchor has on the bodies (&ge;0, &le;1, 0&rarr;no
     * influence, 1&rarr;strong influence)
     */
    private float influence = 1f;
    /**
     * which node in body A to connect (&ge;0, &lt;numNodes)
     */
    private int nodeIndexA;
    /**
     * copy of the pivot location in body B's local coordinates (not null)
     */
    private Vector3f pivotInB;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected Anchor() {
    }

    /**
     * Instantiate an anchor with influence=1.
     *
     * @param softBodyA the soft body for the A end (not null, alias created)
     * @param nodeIndexA the index of the node for the A end (&ge;0,
     * &lt;numNodes)
     * @param rigidBodyB the rigid body for the B end (not null, alias created)
     * @param pivotInB the pivot location in B's scaled local coordinates (not
     * null, unaffected)
     * @param allowCollisions true&rarr;allow collisions between the bodies,
     * false&rarr;disallow such collisions
     */
    public Anchor(PhysicsSoftBody softBodyA, int nodeIndexA,
            PhysicsRigidBody rigidBodyB, Vector3f pivotInB,
            boolean allowCollisions) {
        int numNodes = softBodyA.countNodes();
        Validate.inRange(nodeIndexA, "node index", 0, numNodes - 1);
        Validate.nonNull(rigidBodyB, "rigid body B");
        Validate.finite(pivotInB, "pivot location");

        setBodyA(softBodyA);
        softBodyA.addJoint(this);
        this.nodeIndexA = nodeIndexA;

        setBodyB(rigidBodyB);
        rigidBodyB.addJoint(this);
        this.allowCollisions = allowCollisions;
        this.pivotInB = pivotInB.clone();

        createAnchor();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the location of the connection point in the rigid body at the B end.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in B's scaled local coordinates, either
     * storeResult or a new instance)
     */
    public Vector3f copyPivot(Vector3f storeResult) {
        Vector3f result;
        if (storeResult == null) {
            result = pivotInB.clone();
        } else {
            result = storeResult.set(pivotInB);
        }

        return result;
    }

    /**
     * Access the rigid body at the B end.
     *
     * @return the pre-existing body (not null)
     */
    public PhysicsRigidBody getRigidBody() {
        PhysicsRigidBody result = (PhysicsRigidBody) getBodyB();

        assert result != null;
        return result;
    }

    /**
     * Access the soft body at the A end.
     *
     * @return the pre-existing soft body (not null)
     */
    public PhysicsSoftBody getSoftBody() {
        PhysicsSoftBody result = (PhysicsSoftBody) getBodyA();

        assert result != null;
        return result;
    }

    /**
     * Read how much influence the anchor has on the bodies.
     *
     * @return the amount (&ge;0, &le;1, 0&rarr;no influence, 1&rarr;strong
     * influence)
     */
    public float influence() {
        return influence;
    }

    /**
     * Test whether collisions are allowed between the bodies.
     *
     * @return true if collisions are allowed, otherwise false
     */
    public boolean isAllowCollision() {
        return allowCollisions;
    }

    /**
     * Read the index of the anchored node in body A.
     *
     * @return the node index (&ge;0)
     */
    public int nodeIndex() {
        assert nodeIndexA >= 0 : nodeIndexA;
        return nodeIndexA;
    }

    /**
     * Alter how much influence the anchor has on the bodies.
     *
     * @param amount the degree of influence (&ge;0, &le;1, 0&rarr;no influence,
     * 1&rarr;strong influence, default=1)
     */
    public void setInfluence(float amount) {
        Validate.fraction(amount, "amount");

        this.influence = amount;
        long anchorId = nativeId();
        setInfluence(anchorId, amount);
    }

    /**
     * Alter the pivot location in B's local coordinates.
     *
     * @param location the desired location (not null, unaffected)
     */
    public void setPivotInB(Vector3f location) {
        Validate.nonNull(location, "location");

        pivotInB.set(location);
        long anchorId = nativeId();
        setPivotInB(anchorId, location);
    }
    // *************************************************************************
    // PhysicsJoint methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned Anchor into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this Anchor (not null)
     * @param original the instance from which this Anchor was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        assert !hasAssignedNativeObject();
        Anchor old = (Anchor) original;
        assert old != this;
        assert old.hasAssignedNativeObject();

        super.cloneFields(cloner, original);
        if (hasAssignedNativeObject()) {
            return;
        }

        this.pivotInB = cloner.clone(pivotInB);
        createAnchor();
    }

    /**
     * Test whether this Anchor is enabled.
     *
     * @return true
     */
    @Override
    public boolean isEnabled() {
        return true;
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

        this.allowCollisions
                = capsule.readBoolean(tagAllowCollisions, true);
        this.influence = capsule.readFloat(tagInfluence, 1f);
        this.nodeIndexA = capsule.readInt(tagNodeIndexA, 0);
        this.pivotInB
                = (Vector3f) capsule.readSavable(tagPivotInB, new Vector3f());

        createAnchor();
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

        capsule.write(allowCollisions, tagAllowCollisions, true);
        capsule.write(influence, tagInfluence, 1f);
        capsule.write(nodeIndexA, tagNodeIndexA, 0);
        capsule.write(pivotInB, tagPivotInB, null);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Create the configured btSoftBody::Anchor.
     */
    private void createAnchor() {
        PhysicsSoftBody a = getSoftBody();
        long aId = a.nativeId();
        assert nodeIndexA >= 0 : nodeIndexA;
        assert nodeIndexA < a.countNodes() : nodeIndexA;

        PhysicsRigidBody b = getRigidBody();
        long bId = b.nativeId();

        long anchorId = createAnchor(
                aId, nodeIndexA, bId, pivotInB, allowCollisions, influence);
        setNativeIdNotTracked(anchorId);
    }
    // *************************************************************************
    // native private methods

    native private static long createAnchor(long softBodyId, int nodeIndex,
            long rigidBodyId, Vector3f initialLocation, boolean allowCollisions,
            float influence);

    native private static void setInfluence(long anchorId, float influence);

    native private static void setPivotInB(long anchorId, Vector3f location);
}
