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
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * A 3 degree-of-freedom joint based on Bullet's btPoint2PointConstraint.
 * <p>
 * <i>From the Bullet manual:</i><br>
 * Point to point constraint limits the translation so that the local pivot
 * points of 2 rigidbodies match in worldspace. A chain of rigidbodies can be
 * connected using this constraint.
 *
 * @author normenhansen
 */
public class Point2PointJoint extends PhysicsJoint {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(Point2PointJoint.class.getName());
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public Point2PointJoint() {
    }

    /**
     * Instantiate a single-ended Point2PointJoint.
     * <p>
     * To be effective, the joint must be added to the physics space with the
     * body and the body must be dynamic.
     *
     * @param nodeA the body to constrain (not null, alias created)
     * @param pivotInA the pivot location in A's scaled local coordinates (not
     * null, unaffected)
     * @param pivotInWorld the pivot location in physics-space coordinates (not
     * null, unaffected)
     */
    public Point2PointJoint(PhysicsRigidBody nodeA, Vector3f pivotInA,
            Vector3f pivotInWorld) {
        super(nodeA, JointEnd.A, pivotInA, pivotInWorld);
        createJoint();
    }

    /**
     * Instantiate a double-ended Point2PointJoint.
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
    public Point2PointJoint(PhysicsRigidBody nodeA, PhysicsRigidBody nodeB,
            Vector3f pivotInA, Vector3f pivotInB) {
        super(nodeA, nodeB, pivotInA, pivotInB);
        createJoint();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Read the joint's damping ratio.
     *
     * @return the viscous damping ratio (0&rarr;no damping, 1&rarr;critically
     * damped)
     */
    public float getDamping() {
        return getDamping(objectId);
    }

    /**
     * Read the joint's impulse clamp.
     *
     * @return the clamp value
     */
    public float getImpulseClamp() {
        return getImpulseClamp(objectId);
    }

    /**
     * Read the joint's tau value.
     *
     * @return the tau value
     */
    public float getTau() {
        return getTau(objectId);
    }

    /**
     * Alter the joint's damping.
     *
     * @param value the desired viscous damping ratio (0&rarr;no damping,
     * 1&rarr;critically damped, default=1)
     */
    public void setDamping(float value) {
        setDamping(objectId, value);
    }

    /**
     * Alter the joint's impulse clamp.
     *
     * @param value the desired impulse clamp value (default=0)
     */
    public void setImpulseClamp(float value) {
        setImpulseClamp(objectId, value);
    }

    /**
     * Alter the joint's tau value.
     *
     * @param value the desired tau value (default=0.3)
     */
    public void setTau(float value) {
        setTau(objectId, value);
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

        createJoint();

        Point2PointJoint old = (Point2PointJoint) original;

        float bit = old.getBreakingImpulseThreshold();
        setBreakingImpulseThreshold(bit);

        boolean enableJoint = old.isEnabled();
        setEnabled(enableJoint);

        setDamping(old.getDamping());
        setImpulseClamp(old.getImpulseClamp());
        setTau(old.getTau());
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public Point2PointJoint jmeClone() {
        try {
            Point2PointJoint clone = (Point2PointJoint) super.clone();
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

        createJoint();

        setBreakingImpulseThreshold(breakingImpulseThreshold);
        setEnabled(isEnabled);

        setDamping(capsule.readFloat("damping", 1f));
        setDamping(capsule.readFloat("tau", 0.3f));
        setDamping(capsule.readFloat("impulseClamp", 0f));
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

        capsule.write(getDamping(), "damping", 1f);
        capsule.write(getTau(), "tau", 0.3f);
        capsule.write(getImpulseClamp(), "impulseClamp", 0f);
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
        assert pivotB != null;

        if (nodeB == null) {
            /*
             * Create a single-ended joint.  Bullet assumes single-ended
             * btPoint2PointConstraints are satisfied at creation, so we
             * temporarily re-position the body to satisfy the constraint.
             */
            Vector3f saveLocation = nodeA.getPhysicsLocation(null);

            Vector3f offset = pivotB.subtract(pivotA);
            nodeA.setPhysicsLocation(offset);
            objectId = createJoint1(nodeA.getObjectId(), pivotA);

            nodeA.setPhysicsLocation(saveLocation);

        } else {
            /*
             * Create a double-ended joint.
             */
            objectId = createJoint(nodeA.getObjectId(), nodeB.getObjectId(),
                    pivotA, pivotB);
        }
        assert objectId != 0L;
        logger2.log(Level.FINE, "Created Joint {0}",
                Long.toHexString(objectId));
    }

    native private long createJoint(long bodyIdA, long bodyIdB,
            Vector3f pivotInA, Vector3f pivotInB);

    native private long createJoint1(long bodyIdA, Vector3f pivotInA);

    native private float getDamping(long jointId);

    native private float getImpulseClamp(long jointId);

    native private float getTau(long jointId);

    native private void setDamping(long jointId, float desiredDamping);

    native private void setImpulseClamp(long jointId, float desiredClamp);

    native private void setTau(long jointId, float desiredTau);
}
