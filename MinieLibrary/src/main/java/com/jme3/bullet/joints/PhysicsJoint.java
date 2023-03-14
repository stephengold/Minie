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

import com.jme3.bullet.NativePhysicsObject;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.util.clone.Cloner;
import com.jme3.util.clone.JmeCloneable;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * The abstract base class for physics joints based on Bullet's
 * btTypedConstraint, btSoftBody::Anchor, or btSoftBody::Joint.
 *
 * @author normenhansen
 */
abstract public class PhysicsJoint
        extends NativePhysicsObject
        implements JmeCloneable, Savable {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(PhysicsJoint.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagNodeA = "nodeA";
    final private static String tagNodeB = "nodeB";
    // *************************************************************************
    // fields

    /**
     * body A specified in the constructor, or null for a single-ended joint
     * with body B only
     */
    private PhysicsBody bodyA = null;
    /**
     * body B specified in the constructor, or null for a single-ended joint
     * with body A only
     */
    private PhysicsBody bodyB = null;
    /**
     * space where this joint is added, or null if none
     */
    private PhysicsSpace space = null;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a PhysicsJoint.
     */
    protected PhysicsJoint() { // to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Count how many ends this joint has.
     *
     * @return 1 if single-ended, 2 if double-ended
     */
    public int countEnds() {
        if (bodyA == null || bodyB == null) {
            return 1;
        } else {
            return 2;
        }
    }

    /**
     * Remove this joint from the joint lists of both ends.
     */
    public void destroy() {
        if (bodyA != null) {
            bodyA.removeJoint(this);
            ///bodyA = null;
        }
        if (bodyB != null) {
            bodyB.removeJoint(this);
            ///bodyB = null;
        }
    }

    /**
     * Test whether the specified body is an end of this joint.
     *
     * @param body the body to find (not null, unaffected)
     * @return the enum value if found, otherwise null
     */
    public JointEnd findEnd(PhysicsBody body) {
        Validate.nonNull(body, "body");

        if (body == bodyA) {
            return JointEnd.A;
        } else if (body == bodyB) {
            return JointEnd.B;
        } else {
            return null;
        }
    }

    /**
     * Access the remaining body of this joint.
     *
     * @param body (not null, unaffected)
     * @return the body at the other end (null if not found or the joint is
     * single-ended)
     */
    public PhysicsBody findOtherBody(PhysicsBody body) {
        Validate.nonNull(body, "body");
        assert bodyA != bodyB;

        if (body == bodyA) {
            return bodyB;
        } else if (body == bodyB) {
            return bodyA;
        } else {
            return null;
        }
    }

    /**
     * Access the body at the specified end of this joint.
     *
     * @param end which end of the joint to access (not null)
     * @return the pre-existing body, or null if none
     */
    public PhysicsBody getBody(JointEnd end) {
        Validate.nonNull(end, "end");

        switch (end) {
            case A:
                return bodyA;
            case B:
                return bodyB;
            default:
                throw new IllegalArgumentException("end = " + end);
        }
    }

    /**
     * Access the body at the joint's "A" end.
     *
     * @return the pre-existing body, or null if none
     */
    public PhysicsBody getBodyA() {
        return bodyA;
    }

    /**
     * Access the body at the joint's "B" end.
     *
     * @return the pre-existing body, or null if none
     */
    public PhysicsBody getBodyB() {
        return bodyB;
    }

    /**
     * Return the ID of the native object ({@code btTypedConstraint},
     * {@code btSoftBody::Anchor}, or {@code btSoftBody::Joint}).
     *
     * @return the native identifier (not zero)
     * @deprecated use {@link NativePhysicsObject#nativeId()}
     */
    @Deprecated
    final public long getObjectId() {
        long jointId = nativeId();
        return jointId;
    }

    /**
     * Access the PhysicsSpace where this joint is added.
     *
     * @return the pre-existing instance, or null if none
     */
    public PhysicsSpace getPhysicsSpace() {
        PhysicsSpace result = space;
        return result;
    }

    /**
     * Test whether this joint is enabled.
     *
     * @return true if enabled, otherwise false
     */
    abstract public boolean isEnabled();

    /**
     * Alter which PhysicsSpace this joint is added to. Do not invoke directly!
     * The field is updated automatically when added/removed.
     *
     * @param physicsSpace (may be null)
     */
    public void setPhysicsSpace(PhysicsSpace physicsSpace) {
        this.space = physicsSpace;
    }
    // *************************************************************************
    // new protected methods

    /**
     * Specify the body at the joint's "A" end.
     *
     * @param body the desired body (not null, alias created)
     */
    final protected void setBodyA(PhysicsBody body) {
        assert body != null;
        assert bodyA == null : bodyA;
        this.bodyA = body;
    }

    /**
     * Specify the body at the joint's "B" end.
     *
     * @param body the desired body (not null, alias created)
     */
    final protected void setBodyB(PhysicsBody body) {
        assert body != null;
        assert bodyB == null : bodyB;
        this.bodyB = body;
    }
    // *************************************************************************
    // JmeCloneable methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned object into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this joint (not null)
     * @param original the instance from which this joint was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        assert !hasAssignedNativeObject();
        PhysicsJoint old = (PhysicsJoint) original;
        assert old != this;
        assert old.hasAssignedNativeObject();

        PhysicsBody oldBodyA = old.bodyA;
        this.bodyA = cloner.clone(oldBodyA);
        if (bodyA != null && !bodyA.hasAssignedNativeObject()) {
            bodyA.cloneFields(cloner, oldBodyA);
        }
        assert bodyA == null || bodyA.hasAssignedNativeObject() : bodyA;

        PhysicsBody oldBodyB = old.bodyB;
        this.bodyB = cloner.clone(oldBodyB);
        if (bodyB != null && !bodyB.hasAssignedNativeObject()) {
            bodyB.cloneFields(cloner, oldBodyB);
        }
        assert bodyB == null || bodyB.hasAssignedNativeObject() : bodyA;

        this.space = null;
        /*
         * Each subclass must create the btTypedConstraint, btSoftBody::Anchor,
         * or btSoftBody::Joint and invoke setNativeId().
         */
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public PhysicsJoint jmeClone() {
        try {
            PhysicsJoint clone = (PhysicsJoint) clone();
            clone.unassignNativeObject();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }
    // *************************************************************************
    // Savable methods

    /**
     * De-serialize this joint from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        InputCapsule capsule = importer.getCapsule(this);

        this.bodyA = (PhysicsBody) capsule.readSavable(tagNodeA, null);
        this.bodyB = (PhysicsBody) capsule.readSavable(tagNodeB, null);
        /*
         * Each subclass must create the btTypedConstraint, btSoftBody::Anchor,
         * or btSoftBody::Joint and then read the remaining properties.
         */
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
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(bodyA, tagNodeA, null);
        capsule.write(bodyB, tagNodeB, null);
        // space is never written.
    }
    // *************************************************************************
    // NativePhysicsObject methods

    /**
     * Initialize the native ID.
     *
     * @param jointId the native identifier of the btTypedConstraint,
     * btSoftBody::Anchor, or btSoftBody::Joint (not zero)
     */
    @Override
    protected void setNativeId(long jointId) {
        super.setNativeId(jointId);
        logger.log(Level.FINE, "Created {0}.", this);
    }

    /**
     * Represent this joint as a String.
     *
     * @return a descriptive string of text (not null, not empty)
     */
    @Override
    public String toString() {
        String result = getClass().getSimpleName();
        result = result.replace("Joint", "");
        result = result.replace("Physics", "");
        result = result.replace("Point", "P");
        result = result.replace("Six", "6");
        if (hasAssignedNativeObject()) {
            long jointId = nativeId();
            result += "#" + Long.toHexString(jointId);
        } else {
            result += "#unassigned";
        }

        return result;
    }
}
