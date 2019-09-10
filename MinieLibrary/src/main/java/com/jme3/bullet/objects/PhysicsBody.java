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
package com.jme3.bullet.objects;

import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.export.InputCapsule;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.ArrayList;
import jme3utilities.Validate;

/**
 * The abstract base class for rigid bodies and soft bodies.
 *
 * @author Stephen Gold sgold@sonic.net
 */
abstract public class PhysicsBody extends PhysicsCollisionObject {
    // *************************************************************************
    // constants and loggers

    /**
     * magic mass value used to specify a static rigid body or soft-body node
     */
    final public static float massForStatic = 0f;
    /**
     * field name for serialization
     */
    final private static String tagJoints = "joints";
    // *************************************************************************
    // fields

    /**
     * list of joints that connect to this body: The list isn't populated until
     * the body is added to a PhysicsSpace.
     */
    private ArrayList<PhysicsJoint> joints = new ArrayList<>(4);
    // *************************************************************************
    // new methods exposed

    /**
     * Do not invoke directly! Joints are added automatically when created.
     *
     * @param joint the joint to add (not null, alias created)
     */
    public void addJoint(PhysicsJoint joint) {
        Validate.nonNull(joint, "joint");

        if (!joints.contains(joint)) {
            joints.add(joint);
        }
    }

    /**
     * Clone this body's joints.
     *
     * @param cloner the Cloner that's cloning this body (not null, modified)
     */
    protected void cloneJoints(Cloner cloner) {
        joints = cloner.clone(joints);
    }

    /**
     * Count how many joints connect to this body.
     *
     * @return the count (&ge;0) or 0 if the body isn't added to any
     * PhysicsSpace
     */
    public int countJoints() {
        int result = 0;
        if (isInWorld()) {
            result = joints.size();
        }

        return result;
    }

    /**
     * Copy this body's gravitational acceleration.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return an acceleration vector (in physics-space coordinates, either
     * storeResult or a new vector, not null)
     */
    abstract public Vector3f getGravity(Vector3f storeResult);

    /**
     * Determine the total mass of this body.
     *
     * @return the total mass (&ge;0)
     */
    abstract public float getMass();

    /**
     * Enumerate the joints connected to this body.
     *
     * @return a new array of pre-existing joints, or null if this body is not
     * added to any PhysicsSpace
     */
    public PhysicsJoint[] listJoints() {
        PhysicsJoint[] result;
        if (isInWorld()) {
            int numJoints = joints.size();
            result = new PhysicsJoint[numJoints];
            joints.toArray(result);
        } else {
            result = null;
        }

        return result;
    }

    /**
     * De-serialize joints from the specified InputCapsule, for example when
     * loading from a J3O file.
     *
     * @param capsule (not null, modified)
     * @throws IOException from the capsule
     */
    @SuppressWarnings("unchecked")
    protected void readJoints(InputCapsule capsule) throws IOException {
        joints = capsule.readSavableArrayList(tagJoints, null);
    }

    /**
     * Do not invoke directly! Joints are removed automatically when destroyed.
     *
     * @param joint the joint to remove (not null, unaffected)
     * @see com.jme3.bullet.joints.PhysicsJoint#destroy()
     */
    public void removeJoint(PhysicsJoint joint) {
        Validate.nonNull(joint, "joint");

        boolean success = joints.remove(joint);
        assert success;
    }

    /**
     * Alter this body's gravitational acceleration. TODO scalar alternative
     * <p>
     * Invoke this method <em>after</em> adding the body to a PhysicsSpace.
     * Adding a body to a PhysicsSpace overrides its gravity.
     *
     * @param acceleration the desired acceleration vector (in physics-space
     * coordinates, not null, unaffected)
     */
    abstract public void setGravity(Vector3f acceleration);

    /**
     * Alter this body's total mass.
     *
     * @param mass the desired total mass (&ge;0)
     */
    abstract public void setMass(float mass);

    /**
     * Directly relocate this body's center.
     *
     * @param location the desired location (in physics-space coordinates, not
     * null, unaffected)
     */
    abstract public void setPhysicsLocation(Vector3f location);

    /**
     * Serialize joints to the specified OutputCapsule, for example when saving
     * to a J3O file.
     *
     * @param capsule (not null, modified)
     * @throws IOException from the capsule
     */
    protected void writeJoints(OutputCapsule capsule) throws IOException {
        capsule.writeSavableArrayList(joints, tagJoints, null);
    }
}
