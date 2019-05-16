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
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;

/**
 * The abstract base class for rigid bodies and soft bodies.
 *
 * @author Stephen Gold sgold@sonic.net
 */
abstract public class PhysicsBody extends PhysicsCollisionObject {
    /**
     * Do not invoke directly! Joints are added automatically when created.
     *
     * @param joint the joint to add (not null)
     */
    abstract void addJoint(PhysicsJoint joint);

    /**
     * Count how many joints connect to this body.
     *
     * @return the count (&ge;0) or 0 if the body isn't added to any
     * PhysicsSpace
     */
    abstract int countJoints();

    /**
     * Copy this body's gravitational acceleration.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return an acceleration vector in physics-space coordinates (either
     * storeResult or a new vector, not null)
     */
    abstract Vector3f getGravity(Vector3f storeResult);

    /**
     * Determine the total mass of this body.
     *
     * @return the total mass
     */
    abstract float getMass();

    /**
     * Enumerate the joints connected to this body.
     *
     * @return a new array of pre-existing joints, or null if this body is not
     * added to any space
     */
    abstract PhysicsJoint[] listJoints();

    /**
     * Do not invoke directly! Joints are removed automatically when destroyed.
     *
     * @param joint the joint to remove (not null)
     */
    abstract void removeJoint(PhysicsJoint joint);

    /**
     * Alter this body's gravitational acceleration.
     * <p>
     * Invoke this method <em>after</em> adding the body to a PhysicsSpace.
     * Adding a body to a PhysicsSpace alters its gravity.
     *
     * @param acceleration the desired acceleration vector (in physics-space
     * coordinates, not null, unaffected)
     */
    abstract void setGravity(Vector3f acceleration);

    /**
     * Alter this body's total mass.
     *
     * @param mass the desired total mass (&ge;0)
     */
    abstract void setMass(float mass);

    /**
     * Directly relocate this body's center.
     *
     * @param location the desired location (in physics-space coordinates, not
     * null, unaffected)
     */
    abstract void setPhysicsLocation(Vector3f location);

    /**
     * Directly reorient this body.
     *
     * @param orientation the desired orientation (relative to physics-space
     * coordinates, unit quaternion, not null, unaffected)
     */
    abstract void setPhysicsRotation(Quaternion orientation);

    /**
     * Directly alter this body's transform, including its scale factors.
     *
     * @param transform the desired transform (relative to physics-space
     * coordinates, not null, unaffected)
     */
    abstract void setPhysicsTransform(Transform transform);
}
