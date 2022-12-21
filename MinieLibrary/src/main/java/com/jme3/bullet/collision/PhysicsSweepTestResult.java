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
package com.jme3.bullet.collision;

import com.jme3.math.Vector3f;
import java.util.logging.Logger;

/**
 * Represent the results of a Bullet sweep test.
 *
 * @author normenhansen
 */
final public class PhysicsSweepTestResult {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(PhysicsSweepTestResult.class.getName());
    // *************************************************************************
    // fields

    /**
     * fraction of the way between the transforms (from=0, to=1, &ge;0, &le;1)
     */
    private float hitFraction;
    /**
     * index of the collision-shape part that was hit, or -1 if undefined
     */
    private int partIndex;
    /**
     * index of the collision-shape triangle that was hit, or -1 if undefined
     */
    private int triangleIndex;
    /**
     * collision object that was hit
     */
    private PhysicsCollisionObject collisionObject;
    /**
     * normal vector (in physics-space coordinates) at the point of contact
     */
    private Vector3f normal;
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class by Java.
     * These results are instantiated exclusively by native code.
     */
    private PhysicsSweepTestResult() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Access the collision object that was hit.
     *
     * @return the pre-existing instance (not null)
     */
    public PhysicsCollisionObject getCollisionObject() {
        assert collisionObject != null;
        return collisionObject;
    }

    /**
     * Read the fraction of the way between the transforms (from=0, to=1, &ge;0,
     * &le;1)
     *
     * @return fraction (from=0, to=1, &ge;0, &le;1)
     */
    public float getHitFraction() {
        assert hitFraction >= 0f : hitFraction;
        assert hitFraction <= 1f : hitFraction;
        return hitFraction;
    }

    /**
     * Copy the normal vector at the point of contact.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a unit vector (in physics-space coordinates, either storeResult
     * or a new vector, not null)
     */
    public Vector3f getHitNormalLocal(Vector3f storeResult) {
        Vector3f result;
        if (storeResult == null) {
            result = normal.clone();
        } else {
            result = storeResult.set(normal);
        }
        return result;
    }

    /**
     * Read the part index at the point of contact.
     * <p>
     * If shape that was hit is compound or convex, the index is undefined.
     * <p>
     * If the shape is a GImpactCollisionShape or MeshCollisionShape, the index
     * identifies an IndexedMesh.
     * <p>
     * If the shape is a HeightfieldCollisionShape, the index identifies a grid
     * row.
     *
     * @return the index of the collision-shape part (&ge;0) or -1 if undefined
     */
    public int partIndex() {
        return partIndex;
    }

    /**
     * Read the triangle index at the point of contact.
     * <p>
     * If shape that was hit is convex, the index is undefined.
     * <p>
     * If shape is a CompoundCollisionShape, the index identifies a child shape.
     * <p>
     * If the shape is a GImpactCollisionShape or MeshCollisionShape, the index
     * identifies a triangle in an IndexedMesh.
     * <p>
     * If the shape is a HeightfieldCollisionShape, the index indicates a grid
     * column.
     *
     * @return the index of the collision-shape triangle (&ge;0) or -1 if
     * undefined
     */
    public int triangleIndex() {
        return triangleIndex;
    }
}
