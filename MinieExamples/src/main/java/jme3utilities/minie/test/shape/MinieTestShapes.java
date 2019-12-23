/*
 Copyright (c) 2019, Stephen Gold
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package jme3utilities.minie.test.shape;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.ConeCollisionShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.MultiSphere;
import com.jme3.bullet.collision.shapes.SimplexCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.math.FastMath;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.Map;
import java.util.logging.Logger;
import jme3utilities.math.RectangularSolid;
import jme3utilities.minie.MyShape;

/**
 * Generate some interesting compound collision shapes for use in MinieExamples.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class MinieTestShapes {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(MinieTestShapes.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private MinieTestShapes() {
    }
    /**
     * inverse moment-of-inertia vector for "chair" shape (initialized by
     * makeChair()
     */
    public static Vector3f chairInverseInertia = null;
    // *************************************************************************
    // new methods exposed

    /**
     * Add each test shape to the specified collection of named shapes.
     *
     * @param namedShapes where to add shapes (not null, modified)
     */
    public static void addShapes(Map<String, CollisionShape> namedShapes) {
        CollisionShape barbell = makeBarbell();
        namedShapes.put("barbell", barbell);

        CollisionShape chair = makeChair();
        namedShapes.put("chair", chair);

        CollisionShape knucklebone = makeKnucklebone();
        namedShapes.put("knucklebone", knucklebone);

        CollisionShape ladder = makeLadder();
        namedShapes.put("ladder", ladder);

        CollisionShape top = makeTop();
        namedShapes.put("top", top);

        CollisionShape tray = makeTray();
        namedShapes.put("tray", tray);
    }

    /**
     * Generate a barbell shape with 2 cylindrical plates.
     *
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeBarbell() {
        float barRadius = 0.2f;
        float plateOffset = 2f;
        Vector3f halfExtents
                = new Vector3f(1.4f * plateOffset, barRadius, barRadius);
        CollisionShape child
                = new CylinderCollisionShape(halfExtents, PhysicsSpace.AXIS_X);

        CompoundCollisionShape result = new CompoundCollisionShape();
        result.addChildShape(child);

        float plateRadius = 1f;
        halfExtents.set(barRadius, plateRadius, plateRadius);
        child = new CylinderCollisionShape(halfExtents, PhysicsSpace.AXIS_X);
        result.addChildShape(child, -plateOffset, 0f, 0f);
        result.addChildShape(child, plateOffset, 0f, 0f);

        return result;
    }

    /**
     * Generate a chair with 4 cylindrical legs (asymmetrical). Must override
     * the moments of inertia if used in a dynamic body.
     *
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeChair() {
        float legOffset = 1f;
        float legRadius = 0.2f;
        float seatHalf = legOffset + legRadius;
        Vector3f halfExtents = new Vector3f(seatHalf, 0.2f, seatHalf);
        RectangularSolid solid = new RectangularSolid(halfExtents);
        CollisionShape child = new MultiSphere(solid);

        CompoundCollisionShape result = new CompoundCollisionShape();
        result.addChildShape(child);

        float frontLength = 2f;
        float frontHalf = frontLength / 2f;
        halfExtents.set(legRadius, frontHalf, legRadius);
        child = new CylinderCollisionShape(halfExtents, PhysicsSpace.AXIS_Y);
        result.addChildShape(child, legOffset, -frontHalf, legOffset);
        child = new CylinderCollisionShape(halfExtents, PhysicsSpace.AXIS_Y);
        result.addChildShape(child, -legOffset, -frontHalf, legOffset);

        float rearHalf = 2.5f;
        halfExtents.set(legRadius, rearHalf, legRadius);
        child = new CylinderCollisionShape(halfExtents, PhysicsSpace.AXIS_Y);
        float yOffset = rearHalf - frontLength;
        result.addChildShape(child, legOffset, yOffset, -legOffset);
        child = new CylinderCollisionShape(halfExtents, PhysicsSpace.AXIS_Y);
        result.addChildShape(child, -legOffset, yOffset, -legOffset);

        float backHalf = rearHalf - frontHalf;
        halfExtents.set(legOffset, backHalf, legRadius);
        solid = new RectangularSolid(halfExtents);
        child = new MultiSphere(solid);
        result.addChildShape(child, 0f, backHalf, -legOffset);

        float[] volumes = MyShape.listVolumes(result);
        float sum = 0f;
        for (float volume : volumes) {
            sum += volume;
        }
        FloatBuffer masses = BufferUtils.createFloatBuffer(volumes.length);
        for (int i = 0; i < volumes.length; ++i) {
            masses.put(volumes[i] / sum);
        }
        Vector3f inertia = new Vector3f();
        Transform transform = result.principalAxes(masses, null, inertia);
        chairInverseInertia = Vector3f.UNIT_XYZ.divide(inertia);
        result.correctAxes(transform);

        return result;
    }

    /**
     * Generate a knuclebone with 4 spherical balls.
     *
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeKnucklebone() {
        float stemLength = 2.5f;
        float stemRadius = 0.25f;
        CollisionShape child = new CapsuleCollisionShape(stemRadius, stemLength,
                PhysicsSpace.AXIS_X);

        CompoundCollisionShape result = new CompoundCollisionShape();
        result.addChildShape(child);

        child = new CapsuleCollisionShape(stemRadius, stemLength,
                PhysicsSpace.AXIS_Y);
        result.addChildShape(child);

        child = new CapsuleCollisionShape(stemRadius, stemLength,
                PhysicsSpace.AXIS_Z);
        result.addChildShape(child);

        float ballRadius = 0.4f;
        child = new SphereCollisionShape(ballRadius);
        float stemHalf = stemLength / 2f;
        result.addChildShape(child, stemHalf, 0f, 0f);
        result.addChildShape(child, -stemHalf, 0f, 0f);
        result.addChildShape(child, 0f, stemHalf, 0f);
        result.addChildShape(child, 0f, -stemHalf, 0f);

        return result;
    }

    /**
     * Generate a ladder with 5 cylindrical rungs.
     *
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeLadder() {
        float rungRadius = 0.2f;
        float rungSpacing = 2f;
        float rungHalf = 1f;
        Vector3f halfExtents = new Vector3f(rungHalf, rungRadius, rungRadius);
        CollisionShape child
                = new CylinderCollisionShape(halfExtents, PhysicsSpace.AXIS_X);

        CompoundCollisionShape result = new CompoundCollisionShape();
        result.addChildShape(child, 0f, 2f * rungSpacing, 0f);
        result.addChildShape(child, 0f, rungSpacing, 0f);
        result.addChildShape(child);
        result.addChildShape(child, 0f, -rungSpacing, 0f);
        result.addChildShape(child, 0f, -2f * rungSpacing, 0f);

        float railHalf = 6f;
        halfExtents.set(rungRadius, railHalf, rungRadius);
        child = new BoxCollisionShape(halfExtents);
        result.addChildShape(child, rungHalf, 0f, 0f);
        result.addChildShape(child, -rungHalf, 0f, 0f);

        return result;
    }

    /**
     * Generate a top with a cylindrical body.
     *
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeTop() {
        float bodyRadius = 1.5f;
        float bodyHeight = 0.3f;
        Vector3f halfExtents = new Vector3f(bodyRadius, bodyHeight, bodyRadius);
        CollisionShape child
                = new CylinderCollisionShape(halfExtents, PhysicsSpace.AXIS_Y);

        CompoundCollisionShape result = new CompoundCollisionShape();
        result.addChildShape(child);

        float coneHeight = 1.5f;
        child = new ConeCollisionShape(bodyRadius - 0.06f, coneHeight,
                PhysicsSpace.AXIS_Y);
        result.addChildShape(child, 0f, coneHeight / 2f + bodyHeight, 0f);

        float handleHeight = 1.5f;
        float handleRadius = 0.3f;
        child = new CapsuleCollisionShape(handleRadius, handleHeight,
                PhysicsSpace.AXIS_Y);
        float yOffset = -0.48f * (bodyHeight + handleHeight);
        result.addChildShape(child, 0f, yOffset, 0f);

        return result;
    }

    /**
     * Generate a square tray with a central deflector (asymmetrical). Not
     * intended for use in a dynamic body.
     *
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeTray() {
        /*
         * Start with a box for the base.
         */
        float height = 1.5f;
        float length = 15f;
        CollisionShape child = new BoxCollisionShape(length, height, length);

        CompoundCollisionShape result = new CompoundCollisionShape();
        result.addChildShape(child, 0f, -1.95f * height, 0f);
        /*
         * Place a tetrahedral deflector in the center.
         */
        float size = 3f;
        Vector3f p1 = new Vector3f(0f, size, 0f);
        Vector3f p2 = new Vector3f(-size, -height, size);
        Vector3f p3 = new Vector3f(-size, -height, -size);
        Vector3f p4 = new Vector3f(size * FastMath.sqrt(2f), -height, 0f);
        child = new SimplexCollisionShape(p1, p2, p3, p4);
        result.addChildShape(child);
        /*
         * Arrange 4 bumpers in a square around the deflector.
         */
        float offset = length - height;
        child = new BoxCollisionShape(length, height, height);
        result.addChildShape(child, 0f, 0f, offset);
        result.addChildShape(child, 0f, 0f, -offset);

        child = new BoxCollisionShape(height, height, length);
        result.addChildShape(child, offset, 0f, 0f);
        result.addChildShape(child, -offset, 0f, 0f);

        return result;
    }
}
