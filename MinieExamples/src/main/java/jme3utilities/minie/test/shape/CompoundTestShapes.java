/*
 Copyright (c) 2019-2023, Stephen Gold
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
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.MultiSphere;
import com.jme3.bullet.collision.shapes.PlaneCollisionShape;
import com.jme3.bullet.collision.shapes.SimplexCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.math.FastMath;
import com.jme3.math.Matrix3f;
import com.jme3.math.Plane;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;
import jme3utilities.math.RectangularSolid;
import jme3utilities.mesh.Octasphere;
import jme3utilities.minie.MyShape;
import jme3utilities.minie.test.mesh.StarSlice;

/**
 * Utility class to generate compound collision shapes for use in MinieExamples.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class CompoundTestShapes {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(CompoundTestShapes.class.getName());
    // *************************************************************************
    // fields

    /**
     * inverse moment-of-inertia vector for a "bowl" shape with mass=1, scale=1
     * (initialized by makeBowl()
     */
    public static Vector3f bowlInverseInertia = null;
    /**
     * inverse moment-of-inertia vector for a "chair" shape with mass=1, scale=1
     * (initialized by makeChair()
     */
    public static Vector3f chairInverseInertia = null;
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private CompoundTestShapes() {
    }
    // *************************************************************************
    // new methods exposed - TODO more validation of method arguments

    /**
     * Generate a barbell shape with 2 cylindrical plates.
     *
     * @param barLength the total length of the bar (X axis, in unscaled shape
     * units, &gt;0)
     * @param barRadius the radius of the bar (in unscaled shape units, &gt;0)
     * @param plateRadius the radius of each plate (in unscaled shape units,
     * &gt;0)
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeBarbell(
            float barLength, float barRadius, float plateRadius) {
        float plateOffset = 0.42f * barLength;
        CollisionShape bar = new CylinderCollisionShape(
                barRadius, barLength, PhysicsSpace.AXIS_X);

        float plateThickness = 0.08f * barLength;
        CollisionShape plate = new CylinderCollisionShape(
                plateRadius, plateThickness, PhysicsSpace.AXIS_X);

        CompoundCollisionShape result = new CompoundCollisionShape(3);
        result.addChildShape(bar);
        result.addChildShape(plate, -plateOffset, 0f, 0f);
        result.addChildShape(plate, plateOffset, 0f, 0f);

        return result;
    }

    /**
     * Approximate a hemispherical shell (or bowl), open on the +Z side, using
     * 3-sphere shapes.
     *
     * @param innerRadius (in unscaled shape units, &gt;thickness)
     * @param thickness (in unscaled shape units, &gt;0, &lt;innerRadius)
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeBowl(
            float innerRadius, float thickness) {
        Validate.inRange(
                innerRadius, "inner radius", thickness, Float.MAX_VALUE);
        Validate.inRange(thickness, "thickness", 0f, innerRadius);

        float halfThickness = thickness / 2f;
        float midRadius = innerRadius + halfThickness;
        int numRefineSteps = 2;
        Octasphere mesh = new Octasphere(numRefineSteps, midRadius);
        int numTriangles = mesh.getTriangleCount();

        List<Float> radii = new ArrayList<>(3);
        radii.add(thickness);
        radii.add(thickness);
        radii.add(thickness);
        List<Vector3f> centers = new ArrayList<>(3);
        Vector3f v1 = new Vector3f();
        Vector3f v2 = new Vector3f();
        Vector3f v3 = new Vector3f();
        centers.add(v1);
        centers.add(v2);
        centers.add(v3);
        Vector3f centroid = new Vector3f();
        float maxZ = 1e-4f;
        CompoundCollisionShape result
                = new CompoundCollisionShape(numTriangles / 2);

        for (int triangleI = 0; triangleI < numTriangles; ++triangleI) {
            mesh.getTriangle(triangleI, v1, v2, v3);
            if (v1.z < maxZ && v2.z < maxZ && v3.z < maxZ) {
                centroid.zero();
                centroid.addLocal(v1).addLocal(v2).addLocal(v3);
                centroid.divideLocal(3f);

                v1.subtractLocal(centroid);
                v2.subtractLocal(centroid);
                v3.subtractLocal(centroid);

                MultiSphere triSphere = new MultiSphere(centers, radii);
                result.addChildShape(triSphere, centroid);
            }
        }

        int numChildren = result.countChildren();
        FloatBuffer masses = BufferUtils.createFloatBuffer(numChildren);
        for (int childIndex = 0; childIndex < numChildren; ++childIndex) {
            masses.put(1f / numChildren);
        }
        Vector3f inertia = new Vector3f();
        Transform transform
                = result.principalAxes(masses, null, inertia);
        bowlInverseInertia = Vector3f.UNIT_XYZ.divide(inertia);
        result.correctAxes(transform);

        return result;
    }

    /**
     * Generate a chair with 4 cylindrical legs (asymmetrical). Must override
     * the moments of inertia if used in a dynamic body.
     *
     * @param backLength the length of the back (Y axis, in unscaled shape
     * units, &gt;0)
     * @param legLength the length of each leg (Y axis, in unscaled shape units,
     * &gt;0)
     * @param legOffset the offset of each leg from the center of the seat (in
     * unscaled shape units, &gt;legRadius)
     * @param legRadius the radius of each leg (in unscaled shape units, &gt;0)
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeChair(float backLength,
            float legLength, float legOffset, float legRadius) {
        float seatHalf = legOffset + legRadius;
        Vector3f halfExtents = new Vector3f(seatHalf, 0.2f, seatHalf);
        RectangularSolid solid = new RectangularSolid(halfExtents);
        CollisionShape seat = new MultiSphere(solid);

        CollisionShape frontLeg = new CylinderCollisionShape(
                legRadius, legLength, PhysicsSpace.AXIS_Y);

        float rearLength = legLength + backLength;
        CollisionShape rearLeg = new CylinderCollisionShape(
                legRadius, rearLength, PhysicsSpace.AXIS_Y);

        float rearHalf = rearLength / 2f;
        float legHalf = legLength / 2f;
        float backHalf = backLength / 2f;
        halfExtents.set(legOffset, backHalf, legRadius);
        solid = new RectangularSolid(halfExtents);
        CollisionShape back = new MultiSphere(solid);

        CompoundCollisionShape result = new CompoundCollisionShape(6);
        result.addChildShape(seat);
        result.addChildShape(frontLeg, legOffset, -legHalf, legOffset);
        result.addChildShape(frontLeg, -legOffset, -legHalf, legOffset);
        float yOffset = rearHalf - legLength;
        result.addChildShape(rearLeg, legOffset, yOffset, -legOffset);
        result.addChildShape(rearLeg, -legOffset, yOffset, -legOffset);
        result.addChildShape(back, 0f, backHalf, -legOffset);

        float[] volumes = MyShape.listVolumes(result);
        float sum = 0f;
        for (float volume : volumes) {
            sum += volume;
        }
        FloatBuffer masses = BufferUtils.createFloatBuffer(volumes.length);
        for (float volume : volumes) {
            masses.put(volume / sum);
        }
        Vector3f inertia = new Vector3f();
        Transform transform = result.principalAxes(masses, null, inertia);
        chairInverseInertia = Vector3f.UNIT_XYZ.divide(inertia);
        result.correctAxes(transform);

        return result;
    }

    /**
     * Generate an inverted hollow pyramid with its nadir on the -Y axis. Not
     * intended for use in a dynamic body. TODO use MeshCollisionShape
     *
     * @param numSides the number of sides (&ge;3)
     * @param rimRadius (in unscaled shape units, &gt;0)
     * @param depth rimY minus nadirY (in unscaled shape units, &gt;0)
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeCorner(
            int numSides, float rimRadius, float depth) {
        Validate.inRange(numSides, "number of sides", 3, Integer.MAX_VALUE);
        Validate.positive(rimRadius, "rim radius");
        Validate.positive(depth, "depth");

        float stepAngle = FastMath.TWO_PI / numSides; // in radians
        Matrix3f rotationMatrix = new Matrix3f();
        rotationMatrix.fromAngleAxis(stepAngle, Vector3f.UNIT_Y);

        float rimY = 0.3f * depth;
        Vector3f vertex1 = new Vector3f(rimRadius, rimY, 0f);
        Vector3f vertex2 = rotationMatrix.mult(vertex1, null);
        Vector3f nadir = new Vector3f(0f, rimY - depth, 0f);
        SimplexCollisionShape triangle
                = new SimplexCollisionShape(vertex1, vertex2, nadir);

        CompoundCollisionShape result = new CompoundCollisionShape(numSides);
        result.addChildShape(triangle);
        result.addChildShape(triangle, Vector3f.ZERO, rotationMatrix);
        for (int i = 2; i < numSides; ++i) {
            float angle = stepAngle * i;
            rotationMatrix.fromAngleAxis(angle, Vector3f.UNIT_Y);
            result.addChildShape(triangle, Vector3f.ZERO, rotationMatrix);
        }

        return result;
    }

    /**
     * Generate a rectangular frame, open on the Z axis.
     *
     * @param ihHeight half of the internal height (Y direction, in unscaled
     * shape units, &gt;0)
     * @param ihWidth half of the internal width (X direction, in unscaled shape
     * units, &gt;0)
     * @param halfDepth half of the (external) depth (Z direction, in unscaled
     * shape units, &gt;0)
     * @param halfThickness half the thickness (in unscaled shape units, &gt;0)
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeFrame(float ihHeight,
            float ihWidth, float halfDepth, float halfThickness) {
        Validate.positive(ihHeight, "half height");
        Validate.positive(ihWidth, "half width");
        Validate.positive(halfDepth, "half depth");
        Validate.positive(halfThickness, "half thickness");

        float mhHeight = ihHeight + halfThickness;
        float mhWidth = ihWidth + halfThickness;

        CollisionShape horizontal
                = new BoxCollisionShape(mhWidth, halfThickness, halfDepth);
        CollisionShape vertical
                = new BoxCollisionShape(halfThickness, mhHeight, halfDepth);

        CompoundCollisionShape result = new CompoundCollisionShape(4);
        result.addChildShape(horizontal, halfThickness, -mhHeight, 0f);
        result.addChildShape(horizontal, -halfThickness, mhHeight, 0f);
        result.addChildShape(vertical, mhWidth, halfThickness, 0f);
        result.addChildShape(vertical, -mhWidth, -halfThickness, 0f);

        return result;
    }

    /**
     * Generate an I-beam.
     *
     * @param length (Z axis, in unscaled shape units, &gt;0)
     * @param flangeWidth (X axis, in unscaled shape units, &ge;thickness)
     * @param beamHeight (Y axis, in unscaled shape units, &ge;2*thickness)
     * @param thickness (in unscaled shape units, &gt;0)
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeIBeam(float length,
            float flangeWidth, float beamHeight, float thickness) {
        Validate.positive(length, "length");
        Validate.positive(thickness, "thickness");
        Validate.inRange(
                flangeWidth, "flange width", thickness, Float.MAX_VALUE);
        Validate.inRange(
                beamHeight, "beam height", 2f * thickness, Float.MAX_VALUE);

        float halfLength = length / 2f;
        float halfThickness = thickness / 2f;
        float webHalfHeight = beamHeight / 2f - thickness;
        CollisionShape web = new BoxCollisionShape(
                halfThickness, webHalfHeight, halfLength);
        CollisionShape flange = new BoxCollisionShape(
                flangeWidth / 2f, halfThickness, halfLength);

        CompoundCollisionShape result = new CompoundCollisionShape(3);
        result.addChildShape(web);
        float flangeY = webHalfHeight + halfThickness;
        result.addChildShape(flange, 0f, flangeY, 0f);
        result.addChildShape(flange, 0f, -flangeY, 0f);

        return result;
    }

    /**
     * Generate a knucklebone with 4 spherical balls. (No balls on the Z axis.)
     *
     * @param stemLength the length of each stem (in unscaled shape units,
     * &gt;0)
     * @param stemRadius the radius of each stem (in unscaled shape units,
     * &gt;0)
     * @param ballRadius the radius of each ball (in unscaled shape units,
     * &gt;stemRadius)
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeKnucklebone(
            float stemLength, float stemRadius, float ballRadius) {
        CollisionShape xStem = new CapsuleCollisionShape(
                stemRadius, stemLength, PhysicsSpace.AXIS_X);
        CollisionShape yStem = new CapsuleCollisionShape(
                stemRadius, stemLength, PhysicsSpace.AXIS_Y);
        CollisionShape zStem = new CapsuleCollisionShape(
                stemRadius, stemLength, PhysicsSpace.AXIS_Z);

        CollisionShape ball = new SphereCollisionShape(ballRadius);

        CompoundCollisionShape result = new CompoundCollisionShape(7);
        result.addChildShape(xStem);
        result.addChildShape(yStem);
        result.addChildShape(zStem);

        float stemHalf = stemLength / 2f;
        result.addChildShape(ball, stemHalf, 0f, 0f);
        result.addChildShape(ball, -stemHalf, 0f, 0f);
        result.addChildShape(ball, 0f, stemHalf, 0f);
        result.addChildShape(ball, 0f, -stemHalf, 0f);

        return result;
    }

    /**
     * Generate a ladder with 5 cylindrical rungs.
     *
     * @param rungLength the total length of each rung (X direction, in unscaled
     * shape units, &gt;0)
     * @param rungSpacing the spacing between rungs (Y direction, in unscaled
     * shape units, &gt;2*rungRadius)
     * @param rungRadius the radius of each rung (in unscaled shape units,
     * &gt;0)
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeLadder(
            float rungLength, float rungSpacing, float rungRadius) {
        CollisionShape rung = new CylinderCollisionShape(
                rungRadius, rungLength, PhysicsSpace.AXIS_X);

        float railLength = 6f * rungSpacing;
        float railHalf = railLength / 2f;
        CollisionShape rail
                = new BoxCollisionShape(rungRadius, railHalf, rungRadius);

        CompoundCollisionShape result = new CompoundCollisionShape(7);
        result.addChildShape(rung, 0f, 2f * rungSpacing, 0f);
        result.addChildShape(rung, 0f, rungSpacing, 0f);
        result.addChildShape(rung);
        result.addChildShape(rung, 0f, -rungSpacing, 0f);
        result.addChildShape(rung, 0f, -2f * rungSpacing, 0f);

        float rungHalf = rungLength / 2f;
        result.addChildShape(rail, rungHalf, 0f, 0f);
        result.addChildShape(rail, -rungHalf, 0f, 0f);

        return result;
    }

    /**
     * Generate a lidless rectangular box with its opening on the +Z side.
     *
     * @param iHeight the internal height (Y direction, in unscaled shape units,
     * &gt;0)
     * @param iWidth the internal width (X direction, in unscaled shape units,
     * &gt;0)
     * @param iDepth the internal depth (Z direction, in unscaled shape units,
     * &gt;0)
     * @param wallThickness (in unscaled shape units, &gt;0)
     *
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeLidlessBox(
            float iHeight, float iWidth, float iDepth, float wallThickness) {
        float ihHeight = iHeight / 2f;
        float ihWidth = iWidth / 2f;
        float ihDepth = iDepth / 2f;
        float halfThickness = wallThickness / 2f;

        float fhDepth = ihDepth + halfThickness;
        CompoundCollisionShape result
                = makeFrame(ihHeight, ihWidth, fhDepth, halfThickness);

        BoxCollisionShape bottom
                = new BoxCollisionShape(ihWidth, ihHeight, halfThickness);
        result.addChildShape(bottom, 0f, 0f, -ihDepth);

        return result;
    }

    /**
     * Generate a rectangular link for a chain.
     *
     * @param ihHeight half of the internal height (Y direction, in unscaled
     * shape units, &gt;0)
     * @param ihWidth half of the internal width (X direction, in unscaled shape
     * units, &gt;0)
     * @param radius half the thickness (in unscaled shape units, &gt;0)
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeLink(
            float ihHeight, float ihWidth, float radius) {
        Validate.positive(ihHeight, "half height");
        Validate.positive(ihWidth, "half width");
        Validate.positive(radius, "radius");

        float mhHeight = ihHeight + radius;
        float mhWidth = ihWidth + radius;

        CollisionShape horizontal = new CapsuleCollisionShape(
                radius, 2f * mhWidth, PhysicsSpace.AXIS_X);
        CollisionShape vertical = new CapsuleCollisionShape(
                radius, 2f * mhHeight, PhysicsSpace.AXIS_Y);

        CompoundCollisionShape result = new CompoundCollisionShape(4);
        result.addChildShape(horizontal, 0f, -mhHeight, 0f);
        result.addChildShape(horizontal, 0f, mhHeight, 0f);
        result.addChildShape(vertical, mhWidth, 0f, 0f);
        result.addChildShape(vertical, -mhWidth, 0f, 0f);

        return result;
    }

    /**
     * Generate a mad mallet by compounding 2 cylinders.
     *
     * @param handleR the radius of the handle (in unscaled shape units, &gt;0)
     * @param headR the radius of the head (in unscaled shape units, &gt;0)
     * @param handleHalfLength half the length of the handle (in unscaled shape
     * units, &gt;0)
     * @param headHalfLength half the length of the head (in unscaled shape
     * units, &gt;0)
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeMadMallet(float handleR,
            float headR, float handleHalfLength, float headHalfLength) {
        Validate.positive(handleR, "handle radius");
        Validate.positive(headR, "head radius");
        Validate.positive(handleHalfLength, "handle half length");
        Validate.positive(headHalfLength, "head half length");

        Vector3f hes = new Vector3f(headR, headR, headHalfLength);
        CollisionShape head = new CylinderCollisionShape(hes);

        hes.set(handleR, handleR, handleHalfLength);
        CollisionShape handle = new CylinderCollisionShape(hes);

        CompoundCollisionShape result = new CompoundCollisionShape(2);

        result.addChildShape(handle, 0f, 0f, handleHalfLength);

        Vector3f offset = new Vector3f(0f, 0f, 2f * handleHalfLength);
        Matrix3f rotation = new Matrix3f();
        rotation.fromAngleAxis(FastMath.HALF_PI, Vector3f.UNIT_X);
        result.addChildShape(head, offset, rotation);

        return result;
    }

    /**
     * Approximate an arc of a straight, square-ended pipe (or of a flat ring),
     * open on the Z axis, using hulls.
     *
     * @param innerR the inner radius of an X-Y cross section (in unscaled shape
     * units, &gt;0)
     * @param thickness the thickness of the pipe (in unscaled shape units,
     * &gt;0)
     * @param zLength the length of the pipe (Z direction, in unscaled shape
     * units, &gt;0)
     * @param arc the arc amount (in radians, &gt;0, &le;2pi)
     * @param numChildren the number of child shapes to create (&ge;3)
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makePipe(float innerR, float thickness,
            float zLength, float arc, int numChildren) {
        Validate.positive(innerR, "inner radius");
        Validate.positive(thickness, "thickness");
        Validate.positive(zLength, "length");
        Validate.inRange(arc, "arc", 0f, FastMath.TWO_PI);
        Validate.inRange(
                numChildren, "number of children", 2, Integer.MAX_VALUE);

        float halfLength = zLength / 2f;
        float outerR = innerR + thickness;
        float segmentAngle = arc / numChildren; // in radians

        float xOff;
        float yOff; // TODO more accurate centering
        if (arc < 2) {
            float cos = FastMath.cos(segmentAngle);
            float sin = FastMath.sin(segmentAngle);
            xOff = (1 + cos * outerR) / 2f;
            yOff = sin * innerR / 2f;

        } else if (arc < 4) {
            xOff = 0f;
            yOff = outerR / 2f;

        } else {
            xOff = 0f;
            yOff = 0f;
        }

        CompoundCollisionShape result = new CompoundCollisionShape(numChildren);
        for (int segmentI = 0; segmentI < numChildren; ++segmentI) {
            float theta1 = segmentI * segmentAngle;
            float theta2 = (segmentI + 1) * segmentAngle;
            float cos1 = FastMath.cos(theta1);
            float cos2 = FastMath.cos(theta2);
            float sin1 = FastMath.sin(theta1);
            float sin2 = FastMath.sin(theta2);

            FloatBuffer buffer = BufferUtils.createFloatBuffer(
                    innerR * cos1 - xOff, innerR * sin1 - yOff, halfLength,
                    innerR * cos2 - xOff, innerR * sin2 - yOff, halfLength,
                    outerR * cos1 - xOff, outerR * sin1 - yOff, halfLength,
                    outerR * cos2 - xOff, outerR * sin2 - yOff, halfLength,
                    innerR * cos1 - xOff, innerR * sin1 - yOff, -halfLength,
                    innerR * cos2 - xOff, innerR * sin2 - yOff, -halfLength,
                    outerR * cos1 - xOff, outerR * sin1 - yOff, -halfLength,
                    outerR * cos2 - xOff, outerR * sin2 - yOff, -halfLength
            );
            HullCollisionShape child = new HullCollisionShape(buffer);
            result.addChildShape(child);
        }

        return result;
    }

    /**
     * Generate a wire sieve, backed by a horizontal plane. Not intended for use
     * in a dynamic body.
     *
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeSieve() {
        float wireSpacing = 2f;
        float yHeight = 7f;
        int numXWires = 16;
        int numZWires = 16;

        float xHalfExtent = (numZWires - 1) * wireSpacing / 2f;
        float zHalfExtent = (numXWires - 1) * wireSpacing / 2f;
        CompoundCollisionShape result
                = new CompoundCollisionShape(numXWires + numZWires + 1);

        // Add numXWires wires parallel to the X axis.
        Vector3f p0 = new Vector3f(-xHalfExtent, 0f, 0f);
        Vector3f p1 = new Vector3f(xHalfExtent, 0f, 0f);
        SimplexCollisionShape xWire = new SimplexCollisionShape(p0, p1);

        for (int zIndex = 0; zIndex < numXWires; ++zIndex) {
            float z = -zHalfExtent + wireSpacing * zIndex;
            result.addChildShape(xWire, 0f, yHeight, z);
        }

        // Add numZWires wires parallel to the Z axis.
        p0.set(0f, 0f, -zHalfExtent);
        p1.set(0f, 0f, zHalfExtent);
        SimplexCollisionShape zWire = new SimplexCollisionShape(p0, p1);

        for (int xIndex = 0; xIndex < numZWires; ++xIndex) {
            float x = -xHalfExtent + wireSpacing * xIndex;
            result.addChildShape(zWire, x, yHeight, 0f);
        }

        // Add a plane to catch any drops that pass through the sieve.
        Plane plane = new Plane(Vector3f.UNIT_Y, 0f);
        PlaneCollisionShape pcs = new PlaneCollisionShape(plane);
        result.addChildShape(pcs);

        return result;
    }

    /**
     * Generate a 3-ball snowman with its head on the +Y axis.
     *
     * @param baseRadius (in unscaled shape units, &gt;0)
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeSnowman(float baseRadius) {
        Validate.positive(baseRadius, "base radius");

        float verticalAngle = 2f; // radians
        HullCollisionShape base
                = MinieTestShapes.makeDome(baseRadius, verticalAngle);

        float torsoRadius = 0.8f * baseRadius;
        SphereCollisionShape torso = new SphereCollisionShape(torsoRadius);

        float headRadius = 0.6f * baseRadius;
        SphereCollisionShape head = new SphereCollisionShape(headRadius);

        CompoundCollisionShape result = new CompoundCollisionShape(3);
        result.addChildShape(base, 0f, -0.5f * baseRadius, 0f);
        result.addChildShape(torso, 0f, 0.7f * torsoRadius, 0f);
        result.addChildShape(head, 0f, 2.1f * torsoRadius, 0f);

        return result;
    }

    /**
     * Generate a star shape.
     *
     * @param numPoints the number of points (&ge;2)
     * @param outerRadius the outer radius (in unscaled shape units, &gt;0)
     * @param centerY the half thickness at the center (&gt;0)
     * @param radiusRatio the inner radius divided by the outer radius (&gt;0,
     * &lt;1)
     * @param trianglesPerSlice the number of mesh triangles per slice (4 or 6)
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeStar(
            int numPoints, float outerRadius, float centerY, float radiusRatio,
            int trianglesPerSlice) {
        Validate.inRange(numPoints, "number of points", 2, Integer.MAX_VALUE);
        Validate.positive(outerRadius, "outer radius");
        Validate.positive(centerY, "center Y");
        Validate.fraction(radiusRatio, "radius ratio");
        Validate.inRange(trianglesPerSlice, "triangles per slice", 4, 6);

        float innerRadius = radiusRatio * outerRadius;
        float sliceAngle = FastMath.TWO_PI / numPoints; // in radians
        boolean normals = false;
        StarSlice sliceMesh = new StarSlice(sliceAngle, innerRadius,
                outerRadius, 2f * centerY, normals, trianglesPerSlice);
        CollisionShape sliceShape = new HullCollisionShape(sliceMesh);

        CompoundCollisionShape result = new CompoundCollisionShape(numPoints);
        Matrix3f rotate = new Matrix3f();
        for (int pointIndex = 0; pointIndex < numPoints; ++pointIndex) {
            rotate.fromAngleAxis(sliceAngle * pointIndex, Vector3f.UNIT_Y);
            result.addChildShape(sliceShape, Vector3f.ZERO, rotate);
        }

        return result;
    }

    /**
     * Generate a round pedestal table with its top on the +Y axis.
     *
     * @param topRadius the radius of the top (in unscaled shape units,
     * &gt;pedestalRadius)
     * @param pedestalRadius the radius of the pedestal (in unscaled shape
     * units, &gt;0)
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeTable(
            float topRadius, float pedestalRadius) {
        float thickness = 0.4f;
        float footHalfLength = 0.6f * (topRadius + pedestalRadius);
        CollisionShape foot
                = new BoxCollisionShape(thickness / 2f, 0.25f, footHalfLength);

        float pedestalHeight = 4f;
        CollisionShape pedestal = new CylinderCollisionShape(
                pedestalRadius, pedestalHeight, PhysicsSpace.AXIS_Y);

        CollisionShape top = new CylinderCollisionShape(
                topRadius, thickness, PhysicsSpace.AXIS_Y);

        CompoundCollisionShape result = new CompoundCollisionShape(4);
        result.addChildShape(pedestal, 0f, -1f, 0f);
        result.addChildShape(top, 0f, 1.2f, 0f);

        Vector3f footOffset = new Vector3f(0f, -2.75f, 0f);
        result.addChildShape(foot, footOffset);

        Matrix3f rotY90 = new Matrix3f();
        rotY90.fromAngleAxis(FastMath.HALF_PI, new Vector3f(0f, 1f, 0f));
        result.addChildShape(foot, footOffset, rotY90);

        return result;
    }

    /**
     * Generate a thumb tack (drawing pin) with a round, flat head on the +Y
     * axis.
     *
     * @param headRadius the radius of the head (in unscaled units,
     * &gt;spikeRadius)
     * @param spikeRadius the radius of the spike (in unscaled units, &gt;0)
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeThumbTack(
            float headRadius, float spikeRadius) {
        float headThickness = 0.4f;
        CollisionShape head = new CylinderCollisionShape(
                headRadius, headThickness, PhysicsSpace.AXIS_Y);

        float spikeHeight = 3f;
        CollisionShape spike = new ConeCollisionShape(
                spikeRadius, spikeHeight, PhysicsSpace.AXIS_Y);

        CompoundCollisionShape result = new CompoundCollisionShape(2);
        result.addChildShape(head, 0f, -0.5f, 0f);
        result.addChildShape(spike, 0f, 1f, 0f);

        return result;
    }

    /**
     * Generate a top with a cylindrical body, its handle on the +Y axis.
     *
     * @param bodyRadius the radius of the body (in unscaled shape units,
     * &gt;handleRadius)
     * @param handleRadius the radius of the handle (in unscaled shape units,
     * &gt;0)
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeTop(
            float bodyRadius, float handleRadius) {
        float bodyHeight = 0.6f;
        CollisionShape body = new CylinderCollisionShape(
                bodyRadius, bodyHeight, PhysicsSpace.AXIS_Y);

        float coneHeight = 1.5f;
        CollisionShape cone = new ConeCollisionShape(
                bodyRadius - 0.06f, coneHeight, PhysicsSpace.AXIS_Y);

        float handleHeight = 1.5f;
        CollisionShape handle = new CapsuleCollisionShape(
                handleRadius, handleHeight, PhysicsSpace.AXIS_Y);

        CompoundCollisionShape result = new CompoundCollisionShape(3);
        result.addChildShape(body);
        float yOffset = (coneHeight + bodyHeight) / 2f;
        result.addChildShape(cone, 0f, yOffset, 0f);
        yOffset = -0.5f * (handleHeight + bodyHeight);
        result.addChildShape(handle, 0f, yOffset, 0f);

        return result;
    }

    /**
     * Approximate a torus (or donut), open on the Z axis, using capsules
     * arranged in a circle.
     *
     * @param majorRadius (in unscaled shape units, &gt;minorRadius)
     * @param minorRadius (in unscaled shape units, &gt;0, &lt;majorRadius)
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape
            makeTorus(float majorRadius, float minorRadius) {
        Validate.inRange(
                majorRadius, "major radius", minorRadius, Float.MAX_VALUE);
        Validate.inRange(
                minorRadius, "minor radius", Float.MIN_VALUE, majorRadius);

        int numCapsules = 20;
        float angle = FastMath.TWO_PI / numCapsules;
        float length = majorRadius * angle;
        CollisionShape capsule = new CapsuleCollisionShape(
                minorRadius, length, PhysicsSpace.AXIS_X);

        CompoundCollisionShape result = new CompoundCollisionShape(numCapsules);
        Vector3f offset = new Vector3f();
        Matrix3f rotation = new Matrix3f();

        for (int childI = 0; childI < numCapsules; ++childI) {
            float theta = angle * childI;
            offset.x = majorRadius * FastMath.sin(theta);
            offset.y = majorRadius * FastMath.cos(theta);
            rotation.fromAngleNormalAxis(-theta, Vector3f.UNIT_Z);

            result.addChildShape(capsule, offset, rotation);
        }

        return result;
    }

    /**
     * Generate a square tray with a central deflector. Not intended for use in
     * a dynamic body.
     *
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeTray() {
        float iHeight = 24f;
        float iWidth = 24f;
        float iDepth = 3f;
        float wallThickness = 3f;
        CompoundCollisionShape result
                = makeLidlessBox(iHeight, iWidth, iDepth, wallThickness);

        Matrix3f rotMatrix = new Matrix3f();
        rotMatrix.fromAngleNormalAxis(-FastMath.HALF_PI, Vector3f.UNIT_X);
        result.rotate(rotMatrix);

        float baseY = -1.5f;
        Vector3f depress = new Vector3f(0f, baseY, 0f);
        result.translate(depress);

        // Place a tetrahedral deflector in the center.
        float size = 3f;
        Vector3f p1 = new Vector3f(0f, size, 0f);
        Vector3f p2 = new Vector3f(-size, baseY, size);
        Vector3f p3 = new Vector3f(-size, baseY, -size);
        Vector3f p4 = new Vector3f(size * MyMath.root2, baseY, 0f);
        CollisionShape child = new SimplexCollisionShape(p1, p2, p3, p4);
        result.addChildShape(child);

        return result;
    }

    /**
     * Generate a trident.
     *
     * @param shaftLength (Y direction, in unscaled shape units, &gt;0)
     * @param shaftRadius (in unscaled shape units, &gt;0)
     *
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape
            makeTrident(float shaftLength, float shaftRadius) {
        Validate.positive(shaftLength, "shaft length");
        Validate.positive(shaftRadius, "shaft radius");

        // Create a cylinder for the shaft.
        CollisionShape shaft = new CylinderCollisionShape(
                shaftRadius, shaftLength, PhysicsSpace.AXIS_Y);
        float shaftOffset = 0.2f * shaftLength;

        // Create a box for the crosspiece.
        float halfCross = 5f * shaftRadius;
        float halfThickness = 0.75f * shaftRadius;
        float margin = CollisionShape.getDefaultMargin();
        CollisionShape crosspiece = new BoxCollisionShape(halfCross + margin,
                halfThickness + margin, halfThickness + margin);

        // Create pyramidal hulls for each of the 3 prongs.
        float baseX = halfCross - halfThickness;
        float pointX = halfCross + 2f * halfThickness;
        float crossOffset
                = shaftLength / 2f - shaftOffset + halfThickness + margin;
        float baseY = crossOffset + halfThickness;
        float sideY = baseY + 3f * halfCross;
        float mainY = baseY + 4f * halfCross;
        float[] array1 = {
            halfCross, baseY, +halfThickness,
            halfCross, baseY, -halfThickness,
            baseX, baseY, +halfThickness,
            baseX, baseY, -halfThickness,
            pointX, sideY, 0f
        };
        CollisionShape rightProng = new HullCollisionShape(array1);

        float[] array2 = {
            -halfThickness, baseY, +halfThickness,
            -halfThickness, baseY, -halfThickness,
            +halfThickness, baseY, +halfThickness,
            +halfThickness, baseY, -halfThickness,
            0f, mainY, 0f
        };
        CollisionShape middleProng = new HullCollisionShape(array2);

        float[] array3 = {
            -halfCross, baseY, +halfThickness,
            -halfCross, baseY, -halfThickness,
            -baseX, baseY, +halfThickness,
            -baseX, baseY, -halfThickness,
            -pointX, sideY, 0f
        };
        CollisionShape leftProng = new HullCollisionShape(array3);

        CompoundCollisionShape result = new CompoundCollisionShape(5);
        result.addChildShape(shaft, 0f, -shaftOffset, 0f);
        result.addChildShape(crosspiece, 0f, crossOffset, 0f);
        result.addChildShape(rightProng);
        result.addChildShape(middleProng);
        result.addChildShape(leftProng);

        return result;
    }
}
