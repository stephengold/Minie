/*
 Copyright (c) 2019-2020, Stephen Gold
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
import com.jme3.bullet.collision.shapes.Convex2dShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.HeightfieldCollisionShape;
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
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.Validate;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.RectangularSolid;
import jme3utilities.mesh.DomeMesh;
import jme3utilities.mesh.Icosahedron;
import jme3utilities.mesh.RoundedRectangle;
import jme3utilities.minie.MyShape;
import jme3utilities.minie.test.mesh.StarSlice;
import jme3utilities.minie.test.terrain.MinieTestTerrains;

/**
 * Utility class to generate collision shapes for use in MinieExamples.
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
    // fields

    /**
     * inverse moment-of-inertia vector for "chair" shape (initialized by
     * makeChair()
     */
    public static Vector3f chairInverseInertia = null;
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private MinieTestShapes() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Add some shapes to the specified library.
     *
     * @param namedShapes where to add shapes (not null, modified)
     */
    public static void addShapes(Map<String, CollisionShape> namedShapes) {
        CollisionShape barbell = makeBarbell(); // TODO randomize
        namedShapes.put("barbell", barbell);

        CollisionShape bedOfNails = makeBedOfNails();
        namedShapes.put("bedOfNails", bedOfNails);

        CollisionShape chair = makeChair(); // TODO randomize
        namedShapes.put("chair", chair);

        {
            int numSides = 4;
            float radius = 20f;
            float depth = 12f;
            CollisionShape corner = makeCorner(numSides, radius, depth);
            namedShapes.put("corner", corner);
        }

        CollisionShape dimples = makeDimples();
        namedShapes.put("dimples", dimples);

        CollisionShape knucklebone = makeKnucklebone(); // TODO randomize
        namedShapes.put("knucklebone", knucklebone);

        CollisionShape ladder = makeLadder(); // TODO randomize
        namedShapes.put("ladder", ladder);

        CollisionShape roundedRectangle = makeRoundedRectangle();
        namedShapes.put("roundedRectangle", roundedRectangle);

        CollisionShape sieve = makeSieve();
        namedShapes.put("sieve", sieve);

        CollisionShape smooth = makeSmoothHeightfield();
        namedShapes.put("smooth", smooth);

        CollisionShape table = makeTable(); // TODO randomize
        namedShapes.put("table", table);

        CollisionShape thumbTack = makeThumbTack(); // TODO randomize
        namedShapes.put("thumbTack", thumbTack);

        CollisionShape top = makeTop(); // TODO randomize
        namedShapes.put("top", top);

        CollisionShape tray = makeTray();
        namedShapes.put("tray", tray);

        CollisionShape triangle = makeTriangle();
        namedShapes.put("triangle", triangle);
    }

    /**
     * Generate a barbell shape with 2 cylindrical plates.
     *
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeBarbell() {
        float barRadius = 0.2f;
        float plateOffset = 2f;
        CollisionShape bar = new CylinderCollisionShape(barRadius,
                2.4f * plateOffset, PhysicsSpace.AXIS_X);

        float plateRadius = 1f;
        float plateThickness = 0.4f;
        CollisionShape plate = new CylinderCollisionShape(plateRadius,
                plateThickness, PhysicsSpace.AXIS_X);

        CompoundCollisionShape result = new CompoundCollisionShape();
        result.addChildShape(bar);
        result.addChildShape(plate, -plateOffset, 0f, 0f);
        result.addChildShape(plate, plateOffset, 0f, 0f);

        return result;
    }

    /**
     * Generate a bed-of-nails heightfield. Not intended for use in a dynamic
     * body.
     *
     * @return a new heightfield shape (not null)
     */
    public static HeightfieldCollisionShape makeBedOfNails() {
        int n = 64;
        float[] array = MinieTestTerrains.bedOfNailsArray(n);

        Vector3f scale = new Vector3f(40f / n, 4f, 40f / n);
        int upAxis = PhysicsSpace.AXIS_Y;
        boolean flipQuadEdges = false;
        boolean flipTriangleWinding = false;
        boolean useDiamond = true;
        boolean useZigzag = false;
        HeightfieldCollisionShape result
                = new HeightfieldCollisionShape(n, n, array, scale, upAxis,
                        flipQuadEdges, flipTriangleWinding, useDiamond,
                        useZigzag);

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
        CollisionShape seat = new MultiSphere(solid);

        float frontLength = 2f;
        CollisionShape frontLeg = new CylinderCollisionShape(legRadius,
                frontLength, PhysicsSpace.AXIS_Y);

        float rearLength = 5f;
        CollisionShape rearLeg = new CylinderCollisionShape(legRadius,
                rearLength, PhysicsSpace.AXIS_Y);

        float rearHalf = rearLength / 2f;
        float frontHalf = frontLength / 2f;
        float backHalf = rearHalf - frontHalf;
        halfExtents.set(legOffset, backHalf, legRadius);
        solid = new RectangularSolid(halfExtents);
        CollisionShape back = new MultiSphere(solid);

        CompoundCollisionShape result = new CompoundCollisionShape();
        result.addChildShape(seat);
        result.addChildShape(frontLeg, legOffset, -frontHalf, legOffset);
        frontLeg = (CollisionShape) Heart.deepCopy(frontLeg);
        result.addChildShape(frontLeg, -legOffset, -frontHalf, legOffset);
        float yOffset = rearHalf - frontLength;
        result.addChildShape(rearLeg, legOffset, yOffset, -legOffset);
        rearLeg = (CollisionShape) Heart.deepCopy(rearLeg);
        result.addChildShape(rearLeg, -legOffset, yOffset, -legOffset);
        result.addChildShape(back, 0f, backHalf, -legOffset);

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
     * Generate an inverted hollow pyramid with its nadir on the -Y axis. Not
     * intended for use in a dynamic body.
     *
     * @param numSides (&ge;3)
     * @param rimRadius (&gt;0)
     * @param depth rimY minus nadirY (&gt;0)
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeCorner(int numSides,
            float rimRadius, float depth) {
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

        CompoundCollisionShape result = new CompoundCollisionShape();
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
     * Generate a dimpled heightfield. Not intended for use in a dynamic body.
     *
     * @return a new heightfield shape (not null)
     */
    public static HeightfieldCollisionShape makeDimples() {
        int n = 128;
        float[] array = MinieTestTerrains.dimplesArray(n);

        Vector3f scale = new Vector3f(40f / n, 1f, 40f / n);
        HeightfieldCollisionShape result
                = new HeightfieldCollisionShape(array, scale);

        return result;
    }

    /**
     * Generate a spherical dome or plano-convex lens.
     *
     * @param radius (in unscaled shape units, &gt;0)
     * @param verticalAngle (in radians, &lt;Pi, &gt;0)
     * @return a new shape
     */
    public static HullCollisionShape makeDome(float radius,
            float verticalAngle) {
        Validate.positive(radius, "radius");
        Validate.inRange(verticalAngle, "vertical angle", 0f, FastMath.PI);

        int rimSamples = 20;
        int quadrantSamples = 10;
        DomeMesh mesh = new DomeMesh(rimSamples, quadrantSamples);
        mesh.setVerticalAngle(verticalAngle);
        FloatBuffer buffer = mesh.getFloatBuffer(VertexBuffer.Type.Position);
        /*
         * Scale mesh positions to the desired radius.
         */
        int start = 0;
        int end = buffer.limit();
        Vector3f scale = new Vector3f(radius, radius, radius);
        MyBuffer.scale(buffer, start, end, scale);
        /*
         * Use max-min to center the vertices.
         */
        Vector3f max = new Vector3f();
        Vector3f min = new Vector3f();
        MyBuffer.maxMin(buffer, start, end, max, min);
        Vector3f offset = MyVector3f.midpoint(min, max, null).negateLocal();
        MyBuffer.translate(buffer, start, end, offset);

        HullCollisionShape result = new HullCollisionShape(buffer);

        return result;
    }

    /**
     * Generate a (gridiron) football using overlapping spheres arranged in a
     * row.
     *
     * @param midRadius the radius of the Y-Z cross section at X=0 (&ge;0)
     * @return a new MultiSphere shape (not null)
     */
    public static MultiSphere makeFootball(float midRadius) {
        float genRadius = 2f * midRadius; // curvature radius of the generatrix
        float endRadius = 0.5f * midRadius; // controls pointiness of the ends

        int numSpheres = 9;
        List<Vector3f> centers = new ArrayList<>(numSpheres);
        List<Float> radii = new ArrayList<>(numSpheres);

        float centerY = genRadius - midRadius;
        float maxX = FastMath.sqrt(genRadius * genRadius - centerY * centerY);
        float lastCenterX = maxX - endRadius;

        float xStep = (2f * lastCenterX) / (numSpheres - 1);
        for (int sphereI = 0; sphereI < numSpheres; ++sphereI) {
            float centerX = -lastCenterX + sphereI * xStep;
            // centerX varies from -lastCenterX to +lastCenterX
            centers.add(new Vector3f(centerX, 0f, 0f));

            float radius = genRadius - MyMath.hypotenuse(centerX, centerY);
            radii.add(radius);
        }

        MultiSphere result = new MultiSphere(centers, radii);

        return result;
    }

    /**
     * Generate a rectangular frame.
     *
     * @param ihHeight half of the internal height (Y direction, &gt;0)
     * @param ihWidth half of the internal width (X direction, &gt;0)
     * @param halfDepth half of the (external) depth (Z direction, &gt;0)
     * @param halfThickness half the thickness (&gt;0)
     * @return a new shape
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

        CompoundCollisionShape result = new CompoundCollisionShape();
        result.addChildShape(horizontal, halfThickness, -mhHeight, 0f);
        result.addChildShape(horizontal, -halfThickness, mhHeight, 0f);
        result.addChildShape(vertical, mhWidth, halfThickness, 0f);
        result.addChildShape(vertical, -mhWidth, -halfThickness, 0f);

        return result;
    }

    /**
     * Generate a Z-axis half-pipe shape.
     *
     * @param innerR the inner radius of an X-Y cross section (in unscaled shape
     * units, &gt;0)
     * @param thickness the thickness of the pipe (in unscaled shape units,
     * &gt;0)
     * @param zLength the total length of the pipe along the Z axis (in unscaled
     * shape units, &gt;0)
     * @param numChildren the number of child shapes to create (&ge;2)
     * @return a new compound shape
     */
    public static CompoundCollisionShape makeHalfPipe(float innerR,
            float thickness, float zLength, int numChildren) {
        Validate.positive(innerR, "inner radius");
        Validate.positive(thickness, "thickness");
        Validate.positive(zLength, "length");
        Validate.inRange(numChildren, "number of children",
                2, Integer.MAX_VALUE);

        float halfLength = zLength / 2f;
        float outerR = innerR + thickness;
        float yOff = outerR / 2f;
        float segmentAngle = FastMath.PI / numChildren;

        CompoundCollisionShape result = new CompoundCollisionShape();
        for (int segmentI = 0; segmentI < numChildren; ++segmentI) {
            float theta1 = segmentI * segmentAngle;
            float theta2 = (segmentI + 1) * segmentAngle;
            float cos1 = FastMath.cos(theta1);
            float cos2 = FastMath.cos(theta2);
            float sin1 = FastMath.sin(theta1);
            float sin2 = FastMath.sin(theta2);

            FloatBuffer buffer = BufferUtils.createFloatBuffer(
                    innerR * cos1, innerR * sin1 - yOff, halfLength,
                    innerR * cos2, innerR * sin2 - yOff, halfLength,
                    outerR * cos1, outerR * sin1 - yOff, halfLength,
                    outerR * cos2, outerR * sin2 - yOff, halfLength,
                    innerR * cos1, innerR * sin1 - yOff, -halfLength,
                    innerR * cos2, innerR * sin2 - yOff, -halfLength,
                    outerR * cos1, outerR * sin1 - yOff, -halfLength,
                    outerR * cos2, outerR * sin2 - yOff, -halfLength
            );
            HullCollisionShape child = new HullCollisionShape(buffer);
            result.addChildShape(child);
        }

        return result;
    }

    /**
     * Generate an I-beam.
     *
     * @param length (Z axis, &gt;0)
     * @param flangeWidth (X axis, &ge;thickness)
     * @param beamHeight (Y axis, &ge;2*thickness)
     * @param thickness (&gt;0)
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeIBeam(float length,
            float flangeWidth, float beamHeight, float thickness) {
        Validate.positive(length, "length");
        Validate.positive(thickness, "thickness");
        Validate.inRange(flangeWidth, "flange width", thickness,
                Float.MAX_VALUE);
        Validate.inRange(beamHeight, "beam height", 2f * thickness,
                Float.MAX_VALUE);

        float halfLength = length / 2f;
        float halfThickness = thickness / 2f;
        float webHalfHeight = beamHeight / 2f - thickness;
        CollisionShape web = new BoxCollisionShape(halfThickness, webHalfHeight,
                halfLength);
        CollisionShape flange = new BoxCollisionShape(flangeWidth / 2f,
                halfThickness, halfLength);

        CompoundCollisionShape result = new CompoundCollisionShape();
        result.addChildShape(web);
        float flangeY = webHalfHeight + halfThickness;
        result.addChildShape(flange, 0f, flangeY, 0f);
        result.addChildShape(flange, 0f, -flangeY, 0f);

        return result;
    }

    /**
     * Generate a knucklebone with 4 spherical balls.
     *
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeKnucklebone() {
        float stemLength = 2.5f;
        float stemRadius = 0.25f;
        CollisionShape xStem = new CapsuleCollisionShape(stemRadius, stemLength,
                PhysicsSpace.AXIS_X);
        CollisionShape yStem = new CapsuleCollisionShape(stemRadius, stemLength,
                PhysicsSpace.AXIS_Y);
        CollisionShape zStem = new CapsuleCollisionShape(stemRadius, stemLength,
                PhysicsSpace.AXIS_Z);

        float ballRadius = 0.4f;
        CollisionShape ball = new SphereCollisionShape(ballRadius);

        CompoundCollisionShape result = new CompoundCollisionShape();
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
     * Generate a mad mallet by compounding 2 cylinders. TODO re-order methods
     *
     * @param handleR the radius of the handle (in unscaled shape units, &gt;0)
     * @param headR the radius of the head (in unscaled shape units, &gt;0)
     * @param handleHalfLength half the length of the handle (in unscaled shape
     * units, &gt;0)
     * @param headHalfLength half the length of the head (in unscaled shape
     * units, &gt;0)
     * @return a new compound shape
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

        CompoundCollisionShape compound = new CompoundCollisionShape();

        compound.addChildShape(handle, 0f, 0f, handleHalfLength);

        Vector3f offset = new Vector3f(0f, 0f, 2f * handleHalfLength);
        Matrix3f rotation = new Matrix3f();
        rotation.fromAngleAxis(FastMath.HALF_PI, Vector3f.UNIT_X);
        compound.addChildShape(head, offset, rotation);

        return compound;
    }

    /**
     * Generate a ladder with 5 cylindrical rungs.
     *
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeLadder() {
        float rungRadius = 0.2f;
        float rungLength = 2f;
        CollisionShape rung = new CylinderCollisionShape(rungRadius,
                rungLength, PhysicsSpace.AXIS_X);

        float railHalf = 6f;
        CollisionShape rail
                = new BoxCollisionShape(rungRadius, railHalf, rungRadius);

        CompoundCollisionShape result = new CompoundCollisionShape();
        float rungSpacing = 2f;
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
     * @param iHeight the internal height (Y direction, &gt;0)
     * @param iWidth the internal width (X direction, &gt;0)
     * @param iDepth of the interal depth (Z direction, &gt;0)
     * @param wallThickness (&gt;0)
     *
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeLidlessBox(float iHeight,
            float iWidth, float iDepth, float wallThickness) {
        float ihHeight = iHeight / 2;
        float ihWidth = iWidth / 2;
        float ihDepth = iDepth / 2;
        float halfThickness = wallThickness / 2;

        float fhDepth = ihDepth + 2f * halfThickness;
        CompoundCollisionShape result
                = makeFrame(ihHeight, ihWidth, fhDepth, halfThickness);

        BoxCollisionShape bottom
                = new BoxCollisionShape(ihWidth, ihHeight, halfThickness);
        float bottomZ = -ihDepth - halfThickness;
        result.addChildShape(bottom, 0f, 0f, bottomZ);

        return result;
    }

    /**
     * Generate a large rounded rectangle.
     *
     * @return a new convex 2-D shape (not null)
     */
    public static Convex2dShape makeRoundedRectangle() {
        float halfExtent = 20f;

        float x2 = halfExtent;
        float y2 = halfExtent / Icosahedron.phi;
        float x1 = -x2;
        float y1 = -y2;
        float cornerRadius = 4f;
        float zNorm = 1f;
        Mesh mesh = new RoundedRectangle(x1, x2, y1, y2, cornerRadius, zNorm);
        HullCollisionShape hull = new HullCollisionShape(mesh);
        Convex2dShape result = new Convex2dShape(hull);

        return result;
    }

    /**
     * Generate a wire sieve, backed by a horizontal plane. Not intended for use
     * in a dynamic body.
     *
     * @return a new compound shape
     */
    public static CompoundCollisionShape makeSieve() {
        float wireSpacing = 2f;
        float yHeight = 7f;
        int numXWires = 16;
        int numZWires = 16;

        float xHalfExtent = (numZWires - 1) * wireSpacing / 2f;
        float zHalfExtent = (numXWires - 1) * wireSpacing / 2f;
        CompoundCollisionShape result = new CompoundCollisionShape();
        /*
         * Add numXWires wires parallel to the X axis.
         */
        Vector3f p0 = new Vector3f(-xHalfExtent, 0f, 0f);
        Vector3f p1 = new Vector3f(xHalfExtent, 0f, 0f);
        SimplexCollisionShape xWire = new SimplexCollisionShape(p0, p1);

        for (int zIndex = 0; zIndex < numXWires; ++zIndex) {
            float z = -zHalfExtent + wireSpacing * zIndex;
            result.addChildShape(xWire, 0f, yHeight, z);
        }
        /*
         * Add numZWires wires parallel to the Z axis.
         */
        p0.set(0f, 0f, -zHalfExtent);
        p1.set(0f, 0f, zHalfExtent);
        SimplexCollisionShape zWire = new SimplexCollisionShape(p0, p1);

        for (int xIndex = 0; xIndex < numZWires; ++xIndex) {
            float x = -xHalfExtent + wireSpacing * xIndex;
            result.addChildShape(zWire, x, yHeight, 0f);
        }
        /*
         * Add a plane to catch any drops that pass through the sieve.
         */
        Plane plane = new Plane(Vector3f.UNIT_Y, 0f);
        PlaneCollisionShape pcs = new PlaneCollisionShape(plane);
        result.addChildShape(pcs);

        return result;
    }

    /**
     * Generate a smooth 64x64 heightfield. Not intended for use in a dynamic
     * body.
     *
     * @return a new heightfield shape (not null)
     */
    public static HeightfieldCollisionShape makeSmoothHeightfield() {
        int n = 64;
        float[] array = MinieTestTerrains.quadraticArray(n);

        Vector3f scale = new Vector3f(40f / n, 12.5f, 40f / n);
        HeightfieldCollisionShape result
                = new HeightfieldCollisionShape(array, scale);

        return result;
    }

    /**
     * Generate a 3-ball snowman with its head on the +Y axis.
     *
     * @param baseRadius (&gt;0)
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeSnowman(float baseRadius) {
        Validate.positive(baseRadius, "base radius");

        float verticalAngle = 2f;
        HullCollisionShape base = makeDome(baseRadius, verticalAngle);

        float torsoRadius = 0.8f * baseRadius;
        SphereCollisionShape torso = new SphereCollisionShape(torsoRadius);

        float headRadius = 0.6f * baseRadius;
        SphereCollisionShape head = new SphereCollisionShape(headRadius);

        CompoundCollisionShape result = new CompoundCollisionShape();
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
     * @param trianglesPerSlide the number of mesh triangles per slice (4 or 6)
     * @return a new shape
     */
    public static CompoundCollisionShape makeStar(int numPoints,
            float outerRadius, float centerY, float radiusRatio,
            int trianglesPerSlide) {
        Validate.inRange(numPoints, "number of points", 2, Integer.MAX_VALUE);
        Validate.positive(outerRadius, "outer radius");
        Validate.positive(centerY, "center Y");
        Validate.fraction(radiusRatio, "radius ratio");
        Validate.inRange(trianglesPerSlide, "triangles per slice", 4, 6);

        float innerRadius = radiusRatio * outerRadius;
        float sliceAngle = FastMath.TWO_PI / numPoints; // in radians
        boolean normals = false;
        StarSlice sliceMesh = new StarSlice(sliceAngle, innerRadius,
                outerRadius, 2f * centerY, normals, trianglesPerSlide);
        CollisionShape sliceShape = new HullCollisionShape(sliceMesh);

        CompoundCollisionShape result = new CompoundCollisionShape();
        Matrix3f rotate = new Matrix3f();
        for (int pointIndex = 0; pointIndex < numPoints; ++pointIndex) {
            rotate.fromAngleAxis(sliceAngle * pointIndex, Vector3f.UNIT_Y);
            result.addChildShape(sliceShape, Vector3f.ZERO, rotate);
        }

        return result;
    }

    /**
     * Generate a round pedestal table.
     *
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeTable() {
        float thickness = 0.4f;
        CollisionShape foot = new BoxCollisionShape(thickness / 2, 0.25f, 2f);

        float pedestalRadius = 0.3f;
        float pedestalHeight = 4f;
        CollisionShape pedestal = new CylinderCollisionShape(pedestalRadius,
                pedestalHeight, PhysicsSpace.AXIS_Y);

        float topRadius = 3f;
        CollisionShape top = new CylinderCollisionShape(topRadius, thickness,
                PhysicsSpace.AXIS_Y);

        CompoundCollisionShape result = new CompoundCollisionShape();
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
     * Generate a thumb tack (drawing pin) with a round, flat head.
     *
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeThumbTack() {
        float headRadius = 2f;
        float headThickness = 0.4f;
        CollisionShape head = new CylinderCollisionShape(headRadius,
                headThickness, PhysicsSpace.AXIS_Y);

        float spikeHeight = 3f;
        float spikeRadius = 0.2f;
        CollisionShape spike = new ConeCollisionShape(spikeRadius, spikeHeight,
                PhysicsSpace.AXIS_Y);

        CompoundCollisionShape result = new CompoundCollisionShape();
        result.addChildShape(head, 0f, -0.5f, 0f);
        result.addChildShape(spike, 0f, 1f, 0f);

        return result;
    }

    /**
     * Generate a top with a cylindrical body.
     *
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeTop() {
        float bodyRadius = 1.5f;
        float bodyHeight = 0.6f;
        CollisionShape body = new CylinderCollisionShape(bodyRadius,
                bodyHeight, PhysicsSpace.AXIS_Y);

        float coneHeight = 1.5f;
        CollisionShape cone = new ConeCollisionShape(bodyRadius - 0.06f,
                coneHeight, PhysicsSpace.AXIS_Y);

        float handleHeight = 1.5f;
        float handleRadius = 0.3f;
        CollisionShape handle = new CapsuleCollisionShape(handleRadius,
                handleHeight, PhysicsSpace.AXIS_Y);

        CompoundCollisionShape result = new CompoundCollisionShape();
        result.addChildShape(body);
        float yOffset = (coneHeight + bodyHeight) / 2f;
        result.addChildShape(cone, 0f, yOffset, 0f);
        yOffset = -0.5f * (handleHeight + bodyHeight);
        result.addChildShape(handle, 0f, yOffset, 0f);

        return result;
    }

    /**
     * Approximate a torus or donut using capsules arranged in a circle.
     *
     * @param majorRadius (in unscaled shape units, &gt;minorRadius)
     * @param minorRadius (in unscaled shape units, &gt;0, &lt;majorRadius)
     * @return a new shape
     */
    public static CompoundCollisionShape makeTorus(float majorRadius,
            float minorRadius) {
        Validate.inRange(majorRadius, "major radius", minorRadius,
                Float.MAX_VALUE);
        Validate.inRange(minorRadius, "minor radius", Float.MIN_VALUE,
                majorRadius);

        int numCapsules = 20;
        float angle = FastMath.TWO_PI / numCapsules;
        float length = majorRadius * angle;
        CollisionShape capsule = new CapsuleCollisionShape(minorRadius,
                length, PhysicsSpace.AXIS_X);

        CompoundCollisionShape result = new CompoundCollisionShape();
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

    /**
     * Generate a large isosceles triangle.
     *
     * @return a new simplex shape (not null)
     */
    public static SimplexCollisionShape makeTriangle() {
        float radius = 40f;

        float x = radius * MyMath.rootHalf;
        Vector3f p1 = new Vector3f(x, 0f, x);
        Vector3f p2 = new Vector3f(x, 0f, -x);
        Vector3f p3 = new Vector3f(-x, 0f, 0f);
        SimplexCollisionShape result = new SimplexCollisionShape(p1, p2, p3);

        return result;
    }

    /**
     * Generate a triangular frame with identical sides.
     *
     * @param internalLength the internal length of each side (&gt;0)
     * @param depth the (external) depth (Z direction, &gt;0)
     * @param thickness the thickness of each side (&gt;0)
     * @return a new shape (not null)
     */
    public static CompoundCollisionShape makeTriangularFrame(
            float internalLength, float depth, float thickness) {
        Validate.positive(internalLength, "internal length");
        Validate.positive(depth, "depth");
        Validate.positive(thickness, "thickness");

        float innerX = internalLength / ShapeGenerator.root3;
        float outerX = innerX + 2f * thickness;
        float halfDepth = depth / 2f;

        float stepAngle = FastMath.TWO_PI / 3f; // in radians
        Matrix3f rot1 = new Matrix3f();
        rot1.fromAngleAxis(stepAngle, Vector3f.UNIT_Z);
        Matrix3f rot2 = new Matrix3f();
        rot2.fromAngleAxis(2f * stepAngle, Vector3f.UNIT_Z);

        Vector3f inner1 = rot1.mult(new Vector3f(innerX, 0f, 0f), null);
        Vector3f outer1 = rot1.mult(new Vector3f(outerX, 0f, 0f), null);

        float[] array = {
            innerX, 0f, +halfDepth,
            innerX, 0f, -halfDepth,
            outerX, 0f, +halfDepth,
            outerX, 0f, -halfDepth,
            inner1.x, inner1.y, +halfDepth,
            inner1.x, inner1.y, -halfDepth,
            outer1.x, outer1.y, +halfDepth,
            outer1.x, outer1.y, -halfDepth
        };
        CollisionShape side = new HullCollisionShape(array);

        CompoundCollisionShape result = new CompoundCollisionShape();
        result.addChildShape(side);
        result.addChildShape(side, Vector3f.ZERO, rot1);
        result.addChildShape(side, Vector3f.ZERO, rot2);

        return result;
    }

    /**
     * Generate a trident.
     *
     * @param shaftLength (Y direction, &gt;0)
     * @param shaftRadius (&gt;0)
     *
     * @return a new compound shape (not null)
     */
    public static CompoundCollisionShape makeTrident(float shaftLength,
            float shaftRadius) {
        Validate.positive(shaftLength, "shaft length");
        Validate.positive(shaftRadius, "shaft radius");
        /*
         * Create a cylinder for the shaft.
         */
        CollisionShape shaft = new CylinderCollisionShape(shaftRadius,
                shaftLength, PhysicsSpace.AXIS_Y);
        float shaftOffset = 0.2f * shaftLength;
        /*
         * Create a box for the crosspiece.
         */
        float halfCross = 5f * shaftRadius;
        float halfThickness = 0.75f * shaftRadius;
        float margin = CollisionShape.getDefaultMargin();
        CollisionShape crosspiece = new BoxCollisionShape(halfCross + margin,
                halfThickness + margin, halfThickness + margin);
        /*
         * Create pyramidal hulls for each of the 3 prongs.
         */
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

        CompoundCollisionShape result = new CompoundCollisionShape();
        result.addChildShape(shaft, 0f, -shaftOffset, 0f);
        result.addChildShape(crosspiece, 0f, crossOffset, 0f);
        result.addChildShape(rightProng);
        result.addChildShape(middleProng);
        result.addChildShape(leftProng);

        return result;
    }
}
