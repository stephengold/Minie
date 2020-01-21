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
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.HeightfieldCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.MultiSphere;
import com.jme3.bullet.collision.shapes.SimplexCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.math.FastMath;
import com.jme3.math.Matrix3f;
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
import jme3utilities.Misc;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.RectangularSolid;
import jme3utilities.math.noise.Generator;
import jme3utilities.mesh.DomeMesh;
import jme3utilities.mesh.Icosahedron;
import jme3utilities.mesh.Octahedron;
import jme3utilities.mesh.Prism;
import jme3utilities.minie.MyShape;
import jme3utilities.minie.test.mesh.StarSlice;

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
    // new methods exposed - TODO add makeSnowman(), more randomized shapes

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

        CollisionShape football = makeFootball();
        namedShapes.put("football", football);

        CollisionShape knucklebone = makeKnucklebone();
        namedShapes.put("knucklebone", knucklebone);

        CollisionShape ladder = makeLadder();
        namedShapes.put("ladder", ladder);

        CollisionShape smooth = makeSmoothHeightfield();
        namedShapes.put("smooth", smooth);

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
        frontLeg = (CollisionShape) Misc.deepCopy(frontLeg);
        result.addChildShape(frontLeg, -legOffset, -frontHalf, legOffset);
        float yOffset = rearHalf - frontLength;
        result.addChildShape(rearLeg, legOffset, yOffset, -legOffset);
        rearLeg = (CollisionShape) Misc.deepCopy(rearLeg);
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
     * Generate a (gridiron) football.
     *
     * @return a new MultiSphere shape (not null)
     */
    public static MultiSphere makeFootball() {
        float midRadius = 1f; // radius of the Y-Z cross section at X=0
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
     * Generate a knuclebone with 4 spherical balls.
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
     * Generate a smooth 64x64 heightfield shape.
     *
     * @return a new heightfield shape (not null)
     */
    public static HeightfieldCollisionShape makeSmoothHeightfield() {
        int n = 64;
        float halfNm1 = (n - 1) / 2f;
        float[] heightmap = new float[n * n];
        for (int i = 0; i < n; ++i) {
            float x = -1f + i / halfNm1; // -1 .. +1
            for (int j = 0; j < n; ++j) {
                float y = -1f + j / halfNm1; // -1 .. +1
                float r = MyMath.hypotenuse(x, y);
                int floatIndex = n * i + j;
                heightmap[floatIndex] = -0.4f + (r - 0.8f) * (r - 0.8f);
            }
        }
        Vector3f scale = new Vector3f(20f / halfNm1, 12.5f, 20f / halfNm1);
        HeightfieldCollisionShape result
                = new HeightfieldCollisionShape(heightmap, scale);
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
     * Randomly generate a 4-sphere shape, a box with rounded corners.
     *
     * @param generate pseudo-random generator (not null, modified)
     * @return a new shape (not null)
     */
    public static MultiSphere randomFourSphere(Generator generate) {
        float rx = 0.4f + 0.6f * generate.nextFloat();
        float ry = 1f + generate.nextFloat();
        float rz = 1f + generate.nextFloat();
        Vector3f halfExtents = new Vector3f(rx, ry, rz);

        RectangularSolid solid = new RectangularSolid(halfExtents);
        MultiSphere result = new MultiSphere(solid);

        return result;
    }

    /**
     * Randomly generate a centered HullCollisionShape.
     *
     * @param generate pseudo-random generator (not null, modified)
     * @return a new shape (not null)
     */
    public static HullCollisionShape randomHull(Generator generate) {
        int numVertices = 5 + generate.nextInt(21); // 5 .. 25

        FloatBuffer buffer;
        boolean noNormals = false;
        if (numVertices == 6) {
            /*
             * Generate a regular octahedron (6 vertices).
             */
            float radius = 0.7f + generate.nextFloat();
            Mesh mesh = new Octahedron(radius, noNormals);
            buffer = mesh.getFloatBuffer(VertexBuffer.Type.Position);

        } else if (numVertices == 12) {
            /*
             * Generate a regular icosahedron (12 vertices).
             */
            float radius = 0.6f + generate.nextFloat();
            Mesh mesh = new Icosahedron(radius, noNormals);
            buffer = mesh.getFloatBuffer(VertexBuffer.Type.Position);

        } else if (numVertices < 15 && numVertices % 2 == 0) {
            /*
             * Generate a prism (8, 10, or 14 vertices).
             */
            float radius = 0.6f + 0.5f * generate.nextFloat();
            float height = 1f + generate.nextFloat();
            int numSides = numVertices / 2;
            Mesh mesh = new Prism(numSides, radius, height, noNormals);
            buffer = mesh.getFloatBuffer(VertexBuffer.Type.Position);

        } else if (numVertices > 20) {
            /*
             * Generate a spherical dome or plano-convex lens (181 vertices).
             */
            float radius = 0.7f + generate.nextFloat();
            int rimSamples = 20;
            int quadrantSamples = 10;
            DomeMesh mesh = new DomeMesh(rimSamples, quadrantSamples);
            float verticalAngle = 0.7f + 1.3f * generate.nextFloat();
            mesh.setVerticalAngle(verticalAngle);
            buffer = mesh.getFloatBuffer(VertexBuffer.Type.Position);
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
            Vector3f offset
                    = MyVector3f.midpoint(min, max, null).negateLocal();
            MyBuffer.translate(buffer, start, end, offset);

        } else {
            /*
             * Generate a hull using the origin plus 4-19 random vertices.
             */
            buffer = BufferUtils.createFloatBuffer(
                    MyVector3f.numAxes * numVertices);
            buffer.put(0f).put(0f).put(0f);
            for (int vertexI = 1; vertexI < numVertices; ++vertexI) {
                Vector3f location = generate.nextUnitVector3f();
                location.multLocal(1.5f);
                buffer.put(location.x).put(location.y).put(location.z);
            }
            /*
             * Use arithmetic mean to center the vertices.
             */
            int start = 0;
            int end = buffer.limit();
            Vector3f offset = MyBuffer.mean(buffer, start, end, null);
            offset.negateLocal();
            MyBuffer.translate(buffer, start, end, offset);
        }

        HullCollisionShape result = new HullCollisionShape(buffer);

        return result;
    }

    /**
     * Randomly generate a MultiSphere shape with 1-4 spheres.
     *
     * @param generate pseudo-random generator (not null, modified)
     * @return a new shape (not null)
     */
    public static MultiSphere randomMultiSphere(Generator generate) {
        int numSpheres = 1 + generate.nextInt(4);
        if (numSpheres == 4) {
            MultiSphere result = randomFourSphere(generate);
            return result;
        }

        List<Vector3f> centers = new ArrayList<>(numSpheres);
        List<Float> radii = new ArrayList<>(numSpheres);
        /*
         * The first sphere is always centered.
         */
        centers.add(Vector3f.ZERO);
        float mainRadius = 0.8f + 0.6f * generate.nextFloat();
        radii.add(mainRadius);

        for (int sphereIndex = 1; sphereIndex < numSpheres; ++sphereIndex) {
            /*
             * Add a smaller sphere, offset from the main one.
             */
            Vector3f offset = generate.nextUnitVector3f();
            offset.multLocal(mainRadius);
            centers.add(offset);

            float radius = mainRadius * (0.2f + 0.8f * generate.nextFloat());
            radii.add(radius);
        }

        MultiSphere result = new MultiSphere(centers, radii);

        if (numSpheres == 1) {
            /*
             * Scale the sphere to make an ellipsoid.
             */
            float xScale = 1f + generate.nextFloat();
            float yScale = 0.6f + generate.nextFloat();
            float zScale = 0.4f + generate.nextFloat();
            result.setScale(new Vector3f(xScale, yScale, zScale));
        }

        return result;
    }

    /**
     * Randomly generate a star-shaped compound shape.
     *
     * @param generate pseudo-random generator (not null, modified)
     * @return a new shape (not null)
     */
    public static CompoundCollisionShape randomStar(Generator generate) {
        float centerY = 0.3f + 0.3f * generate.nextFloat();
        float outerRadius = 1f + 1.5f * generate.nextFloat();

        int numPoints = 4 + generate.nextInt(6); // 4 .. 9
        float radiusRatio = 0.2f + 0.5f * generate.nextFloat();
        float innerRadius = radiusRatio * outerRadius;
        float sliceAngle = FastMath.TWO_PI / numPoints; // in radians
        boolean normals = false;
        StarSlice sliceMesh = new StarSlice(sliceAngle, innerRadius,
                outerRadius, 2f * centerY, normals);
        CollisionShape sliceShape = new HullCollisionShape(sliceMesh);

        CompoundCollisionShape result = new CompoundCollisionShape();
        Matrix3f rotate = new Matrix3f();
        for (int i = 0; i < numPoints; ++i) {
            rotate.fromAngleAxis(sliceAngle * i, Vector3f.UNIT_Y);
            result.addChildShape(sliceShape, Vector3f.ZERO, rotate);
        }

        return result;
    }

    /**
     * Randomly generate a tetrahedral SimplexCollisionShape.
     *
     * @param generate pseudo-random generator (not null, modified)
     * @return a new shape (not null)
     */
    public static SimplexCollisionShape randomTetrahedron(Generator generate) {
        float r1 = 0.15f + generate.nextFloat();
        float r2 = 0.15f + generate.nextFloat();
        float r3 = 0.15f + generate.nextFloat();
        float r4 = 0.15f + generate.nextFloat();

        Vector3f p1 = new Vector3f(r1, r1, r1);
        Vector3f p2 = new Vector3f(r2, -r2, -r2);
        Vector3f p3 = new Vector3f(-r3, -r3, r3);
        Vector3f p4 = new Vector3f(-r4, r4, -r4);
        SimplexCollisionShape result
                = new SimplexCollisionShape(p1, p2, p3, p4);

        return result;
    }
}
