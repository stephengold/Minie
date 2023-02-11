/*
 Copyright (c) 2020-2023, Stephen Gold
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

import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.ConeCollisionShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.MultiSphere;
import com.jme3.bullet.collision.shapes.SimplexCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.Mesh.Mode;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Logger;
import jme3utilities.MyString;
import jme3utilities.Validate;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.RectangularSolid;
import jme3utilities.math.noise.Generator;
import jme3utilities.mesh.Cone;
import jme3utilities.mesh.Dodecahedron;
import jme3utilities.mesh.Icosahedron;
import jme3utilities.mesh.Octahedron;
import jme3utilities.mesh.Prism;

/**
 * Generate pseudo-random collision shapes for use in MinieExamples.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class ShapeGenerator extends Generator {
    // *************************************************************************
    // constants and loggers

    /**
     * square root of 3
     */
    final public static float root3 = FastMath.sqrt(3f);
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(ShapeGenerator.class.getName());
    // *************************************************************************
    // constructors

    /**
     * Instantiate a generator.
     */
    public ShapeGenerator() { // explicit to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Generate a box shape.
     *
     * @return a new shape (not null)
     */
    public BoxCollisionShape nextBox() {
        float rx = nextFloat(0.5f, 1.5f);
        float ry = nextFloat(0.5f, 1.5f);
        float rz = nextFloat(0.5f, 1.5f);
        Vector3f halfExtents = new Vector3f(rx, ry, rz);
        BoxCollisionShape result = new BoxCollisionShape(halfExtents);

        return result;
    }

    /**
     * Generate a capsule shape.
     *
     * @return a new shape (not null)
     */
    public CapsuleCollisionShape nextCapsule() {
        float radius = nextFloat(0.2f, 1.2f);
        float height = nextFloat(0.5f, 1.5f);
        int axis = nextInt(3);
        CapsuleCollisionShape result
                = new CapsuleCollisionShape(radius, height, axis);

        return result;
    }

    /**
     * Generate a cone shape.
     *
     * @return a new shape (not null)
     */
    public ConeCollisionShape nextCone() {
        float baseRadius = nextFloat(0.5f, 1.5f);
        float height = nextFloat(0.5f, 2.5f);
        int axisIndex = nextInt(3);
        ConeCollisionShape result
                = new ConeCollisionShape(baseRadius, height, axisIndex);

        return result;
    }

    /**
     * Generate a cylinder shape.
     *
     * @return a new shape (not null)
     */
    public CylinderCollisionShape nextCylinder() {
        float baseRadius = nextFloat(0.5f, 1.5f);
        float height = nextFloat(1f, 4f);
        int axisIndex = nextInt(3);
        CylinderCollisionShape result
                = new CylinderCollisionShape(baseRadius, height, axisIndex);

        return result;
    }

    /**
     * Generate a spherical dome or plano-convex lens.
     *
     * @return a new shape
     */
    public HullCollisionShape nextDome() {
        float radius = nextFloat(0.7f, 1.7f);
        float verticalAngle = nextFloat(0.7f, 2f);
        HullCollisionShape result
                = MinieTestShapes.makeDome(radius, verticalAngle);

        return result;
    }

    /**
     * Generate a (gridiron) football.
     *
     * @return a new shape
     */
    public MultiSphere nextFootball() {
        float midRadius = nextFloat(0.5f, 1.5f);
        MultiSphere result = MinieTestShapes.makeFootball(midRadius);

        return result;
    }

    /**
     * Generate a 4-sphere shape, a box with rounded corners.
     *
     * @return a new shape (not null)
     */
    public MultiSphere nextFourSphere() {
        float rx = nextFloat(0.4f, 1f);
        float ry = nextFloat(1f, 2f);
        float rz = nextFloat(1f, 2f);
        Vector3f halfExtents = new Vector3f(rx, ry, rz);

        RectangularSolid solid = new RectangularSolid(halfExtents);
        MultiSphere result = new MultiSphere(solid);

        return result;
    }

    /**
     * Generate a rectangular frame.
     *
     * @return a new shape
     */
    public CompoundCollisionShape nextFrame() {
        float halfDepth = nextFloat(0.1f, 0.5f);
        float ihHeight = nextFloat(0.7f, 2f);
        float ihWidth = 1.6f * ihHeight;
        float halfThickness = ihHeight * nextFloat(0.1f, 0.2f);
        CompoundCollisionShape result = CompoundTestShapes
                .makeFrame(ihHeight, ihWidth, halfDepth, halfThickness);

        return result;
    }

    /**
     * Approximate a Z-axis half-pipe shape.
     *
     * @return a new shape
     */
    public CompoundCollisionShape nextHalfPipe() {
        float innerRadius = nextFloat(0.5f, 1.5f);
        float thickness = nextFloat(0.2f, 0.5f);
        float length = nextFloat(1f, 4f);
        float arc = FastMath.PI;
        int numChildren = 20;

        CompoundCollisionShape result = CompoundTestShapes
                .makePipe(innerRadius, thickness, length, arc, numChildren);

        return result;
    }

    /**
     * Generate a centered HullCollisionShape, using the origin plus 4-to-19
     * pseudo-random vertices.
     *
     * @return a new shape
     */
    public HullCollisionShape nextHull() {
        int numVertices = nextInt(5, 20);

        FloatBuffer buffer = BufferUtils
                .createFloatBuffer(MyVector3f.numAxes * numVertices);
        buffer.put(0f).put(0f).put(0f);
        Vector3f tmpLocation = new Vector3f();
        for (int vertexI = 1; vertexI < numVertices; ++vertexI) {
            nextUnitVector3f(tmpLocation);
            tmpLocation.multLocal(1.5f);
            buffer.put(tmpLocation.x).put(tmpLocation.y).put(tmpLocation.z);
        }

        // Use arithmetic mean to center the vertices.
        int start = 0;
        int end = buffer.limit();
        Vector3f offset = MyBuffer.mean(buffer, start, end, null);
        offset.negateLocal();
        MyBuffer.translate(buffer, start, end, offset);

        HullCollisionShape result = new HullCollisionShape(buffer);

        return result;
    }

    /**
     * Generate an I-Beam shape.
     *
     * @return a new compound shape (not null)
     */
    public CompoundCollisionShape nextIBeam() {
        float length = nextFloat(1f, 10f);
        float flangeWidth = nextFloat(1f, 2f);
        float beamHeight = nextFloat(1f, 2f);
        float thickness = nextFloat(0.1f, 0.3f);
        CompoundCollisionShape result = CompoundTestShapes
                .makeIBeam(length, flangeWidth, beamHeight, thickness);

        return result;
    }

    /**
     * Generate a shape for a lidless box.
     *
     * @return a new compound shape (not null)
     */
    public CompoundCollisionShape nextLidlessBox() {
        float iHeight = nextFloat(2f, 4f);
        float iWidth = nextFloat(2f, 4f);
        float iDepth = nextFloat(1f, 2f);
        float wallThickness = nextFloat(0.1f, 0.3f);
        CompoundCollisionShape result = CompoundTestShapes
                .makeLidlessBox(iHeight, iWidth, iDepth, wallThickness);

        return result;
    }

    /**
     * Generate a MultiSphere shape with 1-4 spheres.
     *
     * @return a new shape (not null)
     */
    public MultiSphere nextMultiSphere() {
        int numSpheres = nextInt(1, 4);
        if (numSpheres == 4) {
            MultiSphere result = nextFourSphere();
            return result;
        }

        List<Vector3f> centers = new ArrayList<>(numSpheres);
        List<Float> radii = new ArrayList<>(numSpheres);

        // The first sphere is always centered.
        centers.add(Vector3f.ZERO);
        float mainRadius = nextFloat(0.8f, 1.4f);
        radii.add(mainRadius);

        for (int sphereIndex = 1; sphereIndex < numSpheres; ++sphereIndex) {
            // Add a smaller sphere, offset from the main one.
            Vector3f offset = nextUnitVector3f(null);
            offset.multLocal(mainRadius);
            centers.add(offset);

            float radius = mainRadius * nextFloat(0.2f, 1f);
            radii.add(radius);
        }

        MultiSphere result = new MultiSphere(centers, radii);

        if (numSpheres == 1) {
            // Scale the sphere to make an ellipsoid.
            float xScale = nextFloat(1f, 2f);
            float yScale = nextFloat(0.6f, 1.6f);
            float zScale = nextFloat(0.4f, 1.4f);
            result.setScale(new Vector3f(xScale, yScale, zScale));
        }

        return result;
    }

    /**
     * Generate a Platonic solid.
     *
     * @return a new shape (not null)
     */
    public CollisionShape nextPlatonic() {
        Mesh mesh;
        float radius;
        boolean noNormals = false;

        CollisionShape result;
        int solidType = nextInt(0, 4);
        switch (solidType) {
            case 0: // regular tetrahedron
                radius = 1.55f * nextFloat(0.5f, 1.5f);
                float he = radius / root3;
                Vector3f v0 = new Vector3f(-he, +he, +he);
                Vector3f v1 = new Vector3f(+he, -he, +he);
                Vector3f v2 = new Vector3f(+he, +he, -he);
                Vector3f v3 = new Vector3f(-he, -he, -he);
                result = new SimplexCollisionShape(v0, v1, v2, v3);
                break;

            case 1: // cube or regular hexahedron
                radius = 1.4f * nextFloat(0.5f, 1.5f);
                float halfExtent = radius / root3;
                result = new BoxCollisionShape(halfExtent);
                break;

            case 2: // regular octahedron
                radius = 1.4f * nextFloat(0.5f, 1.5f);
                mesh = new Octahedron(radius, noNormals);
                result = new HullCollisionShape(mesh);
                break;

            case 3: // regular dodecahedron
                radius = 1.1f * nextFloat(0.5f, 1.5f);
                mesh = new Dodecahedron(radius, Mode.Triangles);
                result = new HullCollisionShape(mesh);
                break;

            case 4: // regular icosahedron
                radius = 1.13f * nextFloat(0.5f, 1.5f);
                mesh = new Icosahedron(radius, noNormals);
                result = new HullCollisionShape(mesh);
                break;

            default:
                throw new RuntimeException("solidType = " + solidType);
        }

        return result;
    }

    /**
     * Generate a prism.
     *
     * @return a new shape (not null)
     */
    public HullCollisionShape nextPrism() {
        int numSides = nextInt(3, 9);
        float radius = nextFloat(0.6f, 2f);
        float height = nextFloat(0.6f, 1.6f);
        boolean noNormals = false;
        Mesh mesh = new Prism(numSides, radius, height, noNormals);
        HullCollisionShape result = new HullCollisionShape(mesh);

        return result;
    }

    /**
     * Generate a pyramid.
     *
     * @return a new shape (not null)
     */
    public HullCollisionShape nextPyramid() {
        int numSides = nextInt(3, 9);
        float baseRadius = nextFloat(0.8f, 1.8f);
        float yHeight = nextFloat(1f, 2.5f);
        boolean generatePyramid = true;
        Mesh mesh = new Cone(numSides, baseRadius, yHeight, generatePyramid);
        HullCollisionShape result = new HullCollisionShape(mesh);

        return result;
    }

    /**
     * Generate an instance of the named shape.
     *
     * @param shapeName the type of shape to generate (not null, not empty)
     * @return a new shape (not null)
     */
    public CollisionShape nextShape(String shapeName) {
        Validate.nonEmpty(shapeName, "shape name");

        CollisionShape result;
        switch (shapeName) {
            case "box":
                result = nextBox();
                break;

            case "capsule":
                result = nextCapsule();
                break;

            case "cone":
                result = nextCone();
                break;

            case "cylinder":
                result = nextCylinder();
                break;

            case "dome":
                result = nextDome();
                break;

            case "football":
                result = nextFootball();
                break;

            case "frame":
                result = nextFrame();
                break;

            case "halfPipe":
                result = nextHalfPipe();
                break;

            case "hull":
                result = nextHull();
                break;

            case "iBeam":
                result = nextIBeam();
                break;

            case "lidlessBox":
                result = nextLidlessBox();
                break;

            case "multiSphere":
                result = nextMultiSphere();
                break;

            case "platonic":
                result = nextPlatonic();
                break;

            case "prism":
                result = nextPrism();
                break;

            case "pyramid":
                result = nextPyramid();
                break;

            case "snowman":
                result = nextSnowman();
                break;

            case "sphere":
                result = nextSphere();
                break;

            case "star":
                result = nextStar();
                break;

            case "tetrahedron":
                result = nextTetrahedron();
                break;

            case "torus":
                result = nextTorus();
                break;

            case "triangularFrame":
                result = nextTriangularFrame();
                break;

            case "trident":
                result = nextTrident();
                break;

            case "washer":
                result = nextWasher();
                break;

            default:
                String message = "shapeName = " + MyString.quote(shapeName);
                throw new IllegalArgumentException(message);
        }

        return result;
    }

    /**
     * Generate a 3-ball snowman shape with the head on the +Y axis.
     *
     * @return a new compound shape (not null)
     */
    public CompoundCollisionShape nextSnowman() {
        float baseRadius = nextFloat(0.7f, 1.5f);
        CompoundCollisionShape result
                = CompoundTestShapes.makeSnowman(baseRadius);

        return result;
    }

    /**
     * Generate a sphere shape.
     *
     * @return a new shape (not null)
     */
    public SphereCollisionShape nextSphere() {
        float radius = nextFloat(0.5f, 1.5f);
        SphereCollisionShape result = new SphereCollisionShape(radius);

        return result;
    }

    /**
     * Generate a star-shaped compound shape.
     *
     * @return a new shape (not null)
     */
    public CompoundCollisionShape nextStar() {
        float centerY = nextFloat(0.3f, 0.6f);
        float outerRadius = nextFloat(1f, 2.5f);
        int numPoints = nextInt(4, 9);
        float radiusRatio = nextFloat(0.2f, 0.7f);
        int numTriangles = 4 + 2 * nextInt(0, 1);
        CompoundCollisionShape result = CompoundTestShapes.makeStar(
                numPoints, outerRadius, centerY, radiusRatio, numTriangles);

        return result;
    }

    /**
     * Generate a tetrahedral SimplexCollisionShape.
     *
     * @return a new shape (not null)
     */
    public SimplexCollisionShape nextTetrahedron() {
        float r1 = nextFloat(0.4f, 1.6f);
        float r2 = nextFloat(0.4f, 1.6f);
        float r3 = nextFloat(0.4f, 1.6f);
        float r4 = nextFloat(0.4f, 1.6f);

        Vector3f p1 = new Vector3f(r1, r1, r1);
        Vector3f p2 = new Vector3f(r2, -r2, -r2);
        Vector3f p3 = new Vector3f(-r3, -r3, r3);
        Vector3f p4 = new Vector3f(-r4, r4, -r4);
        SimplexCollisionShape result
                = new SimplexCollisionShape(p1, p2, p3, p4);

        return result;
    }

    /**
     * Approximate a torus or donut, open on the Z axis.
     *
     * @return a new shape
     */
    public CompoundCollisionShape nextTorus() {
        float majorRadius = nextFloat(1f, 1.5f);
        float minorRadius = nextFloat(0.2f, 0.6f);
        CompoundCollisionShape result
                = CompoundTestShapes.makeTorus(majorRadius, minorRadius);

        return result;
    }

    /**
     * Generate a triangular frame with identical sides, open on the Z axis.
     *
     * @return a new compound shape (not null)
     */
    public CompoundCollisionShape nextTriangularFrame() {
        float internalLength = nextFloat(2f, 6f);
        float innerR = internalLength / root3;
        float depth = nextFloat(0.2f, 1f);
        float thickness = internalLength * nextFloat(0.1f, 0.2f);
        float arc = FastMath.TWO_PI;
        int numSegments = 3;
        CompoundCollisionShape result = CompoundTestShapes
                .makePipe(innerR, thickness, depth, arc, numSegments);

        return result;
    }

    /**
     * Generate a trident shape.
     *
     * @return a new compound shape (not null)
     */
    public CompoundCollisionShape nextTrident() {
        float shaftLength = nextFloat(4f, 12f);
        float shaftRadius = nextFloat(0.1f, 0.2f);
        CompoundCollisionShape result
                = CompoundTestShapes.makeTrident(shaftLength, shaftRadius);

        return result;
    }

    /**
     * Approximate a flat washer (or flat ring), open on the Z axis.
     *
     * @return a new compound shape (not null)
     */
    public CompoundCollisionShape nextWasher() {
        float innerRadius = nextFloat(0.6f, 1.5f);
        float outerRadius = innerRadius + nextFloat(0.8f, 1.5f);
        float zThickness = nextFloat(0.2f, 0.4f);
        float arc = FastMath.TWO_PI;
        int numChildren = 24;

        CompoundCollisionShape result = CompoundTestShapes.makePipe(innerRadius,
                outerRadius - innerRadius, zThickness, arc, numChildren);

        return result;
    }
}
