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
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.Convex2dShape;
import com.jme3.bullet.collision.shapes.HeightfieldCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.MultiSphere;
import com.jme3.bullet.collision.shapes.SimplexCollisionShape;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;
import jme3utilities.mesh.DomeMesh;
import jme3utilities.mesh.RoundedRectangle;
import jme3utilities.minie.test.terrain.MinieTestTerrains;

/**
 * Utility class to generate collision shapes for use in MinieExamples.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class MinieTestShapes {
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
    // *************************************************************************
    // new methods exposed

    /**
     * Add some shapes to the specified library.
     *
     * @param namedShapes where to add shapes (not null, modified)
     */
    public static void addShapes(Map<String, CollisionShape> namedShapes) {
        {
            float barLength = 4.8f; // TODO randomize
            float barRadius = 0.2f;
            float plateRadius = 1f;
            CollisionShape barbell = CompoundTestShapes
                    .makeBarbell(barLength, barRadius, plateRadius);
            namedShapes.put("barbell", barbell);
        }

        CollisionShape bedOfNails = makeBedOfNails();
        namedShapes.put("bedOfNails", bedOfNails);

        {
            float innerRadius = 3f; // TODO randomize
            float thickness = 0.3f;
            CollisionShape bowl
                    = CompoundTestShapes.makeBowl(innerRadius, thickness);
            namedShapes.put("bowl", bowl);
        }

        {
            float backLength = 3f; // TODO randomize
            float legLength = 2f;
            float legOffset = 1f;
            float legRadius = 0.2f;
            CollisionShape chair = CompoundTestShapes
                    .makeChair(backLength, legLength, legOffset, legRadius);
            namedShapes.put("chair", chair);
        }

        {
            int numSides = 4;
            float radius = 20f;
            float depth = 12f;
            CollisionShape corner
                    = CompoundTestShapes.makeCorner(numSides, radius, depth);
            namedShapes.put("corner", corner);
        }

        CollisionShape dimples = makeDimples();
        namedShapes.put("dimples", dimples);

        {
            float stemLength = 2.5f; // TODO randomize
            float stemRadius = 0.25f;
            float ballRadius = 0.4f;
            CollisionShape knucklebone = CompoundTestShapes.makeKnucklebone(
                    stemLength, stemRadius, ballRadius);
            namedShapes.put("knucklebone", knucklebone);
        }

        {
            float rungLength = 2f; // TODO randomize
            float rungSpacing = 2f;
            float rungRadius = 0.2f;
            CollisionShape ladder = CompoundTestShapes.makeLadder(
                    rungLength, rungSpacing, rungRadius);
            namedShapes.put("ladder", ladder);
        }

        {
            float ihHeight = 1f;
            float ihWidth = 0.5f;
            float radius = 0.25f;
            CollisionShape link
                    = CompoundTestShapes.makeLink(ihHeight, ihWidth, radius);
            namedShapes.put("link", link);
        }

        CollisionShape roundedRectangle = makeRoundedRectangle();
        namedShapes.put("roundedRectangle", roundedRectangle);

        CollisionShape sieve = CompoundTestShapes.makeSieve();
        namedShapes.put("sieve", sieve);

        CollisionShape smooth = makeSmoothHeightfield();
        namedShapes.put("smooth", smooth);

        {
            float topRadius = 3f; // TODO randomize
            float pedestalRadius = 0.3f;
            CollisionShape table
                    = CompoundTestShapes.makeTable(topRadius, pedestalRadius);
            namedShapes.put("table", table);
        }

        {
            float headRadius = 2f; // TODO randomize
            float spikeRadius = 0.2f;
            CollisionShape thumbTack
                    = CompoundTestShapes.makeThumbTack(headRadius, spikeRadius);
            namedShapes.put("thumbTack", thumbTack);
        }

        {
            float bodyRadius = 1.5f; // TODO randomize
            float handleRadius = 0.3f;
            CollisionShape top
                    = CompoundTestShapes.makeTop(bodyRadius, handleRadius);
            namedShapes.put("top", top);
        }

        CollisionShape tray = CompoundTestShapes.makeTray();
        namedShapes.put("tray", tray);

        CollisionShape triangle = makeTriangle();
        namedShapes.put("triangle", triangle);
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
        HeightfieldCollisionShape result = new HeightfieldCollisionShape(
                n, n, array, scale, upAxis, flipQuadEdges,
                flipTriangleWinding, useDiamond, useZigzag);

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
     * @param verticalAngle the central angle from the top to the rim (in
     * radians, &lt;Pi, &gt;0, Pi/2 &rarr; hemisphere)
     * @return a new hull shape
     */
    public static HullCollisionShape makeDome(
            float radius, float verticalAngle) {
        Validate.positive(radius, "radius");
        Validate.inRange(verticalAngle, "vertical angle", 0f, FastMath.PI);

        int rimSamples = 20;
        int quadrantSamples = 10;
        DomeMesh mesh = new DomeMesh(rimSamples, quadrantSamples);
        mesh.setVerticalAngle(verticalAngle);
        FloatBuffer buffer = mesh.getFloatBuffer(VertexBuffer.Type.Position);

        // Scale mesh positions to the desired radius.
        int start = 0;
        int end = buffer.limit();
        Vector3f scale = new Vector3f(radius, radius, radius);
        MyBuffer.scale(buffer, start, end, scale);

        // Use max-min to center the vertices.
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
     * @param midRadius the radius of the Y-Z cross section at X=0 (in unscaled
     * shape units, &ge;0)
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
     * Generate a large rounded rectangle.
     *
     * @return a new convex 2-D shape (not null)
     */
    public static Convex2dShape makeRoundedRectangle() {
        float halfExtent = 30f;

        float x2 = halfExtent;
        float y2 = halfExtent / MyMath.phi;
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
     * Generate a large isosceles triangle.
     *
     * @return a new simplex shape (not null)
     */
    public static SimplexCollisionShape makeTriangle() {
        float radius = 50f;

        float x = radius * MyMath.rootHalf;
        Vector3f p1 = new Vector3f(x, 0f, x);
        Vector3f p2 = new Vector3f(x, 0f, -x);
        Vector3f p3 = new Vector3f(-x, 0f, 0f);
        SimplexCollisionShape result = new SimplexCollisionShape(p1, p2, p3);

        return result;
    }
}
