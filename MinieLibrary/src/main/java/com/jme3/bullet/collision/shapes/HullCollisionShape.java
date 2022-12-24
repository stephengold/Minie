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
package com.jme3.bullet.collision.shapes;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.infos.ChildCollisionShape;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.FastMath;
import com.jme3.math.Plane;
import com.jme3.math.Triangle;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer.Type;
import com.jme3.util.BufferUtils;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Collection;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.RectangularSolid;
import jme3utilities.math.VectorSet;
import jme3utilities.math.VectorSetUsingBuffer;
import vhacd.VHACDHull;
import vhacd4.Vhacd4Hull;

/**
 * A convex-hull collision shape based on Bullet's {@code btConvexHullShape}.
 * For a 2-D convex hull, use Convex2dShape.
 */
public class HullCollisionShape extends ConvexShape {
    // *************************************************************************
    // constants and loggers

    /**
     * number of axes in a vector
     */
    final private static int numAxes = 3;
    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(HullCollisionShape.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagHullMesh = "hullMesh";
    final private static String tagPoints = "points";
    // *************************************************************************
    // fields

    /**
     * direct buffer for passing vertices to Bullet
     * <p>
     * A Java reference must persist after createShape() completes, or else the
     * buffer might get garbage collected.
     */
    private FloatBuffer directBuffer;
    /**
     * array of mesh coordinates (not null, not empty, length a multiple of 3)
     */
    private float[] points;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected HullCollisionShape() {
    }

    /**
     * Instantiate a shape based on the specified collection of locations. For
     * best performance and stability, the convex hull should have no more than
     * 100 vertices.
     *
     * @param locations a collection of location vectors on which to base the
     * shape (not null, not empty, unaffected)
     */
    public HullCollisionShape(Collection<Vector3f> locations) {
        Validate.nonEmpty(locations, "locations");

        int numLocations = locations.size();
        this.points = new float[numAxes * numLocations];
        int j = 0;
        for (Vector3f location : locations) {
            this.points[j + PhysicsSpace.AXIS_X] = location.x;
            this.points[j + PhysicsSpace.AXIS_Y] = location.y;
            this.points[j + PhysicsSpace.AXIS_Z] = location.z;
            j += numAxes;
        }

        createShape();
    }

    /**
     * Instantiate a shape based on an array containing coordinates. For best
     * performance and stability, the convex hull should have no more than 100
     * vertices.
     *
     * @param points an array of coordinates on which to base the shape (not
     * null, not empty, length a multiple of 3, unaffected)
     */
    public HullCollisionShape(float... points) {
        Validate.nonEmpty(points, "points");
        Validate.require(
                points.length % numAxes == 0, "length a multiple of 3");

        this.points = points.clone();
        createShape();
    }

    /**
     * Instantiate a shape based on a flipped buffer containing coordinates. For
     * best performance and stability, the convex hull should have no more than
     * 100 vertices.
     *
     * @param flippedBuffer the coordinates on which to base the shape (not
     * null, limit&gt;0, limit a multiple of 3, unaffected)
     */
    public HullCollisionShape(FloatBuffer flippedBuffer) {
        Validate.nonNull(flippedBuffer, "flipped buffer");
        int numFloats = flippedBuffer.limit();
        Validate.positive(numFloats, "limit");
        Validate.require(numFloats % numAxes == 0, "limit a multiple of 3");

        this.points = new float[numFloats];
        for (int i = 0; i < numFloats; ++i) {
            this.points[i] = flippedBuffer.get(i);
        }

        createShape();
    }

    /**
     * Instantiate a shape based on the specified JME mesh(es). For best
     * performance and stability, the convex hull should have no more than 100
     * vertices.
     *
     * @param meshes the mesh(es) on which to base the shape (all non-null, at
     * least one vertex, unaffected)
     */
    public HullCollisionShape(Mesh... meshes) {
        Validate.nonEmpty(meshes, "meshes");
        this.points = getPoints(meshes);
        Validate.require(points.length > 0, "at least one vertex");

        createShape();
    }

    /**
     * Instantiate an 8-vertex shape to match the specified rectangular solid.
     *
     * @param rectangularSolid the solid on which to base the shape (not null)
     */
    public HullCollisionShape(RectangularSolid rectangularSolid) {
        Vector3f maxima = rectangularSolid.maxima(null);
        Vector3f minima = rectangularSolid.minima(null);

        // Enumerate the local coordinates of the 8 corners of the box.
        Collection<Vector3f> cornerLocations = new ArrayList<>(8);
        cornerLocations.add(new Vector3f(maxima.x, maxima.y, maxima.z));
        cornerLocations.add(new Vector3f(maxima.x, maxima.y, minima.z));
        cornerLocations.add(new Vector3f(maxima.x, minima.y, maxima.z));
        cornerLocations.add(new Vector3f(maxima.x, minima.y, minima.z));
        cornerLocations.add(new Vector3f(minima.x, maxima.y, maxima.z));
        cornerLocations.add(new Vector3f(minima.x, maxima.y, minima.z));
        cornerLocations.add(new Vector3f(minima.x, minima.y, maxima.z));
        cornerLocations.add(new Vector3f(minima.x, minima.y, minima.z));

        // Transform corner locations to shape coordinates.
        int numFloats = numAxes * cornerLocations.size();
        this.points = new float[numFloats];
        int floatIndex = 0;
        Vector3f tempVector = new Vector3f();
        for (Vector3f location : cornerLocations) {
            rectangularSolid.localToWorld(location, tempVector);
            this.points[floatIndex + PhysicsSpace.AXIS_X] = tempVector.x;
            this.points[floatIndex + PhysicsSpace.AXIS_Y] = tempVector.y;
            this.points[floatIndex + PhysicsSpace.AXIS_Z] = tempVector.z;
            floatIndex += numAxes;
        }

        createShape();
    }

    /**
     * Instantiate a shape based on an array of locations. For best performance
     * and stability, the convex hull should have no more than 100 vertices.
     *
     * @param locations an array of location vectors (in shape coordinates, not
     * null, not empty, unaffected)
     */
    public HullCollisionShape(Vector3f... locations) {
        Validate.nonEmpty(locations, "points");

        int numFloats = numAxes * locations.length;
        this.points = new float[numFloats];
        int floatIndex = 0;
        for (Vector3f location : locations) {
            this.points[floatIndex + PhysicsSpace.AXIS_X] = location.x;
            this.points[floatIndex + PhysicsSpace.AXIS_Y] = location.y;
            this.points[floatIndex + PhysicsSpace.AXIS_Z] = location.z;
            floatIndex += numAxes;
        }

        createShape();
    }

    /**
     * Instantiate a shape based on a Vhacd4Hull. For best performance and
     * stability, the convex hull should have no more than 100 vertices.
     *
     * @param vhacd4Hull (not null, unaffected)
     */
    public HullCollisionShape(Vhacd4Hull vhacd4Hull) {
        Validate.nonNull(vhacd4Hull, "V-HACD hull");

        this.points = vhacd4Hull.clonePositions();
        createShape();
    }

    /**
     * Instantiate a shape based on a VHACDHull. For best performance and
     * stability, the convex hull should have no more than 100 vertices.
     *
     * @param vhacdHull (not null, unaffected)
     */
    public HullCollisionShape(VHACDHull vhacdHull) {
        Validate.nonNull(vhacdHull, "V-HACD hull");

        this.points = vhacdHull.clonePositions();
        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Calculate a quick upper bound for the unscaled volume of the hull, based
     * on its axis-aligned bounding box.
     *
     * @return the volume (in unscaled shape units cubed, &ge;0)
     */
    public float aabbVolume() {
        Vector3f maxima = new Vector3f(Float.NEGATIVE_INFINITY,
                Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY);
        Vector3f minima = new Vector3f(Float.POSITIVE_INFINITY,
                Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY);

        Vector3f location = new Vector3f();
        for (int floatI = 0; floatI < points.length; floatI += numAxes) {
            float x = points[floatI + PhysicsSpace.AXIS_X];
            float y = points[floatI + PhysicsSpace.AXIS_Y];
            float z = points[floatI + PhysicsSpace.AXIS_Z];
            location.set(x, y, z);
            MyVector3f.accumulateMinima(minima, location);
            MyVector3f.accumulateMaxima(maxima, location);
        }

        float dx = maxima.x - minima.x;
        float dy = maxima.y - minima.y;
        float dz = maxima.z - minima.z;
        float volume = dx * dy * dz;

        assert volume >= 0f : volume;
        assert Float.isFinite(volume) : volume;
        return volume;
    }

    /**
     * Copy the unscaled vertex locations of the optimized convex hull.
     *
     * @return a new array (not null)
     */
    public float[] copyHullVertices() {
        long shapeId = nativeId();
        int numHullVertices = countHullVertices();
        FloatBuffer buffer
                = BufferUtils.createFloatBuffer(numHullVertices * numAxes);
        getHullVerticesF(shapeId, buffer);

        float[] result = new float[numHullVertices * numAxes];
        for (int floatI = 0; floatI < numHullVertices * numAxes; ++floatI) {
            result[floatI] = buffer.get(floatI);
        }

        return result;
    }

    /**
     * Count the number of vertices in the optimized convex hull.
     *
     * @return the count (&ge;0)
     */
    public int countHullVertices() {
        long shapeId = nativeId();
        int result = countHullVertices(shapeId);

        return result;
    }

    /**
     * Count the vertices used to generate the hull.
     *
     * @return the count (&gt;0)
     */
    public int countMeshVertices() {
        int length = points.length;
        assert (length % numAxes == 0) : length;
        int result = length / numAxes;

        assert result > 0 : result;
        return result;
    }

    /**
     * Calculate the unscaled half extents of the hull.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the unscaled half extent for each local axis (either storeResult
     * or a new vector, not null, no negative component)
     */
    public Vector3f getHalfExtents(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        result.zero();
        for (int i = 0; i < points.length; i += numAxes) {
            float x = FastMath.abs(points[i + PhysicsSpace.AXIS_X]);
            if (x > result.x) {
                result.x = x;
            }
            float y = FastMath.abs(points[i + PhysicsSpace.AXIS_Y]);
            if (y > result.y) {
                result.y = y;
            }
            float z = FastMath.abs(points[i + PhysicsSpace.AXIS_Z]);
            if (z > result.z) {
                result.z = z;
            }
        }

        assert MyVector3f.isAllNonNegative(result) : result;
        return result;
    }

    /**
     * Attempt to divide this shape into 2 child shapes.
     *
     * @param splittingTriangle to define the splitting plane (in shape
     * coordinates, not null, unaffected)
     * @return a pair of hull-based children, the first child generated by the
     * plane's minus side and the 2nd child generated by its plus side; either
     * child may be null, indicating an empty shape
     */
    public ChildCollisionShape[] split(Triangle splittingTriangle) {
        Validate.nonNull(splittingTriangle, "splitting triangle");

        int numVertices = countHullVertices();
        int numFloats = numAxes * numVertices;
        FloatBuffer originalHull = BufferUtils.createFloatBuffer(numFloats);
        long shapeId = nativeId();
        getHullVerticesF(shapeId, originalHull);

        Vector3f normal = splittingTriangle.getNormal(); // alias
        Plane splittingPlane = new Plane(normal, splittingTriangle.get3());
        /*
         * Organize the hull vertices into 2 sets based on
         * which side of the splitting plane they are on.
         */
        VectorSet minusSet = new VectorSetUsingBuffer(numVertices, true);
        VectorSet plusSet = new VectorSetUsingBuffer(numVertices, true);
        Vector3f tmpVertex = new Vector3f();
        for (int vertexI = 0; vertexI < numVertices; ++vertexI) {
            int startPosition = numAxes * vertexI;
            MyBuffer.get(originalHull, startPosition, tmpVertex);
            float pseudoDistance = splittingPlane.pseudoDistance(tmpVertex);
            if (pseudoDistance <= 0f) {
                minusSet.add(tmpVertex);
            }
            if (pseudoDistance >= 0f) {
                plusSet.add(tmpVertex);
            }
            // Note: vertices that lie in the plane will appear in both sets.
        }

        ChildCollisionShape[] result = new ChildCollisionShape[2];
        int numMinus = minusSet.numVectors();
        int numPlus = plusSet.numVectors();
        if (numMinus == 0 || numPlus == 0) {
            // Degenerate case:  all vertices lie to one side of the plane.
            ChildCollisionShape child
                    = new ChildCollisionShape(new Vector3f(), this);
            if (numMinus > 0) {
                result[0] = child;
            } else if (numPlus > 0) {
                result[1] = child;
            }
            return result;
        }

        // Copy all minus-side vertices to a new set.
        FloatBuffer minusBuffer = minusSet.toBuffer();
        VectorSet newMinusSet = new VectorSetUsingBuffer(numVertices, true);
        for (int jNegative = 0; jNegative < numMinus; ++jNegative) {
            MyBuffer.get(minusBuffer, numAxes * jNegative, tmpVertex);
            newMinusSet.add(tmpVertex);
        }
        /*
         * Copy all plus-side vertices to a new set.
         * Also: interpolate each of the original plus-side vertices
         * with each of the original minus-side vertices
         * and add the interpolated locations to both of the new sets.
         */
        FloatBuffer plusBuffer = plusSet.toBuffer();
        VectorSet newPlusSet = new VectorSetUsingBuffer(numVertices, true);
        Vector3f tmp2 = new Vector3f();
        for (int plusI = 0; plusI < numPlus; ++plusI) {
            MyBuffer.get(plusBuffer, numAxes * plusI, tmpVertex);
            newPlusSet.add(tmpVertex);
            float pd = splittingPlane.pseudoDistance(tmpVertex);

            for (int minusI = 0; minusI < numMinus; ++minusI) {
                MyBuffer.get(minusBuffer, numAxes * minusI, tmp2);
                float md = splittingPlane.pseudoDistance(tmp2);
                float denominator = pd - md;
                if (denominator != 0f) {
                    float t = -md / denominator;
                    MyVector3f.lerp(t, tmp2, tmpVertex, tmp2);
                    newMinusSet.add(tmp2);
                    newPlusSet.add(tmp2);
                }
            }
        }
        /*
         * Translate minus-side vertices so their AABB is centered at (0,0,0)
         * and use them to form a new hull shape.
         */
        Vector3f max = tmpVertex; // alias
        Vector3f min = tmp2; // alias
        Vector3f offset = tmpVertex; // alias
        newMinusSet.maxMin(max, min);
        Vector3f minusCenter = MyVector3f.midpoint(max, min, null);
        offset.set(minusCenter).negateLocal();
        FloatBuffer flippedBuffer = newMinusSet.toBuffer();
        MyBuffer.translate(flippedBuffer, 0, flippedBuffer.limit(), offset);
        HullCollisionShape minusShape = new HullCollisionShape(flippedBuffer);
        minusShape.setScale(scale);
        result[0] = new ChildCollisionShape(minusCenter, minusShape);
        /*
         * Translate plus-side vertices so their AABB is centered at (0,0,0)
         * and use them to form a new hull shape.
         */
        newPlusSet.maxMin(max, min);
        Vector3f plusCenter = MyVector3f.midpoint(max, min, null);
        offset.set(plusCenter).negateLocal();
        flippedBuffer = newPlusSet.toBuffer();
        MyBuffer.translate(flippedBuffer, 0, flippedBuffer.limit(), offset);
        HullCollisionShape plusShape = new HullCollisionShape(flippedBuffer);
        plusShape.setScale(scale);
        result[1] = new ChildCollisionShape(plusCenter, plusShape);

        return result;
    }
    // *************************************************************************
    // ConvexShape methods

    /**
     * Test whether this shape can be split by an arbitrary plane.
     *
     * @return true if splittable, false otherwise
     */
    @Override
    public boolean canSplit() {
        return true;
    }

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned shape into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this shape (not null)
     * @param original the instance from which this shape was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);
        this.directBuffer = null; // directBuffer is never cloned.
        this.points = cloner.clone(points);
        createShape();
    }

    /**
     * Calculate how far this shape extends from its center, including margin.
     *
     * @return a distance (in physics-space units, &ge;0)
     */
    @Override
    public float maxRadius() {
        int numHullVertices = countHullVertices();
        FloatBuffer buffer
                = BufferUtils.createFloatBuffer(numHullVertices * numAxes);
        long shapeId = nativeId();
        getHullVerticesF(shapeId, buffer);
        double maxSquaredDistance = 0.0;

        for (int vertexI = 0; vertexI < numHullVertices; ++vertexI) {
            int startOffset = numAxes * vertexI;
            float x = scale.x * buffer.get(startOffset + PhysicsSpace.AXIS_X);
            float y = scale.y * buffer.get(startOffset + PhysicsSpace.AXIS_Y);
            float z = scale.z * buffer.get(startOffset + PhysicsSpace.AXIS_Z);
            double lengthSquared = MyMath.sumOfSquares(x, y, z);
            if (lengthSquared > maxSquaredDistance) {
                maxSquaredDistance = lengthSquared;
            }
        }
        float result = margin + (float) Math.sqrt(maxSquaredDistance);

        return result;
    }

    /**
     * De-serialize this shape from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);
        InputCapsule capsule = importer.getCapsule(this);

        // for backwards compatibility
        Mesh mesh = (Mesh) capsule.readSavable(tagHullMesh, null);
        if (mesh != null) {
            this.points = getPoints(mesh);
        } else {
            this.points = capsule.readFloatArray(tagPoints, new float[0]);
        }
        createShape();
    }

    /**
     * Recalculate this shape's bounding box if necessary.
     */
    @Override
    protected void recalculateAabb() {
        long shapeId = nativeId();
        recalcAabb(shapeId);
    }

    /**
     * Serialize this shape to the specified exporter, for example when saving
     * to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        super.write(exporter);
        OutputCapsule capsule = exporter.getCapsule(this);

        float[] vertices = copyHullVertices();
        capsule.write(vertices, tagPoints, new float[0]);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured shape in Bullet.
     */
    private void createShape() {
        assert directBuffer == null : directBuffer;

        int numFloats = points.length;
        assert numFloats != 0;
        assert (numFloats % numAxes == 0) : numFloats;
        int numVertices = numFloats / numAxes;

        this.directBuffer = BufferUtils.createFloatBuffer(numFloats);
        for (float f : points) {
            if (!Float.isFinite(f)) {
                throw new IllegalArgumentException("illegal coordinate: " + f);
            }
            directBuffer.put(f);
        }

        long shapeId = createShapeF(directBuffer, numVertices);
        setNativeId(shapeId);

        setContactFilterEnabled(enableContactFilter);
        setScale(scale);
        setMargin(margin);
    }

    /**
     * Copy the vertex positions from JME mesh(es).
     *
     * @param meshes the mesh(es) to read (not null)
     * @return a new array (not null, length a multiple of 3)
     */
    private static float[] getPoints(Mesh... meshes) {
        int numVectors = 0;
        for (Mesh mesh : meshes) {
            numVectors += mesh.getVertexCount();
        }
        int numFloats = numAxes * numVectors;
        float[] pointsArray = new float[numFloats];

        int arrayIndex = 0;
        for (Mesh mesh : meshes) {
            FloatBuffer buffer = mesh.getFloatBuffer(Type.Position);
            int bufNumFloats = numAxes * mesh.getVertexCount();
            for (int bufPos = 0; bufPos < bufNumFloats; ++bufPos) {
                pointsArray[arrayIndex] = buffer.get(bufPos);
                ++arrayIndex;
            }
        }
        assert arrayIndex == numFloats : arrayIndex;

        return pointsArray;
    }
    // *************************************************************************
    // native private methods

    native private static int countHullVertices(long shapeId);

    native private static long
            createShapeF(FloatBuffer vertices, int numVertices);

    native private static void
            getHullVerticesF(long shapeId, FloatBuffer vertices);

    native private static void recalcAabb(long shapeId);
}
