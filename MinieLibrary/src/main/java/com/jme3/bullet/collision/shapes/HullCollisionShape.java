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
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.FastMath;
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
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.RectangularSolid;
import vhacd.VHACDHull;

/**
 * A convex-hull CollisionShape based on Bullet's btConvexHullShape. For a 2-D
 * convex hull, use Convex2dShape.
 */
public class HullCollisionShape extends CollisionShape {
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
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public HullCollisionShape() {
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
        points = new float[numAxes * numLocations];
        int j = 0;
        for (Vector3f location : locations) {
            points[j + PhysicsSpace.AXIS_X] = location.x;
            points[j + PhysicsSpace.AXIS_Y] = location.y;
            points[j + PhysicsSpace.AXIS_Z] = location.z;
            j += numAxes;
        }

        createShape();
    }

    /**
     * Instantiate a shape based on an array containing coordinates. For best
     * performance and stability, the convex hull should have no more than 100
     * vertices. TODO use float...
     *
     * @param points an array of coordinates on which to base the shape (not
     * null, not empty, length a multiple of 3, unaffected)
     */
    public HullCollisionShape(float[] points) {
        Validate.nonEmpty(points, "points");
        int numFloats = points.length;
        assert (numFloats % numAxes == 0) : numFloats;

        this.points = points.clone();
        createShape();
    }

    /**
     * Instantiate a shape based on a VHACDHull. For best performance and
     * stability, the convex hull should have no more than 100 vertices.
     *
     * @param vhacdHull (not null, not empty, length a multiple of 3,
     * unaffected)
     */
    public HullCollisionShape(VHACDHull vhacdHull) {
        Validate.nonNull(vhacdHull, "points");
        float positions[] = vhacdHull.positions;
        int numFloats = positions.length;
        assert (numFloats % numAxes == 0) : numFloats;

        points = positions.clone();
        createShape();
    }

    /**
     * Instantiate a shape based on a flipped buffer containing coordinates. For
     * best performance and stability, the convex hull should have no more than
     * 100 vertices.
     *
     * @param flippedBuffer the coordinates on which to base the shape (not
     * null, not empty, limit a multiple of 3, unaffected)
     */
    public HullCollisionShape(FloatBuffer flippedBuffer) {
        Validate.nonNull(flippedBuffer, "flipped buffer");
        int numFloats = flippedBuffer.limit();
        assert numFloats > 0 : numFloats;
        assert numFloats % numAxes == 0 : numFloats;

        points = new float[numFloats];
        for (int i = 0; i < numFloats; ++i) {
            points[i] = flippedBuffer.get(i);
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
        points = getPoints(meshes);
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
        /*
         * Enumerate the local coordinates of the 8 corners of the box.
         */
        Collection<Vector3f> cornerLocations = new ArrayList<>(8);
        cornerLocations.add(new Vector3f(maxima.x, maxima.y, maxima.z));
        cornerLocations.add(new Vector3f(maxima.x, maxima.y, minima.z));
        cornerLocations.add(new Vector3f(maxima.x, minima.y, maxima.z));
        cornerLocations.add(new Vector3f(maxima.x, minima.y, minima.z));
        cornerLocations.add(new Vector3f(minima.x, maxima.y, maxima.z));
        cornerLocations.add(new Vector3f(minima.x, maxima.y, minima.z));
        cornerLocations.add(new Vector3f(minima.x, minima.y, maxima.z));
        cornerLocations.add(new Vector3f(minima.x, minima.y, minima.z));
        /*
         * Transform corner locations to shape coordinates.
         */
        int numFloats = numAxes * cornerLocations.size();
        points = new float[numFloats];
        int floatIndex = 0;
        Vector3f tempVector = new Vector3f();
        for (Vector3f location : cornerLocations) {
            rectangularSolid.localToWorld(location, tempVector);
            points[floatIndex + PhysicsSpace.AXIS_X] = tempVector.x;
            points[floatIndex + PhysicsSpace.AXIS_Y] = tempVector.y;
            points[floatIndex + PhysicsSpace.AXIS_Z] = tempVector.z;
            floatIndex += numAxes;
        }

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
        long shapeId = getObjectId();
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
        long shapeId = getObjectId();
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
     * Estimate the scaled volume of the hull, based on its debug mesh.
     *
     * @return the volume (in physics-space units cubed, &ge;0)
     */
    public float scaledVolume() {
        int meshResolution = DebugShapeFactory.lowResolution;
        float volume = DebugShapeFactory.volumeConvex(this, meshResolution);
        assert volume >= 0f : volume;
        return volume;
    }
    // *************************************************************************
    // CollisionShape methods

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
        directBuffer = null; // directBuffer not cloned
        points = cloner.clone(points);
        createShape();
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public HullCollisionShape jmeClone() {
        try {
            HullCollisionShape clone = (HullCollisionShape) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
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
        long shapeId = getObjectId();
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
            points = getPoints(mesh);
        } else {
            points = capsule.readFloatArray(tagPoints, new float[0]);
        }
        createShape();
    }

    /**
     * Recalculate this shape's bounding box if necessary.
     */
    @Override
    protected void recalculateAabb() {
        long shapeId = getObjectId();
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
    // private methods

    /**
     * Instantiate the configured shape in Bullet.
     */
    private void createShape() {
        assert directBuffer == null : directBuffer;

        int numFloats = points.length;
        assert numFloats != 0;
        assert (numFloats % numAxes == 0) : numFloats;
        int numVertices = numFloats / numAxes;

        directBuffer = BufferUtils.createFloatBuffer(numFloats);
        for (float f : points) {
            if (!Float.isFinite(f)) {
                throw new IllegalArgumentException("illegal coordinate: " + f);
            }
            directBuffer.put(f);
        }

        long shapeId = createShapeF(directBuffer, numVertices);
        setNativeId(shapeId);

        setScale(scale);
        setMargin(margin);
    }

    /**
     * Copy the vertex positions from JME mesh(es).
     *
     * @param meshes the mesh(es) to read (not null)
     * @return a new array (not null, length a multiple of 3)
     */
    private float[] getPoints(Mesh... meshes) {
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
    // native methods

    native private int countHullVertices(long shapeId);

    native private long createShapeF(FloatBuffer vertices, int numVertices);

    native private void getHullVerticesF(long shapeId, FloatBuffer vertices);

    native private void recalcAabb(long shapeId);
}
