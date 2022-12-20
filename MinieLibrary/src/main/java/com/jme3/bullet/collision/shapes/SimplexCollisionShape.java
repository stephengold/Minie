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
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.AbstractTriangle;
import com.jme3.math.FastMath;
import com.jme3.math.LineSegment;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.nio.FloatBuffer;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.MyVolume;

/**
 * A simple point, line-segment, triangle, or tetrahedron collision shape based
 * on Bullet's {@code btBU_Simplex1to4}. These shapes cannot be scaled.
 *
 * @author normenhansen
 */
public class SimplexCollisionShape extends ConvexShape {
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
            = Logger.getLogger(SimplexCollisionShape.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagSimplexPoint1 = "simplexPoint1";
    final private static String tagSimplexPoint2 = "simplexPoint2";
    final private static String tagSimplexPoint3 = "simplexPoint3";
    final private static String tagSimplexPoint4 = "simplexPoint4";
    // *************************************************************************
    // fields

    /**
     * vertex locations
     */
    private Vector3f[] locations;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected SimplexCollisionShape() {
    }

    /**
     * Instantiate a point shape based on the specified location.
     *
     * @param location the location of the point (in shape coordinates, not
     * null, unaffected)
     */
    public SimplexCollisionShape(Vector3f location) {
        this.locations = new Vector3f[1];
        locations[0] = location.clone();
        createShape();
    }

    /**
     * Instantiate a line-segment shape based on the specified endpoints.
     *
     * @param point1 the location of first endpoint (in shape coordinates, not
     * null, unaffected)
     * @param point2 the location of 2nd endpoint (in shape coordinates, not
     * null, unaffected)
     */
    public SimplexCollisionShape(Vector3f point1, Vector3f point2) {
        this.locations = new Vector3f[2];
        locations[0] = point1.clone();
        locations[1] = point2.clone();
        createShape();
    }

    /**
     * Instantiate a triangular shape based on the specified vertices.
     *
     * @param vertex1 the location of first vertex (in shape coordinates, not
     * null, unaffected)
     * @param vertex2 the location of 2nd vertex (in shape coordinates, not
     * null, unaffected)
     * @param vertex3 the location of 3rd vertex (in shape coordinates, not
     * null, unaffected)
     */
    public SimplexCollisionShape(Vector3f vertex1, Vector3f vertex2,
            Vector3f vertex3) {
        this.locations = new Vector3f[3];
        locations[0] = vertex1.clone();
        locations[1] = vertex2.clone();
        locations[2] = vertex3.clone();
        createShape();
    }

    /**
     * Instantiate a tetrahedral shape based on the specified vertices.
     *
     * @param vertex1 the location of first vertex (in shape coordinates, not
     * null, unaffected)
     * @param vertex2 the location of 2nd vertex (in shape coordinates, not
     * null, unaffected)
     * @param vertex3 the location of 3rd vertex (in shape coordinates, not
     * null, unaffected)
     * @param vertex4 the location of 4th vertex (in shape coordinates, not
     * null, unaffected)
     */
    public SimplexCollisionShape(Vector3f vertex1, Vector3f vertex2,
            Vector3f vertex3, Vector3f vertex4) {
        this.locations = new Vector3f[4];
        locations[0] = vertex1.clone();
        locations[1] = vertex2.clone();
        locations[2] = vertex3.clone();
        locations[3] = vertex4.clone();
        createShape();
    }

    /**
     * Instantiate a triangular shape based on the specified AbstractTriangle.
     *
     * @param triangle the triangle (not null, unaffected)
     */
    public SimplexCollisionShape(AbstractTriangle triangle) {
        Validate.nonNull(triangle, "triangle");

        this.locations = new Vector3f[3];
        locations[0] = triangle.get1().clone();
        locations[1] = triangle.get2().clone();
        locations[2] = triangle.get3().clone();
        createShape();
    }

    /**
     * Instantiate a simplex shape based on the specified FloatBuffer range.
     *
     * @param buffer the buffer that contains the vertex locations (not null,
     * unaffected)
     * @param startPosition the buffer position at which the vertex locations
     * start (&ge;0, &le;endPosition-3)
     * @param endPosition the buffer position at which the vertex locations end
     * (&ge;startPosition+3, &le;capacity)
     */
    public SimplexCollisionShape(FloatBuffer buffer, int startPosition,
            int endPosition) {
        Validate.nonNull(buffer, "buffer");
        Validate.inRange(startPosition, "start position", 0,
                endPosition - numAxes);
        Validate.inRange(endPosition, "end position", startPosition + numAxes,
                buffer.capacity());
        int numFloats = endPosition - startPosition;
        Validate.require(numFloats % numAxes == 0, "range a multiple of 3");
        int numVertices = numFloats / numAxes;
        assert numVertices >= 1 : numVertices;
        assert numVertices <= 4 : numVertices;

        this.locations = new Vector3f[numVertices];
        for (int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex) {
            locations[vertexIndex] = new Vector3f();
            MyBuffer.get(buffer, startPosition + vertexIndex * numAxes,
                    locations[vertexIndex]);
        }

        createShape();
    }

    /**
     * Instantiate a line-segment shape based on the specified LineSegment.
     *
     * @param segment the segment (not null, unaffected)
     */
    public SimplexCollisionShape(LineSegment segment) {
        Validate.nonNull(segment, "segment");

        this.locations = new Vector3f[2];
        locations[0] = segment.getNegativeEnd(null);
        locations[1] = segment.getPositiveEnd(null);

        createShape();
    }

    /**
     * Instantiate a simplex shape based on the specified array.
     *
     * @param vertices an array of vertex locations (not null, not empty,
     * unaffected)
     */
    public SimplexCollisionShape(Vector3f[] vertices) {
        Validate.nonEmpty(vertices, "vertices");
        int numVertices = vertices.length;
        assert numVertices <= 4 : numVertices;

        this.locations = new Vector3f[numVertices];
        for (int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex) {
            locations[vertexIndex] = vertices[vertexIndex].clone();
        }

        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the indexed vertex.
     *
     * @param index (&ge;0, &lt;4)
     * @param storeResult storage for the result (modified if not null)
     * @return the location of the vertex (either storeResult or a new instance)
     */
    public Vector3f copyVertex(int index, Vector3f storeResult) {
        int numVertices = locations.length;
        Validate.inRange(index, "index", 0, numVertices - 1);

        Vector3f result;
        if (storeResult == null) {
            result = locations[index].clone();
        } else {
            result = storeResult.set(locations[index]);
        }

        return result;
    }

    /**
     * Copy the unscaled vertex locations.
     *
     * @return a new array (not null)
     */
    public float[] copyVertices() {
        int numVertices = locations.length;
        float[] result = new float[numVertices * numAxes];
        for (int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex) {
            int floatIndex = vertexIndex * numAxes;
            Vector3f location = locations[vertexIndex];
            result[floatIndex + PhysicsSpace.AXIS_X] = location.x;
            result[floatIndex + PhysicsSpace.AXIS_Y] = location.y;
            result[floatIndex + PhysicsSpace.AXIS_Z] = location.z;
        }

        return result;
    }

    /**
     * Count the points used to generate the simplex.
     *
     * @return the count (&ge;1, &le;4)
     */
    public int countMeshVertices() {
        int result = locations.length;

        assert result >= 1 : result;
        assert result <= 4 : result;
        return result;
    }

    /**
     * Calculate the unscaled half extents of the simplex.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the unscaled half extent for each local axis (either storeResult
     * or a new vector, not null, no negative component)
     */
    public Vector3f getHalfExtents(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        result.zero();
        for (Vector3f location : locations) {
            float x = FastMath.abs(location.x);
            if (x > result.x) {
                result.x = x;
            }
            float y = FastMath.abs(location.y);
            if (y > result.y) {
                result.y = y;
            }
            float z = FastMath.abs(location.z);
            if (z > result.z) {
                result.z = z;
            }
        }

        assert MyVector3f.isAllNonNegative(result) : result;
        return result;
    }

    /**
     * Calculate the unscaled volume of the simplex.
     *
     * @return the volume (in shape units cubed, &ge;0)
     */
    public float unscaledVolume() {
        float volume = 0f;
        if (locations.length > 3) {
            volume = (float) MyVolume.tetrahedronVolume(locations[0],
                    locations[1], locations[2], locations[3]);
        }

        assert volume >= 0f : volume;
        return volume;
    }
    // *************************************************************************
    // ConvexShape methods

    /**
     * Test whether the specified scale factors can be applied to this shape.
     * For simplex shapes, scaling must be unity.
     *
     * @param scale the desired scale factor for each local axis (may be null,
     * unaffected)
     * @return true if applicable, otherwise false
     */
    @Override
    public boolean canScale(Vector3f scale) {
        boolean canScale = super.canScale(scale)
                && MyVector3f.isScaleIdentity(scale);

        return canScale;
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
        this.locations = cloner.clone(locations);
        createShape();
    }

    /**
     * Calculate how far the simplex extends from its center, including margin.
     *
     * @return a distance (in physics-space units, &ge;0)
     */
    @Override
    public float maxRadius() {
        double maxLengthSquared = 0.0;

        for (Vector3f location : locations) {
            double lengthSquared = MyVector3f.lengthSquared(location);
            if (lengthSquared > maxLengthSquared) {
                maxLengthSquared = lengthSquared;
            }
        }
        float result = margin + (float) Math.sqrt(maxLengthSquared);

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

        Vector3f v1 = (Vector3f) capsule.readSavable(tagSimplexPoint1, null);
        Vector3f v2 = (Vector3f) capsule.readSavable(tagSimplexPoint2, null);
        Vector3f v3 = (Vector3f) capsule.readSavable(tagSimplexPoint3, null);
        Vector3f v4 = (Vector3f) capsule.readSavable(tagSimplexPoint4, null);

        if (v2 == null) {
            this.locations = new Vector3f[]{v1};
        } else if (v3 == null) {
            this.locations = new Vector3f[]{v1, v2};
        } else if (v4 == null) {
            this.locations = new Vector3f[]{v1, v2, v3};
        } else {
            this.locations = new Vector3f[]{v1, v2, v3, v4};
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
     * Approximate this shape with a HullCollisionShape.
     *
     * @return a new shape
     */
    @Override
    public HullCollisionShape toHullShape() {
        assert MyVector3f.isScaleIdentity(scale) : scale;
        HullCollisionShape result = new HullCollisionShape(locations);
        result.setMargin(margin);

        return result;
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

        capsule.write(locations[0], tagSimplexPoint1, null);
        int numVertices = locations.length;
        if (numVertices > 1) {
            capsule.write(locations[1], tagSimplexPoint2, null);
        }
        if (numVertices > 2) {
            capsule.write(locations[2], tagSimplexPoint3, null);
        }
        if (numVertices > 3) {
            capsule.write(locations[3], tagSimplexPoint4, null);
        }
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured shape in Bullet.
     */
    private void createShape() {
        int numVertices = locations.length;
        long shapeId;
        switch (numVertices) {
            case 1:
                shapeId = createShape(locations[0]);
                break;
            case 2:
                shapeId = createShape(locations[0], locations[1]);
                break;
            case 3:
                shapeId = createShape(locations[0], locations[1], locations[2]);
                break;
            case 4:
                shapeId = createShape(locations[0], locations[1], locations[2],
                        locations[3]);
                break;
            default:
                String message = "numVertices = " + numVertices;
                throw new IllegalArgumentException(message);
        }
        setNativeId(shapeId);

        setContactFilterEnabled(enableContactFilter);
        setScale(scale);
        setMargin(margin);
    }
    // *************************************************************************
    // native private methods

    native private static long createShape(Vector3f vector1);

    native private static long createShape(Vector3f vector1, Vector3f vector2);

    native private static long
            createShape(Vector3f vector1, Vector3f vector2, Vector3f vector3);

    native private static long createShape(Vector3f vector1, Vector3f vector2,
            Vector3f vector3, Vector3f vector4);

    native private static void recalcAabb(long shapeId);
}
