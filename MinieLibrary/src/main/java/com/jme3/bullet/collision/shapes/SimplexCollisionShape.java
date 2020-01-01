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
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.nio.FloatBuffer;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.MyVolume;

/**
 * A simple point, line-segment, triangle, or tetrahedron CollisionShape based
 * on Bullet's btBU_Simplex1to4. These shapes cannot be scaled.
 *
 * @author normenhansen
 */
public class SimplexCollisionShape extends CollisionShape {
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
     * vertex locations (may be null)
     */
    private Vector3f vector1, vector2, vector3, vector4;
    // *************************************************************************
    // constructors - TODO add Vector3f..., FloatBuffer, LineSegment, and AbstractTriangle constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public SimplexCollisionShape() {
    }

    /**
     * Instantiate a point shape based on the specified point.
     *
     * @param point1 the coordinates of the point (not null, unaffected)
     */
    public SimplexCollisionShape(Vector3f point1) {
        vector1 = point1.clone();
        createShape();
    }

    /**
     * Instantiate a line-segment shape based on the specified points.
     *
     * @param point1 the coordinates of 1st point (not null, unaffected)
     * @param point2 the coordinates of 2nd point (not null, unaffected)
     */
    public SimplexCollisionShape(Vector3f point1, Vector3f point2) {
        vector1 = point1.clone();
        vector2 = point2.clone();
        createShape();
    }

    /**
     * Instantiate a triangular shape based on the specified points.
     *
     * @param point1 the coordinates of 1st point (not null, unaffected)
     * @param point2 the coordinates of 2nd point (not null, unaffected)
     * @param point3 the coordinates of 3rd point (not null, unaffected)
     */
    public SimplexCollisionShape(Vector3f point1, Vector3f point2,
            Vector3f point3) {
        vector1 = point1.clone();
        vector2 = point2.clone();
        vector3 = point3.clone();
        createShape();
    }

    /**
     * Instantiate a tetrahedral shape based on the specified points.
     *
     * @param point1 the coordinates of 1st point (not null, unaffected)
     * @param point2 the coordinates of 2nd point (not null, unaffected)
     * @param point3 the coordinates of 3rd point (not null, unaffected)
     * @param point4 the coordinates of 4th point (not null, unaffected)
     */
    public SimplexCollisionShape(Vector3f point1, Vector3f point2,
            Vector3f point3, Vector3f point4) {
        vector1 = point1.clone();
        vector2 = point2.clone();
        vector3 = point3.clone();
        vector4 = point4.clone();
        createShape();
    }

    /**
     * Instantiate a tetrahedral shape based on the specified FloatBuffer range.
     *
     * @param buffer the buffer that contains the vertex locations (not null,
     * unaffected)
     * @param startPosition the position at which the vertex locations start
     * (&ge;0, &le;endPosition-3)
     * @param endPosition the position at which the vertex locations end
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
        assert (numFloats % numAxes == 0) : numFloats;
        int numVertices = numFloats / numAxes;

        switch (numVertices) {
            case 4:
                vector4 = new Vector3f();
                get(buffer, startPosition + 3 * numAxes, vector4);
            case 3:
                vector3 = new Vector3f();
                get(buffer, startPosition + 2 * numAxes, vector3);
            case 2:
                vector2 = new Vector3f();
                get(buffer, startPosition + numAxes, vector2);
            case 1:
                vector1 = new Vector3f();
                get(buffer, startPosition, vector1);
                break;

            default:
                String message = Integer.toString(numVertices);
                throw new RuntimeException(message);
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
        int numVertices = countMeshVertices();
        Validate.inRange(index, "index", 0, numVertices - 1);
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        Vector3f vertex = getVertex(index);
        result.set(vertex);

        return result;
    }

    /**
     * Copy the unscaled vertex locations.
     *
     * @return a new array (not null)
     */
    public float[] copyVertices() {
        int numVertices = countMeshVertices();
        float[] result = new float[numVertices * numAxes];
        for (int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex) {
            int floatIndex = vertexIndex * numAxes;
            Vector3f location = getVertex(vertexIndex);
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
        int result;
        if (vector4 != null) {
            result = 4;
        } else if (vector3 != null) {
            result = 3;
        } else if (vector2 != null) {
            result = 2;
        } else {
            result = 1;
        }

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
        int numMeshVertices = countMeshVertices();
        for (int i = 0; i < numMeshVertices; ++i) {
            Vector3f location = getVertex(i);
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
     * @return the volume (in shape-space units cubed, &ge;0)
     */
    public float unscaledVolume() {
        float volume = 0f;
        if (vector4 != null) {
            volume = (float) MyVolume.tetrahedronVolume(vector1, vector2,
                    vector3, vector4);
        }

        assert volume >= 0f : volume;
        return volume;
    }
    // *************************************************************************
    // CollisionShape methods

    /**
     * Test whether the specified scaling factors can be applied to this shape.
     * For simplex shapes, scaling must be unity.
     *
     * @param scale the desired scaling factor for each local axis (may be null,
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
        vector1 = cloner.clone(vector1);
        vector2 = cloner.clone(vector2);
        vector3 = cloner.clone(vector3);
        vector4 = cloner.clone(vector4);
        createShape();
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public SimplexCollisionShape jmeClone() {
        try {
            SimplexCollisionShape clone = (SimplexCollisionShape) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
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

        vector1 = (Vector3f) capsule.readSavable(tagSimplexPoint1, null);
        vector2 = (Vector3f) capsule.readSavable(tagSimplexPoint2, null);
        vector3 = (Vector3f) capsule.readSavable(tagSimplexPoint3, null);
        vector4 = (Vector3f) capsule.readSavable(tagSimplexPoint4, null);
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

        capsule.write(vector1, tagSimplexPoint1, null);
        capsule.write(vector2, tagSimplexPoint2, null);
        capsule.write(vector3, tagSimplexPoint3, null);
        capsule.write(vector4, tagSimplexPoint4, null);
    }
    // *************************************************************************
    // private methods

    /**
     * Instantiate the configured shape in Bullet.
     */
    private void createShape() {
        long shapeId;
        if (vector4 != null) {
            shapeId = createShape(vector1, vector2, vector3, vector4);
        } else if (vector3 != null) {
            shapeId = createShape(vector1, vector2, vector3);
        } else if (vector2 != null) {
            shapeId = createShape(vector1, vector2);
        } else {
            shapeId = createShape(vector1);
        }
        setNativeId(shapeId);

        setScale(scale);
        setMargin(margin);
    }

    /**
     * Read a Vector3f starting from the given position. Does not alter the
     * buffer's position. TODO use MyBuffer
     *
     * @param buffer the buffer to read from (not null, unaffected)
     * @param startPosition the position at which to start reading (&ge;0)
     * @param storeVector storage for the vector (not null, modified)
     */
    private static void get(FloatBuffer buffer, int startPosition,
            Vector3f storeVector) {
        Validate.nonNull(buffer, "buffer");
        Validate.nonNegative(startPosition, "start position");
        Validate.nonNull(storeVector, "store vector");

        storeVector.x = buffer.get(startPosition + MyVector3f.xAxis);
        storeVector.y = buffer.get(startPosition + MyVector3f.yAxis);
        storeVector.z = buffer.get(startPosition + MyVector3f.zAxis);
    }

    /**
     * Access the indexed vertex.
     *
     * @param index (&ge;0, &lt;4)
     * @return the pre-existing vector (may be null)
     */
    private Vector3f getVertex(int index) {
        Vector3f result;
        switch (index) {
            case 0:
                result = vector1;
                break;
            case 1:
                result = vector2;
                break;
            case 2:
                result = vector3;
                break;
            case 3:
                result = vector4;
                break;
            default:
                throw new IllegalArgumentException(Integer.toString(index));
        }

        return result;
    }
    // *************************************************************************
    // native methods

    native private long createShape(Vector3f vector1);

    native private long createShape(Vector3f vector1, Vector3f vector2);

    native private long createShape(Vector3f vector1, Vector3f vector2,
            Vector3f vector3);

    native private long createShape(Vector3f vector1, Vector3f vector2,
            Vector3f vector3, Vector3f vector4);

    native private void recalcAabb(long shapeId);
}
