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

import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.MyVolume;

/**
 * A simple point, line-segment, triangle, or tetrahedron collision shape based
 * on Bullet's btBU_Simplex1to4.
 *
 * @author normenhansen
 */
public class SimplexCollisionShape extends CollisionShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(SimplexCollisionShape.class.getName());
    // *************************************************************************
    // fields

    /**
     * vertex positions (may be null)
     */
    private Vector3f vector1, vector2, vector3, vector4;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public SimplexCollisionShape() {
    }

    /**
     * Instantiate a point collision shape based on the specified points.
     *
     * @param point1 the coordinates of 1st point (not null, unaffected)
     */
    public SimplexCollisionShape(Vector3f point1) {
        vector1 = point1.clone();
        createShape();
    }

    /**
     * Instantiate a line-segment collision shape based on the specified points.
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
     * Instantiate a triangular collision shape based on the specified points.
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
     * Instantiate a tetrahedral collision shape based on the specified points.
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
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the indexed vertex.
     *
     * @param index (&ge;0, &lt;4)
     * @param storeResult (modified if not null)
     * @return the position of the vertex (either storeResult or a new instance)
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
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned shape into a deep-cloned one, using the specified cloner
     * and original to resolve copied fields.
     *
     * @param cloner the cloner that's cloning this shape (not null)
     * @param original the instance from which this instance was shallow-cloned
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
     * De-serialize this shape, for example when loading from a J3O file.
     *
     * @param im importer (not null)
     * @throws IOException from importer
     */
    @Override
    public void read(JmeImporter im) throws IOException {
        super.read(im);
        InputCapsule capsule = im.getCapsule(this);
        vector1 = (Vector3f) capsule.readSavable("simplexPoint1", null);
        vector2 = (Vector3f) capsule.readSavable("simplexPoint2", null);
        vector3 = (Vector3f) capsule.readSavable("simplexPoint3", null);
        vector4 = (Vector3f) capsule.readSavable("simplexPoint4", null);
        createShape();
    }

    /**
     * Serialize this shape, for example when saving to a J3O file.
     *
     * @param ex exporter (not null)
     * @throws IOException from exporter
     */
    @Override
    public void write(JmeExporter ex) throws IOException {
        super.write(ex);
        OutputCapsule capsule = ex.getCapsule(this);
        capsule.write(vector1, "simplexPoint1", null);
        capsule.write(vector2, "simplexPoint2", null);
        capsule.write(vector3, "simplexPoint3", null);
        capsule.write(vector4, "simplexPoint4", null);
    }
    // *************************************************************************
    // private methods

    /**
     * Instantiate the configured shape in Bullet.
     */
    private void createShape() {
        assert objectId == 0L;

        if (vector4 != null) {
            objectId = createShape(vector1, vector2, vector3, vector4);
        } else if (vector3 != null) {
            objectId = createShape(vector1, vector2, vector3);
        } else if (vector2 != null) {
            objectId = createShape(vector1, vector2);
        } else {
            objectId = createShape(vector1);
        }
        assert objectId != 0L;
        logger2.log(Level.FINE, "Created Shape {0}", Long.toHexString(objectId));

        setScale(scale);
        setMargin(margin);
    }

    native private long createShape(Vector3f vector1);

    native private long createShape(Vector3f vector1, Vector3f vector2);

    native private long createShape(Vector3f vector1, Vector3f vector2,
            Vector3f vector3);

    native private long createShape(Vector3f vector1, Vector3f vector2,
            Vector3f vector3, Vector3f vector4);

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
}
