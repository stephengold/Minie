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
import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Collection;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.RectangularSolid;

/**
 * A convex-hull collision shape based on Bullet's btConvexHullShape.
 */
public class HullCollisionShape extends CollisionShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(HullCollisionShape.class.getName());
    // *************************************************************************
    // fields

    /**
     * buffer for passing vertices to Bullet
     * <p>
     * A Java reference must persist after createShape() completes, or else the
     * buffer might get garbage collected.
     */
    private ByteBuffer bbuf;
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
     * Instantiate a collision shape based on the specified collection of
     * locations. For best performance and stability, the collection should have
     * no more than 100 locations.
     *
     * @param locations a collection of location vectors on which to base the
     * shape (not null, not empty, unaffected)
     */
    public HullCollisionShape(Collection<Vector3f> locations) {
        Validate.nonEmpty(locations, "locations");

        int numLocations = locations.size();
        points = new float[3 * numLocations];
        int j = 0;
        for (Vector3f location : locations) {
            points[j] = location.x;
            points[j + 1] = location.y;
            points[j + 2] = location.z;
            j += 3;
        }

        createShape();
    }

    /**
     * Instantiate a collision shape based on the specified array of
     * coordinates. For best performance and stability, the array should have no
     * more than 300 coordinates.
     *
     * @param points an array of coordinates on which to base the shape (not
     * null, not empty, length a multiple of 3, unaffected)
     */
    public HullCollisionShape(float[] points) {
        Validate.nonEmpty(points, "points");
        int length = points.length;
        assert (length % 3 == 0) : length;

        this.points = points.clone();
        createShape();
    }

    /**
     * Instantiate a collision shape based on the specified JME mesh. For best
     * performance and stability, the mesh should have no more than 100
     * vertices.
     *
     * @param mesh the mesh on which to base the shape (not null, at least one
     * vertex, unaffected)
     */
    public HullCollisionShape(Mesh mesh) {
        Validate.nonNull(mesh, "mesh");
        assert mesh.getVertexCount() > 0;

        points = getPoints(mesh);
        createShape();
    }

    /**
     * Instantiate an 8-vertex collision shape to match the specified
     * rectangular solid.
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
        int numFloats = 3 * cornerLocations.size();
        points = new float[numFloats];
        int floatIndex = 0;
        Vector3f tempVector = new Vector3f();
        for (Vector3f location : cornerLocations) {
            rectangularSolid.localToWorld(location, tempVector);
            points[floatIndex] = tempVector.x;
            points[floatIndex + 1] = tempVector.y;
            points[floatIndex + 2] = tempVector.z;
            floatIndex += 3;
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
        for (int floatIndex = 0; floatIndex < points.length; floatIndex += 3) {
            float x = points[floatIndex];
            float y = points[floatIndex + 1];
            float z = points[floatIndex + 2];
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
     * Count the vertices used to generate the hull.
     *
     * @return the count (&gt;0)
     */
    public int countMeshVertices() {
        int length = points.length;
        assert (length % 3 == 0) : length;
        int result = length / 3;

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
    public final Vector3f getHalfExtents(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        result.zero();
        for (int i = 0; i < points.length; i += 3) {
            float x = FastMath.abs(points[i]);
            if (x > result.x) {
                result.x = x;
            }
            float y = FastMath.abs(points[i + 1]);
            if (y > result.y) {
                result.y = y;
            }
            float z = FastMath.abs(points[i + 2]);
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
        int meshResolution = DebugShapeFactory.highResolution;
        float volume = DebugShapeFactory.volumeConvex(this, meshResolution);
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
        // bbuf not cloned
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
     * De-serialize this shape, for example when loading from a J3O file.
     *
     * @param im importer (not null)
     * @throws IOException from importer
     */
    @Override
    public void read(JmeImporter im) throws IOException {
        super.read(im);
        InputCapsule capsule = im.getCapsule(this);

        // for backwards compatability
        Mesh mesh = (Mesh) capsule.readSavable("hullMesh", null);
        if (mesh != null) {
            points = getPoints(mesh);
        } else {
            points = capsule.readFloatArray("points", new float[0]);
        }
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
        capsule.write(points, "points", new float[0]);
    }
    // *************************************************************************
    // private methods

    /**
     * Instantiate the configured shape in Bullet.
     */
    private void createShape() {
        assert points.length != 0;
        assert objectId == 0L;

        bbuf = BufferUtils.createByteBuffer(points.length * 4);
        for (int i = 0; i < points.length; ++i) {
            float f = points[i];
            if (!Float.isFinite(f)) {
                String msg = "illegal coordinate: " + Float.toString(f);
                throw new IllegalArgumentException(msg);
            }
            bbuf.putFloat(f);
        }
        bbuf.rewind();

        objectId = createShape(bbuf);
        assert objectId != 0L;
        logger2.log(Level.FINE, "Created Shape {0}",
                Long.toHexString(objectId));

        setScale(scale);
        setMargin(margin);
    }

    native private long createShape(ByteBuffer points);

    /**
     * Copy the vertex positions from a JME mesh.
     *
     * @param mesh the mesh to read (not null)
     * @return a new array (not null, length a multiple of 3)
     */
    private float[] getPoints(Mesh mesh) {
        FloatBuffer vertices = mesh.getFloatBuffer(Type.Position);
        vertices.rewind();
        int components = mesh.getVertexCount() * 3;
        float[] pointsArray = new float[components];
        for (int i = 0; i < components; i += 3) {
            pointsArray[i] = vertices.get();
            pointsArray[i + 1] = vertices.get();
            pointsArray[i + 2] = vertices.get();
        }

        return pointsArray;
    }
}
