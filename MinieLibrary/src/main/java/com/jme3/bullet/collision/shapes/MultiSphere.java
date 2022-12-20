/*
 * Copyright (c) 2018-2022 jMonkeyEngine
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

import com.jme3.bounding.BoundingSphere;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.MyVolume;
import jme3utilities.math.RectangularSolid;

/**
 * A convex collision shape based on Bullet's {@code btMultiSphereShape}. Unlike
 * a CapsuleCollisionShape or a SphereCollisionShape, these shapes have margins
 * and can be scaled non-uniformly.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class MultiSphere extends ConvexShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(MultiSphere.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagCenters = "centers";
    final private static String tagRadii = "radii";
    // *************************************************************************
    // fields

    /**
     * copies of the unscaled radii (in shape units, each &ge;0)
     */
    private float[] radii;
    /**
     * copies of the center locations (in shape coordinates)
     */
    private Vector3f[] centers;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected MultiSphere() {
    }

    /**
     * Instantiate a centered sphere shape with the specified radius.
     *
     * @param radius the desired radius (in shape units, &ge;0)
     */
    public MultiSphere(float radius) {
        Validate.nonNegative(radius, "radius");

        this.centers = new Vector3f[1];
        this.centers[0] = new Vector3f(0f, 0f, 0f);
        this.radii = new float[]{radius};

        createShape();
    }

    /**
     * Instantiate a centered Y-axis capsule shape with the specified radius and
     * height.
     *
     * @param radius the desired radius (in shape units, &ge;0)
     * @param height the desired height of the cylindrical portion (in shape
     * units, &ge;0)
     */
    public MultiSphere(float radius, float height) {
        Validate.nonNegative(radius, "radius");
        Validate.nonNegative(height, "height");

        float halfHeight = height / 2f;
        this.centers = new Vector3f[2];
        this.centers[0] = new Vector3f(0f, halfHeight, 0f);
        this.centers[1] = new Vector3f(0f, -halfHeight, 0f);

        this.radii = new float[]{radius, radius};

        createShape();
    }

    /**
     * Instantiate a centered capsule shape with the specified radius, height,
     * and axis.
     *
     * @param radius the desired radius (in shape units, &ge;0)
     * @param height the desired height of the cylindrical portion (in shape
     * units, &ge;0)
     * @param axisIndex which local axis to use for the height: 0&rarr;X,
     * 1&rarr;Y, 2&rarr;Z
     */
    public MultiSphere(float radius, float height, int axisIndex) {
        Validate.nonNegative(radius, "radius");
        Validate.nonNegative(height, "height");

        float halfHeight = height / 2f;

        this.centers = new Vector3f[2];
        switch (axisIndex) {
            case PhysicsSpace.AXIS_X:
                this.centers[0] = new Vector3f(halfHeight, 0f, 0f);
                this.centers[1] = new Vector3f(-halfHeight, 0f, 0f);
                break;
            case PhysicsSpace.AXIS_Y:
                this.centers[0] = new Vector3f(0f, halfHeight, 0f);
                this.centers[1] = new Vector3f(0f, -halfHeight, 0f);
                break;
            case PhysicsSpace.AXIS_Z:
                this.centers[0] = new Vector3f(0f, 0f, halfHeight);
                this.centers[1] = new Vector3f(0f, 0f, -halfHeight);
                break;
            default:
                throw new IllegalArgumentException("axisIndex = " + axisIndex);
        }

        this.radii = new float[]{radius, radius};

        createShape();
    }

    /**
     * Instantiate an eccentric sphere shape to match the specified
     * BoundingSphere.
     *
     * @param boundingSphere (not null, unaffected)
     */
    public MultiSphere(BoundingSphere boundingSphere) {
        this.centers = new Vector3f[1];
        this.centers[0] = boundingSphere.getCenter().clone();

        this.radii = new float[1];
        this.radii[0] = boundingSphere.getRadius();

        createShape();
    }

    /**
     * Instantiate a multi-sphere shape with the specified centers and radii.
     *
     * @param centers the list of center locations (in shape coordinates, not
     * null, not empty)
     * @param radii the list of radii (in shape units, not null, not empty, each
     * &ge;0)
     */
    public MultiSphere(List<Vector3f> centers, List<Float> radii) {
        Validate.nonEmpty(centers, "centers");
        Validate.nonEmpty(radii, "radii");

        int numSpheres = radii.size();
        Validate.require(centers.size() == numSpheres, "lists of equal length");

        this.centers = new Vector3f[numSpheres];
        this.radii = new float[numSpheres];
        for (int i = 0; i < numSpheres; ++i) {
            this.centers[i] = centers.get(i).clone();

            float radius = radii.get(i);
            assert radius >= 0f : radius;
            this.radii[i] = radius;
        }

        createShape();
    }

    /**
     * Instantiate a 4-sphere shape to fill the specified RectangularSolid. The
     * spheres will be of equal size, arranged in a rectangle.
     *
     * @param rectangularSolid the solid on which to base the shape (not null)
     */
    public MultiSphere(RectangularSolid rectangularSolid) {
        Vector3f halfExtents = rectangularSolid.halfExtents(null);
        float radius = MyMath.min(halfExtents.x, halfExtents.y, halfExtents.z);

        // Enumerate the local coordinates of the centers of the 4 spheres.
        Vector3f max = rectangularSolid.maxima(null);
        max.subtractLocal(radius, radius, radius);
        Vector3f min = rectangularSolid.minima(null);
        min.addLocal(radius, radius, radius);
        List<Vector3f> centerLocations = new ArrayList<>(4);
        if (radius == halfExtents.x) {
            float x = max.x;
            centerLocations.add(new Vector3f(x, max.y, max.z));
            centerLocations.add(new Vector3f(x, max.y, min.z));
            centerLocations.add(new Vector3f(x, min.y, max.z));
            centerLocations.add(new Vector3f(x, min.y, min.z));
        } else if (radius == halfExtents.y) {
            float y = max.y;
            centerLocations.add(new Vector3f(max.x, y, max.z));
            centerLocations.add(new Vector3f(max.x, y, min.z));
            centerLocations.add(new Vector3f(min.x, y, max.z));
            centerLocations.add(new Vector3f(min.x, y, min.z));
        } else {
            assert radius == halfExtents.z;
            float z = max.z;
            centerLocations.add(new Vector3f(max.x, max.y, z));
            centerLocations.add(new Vector3f(max.x, min.y, z));
            centerLocations.add(new Vector3f(min.x, max.y, z));
            centerLocations.add(new Vector3f(min.x, min.y, z));
        }

        // Transform centers to shape coordinates.
        this.centers = new Vector3f[4];
        this.radii = new float[]{radius, radius, radius, radius};
        for (int sphereI = 0; sphereI < 4; ++sphereI) {
            Vector3f localCenter = centerLocations.get(sphereI);
            this.centers[sphereI]
                    = rectangularSolid.localToWorld(localCenter, null);
        }

        createShape();
    }

    /**
     * Instantiate a 2-sphere shape to approximate the specified
     * RectangularSolid. The spheres will be of equal size, as in a capsule.
     *
     * @param rectangularSolid the solid on which to base the shape (not null)
     * @param fraction used to determine sphere radii (0&rarr;shortest axis,
     * 1&rarr;medium axis)
     */
    public MultiSphere(RectangularSolid rectangularSolid, float fraction) {
        Vector3f halfExtents = rectangularSolid.halfExtents(null);
        float shortest
                = MyMath.min(halfExtents.x, halfExtents.y, halfExtents.z);
        float medium
                = MyMath.mid(halfExtents.x, halfExtents.y, halfExtents.z);
        float radius = MyMath.lerp(fraction, shortest, medium); // interpolate
        float longest
                = MyMath.max(halfExtents.x, halfExtents.y, halfExtents.z);
        assert longest >= radius;

        // Calculate the local coordinates of the centers of both spheres.
        Vector3f max = rectangularSolid.maxima(null);
        Vector3f min = rectangularSolid.minima(null);
        Vector3f mid = MyVector3f.midpoint(max, min, null);
        List<Vector3f> centerLocations = new ArrayList<>(2);
        if (longest == halfExtents.z) {
            centerLocations.add(new Vector3f(mid.x, mid.y, min.z + radius));
            centerLocations.add(new Vector3f(mid.x, mid.y, max.z - radius));
        } else if (longest == halfExtents.y) {
            centerLocations.add(new Vector3f(mid.x, min.y + radius, mid.z));
            centerLocations.add(new Vector3f(mid.x, max.y - radius, mid.z));
        } else {
            assert longest == halfExtents.x;
            centerLocations.add(new Vector3f(min.x + radius, mid.y, mid.z));
            centerLocations.add(new Vector3f(max.x - radius, mid.y, mid.z));
        }

        // Transform centers to shape coordinates.
        this.centers = new Vector3f[2];
        this.radii = new float[]{radius, radius};
        for (int sphereI = 0; sphereI < 2; ++sphereI) {
            Vector3f localCenter = centerLocations.get(sphereI);
            this.centers[sphereI]
                    = rectangularSolid.localToWorld(localCenter, null);
        }

        createShape();
    }

    /**
     * Instantiate an eccentric sphere shape with the specified center and
     * radius.
     *
     * @param center the location of the center (in shape coordinates, not null,
     * unaffected)
     * @param radius the desired radius (in shape units, &ge;0)
     */
    public MultiSphere(Vector3f center, float radius) {
        Validate.finite(center, "center");
        Validate.nonNegative(radius, "radius");

        this.centers = new Vector3f[1];
        this.centers[0] = center.clone();

        this.radii = new float[]{radius};

        createShape();
    }

    /**
     * Instantiate a multi-sphere shape with the specified centers and radii.
     *
     * @param centers the array of center locations (in shape coordinates, not
     * null, not empty)
     * @param radii the array of radii (in shape units, not null, not empty,
     * each &ge;0)
     */
    public MultiSphere(Vector3f[] centers, float... radii) {
        Validate.nonEmpty(centers, "centers");
        Validate.nonEmpty(radii, "radii");

        int numSpheres = radii.length;
        Validate.require(
                centers.length == numSpheres, "arrays of equal length");

        this.centers = new Vector3f[numSpheres];
        this.radii = new float[numSpheres];
        for (int i = 0; i < numSpheres; ++i) {
            this.centers[i] = centers[i].clone();

            float radius = radii[i];
            assert radius >= 0f : radius;
            this.radii[i] = radius;
        }

        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the location of the center of the indexed sphere.
     *
     * @param sphereIndex which sphere to read (&ge;0)
     * @param storeResult storage for the result (modified if not null)
     * @return the center location (in shape coordinates, either
     * {@code storeResult} or a new instance, not null)
     */
    public Vector3f copyCenter(int sphereIndex, Vector3f storeResult) {
        Validate.inRange(sphereIndex, "sphere index", 0, radii.length - 1);
        Vector3f result = storeResult == null ? new Vector3f() : storeResult;

        result.set(centers[sphereIndex]);

        return result;
    }

    /**
     * Count the spheres in this shape.
     *
     * @return the count (&gt;0)
     */
    public int countSpheres() {
        int count = radii.length;
        assert count > 0 : count;
        return count;
    }

    /**
     * Return the radius of the indexed sphere.
     *
     * @param sphereIndex which sphere to read (&ge;0)
     * @return the unscaled radius (&ge;0)
     */
    public float getRadius(int sphereIndex) {
        Validate.inRange(sphereIndex, "sphere index", 0, radii.length - 1);
        float radius = radii[sphereIndex];

        assert radius >= 0f : radius;
        return radius;
    }
    // *************************************************************************
    // ConvexShape methods

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
        this.radii = cloner.clone(radii);
        this.centers = cloner.clone(centers);
        createShape();
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

        Savable[] savCenters
                = capsule.readSavableArray(tagCenters, new Vector3f[0]);
        int numSpheres = savCenters.length;
        this.centers = new Vector3f[numSpheres];
        for (int sphereIndex = 0; sphereIndex < numSpheres; ++sphereIndex) {
            this.centers[sphereIndex] = (Vector3f) savCenters[sphereIndex];
        }
        this.radii = capsule.readFloatArray(tagRadii, new float[0]);
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
     * Estimate the volume of the shape, including scale and margin.
     *
     * @return the volume (in physics-space units cubed, &ge;0)
     */
    @Override
    public float scaledVolume() {
        float volume;
        int numSpheres = radii.length;
        if (numSpheres == 1) {
            float radius = radii[0];
            float unscaledVolume = MyVolume.sphereVolume(radius);
            volume = unscaledVolume * scale.x * scale.y * scale.z;

        } else if (numSpheres == 2 && radii[0] == radii[1]) { // capsule
            float radius = radii[0];
            float height = centers[0].distance(centers[1]);
            float unscaledVolume = MyVolume.capsuleVolume(radius, height);
            volume = unscaledVolume * scale.x * scale.y * scale.z;

        } else { // use the debug mesh
            volume = super.scaledVolume();
        }

        assert volume >= 0f : volume;
        return volume;
    }

    /**
     * Approximate this shape with a HullCollisionShape.
     *
     * @return a new shape
     */
    @Override
    public HullCollisionShape toHullShape() {
        float medianScale = MyMath.mid(scale.x, scale.y, scale.z);
        assert medianScale > 0f : medianScale;
        float minRadius = MyMath.min(radii);
        float defaultMargin = getDefaultMargin();
        float hullMargin = Math.min(defaultMargin, minRadius * medianScale);
        /*
         * Construct a copy of this shape with its radii reduced
         * to compensate for the hull's collision margin.
         */
        int numSpheres = radii.length;
        float[] reducedRadii = new float[numSpheres];
        for (int sphereIndex = 0; sphereIndex < numSpheres; ++sphereIndex) {
            float rr = radii[sphereIndex] - hullMargin / medianScale;
            if (rr < 1e-6f) {
                rr = 1e-6f;
            }
            reducedRadii[sphereIndex] = rr;
        }
        MultiSphere reducedShape = new MultiSphere(centers, reducedRadii);
        reducedShape.setScale(scale);
        FloatBuffer buffer = DebugShapeFactory
                .debugVertices(reducedShape, DebugShapeFactory.lowResolution);

        // Flip the buffer.
        buffer.rewind();
        buffer.limit(buffer.capacity());

        HullCollisionShape result = new HullCollisionShape(buffer);
        result.setMargin(hullMargin);

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

        capsule.write(centers, tagCenters, null);
        capsule.write(radii, tagRadii, null);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured shape in Bullet.
     */
    private void createShape() {
        int numSpheres = radii.length;
        assert centers.length == numSpheres : numSpheres;

        long shapeId = createShape(centers, radii, numSpheres);
        setNativeId(shapeId);

        setContactFilterEnabled(enableContactFilter);
        setScale(scale);
        setMargin(margin);
    }
    // *************************************************************************
    // native private methods

    native private static long
            createShape(Vector3f[] centers, float[] radii, int numSpheres);

    native private static void recalcAabb(long shapeId);
}
