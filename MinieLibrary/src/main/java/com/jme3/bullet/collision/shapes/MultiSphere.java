/*
 * Copyright (c) 2018 jMonkeyEngine
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
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;
import jme3utilities.math.RectangularSolid;

/**
 * A convex collision shape based on Bullet's btMultiSphereShape. Unlike a
 * CapsuleCollisionShape or a SphereCollisionShape, these shapes have margins
 * and can be scaled non-uniformly.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class MultiSphere extends CollisionShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(MultiSphere.class.getName());
    // *************************************************************************
    // fields

    /**
     * copies of radii (in unscaled units, each &ge;0)
     */
    private float[] radii;
    /**
     * copies of center locations
     */
    private Vector3f[] centers;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public MultiSphere() {
    }

    /**
     * Instantiate a centered sphere shape with the specified radius.
     *
     * @param radius the desired radius (in unscaled units, &ge;0)
     */
    public MultiSphere(float radius) {
        Validate.nonNegative(radius, "radius");

        centers = new Vector3f[1];
        centers[0] = new Vector3f(0f, 0f, 0f);

        radii = new float[1];
        radii[0] = radius;

        createShape();
    }

    /**
     * Instantiate a centered Y-axis capsule shape with the specified radius and
     * height.
     *
     * @param radius the desired radius (in unscaled units, &ge;0)
     * @param height the desired height (of the cylindrical portion) (in
     * unscaled units, &ge;0)
     */
    public MultiSphere(float radius, float height) {
        Validate.nonNegative(radius, "radius");
        Validate.nonNegative(height, "height");

        float halfHeight = height / 2f;

        centers = new Vector3f[2];
        centers[0] = new Vector3f(0f, halfHeight, 0f);
        centers[1] = new Vector3f(0f, -halfHeight, 0f);

        radii = new float[2];
        radii[0] = radius;
        radii[1] = radius;

        createShape();
    }

    /**
     * Instantiate an eccentric sphere shape to match the specified
     * BoundingSphere.
     *
     * @param boundingSphere (not null, unaffected)
     */
    public MultiSphere(BoundingSphere boundingSphere) {
        centers = new Vector3f[1];
        centers[0] = boundingSphere.getCenter().clone();

        radii = new float[1];
        radii[0] = boundingSphere.getRadius();

        createShape();
    }

    /**
     * Instantiate a multi-sphere shape with the specified centers and radii.
     *
     * @param centers the list of center offsets (not null, not empty)
     * @param radii the list of radii (not null, not empty, each &ge;0)
     */
    public MultiSphere(List<Vector3f> centers, List<Float> radii) {
        Validate.nonEmpty(centers, "centers");
        Validate.nonEmpty(radii, "radii");

        int numSpheres = radii.size();
        assert centers.size() == numSpheres : numSpheres;

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
     * Instantiate a 4-sphere shape to fill the specified rectangular solid. The
     * spheres will be of equal size, arranged in a rectangle.
     *
     * @param rectangularSolid the solid on which to base the shape (not null)
     */
    public MultiSphere(RectangularSolid rectangularSolid) {
        Vector3f halfExtents = rectangularSolid.halfExtents(null);
        float radius = MyMath.min(halfExtents.x, halfExtents.y, halfExtents.z);
        /*
         * Enumerate the local coordinates of the centers of the 4 spheres.
         */
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
        /*
         * Transform centers to shape coordinates.
         */
        centers = new Vector3f[4];
        radii = new float[4];
        for (int sphereI = 0; sphereI < centerLocations.size(); sphereI++) {
            Vector3f localCenter = centerLocations.get(sphereI);
            centers[sphereI] = rectangularSolid.localToWorld(localCenter, null);
            radii[sphereI] = radius;
        }

        createShape();
    }

    /**
     * Instantiate an eccentric sphere shape with the specified center and
     * radius.
     *
     * @param center the offset of the center (not null, unaffected)
     * @param radius the desired radius (in unscaled units, &ge;0)
     */
    public MultiSphere(Vector3f center, float radius) {
        Validate.finite(center, "center");
        Validate.nonNegative(radius, "radius");

        centers = new Vector3f[1];
        centers[0] = center.clone();

        radii = new float[1];
        radii[0] = radius;

        createShape();
    }
    // *************************************************************************
    // new methods exposed

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
     * Read the radius of the indexed sphere.
     *
     * @param sphereIndex which sphere to read (&ge;0)
     * @return the radius (in unscaled units, &ge;0)
     */
    public float getRadius(int sphereIndex) {
        Validate.inRange(sphereIndex, "sphere index", 0, radii.length - 1);
        float radius = radii[sphereIndex];

        assert radius >= 0f : radius;
        return radius;
    }

    /**
     * Estimate the scaled volume of this shape, based on its debug mesh.
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
        radii = cloner.clone(radii);
        centers = cloner.clone(centers);
        createShape();
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public MultiSphere jmeClone() {
        try {
            MultiSphere clone = (MultiSphere) super.clone();
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

        centers = (Vector3f[]) capsule.readSavableArray("centers",
                new Vector3f[0]);
        radii = capsule.readFloatArray("radii", new float[0]);
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

        capsule.write(centers, "centers", new Vector3f[0]);
        capsule.write(radii, "radii", new float[0]);
    }
    // *************************************************************************
    // private methods

    /**
     * Instantiate the configured shape in Bullet.
     */
    private void createShape() {
        int numSpheres = radii.length;
        assert centers.length == numSpheres : numSpheres;
        objectId = createShape(centers, radii, numSpheres);
        assert objectId != 0L;
        logger2.log(Level.FINE, "Created Shape {0}",
                Long.toHexString(objectId));

        setScale(scale);
        setMargin(margin);
    }

    native private long createShape(Vector3f[] centers, float[] radii,
            int numSpheres);
}
