/*
 * Copyright (c) 2018-2023 jMonkeyEngine
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
package com.jme3.bullet.animation;

import com.jme3.bullet.RotationOrder;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.MultiSphere;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import java.io.IOException;
import java.nio.FloatBuffer;
import java.util.Objects;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.RectangularSolid;
import jme3utilities.math.VectorSet;

/**
 * Configuration data for a PhysicsLink, not including the bone name, attached
 * model, or RangeOfMotion. Immutable except for
 * {@link #read(com.jme3.export.JmeImporter)}.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class LinkConfig implements Comparable<LinkConfig>, Savable {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(LinkConfig.class.getName());
    /**
     * local copy of {@link com.jme3.math.Quaternion#IDENTITY}
     */
    final private static Quaternion rotateIdentity = new Quaternion();
    /**
     * field names for serialization
     */
    final private static String tagCenterHeuristic = "centerHeuristic";
    final private static String tagMassHeuristic = "massHeuristic";
    final private static String tagMassParameter = "massParameter";
    final private static String tagRotationOrder = "rotationOrder";
    final private static String tagShapeHeuristic = "shapeHeuristic";
    final private static String tagShapeScale = "shapeScale";
    // *************************************************************************
    // fields

    /**
     * which centering heuristic to use
     */
    private CenterHeuristic centerHeuristic;
    /**
     * parameter to use to determine the mass (&gt;0)
     */
    private float massParameter;
    /**
     * which mass heuristic to use
     */
    private MassHeuristic massHeuristic;
    /**
     * rotation order for New6Dof axes, or null for a SixDofJoint
     */
    private RotationOrder rotationOrder;
    /**
     * which shaping heuristic to use
     */
    private ShapeHeuristic shapeHeuristic;
    /**
     * scale factors used to adjust the size of the shape (no negative
     * component)
     */
    private Vector3f shapeScale;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a configuration for a link with mass=1, centered on the
     * unweighted average of the vertex locations, shaped to the convex hull of
     * the vertices.
     */
    public LinkConfig() {
        this.centerHeuristic = CenterHeuristic.Mean;
        this.massParameter = 1f;
        this.massHeuristic = MassHeuristic.Mass;
        this.rotationOrder = null;
        this.shapeHeuristic = ShapeHeuristic.VertexHull;
        this.shapeScale = new Vector3f(1f, 1f, 1f);
    }

    /**
     * Instantiate a configuration for a link with the specified mass, centered
     * on the unweighted average of the vertex locations, shaped to the convex
     * hull of the vertices.
     *
     * @param mass the desired mass (&gt;0)
     */
    public LinkConfig(float mass) {
        Validate.positive(mass, "mass");

        this.centerHeuristic = CenterHeuristic.Mean;
        this.massParameter = mass;
        this.massHeuristic = MassHeuristic.Mass;
        this.rotationOrder = null;
        this.shapeHeuristic = ShapeHeuristic.VertexHull;
        this.shapeScale = new Vector3f(1f, 1f, 1f);
    }

    /**
     * Instantiate a configuration for a link with the specified mass, but with
     * the centering and shaping of the specified configuration.
     *
     * @param mass the desired mass (&gt;0)
     * @param oldConfig (not null)
     */
    public LinkConfig(float mass, LinkConfig oldConfig) {
        Validate.positive(mass, "mass");
        Validate.nonNull(oldConfig, "old configuration");

        this.centerHeuristic = oldConfig.centerHeuristic();
        this.massParameter = mass;
        this.massHeuristic = MassHeuristic.Mass;
        this.rotationOrder = null;
        this.shapeHeuristic = oldConfig.shapeHeuristic();
        this.shapeScale = oldConfig.shapeScale(null);
    }

    /**
     * Instantiate a custom configuration with no RotationOrder.
     *
     * @param massParm parameter to use to determine the mass (&gt;0)
     * @param massH which mass heuristic to use (not null)
     * @param shapeH which shaping heuristic to use (not null)
     * @param sScale used to adjust the size of the shape (not null, no negative
     * component, unaffected)
     * @param centerH which centering heuristic to use (not null)
     */
    public LinkConfig(float massParm, MassHeuristic massH,
            ShapeHeuristic shapeH, Vector3f sScale, CenterHeuristic centerH) {
        Validate.positive(massParm, "mass parameter");
        Validate.nonNull(massH, "mass heuristic");
        Validate.nonNull(shapeH, "shape heuristic");
        Validate.nonNegative(sScale, "shape scale");
        Validate.nonNull(centerH, "center heuristic");

        this.centerHeuristic = centerH;
        this.massParameter = massParm;
        this.massHeuristic = massH;
        this.rotationOrder = null;
        this.shapeHeuristic = shapeH;
        this.shapeScale = sScale.clone();
    }

    /**
     * Instantiate a custom configuration with a RotationOrder.
     *
     * @param massParm parameter to use to determine the mass (&gt;0)
     * @param massH which mass heuristic to use (not null)
     * @param shapeH which shaping heuristic to use (not null)
     * @param sScale used to adjust the size of the shape (not null, no negative
     * component, unaffected)
     * @param centerH which centering heuristic to use (not null)
     * @param axisOrder the rotation order for New6Dof axes, or null for a
     * SixDofJoint
     */
    public LinkConfig(float massParm, MassHeuristic massH,
            ShapeHeuristic shapeH, Vector3f sScale, CenterHeuristic centerH,
            RotationOrder axisOrder) {
        Validate.positive(massParm, "mass parameter");
        Validate.nonNull(massH, "mass heuristic");
        Validate.nonNull(shapeH, "shape heuristic");
        Validate.nonNegative(sScale, "shape scale");
        Validate.nonNull(centerH, "center heuristic");

        this.centerHeuristic = centerH;
        this.massParameter = massParm;
        this.massHeuristic = massH;
        this.rotationOrder = axisOrder;
        this.shapeHeuristic = shapeH;
        this.shapeScale = sScale.clone();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Read which centering heuristic to use.
     *
     * @return the enum value (not null)
     */
    public CenterHeuristic centerHeuristic() {
        assert centerHeuristic != null;
        return centerHeuristic;
    }

    /**
     * Create a CollisionShape for the specified transform, center, and vertex
     * locations.
     *
     * @param vertexToShape the transform from vertex coordinates to de-scaled
     * shape coordinates (not null, unaffected)
     * @param center the location of the shape's center, in vertex coordinates
     * (not null, unaffected)
     * @param vertexLocations the set of vertex locations (not null, not empty,
     * TRASHED)
     * @return a new CollisionShape
     */
    CollisionShape createShape(Transform vertexToShape, Vector3f center,
            VectorSet vertexLocations) {
        Validate.nonNull(vertexToShape, "transform");
        Validate.finite(center, "center");
        int numVectors = vertexLocations.numVectors();
        assert numVectors > 0 : numVectors;

        Vector3f tempLocation = new Vector3f();
        FloatBuffer buffer = vertexLocations.toBuffer();
        buffer.rewind();
        while (buffer.hasRemaining()) {
            buffer.mark();
            tempLocation.x = buffer.get();
            tempLocation.y = buffer.get();
            tempLocation.z = buffer.get();
            /*
             * Translate so that vertex coordinates are relative to
             * the shape's center.
             */
            tempLocation.subtractLocal(center);

            // Transform vertex coordinates to de-scaled shape coordinates.
            vertexToShape.transformVector(tempLocation, tempLocation);
            switch (shapeHeuristic) {
                case AABB:
                case Sphere:
                case VertexHull:
                    // Adjust the size of the shape by scaling the coordinates.
                    tempLocation.multLocal(shapeScale);
                    break;
                default:
            }
            buffer.reset();
            buffer.put(tempLocation.x);
            buffer.put(tempLocation.y);
            buffer.put(tempLocation.z);
        }

        CollisionShape result;
        switch (shapeHeuristic) {
            case AABB:
                Vector3f maxima = new Vector3f();
                Vector3f minima = new Vector3f();
                vertexLocations.maxMin(maxima, minima);
                RectangularSolid solid
                        = new RectangularSolid(minima, maxima, rotateIdentity);
                result = new HullCollisionShape(solid);
                break;

            case Cylinder:
                result = RagUtils.makeCylinder(vertexLocations, shapeScale);
                break;

            case FourSphere:
                solid = RagUtils
                        .makeRectangularSolid(vertexLocations, shapeScale);
                result = new MultiSphere(solid);
                break;

            case MinBox:
                solid = RagUtils
                        .makeRectangularSolid(vertexLocations, shapeScale);
                result = new HullCollisionShape(solid);
                break;

            case Sphere:
                float radius = vertexLocations.maxLength();
                result = new MultiSphere(radius);
                break;

            case TwoSphere:
                solid = RagUtils
                        .makeRectangularSolid(vertexLocations, shapeScale);
                result = new MultiSphere(solid, 0.5f);
                break;

            case VertexHull:
                result = new HullCollisionShape(buffer);
                break;

            default:
                String message = "heuristic = " + shapeHeuristic;
                throw new IllegalArgumentException(message);
        }

        return result;
    }

    /**
     * Calculate the mass, if it can be determined from the configuration alone.
     *
     * @return the mass (in physics units, &gt;0) or NaN if undetermined
     */
    public float mass() {
        float result = Float.NaN;
        if (massHeuristic == MassHeuristic.Mass) {
            assert massParameter > 0f : massParameter;
            result = massParameter;
        }

        return result;
    }

    /**
     * Calculate the mass for a PhysicsLink.
     *
     * @param volume the scaled volume of the CollisionShape (in cubic
     * physics-space units, &ge;0)
     * @return a mass value (in physics mass units, &gt;0)
     */
    float mass(float volume) {
        Validate.nonNegative(volume, "volume");

        float mass;
        switch (massHeuristic) {
            case Density:
                if (volume == 0f) {
                    mass = 1e-6f;
                } else {
                    mass = massParameter * volume;
                }
                break;

            case Mass:
                mass = massParameter;
                break;

            default:
                String message = "heuristic = " + massHeuristic;
                throw new IllegalArgumentException(message);
        }

        assert mass > 0f : mass;
        return mass;
    }

    /**
     * Read which mass heuristic to use.
     *
     * @return the enum value (not null)
     */
    public MassHeuristic massHeuristic() {
        assert massHeuristic != null;
        return massHeuristic;
    }

    /**
     * Read the parameter used to determine the mass.
     *
     * @return the parameter value (&gt;0)
     */
    public float massParameter() {
        assert massParameter > 0f : massParameter;
        return massParameter;
    }

    /**
     * Read the order in which axis rotations will be applied.
     *
     * @return the enum value or null
     */
    public RotationOrder rotationOrder() {
        return rotationOrder;
    }

    /**
     * Read which shape heuristic to use.
     *
     * @return the enum value (not null)
     */
    public ShapeHeuristic shapeHeuristic() {
        assert shapeHeuristic != null;
        return shapeHeuristic;
    }

    /**
     * Copy the scale factors used to adjust the size of the shape.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the scale factor for each axis (either storeResult or a new
     * instance)
     */
    public Vector3f shapeScale(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        result.set(shapeScale);
        return result;
    }
    // *************************************************************************
    // Comparable methods

    /**
     * Compare with another LinkConfig object.
     *
     * @param other (not null, unaffected)
     * @return 0 if the objects are equivalent; negative if this comes before
     * other; positive if this comes after other
     */
    @Override
    public int compareTo(LinkConfig other) {
        int result = centerHeuristic.compareTo(other.centerHeuristic());
        if (result != 0) {
            return result;
        }
        result = Float.compare(massParameter, other.massParameter());
        if (result != 0) {
            return result;
        }
        result = massHeuristic.compareTo(other.massHeuristic());
        if (result != 0) {
            return result;
        }

        RotationOrder otherOrder = other.rotationOrder();
        if (rotationOrder == null && otherOrder != null) {
            return 1;
        } else if (otherOrder == null && rotationOrder != null) {
            return -1;
        } else if (rotationOrder != null && otherOrder != null) {
            result = rotationOrder.compareTo(otherOrder);
            if (result != 0) {
                return result;
            }
        }

        result = shapeHeuristic.compareTo(other.shapeHeuristic());
        if (result != 0) {
            return result;
        }
        result = Float.compare(shapeScale.x, other.shapeScale.x);
        if (result != 0) {
            return result;
        }
        result = Float.compare(shapeScale.y, other.shapeScale.y);
        if (result != 0) {
            return result;
        }
        result = Float.compare(shapeScale.z, other.shapeScale.z);

        return result;
    }
    // *************************************************************************
    // Savable methods

    /**
     * De-serialize this configuration from the specified importer, for example
     * when loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        InputCapsule capsule = importer.getCapsule(this);

        this.centerHeuristic = capsule.readEnum(tagCenterHeuristic,
                CenterHeuristic.class, CenterHeuristic.Mean);
        this.massParameter = capsule.readFloat(tagMassParameter, 1f);
        this.massHeuristic = capsule.readEnum(
                tagMassHeuristic, MassHeuristic.class, MassHeuristic.Mass);
        this.rotationOrder
                = capsule.readEnum(tagRotationOrder, RotationOrder.class, null);
        this.shapeHeuristic = capsule.readEnum(tagShapeHeuristic,
                ShapeHeuristic.class, ShapeHeuristic.VertexHull);
        this.shapeScale = (Vector3f) capsule.readSavable(tagShapeScale, null);
    }

    /**
     * Serialize this configuration to the specified exporter, for example when
     * saving to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(
                centerHeuristic, tagCenterHeuristic, CenterHeuristic.Mean);
        capsule.write(massParameter, tagMassParameter, 1f);
        capsule.write(massHeuristic, tagMassHeuristic, MassHeuristic.Mass);
        capsule.write(rotationOrder, tagRotationOrder, null);
        capsule.write(
                shapeHeuristic, tagShapeHeuristic, ShapeHeuristic.VertexHull);
        capsule.write(shapeScale, tagShapeScale, null);
    }
    // *************************************************************************
    // Object methods

    /**
     * Test for exact equivalence with another Object.
     *
     * @param otherObject the object to compare to (may be null, unaffected)
     * @return true if the objects are equivalent, otherwise false
     */
    @Override
    public boolean equals(Object otherObject) {
        boolean result;
        if (otherObject == this) {
            result = true;
        } else if (otherObject != null
                && otherObject.getClass() == getClass()) {
            LinkConfig other = (LinkConfig) otherObject;
            float massP = other.massParameter();
            result = (centerHeuristic == other.centerHeuristic())
                    && (Float.compare(massParameter, massP) == 0)
                    && (massHeuristic == other.massHeuristic())
                    && (rotationOrder == other.rotationOrder())
                    && (shapeHeuristic == other.shapeHeuristic())
                    && shapeScale.equals(other.shapeScale);
        } else {
            result = false;
        }

        return result;
    }

    /**
     * Generate the hash code for this LinkConfig.
     *
     * @return value for use in hashing
     */
    @Override
    public int hashCode() {
        int hash = 17 + Objects.hashCode(centerHeuristic);
        hash = 11 * hash + Float.floatToIntBits(massParameter);
        hash = 11 * hash + Objects.hashCode(massHeuristic);
        hash = 11 * hash + Objects.hashCode(rotationOrder);
        hash = 11 * hash + Objects.hashCode(shapeHeuristic);
        hash = 11 * hash + Objects.hashCode(shapeScale);

        return hash;
    }
}
