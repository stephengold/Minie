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
package com.jme3.bullet.animation;

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
import java.util.Collection;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.RectangularSolid;
import jme3utilities.minie.MyShape;

/**
 * Configuration data for a PhysicsLink, not including the bone name, attached
 * model, or RangeOfMotion. Immutable except for
 * {@link #read(com.jme3.export.JmeImporter)}.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class LinkConfig implements Savable {
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
        centerHeuristic = CenterHeuristic.Mean;
        massParameter = 1f;
        massHeuristic = MassHeuristic.Mass;
        shapeHeuristic = ShapeHeuristic.VertexHull;
        shapeScale = new Vector3f(1f, 1f, 1f);
    }

    /**
     * Instantiate a configuration for a link with the specified mass, centered
     * on the unweighted average of the vertex locations, shaped to the convex
     * hull of the vertices.
     *
     * @param mass (&gt;0)
     */
    public LinkConfig(float mass) {
        Validate.positive(mass, "mass");

        centerHeuristic = CenterHeuristic.Mean;
        massParameter = mass;
        massHeuristic = MassHeuristic.Mass;
        shapeHeuristic = ShapeHeuristic.VertexHull;
        shapeScale = new Vector3f(1f, 1f, 1f);
    }

    /**
     * Instantiate a configuration for a link with the specified mass, but with
     * the centering and shaping of the specified configuration.
     *
     * @param mass (&gt;0)
     * @param oldConfig (not null)
     */
    public LinkConfig(float mass, LinkConfig oldConfig) {
        Validate.positive(mass, "mass");
        Validate.nonNull(oldConfig, "old configuration");

        centerHeuristic = oldConfig.centerHeuristic();
        massParameter = mass;
        massHeuristic = MassHeuristic.Mass;
        shapeHeuristic = oldConfig.shapeHeuristic();
        shapeScale = oldConfig.shapeScale(null);
    }

    /**
     * Instantiate a configuration for a fully custom link.
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
        Validate.nonNull(massH, "mass heruristic");
        Validate.nonNull(shapeH, "shape heuristic");
        Validate.nonNegative(sScale, "shape scale");
        Validate.nonNull(centerH, "center heuristic");

        centerHeuristic = centerH;
        massParameter = massParm;
        massHeuristic = massH;
        shapeHeuristic = shapeH;
        shapeScale = sScale.clone();
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
     * Create a collision shape for the specified transform, center, and
     * collection of vertex locations.
     *
     * @param vertexToShape the transform from vertex coordinates to de-scaled
     * shape coordinates (not null, unaffected)
     * @param center the location of the shape's center, in vertex coordinates
     * (not null, unaffected)
     * @param vertexLocations the collection of vertex locations (not null, not
     * empty, MODIFIED)
     * @return a new collision shape
     */
    public CollisionShape createShape(Transform vertexToShape, Vector3f center,
            Collection<Vector3f> vertexLocations) {
        Validate.nonNull(vertexToShape, "transform");
        Validate.finite(center, "center");
        Validate.nonEmpty(vertexLocations, "vertex locations");

        for (Vector3f location : vertexLocations) {
            /*
             * Translate so that vertex coordinates are relative to
             * the shape's center.
             */
            location.subtractLocal(center);
            /*
             * Transform vertex coordinates to de-scaled shape coordinates.
             */
            vertexToShape.transformVector(location, location);
            /*
             * Adjust the size of the shape.
             */
            location.multLocal(shapeScale);
        }

        CollisionShape result;
        switch (shapeHeuristic) {
            case AABB:
                Vector3f maxima = new Vector3f(Float.NEGATIVE_INFINITY,
                        Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY);
                Vector3f minima = new Vector3f(Float.POSITIVE_INFINITY,
                        Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY);
                for (Vector3f location : vertexLocations) {
                    MyVector3f.accumulateMinima(minima, location);
                    MyVector3f.accumulateMaxima(maxima, location);
                }
                RectangularSolid solid
                        = new RectangularSolid(minima, maxima, rotateIdentity);
                result = new HullCollisionShape(solid);
                break;

            case FourSphere:
                solid = new RectangularSolid(vertexLocations);
                result = new MultiSphere(solid);
                break;

            case MinBox:
                solid = new RectangularSolid(vertexLocations);
                result = new HullCollisionShape(solid);
                break;

            case Sphere:
                float radius = RagUtils.maxDistance(vertexLocations);
                result = new MultiSphere(radius);
                break;

            case VertexHull:
                result = new HullCollisionShape(vertexLocations);
                break;

            default:
                String message = "heuristic = " + shapeHeuristic.toString();
                throw new IllegalArgumentException(message);
        }

        return result;
    }

    /**
     * Calculate the mass, if it can be determined from the configuration alone.
     *
     * @return the mass (&gt;0) or NaN if undetermined
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
     * Calculate a mass for a physics link.
     *
     * @param shape the link's collision shape (not null, unaffected)
     * @return a mass value (&gt;0)
     */
    public float mass(CollisionShape shape) {
        Validate.nonNull(shape, "shape");

        float mass;
        switch (massHeuristic) {
            case Density:
                float scaledVolume = MyShape.volume(shape);
                mass = massParameter * scaledVolume;
                break;
            case Mass:
                mass = massParameter;
                break;
            default:
                String message = "heuristic = " + massHeuristic.toString();
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
    // Savable methods

    /**
     * De-serialize this configuration, for example when loading from a J3O
     * file.
     *
     * @param im importer (not null)
     * @throws IOException from importer
     */
    @Override
    public void read(JmeImporter im) throws IOException {
        InputCapsule ic = im.getCapsule(this);

        centerHeuristic = ic.readEnum("centerHeuristic", CenterHeuristic.class,
                CenterHeuristic.Mean);
        massParameter = ic.readFloat("massParameter", 1f);
        massHeuristic = ic.readEnum("massHeuristic", MassHeuristic.class,
                MassHeuristic.Mass);
        shapeHeuristic = ic.readEnum("shapeHeuristic", ShapeHeuristic.class,
                ShapeHeuristic.VertexHull);
        shapeScale = (Vector3f) ic.readSavable("shapeScale", null);
    }

    /**
     * Serialize this configuration, for example when saving to a J3O file.
     *
     * @param ex exporter (not null)
     * @throws IOException from exporter
     */
    @Override
    public void write(JmeExporter ex) throws IOException {
        OutputCapsule oc = ex.getCapsule(this);

        oc.write(centerHeuristic, "centerHeuristic", CenterHeuristic.Mean);
        oc.write(massParameter, "massParameter", 1f);
        oc.write(massHeuristic, "massHeuristic", MassHeuristic.Mass);
        oc.write(shapeHeuristic, "shapeHeuristic", ShapeHeuristic.VertexHull);
        oc.write(shapeScale, "shapeScale", null);
    }
}
