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
package com.jme3.bullet.util;

import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.infos.ChildCollisionShape;
import com.jme3.bullet.collision.shapes.infos.DebugMeshNormals;
import com.jme3.math.Matrix3f;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer.Type;
import java.util.List;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A utility class to generate debug meshes for Bullet collision shapes.
 *
 * @author CJ Hare, normenhansen
 */
public class DebugShapeFactory {
    // *************************************************************************
    // constants and loggers

    /**
     * specify high-res debug mesh for convex shapes (up to 256 vertices)
     */
    public static final int highResolution = 1;
    /**
     * specify low-res debug mesh for convex shapes (up to 42 vertices)
     */
    public static final int lowResolution = 0;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(DebugShapeFactory.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private DebugShapeFactory() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Create a spatial for visualizing the specified collision object.
     * <p>
     * This is mostly used internally. To enable physics debug visualization,
     * use {@link com.jme3.bullet.BulletAppState#setDebugEnabled(boolean)}.
     *
     * @param pco the object to visualize (not null, unaffected)
     * @return a new tree of nodes and geometries, or null
     */
    public static Spatial getDebugShape(PhysicsCollisionObject pco) {
        Validate.nonNull(pco, "collision object");

        CollisionShape collisionShape = pco.getCollisionShape();
        DebugMeshNormals normals = pco.debugMeshNormals();
        int resolution = pco.debugMeshResolution();

        Spatial result;
        if (collisionShape instanceof CompoundCollisionShape) {
            CompoundCollisionShape parent
                    = (CompoundCollisionShape) collisionShape;

            Node node = new Node("Bullet debug");

            List<ChildCollisionShape> children = parent.getChildren();
            for (ChildCollisionShape child : children) {
                CollisionShape simpleShape = child.getShape();
                Geometry geometry
                        = createGeometry(simpleShape, normals, resolution);

                // apply translation
                Vector3f translation = child.getLocation(null);
                geometry.setLocalTranslation(translation);

                // apply rotation
                Matrix3f rotation = child.getRotation(null);
                geometry.setLocalRotation(rotation);

                node.attachChild(geometry);
            }
            result = node;

        } else {  // not a compound shape
            result = createGeometry(collisionShape, normals, resolution);
        }

        result.updateGeometricState();

        return result;
    }

    /**
     * Calculate the volume of a debug mesh for the specified convex shape. The
     * shape's current scale and margin are taken into account, but not its
     * debug-mesh resolution.
     *
     * @param shape (not null, unaffected, must be convex)
     * @param meshResolution (0=low, 1=high)
     * @return the scaled volume (in physics-space units cubed, &ge;0)
     */
    public static float volumeConvex(CollisionShape shape, int meshResolution) {
        assert !shape.isConcave();
        Validate.inRange(meshResolution, "mesh resolution", 0, 1);

        long id = shape.getObjectId();
        DebugMeshCallback callback = new DebugMeshCallback();
        getVertices2(id, meshResolution, callback);
        float volume = callback.volumeConvex();

        assert volume >= 0f : volume;
        return volume;
    }
    // *************************************************************************
    // private methods

    /**
     * Create a geometry for visualizing the specified (non-compound) collision
     * shape.
     *
     * @param shape (not null, unaffected)
     * @param normals which normals to generate (not null)
     * @param resolution how much detail for convex shapes (0=low, 1=high)
     * @return a new geometry (not null)
     */
    private static Geometry createGeometry(CollisionShape shape,
            DebugMeshNormals normals, int resolution) {
        assert shape != null;
        assert !(shape instanceof CompoundCollisionShape);
        assert normals != null;
        assert resolution >= 0 : resolution;
        assert resolution <= 1 : resolution;

        Mesh mesh = createMesh(shape, normals, resolution);
        Geometry geometry = new Geometry("Bullet debug", mesh);
        geometry.updateModelBound();

        return geometry;
    }

    /**
     * Create a mesh for visualizing the specified (non-compound) collision
     * shape.
     *
     * @param shape (not null, unaffected)
     * @param normals which normals to generate (not null)
     * @param resolution how much detail for convex shapes (0=low, 1=high)
     * @return a new mesh (not null)
     */
    private static Mesh createMesh(CollisionShape shape,
            DebugMeshNormals normals, int resolution) {
        long id = shape.getObjectId();
        DebugMeshCallback callback = new DebugMeshCallback();
        getVertices2(id, resolution, callback);

        Mesh mesh = new Mesh();
        mesh.setBuffer(Type.Position, 3, callback.getVertices());
        /*
         * Add a normal buffer, if requested.
         */
        switch (normals) {
            case Facet:
                mesh.setBuffer(Type.Normal, 3, callback.getFaceNormals());
                break;
            case None:
                break;
            case Smooth:
                mesh.setBuffer(Type.Normal, 3, callback.getSmoothNormals());
                break;
        }

        return mesh;
    }

    private static native void getVertices2(long shapeId, int meshResolution,
            DebugMeshCallback buffer);
}
