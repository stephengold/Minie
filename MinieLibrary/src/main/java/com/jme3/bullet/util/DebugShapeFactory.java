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
import com.jme3.bullet.collision.shapes.PlaneCollisionShape;
import com.jme3.bullet.collision.shapes.infos.ChildCollisionShape;
import com.jme3.bullet.collision.shapes.infos.DebugMeshNormals;
import com.jme3.bullet.debug.DebugMeshInitListener;
import com.jme3.math.Matrix3f;
import com.jme3.math.Plane;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer;
import com.jme3.util.BufferUtils;
import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.util.Map;
import java.util.WeakHashMap;
import java.util.logging.Logger;
import jme3utilities.MyMesh;
import jme3utilities.Validate;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyVector3f;

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
     * number of axes
     */
    final private static int numAxes = 3;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(DebugShapeFactory.class.getName());
    /**
     * local copy of {@link com.jme3.math.Vector3f#ZERO}
     */
    final private static Vector3f translateIdentity = new Vector3f(0f, 0f, 0f);
    // *************************************************************************
    // fields

    /**
     * largest debug mesh to index (doesn't affect plane shapes, 0&rarr;never
     * index, MAX_VALUE&rarr;always index, default=6000 vertices)
     */
    private static int maxVerticesToIndex = 6_000;
    /**
     * map keys to previously generated debug meshes, for reuse
     */
    final private static Map<DebugMeshKey, Mesh> cache = new WeakHashMap<>(200);
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
     * Forget all previously generated debug meshes.
     */
    public static void clearCache() {
        cache.clear();
    }

    /**
     * Count how many debug meshes are cached.
     *
     * @return the count (&ge;0)
     */
    public static int countCachedMeshes() {
        int result = cache.size();
        return result;
    }

    /**
     * Estimate the footprint of the specified (non-compound, non-plane) shape.
     * The shape's scale and margin are taken into account, but not its
     * debug-mesh resolution.
     *
     * @param shape (not null, not compound, not plane, unaffected)
     * @param shapeToWorld the world transform of the collision object (not
     * null, unaffected)
     * @param meshResolution (0=low, 1=high)
     * @return a new array of corner locations (in world coordinates)
     */
    public static Vector3f[] footprint(CollisionShape shape,
            Transform shapeToWorld, int meshResolution) {
        assert !(shape instanceof CompoundCollisionShape);
        assert !(shape instanceof PlaneCollisionShape);
        Validate.nonNull(shapeToWorld, "shape-to-world");
        Validate.inRange(meshResolution, "mesh resolution", lowResolution,
                highResolution);

        long shapeId = shape.getObjectId();
        DebugMeshCallback callback = new DebugMeshCallback();
        getVertices2(shapeId, meshResolution, callback);
        Vector3f[] cornerLocations = callback.footprint(shapeToWorld);

        return cornerLocations;
    }

    /**
     * For compatibility with the jme3-bullet library.
     *
     * @param shape (not null, not compound, not plane, unaffected)
     * @return a new Mesh (not null)
     */
    public static Mesh getDebugMesh(CollisionShape shape) {
        long shapeId = shape.getObjectId();
        DebugMeshCallback callback = new DebugMeshCallback();
        getVertices2(shapeId, lowResolution, callback);
        FloatBuffer floatBuffer = callback.getVertices();

        Mesh result = new Mesh();
        result.setBuffer(VertexBuffer.Type.Position, numAxes, floatBuffer);

        return result;
    }

    /**
     * For compatibility with the jme3-bullet library.
     *
     * @param shape the shape to visualize (may be null, unaffected)
     * @return a new tree of geometries, or null
     */
    public static Spatial getDebugShape(CollisionShape shape) {
        Spatial result;
        DebugMeshInitListener noListener = null;
        if (shape == null) {
            result = null;

        } else if (shape instanceof CompoundCollisionShape) {
            CompoundCollisionShape compound = (CompoundCollisionShape) shape;
            result = createNode(compound, noListener, DebugMeshNormals.None,
                    lowResolution);

        } else {
            result = createGeometry(shape, noListener, DebugMeshNormals.None,
                    lowResolution);
        }

        return result;
    }

    /**
     * Create a Spatial for visualizing the specified collision object.
     * <p>
     * This is mostly used internally. To enable debug visualization, use
     * {@link com.jme3.bullet.BulletAppState#setDebugEnabled(boolean)}.
     *
     * @param pco the object to visualize (not null, unaffected)
     * @return a new tree of nodes and geometries, or null
     */
    public static Spatial getDebugShape(PhysicsCollisionObject pco) {
        CollisionShape shape = pco.getCollisionShape();
        DebugMeshInitListener listener = pco.debugMeshInitListener();
        DebugMeshNormals normals = pco.debugMeshNormals();
        int resolution = pco.debugMeshResolution();

        Spatial result;
        if (shape instanceof CompoundCollisionShape) {
            CompoundCollisionShape compound = (CompoundCollisionShape) shape;
            result = createNode(compound, listener, normals, resolution);
        } else {
            result = createGeometry(shape, listener, normals, resolution);
        }

        return result;
    }

    /**
     * Estimate how far the specified (non-compound, non-plane) shape extends
     * from some origin, based on its debug mesh. The shape's scale and margin
     * are taken into account, but not its debug-mesh resolution.
     *
     * @param shape (not null, not compound, not plane, unaffected)
     * @param transform the transform to apply to debug-mesh coordinates (not
     * null, unaffected)
     * @param meshResolution (0=low, 1=high)
     * @return the maximum length of the transformed vertex locations (&ge;0)
     */
    public static float maxDistance(CollisionShape shape, Transform transform,
            int meshResolution) {
        assert !(shape instanceof CompoundCollisionShape);
        assert !(shape instanceof PlaneCollisionShape);
        Validate.nonNull(transform, "transform");
        Validate.inRange(meshResolution, "mesh resolution", lowResolution,
                highResolution);

        long shapeId = shape.getObjectId();
        DebugMeshCallback callback = new DebugMeshCallback();
        getVertices2(shapeId, meshResolution, callback);
        float result = callback.maxDistance(transform);

        return result;
    }

    /**
     * Forget all previously generated debug meshes for the identified shape.
     *
     * @param shapeId the ID of the shape to remove
     */
    public static void removeShapeFromCache(long shapeId) {
        for (DebugMeshKey key : cache.keySet()) {
            if (key.shapeId() == shapeId) {
                cache.remove(key);
            }
        }
    }

    /**
     * Alter whether to index new debug meshes. (Doesn't affect cached meshes or
     * plane shapes.) Indexing might boost performance when there are many small
     * meshes; it isn't recommended for very large meshes.
     *
     * @param setting true&rarr;always index, false&rarr;never index
     */
    public static void setIndexBuffers(boolean setting) {
        maxVerticesToIndex = setting ? Integer.MAX_VALUE : -1;
    }

    /**
     * Alter whether to index new debug meshes. (Doesn't affect cached meshes or
     * plane shapes.) Indexing might boost performance when there are many small
     * meshes; it isn't recommended for very large meshes.
     *
     * @param maxVertices the largest mesh to be indexed (vertex count, &ge;-1,
     * default=6000)
     */
    public static void setIndexBuffers(int maxVertices) {
        Validate.inRange(maxVertices, "maxVertices", -1, Integer.MAX_VALUE);
        maxVerticesToIndex = maxVertices;
    }

    /**
     * Calculate the volume of a debug mesh for the specified convex shape. The
     * shape's scale and margin are taken into account, but not its debug-mesh
     * resolution.
     *
     * @param shape (not null, convex, unaffected)
     * @param meshResolution (0=low, 1=high)
     * @return the scaled volume (in physics-space units cubed, &ge;0)
     */
    public static float volumeConvex(CollisionShape shape, int meshResolution) {
        assert shape.isConvex();
        Validate.inRange(meshResolution, "mesh resolution", lowResolution,
                highResolution);

        long shapeId = shape.getObjectId();
        DebugMeshCallback callback = new DebugMeshCallback();
        getVertices2(shapeId, meshResolution, callback);
        float volume = callback.volumeConvex();

        assert volume >= 0f : volume;
        return volume;
    }
    // *************************************************************************
    // private methods

    /**
     * Create a Geometry for visualizing the specified (non-compound) collision
     * shape.
     *
     * @param shape (not null, not compound, unaffected)
     * @param normals which normals to generate (not null)
     * @param resolution how much detail for convex shapes (0=low, 1=high)
     * @return a new Geometry (not null)
     */
    private static Geometry createGeometry(CollisionShape shape,
            DebugMeshInitListener listener, DebugMeshNormals normals,
            int resolution) {
        assert shape != null;
        assert !(shape instanceof CompoundCollisionShape);
        assert normals != null;
        assert resolution >= lowResolution : resolution;
        assert resolution <= highResolution : resolution;

        DebugMeshKey key = new DebugMeshKey(shape, normals, resolution);
        Mesh mesh = cache.get(key);
        if (mesh == null) {
            if (shape instanceof PlaneCollisionShape) {
                mesh = createPlaneMesh((PlaneCollisionShape) shape, normals);
            } else {
                mesh = createMesh(shape, normals, resolution);
            }
            if (listener != null) {
                listener.debugMeshInit(mesh);
            }
            cache.put(key, mesh);
        }

        Geometry geometry = new Geometry("Bullet debug", mesh);
        geometry.updateModelBound();

        return geometry;
    }

    /**
     * Create a Mesh for visualizing the specified (non-compound, non-plane)
     * collision shape.
     *
     * @param shape (not null, not compound, not plane, unaffected)
     * @param normals which normals to generate (not null)
     * @param resolution how much detail for convex shapes (0=low, 1=high)
     * @return a new Mesh (not null)
     */
    private static Mesh createMesh(CollisionShape shape,
            DebugMeshNormals normals, int resolution) {
        assert resolution >= lowResolution : resolution;
        assert resolution <= highResolution : resolution;

        long shapeId = shape.getObjectId();
        DebugMeshCallback callback = new DebugMeshCallback();
        getVertices2(shapeId, resolution, callback);

        Mesh mesh = new Mesh();
        mesh.setBuffer(VertexBuffer.Type.Position, numAxes,
                callback.getVertices());
        /*
         * Add a normal buffer, if requested.
         */
        switch (normals) {
            case Facet:
                mesh.setBuffer(VertexBuffer.Type.Normal, numAxes,
                        callback.getFaceNormals());
                break;
            case None:
                break;
            case Smooth:
                mesh.setBuffer(VertexBuffer.Type.Normal, numAxes,
                        callback.getFaceNormals());
                MyMesh.smoothNormals(mesh);
                break;
        }
        /*
         * If the mesh is not too big, generate an index buffer.
         */
        if (mesh.getVertexCount() <= maxVerticesToIndex) {
            mesh = MyMesh.addIndices(mesh);
        }

        mesh.updateBound();
        mesh.setStatic();

        return mesh;
    }

    /**
     * Create a Node for visualizing the specified CompoundCollisionShape.
     *
     * @param compoundShape (not null, unaffected)
     * @param normals which normals to generate (not null)
     * @param resolution how much detail for convex child shapes (0=low, 1=high)
     * @return a new Node (not null)
     */
    private static Node createNode(CompoundCollisionShape compoundShape,
            DebugMeshInitListener listener, DebugMeshNormals normals,
            int resolution) {
        assert normals != null;
        assert resolution >= lowResolution : resolution;
        assert resolution <= highResolution : resolution;

        Node node = new Node("Bullet debug");

        Vector3f scale = compoundShape.getScale(null);
        Matrix3f tmpRotation = new Matrix3f(); // TODO garbage
        Vector3f tmpOffset = new Vector3f();
        ChildCollisionShape[] children = compoundShape.listChildren();
        for (ChildCollisionShape child : children) {
            CollisionShape childShape = child.getShape();
            Geometry geometry = createGeometry(childShape, listener, normals,
                    resolution);

            // apply scaled offset
            child.copyOffset(tmpOffset);
            tmpOffset.multLocal(scale);
            geometry.setLocalTranslation(tmpOffset);

            // apply rotation
            child.copyRotationMatrix(tmpRotation);
            geometry.setLocalRotation(tmpRotation);

            node.attachChild(geometry);
        }
        node.updateGeometricState();

        return node;
    }

    /**
     * Create a Mesh for visualizing the specified PlaneCollisionShape.
     *
     * @param shape (not null, unaffected)
     * @param normals which normals to generate (not null)
     * @return a new Triangles-mode Mesh (not null)
     */
    private static Mesh createPlaneMesh(PlaneCollisionShape shape,
            DebugMeshNormals normals) {
        /*
         * Transform from the Y-Z plane to the surface of the shape.
         */
        Transform transform = new Transform();
        transform.setScale(1000f);

        Plane plane = shape.getPlane();
        plane.getClosestPoint(translateIdentity, transform.getTranslation());

        Vector3f v1 = plane.getNormal();
        Vector3f v2 = new Vector3f();
        Vector3f v3 = new Vector3f();
        MyVector3f.generateBasis(v1, v2, v3);
        transform.getRotation().fromAxes(v1, v2, v3);
        /*
         * Generate mesh positions for a large 2-sided square.
         */
        int numVertices = 8; // 4 vertices for each size
        int numFloats = numVertices * numAxes;
        FloatBuffer posBuffer = BufferUtils.createFloatBuffer(numFloats);
        for (int sideIndex = 0; sideIndex < 2; ++sideIndex) {
            posBuffer.put(new float[]{
                0f, 0f, 1f,
                0f, 1f, 0f,
                0f, 0f, -1f,
                0f, -1f, 0f});
        }
        assert posBuffer.position() == numFloats;
        posBuffer.flip();

        MyBuffer.transform(posBuffer, 0, numFloats, transform);
        /*
         * Generate an index buffer for a 2-sided square.
         */
        ByteBuffer indexBuffer = BufferUtils.createByteBuffer(new byte[]{
            2, 1, 0,
            3, 2, 0,
            5, 6, 7,
            4, 5, 7});
        int numBytes = indexBuffer.capacity();
        indexBuffer.limit(numBytes);

        Mesh mesh = new Mesh();
        mesh.setBuffer(VertexBuffer.Type.Position, numAxes, posBuffer);
        mesh.setBuffer(VertexBuffer.Type.Index, MyMesh.vpt, indexBuffer);
        /*
         * Add a normal buffer, if requested.
         */
        if (normals != DebugMeshNormals.None) {
            FloatBuffer normBuffer = BufferUtils.createFloatBuffer(numFloats);
            for (int i = 0; i < 4; ++i) {
                normBuffer.put(v1.x).put(v1.y).put(v1.z);
            }
            for (int i = 0; i < 4; ++i) {
                normBuffer.put(-v1.x).put(-v1.y).put(-v1.z);
            }
            normBuffer.flip();

            mesh.setBuffer(VertexBuffer.Type.Normal, numAxes, normBuffer);
        }

        mesh.updateBound();
        mesh.setStatic();

        return mesh;
    }
    // *************************************************************************
    // native methods

    // TODO add a method to return vertices only (instead of a triangle mesh)
    native private static void getVertices2(long shapeId, int meshResolution,
            DebugMeshCallback buffer);
}
