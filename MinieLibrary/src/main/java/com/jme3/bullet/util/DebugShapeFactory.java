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
import com.jme3.bullet.collision.shapes.ConvexShape;
import com.jme3.bullet.collision.shapes.PlaneCollisionShape;
import com.jme3.bullet.collision.shapes.infos.ChildCollisionShape;
import com.jme3.bullet.debug.DebugMeshInitListener;
import com.jme3.bullet.debug.MeshCustomizer;
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
import java.util.Collections;
import java.util.Map;
import java.util.WeakHashMap;
import java.util.logging.Logger;
import jme3utilities.MeshNormals;
import jme3utilities.MyMesh;
import jme3utilities.Validate;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;

/**
 * A utility class to generate debug meshes for Bullet collision shapes.
 *
 * @author CJ Hare, normenhansen
 */
final public class DebugShapeFactory {
    // *************************************************************************
    // constants and loggers

    /**
     * side length of the (square) debug mesh for a PlaneCollisionShape (in mesh
     * units)
     */
    final private static float planeDebugMeshSideLength = 1_500f;
    /**
     * specify high-res debug meshes for convex shapes (up to 256 vertices)
     */
    final public static int highResolution = 1;
    /**
     * specify low-res debug meshes for convex shapes (up to 42 vertices)
     */
    final public static int lowResolution = 0;
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
     * index, MAX_VALUE&rarr;always index)
     */
    private static int maxVerticesToIndex = 6_000;
    /**
     * map keys to previously generated debug meshes, for reuse
     *
     * Synchronized so that it can be updated from the "Physics Cleaner" thread.
     */
    final private static Map<DebugMeshKey, Mesh> cache
            = Collections.synchronizedMap(
                    new WeakHashMap<DebugMeshKey, Mesh>(200));
    /**
     * customization applied to all generated meshes, or null for none
     */
    private static MeshCustomizer meshCustomizer;
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
     * Determine vertex locations for the specified collision shape. Note:
     * recursive!
     *
     * @param shape the input shape (not null, unaffected)
     * @param meshResolution (0=low, 1=high)
     * @return a new, unflipped, direct buffer full of scaled shape coordinates
     * (capacity a multiple of 3)
     */
    public static FloatBuffer debugVertices(
            CollisionShape shape, int meshResolution) {
        Validate.nonNull(shape, "shape");
        Validate.inRange(meshResolution, "mesh resolution", lowResolution,
                highResolution);

        FloatBuffer result;
        if (shape instanceof CompoundCollisionShape) {
            CompoundCollisionShape ccs = (CompoundCollisionShape) shape;
            result = createCompoundVertices(ccs, meshResolution);

        } else if (shape instanceof PlaneCollisionShape) {
            float halfExt = 1000f;
            result = createPlaneVertices((PlaneCollisionShape) shape, halfExt);

        } else {
            long shapeId = shape.nativeId();
            DebugMeshCallback callback = new DebugMeshCallback();
            getVertices(shapeId, meshResolution, callback);
            result = callback.getVertices();
        }

        assert (result.capacity() % numAxes) == 0 : result.capacity();
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
    public static Vector3f[] footprint(
            CollisionShape shape, Transform shapeToWorld, int meshResolution) {
        assert !(shape instanceof CompoundCollisionShape);
        assert !(shape instanceof PlaneCollisionShape);
        Validate.nonNull(shapeToWorld, "shape-to-world");
        Validate.inRange(meshResolution, "mesh resolution", lowResolution,
                highResolution);

        long shapeId = shape.nativeId();
        DebugMeshCallback callback = new DebugMeshCallback();
        getVertices(shapeId, meshResolution, callback);
        Vector3f[] cornerLocations = callback.footprint(shapeToWorld);

        return cornerLocations;
    }

    /**
     * For compatibility with the jme3-jbullet library.
     *
     * @param shape the shape to visualize (not null, not compound, not plane,
     * unaffected)
     * @return a new Triangles-mode Mesh (no indices, no normals)
     */
    public static Mesh getDebugMesh(CollisionShape shape) {
        Validate.nonNull(shape, "shape");

        FloatBuffer floatBuffer = getDebugTriangles(shape, lowResolution);
        Mesh result = new Mesh();
        result.setBuffer(VertexBuffer.Type.Position, numAxes, floatBuffer);
        result.updateBound();

        return result;
    }

    /**
     * For compatibility with the jme3-jbullet library.
     *
     * @param shape the shape to visualize (may be null, unaffected)
     * @return a new Spatial or null
     */
    public static Spatial getDebugShape(CollisionShape shape) {
        Spatial result;
        DebugMeshInitListener noListener = null;
        if (shape == null) {
            result = null;

        } else if (shape instanceof CompoundCollisionShape) {
            result = createNode((CompoundCollisionShape) shape, noListener,
                    MeshNormals.None, lowResolution);

        } else {
            result = createGeometry(
                    shape, noListener, MeshNormals.None, lowResolution);
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
        MeshNormals normals = pco.debugMeshNormals();
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
     * Generate vertex locations for triangles to visualize the specified
     * collision shape. Note: recursive!
     *
     * @param shape the shape to visualize (not null, unaffected)
     * @param meshResolution (0=low, 1=high)
     * @return a new, unflipped, direct buffer full of scaled shape coordinates
     * (capacity a multiple of 9)
     */
    public static FloatBuffer getDebugTriangles(
            CollisionShape shape, int meshResolution) {
        Validate.nonNull(shape, "shape");
        Validate.inRange(meshResolution, "mesh resolution", lowResolution,
                highResolution);

        FloatBuffer result;
        if (shape instanceof CompoundCollisionShape) {
            CompoundCollisionShape ccs = (CompoundCollisionShape) shape;
            result = createCompoundTriangles(ccs, meshResolution);

        } else if (shape instanceof PlaneCollisionShape) {
            float halfExt = 1000f;
            result = createPlaneTriangles((PlaneCollisionShape) shape, halfExt);

        } else {
            long shapeId = shape.nativeId();
            DebugMeshCallback callback = new DebugMeshCallback();
            getTriangles(shapeId, meshResolution, callback);
            result = callback.getVertices();
        }

        assert (result.capacity() % 9) == 0 : result.capacity();
        return result;
    }

    /**
     * Install the specified MeshCustomizer, replacing any customizer previously
     * installed.
     *
     * @param customizer the desired customizer, or null for none
     */
    public static void installMeshCustomizer(MeshCustomizer customizer) {
        meshCustomizer = customizer;
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

        long shapeId = shape.nativeId();
        DebugMeshCallback callback = new DebugMeshCallback();
        getVertices(shapeId, meshResolution, callback);
        float result = callback.maxDistance(transform);

        return result;
    }

    /**
     * Determine the side length of the (square) debug mesh for a
     * PlaneCollisionShape.
     *
     * @return the length (in mesh units, &gt;0)
     */
    static float meshSideLength() {
        float result = planeDebugMeshSideLength;
        return result;
    }

    /**
     * Forget all previously generated debug meshes for the identified shape.
     *
     * @param shapeId the ID of the shape to remove
     */
    public static void removeShapeFromCache(long shapeId) {
        synchronized (cache) {
            for (DebugMeshKey key : cache.keySet()) {
                if (key.shapeId() == shapeId) {
                    cache.remove(key);
                }
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
    public static float volumeConvex(ConvexShape shape, int meshResolution) {
        Validate.inRange(meshResolution, "mesh resolution", lowResolution,
                highResolution);

        long shapeId = shape.nativeId();
        DebugMeshCallback callback = new DebugMeshCallback();
        getTriangles(shapeId, meshResolution, callback);
        float volume = callback.volumeConvex();

        assert volume >= 0f : volume;
        return volume;
    }
    // *************************************************************************
    // private methods

    /**
     * Generate vertex locations for triangles to visualize the specified
     * CompoundCollisionShape.
     *
     * @param compoundShape (not null, unaffected)
     * @param meshResolution (0=low, 1=high)
     *
     * @return a new, unflipped, direct buffer full of scaled shape coordinates
     * (capacity a multiple of 9)
     */
    private static FloatBuffer createCompoundTriangles(
            CompoundCollisionShape compoundShape, int meshResolution) {
        ChildCollisionShape[] children = compoundShape.listChildren();
        int numChildren = children.length;

        FloatBuffer[] bufferArray = new FloatBuffer[numChildren];
        Transform tmpTransform = new Transform();
        int totalFloats = 0;

        for (int childIndex = 0; childIndex < numChildren; ++childIndex) {
            ChildCollisionShape child = children[childIndex];
            CollisionShape baseShape = child.getShape();
            child.copyTransform(tmpTransform);
            FloatBuffer buffer = getDebugTriangles(baseShape, meshResolution);

            int numFloats = buffer.capacity();
            MyBuffer.transform(buffer, 0, numFloats, tmpTransform);
            bufferArray[childIndex] = buffer;
            totalFloats += numFloats;
        }

        FloatBuffer result = BufferUtils.createFloatBuffer(totalFloats);
        for (FloatBuffer buffer : bufferArray) {
            for (int position = 0; position < buffer.capacity(); ++position) {
                float value = buffer.get(position);
                result.put(value);
            }
        }
        assert result.position() == result.capacity();

        return result;
    }

    /**
     * Determine vertex locations for the specified CompoundCollisionShape.
     *
     * @param compoundShape (not null, unaffected)
     * @param meshResolution (0=low, 1=high)
     *
     * @return a new, unflipped, direct buffer full of scaled shape coordinates
     * (capacity a multiple of 3)
     */
    private static FloatBuffer createCompoundVertices(
            CompoundCollisionShape compoundShape, int meshResolution) {
        ChildCollisionShape[] children = compoundShape.listChildren();
        int numChildren = children.length;

        FloatBuffer[] bufferArray = new FloatBuffer[numChildren];
        Transform tmpTransform = new Transform();
        int totalFloats = 0;

        for (int childIndex = 0; childIndex < numChildren; ++childIndex) {
            ChildCollisionShape child = children[childIndex];
            CollisionShape baseShape = child.getShape();
            child.copyTransform(tmpTransform);
            FloatBuffer buffer = debugVertices(baseShape, meshResolution);

            int numFloats = buffer.capacity();
            MyBuffer.transform(buffer, 0, numFloats, tmpTransform);
            bufferArray[childIndex] = buffer;
            totalFloats += numFloats;
        }

        FloatBuffer result = BufferUtils.createFloatBuffer(totalFloats);
        for (FloatBuffer buffer : bufferArray) {
            for (int position = 0; position < buffer.capacity(); ++position) {
                float value = buffer.get(position);
                result.put(value);
            }
        }
        assert result.position() == result.capacity();

        return result;
    }

    /**
     * Create a Geometry for visualizing the specified (non-compound) collision
     * shape.
     *
     * @param shape (not null, not compound, unaffected)
     * @param listener (may be null)
     * @param normals which normals to generate (not null)
     * @param resolution how much detail for convex shapes (0=low, 1=high)
     * @return a new Geometry (not null)
     */
    private static Geometry createGeometry(CollisionShape shape,
            DebugMeshInitListener listener, MeshNormals normals,
            int resolution) {
        assert shape != null;
        assert !(shape instanceof CompoundCollisionShape);
        assert normals != null;
        assert resolution >= lowResolution : resolution;
        assert resolution <= highResolution : resolution;

        DebugMeshKey key = new DebugMeshKey(shape, normals, resolution);
        Mesh mesh;
        synchronized (cache) {
            mesh = cache.get(key);
            if (mesh == null) {
                if (shape instanceof PlaneCollisionShape) {
                    mesh = createPlaneMesh(
                            (PlaneCollisionShape) shape, normals);
                } else {
                    mesh = createMesh(shape, normals, resolution);
                }
                if (meshCustomizer != null) {
                    mesh = meshCustomizer.customizeMesh(mesh);
                }
                if (listener != null) {
                    listener.debugMeshInit(mesh);
                }
                cache.put(key, mesh);
            }
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
    private static Mesh createMesh(
            CollisionShape shape, MeshNormals normals, int resolution) {
        assert resolution >= lowResolution : resolution;
        assert resolution <= highResolution : resolution;

        long shapeId = shape.nativeId();
        DebugMeshCallback callback = new DebugMeshCallback();
        getTriangles(shapeId, resolution, callback);

        Mesh mesh = new Mesh();
        mesh.setBuffer(
                VertexBuffer.Type.Position, numAxes, callback.getVertices());

        // Add a normal buffer, if requested.
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
            case Sphere:
                MyMesh.addSphereNormals(mesh);
                break;
            default:
                String message = "normals = " + normals;
                throw new IllegalArgumentException(message);
        }

        // If the mesh is not too big, generate an index buffer.
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
     * @param listener (may be null)
     * @param normals which normals to generate (not null)
     * @param resolution how much detail for convex child shapes (0=low, 1=high)
     * @return a new Node (not null)
     */
    private static Node createNode(CompoundCollisionShape compoundShape,
            DebugMeshInitListener listener, MeshNormals normals,
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
            CollisionShape baseShape = child.getShape();
            Geometry geometry
                    = createGeometry(baseShape, listener, normals, resolution);

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
     * @return a new, indexed, Triangles-mode Mesh
     */
    private static Mesh createPlaneMesh(
            PlaneCollisionShape shape, MeshNormals normals) {
        // Generate mesh positions for a large 2-sided square.
        int numVertices = 8; // 4 vertices for each size
        int numFloats = numVertices * numAxes;
        FloatBuffer posBuffer = BufferUtils.createFloatBuffer(numFloats);
        for (int sideIndex = 0; sideIndex < 2; ++sideIndex) {
            posBuffer.put(new float[]{
                0f, 0f, 1f,
                0f, 1f, 0f,
                0f, 0f, -1f,
                0f, -1f, 0f
            });
        }
        assert posBuffer.position() == numFloats;
        posBuffer.flip();

        // Transform mesh positions to the surface of the CollisionShape.
        Transform transform = planeTransform(shape);
        float scale = meshSideLength() / MyMath.root2;
        transform.setScale(scale);
        MyBuffer.transform(posBuffer, 0, numFloats, transform);

        // Generate an index buffer for a 2-sided square.
        ByteBuffer indexBuffer = BufferUtils.createByteBuffer(new byte[]{
            2, 1, 0,
            3, 2, 0,
            5, 6, 7,
            4, 5, 7});
        indexBuffer.clear();

        Mesh result = new Mesh();
        result.setBuffer(VertexBuffer.Type.Position, numAxes, posBuffer);
        result.setBuffer(VertexBuffer.Type.Index, MyMesh.vpt, indexBuffer);

        // Add a normal buffer, if requested.
        if (normals != MeshNormals.None) {
            Plane plane = shape.getPlane();
            Vector3f v1 = plane.getNormal();

            FloatBuffer normBuffer = BufferUtils.createFloatBuffer(numFloats);
            for (int i = 0; i < 4; ++i) {
                normBuffer.put(v1.x).put(v1.y).put(v1.z);
            }
            for (int i = 0; i < 4; ++i) {
                normBuffer.put(-v1.x).put(-v1.y).put(-v1.z);
            }
            normBuffer.flip();

            result.setBuffer(VertexBuffer.Type.Normal, numAxes, normBuffer);
        }

        result.updateBound();
        result.setStatic();

        return result;
    }

    /**
     * Generate vertex locations for triangles to visualize the specified
     * PlaneCollisionShape.
     *
     * @param shape (not null, unaffected)
     * @param halfExtent the desired half extent for the result (in scaled shape
     * units, &gt;0)
     * @return a new, unflipped, direct buffer full of scaled shape coordinates
     * (capacity a multiple of 9)
     */
    private static FloatBuffer
            createPlaneTriangles(PlaneCollisionShape shape, float halfExtent) {
        assert shape != null;
        assert halfExtent > 0f : halfExtent;

        // Generate vertex locations for a large square in the Y-Z plane.
        FloatBuffer result = BufferUtils.createFloatBuffer(
                0f, 0f, -1f,
                0f, 1f, 0f,
                0f, 0f, 1f,
                0f, -1f, 0f,
                0f, 0f, -1f,
                0f, 0f, 1f
        );
        int numFloats = result.capacity();

        // Transform vertex locations to the surface of the shape.
        Transform transform = planeTransform(shape);
        transform.setScale(halfExtent);
        MyBuffer.transform(result, 0, numFloats, transform);

        return result;
    }

    /**
     * Generate vertex locations for the specified PlaneCollisionShape.
     *
     * @param shape (not null, unaffected)
     * @param halfExtent the desired half extent for the result (in scaled shape
     * units, &gt;0)
     * @return a new, unflipped, direct buffer full of scaled shape coordinates
     * (capacity a multiple of 3)
     */
    private static FloatBuffer
            createPlaneVertices(PlaneCollisionShape shape, float halfExtent) {
        assert shape != null;
        assert halfExtent > 0f : halfExtent;

        // Generate vertex locations for a large square in the Y-Z plane.
        FloatBuffer result = BufferUtils.createFloatBuffer(
                0f, 0f, -1f,
                0f, 1f, 0f,
                0f, 0f, 1f,
                0f, -1f, 0f
        );
        int numFloats = result.capacity();

        // Transform vertex locations to the surface of the shape.
        Transform transform = planeTransform(shape);
        transform.setScale(halfExtent);
        MyBuffer.transform(result, 0, numFloats, transform);

        return result;
    }

    /**
     * Generate a Transform that maps the Y-Z plane to the surface of the
     * specified PlaneCollisionShape.
     *
     * @param shape (not null, unaffected) units, &gt;0)
     * @return a new Transform with scale=1
     */
    private static Transform planeTransform(PlaneCollisionShape shape) {
        Transform result = new Transform();

        Plane plane = shape.getPlane();
        plane.getClosestPoint(translateIdentity, result.getTranslation());

        Vector3f v1 = plane.getNormal();
        Vector3f v2 = new Vector3f();
        Vector3f v3 = new Vector3f();
        MyVector3f.generateBasis(v1, v2, v3);
        result.getRotation().fromAxes(v1, v2, v3);

        return result;
    }
    // *************************************************************************
    // native private methods

    native private static void getTriangles(
            long shapeId, int meshResolution, DebugMeshCallback buffer);

    native private static void getVertices(
            long shapeId, int meshResolution, DebugMeshCallback buffer);
}
