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

import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.GImpactCollisionShape;
import com.jme3.bullet.collision.shapes.HeightfieldCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.MeshCollisionShape;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.UserData;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.mesh.IndexBuffer;
import com.jme3.terrain.Terrain;
import com.jme3.util.BufferUtils;
import java.nio.Buffer;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.logging.Logger;
import jme3utilities.MyMesh;
import jme3utilities.MySpatial;
import jme3utilities.Validate;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyVector3f;
import vhacd.VHACD;
import vhacd.VHACDHull;
import vhacd.VHACDParameters;
import vhacd4.Vhacd4;
import vhacd4.Vhacd4Hull;
import vhacd4.Vhacd4Parameters;

/**
 * Utility methods for generating collision shapes from models.
 *
 * @author normenhansen, tim8dev
 */
final public class CollisionShapeFactory {
    // *************************************************************************
    // constants and loggers

    /**
     * number of axes in a vector
     */
    final private static int numAxes = 3;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(CollisionShapeFactory.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private CollisionShapeFactory() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Create a simplified shape for a movable object, based on the axis-aligned
     * bounding boxes of its meshes.
     *
     * @param modelRoot the model on which to base the shape (not null,
     * unaffected)
     * @return a CompoundCollisionShape with box shapes as children
     */
    public static CollisionShape createBoxShape(Spatial modelRoot) {
        CompoundCollisionShape result = new CompoundCollisionShape();
        boolean meshAccurate = false;
        boolean dynamic = true;

        if (modelRoot instanceof Geometry) {
            Geometry geometry = (Geometry) modelRoot;
            Vector3f centerOffset = new Vector3f();
            BoxCollisionShape box = createSingleBoxShape(
                    geometry, geometry, centerOffset);
            result.addChildShape(box, centerOffset);

        } else if (modelRoot instanceof Node) {
            Node node = (Node) modelRoot;
            createCompoundShape(node, node, result, meshAccurate, dynamic);

        } else {
            throw new IllegalArgumentException(
                    "The model root must either be a Node or a Geometry!");
        }

        return result;
    }

    /**
     * Create a shape for a movable object, based on the convex hulls of its
     * model's meshes.
     * <p>
     * For mesh-accurate movable objects (CPU-intense!) use
     * GImpactCollisionShape.
     *
     * @param modelRoot the model on which to base the shape (not null,
     * unaffected)
     * @return a new HullCollisionShape (if modelRoot is a Geometry) or a new
     * CompoundCollisionShape with hull shapes as children (if modelRoot is a
     * Node)
     */
    public static CollisionShape createDynamicMeshShape(Spatial modelRoot) {
        if (modelRoot instanceof Geometry) {
            return createSingleHullShape((Geometry) modelRoot, modelRoot);

        } else if (modelRoot instanceof Node) {
            Node node = (Node) modelRoot;
            CompoundCollisionShape result = new CompoundCollisionShape();
            boolean meshAccurate = true;
            boolean dynamic = true;
            createCompoundShape(node, node, result, meshAccurate, dynamic);
            return result;

        } else {
            throw new IllegalArgumentException(
                    "The model root must either be a Node or a Geometry!");
        }
    }

    /**
     * Create a mesh-accurate shape for an movable object, based on its model.
     * Terrain is ignored.
     *
     * @param modelRoot the model on which to base the shape (not null,
     * unaffected)
     * @return a new GImpactCollisionShape
     */
    public static GImpactCollisionShape createGImpactShape(Spatial modelRoot) {
        Validate.nonNull(modelRoot, "model root");

        Mesh mergedMesh = makeMergedMesh(modelRoot);
        GImpactCollisionShape result = new GImpactCollisionShape(mergedMesh);

        return result;
    }

    /**
     * Create a very simple shape for an object, based its model's bounding box.
     *
     * @param modelRoot the model on which to base the shape (not null,
     * unaffected)
     * @return a new CompoundCollisionShape
     */
    public static CollisionShape createMergedBoxShape(Spatial modelRoot) {
        Validate.nonNull(modelRoot, "model root");

        Mesh mergedMesh = makeMergedMesh(modelRoot);
        int numVertices = mergedMesh.getVertexCount();
        int numFloats = numAxes * numVertices;
        FloatBuffer positions
                = mergedMesh.getFloatBuffer(VertexBuffer.Type.Position);
        Vector3f maxima = new Vector3f();
        Vector3f minima = new Vector3f();
        MyBuffer.maxMin(positions, 0, numFloats, maxima, minima);

        Vector3f centerOffset = new Vector3f();
        MyVector3f.midpoint(maxima, minima, centerOffset);
        Vector3f halfExtents = maxima.subtract(centerOffset);
        BoxCollisionShape box = new BoxCollisionShape(halfExtents);

        CompoundCollisionShape result = new CompoundCollisionShape();
        result.addChildShape(box, centerOffset);

        return result;
    }

    /**
     * Create a simplified shape for a movable object, based the convex hull of
     * its model.
     *
     * @param modelRoot the model on which to base the shape (not null,
     * unaffected)
     * @return a new HullCollisionShape
     */
    public static HullCollisionShape createMergedHullShape(Spatial modelRoot) {
        Validate.nonNull(modelRoot, "model root");

        Mesh mergedMesh = makeMergedMesh(modelRoot);
        HullCollisionShape result = new HullCollisionShape(mergedMesh);

        return result;
    }

    /**
     * Create a mesh-accurate shape for an immovable object, based on its model.
     * This version ignores terrain.
     *
     * @param modelRoot the model on which to base the shape (not null,
     * unaffected)
     * @return a new MeshCollisionShape
     */
    public static MeshCollisionShape createMergedMeshShape(Spatial modelRoot) {
        Validate.nonNull(modelRoot, "model root");

        Mesh mergedMesh = makeMergedMesh(modelRoot);
        MeshCollisionShape result = new MeshCollisionShape(mergedMesh);

        return result;
    }

    /**
     * Create a mesh-accurate shape for an immovable object, based on its model.
     * This version handles terrain.
     *
     * @param modelRoot the model on which to base the shape (not null,
     * unaffected)
     * @return a new MeshCollisionShape (if modelRoot is a Geometry) or a new
     * HeightfieldCollisionShape (if modelRoot is a TerrainQuad or TerrainPatch)
     * or a new CompoundCollisionShape with mesh/heightfield shapes as children
     * (if modelRoot is a Node)
     */
    public static CollisionShape createMeshShape(Spatial modelRoot) {
        if (modelRoot instanceof Terrain) {
            return new HeightfieldCollisionShape(
                    (Terrain) modelRoot, modelRoot.getLocalScale());

        } else if (modelRoot instanceof Geometry) {
            return createSingleMeshShape((Geometry) modelRoot, modelRoot);

        } else if (modelRoot instanceof Node) {
            Node node = (Node) modelRoot;
            CompoundCollisionShape result = new CompoundCollisionShape();
            boolean meshAccurate = true;
            boolean dynamic = false;
            createCompoundShape(node, node, result, meshAccurate, dynamic);
            return result;

        } else {
            throw new IllegalArgumentException(
                    "The model root must either be a Node or a Geometry!");
        }
    }

    /**
     * Create a shape for a dynamic object using classic V-HACD.
     *
     * @param modelRoot the model on which to base the shape (not null,
     * unaffected)
     * @param parameters (not null, unaffected)
     * @param addResult the compound shape to append to (modified if not null)
     * @return a compound shape (either addResult or a new shape, not null)
     */
    public static CompoundCollisionShape createVhacdShape(Spatial modelRoot,
            VHACDParameters parameters, CompoundCollisionShape addResult) {
        Validate.nonNull(modelRoot, "model root");
        Validate.nonNull(parameters, "parameters");

        Mesh mergedMesh = makeMergedMesh(modelRoot);

        FloatBuffer positionBuffer
                = mergedMesh.getFloatBuffer(VertexBuffer.Type.Position);
        int numFloats = positionBuffer.limit();
        float[] positionArray = new float[numFloats];
        for (int offset = 0; offset < numFloats; ++offset) {
            positionArray[offset] = positionBuffer.get(offset);
        }

        IndexBuffer indexBuffer = mergedMesh.getIndicesAsList();
        int numIndices = indexBuffer.size();
        int[] indexArray = new int[numIndices];
        for (int offset = 0; offset < numIndices; ++offset) {
            indexArray[offset] = indexBuffer.get(offset);
        }

        // Use the V-HACD algorithm to generate a list of hulls.
        List<VHACDHull> vhacdHulls
                = VHACD.compute(positionArray, indexArray, parameters);
        /*
         * Convert each V-HACD hull to a HullCollisionShape
         * and add that to the result.
         */
        CompoundCollisionShape result;
        if (addResult == null) {
            int numHulls = vhacdHulls.size();
            result = new CompoundCollisionShape(numHulls);
        } else {
            result = addResult;
        }
        for (VHACDHull vhacdHull : vhacdHulls) {
            HullCollisionShape hullShape = new HullCollisionShape(vhacdHull);
            result.addChildShape(hullShape);
        }

        return result;
    }

    /**
     * Create a shape for a dynamic object using V-HACD version 4.
     *
     * @param modelRoot the model on which to base the shape (not null,
     * unaffected)
     * @param parameters (not null, unaffected)
     * @param addResult the compound shape to append to (modified if not null)
     * @return a compound shape (either addResult or a new shape, not null)
     */
    public static CompoundCollisionShape createVhacdShape(Spatial modelRoot,
            Vhacd4Parameters parameters, CompoundCollisionShape addResult) {
        Validate.nonNull(modelRoot, "model root");
        Validate.nonNull(parameters, "parameters");

        Mesh mergedMesh = makeMergedMesh(modelRoot);

        FloatBuffer positionBuffer
                = mergedMesh.getFloatBuffer(VertexBuffer.Type.Position);
        int numFloats = positionBuffer.limit();
        float[] positionArray = new float[numFloats];
        for (int offset = 0; offset < numFloats; ++offset) {
            positionArray[offset] = positionBuffer.get(offset);
        }

        IndexBuffer indexBuffer = mergedMesh.getIndicesAsList();
        int numIndices = indexBuffer.size();
        int[] indexArray = new int[numIndices];
        for (int offset = 0; offset < numIndices; ++offset) {
            indexArray[offset] = indexBuffer.get(offset);
        }

        // Use the V-HACD algorithm to generate a list of hulls.
        List<Vhacd4Hull> vhacdHulls
                = Vhacd4.compute(positionArray, indexArray, parameters);
        /*
         * Convert each V-HACD hull to a HullCollisionShape
         * and add that to the result.
         */
        CompoundCollisionShape result;
        if (addResult == null) {
            int numHulls = vhacdHulls.size();
            result = new CompoundCollisionShape(numHulls);
        } else {
            result = addResult;
        }
        for (Vhacd4Hull vhacdHull : vhacdHulls) {
            HullCollisionShape hullShape = new HullCollisionShape(vhacdHull);
            result.addChildShape(hullShape);
        }

        return result;
    }
    // *************************************************************************
    // private methods

    /**
     * Append transformed mesh triangles to a merged mesh.
     *
     * @param geometry the Geometry from which to read triangles (not null,
     * unaffected)
     * @param modelRoot (not null, unaffected)
     * @param addPositions the position buffer for the merged mesh (not null,
     * modified)
     * @param addIndices the index buffer for the merged mesh (not null,
     * modified)
     */
    private static void appendTriangles(Geometry geometry, Spatial modelRoot,
            FloatBuffer addPositions, IndexBuffer addIndices) {
        Mesh jmeMesh = geometry.getMesh();

        // Append merged-mesh indices to the IndexBuffer.
        int indexBase = addPositions.position() / numAxes;
        IndexBuffer indexBuffer = jmeMesh.getIndicesAsList();
        int numIndices = indexBuffer.size();
        for (int offset = 0; offset < numIndices; ++offset) {
            int indexInGeometry = indexBuffer.get(offset);
            int indexInMergedMesh = indexBase + indexInGeometry;
            addIndices.put(indexInMergedMesh);
        }

        // Append transformed vertex locations to the FloatBuffer.
        Transform transform = relativeTransform(geometry, modelRoot);
        Vector3f tmpPosition = new Vector3f();
        int numVertices = jmeMesh.getVertexCount();
        for (int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex) {
            MyMesh.vertexVector3f(jmeMesh, VertexBuffer.Type.Position,
                    vertexIndex, tmpPosition);
            transform.transformVector(tmpPosition, tmpPosition);
            addPositions.put(tmpPosition.x);
            addPositions.put(tmpPosition.y);
            addPositions.put(tmpPosition.z);
        }
    }

    private static void createCompoundShape(Node modelRoot, Node parent,
            CompoundCollisionShape shape, boolean meshAccurate,
            boolean dynamic) {
        for (Spatial child : parent.getChildren()) {
            Boolean skipChild = child.getUserData(UserData.JME_PHYSICSIGNORE);
            if (skipChild != null && skipChild) {
                continue; // to the next child spatial
            }
            Transform transform = relativeTransform(child, modelRoot);

            CollisionShape childShape;
            if (child instanceof Terrain) {
                childShape = new HeightfieldCollisionShape((Terrain) child,
                        transform.getScale());
                shape.addChildShape(childShape, transform);

            } else if (child instanceof Node) {
                createCompoundShape(modelRoot, (Node) child, shape,
                        meshAccurate, dynamic);

            } else if (child instanceof Geometry) {
                Geometry geometry = (Geometry) child;
                Vector3f centerOffset = new Vector3f(0f, 0f, 0f);
                if (meshAccurate) {
                    if (dynamic) {
                        childShape = createSingleHullShape(geometry, modelRoot);
                    } else {
                        childShape = createSingleMeshShape(geometry, modelRoot);
                    }
                } else {
                    childShape = createSingleBoxShape(
                            geometry, modelRoot, centerOffset);
                    transform.getRotation().mult(centerOffset, centerOffset);
                    transform.getTranslation().addLocal(centerOffset);
                }
                if (childShape != null) {
                    shape.addChildShape(childShape, transform);
                }
            }
        }
    }

    /**
     * Create a BoxCollisionShape for the specified Geometry, based on the
     * axis-aligned bounding box of its Mesh.
     *
     * @param geometry the Geometry on which to base the shape (not null)
     * @param modelRoot the ancestor for which the shape is being generated (not
     * null, unaffected)
     * @param storeCenter storage for the center offset (not null, modified)
     * @return a new instance, or null if the Mesh is null or empty
     */
    private static BoxCollisionShape createSingleBoxShape(
            Geometry geometry, Spatial modelRoot, Vector3f storeCenter) {
        Mesh mesh = geometry.getMesh();
        if (mesh == null) {
            return null;
        }
        int numVertices = mesh.getVertexCount();
        if (numVertices < 1) {
            return null;
        }

        Transform transform = relativeTransform(geometry, modelRoot);

        int numFloats = numAxes * numVertices;
        FloatBuffer positions = mesh.getFloatBuffer(VertexBuffer.Type.Position);
        Vector3f maxima = new Vector3f();
        Vector3f minima = new Vector3f();
        MyBuffer.maxMin(positions, 0, numFloats, maxima, minima);

        MyVector3f.midpoint(maxima, minima, storeCenter);
        Vector3f halfExtents = maxima.subtract(storeCenter);
        BoxCollisionShape result = new BoxCollisionShape(halfExtents);
        result.setScale(transform.getScale());

        return result;
    }

    /**
     * Create a HullCollisionShape for the specified Geometry.
     *
     * @param geometry the Geometry on which to base the shape (not null)
     * @param modelRoot the ancestor for which the shape is being generated (not
     * null, unaffected)
     * @return a new instance (not null)
     */
    private static HullCollisionShape createSingleHullShape(
            Geometry geometry, Spatial modelRoot) {
        Mesh mesh = geometry.getMesh();
        if (mesh == null) {
            return null;
        }

        Transform transform = relativeTransform(geometry, modelRoot);
        // TODO recognize AbstractBox, Cylinder, Quad,
        // and Sphere from com.jme3.scene.shape package
        HullCollisionShape hullShape = new HullCollisionShape(mesh);
        hullShape.setScale(transform.getScale());

        return hullShape;
    }

    /**
     * Create a mesh-accurate CollisionShape for an immovable object.
     *
     * @param geometry the Geometry on which to base the shape (not null)
     * @param modelRoot the ancestor for which the shape is being generated (not
     * null, unaffected)
     * @return a new MeshCollisionShape, or null if the Geometry doesn't contain
     * any triangles
     */
    private static MeshCollisionShape createSingleMeshShape(
            Geometry geometry, Spatial modelRoot) {
        Mesh mesh = geometry.getMesh();
        if (mesh == null || !MyMesh.hasTriangles(mesh)) {
            return null;
        }

        Transform transform = relativeTransform(geometry, modelRoot);
        // TODO recognize AbstractBox, Cylinder, Quad,
        // and Sphere from com.jme3.scene.shape package
        MeshCollisionShape result = new MeshCollisionShape(mesh);
        result.setScale(transform.getScale());

        return result;
    }

    /**
     * Generate a Mesh that merges the triangles of non-empty geometries not
     * tagged with "JmePhysicsIgnore".
     *
     * @param subtree the scene-graph subtree on which to base the Mesh (not
     * null, unaffected)
     * @return a new, indexed Mesh in Triangles mode, its bounds not set
     */
    private static Mesh makeMergedMesh(Spatial subtree) {
        List<Geometry> allGeometries = MySpatial.listGeometries(subtree);
        Collection<Geometry> includedGeometries
                = new ArrayList<>(allGeometries.size());
        int totalIndices = 0;
        int totalVertices = 0;
        for (Geometry geometry : allGeometries) {
            /*
             * Exclude any Geometry tagged with "JmePhysicsIgnore"
             * or having a null/empty mesh.
             */
            Boolean ignore = geometry.getUserData(UserData.JME_PHYSICSIGNORE);
            if (ignore != null && ignore) {
                continue;
            }
            Mesh jmeMesh = geometry.getMesh();
            if (jmeMesh == null) {
                continue;
            }
            IndexBuffer indexBuffer = jmeMesh.getIndicesAsList();
            int numIndices = indexBuffer.size();
            if (numIndices == 0) {
                continue;
            }
            int numVertices = jmeMesh.getVertexCount();
            if (numVertices == 0) {
                continue;
            }

            includedGeometries.add(geometry);
            totalIndices += numIndices;
            totalVertices += numVertices;
        }

        IndexBuffer indexBuffer
                = IndexBuffer.createIndexBuffer(totalVertices, totalIndices);
        int totalFloats = numAxes * totalVertices;
        FloatBuffer positionBuffer = BufferUtils.createFloatBuffer(totalFloats);

        for (Geometry geometry : includedGeometries) {
            appendTriangles(geometry, subtree, positionBuffer, indexBuffer);
        }

        VertexBuffer.Format ibFormat = indexBuffer.getFormat();
        Buffer ibData = indexBuffer.getBuffer();
        Mesh result = new Mesh();
        result.setBuffer(VertexBuffer.Type.Index, MyMesh.vpt, ibFormat, ibData);
        result.setBuffer(VertexBuffer.Type.Position, numAxes, positionBuffer);

        return result;
    }

    /**
     * Calculate the Transform for a ChildCollisionShape relative to the
     * ancestor for which the shape is being generated.
     *
     * @param spatial (not null, unaffected)
     * @param modelRoot the ancestor for which the shape is being generated (not
     * null, unaffected)
     * @return a new Transform (not null)
     */
    private static Transform relativeTransform(
            Spatial spatial, Spatial modelRoot) {
        Transform result = new Transform();
        Spatial currentSpatial = spatial;
        while (currentSpatial != modelRoot) {
            result.combineWithParent(currentSpatial.getLocalTransform());
            currentSpatial = currentSpatial.getParent();
        }
        // Include the model root's scale but not its translation or rotation.
        Transform mrTransform = new Transform(); // TODO garbage
        mrTransform.setScale(modelRoot.getLocalScale());
        result.combineWithParent(mrTransform);

        return result;
    }
}
