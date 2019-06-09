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

import com.jme3.bounding.BoundingBox;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
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
import com.jme3.terrain.geomipmap.TerrainPatch;
import com.jme3.terrain.geomipmap.TerrainQuad;
import java.util.logging.Logger;

/**
 * Utility methods for generating collision shapes from spatials.
 *
 * @author normenhansen, tim8dev
 */
public class CollisionShapeFactory {
    // *************************************************************************
    // constants and loggers

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
     * Create a simplified shape for the specified Spatial, based on the
     * bounding volumes of its geometries. TODO buggy!
     *
     * @param subtree the Spatial on which to base the shape (not null,
     * unaffected)
     * @return a new box/sphere CollisionShape (if spatial is a Geometry) or a
     * CompoundCollisionShape with box/sphere shapes as children (if spatial is
     * a Node)
     */
    public static CollisionShape createBoxShape(Spatial subtree) {
        if (subtree instanceof Geometry) {
            return createSingleBoxShape(subtree);
        } else if (subtree instanceof Node) {
            return createBoxCompoundShape((Node) subtree);
        } else {
            throw new IllegalArgumentException(
                    "The spatial must either be a Node or a Geometry!");
        }
    }

    /**
     * Create a shape for a movable object, based on the specified Spatial.
     * <p>
     * For mesh-accurate movable objects (CPU-intense!) use
     * GImpactCollisionShape.
     *
     * @param subtree the Spatial on which to base the shape (not null,
     * unaffected)
     * @return a new HullCollisionShape (if spatial is a Geometry) or a new
     * CompoundCollisionShape with hull shapes as children (if spatial is a
     * Node)
     */
    public static CollisionShape createDynamicMeshShape(Spatial subtree) {
        if (subtree instanceof Geometry) {
            return createSingleHullShape((Geometry) subtree, subtree);
        } else if (subtree instanceof Node) {
            boolean meshAccurate = true;
            boolean dynamic = true;
            return createCompoundShape((Node) subtree, (Node) subtree,
                    new CompoundCollisionShape(), meshAccurate, dynamic);
        } else {
            throw new IllegalArgumentException(
                    "The spatial must either be a Node or a Geometry!");
        }
    }

    /**
     * Create a shape for an immovable object, based on the specified Spatial.
     *
     * @param subtree the Spatial on which to base the shape (not null,
     * unaffected)
     * @return a new MeshCollisionShape (if spatial is a Geometry) or a new
     * HeightfieldCollisionShape (if spatial is a TerrainQuad or TerrainPatch)
     * or a new CompoundCollisionShape with mesh/heightfield shapes as children
     * (if spatial is a Node)
     */
    public static CollisionShape createMeshShape(Spatial subtree) {
        if (subtree instanceof TerrainQuad) {
            TerrainQuad terrain = (TerrainQuad) subtree;
            return new HeightfieldCollisionShape(terrain.getHeightMap(),
                    terrain.getLocalScale());
        } else if (subtree instanceof TerrainPatch) {
            TerrainPatch terrain = (TerrainPatch) subtree;
            return new HeightfieldCollisionShape(terrain.getHeightMap(),
                    terrain.getLocalScale());
        } else if (subtree instanceof Geometry) {
            return createSingleMeshShape((Geometry) subtree, subtree);
        } else if (subtree instanceof Node) {
            return createMeshCompoundShape((Node) subtree);
        } else {
            throw new IllegalArgumentException(
                    "The spatial must either be a Node or a Geometry!");
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Create a CompoundShape of boxes, based on the bounds of the Geometries in
     * a scene-graph subtree.
     *
     * @param modelRoot the Node on which to base the shape (not null)
     * @return a new shape (not null)
     */
    private static CompoundCollisionShape createBoxCompoundShape(
            Node modelRoot) {
        boolean meshAccurate = false;
        return createCompoundShape(modelRoot, new CompoundCollisionShape(),
                meshAccurate);
    }

    private static CompoundCollisionShape createCompoundShape(Node modelRoot,
            CompoundCollisionShape shape, boolean meshAccurate) {
        boolean dynamic = false;
        return createCompoundShape(modelRoot, modelRoot, shape, meshAccurate,
                dynamic);
    }

    private static CompoundCollisionShape createCompoundShape(Node modelRoot,
            Node parent, CompoundCollisionShape shape, boolean meshAccurate,
            boolean dynamic) {
        for (Spatial child : parent.getChildren()) {
            Boolean skipChild = child.getUserData(UserData.JME_PHYSICSIGNORE);
            if (skipChild != null && skipChild) {
                continue; // to the next child spatial
            }
            Transform transform = getTransform(child, modelRoot);

            CollisionShape childShape;
            if (child instanceof TerrainQuad) {
                TerrainQuad terrainQuad = (TerrainQuad) child;
                childShape = new HeightfieldCollisionShape(
                        terrainQuad.getHeightMap(),
                        transform.getScale());
                shape.addChildShape(childShape, transform);

            } else if (child instanceof Node) {
                createCompoundShape(modelRoot, (Node) child, shape,
                        meshAccurate, dynamic);

            } else if (child instanceof TerrainPatch) {
                TerrainPatch patch = (TerrainPatch) child;
                childShape = new HeightfieldCollisionShape(
                        patch.getHeightMap(),
                        patch.getLocalScale());
                shape.addChildShape(childShape, transform);

            } else if (child instanceof Geometry) {
                Geometry geometry = (Geometry) child;
                if (meshAccurate) {
                    if (dynamic) {
                        childShape = createSingleHullShape(geometry, modelRoot);
                    } else {
                        childShape = createSingleMeshShape(geometry, modelRoot);
                    }
                } else {
                    childShape = createSingleBoxShape(geometry);
                }
                if (childShape != null) {
                    shape.addChildShape(childShape, transform);
                }
            }
        }

        return shape;
    }

    /**
     * Create a mesh-accurate CollisionShape for an immovable object.
     *
     * @param rootNode the Node on which to base the shape (not null)
     * @return a new shape (not null)
     */
    private static CompoundCollisionShape createMeshCompoundShape(
            Node rootNode) {
        boolean meshAccurate = true;
        return createCompoundShape(rootNode, new CompoundCollisionShape(),
                meshAccurate);
    }

    /**
     * Use the bounding volume of the supplied Spatial to create a non-compound
     * CollisionShape.
     *
     * @param spatial the Spatial on which to base the shape (not null,
     * unaffected)
     * @return a new shape to match the Spatial's bounding box (not null)
     */
    private static BoxCollisionShape createSingleBoxShape(Spatial spatial) {
        //TODO using world bound here instead of "local world" bound...
        //TODO the bound could be null or spherical
        BoxCollisionShape shape = new BoxCollisionShape(
                ((BoundingBox) spatial.getWorldBound()).getExtent(new Vector3f()));
        return shape;
    }

    /**
     * Create a HullCollisionShape for the specified Geometry.
     *
     * @param geom the geometry on which to base the shape (not null)
     * @param modelRoot
     */
    private static HullCollisionShape createSingleHullShape(
            Geometry geom, Spatial modelRoot) {
        Mesh mesh = geom.getMesh();
        if (mesh == null) {
            return null;
        }

        Transform transform = getTransform(geom, modelRoot);
        HullCollisionShape hullShape = new HullCollisionShape(mesh);
        hullShape.setScale(transform.getScale());

        return hullShape;
    }

    /**
     * Create a mesh-accurate CollisionShape for an immovable object.
     */
    private static MeshCollisionShape createSingleMeshShape(Geometry geometry,
            Spatial modelRoot) {
        Mesh mesh = geometry.getMesh();
        if (mesh == null) {
            return null;
        }

        Transform transform = getTransform(geometry, modelRoot);
        MeshCollisionShape meshShape = new MeshCollisionShape(mesh);
        meshShape.setScale(transform.getScale());

        return meshShape;
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
    private static Transform getTransform(Spatial spatial, Spatial modelRoot) {
        Transform result = new Transform();
        Spatial currentSpatial = spatial;
        while (currentSpatial != modelRoot) {
            result.combineWithParent(currentSpatial.getLocalTransform());
            currentSpatial = currentSpatial.getParent();
        }
        // Include the model root's scale but not its translation or rotation.
        Transform mrTransform = new Transform();
        mrTransform.setScale(modelRoot.getLocalScale());
        result.combineWithParent(mrTransform);

        return result;
    }
}
