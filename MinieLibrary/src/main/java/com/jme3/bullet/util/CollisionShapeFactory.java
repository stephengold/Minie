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
 * A utility class for generating collision shapes from spatials.
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
     * Create simplified shape(s) for the given spatial, based on the bounding
     * volumes of its geometries. TODO buggy!
     *
     * @param spatial the spatial on which to base the shape (not null,
     * unaffected)
     * @return a new box/sphere collision shape (if spatial is a Geometry) or a
     * CompoundCollisionShape with box/sphere collision shapes as children (if
     * spatial is a Node)
     */
    public static CollisionShape createBoxShape(Spatial spatial) {
        if (spatial instanceof Geometry) {
            return createSingleBoxShape(spatial);
        } else if (spatial instanceof Node) {
            return createBoxCompoundShape((Node) spatial);
        } else {
            throw new IllegalArgumentException(
                    "The spatial must either be a Node or a Geometry!");
        }
    }

    /**
     * Create convex mesh shape(s) for the given spatial, based on convex hulls
     * of its mesh vertices.
     * <p>
     * For mesh-accurate animated meshes (CPU intense!) use GImpact shapes.
     *
     * @param spatial the spatial on which to base the shape (not null,
     * unaffected)
     * @return a new hull collision shape (if spatial is a Geometry) or a new
     * CompoundCollisionShape with hull collision shapes as children (if spatial
     * is a Node)
     */
    public static CollisionShape createDynamicMeshShape(Spatial spatial) {
        if (spatial instanceof Geometry) {
            return createSingleHullShape((Geometry) spatial, spatial);
        } else if (spatial instanceof Node) {
            boolean meshAccurate = true;
            boolean dynamic = true;
            return createCompoundShape((Node) spatial, (Node) spatial,
                    new CompoundCollisionShape(), meshAccurate, dynamic);
        } else {
            throw new IllegalArgumentException(
                    "The spatial must either be a Node or a Geometry!");
        }
    }

    /**
     * Create precise mesh shape(s) for the given spatial.
     * <p>
     * This type of collision shape is mesh-accurate and meant for immovable
     * "world objects". Examples include terrain, houses, or whole shooter
     * levels.
     *
     * @param spatial the spatial on which to base the shape (not null,
     * unaffected)
     * @return a new mesh collision shape (if spatial is a Geometry) or a new
     * heightfield collision shape (if spatial is a TerrainQuad or TerrainPatch)
     * or a new CompoundCollisionShape with mesh/heighfield collision shapes as
     * children (if spatial is a Node)
     */
    public static CollisionShape createMeshShape(Spatial spatial) {
        if (spatial instanceof TerrainQuad) {
            TerrainQuad terrain = (TerrainQuad) spatial;
            return new HeightfieldCollisionShape(terrain.getHeightMap(), terrain.getLocalScale());
        } else if (spatial instanceof TerrainPatch) {
            TerrainPatch terrain = (TerrainPatch) spatial;
            return new HeightfieldCollisionShape(terrain.getHeightMap(), terrain.getLocalScale());
        } else if (spatial instanceof Geometry) {
            return createSingleMeshShape((Geometry) spatial, spatial);
        } else if (spatial instanceof Node) {
            return createMeshCompoundShape((Node) spatial);
        } else {
            throw new IllegalArgumentException(
                    "The spatial must either be a Node or a Geometry!");
        }
    }
    // *************************************************************************
    // private methods

    /**
     * This type of collision shape creates a CompoundShape made out of boxes
     * that are based on the bounds of the Geometries in the tree.
     *
     * @param modelRoot the node on which to base the shape (not null)
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
     * This type of collision shape is mesh-accurate and meant for immovable
     * "world objects". Examples include terrain, houses or whole shooter
     * levels.
     *
     * @param rootNode the node on which to base the shape (not null)
     * @return a new shape (not null)
     */
    private static CompoundCollisionShape createMeshCompoundShape(
            Node rootNode) {
        boolean meshAccurate = true;
        return createCompoundShape(rootNode, new CompoundCollisionShape(),
                meshAccurate);
    }

    /**
     * Use the bounding volume of the supplied spatial to create a non-compound
     * collision shape.
     *
     * @param spatial the spatial on which to base the shape (not null,
     * unaffected)
     * @return a new shape to match the spatial's bounding box (not null)
     */
    private static BoxCollisionShape createSingleBoxShape(Spatial spatial) {
        //TODO using world bound here instead of "local world" bound...
        //TODO the bound could be null or spherical
        BoxCollisionShape shape = new BoxCollisionShape(
                ((BoundingBox) spatial.getWorldBound()).getExtent(new Vector3f()));
        return shape;
    }

    /**
     * Create a hull collision shape for the specified geometry.
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
     * This type of collision shape is mesh-accurate and meant for immovable
     * "world objects". Examples include terrain, houses or whole shooter
     * levels.
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
     * Calculate the correct transform for a child collision shape relative to
     * the ancestor for which the shape is being generated.
     *
     * @param spatial (not null, unaffected)
     * @param modelRoot the ancestor for which the shape is being generated (not
     * null, unaffected)
     * @return a new transform (not null)
     */
    private static Transform getTransform(Spatial spatial, Spatial modelRoot) {
        Transform result = new Transform();
        Spatial currentSpatial = spatial;
        while (currentSpatial != modelRoot) {
            result.combineWithParent(currentSpatial.getLocalTransform());
            currentSpatial = currentSpatial.getParent();
        }
        //include the model root's scale only
        Transform mrTransform = new Transform();
        mrTransform.setScale(modelRoot.getLocalScale());
        result.combineWithParent(mrTransform);

        return result;
    }
}
