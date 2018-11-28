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
            return createSingleBoxShape((Geometry) spatial);
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
            return createSingleDynamicMeshShape((Geometry) spatial, spatial);
        } else if (spatial instanceof Node) {
            return createCompoundShape((Node) spatial, (Node) spatial,
                    new CompoundCollisionShape(), true, true);
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
     * @param rootNode the node on which to base the shape (not null)
     * @return a new shape (not null)
     */
    private static CompoundCollisionShape createBoxCompoundShape(
            Node rootNode) {
        return createCompoundShape(rootNode, new CompoundCollisionShape(),
                false);
    }

    private static CompoundCollisionShape createCompoundShape(
            Node rootNode, CompoundCollisionShape shape, boolean meshAccurate) {
        return createCompoundShape(rootNode, rootNode, shape, meshAccurate,
                false);
    }

    private static CompoundCollisionShape createCompoundShape(Node realRootNode,
            Node rootNode, CompoundCollisionShape shape, boolean meshAccurate,
            boolean dynamic) {
        for (Spatial spatial : rootNode.getChildren()) {
            if (spatial instanceof TerrainQuad) {
                Boolean bool = spatial.getUserData(UserData.JME_PHYSICSIGNORE);
                if (bool != null && bool) {
                    continue; // go to the next child in the loop
                }
                TerrainQuad terrain = (TerrainQuad) spatial;
                Transform trans = getTransform(spatial, realRootNode);
                shape.addChildShape(
                        new HeightfieldCollisionShape(terrain.getHeightMap(),
                                trans.getScale()),
                        trans.getTranslation(),
                        trans.getRotation().toRotationMatrix());
            } else if (spatial instanceof Node) {
                createCompoundShape(realRootNode, (Node) spatial, shape,
                        meshAccurate, dynamic);
            } else if (spatial instanceof TerrainPatch) {
                Boolean bool = spatial.getUserData(UserData.JME_PHYSICSIGNORE);
                if (bool != null && bool) {
                    continue; // go to the next child in the loop
                }
                TerrainPatch terrain = (TerrainPatch) spatial;
                Transform trans = getTransform(spatial, realRootNode);
                shape.addChildShape(
                        new HeightfieldCollisionShape(terrain.getHeightMap(),
                                terrain.getLocalScale()),
                        trans.getTranslation(),
                        trans.getRotation().toRotationMatrix());
            } else if (spatial instanceof Geometry) {
                Boolean bool = spatial.getUserData(UserData.JME_PHYSICSIGNORE);
                if (bool != null && bool) {
                    continue; // go to the next child in the loop
                }

                if (meshAccurate) {
                    CollisionShape childShape = dynamic
                            ? createSingleDynamicMeshShape((Geometry) spatial, realRootNode)
                            : createSingleMeshShape((Geometry) spatial, realRootNode);
                    if (childShape != null) {
                        Transform trans = getTransform(spatial, realRootNode);
                        shape.addChildShape(childShape,
                                trans.getTranslation(),
                                trans.getRotation().toRotationMatrix());
                    }
                } else {
                    Transform trans = getTransform(spatial, realRootNode);
                    shape.addChildShape(
                            createSingleBoxShape(spatial),
                            trans.getTranslation(),
                            trans.getRotation().toRotationMatrix());
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
        return createCompoundShape(rootNode, new CompoundCollisionShape(),
                true);
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
     * @param parent
     */
    private static HullCollisionShape createSingleDynamicMeshShape(
            Geometry geom, Spatial parent) {
        Mesh mesh = geom.getMesh();
        Transform trans = getTransform(geom, parent);
        if (mesh != null) {
            HullCollisionShape dynamicShape = new HullCollisionShape(mesh);
            dynamicShape.setScale(trans.getScale());
            return dynamicShape;
        } else {
            return null;
        }
    }

    /**
     * This type of collision shape is mesh-accurate and meant for immovable
     * "world objects". Examples include terrain, houses or whole shooter
     * levels.
     */
    private static MeshCollisionShape createSingleMeshShape(Geometry geom,
            Spatial parent) {
        Mesh mesh = geom.getMesh();
        Transform trans = getTransform(geom, parent);
        if (mesh != null && mesh.getMode() == Mesh.Mode.Triangles) {
            MeshCollisionShape mColl = new MeshCollisionShape(mesh);
            mColl.setScale(trans.getScale());
            return mColl;
        } else {
            return null;
        }
    }

    /**
     * Calculate the correct transform for a collision shape relative to the
     * ancestor for which the shape was generated.
     *
     * @param spat
     * @param parent
     * @return a new instance (not null)
     */
    private static Transform getTransform(Spatial spat, Spatial parent) {
        Transform shapeTransform = new Transform();
        Spatial parentNode = spat.getParent() != null ? spat.getParent() : spat;
        Spatial currentSpatial = spat;
        //if we have parents combine their transforms
        while (parentNode != null) {
            if (parent == currentSpatial) {
                //real parent -> only apply scale, not transform
                Transform trans = new Transform();
                trans.setScale(currentSpatial.getLocalScale());
                shapeTransform.combineWithParent(trans);
                parentNode = null;
            } else {
                shapeTransform.combineWithParent(currentSpatial.getLocalTransform());
                parentNode = currentSpatial.getParent();
                currentSpatial = parentNode;
            }
        }

        return shapeTransform;
    }
}
