/*
 * Copyright (c) 2009-2023 jMonkeyEngine
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
package com.jme3.bullet.debug;

import com.jme3.asset.AssetManager;
import com.jme3.bullet.PhysicsSoftSpace;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.scene.Node;
import com.jme3.scene.control.Control;
import com.jme3.texture.Texture;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.MyAsset;

/**
 * An AppState to manage debug visualization of a PhysicsSoftSpace.
 *
 * @author Stephen Gold sgold@sonic.net
 *
 * Based on BulletSoftBodyDebugAppState by dokthar.
 */
public class SoftDebugAppState extends BulletDebugAppState {
    // *************************************************************************
    // constants and loggers

    /**
     * local copy of {@link com.jme3.math.ColorRGBA#White}
     */
    final private static ColorRGBA whiteColor = new ColorRGBA(1f, 1f, 1f, 1f);
    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(SoftDebugAppState.class.getName());
    // *************************************************************************
    // fields

    /**
     * limit which clusters are visualized, or null to visualize no clusters
     */
    private BulletDebugAppState.DebugAppStateFilter clusterFilter;
    /**
     * limit which wind velocities are visualized, or null to visualize none
     */
    private BulletDebugAppState.DebugAppStateFilter windVelocityFilter;
    /**
     * map soft bodies to visualization nodes
     */
    private HashMap<PhysicsSoftBody, Node> softBodies = new HashMap<>(64);
    /**
     * material for visualizing all soft-body anchors
     */
    private Material anchorMaterial;
    /**
     * material for visualizing all soft-body clusters
     */
    private Material clusterMaterial;
    /**
     * material for visualizing all soft-body links
     */
    private Material linkMaterial;
    /**
     * material for visualizing all pinned soft-body nodes
     */
    private Material pinMaterial;
    /**
     * material for visualizing all soft-body wind velocities
     */
    private Material windVelocityMaterial;
    /**
     * materials for soft-body faces
     */
    final private Material[] faceMaterials = new Material[3];
    // *************************************************************************
    // constructors

    /**
     * Instantiate an AppState with the specified configuration. This
     * constructor should be invoked only by BulletSoftBodyAppState.
     *
     * @param config the desired configuration (not null, alias created)
     */
    public SoftDebugAppState(DebugConfiguration config) {
        super(config);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Access the Material for visualizing soft-body anchors.
     *
     * @return the pre-existing instance (not null)
     */
    Material getAnchorMaterial() {
        assert anchorMaterial != null;
        return anchorMaterial;
    }

    /**
     * Access the filter that determines which clusters are visualized.
     *
     * @return the filter, or null if none
     */
    BulletDebugAppState.DebugAppStateFilter getClusterFilter() {
        return clusterFilter;
    }

    /**
     * Access the Material for visualizing soft-body clusters.
     *
     * @return the pre-existing instance (not null)
     */
    Material getClusterMaterial() {
        assert clusterMaterial != null;
        return clusterMaterial;
    }

    /**
     * Access a Material for visualizing soft-body faces.
     *
     * @param numSides 0&rarr;invisible, 1&rarr;single-sided material,
     * 2&rarr;double-sided Material
     * @return the pre-existing Material (not null)
     */
    Material getFaceMaterial(int numSides) {
        Material result = faceMaterials[numSides];
        assert result != null;
        return result;
    }

    /**
     * Access the Material for visualizing soft-body links.
     *
     * @return the pre-existing instance (not null)
     */
    Material getLinkMaterial() {
        assert linkMaterial != null;
        return linkMaterial;
    }

    /**
     * Access the Material for visualizing pinned soft-body nodes.
     *
     * @return the pre-existing instance (not null)
     */
    Material getPinMaterial() {
        assert pinMaterial != null;
        return pinMaterial;
    }

    /**
     * Access the Material for visualizing wind velocities.
     *
     * @return the pre-existing instance (not null)
     */
    Material getWindVelocityMaterial() {
        assert windVelocityMaterial != null;
        return windVelocityMaterial;
    }

    /**
     * Alter which soft-body clusters are visualized. For internal use only.
     *
     * @param filter the desired filter, or null to visualize no clusters
     */
    public void setClusterFilter(
            BulletDebugAppState.DebugAppStateFilter filter) {
        this.clusterFilter = filter;
    }

    /**
     * Alter which wind velocities are included in the debug visualization.
     *
     * @param filter the filter to use (alias created) or null to visualize no
     * wind velocities (default=null)
     */
    public void setWindVelocityFilter(
            BulletDebugAppState.DebugAppStateFilter filter) {
        this.windVelocityFilter = filter;
    }
    // *************************************************************************
    // BulletDebugAppState methods

    /**
     * Initialize the materials.
     *
     * @param am the application's AssetManager (not null)
     */
    @Override
    protected void setupMaterials(AssetManager am) {
        assert am != null;
        super.setupMaterials(am);

        this.anchorMaterial
                = createWireMaterial(am, ColorRGBA.Green, "anchorMaterial", 1);

        String matDefPath = "MatDefs/wireframe/multicolor2.j3md";
        this.clusterMaterial = new Material(am, matDefPath);
        ColorRGBA clusterColor = new ColorRGBA(1f, 0f, 0f, 1f); // red
        clusterMaterial.setColor("Color", clusterColor); // creates an alias
        float shapeSize = 10f;
        clusterMaterial.setFloat("PointSize", shapeSize);
        clusterMaterial.setName("clusterMaterial");
        String shapePath = "Textures/shapes/lozenge.png";
        boolean mipmaps = false;
        Texture shapeTexture = MyAsset.loadTexture(am, shapePath, mipmaps);
        clusterMaterial.setTexture("PointShape", shapeTexture);
        RenderState renderState = clusterMaterial.getAdditionalRenderState();
        renderState.setBlendMode(RenderState.BlendMode.Alpha);
        renderState.setDepthTest(false);

        this.faceMaterials[0] = MyAsset.createInvisibleMaterial(am);
        this.faceMaterials[1]
                = createWireMaterial(am, ColorRGBA.Red, "debug red ss", 1);
        this.faceMaterials[2]
                = createWireMaterial(am, ColorRGBA.Red, "debug red ds", 2);

        this.linkMaterial
                = createWireMaterial(am, ColorRGBA.Orange, "linkMaterial", 1);

        this.pinMaterial = new Material(am, matDefPath);
        ColorRGBA pinColor = new ColorRGBA(1f, 0f, 0f, 1f); // red
        pinMaterial.setColor("Color", pinColor); // creates an alias
        shapeSize = 24f;
        pinMaterial.setFloat("PointSize", shapeSize);
        pinMaterial.setName("pinMaterial");
        shapePath = "Textures/shapes/pin.png";
        mipmaps = false;
        shapeTexture = MyAsset.loadTexture(am, shapePath, mipmaps);
        pinMaterial.setTexture("PointShape", shapeTexture);
        renderState = pinMaterial.getAdditionalRenderState();
        renderState.setBlendMode(RenderState.BlendMode.Alpha);
        renderState.setDepthTest(false);

        this.windVelocityMaterial
                = createWireMaterial(am, whiteColor, "wind velocity", 2);
    }

    /**
     * Synchronize the velocity visualizers with the collision objects in the
     * PhysicsSpace.
     */
    @Override
    protected void updateVelocities() {
        super.updateVelocities();
        updateWindVelocities();
    }

    /**
     * Synchronize the collision-shape debug controls and axis visualizers with
     * the collision objects in the PhysicsSpace.
     */
    @Override
    protected void updateShapes() {
        super.updateShapes();
        updateSoftBodies();
    }
    // *************************************************************************
    // private methods

    /**
     * Synchronize the soft-body debug controls with the PhysicsSoftSpace.
     */
    private void updateSoftBodies() {
        HashMap<PhysicsSoftBody, Node> oldMap = softBodies;
        // create new map
        this.softBodies = new HashMap<>(oldMap.size());
        DebugConfiguration config = getConfiguration();
        PhysicsSoftSpace pSpace = (PhysicsSoftSpace) config.getSpace();
        Collection<PhysicsSoftBody> list = pSpace.getSoftBodyList();
        for (PhysicsSoftBody softBody : list) {
            Node node = oldMap.remove(softBody);
            if (node == null) {
                node = new Node(softBody.toString());
                attachChild(node);
            }
            softBodies.put(softBody, node);
        }

        // Detach nodes of soft bodies that have been removed from the space.
        for (Node node : oldMap.values()) {
            node.removeFromParent();
        }
        /*
         * Synchronize the soft-body debug controls and axis visualizers
         * with the soft bodies in the PhysicsSpace.
         */
        BulletDebugAppState.DebugAppStateFilter filter
                = getConfiguration().getFilter();
        for (Map.Entry<PhysicsSoftBody, Node> entry : softBodies.entrySet()) {
            PhysicsSoftBody softBody = entry.getKey();
            boolean displayShape = (filter == null)
                    || filter.displayObject(softBody);

            Node node = entry.getValue();
            Control control = node.getControl(SoftBodyDebugControl.class);
            if (control == null && displayShape) {
                logger.log(Level.FINE, "Create new SoftBodyDebugControl");
                control = new SoftBodyDebugControl(this, softBody);
                node.addControl(control);
            } else if (control != null && !displayShape) {
                node.removeControl(control);
            }

            updateAxes(node, displayShape);
        }
    }

    /**
     * Synchronize the wind-velocity debug controls with the soft bodies in the
     * PhysicsSpace.
     */
    private void updateWindVelocities() {
        if (windVelocityFilter == null) {
            return;
        }

        Map<PhysicsCollisionObject, Node> pcoMap = getPcoMap();
        for (Map.Entry<PhysicsCollisionObject, Node> entry
                : pcoMap.entrySet()) {
            PhysicsCollisionObject pco = entry.getKey();
            boolean display = pco instanceof PhysicsSoftBody
                    && windVelocityFilter.displayObject(pco);

            Node transformedNode = entry.getValue();
            Node parent = transformedNode.getParent();
            Control control
                    = parent.getControl(WindVelocityDebugControl.class);

            if (control == null && display) {
                logger.log(Level.FINE, "Create new WindVelocityDebugControl");
                control = new WindVelocityDebugControl(this, pco);
                parent.addControl(control);
            } else if (control != null && !display) {
                parent.removeControl(control);
            }
        }
    }
}
