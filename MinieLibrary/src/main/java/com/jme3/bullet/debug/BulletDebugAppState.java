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
package com.jme3.bullet.debug;

import com.jme3.app.Application;
import com.jme3.app.state.AbstractAppState;
import com.jme3.app.state.AppStateManager;
import com.jme3.asset.AssetManager;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.joints.Anchor;
import com.jme3.bullet.joints.Constraint;
import com.jme3.bullet.joints.JointEnd;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.joints.SoftPhysicsJoint;
import com.jme3.bullet.objects.PhysicsCharacter;
import com.jme3.bullet.objects.PhysicsGhostObject;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsVehicle;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.control.Control;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.MyAsset;
import jme3utilities.Validate;
import jme3utilities.debug.AxesVisualizer;

/**
 * An AppState to manage debug visualization of a PhysicsSpace.
 *
 * @author normenhansen
 */
public class BulletDebugAppState extends AbstractAppState {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(BulletDebugAppState.class.getName());
    // *************************************************************************
    // fields

    /**
     * application's asset manager: set by initialize()
     */
    private AssetManager assetManager;
    /**
     * limit which bounding boxes are visualized, or null to visualize no
     * bounding boxes
     */
    protected DebugAppStateFilter boundingBoxFilter;
    /**
     * limit which object shapes are visualized, or null to visualize all object
     * shapes
     */
    protected DebugAppStateFilter filter;
    /**
     * limit which swept spheres are visualized, or null to visualize no swept
     * spheres
     */
    private DebugAppStateFilter sweptSphereFilter;
    /**
     * registered init listener, or null if none
     */
    final private DebugInitListener initListener;
    /**
     * length of each axis arrow (in shape units, &gt;0) or 0 for no axis arrows
     */
    private float axisLength = 0f;
    /**
     * line width for axis arrows (in pixels, &ge;1) or 0 for solid axis arrows
     */
    private float axisLineWidth = 1f;
    /**
     * map collision objects to transformed visualization nodes
     */
    private HashMap<PhysicsCollisionObject, Node> pcoMap = new HashMap<>(64);
    /**
     * map physics joints to visualization nodes
     */
    private HashMap<PhysicsJoint, Node> jointMap = new HashMap<>(64);
    /**
     * materials for rigid bodies (and vehicles) that are responsive and either
     * static or kinematic or inactive
     */
    final private Material[] blues = new Material[3];
    /**
     * Material for joints (their A ends)
     */
    private Material green;
    /**
     * materials for rigid bodies (and vehicles) that are responsive, dynamic,
     * and active
     */
    final private Material[] magentas = new Material[3];
    /**
     * materials for responsive physics characters
     */
    final private Material[] pink = new Material[3];
    /**
     * Material for joints (their B ends)
     */
    private Material red;
    /**
     * Material for bounding boxes and swept spheres
     */
    private Material white;
    /**
     * materials for ghosts and other non-responsive collision objects
     */
    final private Material[] yellows = new Material[3];
    /**
     * scene-graph node to parent the geometries
     */
    final private Node physicsDebugRootNode
            = new Node("Physics Debug Root Node");
    /**
     * PhysicsSpace to visualize (not null)
     */
    final private PhysicsSpace space;
    /**
     * view ports in which to render (not null)
     */
    private ViewPort[] viewPorts;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an AppState to visualize the specified space using the
     * specified view ports. This constructor should be invoked only by
     * BulletAppState.
     *
     * @param space the PhysicsSpace to visualize (not null, alias created)
     * @param viewPorts the view ports in which to render (not null, unaffected)
     * @param filter the filter to limit which objects are visualized, or null
     * to visualize all objects (may be null, alias created)
     * @param initListener the init listener, or null if none (may be null,
     * alias created)
     */
    public BulletDebugAppState(PhysicsSpace space, ViewPort[] viewPorts,
            DebugAppStateFilter filter, DebugInitListener initListener) {
        Validate.nonNull(space, "space");
        Validate.nonNull(viewPorts, "view ports");

        this.space = space;

        int numViewPorts = viewPorts.length;
        this.viewPorts = new ViewPort[numViewPorts];
        System.arraycopy(viewPorts, 0, this.viewPorts, 0, numViewPorts);

        this.filter = filter;
        this.initListener = initListener;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Read the length of the axis arrows.
     *
     * @return length (in shape units, &ge;0)
     */
    public float axisLength() {
        assert axisLength >= 0f : axisLength;
        return axisLength;
    }

    /**
     * Read the line width for axis arrows.
     *
     * @return width (in pixels, &ge;1) or 0 for solid arrows
     */
    public float axisLineWidth() {
        assert axisLineWidth >= 0f : axisLineWidth;
        return axisLineWidth;
    }

    /**
     * Access a Material for visualizing active, responsive rigid bodies.
     *
     * @param numSides 0&rarr;invisible, 1&rarr;single-sided Material,
     * 2&rarr;double-sided Material
     * @return the pre-existing Material (not null)
     */
    Material getActiveMaterial(int numSides) {
        Material result = magentas[numSides];
        assert result != null;
        return result;
    }

    /**
     * Access the Material for visualizing bounding boxes.
     *
     * @return the pre-existing Material (not null)
     */
    Material getBoundingBoxMaterial() {
        assert white != null;
        return white;
    }

    /**
     * Access a Material for visualizing responsive characters.
     *
     * @param numSides 0&rarr;invisible, 1&rarr;single-sided Material,
     * 2&rarr;double-sided Material
     * @return the pre-existing Material (not null)
     */
    Material getCharacterMaterial(int numSides) {
        Material result = pink[numSides];
        assert result != null;
        return result;
    }

    /**
     * Access a Material for visualizing non-responsive collision objects.
     *
     * @param numSides 0&rarr;invisible, 1&rarr;single-sided Material,
     * 2&rarr;double-sided Material
     * @return the pre-existing Material (not null)
     */
    Material getGhostMaterial(int numSides) {
        Material result = yellows[numSides];
        assert result != null;
        return result;
    }

    /**
     * Access a Material for visualizing inactive rigid bodies.
     *
     * @param numSides 0&rarr;invisible, 1&rarr;single-sided Material,
     * 2&rarr;double-sided Material
     * @return the pre-existing Material (not null)
     */
    Material getInactiveMaterial(int numSides) {
        Material result = blues[numSides];
        assert result != null;
        return result;
    }

    /**
     * Access a Material for visualizing joints and soft-body anchors.
     *
     * @param end which end to visualize (not null)
     * @return the pre-existing Material (not null)
     */
    Material getJointMaterial(JointEnd end) {
        Material result;
        switch (end) {
            case A:
                result = green;
                break;
            case B:
                result = red;
                break;
            default:
                throw new IllegalArgumentException(end.toString());
        }

        assert result != null;
        return result;
    }

    /**
     * Access the Material for visualizing swept spheres.
     *
     * @return the pre-existing Material (not null)
     */
    Material getSweptSphereMaterial() {
        assert white != null;
        return white;
    }

    /**
     * Alter the length of the axis arrows.
     *
     * @param length (in shape units, &ge;0, default=0)
     */
    public void setAxisLength(float length) {
        Validate.nonNegative(length, "length");
        axisLength = length;
    }

    /**
     * Alter the line width for axis arrows.
     *
     * @param width (in pixels, &ge;1) or 0 for solid arrows (default=1)
     */
    public void setAxisLineWidth(float width) {
        Validate.inRange(width, "width", 0f, Float.MAX_VALUE);
        axisLineWidth = width;
    }

    /**
     * Alter which bounding boxes are visualized.
     *
     * @param filter the desired filter, or null to visualize no bounding boxes
     */
    public void setBoundingBoxFilter(DebugAppStateFilter filter) {
        boundingBoxFilter = filter;

        for (Node transformedNode : pcoMap.values()) {
            Node parent = transformedNode.getParent();
            Control control = parent.getControl(BoundingBoxDebugControl.class);
            parent.removeControl(control);
        }
    }

    /**
     * Alter which objects are visualized.
     *
     * @param filter the desired filter, or null to visualize all objects
     */
    public void setFilter(DebugAppStateFilter filter) {
        this.filter = filter;
    }

    /**
     * Alter which swept spheres are visualized.
     *
     * @param filter the desired filter, or null to visualize no swept spheres
     */
    public void setSweptSphereFilter(DebugAppStateFilter filter) {
        sweptSphereFilter = filter;

        for (Node transformedNode : pcoMap.values()) {
            Node parent = transformedNode.getParent();
            Control control = parent.getControl(SweptSphereDebugControl.class);
            parent.removeControl(control);
        }
    }

    /**
     * Alter the view ports in which to render.
     *
     * @param viewPorts array of view ports (not null, unaffected)
     */
    public void setViewPorts(ViewPort[] viewPorts) {
        int length = viewPorts.length;
        this.viewPorts = new ViewPort[length];
        System.arraycopy(viewPorts, 0, this.viewPorts, 0, length);
    }
    // *************************************************************************
    // new protected methods

    /**
     * Attach the specified Spatial to the debug root node.
     *
     * @param spatial the Spatial to attach (not null)
     */
    protected void attachChild(Spatial spatial) {
        assert spatial != null;
        physicsDebugRootNode.attachChild(spatial);
    }

    /**
     * Access the PhysicsSpace that's being visualized.
     *
     * @return the pre-existing instance (not null)
     */
    protected PhysicsSpace getPhysicsSpace() {
        assert space != null;
        return space;
    }

    /**
     * Initialize the wireframe materials.
     *
     * @param am the application's AssetManager (not null)
     */
    protected void setupMaterials(AssetManager am) {
        assert am != null;

        Material invisible = MyAsset.createInvisibleMaterial(am);
        blues[0] = invisible;
        blues[1] = MyAsset.createWireframeMaterial(am, ColorRGBA.Blue);
        blues[1].setName("debug blue ss");
        blues[2] = MyAsset.createWireframeMaterial(am, ColorRGBA.Blue);
        blues[2].setName("debug blue ds");
        RenderState renderState = blues[2].getAdditionalRenderState();
        renderState.setFaceCullMode(RenderState.FaceCullMode.Off);

        green = MyAsset.createWireframeMaterial(am, ColorRGBA.Green);
        green.setName("debug green");
        renderState = green.getAdditionalRenderState();
        renderState.setFaceCullMode(RenderState.FaceCullMode.Off);

        magentas[0] = invisible;
        magentas[1] = MyAsset.createWireframeMaterial(am, ColorRGBA.Magenta);
        magentas[1].setName("debug magenta ss");
        magentas[2] = MyAsset.createWireframeMaterial(am, ColorRGBA.Magenta);
        magentas[2].setName("debug magenta ds");
        renderState = magentas[2].getAdditionalRenderState();
        renderState.setFaceCullMode(RenderState.FaceCullMode.Off);

        pink[0] = invisible;
        pink[1] = MyAsset.createWireframeMaterial(am, ColorRGBA.Pink);
        pink[1].setName("debug pink ss");
        pink[2] = MyAsset.createWireframeMaterial(am, ColorRGBA.Pink);
        pink[2].setName("debug pink ds");
        renderState = pink[2].getAdditionalRenderState();
        renderState.setFaceCullMode(RenderState.FaceCullMode.Off);

        red = MyAsset.createWireframeMaterial(am, ColorRGBA.Red);
        red.setName("debug red");
        renderState = red.getAdditionalRenderState();
        renderState.setFaceCullMode(RenderState.FaceCullMode.Off);

        white = MyAsset.createWireframeMaterial(am, ColorRGBA.White);
        white.setName("debug white");
        renderState = white.getAdditionalRenderState();
        renderState.setFaceCullMode(RenderState.FaceCullMode.Off);

        yellows[0] = invisible;
        yellows[1] = MyAsset.createWireframeMaterial(am, ColorRGBA.Yellow);
        yellows[1].setName("debug yellow ss");
        yellows[2] = MyAsset.createWireframeMaterial(am, ColorRGBA.Yellow);
        yellows[2].setName("debug yellow ds");
        renderState = yellows[2].getAdditionalRenderState();
        renderState.setFaceCullMode(RenderState.FaceCullMode.Off);
    }

    /**
     * Update the AxesVisualizer for the specified Node.
     *
     * @param node the transformed Node to update (not null)
     * @param displayShape true shape is visualized, otherwise false
     */
    protected void updateAxes(Node node, boolean displayShape) {
        boolean displayAxes = displayShape && axisLength > 0f;
        AxesVisualizer control = node.getControl(AxesVisualizer.class);
        if (control != null) {
            if (displayAxes) {
                control.setAxisLength(axisLength);
                control.setLineWidth(axisLineWidth);
            } else {
                control.setEnabled(false);
                node.removeControl(control);
            }
        } else if (displayAxes) {
            control = new AxesVisualizer(assetManager, axisLength,
                    axisLineWidth);
            node.addControl(control);
            control.setEnabled(true);
        }
    }

    /**
     * Synchronize the collision-shape debug controls and axis visualizers with
     * the collision objects in the PhysicsSpace.
     */
    protected void updateShapes() {
        for (Map.Entry<PhysicsCollisionObject, Node> entry
                : pcoMap.entrySet()) {
            PhysicsCollisionObject pco = entry.getKey();
            boolean displayShape = (filter == null)
                    || filter.displayObject(pco);

            Node node = entry.getValue();
            Control control;
            if (pco instanceof PhysicsCharacter) {
                control = node.getControl(BulletCharacterDebugControl.class);
                if (control == null && displayShape) {
                    logger.log(Level.FINE,
                            "Create new BulletCharacterDebugControl");
                    control = new BulletCharacterDebugControl(this,
                            (PhysicsCharacter) pco);
                    node.addControl(control);
                } else if (control != null && !displayShape) {
                    node.removeControl(control);
                }
                updateAxes(node, displayShape);

            } else if (pco instanceof PhysicsGhostObject) {
                control = node.getControl(BulletGhostObjectDebugControl.class);
                if (control == null && displayShape) {
                    logger.log(Level.FINE,
                            "Create new BulletGhostObjectDebugControl");
                    control = new BulletGhostObjectDebugControl(this,
                            (PhysicsGhostObject) pco);
                    node.addControl(control);
                } else if (control != null && !displayShape) {
                    node.removeControl(control);
                }
                updateAxes(node, displayShape);

            } else if (pco instanceof PhysicsRigidBody) {
                control = node.getControl(BulletRigidBodyDebugControl.class);
                if (control == null && displayShape) {
                    logger.log(Level.FINE,
                            "Create new BulletRigidBodyDebugControl");
                    control = new BulletRigidBodyDebugControl(this,
                            (PhysicsRigidBody) pco);
                    node.addControl(control);
                } else if (control != null && !displayShape) {
                    node.removeControl(control);
                }
                updateAxes(node, displayShape);
            }
        }
    }
    // *************************************************************************
    // AbstractAppState methods

    /**
     * Transition this state from terminating to detached. Should be invoked
     * only by a subclass or by the AppStateManager. Invoked once for each time
     * {@link #initialize(com.jme3.app.state.AppStateManager, com.jme3.app.Application)}
     * is invoked.
     */
    @Override
    public void cleanup() {
        for (ViewPort viewPort : viewPorts) {
            viewPort.detachScene(physicsDebugRootNode);
        }
        super.cleanup();
    }

    /**
     * Initialize this state prior to its first update. Should be invoked only
     * by a subclass or by the AppStateManager.
     *
     * @param stateManager the manager for this state (not null)
     * @param app the application which owns this state (not null)
     */
    @Override
    public void initialize(AppStateManager stateManager, Application app) {
        super.initialize(stateManager, app);

        assetManager = app.getAssetManager();
        setupMaterials(assetManager);

        if (initListener != null) {
            initListener.bulletDebugInit(physicsDebugRootNode);
        }

        for (ViewPort viewPort : viewPorts) {
            viewPort.attachScene(physicsDebugRootNode);
        }
    }

    /**
     * Render this state. Should be invoked only by a subclass or by the
     * AppStateManager. Invoked once per frame, provided the state is attached
     * and enabled.
     *
     * @param rm the render manager (not null)
     */
    @Override
    public void render(RenderManager rm) {
        super.render(rm);
        for (ViewPort viewPort : viewPorts) {
            if (viewPort.isEnabled()) {
                rm.renderScene(physicsDebugRootNode, viewPort);
            }
        }
    }

    /**
     * Update this state prior to rendering. Should be invoked only by a
     * subclass or by the AppStateManager. Invoked once per frame, provided the
     * state is attached and enabled.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void update(float tpf) {
        super.update(tpf);

        updatePcoMap();
        updateShapes();
        updateVehicles();
        updateBoundingBoxes();
        updateSweptSpheres();
        updateJoints();

        // Update the debug root node.
        physicsDebugRootNode.updateLogicalState(tpf);
        physicsDebugRootNode.updateGeometricState();
    }
    // *************************************************************************
    // private methods

    /**
     * Synchronize the bounding-box debug controls with the collision objects in
     * the PhysicsSpace.
     */
    private void updateBoundingBoxes() {
        if (boundingBoxFilter == null) {
            return;
        }

        for (Map.Entry<PhysicsCollisionObject, Node> entry : pcoMap.entrySet()) {
            PhysicsCollisionObject pco = entry.getKey();
            boolean display = boundingBoxFilter.displayObject(pco);

            Node transformedNode = entry.getValue();
            Node parent = transformedNode.getParent();
            Control control = parent.getControl(BoundingBoxDebugControl.class);

            if (control == null && display) {
                logger.log(Level.FINE, "Create new BoundingBoxDebugControl");
                control = new BoundingBoxDebugControl(this, pco);
                parent.addControl(control);
            } else if (control != null && !display) {
                parent.removeControl(control);
            }
        }
    }

    /**
     * Synchronize the joint debug controls with the joints in the PhysicsSpace.
     */
    private void updateJoints() {
        HashMap<PhysicsJoint, Node> oldMap = jointMap;
        //create new map
        jointMap = new HashMap<>(oldMap.size());
        Collection<PhysicsJoint> list = space.getJointList();
        for (PhysicsJoint joint : list) {
            if (filter == null || filter.displayObject(joint)) {
                Node node = oldMap.remove(joint);
                if (node == null) {
                    node = new Node(joint.toString());
                    attachChild(node);

                    Control control;
                    if (joint instanceof Anchor) {
                        logger.log(Level.FINE, "Create new AnchorDebugControl");
                        Anchor anchor = (Anchor) joint;
                        control = new AnchorDebugControl(this, anchor);

                    } else if (joint instanceof Constraint) {
                        logger.log(Level.FINE,
                                "Create new ConstraintDebugControl");
                        Constraint constraint = (Constraint) joint;
                        control = new ConstraintDebugControl(this, constraint);

                    } else {
                        logger.log(Level.FINE,
                                "Create new SoftJointDebugControl");
                        SoftPhysicsJoint softJoint = (SoftPhysicsJoint) joint;
                        control = new SoftJointDebugControl(this, softJoint);
                    }
                    node.addControl(control);
                }
                jointMap.put(joint, node);
            }
        }
        // Detach any leftover nodes.
        for (Node node : oldMap.values()) {
            node.removeFromParent();
        }
    }

    /**
     * Synchronize the visualization nodes with the collision objects in the
     * PhysicsSpace.
     */
    private void updatePcoMap() {
        /*
         * Create visualization nodes for PCOs that have been added.
         */
        HashMap<PhysicsCollisionObject, Node> oldMap = pcoMap;
        pcoMap = new HashMap<>(oldMap.size());
        Collection<PhysicsCollisionObject> list = space.getPcoList();
        for (PhysicsCollisionObject pco : list) {
            Node node = oldMap.remove(pco);
            if (node == null) {
                // 2 nodes for each PCO
                Node parent = new Node(pco.toString());
                attachChild(parent);
                node = new Node(pco + " transformed");
                parent.attachChild(node);
            }
            pcoMap.put(pco, node);
        }
        /*
         * Detach nodes of PCOs that have been removed from the space.
         */
        for (Node transformedNode : oldMap.values()) {
            Node parent = transformedNode.getParent();
            parent.removeFromParent();
        }
    }

    /**
     * Synchronize the swept-sphere debug controls with the collision objects in
     * the PhysicsSpace.
     */
    private void updateSweptSpheres() {
        if (sweptSphereFilter == null) {
            return;
        }

        for (Map.Entry<PhysicsCollisionObject, Node> entry : pcoMap.entrySet()) {
            PhysicsCollisionObject pco = entry.getKey();
            boolean display = sweptSphereFilter.displayObject(pco)
                    && pco.getCcdMotionThreshold() > 0f
                    && pco.getCcdSweptSphereRadius() > 0f;

            Node transformedNode = entry.getValue();
            Node parent = transformedNode.getParent();
            Control control = parent.getControl(SweptSphereDebugControl.class);

            if (control == null && display) {
                logger.log(Level.FINE, "Create new SweptSphereDebugControl");
                control = new SweptSphereDebugControl(this, pco);
                parent.addControl(control);

            } else if (control != null && !display) {
                parent.removeControl(control);
            }
        }
    }

    /**
     * Synchronize the vehicle debug controls with the vehicles in the
     * PhysicsSpace.
     */
    private void updateVehicles() {
        for (PhysicsVehicle vehicle : space.getVehicleList()) {
            boolean display = (filter == null || filter.displayObject(vehicle));

            Node node = pcoMap.get(vehicle);
            Control control = node.getControl(BulletVehicleDebugControl.class);

            if (control == null && display) {
                logger.log(Level.FINE, "Create new BulletVehicleDebugControl");
                control = new BulletVehicleDebugControl(this, vehicle);
                node.addControl(control);

            } else if (control != null && !display) {
                node.removeControl(control);
            }
        }
    }

    /**
     * Interface to restrict which physics objects are visualized.
     */
    public interface DebugAppStateFilter {
        /**
         * Test whether the specified physics object should be rendered in the
         * debug scene.
         *
         * @param obj the joint or collision object to test (unaffected)
         * @return return true if the object should be rendered, false if not
         */
        boolean displayObject(Object obj);
    }
}
