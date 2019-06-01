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
import com.jme3.bullet.joints.JointEnd;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.objects.PhysicsCharacter;
import com.jme3.bullet.objects.PhysicsGhostObject;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsVehicle;
import com.jme3.export.Savable;
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
     * length of each axis arrow (in world units, &gt;0) or 0 for no axis arrows
     */
    private float axisLength = 0f;
    /**
     * line width for wireframe axis arrows (in pixels, &ge;1) or 0 for solid
     * axis arrows
     */
    private float axisLineWidth = 1f;
    /**
     * map physics characters to visualization nodes
     */
    private HashMap<PhysicsCharacter, Node> characters = new HashMap<>(64);
    /**
     * map ghosts to visualization nodes
     */
    private HashMap<PhysicsGhostObject, Node> ghosts = new HashMap<>(64);
    /**
     * map joints to visualization nodes
     */
    private HashMap<PhysicsJoint, Node> joints = new HashMap<>(64);
    /**
     * map collision objects to bounding-box visualization nodes
     */
    private HashMap<PhysicsCollisionObject, Node> pcos = new HashMap<>(64);
    /**
     * map rigid bodies to visualization nodes
     */
    private HashMap<PhysicsRigidBody, Node> bodies = new HashMap<>(64);
    /**
     * map vehicles to visualization nodes
     */
    private HashMap<PhysicsVehicle, Node> vehicles = new HashMap<>(64);
    /**
     * material for rigid bodies (and vehicles) that are responsive and either
     * static/kinematic/inactive
     */
    private Material DEBUG_BLUE;
    /**
     * material for joints (the A ends)
     */
    private Material DEBUG_GREEN;
    /**
     * material for rigid bodies (and vehicles) that are responsive and dynamic
     * and active
     */
    private Material DEBUG_MAGENTA;
    /**
     * material for responsive physics characters
     */
    private Material DEBUG_PINK;
    /**
     * material for joints (the B ends) and soft-body links
     */
    protected Material DEBUG_RED;
    /**
     * material for bounding boxes and swept spheres
     */
    private Material DEBUG_WHITE;
    /**
     * material for ghosts and other non-responsive objects
     */
    private Material DEBUG_YELLOW;
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
     * Instantiate an app state to visualize the specified space using the
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
     * @return length (in world units, &ge;0)
     */
    public float axisLength() {
        assert axisLength >= 0f : axisLength;
        return axisLength;
    }

    /**
     * Read the line width of the axis arrows.
     *
     * @return width (in pixels, &ge;1) or 0 for solid arrows
     */
    public float axisLineWidth() {
        assert axisLineWidth >= 0f : axisLineWidth;
        return axisLineWidth;
    }

    /**
     * Access the Material for visualizing active rigid bodies.
     *
     * @return the pre-existing instance (not null)
     */
    Material getActiveMaterial() {
        assert DEBUG_MAGENTA != null;
        return DEBUG_MAGENTA;
    }

    /**
     * Access the Material for visualizing bounding boxes.
     *
     * @return the pre-existing instance (not null)
     */
    Material getBoundingBoxMaterial() {
        assert DEBUG_WHITE != null;
        return DEBUG_WHITE;
    }

    /**
     * Access the Material for visualizing responsive characters.
     *
     * @return the pre-existing instance (not null)
     */
    Material getCharacterMaterial() {
        assert DEBUG_PINK != null;
        return DEBUG_PINK;
    }

    /**
     * Access the Material for visualizing non-responsive objects.
     *
     * @return the pre-existing instance (not null)
     */
    Material getGhostMaterial() {
        assert DEBUG_YELLOW != null;
        return DEBUG_YELLOW;
    }

    /**
     * Access the Material for visualizing inactive objects.
     *
     * @return the pre-existing instance (not null)
     */
    Material getInactiveMaterial() {
        assert DEBUG_BLUE != null;
        return DEBUG_BLUE;
    }

    /**
     * Access a Material for visualizing joints.
     *
     * @param end which end to visualize (not null)
     * @return the pre-existing instance (not null)
     */
    Material getJointMaterial(JointEnd end) {
        Material result;
        switch (end) {
            case A:
                result = DEBUG_GREEN;
                break;
            case B:
                result = DEBUG_RED;
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
     * @return the pre-existing instance (not null)
     */
    Material getSweptSphereMaterial() {
        assert DEBUG_WHITE != null;
        return DEBUG_WHITE;
    }

    /**
     * Alter the length of the axis arrows.
     *
     * @param length (in world units, &ge;0, default=0)
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
     * Initialize the double-sided wire-frame materials. TODO option for
     * single-sided materials
     *
     * @param am the application's AssetManager (not null)
     */
    protected void setupMaterials(AssetManager am) {
        assert am != null;

        DEBUG_BLUE = MyAsset.createWireframeMaterial(am, ColorRGBA.Blue);
        DEBUG_BLUE.setName("DEBUG_BLUE");
        RenderState renderState = DEBUG_BLUE.getAdditionalRenderState();
        renderState.setFaceCullMode(RenderState.FaceCullMode.Off);

        DEBUG_GREEN = MyAsset.createWireframeMaterial(am, ColorRGBA.Green);
        DEBUG_GREEN.setName("DEBUG_GREEN");
        renderState = DEBUG_GREEN.getAdditionalRenderState();
        renderState.setFaceCullMode(RenderState.FaceCullMode.Off);

        DEBUG_MAGENTA = MyAsset.createWireframeMaterial(am, ColorRGBA.Magenta);
        DEBUG_MAGENTA.setName("DEBUG_MAGENTA");
        renderState = DEBUG_MAGENTA.getAdditionalRenderState();
        renderState.setFaceCullMode(RenderState.FaceCullMode.Off);

        DEBUG_PINK = MyAsset.createWireframeMaterial(am, ColorRGBA.Pink);
        DEBUG_PINK.setName("DEBUG_PINK");
        renderState = DEBUG_PINK.getAdditionalRenderState();
        renderState.setFaceCullMode(RenderState.FaceCullMode.Off);

        DEBUG_RED = MyAsset.createWireframeMaterial(am, ColorRGBA.Red);
        DEBUG_RED.setName("DEBUG_RED");
        renderState = DEBUG_RED.getAdditionalRenderState();
        renderState.setFaceCullMode(RenderState.FaceCullMode.Off);

        DEBUG_WHITE = MyAsset.createWireframeMaterial(am, ColorRGBA.White);
        DEBUG_WHITE.setName("DEBUG_WHITE");
        renderState = DEBUG_WHITE.getAdditionalRenderState();
        renderState.setFaceCullMode(RenderState.FaceCullMode.Off);

        DEBUG_YELLOW = MyAsset.createWireframeMaterial(am, ColorRGBA.Yellow);
        DEBUG_YELLOW.setName("DEBUG_YELLOW");
        renderState = DEBUG_YELLOW.getAdditionalRenderState();
        renderState.setFaceCullMode(RenderState.FaceCullMode.Off);
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
     * Initialize this state prior to its 1st update. Should be invoked only by
     * a subclass or by the AppStateManager.
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

        // Update all object links.
        updateRigidBodies();
        updateGhosts();
        updateCharacters();
        updateJoints();
        updateVehicles();
        if (boundingBoxFilter != null) {
            updateBoundingBoxes();
        }
        if (sweptSphereFilter != null) {
            updateSweptSpheres();
        }

        // Update the debug root node.
        physicsDebugRootNode.updateLogicalState(tpf);
        physicsDebugRootNode.updateGeometricState();
    }

    /**
     * Update the AxesVisualizer for the specified Node.
     *
     * @param node which node to update (not null)
     */
    protected void updateAxes(Node node) {
        AxesVisualizer axes = node.getControl(AxesVisualizer.class);
        if (axes != null) {
            if (axisLength > 0f) {
                axes.setAxisLength(axisLength);
                axes.setLineWidth(axisLineWidth);
            } else {
                node.removeControl(axes);
            }
        } else if (axisLength > 0f) {
            axes = new AxesVisualizer(assetManager, axisLength, axisLineWidth);
            node.addControl(axes);
            axes.setEnabled(true);
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Synchronize the bounding-box debug controls with the collision objects in
     * the PhysicsSpace.
     */
    private void updateBoundingBoxes() {
        assert boundingBoxFilter != null;

        HashMap<PhysicsCollisionObject, Node> oldMap = pcos;
        //create new map
        pcos = new HashMap<>(oldMap.size());
        Collection<PhysicsCollisionObject> list = space.getPcoList();
        for (PhysicsCollisionObject pco : list) {
            if (boundingBoxFilter.displayObject(pco)) {
                Node node = oldMap.remove(pco);
                if (node == null) {
                    node = new Node(pco.toString());
                    attachChild(node);

                    logger.log(Level.FINE,
                            "Create new BoundingBoxDebugControl");
                    Control control = new BoundingBoxDebugControl(this, pco);
                    node.addControl(control);
                }
                pcos.put(pco, node);
            }
        }
        // Detach any leftover nodes.
        for (Node node : oldMap.values()) {
            node.removeFromParent();
        }
    }

    /**
     * Synchronize the character debug controls with the characters in the
     * PhysicsSpace.
     */
    private void updateCharacters() {
        HashMap<PhysicsCharacter, Node> oldMap = characters;
        //create new map
        characters = new HashMap<>(oldMap.size());
        Collection<PhysicsCharacter> list = space.getCharacterList();
        for (PhysicsCharacter character : list) {
            if (filter == null || filter.displayObject(character)) {
                Node node = oldMap.remove(character);
                if (node == null) {
                    node = new Node(character.toString());
                    attachChild(node);

                    logger.log(Level.FINE,
                            "Create new BulletCharacterDebugControl");
                    Control control
                            = new BulletCharacterDebugControl(this, character);
                    node.addControl(control);
                }
                characters.put(character, node);
                updateAxes(node);
            }
        }
        // Detach any leftover nodes.
        for (Node node : oldMap.values()) {
            node.removeFromParent();
        }
    }

    /**
     * Synchronize the ghost debug controls with the ghosts in the PhysicsSpace.
     */
    private void updateGhosts() {
        HashMap<PhysicsGhostObject, Node> oldMap = ghosts;
        //create new map
        ghosts = new HashMap<>(oldMap.size());
        Collection<PhysicsGhostObject> list = space.getGhostObjectList();
        for (PhysicsGhostObject ghost : list) {
            if (filter == null || filter.displayObject(ghost)) {
                Node node = oldMap.remove(ghost);
                if (node == null) {
                    node = new Node(ghost.toString());
                    attachChild(node);

                    logger.log(Level.FINE,
                            "Create new BulletGhostObjectDebugControl");
                    Control control
                            = new BulletGhostObjectDebugControl(this, ghost);
                    node.addControl(control);
                }
                ghosts.put(ghost, node);
                updateAxes(node);
            }
        }
        // Detach any leftover nodes.
        for (Node node : oldMap.values()) {
            node.removeFromParent();
        }
    }

    /**
     * Synchronize the joint debug controls with the joints in the PhysicsSpace.
     */
    private void updateJoints() {
        HashMap<PhysicsJoint, Node> oldMap = joints;
        //create new map
        joints = new HashMap<>(oldMap.size());
        Collection<PhysicsJoint> list = space.getJointList();
        for (PhysicsJoint joint : list) {
            if (filter == null || filter.displayObject(joint)) {
                Node node = oldMap.remove(joint);
                if (node == null) {
                    node = new Node(joint.toString());
                    attachChild(node);

                    logger.log(Level.FINE,
                            "Create new BulletJointDebugControl");
                    Control control = new BulletJointDebugControl(this, joint);
                    node.addControl(control);
                }
                joints.put(joint, node);
            }
        }
        // Detach any leftover nodes.
        for (Node node : oldMap.values()) {
            node.removeFromParent();
        }
    }

    /**
     * Synchronize the rigid-body debug controls with the rigid bodies in the
     * PhysicsSpace.
     */
    private void updateRigidBodies() {
        HashMap<PhysicsRigidBody, Node> oldMap = bodies;
        //create new map
        bodies = new HashMap<>(oldMap.size());
        Collection<PhysicsRigidBody> list = space.getRigidBodyList();
        for (PhysicsRigidBody body : list) {
            if (filter == null || filter.displayObject(body)) {
                Node node = oldMap.remove(body);
                if (node == null) {
                    node = new Node(body.toString());
                    attachChild(node);

                    logger.log(Level.FINE,
                            "Create new BulletRigidBodyDebugControl");
                    Control control
                            = new BulletRigidBodyDebugControl(this, body);
                    node.addControl(control);
                }
                bodies.put(body, node);
                updateAxes(node);
            }
        }
        // Detach any leftover nodes.
        for (Node node : oldMap.values()) {
            node.removeFromParent();
        }
    }

    /**
     * Synchronize the swept-sphere debug controls with the collision objects in
     * the PhysicsSpace.
     */
    private void updateSweptSpheres() {
        assert sweptSphereFilter != null;

        HashMap<PhysicsCollisionObject, Node> oldMap = pcos;
        //create new map
        pcos = new HashMap<>(oldMap.size());
        Collection<PhysicsCollisionObject> list = space.getPcoList();
        for (PhysicsCollisionObject pco : list) {
            if (sweptSphereFilter.displayObject(pco)
                    && pco.getCcdMotionThreshold() > 0f
                    && pco.getCcdSweptSphereRadius() > 0f) {
                Node node = oldMap.remove(pco);
                if (node == null) {
                    node = new Node(pco.toString());
                    attachChild(node);

                    logger.log(Level.FINE,
                            "Create new SweptSphereDebugControl");
                    Control control = new SweptSphereDebugControl(this, pco);
                    node.addControl(control);
                }
                pcos.put(pco, node);
            }
        }
        // Detach any leftover nodes.
        for (Node node : oldMap.values()) {
            node.removeFromParent();
        }
    }

    /**
     * Synchronize the vehicle debug controls with the vehicles in the
     * PhysicsSpace.
     */
    private void updateVehicles() {
        HashMap<PhysicsVehicle, Node> oldMap = vehicles;
        //create new map
        vehicles = new HashMap<>(oldMap.size());
        Collection<PhysicsVehicle> list = space.getVehicleList();
        for (PhysicsVehicle vehicle : list) {
            if (filter == null || filter.displayObject(vehicle)) {
                Node node = oldMap.remove(vehicle);
                if (node == null) {
                    node = new Node(vehicle.toString());
                    attachChild(node);

                    logger.log(Level.FINE,
                            "Create new BulletVehicleDebugControl");
                    Control control
                            = new BulletVehicleDebugControl(this, vehicle);
                    node.addControl(control);
                }
                vehicles.put(vehicle, node);
            }
        }
        // Detach any leftover nodes.
        for (Node node : oldMap.values()) {
            node.removeFromParent();
        }
    }

    /**
     * Interface to restrict which physics objects are visualized.
     */
    public static interface DebugAppStateFilter {
        /**
         * Test whether the specified physics object should be displayed.
         *
         * @param obj the joint or collision object to test (unaffected)
         * @return return true if the object should be displayed, false if not
         */
        boolean displayObject(Savable obj);
    }
}
