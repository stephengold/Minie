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
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.objects.PhysicsCharacter;
import com.jme3.bullet.objects.PhysicsGhostObject;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsVehicle;
import com.jme3.export.Savable;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Node;
import com.jme3.scene.control.Control;
import java.util.Collection;
import java.util.HashMap;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.MyAsset;
import jme3utilities.Validate;

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
     * limit which objects are visualized, or null to visualize all objects
     */
    protected DebugAppStateFilter filter;
    /**
     * registered init listener, or null if none
     */
    final private DebugInitListener initListener;
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
     * map rigid bodies to visualization nodes
     */
    private HashMap<PhysicsRigidBody, Node> bodies = new HashMap<>(64);
    /**
     * map vehicles to visualization nodes
     */
    private HashMap<PhysicsVehicle, Node> vehicles = new HashMap<>(64);
    /**
     * material for inactive rigid bodies
     */
    Material DEBUG_BLUE;
    /**
     * material for joints (the A ends)
     */
    Material DEBUG_GREEN;
    /**
     * material for vehicles and active rigid bodies
     */
    Material DEBUG_MAGENTA;
    /**
     * material for physics characters
     */
    Material DEBUG_PINK;
    /**
     * material for joints (the B ends)
     */
    Material DEBUG_RED;
    /**
     * material for ghosts
     */
    Material DEBUG_YELLOW;
    /**
     * scene-graph node to parent the geometries
     */
    final protected Node physicsDebugRootNode
            = new Node("Physics Debug Root Node");
    /**
     * PhysicsSpace to visualize (not null)
     */
    final protected PhysicsSpace space;
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
     * Alter which objects are visualized.
     *
     * @param filter the desired filter, or null to visualize all objects
     */
    public void setFilter(DebugAppStateFilter filter) {
        this.filter = filter;
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

        setupMaterials(app);

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

        // Update the debug root node.
        physicsDebugRootNode.updateLogicalState(tpf);
        physicsDebugRootNode.updateGeometricState();
    }
    // *************************************************************************
    // private methods

    /**
     * Initialize the materials.
     *
     * @param app the application which owns this state (not null)
     */
    private void setupMaterials(Application app) {
        AssetManager am = app.getAssetManager();

        DEBUG_BLUE = MyAsset.createWireframeMaterial(am, ColorRGBA.Blue);
        DEBUG_BLUE.setName("DEBUG_BLUE");
        DEBUG_GREEN = MyAsset.createWireframeMaterial(am, ColorRGBA.Green);
        DEBUG_GREEN.setName("DEBUG_GREEN");
        DEBUG_MAGENTA = MyAsset.createWireframeMaterial(am, ColorRGBA.Magenta);
        DEBUG_MAGENTA.setName("DEBUG_MAGENTA");
        DEBUG_PINK = MyAsset.createWireframeMaterial(am, ColorRGBA.Pink);
        DEBUG_PINK.setName("DEBUG_PINK");
        DEBUG_RED = MyAsset.createWireframeMaterial(am, ColorRGBA.Red);
        DEBUG_RED.setName("DEBUG_RED");
        DEBUG_YELLOW = MyAsset.createWireframeMaterial(am, ColorRGBA.Yellow);
        DEBUG_YELLOW.setName("DEBUG_YELLOW");
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
                    logger.log(Level.FINE,
                            "Create new character debug control");
                    Control control
                            = new BulletCharacterDebugControl(this, character);
                    node = new Node(character.toString());
                    node.addControl(control);
                    physicsDebugRootNode.attachChild(node);
                }
                characters.put(character, node);
            }
        }
        //remove any leftover nodes
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
                    logger.log(Level.FINE, "Create new ghost debug control");
                    Control control
                            = new BulletGhostObjectDebugControl(this, ghost);
                    node = new Node(ghost.toString());
                    node.addControl(control);
                    physicsDebugRootNode.attachChild(node);
                }
                ghosts.put(ghost, node);
            }
        }
        //remove any leftover nodes
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
                    logger.log(Level.FINE, "Create new joint debug control");
                    Control control = new BulletJointDebugControl(this, joint);
                    node = new Node(joint.toString());
                    node.addControl(control);
                    physicsDebugRootNode.attachChild(node);
                }
                joints.put(joint, node);
            }
        }
        //remove any leftover nodes
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
                    logger.log(Level.FINE,
                            "Create new rigid-body debug control");
                    Control control
                            = new BulletRigidBodyDebugControl(this, body);
                    node = new Node(body.toString());
                    node.addControl(control);
                    physicsDebugRootNode.attachChild(node);
                }
                bodies.put(body, node);
            }
        }
        //remove any leftover nodes
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
                    logger.log(Level.FINE, "Create new vehicle debug control");
                    Control control
                            = new BulletVehicleDebugControl(this, vehicle);
                    node = new Node(vehicle.toString());
                    node.addControl(control);
                    physicsDebugRootNode.attachChild(node);
                }
                vehicles.put(vehicle, node);
            }
        }
        //remove any leftover nodes
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
