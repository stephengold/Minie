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
import com.jme3.scene.Spatial;
import com.jme3.scene.control.Control;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.MyAsset;
import jme3utilities.Validate;

/**
 * An app state to manage a debug visualization of a physics space.
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
    private DebugAppStateFilter filter;
    /**
     * registered init listener, or null if none
     */
    final private DebugInitListener initListener;
    /**
     * map physics characters to visualizations
     */
    private HashMap<PhysicsCharacter, Spatial> characters = new HashMap<>();
    /**
     * map ghosts to visualizations
     */
    private HashMap<PhysicsGhostObject, Spatial> ghosts = new HashMap<>();
    /**
     * map joints to visualizations
     */
    private HashMap<PhysicsJoint, Spatial> joints = new HashMap<>();
    /**
     * map rigid bodies to visualizations
     */
    private HashMap<PhysicsRigidBody, Spatial> bodies = new HashMap<>();
    /**
     * map vehicles to visualizations
     */
    private HashMap<PhysicsVehicle, Spatial> vehicles = new HashMap<>();
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
    final private Node physicsDebugRootNode
            = new Node("Physics Debug Root Node");
    /**
     * physics space to visualize (not null)
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
     * @param space the physics space to visualize (not null, alias created)
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

    private void updateRigidBodies() {
        HashMap<PhysicsRigidBody, Spatial> oldObjects = bodies;
        bodies = new HashMap<>();
        Collection<PhysicsRigidBody> current = space.getRigidBodyList();
        //create new map
        for (PhysicsRigidBody physicsObject : current) {
            //copy existing spatials
            if (oldObjects.containsKey(physicsObject)) {
                Spatial spat = oldObjects.get(physicsObject);
                bodies.put(physicsObject, spat);
                oldObjects.remove(physicsObject);
            } else {
                if (filter == null || filter.displayObject(physicsObject)) {
                    logger.log(Level.FINE, "Create new debug RigidBody");
                    //create new spatial
                    Node node = new Node(physicsObject.toString());
                    Control debugControl = new BulletRigidBodyDebugControl(this,
                            physicsObject);
                    node.addControl(debugControl);
                    bodies.put(physicsObject, node);
                    physicsDebugRootNode.attachChild(node);
                }
            }
        }
        //remove leftover spatials
        for (Map.Entry<PhysicsRigidBody, Spatial> entry : oldObjects.entrySet()) {
            Spatial spatial = entry.getValue();
            spatial.removeFromParent();
        }
    }

    private void updateJoints() {
        HashMap<PhysicsJoint, Spatial> oldObjects = joints;
        joints = new HashMap<>();
        Collection<PhysicsJoint> current = space.getJointList();
        //create new map
        for (PhysicsJoint physicsObject : current) {
            //copy existing spatials
            if (oldObjects.containsKey(physicsObject)) {
                Spatial spat = oldObjects.get(physicsObject);
                joints.put(physicsObject, spat);
                oldObjects.remove(physicsObject);
            } else {
                if (filter == null || filter.displayObject(physicsObject)) {
                    logger.log(Level.FINE, "Create new debug Joint");
                    //create new spatial
                    Node node = new Node(physicsObject.toString());
                    node.addControl(new BulletJointDebugControl(this, physicsObject));
                    joints.put(physicsObject, node);
                    physicsDebugRootNode.attachChild(node);
                }
            }
        }
        //remove leftover spatials
        for (Map.Entry<PhysicsJoint, Spatial> entry : oldObjects.entrySet()) {
            Spatial spatial = entry.getValue();
            spatial.removeFromParent();
        }
    }

    private void updateGhosts() {
        HashMap<PhysicsGhostObject, Spatial> oldObjects = ghosts;
        ghosts = new HashMap<>();
        Collection<PhysicsGhostObject> current = space.getGhostObjectList();
        //create new map
        for (PhysicsGhostObject physicsObject : current) {
            //copy existing spatials
            if (oldObjects.containsKey(physicsObject)) {
                Spatial spat = oldObjects.get(physicsObject);
                ghosts.put(physicsObject, spat);
                oldObjects.remove(physicsObject);
            } else {
                if (filter == null || filter.displayObject(physicsObject)) {
                    logger.log(Level.FINE, "Create new debug GhostObject");
                    //create new spatial
                    Node node = new Node(physicsObject.toString());
                    node.addControl(new BulletGhostObjectDebugControl(this, physicsObject));
                    ghosts.put(physicsObject, node);
                    physicsDebugRootNode.attachChild(node);
                }
            }
        }
        //remove leftover spatials
        for (Map.Entry<PhysicsGhostObject, Spatial> entry : oldObjects.entrySet()) {
            Spatial spatial = entry.getValue();
            spatial.removeFromParent();
        }
    }

    private void updateCharacters() {
        HashMap<PhysicsCharacter, Spatial> oldObjects = characters;
        characters = new HashMap<>();
        Collection<PhysicsCharacter> current = space.getCharacterList();
        //create new map
        for (PhysicsCharacter physicsObject : current) {
            //copy existing spatials
            if (oldObjects.containsKey(physicsObject)) {
                Spatial spat = oldObjects.get(physicsObject);
                characters.put(physicsObject, spat);
                oldObjects.remove(physicsObject);
            } else {
                if (filter == null || filter.displayObject(physicsObject)) {
                    logger.log(Level.FINE, "Create new debug Character");
                    //create new spatial
                    Node node = new Node(physicsObject.toString());
                    node.addControl(new BulletCharacterDebugControl(this, physicsObject));
                    characters.put(physicsObject, node);
                    physicsDebugRootNode.attachChild(node);
                }
            }
        }
        //remove leftover spatials
        for (Map.Entry<PhysicsCharacter, Spatial> entry : oldObjects.entrySet()) {
            Spatial spatial = entry.getValue();
            spatial.removeFromParent();
        }
    }

    private void updateVehicles() {
        HashMap<PhysicsVehicle, Spatial> oldObjects = vehicles;
        vehicles = new HashMap<>();
        Collection<PhysicsVehicle> current = space.getVehicleList();
        //create new map
        for (PhysicsVehicle physicsObject : current) {
            //copy existing spatials
            if (oldObjects.containsKey(physicsObject)) {
                Spatial spat = oldObjects.get(physicsObject);
                vehicles.put(physicsObject, spat);
                oldObjects.remove(physicsObject);
            } else {
                if (filter == null || filter.displayObject(physicsObject)) {
                    logger.log(Level.FINE, "Create new debug Vehicle");
                    //create new spatial
                    Node node = new Node(physicsObject.toString());
                    node.addControl(new BulletVehicleDebugControl(this, physicsObject));
                    vehicles.put(physicsObject, node);
                    physicsDebugRootNode.attachChild(node);
                }
            }
        }
        //remove leftover spatials
        for (Map.Entry<PhysicsVehicle, Spatial> entry : oldObjects.entrySet()) {
            Spatial spatial = entry.getValue();
            spatial.removeFromParent();
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
