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
import com.jme3.app.state.BaseAppState;
import com.jme3.asset.AssetManager;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.joints.Anchor;
import com.jme3.bullet.joints.Constraint;
import com.jme3.bullet.joints.JointEnd;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.joints.SoftPhysicsJoint;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsCharacter;
import com.jme3.bullet.objects.PhysicsGhostObject;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsVehicle;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Transform;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.renderer.queue.RenderQueue;
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
import jme3utilities.math.MyMath;

/**
 * An AppState to manage debug visualization of a PhysicsSpace.
 *
 * @author normenhansen
 */
public class BulletDebugAppState extends BaseAppState {
    // *************************************************************************
    // constants and loggers

    /**
     * local copy of {@link com.jme3.math.ColorRGBA#Blue}
     */
    final private static ColorRGBA blueColor = new ColorRGBA(0f, 0f, 1f, 1f);
    /**
     * local copy of {@link com.jme3.math.ColorRGBA#Brown}
     */
    final private static ColorRGBA brownColor
            = new ColorRGBA(65f / 255f, 40f / 255f, 25f / 255f, 1f);
    /**
     * local copy of {@link com.jme3.math.ColorRGBA#Cyan}
     */
    final private static ColorRGBA cyanColor = new ColorRGBA(0f, 1f, 1f, 1f);
    /**
     * local copy of {@link com.jme3.math.ColorRGBA#Green}
     */
    final private static ColorRGBA greenColor = new ColorRGBA(0f, 1f, 0f, 1f);
    /**
     * local copy of {@link com.jme3.math.ColorRGBA#Magenta}
     */
    final private static ColorRGBA magentaColor = new ColorRGBA(1f, 0f, 1f, 1f);
    /**
     * local copy of {@link com.jme3.math.ColorRGBA#Orange}
     */
    final private static ColorRGBA orangeColor
            = new ColorRGBA(251f / 255f, 130f / 255f, 0f, 1f);
    /**
     * local copy of {@link com.jme3.math.ColorRGBA#Pink}
     */
    final private static ColorRGBA pinkColor
            = new ColorRGBA(1f, 0.68f, 0.68f, 1f);
    /**
     * local copy of {@link com.jme3.math.ColorRGBA#Red}
     */
    final private static ColorRGBA redColor = new ColorRGBA(1f, 0f, 0f, 1f);
    /**
     * local copy of {@link com.jme3.math.ColorRGBA#White}
     */
    final private static ColorRGBA whiteColor = new ColorRGBA(1f, 1f, 1f, 1f);
    /**
     * local copy of {@link com.jme3.math.ColorRGBA#Yellow}
     */
    final private static ColorRGBA yellowColor = new ColorRGBA(1f, 1f, 0f, 1f);
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(BulletDebugAppState.class.getName());
    /**
     * fake Material to indicate child coloring of a CompoundCollisionShape
     */
    final public static Material enableChildColoring = new Material();
    /**
     * local copy of {@link com.jme3.math.Transform#IDENTITY}
     */
    final private static Transform transformIdentity = new Transform();
    // *************************************************************************
    // fields

    /**
     * application's asset manager: set by initialize()
     */
    private AssetManager assetManager;
    /**
     * configuration
     */
    final private DebugConfiguration configuration;
    /**
     * map collision objects to transformed visualization nodes
     */
    private HashMap<PhysicsCollisionObject, Node> pcoMap = new HashMap<>(64);
    /**
     * map physics joints to visualization nodes
     */
    private HashMap<PhysicsJoint, Node> jointMap = new HashMap<>(64);
    /**
     * materials for rigid bodies (and vehicle chassis and colliders) that are
     * responsive and either static or kinematic or inactive
     */
    final private Material[] blues = new Material[3];
    /**
     * materials for child coloring of compound shapes
     */
    final private Material[] childMaterials = new Material[10];
    /**
     * Material for gravity vectors
     */
    private Material gravity;
    /**
     * Material for PhysicsJoint arrows (their A ends)
     */
    private Material jointMaterialA;
    /**
     * Material for PhysicsJoint arrows (their B ends)
     */
    private Material jointMaterialB;
    /**
     * materials for rigid bodies (and vehicle chassis and colliders) that are
     * responsive, dynamic, and active
     */
    final private Material[] magentas = new Material[3];
    /**
     * materials for responsive physics characters
     */
    final private Material[] pink = new Material[3];
    /**
     * Material for bounding boxes, swept spheres, and velocity vectors
     */
    private Material white;
    /**
     * materials for ghosts and other non-responsive collision objects
     */
    final private Material[] yellows = new Material[3];
    /**
     * scene-graph node for (debug) visualization
     */
    final private Node root = new Node("Physics Debug Root Node");
    // *************************************************************************
    // constructors

    /**
     * Instantiate an AppState with the specified configuration. This
     * constructor should be invoked only by BulletAppState.
     *
     * @param config the desired configuration (not null, alias created)
     */
    public BulletDebugAppState(DebugConfiguration config) {
        Validate.nonNull(config, "configuration");
        this.configuration = config;
    }
    // *************************************************************************
    // new methods exposed

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
     * Access the Material for visualizing angular-velocity vectors.
     *
     * @return the pre-existing Material (not null)
     */
    Material getAngularVelocityMaterial() {
        assert magentas[2] != null;
        return magentas[2];
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
     * Access a Material for coloring the indexed child of a
     * CompoundCollisionShape.
     *
     * @param childIndex the child's position in the list of children (&ge;0)
     * @return the pre-existing Material for that position (not null)
     */
    Material getChildMaterial(int childIndex) {
        assert childIndex >= 0 : childIndex;

        int materialIndex = MyMath.modulo(childIndex, childMaterials.length);
        Material result = childMaterials[materialIndex];

        return result;
    }

    /**
     * Access the configuration.
     *
     * @return the pre-existing instance (not null)
     */
    DebugConfiguration getConfiguration() {
        return configuration;
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
     * Access the Material for visualizing gravity vectors.
     *
     * @return the pre-existing Material (not null)
     */
    Material getGravityVectorMaterial() {
        assert gravity != null;
        return gravity;
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
     * Access a Material for visualizing PhysicsJoints.
     *
     * @param end which end to visualize (not null)
     * @return the pre-existing Material (not null)
     */
    Material getJointMaterial(JointEnd end) {
        Material result;
        switch (end) {
            case A:
                result = jointMaterialA;
                break;
            case B:
                result = jointMaterialB;
                break;
            default:
                throw new IllegalArgumentException(end.toString());
        }

        assert result != null;
        return result;
    }

    /**
     * Access the Node containing all the debug visualization.
     *
     * @return the pre-existing instance, or null if unknown
     */
    public Node getRootNode() {
        return root;
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
     * Access the Material for visualizing velocity vectors.
     *
     * @return the pre-existing Material (not null)
     */
    Material getVelocityVectorMaterial() {
        assert white != null;
        return white;
    }

    /**
     * Alter which angular velocities are visualized. For internal use only.
     *
     * @param filter the desired filter, or null to visualize no angular
     * velocities
     */
    public void setAngularVelocityFilter(DebugAppStateFilter filter) {
        configuration.setAngularVelocityFilter(filter);

        for (Node transformedNode : pcoMap.values()) {
            Node parent = transformedNode.getParent();
            Control control
                    = parent.getControl(AngularVelocityDebugControl.class);
            parent.removeControl(control);
        }
    }

    /**
     * Alter which bounding boxes are visualized. For internal use only.
     *
     * @param filter the desired filter, or null to visualize no bounding boxes
     */
    public void setBoundingBoxFilter(DebugAppStateFilter filter) {
        configuration.setBoundingBoxFilter(filter);

        for (Node transformedNode : pcoMap.values()) {
            Node parent = transformedNode.getParent();
            Control control = parent.getControl(BoundingBoxDebugControl.class);
            parent.removeControl(control);
        }
    }

    /**
     * Alter which physics objects are visualized. For compatibility with the
     * jme3-jbullet library.
     *
     * @param filter the desired filter (alias created) or null to visualize all
     * objects
     */
    public void setFilter(DebugAppStateFilter filter) {
        configuration.setFilter(filter);
    }

    /**
     * Alter which gravity vectors are visualized. For internal use only.
     *
     * @param filter the desired filter (alias created) or null to visualize no
     * gravity vectors
     */
    public void setGravityVectorFilter(DebugAppStateFilter filter) {
        configuration.setGravityVectorFilter(filter);

        for (Node transformedNode : pcoMap.values()) {
            Node parent = transformedNode.getParent();
            Control control
                    = parent.getControl(GravityVectorDebugControl.class);
            parent.removeControl(control);
        }
    }

    /**
     * Alter the line width for PhysicsJoint arrows. For internal use only.
     *
     * @param width (in pixels, &ge;1, default=1)
     */
    public void setJointLineWidth(float width) {
        Validate.inRange(width, "width", 1f, Float.MAX_VALUE);

        configuration.setJointLineWidth(width);
        if (jointMaterialA != null) {
            RenderState rs = jointMaterialA.getAdditionalRenderState();
            rs.setLineWidth(width);

            rs = jointMaterialB.getAdditionalRenderState();
            rs.setLineWidth(width);
        }
    }

    /**
     * Alter which swept spheres are visualized. For internal use only.
     *
     * @param filter the desired filter, or null to visualize no swept spheres
     */
    public void setSweptSphereFilter(DebugAppStateFilter filter) {
        configuration.setSweptSphereFilter(filter);

        for (Node transformedNode : pcoMap.values()) {
            Node parent = transformedNode.getParent();
            Control control = parent.getControl(SweptSphereDebugControl.class);
            parent.removeControl(control);
        }
    }

    /**
     * Alter which velocity vectors are visualized. For internal use only.
     *
     * @param filter the desired filter, or null to visualize no velocity
     * vectors
     */
    public void setVelocityVectorFilter(DebugAppStateFilter filter) {
        configuration.setVelocityVectorFilter(filter);

        for (Node transformedNode : pcoMap.values()) {
            Node parent = transformedNode.getParent();
            Control control
                    = parent.getControl(VelocityVectorDebugControl.class);
            parent.removeControl(control);
        }
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
        root.attachChild(spatial);
    }

    /**
     * Create the specified wireframe material.
     *
     * @param assetManager the application's AssetManager (not null)
     * @param color the desired color (not null, unaffected)
     * @param name the desired name for the Material
     * @param numSides the desired number of sides (1 or 2)
     * @return a new instance
     */
    protected Material createWireMaterial(AssetManager assetManager,
            ColorRGBA color, String name, int numSides) {
        Validate.nonNull(assetManager, "asset manager");
        Validate.nonNull(color, "color");
        Validate.inRange(numSides, "number of sides", 1, 2);

        Material result = MyAsset.createWireframeMaterial(assetManager, color);
        result.setName(name);

        RenderState renderState = result.getAdditionalRenderState();
        if (numSides > 1) {
            renderState.setFaceCullMode(RenderState.FaceCullMode.Off);
        }

        return result;
    }

    /**
     * Access the map from collision objects to transformed visualization nodes.
     *
     * @return the pre-existing instance
     */
    protected HashMap<PhysicsCollisionObject, Node> getPcoMap() {
        // TODO should return Map<>
        return pcoMap;
    }

    /**
     * Initialize the wireframe materials and child materials.
     *
     * @param am the application's AssetManager (not null)
     */
    protected void setupMaterials(AssetManager am) {
        assert am != null;

        Material invisible = MyAsset.createInvisibleMaterial(am);
        this.blues[0] = invisible;
        this.blues[1] = createWireMaterial(am, blueColor, "debug blue ss", 1);
        this.blues[1].getAdditionalRenderState().setWireframe(true);
        this.blues[1].setName("debug blue ss");
        this.blues[2] = createWireMaterial(am, blueColor, "debug blue ds", 2);

        this.childMaterials[0] = MyAsset.createUnshadedMaterial(am, whiteColor);
        this.childMaterials[1] = MyAsset.createUnshadedMaterial(am, redColor);
        this.childMaterials[2] = MyAsset.createUnshadedMaterial(am, greenColor);
        this.childMaterials[3] = MyAsset.createUnshadedMaterial(am, blueColor);
        this.childMaterials[4]
                = MyAsset.createUnshadedMaterial(am, yellowColor);
        this.childMaterials[5] = MyAsset.createUnshadedMaterial(am, cyanColor);
        this.childMaterials[6]
                = MyAsset.createUnshadedMaterial(am, orangeColor);
        this.childMaterials[7]
                = MyAsset.createUnshadedMaterial(am, magentaColor);
        this.childMaterials[8] = MyAsset.createUnshadedMaterial(am, pinkColor);
        this.childMaterials[9] = MyAsset.createUnshadedMaterial(am, brownColor);
        for (int childI = 0; childI < childMaterials.length; ++childI) {
            childMaterials[childI].setName("debug child " + childI);
        }

        this.gravity = createWireMaterial(am, cyanColor, "debug gravity", 2);

        this.jointMaterialA = createWireMaterial(
                am, greenColor, "debug joint A wire", 2);
        this.jointMaterialB = createWireMaterial(
                am, redColor, "debug joint B wire", 2);
        float jointLineWidth = configuration.jointLineWidth();
        setJointLineWidth(jointLineWidth);

        this.magentas[0] = invisible;
        this.magentas[1] = createWireMaterial(
                am, magentaColor, "debug magenta ss", 1);
        this.magentas[2] = createWireMaterial(
                am, magentaColor, "debug magenta ds", 2);

        this.pink[0] = invisible;
        this.pink[1] = createWireMaterial(am, pinkColor, "debug pink ss", 1);
        this.pink[2] = createWireMaterial(am, pinkColor, "debug pink ds", 2);

        this.white = createWireMaterial(am, whiteColor, "debug white", 2);

        this.yellows[0] = invisible;
        this.yellows[1]
                = createWireMaterial(am, yellowColor, "debug yellow ss", 1);
        this.yellows[2]
                = createWireMaterial(am, yellowColor, "debug yellow ds", 2);
    }

    /**
     * Update the AxesVisualizer for the specified Node.
     *
     * @param node the transformed Node to update (not null)
     * @param displayShape true shape is visualized, otherwise false
     */
    protected void updateAxes(Node node, boolean displayShape) {
        float axisLength = configuration.axisArrowLength();
        float axisLineWidth = configuration.axisLineWidth();
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
            control = new AxesVisualizer(
                    assetManager, axisLength, axisLineWidth);
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
            DebugAppStateFilter filter = configuration.getFilter();
            PhysicsCollisionObject pco = entry.getKey();
            boolean displayShape
                    = (filter == null) || filter.displayObject(pco);

            Node node = entry.getValue();
            Control control;
            if (pco instanceof PhysicsCharacter) {
                control = node.getControl(BulletCharacterDebugControl.class);
                if (control == null && displayShape) {
                    logger.log(Level.FINE,
                            "Create new BulletCharacterDebugControl");
                    control = new BulletCharacterDebugControl(
                            this, (PhysicsCharacter) pco);
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
                    control = new BulletGhostObjectDebugControl(
                            this, (PhysicsGhostObject) pco);
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
                    control = new BulletRigidBodyDebugControl(
                            this, (PhysicsRigidBody) pco);
                    node.addControl(control);
                } else if (control != null && !displayShape) {
                    node.removeControl(control);
                }
                updateAxes(node, displayShape);
            }
        }
    }

    /**
     * Synchronize the velocity visualizers with the collision objects in the
     * PhysicsSpace.
     */
    protected void updateVelocities() {
        updateAngularVelocities();
        updateVelocityVectors();
    }
    // *************************************************************************
    // BaseAppState methods

    /**
     * Transition this state from terminating to detached. Should be invoked
     * only by a subclass or by the AppStateManager.
     * <p>
     * Invoked once for each time {@link #initialize(
     * com.jme3.app.state.AppStateManager, com.jme3.app.Application)} is
     * invoked.
     *
     * @param app the application which owns this state (not null)
     */
    @Override
    protected void cleanup(Application app) {
        // do nothing
    }

    /**
     * Initialize this state prior to its first update. Should be invoked only
     * by a subclass or by the AppStateManager.
     *
     * @param app the application which owns this state (not null)
     */
    @Override
    public void initialize(Application app) {
        assetManager = app.getAssetManager();
        setupMaterials(assetManager);

        DebugInitListener listener = configuration.getInitListener();
        if (listener != null) {
            listener.bulletDebugInit(root);
        }

        RenderQueue.ShadowMode mode = configuration.shadowMode();
        root.setShadowMode(mode);
    }

    /**
     * Transition this state from enabled to disabled.
     */
    @Override
    protected void onDisable() {
        ViewPort[] viewPorts = configuration.listViewPorts();
        for (ViewPort viewPort : viewPorts) {
            viewPort.detachScene(root);
        }
    }

    /**
     * Transition this state from disabled to enabled.
     */
    @Override
    protected void onEnable() {
        ViewPort[] viewPorts = configuration.listViewPorts();
        for (ViewPort viewPort : viewPorts) {
            viewPort.attachScene(root);
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
        configuration.renderAllViewPorts(rm, root);
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
        updateGravityVectors();
        updateSweptSpheres();
        updateVelocities();
        updateJoints();

        // Update the (debug) root node.
        root.updateLogicalState(tpf);
        Spatial transformSpatial = configuration.getTransformSpatial();
        if (transformSpatial == null) {
            root.setLocalTransform(transformIdentity);
        } else {
            Transform transform = transformSpatial.getWorldTransform(); // alias
            root.setLocalTransform(transform);
        }
        root.updateGeometricState();
    }
    // *************************************************************************
    // private methods

    /**
     * Synchronize the angular-velocity debug controls with the dynamic rigid
     * bodies in the PhysicsSpace.
     */
    private void updateAngularVelocities() {
        DebugAppStateFilter filter = configuration.getAngularVelocityFilter();
        if (filter == null) {
            return;
        }

        for (Map.Entry<PhysicsCollisionObject, Node> entry
                : pcoMap.entrySet()) {
            PhysicsCollisionObject pco = entry.getKey();
            boolean display = pco instanceof PhysicsRigidBody
                    && ((PhysicsRigidBody) pco).isDynamic()
                    && filter.displayObject(pco);

            Node transformedNode = entry.getValue();
            Node parent = transformedNode.getParent();
            Control control
                    = parent.getControl(AngularVelocityDebugControl.class);

            if (control == null && display) {
                logger.log(Level.FINE, "Create AngularVelocityDebugControl");
                control = new AngularVelocityDebugControl(this, pco);
                parent.addControl(control);
            } else if (control != null && !display) {
                parent.removeControl(control);
            }
        }
    }

    /**
     * Synchronize the bounding-box debug controls with the collision objects in
     * the PhysicsSpace.
     */
    private void updateBoundingBoxes() {
        DebugAppStateFilter filter = configuration.getBoundingBoxFilter();
        if (filter == null) {
            return;
        }

        for (Map.Entry<PhysicsCollisionObject, Node> entry
                : pcoMap.entrySet()) {
            PhysicsCollisionObject pco = entry.getKey();
            boolean display = filter.displayObject(pco);

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
     * Synchronize the gravity-vector debug controls with the bodies in the
     * PhysicsSpace.
     */
    private void updateGravityVectors() {
        DebugAppStateFilter filter
                = configuration.getGravityVectorFilter();
        if (filter == null) {
            return;
        }

        for (Map.Entry<PhysicsCollisionObject, Node> entry
                : pcoMap.entrySet()) {
            PhysicsCollisionObject pco = entry.getKey();
            boolean pcoIsKinematic = pco instanceof PhysicsRigidBody
                    && ((PhysicsRigidBody) pco).isKinematic();
            boolean display = pco instanceof PhysicsBody
                    && !pco.isStatic()
                    && !pcoIsKinematic
                    && filter.displayObject(pco);

            Node transformedNode = entry.getValue();
            Node parent = transformedNode.getParent();
            Control control
                    = parent.getControl(GravityVectorDebugControl.class);

            if (control == null && display) {
                logger.log(Level.FINE, "Create new GravityVectorDebugControl");
                control = new GravityVectorDebugControl(this, pco);
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
        DebugAppStateFilter filter = configuration.getFilter();
        HashMap<PhysicsJoint, Node> oldMap = jointMap;
        // create new map
        jointMap = new HashMap<>(oldMap.size());
        PhysicsSpace space = configuration.getSpace();
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
        // Create visualization nodes for PCOs that have been added.
        HashMap<PhysicsCollisionObject, Node> oldMap = pcoMap;
        this.pcoMap = new HashMap<>(oldMap.size());
        PhysicsSpace space = configuration.getSpace();
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

        // Detach nodes of PCOs that have been removed from the space.
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
        DebugAppStateFilter filter = configuration.getSweptSphereFilter();
        if (filter == null) {
            return;
        }

        for (Map.Entry<PhysicsCollisionObject, Node> entry
                : pcoMap.entrySet()) {
            PhysicsCollisionObject pco = entry.getKey();
            boolean display = filter.displayObject(pco)
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
        DebugAppStateFilter filter = configuration.getFilter();
        PhysicsSpace space = configuration.getSpace();
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
     * Synchronize the velocity-vector debug controls with the dynamic rigid
     * bodies in the PhysicsSpace.
     */
    private void updateVelocityVectors() {
        DebugAppStateFilter filter = configuration.getVelocityVectorFilter();
        if (filter == null) {
            return;
        }

        for (Map.Entry<PhysicsCollisionObject, Node> entry
                : pcoMap.entrySet()) {
            PhysicsCollisionObject pco = entry.getKey();
            boolean display = pco instanceof PhysicsRigidBody
                    && ((PhysicsRigidBody) pco).isDynamic()
                    && filter.displayObject(pco);

            Node transformedNode = entry.getValue();
            Node parent = transformedNode.getParent();
            Control control
                    = parent.getControl(VelocityVectorDebugControl.class);

            if (control == null && display) {
                logger.log(Level.FINE, "Create new VelocityVectorDebugControl");
                control = new VelocityVectorDebugControl(this, pco);
                parent.addControl(control);
            } else if (control != null && !display) {
                parent.removeControl(control);
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
