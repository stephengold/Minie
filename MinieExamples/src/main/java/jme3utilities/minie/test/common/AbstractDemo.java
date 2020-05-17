/*
 Copyright (c) 2018-2020, Stephen Gold
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package jme3utilities.minie.test.common;

import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.ConeCollisionShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.PlaneCollisionShape;
import com.jme3.bullet.collision.shapes.infos.DebugMeshNormals;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.bullet.util.PlaneDmiListener;
import com.jme3.font.Rectangle;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Plane;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Box;
import com.jme3.texture.Texture;
import java.util.Map;
import java.util.TreeMap;
import java.util.logging.Logger;
import jme3utilities.MyAsset;
import jme3utilities.MySpatial;
import jme3utilities.MyString;
import jme3utilities.Validate;
import jme3utilities.debug.AxesVisualizer;
import jme3utilities.math.MyVector3f;
import jme3utilities.mesh.Prism;
import jme3utilities.minie.DumpFlags;
import jme3utilities.minie.FilterAll;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.minie.test.shape.MinieTestShapes;
import jme3utilities.minie.test.shape.ShapeGenerator;
import jme3utilities.ui.ActionApplication;
import jme3utilities.ui.HelpUtils;
import jme3utilities.ui.InputMode;

/**
 * An abstract ActionApplication with additional data and methods to test and/or
 * demonstrate the capabilities of Minie.
 *
 * @author Stephen Gold sgold@sonic.net
 */
abstract public class AbstractDemo extends ActionApplication {
    // *************************************************************************
    // constants and loggers

    /**
     * animation/physics speed when paused
     */
    final public static float pausedSpeed = 1e-12f;
    /**
     * message logger for this class
     */
    final public static Logger loggerA
            = Logger.getLogger(AbstractDemo.class.getName());
    /**
     * action strings that onAction() recognizes
     */
    final public static String asCollectGarbage = "collect garbage";
    final public static String asDumpGui = "dump gui";
    final public static String asDumpPhysicsSpace = "dump physicsSpace";
    final public static String asDumpScene = "dump scene";
    final public static String asDumpScenes = "dump scenes";
    final public static String asDumpViewport = "dump viewport";
    final public static String asToggleAabbs = "toggle aabbs";
    final public static String asToggleCcdSpheres = "toggle ccdSpheres";
    final public static String asToggleGravities = "toggle gravities";
    final public static String asToggleHelp = "toggle help";
    final public static String asTogglePause = "toggle pause";
    final public static String asTogglePcoAxes = "toggle pcoAxes";
    final public static String asTogglePhysicsDebug = "toggle physicsDebug";
    final public static String asToggleVelocities = "toggle velocities";
    final public static String asToggleWorldAxes = "toggle worldAxes";
    // *************************************************************************
    // fields

    /**
     * visualizer for the world axes
     */
    private AxesVisualizer worldAxes = null;
    /**
     * filter to control visualization of axis-aligned bounding boxes
     */
    private FilterAll aabbsFilter = null;
    /**
     * filter to control visualization of CCD swept spheres
     */
    private FilterAll ccdSpheresFilter = null;
    /**
     * filter to control visualization of gravity vectors
     */
    private FilterAll gravitiesFilter = null;
    /**
     * filter to control visualization of velocity vectors
     */
    private FilterAll velocitiesFilter = null;
    /**
     * library of named physics collision shapes
     */
    final private Map<String, CollisionShape> namedShapes = new TreeMap<>();
    /**
     * library of named geometry materials
     */
    final private Map<String, Material> namedMaterials = new TreeMap<>();
    /**
     * node for displaying hotkey help in the GUI scene
     */
    private Node helpNode;
    /**
     * dump debugging information to System.out
     */
    final private PhysicsDumper dumper = new PhysicsDumper();
    /**
     * enhanced pseudo-random generator
     */
    final private ShapeGenerator generator = new ShapeGenerator();
    // *************************************************************************
    // new methods exposed

    /**
     * Activate all rigid bodies in the PhysicsSpace.
     */
    public void activateAll() {
        PhysicsSpace space = getPhysicsSpace();
        space.activateAll(true);
    }

    /**
     * Add the specified collision object to the PhysicsSpace.
     *
     * @param pco the object to add (not null, not in world)
     */
    public void addCollisionObject(PhysicsCollisionObject pco) {
        Validate.nonNull(pco, "pco");
        Validate.require(!pco.isInWorld(), "not in world");

        PhysicsSpace space = getPhysicsSpace();
        space.addCollisionObject(pco);

        postAdd(pco);
    }

    /**
     * Add the specified PhysicsJoint to the PhysicsSpace.
     *
     * @param joint the joint to add (not null)
     */
    public void addJoint(PhysicsJoint joint) {
        Validate.nonNull(joint, "joint");

        PhysicsSpace space = getPhysicsSpace();
        space.add(joint);
    }

    /**
     * Configure the specified platform body and add it to the PhysicsSpace.
     *
     * @param body the body to add (not null, static or soft, not in world)
     */
    public void addPlatform(PhysicsBody body) {
        Validate.nonNull(body, "body");
        Validate.require(!body.isInWorld(), "not in world");
        Validate.require(body.isStatic() || body instanceof PhysicsSoftBody,
                "static or soft");

        Material material = findMaterial("platform");
        assert material != null;
        body.setApplicationData(material);
        body.setDebugMaterial(material);

        addCollisionObject(body);
    }

    /**
     * Add a platform to the PhysicsSpace.
     *
     * @param platformType the name of the desired platform type (not null)
     * @param topY the desired Y coordinate of the top surface (in physics-space
     * coordinates)
     */
    public void addPlatform(String platformType, float topY) {
        Validate.nonNull(platformType, "platform type");

        float topRadius = 20f;
        float thickness = 0.2f * topRadius;
        switch (platformType) {
            case "box":
                addBoxPlatform(topY, topRadius, thickness);
                break;

            case "cone":
                addConePlatform(topY, topRadius);
                break;

            case "cylinder":
                addCylinderPlatform(topY, topRadius);
                break;

            case "hull":
                addHullPlatform(topY, topRadius, thickness);
                break;

            case "plane":
                addPlanePlatform(topY);
                break;

            case "roundedRectangle":
                addRoundedRectangle(topY);
                break;

            case "triangle":
                addPlatform(platformType, DebugMeshNormals.Facet, topY);
                break;

            default:
                String message
                        = "platformType = " + MyString.quote(platformType);
                throw new RuntimeException(message);
        }
    }

    /**
     * Add a static rigid body with the named shape to the PhysicsSpace, to
     * serve as a platform.
     *
     * @param shapeName (not null)
     * @param normals which normals to include in the debug mesh
     * @param centerY the desired Y coordinate of the center (in physics-space
     * coordinates)
     */
    public void addPlatform(String shapeName, DebugMeshNormals normals,
            float centerY) {
        Validate.nonNull(shapeName, "shape name");

        CollisionShape shape = findShape(shapeName);
        assert shape != null : shapeName;
        PhysicsRigidBody body
                = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);

        body.setDebugMeshNormals(normals);
        body.setPhysicsLocation(new Vector3f(0f, centerY, 0f));

        addPlatform(body);
    }

    /**
     * Test whether the world axes are enabled.
     *
     * @return true if enabled, otherwise false
     */
    public boolean areWorldAxesEnabled() {
        boolean result;
        if (worldAxes == null) {
            result = false;
        } else {
            result = worldAxes.isEnabled();
        }

        return result;
    }

    /**
     * Add a static cube to the scene, to serve as a platform.
     *
     * @param halfExtent half the desired extent of the cube (&gt;0)
     * @param topY the desired Y coordinate of the top surface (in physics-space
     * coordinates)
     * @return the new physics control (not null)
     */
    public RigidBodyControl attachCubePlatform(float halfExtent, float topY) {
        Validate.positive(halfExtent, "half extent");

        Mesh mesh = new Box(halfExtent, halfExtent, halfExtent);
        Geometry geometry = new Geometry("cube platform", mesh);
        rootNode.attachChild(geometry);

        float centerY = topY - halfExtent;
        geometry.move(0f, centerY, 0f);
        Material platformMaterial = findMaterial("platform");
        assert platformMaterial != null;
        geometry.setMaterial(platformMaterial);
        geometry.setShadowMode(RenderQueue.ShadowMode.Receive);

        BoxCollisionShape shape = new BoxCollisionShape(halfExtent);
        RigidBodyControl control
                = new RigidBodyControl(shape, PhysicsBody.massForStatic);
        geometry.addControl(control);
        control.setApplyScale(true);
        control.setFriction(0.1f);

        PhysicsSpace space = getPhysicsSpace();
        control.setPhysicsSpace(space);
        geometry.addControl(control);

        return control;
    }

    /**
     * Generate a help node and attach it to the GUI scene.
     *
     * @param bounds the desired screen coordinates (not null, unaffected)
     */
    public void attachHelpNode(Rectangle bounds) {
        Validate.nonNull(bounds, "bounds");

        InputMode inputMode = getDefaultInputMode();
        float extraSpace = 20f;
        helpNode = HelpUtils.buildNode(inputMode, bounds, guiFont, extraSpace);
        guiNode.attachChild(helpNode);
    }

    /**
     * Add a visualizer for the axes of the world coordinate system.
     *
     * @param axisLength the desired length for each axis arrow (in world units,
     * &gt;0)
     */
    public void attachWorldAxes(float axisLength) {
        Validate.positive(axisLength, "axis length");

        if (worldAxes != null) {
            rootNode.removeControl(worldAxes);
        }

        worldAxes = new AxesVisualizer(assetManager, axisLength);
        worldAxes.setLineWidth(AxesVisualizer.widthForSolid);

        rootNode.addControl(worldAxes);
        worldAxes.setEnabled(true);
    }

    /**
     * Translate a model's center so that the model rests on the X-Z plane, and
     * its center lies on the Y axis.
     *
     * @param cgModel (not null, modified)
     */
    public void centerCgm(Spatial cgModel) {
        Validate.nonNull(cgModel, "model");

        Vector3f[] minMax = MySpatial.findMinMaxCoords(cgModel);
        Vector3f min = minMax[0];
        Vector3f max = minMax[1];
        Vector3f center = MyVector3f.midpoint(min, max, null);
        Vector3f offset = new Vector3f(center.x, min.y, center.z);

        Vector3f location = cgModel.getWorldTranslation();
        location.subtractLocal(offset);
        MySpatial.setWorldLocation(cgModel, location);
    }

    /**
     * Configure the PhysicsDumper. Invoke during startup.
     */
    public void configureDumper() {
        dumper.setEnabled(DumpFlags.ChildShapes, true);
        dumper.setEnabled(DumpFlags.JointsInBodies, true);
        dumper.setEnabled(DumpFlags.ShadowModes, true);
        dumper.setEnabled(DumpFlags.Transforms, true);
    }

    /**
     * Describe the current physics debug options, assuming debug is enabled.
     *
     * @return a descriptive string of text (not null, not empty)
     */
    public String describePhysicsDebugOptions() {
        String result = "Physics";

        if (aabbsFilter != null) {
            result += "+AABBs";
        }

        if (ccdSpheresFilter != null) {
            result += "+CcdSpheres";
        }

        if (gravitiesFilter != null) {
            result += "+Gravities";
        }

        BulletAppState bulletAppState = getBulletAppState();
        if (bulletAppState.debugAxisLength() > 0f) {
            result += "+PcoAxes";
        }

        if (velocitiesFilter != null) {
            result += "+Velocities";
        }

        return result;
    }

    /**
     * Find the named Material in the library.
     *
     * @param name the name of the Material to find (not null)
     * @return the pre-existing instance, or null if not found
     */
    public Material findMaterial(String name) {
        Validate.nonNull(name, "name");

        Material result = namedMaterials.get(name);
        return result;
    }

    /**
     * Find the named CollisionShape in the library.
     *
     * @param name the name of the shape to find (not null)
     * @return the pre-existing instance, or null if not found
     */
    public CollisionShape findShape(String name) {
        Validate.nonNull(name, "name");

        CollisionShape result = namedShapes.get(name);
        return result;
    }

    /**
     * Initialize the library of named materials. Invoke during startup.
     */
    public void generateMaterials() {
        ColorRGBA green = new ColorRGBA(0f, 0.12f, 0f, 1f);
        Material platform = MyAsset.createShadedMaterial(assetManager, green);
        registerMaterial("platform", platform);

        Texture texture = MyAsset.loadTexture(assetManager,
                "Textures/greenTile.png", true);
        texture.setMinFilter(Texture.MinFilter.Trilinear);
        texture.setWrap(Texture.WrapMode.Repeat);
        Material greenTile
                = MyAsset.createShadedMaterial(assetManager, texture);
        registerMaterial("greenTile", greenTile);
    }

    /**
     * Initialize the library of named collision shapes. Invoke during startup.
     */
    public void generateShapes() {
        MinieTestShapes.addShapes(namedShapes);
    }

    /**
     * Access the PhysicsDumper.
     *
     * @return the pre-existing instance (not null)
     */
    public PhysicsDumper getDumper() {
        return dumper;
    }

    /**
     * Access the ShapeGenerator (enhanced psuedo-random generator).
     *
     * @return the pre-existing instance (not null)
     */
    public ShapeGenerator getGenerator() {
        return generator;
    }

    /**
     * Access the current PhysicsSpace.
     *
     * @return the pre-existing instance (not null)
     */
    public PhysicsSpace getPhysicsSpace() {
        BulletAppState bas = getBulletAppState();
        PhysicsSpace result = bas.getPhysicsSpace();

        assert result != null;
        return result;
    }

    /**
     * Test whether animation and physics simulation are paused.
     *
     * @return true if paused, otherwise false
     */
    public boolean isPaused() {
        if (speed <= pausedSpeed) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Callback invoked after adding a collision object to the PhysicsSpace.
     * Meant to be overridden.
     *
     * @param pco the object that was added (not null)
     */
    public void postAdd(PhysicsCollisionObject pco) {
        // do nothing
    }

    /**
     * Add a Material to the library.
     *
     * @param name the desired name for the Material, which is also the key that
     * will be used to find it (not null)
     * @param material (not null, alias created)
     */
    public void registerMaterial(String name, Material material) {
        Validate.nonNull(name, "name");
        Validate.nonNull(material, "material");
        assert !namedMaterials.containsKey(name);

        material.setName(name);
        namedMaterials.put(name, material);
    }

    /**
     * Add a CollisionShape to the library.
     *
     * @param name the key that will be used to find the shape (not null)
     * @param shape (not null, alias created)
     */
    public void registerShape(String name, CollisionShape shape) {
        Validate.nonNull(name, "name");
        Validate.nonNull(shape, "shape");
        assert !namedShapes.containsKey(name);

        namedShapes.put(name, shape);
    }

    /**
     * Scale the specified C-G model uniformly so that it has the specified
     * height, assuming Y-up orientation.
     *
     * @param cgModel (not null, modified)
     * @param height the desired height (in world units, &gt;0)
     */
    public void setCgmHeight(Spatial cgModel, float height) {
        Validate.nonNull(cgModel, "model");
        Validate.positive(height, "height");

        Vector3f[] minMax = MySpatial.findMinMaxCoords(cgModel);
        Vector3f min = minMax[0];
        Vector3f max = minMax[1];
        float oldHeight = max.y - min.y;
        if (oldHeight > 0f) {
            cgModel.scale(height / oldHeight);
        }
    }

    /**
     * Alter the damping fractions of all rigid bodies in the PhysicsSpace.
     *
     * @param damping the desired fraction (&ge;0, &le;1)
     */
    public void setDampingAll(float damping) {
        Validate.fraction(damping, "damping");

        PhysicsSpace physicsSpace = getPhysicsSpace();
        for (PhysicsRigidBody rigidBody : physicsSpace.getRigidBodyList()) {
            rigidBody.setDamping(damping, damping);
        }
    }

    /**
     * Alter the friction of all collision objects in the PhysicsSpace.
     *
     * @param friction the desired friction coefficient (&ge;0)
     */
    public void setFrictionAll(float friction) {
        Validate.nonNegative(friction, "friction");

        PhysicsSpace physicsSpace = getPhysicsSpace();
        for (PhysicsCollisionObject pco : physicsSpace.getPcoList()) {
            pco.setFriction(friction);
        }
    }

    /**
     * Alter the gravity vectors of the PhysicsSpace and all bodies in it.
     *
     * @param gravity the desired magnitude (&ge;0)
     */
    public void setGravityAll(float gravity) {
        Validate.nonNegative(gravity, "gravity");

        PhysicsSpace physicsSpace = getPhysicsSpace();
        Vector3f gravityVector = new Vector3f(0f, -gravity, 0f);
        physicsSpace.setGravity(gravityVector);

        for (PhysicsCollisionObject pco : physicsSpace.getPcoList()) {
            if (pco instanceof PhysicsBody) {
                PhysicsBody body = (PhysicsBody) pco;
                body.setGravity(gravityVector);
            }
        }
    }

    /**
     * Alter the restitution of all collision objects in the PhysicsSpace.
     *
     * @param restitution the desired restitution fraction (&ge;0, &le;1)
     */
    public void setRestitutionAll(float restitution) {
        Validate.fraction(restitution, "restitution");

        PhysicsSpace physicsSpace = getPhysicsSpace();
        for (PhysicsCollisionObject pco : physicsSpace.getPcoList()) {
            pco.setRestitution(restitution);
        }
    }

    /**
     * Toggle physics-debug visualization on/off.
     */
    public void togglePhysicsDebug() {
        BulletAppState bulletAppState = getBulletAppState();
        boolean enabled = bulletAppState.isDebugEnabled();
        bulletAppState.setDebugEnabled(!enabled);
    }

    /**
     * Remove a CollisionShape from the library, if it's been registered.
     *
     * @param key the key used to register the shape (not null)
     */
    public void unregisterShape(String key) {
        Validate.nonNull(key, "key");
        namedShapes.remove(key);
    }
    // *************************************************************************
    // new protected methods

    /**
     * Access the active BulletAppState.
     *
     * @return the pre-existing instance (not null)
     */
    abstract protected BulletAppState getBulletAppState();

    /**
     * Determine the length of debug axis arrows (when they're visible).
     *
     * @return the desired length (in physics-space units, &ge;0)
     */
    abstract protected float maxArrowLength();
    // *************************************************************************
    // ActionApplication methods

    /**
     * Process an action that wasn't handled by the active InputMode.
     *
     * @param actionString textual description of the action (not null)
     * @param ongoing true if the action is ongoing, otherwise false
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void onAction(String actionString, boolean ongoing, float tpf) {
        if (ongoing) {
            switch (actionString) {
                case asCollectGarbage:
                    System.gc();
                    return;

                case asDumpGui:
                    dumper.dump(guiNode);
                    return;
                case asDumpPhysicsSpace:
                    PhysicsSpace physicsSpace = getPhysicsSpace();
                    dumper.dump(physicsSpace);
                    return;
                case asDumpScene:
                    dumper.dump(rootNode);
                    return;
                case asDumpScenes:
                    dumper.dump(renderManager);
                    return;
                case asDumpViewport:
                    dumper.dump(viewPort);
                    return;

                case asToggleAabbs:
                    toggleAabbs();
                    return;
                case asToggleCcdSpheres:
                    toggleCcdSpheres();
                    return;
                case asToggleGravities:
                    toggleGravities();
                    return;
                case asToggleHelp:
                    toggleHelp();
                    return;
                case asTogglePause:
                    togglePause();
                    return;
                case asTogglePcoAxes:
                    togglePcoAxes();
                    return;
                case asTogglePhysicsDebug:
                    togglePhysicsDebug();
                    return;
                case asToggleVelocities:
                    toggleVelocities();
                    return;
                case asToggleWorldAxes:
                    toggleWorldAxes();
                    return;
            }
        }
        super.onAction(actionString, ongoing, tpf);
    }
    // *************************************************************************
    // private methods

    /**
     * Add a large, static box to the PhysicsSpace, to serve as a platform.
     *
     * @param topY the desired Y coordinate of the top surface (in physics-space
     * coordinates)
     * @param topHalfExtent half the desired extent of the top surface (&gt;0)
     * @param thickness the desired thickenss (in physics-space units, &gt;0)
     */
    private void addBoxPlatform(float topY, float topHalfExtent,
            float thickness) {
        Validate.positive(topHalfExtent, "top half extent");
        Validate.positive(thickness, "thickness");

        CollisionShape shape = new BoxCollisionShape(topHalfExtent,
                thickness / 2f, topHalfExtent);
        PhysicsRigidBody body
                = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);

        body.setDebugMeshNormals(DebugMeshNormals.Facet);
        body.setPhysicsLocation(new Vector3f(0f, topY - thickness / 2f, 0f));

        addPlatform(body);
    }

    /**
     * Add a large, downward-pointing, static cone to the PhysicsSpace, to serve
     * as a platform.
     *
     * @param topY the desired Y coordinate of the top surface (in physics-space
     * coordinates)
     * @param topRadius the desired radius of the top surface (&gt;0)
     */
    private void addConePlatform(float topY, float topRadius) {
        Validate.positive(topRadius, "top radius");

        float height = 2f * topRadius; // tall to mitigate smoothed normals
        ConeCollisionShape shape = new ConeCollisionShape(topRadius, height);

        PhysicsRigidBody body
                = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);

        body.setDebugMeshNormals(DebugMeshNormals.Smooth);
        body.setDebugMeshResolution(DebugShapeFactory.highResolution);
        body.setPhysicsLocation(new Vector3f(0f, topY - height / 2f, 0f));
        /*
         * Rotate the cone 180 degrees around the X axis
         * so that it points downward instead of upward.
         */
        Quaternion orientation = new Quaternion();
        orientation.fromAngles(FastMath.PI, 0f, 0f);
        body.setPhysicsRotation(orientation);

        addPlatform(body);
    }

    /**
     * Add a large, static cylinder to the PhysicsSpace, to serve as a platform.
     *
     * @param topY the desired Y coordinate of the top surface (in physics-space
     * coordinates)
     * @param topRadius the desired radius of the top surface (&gt;0)
     */
    private void addCylinderPlatform(float topY, float topRadius) {
        Validate.positive(topRadius, "top radius");

        float height = 3f * topRadius; // tall to mitigate smoothed normals
        CylinderCollisionShape shape = new CylinderCollisionShape(topRadius,
                height, PhysicsSpace.AXIS_Y);

        PhysicsRigidBody body
                = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);

        body.setDebugMeshNormals(DebugMeshNormals.Smooth);
        body.setDebugMeshResolution(DebugShapeFactory.highResolution);
        body.setPhysicsLocation(new Vector3f(0f, topY - height / 2f, 0f));

        addPlatform(body);
    }

    /**
     * Add a large, static petagonal prism to the PhysicsSpace, to serve as a
     * platform.
     *
     * @param topY the desired Y coordinate of the top surface (in physics-space
     * coordinates)
     * @param topRadius the desired radius of the top surface (&gt;0)
     * @param thickness the desired thickenss (in physics-space units, &gt;0)
     */
    private void addHullPlatform(float topY, float topRadius, float thickness) {
        Validate.positive(topRadius, "top radius");
        Validate.positive(thickness, "thickness");

        boolean normals = false;
        Mesh mesh = new Prism(5, topRadius, thickness, normals);
        HullCollisionShape shape = new HullCollisionShape(mesh);

        PhysicsRigidBody body
                = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);

        body.setDebugMeshNormals(DebugMeshNormals.Facet);
        body.setPhysicsLocation(new Vector3f(0f, topY - thickness / 2f, 0f));

        addPlatform(body);
    }

    /**
     * Add a static plane to the PhysicsSpace, to serve as a platform.
     *
     * @param topY the desired Y coordinate of the top surface (in physics-space
     * coordinates)
     */
    private void addPlanePlatform(float topY) {
        Plane plane = new Plane(Vector3f.UNIT_Y, topY);
        PlaneCollisionShape shape = new PlaneCollisionShape(plane);

        PhysicsRigidBody body
                = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);

        PlaneDmiListener planeDmiListener = new PlaneDmiListener(1f);
        body.setDebugMeshInitListener(planeDmiListener);

        body.setDebugMeshNormals(DebugMeshNormals.Facet);

        addPlatform(body);

        Material material = findMaterial("greenTile");
        body.setApplicationData(material);
        body.setDebugMaterial(material);
    }

    /**
     * Add a rounded rectangle to the PhysicsSpace, to serve as a platform.
     *
     * @param topY the desired Y coordinate of the top surface (in physics-space
     * coordinates)
     */
    private void addRoundedRectangle(float topY) {
        CollisionShape shape = findShape("roundedRectangle");
        PhysicsRigidBody body
                = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);

        body.setDebugMeshNormals(DebugMeshNormals.Facet);
        Quaternion rotation = new Quaternion();
        rotation.fromAngles(FastMath.HALF_PI, 0f, 0f);
        body.setPhysicsRotation(rotation);
        body.setPhysicsLocation(new Vector3f(0f, topY, 0f));

        addPlatform(body);
    }

    /**
     * Toggle visualization of collision-object bounding boxes.
     */
    private void toggleAabbs() {
        if (aabbsFilter == null) {
            aabbsFilter = new FilterAll(true);
        } else {
            aabbsFilter = null;
        }

        BulletAppState bulletAppState = getBulletAppState();
        bulletAppState.setDebugBoundingBoxFilter(aabbsFilter);
    }

    /**
     * Toggle visualization of CCD swept spheres.
     */
    private void toggleCcdSpheres() {
        if (ccdSpheresFilter == null) {
            ccdSpheresFilter = new FilterAll(true);
        } else {
            ccdSpheresFilter = null;
        }

        BulletAppState bulletAppState = getBulletAppState();
        bulletAppState.setDebugSweptSphereFilter(ccdSpheresFilter);
    }

    /**
     * Toggle visualization of body gravities.
     */
    private void toggleGravities() {
        if (gravitiesFilter == null) {
            gravitiesFilter = new FilterAll(true);
        } else {
            gravitiesFilter = null;
        }

        BulletAppState bulletAppState = getBulletAppState();
        bulletAppState.setDebugGravityVectorFilter(gravitiesFilter);
    }

    /**
     * Toggle visibility of the help node.
     */
    private void toggleHelp() {
        if (helpNode.getCullHint() == Spatial.CullHint.Always) {
            helpNode.setCullHint(Spatial.CullHint.Never);
        } else {
            helpNode.setCullHint(Spatial.CullHint.Always);
        }
    }

    /**
     * Toggle the animation and physics simulation: paused/running.
     */
    private void togglePause() {
        float newSpeed = isPaused() ? 1f : pausedSpeed;
        setSpeed(newSpeed);
    }

    /**
     * Toggle visualization of collision-object axes.
     */
    private void togglePcoAxes() {
        BulletAppState bulletAppState = getBulletAppState();

        float axisLength = bulletAppState.debugAxisLength();
        if (axisLength != 0f) {
            axisLength = 0f;
        } else {
            axisLength = maxArrowLength();
        }
        bulletAppState.setDebugAxisLength(axisLength);
    }

    /**
     * Toggle visualization of rigid-body velocities.
     */
    private void toggleVelocities() {
        if (velocitiesFilter == null) {
            velocitiesFilter = new FilterAll(true);
        } else {
            velocitiesFilter = null;
        }

        BulletAppState bulletAppState = getBulletAppState();
        bulletAppState.setDebugVelocityVectorFilter(velocitiesFilter);
    }

    /**
     * Toggle visualization of world axes.
     */
    private void toggleWorldAxes() {
        boolean enabled = worldAxes.isEnabled();
        worldAxes.setEnabled(!enabled);
    }
}
