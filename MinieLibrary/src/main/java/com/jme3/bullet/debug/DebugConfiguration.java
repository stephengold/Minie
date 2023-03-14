/*
 * Copyright (c) 2020-2022 jMonkeyEngine
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
import com.jme3.bullet.PhysicsSpace;
import com.jme3.renderer.Camera;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Spatial;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * Configuration data for physics (debug) visualization. For internal use only.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class DebugConfiguration {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(DebugConfiguration.class.getName());
    // *************************************************************************
    // fields

    /**
     * true if-and-only-if (debug) visualization is enabled
     */
    private boolean isEnabled = false;
    /**
     * limit which angular velocities are visualized, or null to visualize no
     * angular velocities
     */
    private BulletDebugAppState.DebugAppStateFilter angularVelocityFilter;
    /**
     * limit which bounding boxes are visualized, or null to visualize no
     * bounding boxes
     */
    private BulletDebugAppState.DebugAppStateFilter boundingBoxFilter;
    /**
     * limit which objects are visualized, or null to visualize all objects
     */
    private BulletDebugAppState.DebugAppStateFilter filter;
    /**
     * limit which gravity vectors are visualized, or null to visualize no
     * gravity vectors
     */
    private BulletDebugAppState.DebugAppStateFilter gravityVectorFilter;
    /**
     * limit which swept spheres are visualized, or null to visualize no swept
     * spheres
     */
    private BulletDebugAppState.DebugAppStateFilter sweptSphereFilter;
    /**
     * limit which velocity vectors are visualized, or null to visualize no
     * velocity vectors
     */
    private BulletDebugAppState.DebugAppStateFilter velocityVectorFilter;
    /**
     * Camera for visualization, or null if unknown
     */
    private Camera camera;
    /**
     * callback to be invoked just before the (debug) scene is added to
     * viewports, or null if none
     */
    private DebugInitListener initListener;
    /**
     * length of each axis arrow (in physics-space units, &gt;0) or 0 for no
     * axis arrows
     */
    private float axisArrowLength = 0f;
    /**
     * line width for wireframe axis arrows (in pixels, &ge;1) or 0 for solid
     * axis arrows
     */
    private float axisLineWidth = 1f;
    /**
     * line width for PhysicsJoint arrows (in pixels, &ge;1)
     */
    private float jointLineWidth = 1f;
    /**
     * PhysicsSpace, or null if none
     */
    private PhysicsSpace space;
    /**
     * ShadowMode for the (debug) root node
     */
    private RenderQueue.ShadowMode shadowMode = RenderQueue.ShadowMode.Off;
    /**
     * used to convert physics-space coordinates into world coordinates
     */
    private Spatial transformSpatial;
    /**
     * view ports in which to render the visualization
     */
    private ViewPort[] viewPorts;
    // *************************************************************************
    // constructors - TODO de-publicize

    /**
     * Instantiate a configuration with no space and no viewports.
     */
    public DebugConfiguration() { // to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Determine the length of axis arrows.
     *
     * @return length (in physics-space units, &ge;0)
     */
    public float axisArrowLength() {
        assert axisArrowLength >= 0f : axisArrowLength;
        return axisArrowLength;
    }

    /**
     * Determine the line width of axis arrows.
     *
     * @return width (in pixels, &ge;1) or 0 for solid arrows
     */
    public float axisLineWidth() {
        assert axisLineWidth >= 0f : axisLineWidth;
        return axisLineWidth;
    }

    /**
     * Access the filter that limits which angular velocities are visualized.
     *
     * @return the pre-existing instance, or null
     */
    BulletDebugAppState.DebugAppStateFilter getAngularVelocityFilter() {
        return angularVelocityFilter;
    }

    /**
     * Access the filter that limits which bounding boxes are visualized.
     *
     * @return the pre-existing instance, or null
     */
    BulletDebugAppState.DebugAppStateFilter getBoundingBoxFilter() {
        return boundingBoxFilter;
    }

    /**
     * Access the Camera used for visualization.
     *
     * @return the pre-existing instance, or null if unknown
     */
    public Camera getCamera() {
        return camera;
    }

    /**
     * Access the filter that limits which physics objects are visualized.
     *
     * @return the pre-existing instance, or null
     */
    BulletDebugAppState.DebugAppStateFilter getFilter() {
        return filter;
    }

    /**
     * Access the filter that limits which gravity vectors are visualized.
     *
     * @return the pre-existing instance, or null
     */
    BulletDebugAppState.DebugAppStateFilter getGravityVectorFilter() {
        return gravityVectorFilter;
    }

    /**
     * Access the callback invoked just before the (debug) scene is added to
     * viewports.
     *
     * @return the pre-existing instance, or null
     */
    DebugInitListener getInitListener() {
        return initListener;
    }

    /**
     * Access the PhysicsSpace.
     *
     * @return the pre-existing instance, or null if none
     */
    public PhysicsSpace getSpace() {
        return space;
    }

    /**
     * Access the filter that limits which swept spheres are visualized.
     *
     * @return the pre-existing instance, or null
     */
    BulletDebugAppState.DebugAppStateFilter getSweptSphereFilter() {
        return sweptSphereFilter;
    }

    /**
     * Access the Spatial used to convert physics-space coordinates into world
     * coordinates.
     *
     * @return spatial the pre-existing Spatial, or null if none
     */
    public Spatial getTransformSpatial() {
        return transformSpatial;
    }

    /**
     * Access the filter that limits which velocity vectors are visualized.
     *
     * @return the pre-existing instance, or null
     */
    BulletDebugAppState.DebugAppStateFilter getVelocityVectorFilter() {
        return velocityVectorFilter;
    }

    /**
     * Set the camera and viewport based on the specified Application, unless
     * they've already been set.
     *
     * @param application the Application from which to obtain the default
     * camera and viewport (not null, unaffected)
     */
    public void initialize(Application application) {
        Validate.nonNull(application, "application");

        if (camera == null) {
            this.camera = application.getCamera();
        }
        if (viewPorts == null) {
            this.viewPorts = new ViewPort[1];
            viewPorts[0] = application.getViewPort();
        }
    }

    /**
     * Test whether (debug) visualization is enabled.
     *
     * @return true if enabled, otherwise false
     */
    public boolean isEnabled() {
        return isEnabled;
    }

    /**
     * Determine the line width of joint arrows.
     *
     * @return width (in pixels, &ge;1)
     */
    float jointLineWidth() {
        assert jointLineWidth >= 1f : jointLineWidth;
        return jointLineWidth;
    }

    /**
     * Enumerate view ports that render the (debug) visualization.
     *
     * @return a new array of pre-existing instances (not null)
     */
    ViewPort[] listViewPorts() {
        final int numViewPorts = viewPorts.length;
        ViewPort[] result = new ViewPort[numViewPorts];
        System.arraycopy(viewPorts, 0, result, 0, numViewPorts);

        return result;
    }

    /**
     * Render all view ports.
     *
     * @param renderManager the RenderManager to use (not null)
     * @param rootNode the root node to use (not null)
     */
    void renderAllViewPorts(RenderManager renderManager, Spatial rootNode) {
        for (ViewPort viewPort : viewPorts) {
            if (viewPort.isEnabled()) {
                renderManager.renderScene(rootNode, viewPort);
            }
        }
    }

    /**
     * Alter which angular velocities are included in the visualization.
     *
     * @param filter the desired filter (alias created) or null to visualize no
     * angular velocities (default=null)
     */
    public void setAngularVelocityFilter(
            BulletDebugAppState.DebugAppStateFilter filter) {
        this.angularVelocityFilter = filter;
    }

    /**
     * Alter the length of axis arrows.
     *
     * @param length the desired length (in world units, &ge;0)
     */
    public void setAxisArrowLength(float length) {
        Validate.nonNegative(length, "length");
        this.axisArrowLength = length;
    }

    /**
     * Alter the line width for axis arrows.
     *
     * @param width the desired width (in pixels, &ge;1) or 0 for solid arrows
     * (default=1)
     */
    public void setAxisLineWidth(float width) {
        Validate.inRange(width, "width", 0f, Float.MAX_VALUE);
        this.axisLineWidth = width;
    }

    /**
     * Alter which bounding boxes are included in the visualization.
     *
     * @param filter the filter to use (alias created) or null to visualize no
     * bounding boxes (default=null)
     */
    public void setBoundingBoxFilter(
            BulletDebugAppState.DebugAppStateFilter filter) {
        this.boundingBoxFilter = filter;
    }

    /**
     * Alter which Camera is used for (debug) visualization.
     *
     * @param camera the Camera to use (alias created) or null for unknown
     * (defaults to the application's main camera)
     */
    public void setCamera(Camera camera) {
        this.camera = camera;
    }

    /**
     * Alter whether (debug) visualization is enabled. Changes take effect on
     * the next update.
     *
     * @param enable true &rarr; enable, false &rarr; disable (default=false)
     */
    public void setEnabled(boolean enable) {
        this.isEnabled = enable;
    }

    /**
     * Alter which physics objects are included in the visualization.
     *
     * @param filter the desired filter (alias created) or null to visualize all
     * objects (default=null)
     */
    public void setFilter(BulletDebugAppState.DebugAppStateFilter filter) {
        this.filter = filter;
    }

    /**
     * Alter which gravity vectors are included in the visualization.
     *
     * @param filter the desired filter (alias created) or null to visualize no
     * gravity vectors (default=null)
     */
    public void setGravityVectorFilter(
            BulletDebugAppState.DebugAppStateFilter filter) {
        this.gravityVectorFilter = filter;
    }

    /**
     * Replace or remove the init listener for the BulletDebugAppState.
     *
     * @param listener the listener to register (alias created) or null to
     * de-register the current listener (default=null)
     */
    public void setInitListener(DebugInitListener listener) {
        this.initListener = listener;
    }

    /**
     * Alter the line width for PhysicsJoint arrows.
     *
     * @param width the desired width (in pixels, &ge;1, default=1)
     */
    public void setJointLineWidth(float width) {
        this.jointLineWidth = width;
    }

    /**
     * Alter the shadow mode for the (debug) root node.
     *
     * @param mode the desired mode (not null, default=Off)
     */
    public void setShadowMode(RenderQueue.ShadowMode mode) {
        Validate.nonNull(mode, "mode");
        this.shadowMode = mode;
    }

    /**
     * Alter which PhysicsSpace will be visualized.
     *
     * @param physicsSpace the PhysicsSpace to visualize (alias created) or null
     * for none
     */
    public void setSpace(PhysicsSpace physicsSpace) {
        this.space = physicsSpace;
    }

    /**
     * Alter which swept spheres are included in the visualization.
     *
     * @param filter the filter to use (alias created) or null to visualize no
     * swept spheres (default=null)
     */
    public void setSweptSphereFilter(
            BulletDebugAppState.DebugAppStateFilter filter) {
        this.sweptSphereFilter = filter;
    }

    /**
     * Alter the conversion from physics-space coordinates to world coordinates.
     *
     * @param spatial the Spatial to use for coordinate transformations (alias
     * created) or null for physics=world (default=null)
     */
    public void setTransformSpatial(Spatial spatial) {
        this.transformSpatial = spatial;
    }

    /**
     * Alter which velocity vectors are included in the visualization.
     *
     * @param filter the desired filter (alias created) or null to visualize no
     * velocity vectors (default=null)
     */
    public void setVelocityVectorFilter(
            BulletDebugAppState.DebugAppStateFilter filter) {
        this.velocityVectorFilter = filter;
    }

    /**
     * Alter which view ports will render the visualization.
     *
     * @param viewPorts (not null, aliases created)
     */
    public void setViewPorts(ViewPort... viewPorts) {
        Validate.nonNull(viewPorts, "view ports");

        final int numViewPorts = viewPorts.length;
        this.viewPorts = new ViewPort[numViewPorts];
        System.arraycopy(viewPorts, 0, this.viewPorts, 0, numViewPorts);
    }

    /**
     * Determine the ShadowMode for the (debug) root node.
     *
     * @return the enum value (not null)
     */
    RenderQueue.ShadowMode shadowMode() {
        assert shadowMode != null;
        return shadowMode;
    }
}
