/*
 * Copyright (c) 2019-2023 jMonkeyEngine
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

import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.material.Material;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import java.util.logging.Logger;
import jme3utilities.debug.SphereMeshes;
import jme3utilities.math.MyVector3f;

/**
 * A physics-debug control used to visualize a sphere used for continuous
 * collision detection (CCD).
 *
 * @author Stephen Gold sgold@sonic.net
 */
class SweptSphereDebugControl extends AbstractPhysicsDebugControl {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(SweptSphereDebugControl.class.getName());
    // *************************************************************************
    // fields

    /**
     * geometry for the swept sphere
     */
    final private Geometry geom;
    /**
     * collision object to visualize
     */
    final private PhysicsCollisionObject pco;
    /**
     * physics-space coordinates of the sphere's center
     */
    final private Vector3f center;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled control to visualize the swept sphere of the
     * specified collision object.
     *
     * @param debugAppState which app state (not null, alias created)
     * @param pco which collision object to visualize (not null, alias created)
     */
    SweptSphereDebugControl(
            BulletDebugAppState debugAppState, PhysicsCollisionObject pco) {
        super(debugAppState);
        this.pco = pco;

        Mesh mesh = updateMesh(null);
        this.geom = new Geometry("swept sphere of " + pco, mesh);

        float radius = pco.getCcdSweptSphereRadius();
        geom.setLocalScale(radius);

        this.center = pco.getPhysicsLocation(null);
        geom.setLocalTranslation(center);

        Material material = debugAppState.getSweptSphereMaterial();
        geom.setMaterial(material);
        geom.setShadowMode(RenderQueue.ShadowMode.Off);
    }
    // *************************************************************************
    // AbstractPhysicsDebugControl methods

    /**
     * Update this control. Invoked once per frame during the logical-state
     * update, provided the control is enabled and added to a scene. Should be
     * invoked only by a subclass or by AbstractControl.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    protected void controlUpdate(float tpf) {
        Mesh oldMesh = geom.getMesh();
        Mesh newMesh = updateMesh(oldMesh);
        if (oldMesh != newMesh) {
            geom.setMesh(newMesh);
        }

        float radius = pco.getCcdSweptSphereRadius();
        geom.setLocalScale(radius);

        pco.getPhysicsLocation(center);
        geom.setLocalTranslation(center);

        DebugConfiguration config = debugAppState.getConfiguration();
        Camera camera = config.getCamera();
        if (camera != null) {
            Vector3f offset = camera.getLocation().subtract(center);
            Vector3f axis1 = new Vector3f(); // TODO garbage
            Vector3f axis2 = new Vector3f();
            MyVector3f.generateBasis(offset, axis1, axis2);

            Quaternion orientation = new Quaternion(); // TODO garbage
            orientation.fromAxes(axis2, offset, axis1);
            geom.setLocalRotation(orientation);
        }
    }

    /**
     * Alter which Spatial is controlled. Invoked when the Control is added to
     * or removed from a Spatial. Should be invoked only by a subclass or from
     * Spatial. Do not invoke directly from user code.
     *
     * @param spatial the Spatial to control (or null)
     */
    @Override
    public void setSpatial(Spatial spatial) {
        if (spatial instanceof Node) {
            Node node = (Node) spatial;
            node.attachChild(geom);
        } else if (spatial == null && this.spatial != null) {
            Node node = (Node) this.spatial;
            node.detachChild(geom);
        }
        super.setSpatial(spatial);
    }
    // *************************************************************************
    // private methods

    /**
     * Update the Mesh of the debug geometry, based on whether the Camera is
     * known.
     *
     * @param oldMesh the mesh in use
     * @return the updated mesh to use
     */
    private Mesh updateMesh(Mesh oldMesh) {
        DebugConfiguration config = debugAppState.getConfiguration();
        Camera camera = config.getCamera();
        SphereMeshes meshType = (camera == null) ? SphereMeshes.Icosphere
                : SphereMeshes.LoopMesh;

        Mesh result = oldMesh;
        if (!meshType.isInstance(result)) {
            float radius = 1f;
            boolean wantNormals = false;
            boolean wantUvs = false;
            result = meshType.makeSphere(radius, wantNormals, wantUvs);
        }

        return result;
    }
}
