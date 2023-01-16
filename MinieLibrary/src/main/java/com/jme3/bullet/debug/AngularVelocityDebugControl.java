/*
 * Copyright (c) 2020-2023 jMonkeyEngine
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
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.material.Material;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.debug.Arrow;
import java.util.logging.Logger;

/**
 * A physics-debug control used to visualize the angular velocity of a dynamic
 * rigid body.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class AngularVelocityDebugControl extends AbstractPhysicsDebugControl {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final static Logger logger
            = Logger.getLogger(AngularVelocityDebugControl.class.getName());
    // *************************************************************************
    // fields

    /**
     * geometry to visualize the angular velocity (not null)
     */
    final private Geometry geom;
    /**
     * rigid body to visualize (not null)
     */
    final private PhysicsRigidBody rigidBody;
    /**
     * temporary storage for the location of a body's center
     */
    final private Vector3f tmpCenter = new Vector3f();
    /**
     * temporary storage for a body's angular velocity
     */
    final private static Vector3f tmpExtent = new Vector3f();
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled control to visualize the angular velocity of the
     * specified rigid body.
     *
     * @param debugAppState which app state (not null, alias created)
     * @param pco which rigid body to visualize (not null, alias created)
     */
    AngularVelocityDebugControl(
            BulletDebugAppState debugAppState, PhysicsCollisionObject pco) {
        super(debugAppState);
        this.rigidBody = (PhysicsRigidBody) pco;

        rigidBody.getAngularVelocity(tmpExtent);
        Arrow mesh = new Arrow(tmpExtent);
        this.geom = new Geometry("angular velocity of " + rigidBody, mesh);

        rigidBody.getPhysicsLocation(tmpCenter);
        geom.setLocalTranslation(tmpCenter);

        Material material = debugAppState.getAngularVelocityMaterial();
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
        rigidBody.getPhysicsLocation(tmpCenter);
        geom.setLocalTranslation(tmpCenter);

        Mesh mesh = geom.getMesh();
        Arrow arrow = (Arrow) mesh;
        rigidBody.getAngularVelocity(tmpExtent);
        arrow.setArrowExtent(tmpExtent);
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
}
