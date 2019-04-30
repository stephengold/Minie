/*
 * Copyright (c) 2019 jMonkeyEngine
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
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.debug.WireSphere;
import java.util.logging.Logger;

/**
 * A physics-debug control used to visualize a sphere used for continuous
 * collision detection (CCD).
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class SweptSphereDebugControl extends AbstractPhysicsDebugControl {
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
    SweptSphereDebugControl(BulletDebugAppState debugAppState,
            PhysicsCollisionObject pco) {
        super(debugAppState);
        this.pco = pco;

        float radius = 1f;
        WireSphere mesh = new WireSphere(radius);
        geom = new Geometry("swept sphere of " + pco.toString(), mesh);

        radius = pco.getCcdSweptSphereRadius();
        geom.setLocalScale(radius);

        center = pco.getPhysicsLocation(null);
        geom.setLocalTranslation(center);

        Material mat = debugAppState.DEBUG_WHITE;
        geom.setMaterial(mat);
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
        float radius = pco.getCcdSweptSphereRadius();
        geom.setLocalScale(radius);

        pco.getPhysicsLocation(center);
        geom.setLocalTranslation(center);
    }

    /**
     * Alter which spatial is controlled. Invoked when the control is added to
     * or removed from a spatial. Should be invoked only by a subclass or from
     * Spatial. Do not invoke directly from user code.
     *
     * @param spatial the spatial to control (or null)
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
