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

import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.debug.Arrow;
import java.util.logging.Logger;

/**
 * A physics-debug control used to visualize a PhysicsJoint.
 *
 * @author normenhansen
 */
public class BulletJointDebugControl extends AbstractPhysicsDebugControl {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(BulletJointDebugControl.class.getName());
    /**
     * local copy of {@link com.jme3.math.Vector3f#ZERO}
     */
    final private static Vector3f translateIdentity = new Vector3f(0f, 0f, 0f);
    // *************************************************************************
    // fields

    final private PhysicsJoint joint;
    final private Geometry geomA;
    final private Arrow arrowA;
    final private Geometry geomB;
    final private Arrow arrowB;
    final private Transform a = new Transform();
    final private Transform b = new Transform();
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled control to visualize the specified joint.
     *
     * @param debugAppState which app state (not null, alias created)
     * @param jo the joint to visualize (not null, alias created)
     */
    public BulletJointDebugControl(BulletDebugAppState debugAppState,
            PhysicsJoint jo) {
        super(debugAppState);
        joint = jo;

        geomA = new Geometry(jo.toString());
        arrowA = new Arrow(Vector3f.ZERO);
        geomA.setMesh(arrowA);
        geomA.setMaterial(debugAppState.DEBUG_GREEN);

        geomB = new Geometry(jo.toString());
        arrowB = new Arrow(Vector3f.ZERO);
        geomB.setMesh(arrowB);
        geomB.setMaterial(debugAppState.DEBUG_RED);
    }
    // *************************************************************************
    // AbstractPhysicsDebugControl methods

    /**
     * Alter which spatial is controlled. Invoked when the control is added to
     * or removed from a spatial. Should be invoked only by a subclass or from
     * Spatial. Do not invoke directly from user code.
     *
     * @param spatial the spatial to control (or null)
     */
    @Override
    public void setSpatial(Spatial spatial) {
        if (spatial != null && spatial instanceof Node) {
            Node node = (Node) spatial;
            node.attachChild(geomA);
            node.attachChild(geomB);
        } else if (spatial == null && this.spatial != null) {
            Node node = (Node) this.spatial;
            node.detachChild(geomA);
            node.detachChild(geomB);
        }
        super.setSpatial(spatial);
    }

    /**
     * Update this control. Invoked once per frame during the logical-state
     * update, provided the control is enabled and added to a scene. Should be
     * invoked only by a subclass or by AbstractControl.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    protected void controlUpdate(float tpf) {
        if (joint.isEnabled()) {
            PhysicsRigidBody bodyA = joint.getBodyA();
            if (bodyA == null) {
                geomA.setCullHint(Spatial.CullHint.Always);
            } else {
                bodyA.getPhysicsLocation(a.getTranslation());
                bodyA.getPhysicsRotation(a.getRotation());
                geomA.setLocalTransform(a);
                geomA.setCullHint(Spatial.CullHint.Never);
                arrowA.setArrowExtent(joint.getPivotA(null));
            }

            PhysicsRigidBody bodyB = joint.getBodyB();
            if (bodyB == null) {
                geomB.setCullHint(Spatial.CullHint.Always);
            } else {
                bodyB.getPhysicsLocation(b.getTranslation());
                bodyB.getPhysicsRotation(b.getRotation());
                geomB.setLocalTransform(b);
                geomB.setCullHint(Spatial.CullHint.Never);
                arrowB.setArrowExtent(joint.getPivotB(null));
            }

        } else {
            geomA.setCullHint(Spatial.CullHint.Always);
            geomB.setCullHint(Spatial.CullHint.Always);
        }
    }
}
