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

import com.jme3.bullet.joints.Constraint;
import com.jme3.bullet.joints.JointEnd;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.material.Material;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.debug.Arrow;
import java.util.logging.Logger;

/**
 * A physics-debug control to visualize a Constraint.
 *
 * @author Stephen Gold sgold@sonic.net
 *
 * Based on BulletJointDebugControl by normenhansen.
 */
class ConstraintDebugControl extends AbstractPhysicsDebugControl {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(ConstraintDebugControl.class.getName());
    /**
     * local copy of {@link com.jme3.math.Vector3f#ZERO}
     */
    final private static Vector3f translateIdentity = new Vector3f(0f, 0f, 0f);
    // *************************************************************************
    // fields

    /**
     * Constraint to visualize (not null)
     */
    final private Constraint constraint;
    /**
     * geometry to visualize the A end
     */
    final private Geometry geomA;
    /**
     * geometry to visualize the B end
     */
    final private Geometry geomB;
    /**
     * temporary storage for a body's physics transform
     */
    final private static Transform tmpTransform = new Transform();
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled Control to visualize the specified Constraint.
     *
     * @param debugAppState which app state (not null, alias created)
     * @param constraint the Constraint to visualize (not null, alias created)
     */
    ConstraintDebugControl(BulletDebugAppState debugAppState,
            Constraint constraint) {
        super(debugAppState);
        this.constraint = constraint;

        geomA = new Geometry(constraint.toString());
        Arrow arrowA = new Arrow(translateIdentity);
        geomA.setMesh(arrowA);
        Material materialA = debugAppState.getJointMaterial(JointEnd.A);
        geomA.setMaterial(materialA);
        geomA.setShadowMode(RenderQueue.ShadowMode.Off);

        geomB = new Geometry(constraint.toString());
        Arrow arrowB = new Arrow(translateIdentity);
        geomB.setMesh(arrowB);
        Material materialB = debugAppState.getJointMaterial(JointEnd.B);
        geomB.setMaterial(materialB);
        geomB.setShadowMode(RenderQueue.ShadowMode.Off);
    }
    // *************************************************************************
    // AbstractPhysicsDebugControl methods

    /**
     * Update this Control. Invoked once per frame during the logical-state
     * update, provided the Control is enabled and added to a scene. Should be
     * invoked only by a subclass or by AbstractControl.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    protected void controlUpdate(float tpf) {
        if (constraint.isEnabled()) {
            Vector3f vector = tmpTransform.getTranslation(); // alias
            Quaternion rotation = tmpTransform.getRotation(); // alias

            PhysicsRigidBody bodyA = constraint.getBodyA();
            if (bodyA == null) {
                geomA.setCullHint(Spatial.CullHint.Always);
            } else {
                bodyA.getPhysicsLocation(vector);
                bodyA.getPhysicsRotation(rotation);
                geomA.setLocalTransform(tmpTransform);

                geomA.setCullHint(Spatial.CullHint.Never); // TODO dynamic

                Arrow arrow = (Arrow) geomA.getMesh();
                constraint.getPivotA(vector);
                arrow.setArrowExtent(vector);
            }

            PhysicsRigidBody bodyB = constraint.getBodyB();
            if (bodyB == null) {
                geomB.setCullHint(Spatial.CullHint.Always);
            } else {
                bodyB.getPhysicsLocation(vector);
                bodyB.getPhysicsRotation(rotation);
                geomB.setLocalTransform(tmpTransform);

                geomB.setCullHint(Spatial.CullHint.Never);

                Arrow arrow = (Arrow) geomB.getMesh();
                constraint.getPivotB(vector);
                arrow.setArrowExtent(vector);
            }

        } else {
            geomA.setCullHint(Spatial.CullHint.Always);
            geomB.setCullHint(Spatial.CullHint.Always);
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
            assert this.spatial == null;
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
}
