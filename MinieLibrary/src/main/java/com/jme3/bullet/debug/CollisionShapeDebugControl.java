/*
 * Copyright (c) 2020 jMonkeyEngine
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

import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.material.Material;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import java.util.List;
import java.util.logging.Logger;

/**
 * An abstract physics-debug control with logic shared between
 * BulletCharacterDebugControl, BulletGhostObjectDebugControl,
 * BulletRigidBodyDebugControl, and ColliderDebugControl.
 *
 * @author Stephen Gold sgold@sonic.net
 */
abstract public class CollisionShapeDebugControl
        extends AbstractPhysicsDebugControl {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger loggerS
            = Logger.getLogger(CollisionShapeDebugControl.class.getName());
    // *************************************************************************
    // fields

    /**
     * last shape that was set (not null)
     */
    private CollisionShape lastShape;
    /**
     * collision-shape margin when last shape was set
     */
    private float oldMargin;
    /**
     * Spatial to visualize lastShape (not null)
     */
    protected Spatial debugSpatial;
    /**
     * temporary storage for the scale vector
     */
    final private static Vector3f newScale = new Vector3f();
    /**
     * physics scale when last shape was set
     */
    final private Vector3f oldScale = new Vector3f();
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled Control to serve the specified debug app state.
     *
     * @param debugAppState which app state (not null, alias created)
     */
    public CollisionShapeDebugControl(BulletDebugAppState debugAppState) {
        super(debugAppState);
    }
    // *************************************************************************
    // new protected methods

    /**
     * Color each child of the debug node for a CompoundCollisionShape, when
     * child coloring is enabled.
     */
    protected void colorChildren() {
        Node debugNode = (Node) debugSpatial;
        List<Spatial> children = debugNode.getChildren();
        int numChildren = children.size();
        for (int childIndex = 0; childIndex < numChildren; ++childIndex) {
            Spatial child = children.get(childIndex);
            Material material = debugAppState.getChildMaterial(childIndex);
            child.setMaterial(material);
        }
    }

    /**
     * Compare the specified CollisionShape with the lastShape.
     *
     * @param shape the current shape (not null, unaffected)
     * @return true if the shape has changed, otherwise false
     */
    protected boolean hasShapeChanged(CollisionShape shape) {
        float newMargin = shape.getMargin();
        shape.getScale(newScale);

        boolean result;
        if (shape instanceof CompoundCollisionShape) {
            result = true;
        } else if (lastShape != shape) {
            result = true;
        } else if (oldMargin != newMargin) {
            result = true;
        } else if (!oldScale.equals(newScale)) {
            result = true;
        } else {
            result = false;
        }

        return result;
    }

    /**
     * Set the lastShape.
     *
     * @param shape (not null, alias created)
     */
    protected void setShape(CollisionShape shape) {
        lastShape = shape;
        oldMargin = lastShape.getMargin();
        lastShape.getScale(oldScale);
    }
    // *************************************************************************
    // AbstractPhysicsDebugControl methods

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
            node.attachChild(debugSpatial);
        } else if (spatial == null && this.spatial != null) {
            Node node = (Node) this.spatial;
            node.detachChild(debugSpatial);
        }
        super.setSpatial(spatial);
    }
}
