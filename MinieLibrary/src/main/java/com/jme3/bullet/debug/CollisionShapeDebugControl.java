/*
 * Copyright (c) 2020-2024 jMonkeyEngine
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
 * An abstract physics-debug control with additional logic that's shared between
 * BulletCharacterDebugControl, BulletGhostObjectDebugControl,
 * BulletRigidBodyDebugControl, and ColliderDebugControl.
 *
 * @author Stephen Gold sgold@sonic.net
 */
abstract class CollisionShapeDebugControl
        extends AbstractPhysicsDebugControl {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final static Logger loggerS
            = Logger.getLogger(CollisionShapeDebugControl.class.getName());
    // *************************************************************************
    // fields

    /**
     * ChildCollisionShape summary list when lastShape was set (for compound
     * shapes only)
     */
    final private ChildSummaryList lastSummaryList = new ChildSummaryList();
    /**
     * temporary storage for ChildCollisionShape summaries
     */
    final private static ChildSummaryList newSummaryList
            = new ChildSummaryList();
    /**
     * shape for which debugSpatial was generated (not null)
     */
    private CollisionShape lastShape;
    /**
     * collision-shape margin when lastShape was set
     */
    private float lastMargin;
    /**
     * Spatial to visualize lastShape (not null)
     */
    protected Spatial debugSpatial;
    /**
     * temporary storage for the scale vector
     */
    final private static Vector3f newScale = new Vector3f();
    /**
     * collision-shape scale when lastShape was set
     */
    final private Vector3f lastScale = new Vector3f();
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled Control to serve the specified debug app state.
     *
     * @param debugAppState which app state (not null, alias created)
     */
    CollisionShapeDebugControl(BulletDebugAppState debugAppState) {
        super(debugAppState);
    }
    // *************************************************************************
    // new protected methods

    /**
     * Color each child of the debug node for a CompoundCollisionShape. Invoked
     * when child coloring is enabled.
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
     * @param newShape the new shape (not null, unaffected)
     * @return true if the shape has changed, otherwise false
     */
    protected boolean hasShapeChanged(CollisionShape newShape) {
        float newMargin = newShape.getMargin();
        newShape.getScale(newScale);

        boolean result = false;
        if (lastShape != newShape) {
            result = true;
        } else if (lastMargin != newMargin) {
            result = true;
        } else if (!lastScale.equals(newScale)) {
            result = true;
        } else if (newShape instanceof CompoundCollisionShape) {
            newSummaryList.update((CompoundCollisionShape) newShape);
            if (!newSummaryList.equals(lastSummaryList)) {
                result = true;
            }
        }

        return result;
    }

    /**
     * Replace the lastShape and update related fields.
     *
     * @param shape (not null, alias created)
     */
    protected void setShape(CollisionShape shape) {
        this.lastShape = shape;
        this.lastMargin = shape.getMargin();
        shape.getScale(lastScale);
        if (shape instanceof CompoundCollisionShape) {
            lastSummaryList.update((CompoundCollisionShape) shape);
        }
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
