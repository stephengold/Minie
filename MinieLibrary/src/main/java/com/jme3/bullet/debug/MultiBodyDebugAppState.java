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

import com.jme3.bullet.MultiBody;
import com.jme3.bullet.MultiBodySpace;
import com.jme3.bullet.objects.MultiBodyCollider;
import com.jme3.scene.Node;
import com.jme3.scene.control.Control;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * An AppState to manage debug visualization of a MultiBodySpace.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class MultiBodyDebugAppState extends BulletDebugAppState {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(MultiBodyDebugAppState.class.getName());
    // *************************************************************************
    // fields

    /**
     * map multibodies to visualization nodes
     */
    private HashMap<MultiBodyCollider, Node> colliderMap = new HashMap<>(64);
    // *************************************************************************
    // constructors

    /**
     * Instantiate an AppState with the specified configuration. This
     * constructor should be invoked only by MultiBodyDebugAppState.
     *
     * @param config the desired configuration (not null, alias created)
     */
    public MultiBodyDebugAppState(DebugConfiguration config) {
        super(config);
    }
    // *************************************************************************
    // BulletDebugAppState methods

    /**
     * Synchronize the collision-shape debug controls and axis visualizers with
     * the collision objects in the PhysicsSpace.
     */
    @Override
    protected void updateShapes() {
        super.updateShapes();
        updateMultiBodies();
    }
    // *************************************************************************
    // private methods

    /**
     * Synchronize the collider debug controls with the MultiBodySpace.
     */
    private void updateMultiBodies() {
        HashMap<MultiBodyCollider, Node> oldMap = colliderMap;
        // create new map
        this.colliderMap = new HashMap<>(oldMap.size());
        DebugConfiguration config = getConfiguration();
        MultiBodySpace pSpace = (MultiBodySpace) config.getSpace();
        Collection<MultiBody> list = pSpace.getMultiBodyList();
        for (MultiBody multiBody : list) {
            List<MultiBodyCollider> list2 = multiBody.listColliders();
            for (MultiBodyCollider collider : list2) {
                Node node = oldMap.remove(collider);
                if (node == null) {
                    node = new Node(collider.toString());
                    attachChild(node);
                }
                colliderMap.put(collider, node);
            }
        }

        // Detach nodes of colliders that have been removed from the space.
        for (Node node : oldMap.values()) {
            node.removeFromParent();
        }
        /*
         * Synchronize the collider debug controls and axis visualizers
         * with the colliders in the PhysicsSpace.
         */
        BulletDebugAppState.DebugAppStateFilter filter
                = getConfiguration().getFilter();
        for (Map.Entry<MultiBodyCollider, Node> entry
                : colliderMap.entrySet()) {
            MultiBodyCollider collider = entry.getKey();
            boolean displayShape = (filter == null)
                    || filter.displayObject(collider);

            Node node = entry.getValue();
            Control control = node.getControl(ColliderDebugControl.class);
            if (control == null && displayShape) {
                logger.log(Level.FINE, "Create new MultiBodyDebugControl");
                control = new ColliderDebugControl(this, collider);
                node.addControl(control);
            } else if (control != null && !displayShape) {
                node.removeControl(control);
            }

            updateAxes(node, displayShape);
        }
    }
}
