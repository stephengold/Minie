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

import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.objects.PhysicsCharacter;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.material.Material;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import java.util.logging.Logger;

/**
 * A physics-debug control used to visualize a PhysicsCharacter.
 *
 * @author normenhansen
 */
public class BulletCharacterDebugControl extends AbstractPhysicsDebugControl {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(BulletCharacterDebugControl.class.getName());
    // *************************************************************************
    // fields

    /**
     * collision-shape margin for which geom was generated
     */
    private float oldMargin;
    /**
     * character to visualize (not null)
     */
    final private PhysicsCharacter character;
    /**
     * temporary storage for physics location
     */
    final private Vector3f location = new Vector3f();
    /**
     * shape for which geom was generated (not null)
     */
    private CollisionShape myShape;
    /**
     * geometry to visualize myShape (not null)
     */
    private Spatial geom;
    /**
     * physics scale for which geom was generated
     */
    final private Vector3f oldScale = new Vector3f();
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled control to visualize the specified character.
     *
     * @param debugAppState which app state (not null, alias created)
     * @param ch the character to visualize (not null, alias created)
     */
    public BulletCharacterDebugControl(BulletDebugAppState debugAppState,
            PhysicsCharacter ch) {
        super(debugAppState);
        character = ch;

        myShape = character.getCollisionShape();
        oldMargin = myShape.getMargin();
        myShape.getScale(oldScale);

        geom = DebugShapeFactory.getDebugShape(character);
        geom.setName(ch.toString());
        updateMaterial();
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
            node.attachChild(geom);
        } else if (spatial == null && this.spatial != null) {
            Node node = (Node) this.spatial;
            node.detachChild(geom);
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
        CollisionShape newShape = character.getCollisionShape();
        float newMargin = newShape.getMargin();
        Vector3f newScale = newShape.getScale(null);
        if (myShape != newShape || oldMargin != newMargin
                || !oldScale.equals(newScale)) {
            myShape = newShape;
            oldMargin = newMargin;
            oldScale.set(newScale);

            Node node = (Node) spatial;
            node.detachChild(geom);

            geom = DebugShapeFactory.getDebugShape(character);
            geom.setName(character.toString());

            node.attachChild(geom);
        }

        updateMaterial();
        character.getPhysicsLocation(location);
        applyPhysicsTransform(location, Quaternion.IDENTITY);
    }
    // *************************************************************************
    // private methods

    /**
     * Update the material applied to the debug geometry based on the properties
     * of the character.
     */
    private void updateMaterial() {
        Material material = character.getDebugMaterial();
        if (material == null) {
            material = debugAppState.DEBUG_PINK;
        }
        geom.setMaterial(material);
    }
}
