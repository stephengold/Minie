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
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.MeshNormals;

/**
 * A physics-debug control used to visualize a PhysicsCharacter.
 *
 * @author normenhansen
 */
public class BulletCharacterDebugControl extends CollisionShapeDebugControl {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(BulletCharacterDebugControl.class.getName());
    /**
     * local copy of {@link com.jme3.math.Quaternion#IDENTITY}
     */
    final private static Quaternion rotateIdentity = new Quaternion();
    // *************************************************************************
    // fields

    /**
     * debug-mesh resolution for which debugSpatial was generated
     */
    private int oldResolution;
    /**
     * debug-mesh normals option for which debugSpatial was generated
     */
    private MeshNormals oldNormals;
    /**
     * character to visualize (not null)
     */
    final private PhysicsCharacter character;
    /**
     * temporary storage for the location of a character's center
     */
    final private static Vector3f tmpCenter = new Vector3f();
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled control to visualize the specified character.
     *
     * @param debugAppState which app state (not null, alias created)
     * @param ch the character to visualize (not null, alias created)
     */
    public BulletCharacterDebugControl(
            BulletDebugAppState debugAppState, PhysicsCharacter ch) {
        super(debugAppState);
        this.character = ch;

        super.setShape(character.getCollisionShape());
        this.oldNormals = character.debugMeshNormals();
        this.oldResolution = character.debugMeshResolution();

        this.debugSpatial = DebugShapeFactory.getDebugShape(character);
        debugSpatial.setName(ch.toString());
        updateMaterial();
    }
    // *************************************************************************
    // CollisionShapeDebugControl methods

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
        MeshNormals newNormals = character.debugMeshNormals();
        int newResolution = character.debugMeshResolution();

        boolean rebuild;
        if (hasShapeChanged(newShape)) {
            rebuild = true;
        } else if (oldNormals != newNormals) {
            rebuild = true;
        } else if (oldResolution != newResolution) {
            rebuild = true;
        } else {
            rebuild = false;
        }

        if (rebuild) {
            logger.log(Level.INFO, "Rebuild debugSpatial for {0}.", character);
            setShape(newShape);
            oldNormals = newNormals;
            oldResolution = newResolution;

            Node node = (Node) spatial;
            node.detachChild(debugSpatial);

            this.debugSpatial = DebugShapeFactory.getDebugShape(character);
            debugSpatial.setName(character.toString());

            node.attachChild(debugSpatial);
        }

        updateMaterial();

        character.getPhysicsLocation(tmpCenter);
        applyPhysicsTransform(tmpCenter, rotateIdentity);
    }
    // *************************************************************************
    // private methods

    /**
     * Update the Material applied to the debug geometries, based on properties
     * of the character.
     */
    private void updateMaterial() {
        Material material = character.getDebugMaterial();

        if (material == BulletDebugAppState.enableChildColoring) {
            if (debugSpatial instanceof Node) {
                colorChildren();
                return;
            }
            material = null;
        }

        if (material == null) { // apply one of the default materials
            int numSides = character.debugNumSides();
            if (character.isContactResponse()) {
                material = debugAppState.getCharacterMaterial(numSides);
            } else {
                material = debugAppState.getGhostMaterial(numSides);
            }
        }
        debugSpatial.setMaterial(material);
    }
}
