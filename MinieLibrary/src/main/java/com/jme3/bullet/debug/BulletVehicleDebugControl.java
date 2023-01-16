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

import com.jme3.bullet.objects.PhysicsVehicle;
import com.jme3.bullet.objects.VehicleWheel;
import com.jme3.bullet.objects.infos.RigidBodyMotionState;
import com.jme3.material.Material;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.debug.Arrow;
import java.util.logging.Logger;

/**
 * A physics-debug control used to visualize a PhysicsVehicle.
 *
 * @author normenhansen
 */
public class BulletVehicleDebugControl extends AbstractPhysicsDebugControl {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(BulletVehicleDebugControl.class.getName());
    // *************************************************************************
    // fields

    final private Node suspensionNode;
    /**
     * vehicle to visualize (not null)
     */
    final private PhysicsVehicle vehicle;
    /**
     * temporary storage for physics rotation
     */
    final private Quaternion rotation = new Quaternion();
    /**
     * temporary storage for physics location
     */
    final private Vector3f location = new Vector3f();
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled control to visualize the specified vehicle.
     *
     * @param debugAppState which app state (not null, alias created)
     * @param ve which vehicle to visualize (not null, alias created)
     */
    public BulletVehicleDebugControl(
            BulletDebugAppState debugAppState, PhysicsVehicle ve) {
        super(debugAppState);
        this.vehicle = ve;
        this.suspensionNode = new Node("Suspension");
        createVehicle();
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
        for (int i = 0; i < vehicle.getNumWheels(); ++i) {
            VehicleWheel physicsVehicleWheel = vehicle.getWheel(i);
            Vector3f wLocation = physicsVehicleWheel.getLocation(null);
            Vector3f wDirection = physicsVehicleWheel.getDirection(null);
            Vector3f axle = physicsVehicleWheel.getAxle(null);
            float restLength = physicsVehicleWheel.getRestLength();
            float radius = physicsVehicleWheel.getRadius();

            Geometry locGeom = (Geometry) suspensionNode.getChild(
                    "WheelLocationDebugShape" + i);
            Geometry dirGeom = (Geometry) suspensionNode.getChild(
                    "WheelDirectionDebugShape" + i);
            Geometry axleGeom = (Geometry) suspensionNode.getChild(
                    "WheelAxleDebugShape" + i);
            Geometry wheelGeom = (Geometry) suspensionNode.getChild(
                    "WheelRadiusDebugShape" + i);

            Arrow locArrow = (Arrow) locGeom.getMesh();
            locArrow.setArrowExtent(wLocation);
            Arrow axleArrow = (Arrow) axleGeom.getMesh();
            axleArrow.setArrowExtent(
                    axle.normalizeLocal().multLocal(0.3f));
            Arrow wheelArrow = (Arrow) wheelGeom.getMesh();
            wheelArrow.setArrowExtent(
                    wDirection.normalizeLocal().multLocal(radius));
            Arrow dirArrow = (Arrow) dirGeom.getMesh();
            dirArrow.setArrowExtent(
                    wDirection.normalizeLocal().multLocal(restLength));

            dirGeom.setLocalTranslation(wLocation);
            axleGeom.setLocalTranslation(wLocation.addLocal(wDirection));
            wheelGeom.setLocalTranslation(wLocation);
        }
        RigidBodyMotionState motionState = vehicle.getMotionState();
        motionState.getLocation(location);
        motionState.getOrientation(rotation);
        applyPhysicsTransform(location, rotation);
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
            node.attachChild(suspensionNode);
        } else if (spatial == null && this.spatial != null) {
            Node node = (Node) this.spatial;
            node.detachChild(suspensionNode);
        }
        super.setSpatial(spatial);
    }
    // *************************************************************************
    // private methods

    private void createVehicle() {
        suspensionNode.detachAllChildren();
        for (int i = 0; i < vehicle.getNumWheels(); ++i) {
            VehicleWheel physicsVehicleWheel = vehicle.getWheel(i);
            Vector3f wLocation = physicsVehicleWheel.getLocation(null);
            Vector3f wDirection = physicsVehicleWheel.getDirection(null);
            Vector3f axle = physicsVehicleWheel.getAxle(null);
            float restLength = physicsVehicleWheel.getRestLength();
            float radius = physicsVehicleWheel.getRadius();

            Arrow locArrow = new Arrow(wLocation);
            Arrow axleArrow = new Arrow(axle.normalizeLocal().multLocal(0.3f));
            Arrow wheelArrow = new Arrow(
                    wDirection.normalizeLocal().multLocal(radius));
            Arrow dirArrow = new Arrow(
                    wDirection.normalizeLocal().multLocal(restLength));

            Geometry locGeom = new Geometry(
                    "WheelLocationDebugShape" + i, locArrow);
            Geometry dirGeom = new Geometry(
                    "WheelDirectionDebugShape" + i, dirArrow);
            Geometry axleGeom = new Geometry(
                    "WheelAxleDebugShape" + i, axleArrow);
            Geometry wheelGeom = new Geometry(
                    "WheelRadiusDebugShape" + i, wheelArrow);

            dirGeom.setLocalTranslation(wLocation);
            axleGeom.setLocalTranslation(wLocation.add(wDirection));
            wheelGeom.setLocalTranslation(wLocation.add(wDirection));

            Material material = debugAppState.getActiveMaterial(1);
            locGeom.setMaterial(material);
            dirGeom.setMaterial(material);
            axleGeom.setMaterial(material);
            wheelGeom.setMaterial(material);

            suspensionNode.attachChild(locGeom);
            suspensionNode.attachChild(dirGeom);
            suspensionNode.attachChild(axleGeom);
            suspensionNode.attachChild(wheelGeom);
        }
    }
}
