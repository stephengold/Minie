/*
 Copyright (c) 2014-2018, Stephen Gold
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package jme3utilities.minie;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Vector3f;
import com.jme3.scene.Spatial;
import com.jme3.scene.control.Control;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.MyString;
import jme3utilities.Validate;

/**
 * Simple ghost control which enables another ("successor") control when its
 * collision shape overlaps with an eligible rigid body.
 * <p>
 * Each instance is disabled at creation.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class BubbleControl
        extends SimpleGhostControl
        implements OverlapListener {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(BubbleControl.class.getName());
    // *************************************************************************
    // fields

    /**
     * successor control to be enabled when the collision shape is touched, or
     * null for none
     */
    final private Control successor;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a disabled control.
     *
     * @param initialShape initial collision shape (not null)
     * @param physicsSpace physics space (not null)
     * @param successor control to be enabled when the collision shape is
     * touched, or null for none
     */
    public BubbleControl(CollisionShape initialShape, PhysicsSpace physicsSpace,
            Control successor) {
        super(false, initialShape, physicsSpace);
        assert initialShape != null;

        this.successor = successor;

        assert !isEnabled();
    }
    // *************************************************************************
    // OverlapListener methods

    /**
     * Callback to handle an overlapping rigid body.
     *
     * @param overlappingBody rigid body (not null)
     * @param overlappingSpatial spatial of the rigid body (not null)
     * @param localPoint location of the overlap in the collision shape (rotated
     * and translated to the this control's spatial, but at world scale, not
     * null)
     */
    @Override
    public void onOverlap(PhysicsRigidBody overlappingBody,
            Spatial overlappingSpatial, Vector3f localPoint) {
        Validate.nonNull(overlappingBody, "body");
        Validate.nonNull(overlappingSpatial, "spatial");
        Validate.nonNull(localPoint, "location");

        if (!isEnabled()) {
            return;
        }
        /*
         * Any eligible rigid body which overlaps the bubble's collision shape
         * will pop the bubble.
         */
        if (isEligible(overlappingSpatial)) {
            logger2.log(Level.INFO, "{0} pops {1} at {2}", new Object[]{
                MyString.quote(overlappingSpatial.getName()),
                MyString.quote(spatial.getName()),
                localPoint
            });
            if (successor != null) {
                enableSuccessor(successor);
            }
            setEnabled(false);
        }
    }
    // *************************************************************************
    // SimpleGhostControl methods

    /**
     * Add this control's object to physics space.
     */
    @Override
    public void onAdd() {
        /*
         * Assume the shape of the rigid body.
         */
        RigidBodyControl rigidBodyControl = spatial.getControl(
                RigidBodyControl.class);
        CollisionShape shape = rigidBodyControl.getCollisionShape();
        setCollisionShape(shape);

        super.onAdd();
    }
    // *************************************************************************
    // new protected methods

    /**
     * Enable the successor.
     *
     * @param successor control to enable (not null)
     */
    protected void enableSuccessor(Control successor) {
        Validate.nonNull(successor, "successor control");
        MyControlP.setEnabled(successor, true);
    }

    /**
     * Test whether a rigid body is eligible to pop the bubble.
     * <p>
     * Meant to be overridden.
     *
     * @param object object to test (not null)
     * @return true if eligible, otherwise false
     */
    protected boolean isEligible(Spatial object) {
        Validate.nonNull(object, "object");
        return true;
    }
}
