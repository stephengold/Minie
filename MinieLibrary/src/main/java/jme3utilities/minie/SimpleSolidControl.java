/*
 Copyright (c) 2013-2018, Stephen Gold
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
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.joints.PhysicsJoint;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * Simplified rigid-body control for a solid object.
 * <p>
 * Implements key methods in order to simplify the development of subclasses.
 * <p>
 * Assumes that the object will live in a single physics space and listen to
 * physics ticks.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class SimpleSolidControl
        extends RigidBodyControl
        implements PhysicsTickListener {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger3
            = Logger.getLogger(SimpleSolidControl.class.getName());
    // *************************************************************************
    // constructors

    /**
     * Instantiate a dynamic control for an object with a specified shape and
     * mass.
     *
     * @param enabled true for an enabled object, false for a disabled one
     * @param initialShape collision shape for the object (not null)
     * @param mass object's mass (in kilograms, &gt;0) or zero for a static
     * object
     * @param physicsSpace physics space that will contain the object (not null)
     */
    public SimpleSolidControl(boolean enabled, CollisionShape initialShape,
            float mass, PhysicsSpace physicsSpace) {
        super(initialShape, mass);
        Validate.nonNegative(mass, "mass");
        Validate.nonNull(physicsSpace, "physics space");
        space = physicsSpace;
        setEnabled(enabled);

        assert isKinematicSpatial();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Alter the object's collision shape. Assumes that the object has already
     * been added to a physics space.
     *
     * @param newShape (not null)
     */
    public void changeShape(CollisionShape newShape) {
        Validate.nonNull(newShape, "shape");
        if (space == null) {
            throw new IllegalStateException("should be in a physics space");
        }
        /*
         * The remove() method will null out the "space" field,
         * so save a reference.
         */
        PhysicsSpace physicsSpace = space;
        /*
         * In order to resize a physical object, we must
         * remove it from physics space and then re-add it.
         */
        physicsSpace.remove(this);
        super.setCollisionShape(newShape);
        physicsSpace.add(this);
    }
    // *************************************************************************
    // PhysicsTickListener methods

    /**
     * Callback invoked after each physics tick.
     *
     * Does nothing. Meant to be overridden.
     *
     * @param physicsSpace (not null)
     * @param elapsedTime (in seconds, &ge;0)
     */
    @Override
    public void physicsTick(PhysicsSpace physicsSpace, float elapsedTime) {
        /* do nothing */
    }

    /**
     * Callback invoked before each physics tick.
     *
     * Does nothing. Meant to be overridden.
     *
     * @param physicsSpace (not null)
     * @param elapsedTime (in seconds, &ge;0)
     */
    @Override
    public void prePhysicsTick(PhysicsSpace physicsSpace, float elapsedTime) {
        /* do nothing */
    }
    // *************************************************************************
    // RigidBodyControl methods

    /**
     * Test whether this control is enabled.
     *
     * @return true if enabled, otherwise false
     */
    @Override
    final public boolean isEnabled() {
        return enabled;
    }

    /**
     * Enable or disable the control.
     *
     * @param newState true to enable or false to disable
     */
    @Override
    final public void setEnabled(boolean newState) {
        enabled = newState;
        if (enabled && !added) {
            onAdd();
            added = true;
        } else if (!enabled && added) {
            onRemove();
            added = false;
        }
    }
    // *************************************************************************
    // new protected methods

    /**
     * (Re-)Initialize this control each time it is added.
     *
     * Meant to be overridden.
     */
    protected void onAdd() {
        if (spatial != null) {
            setPhysicsLocation(spatial.getWorldTranslation());
            setPhysicsRotation(spatial.getWorldRotation());
        }

        for (PhysicsJoint j : listJoints()) {
            if (!space.getJointList().contains(j)) {
                space.add(j);
            }
        }
        space.addCollisionObject(this);
        space.addTickListener(this);
    }

    /**
     * De-initialize this control each time it is removed.
     *
     * Meant to be overridden.
     */
    protected void onRemove() {
        for (PhysicsJoint j : listJoints()) {
            if (space.getJointList().contains(j)) {
                space.remove(j);
            }
        }
        space.removeCollisionObject(this);
        space.removeTickListener(this);
    }
}
