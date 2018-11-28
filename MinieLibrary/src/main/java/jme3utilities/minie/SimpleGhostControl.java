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
import com.jme3.bullet.control.GhostControl;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * Simplified physics control for a non-solid object.
 * <p>
 * Implements key methods in order to simplify the development of subclasses.
 * <p>
 * Assumes that the object will live in a single physics space and listen to
 * physics ticks.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class SimpleGhostControl
        extends GhostControl
        implements PhysicsTickListener {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(SimpleGhostControl.class.getName());
    // *************************************************************************
    // constructors

    /**
     * Instantiate a control for an object with a specified shape.
     *
     * @param enabled true for an enabled control, false for a disabled one
     * @param initialShape initial collision shape for the object
     * @param physicsSpace physics space which will contain the object (not
     * null)
     */
    public SimpleGhostControl(boolean enabled, CollisionShape initialShape,
            PhysicsSpace physicsSpace) {
        super(initialShape);

        Validate.nonNull(physicsSpace, "physics space");
        space = physicsSpace;

        setEnabled(enabled);
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
         * remove the control from physics space and then re-add it.
         */
        physicsSpace.remove(this);
        super.setCollisionShape(newShape);
        physicsSpace.add(this);
    }
    // *************************************************************************
    // GhostControl methods

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
    // PhysicsTickListener methods

    /**
     * Callback from Bullet, invoked just after the physics has been stepped.
     * Only performs checks. Meant to be overridden.
     *
     * @param physicsSpace the space that was just stepped (not null)
     * @param elapsedTime the time per physics step (in seconds, &ge;0)
     */
    @Override
    public void physicsTick(PhysicsSpace physicsSpace, float elapsedTime) {
        Validate.nonNull(physicsSpace, "physics space");
        Validate.nonNegative(elapsedTime, "interval");
    }

    /**
     * Callback from Bullet, invoked just before the physics is stepped. Only
     * performs checks. Meant to be overridden.
     *
     * @param physicsSpace the space that is about to be stepped (not null)
     * @param elapsedTime the time per physics step (in seconds, &ge;0)
     */
    @Override
    public void prePhysicsTick(PhysicsSpace physicsSpace, float elapsedTime) {
        Validate.nonNull(physicsSpace, "physics space");
        Validate.nonNegative(elapsedTime, "interval");
    }
    // *************************************************************************
    // new protected methods

    /**
     * Add this control's object to its physics space.
     * <p>
     * Meant to be overridden.
     */
    protected void onAdd() {
        if (spatial != null) {
            setPhysicsLocation(spatial.getWorldTranslation());
            setPhysicsRotation(spatial.getWorldRotation());
        }
        space.addCollisionObject(this);
        space.addTickListener(this);
    }

    /**
     * Remove this control's object from its physics space.
     * <p>
     * Meant to be overridden.
     */
    protected void onRemove() {
        space.removeCollisionObject(this);
        space.removeTickListener(this);
    }
}
