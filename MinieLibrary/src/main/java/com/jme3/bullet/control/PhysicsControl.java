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
package com.jme3.bullet.control;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.scene.control.Control;

/**
 * An interface for a scene-graph control that links physics object(s) to a
 * Spatial.
 *
 * @author normenhansen
 */
public interface PhysicsControl extends Control {
    /**
     * Access the PhysicsSpace to which the physics objects are (or would be)
     * added.
     *
     * @return the pre-existing space, or null for none
     */
    PhysicsSpace getPhysicsSpace();

    /**
     * Test whether this control is enabled.
     *
     * @return true if enabled, otherwise false
     */
    boolean isEnabled();

    /**
     * Enable or disable this control.
     * <p>
     * The physics objects are removed from its PhysicsSpace when the control is
     * disabled. When the control is enabled again, the physics objects are
     * moved to the current location of the Spatial and then added to the
     * PhysicsSpace.
     *
     * @param state true&rarr;enable the control, false&rarr;disable it
     */
    void setEnabled(boolean state);

    /**
     * If this control is enabled, add its physics objects to the specified
     * PhysicsSpace. If not enabled, alter where the objects would be added. The
     * objects are removed from any other space they're currently in.
     *
     * @param space where to add, or null to simply remove
     */
    void setPhysicsSpace(PhysicsSpace space);
}
