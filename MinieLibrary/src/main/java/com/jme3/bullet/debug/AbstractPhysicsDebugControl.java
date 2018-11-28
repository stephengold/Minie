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

import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Spatial;
import com.jme3.scene.control.AbstractControl;
import com.jme3.util.clone.Cloner;
import java.io.IOException;

/**
 * The abstract base class for physics-debug controls (such as
 * BulletRigidBodyDebugControl) used to visualize individual collision objects
 * and joints.
 *
 * @author normenhansen
 */
public abstract class AbstractPhysicsDebugControl extends AbstractControl {

    final private Quaternion tmp_inverseWorldRotation = new Quaternion();
    /**
     * the app state that this control serves
     */
    protected final BulletDebugAppState debugAppState;

    /**
     * Instantiate an enabled control to serve the specified debug app state.
     *
     * @param debugAppState which app state (not null, alias created)
     */
    public AbstractPhysicsDebugControl(BulletDebugAppState debugAppState) {
        this.debugAppState = debugAppState;
    }

    /**
     * Apply the specified location and orientation to the controlled spatial.
     *
     * @param worldLocation location vector (in physics-space coordinates, not
     * null, unaffected)
     * @param worldRotation orientation (in physics-space coordinates, not null,
     * unaffected)
     */
    protected void applyPhysicsTransform(Vector3f worldLocation,
            Quaternion worldRotation) {
        applyPhysicsTransform(worldLocation, worldRotation, spatial);
    }

    /**
     * Apply the specified location and orientation to the specified spatial.
     *
     * @param worldLocation location vector (in physics-space coordinates, not
     * null, unaffected)
     * @param worldRotation orientation (in physics-space coordinates, not null,
     * unaffected)
     * @param spatial where to apply (may be null)
     */
    private void applyPhysicsTransform(Vector3f worldLocation,
            Quaternion worldRotation, Spatial spatial) {
        if (spatial != null) {
            Vector3f localLocation = spatial.getLocalTranslation();
            Quaternion localRotationQuat = spatial.getLocalRotation();
            if (spatial.getParent() != null) {
                localLocation.set(worldLocation).subtractLocal(
                        spatial.getParent().getWorldTranslation());
                localLocation.divideLocal(spatial.getParent().getWorldScale());
                tmp_inverseWorldRotation.set(
                        spatial.getParent().getWorldRotation())
                        .inverseLocal().multLocal(localLocation);
                localRotationQuat.set(worldRotation);
                tmp_inverseWorldRotation.set(
                        spatial.getParent().getWorldRotation())
                        .inverseLocal().mult(localRotationQuat,
                                localRotationQuat);
                spatial.setLocalTranslation(localLocation);
                spatial.setLocalRotation(localRotationQuat);
            } else {
                spatial.setLocalTranslation(worldLocation);
                spatial.setLocalRotation(worldRotation);
            }
        }
    }

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned control into a deep-cloned one, using the specified cloner
     * and original to resolve copied fields.
     *
     * @param cloner the cloner that's cloning this control (unused)
     * @param original the control from which this control was shallow-cloned
     * (unused)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        throw new UnsupportedOperationException();
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return never
     * @throws UnsupportedOperationException always
     */
    @Override
    public Object jmeClone() {
        throw new UnsupportedOperationException();
    }

    /**
     * De-serialize this control from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param im importer (unused)
     * @throws java.io.IOException never
     * @throws UnsupportedOperationException always
     */
    @Override
    public void read(JmeImporter im) throws IOException {
        throw new UnsupportedOperationException();
    }

    /**
     * Render this control. Invoked once per view port per frame, provided the
     * control is enabled and added to a scene. Should be invoked only by a
     * subclass or by AbstractControl.
     *
     * @param rm the render manager (not null)
     * @param vp the view port to render (not null)
     */
    @Override
    protected void controlRender(RenderManager rm, ViewPort vp) {
    }

    /**
     * Serialize this object, for example when saving to a J3O file.
     *
     * @param ex exporter (unused)
     * @throws IOException never
     * @throws UnsupportedOperationException always
     */
    @Override
    public void write(JmeExporter ex) throws IOException {
        throw new UnsupportedOperationException();
    }
}
