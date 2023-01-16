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
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Spatial;
import com.jme3.scene.control.Control;
import com.jme3.util.clone.Cloner;
import com.jme3.util.clone.JmeCloneable;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.MySpatial;

/**
 * Manage the lifecycle of a physics object linked to a Spatial in a scene
 * graph.
 *
 * @author normenhansen
 */
abstract public class AbstractPhysicsControl
        implements JmeCloneable, PhysicsControl {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(AbstractPhysicsControl.class.getName());
    /**
     * local copy of {@link com.jme3.math.Quaternion#IDENTITY}
     */
    final private static Quaternion rotateIdentity = new Quaternion();
    /**
     * field names for serialization
     */
    final private static String tagApplyLocalPhysics = "applyLocalPhysics";
    final private static String tagEnabled = "enabled";
    final private static String tagSpatial = "spatial";
    /**
     * local copy of {@link com.jme3.math.Vector3f#ZERO}
     */
    final private static Vector3f translateIdentity = new Vector3f(0f, 0f, 0f);
    // *************************************************************************
    // fields

    /**
     * true&rarr;body is added to the PhysicsSpace, false&rarr;not added
     */
    protected boolean added = false;
    /**
     * true&rarr;Control is enabled, false&rarr;Control is disabled
     */
    private boolean enabled = true;
    /**
     * true &rarr; physics-space coordinates match local transform, false &rarr;
     * physics-space coordinates match world transform
     */
    private boolean localPhysics = false;
    /**
     * space to which the physics object is (or would be) added TODO
     * CollisionSpace
     */
    private PhysicsSpace space = null;
    /**
     * temporary storage during calculations TODO thread safety
     */
    private Quaternion tmpInverseWorldRotation = new Quaternion();
    /**
     * Spatial to which this Control is added, or null if none
     */
    private Spatial controlledSpatial;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled control that isn't added to any space or spatial.
     */
    protected AbstractPhysicsControl() { // avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Access the Spatial to which this Control is added.
     *
     * @return the Spatial, or null if none
     */
    public Spatial getSpatial() {
        return controlledSpatial;
    }

    /**
     * Test whether physics-space coordinates should match the spatial's local
     * coordinates.
     *
     * @return true if matching local coordinates, false if matching world
     * coordinates
     */
    public boolean isApplyPhysicsLocal() {
        return localPhysics;
    }

    /**
     * Alter whether physics-space coordinates should match the Spatial's local
     * coordinates.
     *
     * @param applyPhysicsLocal true&rarr;match local coordinates,
     * false&rarr;match world coordinates (default=false)
     */
    public void setApplyPhysicsLocal(boolean applyPhysicsLocal) {
        this.localPhysics = applyPhysicsLocal;
    }
    // *************************************************************************
    // new protected methods

    /**
     * Add all managed physics objects to the PhysicsSpace.
     */
    abstract protected void addPhysics();

    /**
     * Apply a physics transform to the controlled Spatial. TODO use MySpatial
     *
     * @param physicsLocation the desired location (in physics-space
     * coordinates, not null, unaffected)
     * @param physicsOrientation the desired orientation (in physics-space
     * coordinates, not null, unaffected)
     */
    protected void applyPhysicsTransform(
            Vector3f physicsLocation, Quaternion physicsOrientation) {
        if (enabled && controlledSpatial != null) {
            Vector3f localLocation = controlledSpatial.getLocalTranslation();
            Quaternion localRotationQuat
                    = controlledSpatial.getLocalRotation(); // alias
            if (!localPhysics && controlledSpatial.getParent() != null) {
                localLocation
                        .set(physicsLocation)
                        .subtractLocal(
                                controlledSpatial.getParent()
                                        .getWorldTranslation());
                localLocation.divideLocal(
                        controlledSpatial.getParent().getWorldScale());
                tmpInverseWorldRotation
                        .set(controlledSpatial.getParent().getWorldRotation())
                        .inverseLocal().multLocal(localLocation);
                localRotationQuat.set(physicsOrientation);
                tmpInverseWorldRotation
                        .set(controlledSpatial.getParent().getWorldRotation())
                        .inverseLocal()
                        .mult(localRotationQuat, localRotationQuat);

                controlledSpatial.setLocalTranslation(localLocation);
                controlledSpatial.setLocalRotation(localRotationQuat);
            } else {
                controlledSpatial.setLocalTranslation(physicsLocation);
                controlledSpatial.setLocalRotation(physicsOrientation);
            }
        }
    }

    /**
     * Create spatial-dependent data. Invoked when this Control is added to a
     * Spatial.
     *
     * @param spatial the controlled Spatial (not null)
     */
    abstract protected void createSpatialData(Spatial spatial);

    /**
     * Access whichever (spatial) rotation corresponds to the physics rotation.
     *
     * @return a pre-existing rotation Quaternion (in physics-space coordinates,
     * not null)
     */
    protected Quaternion getSpatialRotation() {
        Quaternion result;
        if (MySpatial.isIgnoringTransforms(controlledSpatial)) {
            result = rotateIdentity;
        } else if (localPhysics) {
            result = controlledSpatial.getLocalRotation(); // alias
        } else {
            result = controlledSpatial.getWorldRotation(); // alias
        }

        return result;
    }

    /**
     * Access whichever (spatial) translation corresponds to the physics
     * location.
     *
     * @return a pre-existing location vector (in physics-space coordinates, not
     * null)
     */
    protected Vector3f getSpatialTranslation() {
        Vector3f result;
        if (MySpatial.isIgnoringTransforms(controlledSpatial)) {
            result = translateIdentity;
        } else if (localPhysics) {
            result = controlledSpatial.getLocalTranslation(); // alias
        } else {
            result = controlledSpatial.getWorldTranslation(); // alias
        }

        return result;
    }

    /**
     * Remove all managed physics objects from the PhysicsSpace.
     */
    abstract protected void removePhysics();

    /**
     * Destroy spatial-dependent data. Invoked when this Control is removed from
     * its Spatial.
     *
     * @param spatial the previous controlled Spatial (not null)
     */
    abstract protected void removeSpatialData(Spatial spatial);

    /**
     * Translate the physics object to the specified location.
     *
     * @param location the desired location (in physics-space coordinates, not
     * null, unaffected)
     */
    abstract protected void setPhysicsLocation(Vector3f location);

    /**
     * Rotate the physics object to the specified orientation.
     *
     * @param orientation the desired orientation (in physics-space coordinates,
     * not null, unaffected)
     */
    abstract protected void setPhysicsRotation(Quaternion orientation);
    // *************************************************************************
    // JmeCloneable methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned Control into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this Control (not null)
     * @param original the instance from which this Control was shallow-cloned
     * (unused)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        // space is never cloned.
        this.tmpInverseWorldRotation = cloner.clone(tmpInverseWorldRotation);
        this.controlledSpatial = cloner.clone(controlledSpatial);
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new Control (not null)
     */
    @Override
    public AbstractPhysicsControl jmeClone() {
        if (added) {
            String message = "Can't clone a " + getClass().getSimpleName()
                    + " while it's added to a physics space.";
            throw new IllegalStateException(message);
        }
        try {
            AbstractPhysicsControl clone = (AbstractPhysicsControl) clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }
    // *************************************************************************
    // PhysicsControl methods

    /**
     * Clone this Control for a different Spatial. No longer used as of JME 3.1.
     *
     * @param spatial (unused)
     * @return never
     * @throws UnsupportedOperationException always
     */
    @Override
    public Control cloneForSpatial(Spatial spatial) {
        throw new UnsupportedOperationException(
                "cloneForSpatial() isn't implemented.");
    }

    /**
     * Access the PhysicsSpace to which the object is (or would be) added.
     *
     * @return the pre-existing space, or null for none
     */
    @Override
    public PhysicsSpace getPhysicsSpace() {
        return space;
    }

    /**
     * Test whether this Control is enabled.
     *
     * @return true if enabled, otherwise false
     */
    @Override
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * De-serialize this Control from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        InputCapsule capsule = importer.getCapsule(this);

        this.enabled = capsule.readBoolean(tagEnabled, true);
        this.controlledSpatial
                = (Spatial) capsule.readSavable(tagSpatial, null);
        this.localPhysics = capsule.readBoolean(tagApplyLocalPhysics, false);
    }

    /**
     * Render this Control. Invoked once per ViewPort per frame, provided the
     * Control is added to a scene. Should be invoked only by a subclass or by
     * the RenderManager.
     *
     * @param rm the RenderManager (unused)
     * @param vp the ViewPort to render (unused)
     */
    @Override
    public void render(RenderManager rm, ViewPort vp) {
        // do nothing
    }

    /**
     * Enable or disable this Control.
     * <p>
     * When the Control is disabled, the physics object is removed from any
     * PhysicsSpace. When the Control is enabled again, the physics object is
     * moved to the location of the Spatial and then added to the PhysicsSpace.
     *
     * @param enable true&rarr;enable the Control, false&rarr;disable it
     */
    @Override
    public void setEnabled(boolean enable) {
        this.enabled = enable;
        if (space != null) {
            if (enable && !added) {
                if (controlledSpatial != null) {
                    setPhysicsLocation(getSpatialTranslation());
                    setPhysicsRotation(getSpatialRotation());
                }
                addPhysics();
                this.added = true;
            } else if (!enable && added) {
                removePhysics();
                this.added = false;
            }
        }
    }

    /**
     * If enabled, add this control's physics objects to the specified
     * PhysicsSpace. If not enabled, alter where the objects would be added. The
     * objects are removed from any other space they're in.
     *
     * @param newSpace where to add, or null to simply remove
     */
    @Override
    public void setPhysicsSpace(PhysicsSpace newSpace) {
        if (space == newSpace) {
            return;
        }

        if (added) {
            removePhysics();
            this.added = false;
        }
        this.space = newSpace;
        if (newSpace != null && isEnabled()) {
            addPhysics();
            this.added = true;
        }
        /*
         * If the Control isn't enabled, its physics objects will be
         * added to the new space when the Control becomes enabled.
         */
    }

    /**
     * Alter which Spatial is controlled. Invoked when the Control is added to
     * or removed from a Spatial. Should be invoked only by a subclass or from
     * Spatial. Do not invoke directly from user code. The correct way to add a
     * Control to a Spatial is with spatial.addControl().
     *
     * @param newSpatial the Spatial to control (or null)
     */
    @Override
    public void setSpatial(Spatial newSpatial) {
        if (controlledSpatial == newSpatial) {
            return;
        } else if (controlledSpatial != null) {
            removeSpatialData(controlledSpatial);
        }

        this.controlledSpatial = newSpatial;

        if (newSpatial != null) {
            createSpatialData(controlledSpatial);
            setPhysicsLocation(getSpatialTranslation());
            setPhysicsRotation(getSpatialRotation());
        }
    }

    /**
     * Serialize this Control to the specified exporter, for example when saving
     * to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(enabled, tagEnabled, true);
        capsule.write(localPhysics, tagApplyLocalPhysics, false);
        // added, space, and tmpInverseWorldRotation are never written.
        capsule.write(controlledSpatial, tagSpatial, null);
    }
}
