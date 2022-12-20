/*
 * Copyright (c) 2019-2022 jMonkeyEngine
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
import com.jme3.bullet.collision.shapes.ConvexShape;
import com.jme3.bullet.objects.PhysicsCharacter;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Spatial;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * A PhysicsControl to link a PhysicsCharacter to a Spatial.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class CharacterControl extends AbstractPhysicsControl {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(CharacterControl.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagCharacter = "character";
    final private static String tagViewDirection = "viewDirection";
    // *************************************************************************
    // fields

    /**
     * underlying collision object
     */
    private PhysicsCharacter character = null;
    /**
     * temporary storage for a character's orientation
     */
    final private static Quaternion tmpOrientation = new Quaternion();
    /**
     * view direction
     */
    private Vector3f viewDirection = new Vector3f(0f, 0f, 1f);
    /**
     * temporary storage for vectors
     */
    final private static Vector3f tmpVector = new Vector3f();
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected CharacterControl() {
    }

    /**
     * Instantiate an enabled Control with the specified CollisionShape and step
     * height.
     *
     * @param shape the desired shape (not null, alias created)
     * @param stepHeight the maximum amount of normal vertical movement (in
     * physics-space units)
     */
    public CharacterControl(ConvexShape shape, float stepHeight) {
        this.character = new PhysicsCharacter(shape, stepHeight);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Access the PhysicsCharacter managed by this Control.
     *
     * @return the pre-existing instance (not null)
     */
    public PhysicsCharacter getCharacter() {
        assert character != null;
        return character;
    }

    /**
     * Copy the character's location.
     *
     * @return a new location vector (in physics-space coordinates, not null)
     */
    public Vector3f getPhysicsLocation() {
        Vector3f result = character.getPhysicsLocation(null);
        return result;
    }

    /**
     * Copy the character's view direction. This need not agree with the
     * spatial's forward direction.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a direction vector (in physics-space coordinates, either the
     * provided storage or a new vector, not null)
     */
    public Vector3f getViewDirection(Vector3f storeResult) {
        Vector3f result;
        if (storeResult == null) {
            result = viewDirection.clone();
        } else {
            result = storeResult.set(viewDirection);
        }

        return result;
    }

    /**
     * Jump in the "up" direction.
     */
    public void jump() {
        character.jump();
    }

    /**
     * Test whether the character is on the ground.
     *
     * @return true if on the ground, otherwise false
     */
    public boolean onGround() {
        boolean result = character.onGround();
        return result;
    }

    /**
     * Alter this character's maximum fall speed (terminal velocity).
     *
     * @param speed the desired speed (in physics-space units per second,
     * default=55)
     */
    public void setFallSpeed(float speed) {
        character.setFallSpeed(speed);
    }

    /**
     * Alter the character's gravitational acceleration.
     *
     * @param downwardAcceleration the desired downward acceleration (in
     * physics-space units per second squared, not null, default=29.4)
     */
    public void setGravity(float downwardAcceleration) {
        character.setGravity(downwardAcceleration);
    }

    /**
     * Alter the character's jump speed.
     *
     * @param speed the desired speed (in physics-space units per second,
     * default=10)
     */
    public void setJumpSpeed(float speed) {
        character.setJumpSpeed(speed);
    }

    /**
     * Alter the view direction.
     *
     * @param direction the desired direction (in physics-space coordinates, not
     * null, not zero)
     */
    public void setViewDirection(Vector3f direction) {
        Validate.nonZero(direction, "direction");

        viewDirection.set(direction);
        MyVector3f.normalizeLocal(viewDirection);
    }

    /**
     * Alter the character's walk offset. The offset must be perpendicular to
     * the "up" direction. It will continue to be applied until altered again.
     *
     * @param offset the desired position increment for each simulation step (in
     * physics-space coordinates, not null, unaffected)
     */
    public void setWalkDirection(Vector3f offset) {
        Validate.finite(offset, "offset");
        character.setWalkDirection(offset);
    }
    // *************************************************************************
    // AbstractPhysicsControl methods

    /**
     * Add all managed physics objects to the PhysicsSpace.
     */
    @Override
    protected void addPhysics() {
        PhysicsSpace space = getPhysicsSpace();
        space.addCollisionObject(character);
    }

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned Control into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this Control (not null, modified)
     * @param original the instance from which this Control was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);

        this.character = cloner.clone(character);
        // tmpQuaternion and tmpVector are never cloned.
        this.viewDirection = cloner.clone(viewDirection);
    }

    /**
     * Create spatial-dependent data. Invoked when this Control is added to a
     * Spatial.
     *
     * @param spatial the controlled spatial (not null, alias created)
     */
    @Override
    protected void createSpatialData(Spatial spatial) {
        character.setUserObject(spatial);
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
        super.read(importer);
        InputCapsule capsule = importer.getCapsule(this);

        this.character
                = (PhysicsCharacter) capsule.readSavable(tagCharacter, null);
        this.viewDirection = (Vector3f) capsule
                .readSavable(tagViewDirection, new Vector3f(0f, 0f, 1f));

        if (character != null) {
            Spatial controlled = getSpatial();
            character.setUserObject(controlled);
        }
    }

    /**
     * Remove all managed physics objects from the PhysicsSpace.
     */
    @Override
    protected void removePhysics() {
        PhysicsSpace space = getPhysicsSpace();
        space.removeCollisionObject(character);
    }

    /**
     * Destroy spatial-dependent data. Invoked when this Control is removed from
     * its Spatial.
     *
     * @param spatial the Spatial to which this Control was added (unused)
     */
    @Override
    protected void removeSpatialData(Spatial spatial) {
        character.setUserObject(null);
    }

    /**
     * Translate the PhysicsCharacter to the specified location.
     *
     * @param location the desired location (not null, unaffected)
     */
    @Override
    public void setPhysicsLocation(Vector3f location) {
        character.setPhysicsLocation(location);
    }

    /**
     * Rotate the PhysicsCharacter to the specified orientation.
     *
     * @param orientation the desired orientation (not null, unaffected)
     */
    @Override
    protected void setPhysicsRotation(Quaternion orientation) {
        // do nothing
    }

    /**
     * Update this Control. Invoked once per frame during the logical-state
     * update, provided the Control is added to a scene. Do not invoke directly
     * from user code.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void update(float tpf) {
        if (!isEnabled()) {
            return;
        }

        character.getUpDirection(tmpVector);
        tmpOrientation.lookAt(viewDirection, tmpVector);

        character.getPhysicsLocation(tmpVector);
        applyPhysicsTransform(tmpVector, tmpOrientation);
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
        super.write(exporter);
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(character, tagCharacter, null);
        capsule.write(viewDirection, tagViewDirection, null);
    }
}
