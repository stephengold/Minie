/*
 * Copyright (c) 2019 jMonkeyEngine
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
package jme3utilities.minie;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.control.AbstractPhysicsControl;
import com.jme3.bullet.objects.PhysicsCharacter;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Spatial;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A physics control to link a PhysicsCharacter to a Spatial. Compare with
 * BetterCharacterControl and JME's CharacterControl.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class MinieCharacterControl extends AbstractPhysicsControl {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(MinieCharacterControl.class.getName());
    /**
     * local copy of {@link com.jme3.math.Vector3f#ZERO}
     */
    final private static Vector3f translateIdentity = new Vector3f(0f, 0f, 0f);
    // *************************************************************************
    // fields

    /**
     * underlying collision object
     */
    private PhysicsCharacter character = null;
    /**
     * copy of the PhysicsCharacter location
     */
    final private Vector3f location = new Vector3f();
    /**
     * view direction
     */
    private Vector3f viewDirection = new Vector3f(Vector3f.UNIT_Z);
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public MinieCharacterControl() {
    }

    /**
     * Instantiate an enabled control with the specified CollisionShape and step
     * height.
     *
     * @param shape the desired shape (not null, alias created)
     * @param stepHeight the maximum amount of normal vertical movement (in
     * physics-space units)
     */
    public MinieCharacterControl(CollisionShape shape, float stepHeight) {
        character = new PhysicsCharacter(shape, stepHeight);
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
     * Jump in the "up" direction.
     */
    public void jump() {
        character.jump(translateIdentity);
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
     * Alter the character's fall speed.
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
     * physics-space units per second squared, not null, unaffected,
     * default=29.4)
     */
    public void setGravity(float downwardAcceleration) {
        Vector3f gVector = character.getUpDirection(null);
        gVector.multLocal(-downwardAcceleration);
        character.setGravity(gVector);
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
        viewDirection.normalizeLocal();
    }

    /**
     * Alter the character's walk offset. The offset will continue to be applied
     * until altered again.
     *
     * @param offset the desired position increment for each physics tick (in
     * physics-space coordinates, not null, unaffected)
     */
    public void setWalkDirection(Vector3f offset) {
        Validate.nonNull(offset, "offset");
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
     * @param original the Control from which this Control was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);

        character = cloner.clone(character);
        // tmpQuaternion not cloned
        // tmpVector not cloned
        viewDirection = cloner.clone(viewDirection);
    }

    /**
     * Create spatial-dependent data. Invoked when this control is added to a
     * spatial.
     *
     * @param spat the controlled spatial (not null)
     */
    @Override
    protected void createSpatialData(Spatial spat) {
        character.setUserObject(spat);
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public MinieCharacterControl jmeClone() {
        try {
            MinieCharacterControl clone
                    = (MinieCharacterControl) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }

    /**
     * De-serialize this Control, for example when loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);
        InputCapsule capsule = importer.getCapsule(this);

        character = (PhysicsCharacter) capsule.readSavable("character", null);
        viewDirection = (Vector3f) capsule.readSavable("viewDirection",
                new Vector3f(Vector3f.UNIT_Z));
    }

    @Override
    protected void removePhysics() {
        PhysicsSpace space = getPhysicsSpace();
        space.removeCollisionObject(character);
    }

    /**
     * Destroy spatial-dependent data. Invoked when this control is removed from
     * a spatial.
     *
     * @param spat the previously controlled spatial (not null)
     */
    @Override
    protected void removeSpatialData(Spatial spat) {
        character.setUserObject(null);
    }

    /**
     * Render this control. Invoked once per view port per frame, provided the
     * control is added to a scene. Should be invoked only by a subclass or by
     * the RenderManager.
     *
     * @param rm the render manager (not null)
     * @param vp the view port to render (not null)
     */
    @Override
    public void render(RenderManager rm, ViewPort vp) {
        // does nothing
    }

    /**
     * Translate the PhysicsCharacter to the specified location.
     *
     * @param vec the desired location (not null, unaffected)
     */
    @Override
    public void setPhysicsLocation(Vector3f vec) {
        character.setPhysicsLocation(vec);
        location.set(vec);
    }

    /**
     * Rotate the PhysicsCharacter to the specified orientation.
     *
     * @param quat the desired orientation (not null, unaffected)
     */
    @Override
    protected void setPhysicsRotation(Quaternion quat) {
        // does nothing
    }

    /**
     * Update this control. Invoked once per frame during the logical-state
     * update, provided the control is added to a scene graph. Do not invoke
     * directly from user code.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void update(float tpf) {
        character.getPhysicsLocation(location);
        Vector3f up = character.getUpDirection(null);
        Quaternion orientation = new Quaternion();
        orientation.lookAt(viewDirection, up);
        applyPhysicsTransform(location, orientation);
    }

    /**
     * Serialize this Control, for example when saving to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        super.write(exporter);
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(character, "character", null);
        capsule.write(viewDirection, "viewDirection", null);
    }
}
