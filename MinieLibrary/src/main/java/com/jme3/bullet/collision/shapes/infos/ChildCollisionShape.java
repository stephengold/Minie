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
package com.jme3.bullet.collision.shapes.infos;

import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.math.Matrix3f;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import com.jme3.util.clone.JmeCloneable;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * An element of a CompoundCollisionShape, consisting of a (non-compound) child
 * shape, offset and rotated with respect to its parent.
 *
 * @author normenhansen
 */
public class ChildCollisionShape
        implements JmeCloneable, Savable {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(ChildCollisionShape.class.getName());
    // *************************************************************************
    // fields

    /**
     * translation relative to parent shape (not null)
     */
    private Vector3f location;
    /**
     * rotation relative to parent shape (not null)
     */
    private Matrix3f rotation;
    /**
     * base shape (not null, not a compound shape)
     */
    private CollisionShape shape;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public ChildCollisionShape() {
    }

    /**
     * Instantiate a child shape for use in a compound shape.
     *
     * @param location translation relative to the parent (not null, unaffected)
     * @param rotation rotation relative to the parent (not null, unaffected)
     * @param shape the base shape (not null, not a compound shape, alias
     * created)
     */
    public ChildCollisionShape(Vector3f location, Matrix3f rotation,
            CollisionShape shape) {
        Validate.nonNull(location, "location");
        Validate.nonNull(rotation, "rotation");
        Validate.nonNull(shape, "shape");
        if (shape instanceof CompoundCollisionShape) {
            throw new IllegalArgumentException(
                    "CompoundCollisionShapes cannot be child shapes!");
        }

        this.location = location.clone();
        this.rotation = rotation.clone();
        this.shape = shape;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the translation relative to the parent shape.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a translation vector (either storeResult or a new vector, not
     * null)
     */
    public Vector3f getLocation(Vector3f storeResult) {
        if (storeResult == null) {
            return location.clone();
        } else {
            return storeResult.set(location);
        }
    }

    /**
     * Copy the rotation relative to the parent shape.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a rotation matrix (either storeResult or a new matrix, not null)
     */
    public Matrix3f getRotation(Matrix3f storeResult) {
        if (storeResult == null) {
            return rotation.clone();
        } else {
            return storeResult.set(rotation);
        }
    }

    /**
     * Access the base shape.
     *
     * @return the pre-existing shape (not null)
     */
    public CollisionShape getShape() {
        assert shape != null;
        return shape;
    }
    // *************************************************************************
    // JmeCloneable methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned element into a deep-cloned one, using the specified cloner
     * and original to resolve copied fields.
     *
     * @param cloner the cloner that's cloning this element (not null)
     * @param original the instance from which this instance was shallow-cloned
     * (unused)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        location = cloner.clone(location);
        rotation = cloner.clone(rotation);
        shape = cloner.clone(shape);
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public ChildCollisionShape jmeClone() {
        try {
            ChildCollisionShape clone = (ChildCollisionShape) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }
    // *************************************************************************
    // Savable methods

    /**
     * Serialize this shape, for example when saving to a J3O file.
     *
     * @param ex exporter (not null)
     * @throws IOException from exporter
     */
    @Override
    public void write(JmeExporter ex) throws IOException {
        OutputCapsule capsule = ex.getCapsule(this);
        capsule.write(location, "location", new Vector3f());
        capsule.write(rotation, "rotation", new Matrix3f());
        capsule.write(shape, "shape",
                new BoxCollisionShape(new Vector3f(1f, 1f, 1f)));
    }

    /**
     * De-serialize this shape, for example when loading from a J3O file.
     *
     * @param im importer (not null)
     * @throws IOException from importer
     */
    @Override
    public void read(JmeImporter im) throws IOException {
        InputCapsule capsule = im.getCapsule(this);
        location = (Vector3f) capsule.readSavable("location", new Vector3f());
        rotation = (Matrix3f) capsule.readSavable("rotation", new Matrix3f());
        shape = (CollisionShape) capsule.readSavable("shape",
                new BoxCollisionShape(new Vector3f(1f, 1f, 1f)));
    }
}
