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
package com.jme3.bullet.collision.shapes;

import com.jme3.bullet.collision.shapes.infos.ChildCollisionShape;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Matrix3f;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * A collision shape formed by combining convex child shapes, based on Bullet's
 * btCompoundShape.
 *
 * @author normenhansen
 */
public class CompoundCollisionShape extends CollisionShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(CompoundCollisionShape.class.getName());
    // *************************************************************************
    // fields

    /**
     * child shapes of this shape
     */
    private ArrayList<ChildCollisionShape> children = new ArrayList<>(6);
    // *************************************************************************
    // constructors

    /**
     * Instantiate an empty compound shape (with no children).
     */
    public CompoundCollisionShape() {
        createEmpty();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Add a child shape with the specified local translation.
     *
     * @param shape the child shape to add (not null, not a compound shape,
     * alias created)
     * @param location the local coordinates of the child shape's center (not
     * null, unaffected)
     */
    public void addChildShape(CollisionShape shape, Vector3f location) {
        addChildShape(shape, location, new Matrix3f());
    }

    /**
     * Add a child shape with the specified local translation and orientation.
     *
     * @param shape the child shape to add (not null, not a compound shape,
     * alias created)
     * @param location the local coordinates of the child shape's center (not
     * null, unaffected)
     * @param rotation the local orientation of the child shape (not null,
     * unaffected)
     */
    public void addChildShape(CollisionShape shape, Vector3f location,
            Matrix3f rotation) {
        if (shape instanceof CompoundCollisionShape) {
            throw new IllegalArgumentException(
                    "CompoundCollisionShapes cannot have CompoundCollisionShapes as children!");
        }
        children.add(new ChildCollisionShape(location.clone(),
                rotation.clone(), shape));
        addChildShape(objectId, shape.getObjectId(), location, rotation);
    }

    /**
     * Remove a child from this shape.
     *
     * @param shape the child shape to remove (not null)
     */
    public void removeChildShape(CollisionShape shape) {
        removeChildShape(objectId, shape.getObjectId());
        for (Iterator<ChildCollisionShape> it = children.iterator();
                it.hasNext();) {
            ChildCollisionShape childCollisionShape = it.next();
            if (childCollisionShape.getShape() == shape) {
                it.remove();
            }
        }
    }

    /**
     * Access the list of children.
     *
     * @return the pre-existing list (not null) TODO
     */
    public List<ChildCollisionShape> getChildren() {
        return children;
    }
    // *************************************************************************
    // CollisionShape methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned shape into a deep-cloned one, using the specified cloner
     * and original to resolve copied fields.
     *
     * @param cloner the cloner that's cloning this shape (not null)
     * @param original the instance from which this instance was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);
        children = cloner.clone(children);
        createEmpty();
        loadChildren();
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public CompoundCollisionShape jmeClone() {
        try {
            CompoundCollisionShape clone
                    = (CompoundCollisionShape) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }

    /**
     * Serialize this shape, for example when saving to a J3O file.
     *
     * @param ex exporter (not null)
     * @throws IOException from exporter
     */
    @Override
    public void write(JmeExporter ex) throws IOException {
        super.write(ex);
        OutputCapsule capsule = ex.getCapsule(this);
        capsule.writeSavableArrayList(children, "children",
                new ArrayList<ChildCollisionShape>());
    }

    /**
     * De-serialize this shape, for example when loading from a J3O file.
     *
     * @param im importer (not null)
     * @throws IOException from importer
     */
    @Override
    @SuppressWarnings("unchecked")
    public void read(JmeImporter im) throws IOException {
        super.read(im);
        InputCapsule capsule = im.getCapsule(this);
        children = capsule.readSavableArrayList("children",
                new ArrayList<ChildCollisionShape>());
        loadChildren();
    }
    // *************************************************************************
    // private methods

    native private long addChildShape(long objectId, long childId,
            Vector3f location, Matrix3f rotation);

    /**
     * Instantiate an empty btCompoundShape.
     */
    private void createEmpty() {
        assert objectId == 0L;

        objectId = createShape();
        assert objectId != 0L;
        logger2.log(Level.FINE, "Created Shape {0}", Long.toHexString(objectId));

        setScale(scale);
        setMargin(margin);
    }

    native private long createShape();

    /**
     * Add the configured children to the empty btCompoundShape.
     */
    private void loadChildren() {
        assert objectId != 0L;

        for (ChildCollisionShape child : children) {
            addChildShape(objectId, child.getShape().getObjectId(),
                    child.getLocation(null), child.getRotation(null));
        }
    }

    native private long removeChildShape(long objectId, long childId);
}
