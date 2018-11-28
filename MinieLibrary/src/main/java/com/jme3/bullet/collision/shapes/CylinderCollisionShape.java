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

import com.jme3.bullet.PhysicsSpace;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * A cylindrical collision shape based on Bullet's btCylinderShapeX, new
 * btCylinderShape, or btCylinderShapeZ.
 *
 * @author normenhansen
 */
public class CylinderCollisionShape extends CollisionShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(CylinderCollisionShape.class.getName());
    // *************************************************************************
    // fields

    /**
     * copy of unscaled half extent for each local axis (not null, no negative
     * component)
     */
    private Vector3f halfExtents = new Vector3f(0.5f, 0.5f, 0.5f);
    /**
     * copy of main (height) axis (0&rarr;X, 1&rarr;Y, 2&rarr;Z)
     */
    private int axis;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public CylinderCollisionShape() {
    }

    /**
     * Instantiate a Z-axis cylinder shape with the specified half extents.
     *
     * @param halfExtents the desired unscaled half extents (not null, no
     * negative component, unaffected)
     */
    public CylinderCollisionShape(Vector3f halfExtents) {
        Validate.nonNegative(halfExtents, "half extents");

        this.halfExtents.set(halfExtents);
        axis = PhysicsSpace.AXIS_Z;
        createShape();
    }

    /**
     * Instantiate a cylinder shape around the specified axis.
     *
     * @param halfExtents the desired unscaled half extents (not null, no
     * negative component, unaffected)
     * @param axis which local axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     */
    public CylinderCollisionShape(Vector3f halfExtents, int axis) {
        Validate.nonNegative(halfExtents, "half extents");
        Validate.inRange(axis, "axis", 0, 2);

        this.halfExtents.set(halfExtents);
        this.axis = axis;
        createShape();
    }
    // *************************************************************************

    /**
     * Test whether the specified scaling factors can be applied to this shape.
     * For cylinder shapes, radial scaling must be uniform.
     *
     * @param scale the desired scaling factor for each local axis (may be null,
     * unaffected)
     * @return true if applicable, otherwise false
     */
    @Override
    public boolean canScale(Vector3f scale) {
        boolean canScale = super.canScale(scale);
        if (canScale) {
            if (axis == PhysicsSpace.AXIS_X && scale.y != scale.z) {
                canScale = false;
            } else if (axis == PhysicsSpace.AXIS_Y && scale.x != scale.z) {
                canScale = false;
            } else if (axis == PhysicsSpace.AXIS_Z && scale.x != scale.y) {
                canScale = false;
            }
        }

        return canScale;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the half extents of the cylinder.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the unscaled half extent for each local axis (either storeResult
     * or a new vector, not null, no negative component)
     */
    public final Vector3f getHalfExtents(Vector3f storeResult) {
        assert MyVector3f.isAllNonNegative(halfExtents) : halfExtents;
        if (storeResult == null) {
            return halfExtents.clone();
        } else {
            return storeResult.set(halfExtents);
        }
    }

    /**
     * Determine the main (height) axis of the cylinder.
     *
     * @return the axis index: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     */
    public int getAxis() {
        assert axis == PhysicsSpace.AXIS_X
                || axis == PhysicsSpace.AXIS_Y
                || axis == PhysicsSpace.AXIS_Z : axis;
        return axis;
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
        halfExtents = cloner.clone(halfExtents);
        createShape();
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public CylinderCollisionShape jmeClone() {
        try {
            CylinderCollisionShape clone
                    = (CylinderCollisionShape) super.clone();
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
        capsule.write(halfExtents, "halfExtents",
                new Vector3f(0.5f, 0.5f, 0.5f));
        capsule.write(axis, "axis", PhysicsSpace.AXIS_Y);
    }

    /**
     * De-serialize this shape, for example when loading from a J3O file.
     *
     * @param im importer (not null)
     * @throws IOException from importer
     */
    @Override
    public void read(JmeImporter im) throws IOException {
        super.read(im);
        InputCapsule capsule = im.getCapsule(this);
        Vector3f he = (Vector3f) capsule.readSavable("halfExtents",
                new Vector3f(0.5f, 0.5f, 0.5f));
        halfExtents.set(he);
        axis = capsule.readInt("axis", PhysicsSpace.AXIS_Y);
        createShape();
    }
    // *************************************************************************
    // private methods

    /**
     * Instantiate the configured shape in Bullet.
     */
    private void createShape() {
        assert axis == PhysicsSpace.AXIS_X
                || axis == PhysicsSpace.AXIS_Y
                || axis == PhysicsSpace.AXIS_Z : axis;
        assert MyVector3f.isAllNonNegative(halfExtents) : halfExtents;
        assert objectId == 0L : objectId;

        objectId = createShape(axis, halfExtents);
        assert objectId != 0L;
        logger2.log(Level.FINE, "Created Shape {0}", Long.toHexString(objectId));

        setScale(scale);
        setMargin(margin);
    }

    native private long createShape(int axis, Vector3f halfExtents);
}
