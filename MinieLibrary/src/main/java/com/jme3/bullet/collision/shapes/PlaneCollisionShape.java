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

import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Plane;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Logger;

/**
 * A planar collision shape based on Bullet's {@code btStaticPlaneShape}. Not
 * for use in dynamic bodies. Collisions between HeightfieldCollisionShape,
 * MeshCollisionShape, and PlaneCollisionShape objects are never detected.
 *
 * @author normenhansen
 */
public class PlaneCollisionShape extends CollisionShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(PlaneCollisionShape.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagCollisionPlane = "collisionPlane";
    // *************************************************************************
    // fields

    /**
     * defining plane
     */
    private Plane plane;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected PlaneCollisionShape() {
    }

    /**
     * Instantiate a plane shape defined by the specified plane.
     *
     * @param plane the desired plane (not null, unaffected)
     */
    public PlaneCollisionShape(Plane plane) {
        this.plane = plane.clone();
        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the defining plane.
     *
     * @return a new instance (not null)
     */
    final public Plane getPlane() {
        Plane result = plane.clone();
        return result;
    }
    // *************************************************************************
    // CollisionShape methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned shape into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this shape (not null)
     * @param original the instance from which this shape was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);
        this.plane = cloner.clone(plane);
        createShape();
    }

    /**
     * Determine how far this shape extends from its center.
     *
     * @return the distance (in physics-space units, &ge;0)
     */
    @Override
    public float maxRadius() {
        return Float.POSITIVE_INFINITY;
    }

    /**
     * De-serialize this shape from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);

        InputCapsule capsule = importer.getCapsule(this);
        this.plane
                = (Plane) capsule.readSavable(tagCollisionPlane, new Plane());
        createShape();
    }

    /**
     * Serialize this shape to the specified exporter, for example when saving
     * to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        super.write(exporter);

        OutputCapsule capsule = exporter.getCapsule(this);
        capsule.write(plane, tagCollisionPlane, new Plane());
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured {@code btStaticPlaneShape}.
     */
    private void createShape() {
        long shapeId = createShape(plane.getNormal(), plane.getConstant());
        setNativeId(shapeId);

        setContactFilterEnabled(enableContactFilter);
        setScale(scale);
        setMargin(margin);
    }
    // *************************************************************************
    // native private methods

    native private static long createShape(Vector3f normal, float constant);
}
