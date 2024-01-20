/*
 * Copyright (c) 2024 jMonkeyEngine
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
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * An abstract base class for custom collision shapes defined in terms of their
 * supporting vertices, based on Bullet's {@code btConvexInternalShape}.
 *
 * @author Stephen Gold sgold@sonic.net
 */
abstract public class CustomConvexShape extends ConvexShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger loggerY
            = Logger.getLogger(CustomConvexShape.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagHalfExtents = "halfExtents";
    final private static String tagInertia = "inertia";
    // *************************************************************************
    // fields

    /**
     * temporary storage for one vector per thread
     */
    final protected static ThreadLocal<Vector3f> threadTmpVector
            = new ThreadLocal<Vector3f>() {
        @Override
        protected Vector3f initialValue() {
            return new Vector3f();
        }
    };
    /**
     * copy of the half extents on each local axis, for scale=(1,1,1) and
     * margin=0, or {@code null} to calculate AABBs using the supporting
     * vertices
     */
    private Vector3f halfExtents;
    /**
     * copy of the rotational inertia for each local axis, for a shape with
     * mass=1 and scale=(1,1,1)
     */
    private Vector3f inertia;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected CustomConvexShape() {
    }

    /**
     * Instantiate a custom collision shape with the specified extent and
     * inertia.
     *
     * @param halfExtents the desired half extents on each local axis, for
     * scale=(1,1,1) and margin=0 (unaffected), or {@code null} to calculate
     * AABBs using the supporting vertices
     * @param inertia the desired local inertia vector for a shape with mass=1
     * and scale=(1,1,1) (not null, all components &gt;0, unaffected)
     */
    public CustomConvexShape(Vector3f halfExtents, Vector3f inertia) {
        Validate.positive(inertia, "inertia");

        this.halfExtents = (halfExtents == null) ? null : halfExtents.clone();
        this.inertia = inertia.clone();
        createShape();
    }
    // *************************************************************************
    // new protected methods

    /**
     * Locate the shape's supporting vertex for the specified direction,
     * excluding collision margin.
     * <p>
     * This method is invoked by native code.
     *
     * @param dirX the X-coordinate of the direction to test (in scaled shape
     * coordinates)
     * @param dirY the Y-coordinate of the direction to test (in scaled shape
     * coordinates)
     * @param dirZ the Z-coordinate of the direction to test (in scaled shape
     * coordinates)
     * @return the location of the supporting vertex (in scaled shape
     * coordinates)
     */
    abstract protected Vector3f locateSupport(
            float dirX, float dirY, float dirZ);
    // *************************************************************************
    // ConvexShape methods

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

        this.halfExtents = cloner.clone(halfExtents);
        this.inertia = cloner.clone(inertia);
        createShape();
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

        this.halfExtents = (Vector3f) capsule.readSavable(tagHalfExtents, null);
        this.inertia = (Vector3f) capsule.readSavable(
                tagInertia, new Vector3f(1f, 1f, 1f));
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

        capsule.write(halfExtents, tagHalfExtents, null);
        capsule.write(inertia, tagInertia, new Vector3f(1f, 1f, 1f));
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured shape in Bullet.
     */
    private void createShape() {
        long shapeId = createShapeNative(halfExtents, inertia);
        setNativeId(shapeId);

        setContactFilterEnabled(enableContactFilter);
        setScale(scale);
        setMargin(margin);
    }
    // *************************************************************************
    // native private methods

    native private long createShapeNative(Vector3f he, Vector3f inertia);
}
