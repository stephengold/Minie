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
import com.jme3.bullet.collision.shapes.infos.CompoundMesh;
import com.jme3.bullet.collision.shapes.infos.IndexedMesh;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Triangle;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * A mesh collisions shape based on Bullet's {@code btGImpactMeshShape}.
 *
 * @author normenhansen
 */
public class GImpactCollisionShape extends CollisionShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(GImpactCollisionShape.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagNativeMesh = "nativeMesh";
    // *************************************************************************
    // fields

    /**
     * native mesh used to construct this shape
     */
    private CompoundMesh nativeMesh;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected GImpactCollisionShape() {
    }

    /**
     * Instantiate a shape based on the specified CompoundMesh and offset.
     *
     * @param mesh the mesh on which to base the shape (not null, unaffected)
     * @param offset the offset to add to the vertex positions (not null,
     * unaffected)
     */
    public GImpactCollisionShape(CompoundMesh mesh, Vector3f offset) {
        this.nativeMesh = new CompoundMesh(mesh, offset);
        createShape();
    }

    /**
     * Instantiate a shape based on the specified native mesh(es).
     *
     * @param submeshes the mesh(es) on which to base the shape (not null)
     */
    public GImpactCollisionShape(IndexedMesh... submeshes) {
        this.nativeMesh = new CompoundMesh();
        for (IndexedMesh submesh : submeshes) {
            nativeMesh.add(submesh);
        }
        createShape();
    }

    /**
     * Instantiate a shape based on the specified JME mesh(es).
     *
     * @param jmeMeshes the mesh(es) on which to base the shape (not null,
     * unaffected)
     */
    public GImpactCollisionShape(Mesh... jmeMeshes) {
        this.nativeMesh = new CompoundMesh(jmeMeshes);
        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Count how many triangles are in the mesh.
     *
     * @return the count (&ge;0)
     */
    public int countMeshTriangles() {
        int result = nativeMesh.countTriangles();
        return result;
    }

    /**
     * Count how many vertices are in the mesh.
     *
     * @return the count (&ge;0)
     */
    public int countMeshVertices() {
        int numVertices = nativeMesh.countVertices();
        return numVertices;
    }

    /**
     * Attempt to divide this shape into 2 shapes.
     *
     * @param splittingTriangle to define the splitting plane (in shape
     * coordinates, not null, unaffected)
     * @return a pair of children, the first child generated by the plane's
     * minus side and the 2nd child generated by its plus side; either child may
     * be null, indicating an empty shape
     */
    public ChildCollisionShape[] split(Triangle splittingTriangle) {
        Validate.nonNull(splittingTriangle, "splitting triangle");

        CompoundMesh[] mp = nativeMesh.split(splittingTriangle);
        ChildCollisionShape[] result = new ChildCollisionShape[2];
        int numMinus = (mp[0] == null) ? 0 : mp[0].countTriangles();
        int numPlus = (mp[1] == null) ? 0 : mp[1].countTriangles();
        if (numMinus == 0 || numPlus == 0) {
            // Degenerate case:  all triangles lie to one side of the plane.
            ChildCollisionShape child
                    = new ChildCollisionShape(new Vector3f(), this);
            if (numMinus > 0) {
                result[0] = child;
            } else if (numPlus > 0) {
                result[1] = child;
            }

        } else {
            Vector3f max = new Vector3f();
            Vector3f min = new Vector3f();
            Vector3f offset = max; // alias
            Vector3f center = min; // alias
            for (int i = 0; i < 2; ++i) {
                mp[i].maxMin(max, min);
                MyVector3f.midpoint(max, min, center);
                offset.set(center).negateLocal();
                GImpactCollisionShape shape
                        = new GImpactCollisionShape(mp[i], offset);
                shape.setScale(scale);
                result[i] = new ChildCollisionShape(center, shape);
            }
        }

        return result;
    }
    // *************************************************************************
    // CollisionShape methods

    /**
     * Test whether this shape can be split by an arbitrary plane.
     *
     * @return true if splittable, false otherwise
     */
    @Override
    public boolean canSplit() {
        return true;
    }

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

        this.nativeMesh = cloner.clone(nativeMesh);
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
        this.nativeMesh
                = (CompoundMesh) capsule.readSavable(tagNativeMesh, null);
        createShape();
    }

    /**
     * Recalculate this shape's bounding box if necessary.
     */
    @Override
    protected void recalculateAabb() {
        long shapeId = nativeId();
        recalcAabb(shapeId);
    }

    /**
     * Alter the scale factors of this shape.
     * <p>
     * Note that if the shape is shared (between collision objects and/or
     * compound shapes) changes can have unintended consequences.
     *
     * @param scale the desired scale factor for each local axis (not null, no
     * negative component, unaffected, default=(1,1,1))
     */
    @Override
    public void setScale(Vector3f scale) {
        super.setScale(scale);
        recalculateAabb();
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
        capsule.write(nativeMesh, tagNativeMesh, null);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured {@code btGImpactMeshShape}.
     */
    private void createShape() {
        long meshId = nativeMesh.nativeId();
        long shapeId = createShape(meshId);
        setNativeId(shapeId);

        setContactFilterEnabled(enableContactFilter);
        setScale(scale);
        setMargin(margin);
    }
    // *************************************************************************
    // native private methods

    native private static long createShape(long meshId);

    native private static void recalcAabb(long shapeId);
}
