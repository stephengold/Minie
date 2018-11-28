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
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.util.BufferUtils;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * A terrain collision shape based on Bullet's btHeightfieldTerrainShape.
 * <p>
 * This is much more efficient than a regular mesh, but it has a couple
 * limitations:
 * <ul>
 * <li>No rotation or translation.</li>
 * <li>The collision bounding box must be centered on (0,0,0) with the height
 * above and below the X-Z plane being equal on either side. If not, the whole
 * collision box is shifted vertically and objects won't collide properly.</li>
 * </ul>
 *
 * @author Brent Owens
 */
public class HeightfieldCollisionShape extends CollisionShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(HeightfieldCollisionShape.class.getName());
    // *************************************************************************
    // fields

    /**
     * copy of number of rows in the heightfield (&gt;1)
     */
    private int heightStickWidth;
    /**
     * copy of number of columns in the heightfield (&gt;1)
     */
    private int heightStickLength;
    /**
     * array of heightfield samples
     */
    private float[] heightfieldData;
    private float heightScale = 1f;
    private float minHeight;
    private float maxHeight;
    /**
     * copy of height axis (0&rarr;X, 1&rarr;Y, 2&rarr;Z)
     */
    private int upAxis = PhysicsSpace.AXIS_Y;
    private boolean flipQuadEdges = false;
    /**
     * buffer for passing height data to Bullet
     * <p>
     * A Java reference must persist after createShape() completes, or else the
     * buffer might get garbage collected.
     */
    private ByteBuffer bbuf;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public HeightfieldCollisionShape() {
    }

    /**
     * Instantiate a new shape for the specified height map.
     *
     * @param heightmap (not null, length&ge;4, length a perfect square,
     * unaffected)
     */
    public HeightfieldCollisionShape(float[] heightmap) {
        Validate.nonEmpty(heightmap, "heightmap");
        assert heightmap.length >= 4 : heightmap.length;

        createCollisionHeightfield(heightmap, Vector3f.UNIT_XYZ);
    }

    /**
     * Instantiate a new shape for the specified height map and scale vector.
     *
     * @param heightmap (not null, length&ge;4, length a perfect square,
     * unaffected)
     * @param scale (not null, no negative component, unaffected, default=1,1,1)
     */
    public HeightfieldCollisionShape(float[] heightmap, Vector3f scale) {
        Validate.nonEmpty(heightmap, "heightmap");
        assert heightmap.length >= 4 : heightmap.length;

        createCollisionHeightfield(heightmap, scale);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Count how many data points are in the heightfield.
     *
     * @return the count (&gt;0)
     */
    public int countMeshVertices() {
        int count = heightfieldData.length;

        assert count > 0 : count;
        return count;
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
        // bbuf not cloned
        // heightfieldData not cloned
        createShape();
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public HeightfieldCollisionShape jmeClone() {
        try {
            HeightfieldCollisionShape clone
                    = (HeightfieldCollisionShape) super.clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
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
        heightStickWidth = capsule.readInt("heightStickWidth", 0);
        heightStickLength = capsule.readInt("heightStickLength", 0);
        heightScale = capsule.readFloat("heightScale", 0f);
        minHeight = capsule.readFloat("minHeight", 0f);
        maxHeight = capsule.readFloat("maxHeight", 0f);
        upAxis = capsule.readInt("upAxis", PhysicsSpace.AXIS_Y);
        heightfieldData = capsule.readFloatArray("heightfieldData",
                new float[0]);
        flipQuadEdges = capsule.readBoolean("flipQuadEdges", false);
        createShape();
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
        capsule.write(heightStickWidth, "heightStickWidth", 0);
        capsule.write(heightStickLength, "heightStickLength", 0);
        capsule.write(heightScale, "heightScale", 0f);
        capsule.write(minHeight, "minHeight", 0f);
        capsule.write(maxHeight, "maxHeight", 0f);
        capsule.write(upAxis, "upAxis", PhysicsSpace.AXIS_Y);
        capsule.write(heightfieldData, "heightfieldData", new float[0]);
        capsule.write(flipQuadEdges, "flipQuadEdges", false);
    }
    // *************************************************************************
    // private methods

    private void createCollisionHeightfield(float[] heightmap,
            Vector3f worldScale) {
        assert heightmap != null;
        assert MyVector3f.isAllNonNegative(worldScale);

        scale.set(worldScale);
        heightfieldData = heightmap;

        float min = heightfieldData[0];
        float max = heightfieldData[0];
        // calculate min and max height
        for (int i = 0; i < heightfieldData.length; i++) {
            if (heightfieldData[i] < min) {
                min = heightfieldData[i];
            }
            if (heightfieldData[i] > max) {
                max = heightfieldData[i];
            }
        }
        // we need to center the terrain collision box at 0,0,0 for BulletPhysics. And to do that we need to set the
        // min and max height to be equal on either side of the y axis, otherwise it gets shifted and collision is incorrect.
        if (max < 0) {
            max = -min;
        } else {
            if (Math.abs(max) > Math.abs(min)) {
                min = -max;
            } else {
                max = -min;
            }
        }
        minHeight = min;
        maxHeight = max;

        heightStickWidth = (int) FastMath.sqrt(heightfieldData.length);
        assert heightStickWidth > 1 : heightStickWidth;
        heightStickLength = heightStickWidth;
        int elements = heightStickLength * heightStickWidth;
        assert elements == heightfieldData.length : heightfieldData.length;

        createShape();
    }

    /**
     * Instantiate the configured shape in Bullet.
     */
    private void createShape() {
        assert objectId == 0L;

        bbuf = BufferUtils.createByteBuffer(heightfieldData.length * 4);
        for (int i = 0; i < heightfieldData.length; i++) {
            float f = heightfieldData[i];
            bbuf.putFloat(f);
        }

        objectId = createShape(heightStickWidth, heightStickLength, bbuf,
                heightScale, minHeight, maxHeight, upAxis, flipQuadEdges);
        assert objectId != 0L;
        logger2.log(Level.FINE, "Created Shape {0}", Long.toHexString(objectId));

        setScale(scale);
        setMargin(margin);
    }

    native private long createShape(int heightStickWidth, int heightStickLength,
            ByteBuffer heightfieldData, float heightScale, float minHeight,
            float maxHeight, int upAxis, boolean flipQuadEdges);
}
