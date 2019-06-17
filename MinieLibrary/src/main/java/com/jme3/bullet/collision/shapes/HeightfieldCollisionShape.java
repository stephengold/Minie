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
import java.nio.FloatBuffer;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A CollisionShape for terrain defined by a matrix of height values, based on
 * Bullet's btHeightfieldTerrainShape. Should be more efficient than an
 * equivalent MeshCollisionShape.
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
    /**
     * local copy of {@link com.jme3.math.Vector3f#UNIT_XYZ}
     */
    final private static Vector3f scaleIdentity = new Vector3f(1f, 1f, 1f);
    // *************************************************************************
    // fields TODO re-order

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
     * copy of the height-axis index (0&rarr;X, 1&rarr;Y, 2&rarr;Z, default=1)
     */
    private int upAxis = PhysicsSpace.AXIS_Y;
    /**
     * TODO description (default=false)
     */
    private boolean flipQuadEdges = false;
    /**
     * true&rarr;left-hand winding of triangles (default=false)
     */
    boolean flipTriangleWinding = false;
    /**
     * true&rarr;diagonals alternate on both horizontal axes (default=false)
     */
    boolean useDiamond = false;
    /**
     * true&rarr;diagonals alternate on one horizontal axis (default=false)
     */
    boolean useZigzag = false;
    /**
     * buffer for passing height data to Bullet
     * <p>
     * A Java reference must persist after createShape() completes, or else the
     * buffer might get garbage collected.
     */
    private FloatBuffer bbuf;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public HeightfieldCollisionShape() {
    }

    /**
     * Instantiate a new, un-scaled shape for the specified height map.
     *
     * @param heightmap (not null, length&ge;4, length a perfect square,
     * unaffected)
     */
    public HeightfieldCollisionShape(float[] heightmap) {
        Validate.nonEmpty(heightmap, "heightmap");
        assert heightmap.length >= 4 : heightmap.length;

        createCollisionHeightfield(heightmap, scaleIdentity);
    }

    /**
     * Instantiate a square shape for the specified height map and scale vector.
     *
     * @param heightmap (not null, length&ge;4, length a perfect square,
     * unaffected)
     * @param scale the desired scaling factor for each local axis (not null, no
     * negative component, unaffected, default=(1,1,1))
     */
    public HeightfieldCollisionShape(float[] heightmap, Vector3f scale) {
        Validate.nonEmpty(heightmap, "heightmap");
        assert heightmap.length >= 4 : heightmap.length;
        Validate.nonNegative(scale, "scale");

        createCollisionHeightfield(heightmap, scale);
    }

    /**
     * Instantiate a rectangular shape for the specified parameters.
     *
     * @param stickLength the number of rows in the heightfield (&gt;1)
     * @param stickWidth number of columns in the heightfield (&gt;1)
     * @param heightmap (not null, length = stickLength*stickWidth, unaffected)
     * @param scale the desired scaling factor for each local axis (not null, no
     * negative component, unaffected, default=(1,1,1))
     * @param upAxis the height-axis index (0&rarr;X, 1&rarr;Y, 2&rarr;Z)
     * @param flipQuadEdges TODO description
     * @param flipTriangleWinding true&rarr;left-hand winding of triangles
     * @param useDiamond true&rarr;diagonals alternate on both horizontal axes
     * @param useZigzag true&rarr;diagonals alternate on one horizontal axis
     */
    public HeightfieldCollisionShape(int stickLength, int stickWidth,
            float[] heightmap, Vector3f scale, int upAxis,
            boolean flipQuadEdges, boolean flipTriangleWinding,
            boolean useDiamond, boolean useZigzag) {
        Validate.inRange(stickLength, "stick length", 2, Integer.MAX_VALUE);
        Validate.inRange(stickWidth, "stick width", 2, Integer.MAX_VALUE);
        Validate.nonEmpty(heightmap, "heightmap");
        assert heightmap.length >= stickLength * stickWidth : heightmap.length;
        Validate.nonNegative(scale, "scale");
        Validate.inRange(upAxis, "up axis", 0, 2);

        heightStickLength = stickLength;
        heightStickWidth = stickWidth;
        heightfieldData = heightmap;
        this.scale.set(scale);
        this.upAxis = upAxis;
        this.flipQuadEdges = flipQuadEdges;
        this.flipTriangleWinding = flipTriangleWinding;
        this.useDiamond = useDiamond;
        this.useZigzag = useZigzag;

        calculateMinAndMax();
        createShape();
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
        // bbuf not cloned
        // heightfieldData not cloned
        createShape();
    }

    /**
     * Finalize this shape just before it is destroyed. Should be invoked only
     * by a subclass or by the garbage collector.
     *
     * @throws Throwable ignored by the garbage collector
     */
    @Override
    protected void finalize() throws Throwable {
        long nativeId = getObjectId();
        //finalizeNative(nativeId);
        super.finalize();
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

        heightStickWidth = capsule.readInt("heightStickWidth", 0);
        heightStickLength = capsule.readInt("heightStickLength", 0);
        heightScale = capsule.readFloat("heightScale", 0f);
        minHeight = capsule.readFloat("minHeight", 0f);
        maxHeight = capsule.readFloat("maxHeight", 0f);
        upAxis = capsule.readInt("upAxis", PhysicsSpace.AXIS_Y);
        heightfieldData = capsule.readFloatArray("heightfieldData",
                new float[0]);
        flipQuadEdges = capsule.readBoolean("flipQuadEdges", false);
        flipTriangleWinding = capsule.readBoolean("flipTriangleWinding", false);
        useDiamond = capsule.readBoolean("useDiamond", false);
        useZigzag = capsule.readBoolean("useZigzag", false);

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

        capsule.write(heightStickWidth, "heightStickWidth", 0);
        capsule.write(heightStickLength, "heightStickLength", 0);
        capsule.write(heightScale, "heightScale", 0f);
        capsule.write(minHeight, "minHeight", 0f);
        capsule.write(maxHeight, "maxHeight", 0f);
        capsule.write(upAxis, "upAxis", PhysicsSpace.AXIS_Y);
        capsule.write(heightfieldData, "heightfieldData", new float[0]);
        capsule.write(flipQuadEdges, "flipQuadEdges", false);
        capsule.write(flipTriangleWinding, "flipTriangleWinding", false);
        capsule.write(useDiamond, "useDiamond", false);
        capsule.write(useZigzag, "useZigzag", false);
    }
    // *************************************************************************
    // private methods

    /**
     * Calculate min and max heights for the heightfield data.
     */
    private void calculateMinAndMax() {
        int elements = heightStickLength * heightStickWidth;
        assert elements == heightfieldData.length : heightfieldData.length;

        float min = heightfieldData[0];
        float max = heightfieldData[0];
        /*
         * Find the min and max heights in the data.
         */
        for (float height : heightfieldData) {
            if (height < min) {
                min = height;
            }
            if (height > max) {
                max = height;
            }
        }
        /*
         * Center the terrain's bounding box at y=0 by setting the
         * min and max height to have equal magnitudes and opposite signs.
         * Otherwise, the collision shape won't match the rendered heights.
         */
        if (max < 0) {
            max = -min;
        } else if (Math.abs(max) > Math.abs(min)) {
            min = -max;
        } else {
            max = -min;
        }
        minHeight = min;
        maxHeight = max;
    }

    /**
     * Instantiate a square btHeightfieldTerrainShape.
     */
    private void createCollisionHeightfield(float[] heightmap,
            Vector3f worldScale) {
        scale.set(worldScale);

        heightfieldData = heightmap;
        heightStickWidth = (int) FastMath.sqrt(heightfieldData.length);
        assert heightStickWidth > 1 : heightStickWidth;

        heightStickLength = heightStickWidth;

        calculateMinAndMax();
        createShape();
    }

    /**
     * Instantiate the configured btHeightfieldTerrainShape.
     */
    private void createShape() {
        assert objectId == 0L;

        bbuf = BufferUtils.createFloatBuffer(heightfieldData.length);
        for (float height : heightfieldData) {
            bbuf.put(height);
        }

        objectId = createShape(heightStickWidth, heightStickLength, bbuf,
                heightScale, minHeight, maxHeight, upAxis, flipQuadEdges);
        //       objectId = createShape2(heightStickWidth, heightStickLength, bbuf,
        //               heightScale, minHeight, maxHeight, upAxis, flipQuadEdges,
        //               flipTriangleWinding, useDiamond, useZigzag);
        assert objectId != 0L;
        logger2.log(Level.FINE, "Created Shape {0}",
                Long.toHexString(objectId));

        setScale(scale);
        setMargin(margin);
    }
    // *************************************************************************
    // native private methods

    native private long createShape(int stickWidth, int stickLength,
            FloatBuffer heightfieldData, float heightScale, float minHeight,
            float maxHeight, int upAxis, boolean flipQuadEdges);

    native private long createShape2(int stickWidth, int stickLength,
            FloatBuffer heightfieldData, float heightScale, float minHeight,
            float maxHeight, int upAxis, boolean flipQuadEdges,
            boolean flipTriangleWinding, boolean useDiamond, boolean useZigzag);

    native private void finalizeNative(long shapeId);
}
