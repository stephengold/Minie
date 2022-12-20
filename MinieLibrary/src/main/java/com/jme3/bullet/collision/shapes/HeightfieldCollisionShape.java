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
import com.jme3.bullet.collision.shapes.infos.IndexedMesh;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.terrain.Terrain;
import com.jme3.terrain.heightmap.HeightMap;
import com.jme3.util.BufferUtils;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.nio.FloatBuffer;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A collision shape for terrain defined by a matrix of height values, based on
 * Bullet's {@code btHeightfieldTerrainShape}. Should be more efficient than an
 * equivalent MeshCollisionShape. Not for use in dynamic bodies. Collisions
 * between HeightfieldCollisionShape, MeshCollisionShape, and
 * PlaneCollisionShape objects are never detected.
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
     * field names for serialization
     */
    final private static String tagFlipQuadEdges = "flipQuadEdges";
    final private static String tagFlipTriangleWinding = "flipTriangleWinding";
    final private static String tagHeightfieldData = "heightfieldData";
    final private static String tagHeightScale = "heightScale";
    final private static String tagHeightStickLength = "heightStickLength";
    final private static String tagHeightStickWidth = "heightStickWidth";
    final private static String tagMaxHeight = "maxHeight";
    final private static String tagMinHeight = "minHeight";
    final private static String tagUpAxis = "upAxis";
    final private static String tagUseDiamond = "useDiamond";
    final private static String tagUseZigzag = "useZigzag";
    /**
     * local copy of {@link com.jme3.math.Vector3f#UNIT_XYZ}
     */
    final private static Vector3f scaleIdentity = new Vector3f(1f, 1f, 1f);
    // *************************************************************************
    // fields

    /**
     * reverse the direction of the first diagonal
     */
    private boolean flipQuadEdges = true;
    /**
     * true&rarr;left-hand winding of triangles
     */
    private boolean flipTriangleWinding = false;
    /**
     * true&rarr;diagonals alternate on both horizontal axes
     */
    private boolean useDiamond = false;
    /**
     * true&rarr;diagonals alternate on one horizontal axis
     */
    private boolean useZigzag = false;
    /**
     * scale factor for Bullet to apply to the heightfield
     */
    private float heightScale = 1f;
    /**
     * highest sample in the heightfield or -minHeight, whichever is higher
     */
    private float maxHeight;
    /**
     * lowest sample in the heightfield or -maxHeight, whichever is lower
     */
    private float minHeight;
    /**
     * array of heightfield samples
     */
    private float[] heightfieldData;
    /**
     * direct buffer for passing height data to Bullet
     * <p>
     * A Java reference must persist after createShape() completes, or else the
     * buffer might get garbage collected.
     */
    private FloatBuffer directBuffer;
    /**
     * copy of number of columns in the heightfield (&gt;1)
     */
    private int heightStickLength;
    /**
     * copy of number of rows in the heightfield (&gt;1)
     */
    private int heightStickWidth;
    /**
     * copy of the height-axis index (0&rarr;X, 1&rarr;Y, 2&rarr;Z)
     */
    private int upAxis = PhysicsSpace.AXIS_Y;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected HeightfieldCollisionShape() {
    }

    /**
     * Instantiate a square shape for the specified height map.
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
     * Instantiate a square shape for the specified HeightMap. If the HeightMap
     * isn't populated, invoke its load() method.
     *
     * @param heightMap (not null, size &ge; 2)
     */
    public HeightfieldCollisionShape(HeightMap heightMap) {
        float[] array = heightMap.getHeightMap();
        if (array == null) { // not populated
            boolean success = heightMap.load();
            assert success;
            array = heightMap.getHeightMap();
            assert array != null;
        }
        assert array.length >= 4 : array.length;

        createCollisionHeightfield(array, scaleIdentity);
    }

    /**
     * Instantiate a square shape for the specified height map and scale vector.
     *
     * @param heightmap (not null, length&ge;4, length a perfect square,
     * unaffected)
     * @param scale the desired scale factor for each local axis (not null, no
     * negative component, unaffected, default=(1,1,1))
     */
    public HeightfieldCollisionShape(float[] heightmap, Vector3f scale) {
        Validate.nonEmpty(heightmap, "heightmap");
        assert heightmap.length >= 4 : heightmap.length;
        Validate.nonNegative(scale, "scale");

        createCollisionHeightfield(heightmap, scale);
    }

    /**
     * Instantiate a square shape for the specified terrain and scale vector.
     *
     * @param terrain (not null, size &ge;2, unaffected)
     * @param scale the desired scale factor for each local axis (not null, no
     * negative component, unaffected, default=(1,1,1))
     */
    public HeightfieldCollisionShape(Terrain terrain, Vector3f scale) {
        Validate.nonNegative(scale, "scale");
        Validate.inRange(terrain.getTerrainSize(), "terrain size", 2,
                Integer.MAX_VALUE);

        float[] heightmap = terrain.getHeightMap();
        assert heightmap.length >= 4 : heightmap.length;
        createCollisionHeightfield(heightmap, scale);
    }

    /**
     * Instantiate a rectangular shape for the specified parameters.
     *
     * @param stickLength the number of rows in the heightfield (&gt;1)
     * @param stickWidth number of columns in the heightfield (&gt;1)
     * @param heightmap (not null, length&ge;stickLength*stickWidth, unaffected)
     * @param scale the desired scale factor for each local axis (not null, no
     * negative component, unaffected, default=(1,1,1))
     * @param upAxis the height-axis index (0&rarr;X, 1&rarr;Y, 2&rarr;Z,
     * default=1)
     * @param flipQuadEdges true&rarr;reverse the direction of the first
     * diagonal (default=true)
     * @param flipTriangleWinding true&rarr;left-hand winding of triangles
     * (default=false)
     * @param useDiamond true&rarr;diagonals alternate on both horizontal axes
     * (default=false)
     * @param useZigzag true&rarr;diagonals alternate on one horizontal axis
     * (default=false)
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
        Validate.axisIndex(upAxis, "up axis");

        this.heightStickLength = stickLength;
        this.heightStickWidth = stickWidth;
        this.heightfieldData = heightmap.clone();
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
        // directBuffer and heightfieldData are never cloned.
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

        this.heightStickWidth = capsule.readInt(tagHeightStickWidth, 0);
        this.heightStickLength = capsule.readInt(tagHeightStickLength, 0);
        this.heightScale = capsule.readFloat(tagHeightScale, 0f);
        this.minHeight = capsule.readFloat(tagMinHeight, 0f);
        this.maxHeight = capsule.readFloat(tagMaxHeight, 0f);
        this.upAxis = capsule.readInt(tagUpAxis, PhysicsSpace.AXIS_Y);
        this.heightfieldData
                = capsule.readFloatArray(tagHeightfieldData, new float[0]);
        this.flipQuadEdges = capsule.readBoolean(tagFlipQuadEdges, true);
        this.flipTriangleWinding
                = capsule.readBoolean(tagFlipTriangleWinding, false);
        this.useDiamond = capsule.readBoolean(tagUseDiamond, false);
        this.useZigzag = capsule.readBoolean(tagUseZigzag, false);

        createShape();
    }

    /**
     * Approximate this shape with a splittable shape.
     *
     * @return a new splittable shape
     */
    @Override
    public CollisionShape toSplittableShape() {
        // Generate debug triangles.
        FloatBuffer buffer = DebugShapeFactory
                .getDebugTriangles(this, DebugShapeFactory.lowResolution);
        IndexedMesh nativeMesh = new IndexedMesh(buffer);
        MeshCollisionShape result = new MeshCollisionShape(true, nativeMesh);

        return result;
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

        capsule.write(heightStickWidth, tagHeightStickWidth, 0);
        capsule.write(heightStickLength, tagHeightStickLength, 0);
        capsule.write(heightScale, tagHeightScale, 0f);
        capsule.write(minHeight, tagMinHeight, 0f);
        capsule.write(maxHeight, tagMaxHeight, 0f);
        capsule.write(upAxis, tagUpAxis, PhysicsSpace.AXIS_Y);
        capsule.write(heightfieldData, tagHeightfieldData, new float[0]);
        capsule.write(flipQuadEdges, tagFlipQuadEdges, true);
        capsule.write(flipTriangleWinding, tagFlipTriangleWinding, false);
        capsule.write(useDiamond, tagUseDiamond, false);
        capsule.write(useZigzag, tagUseZigzag, false);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Calculate min and max heights for the heightfield data.
     */
    private void calculateMinAndMax() {
        int elements = heightStickLength * heightStickWidth;
        assert elements == heightfieldData.length : heightfieldData.length;

        float min = heightfieldData[0];
        float max = heightfieldData[0];

        // Find the min and max heights in the data.
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
        this.minHeight = min;
        this.maxHeight = max;
    }

    /**
     * Instantiate a square {@code btHeightfieldTerrainShape}.
     *
     * @param heightmap (not null, length&ge;4, length a perfect square,
     * unaffected)
     * @param worldScale the desired scale factor for each local axis (not null,
     * no negative component, unaffected)
     */
    private void
            createCollisionHeightfield(float[] heightmap, Vector3f worldScale) {
        scale.set(worldScale);

        this.heightfieldData = heightmap.clone();
        this.heightStickWidth = (int) FastMath.sqrt(heightfieldData.length);
        assert heightStickWidth > 1 : heightStickWidth;

        this.heightStickLength = heightStickWidth;

        calculateMinAndMax();
        createShape();
    }

    /**
     * Instantiate the configured {@code btHeightfieldTerrainShape}.
     */
    private void createShape() {
        this.directBuffer
                = BufferUtils.createFloatBuffer(heightfieldData.length);
        for (float height : heightfieldData) {
            if (!Float.isFinite(height)) {
                throw new IllegalArgumentException("illegal height: " + height);
            }
            directBuffer.put(height);
        }

        long shapeId = createShape2(heightStickWidth, heightStickLength,
                directBuffer, heightScale, minHeight, maxHeight, upAxis,
                flipQuadEdges, flipTriangleWinding, useDiamond, useZigzag);
        setNativeId(shapeId);

        setContactFilterEnabled(enableContactFilter);
        setScale(scale);
        setMargin(margin);
    }

    /**
     * Free the identified tracked native object. Invoked by reflection.
     *
     * @param shapeId the native identifier (not zero)
     */
    private static void freeNativeObject(long shapeId) {
        assert shapeId != 0L;
        finalizeNative(shapeId);
    }
    // *************************************************************************
    // native private methods

    native private static long createShape2(int stickWidth, int stickLength,
            FloatBuffer heightfieldData, float heightScale, float minHeight,
            float maxHeight, int upAxis, boolean flipQuadEdges,
            boolean flipTriangleWinding, boolean useDiamond, boolean useZigzag);

    native private static void finalizeNative(long shapeId);
}
