/*
 Copyright (c) 2020, Stephen Gold
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package jme3utilities.minie.test.terrain;

import com.jme3.asset.AssetManager;
import com.jme3.asset.TextureKey;
import com.jme3.math.FastMath;
import com.jme3.terrain.geomipmap.TerrainQuad;
import com.jme3.terrain.heightmap.AbstractHeightMap;
import com.jme3.terrain.heightmap.HeightMap;
import com.jme3.terrain.heightmap.ImageBasedHeightMap;
import com.jme3.texture.Image;
import com.jme3.texture.Texture;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;

/**
 * Generate some interesting height arrays and terrains for use in
 * MinieExamples.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class MinieTestTerrains {
    // *************************************************************************
    // constants and loggers

    /**
     * 3x3 height array
     */
    final private static float[] nineHeights = new float[]{
        1f, 0f, 1f,
        0f, 0.5f, 0f,
        1f, 0f, 1f
    };
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(MinieTestTerrains.class.getName());
    // *************************************************************************
    // fields

    /**
     * 513x513 quad
     */
    public static TerrainQuad largeQuad;
    /**
     * 3x3 quad
     */
    public static TerrainQuad smallQuad;
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private MinieTestTerrains() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Generate a square height array for a bed of nails.
     *
     * @param size the desired size (in rows or columns, &ge;2)
     * @return a new array
     */
    public static float[] bedOfNailsArray(int size) {
        Validate.inRange(size, "size", 2, Integer.MAX_VALUE);

        float[] result = new float[size * size];
        for (int rowIndex = 0; rowIndex < size; ++rowIndex) {
            for (int columnIndex = 0; columnIndex < size; ++columnIndex) {
                boolean nail = (MyMath.modulo(rowIndex + columnIndex, 8) == 3
                        && MyMath.modulo(rowIndex - columnIndex, 8) == 1);

                int floatIndex = size * rowIndex + columnIndex;
                result[floatIndex] = nail ? 1f : 0f;
            }
        }

        return result;
    }

    /**
     * Generate a square height array for a dimpled surface.
     *
     * @param size the desired size (in rows or columns, &ge;2)
     * @return a new array
     */
    public static float[] dimplesArray(int size) {
        Validate.inRange(size, "size", 2, Integer.MAX_VALUE);

        float dimpleDepth = 3f;
        float sphereRadius = 20f;
        int halfSpacing = 16;

        float sphereR2 = sphereRadius * sphereRadius;
        float hThreshold = sphereRadius - dimpleDepth;
        float[] result = new float[size * size];
        int spacing = 2 * halfSpacing; // between dimples

        for (int rowIndex = 0; rowIndex < size; ++rowIndex) {
            float x = MyMath.modulo(rowIndex, spacing) - halfSpacing;
            for (int columnIndex = 0; columnIndex < size; ++columnIndex) {
                float y = MyMath.modulo(columnIndex, spacing) - halfSpacing;
                float xy2 = x * x + y * y;

                float height = 0f;
                if (xy2 < sphereR2) {
                    float h = FastMath.sqrt(sphereR2 - xy2);
                    if (h > hThreshold) {
                        height = hThreshold - h;
                    }
                }

                int floatIndex = size * rowIndex + columnIndex;
                result[floatIndex] = height;
            }
        }

        return result;
    }

    /**
     * Initialize the terrain quads during startup.
     *
     * @param assetManager (not null)
     */
    public static void initialize(AssetManager assetManager) {
        HeightMap heightMap = loadHeightMap(assetManager);
        int patchSize = 33; // in pixels
        int terrainDiameter = heightMap.getSize(); // in pixels
        int mapSize = terrainDiameter + 1; // number of samples on a side
        float[] heightArray = heightMap.getHeightMap();
        largeQuad = new TerrainQuad("large terrain", patchSize, mapSize,
                heightArray);

        patchSize = 3;
        mapSize = 3;
        smallQuad = new TerrainQuad("small terrain", patchSize, mapSize,
                nineHeights);
    }

    /**
     * Load a 513x513 height map from a texture asset.
     *
     * @param assetManager (not null)
     * @return a new instance
     */
    public static HeightMap loadHeightMap(AssetManager assetManager) {
        boolean flipY = false;
        TextureKey key = new TextureKey(
                "Textures/BumpMapTest/Simple_height.png", flipY);
        Texture texture = assetManager.loadTexture(key);
        Image image = texture.getImage();

        float heightScale = 1f;
        AbstractHeightMap result = new ImageBasedHeightMap(image, heightScale);
        result.load();

        return result;
    }

    /**
     * Generate a square height array for a quadratic surface of revolution.
     *
     * @param size the desired size (in rows or columns, &ge;2)
     * @return a new array
     */
    public static float[] quadraticArray(int size) {
        Validate.inRange(size, "size", 2, Integer.MAX_VALUE);

        float halfNm1 = (size - 1) / 2f;
        float[] result = new float[size * size];
        for (int rowIndex = 0; rowIndex < size; ++rowIndex) {
            float x = -1f + rowIndex / halfNm1; // -1 .. +1
            for (int columnIndex = 0; columnIndex < size; ++columnIndex) {
                float y = -1f + columnIndex / halfNm1; // -1 .. +1
                float r = MyMath.hypotenuse(x, y);
                float height = -0.4f + (r - 0.8f) * (r - 0.8f);

                int floatIndex = size * rowIndex + columnIndex;
                result[floatIndex] = height;
            }
        }

        return result;
    }
}
