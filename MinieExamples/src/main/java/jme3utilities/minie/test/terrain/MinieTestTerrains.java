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
import com.jme3.terrain.geomipmap.TerrainQuad;
import com.jme3.terrain.heightmap.AbstractHeightMap;
import com.jme3.terrain.heightmap.HeightMap;
import com.jme3.terrain.heightmap.ImageBasedHeightMap;
import com.jme3.texture.Image;
import com.jme3.texture.Texture;
import java.util.logging.Logger;

/**
 * Generate some interesting terrains for use in MinieExamples.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class MinieTestTerrains {
    // *************************************************************************
    // constants and loggers

    /**
     * height array for a 3x3 heightfield
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
     * Load a simple height map from a texture asset.
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
}
