/*
 Copyright (c) 2018, Stephen Gold
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
package jme3utilities.minie.test;

import com.jme3.asset.AssetManager;
import com.jme3.export.JmeImporter;
import com.jme3.material.MatParam;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.scene.Geometry;
import com.jme3.texture.Texture;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.MyAsset;
import jme3utilities.Validate;
import jme3utilities.mesh.PointMesh;

/**
 * Visualize a single location in space. TODO move to Debug library
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class PointVisualizer extends Geometry {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(PointVisualizer.class.getName());
    // *************************************************************************
    // fields

    private AssetManager assetManager;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil. Do not invoke
     * directly!
     */
    public PointVisualizer() {
        assetManager = null;
    }

    /**
     * Instantiate an enabled visualizer with the specified size and color and
     * the named shape.
     *
     * @param assetManager for loading assets (not null)
     * @param size the desired size (in pixels, &gt;0)
     * @param color the desired color (unaffected) or null for the default
     * @param shapeName the name of a sprite texture (either "ring", "saltire",
     * "solid circle", or null for a solid square)
     */
    public PointVisualizer(AssetManager assetManager, int size,
            ColorRGBA color, String shapeName) {
        super(shapeName, new PointMesh());
        Validate.nonNull(assetManager, "asset manager");
        Validate.positive(size, "size");

        this.assetManager = assetManager;

        String matDefPath = "MatDefs/wireframe/multicolor2.j3md";
        Material mat = new Material(assetManager, matDefPath);
        setMaterial(mat);

        material.setFloat("PointSize", (float) size);

        if (color != null) {
            material.setColor("Color", color.clone());
        }
        RenderState renderState = material.getAdditionalRenderState();
        renderState.setBlendMode(RenderState.BlendMode.Alpha);
        renderState.setDepthTest(false);

        if (shapeName != null) {
            String shapePath
                    = String.format("Textures/shapes/%s.png", shapeName);
            Texture texture = MyAsset.loadTexture(assetManager, shapePath);
            material.setTexture("PointShape", texture);
        }
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the color.
     *
     * @param storeResult (modified if not null)
     * @return the color (either storeResult or a new instance)
     */
    public ColorRGBA copyColor(ColorRGBA storeResult) {
        ColorRGBA result
                = (storeResult == null) ? new ColorRGBA() : storeResult;
        MatParam parameter = material.getParam("Color");
        Object value = parameter.getValue();
        result.set((ColorRGBA) value);

        return result;
    }

    /**
     * Test whether this visualizer is enabled.
     *
     * @return true if enabled, otherwise false
     */
    public boolean isEnabled() {
        boolean isEnabled
                = (cullHint == CullHint.Never || cullHint == CullHint.Dynamic);
        return isEnabled;
    }

    /**
     * Alter the color.
     *
     * @param desiredColor the desired color (not null, unaffected)
     */
    public void setColor(ColorRGBA desiredColor) {
        material.setColor("Color", desiredColor.clone());
    }

    /**
     * Enable or disable this visualizer.
     *
     * @param enable true &rarr; enable, false &rarr; disable
     */
    public void setEnabled(boolean enable) {
        boolean wasEnabled = isEnabled();
        if (wasEnabled && !enable) {
            cullHint = CullHint.Always;
        } else if (enable && !wasEnabled) {
            cullHint = CullHint.Never;
        }
    }

    /**
     * Alter the shape.
     *
     * @param shapeName the name of a sprite texture (either "ring", "saltire",
     * or "solid circle")
     */
    public void setShape(String shapeName) {
        Validate.nonEmpty(shapeName, "shape name");

        String shapePath
                = String.format("Textures/shapes/%s.png", shapeName);
        Texture texture = MyAsset.loadTexture(assetManager, shapePath);
        setShape(texture);
    }

    /**
     * Alter the shape.
     *
     * @param desiredTexture the desired sprite texture (not null, alias
     * created)
     */
    public void setShape(Texture desiredTexture) {
        Validate.nonNull(desiredTexture, "shape texture");
        material.setTexture("PointShape", desiredTexture);
    }

    /**
     * Alter the size.
     *
     * @param desiredSize the desired size (in pixels, &gt;0)
     */
    public void setSize(int desiredSize) {
        Validate.positive(desiredSize, "size");
        material.setFloat("PointSize", (float) desiredSize);
    }

    /**
     * Read the size.
     *
     * @return the size (in pixels, &gt;0)
     */
    public int size() {
        MatParam parameter = material.getParam("PointSize");
        Object value = parameter.getValue();
        Float floatValue = (Float) value;
        int result = Math.round(floatValue);

        return result;
    }
    // *************************************************************************
    // Saveable methods

    /**
     * De-serialize this visualizer, for example when loading from a J3O file.
     *
     * @param im importer (not null)
     * @throws IOException from superclass
     */
    @Override
    public void read(JmeImporter im) throws IOException {
        super.read(im);
        assetManager = im.getAssetManager();
    }
}
