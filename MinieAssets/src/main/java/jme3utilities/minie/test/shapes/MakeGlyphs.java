/*
 Copyright (c) 2020-2023, Stephen Gold
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
package jme3utilities.minie.test.shapes;

import com.atr.jme.font.TrueTypeFont;
import com.atr.jme.font.asset.TrueTypeKeyMesh;
import com.atr.jme.font.asset.TrueTypeLoader;
import com.atr.jme.font.util.StringContainer;
import com.atr.jme.font.util.Style;
import com.jme3.asset.DesktopAssetManager;
import com.jme3.asset.plugins.ClasspathLocator;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.material.plugins.J3MLoader;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.mesh.IndexBuffer;
import com.jme3.system.NativeLibraryLoader;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MyMesh;
import jme3utilities.MySpatial;
import jme3utilities.MyString;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyVector3f;

/**
 * A console application to generate collision-shape assets for all 26 letters
 * of the English alphabet in upper case.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class MakeGlyphs {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(MakeGlyphs.class.getName());
    /**
     * filesystem path to the asset directory/folder for output
     */
    final private static String assetDirPath
            = "../MinieExamples/src/main/resources";
    // *************************************************************************
    // fields

    /**
     * list of vertices in a triangular prism
     */
    final private static List<Vector3f> prismVertices
            = new ArrayList<>(2 * MyMesh.vpt);
    /**
     * individual vertices in the above list
     */
    final private static Vector3f v0a = new Vector3f();
    final private static Vector3f v0b = new Vector3f();
    final private static Vector3f v1a = new Vector3f();
    final private static Vector3f v1b = new Vector3f();
    final private static Vector3f v2a = new Vector3f();
    final private static Vector3f v2b = new Vector3f();
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private MakeGlyphs() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the MakeGlyphs application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);

        // Mute the chatty loggers found in some imported packages.
        Heart.setLoggingLevels(Level.WARNING);

        // Log the working directory.
        String userDir = System.getProperty("user.dir");
        logger.log(Level.INFO, "working directory is {0}",
                MyString.quote(userDir));

        // Generate the collision shapes.
        makeAlphabet();
    }
    // *************************************************************************
    // private methods

    /**
     * Load ProFont from a classpath asset, using JME-TTF.
     *
     * @return a new font instance
     */
    @SuppressWarnings("unchecked")
    private static TrueTypeFont loadProFont() {
        DesktopAssetManager assetManager = new DesktopAssetManager();
        assetManager.registerLocator(null, ClasspathLocator.class);
        assetManager.registerLoader(J3MLoader.class, "j3m", "j3md");
        assetManager.registerLoader(TrueTypeLoader.class, "ttf");

        String fontAssetPath = "Interface/Fonts/ProFontWindows.ttf";
        Style style = Style.Plain;
        int pointSize = 9;
        TrueTypeKeyMesh assetKey
                = new TrueTypeKeyMesh(fontAssetPath, style, pointSize);
        TrueTypeFont result = (TrueTypeFont) assetManager.loadAsset(assetKey);

        return result;
    }

    /**
     * Generate collision shapes for all 26 letters of the English alphabet in
     * upper case.
     */
    private static void makeAlphabet() {
        // Initialize the vertex list.
        prismVertices.clear();
        prismVertices.add(v0a);
        prismVertices.add(v0b);
        prismVertices.add(v1a);
        prismVertices.add(v1b);
        prismVertices.add(v2a);
        prismVertices.add(v2b);

        // Generate the shapes.
        TrueTypeFont font = loadProFont();
        Transform transformA = new Transform();
        transformA.getTranslation().set(0f, 0f, 0.5f);
        Transform transformB = new Transform();
        transformB.getTranslation().set(0f, 0f, -0.5f);
        for (char letter = 'A'; letter <= 'Z'; ++letter) {
            makeGlyphShape(font, letter, transformA, transformB);
        }
        for (char letter = '0'; letter <= '9'; ++letter) {
            makeGlyphShape(font, letter, transformA, transformB);
        }
    }

    /**
     * Generate a collision shape for the specified character.
     *
     * @param font (not null)
     * @param character which character
     * @param transformA the first coordinate transform to apply to triangles
     * (not null, unaffected)
     * @param transformB the 2nd coordinate transform to apply to triangles (not
     * null, unaffected)
     */
    private static void makeGlyphShape(TrueTypeFont font, char character,
            Transform transformA, Transform transformB) {
        // Convert the specified character to a TrueTypeNode.
        String string = Character.toString(character);
        int kerning = 0;
        ColorRGBA color = null;
        StringContainer.Align hAlign = StringContainer.Align.Center;
        StringContainer.VAlign vAlign = StringContainer.VAlign.Center;
        Spatial ttNode = font.getText(string, kerning, color, hAlign, vAlign);

        // Access the generated mesh.
        List<Geometry> list = MySpatial.listGeometries(ttNode);
        assert list.size() == 1;
        Mesh mesh = list.get(0).getMesh();
        assert mesh.getMode() == Mesh.Mode.Triangles : mesh.getMode();

        CompoundCollisionShape compoundShape = new CompoundCollisionShape();
        /*
         * For each triangle in the mesh, add a triangular prism
         * to the compound shape.
         */
        IndexBuffer triangleIndices = mesh.getIndicesAsList();
        int numIndices = triangleIndices.size();
        FloatBuffer positions = mesh.getFloatBuffer(VertexBuffer.Type.Position);
        for (int startOff = 0; startOff < numIndices; startOff += MyMesh.vpt) {
            int ti0 = triangleIndices.get(startOff);
            int ti1 = triangleIndices.get(startOff + 1);
            int ti2 = triangleIndices.get(startOff + 2);

            MyBuffer.get(positions, MyVector3f.numAxes * ti0, v0a);
            MyBuffer.get(positions, MyVector3f.numAxes * ti1, v1a);
            MyBuffer.get(positions, MyVector3f.numAxes * ti2, v2a);

            transformB.transformVector(v0a, v0b);
            transformB.transformVector(v1a, v1b);
            transformB.transformVector(v2a, v2b);

            transformA.transformVector(v0a, v0a);
            transformA.transformVector(v1a, v1a);
            transformA.transformVector(v2a, v2a);

            CollisionShape prism = new HullCollisionShape(prismVertices);
            compoundShape.addChildShape(prism);
        }

        // Write the compound shape to a J3O file.
        String assetPath
                = String.format("CollisionShapes/glyphs/%s.j3o", string);
        String filePath = String.format("%s/%s", assetDirPath, assetPath);
        Heart.writeJ3O(filePath, compoundShape);
    }
}
