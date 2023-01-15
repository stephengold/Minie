/*
 Copyright (c) 2019-2023, Stephen Gold
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

import com.jme3.asset.DesktopAssetManager;
import com.jme3.asset.plugins.ClasspathLocator;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.util.CollisionShapeFactory;
import com.jme3.export.Savable;
import com.jme3.material.plugins.J3MLoader;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.plugins.MTLLoader;
import com.jme3.scene.plugins.OBJLoader;
import com.jme3.system.NativeLibraryLoader;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MySpatial;
import jme3utilities.MyString;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;
import vhacd.VHACDParameters;

/**
 * A console application to generate the collision-shape assets "teapot.j3o" and
 * "teapotGi.j3o".
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class MakeTeapot {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(MakeTeapot.class.getName());
    /**
     * filesystem path to the asset directory/folder for output
     */
    final private static String assetDirPath
            = "../MinieExamples/src/main/resources";
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private MakeTeapot() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the MakeTeapot application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);

        // Mute the chatty loggers found in some imported packages.
        Heart.setLoggingLevels(Level.WARNING);
        Logger.getLogger(OBJLoader.class.getName()).setLevel(Level.SEVERE);

        // Log the working directory.
        String userDir = System.getProperty("user.dir");
        logger.log(Level.INFO, "working directory is {0}",
                MyString.quote(userDir));

        // Generate the collision shape.
        makeTeapot();
    }
    // *************************************************************************
    // private methods

    /**
     * Generate a collision shape for a teapot.
     */
    private static void makeTeapot() {
        DesktopAssetManager assetManager = new DesktopAssetManager();
        assetManager.registerLoader(OBJLoader.class, "obj");
        assetManager.registerLoader(MTLLoader.class, "mtl");
        assetManager.registerLoader(J3MLoader.class, "j3m", "j3md");
        assetManager.registerLocator(null, ClasspathLocator.class);
        /*
         * Import the Utah Teapot model (by Martin Newell and Jim Blinn)
         * from jme3-testdata-3.1.0-stable.jar.
         */
        String objAssetPath = "Models/Teapot/Teapot.obj";
        Spatial geom = assetManager.loadModel(objAssetPath);

        // Translate and uniformly scale the model to fit inside a 2x2x2 cube.
        geom.setLocalTransform(Transform.IDENTITY);
        Vector3f[] minMax = MySpatial.findMinMaxCoords(geom);
        Vector3f center = MyVector3f.midpoint(minMax[0], minMax[1], null);
        Vector3f offset = center.negate();
        geom.setLocalTranslation(offset);

        Node parent = new Node();
        parent.attachChild(geom);
        Vector3f extents = minMax[1].subtract(minMax[0]);
        float radius = MyMath.max(extents.x, extents.y, extents.z) / 2f;
        parent.setLocalScale(1f / radius);

        Node cgmRoot = new Node();
        cgmRoot.attachChild(parent);

        // Using V-HACD, generate a CollisionShape to approximate the Mesh.
        VHACDParameters parameters = new VHACDParameters();
        CompoundCollisionShape shape = ShapeUtils.createVhacdShape(
                cgmRoot, parameters, "MakeTeapot");

        // Write the shape to the asset file.
        String assetPath = "CollisionShapes/teapot.j3o";
        String writeFilePath = String.format("%s/%s", assetDirPath, assetPath);
        Heart.writeJ3O(writeFilePath, shape);

        // Generate a CollisionShape using GImpact and write to asset file.
        Savable giShape = CollisionShapeFactory.createGImpactShape(cgmRoot);
        assetPath = "CollisionShapes/teapotGi.j3o";
        writeFilePath = String.format("%s/%s", assetDirPath, assetPath);
        Heart.writeJ3O(writeFilePath, giShape);
    }
}
