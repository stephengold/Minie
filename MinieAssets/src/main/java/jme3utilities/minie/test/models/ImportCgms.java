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
package jme3utilities.minie.test.models;

import com.jme3.anim.Armature;
import com.jme3.anim.Joint;
import com.jme3.anim.util.AnimMigrationUtils;
import com.jme3.app.SimpleApplication;
import com.jme3.asset.AssetInfo;
import com.jme3.asset.BlenderKey;
import com.jme3.asset.TextureKey;
import com.jme3.material.MatParam;
import com.jme3.material.Material;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.UserData;
import com.jme3.scene.plugins.blender.BlenderLoader;
import com.jme3.scene.plugins.blender.meshes.TemporalMesh;
import com.jme3.scene.plugins.ogre.MaterialLoader;
import com.jme3.scene.plugins.ogre.MeshLoader;
import com.jme3.shader.VarType;
import com.jme3.system.JmeContext;
import com.jme3.texture.Texture;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.imageio.ImageIO;
import jme3utilities.Heart;
import jme3utilities.MySkeleton;
import jme3utilities.MySpatial;
import jme3utilities.MyString;

/**
 * A headless SimpleApplication to import certain C-G models used in the
 * MinieExamples sub-project.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class ImportCgms extends SimpleApplication {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(ImportCgms.class.getName());
    /**
     * filesystem path to the destination asset root (the directory/folder for
     * output)
     */
    final private static String assetDirPath
            = "../MinieExamples/src/main/resources";
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the ImportCgms application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        // Mute the chatty loggers found in some imported packages.
        Heart.setLoggingLevels(Level.WARNING);

        // Instantiate the application.
        ImportCgms application = new ImportCgms();

        // Log the working directory.
        String userDir = System.getProperty("user.dir");
        logger.log(Level.INFO, "working directory is {0}",
                MyString.quote(userDir));

        // Import the C-G models.
        application.start(JmeContext.Type.Headless);
    }
    // *************************************************************************
    // SimpleApplication methods

    /**
     * Convert non-J3O format computer-graphics models to J3O format for faster
     * loading. Also write out the textures used by those models.
     */
    @Override
    public void simpleInitApp() {
        Logger.getLogger(MaterialLoader.class.getName()).setLevel(Level.SEVERE);
        Logger.getLogger(MeshLoader.class.getName()).setLevel(Level.SEVERE);
        Logger.getLogger(TemporalMesh.class.getName()).setLevel(Level.SEVERE);

        // Import the BaseMesh model from jme3-testdata-3.1.0-stable.jar:
        assetManager.registerLoader(BlenderLoader.class, "blend");
        BlenderKey blendKey = new BlenderKey("Blender/2.4x/BaseMesh_249.blend");
        Spatial baseMesh = assetManager.loadModel(blendKey);
        AnimMigrationUtils.migrate(baseMesh);

        // Ensure that every Joint has a name.
        List<Armature> list = MySkeleton.listArmatures(baseMesh, null);
        assert list.size() == 1 : list.size();
        Armature armature = list.get(0);
        int numJoints = armature.getJointCount();
        for (int jointIndex = 0; jointIndex < numJoints; ++jointIndex) {
            Joint joint = armature.getJoint(jointIndex);
            if (joint.getName().isEmpty()) {
                joint.setName("bone_" + jointIndex);
            }
        }
        writeToJ3O(baseMesh, "Models/BaseMesh/BaseMesh.j3o");
        /*
         * Import the Duck model (by Sony Computer Entertainment Inc.)
         * from src/main/resources:
         */
        Spatial duck = assetManager.loadModel("Models/Duck/Duck.gltf");
        writeToJ3O(duck, "Models/Duck/Duck.j3o");
        writeTextures(duck);

        // Import the Elephant model from jme3-testdata-3.1.0-stable.jar:
        Spatial elephant
                = assetManager.loadModel("Models/Elephant/Elephant.mesh.xml");
        writeToJ3O(elephant, "Models/Elephant/Elephant.j3o");
        writeTextures(elephant);
        /*
         * Import the Jaime model (by Rémy Bouquet)
         * from jme3-testdata-3.1.0-stable.jar:
         */
        Spatial jaime = assetManager.loadModel("Models/Jaime/Jaime.j3o");
        AnimMigrationUtils.migrate(jaime);
        writeToJ3O(jaime, "Models/Jaime/Jaime-new.j3o");
        /*
         * Import the MhGame model (by Stephen Gold)
         * from src/main/resources:
         */
        Spatial mhGame
                = assetManager.loadModel("Models/MhGame/MhGame.mesh.xml");
        writeToJ3O(mhGame, "Models/MhGame/MhGame.j3o");
        writeTextures(mhGame);

        // Import the MonkeyHead model from jme3-testdata-3.1.0-stable.jar:
        Spatial monkeyHead = assetManager.loadModel(
                "Models/MonkeyHead/MonkeyHead.mesh.xml");
        writeToJ3O(monkeyHead, "Models/MonkeyHead/MonkeyHead.j3o");
        writeTextures(monkeyHead);

        // Import the Ninja model from jme3-testdata-3.1.0-stable.jar:
        Spatial ninja = assetManager.loadModel("Models/Ninja/Ninja.mesh.xml");
        writeToJ3O(ninja, "Models/Ninja/Ninja.j3o");
        writeTextures(ninja);
        /*
         * Import the CandyDish model (by Stephen Gold)
         * from src/main/resources:
         */
        Spatial candyDish
                = assetManager.loadModel("Models/CandyDish/CandyDish.glb");
        writeToJ3O(candyDish, "Models/CandyDish/CandyDish.j3o");
        /*
         * Import the Oto model (by OtoTheCleaner)
         * from jme3-testdata-3.1.0-stable.jar:
         */
        Spatial oto = assetManager.loadModel("Models/Oto/Oto.mesh.xml");
        writeToJ3O(oto, "Models/Oto/Oto.j3o");
        writeTextures(oto);
        /*
         * Import the Sinbad model (by Zi Ye)
         * from jme3-testdata-3.1.0-stable.jar:
         */
        Node sinbad = (Node) assetManager.loadModel(
                "Models/Sinbad/Sinbad.mesh.xml");
        sinbad.getChild(4).setUserData(UserData.JME_PHYSICSIGNORE, true);
        writeToJ3O(sinbad, "Models/Sinbad/Sinbad.j3o");
        writeTextures(sinbad);

        Spatial sword = assetManager.loadModel("Models/Sinbad/Sword.mesh.xml");
        writeToJ3O(sword, "Models/Sinbad/Sword.j3o");
        writeTextures(sword);

        stop();
    }
    // *************************************************************************
    // private methods

    /**
     * Write the image of a texture to JPG or PNG file.
     *
     * @param key access the texture to be written (not null)
     */
    private void writeImage(TextureKey key) {
        String suffix = key.getExtension();
        String assetPath = key.getName();

        AssetInfo info = assetManager.locateAsset(key);
        InputStream stream = info.openStream();

        BufferedImage image;
        try {
            image = ImageIO.read(stream);
        } catch (IOException exception) {
            logger.log(Level.SEVERE, "failed to read {0}",
                    MyString.quote(assetPath));
            throw new RuntimeException(exception);
        }

        String writeFilePath = String.format("%s/%s", assetDirPath, assetPath);
        File file = new File(writeFilePath);

        // Create the parent folder.
        File parent = file.getParentFile();
        if (parent != null && !parent.exists()) {
            boolean success = parent.mkdirs();
            if (!success) {
                String message = "mkdirs() failed while saving "
                        + MyString.quote(writeFilePath);
                throw new RuntimeException(message);
            }
        }
        /*
         * Write the texture's BufferedImage in the
         * format specified by its suffix.
         */
        try {
            ImageIO.write(image, suffix, file);
        } catch (IOException exception) {
            throw new RuntimeException(exception);
        }
        logger.log(Level.INFO, "wrote file {0}", MyString.quote(writeFilePath));
    }

    /**
     * Write the image of each 2-D texture used in the specified model.
     *
     * @param model the scene-graph subtree to analyze (may be null)
     */
    private void writeTextures(Spatial model) {
        // Collect all unique 2-D textures used in the model.
        Set<TextureKey> textureKeys = new HashSet<>(20);
        for (Material materials : MySpatial.listMaterials(model, null)) {
            for (MatParam matParam : materials.getParams()) {
                if (matParam.getVarType() == VarType.Texture2D) {
                    Texture texture = (Texture) matParam.getValue();
                    TextureKey key = (TextureKey) texture.getKey();
                    textureKeys.add(key);
                }
            }
        }

        // Write each texture to a JPG file.
        for (TextureKey textureKey : textureKeys) {
            writeImage(textureKey);
        }
    }

    /**
     * Write the specified model to a J3O file.
     *
     * @param model the scene-graph subtree to write (not null)
     * @param writeAssetPath the asset-path portion of the filename (not null,
     * not empty, should end in ".j3o")
     */
    private static void writeToJ3O(Spatial model, String writeAssetPath) {
        String writeFilePath
                = String.format("%s/%s", assetDirPath, writeAssetPath);
        Heart.writeJ3O(writeFilePath, model);
    }
}
