/*
 Copyright (c) 2019, Stephen Gold
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

import com.jme3.asset.AssetInfo;
import com.jme3.asset.TextureKey;
import com.jme3.export.JmeExporter;
import com.jme3.export.binary.BinaryExporter;
import com.jme3.material.MatParam;
import com.jme3.material.Material;
import com.jme3.scene.Spatial;
import com.jme3.shader.VarType;
import com.jme3.texture.Texture;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.util.HashSet;
import java.util.Set;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.imageio.ImageIO;
import jme3utilities.Misc;
import jme3utilities.MySpatial;
import jme3utilities.MyString;
import jme3utilities.ui.ActionApplication;

/**
 * ActionApplication to import the Sinbad model.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class ImportSinbad extends ActionApplication {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(ImportSinbad.class.getName());
    /**
     * filesystem path to the asset root (directory/folder for output)
     */
    final private static String assetDirPath
            = "../MinieExamples/src/main/resources";
    // *************************************************************************
    // new methods exposed

    /**
     * Import the Sinbad model.
     */
    @Override
    public void actionInitializeApplication() {
        Spatial model = assetManager.loadModel("Models/Sinbad/Sinbad.mesh.xml");
        /*
         * Write the model to a J3O file.
         */
        String writeAssetPath = "Models/Sinbad/Sinbad.j3o";
        String writeFilePath
                = String.format("%s/%s", assetDirPath, writeAssetPath);
        JmeExporter exporter = BinaryExporter.getInstance();
        File writeFile = new File(writeFilePath);
        try {
            exporter.save(model, writeFile);
        } catch (IOException exception) {
            logger.log(Level.SEVERE, "write to {0} failed",
                    MyString.quote(writeFilePath));
            throw new RuntimeException(exception);
        }
        logger.log(Level.INFO, "wrote file {0}", MyString.quote(writeFilePath));
        /*
         * Collect all unique 2-D textures used in the model.
         */
        Set<TextureKey> textureKeys = new HashSet<>();
        for (Material materials : MySpatial.listMaterials(model, null)) {
            for (MatParam matParam : materials.getParams()) {
                if (matParam.getVarType() == VarType.Texture2D) {
                    Texture texture = (Texture) matParam.getValue();
                    TextureKey key = (TextureKey) texture.getKey();
                    textureKeys.add(key);
                }
            }
        }
        /*
         * Write each texture to a JPG file.
         */
        for (TextureKey textureKey : textureKeys) {
            writeToJpg(textureKey);
        }

        stop();
    }

    /**
     * Main entry point for the ImportSinbad application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        /*
         * Mute the chatty loggers found in some imported packages.
         */
        Misc.setLoggingLevels(Level.WARNING);
        /*
         * Set the logging level for this class and also for writeMap().
         */
        logger.setLevel(Level.INFO);
        Logger.getLogger(Misc.class.getName()).setLevel(Level.INFO);
        /*
         * Instantiate the application.
         */
        ImportSinbad application = new ImportSinbad();
        /*
         * Log the working directory.
         */
        String userDir = System.getProperty("user.dir");
        logger.log(Level.INFO, "working directory is {0}",
                MyString.quote(userDir));
        /*
         * Import the assets.
         */
        application.start();
    }
    // *************************************************************************
    // private methods

    /**
     * Write the specified texture to a JPG file.
     */
    private void writeToJpg(TextureKey key) {
        String assetPath = key.getName();

        AssetInfo info = assetManager.locateAsset(key);
        InputStream stream = info.openStream();

        BufferedImage image = null;
        try {
            image = ImageIO.read(stream);
        } catch (IOException exception) {
            logger.log(Level.SEVERE, "failed to read {0}",
                    MyString.quote(assetPath));
            throw new RuntimeException(exception);
        }

        String writeFilePath = String.format("%s/%s", assetDirPath, assetPath);
        File file = new File(writeFilePath);
        /*
         * Create the parent folder.
         */
        File parent = file.getParentFile();
        if (parent != null && !parent.exists()) {
            boolean success = parent.mkdirs();
            if (!success) {
                logger.log(Level.SEVERE, "Mkdirs failed while saving {0}",
                        MyString.quote(writeFilePath));
                throw new RuntimeException();
            }
        }
        /*
         * Write the texture's BufferedImage in JPG format.
         */
        try {
            ImageIO.write(image, "jpg", file);
        } catch (IOException exception) {
            logger.log(Level.SEVERE, "failed to write {0}",
                    MyString.quote(writeFilePath));
            throw new RuntimeException(exception);
        }
        logger.log(Level.INFO, "wrote file {0}", MyString.quote(writeFilePath));
    }
}
