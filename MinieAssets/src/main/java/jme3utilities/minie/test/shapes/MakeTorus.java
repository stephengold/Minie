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
package jme3utilities.minie.test.shapes;

import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.util.CollisionShapeFactory;
import com.jme3.export.JmeExporter;
import com.jme3.export.binary.BinaryExporter;
import com.jme3.scene.Geometry;
import com.jme3.scene.shape.Torus;
import com.jme3.system.NativeLibraryLoader;
import java.io.File;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Misc;
import jme3utilities.MyString;
import vhacd.VHACDParameters;

/**
 * Console application to generate the collision-shape asset "torus.j3o".
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class MakeTorus {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(MakeTorus.class.getName());
    /**
     * filesystem path to the asset directory/folder for output
     */
    final private static String assetDirPath
            = "../MinieExamples/src/main/resources";
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the MakeTorus application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);
        /*
         * Mute the chatty loggers found in some imported packages.
         */
        Misc.setLoggingLevels(Level.WARNING);
        /*
         * Set the logging level for this class and also for writeMap().
         */
        //logger.setLevel(Level.INFO);
        //Logger.getLogger(Misc.class.getName()).setLevel(Level.INFO);
        /*
         * Instantiate the application.
         */
        MakeTorus application = new MakeTorus();
        /*
         * Log the working directory.
         */
        String userDir = System.getProperty("user.dir");
        logger.log(Level.INFO, "working directory is {0}",
                MyString.quote(userDir));
        /*
         * Generate collision shape.
         */
        application.makeTorus();
    }
    // *************************************************************************
    // private methods

    /**
     * Generate a collision shape for a small torus.
     */
    private void makeTorus() {
        /*
         * Generate a Mesh to approximate a torus.
         */
        int cSamples = 32;
        int rSamples = 32;
        float minorRadius = 0.04f;
        float majorRadius = 0.3f;
        Torus mesh = new Torus(cSamples, rSamples, minorRadius, majorRadius);
        /*
         * Generate a CollisionShape to approximate the Mesh.
         */
        Geometry geometry = new Geometry("torus", mesh);
        VHACDParameters parms = new VHACDParameters();
        CompoundCollisionShape torusShape
                = CollisionShapeFactory.createVhacdShape(geometry, parms, null);
        /*
         * Write the shape to the asset file.
         */
        String assetPath = "CollisionShapes/torus.j3o";
        String writeFilePath = String.format("%s/%s", assetDirPath, assetPath);
        JmeExporter exporter = BinaryExporter.getInstance();
        File writeFile = new File(writeFilePath);
        try {
            exporter.save(torusShape, writeFile);
        } catch (IOException exception) {
            logger.log(Level.SEVERE, "write to {0} failed",
                    MyString.quote(writeFilePath));
            throw new RuntimeException(exception);
        }
        logger.log(Level.INFO, "wrote file {0}", MyString.quote(writeFilePath));
    }
}
