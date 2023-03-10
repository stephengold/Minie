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
package jme3utilities.minie.test.textures;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MyString;

/**
 * A console application to generate the texture "greenTile.png".
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class MakeGreenTile {
    // *************************************************************************
    // constants and loggers

    /**
     * size of the texture map (pixels per side)
     */
    final private static int textureSize = 64;
    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(MakeGreenTile.class.getName());
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
    private MakeGreenTile() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the MakeGreenTile application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        // Mute the chatty loggers found in some imported packages.
        Heart.setLoggingLevels(Level.WARNING);

        // Log the working directory.
        String userDir = System.getProperty("user.dir");
        logger.log(Level.INFO, "working directory is {0}",
                MyString.quote(userDir));

        // Generate color image map.
        makeGreenTile();
    }
    // *************************************************************************
    // private methods

    /**
     * Generate an image map for a green square with a dark border.
     */
    private static void makeGreenTile() {
        // Create a blank, color, buffered image for the texture map.
        BufferedImage image = new BufferedImage(textureSize, textureSize,
                BufferedImage.TYPE_4BYTE_ABGR);
        Graphics2D graphics = image.createGraphics();

        float opacity = 1f;
        Color dark = new Color(0.1f, 0.1f, 0.1f, opacity);
        graphics.setColor(dark);
        graphics.fillRect(0, 0, textureSize, textureSize);

        Color green = new Color(0f, 0.4f, 0f, opacity);
        graphics.setColor(green);
        graphics.fillRect(4, 4, textureSize - 8, textureSize - 8);

        // Write the image to the asset file.
        String assetPath = "Textures/greenTile.png";
        String filePath = String.format("%s/%s", assetDirPath, assetPath);
        try {
            Heart.writeImage(filePath, image);
        } catch (IOException exception) {
            throw new RuntimeException(exception);
        }
    }
}
