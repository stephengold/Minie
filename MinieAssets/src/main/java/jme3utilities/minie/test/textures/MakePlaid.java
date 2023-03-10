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
 * A console application to generate the texture "plaid.png".
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class MakePlaid {
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
            = Logger.getLogger(MakePlaid.class.getName());
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
    private MakePlaid() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the MakePlaid application.
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
        makePlaid();
    }
    // *************************************************************************
    // private methods

    /**
     * Generate an image map for a simple red-and-white plaid.
     */
    private static void makePlaid() {
        // Create a blank, color buffered image for the texture map.
        BufferedImage image = new BufferedImage(textureSize, textureSize,
                BufferedImage.TYPE_4BYTE_ABGR);
        Graphics2D graphics = image.createGraphics();

        float brightness = 1f;
        float halfBright = brightness / 2f;
        float opacity = 1f;
        Color pink = new Color(brightness, halfBright, halfBright, opacity);
        Color red = new Color(brightness, 0f, 0f, opacity);
        Color white = new Color(brightness, brightness, brightness, opacity);

        int halfSize = textureSize / 2;
        graphics.setColor(white);
        graphics.fillRect(0, 0, halfSize, halfSize);
        graphics.setColor(pink);
        graphics.fillRect(halfSize, 0, halfSize, halfSize);
        graphics.fillRect(0, halfSize, halfSize, halfSize);
        graphics.setColor(red);
        graphics.fillRect(halfSize, halfSize, halfSize, halfSize);

        // Write the image to the asset file.
        String assetPath = "Textures/plaid.png";
        String filePath = String.format("%s/%s", assetDirPath, assetPath);
        try {
            Heart.writeImage(filePath, image);
        } catch (IOException exception) {
            throw new RuntimeException(exception);
        }
    }
}
