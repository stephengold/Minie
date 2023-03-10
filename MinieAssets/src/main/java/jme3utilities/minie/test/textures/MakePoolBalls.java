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
package jme3utilities.minie.test.textures;

import com.jme3.math.FastMath;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MyString;
import jme3utilities.math.MyMath;

/**
 * A console application to generate 15 textures for pool balls.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class MakePoolBalls {
    // *************************************************************************
    // constants and loggers

    /**
     * alpha component for an opaque Color
     */
    final private static float opaque = 1f;
    /**
     * color for numerals and the 8 ball
     */
    final private static Color black = new Color(0f, 0f, 0f, opaque);
    /**
     * color of the 2 ball and the 10 ball
     */
    final private static Color blue = new Color(0f, 0f, 1f, opaque);
    /**
     * color of the 6 ball and the 14 ball
     */
    final private static Color green = new Color(0f, 0.4f, 0f, opaque);
    /**
     * color of the 7 ball and the 15 ball
     */
    final private static Color maroon = new Color(0.7f, 0.1f, 0.5f, opaque);
    /**
     * color of the 5 ball and the 13 ball
     */
    final private static Color orange = new Color(1f, 0.5f, 0f, opaque);
    /**
     * color of the 3 ball and the 11 ball
     */
    final private static Color red = new Color(1f, 0f, 0f, opaque);
    /**
     * color of the 4 ball and the 12 ball
     */
    final private static Color violet = new Color(0.5f, 0f, 1f, opaque);
    /**
     * color for the stripe/spot backgrounds
     */
    final private static Color white = new Color(1f, 1f, 1f, opaque);
    /**
     * color of the 1 ball and the 9 ball
     */
    final private static Color yellow = new Color(0.8f, 0.8f, 0f, opaque);
    /**
     * height of the texture map (in pixels)
     */
    final private static int textureHeight = 128;
    /**
     * width of the texture map (in pixels)
     */
    final private static int textureWidth = 256;
    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(MakePoolBalls.class.getName());
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
    private MakePoolBalls() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the MakePoolBalls application.
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

        // Generate color image maps.
        for (int ballId = 1; ballId <= 15; ++ballId) {
            makePoolBall(ballId);
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Generate the name of the identified pool ball.
     *
     * @param ballId which ball (8=eight ball, &ge;1, &le;15)
     * @return the name
     */
    private static String ballName(int ballId) {
        String result = ballId + "Ball";
        return result;
    }

    /**
     * Test whether a pixel is in the spot region of a pool ball. Tuned for
     * TextureMode.Original.
     *
     * @param x the X coordinate of the pixel (&ge;0, &lt;textureSize)
     * @param y the Y coordinate of the pixel (&ge;0, &lt;textureSize)
     * @return true if in the spot region, otherwise false
     */
    private static boolean inSpotRegion(int x, int y) {
        float xx = FastMath.PI * (x + 0.5f) / textureWidth; // 0 -> pi
        float yy = (y + 0.5f) / textureHeight; // 0 -> 1

        final float xCenter1 = 1f;
        final float xCenter2 = FastMath.HALF_PI + 1f;
        final float yCenter = 0.5f;
        final float r2 = 0.03f;

        if (MyMath.sumOfSquares(xx - xCenter1, yy - yCenter) < r2) {
            return true;
        } else if (MyMath.sumOfSquares(xx - xCenter2, yy - yCenter) < r2) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Test whether a pixel is in the stripe region of a pool ball.
     *
     * @param x the X coordinate of the pixel (&ge;0, &lt;textureSize)
     * @param y the Y coordinate of the pixel (&ge;0, &lt;textureSize)
     * @return true if in the stripe region, otherwise false
     */
    private static boolean inStripeRegion(int x, int y) {
        float yy = (y + 0.5f) / textureHeight;

        if (Math.abs(yy - 0.5f) < 0.25f) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Generate an image map for a single pool ball.
     *
     * @param ballId which ball (8=eight ball, &ge;1, &le;15)
     */
    private static void makePoolBall(int ballId) {
        // Create a blank, color buffered image for the texture map.
        BufferedImage image = new BufferedImage(textureWidth, textureHeight,
                BufferedImage.TYPE_4BYTE_ABGR);
        Graphics2D graphics = image.createGraphics();

        Color color;
        switch (ballId % 8) {
            case 0:
                color = black;
                break;
            case 1:
                color = yellow;
                break;
            case 2:
                color = blue;
                break;
            case 3:
                color = red;
                break;
            case 4:
                color = violet;
                break;
            case 5:
                color = orange;
                break;
            case 6:
                color = green;
                break;
            case 7:
                color = maroon;
                break;
            default:
                throw new IllegalArgumentException("ballId = " + ballId);
        }

        // Draw the texture, one pixel at a time.
        for (int x = 0; x < textureWidth; ++x) {
            for (int y = 0; y < textureHeight; ++y) {
                if (inSpotRegion(x, y)) {
                    graphics.setColor(white);

                } else if (ballId <= 8) { // solid pattern
                    graphics.setColor(color);

                } else { // stripe pattern
                    if (inStripeRegion(x, y)) {
                        graphics.setColor(color);
                    } else {
                        graphics.setColor(white);
                    }
                }

                graphics.fillRect(x, y, 1, 1);
            }
        }

        // Write the image to the asset file.
        String ballName = ballName(ballId);
        String assetPath = "Textures/poolBalls/" + ballName + ".png";
        String filePath = String.format("%s/%s", assetDirPath, assetPath);
        try {
            Heart.writeImage(filePath, image);
        } catch (IOException exception) {
            throw new RuntimeException(exception);
        }
    }
}
