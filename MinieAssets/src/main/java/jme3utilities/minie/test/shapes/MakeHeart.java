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

import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.math.FastMath;
import com.jme3.math.Matrix3f;
import com.jme3.math.Vector3f;
import com.jme3.system.NativeLibraryLoader;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MyString;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.noise.Generator;

/**
 * A console application to generate the collision-shape asset "heart.j3o".
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class MakeHeart {
    // *************************************************************************
    // constants and loggers

    /**
     * maximum uncertainty for solutions (&gt;0)
     */
    final private static double tolerance = 1e-7;
    /**
     * number of axes in a vector
     */
    final private static int numAxes = 3;
    /**
     * number of sample points used to generate the convex hull
     */
    final private static int numSamples = 200;
    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(MakeHeart.class.getName());
    /**
     * filesystem path to the asset directory/folder for output
     */
    final private static String assetDirPath
            = "../MinieExamples/src/main/resources";
    // *************************************************************************
    // fields

    /**
     * coordinate locations of sample points
     */
    final private static FloatBuffer sampleBuffer
            = BufferUtils.createFloatBuffer(numAxes * numSamples);
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private MakeHeart() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the MakeHeart application.
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

        // Generate the collision shape.
        makeHeart();
    }
    // *************************************************************************
    // private methods

    /**
     * Generate a collision shape for a 3-D heart.
     */
    private static void makeHeart() {
        // Phase 1: add solutions for y=0, x>0
        int n1 = 30;
        for (int i = 0; i < n1; ++i) {
            double gamma = i * Math.PI / (n1 - 1);
            solve1(gamma);
        }

        // Phase 2: add solutions for x=0, y>0 (symmetric in y)
        int n2 = 15;
        for (int i = 0; i < n2; ++i) {
            double beta = (i + 1) * Math.PI / n2;
            solve2(beta);
        }

        // Phase 3: add solutions for x>0, y>0 (symmetric in y)
        Generator generate = new Generator();
        while (sampleBuffer.remaining() >= 2 * numAxes) {
            double x = 1.2 * generate.nextDouble();
            double z = -0.9 + 2.2 * generate.nextDouble();
            solve3(x, z);
        }
        /*
         * Phase 4: combine 2 hulls (one with a 180-degree rotation)
         * to generate a compound shape.
         */
        Vector3f scale = new Vector3f(1f, 0.7f, 1f);
        MyBuffer.scale(sampleBuffer, 0, sampleBuffer.limit(), scale);
        HullCollisionShape half = new HullCollisionShape(sampleBuffer);
        CompoundCollisionShape compound = new CompoundCollisionShape();
        compound.addChildShape(half);
        Matrix3f rotation = new Matrix3f();
        rotation.fromAngleAxis(FastMath.PI, Vector3f.UNIT_Z);
        compound.addChildShape(half, Vector3f.ZERO, rotation);

        // Phase 5: write the shape to the asset file.
        String assetPath = "CollisionShapes/heart.j3o";
        String writeFilePath = String.format("%s/%s", assetDirPath, assetPath);
        Heart.writeJ3O(writeFilePath, compound);
    }

    /**
     * Evaluate the parametric function for the shape. The formula was adapted
     * from https://commons.wikimedia.org/wiki/File:Heart3D.png
     *
     * @param x the X coordinate of the point to test
     * @param y the Y coordinate of the point to test
     * @param z the Z coordinate of the point to test
     * @return error
     */
    private static double plug(double x, double y, double z) {
        double x2 = x * x;
        double y2 = y * y;
        double z2 = z * z;
        double subExp = x2 + 2.25 * y2 + z2 - 1.0;
        double r3 = subExp * subExp * subExp;
        double z3 = z2 * z;
        double error = r3 - (x2 + 0.045 * y2) * z3;

        return error;
    }

    /**
     * Use bisection search to find a value of R for which (R sin gamma, 0, R
     * cos gamma) is a zero of the parametric function. Always adds one sample
     * to the buffer.
     *
     * @param gamma angle from the +Z axis (in radians)
     */
    private static void solve1(double gamma) {
        double sinGamma = Math.sin(gamma);
        if (sinGamma < 0.0 && sinGamma > -1e-10) {
            sinGamma = 0.0;
        }
        double cosGamma = Math.cos(gamma);

        double r1 = 0.75;
        boolean s1 = trial1(cosGamma, sinGamma, r1);
        double r2 = 1.5;
        boolean s2 = trial1(cosGamma, sinGamma, r2);
        assert s1 != s2;
        if (s1) {
            double r = r1;
            r1 = r2;
            r2 = r;
        }

        while (Math.abs(r1 - r2) > tolerance) {
            assert !trial1(cosGamma, sinGamma, r1);
            assert trial1(cosGamma, sinGamma, r2);

            // Bisect to obtain a new estimate of the solution.
            double r = (r1 + r2) / 2.0;
            boolean s = trial1(cosGamma, sinGamma, r);
            if (s) {
                r2 = r;
            } else {
                r1 = r;
            }
        }
        double r = (r1 + r2) / 2.0;
        double x = r * sinGamma;
        double z = r * cosGamma;
        writeSample(x, 0.0, z);
    }

    /**
     * Use bisection search to find a value of R for which (0, R sin gamma, R
     * cos gamma) is a zero of the parametric function. Always adds 2 samples to
     * the buffer.
     *
     * @param gamma angle from the +Z axis (in radians)
     */
    private static void solve2(double gamma) {
        double sinGamma = Math.sin(gamma);
        double cosGamma = Math.cos(gamma);

        double r1 = 0.65;
        boolean s1 = trial2(cosGamma, sinGamma, r1);
        double r2 = 1.1;
        boolean s2 = trial2(cosGamma, sinGamma, r2);
        assert s1 != s2;
        if (s1) {
            double r = r1;
            r1 = r2;
            r2 = r;
        }

        while (Math.abs(r1 - r2) > tolerance) {
            assert !trial2(cosGamma, sinGamma, r1);
            assert trial2(cosGamma, sinGamma, r2);

            // Bisect to obtain a new estimate of R.
            double r = (r1 + r2) / 2.0;
            boolean s = trial2(cosGamma, sinGamma, r);
            if (s) {
                r2 = r;
            } else {
                r1 = r;
            }
        }

        double r = (r1 + r2) / 2.0;
        double y = r * sinGamma;
        double z = r * cosGamma;
        writeSample(0.0, y, z);
        writeSample(0.0, -y, z);
    }

    /**
     * Use bisection search to find a value of R for which (x, R, z) is a zero
     * of the parametric function. If successful, adds 2 samples to the buffer.
     *
     * @param x the X coordinate to solve for
     * @param z the Z coordinate to solve for
     * @return true if successful, otherwise false
     */
    private static boolean solve3(double x, double z) {
        double r1 = 0.0;
        boolean s1 = trial3(x, z, r1);
        double r2 = 0.7;
        boolean s2 = trial3(x, z, r2);
        if (s1 == s2) {
            return false;
        }
        if (s1) {
            double r = r1;
            r1 = r2;
            r2 = r;
        }

        while (Math.abs(r1 - r2) > tolerance) {
            assert !trial3(x, z, r1);
            assert trial3(x, z, r2);

            // Bisect to obtain a new estimate of the solution.
            double r = (r1 + r2) / 2.0;
            boolean s = trial3(x, z, r);
            if (s) {
                r2 = r;
            } else {
                r1 = r;
            }
        }

        double r = (r1 + r2) / 2.0;
        writeSample(x, r, z);
        writeSample(x, -r, z);
        return true;
    }

    private static boolean trial1(double cosGamma, double sinGamma, double r) {
        double x = r * sinGamma;
        double z = r * cosGamma;
        double error = plug(x, 0.0, z);

        return error > 0.0;
    }

    private static boolean trial2(double cosGamma, double sinGamma, double r) {
        double y = r * sinGamma;
        double z = r * cosGamma;
        double error = plug(0.0, y, z);

        return error > 0.0;
    }

    private static boolean trial3(double x, double z, double r) {
        double error = plug(x, r, z);
        return error > 0.0;
    }

    /**
     * Write a sample point to the buffer.
     *
     * @param xx X coordinate of the sample (&ge;0)
     * @param yy Y coordinate of the sample
     * @param zz Z coordinate of the sample
     */
    private static void writeSample(double xx, double yy, double zz) {
        float x = (float) xx;
        float y = (float) yy;
        float z = (float) zz;
        sampleBuffer.put(x).put(y).put(z);

        //double error = plug(x, y, z);
        //double r = MyMath.hypotenuse(xx, yy, zz);
        //System.out.printf("x=%.3f y=%.3f z=%.3f   r=%.4f    error=%s%n",
        //        x, y, z, r, error);
    }
}
