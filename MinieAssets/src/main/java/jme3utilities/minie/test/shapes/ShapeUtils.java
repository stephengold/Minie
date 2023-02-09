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

import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.infos.ChildCollisionShape;
import com.jme3.bullet.util.CollisionShapeFactory;
import com.jme3.scene.Spatial;
import java.util.logging.Logger;
import vhacd.VHACD;
import vhacd.VHACDParameters;
import vhacd4.Vhacd4;
import vhacd4.Vhacd4Parameters;

/**
 * Utility methods to generate collision shapes.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final class ShapeUtils {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(ShapeUtils.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private ShapeUtils() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Create a collision shape using classic V-HACD and print statistics.
     *
     * @param modelRoot the model on which to base the shape (not null,
     * unaffected)
     * @param parameters (not null, unaffected)
     * @param prefix the prefix for each line of output
     * @return a new compound shape
     */
    static CompoundCollisionShape createVhacdShape(
            Spatial modelRoot, VHACDParameters parameters, String prefix) {
        System.out.println(parameters);
        VHACD.addProgressListener(new ProgressListener(prefix));

        long startTime = System.nanoTime();
        CompoundCollisionShape result = CollisionShapeFactory.createVhacdShape(
                modelRoot, parameters, null);
        long elapsedNanoseconds = System.nanoTime() - startTime;

        printSummary(result, elapsedNanoseconds);

        return result;
    }

    /**
     * Create a collision shape using V-HACD v4 and print statistics.
     *
     * @param modelRoot the model on which to base the shape (not null,
     * unaffected)
     * @param parameters (not null, unaffected)
     * @param prefix the prefix for each line of output
     * @return a new compound shape
     */
    static CompoundCollisionShape createVhacdShape(
            Spatial modelRoot, Vhacd4Parameters parameters, String prefix) {
        System.out.println(parameters);
        Vhacd4.addProgressListener(new ProgressListener(prefix));

        long startTime = System.nanoTime();
        CompoundCollisionShape result = CollisionShapeFactory.createVhacdShape(
                modelRoot, parameters, null);
        long elapsedNanoseconds = System.nanoTime() - startTime;

        printSummary(result, elapsedNanoseconds);

        return result;
    }

    /**
     * Print a summary of a compound hull shape to System.out and also check for
     * V-HACD failure.
     *
     * @param result the generated collision shape (not null, unaffected)
     * @param nanoseconds the time spent generating the shape (in nanoseconds,
     * &ge;0)
     */
    private static void printSummary(
            CompoundCollisionShape result, long nanoseconds) {
        int numChildren = result.countChildren();
        if (numChildren == 0) {
            throw new RuntimeException("V-HACD failed!");
        }

        int numVertices = 0;
        ChildCollisionShape[] children = result.listChildren();
        for (ChildCollisionShape child : children) {
            CollisionShape childShape = child.getShape();
            HullCollisionShape hull = (HullCollisionShape) childShape;
            numVertices += hull.countHullVertices();
        }
        System.out.printf("  number of hulls = %d (%.3f sec, %d vertices)%n",
                numChildren, nanoseconds * 1e-9f, numVertices);
    }
}
