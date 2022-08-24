/*
 Copyright (c) 2022, Stephen Gold
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
package jme3utilities.minie.tuner;

import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.infos.ChildCollisionShape;
import com.jme3.bullet.util.CollisionShapeFactory;
import com.jme3.scene.Spatial;
import java.io.PrintStream;
import java.util.Map;
import java.util.logging.Logger;
import jme3utilities.Validate;
import vhacd.VHACDParameters;
import vhacd4.Vhacd4Parameters;

/**
 * The parameters of a V-HACD test plus the corresponding results.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final class DecompositionTest implements Runnable {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final static Logger logger
            = Logger.getLogger(DecompositionTest.class.getName());
    // *************************************************************************
    // fields

    /**
     * output of the test, or null if not run yet
     */
    private CompoundCollisionShape shape;
    /**
     * run time of the test (in seconds)
     */
    private float latency;
    /**
     * 0 before running, 1 while running, 2 when done
     */
    private int runState = 0;
    /**
     * total number of vertices in the output, or null if not yet calculated
     */
    private Integer totalVertices;
    /**
     * input parameters for V-HACD v4, or null for classic
     */
    final private Vhacd4Parameters v4;
    /**
     * input parameters for classic V-HACD, or null for v4
     */
    final private VHACDParameters classic;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a V-HACD v4 test with the specified parameters.
     *
     * @param parameters (not null, unaffected)
     */
    DecompositionTest(Vhacd4Parameters parameters) {
        Validate.nonNull(parameters, "parameters");

        this.v4 = parameters.clone();
        this.classic = null;
    }

    /**
     * Instantiate a classic V-HACD test with the specified parameters.
     *
     * @param parameters (not null, unaffected)
     */
    DecompositionTest(VHACDParameters parameters) {
        Validate.nonNull(parameters, "parameters");

        this.v4 = null;
        this.classic = parameters.clone();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Return a copy of the parameters, assuming classic V-HACD.
     *
     * @return the pre-existing instance (not null)
     */
    VHACDParameters copyClassic() {
        VHACDParameters result = classic.clone();
        return result;
    }

    /**
     * Return a copy of the parameters, assuming V-HACD v4.
     *
     * @return a new instance (not null)
     */
    Vhacd4Parameters copyV4() {
        Vhacd4Parameters result = v4.clone();
        return result;
    }

    /**
     * Count the vertices in the output, assuming the test has been run.
     *
     * @return the pre-existing instance (not null)
     */
    int countVertices() {
        assert hasBeenRun();
        if (totalVertices == null) {
            calculateTotalVertices();
        }

        return totalVertices;
    }

    /**
     * Return the output of this test, assuming it has been run.
     *
     * @return the pre-existing instance (not null)
     */
    CompoundCollisionShape getShape() {
        assert hasBeenRun();
        return shape;
    }

    /**
     * Determine whether this test has been run.
     *
     * @return true if run, otherwise false
     */
    boolean hasBeenRun() {
        if (runState == 2) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Test whether classic V-HACD was/will be used.
     *
     * @return true for classic, false for v4
     */
    boolean isClassic() {
        if (classic == null) {
            return false;
        } else {
            return true;
        }
    }

    /**
     * Test whether the test is currently running.
     *
     * @return true if running, otherwise false
     */
    boolean isRunning() {
        if (runState == 1) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Return the latency of the test, assuming it has been run.
     *
     * @return the latency (in seconds, &ge;0)
     */
    float latency() {
        assert hasBeenRun();
        assert latency >= 0f : latency;
        return latency;
    }

    /**
     * Return the maximum vertices per hull.
     *
     * @return the value (&ge;4, &le;2_048)
     */
    int maxVerticesPerHull() {
        int result;

        if (classic == null) {
            result = v4.getMaxVerticesPerHull();
        } else {
            result = classic.getMaxVerticesPerHull();
        }

        assert result >= 4 : result;
        assert result <= 2_048 : result;
        return result;
    }

    /**
     * Return the voxel resolution.
     *
     * @return the value (&ge;10_000, &le;64_000_000)
     */
    int resolution() {
        int result;

        if (classic == null) {
            result = v4.getVoxelResolution();
        } else {
            result = classic.getVoxelResolution();
        }

        assert result >= 10_000 : result;
        assert result <= 64_000_000 : result;
        return result;
    }

    /**
     * Create or find a test identical to this one, but with the specified
     * alpha.
     *
     * @param alpha the desired alpha (&ge;0, &le;1)
     * @return a new or pre-existing test
     */
    DecompositionTest setAlpha(double alpha) {
        Validate.fraction(alpha, "alpha");

        VHACDParameters copy = copyClassic();
        copy.setAlpha(alpha);

        Model model = VhacdTuner.getModel();
        DecompositionTest result = model.getTest(copy);

        return result;
    }

    /**
     * Create or find a test identical to this one, but with the specified beta.
     *
     * @param beta the desired beta (&ge;0, &le;1)
     * @return a new or pre-existing test
     */
    DecompositionTest setBeta(double beta) {
        Validate.fraction(beta, "beta");

        VHACDParameters copy = copyClassic();
        copy.setBeta(beta);

        Model model = VhacdTuner.getModel();
        DecompositionTest result = model.getTest(copy);

        return result;
    }

    /**
     * Create or find a test identical to this one, but with the specified
     * convex-hull downsampling.
     *
     * @param precision the desired precision (&ge;1, &le;16)
     * @return a new or pre-existing test
     */
    DecompositionTest setHullDS(int precision) {
        Validate.inRange(precision, "precision", 1, 16);

        VHACDParameters copy = copyClassic();
        copy.setConvexHullDownSampling(precision);

        Model model = VhacdTuner.getModel();
        DecompositionTest result = model.getTest(copy);

        return result;
    }

    /**
     * Create or find a test identical to this one, but with the specified
     * maximum concavity.
     *
     * @param concavity the desired concavity (&ge;0, &le;1)
     * @return a new or pre-existing test
     */
    DecompositionTest setMaxConcavity(double concavity) {
        Validate.fraction(concavity, "concavity");

        VHACDParameters copy = copyClassic();
        copy.setMaxConcavity(concavity);

        Model model = VhacdTuner.getModel();
        DecompositionTest result = model.getTest(copy);

        return result;
    }

    /**
     * Create or find a test identical to this one, but with the specified
     * maximum number of hulls.
     *
     * @param limit the desired maximum number (&ge;1, &le;1_024)
     * @return a new or pre-existing test
     */
    DecompositionTest setMaxHulls(int limit) {
        Validate.inRange(limit, "limit", 1, 1_024);

        Vhacd4Parameters copy = copyV4();
        copy.setMaxHulls(limit);

        Model model = VhacdTuner.getModel();
        DecompositionTest result = model.getTest(copy);

        return result;
    }

    /**
     * Create or find a test identical to this one, but with the specified
     * maximum recursion.
     *
     * @param depth the desired maximum depth (&ge;2, &le;64)
     * @return a new or pre-existing test
     */
    DecompositionTest setMaxRecursion(int depth) {
        Validate.inRange(depth, "depth", 2, 64);

        Vhacd4Parameters copy = copyV4();
        copy.setMaxRecursion(depth);
        Model model = VhacdTuner.getModel();
        DecompositionTest result = model.getTest(copy);

        return result;
    }

    /**
     * Create or find a test identical to this one, but with the specified
     * maximum vertices per hull.
     *
     * @param limit the desired maximum number (&ge;4, &le;2_048)
     * @return a new or pre-existing test
     */
    DecompositionTest setMaxVerticesPerHull(int limit) {
        Validate.inRange(limit, "limit", 4, 2_048);

        Model model = VhacdTuner.getModel();
        DecompositionTest result;
        if (classic == null) {
            Vhacd4Parameters copy = copyV4();
            copy.setMaxVerticesPerHull(limit);
            result = model.getTest(copy);
        } else {
            VHACDParameters copy = copyClassic();
            copy.setMaxVerticesPerHull(limit);
            result = model.getTest(copy);
        }

        return result;
    }

    /**
     * Create or find a test identical to this one, but with the specified
     * minimum edge length.
     *
     * @param length the desired minimum length (&ge;1, &le;32)
     * @return a new or pre-existing test
     */
    DecompositionTest setMinEdgeLength(int length) {
        Validate.inRange(length, "length", 1, 32);

        Vhacd4Parameters copy = copyV4();
        copy.setMinEdgeLength(length);
        Model model = VhacdTuner.getModel();
        DecompositionTest result = model.getTest(copy);

        return result;
    }

    /**
     * Create or find a test identical to this one, but with the specified min
     * volume per hull.
     *
     * @param volume the desired volume (&ge;0, &le;0.1)
     * @return a new or pre-existing test
     */
    DecompositionTest setMinVolumePH(double volume) {
        Validate.inRange(volume, "volume", 0.0, 0.1);

        VHACDParameters copy = copyClassic();
        copy.setMinVolumePerHull(volume);

        Model model = VhacdTuner.getModel();
        DecompositionTest result = model.getTest(copy);

        return result;
    }

    /**
     * Create or find a test identical to this one, but with the specified plane
     * downsampling.
     *
     * @param precision the desired precision (&ge;1, &le;16)
     * @return a new or pre-existing test
     */
    DecompositionTest setPlaneDS(int precision) {
        Validate.inRange(precision, "precision", 1, 16);

        VHACDParameters copy = copyClassic();
        copy.setPlaneDownSampling(precision);

        Model model = VhacdTuner.getModel();
        DecompositionTest result = model.getTest(copy);

        return result;
    }

    /**
     * Create or find a test identical to this one, but with the specified
     * resolution.
     *
     * @param maxVoxels the desired resolution (&ge;10_000, &le;64_000_000)
     * @return a new or pre-existing test
     */
    DecompositionTest setResolution(int maxVoxels) {
        Validate.inRange(maxVoxels, "maxVoxels", 10_000, 64_000_000);

        Model model = VhacdTuner.getModel();
        DecompositionTest result;
        if (classic == null) {
            Vhacd4Parameters copy = copyV4();
            copy.setVoxelResolution(maxVoxels);
            result = model.getTest(copy);
        } else {
            VHACDParameters copy = copyClassic();
            copy.setVoxelResolution(maxVoxels);
            result = model.getTest(copy);
        }

        return result;
    }

    /**
     * Create or find a test identical to this one, but with the specified
     * volume percent error.
     *
     * @param percentage the desired percent error (&ge;0, &le;100)
     * @return a new or pre-existing test
     */
    DecompositionTest setVolumePercentError(double percentage) {
        Validate.inRange(percentage, "percentage", 0.0, 100.0);

        Vhacd4Parameters copy = copyV4();
        copy.setVolumePercentError(percentage);

        Model model = VhacdTuner.getModel();
        DecompositionTest result = model.getTest(copy);

        return result;
    }

    /**
     * Represent this instance as a Map, in order to make comparisons easier.
     *
     * @return a map of property names to values
     */
    Map<String, Object> toMap() {
        Map<String, Object> result;
        if (classic == null) {
            result = v4.toMap();
        } else {
            result = classic.toMap();
        }
        result.put("classic", isClassic());
        result.put("runState", runState);
        if (runState == 2) {
            int numHulls = shape.countChildren();
            result.put("hulls", numHulls);
            result.put("seconds", latency);
            result.put("vertices", totalVertices);
        }

        return result;
    }

    /**
     * Write the V-HACD parameters to a stream, as Java source code.
     *
     * @param stream the output stream (not null)
     */
    void write(PrintStream stream) {
        if (v4 == null) {
            stream.printf("import vhacd.VHACDParameters;%n%n"
                    + "public class WParameters extends VHACDParameters {%n%n"
                    + "    public WParameters() {%n"
            );
            stream.printf("        setACDMode(ACDMode.%s);%n",
                    classic.getACDMode());
            stream.printf("        setAlpha(%s);%n",
                    classic.getAlpha());
            stream.printf("        setBeta(%s);%n",
                    classic.getBeta());
            stream.printf("        setConvexHullDownSampling(%s);%n",
                    classic.getConvexHullDownSampling());
            stream.printf("        setDebugEnabled(%s);%n",
                    classic.getDebugEnabled());
            stream.printf("        setMaxConcavity(%s);%n",
                    classic.getMaxConcavity());
            stream.printf("        setMaxVerticesPerHull(%s);%n",
                    classic.getMaxVerticesPerHull());
            stream.printf("        setMinVolumePerHull(%s);%n",
                    classic.getMinVolumePerHull());
            stream.printf("        setPCA(%s);%n",
                    classic.getPCA());
            stream.printf("        setPlaneDownSampling(%s);%n",
                    classic.getPlaneDownSampling());
            stream.printf("        setVoxelResolution(%s);%n",
                    classic.getVoxelResolution());
            stream.printf("    }%n}%n");

        } else {
            stream.printf("import vhacd.Vhacd4Parameters;%n%n"
                    + "public class WParameters extends Vhacd4Parameters {%n%n"
                    + "    public WParameters() {%n"
            );
            stream.printf("        setAsync(%s);%n",
                    v4.isAsync());
            stream.printf("        setFillMode(FillMode.%s);%n",
                    v4.getFillMode());
            stream.printf("        setFindBestPlane(%s);%n, ",
                    v4.isFindBestPlane());
            stream.printf("        setMaxHulls(%s);%n",
                    v4.getMaxHulls());
            stream.printf("        setMaxRecursion(%s);%n",
                    v4.getMaxRecursion());
            stream.printf("        setMaxVerticesPerHull(%s);%n",
                    v4.getMaxVerticesPerHull());
            stream.printf("        setMinEdgeLength(%s);%n",
                    v4.getMinEdgeLength());
            stream.printf("        setShrinkWrap(%s);%n",
                    v4.isShrinkWrap());
            stream.printf("        setVolumePercentError(%s);%n",
                    v4.getVolumePercentError());
            stream.printf("        setVoxelResolution(%s);%n",
                    v4.getVoxelResolution());
            stream.printf("    }%n}%n");
        }
    }
    // *************************************************************************
    // Runnable methods

    /**
     * Compute the collision shape using V-HACD.
     */
    @Override
    public void run() {
        assert !hasBeenRun();
        this.runState = 1;

        Model model = VhacdTuner.getModel();
        Spatial modelRoot = model.getRootSpatial();
        CompoundCollisionShape collisionShape;

        long startTime = System.nanoTime();
        if (v4 == null) {
            collisionShape = CollisionShapeFactory.createVhacdShape(
                    modelRoot, classic, null);
        } else {
            collisionShape = CollisionShapeFactory.createVhacdShape(
                    modelRoot, v4, null);
        }

        long elapsedNanoseconds = System.nanoTime() - startTime;
        assert elapsedNanoseconds >= 0L : elapsedNanoseconds;
        float elapsedSeconds = elapsedNanoseconds * 1e-9f;
        assert elapsedSeconds >= 0f : elapsedSeconds;

        assert collisionShape != null;
        this.shape = collisionShape;
        this.latency = elapsedSeconds;

        this.runState = 2; // Do this last!
    }
    // *************************************************************************
    // Object methods

    /**
     * Represent this instance as a String.
     *
     * @return a descriptive string of text (not null, not empty)
     */
    @Override
    public String toString() {
        String result;
        if (classic == null) {
            result = v4.toString();
        } else {
            result = classic.toString();
        }

        switch (runState) {
            case 0:
                result += " not started";
                break;

            case 1:
                result += " running";
                break;

            case 2:
                int numHulls = shape.countChildren();
                result = String.format("%s -> %f sec, %d hulls, %d vertices",
                        result, latency, numHulls, totalVertices);
                break;

            default:
                throw new IllegalStateException("runState = " + runState);
        }

        return result;
    }
    // *************************************************************************
    // private methods

    /**
     * Calculate the total number of vertices in the output shape.
     */
    private void calculateTotalVertices() {
        int sum = 0;
        ChildCollisionShape[] children = shape.listChildren();
        for (ChildCollisionShape child : children) {
            CollisionShape childShape = child.getShape();
            HullCollisionShape hull = (HullCollisionShape) childShape;
            sum += hull.countHullVertices();
        }
        this.totalVertices = sum;
    }
}
