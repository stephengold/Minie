/*
Copyright (c) 2016, Riccardo Balbo
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package vhacd;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

public class VHACDParameters implements Cloneable {

    final public static Logger logger
            = Logger.getLogger(VHACDParameters.class.getName());
    private boolean DEBUG;
    private long objectId = 0L;

    public VHACDParameters() {
        objectId = create();

        setConcavity(objectId, 0.0025);
        setGamma(objectId, 0.00125);
        setMaxNumVerticesPerCH(objectId, 32);
    }

    long getId() {
        assert objectId != 0L;
        return objectId;
    }

    public void setDebugEnabled(boolean d) {
        DEBUG = d;
    }

    public boolean getDebugEnabled() {
        return DEBUG;
    }

    /**
     *
     * Set maximum number of voxels generated during the voxelization stage
     *
     * @param v default = 100000, min = 10000, max = 64000000
     */
    public void setVoxelResolution(int v) {
        Validate.inRange(v, "maxVoxels", 10_000, 64_000_000);
        setResolution(objectId, v);
    }

    public int getVoxelResolution() {
        int result = getResolution(objectId);
        return result;
    }

    /**
     *
     * Set maximum number of clipping stages. During each split stage, all the
     * model parts (with a concavity higher than the user defined threshold) are
     * clipped according the "best" clipping plane
     *
     * @param v default = 20, min = 1, max = 32
     */
    public void setClippingDepth(int v) {
        Validate.inRange(v, "depth", 1, 32);
        setDepth(objectId, v);
    }

    public int getClippingDepth() {
        int result = getDepth(objectId);
        return result;
    }

    /**
     *
     * Set maximum concavity
     *
     * @param v default = 0.0025, min = 0.0, max = 1.0
     */
    public void setMaxConcavity(double v) {
        Validate.fraction(v, "depth");
        setConcavity(objectId, v);
    }

    public double getMaxConcavity() {
        double result = getConcavity(objectId);
        return result;
    }

    /**
     *
     * Set granularity of the search for the "best" clipping plane
     *
     * @param v default = 4, min = 1, max = 16
     */
    public void setPlaneDownSampling(int v) {
        Validate.inRange(v, "granularity", 1, 16);
        setPlaneDownsampling(objectId, v);
    }

    public int getPlaneDownSampling() {
        int result = getPlaneDownsampling(objectId);
        return result;
    }

    /**
     *
     * Set precision of the convex-hull generation process during the clipping
     * plane selection stage
     *
     * @param v default = 4, min = 1, max = 16
     */
    public void setConvexHullDownSampling(int v) {
        Validate.inRange(v, "precision", 1, 16);
        setConvexhullDownsampling(objectId, v);
    }

    public int getConvexHullDownSampling() {
        int result = getConvexhullDownsampling(objectId);
        return result;
    }

    /**
     *
     * Set bias toward clipping along symmetry planes
     *
     * @param v default = 0.05, min = 0.0, max = 1.0,
     */
    public void setAlpha(double v) {
        Validate.fraction(v, "alpha");
        setAlpha(objectId, v);
    }

    public double getAlpha() {
        return getAlpha(objectId);
    }

    /**
     *
     * Set bias toward clipping along revolution axes
     *
     * @param v default = 0.05, min = 0.0, max = 1.0
     */
    public void setBeta(double v) {
        Validate.fraction(v, "beta");
        setBeta(objectId, v);
    }

    public double getBeta() {
        double result = getBeta(objectId);
        return result;
    }

    /**
     *
     * Set maximum allowed concavity during the merge stage
     *
     * @param v default = 0.00125, min = 0.0, max = 1.0
     */
    public void setGamma(double v) {
        Validate.fraction(v, "gamma");
        setGamma(objectId, v);
    }

    public double getGamma() {
        double result = getGamma(objectId);
        return result;
    }

    /**
     *
     * Enable/disable normalizing the mesh before applying the convex
     * decomposition
     *
     * @param v default = False
     */
    public void setPCA(boolean v) {
        setPca(objectId, v);
    }

    public boolean getPCA() {
        boolean result = getPca(objectId);
        return result;
    }

    /**
     *
     * Set approximate convex decomposition mode
     *
     * @param mode default = VOXEL
     */
    public void setACDMode(ACDMode mode) {
        setMode(objectId, mode.ordinal());
    }

    public ACDMode getACDMode() {
        int ordinal = getMode(objectId);
        ACDMode result = ACDMode.values()[ordinal];

        return result;
    }

    /**
     *
     * Set minimum volume to add vertices to convex-hulls
     *
     * @param v default = 0.0001, min = 0.0, max = 0.01
     */
    public void setMinVolumePerHull(double v) {
        Validate.inRange(v, "min volume", 0.0, 0.01);
        setMinVolumePerCH(objectId, v);
    }

    public double getMinVolumePerHull() {
        double result = getMinVolumePerCH(objectId);
        return result;
    }

    /**
     *
     * Set maximum number of vertices per convex-hull
     *
     * @param v default = 32, min = 4, max = 1024)
     */
    public void setMaxVerticesPerHull(int v) {
        Validate.inRange(v, "max vertices", 4, 1024);
        setMaxNumVerticesPerCH(objectId, v);
    }

    public int getMaxVerticesPerHull() {
        int result = getMaxNumVerticesPerCH(objectId);
        return result;
    }

    @Override
    public boolean equals(Object op2) {
        if (op2 instanceof VHACDParameters) {
            VHACDParameters other = (VHACDParameters) op2;
            boolean result = getACDMode() == other.getACDMode()
                    && getAlpha() == other.getAlpha()
                    && getBeta() == other.getBeta()
                    && getConvexHullApproximation() == other.getConvexHullApproximation()
                    && getConvexHullDownSampling() == other.getConvexHullDownSampling()
                    && getClippingDepth() == other.getClippingDepth()
                    && getGamma() == other.getGamma()
                    && getMaxConcavity() == other.getMaxConcavity()
                    && getMaxVerticesPerHull() == other.getMaxVerticesPerHull()
                    && getMinVolumePerHull() == other.getMinVolumePerHull()
                    && getOclAcceleration() == other.getOclAcceleration()
                    && getPCA() == other.getPCA()
                    && getPlaneDownSampling() == other.getPlaneDownSampling()
                    && getVoxelResolution() == other.getVoxelResolution();
            return result;
        }
        return false;
    }

    public void fromInputStream(InputStream is) throws IOException {
        DataInputStream dis = new DataInputStream(is);

        setMaxConcavity(dis.readDouble());
        setAlpha(dis.readDouble());
        setBeta(dis.readDouble());
        setGamma(dis.readDouble());
        setMinVolumePerHull(dis.readDouble());

        setVoxelResolution(dis.readInt());
        setMaxVerticesPerHull(dis.readInt());
        setClippingDepth(dis.readInt());
        setPlaneDownSampling(dis.readInt());
        setConvexHullDownSampling(dis.readInt());
        setPCA(dis.readInt() != 0);
        setACDMode(ACDMode.values()[dis.readInt()]);
        setConvexHullApproximation(dis.readInt());
        setOclAcceleration(dis.readInt());
    }

    public void toOutputStream(OutputStream os) throws IOException {
        DataOutputStream dos = new DataOutputStream(os);

        dos.writeDouble(getMaxConcavity());
        dos.writeDouble(getAlpha());
        dos.writeDouble(getBeta());
        dos.writeDouble(getGamma());
        dos.writeDouble(getMinVolumePerHull());

        dos.writeInt(getVoxelResolution());
        dos.writeInt(getMaxVerticesPerHull());
        dos.writeInt(getClippingDepth());
        dos.writeInt(getPlaneDownSampling());
        dos.writeInt(getConvexHullDownSampling());
        dos.writeInt(getPCA() ? 1 : 0);
        dos.writeInt(getACDMode().ordinal());
        dos.writeInt(getConvexHullApproximation());
        dos.writeInt(getOclAcceleration());
    }

    @Override
    public VHACDParameters clone() {
        try {
            VHACDParameters clone = (VHACDParameters) super.clone();
            clone.setACDMode(getACDMode());
            clone.setAlpha(getAlpha());
            clone.setBeta(getBeta());
            clone.setClippingDepth(getClippingDepth());
            clone.setConvexHullApproximation(getConvexHullApproximation());
            clone.setConvexHullDownSampling(getConvexHullDownSampling());
            clone.setGamma(getGamma());
            clone.setMaxConcavity(getMaxConcavity());
            clone.setMaxVerticesPerHull(getMaxVerticesPerHull());
            clone.setMinVolumePerHull(getMinVolumePerHull());
            clone.setOclAcceleration(getOclAcceleration());
            clone.setPCA(getPCA());
            clone.setPlaneDownSampling(getPlaneDownSampling());
            clone.setVoxelResolution(getVoxelResolution());
            return clone;
        } catch (CloneNotSupportedException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    protected void finalize() throws Throwable {
        super.finalize();
        logger.log(Level.FINE, "Finalizing {0}.", this);
        finalizeNative(objectId);
    }

    private int getConvexHullApproximation() {
        return getConvexhullApproximation(objectId);
    }

    private int getOclAcceleration() {
        return getOclAcceleration(objectId);
    }

    private void setConvexHullApproximation(int value) {
        setConvexhullApproximation(objectId, value);
    }

    private void setOclAcceleration(int value) {
        setOclAcceleration(objectId, value);
    }

    native private static long create();

    native private static void finalizeNative(long objectId);

    native private static double getAlpha(long objectId);

    native private static double getBeta(long objectId);

    native private static double getConcavity(long objectId);

    native private static int getConvexhullApproximation(long objectId);

    native private static int getConvexhullDownsampling(long objectId);

    native private static int getDepth(long objectId);

    native private static double getGamma(long objectId);

    native private static int getMaxNumVerticesPerCH(long objectId);

    native private static double getMinVolumePerCH(long objectId);

    native private static int getMode(long objectId);

    native private static int getOclAcceleration(long objectId);

    native private static boolean getPca(long objectId);

    native private static int getPlaneDownsampling(long objectId);

    native private static int getResolution(long objectId);

    native private static void setAlpha(long objectId, double alpha);

    native private static void setBeta(long objectId, double beta);

    native private static void setConcavity(long objectId, double depth);

    native private static void setConvexhullApproximation(long objectId,
            int value);

    native private static void setConvexhullDownsampling(long objectId,
            int precision);

    native private static void setDepth(long objectId, int depth);

    native private static void setGamma(long objectId, double beta);

    native private static void setMaxNumVerticesPerCH(long objectId,
            int numVertices);

    native private static void setMinVolumePerCH(long objectId, double volume);

    native private static void setMode(long objectId, int ordinal);

    native private static void setOclAcceleration(long objectId, int value);

    native private static void setPca(long objectId, boolean enable);

    native private static void setPlaneDownsampling(long objectId,
            int granularity);

    native private static void setResolution(long objectId, int maxVoxels);
}
