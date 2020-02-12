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

/**
 * A set of tuning parameters for convex decomposition, based on V-HACD's
 * IVHACD::Parameters.
 */
public class VHACDParameters implements Cloneable {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(VHACDParameters.class.getName());
    // *************************************************************************
    // fields

    /**
     * true&rarr;enable debug output
     */
    private boolean DEBUG;
    /**
     * unique identifier of the IVHACD::Parameters
     */
    private long objectId = 0L;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the default tuning parameters.
     */
    public VHACDParameters() {
        objectId = create();

        setConcavity(objectId, 0.0025);
        setMaxNumVerticesPerCH(objectId, 32);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Read selected parameters from an InputStream.
     *
     * @param is (not null)
     * @throws IOException from DataInputStream
     */
    public void fromInputStream(InputStream is) throws IOException {
        DataInputStream dis = new DataInputStream(is);

        setMaxConcavity(dis.readDouble());
        setAlpha(dis.readDouble());
        setBeta(dis.readDouble());
        setMinVolumePerHull(dis.readDouble());

        setVoxelResolution(dis.readInt());
        setMaxVerticesPerHull(dis.readInt());
        setPlaneDownSampling(dis.readInt());
        setConvexHullDownSampling(dis.readInt());
        setPCA(dis.readInt() != 0);
        setACDMode(ACDMode.values()[dis.readInt()]);
        setConvexHullApproximation(dis.readInt());
        setOclAcceleration(dis.readInt());
    }

    /**
     * Read the decomposition mode (native field: m_mode).
     *
     * @return an enum value (not null)
     */
    public ACDMode getACDMode() {
        int ordinal = getMode(objectId);
        ACDMode result = ACDMode.values()[ordinal];

        return result;
    }

    /**
     * Read the bias toward clipping along symmetry planes. (native field:
     * m_alpha).
     *
     * @return alpha (&ge;0, &le;1)
     */
    public double getAlpha() {
        return getAlpha(objectId);
    }

    /**
     * Read the bias toward clipping along revolution axes (native field:
     * m_beta).
     *
     * @return beta (&ge;0, &le;1)
     */
    public double getBeta() {
        double result = getBeta(objectId);
        return result;
    }

    /**
     * Read the precision of the convex-hull generation process (native field:
     * m_convexhullDownsampling).
     *
     * @return precision (&ge;1, &le;16)
     */
    public int getConvexHullDownSampling() {
        int result = getConvexhullDownsampling(objectId);
        return result;
    }

    /**
     * Test whether debug output is enabled.
     *
     * @return true if enabled, otherwise false
     */
    public boolean getDebugEnabled() {
        return DEBUG;
    }

    /**
     * Read the ID of the native object.
     *
     * @return the unique identifier (not zero)
     */
    long getId() {
        assert objectId != 0L;
        return objectId;
    }

    /**
     * Read the maximum concavity (native field: m_concavity).
     *
     * @return concavity (&ge;0, &le;1)
     */
    public double getMaxConcavity() {
        double result = getConcavity(objectId);
        return result;
    }

    /**
     * Read the maximum number of vertices per hull (native field:
     * m_maxNumVerticesPerCH).
     *
     * @return the limit (&ge;4, &le;1024)
     */
    public int getMaxVerticesPerHull() {
        int result = getMaxNumVerticesPerCH(objectId);
        return result;
    }

    /**
     * Read the minimum volume for added vertices (native field:
     * m_minVolumePerCH).
     *
     * @return the volume (&ge;0, &le;0.01)
     */
    public double getMinVolumePerHull() {
        double result = getMinVolumePerCH(objectId);
        return result;
    }

    /**
     * Test whether to normalize the mesh (native field: m_pca).
     *
     * @return true &rarr; normalize, false &rarr; don't normalize
     */
    public boolean getPCA() {
        boolean result = getPca(objectId);
        return result;
    }

    /**
     * Read the granularity of the search (native field: m_planeDownsampling).
     *
     * @return granularity (&ge;1, &le;16)
     */
    public int getPlaneDownSampling() {
        int result = getPlaneDownsampling(objectId);
        return result;
    }

    /**
     * Read the maximum number of voxels generated during the voxelization stage
     * (native field: m_resolution).
     *
     * @return number (&ge;10000, &le;64000000)
     */
    public int getVoxelResolution() {
        int result = getResolution(objectId);
        return result;
    }

    /**
     * Set approximate convex decomposition mode (native field: m_mode).
     *
     * @param mode default = VOXEL
     */
    public void setACDMode(ACDMode mode) {
        setMode(objectId, mode.ordinal());
    }

    /**
     * Set bias toward clipping along symmetry planes (native field: m_alpha).
     *
     * @param v default = 0.05, min = 0.0, max = 1.0,
     */
    public void setAlpha(double v) {
        Validate.fraction(v, "alpha");
        setAlpha(objectId, v);
    }

    /**
     * Set bias toward clipping along revolution axes (native field: m_beta).
     *
     * @param v default = 0.05, min = 0.0, max = 1.0
     */
    public void setBeta(double v) {
        Validate.fraction(v, "beta");
        setBeta(objectId, v);
    }

    /**
     * Set precision of the convex-hull generation process during the clipping
     * plane selection stage (native field: m_convexhullDownsampling).
     *
     * @param v default = 4, min = 1, max = 16
     */
    public void setConvexHullDownSampling(int v) {
        Validate.inRange(v, "precision", 1, 16);
        setConvexhullDownsampling(objectId, v);
    }

    /**
     * Alter whether debug output is enabled.
     *
     * @param d true &rarr; enable, false &rarr; disable (default=false)
     */
    public void setDebugEnabled(boolean d) {
        DEBUG = d;
    }

    /**
     * Set maximum concavity (native field: m_concavity).
     *
     * @param v default = 0.0025, min = 0.0, max = 1.0
     */
    public void setMaxConcavity(double v) {
        Validate.fraction(v, "depth");
        setConcavity(objectId, v);
    }

    /**
     * Set maximum number of vertices per convex-hull (native field:
     * m_maxNumVerticesPerCH).
     *
     * @param v default = 32, min = 4, max = 1024)
     */
    public void setMaxVerticesPerHull(int v) {
        Validate.inRange(v, "max vertices", 4, 1024);
        setMaxNumVerticesPerCH(objectId, v);
    }

    /**
     * Set minimum volume to add vertices to convex-hulls (native field:
     * m_minVolumePerCH).
     *
     * @param v default = 0.0001, min = 0.0, max = 0.01
     */
    public void setMinVolumePerHull(double v) {
        Validate.inRange(v, "min volume", 0.0, 0.01);
        setMinVolumePerCH(objectId, v);
    }

    /**
     * Enable/disable normalizing the mesh before applying the convex
     * decomposition (native field: m_pca).
     *
     * @param v default = False
     */
    public void setPCA(boolean v) {
        setPca(objectId, v);
    }

    /**
     * Set granularity of the search for the "best" clipping plane (native
     * field: m_planeDownsampling).
     *
     * @param v default = 4, min = 1, max = 16
     */
    public void setPlaneDownSampling(int v) {
        Validate.inRange(v, "granularity", 1, 16);
        setPlaneDownsampling(objectId, v);
    }

    /**
     * Set maximum number of voxels generated during the voxelization stage
     * (native field: m_resolution).
     *
     * @param v default = 100000, min = 10000, max = 64000000
     */
    public void setVoxelResolution(int v) {
        Validate.inRange(v, "maxVoxels", 10_000, 64_000_000);
        setResolution(objectId, v);
    }

    /**
     * Write selected parameters to an OutputStream.
     *
     * @param os (not null)
     * @throws IOException from DataOutputStream
     */
    public void toOutputStream(OutputStream os) throws IOException {
        DataOutputStream dos = new DataOutputStream(os);

        dos.writeDouble(getMaxConcavity());
        dos.writeDouble(getAlpha());
        dos.writeDouble(getBeta());
        dos.writeDouble(getMinVolumePerHull());

        dos.writeInt(getVoxelResolution());
        dos.writeInt(getMaxVerticesPerHull());
        dos.writeInt(getPlaneDownSampling());
        dos.writeInt(getConvexHullDownSampling());
        dos.writeInt(getPCA() ? 1 : 0);
        dos.writeInt(getACDMode().ordinal());
        dos.writeInt(getConvexHullApproximation());
        dos.writeInt(getOclAcceleration());
    }
    // *************************************************************************
    // Object methods

    /**
     * Create a copy of these parameters.
     *
     * @return a new instance, equivalent to this one
     */
    @Override
    public VHACDParameters clone() {
        try {
            VHACDParameters clone = (VHACDParameters) super.clone();
            clone.setACDMode(getACDMode());
            clone.setAlpha(getAlpha());
            clone.setBeta(getBeta());
            clone.setConvexHullApproximation(getConvexHullApproximation());
            clone.setConvexHullDownSampling(getConvexHullDownSampling());
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

    /**
     * Test for exact equivalence with another Object.
     *
     * @param otherObject (may be null, unaffected)
     * @return true if the objects are equivalent, otherwise false
     */
    @Override
    public boolean equals(Object otherObject) {
        boolean result;
        if (otherObject == this) {
            result = true;
        } else if (otherObject != null
                && otherObject.getClass() == getClass()) {
            VHACDParameters other = (VHACDParameters) otherObject;
            result = getACDMode() == other.getACDMode()
                    && (Double.compare(getAlpha(), other.getAlpha()) == 0)
                    && (Double.compare(getBeta(), other.getBeta()) == 0)
                    && getConvexHullApproximation() == other.getConvexHullApproximation()
                    && getConvexHullDownSampling() == other.getConvexHullDownSampling()
                    && getDebugEnabled() == other.getDebugEnabled()
                    && (Double.compare(getMaxConcavity(), other.getMaxConcavity()) == 0)
                    && getMaxVerticesPerHull() == other.getMaxVerticesPerHull()
                    && (Double.compare(getMinVolumePerHull(), other.getMinVolumePerHull()) == 0)
                    && getOclAcceleration() == other.getOclAcceleration()
                    && getPCA() == other.getPCA()
                    && getPlaneDownSampling() == other.getPlaneDownSampling()
                    && getVoxelResolution() == other.getVoxelResolution();
        } else {
            result = false;
        }

        return result;
    }

    /**
     * Generate the hash code for this object.
     *
     * @return value for use in hashing
     */
    @Override
    public int hashCode() {
        int hash = 5;
        hash = 83 * hash + (this.DEBUG ? 1 : 0);
        hash = 83 * hash + getACDMode().hashCode();
        hash = 83 * hash + Double.hashCode(getAlpha());
        hash = 83 * hash + Double.hashCode(getBeta());
        hash = 83 * hash + getConvexHullApproximation();
        hash = 83 * hash + getConvexHullDownSampling();
        hash = 83 * hash + Double.hashCode(getMaxConcavity());
        hash = 83 * hash + getMaxVerticesPerHull();
        hash = 83 * hash + Double.hashCode(getMinVolumePerHull());
        hash = 83 * hash + getOclAcceleration();
        hash = 83 * hash + Boolean.hashCode(getPCA());
        hash = 83 * hash + getPlaneDownSampling();
        hash = 83 * hash + getVoxelResolution();

        return hash;
    }

    /**
     * Finalize this instance just before it is destroyed. Should be invoked
     * only by a subclass or by the garbage collector.
     *
     * @throws Throwable ignored by the garbage collector
     */
    @Override
    protected void finalize() throws Throwable {
        super.finalize();
        logger.log(Level.FINE, "Finalizing {0}.", this);
        finalizeNative(objectId);
    }
    // *************************************************************************
    // private methods

    /**
     * native field: m_convexhullApproximation
     *
     * @return the value
     */
    private int getConvexHullApproximation() {
        return getConvexhullApproximation(objectId);
    }

    /**
     * native field: m_oclAcceleration
     *
     * @return the value
     */
    private int getOclAcceleration() {
        return getOclAcceleration(objectId);
    }

    /**
     * native field: m_convexhullApproximation
     *
     * @param value the desired value (default=true)
     */
    private void setConvexHullApproximation(int value) {
        setConvexhullApproximation(objectId, value);
    }

    /**
     * native field: m_oclAcceleration
     *
     * @param value the desired value (default=true)
     */
    private void setOclAcceleration(int value) {
        setOclAcceleration(objectId, value);
    }
    // *************************************************************************
    // native methods

    native private static long create();

    native private static void finalizeNative(long objectId);

    native private static double getAlpha(long objectId);

    native private static double getBeta(long objectId);

    native private static double getConcavity(long objectId);

    native private static int getConvexhullApproximation(long objectId);

    native private static int getConvexhullDownsampling(long objectId);

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
