/*
 Copyright (c) 2016, Riccardo Balbo
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

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
package vhacd;

import com.jme3.bullet.NativePhysicsObject;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Map;
import java.util.TreeMap;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A set of tuning parameters for convex decomposition, based on classic
 * V-HACD's IVHACD::Parameters.
 */
public class VHACDParameters
        extends NativePhysicsObject
        implements Cloneable {
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
    private boolean debug;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the default tuning parameters.
     */
    public VHACDParameters() {
        long objectId = create();
        super.setNativeId(objectId);

        setConcavity(objectId, 0.0025);
        setMaxNumVerticesPerCH(objectId, 32);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Read selected parameters from the specified InputStream.
     *
     * @param is the stream to read (not null)
     * @throws IOException from DataInputStream
     */
    public void fromInputStream(InputStream is) throws IOException {
        try (DataInputStream dis = new DataInputStream(is)) {
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
    }

    /**
     * Return the decomposition mode (native field: m_mode).
     *
     * @return an enum value (not null)
     */
    public ACDMode getACDMode() {
        long objectId = nativeId();
        int ordinal = getMode(objectId);
        ACDMode result = ACDMode.values()[ordinal];

        return result;
    }

    /**
     * Return the bias toward clipping along symmetry planes. (native field:
     * m_alpha).
     *
     * @return alpha (&ge;0, &le;1)
     */
    public double getAlpha() {
        long objectId = nativeId();
        double result = getAlpha(objectId);

        return result;
    }

    /**
     * Return the bias toward clipping along revolution axes (native field:
     * m_beta).
     *
     * @return beta (&ge;0, &le;1)
     */
    public double getBeta() {
        long objectId = nativeId();
        double result = getBeta(objectId);

        return result;
    }

    /**
     * Return the precision of the convex-hull generation process (native field:
     * m_convexhullDownsampling).
     *
     * @return precision (&ge;1, &le;16)
     */
    public int getConvexHullDownSampling() {
        long objectId = nativeId();
        int result = getConvexhullDownsampling(objectId);

        return result;
    }

    /**
     * Test whether debug output is enabled.
     *
     * @return true if enabled, otherwise false
     */
    public boolean getDebugEnabled() {
        return debug;
    }

    /**
     * Return the maximum concavity (native field: m_concavity).
     *
     * @return concavity (&ge;0, &le;1)
     */
    public double getMaxConcavity() {
        long objectId = nativeId();
        double result = getConcavity(objectId);

        return result;
    }

    /**
     * Return the maximum number of vertices per convex hull (native field:
     * m_maxNumVerticesPerCH).
     *
     * @return the limit (&ge;4, &le;1024)
     */
    public int getMaxVerticesPerHull() {
        long objectId = nativeId();
        int result = getMaxNumVerticesPerCH(objectId);

        return result;
    }

    /**
     * Return the minimum volume for added vertices (native field:
     * m_minVolumePerCH).
     *
     * @return the volume (&ge;0, &le;0.01)
     */
    public double getMinVolumePerHull() {
        long objectId = nativeId();
        double result = getMinVolumePerCH(objectId);

        return result;
    }

    /**
     * Test whether to normalize the mesh (native field: m_pca).
     *
     * @return true &rarr; normalize, false &rarr; don't normalize
     */
    public boolean getPCA() {
        long objectId = nativeId();
        boolean result = getPca(objectId);

        return result;
    }

    /**
     * Return the granularity of the search (native field: m_planeDownsampling).
     *
     * @return granularity (&ge;1, &le;16)
     */
    public int getPlaneDownSampling() {
        long objectId = nativeId();
        int result = getPlaneDownsampling(objectId);

        return result;
    }

    /**
     * Return the maximum number of voxels generated during the voxelization
     * stage (native field: m_resolution).
     *
     * @return number (&ge;10000, &le;64000000)
     */
    public int getVoxelResolution() {
        long objectId = nativeId();
        int result = getResolution(objectId);

        return result;
    }

    /**
     * Set approximate convex decomposition mode (native field: m_mode).
     *
     * @param mode default = VOXEL
     */
    public void setACDMode(ACDMode mode) {
        long objectId = nativeId();
        setMode(objectId, mode.ordinal());
    }

    /**
     * Set bias toward clipping along symmetry planes (native field: m_alpha).
     *
     * @param alpha default = 0.05, min = 0.0, max = 1.0,
     */
    public void setAlpha(double alpha) {
        Validate.fraction(alpha, "alpha");

        long objectId = nativeId();
        setAlpha(objectId, alpha);
    }

    /**
     * Set bias toward clipping along revolution axes (native field: m_beta).
     *
     * @param beta default = 0.05, min = 0.0, max = 1.0
     */
    public void setBeta(double beta) {
        Validate.fraction(beta, "beta");

        long objectId = nativeId();
        setBeta(objectId, beta);
    }

    /**
     * Set precision of the convex-hull generation process during the clipping
     * plane selection stage (native field: m_convexhullDownsampling).
     *
     * @param precision default = 4, min = 1, max = 16
     */
    public void setConvexHullDownSampling(int precision) {
        Validate.inRange(precision, "precision", 1, 16);

        long objectId = nativeId();
        setConvexhullDownsampling(objectId, precision);
    }

    /**
     * Alter whether debug output is enabled.
     *
     * @param setting true &rarr; enable, false &rarr; disable (default=false)
     */
    public void setDebugEnabled(boolean setting) {
        this.debug = setting;
    }

    /**
     * Set maximum concavity (native field: m_concavity).
     *
     * @param concavity default = 0.0025, min = 0.0, max = 1.0
     * <p>
     * Note: the native default is 0.001.
     */
    public void setMaxConcavity(double concavity) {
        Validate.fraction(concavity, "concavity");

        long objectId = nativeId();
        setConcavity(objectId, concavity);
    }

    /**
     * Alter the maximum number of vertices per convex hull (native field:
     * m_maxNumVerticesPerCH).
     *
     * @param limit default = 32, min = 4, max = 1024
     * <p>
     * Note: the native default is 64.
     */
    public void setMaxVerticesPerHull(int limit) {
        Validate.inRange(limit, "limit", 4, 1024);

        long objectId = nativeId();
        setMaxNumVerticesPerCH(objectId, limit);
    }

    /**
     * Set the minimum volume to add vertices to convex hulls (native field:
     * m_minVolumePerCH).
     *
     * @param volume default = 0.0001, min = 0.0, max = 0.01
     */
    public void setMinVolumePerHull(double volume) {
        Validate.inRange(volume, "min volume", 0.0, 0.01);

        long objectId = nativeId();
        setMinVolumePerCH(objectId, volume);
    }

    /**
     * Enable/disable normalizing the mesh before applying the convex
     * decomposition (native field: m_pca).
     *
     * @param v default = false
     */
    public void setPCA(boolean v) {
        long objectId = nativeId();
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

        long objectId = nativeId();
        setPlaneDownsampling(objectId, v);
    }

    /**
     * Alter the maximum number of voxels generated during the voxelization
     * stage (native field: m_resolution).
     *
     * @param maxVoxels default = 100_000, min = 10_000, max = 64_000_000
     */
    public void setVoxelResolution(int maxVoxels) {
        Validate.inRange(maxVoxels, "maxVoxels", 10_000, 64_000_000);

        long objectId = nativeId();
        setResolution(objectId, maxVoxels);
    }

    /**
     * Represent this instance as a Map, in order to make comparisons easier.
     *
     * @return a map of property names to values
     */
    public Map<String, Object> toMap() {
        Map<String, Object> result = new TreeMap<>();

        result.put("ACDMode", getACDMode());
        result.put("alpha", getAlpha());
        result.put("beta", getBeta());
        result.put("debug", debug);
        result.put("hullDS", getConvexHullDownSampling());
        result.put("maxConcavity", getMaxConcavity());
        result.put("maxVerticesPH", getMaxVerticesPerHull());
        result.put("minVolumePH", getMinVolumePerHull());
        result.put("resolution", getVoxelResolution());
        result.put("PCA", getPCA());
        result.put("planeDS", getPlaneDownSampling());

        return result;
    }

    /**
     * Write selected parameters to the specified OutputStream.
     *
     * @param os the stream to write (not null)
     * @throws IOException from DataOutputStream
     */
    public void toOutputStream(OutputStream os) throws IOException {
        try (DataOutputStream dos = new DataOutputStream(os)) {
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
    }
    // *************************************************************************
    // NativePhysicsObject methods

    /**
     * Create a copy of these parameters.
     *
     * @return a new instance, equivalent to this one
     */
    @Override
    public VHACDParameters clone() {
        try {
            VHACDParameters clone = (VHACDParameters) super.clone();
            long objectId = create();
            clone.reassignNativeId(objectId);

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

        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
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
            int cha = other.getConvexHullApproximation();
            int chds = other.getConvexHullDownSampling();
            double maxConcavity = other.getMaxConcavity();
            double mvph = other.getMinVolumePerHull();
            result = getACDMode() == other.getACDMode()
                    && (Double.compare(getAlpha(), other.getAlpha()) == 0)
                    && (Double.compare(getBeta(), other.getBeta()) == 0)
                    && getConvexHullApproximation() == cha
                    && getConvexHullDownSampling() == chds
                    && getDebugEnabled() == other.getDebugEnabled()
                    && (Double.compare(getMaxConcavity(), maxConcavity) == 0)
                    && getMaxVerticesPerHull() == other.getMaxVerticesPerHull()
                    && (Double.compare(getMinVolumePerHull(), mvph) == 0)
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
     * Generate the hash code for this instance.
     *
     * @return a 32-bit value for use in hashing
     */
    @Override
    public int hashCode() {
        int hash = 5;
        hash = 83 * hash + (debug ? 1 : 0);
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
     * Represent this instance as a String.
     *
     * @return a descriptive string of text (not null, not empty)
     */
    @Override
    public String toString() {
        String result = String.format("VHACDParameters[%n"
                + " %s  alpha=%s  beta=%s  debug=%s  hullDS=%s%n"
                + " maxConcavity=%s  maxVerticesPH=%s  minVolumePH=%s%n"
                + " resolution=%s  PCA=%s  planeDS=%s%n"
                + "]",
                getACDMode(), getAlpha(), getBeta(), getDebugEnabled(),
                getConvexHullDownSampling(), getMaxConcavity(),
                getMaxVerticesPerHull(), getMinVolumePerHull(),
                getVoxelResolution(), getPCA(), getPlaneDownSampling()
        );

        return result;
    }
    // *************************************************************************
    // Java private methods

    /**
     * Free the identified tracked native object. Invoked by reflection.
     *
     * @param objectId the native identifier (not zero)
     */
    private static void freeNativeObject(long objectId) {
        assert objectId != 0L;
        finalizeNative(objectId);
    }

    /**
     * native field: m_convexhullApproximation
     *
     * @return the value
     */
    private int getConvexHullApproximation() {
        long objectId = nativeId();
        return getConvexhullApproximation(objectId);
    }

    /**
     * native field: m_oclAcceleration
     *
     * @return the value
     */
    private int getOclAcceleration() {
        long objectId = nativeId();
        return getOclAcceleration(objectId);
    }

    /**
     * native field: m_convexhullApproximation
     *
     * @param value the desired value (default=true)
     */
    private void setConvexHullApproximation(int value) {
        long objectId = nativeId();
        setConvexhullApproximation(objectId, value);
    }

    /**
     * native field: m_oclAcceleration
     *
     * @param value the desired value (default=true)
     */
    private void setOclAcceleration(int value) {
        long objectId = nativeId();
        setOclAcceleration(objectId, value);
    }
    // *************************************************************************
    // native private methods

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

    native private static void setConvexhullApproximation(
            long objectId, int value);

    native private static void setConvexhullDownsampling(
            long objectId, int precision);

    native private static void setMaxNumVerticesPerCH(long objectId, int limit);

    native private static void setMinVolumePerCH(long objectId, double volume);

    native private static void setMode(long objectId, int ordinal);

    native private static void setOclAcceleration(long objectId, int value);

    native private static void setPca(long objectId, boolean enable);

    native private static void setPlaneDownsampling(
            long objectId, int granularity);

    native private static void setResolution(long objectId, int maxVoxels);
}
