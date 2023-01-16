/*
 * Copyright (c) 2009-2023 jMonkeyEngine
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of 'jMonkeyEngine' nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package com.jme3.bullet.objects.infos;

import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.util.clone.Cloner;
import com.jme3.util.clone.JmeCloneable;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * Provide access to fields of the native btSoftBody::Config struct. Soft bodies
 * are one-to-one with config instances.
 *
 * @author Stephen Gold sgold@sonic.net
 *
 * Based on PhysicsSoftBody by dokthar.
 */
public class SoftBodyConfig implements JmeCloneable, Savable {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(SoftBodyConfig.class.getName());
    /**
     * field names for serialization
     */
    final private static String tagBody = "body";
    final private static String tagClusterIterations = "clusterIterations";
    final private static String tagCollisionFlags = "collisionFlags";
    final private static String tagDriftIterations = "driftIterations";
    final private static String tagPositionIterations = "positionIterations";
    final private static String tagVelocityIterations = "velocityIterations";
    // *************************************************************************
    // fields

    /**
     * corresponding soft body
     */
    private PhysicsSoftBody body = null;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected SoftBodyConfig() {
    }

    /**
     * Instantiate a config with the default properties.
     *
     * @param body the corresponding soft body (not null)
     */
    public SoftBodyConfig(PhysicsSoftBody body) {
        assert body != null;
        this.body = body;
    }
    // *********************************************************************
    // new methods exposed

    /**
     * Read the aerodynamics model.
     *
     * @return an enum value (not null)
     */
    public Aero aerodynamics() {
        long bodyId = body.nativeId();
        int ordinal = getAeroModel(bodyId);
        Aero result = Aero.values()[ordinal];

        return result;
    }

    /**
     * Read the number of cluster-solver iterations (native field: citerations).
     *
     * @return the iteration count (&ge;0)
     */
    public int clusterIterations() {
        long bodyId = body.nativeId();
        int result = getClusterIterations(bodyId);

        return result;
    }

    /**
     * Read the collisions flags (native field: collisions). Flags are defined
     * in {@link ConfigFlag}.
     *
     * @return the flags that are set, ORed together
     */
    public int collisionFlags() {
        long bodyId = body.nativeId();
        int result = getCollisionsFlags(bodyId);

        return result;
    }

    /**
     * Copy all parameter values from the specified config.
     *
     * @param source the config to copy from (not null, unaffected)
     */
    public void copyAll(SoftBodyConfig source) {
        long destId = body.nativeId();
        long sourceId = source.body.nativeId();
        copyValues(destId, sourceId);
    }

    /**
     * Read the number of drift-solver iterations (native field: diterations).
     *
     * @return the iteration count (&ge;0)
     */
    public int driftIterations() {
        long bodyId = body.nativeId();
        int result = getDriftIterations(bodyId);

        return result;
    }

    /**
     * Read the specified parameter.
     *
     * @param parameter which parameter to read (not null)
     * @return the parameter value
     */
    public float get(Sbcp parameter) {
        Validate.nonNull(parameter, "parameter");

        long bodyId = body.nativeId();

        float result;
        switch (parameter) {
            case AnchorHardness:
                result = getAnchorsHardness(bodyId);
                break;
            case ClusterKineticHardness:
                result = getClusterKineticHardness(bodyId);
                break;
            case ClusterKineticSplit:
                result = getClusterKineticImpulseSplitCoef(bodyId);
                break;
            case ClusterRigidHardness:
                result = getClusterRigidHardness(bodyId);
                break;
            case ClusterRigidSplit:
                result = getClusterRigidImpulseSplitCoef(bodyId);
                break;
            case ClusterSoftHardness:
                result = getClusterSoftHardness(bodyId);
                break;
            case ClusterSoftSplit:
                result = getClusterSoftImpulseSplitCoef(bodyId);
                break;
            case Damping:
                result = getDampingCoef(bodyId);
                break;
            case Drag:
                result = getDragCoef(bodyId);
                break;
            case DynamicFriction:
                result = getDynamicFrictionCoef(bodyId);
                break;
            case KineticHardness:
                result = getKineticContactsHardness(bodyId);
                break;
            case Lift:
                result = getLiftCoef(bodyId);
                break;
            case MaxVolumeRatio:
                result = getMaximumVolumeRatio(bodyId);
                break;
            case PoseMatching:
                result = getPoseMatchingCoef(bodyId);
                break;
            case Pressure:
                result = getPressureCoef(bodyId);
                break;
            case RigidHardness:
                result = getRigidContactsHardness(bodyId);
                break;
            case SoftHardness:
                result = getSoftContactsHardness(bodyId);
                break;
            case TimeScale:
                result = getTimeScale(bodyId);
                break;
            case VelocityCorrection:
                result = getVelocitiesCorrectionFactor(bodyId);
                break;
            case VolumeConservation:
                result = getVolumeConservationCoef(bodyId);
                break;
            default:
                throw new IllegalArgumentException(toString());
        }

        assert parameter.canSet(result) : result;
        return result;
    }

    /**
     * Read the number of position-solver iterations (native field:
     * piterations).
     *
     * @return the iteration count (&ge;0)
     */
    public int positionIterations() {
        long bodyId = body.nativeId();
        int result = getPositionIterations(bodyId);

        return result;
    }

    /**
     * Alter the specified parameter.
     *
     * @param parameter which parameter to set (not null)
     * @param desiredValue the desired parameter value
     */
    public void set(Sbcp parameter, float desiredValue) {
        if (!parameter.canSet(desiredValue)) {
            String message = String.format("%s cannot be set to %f",
                    parameter, desiredValue);
            throw new IllegalArgumentException(message);
        }

        long bodyId = body.nativeId();

        switch (parameter) {
            case AnchorHardness:
                setAnchorsHardness(bodyId, desiredValue);
                break;
            case ClusterKineticHardness:
                setClusterKineticHardness(bodyId, desiredValue);
                break;
            case ClusterKineticSplit:
                setClusterKineticImpulseSplitCoef(bodyId, desiredValue);
                break;
            case ClusterRigidHardness:
                setClusterRigidHardness(bodyId, desiredValue);
                break;
            case ClusterRigidSplit:
                setClusterRigidImpulseSplitCoef(bodyId, desiredValue);
                break;
            case ClusterSoftHardness:
                setClusterSoftHardness(bodyId, desiredValue);
                break;
            case ClusterSoftSplit:
                setClusterSoftImpulseSplitCoef(bodyId, desiredValue);
                break;
            case Damping:
                setDampingCoef(bodyId, desiredValue);
                break;
            case Drag:
                setDragCoef(bodyId, desiredValue);
                break;
            case DynamicFriction:
                setDynamicFrictionCoef(bodyId, desiredValue);
                break;
            case Lift:
                setLiftCoef(bodyId, desiredValue);
                break;
            case KineticHardness:
                setKineticContactsHardness(bodyId, desiredValue);
                break;
            case MaxVolumeRatio:
                setMaximumVolumeRatio(bodyId, desiredValue);
                break;
            case PoseMatching:
                setPoseMatchingCoef(bodyId, desiredValue);
                break;
            case Pressure:
                setPressureCoef(bodyId, desiredValue);
                break;
            case SoftHardness:
                setSoftContactsHardness(bodyId, desiredValue);
                break;
            case RigidHardness:
                setRigidContactsHardness(bodyId, desiredValue);
                break;
            case TimeScale:
                setTimeScale(bodyId, desiredValue);
                break;
            case VelocityCorrection:
                setVelocitiesCorrectionFactor(bodyId, desiredValue);
                break;
            case VolumeConservation:
                setVolumeConservationCoef(bodyId, desiredValue);
                break;
            default:
                throw new IllegalArgumentException(toString());
        }
    }

    /**
     * Alter the aerodynamics model.
     *
     * @param model the desired aerodynamics model (not null, default=V_Point)
     */
    public void setAerodynamics(Aero model) {
        long bodyId = body.nativeId();
        int ordinal = model.ordinal();
        setAeroModel(bodyId, ordinal);
    }

    /**
     * Alter the number of cluster-solver iterations (native field:
     * citerations).
     *
     * @param numIterations the desired number of iterations (&ge;0, default=4)
     */
    public void setClusterIterations(int numIterations) {
        long bodyId = body.nativeId();
        setClusterIterations(bodyId, numIterations);
    }

    /**
     * Alter the collision flags (default=SDF_RS). Flag values are defined in
     * {@link ConfigFlag}.
     *
     * @param flag the first flag to set, or 0x0 to clear all flags
     * @param additionalFlags ... additional flags to set. Flags are ORed
     * together.
     */
    public void setCollisionFlags(int flag, int... additionalFlags) {
        int combinedFlags = flag;
        for (int additionalFlag : additionalFlags) {
            combinedFlags |= additionalFlag;
        }

        long bodyId = body.nativeId();
        setCollisionsFlags(bodyId, combinedFlags);
    }

    /**
     * Alter the number of drift-solver iterations (native field: diterations).
     *
     * @param numIterations the desired number of iterations (&ge;0, default=0)
     */
    public void setDriftIterations(int numIterations) {
        long bodyId = body.nativeId();
        setDriftIterations(bodyId, numIterations);
    }

    /**
     * Alter the number of position-solver iterations (native field:
     * piterations).
     *
     * @param numIterations the desired number of iterations (&ge;0, default=1)
     */
    public void setPositionIterations(int numIterations) {
        long bodyId = body.nativeId();
        setPositionIterations(bodyId, numIterations);
    }

    /**
     * Alter the number of velocity-solver iterations (native field:
     * viterations).
     *
     * @param numIterations the desired number of iterations (&ge;0, default=0)
     */
    public void setVelocityIterations(int numIterations) {
        long bodyId = body.nativeId();
        setVelocitiesIterations(bodyId, numIterations);
    }

    /**
     * Read the number of velocity-solver iterations (native field:
     * viterations).
     *
     * @return the iteration count (&ge;0)
     */
    public int velocityIterations() {
        long bodyId = body.nativeId();
        int result = getVelocitiesIterations(bodyId);

        return result;
    }
    // *************************************************************************
    // JmeCloneable methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned config into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this config (not null)
     * @param original the instance from which this config was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        body = cloner.clone(body);

        SoftBodyConfig oldConfig = (SoftBodyConfig) original;
        copyAll(oldConfig);
    }

    /**
     * Create a shallow clone for the JME cloner.
     *
     * @return a new instance
     */
    @Override
    public SoftBodyConfig jmeClone() {
        try {
            SoftBodyConfig clone = (SoftBodyConfig) clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }
    // *************************************************************************
    // Savable methods

    /**
     * De-serialize this config from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        assert body == null;

        InputCapsule capsule = importer.getCapsule(this);
        body = (PhysicsSoftBody) capsule.readSavable(tagBody, null);

        setClusterIterations(capsule.readInt(tagClusterIterations, 4));
        setCollisionFlags(capsule.readInt(tagCollisionFlags, ConfigFlag.CL_RS));
        setDriftIterations(capsule.readInt(tagDriftIterations, 0));
        setPositionIterations(capsule.readInt(tagPositionIterations, 1));
        setVelocityIterations(capsule.readInt(tagVelocityIterations, 0));

        for (Sbcp sbcp : Sbcp.values()) {
            String tag = sbcp.toString();
            float defValue = sbcp.defValue();
            float readValue = capsule.readFloat(tag, defValue);
            set(sbcp, readValue);
        }
    }

    /**
     * Serialize this config to the specified exporter, for example when saving
     * to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(body, tagBody, null);
        capsule.write(clusterIterations(), tagClusterIterations, 4);
        capsule.write(collisionFlags(), tagCollisionFlags, ConfigFlag.CL_RS);
        capsule.write(driftIterations(), tagDriftIterations, 0);
        capsule.write(positionIterations(), tagPositionIterations, 1);
        capsule.write(velocityIterations(), tagVelocityIterations, 0);

        for (Sbcp sbcp : Sbcp.values()) {
            float value = get(sbcp);
            String tag = sbcp.toString();
            float defValue = sbcp.defValue();
            capsule.write(value, tag, defValue);
        }
    }
    // *************************************************************************
    // native private methods

    native private static void copyValues(long destId, long sourceId);

    native private static int getAeroModel(long bodyId);

    native private static float getAnchorsHardness(long bodyId);

    native private static int getClusterIterations(long bodyId);

    native private static float getClusterKineticHardness(long bodyId);

    native private static float getClusterKineticImpulseSplitCoef(long bodyId);

    native private static float getClusterRigidHardness(long bodyId);

    native private static float getClusterRigidImpulseSplitCoef(long bodyId);

    native private static float getClusterSoftHardness(long bodyId);

    native private static float getClusterSoftImpulseSplitCoef(long bodyId);

    native private static int getCollisionsFlags(long bodyId);

    native private static float getDampingCoef(long bodyId);

    native private static float getDragCoef(long bodyId);

    native private static int getDriftIterations(long bodyId);

    native private static float getDynamicFrictionCoef(long bodyId);

    native private static float getKineticContactsHardness(long bodyId);

    native private static float getLiftCoef(long bodyId);

    native private static float getMaximumVolumeRatio(long bodyId);

    native private static float getPoseMatchingCoef(long bodyId);

    native private static int getPositionIterations(long bodyId);

    native private static float getPressureCoef(long bodyId);

    native private static float getRigidContactsHardness(long bodyId);

    native private static float getSoftContactsHardness(long bodyId);

    native private static float getTimeScale(long bodyId);

    native private static float getVelocitiesCorrectionFactor(long bodyId);

    native private static int getVelocitiesIterations(long bodyId);

    native private static float getVolumeConservationCoef(long bodyId);

    native private static void setAeroModel(long bodyId, int aeroModel);

    native private static void setAnchorsHardness(long bodyId, float hardness);

    native private static void
            setClusterIterations(long bodyId, int numIterations);

    native private static void
            setClusterKineticHardness(long bodyId, float hardness);

    native private static void
            setClusterKineticImpulseSplitCoef(long bodyId, float coefficient);

    native private static void
            setClusterRigidHardness(long bodyId, float hardness);

    native private static void
            setClusterRigidImpulseSplitCoef(long bodyId, float coefficient);

    native private static void
            setClusterSoftHardness(long bodyId, float hardness);

    native private static void
            setClusterSoftImpulseSplitCoef(long bodyId, float coef);

    native private static void setCollisionsFlags(long bodyId, int flags);

    native private static void setDampingCoef(long bodyId, float coefficient);

    native private static void setDragCoef(long bodyId, float coefficient);

    native private static void
            setDriftIterations(long bodyId, int numIterations);

    native private static void
            setDynamicFrictionCoef(long bodyId, float coefficient);

    native private static void
            setKineticContactsHardness(long bodyId, float hardness);

    native private static void setLiftCoef(long bodyId, float coefficient);

    native private static void setMaximumVolumeRatio(long bodyId, float ratio);

    native private static void
            setPoseMatchingCoef(long bodyId, float coefficient);

    native private static void
            setPositionIterations(long bodyId, int numIterations);

    native private static void setPressureCoef(long bodyId, float coefficient);

    native private static void
            setRigidContactsHardness(long bodyId, float hardness);

    native private static void
            setSoftContactsHardness(long bodyId, float hardness);

    native private static void setTimeScale(long bodyId, float scale);

    native private static void
            setVelocitiesCorrectionFactor(long bodyId, float factor);

    native private static void
            setVelocitiesIterations(long bodyId, int numIterations);

    native private static void
            setVolumeConservationCoef(long bodyId, float coefficient);
}
