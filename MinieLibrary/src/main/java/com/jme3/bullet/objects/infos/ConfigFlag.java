/*
 * Copyright (c) 2019-2022 jMonkeyEngine
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

import java.util.ArrayList;
import java.util.Collection;
import java.util.logging.Logger;

/**
 * Named collision flags for use with a SoftBodyConfig.
 *
 * @author Stephen Gold sgold@sonic.net
 * @see SoftBodyConfig#collisionFlags()
 */
final public class ConfigFlag {
    // *************************************************************************
    // constants and loggers

    /**
     * enable the SDF-based handler for rigid-versus-soft collisions
     */
    final public static int SDF_RS = 0x1;
    /**
     * enable the Cluster-versus-Convex handler for rigid-versus-soft collisions
     */
    final public static int CL_RS = 0x2;
    /**
     * enable the SDF-based handler for rigid-versus-deformable collisions
     */
    final public static int SDF_RD = 0x4;
    /**
     * enable the Node-versus-Face handler for soft-versus-soft collisions
     */
    final public static int VF_SS = 0x10;
    /**
     * enable the Cluster-versus-Cluster handler for soft-versus-soft collisions
     */
    final public static int CL_SS = 0x20;
    /**
     * enable self collisions for clusters
     */
    final public static int CL_SELF = 0x40;
    /**
     * enable the Vertex-versus-Face handler for soft-versus-soft collisions
     */
    final public static int VF_DD = 0x80;
    /**
     * enable the GJK-based handler for rigid-versus-deformable face collisions
     */
    final public static int SDF_RDF = 0x100;
    /**
     * enable the GJK-based handler for multibody-versus-deformable face
     * collisions
     */
    final public static int SDF_MDF = 0x200;
    /**
     * enable the SDF-based handler for rigid-versus-deformable node collisions
     */
    final public static int SDF_RDN = 0x400;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(ConfigFlag.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private ConfigFlag() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Generate a textual description of the specified flags.
     *
     * @param flags the config flags to describe
     * @return description (not null, may be empty)
     */
    public static String describe(int flags) {
        Collection<String> flagList = new ArrayList<>(5);
        if ((flags & SDF_RS) != 0x0) {
            flagList.add("SDF_RS");
        }
        if ((flags & CL_RS) != 0x0) {
            flagList.add("CL_RS");
        }
        if ((flags & SDF_RD) != 0x0) {
            flagList.add("SDF_RD");
        }
        if ((flags & VF_SS) != 0x0) {
            flagList.add("VF_SS");
        }
        if ((flags & CL_SS) != 0x0) {
            flagList.add("CL_SS");
        }
        if ((flags & CL_SELF) != 0x0) {
            flagList.add("CL_SELF");
        }
        if ((flags & VF_DD) != 0x0) {
            flagList.add("VF_DD");
        }
        if ((flags & SDF_RDF) != 0x0) {
            flagList.add("SDF_RDF");
        }
        if ((flags & SDF_MDF) != 0x0) {
            flagList.add("SDF_MDF");
        }
        if ((flags & SDF_RDN) != 0x0) {
            flagList.add("SDF_RDN");
        }

        StringBuilder result = new StringBuilder(60);
        boolean addSeparators = false;
        for (String flagName : flagList) {
            if (addSeparators) {
                result.append(',');
            } else {
                addSeparators = true;
            }
            result.append(flagName);
        }

        return result.toString();
    }
}
