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
package com.jme3.bullet;

import java.util.ArrayList;
import java.util.Collection;
import java.util.logging.Logger;

/**
 * Named flags for use with a ray test. See
 * https://github.com/bulletphysics/bullet3/blob/master/src/BulletCollision/NarrowPhaseCollision/btRaycastCallback.h
 *
 * @author Stephen Gold sgold@sonic.net
 * @see com.jme3.bullet.CollisionSpace#getRayTestFlags()
 */
final public class RayTestFlag {
    // *************************************************************************
    // constants and loggers

    /**
     * filter back faces (native value: kF_FilterBackfaces)
     */
    final public static int FilterBackfaces = 0x1;
    /**
     * when a ray hits a back-facing triangle, don't reverse the face normal
     * (native value: kF_KeepUnflippedNormal)
     */
    final public static int KeepUnflippedNormal = 0x2;
    /**
     * use the fast/approximate algorithm for ray-versus-convex intersection,
     * which is the default if no flags are set (native value:
     * kF_UseSubSimplexConvexCastRaytest)
     */
    final public static int SubSimplexRaytest = 0x4;
    /**
     * use the GJK algorithm for ray-versus-convex intersection (native value:
     * kF_UseGjkConvexCastRaytest)
     */
    final public static int GjkRaytest = 0x8;
    /**
     * disable the heightfield raycast accelerator (native value:
     * kF_DisableHeightfieldAccelerator)
     */
    final public static int DisableHeightfieldAccelerator = 0x10;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(RayTestFlag.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private RayTestFlag() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Generate a textual description of the specified flags.
     *
     * @param flags the ray-test flags to describe
     * @return description (not null, may be empty)
     */
    public static String describe(int flags) {
        Collection<String> flagList = new ArrayList<>(5);
        if ((flags & FilterBackfaces) != 0x0) {
            flagList.add("FilterBackfaces");
        }
        if ((flags & KeepUnflippedNormal) != 0x0) {
            flagList.add("KeepUnflippedNormal");
        }
        if ((flags & SubSimplexRaytest) != 0x0) {
            flagList.add("SubSimplex");
        }
        if ((flags & GjkRaytest) != 0x0) {
            flagList.add("Gjk");
        }
        if ((flags & DisableHeightfieldAccelerator) == 0x0) {
            flagList.add("HeightfieldAccel");
        }

        StringBuilder result = new StringBuilder(40);
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
