/*
 * Copyright (c) 2019-2021 jMonkeyEngine
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

import com.jme3.math.Matrix3f;
import com.jme3.math.Vector3f;
import jme3utilities.Validate;

/**
 * Enumerate the orders in which axis rotations can be applied (native enum:
 * RotateOrder).
 *
 * @author Stephen Gold sgold@sonic.net
 */
public enum RotationOrder {
    // *************************************************************************
    // values

    /**
     * X then Y then Z (native name: RO_XYZ)
     */
    XYZ,
    /**
     * X then Z then Y (native name: RO_XZY)
     */
    XZY,
    /**
     * Y then X then Z (native name: RO_YXZ)
     */
    YXZ,
    /**
     * Y then Z then X (native name: RO_YZX)
     */
    YZX,
    /**
     * Z then X then Y (native name: RO_ZXY)
     */
    ZXY,
    /**
     * Z then Y then X (native name: RO_ZYX)
     */
    ZYX;
    // *************************************************************************
    // new methods exposed

    /**
     * Convert a rotation matrix to Euler angles for this RotationOrder.
     *
     * @param rotMatrix the matrix to convert (not null, unaffected)
     * @param storeResult storage for the result (modified if not null)
     * @return the Euler angles (either storeResult or a new vector, not null)
     */
    public Vector3f matrixToEuler(Matrix3f rotMatrix, Vector3f storeResult) {
        Validate.nonNull(rotMatrix, "rot matrix");
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        int rotOrder = ordinal();
        /*
         * matrixToEuler() returns false if the solution is not unique,
         * but we ignore that information
         */
        matrixToEuler(rotOrder, rotMatrix, result);

        return result;
    }
    // *************************************************************************
    // native private methods

    native private static boolean matrixToEuler(
            int rotOrder, Matrix3f rotMatrix, Vector3f storeVector);
}
