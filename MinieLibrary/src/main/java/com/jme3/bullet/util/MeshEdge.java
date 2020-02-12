/*
 * Copyright (c) 2009-2016 jMonkeyEngine
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
package com.jme3.bullet.util;

import java.util.logging.Logger;

/**
 * Represent an edge in a mesh.
 *
 * @author dokthar
 */
class MeshEdge {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(MeshEdge.class.getName());
    // *************************************************************************
    // fields

    /**
     * index of the first mesh vertex (&ge;0, &lt;index2)
     */
    private final int index1;
    /**
     * index of the 2nd mesh vertex (&ge;0, &gt;index1)
     */
    private final int index2;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a mesh edge from a pair of vertex indices.
     */
    MeshEdge(int indexA, int indexB) {
        assert indexA >= 0 : indexA;
        assert indexB >= 0 : indexB;
        assert indexA != indexB;

        if (indexA < indexB) {
            index1 = indexA;
            index2 = indexB;
        } else {
            index2 = indexA;
            index1 = indexB;
        }
        assert index1 < index2;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Access the first (smaller) index.
     *
     * @return the vertex index (&ge;0)
     */
    int index1() {
        assert index1 >= 0 : index1;
        return index1;
    }

    /**
     * Access the 2nd (larger) index.
     *
     * @return the vertex index (&ge;0)
     */
    int index2() {
        assert index2 >= 0 : index2;
        return index2;
    }
    // *************************************************************************
    // Object methods

    /**
     * Test for exact equivalence with another Object.
     *
     * @param otherObject the object to compare to (may be null, unaffected)
     * @return true if the objects are equivalent, otherwise false
     */
    @Override
    public boolean equals(Object otherObject) {
        boolean result;
        if (otherObject == this) {
            result = true;
        } else if (otherObject != null
                && otherObject.getClass() == getClass()) {
            MeshEdge otherEdge = (MeshEdge) otherObject;
            result = (otherEdge.index1 == index1)
                    && (otherEdge.index2 == index2);
        } else {
            result = false;
        }

        return result;
    }

    /**
     * Generate the hash code for this MeshEdge.
     *
     * @return the value to use for hashing
     */
    @Override
    public int hashCode() {
        int result = 7;
        result = 23 * result + index1;
        result = 23 * result + index2;

        return result;
    }
}
