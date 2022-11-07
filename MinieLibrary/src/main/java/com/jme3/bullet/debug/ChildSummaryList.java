/*
 * Copyright (c) 2020-2022 jMonkeyEngine
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
package com.jme3.bullet.debug;

import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.infos.ChildCollisionShape;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Logger;

/**
 * A concise summary of the children of a CompoundCollisionShape, used to detect
 * changes that might affect debug visualization.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class ChildSummaryList {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final static Logger logger
            = Logger.getLogger(ChildSummaryList.class.getName());
    // *************************************************************************
    // fields

    /**
     * list of child summaries: right-padded with invalid entries, the list can
     * only get longer, never shorter
     */
    final private List<ChildSummary> list = new ArrayList<>(8);
    // *************************************************************************
    // new methods exposed

    /**
     * Determine the number of valid child summaries in this list.
     *
     * @return the count (&ge;0)
     */
    int countValid() {
        int result = 0;
        while (result < list.size()) {
            ChildSummary summary = list.get(result);
            if (!summary.isValid()) {
                break;
            }
            ++result;
        }

        return result;
    }

    /**
     * Configure based on the specified CompoundCollisionShape.
     *
     * @param compound (not null, unaffected)
     */
    void update(CompoundCollisionShape compound) {
        ChildCollisionShape[] children = compound.listChildren();
        int numChildren = children.length;

        // Enlarge the list, padding with invalid entries.
        while (numChildren > list.size()) {
            list.add(new ChildSummary());
        }

        for (int childIndex = 0; childIndex < list.size(); ++childIndex) {
            ChildSummary summary = list.get(childIndex);
            if (childIndex < numChildren) {
                ChildCollisionShape child = children[childIndex];
                summary.update(child);
            } else {
                summary.update(null);
            }
        }
    }
    // *************************************************************************
    // Object methods

    /**
     * Test for exact equivalence with another object.
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
            int numValid = countValid();
            ChildSummaryList other = (ChildSummaryList) otherObject;
            if (numValid != other.countValid()) {
                result = false;
            } else {
                result = true;
                for (int childIndex = 0; childIndex < numValid; ++childIndex) {
                    ChildSummary summary = list.get(childIndex);
                    ChildSummary otherSummary = other.list.get(childIndex);
                    if (!summary.equals(otherSummary)) {
                        result = false;
                        break;
                    }
                }
            }

        } else {
            result = false;
        }

        return result;
    }

    /**
     * Generate the hash code for this list.
     *
     * @return a 32-bit value for use in hashing
     */
    @Override
    public int hashCode() {
        int hash = 13;
        int numValid = countValid();
        for (int childIndex = 0; childIndex < numValid; ++childIndex) {
            ChildSummary summary = list.get(childIndex);
            hash = 47 * hash + summary.hashCode();
        }

        return hash;
    }
}
