/*
 Copyright (c) 2019-2022, Stephen Gold
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
package jme3utilities.minie.wizard;

import com.jme3.bullet.animation.DacConfiguration;
import java.util.logging.Logger;

/**
 * The value of an item in the TreeBox of the "links" screen.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class LinkValue {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(LinkValue.class.getName());
    // *************************************************************************
    // fields

    /**
     * name of the bone/torso (not null)
     */
    final private String boneName;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a value from a bone name or torsoName.
     *
     * @param boneName the name of the bone/torso (not null)
     */
    LinkValue(String boneName) {
        assert boneName != null;
        this.boneName = boneName;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Read the name of the bone/torso.
     *
     * @return the bone name (not null)
     */
    String boneName() {
        assert boneName != null;
        return boneName;
    }
    // *************************************************************************
    // Object methods

    /**
     * Convert the value to text for TreeBox display.
     *
     * @return display text for the item (not null, not empty)
     */
    @Override
    public String toString() {
        String result;
        if (boneName.equals(DacConfiguration.torsoName)) {
            result = "Torso:";
        } else {
            result = "Bone:" + boneName;
        }

        Model model = DacWizard.getModel();
        int numBones = model.countManagedBones(boneName);
        int numVertices = model.countVertices(boneName);
        result += String.format(
                " (%d bone%s, %d vert%s)", numBones, (numBones == 1) ? "" : "s",
                numVertices, (numVertices == 1) ? "ex" : "ices");

        return result;
    }
}
