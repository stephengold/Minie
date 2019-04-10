/*
 Copyright (c) 2019, Stephen Gold
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

import java.util.logging.Logger;
import jme3utilities.MyString;

/**
 * The value of an item in the TreeBox of the "bones" screen.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class BoneValue {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final static Logger logger = Logger.getLogger(BoneValue.class.getName());
    // *************************************************************************
    // fields

    /**
     * index of the bone (&ge;0)
     */
    final private int boneIndex;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a value from a bone index.
     *
     * @param boneIndex the index of the bone (&ge;0)
     */
    BoneValue(int boneIndex) {
        assert boneIndex >= 0 : boneIndex;
        this.boneIndex = boneIndex;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Read the bone index.
     *
     * @return the bone index (&ge;0)
     */
    int boneIndex() {
        assert boneIndex >= 0 : boneIndex;
        return boneIndex;
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
        Model model = DacWizard.getModel();
        String boneName = model.boneName(boneIndex);
        int numTracks = model.countTracks(boneIndex);
        String influence = model.describeBoneInfluence(boneIndex);
        String result = String.format("%s: used in %d track%s, %s",
                MyString.quote(boneName), numTracks, numTracks == 1 ? "" : "s",
                influence);

        return result;
    }
}
