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
package jme3utilities.minie.test.tunings;

import com.jme3.bullet.animation.BoneLink;
import com.jme3.bullet.animation.CenterHeuristic;
import com.jme3.bullet.animation.DynamicAnimControl;
import com.jme3.bullet.animation.LinkConfig;
import com.jme3.bullet.animation.MassHeuristic;
import com.jme3.bullet.animation.RangeOfMotion;
import com.jme3.bullet.animation.ShapeHeuristic;
import com.jme3.math.Vector3f;
import java.util.logging.Logger;

/**
 * A DynamicAnimControl configured specifically for the CesiumMan model.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class CesiumManControl
        extends DynamicAnimControl
        implements Biped {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger4
            = Logger.getLogger(CesiumManControl.class.getName());
    // *************************************************************************
    // constructors

    /**
     * Instantiate a new control tuned for the CesiumMan model.
     */
    public CesiumManControl() {
        super();
        LinkConfig hull = new LinkConfig(1f, MassHeuristic.Density,
                ShapeHeuristic.VertexHull, new Vector3f(1f, 1f, 1f),
                CenterHeuristic.Mean);

        super.setConfig(torsoName, hull);

        // chest
        super.link("Bone_11", hull,
                new RangeOfMotion(0.5f, -1f, 0.7f, -0.7f, 0.7f, -0.7f));
        super.link("Bone_12", hull,
                new RangeOfMotion(0.3f, -0.6f, 0.2f, -0.2f, 0.2f, -0.2f));
        super.link("Bone_19", hull,
                new RangeOfMotion(0.2f, -0.5f, 0.2f, -0.2f, 0.2f, -0.2f));
        super.link("Bone_20", hull,
                new RangeOfMotion(0.5f, 0.5f, 0.2f));

        // right arm
        super.link("Bone_13", hull,
                new RangeOfMotion(1f, -1f, 0.2f, -0.2f, 1f, -0.2f));
        super.link("Bone_14", hull,
                new RangeOfMotion(0f, 0f, 1f, -1f, 2f, 0f));
        super.link("Bone_15", hull,
                new RangeOfMotion(0.6f, 0f, 0.1f));

        // left arm
        super.link("Bone_16", hull,
                new RangeOfMotion(1f, -1f, 0.2f, -0.2f, 0.2f, -1f));
        super.link("Bone_17", hull,
                new RangeOfMotion(0f, 0f, 1f, -1f, 0f, -2f));
        super.link("Bone_18", hull,
                new RangeOfMotion(0.6f, 0f, 0.1f));

        // right leg
        super.link("Bone_3", hull,
                new RangeOfMotion(0.2f, -1f, 0.1f, -0.1f, 0.8f, -0.2f));
        super.link("Bone_4", hull,
                new RangeOfMotion(1.6f, 0f, 0f, 0f, 0f, 0f));
        super.link("Bone_5", hull,
                new RangeOfMotion(0.5f, 0.4f, 0.2f));
        super.link("Bone_6", hull,
                new RangeOfMotion(0.5f, 0.4f, 0.2f));

        // left leg
        super.link("Bone_7", hull,
                new RangeOfMotion(0.2f, -1f, 0.1f, -0.1f, 0.2f, -0.8f));
        super.link("Bone_8", hull,
                new RangeOfMotion(1.6f, 0f, 0f, 0f, 0f, 0f));
        super.link("Bone_9", hull,
                new RangeOfMotion(0.5f, 0.4f, 0.2f));
        super.link("Bone_10", hull,
                new RangeOfMotion(0.5f, 0.4f, 0.2f));
    }
    // *************************************************************************
    // Biped methods

    /**
     * Access the BoneLink that manages the model's left foot.
     *
     * @return the pre-existing instance (not null)
     */
    @Override
    public BoneLink getLeftFoot() {
        return findBoneLink("Bone_10");
    }

    /**
     * Access the BoneLink that manages the model's right foot.
     *
     * @return the pre-existing instance (not null)
     */
    @Override
    public BoneLink getRightFoot() {
        return findBoneLink("Bone_6");
    }
}
