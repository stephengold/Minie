/*
 Copyright (c) 2022-2023 Stephen Gold
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

import com.jme3.bullet.RotationOrder;
import com.jme3.bullet.animation.BoneLink;
import com.jme3.bullet.animation.CenterHeuristic;
import com.jme3.bullet.animation.DynamicAnimControl;
import com.jme3.bullet.animation.LinkConfig;
import com.jme3.bullet.animation.MassHeuristic;
import com.jme3.bullet.animation.RangeOfMotion;
import com.jme3.bullet.animation.ShapeHeuristic;
import com.jme3.math.Vector3f;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A DynamicAnimControl configured specifically for models with the Mixamo rig.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class MixamoControl
        extends DynamicAnimControl
        implements Biped {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger4
            = Logger.getLogger(BaseMeshControl.class.getName());
    /**
     * local copy of {@link com.jme3.math.Vector3f#UNIT_XYZ}
     */
    final private static Vector3f scaleIdentity = new Vector3f(1f, 1f, 1f);
    // *************************************************************************
    // constructors

    /**
     * Instantiate a new control tuned for a model with the Mixamo rig.
     *
     * @param density the desired mass per unit volume (&gt;0)
     */
    public MixamoControl(float density) {
        super();
        Validate.positive(density, "density");

        setIgnoredHops(2);

        LinkConfig fourSphere = new LinkConfig(density, MassHeuristic.Density,
                ShapeHeuristic.FourSphere, scaleIdentity,
                CenterHeuristic.Mean, RotationOrder.XZY);
        LinkConfig twoSphere = new LinkConfig(density, MassHeuristic.Density,
                ShapeHeuristic.TwoSphere, scaleIdentity,
                CenterHeuristic.Mean, RotationOrder.XZY);
        LinkConfig vertexHull = new LinkConfig(density, MassHeuristic.Density,
                ShapeHeuristic.VertexHull, scaleIdentity,
                CenterHeuristic.Mean, RotationOrder.XZY);

        // trunk, neck, and head
        super.setConfig(torsoName, fourSphere);
        super.link("mixamorig:Spine", vertexHull,
                new RangeOfMotion(0.2f, -1f, 0.1f, -0.1f, 0.1f, -0.1f));
        super.link("mixamorig:Spine1", vertexHull,
                new RangeOfMotion(0.2f, 0.3f, 0.3f));
        super.link("mixamorig:Spine2", vertexHull,
                new RangeOfMotion(0.4f, 0.6f, 0.5f));
        super.link("mixamorig:Neck", vertexHull,
                new RangeOfMotion(0.6f, -0.3f, 0.6f, -0.6f, 0.4f, -0.4f));
        super.link("mixamorig:Head", fourSphere,
                new RangeOfMotion(0.6f, -0.3f, 0.6f, -0.6f, 0.7f, -0.7f));

        // left arm
        super.link("mixamorig:LeftShoulder", vertexHull,
                new RangeOfMotion(0.4f, -0.2f, 0f, 0f, 0.6f, -0.3f));
        super.link("mixamorig:LeftArm", vertexHull,
                new RangeOfMotion(0.2f, -1.5f, 0.5f, -0.5f, 1f, -1.6f));
        super.link("mixamorig:LeftForeArm", twoSphere,
                new RangeOfMotion(0f, -2f, 1f, -1f, 0f, 0f));
        super.link("mixamorig:LeftHand", fourSphere,
                new RangeOfMotion(0.9f, 0f, 0.3f));

        // right arm
        super.link("mixamorig:RightShoulder", vertexHull,
                new RangeOfMotion(0.4f, -0.2f, 0f, 0f, 0.6f, -0.3f));
        super.link("mixamorig:RightArm", vertexHull,
                new RangeOfMotion(0.2f, -1.5f, 0.5f, -0.5f, 1.6f, -1f));
        super.link("mixamorig:RightForeArm", twoSphere,
                new RangeOfMotion(0f, -2f, 1f, -1f, 0f, 0f));
        super.link("mixamorig:RightHand", fourSphere,
                new RangeOfMotion(0.9f, 0f, 0.3f));

        // left leg
        super.link("mixamorig:LeftUpLeg", twoSphere,
                new RangeOfMotion(0.2f, -1.1f, 0.4f, -0.4f, 0.2f, -0.4f));
        super.link("mixamorig:LeftLeg", fourSphere,
                new RangeOfMotion(2f, 0f, 0.1f, -0.1f, 0f, 0f));
        super.link("mixamorig:LeftFoot", vertexHull,
                new RangeOfMotion(0.6f, -0.4f, 0.4f, -0.4f, 0.4f, -0.4f));

        // right leg
        super.link("mixamorig:RightUpLeg", twoSphere,
                new RangeOfMotion(0.2f, -1.1f, 0.4f, -0.4f, 0.4f, -0.2f));
        super.link("mixamorig:RightLeg", fourSphere,
                new RangeOfMotion(2f, 0f, 0.1f, -0.1f, 0f, 0f));
        super.link("mixamorig:RightFoot", vertexHull,
                new RangeOfMotion(0.6f, -0.4f, 0.4f, -0.4f, 0.4f, -0.4f));
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
        BoneLink result = findBoneLink("mixamorig:LeftFoot");
        return result;
    }

    /**
     * Access the BoneLink that manages the model's right foot.
     *
     * @return the pre-existing instance (not null)
     */
    @Override
    public BoneLink getRightFoot() {
        BoneLink result = findBoneLink("mixamorig:RightFoot");
        return result;
    }
}
