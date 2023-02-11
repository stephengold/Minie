/*
 Copyright (c) 2018-2023 Stephen Gold
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

/**
 * A DynamicAnimControl configured specifically for the MhGame model.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class MhGameControl
        extends DynamicAnimControl
        implements Biped, Face {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger4
            = Logger.getLogger(MhGameControl.class.getName());
    // *************************************************************************
    // constructors

    /**
     * Instantiate a new control tuned for the MhGame model.
     */
    public MhGameControl() {
        super();
        LinkConfig hull = new LinkConfig(1f, MassHeuristic.Density,
                ShapeHeuristic.VertexHull, new Vector3f(1f, 1f, 1f),
                CenterHeuristic.Mean, RotationOrder.XZY);

        // Generate FourSphere shapes for links with > 1,000 mesh vertices.
        LinkConfig simplified = new LinkConfig(1f, MassHeuristic.Density,
                ShapeHeuristic.FourSphere, new Vector3f(1f, 1f, 1f),
                CenterHeuristic.Mean, RotationOrder.XZY);

        super.setConfig(torsoName, hull);
        super.setMainBoneName("pelvis");

        super.link("spine_03", simplified,
                new RangeOfMotion(0.8f, 0.5f, 0.5f)); // 2,047 vertices
        super.link("neck_01", hull,
                new RangeOfMotion(0.5f, 0.5f, 0.3f));
        super.link("head", simplified,
                new RangeOfMotion(0.4f, 0.6f, 0.2f)); // 4,288 vertices

        super.link("clavicle_r", hull,
                new RangeOfMotion(0.4f, 0f, 0.2f));
        super.link("upperarm_r", hull,
                new RangeOfMotion(0.2f, -1f, 0.8f, -0.8f, 1f, -1f));
        super.link("lowerarm_r", hull,
                new RangeOfMotion(0.8f, -1.3f, 0f, 0f, 0f, 0f));
        super.link("hand_r", simplified,
                new RangeOfMotion(0.7f, 0f, 0.2f)); // 1,596 vertices

        super.link("clavicle_l", hull,
                new RangeOfMotion(0.4f, 0f, 0.2f));
        super.link("upperarm_l", hull,
                new RangeOfMotion(0.2f, -1f, 0.8f, -0.8f, 1f, -1f));
        super.link("lowerarm_l", hull,
                new RangeOfMotion(0.8f, -1.3f, 0f, 0f, 0f, 0f));
        super.link("hand_l", simplified,
                new RangeOfMotion(0.7f, 0f, 0.2f)); // 1,596 vertices

        super.link("thigh_r", hull,
                new RangeOfMotion(1f, -0.2f, 0.1f, -0.2f, 0.1f, -0.2f));
        super.link("calf_r", hull,
                new RangeOfMotion(0f, -2.2f, 0f, 0f, 0f, 0f));
        super.link("foot_r", simplified,
                new RangeOfMotion(1f, 0.3f, 0.5f)); // 1,090 vertices

        super.link("thigh_l", hull,
                new RangeOfMotion(1f, -0.2f, 0.2f, -0.1f, 0.2f, -0.1f));
        super.link("calf_l", hull,
                new RangeOfMotion(0f, -2.2f, 0f, 0f, 0f, 0f));
        super.link("foot_l", simplified,
                new RangeOfMotion(1f, 0.3f, 0.5f)); // 1,090 vertices
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
        BoneLink result = findBoneLink("foot_l");
        return result;
    }

    /**
     * Access the BoneLink that manages the model's right foot.
     *
     * @return the pre-existing instance (not null)
     */
    @Override
    public BoneLink getRightFoot() {
        BoneLink result = findBoneLink("foot_r");
        return result;
    }
    // *************************************************************************
    // Face methods

    /**
     * Read the vertex spec for the center of the model's face. This is
     * typically on the bridge of the nose, halfway between the pupils.
     *
     * @return the vertex specification (not null, not empty)
     * @see com.jme3.bullet.animation.DynamicAnimControl#findManagerForVertex(
     * java.lang.String, com.jme3.math.Vector3f, com.jme3.math.Vector3f)
     */
    @Override
    public String faceCenterSpec() {
        return "9313/male_generic";
    }

    /**
     * Copy the direction the model's head is facing.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a direction vector (unit vector in the physics link's local
     * coordinates, either storeResult or a new vector)
     */
    @Override
    public Vector3f faceDirection(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        result.set(0f, 0f, 1f);
        return result;
    }
}
