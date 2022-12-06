/*
 Copyright (c) 2018-2022, Stephen Gold
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
 * A DynamicAnimControl configured specifically for the Ninja model.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class NinjaControl
        extends DynamicAnimControl
        implements Biped, Face {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger4
            = Logger.getLogger(NinjaControl.class.getName());
    // *************************************************************************
    // constructors

    /**
     * Instantiate a new control tuned for the Ninja model.
     */
    public NinjaControl() {
        super();
        LinkConfig hull = new LinkConfig(1f, MassHeuristic.Density,
                ShapeHeuristic.VertexHull, new Vector3f(1f, 1f, 1f),
                CenterHeuristic.Mean, RotationOrder.XZY);

        super.setConfig(torsoName, hull);
        super.setMainBoneName("Joint2");

        // chest
        super.link("Joint4", hull,
                new RangeOfMotion(1f, -0.5f, 0.8f, -0.8f, 0.3f, -0.3f));
        super.link("Joint6", hull,
                new RangeOfMotion(0.5f, 0.8f, 0f));

        // head
        super.link("Joint7", hull,
                new RangeOfMotion(0.8f, -0.5f, 1f, -1f, 0.8f, -0.8f));

        // right arm and katana
        super.link("Joint9", hull,
                new RangeOfMotion(0.3f, -1f, 1f, -1f, 0.3f, -1f));
        super.link("Joint11", hull,
                new RangeOfMotion(0f, -1.4f, 0.8f, -0.8f, 0f, -0f));
        super.link("Joint12", hull,
                new RangeOfMotion(0.5f, 1f, 0f));

        // left arm
        super.link("Joint14", hull,
                new RangeOfMotion(0.3f, -1f, 1f, -1f, 1f, -0.3f));
        super.link("Joint16", hull,
                new RangeOfMotion(0f, -1.4f, 0.8f, -0.8f, 0f, -0f));
        super.link("Joint17", hull,
                new RangeOfMotion(0.5f, 1f, 0f));

        // right leg
        super.link("Joint18", hull,
                new RangeOfMotion(0.3f, -1f, 0.3f, -0.3f, 0.2f, -0.5f));
        super.link("Joint19", hull,
                new RangeOfMotion(1.2f, 0f, 0.2f, -0.2f, 0f, 0f));
        super.link("Joint21", hull,
                new RangeOfMotion(0.5f, 0.2f, 0.1f));

        // left leg
        super.link("Joint23", hull,
                new RangeOfMotion(0.3f, -1f, 0.3f, -0.3f, 0.5f, -0.2f));
        super.link("Joint24", hull,
                new RangeOfMotion(1.2f, 0f, 0.2f, -0.2f, 0f, 0f));
        super.link("Joint26", hull,
                new RangeOfMotion(0.5f, 0.2f, 0.1f));
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
        BoneLink result = findBoneLink("Joint26");
        return result;
    }

    /**
     * Access the BoneLink that manages the model's right foot.
     *
     * @return the pre-existing instance (not null)
     */
    @Override
    public BoneLink getRightFoot() {
        BoneLink result = findBoneLink("Joint21");
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
        return "277/Ninja-geom-1";
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
        result.set(0f, 0f, -1f);
        return result;
    }
}
