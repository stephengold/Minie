/*
 Copyright (c) 2020-2022 Stephen Gold
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
 * A DynamicAnimControl configured specifically for the BaseMesh model.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class BaseMeshControl
        extends DynamicAnimControl
        implements Biped, Face {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger4
            = Logger.getLogger(BaseMeshControl.class.getName());
    // *************************************************************************
    // constructors

    /**
     * Instantiate a new control tuned for the BaseMesh model.
     */
    public BaseMeshControl() {
        super();
        LinkConfig hull = new LinkConfig(1f, MassHeuristic.Density,
                ShapeHeuristic.VertexHull, new Vector3f(1f, 1f, 1f),
                CenterHeuristic.Mean, RotationOrder.XZY);

        super.setConfig(torsoName, hull);

        // chest-neck-head
        super.link("spine2", hull,
                new RangeOfMotion(0.8f, -0.3f, 0.3f, -0.3f, 0.3f, -0.3f));
        super.link("neck", hull,
                new RangeOfMotion(0.2f, -0.4f, 0.5f, -0.5f, 0.5f, -0.5f));
        super.link("head", hull,
                new RangeOfMotion(0.2f, -0.4f, 0.5f, -0.5f, 0.5f, -0.5f));

        // right arm
        super.link("spine2_R.001", hull,
                new RangeOfMotion(1f, -0.2f, 0.8f, -0.8f, 0.8f, -0.8f));
        super.link("spine2_R.002", hull,
                new RangeOfMotion(2.5f, 0f, 0.5f, -0.5f, 0f, 0f));
        super.link("spine2_R.003", hull,
                new RangeOfMotion(0.7f, 0f, 0.2f));

        // left arm
        super.link("spine2_L.001", hull,
                new RangeOfMotion(1f, -0.2f, 0.8f, -0.8f, 0.8f, -0.8f));
        super.link("spine2_L.002", hull,
                new RangeOfMotion(2.5f, 0f, 0.5f, -0.5f, 0f, 0f));
        super.link("spine2_L.003", hull,
                new RangeOfMotion(0.7f, 0f, 0.2f));

        // right leg
        super.link("leg_R.001", hull,
                new RangeOfMotion(1f, -0.2f, 0.2f, -0.2f, 0.2f, -0.2f));
        super.link("leg_R.002", hull,
                new RangeOfMotion(0f, -2f, 0.4f, -0.4f, 0f, 0f));
        super.link("leg_R.003", hull,
                new RangeOfMotion(0.6f, 0.1f, 0.3f));

        // left leg
        super.link("leg_L.001", hull,
                new RangeOfMotion(1f, -0.2f, 0.2f, -0.2f, 0.2f, -0.2f));
        super.link("leg_L.002", hull,
                new RangeOfMotion(0f, -2f, 0.4f, -0.4f, 0f, 0f));
        super.link("leg_L.003", hull,
                new RangeOfMotion(0.6f, 0.1f, 0.3f));
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
        BoneLink result = findBoneLink("leg_L.003");
        return result;
    }

    /**
     * Access the BoneLink that manages the model's right foot.
     *
     * @return the pre-existing instance (not null)
     */
    @Override
    public BoneLink getRightFoot() {
        BoneLink result = findBoneLink("leg_R.003");
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
        return "2321/BaseMesh_011";
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
