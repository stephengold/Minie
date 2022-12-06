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
 * A DynamicAnimControl configured specifically for the Jaime model.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class JaimeControl
        extends DynamicAnimControl
        implements Binocular, Biped, Face {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger4
            = Logger.getLogger(JaimeControl.class.getName());
    // *************************************************************************
    // constructors

    /**
     * Instantiate a new DynamicAnimControl tuned for the Jaime model.
     */
    public JaimeControl() {
        super();
        LinkConfig hull = new LinkConfig(0.005f, MassHeuristic.Mass,
                ShapeHeuristic.VertexHull, new Vector3f(1f, 1f, 1f),
                CenterHeuristic.Mean, RotationOrder.XZY);
        LinkConfig hullZxy = new LinkConfig(0.005f, MassHeuristic.Mass,
                ShapeHeuristic.VertexHull, new Vector3f(1f, 1f, 1f),
                CenterHeuristic.Mean, RotationOrder.ZXY);

        super.setConfig(torsoName, hull);
        super.setMainBoneName("pelvis");

        super.link("spine", hull,
                new RangeOfMotion(1f));
        super.link("ribs", hull,
                new RangeOfMotion(0.6f, 0.4f, 0.4f));
        super.link("head", hull,
                new RangeOfMotion(0.3f, -0.6f, 0.5f, -0.5f, 0.5f, -0.5f));
        super.link("eye.L", hull,
                new RangeOfMotion(0.5f, 0f, 0.5f));
        super.link("eye.R", hull,
                new RangeOfMotion(0.5f, 0f, 0.5f));

        super.link("tail.001", hull,
                new RangeOfMotion(0.5f, 0.2f, 0.5f));
        super.link("tail.002", hull,
                new RangeOfMotion(0.5f, 0.2f, 0.5f));
        super.link("tail.003", hull,
                new RangeOfMotion(0.5f, 0.2f, 0.5f));
        super.link("tail.004", hull,
                new RangeOfMotion(0.5f, 0.2f, 0.5f));
        super.link("tail.005", hull,
                new RangeOfMotion(0.5f, 0.2f, 0.5f));
        super.link("tail.007", hull,
                new RangeOfMotion(0.5f, 0.2f, 0.5f));
        super.link("tail.009", hull,
                new RangeOfMotion(0.5f, 0.2f, 0.5f));

        super.link("shoulder.R", hull,
                new RangeOfMotion(0.8f, -1.6f, 0f, 0f, 0.3f, -0.6f));
        super.link("upper_arm.R", hullZxy,
                new RangeOfMotion(0.8f, -1.6f, 1f, -1f, 1.6f, -1.8f));
        super.link("forearm.R", hull,
                new RangeOfMotion(0f, -2f, 1f, -1f, 0f, 0f));
        super.link("hand.R", hull,
                new RangeOfMotion(0.3f, -0.8f, 0f, 0f, 0.2f, -0.2f));

        super.link("shoulder.L", hull,
                new RangeOfMotion(1.6f, -0.8f, 0f, 0f, 0.6f, -0.3f));
        super.link("upper_arm.L", hullZxy,
                new RangeOfMotion(0.8f, -1.6f, 1f, -1f, 1.6f, -1.8f));
        super.link("forearm.L", hull,
                new RangeOfMotion(0f, -2f, 1f, -1f, 0f, 0f));
        super.link("hand.L", hull,
                new RangeOfMotion(0.8f, -0.3f, 0f, 0f, 0.2f, -0.2f));

        super.link("thigh.R", hull,
                new RangeOfMotion(1f, -0.4f, 0.4f, -0.4f, 0.5f, -0.5f));
        super.link("shin.R", hullZxy,
                new RangeOfMotion(0f, 0f, 0f, 0f, 2f, 0f));
        super.link("foot.R", hull,
                new RangeOfMotion(0.6f, 0.2f, 0f));

        super.link("thigh.L", hull,
                new RangeOfMotion(1f, -0.4f, 0.4f, -0.4f, 0.5f, -0.5f));
        super.link("shin.L", hullZxy,
                new RangeOfMotion(0f, 0f, 0f, 0f, 0f, -2f));
        super.link("foot.L", hull,
                new RangeOfMotion(0.6f, 0.2f, 0f));
    }
    // *************************************************************************
    // Binocular methods

    /**
     * Copy the center-of-vision direction for the model's left eye.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a direction vector (unit vector in the physics link's local
     * coordinates, either storeResult or a new vector)
     */
    @Override
    public Vector3f leftEyeLookDirection(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        result.set(0f, 1f, 0f);
        return result;
    }

    /**
     * Read the vertex spec for the model's left pupil.
     *
     * @return the vertex specification (not null, not empty)
     * @see com.jme3.bullet.animation.DynamicAnimControl#findManagerForVertex(
     * java.lang.String, com.jme3.math.Vector3f, com.jme3.math.Vector3f)
     */
    @Override
    public String leftPupilSpec() {
        return "2949/JaimeGeom-geom-1";
    }

    /**
     * Copy the center-of-vision direction for the model's right eye.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a direction vector (unit vector in the physics link's local
     * coordinates, either storeResult or a new vector)
     */
    @Override
    public Vector3f rightEyeLookDirection(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        result.set(0f, 1f, 0f);
        return result;
    }

    /**
     * Read the vertex spec for the model's right pupil.
     *
     * @return the vertex specification (not null, not empty)
     * @see com.jme3.bullet.animation.DynamicAnimControl#findManagerForVertex(
     * java.lang.String, com.jme3.math.Vector3f, com.jme3.math.Vector3f)
     */
    @Override
    public String rightPupilSpec() {
        return "3046/JaimeGeom-geom-1";
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
        BoneLink result = findBoneLink("foot.L");
        return result;
    }

    /**
     * Access the BoneLink that manages the model's right foot.
     *
     * @return the pre-existing instance (not null)
     */
    @Override
    public BoneLink getRightFoot() {
        BoneLink result = findBoneLink("foot.R");
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
        return "122/JaimeGeom-geom-1";
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
