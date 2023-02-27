/*
 * Copyright (c) 2018-2023 jMonkeyEngine
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
package com.jme3.bullet.animation;

import com.jme3.anim.Armature;
import com.jme3.anim.Joint;
import com.jme3.animation.Bone;
import com.jme3.animation.Skeleton;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.RotationOrder;
import com.jme3.bullet.collision.PhysicsCollisionEvent;
import com.jme3.bullet.collision.PhysicsCollisionListener;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.joints.New6Dof;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.joints.Point2PointJoint;
import com.jme3.bullet.joints.motors.MotorParam;
import com.jme3.bullet.joints.motors.RotationMotor;
import com.jme3.bullet.joints.motors.TranslationMotor;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer;
import com.jme3.util.SafeArrayList;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Logger;
import jme3utilities.MyMesh;
import jme3utilities.MySkeleton;
import jme3utilities.MySpatial;
import jme3utilities.MyString;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * Before adding this Control to a Spatial, configure it by invoking {@link
 * #link(java.lang.String, float, com.jme3.bullet.animation.RangeOfMotion)} for
 * each bone that should have its own rigid body. Leave unlinked bones near the
 * root of the skeleton to form the torso of the ragdoll.
 * <p>
 * When you add the Control to a Spatial, it generates a ragdoll consisting of a
 * rigid body for the torso and another for each linked bone. It also creates a
 * PhysicsJoint connecting each rigid body to its parent in the link hierarchy.
 * The mass of each rigid body and the range-of-motion of each joint can be
 * reconfigured on the fly.
 * <p>
 * Each link is either dynamic (driven by forces and torques) or kinematic
 * (unperturbed by forces and torques). Transitions from dynamic to kinematic
 * can be immediate or gradual.
 *
 * @author Stephen Gold sgold@sonic.net
 *
 * Based on KinematicRagdollControl by Normen Hansen and Rémy Bouquet (Nehon).
 */
public class DynamicAnimControl
        extends DacLinks
        implements PhysicsCollisionListener {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger35
            = Logger.getLogger(DynamicAnimControl.class.getName());
    /**
     * local copy of {@link com.jme3.math.Matrix3f#IDENTITY}
     */
    final private static Matrix3f matrixIdentity = new Matrix3f();
    /**
     * field names for serialization
     */
    final private static String tagCenterLocation = "centerLocation";
    final private static String tagCenterVelocity = "centerVelocity";
    final private static String tagIkJoints = "ikJoints";
    final private static String tagRagdollMass = "ragdollMass";
    /**
     * local copy of {@link com.jme3.math.Vector3f#ZERO}
     */
    final private static Vector3f translateIdentity = new Vector3f(0f, 0f, 0f);
    // *************************************************************************
    // fields

    /**
     * list of IK joints
     */
    private ArrayList<IKJoint> ikJoints = new ArrayList<>(20);
    /**
     * report completion of the current blend-to-kinematic operation
     */
    private CompletionListener<DynamicAnimControl> blendListener;
    /**
     * calculated total mass, not including released attachments
     */
    private float ragdollMass = 0f;
    /**
     * list of registered collision listeners
     */
    private List<RagdollCollisionListener> collisionListeners
            = new SafeArrayList<>(RagdollCollisionListener.class);
    /**
     * center-of-mass actual location (in physics-space coordinates)
     */
    private Vector3f centerLocation = new Vector3f();
    /**
     * center-of-mass estimated velocity (psu/second in physics-space
     * coordinates)
     */
    private Vector3f centerVelocity = new Vector3f();
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled Control without any linked bones or attachments
     * (torso only).
     */
    public DynamicAnimControl() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Add a collision listener to this Control.
     *
     * @param listener the listener to register (not null, alias created)
     */
    public void addCollisionListener(RagdollCollisionListener listener) {
        Validate.nonNull(listener, "listener");
        collisionListeners.add(listener);
    }

    /**
     * Begin blending the specified BoneLink and all its descendants into an
     * amputated state. This has the effect of hiding those links.
     *
     * @param rootLink the root of the subtree to amputate (not null, belongs to
     * this control)
     * @param blendInterval the duration of the blend interval (in seconds,
     * &ge;0)
     */
    public void amputateSubtree(BoneLink rootLink, float blendInterval) {
        Validate.nonNull(rootLink, "root link");
        Validate.require(
                rootLink.getControl() == this, "link belongs to this control");
        Validate.nonNegative(blendInterval, "blend interval");
        verifyAddedToSpatial("change modes");

        blendDescendants(rootLink, KinematicSubmode.Amputated, blendInterval);
        rootLink.blendToKinematicMode(
                KinematicSubmode.Amputated, blendInterval);
    }

    /**
     * Begin blending the specified link and all its descendants to kinematic
     * animation.
     *
     * @param rootLink the root of the subtree to bind (not null, belongs to
     * this control)
     * @param blendInterval the duration of the blend interval (in seconds,
     * &ge;0)
     */
    public void animateSubtree(PhysicsLink rootLink, float blendInterval) {
        Validate.nonNull(rootLink, "root link");
        Validate.require(
                rootLink.getControl() == this, "link belongs to this control");
        Validate.nonNegative(blendInterval, "blend interval");
        verifyAddedToSpatial("change modes");

        blendSubtree(rootLink, KinematicSubmode.Animated, blendInterval);
    }

    /**
     * Begin blending the specified link and all its descendants into bind pose.
     *
     * @param rootLink the root of the subtree to bind (not null, belongs to
     * this control)
     * @param blendInterval the duration of the blend interval (in seconds,
     * &ge;0)
     */
    public void bindSubtree(PhysicsLink rootLink, float blendInterval) {
        Validate.nonNull(rootLink, "root link");
        Validate.require(
                rootLink.getControl() == this, "link belongs to this control");
        Validate.nonNegative(blendInterval, "blend interval");
        verifyAddedToSpatial("change modes");

        blendSubtree(rootLink, KinematicSubmode.Bound, blendInterval);
    }

    /**
     * Begin blending all links to purely kinematic mode, driven by animation.
     * <p>
     * Allowed only when the Control IS added to a Spatial.
     *
     * @param blendInterval the duration of the blend interval (in seconds,
     * &ge;0)
     * @param endModelTransform the desired local transform for the controlled
     * spatial when the blend completes (alias created) or null for no change to
     * the local transform
     */
    public void blendToKinematicMode(
            float blendInterval, Transform endModelTransform) {
        Validate.nonNegative(blendInterval, "blend interval");
        verifyAddedToSpatial("change modes");

        blendToKinematicMode(
                KinematicSubmode.Animated, blendInterval, endModelTransform);
    }

    /**
     * Begin blending all links to purely kinematic mode.
     * <p>
     * Allowed only when the Control IS added to a Spatial.
     *
     * @param submode enum value (not null)
     * @param blendInterval the duration of the blend interval (in seconds,
     * &ge;0)
     * @param endModelTransform the desired local transform for the controlled
     * spatial when the blend completes (alias created) or null for no change to
     * the local transform
     */
    public void blendToKinematicMode(KinematicSubmode submode,
            float blendInterval, Transform endModelTransform) {
        Validate.nonNull(submode, "submode");
        Validate.nonNegative(blendInterval, "blend interval");
        verifyAddedToSpatial("change modes");

        getTorsoLink().blendToKinematicMode(
                submode, blendInterval, endModelTransform);
        for (BoneLink boneLink : getBoneLinks()) {
            boneLink.blendToKinematicMode(submode, blendInterval);
        }
        for (AttachmentLink link : listAttachmentLinks()) {
            if (!link.isReleased()) {
                link.blendToKinematicMode(blendInterval, null);
            }
        }
    }

    /**
     * Calculate the ragdoll's total mass and center of mass, excluding released
     * attachments.
     * <p>
     * Allowed only when the Control IS added to a Spatial.
     *
     * @param storeLocation storage for the location of the center (in
     * physics-space coordinates, modified if not null)
     * @param storeVelocity storage for the velocity of the center (psu/second
     * in physics-space coordinates, modified if not null)
     * @return the total mass (&gt;0)
     */
    public float centerOfMass(Vector3f storeLocation, Vector3f storeVelocity) {
        verifyReadyForDynamicMode("calculate the center of mass");

        recalculateCenter();
        if (storeLocation != null) {
            storeLocation.set(centerLocation);
        }
        if (storeVelocity != null) {
            storeVelocity.set(centerVelocity);
        }

        return ragdollMass;
    }

    /**
     * Release (to gravity) all unreleased attachments.
     */
    public void dropAttachments() {
        for (AttachmentLink link : listAttachmentLinks()) {
            if (!link.isReleased()) {
                Vector3f gravity = gravity(null);
                link.setDynamic(gravity);
                link.release();
            }
        }
    }

    /**
     * Find the link that manages the specified vertex and optionally calculate
     * the location of that vertex.
     * <p>
     * A software skin update must precede any request for vertex locations.
     * TODO use the Wes library to avoid this limitation?
     *
     * @param vertexSpecifier a String of the form "index/geometry" or
     * "index/geometry/bone" (not null, not empty)
     * @param storeWorldLocation storage for location in physics-space
     * coordinates (modified if not null)
     * @param storeLocalLocation storage for location in the local coordinates
     * of the link's body (modified if not null)
     * @return the pre-existing link (not null)
     */
    public PhysicsLink findManagerForVertex(String vertexSpecifier,
            Vector3f storeWorldLocation, Vector3f storeLocalLocation) {
        Validate.nonEmpty(vertexSpecifier, "vertex specifier");
        Vector3f worldLocation = (storeWorldLocation == null)
                ? new Vector3f() : storeWorldLocation;

        // Parse the vertex index and geometry name from the specifier.
        String[] fields = vertexSpecifier.split("/");
        int numFields = fields.length;
        if (numFields < 2 || numFields > 3) {
            String message = "malformed vertex specifier "
                    + MyString.quote(vertexSpecifier);
            throw new IllegalArgumentException(message);
        }

        // Find the mesh that contains the vertex.
        Armature armature = getArmature();
        Skeleton skeleton = getSkeleton();
        Spatial subtree;
        if (numFields == 3) { // The vertex is in an attached model.
            String attachName = fields[2];
            if (armature == null) {
                Bone attachBone = skeleton.getBone(attachName);
                if (attachBone == null) {
                    String message = String.format(
                            "non-existent bone %s in vertex specifier",
                            MyString.quote(attachName));
                    throw new IllegalArgumentException(message);
                }
                subtree = MySkeleton.getAttachments(attachBone);
                if (subtree == null) {
                    String message = String.format("no attachment to bone %s",
                            MyString.quote(attachName));
                    throw new IllegalArgumentException(message);
                }

            } else { // armature != null
                Joint attachArmatureJoint = armature.getJoint(attachName);
                if (attachArmatureJoint == null) {
                    String message = String.format(
                            "non-existent bone %s in vertex specifier",
                            MyString.quote(attachName));
                    throw new IllegalArgumentException(message);
                }
                subtree = MySkeleton.getAttachments(attachArmatureJoint);
                if (subtree == null) {
                    String message = String.format("no attachment to bone %s",
                            MyString.quote(attachName));
                    throw new IllegalArgumentException(message);
                }
            }

        } else { // The vertex is in the controlled model.
            subtree = getSpatial();
            assert subtree != null;
        }
        String geometryName = fields[1];
        Spatial gSpatial = MySpatial.findNamed(subtree, geometryName);
        if (gSpatial == null) {
            String message = String.format(
                    "non-existent geometry %s in vertex specifier",
                    MyString.quote(fields[1]));
            throw new IllegalArgumentException(message);
        }
        Geometry geometry = (Geometry) gSpatial;
        Mesh mesh = geometry.getMesh();

        // Calculate the mesh location (pos) of the vertex.
        int vertexIndex;
        try {
            vertexIndex = Integer.parseInt(fields[0]);
        } catch (NumberFormatException exception) {
            vertexIndex = -1;
        }
        int numVertices = mesh.getVertexCount();
        if (vertexIndex < 0 || vertexIndex >= numVertices) {
            String message = String.format(
                    "non-existent vertex %s in vertex specifier (legal range: "
                    + "0 to %d)", MyString.quote(fields[0]), numVertices - 1);
            throw new IllegalArgumentException(message);
        }
        Vector3f pos = MyMesh.vertexVector3f(
                mesh, VertexBuffer.Type.Position, vertexIndex, null);

        // Find the manager and convert the pos to a world location.
        PhysicsLink manager;
        if (numFields == 3) { // The vertex is in an attached model.
            assert !MyMesh.isAnimated(mesh);
            manager = findAttachmentLink(fields[2]);
            geometry.localToWorld(pos, worldLocation);

        } else { // The vertex is in the controlled model.
            assert MyMesh.isAnimated(mesh);
            String[] managerMap;
            if (armature == null) {
                managerMap = managerMap(skeleton);
            } else {
                managerMap = managerMap(armature);
            }
            String managerName = RagUtils.findManager(
                    mesh, vertexIndex, new int[4], new float[4], managerMap);
            if (managerName.equals(torsoName)) {
                manager = getTorsoLink();
            } else {
                manager = findBoneLink(managerName);
            }
            assert manager != null;
            Transform meshToWorld = meshTransform(null);
            meshToWorld.transformVector(pos, worldLocation);
        }

        if (storeLocalLocation != null) {
            Transform localToWorld = manager.physicsTransform(null);
            localToWorld.setScale(1f);
            localToWorld
                    .transformInverseVector(worldLocation, storeLocalLocation);
        }

        return manager;
    }

    /**
     * Add an IK joint that will fix the specified link in its current position.
     * Its location can be altered via the joint's TranslationMotor.
     * <p>
     * Allowed only when the Control IS added to a Spatial and all links are
     * ready for dynamic mode.
     *
     * @param link which link to fix (not null)
     * @param disableForRagdoll true&rarr;disable the Constraint when entering
     * ragdoll mode, false&rarr;unaffected by ragdoll mode
     * @return a new joint, with the link body at the B end
     */
    public IKJoint fixToWorld(PhysicsLink link, boolean disableForRagdoll) {
        verifyReadyForDynamicMode("add an IK joint");

        Transform localToWorld = link.physicsTransform(null);
        Quaternion rotToWorld = localToWorld.getRotation(); // alias
        Matrix3f worldToLocal = rotToWorld.toRotationMatrix().invertLocal();
        PhysicsRigidBody linkBody = link.getRigidBody();
        New6Dof new6dof = new New6Dof(linkBody, translateIdentity,
                translateIdentity, worldToLocal, matrixIdentity,
                RotationOrder.XYZ);

        // Initialize the fix location in physics-space coordinates.
        TranslationMotor motor = new6dof.getTranslationMotor();
        Vector3f location = localToWorld.getTranslation(); // alias
        motor.set(MotorParam.LowerLimit, location);
        motor.set(MotorParam.UpperLimit, location);

        // Lock all rotation axes at 0.
        for (int axisIndex = 0; axisIndex < MyVector3f.numAxes; ++axisIndex) {
            RotationMotor rotMotor = new6dof.getRotationMotor(axisIndex);
            rotMotor.setSpringEnabled(true);
            int rotDofIndex = axisIndex + 3;
            new6dof.set(MotorParam.UpperLimit, rotDofIndex, 0f);
            new6dof.set(MotorParam.LowerLimit, rotDofIndex, 0f);
        }

        IKJoint result = new IKJoint(new6dof, disableForRagdoll);
        ikJoints.add(result);
        getPhysicsSpace().addJoint(new6dof);

        assert new6dof.getBodyB() == linkBody;
        return result;
    }

    /**
     * Immediately freeze the specified link and all its descendants. Note:
     * recursive!
     * <p>
     * Allowed only when the Control IS added to a Spatial.
     *
     * @param rootLink the root of the subtree to freeze (not null, belongs to
     * this control)
     * @param forceKinematic true&rarr;force link to kinematic mode,
     * false&rarr;preserve link modes
     */
    public void freezeSubtree(PhysicsLink rootLink, boolean forceKinematic) {
        Validate.nonNull(rootLink, "root link");
        Validate.require(
                rootLink.getControl() == this, "link belongs to this control");
        verifyAddedToSpatial("change modes");

        rootLink.freeze(forceKinematic);

        PhysicsLink[] children = rootLink.listChildren();
        for (PhysicsLink child : children) {
            freezeSubtree(child, forceKinematic);
        }
    }

    /**
     * Access the blend listener.
     *
     * @return the pre-existing instance, or null of none
     */
    public CompletionListener<DynamicAnimControl> getBlendListener() {
        return blendListener;
    }

    /**
     * Calculate the ragdoll's total kinetic energy, excluding released
     * attachments.
     *
     * @return the total kinetic energy, or NaN if any link isn't in dynamic
     * mode
     */
    public double kineticEnergy() {
        double result = 0.0;
        List<PhysicsLink> links = listLinks(PhysicsLink.class);
        for (PhysicsLink link : links) {
            if (!link.isReleased()) {
                PhysicsRigidBody rigidBody = link.getRigidBody();
                if (rigidBody.isDynamic()) {
                    double energy = rigidBody.kineticEnergy();
                    result += energy;
                } else {
                    result = Float.NaN;
                    break;
                }
            }
        }

        return result;
    }

    /**
     * Calculate the ragdoll's total mechanical energy, excluding released
     * attachments.
     *
     * @return the total mechanical energy, or NaN if any link isn't in dynamic
     * mode
     */
    public double mechanicalEnergy() {
        double result = 0.0;
        List<PhysicsLink> links = listLinks(PhysicsLink.class);
        for (PhysicsLink link : links) {
            if (!link.isReleased()) {
                PhysicsRigidBody rigidBody = link.getRigidBody();
                if (rigidBody.isDynamic()) {
                    double energy = rigidBody.mechanicalEnergy();
                    result += energy;
                } else {
                    result = Float.NaN;
                    break;
                }
            }
        }

        return result;
    }

    /**
     * Enumerate IK joints managed by this Control.
     *
     * @return a new array of pre-existing joints (not null, not empty)
     */
    public IKJoint[] listIKJoints() {
        int numJoints = ikJoints.size();
        IKJoint[] result = new IKJoint[numJoints];
        ikJoints.toArray(result);

        return result;
    }

    /**
     * Add an IK joint that will cause the specified link to move until its
     * pivot location coincides with that of the specified goal body.
     *
     * @param link which link to move (not null)
     * @param pivotInLinkBody the pivot location (in the link body's local
     * coordinates, not null, unaffected)
     * @param goalBody a rigid body that represents the goal (not null, alias
     * created)
     * @param pivotInGoalBody the pivot location (in the goal's local
     * coordinates, not null, unaffected)
     * @return a new joint, with the link body at the A end and the goal at the
     * B end, which will be disabled by ragdoll mode (not null)
     */
    public IKJoint moveToBody(PhysicsLink link, Vector3f pivotInLinkBody,
            PhysicsRigidBody goalBody, Vector3f pivotInGoalBody) {
        Validate.nonNull(pivotInLinkBody, "pivot in link body");
        Validate.nonNull(goalBody, "goal body");
        Validate.nonNull(pivotInGoalBody, "pivot in goal body");

        PhysicsRigidBody linkBody = link.getRigidBody();
        Point2PointJoint newJoint = new Point2PointJoint(
                linkBody, goalBody, pivotInLinkBody, pivotInGoalBody);
        IKJoint ikJoint = new IKJoint(newJoint, true);

        ikJoints.add(ikJoint);
        getPhysicsSpace().addJoint(newJoint);

        assert newJoint.getBodyA() == linkBody;
        return ikJoint;
    }

    /**
     * Add an IK joint that will cause the specified link to move until a fixed
     * pivot location coincides with the specified goal location.
     *
     * @param link which link to move (not null)
     * @param pivotInLinkBody the pivot location (in the link body's local
     * coordinates, not null, unaffected)
     * @param goalInWorld the goal location (in physics-space coordinates, not
     * null, unaffected)
     * @return a new joint, with the link body at the A end, which will be
     * disabled by ragdoll mode (not null)
     */
    public IKJoint moveToWorld(
            PhysicsLink link, Vector3f pivotInLinkBody, Vector3f goalInWorld) {
        Validate.finite(pivotInLinkBody, "pivot in link body");
        Validate.finite(goalInWorld, "goal location");

        PhysicsRigidBody linkBody = link.getRigidBody();
        Point2PointJoint newJoint
                = new Point2PointJoint(linkBody, pivotInLinkBody, goalInWorld);
        IKJoint ikJoint = new IKJoint(newJoint, true);
        ikJoints.add(ikJoint);
        getPhysicsSpace().addJoint(newJoint);

        assert newJoint.getBodyA() == linkBody;
        return ikJoint;
    }

    /**
     * Add an IK joint that will restrict the specified links to pivoting around
     * each other.
     * <p>
     * Allowed only when the Control IS added to a Spatial and all links are
     * ready for dynamic mode.
     *
     * @param linkA the first link to pin (not null)
     * @param linkB the 2nd link to pin (not null)
     * @param pivotInA the pivot location in the A's scaled local coordinates
     * (not null, unaffected)
     * @param pivotInB the pivot location in the B's scaled local coordinates
     * (not null, unaffected)
     * @return a new joint with A's body at the A end, which will be disabled by
     * ragdoll mode (not null)
     */
    public IKJoint pinToSelf(PhysicsLink linkA, PhysicsLink linkB,
            Vector3f pivotInA, Vector3f pivotInB) {
        verifyReadyForDynamicMode("add an IK joint");

        PhysicsRigidBody bodyA = linkA.getRigidBody();
        PhysicsRigidBody bodyB = linkB.getRigidBody();
        Point2PointJoint newJoint
                = new Point2PointJoint(bodyA, bodyB, pivotInA, pivotInB);
        IKJoint ikJoint = new IKJoint(newJoint, true);
        ikJoints.add(ikJoint);
        getPhysicsSpace().addJoint(newJoint);

        assert newJoint.getBodyA() == bodyA;
        assert newJoint.getBodyB() == bodyB;
        return ikJoint;
    }

    /**
     * Add an IK joint that restricts the specified link to rotating around a
     * pin location. The pin is initially located at the link's center. Its
     * location can be altered via the joint's TranslationMotor.
     * <p>
     * Allowed only when the Control IS added to a Spatial and all links are
     * ready for dynamic mode.
     *
     * @param link which link to pin (not null)
     * @param disableForRagdoll true&rarr;disable the joint when entering
     * ragdoll mode, false&rarr;unaffected by ragdoll mode
     * @return a new joint, with the link body at the B end
     */
    public IKJoint pinToWorld(PhysicsLink link, boolean disableForRagdoll) {
        verifyReadyForDynamicMode("add an IK joint");

        PhysicsRigidBody linkBody = link.getRigidBody();
        New6Dof new6dof = new New6Dof(linkBody, translateIdentity,
                translateIdentity, matrixIdentity, matrixIdentity,
                RotationOrder.XYZ);

        // Initialize the pin location in physics-space coordinates.
        TranslationMotor motor = new6dof.getTranslationMotor();
        Vector3f location = linkBody.getPhysicsLocation(null);
        motor.set(MotorParam.LowerLimit, location);
        motor.set(MotorParam.UpperLimit, location);

        IKJoint result = new IKJoint(new6dof, disableForRagdoll);
        ikJoints.add(result);
        getPhysicsSpace().addJoint(new6dof);

        assert new6dof.getBodyB() == linkBody;
        return result;
    }

    /**
     * Add an IK joint that restricts the specified link to rotating around the
     * specified location.
     * <p>
     * Allowed only when the Control IS added to a Spatial and all links are
     * ready for dynamic mode.
     *
     * @param link which link to pin (not null)
     * @param pivotInWorld the pin location (in physics-space coordinates, not
     * null, unaffected)
     * @return a new joint, with the link body at the A end, which will be
     * disabled by ragdoll mode
     */
    public IKJoint pinToWorld(PhysicsLink link, Vector3f pivotInWorld) {
        Validate.nonNull(pivotInWorld, "pivot location");
        verifyReadyForDynamicMode("add an IK joint");

        PhysicsRigidBody linkBody = link.getRigidBody();
        Transform localToWorld = link.physicsTransform(null);
        localToWorld.setScale(1f);
        Vector3f pivotInBody
                = localToWorld.transformInverseVector(pivotInWorld, null);
        Point2PointJoint newJoint = new Point2PointJoint(linkBody, pivotInBody);
        IKJoint ikJoint = new IKJoint(newJoint, true);
        ikJoints.add(ikJoint);
        getPhysicsSpace().addJoint(newJoint);

        assert newJoint.getBodyA() == linkBody;
        return ikJoint;
    }

    /**
     * Record the current bone transforms for use in a kinematic reset.
     */
    public void saveCurrentPose() {
        List<BoneLink> boneLinks = getBoneLinks();
        TorsoLink torso = getTorsoLink();
        int numManaged = torso.countManaged();
        Transform[] resetTransforms = new Transform[numManaged];

        Skeleton skeleton = getSkeleton();
        if (skeleton == null) { // new animation system
            Armature armature = getArmature();

            for (int mbIndex = 0; mbIndex < numManaged; ++mbIndex) {
                int jointIndex = torso.boneIndex(mbIndex);
                Joint joint = armature.getJoint(jointIndex);
                resetTransforms[mbIndex]
                        = joint.getLocalTransform().clone();
            }
            torso.setEndBoneTransforms(resetTransforms);

            for (BoneLink boneLink : boneLinks) {
                numManaged = boneLink.countManaged();
                resetTransforms = new Transform[numManaged];
                for (int mbIndex = 0; mbIndex < numManaged; ++mbIndex) {
                    int jointIndex = boneLink.boneIndex(mbIndex);
                    Joint joint = armature.getJoint(jointIndex);
                    resetTransforms[mbIndex]
                            = joint.getLocalTransform().clone();
                }
                boneLink.setEndBoneTransforms(resetTransforms);
            }

        } else { // old animation system
            for (int mbIndex = 0; mbIndex < numManaged; ++mbIndex) {
                int boneIndex = torso.boneIndex(mbIndex);
                Bone bone = skeleton.getBone(boneIndex);
                resetTransforms[mbIndex]
                        = MySkeleton.copyLocalTransform(bone, null);
            }
            torso.setEndBoneTransforms(resetTransforms);

            for (BoneLink boneLink : boneLinks) {
                numManaged = boneLink.countManaged();
                resetTransforms = new Transform[numManaged];
                for (int mbIndex = 0; mbIndex < numManaged; ++mbIndex) {
                    int boneIndex = boneLink.boneIndex(mbIndex);
                    Bone bone = skeleton.getBone(boneIndex);
                    resetTransforms[mbIndex]
                            = MySkeleton.copyLocalTransform(bone, null);
                }
                boneLink.setEndBoneTransforms(resetTransforms);
            }
        }
    }

    /**
     * Replace the current blend listener. Note that the listener is
     * automatically removed after each invocation, so typically this method is
     * re-invoked before each blend.
     *
     * @param listener the desired listener, or null for none
     */
    public void
            setBlendListener(CompletionListener<DynamicAnimControl> listener) {
        this.blendListener = listener;
    }

    /**
     * Alter the contact-response setting of the specified link and all its
     * descendants (excluding released attachments). Note: recursive!
     * <p>
     * Allowed only when the Control IS added to a Spatial.
     *
     * @param rootLink the root of the subtree to modify (not null, belongs to
     * this control)
     * @param desiredResponse true for the usual rigid-body response, false for
     * ghost-like response
     */
    public void setContactResponseSubtree(
            PhysicsLink rootLink, boolean desiredResponse) {
        Validate.nonNull(rootLink, "root link");
        Validate.require(
                rootLink.getControl() == this, "link belongs to this control");
        verifyAddedToSpatial("change modes");

        if (!rootLink.isReleased()) {
            PhysicsRigidBody rigidBody = rootLink.getRigidBody();
            rigidBody.setContactResponse(desiredResponse);

            PhysicsLink[] children = rootLink.listChildren();
            for (PhysicsLink child : children) {
                setContactResponseSubtree(child, desiredResponse);
            }
        }
    }

    /**
     * Immediately put the specified link and all its ancestors (excluding the
     * torso) into dynamic mode. Note: recursive!
     * <p>
     * Allowed only when the Control IS added to a Spatial and all links are
     * ready for dynamic mode.
     *
     * @param startLink the start of the chain to modify (not null)
     * @param chainLength the maximum number of links to modify (&ge;0)
     * @param uniformAcceleration the uniform acceleration vector (in
     * physics-space coordinates, not null, unaffected)
     * @param lockAll true to lock all axes of links (except the torso)
     */
    public void setDynamicChain(PhysicsLink startLink, int chainLength,
            Vector3f uniformAcceleration, boolean lockAll) {
        if (chainLength == 0) {
            return;
        }
        Validate.positive(chainLength, "chain length");
        Validate.nonNull(startLink, "start link");
        Validate.finite(uniformAcceleration, "uniform acceleration");
        verifyReadyForDynamicMode("put links into dynamic mode");

        if (startLink instanceof BoneLink) {
            BoneLink boneLink = (BoneLink) startLink;
            boneLink.setDynamic(uniformAcceleration, lockAll, lockAll, lockAll);
        } else if (startLink instanceof AttachmentLink) {
            AttachmentLink attachmentLink = (AttachmentLink) startLink;
            attachmentLink.setDynamic(uniformAcceleration);
        }

        PhysicsLink parent = startLink.getParent();
        if (parent != null && chainLength > 1) {
            setDynamicChain(
                    parent, chainLength - 1, uniformAcceleration, lockAll);
        }
    }

    /**
     * Immediately put the specified link and all its descendants (excluding
     * released attachments) into dynamic mode. Note: recursive!
     * <p>
     * Allowed only when the Control IS added to a Spatial.
     *
     * @param rootLink the root of the subtree to modify (not null, belongs to
     * this control)
     * @param uniformAcceleration the uniform acceleration vector (in
     * physics-space coordinates, not null, unaffected)
     * @param lockAll true to lock all axes of links (except the torso)
     */
    public void setDynamicSubtree(PhysicsLink rootLink,
            Vector3f uniformAcceleration, boolean lockAll) {
        Validate.nonNull(rootLink, "root link");
        Validate.require(
                rootLink.getControl() == this, "link belongs to this control");
        Validate.nonNull(uniformAcceleration, "uniform acceleration");
        verifyAddedToSpatial("change modes");

        if (rootLink == getTorsoLink()) {
            getTorsoLink().setDynamic(uniformAcceleration);
        } else if (rootLink instanceof BoneLink) {
            BoneLink boneLink = (BoneLink) rootLink;
            boneLink.setDynamic(uniformAcceleration, lockAll, lockAll, lockAll);
        } else {
            AttachmentLink attachmentLink = (AttachmentLink) rootLink;
            if (!attachmentLink.isReleased()) {
                attachmentLink.setDynamic(uniformAcceleration);
            }
        }

        PhysicsLink[] children = rootLink.listChildren();
        for (PhysicsLink child : children) {
            setDynamicSubtree(child, uniformAcceleration, lockAll);
        }
    }

    /**
     * Immediately put all links into purely kinematic mode. The transform of
     * the controlled spatial is unaffected.
     * <p>
     * Allowed only when the Control IS added to a Spatial.
     */
    public void setKinematicMode() {
        verifyAddedToSpatial("set kinematic mode");

        float blendInterval = 0f; // in seconds
        blendToKinematicMode(KinematicSubmode.Animated, blendInterval, null);
    }

    /**
     * Immediately put all links into purely kinematic mode. The transform of
     * the controlled spatial is unaffected.
     * <p>
     * Allowed only when the Control IS added to a Spatial.
     *
     * @param submode enum value (not null)
     */
    public void setKinematicMode(KinematicSubmode submode) {
        Validate.nonNull(submode, "submode");
        verifyAddedToSpatial("set kinematic mode");

        float blendInterval = 0f; // in seconds
        blendToKinematicMode(submode, blendInterval, null);
    }

    /**
     * Immediately put all links and IK joints into ragdoll mode.
     * <p>
     * Allowed only when the Control IS added to a Spatial and all links are
     * ready for dynamic mode.
     */
    public void setRagdollMode() {
        verifyReadyForDynamicMode("set ragdoll mode");

        TorsoLink torsoLink = getTorsoLink();
        torsoLink.setRagdollMode();
        for (BoneLink boneLink : getBoneLinks()) {
            boneLink.setRagdollMode();
        }
        for (AttachmentLink link : listAttachmentLinks()) {
            link.setRagdollMode();
        }
        for (IKJoint joint : ikJoints) {
            joint.setRagdollMode();
        }
    }
    // *************************************************************************
    // DacPhysicsLinks methods

    /**
     * Add all managed physics objects to the PhysicsSpace.
     */
    @Override
    protected void addPhysics() {
        super.addPhysics();

        PhysicsSpace space = getPhysicsSpace();
        space.addCollisionListener(this);
        space.addTickListener(this);

        for (IKJoint ikJoint : ikJoints) {
            PhysicsJoint joint = ikJoint.getPhysicsJoint();
            space.addJoint(joint);
        }
    }

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned Control into a deep-cloned one, using the specified Cloner
     * and original to resolve copied fields.
     *
     * @param cloner the Cloner that's cloning this Control (not null, modified)
     * @param original the instance from which this Control was shallow-cloned
     * (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        super.cloneFields(cloner, original);

        this.ikJoints = cloner.clone(ikJoints);
        this.collisionListeners = cloner.clone(collisionListeners);
        this.centerLocation = cloner.clone(centerLocation);
        this.centerVelocity = cloner.clone(centerVelocity);
    }

    /**
     * De-serialize this Control from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    @SuppressWarnings("unchecked")
    public void read(JmeImporter importer) throws IOException {
        super.read(importer);
        InputCapsule capsule = importer.getCapsule(this);

        this.ikJoints = capsule
                .readSavableArrayList(tagIkJoints, new ArrayList(1));
        this.ragdollMass = capsule.readFloat(tagRagdollMass, 1f);
        this.centerLocation = (Vector3f) capsule
                .readSavable(tagCenterLocation, new Vector3f());
        this.centerVelocity = (Vector3f) capsule
                .readSavable(tagCenterVelocity, new Vector3f());
    }

    /**
     * Remove all managed physics objects from the PhysicsSpace.
     */
    @Override
    protected void removePhysics() {
        super.removePhysics();

        PhysicsSpace space = getPhysicsSpace();
        space.removeCollisionListener(this);
        space.removeTickListener(this);

        for (IKJoint ikJoint : ikJoints) {
            PhysicsJoint joint = ikJoint.getPhysicsJoint();
            space.removeJoint(joint);
        }
    }

    /**
     * Update this Control. Invoked once per frame during the logical-state
     * update, provided the control is added to a scene. Do not invoke directly
     * from user code.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void update(float tpf) {
        super.update(tpf);

        if (blendListener != null) {
            float weight = getTorsoLink().kinematicWeight();
            if (weight == 1f) {
                blendListener.onCompletion(this);
                this.blendListener = null;
            }
        }
    }

    /**
     * Serialize this Control to the specified exporter, for example when saving
     * to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        super.write(exporter);
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.writeSavableArrayList(ikJoints, tagIkJoints, null);
        // blendListener and collisionListeners are never written.
        capsule.write(ragdollMass, tagRagdollMass, 1f);
        capsule.write(centerLocation, tagCenterLocation, null);
        capsule.write(centerVelocity, tagCenterVelocity, null);
    }
    // *************************************************************************
    // PhysicsCollisionListener methods

    /**
     * For internal use only: callback for collision events.
     *
     * @param event (not null)
     */
    @Override
    public void collision(PhysicsCollisionEvent event) {
        if (event.getNodeA() == null && event.getNodeB() == null) {
            return;
        }
        /*
         * Determine which bone was involved (if any) and also the
         * other collision object involved.
         */
        boolean isThisControlInvolved = false;
        PhysicsLink physicsLink = null;
        PhysicsCollisionObject otherPco = null;
        PhysicsCollisionObject pcoA = event.getObjectA();
        PhysicsCollisionObject pcoB = event.getObjectB();

        Object userA = pcoA.getUserObject();
        Object userB = pcoB.getUserObject();
        if (userA instanceof PhysicsLink) {
            physicsLink = (PhysicsLink) userA;
            DacLinks control = physicsLink.getControl();
            if (control == this) {
                isThisControlInvolved = true;
            }
            otherPco = pcoB;
        }
        if (userB instanceof PhysicsLink) {
            physicsLink = (PhysicsLink) userB;
            DacLinks control = physicsLink.getControl();
            if (control == this) {
                isThisControlInvolved = true;
            }
            otherPco = pcoA;
        }

        // Discard collisions that don't involve this Control.
        if (!isThisControlInvolved) {
            return;
        }

        // Discard low-impulse collisions.
        float impulseThreshold = eventDispatchImpulseThreshold();
        if (event.getAppliedImpulse() < impulseThreshold) {
            return;
        }

        // Dispatch an event.
        for (RagdollCollisionListener listener : collisionListeners) {
            listener.collide(physicsLink, otherPco, event);
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Begin blending the descendants of the specified link to the specified
     * kinematic submode. Note: recursive!
     *
     * @param rootLink the root of the subtree to blend (not null, belongs to
     * this control)
     * @param submode an enum value (not null)
     * @param blendInterval the duration of the blend interval (in seconds,
     * &ge;0)
     */
    private static void blendDescendants(PhysicsLink rootLink,
            KinematicSubmode submode, float blendInterval) {
        assert rootLink != null;
        assert submode != null;
        assert blendInterval >= 0f : blendInterval;

        PhysicsLink[] children = rootLink.listChildren();
        for (PhysicsLink child : children) {
            if (child instanceof BoneLink) {
                BoneLink boneLink = (BoneLink) child;
                boneLink.blendToKinematicMode(submode, blendInterval);
            } else {
                AttachmentLink attachmentLink = (AttachmentLink) child;
                if (!attachmentLink.isReleased()) {
                    attachmentLink.blendToKinematicMode(blendInterval, null);
                }
            }
            blendDescendants(child, submode, blendInterval);
        }
    }

    /**
     * Begin blending the specified link and all its descendants to the
     * specified kinematic submode.
     *
     * @param rootLink the root of the subtree to blend (not null, belongs to
     * this control)
     * @param submode the desired submode (not null)
     * @param blendInterval the duration of the blend interval (in seconds,
     * &ge;0)
     */
    private void blendSubtree(PhysicsLink rootLink, KinematicSubmode submode,
            float blendInterval) {
        assert rootLink != null;
        assert submode != null;
        assert blendInterval >= 0f : blendInterval;

        blendDescendants(rootLink, submode, blendInterval);

        if (rootLink == getTorsoLink()) {
            getTorsoLink().blendToKinematicMode(submode, blendInterval, null);
        } else if (rootLink instanceof BoneLink) {
            BoneLink boneLink = (BoneLink) rootLink;
            boneLink.blendToKinematicMode(submode, blendInterval);
        } else {
            AttachmentLink attachmentLink = (AttachmentLink) rootLink;
            if (!attachmentLink.isReleased()) {
                attachmentLink.blendToKinematicMode(blendInterval, null);
            }
        }
    }

    /**
     * Recalculate the total mass of the ragdoll, not including released
     * attachments. Also updates the location and estimated velocity of the
     * center of mass.
     */
    private void recalculateCenter() {
        double massSum = 0.0;
        Vector3f locationSum = new Vector3f();
        Vector3f velocitySum = new Vector3f();
        Vector3f tmpVector = new Vector3f();
        List<PhysicsLink> links = listLinks(PhysicsLink.class);
        for (PhysicsLink link : links) {
            if (!link.isReleased()) {
                PhysicsRigidBody rigidBody = link.getRigidBody();
                float mass = rigidBody.getMass();
                massSum += mass;

                rigidBody.getPhysicsLocation(tmpVector);
                tmpVector.multLocal(mass);
                locationSum.addLocal(tmpVector);

                link.velocity(tmpVector);
                tmpVector.multLocal(mass);
                velocitySum.addLocal(tmpVector);
            }
        }

        float invMass = (float) (1.0 / massSum);
        locationSum.mult(invMass, centerLocation);
        velocitySum.mult(invMass, centerVelocity);
        this.ragdollMass = (float) massSum;
    }
}
