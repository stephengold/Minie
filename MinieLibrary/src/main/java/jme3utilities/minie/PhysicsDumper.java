/*
 Copyright (c) 2013-2023, Stephen Gold
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
package jme3utilities.minie;

import com.jme3.app.state.AppState;
import com.jme3.bounding.BoundingBox;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.DeformableSpace;
import com.jme3.bullet.MultiBody;
import com.jme3.bullet.MultiBodyJointType;
import com.jme3.bullet.MultiBodyLink;
import com.jme3.bullet.MultiBodySpace;
import com.jme3.bullet.PhysicsSoftSpace;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.RayTestFlag;
import com.jme3.bullet.SoftBodyWorldInfo;
import com.jme3.bullet.SolverInfo;
import com.jme3.bullet.SolverMode;
import com.jme3.bullet.SolverType;
import com.jme3.bullet.collision.Activation;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.GImpactCollisionShape;
import com.jme3.bullet.collision.shapes.HeightfieldCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.MeshCollisionShape;
import com.jme3.bullet.collision.shapes.SimplexCollisionShape;
import com.jme3.bullet.collision.shapes.infos.ChildCollisionShape;
import com.jme3.bullet.debug.BulletDebugAppState;
import com.jme3.bullet.joints.New6Dof;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.joints.SixDofJoint;
import com.jme3.bullet.joints.motors.RotationalLimitMotor;
import com.jme3.bullet.joints.motors.TranslationalLimitMotor;
import com.jme3.bullet.objects.MultiBodyCollider;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsCharacter;
import com.jme3.bullet.objects.PhysicsGhostObject;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.bullet.objects.PhysicsVehicle;
import com.jme3.bullet.objects.VehicleWheel;
import com.jme3.bullet.objects.infos.Cluster;
import com.jme3.bullet.objects.infos.RigidBodyMotionState;
import com.jme3.bullet.objects.infos.SoftBodyConfig;
import com.jme3.bullet.objects.infos.SoftBodyMaterial;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import java.io.PrintStream;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.Collection;
import java.util.logging.Logger;
import jme3utilities.MyString;
import jme3utilities.Validate;
import jme3utilities.debug.Describer;
import jme3utilities.debug.Dumper;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyQuaternion;
import jme3utilities.math.MyVector3f;

/**
 * Dump Minie data structures for debugging purposes.
 * <p>
 * The level of detail can be configured dynamically.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class PhysicsDumper extends Dumper {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(PhysicsDumper.class.getName());
    /**
     * local copy of {@link com.jme3.math.Vector3f#UNIT_XYZ}
     */
    final private static Vector3f scaleIdentity = new Vector3f(1f, 1f, 1f);
    // *************************************************************************
    // fields

    /**
     * enable dumping of children in compound collision shapes
     */
    private boolean dumpChildShapes = false;
    /**
     * enable dumping of clusters in soft bodies
     */
    private boolean dumpClustersInSofts = false;
    /**
     * enable dumping of ignored objects in collision objects
     */
    private boolean dumpIgnores = false;
    /**
     * enable dumping of physics joints in bodies
     */
    private boolean dumpJointsInBodies = false;
    /**
     * enable dumping of joints in physics spaces
     */
    private boolean dumpJointsInSpaces = false;
    /**
     * enable dumping of motors in physics joints
     */
    private boolean dumpMotors = false;
    /**
     * enable dumping native IDs of physics objects
     */
    private boolean dumpNativeIDs = false;
    /**
     * enable dumping of soft-body nodes in clusters
     */
    private boolean dumpNodesInClusters = false;
    /**
     * enable dumping of nodes in soft bodies
     */
    private boolean dumpNodesInSofts = false;
    /**
     * enable dumping of collision objects in physics spaces
     */
    private boolean dumpPcos = true;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a PhysicsDumper that uses {@code System.out} for output.
     */
    public PhysicsDumper() {
        super();
        PhysicsDescriber newDescriber = new PhysicsDescriber();
        setDescriber(newDescriber);
    }

    /**
     * Instantiate a PhysicsDumper that uses the specified output stream.
     *
     * @param printStream output stream (not null)
     */
    public PhysicsDumper(PrintStream printStream) {
        super(printStream);
        PhysicsDescriber newDescriber = new PhysicsDescriber();
        setDescriber(newDescriber);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Dump the specified BulletAppState.
     *
     * @param appState the app state to dump (not null, unaffected)
     */
    public void dump(BulletAppState appState) {
        Validate.nonNull(appState, "app state");
        dumpBas(appState, "");
    }

    /**
     * Dump the specified CollisionShape.
     *
     * @param shape the shape to dump (not null, unaffected)
     * @param indent (not null)
     */
    public void dump(CollisionShape shape, String indent) {
        Validate.nonNull(shape, "shape");
        Validate.nonNull(indent, "indent");

        addLine(indent);

        PhysicsDescriber describer = getDescriber();
        String desc = describer.describe(shape);
        stream.print(desc);

        Vector3f scale = shape.getScale(null);
        desc = describer.describeScale(scale);
        addDescription(desc);

        long objectId = shape.nativeId();
        addNativeId(objectId);

        if (dumpChildShapes && shape instanceof CompoundCollisionShape) {
            String moreIndent = indent + indentIncrement();
            dumpChildren((CompoundCollisionShape) shape, moreIndent);
        }
    }

    /**
     * Dump the specified MultiBodyCollider.
     *
     * @param collider the collider to dump (not null, unaffected)
     * @param indent (not null)
     */
    public void dump(MultiBodyCollider collider, String indent) {
        Validate.nonNull(collider, "collider");
        Validate.nonNull(indent, "indent");

        addLine(indent);
        stream.print("Collider");

        PhysicsDescriber describer = getDescriber();
        String desc = describer.describeApplicationData(collider);
        stream.print(desc);
        desc = describer.describeUser(collider);
        stream.print(desc);

        if (!collider.isActive()) {
            stream.print("/inactive");
        }
        if (!collider.isContactResponse()) {
            stream.print("/NOresponse");
        }
        if (!collider.isInWorld()) {
            stream.print("/NOspace");
        }

        float mass = collider.mass();
        String massText = MyString.describe(mass);
        stream.printf(" mass=%s", massText);

        Vector3f loc = collider.getPhysicsLocation(null);
        stream.printf(" loc[%s]", MyVector3f.describe(loc));

        desc = describer.describeGroups(collider);
        stream.print(desc);

        long objectId = collider.nativeId();
        addNativeId(objectId);
        /*
         * The 2nd line has the shape and scale.
         * There may be additional lines for child shapes.
         */
        CollisionShape shape = collider.getCollisionShape();
        dump(shape, indent + " ");
        // TODO ignored objects
    }

    /**
     * Dump the specified PhysicsCharacter.
     *
     * @param character the character to dump (not null, unaffected)
     * @param indent (not null)
     */
    public void dump(PhysicsCharacter character, String indent) {
        Validate.nonNull(character, "character");
        Validate.nonNull(indent, "indent");

        stream.printf("%n%sCharacter", indent);

        PhysicsDescriber describer = getDescriber();
        String desc = describer.describeApplicationData(character);
        stream.print(desc);
        desc = describer.describeUser(character);
        stream.print(desc);

        Vector3f location = character.getPhysicsLocation(null);
        String locString = MyVector3f.describe(location);
        stream.printf(" loc[%s]", locString);

        Vector3f walk = character.getWalkDirection(null);
        stream.printf(" walk[%s]", MyVector3f.describeDirection(walk));

        Vector3f lin = character.getLinearVelocity(null);
        stream.printf(" v[%s]", MyVector3f.describe(lin));

        Vector3f ang = character.getAngularVelocity(null);
        stream.printf(" angV[%s]", MyVector3f.describe(ang));

        long objectId = character.nativeId();
        addNativeId(objectId);

        // The 2nd line has the character's configuration.
        addLine(indent);
        Vector3f grav = character.getGravity(null);
        stream.printf(" grav[%s]", MyVector3f.describe(grav));

        Vector3f up = character.getUpDirection(null);
        stream.printf(" up[%s]", MyVector3f.describeDirection(up));

        stream.print(" jumpSp=");
        float jump = character.getJumpSpeed();
        stream.print(MyString.describe(jump));

        float angularDamping = character.getAngularDamping();
        float linearDamping = character.getLinearDamping();
        stream.print("] damp[l=");
        stream.print(MyString.describe(linearDamping));
        stream.print(" a=");
        stream.print(MyString.describe(angularDamping));

        stream.print("] max[fallSp=");
        float fall = character.getFallSpeed();
        stream.print(MyString.describe(fall));

        stream.print(" pen=");
        float maxPen = character.getMaxPenetrationDepth();
        stream.print(MyString.describe(maxPen));

        stream.print(" slope=");
        float maxSlope = character.getMaxSlope();
        stream.print(MyString.describe(maxSlope));

        stream.print(" stepHt=");
        float maxStepHt = character.getStepHeight();
        stream.print(MyString.describe(maxStepHt));
        stream.print("] ");

        boolean gsTest = character.isUsingGhostSweepTest();
        if (!gsTest) {
            stream.print("NO");
        }
        stream.print("gsTest");

        desc = describer.describeGroups(character);
        stream.print(desc);
        /*
         * The 3rd line has the shape and scale.
         * There may be additional lines for child shapes.
         */
        CollisionShape shape = character.getCollisionShape();
        dump(shape, indent + " ");

        addLine(indent);
        int numIgnores = character.countIgnored();
        stream.printf(
                " with %d ignore%s", numIgnores, (numIgnores == 1) ? "" : "s");
        if (dumpIgnores && numIgnores > 0) {
            dumpIgnores(character, indent);
        }
    }

    /**
     * Dump the specified PhysicsGhostObject.
     *
     * @param ghost the ghost object to dump (not null, unaffected)
     * @param indent (not null)
     */
    public void dump(PhysicsGhostObject ghost, String indent) {
        Validate.nonNull(ghost, "ghost");
        Validate.nonNull(indent, "indent");

        stream.printf("%n%sGhost", indent);

        PhysicsDescriber describer = getDescriber();
        String desc = describer.describeApplicationData(ghost);
        stream.print(desc);
        desc = describer.describeUser(ghost);
        stream.print(desc);

        Vector3f location = ghost.getPhysicsLocation(null);
        String locString = MyVector3f.describe(location);
        stream.printf(" loc[%s]", locString);

        Quaternion orientation = ghost.getPhysicsRotation(null);
        if (!MyQuaternion.isRotationIdentity(orientation)) {
            String orientText = MyQuaternion.describe(orientation);
            stream.printf(" orient[%s]", orientText);
        }

        long objectId = ghost.nativeId();
        addNativeId(objectId);
        /*
         * The 2nd line has the shape and scale.
         * There may be additional lines for child shapes.
         */
        CollisionShape shape = ghost.getCollisionShape();
        dump(shape, indent + " ");
        /*
         * The next line has the bounding box, group info,
         * and number of ignores.
         */
        addLine(indent);
        if (shape instanceof CompoundCollisionShape
                || shape instanceof GImpactCollisionShape
                || shape instanceof HeightfieldCollisionShape
                || shape instanceof HullCollisionShape
                || shape instanceof MeshCollisionShape
                || shape instanceof SimplexCollisionShape) {
            BoundingBox aabb = shape.boundingBox(location, orientation, null);
            desc = describer.describe(aabb);
            stream.printf(" aabb[%s]", desc);
        }

        desc = describer.describeGroups(ghost);
        stream.print(desc);

        int numIgnores = ghost.countIgnored();
        stream.printf(
                " with %d ignore%s", numIgnores, (numIgnores == 1) ? "" : "s");
        if (dumpIgnores && numIgnores > 0) {
            dumpIgnores(ghost, indent);
        }
    }

    /**
     * Dump the specified PhysicsJoint in a PhysicsSpace context.
     *
     * @param joint the joint to dump (not null, unaffected)
     * @param indent (not null)
     */
    public void dump(PhysicsJoint joint, String indent) {
        Validate.nonNull(joint, "joint");
        Validate.nonNull(indent, "indent");

        String moreIndent = indent + indentIncrement();
        addLine(moreIndent);
        PhysicsDescriber describer = getDescriber();
        String desc
                = describer.describeJointInSpace(joint, dumpNativeIDs);
        stream.print(desc);

        String mmIndent = moreIndent + indentIncrement();
        if (joint instanceof SixDofJoint) {
            SixDofJoint sixDof = (SixDofJoint) joint;

            desc = describer.describeAngular(sixDof);
            stream.printf("%n%s %s", moreIndent, desc);
            desc = describer.describeLinear(sixDof);
            stream.printf("%n%s %s", moreIndent, desc);

            if (dumpMotors) {
                for (int axisI = 0; axisI < MyVector3f.numAxes; ++axisI) {
                    String axisName = MyString.axisName(axisI);
                    stream.printf("%n%srot%s: ", mmIndent, axisName);
                    RotationalLimitMotor motor
                            = sixDof.getRotationalLimitMotor(axisI);
                    desc = describer.describe(motor);
                    stream.print(desc);
                }

                TranslationalLimitMotor motor
                        = sixDof.getTranslationalLimitMotor();
                for (int axisI = 0; axisI < MyVector3f.numAxes; ++axisI) {
                    String axisName = MyString.axisName(axisI);
                    stream.printf("%n%stra%s: ", mmIndent, axisName);
                    desc = describer.describe(motor, axisI);
                    stream.print(desc);
                }
            }

        } else if (joint instanceof New6Dof) {
            New6Dof sixDof = (New6Dof) joint;

            addLine(moreIndent);
            Vector3f offset = sixDof.getPivotOffset(null);
            stream.printf(" offset[%s]", MyVector3f.describe(offset));
            Vector3f locA = sixDof.calculatedOriginA(null);
            stream.printf(" locA[%s]", MyVector3f.describe(locA));
            Vector3f locB = sixDof.calculatedOriginB(null);
            stream.printf(" locB[%s]", MyVector3f.describe(locB));

            addLine(moreIndent);
            Vector3f angles = sixDof.getAngles(null);
            stream.printf(" angles[%s]", MyVector3f.describe(angles));
            desc = sixDof.getRotationOrder().toString();
            stream.printf(" ro=%s", desc);
            Matrix3f basA = sixDof.calculatedBasisA(null);
            stream.printf(" basA[%s]", PhysicsDescriber.describeMatrix(basA));
            Matrix3f basB = sixDof.calculatedBasisB(null);
            stream.printf(" basB[%s]", PhysicsDescriber.describeMatrix(basB));

            if (dumpMotors) {
                for (int dofIndex = 0; dofIndex < 6; ++dofIndex) {
                    int axisIndex = dofIndex % MyVector3f.numAxes;
                    String tr = (dofIndex < 3) ? "T" : "R";
                    String axisName = MyString.axisName(axisIndex);
                    stream.printf("%n%s%s%s:", mmIndent, tr, axisName);
                    desc = describer.describeDof(sixDof, dofIndex);
                    stream.print(desc);
                }
            }
        }
    }

    /**
     * Dump the specified PhysicsRigidBody.
     *
     * @param body the rigid body to dump (not null, unaffected)
     * @param indent (not null)
     */
    public void dump(PhysicsRigidBody body, String indent) {
        Validate.nonNull(body, "body");
        Validate.nonNull(indent, "indent");

        addLine(indent);
        if (body instanceof PhysicsVehicle) {
            stream.print("Vehicle ");
        } else {
            stream.print("Rigid ");
        }

        String desc = MyPco.describe(body);
        stream.print(desc);

        PhysicsDescriber describer = getDescriber();
        desc = describer.describeApplicationData(body);
        stream.print(desc);
        desc = describer.describeUser(body);
        stream.print(desc);

        RigidBodyMotionState motionState = body.getMotionState();
        Vector3f msLoc = motionState.getLocation(null);
        String locString = MyVector3f.describe(msLoc);
        stream.printf(" msLoc[%s]", locString);
        Vector3f location = body.getPhysicsLocation(null);
        if (!location.equals(msLoc)) {
            locString = MyVector3f.describe(location);
            stream.printf(" loc[%s]", locString);
        }

        Quaternion orientation = body.getPhysicsRotation(null);
        if (!MyQuaternion.isRotationIdentity(orientation)) {
            String orientText = MyQuaternion.describe(orientation);
            stream.printf(" orient[%s]", orientText);
        }

        long objectId = body.nativeId();
        addNativeId(objectId);

        // 2nd line: activation state and contact parameters
        addLine(indent);
        addActivationState(body);
        addContactParameters(body);

        if (body.isDynamic()) {
            // The next 3 lines describes the dynamic properties.
            addDynamicProperties(body, indent);
        }
        /*
         * The next line has the shape and scale.
         * There may be additional lines for child shapes.
         */
        CollisionShape shape = body.getCollisionShape();
        dump(shape, indent + " ");
        /*
         * The next line has the bounding box, group info, number of wheels,
         * and number of ignores/joints.
         */
        addLine(indent);
        if (shape instanceof CompoundCollisionShape
                || shape instanceof GImpactCollisionShape
                || shape instanceof HeightfieldCollisionShape
                || shape instanceof HullCollisionShape
                || shape instanceof MeshCollisionShape
                || shape instanceof SimplexCollisionShape) {
            BoundingBox aabb = shape.boundingBox(location, orientation, null);
            desc = describer.describe(aabb);
            stream.printf(" aabb[%s]", desc);
        }

        desc = describer.describeGroups(body);
        stream.print(desc);

        stream.print(" with");
        if (body instanceof PhysicsVehicle) {
            PhysicsVehicle vehicle = (PhysicsVehicle) body;
            int numWheels = vehicle.getNumWheels();
            stream.printf(
                    " %d wheel%s", numWheels, (numWheels == 1) ? "" : "s");
            if (numWheels > 0) {
                dumpWheels(vehicle, indent, numWheels);
            } else {
                stream.print(',');
            }
        }

        int numIgnores = body.countIgnored();
        stream.printf(" %d ignore%s", numIgnores, (numIgnores == 1) ? "" : "s");
        if (dumpIgnores && numIgnores > 0) {
            dumpIgnores(body, indent);
        }

        int numJoints = body.countJoints();
        stream.printf(
                " and %d joint%s", numJoints, (numJoints == 1) ? "" : "s");
        if (dumpJointsInBodies && numJoints > 0) {
            dumpJoints(body, indent);
        }
    }

    /**
     * Dump the specified PhysicsSoftBody.
     *
     * @param body the soft body to dump (not null, unaffected)
     * @param indent (not null)
     */
    public void dump(PhysicsSoftBody body, String indent) {
        Validate.nonNull(body, "body");
        Validate.nonNull(indent, "indent");

        stream.printf("%n%sSoft ", indent);

        PhysicsDescriber describer = getDescriber();
        BoundingBox aabb = body.boundingBox(null);
        String desc = describer.describe(aabb);
        stream.print(desc);

        stream.print(" mass=");
        float mass = body.getMass();
        desc = MyString.describe(mass);
        stream.print(desc);

        stream.print(" marg=");
        float margin = body.margin();
        desc = MyString.describe(margin);
        stream.print(desc);

        long objectId = body.nativeId();
        addNativeId(objectId);

        stream.printf("%n%s  vol=", indent);
        float volume = body.volume();
        desc = MyString.describe(volume);
        stream.print(desc);

        stream.print(" wind[");
        Vector3f wind = body.windVelocity(null);
        desc = MyVector3f.describe(wind);
        stream.print(desc);
        stream.print(']');

        desc = describer.describeApplicationData(body);
        stream.print(desc);
        desc = describer.describeUser(body);
        stream.print(desc);

        int numLinks = body.countLinks();
        int numFaces = body.countFaces();
        int numTetras = body.countTetras();
        stream.printf(" with %d link%s, %d face%s, %d tetra%s",
                numLinks, (numLinks == 1) ? "" : "s",
                numFaces, (numFaces == 1) ? "" : "s",
                numTetras, (numTetras == 1) ? "" : "s");

        Quaternion orientation = body.getPhysicsRotation(null);
        if (!MyQuaternion.isRotationIdentity(orientation)) {
            desc = MyQuaternion.describe(orientation);
            stream.printf(" orient[%s]", desc);
        }

        // 3rd & 4th lines describe the config.
        SoftBodyConfig config = body.getSoftConfig();
        desc = describer.describe1(config);
        stream.printf("%n%s %s", indent, desc);
        desc = describer.describe2(config);
        stream.printf("%n%s %s", indent, desc);

        // 5th line describes the material.
        SoftBodyMaterial material = body.getSoftMaterial();
        desc = describer.describe(material);
        stream.printf("%n%s %s", indent, desc);

        // 6th line describes the world info.
        SoftBodyWorldInfo info = body.getWorldInfo();
        desc = describer.describe(info);
        stream.printf("%n%s %s ", indent, desc);
        if (!body.isWorldInfoProtected()) {
            stream.print("NOT");
        }
        stream.print("protected");
        objectId = info.nativeId();
        addNativeId(objectId);

        // 7th line describes the group info and number of anchors.
        desc = describer.describeGroups(body);
        stream.printf("%n%s%s", indent, desc);

        // physics joints in the soft body
        int numJoints = body.countJoints();
        stream.printf(
                " with %d joint%s", numJoints, (numJoints == 1) ? "" : "s");
        if (dumpJointsInBodies && numJoints > 0) {
            dumpJoints(body, indent);
            addLine(indent);
        } else {
            stream.print(',');
        }

        // clusters in the soft body
        int numClusters = body.countClusters();
        stream.printf(
                " %d cluster%s", numClusters, (numClusters == 1) ? "" : "s");
        if (dumpClustersInSofts && numClusters > 0) {
            dumpClusters(body, indent);
        } else {
            stream.print(',');
        }

        // nodes in the soft body
        int numNodes = body.countNodes();
        stream.printf(" %d node%s", numNodes, (numNodes == 1) ? "" : "s");
        int numPinned = body.countPinnedNodes();
        if (numPinned > 0) {
            stream.printf(" (%d pinned)", numPinned);
        }
        if (dumpNodesInSofts && numNodes > 0) {
            dumpNodes(body, indent);
        }
        // TODO ignored objects
    }

    /**
     * Dump the specified PhysicsSpace.
     *
     * @param space the PhysicsSpace to dump (not null, unaffected)
     */
    public void dump(PhysicsSpace space) {
        dump(space, "", null);
    }

    /**
     * Dump the specified PhysicsSpace with the specified filter. TODO dump a
     * CollisionSpace
     *
     * @param space the PhysicsSpace to dump (not null, unaffected)
     * @param indent (not null, may be empty)
     * @param filter determines which physics objects are dumped, or null to
     * dump all (unaffected)
     */
    public void dump(PhysicsSpace space, String indent,
            BulletDebugAppState.DebugAppStateFilter filter) {
        Validate.nonNull(indent, "indent");

        String type = space.getClass().getSimpleName();
        Collection<PhysicsCharacter> characters = space.getCharacterList();
        int numCharacters = characters.size();
        Collection<PhysicsGhostObject> ghosts = space.getGhostObjectList();
        int numGhosts = ghosts.size();
        stream.printf("%n%s%s with %d char%s, %d ghost%s, ",
                indent, type, numCharacters, (numCharacters == 1) ? "" : "s",
                numGhosts, (numGhosts == 1) ? "" : "s");

        Collection<PhysicsJoint> joints = space.getJointList();
        int numJoints = joints.size();
        stream.printf("%d joint%s, ", numJoints, (numJoints == 1) ? "" : "s");

        Collection<MultiBody> multibodies = new ArrayList<>(0);
        if (space instanceof MultiBodySpace) {
            multibodies = ((MultiBodySpace) space).getMultiBodyList();
            int numMultis = multibodies.size();
            stream.printf(
                    "%d multi%s, ", numMultis, (numMultis == 1) ? "" : "s");
        }

        Collection<PhysicsRigidBody> rigidBodies = space.getRigidBodyList();
        int numRigids = rigidBodies.size();
        stream.printf("%d rigid%s, ", numRigids, (numRigids == 1) ? "" : "s");

        Collection<PhysicsSoftBody> softBodies = new ArrayList<>(0);
        if (space instanceof PhysicsSoftSpace) {
            softBodies = ((PhysicsSoftSpace) space).getSoftBodyList();
            int numSofts = softBodies.size();
            stream.printf("%d soft%s, ", numSofts, (numSofts == 1) ? "" : "s");
        } else if (space instanceof DeformableSpace) {
            softBodies = ((DeformableSpace) space).getSoftBodyList();
            int numSofts = softBodies.size();
            stream.printf("%d soft%s, ", numSofts, (numSofts == 1) ? "" : "s");
        }

        int numVehicles = space.getVehicleList().size();
        stream.printf("%d vehicle%s", numVehicles,
                (numVehicles == 1) ? "" : "s");

        long spaceId = space.nativeId();
        addNativeId(spaceId);

        // 2nd line
        addLine(indent);
        PhysicsSpace.BroadphaseType bphase = space.getBroadphaseType();
        stream.printf(" bphase=%s", bphase);

        Vector3f grav = space.getGravity(null);
        stream.printf(" grav[%s] timeStep[", MyVector3f.describe(grav));
        int maxSS = space.maxSubSteps();
        if (maxSS == 0) {
            float maxTimeStep = space.maxTimeStep();
            String mtsDesc = MyString.describe(maxTimeStep);
            stream.printf("VAR max=%s", mtsDesc);
        } else {
            float accuracy = space.getAccuracy();
            String accuDesc = MyString.describe(accuracy);
            stream.printf("%s maxSS=%d", accuDesc, maxSS);
        }

        int cCount = space.countCollisionListeners();
        int cgCount = space.countCollisionGroupListeners();
        int tCount = space.countTickListeners();
        stream.printf("] listeners[c=%d cg=%d t=%d]", cCount, cgCount, tCount);

        // 3rd line: solver type and info
        addLine(indent);
        SolverType solverType = space.getSolverType();
        SolverInfo solverInfo = space.getSolverInfo();
        int iters = solverInfo.numIterations();
        float cfm = solverInfo.globalCfm();
        stream.printf(" solver[%s iters=%d cfm=%s",
                solverType, iters, MyString.describe(cfm));
        int batch = solverInfo.minBatch();
        stream.printf(" batch=%d splitImp[th=", batch);
        boolean enabledGlobally = solverInfo.isSplitImpulseEnabled();
        if (enabledGlobally) {
            stream.print("global");
        } else {
            float th = solverInfo.splitImpulseThreshold();
            stream.print(MyString.describe(th));
        }
        float erp = solverInfo.splitImpulseErp();
        stream.printf(" erp=%s]", MyString.describe(erp));
        int mode = solverInfo.mode();
        stream.printf(" mode=%s]", SolverMode.describe(mode));

        // 4th line: use flags, raytest flags, and world extent
        addLine(indent);
        if (space.isCcdWithStaticOnly()) {
            stream.print(" CCDwso");
        }
        if (space.isUsingDeterministicDispatch()) {
            stream.print(" DeterministicDispatch");
        }
        if (space.isUsingScr()) {
            stream.print(" SCR");
        }
        int rayTestFlags = space.getRayTestFlags();
        String rayTest = RayTestFlag.describe(rayTestFlags);
        stream.printf(" rayTest=%s", rayTest);

        if (bphase == PhysicsSpace.BroadphaseType.AXIS_SWEEP_3
                || bphase == PhysicsSpace.BroadphaseType.AXIS_SWEEP_3_32) {
            Vector3f worldMin = space.getWorldMin(null);
            String minDesc = MyVector3f.describe(worldMin);
            Vector3f worldMax = space.getWorldMax(null);
            String maxDesc = MyVector3f.describe(worldMax);
            stream.printf(" worldMin[%s] worldMax[%s]", minDesc, maxDesc);
        }

        // For soft spaces, 5th line has the world info.
        PhysicsDescriber describer = getDescriber();
        if (space instanceof PhysicsSoftSpace) {
            SoftBodyWorldInfo info = ((PhysicsSoftSpace) space).getWorldInfo();
            String infoDesc = describer.describe(info);
            stream.printf("%n%s %s", indent, infoDesc);
            long objectId = info.nativeId();
            addNativeId(objectId);
        }

        if (dumpPcos) {
            String moreIndent = indent + indentIncrement();
            for (PhysicsCharacter character : characters) {
                if (filter == null || filter.displayObject(character)) {
                    dump(character, moreIndent);
                }
            }
            for (PhysicsGhostObject ghost : ghosts) {
                if (filter == null || filter.displayObject(ghost)) {
                    dump(ghost, moreIndent);
                }
            }
            for (MultiBody multibody : multibodies) {
                dumpMultiBody(multibody, moreIndent, filter);
            }
            for (PhysicsRigidBody rigid : rigidBodies) {
                if (filter == null || filter.displayObject(rigid)) {
                    dump(rigid, moreIndent);
                }
            }
            for (PhysicsSoftBody soft : softBodies) {
                if (filter == null || filter.displayObject(soft)) {
                    dump(soft, moreIndent);
                }
            }
        }

        if (dumpJointsInSpaces) {
            dumpJoints(joints, indent, filter);
        }

        stream.println();
    }

    /**
     * Dump the specified BulletAppState.
     *
     * @param appState the app state to dump (not null, unaffected)
     * @param indent (not null)
     */
    public void dumpBas(BulletAppState appState, String indent) {
        Validate.nonNull(indent, "indent");

        String className = appState.getClass().getSimpleName();
        stream.print(className);

        if (appState.isEnabled()) {
            stream.print(" enabled ");

            if (!appState.isDebugEnabled()) {
                stream.print("NO");
            }
            stream.print("debug ");

            float speed = appState.getSpeed();
            String speedString = MyString.describe(speed);
            stream.printf("speed=%s", speedString);

            PhysicsSpace.BroadphaseType broadphaseType
                    = appState.getBroadphaseType();
            stream.printf(" bphase=%s", broadphaseType);

            PhysicsSpace space = appState.getPhysicsSpace();
            String moreIndent = indent + indentIncrement();
            dump(space, moreIndent, null);
        } else {
            stream.println(" disabled");
        }
    }

    /**
     * Test whether the specified dump flag is set.
     *
     * @param dumpFlag which flag to test (not null)
     * @return true if output is enabled, otherwise false
     */
    public boolean isEnabled(DumpFlags dumpFlag) {
        boolean result;

        switch (dumpFlag) {
            case BoundsInSpatials:
                result = isDumpBounds();
                break;

            case Buckets:
                result = isDumpBucket();
                break;

            case ChildShapes:
                result = dumpChildShapes;
                break;

            case ClustersInSofts:
                result = dumpClustersInSofts;
                break;

            case CullHints:
                result = isDumpCull();
                break;

            case Ignores:
                result = dumpIgnores;
                break;

            case JointsInBodies:
                result = dumpJointsInBodies;
                break;

            case JointsInSpaces:
                result = dumpJointsInSpaces;
                break;

            case MatParams:
                result = isDumpMatParam();
                break;

            case Motors:
                result = dumpMotors;
                break;

            case NativeIDs:
                result = dumpNativeIDs;
                break;

            case NodesInClusters:
                result = dumpNodesInClusters;
                break;

            case NodesInSofts:
                result = dumpNodesInSofts;
                break;

            case Overrides:
                result = isDumpOverride();
                break;

            case Pcos:
                result = dumpPcos;
                break;

            case ShadowModes:
                result = isDumpShadow();
                break;

            case Transforms:
                result = isDumpTransform();
                break;

            case UserData:
                result = isDumpUser();
                break;

            case VertexData:
                result = isDumpVertex();
                break;

            default:
                throw new IllegalArgumentException("dumpFlag = " + dumpFlag);
        }

        return result;
    }

    /**
     * Configure the specified dump flag.
     *
     * @param dumpFlag which flag to set (not null)
     * @param newValue true to enable output, false to disable it
     * @return this instance for chaining
     */
    public PhysicsDumper setEnabled(DumpFlags dumpFlag, boolean newValue) {
        switch (dumpFlag) {
            case BoundsInSpatials:
                setDumpBounds(newValue);
                break;

            case Buckets:
                setDumpBucket(newValue);
                break;

            case ChildShapes:
                dumpChildShapes = newValue;
                break;

            case ClustersInSofts:
                dumpClustersInSofts = newValue;
                break;

            case CullHints:
                setDumpCull(newValue);
                break;

            case Ignores:
                dumpIgnores = newValue;
                break;

            case JointsInBodies:
                dumpJointsInBodies = newValue;
                break;

            case JointsInSpaces:
                dumpJointsInSpaces = newValue;
                break;

            case MatParams:
                setDumpMatParam(newValue);
                break;

            case Motors:
                dumpMotors = newValue;
                break;

            case NativeIDs:
                dumpNativeIDs = newValue;
                break;

            case NodesInClusters:
                dumpNodesInClusters = newValue;
                break;

            case NodesInSofts:
                dumpNodesInSofts = newValue;
                break;

            case Overrides:
                setDumpOverride(newValue);
                break;

            case Pcos:
                dumpPcos = newValue;
                break;

            case ShadowModes:
                setDumpShadow(newValue);
                break;

            case Transforms:
                setDumpTransform(newValue);
                break;

            case UserData:
                setDumpUser(newValue);
                break;

            case VertexData:
                setDumpVertex(newValue);
                break;

            default:
                throw new IllegalArgumentException("dumpFlag = " + dumpFlag);
        }

        return this;
    }
    // *************************************************************************
    // Dumper methods

    /**
     * Create a deep copy of this dumper.
     *
     * @return a new instance, equivalent to this one, with its own Describer
     * @throws CloneNotSupportedException if the superclass isn't cloneable
     */
    @Override
    public PhysicsDumper clone() throws CloneNotSupportedException {
        PhysicsDumper clone = (PhysicsDumper) super.clone();
        return clone;
    }

    /**
     * Dump the specified AppState.
     *
     * @param appState the AppState to dump (not null, unaffected)
     * @param indent (not null)
     */
    @Override
    public void dump(AppState appState, String indent) {
        Validate.nonNull(appState, "app state");
        Validate.nonNull(indent, "indent");

        if (appState instanceof BulletAppState) {
            dumpBas((BulletAppState) appState, indent);
        } else {
            super.dump(appState, indent);
        }
    }

    /**
     * Access the Describer used by this Dumper.
     *
     * @return the pre-existing instance (not null)
     */
    @Override
    public PhysicsDescriber getDescriber() {
        Describer describer = super.getDescriber();
        PhysicsDescriber result = (PhysicsDescriber) describer;

        return result;
    }
    // *************************************************************************
    // private methods

    /**
     * Print the activation state of the specified rigid body, unless it happens
     * to be in the expected state.
     *
     * @param body (not null, unaffected)
     */
    private void addActivationState(PhysicsRigidBody body) {
        int expectedState;
        if (body.isKinematic() || body instanceof PhysicsVehicle) {
            expectedState = Activation.exempt;
        } else if (body.isActive()) {
            expectedState = Activation.active;
        } else {
            expectedState = Activation.sleeping;
        }

        int activationState = body.getActivationState();
        if (activationState != expectedState) {
            stream.printf(" act=%d", activationState);
        }
    }

    /**
     * Print the contact parameters of the specified rigid body.
     *
     * @param body (not null, unaffected)
     */
    private void addContactParameters(PhysicsRigidBody body) {
        float fric = body.getFriction();
        stream.print(" contact[fric=");
        stream.print(MyString.describe(fric));

        float rest = body.getRestitution();
        stream.print(" rest=");
        stream.print(MyString.describe(rest));

        float damp = body.getContactDamping();
        stream.print(" damp=");
        stream.print(MyString.describe(damp));

        float pth = body.getContactProcessingThreshold();
        stream.print(" pth=");
        stream.print(MyString.describe(pth));

        float stiff = body.getContactStiffness();
        stream.print(" stiff=");
        stream.print(MyString.describe(stiff));
        stream.print(']');
    }

    /**
     * Print dynamic properties of the specified rigid body.
     *
     * @param rigidBody (not null, unaffected)
     * @param indent (not null)
     */
    private void addDynamicProperties(PhysicsRigidBody rigidBody,
            String indent) {
        // first line: gravity, CCD, damping, and sleep/activation
        addLine(indent);

        Vector3f gravity = rigidBody.getGravity(null);
        String graString = MyVector3f.describe(gravity);
        stream.printf(" grav[%s] ", graString);

        if (!rigidBody.isGravityProtected()) {
            stream.print("NOT");
        }
        stream.print("protected ccd[mth=");
        float ccdMt = rigidBody.getCcdMotionThreshold();
        stream.print(MyString.describe(ccdMt));
        if (ccdMt > 0f) {
            stream.print(" r=");
            float ccdR = rigidBody.getCcdSweptSphereRadius();
            stream.print(MyString.describe(ccdR));
        }

        float angularDamping = rigidBody.getAngularDamping();
        float linearDamping = rigidBody.getLinearDamping();
        stream.print("] damp[l=");
        stream.print(MyString.describe(linearDamping));
        stream.print(" a=");
        stream.print(MyString.describe(angularDamping));

        float linearThreshold = rigidBody.getLinearSleepingThreshold();
        float angularThreshold = rigidBody.getAngularSleepingThreshold();
        stream.print("] sleep[lth=");
        stream.print(MyString.describe(linearThreshold));
        stream.print(" ath=");
        stream.print(MyString.describe(angularThreshold));
        if (rigidBody.isActive()) {
            float deactivationTime = rigidBody.getDeactivationTime();
            stream.print(" time=");
            stream.print(MyString.describe(deactivationTime));
        }
        stream.print(']');

        // 2nd line: linear velocity, applied force, linear factor
        addLine(indent);

        Vector3f v = rigidBody.getLinearVelocity(null);
        stream.printf(" v[%s]", MyVector3f.describe(v));
        Vector3f force = rigidBody.totalAppliedForce(null);
        stream.printf(" force[%s]", MyVector3f.describe(force));
        Vector3f lFact = rigidBody.getLinearFactor(null);
        stream.printf(" lFact[%s]", MyVector3f.describe(lFact));

        // 3rd line: inertia, angular velocity, applied torque, angular factor
        addLine(indent);

        stream.print(" inert[");
        Vector3f iiLocal = rigidBody.getInverseInertiaLocal(null);
        Vector3f inert = scaleIdentity.divide(iiLocal);
        stream.print(MyVector3f.describe(inert));
        stream.print(']');

        Vector3f angularVelocity = rigidBody.getAngularVelocity(null);
        stream.printf(" w[%s]", MyVector3f.describe(angularVelocity));
        Vector3f torq = rigidBody.totalAppliedTorque(null);
        stream.printf(" torq[%s]", MyVector3f.describe(torq));
        Vector3f aFact = rigidBody.getAngularFactor(null);
        stream.printf(" aFact[%s]", MyVector3f.describe(aFact));
    }

    /**
     * Add a native ID, if the flag is set.
     *
     * @param id the unique identifier (not zero)
     */
    private void addNativeId(long id) {
        if (dumpNativeIDs) {
            stream.print(" #");
            String hex = Long.toHexString(id);
            stream.print(hex);
        }
    }

    /**
     * Generate a textual description of the indexed vector in the specified
     * FloatBuffer.
     *
     * @param buffer the buffer to read (not null, unaffected)
     * @param vectorIndex the index of the vector in the buffer (&ge;0)
     * @return descriptive text (not null, not empty)
     */
    private static String describeVector(FloatBuffer buffer, int vectorIndex) {
        Vector3f vector = new Vector3f();
        MyBuffer.get(buffer, MyVector3f.numAxes * vectorIndex, vector);
        String locString = MyVector3f.describe(vector);

        return locString;
    }

    /**
     * Dump all children of the specified CompoundCollisionShape.
     *
     * @param parent the shape to dump (not null, unaffected)
     * @param indent (not null)
     */
    private void dumpChildren(CompoundCollisionShape parent, String indent) {
        PhysicsDescriber describer = getDescriber();
        ChildCollisionShape[] children = parent.listChildren();
        for (ChildCollisionShape child : children) {
            addLine(indent);
            CollisionShape baseShape = child.getShape();
            String desc = describer.describe(baseShape);
            stream.print(desc);

            Vector3f offset = child.copyOffset(null);
            if (!MyVector3f.isZero(offset)) {
                stream.print(" offset[");
                desc = MyVector3f.describe(offset);
                stream.print(desc);
                stream.print(']');
            }

            Quaternion rot = child.copyRotation(null);
            if (!MyQuaternion.isRotationIdentity(rot)) {
                stream.print(" rot[");
                desc = MyQuaternion.describe(rot);
                stream.print(desc);
                stream.print(']');
            }

            Vector3f scale = baseShape.getScale(null);
            desc = describer.describeScale(scale);
            addDescription(desc);

            long objectId = baseShape.nativeId();
            addNativeId(objectId);
        }
    }

    /**
     * Dump all clusters in the specified soft body.
     *
     * @param softBody the body to dump (not null, unaffected)
     * @param indent (not null)
     */
    private void dumpClusters(PhysicsSoftBody softBody, String indent) {
        stream.print(':');
        FloatBuffer coms = softBody.copyClusterCenters(null);
        FloatBuffer masses = softBody.copyClusterMasses(null);
        int numClusters = softBody.countClusters();
        for (int clusterIndex = 0; clusterIndex < numClusters; ++clusterIndex) {
            stream.printf("%n%s  [%d] com[", indent, clusterIndex);
            String desc = describeVector(coms, clusterIndex);
            stream.print(desc);

            stream.print("] mass=");
            float mass = masses.get(clusterIndex);
            stream.print(MyString.describe(mass));

            stream.print(" damp[ang=");
            float angularDamping
                    = softBody.get(Cluster.AngularDamping, clusterIndex);
            stream.print(MyString.describe(angularDamping));

            stream.print(" lin=");
            float linearDamping
                    = softBody.get(Cluster.LinearDamping, clusterIndex);
            stream.print(MyString.describe(linearDamping));

            stream.print(" node=");
            float nodeDamping = softBody.get(Cluster.NodeDamping, clusterIndex);
            stream.print(MyString.describe(nodeDamping));

            stream.print("] match=");
            float matching = softBody.get(Cluster.Matching, clusterIndex);
            stream.print(MyString.describe(matching));

            stream.print(" scif=");
            float selfImpulse = softBody.get(Cluster.SelfImpulse, clusterIndex);
            stream.print(MyString.describe(selfImpulse));

            stream.print(" maxSci=");
            float maxSelfImpulse
                    = softBody.get(Cluster.MaxSelfImpulse, clusterIndex);
            stream.print(MyString.describe(maxSelfImpulse));

            int numNodes = softBody.countNodesInCluster(clusterIndex);
            stream.printf("  %d node%s", numNodes, (numNodes == 1) ? "" : "s");

            if (dumpMotors) {
                dumpNodesInCluster(softBody, clusterIndex);
            }
        }
        addLine(indent);
    }

    /**
     * Dump all ignored objects in the specified collision object.
     *
     * @param pco the body to dump (not null, unaffected)
     * @param indent (not null)
     */
    private void dumpIgnores(PhysicsCollisionObject pco, String indent) {
        stream.print(':');
        PhysicsCollisionObject[] ignoreList = pco.listIgnoredPcos();
        String moreIndent = indent + indentIncrement();
        for (PhysicsCollisionObject otherPco : ignoreList) {
            addLine(moreIndent);
            PhysicsDescriber describer = getDescriber();
            String desc = describer.describePco(otherPco, dumpNativeIDs);
            stream.print(desc);
        }
        addLine(indent);
    }

    /**
     * Dump the specified joints in a PhysicsSpace context.
     *
     * @param joints (not null, unaffected)
     * @param indent (not null, may be empty)
     * @param filter determines which physics objects are dumped, or null to
     * dump all (unaffected)
     */
    private void dumpJoints(Collection<? extends PhysicsJoint> joints,
            String indent, BulletDebugAppState.DebugAppStateFilter filter) {
        for (PhysicsJoint joint : joints) {
            if (filter == null || filter.displayObject(joint)) {
                dump(joint, indent);
            }
        }
    }

    /**
     * Dump all joints in the specified body.
     *
     * @param body the body to dump (not null, unaffected)
     * @param indent (not null)
     */
    private void dumpJoints(PhysicsBody body, String indent) {
        stream.print(':');
        PhysicsJoint[] joints = body.listJoints();
        PhysicsDescriber describer = getDescriber();
        String moreIndent = indent + indentIncrement();
        for (PhysicsJoint joint : joints) {
            String desc
                    = describer.describeJointInBody(joint, body, dumpNativeIDs);
            stream.printf("%n%s%s", moreIndent, desc);
        }
    }

    /**
     * Dump the specified MultiBodyLink.
     *
     * @param link the link to dump (not null, unaffected)
     * @param indent (not null)
     * @param filter for colliders (may be null, unaffected)
     */
    private void dumpLink(MultiBodyLink link, String indent,
            BulletDebugAppState.DebugAppStateFilter filter) {
        addLine(indent);
        int index = link.index();
        MultiBodyJointType jointType = link.jointType();
        stream.printf("Link[%d] %s->", index, jointType);

        MultiBodyLink parent = link.getParentLink();
        if (parent == null) {
            stream.print("base");
        } else {
            int parentIndex = parent.index();
            stream.print(parentIndex);
        }

        long objectId = link.nativeId();
        addNativeId(objectId);

        MultiBodyCollider collider = link.getCollider();
        if (collider != null) {
            if (filter == null || filter.displayObject(collider)) {
                dump(collider, indent + indentIncrement());
            }
        }
    }

    /**
     * Dump the specified MultiBody.
     *
     * @param multibody the multibody to dump (not null, unaffected)
     * @param indent (not null)
     * @param filter for colliders (may be null, unaffected)
     */
    private void dumpMultiBody(MultiBody multibody, String indent,
            BulletDebugAppState.DebugAppStateFilter filter) {

        addLine(indent);
        stream.print("MultiBody");
        PhysicsDescriber describer = getDescriber();
        String desc = describer.describeGroups(multibody);
        stream.print(desc);

        if (multibody.hasFixedBase()) {
            stream.print("/fixed");
        }
        if (!multibody.isUsingGyroTerm()) {
            stream.print("/NOgyro");
        }
        if (!multibody.canSleep()) {
            stream.print("/NOsleep");
        }
        if (multibody.isUsingRK4()) {
            stream.print("/RK4");
        }

        int numColliders = multibody.listColliders().size();
        int numLinks = multibody.countConfiguredLinks();
        stream.printf(" with %d collider%s, %d link%s",
                numColliders, (numColliders == 1) ? "" : "s",
                numLinks, (numLinks == 1) ? "" : "s");

        long objectId = multibody.nativeId();
        addNativeId(objectId);

        addLine(indent);
        float angularDamping = multibody.angularDamping();
        float linearDamping = multibody.linearDamping();
        stream.print(" damp[l=");
        stream.print(MyString.describe(linearDamping));
        stream.print(" a=");
        stream.print(MyString.describe(angularDamping));
        stream.print(']');

        float maxImp = multibody.maxAppliedImpulse();
        float maxV = multibody.maxCoordinateVelocity();
        stream.print(" max[imp=");
        stream.print(MyString.describe(maxImp));
        stream.print(" v=");
        stream.print(MyString.describe(maxV));
        stream.print(']');

        String moreIndent = indent + indentIncrement();
        MultiBodyCollider collider = multibody.getBaseCollider();
        if (collider != null) {
            if (filter == null || filter.displayObject(collider)) {
                dump(collider, moreIndent);
            }
        }
        for (int linkIndex = 0; linkIndex < numLinks; ++linkIndex) {
            MultiBodyLink link = multibody.getLink(linkIndex);
            dumpLink(link, moreIndent, filter);
        }
    }

    /**
     * Dump all nodes in the specified soft body.
     *
     * @param softBody the soft body to dump (not null, unaffected)
     * @param indent (not null)
     */
    private void dumpNodes(PhysicsSoftBody softBody, String indent) {
        stream.print(':');

        FloatBuffer locations = softBody.copyLocations(null);
        FloatBuffer masses = softBody.copyMasses(null);
        FloatBuffer velocities = softBody.copyVelocities(null);
        IntBuffer linkIndices = softBody.copyLinks(null);
        int numNodes = softBody.countNodes();
        int numLinks = softBody.countLinks();
        for (int nodeIndex = 0; nodeIndex < numNodes; ++nodeIndex) {
            int degree = MyBuffer
                    .frequency(linkIndices, 0, 2 * numLinks, nodeIndex);
            float nodeMass = masses.get(nodeIndex);
            String locString = describeVector(locations, nodeIndex);
            String vString = describeVector(velocities, nodeIndex);
            stream.printf("%n%s  [%d] deg=%d mass=%s loc[%s] v[%s]",
                    indent, nodeIndex, degree, MyString.describe(nodeMass),
                    locString, vString);
        }
    }

    /**
     * Dump the indices of all nodes in the specified cluster.
     *
     * @param softBody the soft body to dump (not null, unaffected)
     * @param clusterIndex which cluster (&ge;0, &lt;numClusters)
     */
    private void dumpNodesInCluster(
            PhysicsSoftBody softBody, int clusterIndex) {
        IntBuffer nodeIndices = softBody.listNodesInCluster(clusterIndex, null);
        int numIndices = nodeIndices.capacity();
        int numNodesInBody = softBody.countNodes();
        if (numIndices == numNodesInBody) {
            stream.print("(all)");
            return;
        }

        // convert the IntBuffer to a BitSet
        BitSet bitSet = new BitSet(numNodesInBody);
        for (int i = 0; i < numIndices; ++i) {
            int nodeIndex = nodeIndices.get(i);
            bitSet.set(nodeIndex);
        }

        stream.print('(');
        boolean addSeparators = false;
        for (int nodeIndex = 0; nodeIndex < numNodesInBody; ++nodeIndex) {
            if (bitSet.get(nodeIndex)) {
                if (addSeparators) {
                    stream.print(',');
                } else {
                    addSeparators = true;
                }
                int runLength = bitSet.nextClearBit(nodeIndex) - nodeIndex;
                if (runLength < 3) {
                    stream.printf("%d", nodeIndex);
                } else {
                    int endIndex = nodeIndex + runLength - 1;
                    stream.printf("%d-%d", nodeIndex, endIndex);
                    nodeIndex = endIndex;
                }
            }
        }
        stream.print(')');
    }

    /**
     * Dump wheels in the specified vehicle.
     *
     * @param vehicle the vehicle to dump (not null, unaffected)
     * @param indent (not null)
     * @param numWheels the number of wheels to dump (&gt;0)
     */
    private void dumpWheels(PhysicsVehicle vehicle, String indent,
            int numWheels) {
        stream.print(':');
        PhysicsDescriber describer = getDescriber();
        String moreIndent = indent + indentIncrement();
        for (int wheelIndex = 0; wheelIndex < numWheels; ++wheelIndex) {
            stream.printf("%n%s[%d] ", moreIndent, wheelIndex);
            VehicleWheel wheel = vehicle.getWheel(wheelIndex);
            String desc = describer.describe(wheel);
            stream.print(desc);

            stream.printf("%n%s ", moreIndent);
            desc = describer.describe2(wheel);
            stream.print(desc);

            stream.print(" raycast=");
            float raycast = vehicle.castRay(wheelIndex);
            stream.print(MyString.describe(raycast));

            if (raycast >= 0f) {
                stream.print(" skid=");
                float skid = wheel.getSkidInfo();
                stream.print(MyString.describe(skid));
            }
        }
        addLine(indent);
    }
}
