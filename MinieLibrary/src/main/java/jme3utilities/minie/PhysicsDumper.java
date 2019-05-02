/*
 Copyright (c) 2013-2019, Stephen Gold
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
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.RayTestFlag;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.objects.PhysicsCharacter;
import com.jme3.bullet.objects.PhysicsGhostObject;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsVehicle;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import java.io.PrintStream;
import java.util.Collection;
import java.util.logging.Logger;
import jme3utilities.MyString;
import jme3utilities.Validate;
import jme3utilities.debug.Describer;
import jme3utilities.debug.Dumper;
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
    // *************************************************************************
    // fields

    /**
     * enable dumping of joints in rigid bodies
     */
    private boolean dumpJointsInBodies = false;
    /**
     * enable dumping of joints in physics spaces
     */
    private boolean dumpJointsInSpaces = false;
    /**
     * enable dumping of collision objects in physics spaces
     */
    private boolean dumpPcos = true;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a PhysicsDumper that uses System.out for output.
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
     * Dump the specified PhysicsCharacter.
     *
     * @param character the character to dump (not null, unaffected)
     * @param indent (not null)
     */
    public void dump(PhysicsCharacter character, String indent) {
        Validate.nonNull(indent, "indent");

        long objectId = character.getObjectId();
        stream.printf("%n%sCharacter #%s", indent, Long.toHexString(objectId));

        String desc = getDescriber().describeUser(character);
        stream.print(desc);

        Vector3f location = character.getPhysicsLocation();
        String locString = MyVector3f.describe(location);
        stream.printf(" loc[%s]", locString);
    }

    /**
     * Dump the specified PhysicsGhostObject.
     *
     * @param ghost the ghost object to dump (not null, unaffected)
     * @param indent (not null)
     */
    public void dump(PhysicsGhostObject ghost, String indent) {
        Validate.nonNull(indent, "indent");

        long objectId = ghost.getObjectId();
        stream.printf("%n%sGhost #%s", indent, Long.toHexString(objectId));

        String desc = getDescriber().describeUser(ghost);
        stream.print(desc);

        Vector3f location = ghost.getPhysicsLocation(null);
        String locString = MyVector3f.describe(location);
        stream.printf(" loc[%s]", locString);
    }

    /**
     * Dump the specified PhysicsRigidBody.
     *
     * @param body the rigid body to dump (not null, unaffected)
     * @param indent (not null)
     */
    public void dump(PhysicsRigidBody body, String indent) {
        Validate.nonNull(indent, "indent");

        long objectId = body.getObjectId();
        stream.printf("%n%sRigid #%s ", indent, Long.toHexString(objectId));

        String desc = MyPco.describe(body);
        stream.print(desc);

        desc = getDescriber().describeUser(body);
        stream.print(desc);

        Vector3f location = body.getPhysicsLocation(null);
        String locString = MyVector3f.describe(location);
        stream.printf(" loc[%s]", locString);

        Quaternion orientation = body.getPhysicsRotation(null);
        if (!MyQuaternion.isRotationIdentity(orientation)) {
            String orientText = MyQuaternion.describe(orientation);
            stream.printf(" orient[%s]", orientText);
        }

        if (body.isDynamic()) {
            Vector3f velocity = body.getLinearVelocity(null);
            String velString = MyVector3f.describe(velocity);
            stream.printf(" v[%s]", velString);

            Vector3f gravity = body.getGravity(null);
            String graString = MyVector3f.describe(gravity);
            stream.printf(" grav[%s]", graString);

            float linST = body.getLinearSleepingThreshold();
            stream.print(" linST=" + MyString.describe(linST));
        }
        /*
         * 2nd line has the shape, group info, and the number of joints.
         */
        CollisionShape shape = body.getCollisionShape();
        PhysicsDescriber describer = getDescriber();
        desc = describer.describe(shape);
        stream.printf("%n%s %s", indent, desc);

        Vector3f scale = shape.getScale(null);
        desc = describer.describeScale(scale);
        if (!desc.isEmpty()) {
            stream.print(" " + desc);
        }

        int group = body.getCollisionGroup();
        if (group != PhysicsCollisionObject.COLLISION_GROUP_01) {
            stream.printf(" group=0x%x", group);
        }
        int groupMask = body.getCollideWithGroups();
        if (groupMask != PhysicsCollisionObject.COLLISION_GROUP_01) {
            stream.printf(" mask=0x%x", groupMask);
        }

        PhysicsJoint[] joints = body.listJoints();
        stream.printf(" joints=%d", joints.length);
        if (dumpJointsInBodies) {
            if (joints.length > 0) {
                stream.print(":");
            }
            /*
             * Each joint gets its own line in the dump.
             */
            for (PhysicsJoint joint : joints) {
                desc = describer.describeJointInBody(joint, body);
                stream.printf("%n%s  %s", indent, desc);
            }
        }
    }

    /**
     * Dump the specified PhysicsSpace.
     *
     * @param space the PhysicsSpace to dump (not null, unaffected)
     */
    public void dump(PhysicsSpace space) {
        dump(space, "");
    }

    /**
     * Dump the specified PhysicsSpace.
     *
     * @param space the PhysicsSpace to dump (not null, unaffected)
     * @param indent (not null)
     */
    public void dump(PhysicsSpace space, String indent) {
        Validate.nonNull(indent, "indent");

        String type = space.getClass().getSimpleName();
        long spaceId = space.getSpaceId();
        Collection<PhysicsCharacter> characters = space.getCharacterList();
        int numCharacters = characters.size();
        Collection<PhysicsGhostObject> ghosts = space.getGhostObjectList();
        int numGhosts = ghosts.size();
        stream.printf("%n%s%s #%s with %d char%s, %d ghost%s, ",
                indent, type, Long.toHexString(spaceId),
                numCharacters, (numCharacters == 1) ? "" : "s",
                numGhosts, (numGhosts == 1) ? "" : "s");

        Collection<PhysicsJoint> joints = space.getJointList();
        int numJoints = joints.size();
        stream.printf("%d joint%s, ", numJoints, (numJoints == 1) ? "" : "s");

        Collection<PhysicsRigidBody> rigidBodies = space.getRigidBodyList();
        int numRigids = rigidBodies.size();
        Collection<PhysicsVehicle> vehicles = space.getVehicleList();
        int numVehicles = vehicles.size();
        stream.printf("%d rigid%s, %d vehicle%s",
                numRigids, (numRigids == 1) ? "" : "s",
                numVehicles, (numVehicles == 1) ? "" : "s");
        /*
         * 2nd line
         */
        float accuracy = space.getAccuracy();
        String accuString = MyString.describe(accuracy);
        PhysicsSpace.BroadphaseType broadphaseType = space.getBroadphaseType();
        Vector3f gravity = space.getGravity(null);
        String gravString = MyVector3f.describe(gravity);
        int maxSubSteps = space.maxSubSteps();
        stream.printf("%n%s accu=%s bphase=%s grav[%s] maxStep=%d",
                indent, accuString, broadphaseType, gravString, maxSubSteps);
        /*
         * 3rd line
         */
        int numIterations = space.getSolverNumIterations();
        int rayTestFlags = space.getRayTestFlags();
        PhysicsDescriber describer = getDescriber();
        String rtText = RayTestFlag.describe(rayTestFlags);
        Vector3f worldMin = space.getWorldMin(null);
        String minString = MyVector3f.describe(worldMin);
        Vector3f worldMax = space.getWorldMax(null);
        String maxString = MyVector3f.describe(worldMax);
        stream.printf("%n%s iters=%d rayTest[%s] worldMin[%s] worldMax[%s]",
                indent, numIterations, rtText, minString, maxString);

        if (dumpPcos) {
            String moreIndent = indent + indentIncrement();
            for (PhysicsCharacter character : characters) {
                dump(character, moreIndent);
            }
            for (PhysicsGhostObject ghost : ghosts) {
                dump(ghost, moreIndent);
            }
            for (PhysicsRigidBody rigid : rigidBodies) {
                dump(rigid, moreIndent);
            }
            for (PhysicsVehicle vehicle : vehicles) {
                dump(vehicle, moreIndent);
            }
        }

        if (dumpJointsInSpaces) {
            String moreIndent = indent + indentIncrement();
            for (PhysicsJoint joint : joints) {
                String desc = describer.describeJointInSpace(joint);
                stream.printf("%n%s%s", moreIndent, desc);
            }
        }
    }

    /**
     * Dump the specified PhysicsVehicle.
     *
     * @param vehicle the vehicle to dump (not null, unaffected)
     * @param indent (not null)
     */
    public void dump(PhysicsVehicle vehicle, String indent) {
        Validate.nonNull(indent, "indent");

        long objectId = vehicle.getObjectId();
        float mass = vehicle.getMass();
        stream.printf("%sVehicle #%s mass=%f", indent,
                Long.toHexString(objectId), mass);

        String desc = getDescriber().describeUser(vehicle);
        stream.print(desc);

        Vector3f location = vehicle.getPhysicsLocation(null);
        String locString = MyVector3f.describe(location);
        stream.printf(" loc[%s]", locString);
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
            dump(space, indent);
        } else {
            stream.printf(" disabled");
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
            case Buckets:
                result = isDumpBucket();
                break;

            case CullHints:
                result = isDumpCull();
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

            default:
                throw new IllegalArgumentException(dumpFlag.toString());
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
            case Buckets:
                setDumpBucket(newValue);
                break;

            case CullHints:
                setDumpCull(newValue);
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

            default:
                throw new IllegalArgumentException(dumpFlag.toString());
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
     * @param appState the app state to dump (not null, unaffected)
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
     * Access the describer used by this dumper.
     *
     * @return the pre-existing instance (not null)
     */
    @Override
    public PhysicsDescriber getDescriber() {
        Describer describer = super.getDescriber();
        PhysicsDescriber result = (PhysicsDescriber) describer;

        return result;
    }
}
