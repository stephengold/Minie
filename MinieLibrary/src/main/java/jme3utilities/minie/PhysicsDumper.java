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
     * Dump the specified BulletAppState. TODO re-order methods
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
        stream.printf(" loc=[%s]", locString);
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
        stream.printf(" loc=[%s]", locString);
    }

    /**
     * Dump the specified PhysicsJoint. TODO move guts to PhysicsDescriber
     *
     * @param joint the joint to dump (not null, unaffected)
     * @param indent (not null)
     */
    public void dump(PhysicsJoint joint, String indent) {
        Validate.nonNull(indent, "indent");

        String type = joint.getClass().getSimpleName();
        if (type.endsWith("Joint")) {
            type = MyString.removeSuffix(type, "Joint");
        }
        long objectId = joint.getObjectId();
        stream.printf("%n%s%s #%s", indent, type, Long.toHexString(objectId));

        int numDyn = 0;

        PhysicsRigidBody bodyA = joint.getBodyA();
        if (bodyA != null) {
            long aId = joint.getBodyA().getObjectId();
            stream.printf(" a=%s", Long.toHexString(aId));
            if (!bodyA.isInWorld()) {
                stream.print("_NOT_IN_WORLD");
            }
            if (bodyA.isDynamic()) {
                ++numDyn;
            }
        }

        PhysicsRigidBody bodyB = joint.getBodyB();
        if (bodyB != null) {
            long bId = bodyB.getObjectId();
            stream.printf(" b=%s", Long.toHexString(bId));
            if (!bodyB.isInWorld()) {
                stream.print("_NOT_IN_WORLD");
            }
            if (bodyB.isDynamic()) {
                ++numDyn;
            }
        }

        if (!joint.isEnabled()) {
            stream.print("   DISABLED");
        }
        if (numDyn == 0) {
            stream.printf("   NO_DYNAMIC_END");
        }

        float bit = joint.getBreakingImpulseThreshold();
        if (bit != Float.MAX_VALUE) {
            stream.printf(" bit=%s", Float.toString(bit));
        }
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
        stream.printf("%n%sBody #%s ", indent, Long.toHexString(objectId));

        String desc = MyObject.describe(body);
        stream.print(desc);

        desc = getDescriber().describeUser(body);
        stream.print(desc);

        Vector3f location = body.getPhysicsLocation(null);
        String locString = MyVector3f.describe(location);
        stream.printf(" loc=[%s]", locString);

        Quaternion orientation = body.getPhysicsRotation(null);
        if (!MyQuaternion.isRotationIdentity(orientation)) {
            String orient = MyQuaternion.describe(orientation);
            stream.printf(" orient=[%s]", orient);
        }

        if (body.isDynamic()) {
            Vector3f velocity = body.getLinearVelocity(null);
            String velString = MyVector3f.describe(velocity);
            stream.printf(" vel=[%s]", velString);

            Vector3f gravity = body.getGravity(null);
            String graString = MyVector3f.describe(gravity);
            stream.printf(" gra=[%s]", graString);

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
        if (joints.length > 0) {
            stream.print(":");
        }
        /*
         * Each joint has its own line in the dump.
         */
        for (PhysicsJoint joint : joints) {
            String type = joint.getClass().getSimpleName();
            if (type.endsWith("Joint")) {
                type = MyString.removeSuffix(type, "Joint");
            }
            long jointId = joint.getObjectId();
            stream.printf("%n%s  %s #%s ", indent, type,
                    Long.toHexString(jointId));

            if (!joint.isEnabled()) {
                stream.print("DISABLED ");
            }

            long otherId = 0L;
            Vector3f pivot;
            if (joint.getBodyA() == body) {
                PhysicsRigidBody bodyB = joint.getBodyB();
                if (bodyB != null) {
                    otherId = bodyB.getObjectId();
                }
                pivot = joint.getPivotA(null);
            } else {
                assert joint.getBodyB() == body;
                PhysicsRigidBody bodyA = joint.getBodyA();
                if (bodyA != null) {
                    otherId = joint.getBodyA().getObjectId();
                }
                pivot = joint.getPivotB(null);
            }
            if (otherId == 0L) {
                stream.print("single-ended");
            } else {
                stream.printf("to=#%s", Long.toHexString(otherId));
            }
            stream.printf(" piv=[%s]", MyVector3f.describe(pivot));
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
     * Dump the specified PhysicsSpace. TODO joint list should be optional
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
        stream.printf("%n%s%s #%s contains %d character%s, %d ghost%s, ",
                indent, type, Long.toHexString(spaceId),
                numCharacters, (numCharacters == 1) ? "" : "s",
                numGhosts, (numGhosts == 1) ? "" : "s");

        Collection<PhysicsJoint> joints = space.getJointList();
        int numJoints = joints.size();
        Collection<PhysicsRigidBody> rigidBodies = space.getRigidBodyList();
        int numBodies = rigidBodies.size();
        Collection<PhysicsVehicle> vehicles = space.getVehicleList();
        int numVehicles = vehicles.size();
        stream.printf("%d joint%s, %d rigid bod%s, and %d vehicle%s",
                numJoints, (numJoints == 1) ? "" : "s",
                numBodies, (numBodies == 1) ? "y" : "ies",
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
        stream.printf("%n%s accu=%s, bphase=%s, grav=[%s], maxStep=%d",
                indent, accuString, broadphaseType, gravString, maxSubSteps);
        /*
         * 3rd line
         */
        int numIterations = space.getSolverNumIterations();
        int rayTestFlags = space.getRayTestFlags();
        PhysicsDescriber describer = getDescriber();
        String rtText = describer.describeRayTestFlags(rayTestFlags);
        Vector3f worldMin = space.getWorldMin(null);
        String minString = MyVector3f.describe(worldMin);
        Vector3f worldMax = space.getWorldMax(null);
        String maxString = MyVector3f.describe(worldMax);
        stream.printf("%n%s iters=%d, rayTest=(%s), wMin=[%s], wMax=[%s]",
                indent, numIterations, rtText, minString, maxString);

        String moreIndent = indent + indentIncrement();
        for (PhysicsCharacter character : characters) {
            dump(character, moreIndent);
        }
        for (PhysicsGhostObject ghost : ghosts) {
            dump(ghost, moreIndent);
        }
        for (PhysicsJoint joint : joints) {
            dump(joint, moreIndent);
        }
        for (PhysicsRigidBody rigid : rigidBodies) {
            dump(rigid, moreIndent);
        }
        for (PhysicsVehicle vehicle : vehicles) {
            dump(vehicle, moreIndent);
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
        stream.printf(" loc=[%s]", locString);
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
