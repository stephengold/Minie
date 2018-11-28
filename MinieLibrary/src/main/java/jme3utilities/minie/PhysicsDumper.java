/*
 Copyright (c) 2013-2018, Stephen Gold
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

import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
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
import jme3utilities.debug.Describer;
import jme3utilities.debug.Dumper;
import jme3utilities.math.MyQuaternion;

/**
 * Dump portions of a jME3 scene graph for debugging.
 * <p>
 * {@link #dump(com.jme3.scene.Spatial)} is the usual interface to this class.
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
     * Instantiate a dumper that will use System.out for output.
     */
    public PhysicsDumper() {
        super();
        PhysicsDescriber newDescriber = new PhysicsDescriber();
        setDescriber(newDescriber);
    }

    /**
     * Instantiate a dumper that will use the specified output stream.
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
     * Dump the specified Bullet app state.
     *
     * @param appState the app state to dump (not null, unaffected)
     */
    public void dump(BulletAppState appState) {
        stream.printf("%nBulletAppState ");
        if (appState.isEnabled()) {
            stream.print("enabled ");
            if (!appState.isDebugEnabled()) {
                stream.print("NO");
            }
            stream.print("debug ");
            float speed = appState.getSpeed();
            stream.printf("speed=%f", speed);

            PhysicsSpace.BroadphaseType broadphaseType
                    = appState.getBroadphaseType();
            stream.printf(" bphase=%s", broadphaseType);

            PhysicsSpace space = appState.getPhysicsSpace();
            dump(space);
        } else {
            stream.printf("disabled");
        }
    }

    /**
     * Dump the specified physics character.
     *
     * @param character the character to dump (not null, unaffected)
     */
    public void dump(PhysicsCharacter character) {
        long objectId = character.getObjectId();
        stream.printf("  Character #%s", Long.toHexString(objectId));

        String desc = MyObject.describeUser(character);
        stream.print(desc);

        Vector3f location = character.getPhysicsLocation();
        stream.printf(" loc=[%.3f, %.3f, %.3f]",
                location.x, location.y, location.z);

        stream.println();

    }

    /**
     * Dump the specified ghost object.
     *
     * @param ghost the ghost object to dump (not null, unaffected)
     */
    public void dump(PhysicsGhostObject ghost) {
        long objectId = ghost.getObjectId();
        stream.printf("  Ghost #%s", Long.toHexString(objectId));

        String desc = MyObject.describeUser(ghost);
        stream.print(desc);

        Vector3f location = ghost.getPhysicsLocation(null);
        stream.printf(" loc=[%.3f, %.3f, %.3f]",
                location.x, location.y, location.z);

        stream.println();
    }

    /**
     * Dump the specified joint.
     *
     * @param joint the joint to dump (not null, unaffected)
     */
    public void dump(PhysicsJoint joint) {
        String type = joint.getClass().getSimpleName();
        if (type.endsWith("Joint")) {
            type = MyString.removeSuffix(type, "Joint");
        }

        long objectId = joint.getObjectId();
        stream.printf("  %s #%s", type, Long.toHexString(objectId));

        if (!joint.isEnabled()) {
            stream.print(" DISABLED");
        }

        long aId = joint.getBodyA().getObjectId();
        stream.printf(" a=%s", Long.toHexString(aId));

        PhysicsRigidBody bodyB = joint.getBodyB();
        if (bodyB != null) {
            long bId = bodyB.getObjectId();
            stream.printf(" b=%s", Long.toHexString(bId));
        }

        float bit = joint.getBreakingImpulseThreshold();
        if (bit != Float.MAX_VALUE) {
            stream.printf(" bit=%s", Float.toString(bit));
        }

        stream.println();
    }

    /**
     * Dump the specified rigid body.
     *
     * @param body the rigid body to dump (not null, unaffected)
     */
    public void dump(PhysicsRigidBody body) {
        long objectId = body.getObjectId();
        stream.printf("  Body #%s ", Long.toHexString(objectId));

        String desc = MyObject.describe(body);
        stream.print(desc);

        desc = MyObject.describeUser(body);
        stream.print(desc);

        Vector3f location = body.getPhysicsLocation(null);
        stream.printf(" loc=[%.3f, %.3f, %.3f]",
                location.x, location.y, location.z);

        Quaternion orientation = body.getPhysicsRotation(null);
        if (!MyQuaternion.isRotationIdentity(orientation)) {
            stream.printf(" orient=%s", orientation);
        }

        CollisionShape shape = body.getCollisionShape();
        PhysicsDescriber describer = getDescriber();
        desc = describer.describe(shape);
        stream.printf("%n   shape=%s", desc);

        Vector3f scale = shape.getScale(null);
        desc = describer.describeScale(scale);
        if (!desc.isEmpty()) {
            stream.print(" " + desc);
        }

        PhysicsJoint[] joints = body.listJoints();
        stream.printf(" joints=%d", joints.length);
        if (joints.length > 0) {
            stream.print(":");
        }
        stream.println();

        for (PhysicsJoint joint : joints) {
            String type = joint.getClass().getSimpleName();
            if (type.endsWith("Joint")) {
                type = MyString.removeSuffix(type, "Joint");
            }

            long jointId = joint.getObjectId();
            stream.printf("    %s #%s ", type, Long.toHexString(jointId));

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
                otherId = joint.getBodyA().getObjectId();
                pivot = joint.getPivotB(null);
            }
            if (otherId == 0L) {
                stream.print("single-ended");
            } else {
                stream.printf("to=#%s", Long.toHexString(otherId));
            }
            stream.printf(" piv=%s%n", pivot);
        }
    }

    /**
     * Dump the specified physics space.
     *
     * @param space the physics space to dump (not null, unaffected)
     */
    public void dump(PhysicsSpace space) {
        Collection<PhysicsCharacter> characters = space.getCharacterList();
        Collection<PhysicsGhostObject> ghosts = space.getGhostObjectList();
        Collection<PhysicsJoint> joints = space.getJointList();
        Collection<PhysicsRigidBody> rigidBodies = space.getRigidBodyList();
        Collection<PhysicsVehicle> vehicles = space.getVehicleList();

        long spaceId = space.getSpaceId();
        int numCharacters = characters.size();
        int numGhosts = ghosts.size();
        int numJoints = joints.size();
        int numBodies = rigidBodies.size();
        int numVehicles = vehicles.size();

        stream.printf("%nSpace #%s with %d character%s, %d ghost%s, ",
                Long.toHexString(spaceId),
                numCharacters, (numCharacters == 1) ? "" : "s",
                numGhosts, (numGhosts == 1) ? "" : "s");
        stream.printf("%d joint%s, %d rigid bod%s, and %d vehicle%s%n",
                numJoints, (numJoints == 1) ? "" : "s",
                numBodies, (numBodies == 1) ? "y" : "ies",
                numVehicles, (numVehicles == 1) ? "" : "s");

        float accuracy = space.getAccuracy();
        PhysicsSpace.BroadphaseType broadphaseType = space.getBroadphaseType();
        Vector3f gravity = space.getGravity(null);
        int maxSubSteps = space.maxSubSteps();

        stream.printf(" accu=%f, bphase=%s, grav=%s, maxStep=%d%n",
                accuracy, broadphaseType, gravity, maxSubSteps);

        int numIterations = space.getSolverNumIterations();
        int rayTestFlags = space.getRayTestFlags();
        PhysicsDescriber describer = getDescriber();
        String rtText = describer.describeRayTestFlags(rayTestFlags);
        Vector3f worldMax = space.getWorldMax(null);
        Vector3f worldMin = space.getWorldMin(null);
        stream.printf(" iters=%d, rayTest=(%s), wMin=%s, wMax=%s%n",
                numIterations, rtText, worldMin, worldMax);

        for (PhysicsCharacter character : characters) {
            dump(character);
        }
        for (PhysicsGhostObject ghost : ghosts) {
            dump(ghost);
        }
        for (PhysicsJoint joint : joints) {
            dump(joint);
        }
        for (PhysicsRigidBody rigid : rigidBodies) {
            dump(rigid);
        }
        for (PhysicsVehicle vehicle : vehicles) {
            dump(vehicle);
        }
    }

    /**
     * Dump the specified vehicle.
     *
     * @param vehicle the vehicle to dump (not null, unaffected)
     */
    public void dump(PhysicsVehicle vehicle) {
        long objectId = vehicle.getObjectId();
        float mass = vehicle.getMass();
        stream.printf("  Vehicle #%s mass=%f", Long.toHexString(objectId),
                mass);

        String desc = MyObject.describeUser(vehicle);
        stream.print(desc);

        Vector3f location = vehicle.getPhysicsLocation(null);
        stream.printf(" loc=[%.3f, %.3f, %.3f]",
                location.x, location.y, location.z);

        stream.println();
    }
    // *************************************************************************
    // Dumper methods

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
