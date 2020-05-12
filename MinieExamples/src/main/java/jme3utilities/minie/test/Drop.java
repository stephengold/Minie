/*
 Copyright (c) 2020, Stephen Gold
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
package jme3utilities.minie.test;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.infos.DebugMeshNormals;
import com.jme3.bullet.debug.BulletDebugAppState;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.Collection;
import java.util.TreeSet;
import java.util.logging.Logger;
import jme3utilities.MyString;
import jme3utilities.Validate;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.minie.test.common.AbstractDemo;
import jme3utilities.minie.test.shape.MinieTestShapes;
import jme3utilities.minie.test.shape.ShapeGenerator;

/**
 * A group of dynamic bodies and physics joints in the DropTest application.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class Drop implements BulletDebugAppState.DebugAppStateFilter {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final static Logger logger = Logger.getLogger(Drop.class.getName());
    // *************************************************************************
    // fields

    /**
     * reference to the application instance
     */
    final private AbstractDemo appInstance;
    /**
     * all dynamic bodies in this Drop
     */
    final private Collection<PhysicsBody> allBodies = new TreeSet<>();
    /**
     * all physics joints in this Drop
     */
    final private Collection<PhysicsJoint> allJoints = new TreeSet<>();
    /**
     * type of Drop
     */
    final private String typeName;
    /**
     * local inverse inertia vector (or null)
     */
    private Vector3f inverseInertia = null;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a Drop of the specified type, with the specified mass and
     * application data.
     *
     * @param appInstance (alias created)
     * @param typeName (not null)
     * @param totalMass (&gt;0)
     * @param appData (alias created)
     */
    Drop(AbstractDemo appInstance, String typeName, float totalMass,
            Object appData) {
        Validate.nonNull(typeName, "type name");
        Validate.positive(totalMass, "total mass");

        this.appInstance = appInstance;
        this.typeName = typeName;
        create(totalMass, appData);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Add dynamic bodies.
     *
     * @param bodies (no nulls, not in any PhysicsSpace, aliases created)
     */
    void addBodies(PhysicsBody... bodies) {
        Validate.nonNullArray(bodies, "bodies");

        for (PhysicsBody body : bodies) {
            assert !body.isInWorld() : body;
            assert body instanceof PhysicsSoftBody
                    || ((PhysicsRigidBody) body).isDynamic() : body;
            allBodies.add(body);
        }
    }

    /**
     * Add physics joints.
     *
     * @param joints (no nulls, not in any PhysicsSpace, aliases created)
     */
    void addJoints(PhysicsJoint... joints) {
        Validate.nonNullArray(joints, "joints");

        for (PhysicsJoint joint : joints) {
            assert joint.getPhysicsSpace() == null : joint;
            PhysicsBody a = joint.getBodyA();
            assert a == null || displayObject(a) : a;
            PhysicsBody b = joint.getBodyB();
            assert b == null || displayObject(b) : b;

            allJoints.add(joint);
        }
    }

    /**
     * Add all bodies and joints to the application's PhysicsSpace.
     */
    void addToSpace() {
        for (PhysicsBody body : allBodies) {
            appInstance.addCollisionObject(body);
        }
        for (PhysicsJoint joint : allJoints) {
            appInstance.addJoint(joint);
        }
    }

    /**
     * Dump detailed information about the bodies.
     */
    void dumpBodies() {
        PhysicsDumper dumper = appInstance.getDumper();
        for (PhysicsBody body : allBodies) {
            if (body instanceof PhysicsRigidBody) {
                dumper.dump((PhysicsRigidBody) body, "");
            } else {
                dumper.dump((PhysicsSoftBody) body, "");
            }
        }
    }

    /**
     * Test whether any body is active.
     *
     * @return true if active, false if all rigid bodies are asleep
     */
    boolean isAnyActive() {
        boolean result = false;
        for (PhysicsBody body : allBodies) {
            if (body instanceof PhysicsSoftBody || body.isActive()) {
                result = true;
                break;
            }
        }

        return result;
    }

    /**
     * Apply a vertical impulse to each rigid body.
     *
     * @param deltaV the desired change in Y velocity
     */
    void pop(float deltaV) {
        for (PhysicsBody body : allBodies) {
            if (body instanceof PhysicsRigidBody) {
                float impulse = body.getMass() * deltaV;
                Vector3f impulseVector = new Vector3f(0f, impulse, 0f);
                ShapeGenerator random = appInstance.getGenerator();
                Vector3f offset = random.nextVector3f().multLocal(0.2f);
                ((PhysicsRigidBody) body).applyImpulse(impulseVector, offset);
            }
        }
    }

    /**
     * Remove all joints and bodies from the application's PhysicsSpace.
     */
    void removeFromSpace() {
        PhysicsSpace space = appInstance.getPhysicsSpace();
        for (PhysicsJoint joint : allJoints) {
            space.removeJoint(joint);
        }
        for (PhysicsBody body : allBodies) {
            space.removeCollisionObject(body);
        }
    }

    /**
     * Access the type of Drop.
     *
     * @return the name of the drop type
     */
    String typeName() {
        return typeName;
    }
    // *************************************************************************
    // DebugAppStateFilter methods

    /**
     * Test whether the specified physics object is included in this Drop.
     *
     * @param obj the joint or collision object to test (unaffected)
     * @return return true if included, false if not
     */
    @Override
    public boolean displayObject(Object obj) {
        boolean result;
        if (obj instanceof PhysicsBody) {
            result = allBodies.contains((PhysicsBody) obj);
        } else if (obj instanceof PhysicsJoint) {
            result = allJoints.contains((PhysicsJoint) obj);
        } else {
            result = false;
        }

        return result;
    }
    // *************************************************************************
    // private methods

    private void create(float totalMass, Object appData) {
        ShapeGenerator random = appInstance.getGenerator();

        DebugMeshNormals debugMeshNormals;
        CollisionShape dropShape;
        switch (typeName) {
            case "ankh":
            case "duck":
            case "heart":
            case "horseshoe":
            case "sword":
            case "table":
            case "teapot":
            case "thumbTack":
                dropShape = appInstance.findShape(typeName);
                assert dropShape != null : typeName;
                debugMeshNormals = DebugMeshNormals.Facet;
                break;

            case "banana":
            case "barbell":
            case "barrel":
            case "bowlingPin":
            case "knucklebone":
            case "ladder":
            case "top":
                dropShape = appInstance.findShape(typeName);
                assert dropShape != null : typeName;
                debugMeshNormals = DebugMeshNormals.Smooth;
                break;

            case "box":
            case "frame":
            case "halfPipe":
            case "hull":
            case "iBeam":
            case "lidlessBox":
            case "platonic":
            case "prism":
            case "pyramid":
            case "star":
            case "tetrahedron":
            case "triangularFrame":
            case "trident":
                dropShape = random.nextShape(typeName);
                debugMeshNormals = DebugMeshNormals.Facet;
                break;

            case "capsule":
            case "cone":
            case "cylinder":
            case "dome":
            case "football":
            case "multiSphere":
            case "snowman":
            case "torus":
                dropShape = random.nextShape(typeName);
                debugMeshNormals = DebugMeshNormals.Smooth;
                break;

            case "chair":
                dropShape = appInstance.findShape("chair");
                assert dropShape != null : typeName;
                inverseInertia = MinieTestShapes.chairInverseInertia;
                debugMeshNormals = DebugMeshNormals.Facet;
                break;

            case "digit":
                dropShape = randomDigit();
                debugMeshNormals = DebugMeshNormals.Facet;
                break;

            case "letter":
                dropShape = randomLetter();
                debugMeshNormals = DebugMeshNormals.Facet;
                break;

            case "madMallet":
                dropShape = randomMallet(false);
                debugMeshNormals = DebugMeshNormals.Smooth;
                break;

            case "mallet":
                dropShape = randomMallet(true);
                debugMeshNormals = DebugMeshNormals.Smooth;
                break;

            case "sphere":
                dropShape = random.nextShape(typeName);
                debugMeshNormals = DebugMeshNormals.Sphere;
                break;

            default:
                String message = "typeName = " + MyString.quote(typeName);
                throw new IllegalArgumentException(message);
        }

        Vector3f startLocation = random.nextVector3f();
        startLocation.multLocal(2.5f, 5f, 2.5f);
        startLocation.y += 20f;

        Quaternion startOrientation = random.nextQuaternion();

        PhysicsRigidBody body = new PhysicsRigidBody(dropShape, totalMass);

        body.setApplicationData(appData);
        body.setCcdMotionThreshold(5f);

        float sweptSphereRadius = dropShape.maxRadius();
        body.setCcdSweptSphereRadius(sweptSphereRadius);

        body.setDebugMeshNormals(debugMeshNormals);
        body.setDebugMeshResolution(DebugShapeFactory.highResolution);
        body.setPhysicsLocation(startLocation);
        body.setPhysicsRotation(startOrientation);
        if (inverseInertia != null) {
            body.setInverseInertiaLocal(inverseInertia);
        }

        allBodies.add(body);
    }

    /**
     * Randomly select the shape of a decimal digit.
     */
    private CollisionShape randomDigit() {
        ShapeGenerator random = appInstance.getGenerator();
        char glyphChar = (char) ('0' + random.nextInt(10));
        String glyphString = Character.toString(glyphChar);
        CollisionShape result = appInstance.findShape(glyphString);

        return result;
    }

    /**
     * Randomly select the shape of an uppercase letter.
     */
    private CollisionShape randomLetter() {
        ShapeGenerator random = appInstance.getGenerator();
        char glyphChar = (char) ('A' + random.nextInt(26));
        String glyphString = Character.toString(glyphChar);
        CollisionShape result = appInstance.findShape(glyphString);

        return result;
    }

    /**
     * Randomly generate an asymmetrical compound shape consisting of 2
     * cylinders.
     *
     * @param correctAxes if true, correct the shape's center of mass and
     * principal axes
     */
    private CollisionShape randomMallet(boolean correctAxes) {
        ShapeGenerator random = appInstance.getGenerator();
        float handleR = 0.5f;
        float headR = handleR + random.nextFloat();
        float headHalfLength = headR + random.nextFloat();
        float handleHalfLength = headHalfLength + random.nextFloat(0f, 2.5f);
        CompoundCollisionShape result = MinieTestShapes.makeMadMallet(handleR,
                headR, handleHalfLength, headHalfLength);
        /*
         * At this point, the shape's center of mass lies at the bare end
         * of the handle:  a "mad" mallet that prefers to stand upright.
         */
        if (correctAxes) {
            float handleMass = 0.15f;
            float headMass = 1f - handleMass; // Put 85% of mass in the head.
            FloatBuffer masses
                    = BufferUtils.createFloatBuffer(handleMass, headMass);

            Vector3f inertia = new Vector3f();
            Transform transform = result.principalAxes(masses, null, inertia);
            inverseInertia = Vector3f.UNIT_XYZ.divide(inertia);
            result.correctAxes(transform);
        }

        return result;
    }
}
