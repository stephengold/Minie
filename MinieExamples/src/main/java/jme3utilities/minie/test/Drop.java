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
import com.jme3.bullet.RotationOrder;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.infos.DebugMeshNormals;
import com.jme3.bullet.debug.BulletDebugAppState;
import com.jme3.bullet.joints.New6Dof;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.joints.motors.MotorParam;
import com.jme3.bullet.joints.motors.RotationMotor;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.material.Material;
import com.jme3.math.FastMath;
import com.jme3.math.Matrix3f;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.Collection;
import java.util.TreeSet;
import java.util.logging.Logger;
import jme3utilities.MyString;
import jme3utilities.Validate;
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
     * application that contains this drop
     */
    final private AbstractDemo appInstance;
    /**
     * bodies in this Drop, all must be dynamic
     */
    final private Collection<PhysicsBody> allBodies = new TreeSet<>();
    /**
     * all physics joints in this Drop
     */
    final private Collection<PhysicsJoint> allJoints = new TreeSet<>();
    /**
     * total mass of all bodies in this Drop
     */
    final private float totalMass;
    /**
     * type of Drop
     */
    final private String typeName;
    /**
     * initial local-to-PhysicsSpace Transform
     */
    final private Transform startPosition;
    /**
     * local inverse inertia vector (or null)
     */
    private Vector3f inverseInertia = null;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a Drop of the specified type, with the specified mass,
     * initial position, and application data.
     *
     * @param appInstance (not null, alias created)
     * @param typeName (not null)
     * @param totalMass (&gt;0)
     * @param startPosition the local-to-PhysicsSpace Transform (not null,
     * unaffected)
     */
    Drop(AbstractDemo appInstance, String typeName, float totalMass,
            Transform startPosition) {
        Validate.nonNull(appInstance, "application instance");
        Validate.nonNull(typeName, "type name");
        Validate.positive(totalMass, "total mass");
        Validate.nonNull(startPosition, "start position");

        this.appInstance = appInstance;
        this.typeName = typeName;
        this.totalMass = totalMass;
        this.startPosition = startPosition.clone();

        create();
    }
    // *************************************************************************
    // new methods exposed

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

    /**
     * Add a rigid body to this Drop and assign it a pseudo-random debug
     * material.
     *
     * @param shape (not null)
     * @param mass (&gt;0)
     * @param debugMeshNormals (not null)
     * @param position (not null, unaffected)
     * @return the new body (not null, not in any space)
     */
    private PhysicsRigidBody addRigidBody(CollisionShape shape, float mass,
            DebugMeshNormals debugMeshNormals, Transform position) {
        assert shape != null : typeName;
        assert mass > 0f : mass;
        assert debugMeshNormals != null : typeName;

        PhysicsRigidBody result = new PhysicsRigidBody(shape, mass);
        allBodies.add(result);

        ShapeGenerator random = appInstance.getGenerator();
        String materialName = "drop" + random.nextInt(0, 3);
        Material debugMaterial = appInstance.findMaterial(materialName);
        assert debugMaterial != null : materialName;
        result.setApplicationData(debugMaterial);

        result.setCcdMotionThreshold(5f);

        float sweptSphereRadius = shape.maxRadius();
        result.setCcdSweptSphereRadius(sweptSphereRadius);

        result.setDebugMeshNormals(debugMeshNormals);
        result.setDebugMeshResolution(DebugShapeFactory.highResolution);
        if (inverseInertia != null) {
            result.setInverseInertiaLocal(inverseInertia);
        }
        result.setPhysicsLocation(position.getTranslation());
        result.setPhysicsRotation(position.getRotation());

        return result;
    }

    /**
     * Create bodies and joints based on the configuration.
     */
    private void create() {
        ShapeGenerator random = appInstance.getGenerator();

        CollisionShape shape;
        switch (typeName) {
            case "ankh":
            case "duck":
            case "heart":
            case "horseshoe":
            case "sword":
            case "table":
            case "teapot":
            case "thumbTack":
                shape = appInstance.findShape(typeName);
                addRigidBody(shape, totalMass, DebugMeshNormals.Facet,
                        startPosition);
                break;

            case "banana":
            case "barbell":
            case "barrel":
            case "bowlingPin":
            case "knucklebone":
            case "ladder":
            case "top":
                shape = appInstance.findShape(typeName);
                addRigidBody(shape, totalMass, DebugMeshNormals.Smooth,
                        startPosition);
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
                shape = random.nextShape(typeName);
                addRigidBody(shape, totalMass, DebugMeshNormals.Facet,
                        startPosition);
                break;

            case "capsule":
            case "cone":
            case "cylinder":
            case "dome":
            case "football":
            case "multiSphere":
            case "snowman":
            case "torus":
                shape = random.nextShape(typeName);
                addRigidBody(shape, totalMass, DebugMeshNormals.Smooth,
                        startPosition);
                break;

            case "chair":
                shape = appInstance.findShape("chair");
                inverseInertia = MinieTestShapes.chairInverseInertia;
                addRigidBody(shape, totalMass, DebugMeshNormals.Facet,
                        startPosition);
                break;

            case "digit":
                shape = randomDigit();
                addRigidBody(shape, totalMass, DebugMeshNormals.Facet,
                        startPosition);
                break;

            case "diptych":
                createJointed();
                break;

            case "letter":
                shape = randomLetter();
                addRigidBody(shape, totalMass, DebugMeshNormals.Facet,
                        startPosition);
                break;

            case "madMallet":
                shape = randomMallet(false);
                addRigidBody(shape, totalMass, DebugMeshNormals.Smooth,
                        startPosition);
                break;

            case "mallet":
                shape = randomMallet(true);
                addRigidBody(shape, totalMass, DebugMeshNormals.Smooth,
                        startPosition);
                break;

            case "sphere":
                shape = random.nextShape(typeName);
                addRigidBody(shape, totalMass, DebugMeshNormals.Sphere,
                        startPosition);
                break;

            default:
                String message = "typeName = " + MyString.quote(typeName);
                throw new IllegalArgumentException(message);
        }
    }

    /**
     * Create a diptych consisting of 2 thin boxes joined by a hinge.
     */
    private void createDiptych() {
        float halfWidth = 1f;
        float halfHeight = 2f;
        float halfThickness = 0.2f;
        CollisionShape boxShape
                = new BoxCollisionShape(halfWidth, halfHeight, halfThickness);

        float boxMass = totalMass / 2f;

        Vector3f pivotInA = new Vector3f(halfWidth, 0f, halfThickness);
        Transform aPosition = new Transform();
        aPosition.setTranslation(pivotInA.negate());
        aPosition.combineWithParent(startPosition);
        PhysicsRigidBody a = addRigidBody(boxShape, boxMass,
                DebugMeshNormals.Facet, aPosition);

        Vector3f pivotInB = new Vector3f(-halfWidth, 0f, halfThickness);
        Transform bPosition = new Transform();
        bPosition.setTranslation(pivotInB.negate());
        bPosition.combineWithParent(startPosition);
        PhysicsRigidBody b = addRigidBody(boxShape, boxMass,
                DebugMeshNormals.Facet, bPosition);

        Matrix3f rotInA = Matrix3f.IDENTITY;
        Matrix3f rotInB = Matrix3f.IDENTITY;
        New6Dof hinge = new New6Dof(a, b, pivotInA, pivotInB, rotInA, rotInB,
                RotationOrder.YZX);
        allJoints.add(hinge);

        RotationMotor xMotor = hinge.getRotationMotor(PhysicsSpace.AXIS_X);
        xMotor.set(MotorParam.LowerLimit, 0f);
        xMotor.set(MotorParam.UpperLimit, 0f);

        RotationMotor yMotor = hinge.getRotationMotor(PhysicsSpace.AXIS_Y);
        yMotor.set(MotorParam.LowerLimit, 0.4f);
        yMotor.set(MotorParam.UpperLimit, FastMath.PI);
        yMotor.setSpringEnabled(true);
        yMotor.set(MotorParam.Equilibrium, 3f);
        yMotor.set(MotorParam.Stiffness, 3e-4f);

        RotationMotor zMotor = hinge.getRotationMotor(PhysicsSpace.AXIS_Z);
        zMotor.set(MotorParam.LowerLimit, 0f);
        zMotor.set(MotorParam.UpperLimit, 0f);
    }

    /**
     * Create a jointed Drop based on the configuration.
     */
    private void createJointed() {
        switch (typeName) {
            case "diptych":
                createDiptych();
                break;

            default:
                String message = "typeName = " + MyString.quote(typeName);
                throw new IllegalArgumentException(message);
        }
    }

    /**
     * Pseudo-randomly select the shape of a decimal digit.
     */
    private CollisionShape randomDigit() {
        ShapeGenerator random = appInstance.getGenerator();
        char glyphChar = (char) ('0' + random.nextInt(10));
        String glyphString = Character.toString(glyphChar);
        CollisionShape result = appInstance.findShape(glyphString);
        assert result != null : glyphChar;

        return result;
    }

    /**
     * Pseudo-randomly select the shape of an uppercase letter.
     */
    private CollisionShape randomLetter() {
        ShapeGenerator random = appInstance.getGenerator();
        char glyphChar = (char) ('A' + random.nextInt(26));
        String glyphString = Character.toString(glyphChar);
        CollisionShape result = appInstance.findShape(glyphString);
        assert result != null : glyphChar;

        return result;
    }

    /**
     * Pseudo-randomly generate an asymmetrical compound shape consisting of 2
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
