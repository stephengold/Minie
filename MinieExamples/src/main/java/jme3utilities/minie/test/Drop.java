/*
 Copyright (c) 2020-2023, Stephen Gold
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

import com.jme3.asset.AssetManager;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.RotationOrder;
import com.jme3.bullet.animation.BoneLink;
import com.jme3.bullet.animation.DynamicAnimControl;
import com.jme3.bullet.animation.TorsoLink;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.debug.BulletDebugAppState;
import com.jme3.bullet.joints.New6Dof;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.joints.motors.MotorParam;
import com.jme3.bullet.joints.motors.RotationMotor;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsGhostObject;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.bullet.objects.infos.ConfigFlag;
import com.jme3.bullet.objects.infos.Sbcp;
import com.jme3.bullet.objects.infos.SoftBodyConfig;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.bullet.util.NativeSoftBodyUtil;
import com.jme3.math.FastMath;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.Collection;
import java.util.List;
import java.util.Random;
import java.util.TreeSet;
import java.util.logging.Logger;
import jme3utilities.MeshNormals;
import jme3utilities.MyString;
import jme3utilities.Validate;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.noise.Generator;
import jme3utilities.mesh.ClothGrid;
import jme3utilities.mesh.Icosphere;
import jme3utilities.minie.test.common.PhysicsDemo;
import jme3utilities.minie.test.shape.CompoundTestShapes;
import jme3utilities.minie.test.shape.ShapeGenerator;
import jme3utilities.minie.test.tunings.BaseMeshControl;

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
     * application that contains this Drop
     */
    final private PhysicsDemo appInstance;
    /**
     * physics bodies in this Drop, all must be dynamic
     */
    final private Collection<PhysicsBody> allBodies = new TreeSet<>();
    /**
     * all physics joints in this Drop
     */
    final private Collection<PhysicsJoint> allJoints = new TreeSet<>();
    /**
     * included ragdoll, or null if none
     */
    private DynamicAnimControl dac = null;
    /**
     * configured total mass of all bodies in this Drop
     */
    final private float totalMass;
    /**
     * configured type of Drop
     */
    final private String typeName;
    /**
     * configured initial local-to-PhysicsSpace Transform
     */
    final private Transform startPosition;
    /**
     * local inverse inertia vector for next rigid body (or null)
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
    Drop(PhysicsDemo appInstance, String typeName, float totalMass,
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
     * Add all controls, bodies, and joints to the application's PhysicsSpace.
     */
    void addToSpace() {
        if (hasDac()) {
            PhysicsSpace space = appInstance.getPhysicsSpace();
            dac.setPhysicsSpace(space);
            for (PhysicsBody body : allBodies) {
                appInstance.postAdd(body);
            }
        }

        for (PhysicsBody body : allBodies) {
            if (!body.isInWorld()) {
                appInstance.addCollisionObject(body);
            }
        }

        for (PhysicsJoint joint : allJoints) {
            if (joint.getPhysicsSpace() == null) {
                appInstance.addJoint(joint);
            }
        }
    }

    /**
     * Test whether this Drop includes a DynamicAnimControl.
     *
     * @return true if a DynamicAnimControl is included, otherwise false
     */
    boolean hasDac() {
        if (dac == null) {
            return false;
        } else {
            return true;
        }
    }

    /**
     * Test for contacts with this drop's convex hull.
     *
     * @return true if there are contacts, otherwise false
     */
    boolean hasHullContacts() {
        int numBodies = allBodies.size();
        FloatBuffer[] vertices = new FloatBuffer[numBodies];
        int bodyIndex = 0;
        int totalFloats = 0;
        for (PhysicsBody body : allBodies) {
            int numFloats;
            if (body instanceof PhysicsRigidBody) {
                CollisionShape shape = body.getCollisionShape();
                vertices[bodyIndex] = DebugShapeFactory.debugVertices(
                        shape, DebugShapeFactory.lowResolution);
                numFloats = vertices[bodyIndex].capacity();

                // Transform scaled shape coordinates to physics-space.
                Quaternion orientation = body.getPhysicsRotation(null);
                MyBuffer.rotate(vertices[bodyIndex], 0, numFloats, orientation);
                Vector3f location = body.getPhysicsLocation(null);
                MyBuffer.translate(vertices[bodyIndex], 0, numFloats, location);

            } else {
                PhysicsSoftBody softBody = (PhysicsSoftBody) body;
                int numNodes = softBody.countNodes();
                numFloats = MyVector3f.numAxes * numNodes;
                vertices[bodyIndex] = BufferUtils.createFloatBuffer(numFloats);
                softBody.copyLocations(vertices[bodyIndex]);
            }

            vertices[bodyIndex].limit(numFloats);
            totalFloats += numFloats;
            ++bodyIndex;
        }

        HullCollisionShape hullShape;
        if (numBodies == 1) {
            hullShape = new HullCollisionShape(vertices[0]);

        } else {
            FloatBuffer all = BufferUtils.createFloatBuffer(totalFloats);
            for (FloatBuffer buffer : vertices) {
                buffer.rewind();
                all.put(buffer);
            }
            all.limit(totalFloats);
            hullShape = new HullCollisionShape(all);
        }

        PhysicsGhostObject ghost = new PhysicsGhostObject(hullShape);
        PhysicsSpace space = appInstance.getPhysicsSpace();
        int numContacts = space.contactTest(ghost, null);
        boolean result = (numContacts > 0);
        //space.addCollisionObject(ghost);

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
                Generator random = appInstance.getGenerator();
                Vector3f offset = random.nextVector3f().multLocal(0.2f);
                ((PhysicsRigidBody) body).applyImpulse(impulseVector, offset);
            } else {
                PhysicsSoftBody softBody = (PhysicsSoftBody) body;
                Vector3f velocityVector = new Vector3f(0f, deltaV, 0f);
                softBody.addVelocity(velocityVector);
            }
        }
    }

    /**
     * Remove all controls, joints, and bodies from the application's
     * PhysicsSpace.
     */
    void removeFromSpace() {
        if (hasDac()) {
            dac.setPhysicsSpace(null);
        }

        PhysicsSpace space = appInstance.getPhysicsSpace();
        for (PhysicsJoint joint : allJoints) {
            if (joint.getPhysicsSpace() == space) {
                space.removeJoint(joint);
            }
        }

        for (PhysicsBody body : allBodies) {
            if (body.isInWorld()) {
                space.removeCollisionObject(body);
            }
        }
    }

    /**
     * Callback invoked once per frame.
     */
    void update() {
        // As soon as the DAC is ready, put all physics links into dynamic mode.
        if (hasDac() && dac.isReady()) {
            TorsoLink torso = dac.getTorsoLink();
            if (torso.isKinematic()) {
                PhysicsSpace space = appInstance.getPhysicsSpace();
                Vector3f gravity = space.getGravity(null);
                dac.setDynamicSubtree(torso, gravity, false);
            }
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
            result = allBodies.contains(obj);
        } else if (obj instanceof PhysicsJoint) {
            result = allJoints.contains(obj);
        } else {
            result = false;
        }

        return result;
    }
    // *************************************************************************
    // private methods

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
                createRigidBody(
                        shape, totalMass, MeshNormals.Facet, startPosition);
                break;

            case "banana":
            case "barbell":
            case "barrel":
            case "bowlingPin":
            case "knucklebone":
            case "ladder":
            case "link":
            case "top":
                shape = appInstance.findShape(typeName);
                createRigidBody(
                        shape, totalMass, MeshNormals.Smooth, startPosition);
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
            case "washer":
                shape = random.nextShape(typeName);
                createRigidBody(
                        shape, totalMass, MeshNormals.Facet, startPosition);
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
                createRigidBody(
                        shape, totalMass, MeshNormals.Smooth, startPosition);
                break;

            case "bowl":
                shape = appInstance.findShape("bowl");
                inverseInertia = CompoundTestShapes.bowlInverseInertia.divide(
                        totalMass);
                createRigidBody(
                        shape, totalMass, MeshNormals.Smooth, startPosition);
                break;

            case "chair":
                shape = appInstance.findShape("chair");
                inverseInertia = CompoundTestShapes.chairInverseInertia.divide(
                        totalMass);
                createRigidBody(
                        shape, totalMass, MeshNormals.Facet, startPosition);
                break;

            case "digit":
                shape = randomDigit();
                createRigidBody(
                        shape, totalMass, MeshNormals.Facet, startPosition);
                break;

            case "breakableRod":
            case "chain":
            case "diptych":
            case "flail":
            case "ragdoll":
                createJointed();
                break;

            case "letter":
                shape = randomLetter();
                createRigidBody(
                        shape, totalMass, MeshNormals.Facet, startPosition);
                break;

            case "madMallet":
                shape = randomMallet(false);
                createRigidBody(
                        shape, totalMass, MeshNormals.Smooth, startPosition);
                break;

            case "mallet":
                shape = randomMallet(true);
                createRigidBody(
                        shape, totalMass, MeshNormals.Smooth, startPosition);
                break;

            case "sphere":
                shape = random.nextShape(typeName);
                createRigidBody(
                        shape, totalMass, MeshNormals.Sphere, startPosition);
                break;

            case "cloth":
            case "squishyBall":
                createSoftBody();
                break;

            default:
                String message = "typeName = " + MyString.quote(typeName);
                throw new IllegalArgumentException(message);
        }
    }

    /**
     * Create a breakable rod. Its length runs along the drop's local Y axis.
     *
     * @param totalLength the total length (in physics-space units, &gt;0)
     * @param radius the radius (in physics-space units, &gt;0)
     */
    private void createBreakableRod(float totalLength, float radius) {
        float pieceLength = totalLength / 2f;
        float pieceMass = totalMass / 2f;

        CylinderCollisionShape pieceShape = new CylinderCollisionShape(
                radius, pieceLength, PhysicsSpace.AXIS_Y);

        Transform tmpPosition = new Transform();
        tmpPosition.getTranslation().y = pieceLength / 2;
        Vector3f pivotInA = tmpPosition.getTranslation().negate();
        tmpPosition.combineWithParent(startPosition);
        PhysicsRigidBody a = createRigidBody(
                pieceShape, pieceMass, MeshNormals.Smooth, tmpPosition);

        tmpPosition.loadIdentity();
        tmpPosition.getTranslation().y = -pieceLength / 2;
        Vector3f pivotInB = tmpPosition.getTranslation().negate();
        tmpPosition.combineWithParent(startPosition);
        PhysicsRigidBody b = createRigidBody(
                pieceShape, pieceMass, MeshNormals.Smooth, tmpPosition);

        Matrix3f rotInA = Matrix3f.IDENTITY;
        Matrix3f rotInB = Matrix3f.IDENTITY;
        New6Dof fixed = new New6Dof(
                a, b, pivotInA, pivotInB, rotInA, rotInB, RotationOrder.YZX);
        allJoints.add(fixed);

        fixed.setBreakingImpulseThreshold(4f);

        for (int i = PhysicsSpace.AXIS_X; i <= PhysicsSpace.AXIS_Z; ++i) {
            RotationMotor motor = fixed.getRotationMotor(i);
            motor.set(MotorParam.LowerLimit, 0f);
            motor.set(MotorParam.UpperLimit, 0f);
        }
    }

    /**
     * Create a chain of links. Initially the chain is laid out along the drop's
     * local Y axis.
     *
     * @param numLinks the number of links (&gt;0)
     * @param thickness the thickness of each link (in physics-space units,
     * &gt;0)
     */
    private void createChain(int numLinks, float thickness) {
        float halfThickness = thickness / 2f;
        float linkIhh = 3f * halfThickness;
        float linkIhw = 1.1f * halfThickness;
        float angleStep = FastMath.HALF_PI;
        float yStep = 1.9f * linkIhh; // center-to-center
        float linkMass = totalMass / numLinks;

        CompoundCollisionShape shape
                = CompoundTestShapes.makeLink(linkIhh, linkIhw, halfThickness);

        Quaternion rotationStep = new Quaternion();
        rotationStep.fromAngles(0f, angleStep, 0f);

        float y0 = -yStep * (numLinks - 1) / 2f;
        Vector3f startOffset = new Vector3f(0f, y0, 0f);
        Transform transform = new Transform(startOffset);
        Vector3f linkLocation = transform.getTranslation(); // alias
        Quaternion linkOrientation = transform.getRotation(); // alias

        Transform tmpPosition = new Transform();
        for (int linkIndex = 0; linkIndex < numLinks; ++linkIndex) {
            tmpPosition.set(transform);
            tmpPosition.combineWithParent(startPosition);
            createRigidBody(
                    shape, linkMass, MeshNormals.Smooth, tmpPosition);

            linkLocation.y += yStep;
            rotationStep.mult(linkOrientation, linkOrientation);
        }
    }

    /**
     * Create a diptych consisting of 2 thin boxes, joined by a hinge. The axis
     * of the hinge is the drop's local Y axis.
     *
     * @param height the desired height of each box (&gt;0)
     * @param thickness the desired thickness of each box (&gt;0)
     * @param boxWidth the desired width of each box (&gt;0)
     */
    private void createDiptych(float height, float thickness, float boxWidth) {
        float boxHalfWidth = boxWidth / 2f;
        float halfThickness = thickness / 2f;
        CollisionShape boxShape = new BoxCollisionShape(
                boxHalfWidth, height / 2f, halfThickness);

        float boxMass = totalMass / 2f;

        Vector3f pivotInA = new Vector3f(boxHalfWidth, 0f, halfThickness);
        Transform aPosition = new Transform();
        aPosition.setTranslation(pivotInA.negate());
        aPosition.combineWithParent(startPosition);

        PhysicsRigidBody a = createRigidBody(
                boxShape, boxMass, MeshNormals.Facet, aPosition);

        Vector3f pivotInB = new Vector3f(-boxHalfWidth, 0f, halfThickness);
        Transform bPosition = new Transform();
        bPosition.setTranslation(pivotInB.negate());
        bPosition.combineWithParent(startPosition);

        PhysicsRigidBody b = createRigidBody(
                boxShape, boxMass, MeshNormals.Facet, bPosition);

        Matrix3f rotInA = Matrix3f.IDENTITY;
        Matrix3f rotInB = Matrix3f.IDENTITY;
        New6Dof hinge = new New6Dof(
                a, b, pivotInA, pivotInB, rotInA, rotInB, RotationOrder.YZX);
        allJoints.add(hinge);

        RotationMotor xMotor = hinge.getRotationMotor(PhysicsSpace.AXIS_X);
        xMotor.set(MotorParam.LowerLimit, 0f);
        xMotor.set(MotorParam.UpperLimit, 0f);

        RotationMotor yMotor = hinge.getRotationMotor(PhysicsSpace.AXIS_Y);
        yMotor.set(MotorParam.LowerLimit, 0.1f);
        yMotor.set(MotorParam.Equilibrium, 1.6f);
        yMotor.set(MotorParam.UpperLimit, FastMath.PI);
        yMotor.setSpringEnabled(true);
        yMotor.set(MotorParam.Stiffness, 0.01f); // a very weak spring

        RotationMotor zMotor = hinge.getRotationMotor(PhysicsSpace.AXIS_Z);
        zMotor.set(MotorParam.LowerLimit, 0f);
        zMotor.set(MotorParam.UpperLimit, 0f);
    }

    /**
     * Create a flail consisting of 2 unequal staves, joined by a pivot.
     * Initially both staves lie on the drop's local Y axis.
     *
     * @param aLength the length of the "A" staff (&gt;0)
     * @param bLength the length of the "B" staff (&gt;0)
     * @param radius the radius of each staff (&gt;0)
     */
    private void createFlail(float aLength, float bLength, float radius) {
        float linearDensity = totalMass / (aLength + bLength);

        CollisionShape aShape = new CapsuleCollisionShape(
                radius, aLength, PhysicsSpace.AXIS_Y);
        float aMass = aLength * linearDensity;

        Vector3f pivotInA = new Vector3f(0f, 3f * radius + aLength / 2f, 0f);
        Transform aPosition = new Transform();
        aPosition.setTranslation(pivotInA.negate());
        aPosition.combineWithParent(startPosition);

        PhysicsRigidBody a = createRigidBody(
                aShape, aMass, MeshNormals.Smooth, aPosition);

        CollisionShape bShape = new CapsuleCollisionShape(
                radius, bLength, PhysicsSpace.AXIS_Y);
        float bMass = bLength * linearDensity;

        Vector3f pivotInB = new Vector3f(0f, -3f * radius - bLength / 2f, 0f);
        Transform bPosition = new Transform();
        bPosition.setTranslation(pivotInB.negate());
        bPosition.combineWithParent(startPosition);

        PhysicsRigidBody b = createRigidBody(
                bShape, bMass, MeshNormals.Smooth, bPosition);

        Matrix3f rotInA = Matrix3f.IDENTITY;
        Matrix3f rotInB = Matrix3f.IDENTITY;
        New6Dof pivot = new New6Dof(
                a, b, pivotInA, pivotInB, rotInA, rotInB, RotationOrder.XYZ);
        allJoints.add(pivot);
    }

    /**
     * Create a jointed Drop based on the configuration.
     */
    private void createJointed() {
        switch (typeName) {
            case "breakableRod": {
                float totalLength = 5f;
                float radius = 1f;
                createBreakableRod(totalLength, radius);
                break;
            }

            case "chain": {
                int numLinks = 10;
                float thickness = 0.4f;
                createChain(numLinks, thickness);
                break;
            }

            case "diptych": {
                float height = 4f;
                float thickness = 0.4f;
                float boxWidth = 2f;
                createDiptych(height, thickness, boxWidth);
                break;
            }

            case "flail": {
                float aLength = 5f;
                float bLength = 2.5f;
                float radius = 0.2f;
                createFlail(aLength, bLength, radius);
                break;
            }

            case "ragdoll": {
                float scale = 5f;
                createRagdoll("Models/BaseMesh/BaseMesh.j3o", scale,
                        new BaseMeshControl());
                break;
            }

            default:
                String message = "typeName = " + MyString.quote(typeName);
                throw new IllegalArgumentException(message);
        }
    }

    /**
     * Create a ragdoll using DynamicAnimControl.
     *
     * @param assetPath asset path to the model J3O (not null, not empty)
     * @param scale the uniform scale factor to apply (&gt;0)
     * @param control a configured DynamicAnimControl to use (not null)
     */
    private void createRagdoll(
            String assetPath, float scale, DynamicAnimControl control) {
        assert scale > 0f : scale;

        AssetManager assetManager = appInstance.getAssetManager();
        Node cgModel = (Node) assetManager.loadModel(assetPath);
        cgModel.setLocalTransform(startPosition);
        cgModel.setLocalScale(scale);

        this.dac = control;

        PhysicsRigidBody[] bodies = dac.listRigidBodies();
        for (PhysicsRigidBody body : bodies) {
            allBodies.add(body);
            body.setDebugMeshNormals(MeshNormals.Smooth);
        }

        List<BoneLink> boneLinks = dac.listLinks(BoneLink.class);
        for (BoneLink link : boneLinks) {
            PhysicsJoint joint = link.getJoint();
            allJoints.add(joint);
        }
    }

    /**
     * Create a rigid body at the specified position and add it to this Drop.
     *
     * @param shape (not null)
     * @param mass (&gt;0)
     * @param debugMeshNormals (not null)
     * @param position (not null, unaffected)
     * @return the new body (not null, not in any space)
     */
    private PhysicsRigidBody createRigidBody(CollisionShape shape, float mass,
            MeshNormals debugMeshNormals, Transform position) {
        assert shape != null : typeName;
        assert mass > 0f : mass;
        assert debugMeshNormals != null : typeName;

        PhysicsRigidBody result = new PhysicsRigidBody(shape, mass);
        allBodies.add(result);

        result.setDebugMeshNormals(debugMeshNormals);
        if (inverseInertia != null) {
            result.setInverseInertiaLocal(inverseInertia);
        }
        result.setPhysicsLocation(position.getTranslation());
        result.setPhysicsRotation(position.getRotation());

        return result;
    }

    /**
     * Create a soft Drop based on the configuration.
     */
    private void createSoftBody() {
        PhysicsSoftBody softBody;
        switch (typeName) {
            case "cloth": {
                int numLines = 41;
                float lineSpacing = 0.3f; // mesh units
                Mesh mesh = new ClothGrid(numLines, numLines, lineSpacing);

                softBody = new PhysicsSoftBody();
                NativeSoftBodyUtil.appendFromTriMesh(mesh, softBody);
                softBody.applyTranslation(startPosition.getTranslation());

                softBody.setDebugMeshNormals(MeshNormals.Smooth);
                softBody.setMargin(lineSpacing);

                SoftBodyConfig config = softBody.getSoftConfig();
                config.setPositionIterations(20);  // default = 1
                break;
            }

            case "squishyBall": {
                int numRefinementIterations = 3;
                float radius = 3f;
                Mesh mesh = new Icosphere(numRefinementIterations, radius);

                softBody = new PhysicsSoftBody();
                NativeSoftBodyUtil.appendFromTriMesh(mesh, softBody);
                softBody.applyTransform(startPosition);

                softBody.setDebugMeshNormals(MeshNormals.Smooth);

                boolean setVolumePose = false;
                boolean setFramePose = true;
                softBody.setPose(setVolumePose, setFramePose);

                SoftBodyConfig config = softBody.getSoftConfig();
                config.set(Sbcp.PoseMatching, 0.1f);
                config.setCollisionFlags(ConfigFlag.SDF_RS, ConfigFlag.VF_SS);
                config.setPositionIterations(9);
                break;
            }

            default:
                String message = "typeName = " + MyString.quote(typeName);
                throw new IllegalArgumentException(message);
        }

        softBody.setMass(totalMass);
        SoftBodyConfig config = softBody.getSoftConfig();
        config.setCollisionFlags(ConfigFlag.SDF_RS, ConfigFlag.VF_SS);
        config.setPositionIterations(3);

        allBodies.add(softBody);
    }

    /**
     * Pseudo-randomly select the shape of a decimal digit.
     *
     * @return the pre-existing instance (not null)
     */
    private CollisionShape randomDigit() {
        Random random = appInstance.getGenerator();
        char glyphChar = (char) ('0' + random.nextInt(10));
        String glyphString = Character.toString(glyphChar);
        CollisionShape result = appInstance.findShape(glyphString);
        assert result != null : glyphChar;

        return result;
    }

    /**
     * Pseudo-randomly select the shape of an uppercase letter.
     *
     * @return the pre-existing instance (not null)
     */
    private CollisionShape randomLetter() {
        Random random = appInstance.getGenerator();
        char glyphChar = (char) ('A' + random.nextInt(26));
        String glyphString = Character.toString(glyphChar);
        CollisionShape result = appInstance.findShape(glyphString);
        assert result != null : glyphChar;

        return result;
    }

    /**
     * Pseudo-randomly generate an asymmetrical compound shape consisting of 2
     * cylinders, a head and a handle.
     *
     * @param correctAxes if true, correct the shape's center of mass and
     * principal axes
     * @return a new instance (not null)
     */
    private CollisionShape randomMallet(boolean correctAxes) {
        Generator random = appInstance.getGenerator();
        float handleR = 0.5f;
        float headR = handleR + random.nextFloat();
        float headHalfLength = headR + random.nextFloat();
        float handleHalfLength = headHalfLength + random.nextFloat(0f, 2.5f);
        CompoundCollisionShape result = CompoundTestShapes.makeMadMallet(
                handleR, headR, handleHalfLength, headHalfLength);
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
            this.inverseInertia = Vector3f.UNIT_XYZ.divide(inertia);
            result.correctAxes(transform);
        }

        return result;
    }
}
