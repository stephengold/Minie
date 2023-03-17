/*
 Copyright (c) 2019-2023, Stephen Gold
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

import com.jme3.bullet.CollisionSpace;
import com.jme3.bullet.DeformableSpace;
import com.jme3.bullet.FillMode;
import com.jme3.bullet.MultiBody;
import com.jme3.bullet.MultiBodyLink;
import com.jme3.bullet.MultiBodySpace;
import com.jme3.bullet.PhysicsSoftSpace;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.RayTestFlag;
import com.jme3.bullet.RotationOrder;
import com.jme3.bullet.SoftBodyWorldInfo;
import com.jme3.bullet.SolverInfo;
import com.jme3.bullet.SolverType;
import com.jme3.bullet.collision.Activation;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.Box2dShape;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.ConeCollisionShape;
import com.jme3.bullet.collision.shapes.Convex2dShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.EmptyShape;
import com.jme3.bullet.collision.shapes.GImpactCollisionShape;
import com.jme3.bullet.collision.shapes.HeightfieldCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.MeshCollisionShape;
import com.jme3.bullet.collision.shapes.MultiSphere;
import com.jme3.bullet.collision.shapes.PlaneCollisionShape;
import com.jme3.bullet.collision.shapes.SimplexCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.joints.Constraint;
import com.jme3.bullet.joints.GearJoint;
import com.jme3.bullet.joints.HingeJoint;
import com.jme3.bullet.joints.JointEnd;
import com.jme3.bullet.joints.New6Dof;
import com.jme3.bullet.joints.NewHinge;
import com.jme3.bullet.joints.Point2PointJoint;
import com.jme3.bullet.joints.SixDofJoint;
import com.jme3.bullet.joints.SixDofSpringJoint;
import com.jme3.bullet.joints.SliderJoint;
import com.jme3.bullet.joints.SoftAngularJoint;
import com.jme3.bullet.joints.motors.MotorParam;
import com.jme3.bullet.joints.motors.RotationMotor;
import com.jme3.bullet.joints.motors.RotationalLimitMotor;
import com.jme3.bullet.joints.motors.TranslationMotor;
import com.jme3.bullet.joints.motors.TranslationalLimitMotor;
import com.jme3.bullet.objects.MultiBodyCollider;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsCharacter;
import com.jme3.bullet.objects.PhysicsGhostObject;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.bullet.objects.PhysicsVehicle;
import com.jme3.bullet.objects.VehicleWheel;
import com.jme3.bullet.objects.infos.Aero;
import com.jme3.bullet.objects.infos.ConfigFlag;
import com.jme3.bullet.objects.infos.RigidBodyMotionState;
import com.jme3.bullet.objects.infos.Sbcp;
import com.jme3.bullet.objects.infos.SoftBodyConfig;
import com.jme3.bullet.objects.infos.SoftBodyMaterial;
import com.jme3.bullet.objects.infos.VehicleTuning;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.bullet.util.NativeLibrary;
import com.jme3.bullet.util.NativeSoftBodyUtil;
import com.jme3.math.FastMath;
import com.jme3.math.Matrix3f;
import com.jme3.math.Plane;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.debug.WireBox;
import com.jme3.scene.shape.Quad;
import com.jme3.scene.shape.Torus;
import com.jme3.system.NativeLibraryLoader;
import com.simsilica.mathd.Quatd;
import com.simsilica.mathd.Vec3d;
import java.util.ArrayList;
import java.util.List;
import jme3utilities.MeshNormals;
import jme3utilities.math.MyMath;
import org.junit.Assert;
import org.junit.Test;
import vhacd.ACDMode;
import vhacd.VHACDParameters;
import vhacd4.Vhacd4Parameters;

/**
 * Verify the defaults for physics objects.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestDefaults {
    // *************************************************************************
    // fields

    private static BoxCollisionShape box; // initialized by testShapes()
    private static PhysicsRigidBody rigidA;
    private static PhysicsSoftBody softA;
    // *************************************************************************
    // new methods exposed

    /**
     * Verify the defaults for physics objects.
     */
    @Test
    public void testDefaults() {
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);

        // default margin for collision shapes
        float margin = CollisionShape.getDefaultMargin();
        Assert.assertEquals(0.04f, margin, 0f);

        // deactivation deadline
        float deadline = PhysicsBody.getDeactivationDeadline();
        Assert.assertEquals(2f, deadline, 0f);

        // deactivation enabled flag
        boolean enabled = PhysicsBody.isDeactivationEnabled();
        Assert.assertTrue(enabled);

        PhysicsBody.setDeactivationEnabled(false);
        enabled = PhysicsBody.isDeactivationEnabled();
        Assert.assertFalse(enabled);

        CollisionSpace cSpace = new CollisionSpace(
                new Vector3f(-10000f, -10000f, -10000f),
                new Vector3f(10000f, 10000f, 10000f),
                PhysicsSpace.BroadphaseType.AXIS_SWEEP_3);
        testCollisionSpace(cSpace);

        PhysicsSpace pSpace
                = new PhysicsSpace(PhysicsSpace.BroadphaseType.AXIS_SWEEP_3);
        testPhysicsSpace(pSpace);

        MultiBodySpace mbSpace = new MultiBodySpace(
                new Vector3f(-10000f, -10000f, -10000f),
                new Vector3f(10000f, 10000f, 10000f),
                PhysicsSpace.BroadphaseType.AXIS_SWEEP_3);
        testPhysicsSpace(mbSpace);

        PhysicsSoftSpace space
                = new PhysicsSoftSpace(PhysicsSpace.BroadphaseType.DBVT);
        testPhysicsSpace(space);

        DeformableSpace dSpace = new DeformableSpace(
                new Vector3f(-10000f, -10000f, -10000f),
                new Vector3f(10000f, 10000f, 10000f),
                PhysicsSpace.BroadphaseType.AXIS_SWEEP_3, SolverType.SI);
        testPhysicsSpace(dSpace);

        testShapes();
        testCollisionObjects();
        // TODO GhostControl, CharacterControl, VehicleControl

        RigidBodyControl dynamicRbc = new RigidBodyControl(box);
        testRigidBody(dynamicRbc);

        RigidBodyMotionState motion = new RigidBodyMotionState();
        testMotionState(motion);

        SoftBodyWorldInfo worldInfo = new SoftBodyWorldInfo();
        Assert.assertEquals(1.2f, worldInfo.airDensity(), 0f);
        Utils.assertEquals(0f, -10f, 0f, worldInfo.copyGravity(null), 0f);
        Assert.assertEquals(1000f, worldInfo.maxDisplacement(), 0f);
        Assert.assertEquals(0f, worldInfo.waterDensity(), 0f);
        Utils.assertEquals(0f, 0f, 0f, worldInfo.copyWaterNormal(null), 0f);
        Assert.assertEquals(0f, worldInfo.waterOffset(), 0f);

        VehicleTuning tuning = new VehicleTuning();
        Assert.assertEquals(10.5f, tuning.getFrictionSlip(), 0f);
        Assert.assertEquals(6000f, tuning.getMaxSuspensionForce(), 0f);
        Assert.assertEquals(500f, tuning.getMaxSuspensionTravelCm(), 0f);
        Assert.assertEquals(0.83f, tuning.getSuspensionCompression(), 0f);
        Assert.assertEquals(0.88f, tuning.getSuspensionDamping(), 0f);
        Assert.assertEquals(5.88f, tuning.getSuspensionStiffness(), 0f);

        testMultiBody();
        testJoints();

        Vhacd4Parameters p4 = new Vhacd4Parameters();
        Assert.assertTrue(p4.isAsync());
        Assert.assertFalse(p4.getDebugEnabled());
        Assert.assertEquals(FillMode.FloodFill, p4.getFillMode());
        Assert.assertFalse(p4.isFindBestPlane());
        Assert.assertEquals(64, p4.getMaxHulls());
        Assert.assertEquals(14, p4.getMaxRecursion());
        Assert.assertEquals(32, p4.getMaxVerticesPerHull());
        Assert.assertEquals(2, p4.getMinEdgeLength());
        Assert.assertTrue(p4.isShrinkWrap());
        Assert.assertEquals(1.0, p4.getVolumePercentError(), 0.0);
        Assert.assertEquals(100_000, p4.getVoxelResolution());

        VHACDParameters parameters = new VHACDParameters();
        Assert.assertFalse(parameters.getDebugEnabled());
        Assert.assertEquals(ACDMode.VOXEL, parameters.getACDMode());
        Assert.assertEquals(0.05, parameters.getAlpha(), 0.0);
        Assert.assertEquals(0.05, parameters.getBeta(), 0.0);
        Assert.assertEquals(4, parameters.getConvexHullDownSampling());
        Assert.assertEquals(0.0025, parameters.getMaxConcavity(), 0.0);
        Assert.assertEquals(32, parameters.getMaxVerticesPerHull());
        Assert.assertEquals(0.0001, parameters.getMinVolumePerHull(), 0.0);
        Assert.assertFalse(parameters.getPCA());
        Assert.assertEquals(4, parameters.getPlaneDownSampling());
        Assert.assertEquals(100_000, parameters.getVoxelResolution());
    }
    // *************************************************************************
    // private methods

    private static void testCollisionObjects() {
        PhysicsCharacter character = new PhysicsCharacter(box, 1f);
        testPco(character);
        Assert.assertEquals(Activation.active, character.getActivationState());
        Assert.assertEquals(0f, character.getAngularDamping(), 0f);
        Utils.assertEquals(0f, 0f, 0f, character.getAngularVelocity(null), 0f);
        Assert.assertEquals(55f, character.getFallSpeed(), 0f);
        Utils.assertEquals(0f, -29.4f, 0f, character.getGravity(null), 1e-4f);
        Assert.assertEquals(29.4f, character.getGravity(null).length(), 1e-4f);
        Assert.assertEquals(10f, character.getJumpSpeed(), 0f);
        Assert.assertEquals(0f, character.getLinearDamping(), 0f);
        Utils.assertEquals(0f, 0f, 0f, character.getLinearVelocity(null), 0f);
        Assert.assertEquals(0.2f, character.getMaxPenetrationDepth(), 0f);
        Assert.assertEquals(FastMath.QUARTER_PI, character.getMaxSlope(), 0f);
        Utils.assertEquals(0f, 0f, 0f, character.getPhysicsLocation(null), 0f);
        Utils.assertEquals(0f, 1f, 0f, character.getUpDirection(null), 1e-5f);
        Utils.assertEquals(0f, 0f, 0f, character.getWalkDirection(null), 0f);
        Assert.assertTrue(character.isContactResponse());
        Assert.assertFalse(character.isStatic());
        Assert.assertTrue(character.isUsingGhostSweepTest());

        PhysicsGhostObject ghost = new PhysicsGhostObject(box);
        testPco(ghost);
        Assert.assertFalse(ghost.isContactResponse());
        Assert.assertTrue(ghost.isStatic());

        PhysicsRigidBody staticPrb
                = new PhysicsRigidBody(box, PhysicsBody.massForStatic);
        testRigidBody(staticPrb);
        RigidBodyControl staticRbc
                = new RigidBodyControl(box, PhysicsBody.massForStatic);
        testRigidBody(staticRbc);

        rigidA = new PhysicsRigidBody(box);
        testRigidBody(rigidA);

        softA = new PhysicsSoftBody();
        testPco(softA);
        Assert.assertTrue(softA.isContactResponse());
        Assert.assertFalse(softA.isStatic());
        Assert.assertFalse(softA.isWorldInfoProtected());
        Assert.assertEquals(0, softA.countClusters());
        Assert.assertEquals(0, softA.countFaces());
        Assert.assertEquals(0, softA.countJoints());
        Assert.assertEquals(0, softA.countLinks());
        Assert.assertEquals(0, softA.countNodes());
        Assert.assertEquals(0, softA.countTetras());
        Assert.assertEquals(0f, softA.getMass(), 0f);
        Assert.assertEquals(Activation.active, softA.getActivationState());

        SoftBodyConfig config = softA.getSoftConfig();
        Assert.assertEquals(Aero.V_Point, config.aerodynamics());
        Assert.assertEquals(4, config.clusterIterations());
        Assert.assertEquals(ConfigFlag.SDF_RS, config.collisionFlags());
        Assert.assertEquals(0, config.driftIterations());
        Assert.assertEquals(0.7f, config.get(Sbcp.AnchorHardness), 0f);
        Assert.assertEquals(1f, config.get(Sbcp.ClusterKineticHardness), 0f);
        Assert.assertEquals(0.5f, config.get(Sbcp.ClusterKineticSplit), 0f);
        Assert.assertEquals(0.1f, config.get(Sbcp.ClusterRigidHardness), 0f);
        Assert.assertEquals(0.5f, config.get(Sbcp.ClusterRigidSplit), 0f);
        Assert.assertEquals(0.5f, config.get(Sbcp.ClusterSoftHardness), 0f);
        Assert.assertEquals(0.5f, config.get(Sbcp.ClusterSoftSplit), 0f);
        Assert.assertEquals(0f, config.get(Sbcp.Damping), 0f);
        Assert.assertEquals(0f, config.get(Sbcp.Drag), 0f);
        Assert.assertEquals(0.2f, config.get(Sbcp.DynamicFriction), 0f);
        Assert.assertEquals(0.1f, config.get(Sbcp.KineticHardness), 0f);
        Assert.assertEquals(0f, config.get(Sbcp.Lift), 0f);
        Assert.assertEquals(1f, config.get(Sbcp.MaxVolumeRatio), 0f);
        Assert.assertEquals(0f, config.get(Sbcp.PoseMatching), 0f);
        Assert.assertEquals(0f, config.get(Sbcp.Pressure), 0f);
        Assert.assertEquals(1f, config.get(Sbcp.RigidHardness), 0f);
        Assert.assertEquals(1f, config.get(Sbcp.SoftHardness), 0f);
        Assert.assertEquals(1f, config.get(Sbcp.TimeScale), 0f);
        Assert.assertEquals(1f, config.get(Sbcp.VelocityCorrection), 0f);
        Assert.assertEquals(0f, config.get(Sbcp.VolumeConservation), 0f);
        Assert.assertEquals(1, config.positionIterations());
        Assert.assertEquals(0, config.velocityIterations());

        SoftBodyMaterial mat = softA.getSoftMaterial();
        Assert.assertEquals(1f, mat.angularStiffness(), 0f);
        Assert.assertEquals(1f, mat.linearStiffness(), 0f);
        Assert.assertEquals(1f, mat.volumeStiffness(), 0f);

        Mesh wireBox = new WireBox();
        NativeSoftBodyUtil.appendFromLineMesh(wireBox, softA);
        softA.generateClusters(2, 999);
        softA.setMass(1f);

        PhysicsVehicle vehicle = new PhysicsVehicle(box);
        testRigidBody(vehicle);
        Assert.assertEquals(10.5f, vehicle.getFrictionSlip(), 0f);
        Assert.assertEquals(6000f, vehicle.getMaxSuspensionForce(), 0f);
        Assert.assertEquals(500f, vehicle.getMaxSuspensionTravelCm(), 0f);
        Assert.assertEquals(0, vehicle.getNumWheels());
        Assert.assertEquals(0.83f, vehicle.getSuspensionCompression(), 0f);
        Assert.assertEquals(0.88f, vehicle.getSuspensionDamping(), 0f);
        Assert.assertEquals(5.88f, vehicle.getSuspensionStiffness(), 0f);

        PhysicsSpace space
                = new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        space.addCollisionObject(vehicle);
        Assert.assertEquals(PhysicsSpace.AXIS_Z, vehicle.forwardAxisIndex());
        Assert.assertNotEquals(0L, vehicle.getVehicleId());
        Assert.assertEquals(0f, vehicle.getCurrentVehicleSpeedKmHour(), 0f);
        Utils.assertEquals(0f, 0f, 1f, vehicle.getForwardVector(null), 0f);
        Assert.assertEquals(PhysicsSpace.AXIS_X, vehicle.rightAxisIndex());
        Assert.assertEquals(PhysicsSpace.AXIS_Y, vehicle.upAxisIndex());

        Vector3f connectionPoint = Vector3f.ZERO;
        Vector3f direction = new Vector3f(0f, -1f, 0f);
        Vector3f axle = new Vector3f(-1f, 0f, 0f);
        float suspensionRestLength = 0.1f;
        float wheelRadius = 1f;
        boolean isFrontWheel = true;
        VehicleWheel wheel = vehicle.addWheel(
                null, connectionPoint, direction, axle, suspensionRestLength,
                wheelRadius, isFrontWheel);
        Assert.assertEquals(0f, wheel.getBrake(), 0f);
        Assert.assertEquals(0f, wheel.getEngineForce(), 0f);
        Assert.assertEquals(10.5f, wheel.getFrictionSlip(), 0f);
        Assert.assertEquals(500f, wheel.getMaxSuspensionTravelCm(), 0f);
        Assert.assertEquals(1f, wheel.getRollInfluence(), 0f);
        Assert.assertEquals(0f, wheel.getRotationAngle(), 0f);
        Assert.assertEquals(0f, wheel.getSteerAngle(), 0f);
        Assert.assertEquals(5.88f, wheel.getSuspensionStiffness(), 0f);
        Assert.assertEquals(0.83f, wheel.getWheelsDampingCompression(), 0f);
        Assert.assertEquals(0.88f, wheel.getWheelsDampingRelaxation(), 0f);
        Assert.assertFalse(wheel.isApplyLocal());
    }

    /**
     * Verify defaults common to all newly-created collision spaces.
     *
     * @param space the space to test (not null, unaffected)
     */
    private static void testCollisionSpace(CollisionSpace space) {
        Assert.assertNotNull(space);
        Assert.assertTrue(space.isEmpty());
        Assert.assertEquals(0, space.countCollisionGroupListeners());
        Assert.assertEquals(0, space.countCollisionObjects());
        Assert.assertEquals(
                RayTestFlag.SubSimplexRaytest, space.getRayTestFlags());
        Assert.assertTrue(space.getGhostObjectList().isEmpty());
        Assert.assertTrue(space.getPcoList().isEmpty());
        Assert.assertNotEquals(0L, space.nativeId());
        Assert.assertTrue(space.isForceUpdateAllAabbs());
        Assert.assertFalse(space.isUsingDeterministicDispatch());
    }

    private static void testConstraint(Constraint constraint) {
        Assert.assertFalse(constraint.isFeedback());
        constraint.setFeedback(true);

        Assert.assertEquals(0f, constraint.getAppliedImpulse(), 0f);

        float simdInf = NativeLibrary.isDoublePrecision()
                ? Float.POSITIVE_INFINITY : Float.MAX_VALUE;
        Assert.assertEquals(
                simdInf, constraint.getBreakingImpulseThreshold(), 0f);
        Assert.assertTrue(constraint.isCollisionBetweenLinkedBodies());
        Assert.assertTrue(constraint.isEnabled());
    }

    private static void testJoints() {
        PhysicsRigidBody rigidB = new PhysicsRigidBody(box);

        // TODO Anchor
        // TODO ConeJoint
        GearJoint gear = new GearJoint(rigidA, rigidB, new Vector3f(1f, 0f, 0f),
                new Vector3f(1f, 0f, 0f));
        testGear(gear, 2);

        HingeJoint seHinge = new HingeJoint(
                rigidA, new Vector3f(), new Vector3f(),
                new Vector3f(1f, 0f, 0f), new Vector3f(1f, 0f, 0f), JointEnd.A);
        testHinge(seHinge, 1);

        HingeJoint deHinge = new HingeJoint(
                rigidA, rigidB, new Vector3f(), new Vector3f(),
                new Vector3f(1f, 0f, 0f), new Vector3f(1f, 0f, 0f));
        testHinge(deHinge, 2);

        New6Dof seNew6Dof = new New6Dof(
                rigidB, new Vector3f(), new Vector3f(), new Matrix3f(),
                new Matrix3f(), RotationOrder.ZYX);
        testNew6Dof(seNew6Dof, 1);

        New6Dof deNew6Dof = new New6Dof(
                rigidA, rigidB, new Vector3f(), new Vector3f(), new Matrix3f(),
                new Matrix3f(), RotationOrder.XYZ);
        testNew6Dof(deNew6Dof, 2);

        NewHinge newHinge = new NewHinge(rigidA, rigidB, Vector3f.ZERO,
                Vector3f.UNIT_Z, Vector3f.UNIT_X);
        testNewHinge(newHinge);

        Point2PointJoint sep2p = new Point2PointJoint(rigidA, new Vector3f());
        testP2P(sep2p, 1);

        Point2PointJoint dep2p = new Point2PointJoint(
                rigidA, rigidB, new Vector3f(), new Vector3f());
        testP2P(dep2p, 2);

        SixDofJoint deSix = new SixDofJoint(
                rigidA, rigidB, new Vector3f(), new Vector3f(), false);
        testSixDof(deSix, 2);

        SixDofSpringJoint deSpring = new SixDofSpringJoint(rigidA, rigidB,
                new Vector3f(), new Vector3f(), new Matrix3f(), new Matrix3f(),
                false);
        testSixDof(deSpring, 2);

        SliderJoint deSlider = new SliderJoint(rigidA, rigidB,
                new Vector3f(0f, 0f, 0f), new Vector3f(0f, 0f, 0f), false);
        testSlider(deSlider, 2);

        SoftAngularJoint srAngularJoint = new SoftAngularJoint(
                new Vector3f(0f, 0f, 0f), softA, 0, rigidB);
        Assert.assertEquals(2, srAngularJoint.countEnds());
        Assert.assertEquals(1f, srAngularJoint.getCFM(), 0f);
        Assert.assertTrue(srAngularJoint.isEnabled());
        Assert.assertEquals(1f, srAngularJoint.getERP(), 0f);
        Assert.assertEquals(1f, srAngularJoint.getSplit(), 0f);
    }

    private static void testMultiBody() {
        int numLinks = 1;
        float mass = 1f;
        Vector3f inertia = new Vector3f(1f, 1f, 1f);
        boolean fixedBase = false;
        boolean canSleep = true;
        MultiBody mb
                = new MultiBody(numLinks, mass, inertia, fixedBase, canSleep);
        mb.addBaseCollider(box);
        MultiBodyLink parent = null;
        Quaternion orientation = new Quaternion();
        Vector3f parent2Pivot = new Vector3f(1f, 0f, 0f);
        Vector3f pivot2Link = new Vector3f(0f, 0f, 1f);
        boolean disableCollision = true;
        MultiBodyLink link = mb.configureSphericalLink(mass, inertia, parent,
                orientation, parent2Pivot, pivot2Link, disableCollision);
        link.addCollider(box);

        Assert.assertEquals(0.04f, mb.angularDamping(), 0f);
        Utils.assertEquals(0f, 0f, 0f, mb.baseAngularVelocity(null), 0f);
        Utils.assertEquals(0f, 0f, 0f, mb.baseForce(null), 0f);
        Utils.assertEquals(0f, 0f, 0f, mb.baseLocation(null), 0f);
        Utils.assertEquals(0f, 0f, 0f, 1f, mb.baseOrientation(null), 0f);
        Utils.assertEquals(0f, 0f, 0f, mb.baseTorque(null), 0f);
        Utils.assertEquals(0f, 0f, 0f, mb.baseVelocity(null), 0f);
        Assert.assertTrue(mb.canWakeup());
        Assert.assertEquals(PhysicsCollisionObject.COLLISION_GROUP_01,
                mb.collideWithGroups());
        Assert.assertEquals(PhysicsCollisionObject.COLLISION_GROUP_01,
                mb.collisionGroup());
        Assert.assertEquals(1, mb.countConfiguredLinks());
        Assert.assertEquals(3, mb.countDofs());
        Assert.assertEquals(4, mb.countPositionVariables());
        Assert.assertFalse(mb.isUsingGlobalVelocities());
        Assert.assertTrue(mb.isUsingGyroTerm());
        Assert.assertFalse(mb.isUsingRK4());
        Assert.assertEquals(0.04f, mb.linearDamping(), 0f);
        Assert.assertEquals(1000f, mb.maxAppliedImpulse(), 0f);
        Assert.assertEquals(100f, mb.maxCoordinateVelocity(), 0f);

        MultiBodyCollider baseCollider = mb.getBaseCollider();
        Utils.assertEquals(
                0f, 0f, 0f, 1f, baseCollider.getPhysicsRotation(null), 0f);
        testPco(baseCollider);
        Assert.assertFalse(baseCollider.isStatic());

        MultiBodyCollider linkCollider = link.getCollider();
        Utils.assertEquals(
                0f, 0f, 0f, 1f, linkCollider.getPhysicsRotation(null), 0f);
        testPco(linkCollider);
        Assert.assertFalse(linkCollider.isStatic());
    }

    private static void testGear(GearJoint constraint, int numEnds) {
        Assert.assertEquals(numEnds, constraint.countEnds());
        testConstraint(constraint);

        Assert.assertEquals(1f, constraint.getRatio(), 0f);
    }

    private static void testHinge(HingeJoint constraint, int numEnds) {
        Assert.assertEquals(numEnds, constraint.countEnds());
        testConstraint(constraint);

        Assert.assertFalse(constraint.isAngularOnly());
        Assert.assertEquals(0.3f, constraint.getBiasFactor(), 0f);
        Assert.assertEquals(0.9f, constraint.getLimitSoftness(), 0f);
        Assert.assertEquals(1f, constraint.getLowerLimit(), 0f);
        Assert.assertEquals(1f, constraint.getRelaxationFactor(), 0f);
        Assert.assertEquals(-1f, constraint.getUpperLimit(), 0f);
    }

    private static void testMotionState(RigidBodyMotionState motion) {
        Vector3f x = motion.getLocation(null);
        Utils.assertEquals(0f, 0f, 0f, x, 0f);

        Vec3d xx = motion.getLocationDp(null);
        Utils.assertEquals(0., 0., 0., xx, 0.);

        Matrix3f m = new Matrix3f();
        motion.getOrientation(m);
        Assert.assertTrue(m.isIdentity());

        Quaternion q = new Quaternion();
        motion.getOrientation(q);
        Utils.assertEquals(0f, 0f, 0f, 1f, q, 0f);

        // TODO when Matrix3d.isIdentity() is available:
        //Matrix3d mm = motion.getOrientationMatrixDp(null);
        //Assert.assertTrue(mm.isIdentity());
        Quatd qq = motion.getOrientationQuaternionDp(null);
        Utils.assertEquals(0., 0., 0., 1., qq, 0.);

        Assert.assertFalse(motion.isApplyPhysicsLocal());

        Transform t = motion.physicsTransform(null);
        Assert.assertTrue(MyMath.isIdentity(t));
    }

    private static void testNew6Dof(New6Dof constraint, int numEnds) {
        Assert.assertEquals(numEnds, constraint.countEnds());
        testConstraint(constraint);

        for (int axisIndex = PhysicsSpace.AXIS_X;
                axisIndex <= PhysicsSpace.AXIS_Z; ++axisIndex) {
            RotationMotor motor = constraint.getRotationMotor(axisIndex);
            Assert.assertFalse(motor.isDampingLimited());
            Assert.assertFalse(motor.isMotorEnabled());
            Assert.assertFalse(motor.isServoEnabled());
            Assert.assertFalse(motor.isSpringEnabled());
            Assert.assertFalse(motor.isStiffnessLimited());

            for (MotorParam param : MotorParam.values()) {
                float defaultValue = param.defaultForRotationMotor();
                float value = motor.get(param);
                Assert.assertEquals(defaultValue, value, 0f);
            }
        }

        TranslationMotor motor = constraint.getTranslationMotor();
        for (int axisIndex = PhysicsSpace.AXIS_X;
                axisIndex <= PhysicsSpace.AXIS_Z; ++axisIndex) {
            Assert.assertFalse(motor.isDampingLimited(axisIndex));
            Assert.assertFalse(motor.isMotorEnabled(axisIndex));
            Assert.assertFalse(motor.isServoEnabled(axisIndex));
            Assert.assertFalse(motor.isSpringEnabled(axisIndex));
            Assert.assertFalse(motor.isStiffnessLimited(axisIndex));
        }

        for (MotorParam param : MotorParam.values()) {
            float def = param.defaultForTranslationMotor();
            Vector3f value = motor.get(param, null);
            Utils.assertEquals(def, def, def, value, 0f);
        }
    }

    private static void testNewHinge(NewHinge constraint) {
        Assert.assertEquals(2, constraint.countEnds());
        testConstraint(constraint);

        for (int axisIndex = PhysicsSpace.AXIS_X;
                axisIndex <= PhysicsSpace.AXIS_Z; ++axisIndex) {
            RotationMotor motor = constraint.getRotationMotor(axisIndex);
            Assert.assertFalse(motor.isDampingLimited());
            Assert.assertFalse(motor.isMotorEnabled());
            Assert.assertFalse(motor.isServoEnabled());
            Assert.assertFalse(motor.isSpringEnabled());
            Assert.assertFalse(motor.isStiffnessLimited());

            for (MotorParam param : MotorParam.values()) {
                float defaultValue = param.defaultForRotationMotor();
                if (param == MotorParam.LowerLimit) {
                    if (axisIndex == PhysicsSpace.AXIS_Y) {
                        defaultValue = 0f;
                    } else if (axisIndex == PhysicsSpace.AXIS_Z) {
                        defaultValue = -FastMath.PI / 4f;
                    }
                } else if (param == MotorParam.UpperLimit) {
                    if (axisIndex == PhysicsSpace.AXIS_Y) {
                        defaultValue = 0f;
                    } else if (axisIndex == PhysicsSpace.AXIS_Z) {
                        defaultValue = FastMath.PI / 4f;
                    }
                }
                float value = motor.get(param);
                Assert.assertEquals(defaultValue, value, 0f);
            }
        }

        TranslationMotor motor = constraint.getTranslationMotor();
        for (int axisIndex = PhysicsSpace.AXIS_X;
                axisIndex <= PhysicsSpace.AXIS_Z; ++axisIndex) {
            Assert.assertFalse(motor.isDampingLimited(axisIndex));
            Assert.assertFalse(motor.isMotorEnabled(axisIndex));
            Assert.assertFalse(motor.isServoEnabled(axisIndex));
            if (axisIndex == PhysicsSpace.AXIS_Z) {
                Assert.assertTrue(motor.isSpringEnabled(axisIndex));
            } else {
                Assert.assertFalse(motor.isSpringEnabled(axisIndex));
            }
            Assert.assertFalse(motor.isStiffnessLimited(axisIndex));
        }

        for (MotorParam param : MotorParam.values()) {
            Vector3f value = motor.get(param, null);
            switch (param) {
                case Damping:
                    value.z = 0.01f;
                    break;
                case LowerLimit:
                    value.z = -1f;
                    break;
                case Stiffness:
                    value.z = 4f * FastMath.PI * FastMath.PI;
                    break;
                case UpperLimit:
                    value.z = 1f;
                    break;
                default:
                    float def = param.defaultForTranslationMotor();
                    Utils.assertEquals(def, def, def, value, 0f);
            }
        }
    }

    private static void testP2P(Point2PointJoint p2p, int numEnds) {
        Assert.assertEquals(numEnds, p2p.countEnds());
        testConstraint(p2p);

        Assert.assertEquals(1f, p2p.getDamping(), 0f);
        Assert.assertEquals(0f, p2p.getImpulseClamp(), 0f);
        Assert.assertEquals(0.3f, p2p.getTau(), 0f);
    }

    /**
     * Test the defaults that are common to all newly-created collision objects.
     *
     * @param pco the object to test (not null, unaffected)
     */
    private static void testPco(PhysicsCollisionObject pco) {
        long id = pco.nativeId();
        Assert.assertNotEquals(0L, id);
        Assert.assertEquals(pco, PhysicsCollisionObject.findInstance(id));
        Assert.assertEquals(0, pco.countIgnored());
        Assert.assertFalse(pco.isInWorld());
        Assert.assertEquals(0L, pco.spaceId());
        Assert.assertTrue(pco.isActive());
        Assert.assertFalse(pco.isInWorld());

        Utils.assertEquals(1f, 1f, 1f, pco.getAnisotropicFriction(null), 0f);
        Assert.assertFalse(pco.hasAnisotropicFriction(1));
        Assert.assertFalse(pco.hasAnisotropicFriction(2));
        Assert.assertNull(pco.getApplicationData());
        Assert.assertEquals(0f, pco.getCcdMotionThreshold(), 0f);
        Assert.assertEquals(0f, pco.getCcdSquareMotionThreshold(), 0f);
        Assert.assertEquals(0f, pco.getCcdSweptSphereRadius(), 0f);
        Assert.assertEquals(PhysicsCollisionObject.COLLISION_GROUP_01,
                pco.getCollideWithGroups());
        Assert.assertEquals(PhysicsCollisionObject.COLLISION_GROUP_01,
                pco.getCollisionGroup());
        Assert.assertNull(pco.getCollisionSpace());
        Assert.assertEquals(0.1f, pco.getContactDamping(), 0f);

        float largeFloat = NativeLibrary.isDoublePrecision() ? 1e30f : 1e18f;
        Assert.assertEquals(
                largeFloat, pco.getContactProcessingThreshold(), 0f);
        Assert.assertEquals(largeFloat, pco.getContactStiffness(), 0f);

        Assert.assertEquals(0f, pco.getDeactivationTime(), 0f);
        Assert.assertNull(pco.getDebugMaterial());
        Assert.assertNull(pco.debugMeshInitListener());
        Assert.assertSame(MeshNormals.None, pco.debugMeshNormals());
        Assert.assertEquals(
                DebugShapeFactory.lowResolution, pco.debugMeshResolution());
        Assert.assertEquals(1, pco.debugNumSides());
        Assert.assertEquals(0.5f, pco.getFriction(), 0f);

        Utils.assertEquals(0f, 0f, 0f, pco.getPhysicsLocation(null), 0f);
        Utils.assertEquals(0., 0., 0., pco.getPhysicsLocationDp(null), 0.);
        Utils.assertEquals(0f, 0f, 0f, 1f, pco.getPhysicsRotation(null), 0f);
        Utils.assertEquals(0., 0., 0., 1., pco.getPhysicsRotationDp(null), 0.);
        Assert.assertEquals(
                Matrix3f.IDENTITY, pco.getPhysicsRotationMatrix(null));
        // TODO test disabled until Matrix3d.equals() is fixed:
        //Assert.assertEquals(
        //        new Matrix3d(), pco.getPhysicsRotationMatrixDp(null));

        Assert.assertNull(pco.proxyGroup());
        Assert.assertNull(pco.proxyMask());
        Assert.assertEquals(0f, pco.getRestitution(), 0f);
        Assert.assertEquals(0f, pco.getRollingFriction(), 0f);
        Assert.assertEquals(0f, pco.getSpinningFriction(), 0f);
        Assert.assertNull(pco.getUserObject());
    }

    /**
     * Verify defaults common to all newly-created physics spaces.
     *
     * @param space the space to test (not null, unaffected)
     */
    private static void testPhysicsSpace(PhysicsSpace space) {
        testCollisionSpace(space);

        Assert.assertEquals(0, space.countCollisionGroupListeners());
        Assert.assertEquals(0, space.countCollisionListeners());
        Assert.assertEquals(0, space.countJoints());
        Assert.assertEquals(0, space.countRigidBodies());
        Assert.assertEquals(0, space.countTickListeners());

        Assert.assertEquals(1 / 60f, space.getAccuracy(), 0f);
        Utils.assertEquals(0f, -9.81f, 0f, space.getGravity(null), 0f);
        Assert.assertFalse(space.isCcdWithStaticOnly());
        Assert.assertFalse(space.isUsingScr());
        Assert.assertEquals(4, space.maxSubSteps());
        Assert.assertEquals(0.1f, space.maxTimeStep(), 0f);

        SolverInfo info = space.getSolverInfo();
        Assert.assertNotNull(info);
        Assert.assertNotEquals(0L, info.nativeId());
        Assert.assertEquals(0.2f, info.contactErp(), 0f);
        Assert.assertEquals(0f, info.globalCfm(), 0f);
        Assert.assertEquals(0.2f, info.jointErp(), 0f);
        Assert.assertEquals(128, info.minBatch());

        String className = space.getClass().getSimpleName();
        int expectedMode = (className.equals("MultiBodySpace")
                || className.equals("DeformableSpace")) ? 0x114 : 0x104;
        Assert.assertEquals(expectedMode, info.mode());

        Assert.assertEquals(10, info.numIterations());
        Assert.assertTrue(info.isSplitImpulseEnabled());
        Assert.assertEquals(0.1f, info.splitImpulseErp(), 0f);
        Assert.assertEquals(-0.04f, info.splitImpulseThreshold(), 0f);

        if (space instanceof MultiBodySpace) {
            MultiBodySpace mbSpace = (MultiBodySpace) space;
            Assert.assertEquals(0, mbSpace.countMultiBodies());

            if (space instanceof DeformableSpace) {
                DeformableSpace dSpace = (DeformableSpace) space;
                Assert.assertEquals(0, dSpace.countSoftBodies());

                SoftBodyWorldInfo sbwi = dSpace.getWorldInfo();
                Assert.assertEquals(1.2f, sbwi.airDensity(), 0f);
                Utils.assertEquals(0f, -9.81f, 0f, sbwi.copyGravity(null), 0f);
                Assert.assertEquals(1000f, sbwi.maxDisplacement(), 0f);
                Assert.assertEquals(0f, sbwi.waterDensity(), 0f);
                Utils.assertEquals(0f, 0f, 0f, sbwi.copyWaterNormal(null), 0f);
                Assert.assertEquals(0f, sbwi.waterOffset(), 0f);
            }

        } else if (space instanceof PhysicsSoftSpace) {
            PhysicsSoftSpace softSpace = (PhysicsSoftSpace) space;
            Assert.assertEquals(0, softSpace.countSoftBodies());

            SoftBodyWorldInfo sbwi = softSpace.getWorldInfo();
            Assert.assertEquals(1.2f, sbwi.airDensity(), 0f);
            Utils.assertEquals(0f, -9.81f, 0f, sbwi.copyGravity(null), 0f);
            Assert.assertEquals(1000f, sbwi.maxDisplacement(), 0f);
            Assert.assertEquals(0f, sbwi.waterDensity(), 0f);
            Utils.assertEquals(0f, 0f, 0f, sbwi.copyWaterNormal(null), 0f);
            Assert.assertEquals(0f, sbwi.waterOffset(), 0f);
        }
    }

    /**
     * Test the defaults that are common to all newly-created rigid bodies.
     *
     * @param prb the body to test (not null, unaffected)
     */
    private static void testRigidBody(PhysicsRigidBody prb) {
        Assert.assertNotNull(prb);
        testPco(prb);

        Assert.assertEquals(0, prb.countJoints());
        Assert.assertEquals(Activation.active, prb.getActivationState());
        Assert.assertEquals(0f, prb.getAngularDamping(), 0f);
        Utils.assertEquals(1f, 1f, 1f, prb.getAngularFactor(null), 0f);
        Assert.assertEquals(1f, prb.getAngularSleepingThreshold(), 0f);
        Utils.assertEquals(0f, 0f, 0f, prb.getGravity(null), 0f);
        Assert.assertEquals(0f, prb.getLinearDamping(), 0f);
        Utils.assertEquals(1f, 1f, 1f, prb.getLinearFactor(null), 0f);
        Assert.assertEquals(0.8f, prb.getLinearSleepingThreshold(), 0f);
        Assert.assertTrue(prb.isContactResponse());
        Assert.assertFalse(prb.isGravityProtected());
        Assert.assertFalse(prb.isKinematic());
        Assert.assertEquals(0, prb.listJoints().length);
        Utils.assertEquals(0f, 0f, 0f, prb.totalAppliedForce(null), 0f);
        Utils.assertEquals(0f, 0f, 0f, prb.totalAppliedTorque(null), 0f);

        RigidBodyMotionState motion = prb.getMotionState();
        Assert.assertNotNull(motion);
        testMotionState(motion);

        if (prb.getMass() > 0f) {
            Utils.assertEquals(0f, 0f, 0f, prb.getAngularVelocity(null), 0f);
            Utils.assertEquals(
                    0f, 0f, 0f, prb.getAngularVelocityLocal(null), 0f);
            Utils.assertEquals(0f, 0f, 0f, prb.getLinearVelocity(null), 0f);
            Assert.assertEquals(1f, prb.getMass(), 0f);
            Assert.assertEquals(0f, prb.getSquaredSpeed(), 0f);
            Assert.assertTrue(prb.isDynamic());
            Assert.assertFalse(prb.isStatic());
            Assert.assertEquals(0f, prb.kineticEnergy(), 0f);
            Assert.assertEquals(0f, prb.mechanicalEnergy(), 0f);

        } else {
            Assert.assertEquals(0f, prb.getMass(), 0f);
            Assert.assertFalse(prb.isDynamic());
            Assert.assertTrue(prb.isStatic());
        }

        if (prb instanceof RigidBodyControl) {
            RigidBodyControl rbc = (RigidBodyControl) prb;
            Assert.assertNull(rbc.getSpatial());
            Assert.assertFalse(rbc.isApplyPhysicsLocal());
            Assert.assertFalse(rbc.isApplyScale());
            Assert.assertTrue(rbc.isKinematicSpatial());
        }
    }

    /**
     * Test the defaults that are common to all newly-created collision shapes.
     *
     * @param shape the shape to test (not null, unaffected)
     */
    private static void testShape(CollisionShape shape) {
        Assert.assertNotNull(shape);
        Assert.assertNotEquals(0L, shape.nativeId());
        Utils.assertEquals(1f, 1f, 1f, shape.getScale(null), 0f);
        Utils.assertEquals(1., 1., 1., shape.getScaleDp(null), 0.);
        Assert.assertTrue(shape.isContactFilterEnabled());
    }

    /**
     * Verify the defaults for all collision shapes.
     */
    private static void testShapes() {
        testShapesConcave();
        testShapesConvex();

        // Compound
        CompoundCollisionShape compound = new CompoundCollisionShape();
        testShape(compound);
        Assert.assertEquals(0, compound.countChildren());
        Assert.assertEquals(0.04f, compound.getMargin(), 0f);
        Assert.assertFalse(compound.isConcave());
        Assert.assertFalse(compound.isConvex());
        Assert.assertFalse(compound.isInfinite());
        Assert.assertFalse(compound.isNonMoving());
        Assert.assertFalse(compound.isPolyhedral());
    }

    private static void testShapesConcave() {
        // Empty
        EmptyShape empty = new EmptyShape(true);
        testShape(empty);
        Assert.assertEquals(0.04f, empty.getMargin(), 0f);
        Assert.assertTrue(empty.isConcave());
        Assert.assertFalse(empty.isConvex());
        Assert.assertFalse(empty.isInfinite());
        Assert.assertTrue(empty.isNonMoving());
        Assert.assertFalse(empty.isPolyhedral());
        Assert.assertEquals(0f, empty.unscaledVolume(), 0f);

        // GImpact
        Torus torus = new Torus(16, 16, 0.2f, 0.8f);
        GImpactCollisionShape gimpact = new GImpactCollisionShape(torus);
        testShape(gimpact);
        Assert.assertEquals(0.04f, gimpact.getMargin(), 0f);
        Assert.assertTrue(gimpact.isConcave());
        Assert.assertFalse(gimpact.isConvex());
        Assert.assertFalse(gimpact.isInfinite());
        Assert.assertFalse(gimpact.isNonMoving());
        Assert.assertFalse(gimpact.isPolyhedral());

        // Heightfield
        float[] nineHeights = {1f, 0f, 1f, 0f, 0.5f, 0f, 1f, 0f, 1f};
        HeightfieldCollisionShape hcs
                = new HeightfieldCollisionShape(nineHeights);
        testShape(hcs);
        Assert.assertEquals(9, hcs.countMeshVertices());
        Assert.assertEquals(0.04f, hcs.getMargin(), 0f);
        Assert.assertTrue(hcs.isConcave());
        Assert.assertFalse(hcs.isConvex());
        Assert.assertFalse(hcs.isInfinite());
        Assert.assertTrue(hcs.isNonMoving());
        Assert.assertFalse(hcs.isPolyhedral());

        // Mesh
        Quad quad = new Quad(1f, 1f);
        MeshCollisionShape mesh = new MeshCollisionShape(quad);
        testShape(mesh);
        Assert.assertEquals(2, mesh.countMeshTriangles());
        Assert.assertEquals(4, mesh.countMeshVertices());
        Assert.assertEquals(0.04f, mesh.getMargin(), 0f);
        Assert.assertTrue(mesh.isConcave());
        Assert.assertFalse(mesh.isConvex());
        Assert.assertFalse(mesh.isInfinite());
        Assert.assertTrue(mesh.isNonMoving());
        Assert.assertFalse(mesh.isPolyhedral());

        // Plane
        Plane plane = new Plane(new Vector3f(0f, 1f, 0f), 0f);
        PlaneCollisionShape pcs = new PlaneCollisionShape(plane);
        testShape(pcs);
        Assert.assertEquals(0.04f, pcs.getMargin(), 0f);
        Assert.assertEquals(0f, pcs.getPlane().getConstant(), 0f);
        Utils.assertEquals(0f, 1f, 0f, pcs.getPlane().getNormal(), 0f);
        Assert.assertTrue(pcs.isConcave());
        Assert.assertFalse(pcs.isConvex());
        Assert.assertTrue(pcs.isInfinite());
        Assert.assertTrue(pcs.isNonMoving());
        Assert.assertFalse(pcs.isPolyhedral());
    }

    private static void testShapesConvex() {
        // Box2d
        Box2dShape box2d = new Box2dShape(1f, 2f);
        testShape(box2d);
        Assert.assertEquals(0.04f, box2d.getMargin(), 0f);
        Assert.assertFalse(box2d.isConcave());
        Assert.assertTrue(box2d.isConvex());
        Assert.assertFalse(box2d.isInfinite());
        Assert.assertFalse(box2d.isNonMoving());
        Assert.assertFalse(box2d.isPolyhedral());

        // Box
        box = new BoxCollisionShape(1f);
        testShape(box);
        Assert.assertEquals(0.04f, box.getMargin(), 0f);
        Assert.assertFalse(box.isConcave());
        Assert.assertTrue(box.isConvex());
        Assert.assertFalse(box.isInfinite());
        Assert.assertFalse(box.isNonMoving());
        Assert.assertTrue(box.isPolyhedral());
        Assert.assertEquals(8f, box.unscaledVolume(), 0f);

        // Capsule
        CapsuleCollisionShape capsule = new CapsuleCollisionShape(1f, 1f);
        testShape(capsule);
        Assert.assertEquals(PhysicsSpace.AXIS_Y, capsule.getAxis());
        Assert.assertEquals(1f, capsule.getHeight(), 0f);
        Assert.assertEquals(0f, capsule.getMargin(), 0f);
        Assert.assertFalse(capsule.isConcave());
        Assert.assertTrue(capsule.isConvex());
        Assert.assertFalse(capsule.isInfinite());
        Assert.assertFalse(capsule.isNonMoving());
        Assert.assertFalse(capsule.isPolyhedral());
        Assert.assertEquals(
                7f * FastMath.PI / 3f, capsule.unscaledVolume(), 1e-4f);

        // Cone
        ConeCollisionShape cone = new ConeCollisionShape(1f, 1f);
        testShape(cone);
        Assert.assertEquals(PhysicsSpace.AXIS_Y, cone.getAxis());
        Assert.assertEquals(1f, cone.getHeight(), 0f);
        Assert.assertEquals(0.04f, cone.getMargin(), 0f);
        Assert.assertFalse(cone.isConcave());
        Assert.assertTrue(cone.isConvex());
        Assert.assertFalse(cone.isInfinite());
        Assert.assertFalse(cone.isNonMoving());
        Assert.assertFalse(cone.isPolyhedral());
        Assert.assertEquals(
                FastMath.PI / 3f, cone.unscaledVolume(), 1e-5f);

        // Convex2d
        ConeCollisionShape flatCone
                = new ConeCollisionShape(10f, 0f, PhysicsSpace.AXIS_Z);
        Convex2dShape convex2d = new Convex2dShape(flatCone);
        testShape(convex2d);
        Assert.assertEquals(0.04f, convex2d.getMargin(), 0f);
        Assert.assertFalse(convex2d.isConcave());
        Assert.assertTrue(convex2d.isConvex());
        Assert.assertFalse(convex2d.isInfinite());
        Assert.assertFalse(convex2d.isNonMoving());
        Assert.assertFalse(convex2d.isPolyhedral());

        // Cylinder
        CylinderCollisionShape cylinder
                = new CylinderCollisionShape(new Vector3f(1f, 1f, 1f));
        testShape(cylinder);
        Assert.assertEquals(PhysicsSpace.AXIS_Z, cylinder.getAxis());
        Assert.assertEquals(2f, cylinder.getHeight(), 0f);
        Assert.assertEquals(0.04f, cylinder.getMargin(), 0f);
        Assert.assertFalse(cylinder.isConcave());
        Assert.assertTrue(cylinder.isConvex());
        Assert.assertFalse(cylinder.isInfinite());
        Assert.assertFalse(cylinder.isNonMoving());
        Assert.assertFalse(cylinder.isPolyhedral());
        Assert.assertEquals(FastMath.TWO_PI, cylinder.unscaledVolume(), 1e-4f);

        // Hull
        List<Vector3f> prismVertices = new ArrayList<>(6);
        prismVertices.add(new Vector3f(1f, 1f, 1f));
        prismVertices.add(new Vector3f(1f, 1f, -1f));
        prismVertices.add(new Vector3f(-1f, 1f, 0f));
        prismVertices.add(new Vector3f(1f, -1f, 1f));
        prismVertices.add(new Vector3f(1f, -1f, -1f));
        prismVertices.add(new Vector3f(-1f, -1f, 0f));
        HullCollisionShape hull = new HullCollisionShape(prismVertices);
        testShape(hull);
        Assert.assertEquals(8f, hull.aabbVolume(), 0.001f);
        Assert.assertEquals(6, hull.countHullVertices());
        Assert.assertEquals(6, hull.countMeshVertices());
        Assert.assertEquals(0.04f, hull.getMargin(), 0f);
        Assert.assertFalse(hull.isConcave());
        Assert.assertTrue(hull.isConvex());
        Assert.assertFalse(hull.isInfinite());
        Assert.assertFalse(hull.isNonMoving());
        Assert.assertTrue(hull.isPolyhedral());
        Assert.assertEquals(4f, hull.scaledVolume(), 1f);

        // MultiSphere
        MultiSphere multiSphere = new MultiSphere(1f);
        testShape(multiSphere);
        Utils.assertEquals(0f, 0f, 0f, multiSphere.copyCenter(0, null), 0f);
        Assert.assertEquals(1, multiSphere.countSpheres());
        Assert.assertEquals(1f, multiSphere.getRadius(0), 0f);
        Assert.assertEquals(
                4f * FastMath.PI / 3f, multiSphere.scaledVolume(), 1e-4f);
        Assert.assertFalse(multiSphere.isConcave());
        Assert.assertTrue(multiSphere.isConvex());
        Assert.assertFalse(multiSphere.isInfinite());
        Assert.assertFalse(multiSphere.isNonMoving());
        Assert.assertFalse(multiSphere.isPolyhedral());

        // Simplex of 1 vertex
        SimplexCollisionShape simplex1
                = new SimplexCollisionShape(new Vector3f(0f, 0f, 0f));
        testSimplexShape(simplex1);
        Assert.assertEquals(1, simplex1.countMeshVertices());
        Utils.assertEquals(0f, 0f, 0f, simplex1.copyVertex(0, null), 0f);
        Utils.assertEquals(0f, 0f, 0f, simplex1.getHalfExtents(null), 0f);
        Assert.assertEquals(0f, simplex1.unscaledVolume(), 0f);

        // Simplex of 2 vertices
        SimplexCollisionShape simplex2 = new SimplexCollisionShape(
                new Vector3f(1f, 0f, 0f), new Vector3f(-1, 0f, 0f));
        testSimplexShape(simplex2);
        Assert.assertEquals(2, simplex2.countMeshVertices());
        Utils.assertEquals(1f, 0f, 0f, simplex2.copyVertex(0, null), 0f);
        Utils.assertEquals(-1f, 0f, 0f, simplex2.copyVertex(1, null), 0f);
        Utils.assertEquals(1f, 0f, 0f, simplex2.getHalfExtents(null), 0f);
        Assert.assertEquals(0f, simplex2.unscaledVolume(), 0f);

        // Simplex of 3 vertices
        SimplexCollisionShape simplex3 = new SimplexCollisionShape(
                new Vector3f(0f, 1f, 1f),
                new Vector3f(1f, 1f, 0f),
                new Vector3f(1f, 0f, 1f)
        );
        testSimplexShape(simplex3);
        Assert.assertEquals(3, simplex3.countMeshVertices());
        Utils.assertEquals(0f, 1f, 1f, simplex3.copyVertex(0, null), 0f);
        Utils.assertEquals(1f, 1f, 0f, simplex3.copyVertex(1, null), 0f);
        Utils.assertEquals(1f, 0f, 1f, simplex3.copyVertex(2, null), 0f);
        Utils.assertEquals(1f, 1f, 1f, simplex3.getHalfExtents(null), 0f);
        Assert.assertEquals(0f, simplex3.unscaledVolume(), 0f);

        // Simplex of 4 vertices
        SimplexCollisionShape simplex4 = new SimplexCollisionShape(
                new Vector3f(0f, 1f, 1f),
                new Vector3f(0f, 1f, -1f),
                new Vector3f(1f, -1f, 0f),
                new Vector3f(-1f, -1f, 0f)
        );
        testSimplexShape(simplex4);
        Assert.assertEquals(4, simplex4.countMeshVertices());
        Utils.assertEquals(0f, 1f, 1f, simplex4.copyVertex(0, null), 0f);
        Utils.assertEquals(0f, 1f, -1f, simplex4.copyVertex(1, null), 0f);
        Utils.assertEquals(1f, -1f, 0f, simplex4.copyVertex(2, null), 0f);
        Utils.assertEquals(-1f, -1f, 0f, simplex4.copyVertex(3, null), 0f);
        Utils.assertEquals(1f, 1f, 1f, simplex4.getHalfExtents(null), 0f);
        Assert.assertEquals(4f / 3f, simplex4.unscaledVolume(), 1e-6f);

        // Sphere
        SphereCollisionShape sphere = new SphereCollisionShape(1f);
        testShape(sphere);
        Assert.assertEquals(0f, sphere.getMargin(), 0f);
        Assert.assertFalse(sphere.isConcave());
        Assert.assertTrue(sphere.isConvex());
        Assert.assertFalse(sphere.isInfinite());
        Assert.assertFalse(sphere.isNonMoving());
        Assert.assertFalse(sphere.isPolyhedral());
        Assert.assertEquals(
                4f * FastMath.PI / 3f, sphere.unscaledVolume(), 1e-4f);
    }

    /**
     * Test the defaults that are common to all newly-created simplex shapes.
     *
     * @param simplex the shape to test (not null, unaffected)
     */
    private static void testSimplexShape(SimplexCollisionShape simplex) {
        testShape(simplex);
        Assert.assertEquals(0.04f, simplex.getMargin(), 0f);

        Assert.assertFalse(simplex.isConcave());
        Assert.assertTrue(simplex.isConvex());
        Assert.assertFalse(simplex.isInfinite());
        Assert.assertFalse(simplex.isNonMoving());
        Assert.assertTrue(simplex.isPolyhedral());
    }

    private static void testSixDof(SixDofJoint six, int numEnds) {
        Assert.assertEquals(numEnds, six.countEnds());
        testConstraint(six);

        RotationalLimitMotor rlm
                = six.getRotationalLimitMotor(PhysicsSpace.AXIS_X);
        Assert.assertEquals(0f, rlm.getAccumulatedImpulse(), 0f);
        Assert.assertEquals(1f, rlm.getDamping(), 0f);
        Assert.assertFalse(rlm.isEnableMotor());
        Assert.assertEquals(0.2f, rlm.getERP(), 0f);
        Assert.assertEquals(0.5f, rlm.getLimitSoftness(), 0f);
        Assert.assertEquals(1f, rlm.getLowerLimit(), 0f);
        Assert.assertEquals(300f, rlm.getMaxLimitForce(), 0f);
        Assert.assertEquals(6f, rlm.getMaxMotorForce(), 0f);
        Assert.assertEquals(0f, rlm.getNormalCFM(), 0f);
        Assert.assertEquals(0f, rlm.getRestitution(), 0f);
        Assert.assertEquals(0f, rlm.getStopCFM(), 0f);
        Assert.assertEquals(0f, rlm.getTargetVelocity(), 0f);
        Assert.assertEquals(-1f, rlm.getUpperLimit(), 0f);

        TranslationalLimitMotor tlm = six.getTranslationalLimitMotor();
        Utils.assertEquals(0f, 0f, 0f, tlm.getAccumulatedImpulse(null), 0f);
        Assert.assertEquals(1f, tlm.getDamping(), 0f);
        Assert.assertFalse(tlm.isEnabled(0));
        Assert.assertFalse(tlm.isEnabled(1));
        Assert.assertFalse(tlm.isEnabled(2));
        Utils.assertEquals(0.2f, 0.2f, 0.2f, tlm.getERP(null), 0f);
        Assert.assertEquals(0.7f, tlm.getLimitSoftness(), 0f);
        Utils.assertEquals(0f, 0f, 0f, tlm.getLowerLimit(null), 0f);
        Utils.assertEquals(0f, 0f, 0f, tlm.getMaxMotorForce(null), 0f);
        Utils.assertEquals(0f, 0f, 0f, tlm.getNormalCFM(null), 0f);
        Assert.assertEquals(0f, tlm.getRestitution(), 0.5f);
        Utils.assertEquals(0f, 0f, 0f, tlm.getStopCFM(null), 0f);
        Utils.assertEquals(0f, 0f, 0f, tlm.getTargetVelocity(null), 0f);
        Utils.assertEquals(0f, 0f, 0f, tlm.getUpperLimit(null), 0f);

        if (six instanceof SixDofSpringJoint) {
            SixDofSpringJoint spring = (SixDofSpringJoint) six;
            for (int dofIndex = 0; dofIndex < 6; ++dofIndex) {
                Assert.assertEquals(1f, spring.getDamping(dofIndex), 0f);
                Assert.assertEquals(
                        0f, spring.getEquilibriumPoint(dofIndex), 0f);
                Assert.assertEquals(0f, spring.getStiffness(dofIndex), 0f);
                Assert.assertFalse(spring.isSpringEnabled(dofIndex));
            }
        }
    }

    private static void testSlider(SliderJoint slider, int numEnds) {
        Assert.assertEquals(numEnds, slider.countEnds());
        testConstraint(slider);

        Assert.assertEquals(0f, slider.getDampingDirAng(), 0f);
        Assert.assertEquals(0f, slider.getDampingDirLin(), 0f);
        Assert.assertEquals(1f, slider.getDampingLimAng(), 0f);
        Assert.assertEquals(1f, slider.getDampingLimLin(), 0f);
        Assert.assertEquals(1f, slider.getDampingOrthoAng(), 0f);
        Assert.assertEquals(1f, slider.getDampingOrthoLin(), 0f);
        Assert.assertEquals(0f, slider.getLowerAngLimit(), 0f);
        Assert.assertEquals(1f, slider.getLowerLinLimit(), 0f);
        Assert.assertEquals(0f, slider.getMaxAngMotorForce(), 0f);
        Assert.assertEquals(0f, slider.getMaxLinMotorForce(), 0f);
        Assert.assertEquals(0.7f, slider.getRestitutionDirAng(), 0f);
        Assert.assertEquals(0.7f, slider.getRestitutionDirLin(), 0f);
        Assert.assertEquals(0.7f, slider.getRestitutionLimAng(), 0f);
        Assert.assertEquals(0.7f, slider.getRestitutionLimLin(), 0f);
        Assert.assertEquals(0.7f, slider.getRestitutionOrthoAng(), 0f);
        Assert.assertEquals(0.7f, slider.getRestitutionOrthoLin(), 0f);
        Assert.assertEquals(1f, slider.getSoftnessDirAng(), 0f);
        Assert.assertEquals(1f, slider.getSoftnessDirLin(), 0f);
        Assert.assertEquals(1f, slider.getSoftnessLimAng(), 0f);
        Assert.assertEquals(1f, slider.getSoftnessLimLin(), 0f);
        Assert.assertEquals(1f, slider.getSoftnessOrthoAng(), 0f);
        Assert.assertEquals(1f, slider.getSoftnessOrthoLin(), 0f);
        Assert.assertEquals(0f, slider.getTargetAngMotorVelocity(), 0f);
        Assert.assertEquals(0f, slider.getTargetLinMotorVelocity(), 0f);
        Assert.assertEquals(0f, slider.getUpperAngLimit(), 0f);
        Assert.assertEquals(-1f, slider.getUpperLinLimit(), 0f);
        Assert.assertFalse(slider.isPoweredAngMotor());
        Assert.assertFalse(slider.isPoweredLinMotor());
    }
}
