/*
 Copyright (c) 2019, Stephen Gold
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

import com.jme3.bullet.PhysicsSoftSpace;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.RayTestFlag;
import com.jme3.bullet.SoftBodyWorldInfo;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.ConeCollisionShape;
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
import com.jme3.bullet.collision.shapes.infos.DebugMeshNormals;
import com.jme3.bullet.joints.SixDofJoint;
import com.jme3.bullet.joints.SliderJoint;
import com.jme3.bullet.joints.SoftAngularJoint;
import com.jme3.bullet.joints.motors.RotationalLimitMotor;
import com.jme3.bullet.joints.motors.TranslationalLimitMotor;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.bullet.util.NativeLibrary;
import com.jme3.bullet.util.NativeSoftBodyUtil;
import com.jme3.math.FastMath;
import com.jme3.math.Plane;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.debug.WireBox;
import com.jme3.scene.shape.Quad;
import com.jme3.scene.shape.Torus;
import com.jme3.system.NativeLibraryLoader;
import java.util.ArrayList;
import java.util.List;
import org.junit.Assert;
import org.junit.Test;

/**
 * Verify the defaults for physics objects.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestDefaults {
    // *************************************************************************
    // fields

    private CollisionShape box; // initialized by testShapes()
    // *************************************************************************
    // new methods exposed

    /**
     * Verify the defaults for physics objects.
     */
    @Test
    public void testDefaults() {
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);

        PhysicsSoftSpace space = new PhysicsSoftSpace(
                new Vector3f(-10000f, -10000f, -10000f),
                new Vector3f(10000f, 10000f, 10000f),
                PhysicsSpace.BroadphaseType.DBVT);
        Assert.assertEquals(0, space.countJoints());
        Assert.assertEquals(0, space.countRigidBodies());
        Assert.assertEquals(0, space.countSoftBodies());
        Assert.assertEquals(1 / 60f, space.getAccuracy(), 0f);
        assertEquals(0f, -9.81f, 0f, space.getGravity(null), 0f);
        Assert.assertEquals(4, space.maxSubSteps());
        Assert.assertEquals(0.1f, space.maxTimeStep(), 0f);
        Assert.assertEquals(RayTestFlag.SubSimplexRaytest,
                space.getRayTestFlags());
        Assert.assertEquals(10, space.getSolverNumIterations());

        SoftBodyWorldInfo info = new SoftBodyWorldInfo();
        Assert.assertEquals(1.2f, info.airDensity(), 0f);
        assertEquals(0f, -10f, 0f, info.copyGravity(null), 0f);
        Assert.assertEquals(1000f, info.maxDisplacement(), 0f);
        Assert.assertEquals(0f, info.waterDensity(), 0f);
        assertEquals(0f, 0f, 0f, info.copyWaterNormal(null), 0f);
        Assert.assertEquals(0f, info.waterOffset(), 0f);

        testShapes();

        PhysicsRigidBody rigidA = new PhysicsRigidBody(box);
        testPco(rigidA);
        Assert.assertFalse(rigidA.isStatic());
        Assert.assertEquals(0, rigidA.countJoints());
        Assert.assertEquals(0f, rigidA.getAngularDamping(), 0f);
        Assert.assertEquals(1f, rigidA.getAngularFactor(), 0f);
        Assert.assertEquals(1f, rigidA.getAngularSleepingThreshold(), 0f);
        assertEquals(0f, 0f, 0f, rigidA.getAngularVelocity(null), 0);
        assertEquals(0f, 0f, 0f, rigidA.getLinearVelocity(null), 0);
        Assert.assertFalse(rigidA.isKinematic());
        Assert.assertEquals(1f, rigidA.getMass(), 0f);
        Assert.assertEquals(1, rigidA.getActivationState());

        PhysicsRigidBody rigidB = new PhysicsRigidBody(box);
        SixDofJoint six = new SixDofJoint(rigidA, rigidB, new Vector3f(),
                new Vector3f(), false);
        Assert.assertFalse(six.isFeedback());
        six.setFeedback(true);

        Assert.assertEquals(2, six.countEnds());
        Assert.assertEquals(0f, six.getAppliedImpulse(), 0f);

        float simdInf = NativeLibrary.isDoublePrecision()
                ? Float.POSITIVE_INFINITY : Float.MAX_VALUE;
        Assert.assertEquals(simdInf, six.getBreakingImpulseThreshold(), 0f);
        Assert.assertTrue(six.isCollisionBetweenLinkedBodies());
        Assert.assertTrue(six.isEnabled());

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
        assertEquals(0f, 0f, 0f, tlm.getAccumulatedImpulse(null), 0f);
        Assert.assertEquals(1f, tlm.getDamping(), 0f);
        Assert.assertFalse(tlm.isEnabled(0));
        Assert.assertFalse(tlm.isEnabled(1));
        Assert.assertFalse(tlm.isEnabled(2));
        assertEquals(0.2f, 0.2f, 0.2f, tlm.getERP(null), 0f);
        Assert.assertEquals(0.7f, tlm.getLimitSoftness(), 0f);
        assertEquals(0f, 0f, 0f, tlm.getLowerLimit(null), 0f);
        assertEquals(0f, 0f, 0f, tlm.getMaxMotorForce(null), 0f);
        assertEquals(0f, 0f, 0f, tlm.getNormalCFM(null), 0f);
        Assert.assertEquals(0f, tlm.getRestitution(), 0.5f);
        assertEquals(0f, 0f, 0f, tlm.getStopCFM(null), 0f);
        assertEquals(0f, 0f, 0f, tlm.getTargetVelocity(null), 0f);
        assertEquals(0f, 0f, 0f, tlm.getUpperLimit(null), 0f);

        PhysicsSoftBody softA = new PhysicsSoftBody();
        testPco(softA);
        Assert.assertEquals(0, softA.countClusters());
        Assert.assertEquals(0, softA.countFaces());
        Assert.assertEquals(0, softA.countJoints());
        Assert.assertEquals(0, softA.countLinks());
        Assert.assertEquals(0, softA.countNodes());
        Assert.assertEquals(0, softA.countTetras());
        Assert.assertEquals(0f, softA.getMass(), 0f);
        Assert.assertEquals(1, softA.getActivationState());

        Mesh wireBox = new WireBox();
        NativeSoftBodyUtil.appendFromLineMesh(wireBox, softA);
        softA.generateClusters(2, 999);
        softA.setMass(1f);

        SliderJoint slider = new SliderJoint(rigidA, rigidB,
                new Vector3f(0f, 0f, 0f), new Vector3f(0f, 0f, 0f), false);
        Assert.assertEquals(2, slider.countEnds());
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
        Assert.assertTrue(slider.isEnabled());
        Assert.assertFalse(slider.isPoweredAngMotor());
        Assert.assertFalse(slider.isPoweredLinMotor());

        SoftAngularJoint sraj = new SoftAngularJoint(new Vector3f(0f, 0f, 0f),
                softA, 0, rigidB);
        Assert.assertEquals(2, sraj.countEnds());
        Assert.assertEquals(1f, sraj.getCFM(), 0f);
        Assert.assertTrue(sraj.isEnabled());
        Assert.assertEquals(1f, sraj.getERP(), 0f);
        Assert.assertEquals(1f, sraj.getSplit(), 0f);
    }

    void assertEquals(float x, float y, float z, Vector3f vector,
            float tolerance) {
        Assert.assertEquals(x, vector.x, tolerance);
        Assert.assertEquals(y, vector.y, tolerance);
        Assert.assertEquals(z, vector.z, tolerance);
    }
    // *************************************************************************
    // private methods

    /**
     * Test the defaults that are common to all newly-created collision objects.
     *
     * @param pco the object to test (not null, unaffected)
     */
    private void testPco(PhysicsCollisionObject pco) {
        Assert.assertFalse(pco.isInWorld());
        Assert.assertTrue(pco.isActive());
        assertEquals(1f, 1f, 1f, pco.getAnisotropicFriction(null), 0f);
        Assert.assertFalse(pco.hasAnisotropicFriction(1));
        Assert.assertFalse(pco.hasAnisotropicFriction(2));
        Assert.assertEquals(0f, pco.getCcdMotionThreshold(), 0f);
        Assert.assertEquals(0f, pco.getCcdSweptSphereRadius(), 0f);
        Assert.assertEquals(PhysicsCollisionObject.COLLISION_GROUP_01,
                pco.getCollideWithGroups());
        Assert.assertEquals(PhysicsCollisionObject.COLLISION_GROUP_01,
                pco.getCollisionGroup());
        Assert.assertEquals(0.1f, pco.getContactDamping(), 0f);

        float largeFloat = NativeLibrary.isDoublePrecision() ? 1e30f : 1e18f;
        Assert.assertEquals(largeFloat, pco.getContactProcessingThreshold(), 0f);
        Assert.assertEquals(largeFloat, pco.getContactStiffness(), 0f);

        Assert.assertTrue(pco.isContactResponse());
        Assert.assertEquals(0f, pco.getDeactivationTime(), 0f);
        Assert.assertNull(pco.getDebugMaterial());
        Assert.assertNull(pco.debugMeshInitListener());
        Assert.assertSame(DebugMeshNormals.None, pco.debugMeshNormals());
        Assert.assertEquals(DebugShapeFactory.lowResolution,
                pco.debugMeshResolution());
        Assert.assertEquals(1, pco.debugNumSides());
        Assert.assertEquals(0.5f, pco.getFriction(), 0f);
        Assert.assertEquals(0f, pco.getRestitution(), 0f);
        Assert.assertEquals(0f, pco.getRollingFriction(), 0f);
        Assert.assertEquals(0f, pco.getSpinningFriction(), 0f);
        Assert.assertNull(pco.getUserObject());
    }

    /**
     * Test the defaults that are common to all newly-created collision shapes.
     *
     * @param shape the shape to test (not null, unaffected)
     */
    private void testShape(CollisionShape shape) {
        assertEquals(1f, 1f, 1f, shape.getScale(null), 0f);
    }

    /**
     * Verify the defaults for all collision shapes.
     */
    private void testShapes() {
        /*
         * Box
         */
        box = new BoxCollisionShape(1f);
        testShape(box);
        Assert.assertEquals(0.04f, box.getMargin(), 0f);
        Assert.assertFalse(box.isConcave());
        Assert.assertTrue(box.isConvex());
        Assert.assertFalse(box.isInfinite());
        Assert.assertFalse(box.isNonMoving());
        Assert.assertTrue(box.isPolyhedral());
        /*
         * Capsule
         */
        CapsuleCollisionShape capsule = new CapsuleCollisionShape(1f, 1f);
        testShape(capsule);
        Assert.assertEquals(PhysicsSpace.AXIS_Y, capsule.getAxis());
        Assert.assertEquals(0f, capsule.getMargin(), 0f);
        Assert.assertFalse(capsule.isConcave());
        Assert.assertTrue(capsule.isConvex());
        Assert.assertFalse(capsule.isInfinite());
        Assert.assertFalse(capsule.isNonMoving());
        Assert.assertFalse(capsule.isPolyhedral());
        /*
         * Compound
         */
        CompoundCollisionShape compound = new CompoundCollisionShape();
        testShape(compound);
        Assert.assertEquals(0, compound.countChildren());
        Assert.assertEquals(0.04f, compound.getMargin(), 0f);
        Assert.assertFalse(compound.isConcave());
        Assert.assertFalse(compound.isConvex());
        Assert.assertFalse(compound.isInfinite());
        Assert.assertFalse(compound.isNonMoving());
        Assert.assertFalse(compound.isPolyhedral());
        /*
         * Cone
         */
        ConeCollisionShape cone = new ConeCollisionShape(1f, 1f);
        testShape(cone);
        Assert.assertEquals(PhysicsSpace.AXIS_Y, cone.getAxis());
        Assert.assertEquals(0.04f, cone.getMargin(), 0f);
        Assert.assertFalse(cone.isConcave());
        Assert.assertTrue(cone.isConvex());
        Assert.assertFalse(cone.isInfinite());
        Assert.assertFalse(cone.isNonMoving());
        Assert.assertFalse(cone.isPolyhedral());
        /*
         * Cylinder
         */
        CylinderCollisionShape cylinder
                = new CylinderCollisionShape(new Vector3f(1f, 1f, 1f));
        testShape(cylinder);
        Assert.assertEquals(PhysicsSpace.AXIS_Z, cylinder.getAxis());
        Assert.assertEquals(0.04f, cylinder.getMargin(), 0f);
        Assert.assertFalse(cylinder.isConcave());
        Assert.assertTrue(cylinder.isConvex());
        Assert.assertFalse(cylinder.isInfinite());
        Assert.assertFalse(cylinder.isNonMoving());
        Assert.assertFalse(cylinder.isPolyhedral());
        /*
         * Empty
         */
        EmptyShape empty = new EmptyShape(true);
        testShape(empty);
        Assert.assertEquals(0.04f, empty.getMargin(), 0f);
        Assert.assertTrue(empty.isConcave());
        Assert.assertFalse(empty.isConvex());
        Assert.assertFalse(empty.isInfinite());
        Assert.assertTrue(empty.isNonMoving());
        Assert.assertFalse(empty.isPolyhedral());
        /*
         * GImpact
         */
        Torus torus = new Torus(16, 16, 0.2f, 0.8f);
        GImpactCollisionShape gimpact = new GImpactCollisionShape(torus);
        testShape(gimpact);
        Assert.assertEquals(0.04f, empty.getMargin(), 0f);
        Assert.assertTrue(gimpact.isConcave());
        Assert.assertFalse(gimpact.isConvex());
        Assert.assertFalse(gimpact.isInfinite());
        Assert.assertFalse(gimpact.isNonMoving());
        Assert.assertFalse(gimpact.isPolyhedral());
        /*
         * Heightfield
         */
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
        /*
         * Hull
         */
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
        /*
         * Mesh
         */
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
        /*
         * MultiSphere
         */
        MultiSphere multiSphere = new MultiSphere(1f);
        testShape(multiSphere);
        assertEquals(0f, 0f, 0f, multiSphere.copyCenter(0, null), 0f);
        Assert.assertEquals(1, multiSphere.countSpheres());
        Assert.assertEquals(1f, multiSphere.getRadius(0), 0f);
        Assert.assertEquals(4f * FastMath.PI / 3f, multiSphere.scaledVolume(),
                0.001f);
        Assert.assertFalse(multiSphere.isConcave());
        Assert.assertTrue(multiSphere.isConvex());
        Assert.assertFalse(multiSphere.isInfinite());
        Assert.assertFalse(multiSphere.isNonMoving());
        Assert.assertFalse(multiSphere.isPolyhedral());
        /*
         * Plane
         */
        Plane plane = new Plane(new Vector3f(0f, 1f, 0f), 0f);
        PlaneCollisionShape pcs = new PlaneCollisionShape(plane);
        testShape(pcs);
        Assert.assertEquals(0.04f, pcs.getMargin(), 0f);
        Assert.assertEquals(0f, pcs.getPlane().getConstant(), 0f);
        assertEquals(0f, 1f, 0f, pcs.getPlane().getNormal(), 0f);
        Assert.assertTrue(pcs.isConcave());
        Assert.assertFalse(pcs.isConvex());
        Assert.assertTrue(pcs.isInfinite());
        Assert.assertTrue(pcs.isNonMoving());
        Assert.assertFalse(pcs.isPolyhedral());
        /*
         * Simplex
         */
        SimplexCollisionShape simplex
                = new SimplexCollisionShape(new Vector3f(0f, 0f, 0f));
        testShape(simplex);
        assertEquals(0f, 0f, 0f, simplex.copyVertex(0, null), 0f);
        Assert.assertEquals(1, simplex.countMeshVertices());
        assertEquals(0f, 0f, 0f, simplex.getHalfExtents(null), 0f);
        Assert.assertEquals(0.04f, simplex.getMargin(), 0f);
        Assert.assertFalse(simplex.isConcave());
        Assert.assertTrue(simplex.isConvex());
        Assert.assertFalse(simplex.isInfinite());
        Assert.assertFalse(simplex.isNonMoving());
        Assert.assertTrue(simplex.isPolyhedral());
        Assert.assertEquals(0f, simplex.unscaledVolume(), 0f);
        /*
         * Sphere
         */
        CollisionShape sphere = new SphereCollisionShape(1f);
        testShape(sphere);
        Assert.assertEquals(0f, sphere.getMargin(), 0f);
        Assert.assertFalse(sphere.isConcave());
        Assert.assertTrue(sphere.isConvex());
        Assert.assertFalse(sphere.isInfinite());
        Assert.assertFalse(sphere.isNonMoving());
        Assert.assertFalse(sphere.isPolyhedral());
    }
}
