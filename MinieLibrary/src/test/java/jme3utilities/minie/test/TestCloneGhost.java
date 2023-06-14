/*
 Copyright (c) 2018-2023, Stephen Gold
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
import com.jme3.asset.DesktopAssetManager;
import com.jme3.bullet.collision.AfMode;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.MultiSphere;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.control.GhostControl;
import com.jme3.bullet.objects.PhysicsGhostObject;
import com.jme3.export.binary.BinaryExporter;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.system.NativeLibraryLoader;
import jme3utilities.Heart;
import jme3utilities.math.MyQuaternion;
import org.junit.Assert;
import org.junit.Test;

/**
 * Test cloning/saving/loading on PhysicsGhostObject and its subclass.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestCloneGhost {
    // *************************************************************************
    // fields

    /**
     * AssetManager required by the BinaryImporter
     */
    final private static AssetManager assetManager = new DesktopAssetManager();
    // *************************************************************************
    // new methods exposed

    /**
     * Test cloning/saving/loading on PhysicsGhostObject and its subclasses.
     */
    @Test
    public void testCloneGhost() {
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);
        CollisionShape shape = new SphereCollisionShape(1f);

        // PhysicsGhostObject
        PhysicsGhostObject pgo = new PhysicsGhostObject(shape);
        setParameters(pgo, 0f);
        verifyParameters(pgo, 0f);
        PhysicsGhostObject pgoClone = Heart.deepCopy(pgo);
        cloneTest(pgo, pgoClone);

        // GhostControl (a subclass of PhysicsGhostObject)
        GhostControl gc = new GhostControl(shape);
        setParameters(gc, 0f);
        verifyParameters(gc, 0f);
        GhostControl gcClone = Heart.deepCopy(gc);
        cloneTest(gc, gcClone);

        testCloneGhostPair();
    }
    // *************************************************************************
    // private methods

    private static void cloneTest(
            PhysicsGhostObject pgo, PhysicsGhostObject pgoClone) {
        Utils.cloneTest(pgo, pgoClone);

        verifyParameters(pgo, 0f);
        verifyParameters(pgoClone, 0f);

        setParameters(pgo, 0.3f);
        verifyParameters(pgo, 0.3f);
        verifyParameters(pgoClone, 0f);

        setParameters(pgoClone, 0.6f);
        verifyParameters(pgo, 0.3f);
        verifyParameters(pgoClone, 0.6f);

        if (pgo instanceof GhostControl) {
            GhostControl pgoCopy = BinaryExporter
                    .saveAndLoad(assetManager, (GhostControl) pgo);
            verifyParameters(pgoCopy, 0.3f);

            GhostControl gcCloneCopy = BinaryExporter
                    .saveAndLoad(assetManager, (GhostControl) pgoClone);
            verifyParameters(gcCloneCopy, 0.6f);
        }
    }

    /**
     * Modify PhysicsGhostObject parameters based on the specified key value.
     *
     * @param pgo the ghost object to modify (not null)
     * @param b the key value
     */
    private static void setParameters(PhysicsGhostObject pgo, float b) {
        int afMode = Math.round(b / 0.3f);
        pgo.setAnisotropicFriction(
                new Vector3f(b + 0.004f, b + 0.005f, b + 0.006f), afMode);

        pgo.setCcdMotionThreshold(b + 0.07f);
        pgo.setCcdSweptSphereRadius(b + 0.08f);
        pgo.setContactDamping(b + 0.084f);
        pgo.setContactProcessingThreshold(b + 0.0845f);
        pgo.setContactStiffness(b + 0.085f);
        pgo.setDeactivationTime(b + 0.087f);
        pgo.setFriction(b + 0.09f);
        pgo.setPhysicsLocation(new Vector3f(b + 0.18f, b + 0.19f, b + 0.20f));
        pgo.setRestitution(b + 0.205f);

        Quaternion orient
                = new Quaternion(b + 0.21f, b + 0.22f, b + 0.23f, b + 0.24f);
        MyQuaternion.normalizeLocal(orient);
        Matrix3f matrix = orient.toRotationMatrix();
        pgo.setPhysicsRotation(matrix);

        pgo.setRollingFriction(b + 0.25f);
        pgo.setSpinningFriction(b + 0.26f);
    }

    /**
     * Clone connected pairs of ghost objects.
     */
    private static void testCloneGhostPair() {
        CollisionShape shape = new MultiSphere(0.2f);

        // 2 PGOs that ignore each another
        PhysicsGhostObject pgo1 = new PhysicsGhostObject(shape);
        setParameters(pgo1, 0.3f);
        PhysicsGhostObject pgo2 = new PhysicsGhostObject(shape);
        setParameters(pgo2, 0.6f);
        pgo1.addToIgnoreList(pgo2);

        PhysicsGhostObject pgo1Clone = Heart.deepCopy(pgo1);
        Utils.cloneTest(pgo1, pgo1Clone);
        Assert.assertEquals(1, pgo1Clone.countIgnored());
        verifyParameters(pgo1Clone, 0.3f);

        PhysicsCollisionObject[] ignoresClone = pgo1Clone.listIgnoredPcos();
        PhysicsGhostObject pgo2Clone = (PhysicsGhostObject) ignoresClone[0];
        Utils.cloneTest(pgo2, pgo2Clone);
        Assert.assertEquals(1, pgo2Clone.countIgnored());
        verifyParameters(pgo2Clone, 0.6f);
    }

    /**
     * Verify that all PhysicsGhostObject parameters have their expected values
     * for the specified key value.
     *
     * @param pgo the ghost object to verify (not null, unaffected)
     * @param b the key value
     */
    private static void verifyParameters(PhysicsGhostObject pgo, float b) {
        int index = Math.round(b / 0.3f);
        if (index == 0) {
            Assert.assertFalse(pgo.hasAnisotropicFriction(AfMode.either));
        } else {
            Assert.assertTrue(pgo.hasAnisotropicFriction(index));
            Vector3f c = pgo.getAnisotropicFriction(null);
            Utils.assertEquals(b + 0.004f, b + 0.005f, b + 0.006f, c, 0f);
        }

        Assert.assertEquals(b + 0.07f, pgo.getCcdMotionThreshold(), 0f);
        Assert.assertEquals(b + 0.08f, pgo.getCcdSweptSphereRadius(), 0f);
        Assert.assertEquals(b + 0.084f, pgo.getContactDamping(), 0f);
        Assert.assertEquals(
                b + 0.0845f, pgo.getContactProcessingThreshold(), 0f);
        Assert.assertEquals(b + 0.085f, pgo.getContactStiffness(), 0f);
        Assert.assertEquals(b + 0.087f, pgo.getDeactivationTime(), 0f);
        Assert.assertEquals(b + 0.09f, pgo.getFriction(), 0f);

        Vector3f x = pgo.getPhysicsLocation(null);
        Utils.assertEquals(b + 0.18f, b + 0.19f, b + 0.20f, x, 0f);
        Assert.assertEquals(b + 0.205f, pgo.getRestitution(), 0f);

        Quaternion orient
                = new Quaternion(b + 0.21f, b + 0.22f, b + 0.23f, b + 0.24f);
        MyQuaternion.normalizeLocal(orient);
        Matrix3f matrix = orient.toRotationMatrix();
        Matrix3f m = pgo.getPhysicsRotationMatrix(null);
        Assert.assertEquals(m, matrix);

        Assert.assertEquals(b + 0.25f, pgo.getRollingFriction(), 0f);
        Assert.assertEquals(b + 0.26f, pgo.getSpinningFriction(), 0f);
    }
}
