/*
 Copyright (c) 2018-2022, Stephen Gold
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
import com.jme3.bullet.collision.shapes.CollisionShape;
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
import org.junit.Test;

/**
 * Test cloning/saving/loading on PhysicsGhostObject and its subclass. TODO
 * replace asserts with JUnit Assert
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
    }
    // *************************************************************************
    // private methods

    private static void cloneTest(
            PhysicsGhostObject pgo, PhysicsGhostObject pgoClone) {
        assert pgoClone.nativeId() != pgo.nativeId();

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
     * Verify that all PhysicsGhostObject parameters have their expected values
     * for the specified key value.
     *
     * @param pgo the ghost object to verify (not null, unaffected)
     * @param b the key value
     */
    private static void verifyParameters(PhysicsGhostObject pgo, float b) {
        int index = Math.round(b / 0.3f);
        if (index == 0) {
            assert !pgo.hasAnisotropicFriction(AfMode.either);
        } else {
            assert pgo.hasAnisotropicFriction(index);
            Vector3f c = pgo.getAnisotropicFriction(null);
            assert c.x == b + 0.004f : c;
            assert c.y == b + 0.005f : c;
            assert c.z == b + 0.006f : c;
        }

        assert pgo.getCcdMotionThreshold() == b + 0.07f;
        assert pgo.getCcdSweptSphereRadius() == b + 0.08f;
        assert pgo.getContactDamping() == b + 0.084f;
        assert pgo.getContactProcessingThreshold() == b + 0.0845f;
        assert pgo.getContactStiffness() == b + 0.085f;
        assert pgo.getDeactivationTime() == b + 0.087f;
        assert pgo.getFriction() == b + 0.09f;

        Vector3f x = pgo.getPhysicsLocation(null);
        assert x.x == b + 0.18f : x;
        assert x.y == b + 0.19f : x;
        assert x.z == b + 0.20f : x;
        assert pgo.getRestitution() == b + 0.205f;

        Quaternion orient
                = new Quaternion(b + 0.21f, b + 0.22f, b + 0.23f, b + 0.24f);
        MyQuaternion.normalizeLocal(orient);
        Matrix3f matrix = orient.toRotationMatrix();
        Matrix3f m = pgo.getPhysicsRotationMatrix(null);
        assert m.equals(matrix);

        assert pgo.getRollingFriction() == b + 0.25f;
        assert pgo.getSpinningFriction() == b + 0.26f;
    }
}
