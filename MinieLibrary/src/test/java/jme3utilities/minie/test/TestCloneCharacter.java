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
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.objects.PhysicsCharacter;
import com.jme3.export.binary.BinaryExporter;
import com.jme3.math.Vector3f;
import com.jme3.system.NativeLibraryLoader;
import jme3utilities.Heart;
import org.junit.Test;

/**
 * Test cloning/saving/loading a PhysicsCharacter. TODO replace asserts with
 * JUnit Assert
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestCloneCharacter {
    // *************************************************************************
    // fields

    /**
     * AssetManager required by the BinaryImporter
     */
    final private static AssetManager assetManager = new DesktopAssetManager();
    // *************************************************************************
    // new methods exposed

    /**
     * Test cloning/saving/loading of a PhysicsCharacter.
     */
    @Test
    public void testCloneCharacter() {
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);
        SphereCollisionShape shape = new SphereCollisionShape(1f);
        PhysicsCharacter ch = new PhysicsCharacter(shape, 1f);
        setParameters(ch, 0f);
        verifyParameters(ch, 0f);

        PhysicsCharacter chClone = Heart.deepCopy(ch);
        cloneTest(ch, chClone);
    }
    // *************************************************************************
    // private methods

    private static void cloneTest(
            PhysicsCharacter ch, PhysicsCharacter chClone) {
        assert chClone.nativeId() != ch.nativeId();
        assert chClone.getControllerId() != ch.getControllerId();

        verifyParameters(ch, 0f);
        verifyParameters(chClone, 0f);

        setParameters(ch, 0.3f);
        verifyParameters(ch, 0.3f);
        verifyParameters(chClone, 0f);

        setParameters(chClone, 0.6f);
        verifyParameters(ch, 0.3f);
        verifyParameters(chClone, 0.6f);

        PhysicsCharacter chCopy = BinaryExporter.saveAndLoad(assetManager, ch);
        verifyParameters(chCopy, 0.3f);

        PhysicsCharacter chCloneCopy
                = BinaryExporter.saveAndLoad(assetManager, chClone);
        verifyParameters(chCloneCopy, 0.6f);

        PhysicsCharacter xmlCopy = Utils.saveAndLoadXml(assetManager, ch);
        verifyParameters(xmlCopy, 0.3f);
    }

    /**
     * Modify PhysicsCharacter parameters based on the specified key value.
     *
     * @param ch the character to modify (not null)
     * @param b the key value
     */
    private static void setParameters(PhysicsCharacter ch, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);
        int index = Math.round(b / 0.3f);
        Vector3f gravity = new Vector3f(b - 0.2f, b + 0.8f, b - 0.3f);

        // "up" direction must oppose gravity vector
        Vector3f up = gravity.normalize().negateLocal();

        // walk offset must be perpendicular to "up" direction
        Vector3f walkOffset
                = new Vector3f(b + 0.6f, b + 0.2f, b + 0.4f).cross(up);

        ch.setAngularDamping(b + 0.01f);
        ch.setAngularVelocity(new Vector3f(b + 0.04f, b + 0.05f, b + 0.06f));
        ch.setAnisotropicFriction(
                new Vector3f(b + 0.004f, b + 0.005f, b + 0.006f), index);
        ch.setCcdMotionThreshold(b + 0.07f);
        ch.setCcdSweptSphereRadius(b + 0.08f);
        ch.setContactDamping(b + 0.084f);
        ch.setContactProcessingThreshold(b + 0.0845f);
        ch.setContactResponse(flag);
        ch.setContactStiffness(b + 0.085f);
        ch.setDeactivationTime(b + 0.087f);
        ch.setFallSpeed(b + 0.09f);
        ch.setFriction(b + 0.095f);
        ch.setGravity(gravity);
        ch.setJumpSpeed(b + 0.125f);
        ch.setLinearDamping(b + 0.13f);
        ch.setMaxPenetrationDepth(b + 0.281f);
        ch.setMaxSlope(b + 0.282f);
        ch.setPhysicsLocation(new Vector3f(b + 0.18f, b + 0.19f, b + 0.20f));
        ch.setRestitution(b + 0.25f);
        ch.setRollingFriction(b + 0.26f);
        ch.setSpinningFriction(b + 0.27f);
        ch.setStepHeight(b + 0.29f);
        ch.setSweepTest(!flag);
        ch.setUp(up);
        ch.setWalkDirection(walkOffset);
    }

    /**
     * Verify that all PhysicsCharacter parameters have their expected values
     * for the specified key value.
     *
     * @param ch the character to verify (not null, unaffected)
     * @param b the key value
     */
    private static void verifyParameters(PhysicsCharacter ch, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);
        int index = Math.round(b / 0.3f);
        Vector3f gravity = new Vector3f(b - 0.2f, b + 0.8f, b - 0.3f);

        // "up" direction must oppose gravity vector
        Vector3f up = gravity.normalize().negateLocal();

        // walk offset must be perpendicular to "up" direction
        Vector3f walkOffset
                = new Vector3f(b + 0.6f, b + 0.2f, b + 0.4f).cross(up);

        assert ch.getAngularDamping() == b + 0.01f;
        Utils.assertEquals(b + 0.04f, b + 0.05f, b + 0.06f,
                ch.getAngularVelocity(null), 0f);

        if (index == 0) {
            assert !ch.hasAnisotropicFriction(AfMode.either);
        } else {
            assert ch.hasAnisotropicFriction(index);
            Utils.assertEquals(b + 0.004f, b + 0.005f, b + 0.006f,
                    ch.getAnisotropicFriction(null), 0f);
        }

        assert ch.getCcdMotionThreshold() == b + 0.07f;
        assert ch.getCcdSweptSphereRadius() == b + 0.08f;
        assert ch.getContactDamping() == b + 0.084f;
        assert ch.getContactProcessingThreshold() == b + 0.0845f;
        assert ch.isContactResponse() == flag;
        assert ch.getContactStiffness() == b + 0.085f;
        assert ch.getDeactivationTime() == b + 0.087f;
        assert ch.getFallSpeed() == b + 0.09f;
        assert ch.getFriction() == b + 0.095f;
        Utils.assertEquals(gravity, ch.getGravity(null), 1e-5f);
        assert ch.getJumpSpeed() == b + 0.125f;
        assert ch.getLinearDamping() == b + 0.13f;
        assert ch.getMaxPenetrationDepth() == b + 0.281f;
        assert ch.getMaxSlope() == b + 0.282f;
        Utils.assertEquals(b + 0.18f, b + 0.19f, b + 0.20f,
                ch.getPhysicsLocation(null), 0f);
        assert ch.getRestitution() == b + 0.25f;
        assert ch.getRollingFriction() == b + 0.26f;
        assert ch.getSpinningFriction() == b + 0.27f;
        assert ch.getStepHeight() == b + 0.29f;
        assert ch.isUsingGhostSweepTest() == !flag;
        Utils.assertEquals(up, ch.getUpDirection(null), 1e-5f);
        Utils.assertEquals(walkOffset, ch.getWalkDirection(null), 1e-5f);
    }
}
