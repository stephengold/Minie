/*
 Copyright (c) 2018-2019, Stephen Gold
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
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.objects.PhysicsCharacter;
import com.jme3.export.binary.BinaryExporter;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.system.NativeLibraryLoader;
import jme3utilities.Misc;
import org.junit.Test;

/**
 * Test cloning/saving/loading a PhysicsCharacter.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestCloneCharacter {
    // *************************************************************************
    // fields

    /**
     * AssetManager required by the BinaryImporter
     */
    final private AssetManager assetManager = new DesktopAssetManager();
    // *************************************************************************
    // new methods exposed

    /**
     * Test cloning/saving/loading on PhysicsCharacter.
     */
    @Test
    public void testCloneCharacter() {
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);
        CollisionShape shape = new SphereCollisionShape(1f);
        PhysicsCharacter ch = new PhysicsCharacter(shape, 1f);
        setParameters(ch, 0f);
        verifyParameters(ch, 0f);

        PhysicsCharacter chClone = (PhysicsCharacter) Misc.deepCopy(ch);
        cloneTest(ch, chClone);
    }
    // *************************************************************************
    // private methods

    private void cloneTest(PhysicsCharacter ch, PhysicsCharacter chClone) {
        assert chClone.getObjectId() != ch.getObjectId();
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
    }

    /**
     * Modify PhysicsCharacter parameters based on the specified key value.
     *
     * @param ch the character to modify (not null)
     * @param b the key value
     */
    private void setParameters(PhysicsCharacter ch, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);
        ch.setContactResponse(flag);
        ch.setSweepTest(!flag);

        int index = Math.round(b / 0.3f);
        ch.setAnisotropicFriction(
                new Vector3f(b + 0.004f, b + 0.005f, b + 0.006f), index);

        ch.setAngularDamping(b + 0.01f);
        ch.setAngularVelocity(new Vector3f(b + 0.04f, b + 0.05f, b + 0.06f));
        ch.setCcdMotionThreshold(b + 0.07f);
        ch.setCcdSweptSphereRadius(b + 0.08f);
        ch.setContactDamping(b + 0.084f);
        ch.setContactProcessingThreshold(b + 0.0845f);
        ch.setContactStiffness(b + 0.085f);
        ch.setDeactivationTime(b + 0.087f);
        ch.setFallSpeed(b + 0.09f);
        ch.setFriction(b + 0.095f);
        ch.setGravity(new Vector3f(b + 0.10f, b + 0.11f, b + 0.12f));
        ch.setJumpSpeed(b + 0.125f);
        ch.setLinearDamping(b + 0.13f);
        /*
         * Walk direction affects linear velocity, so set it first!
         */
        ch.setWalkDirection(new Vector3f(b + 0.291f, b + 0.292f, b + 0.293f));
        ch.setLinearVelocity(new Vector3f(b + 0.26f, b + 0.27f, b + 0.28f));

        ch.setMaxPenetrationDepth(b + 0.281f);
        ch.setMaxSlope(b + 0.282f);
        ch.setPhysicsLocation(new Vector3f(b + 0.18f, b + 0.19f, b + 0.20f));
        ch.setRestitution(b + 0.25f);
        ch.setRollingFriction(b + 0.26f);
        ch.setSpinningFriction(b + 0.27f);
        ch.setStepHeight(b + 0.29f);
    }

    /**
     * Verify that all PhysicsCharacter parameters have their expected values
     * for the specified key value.
     *
     * @param ch the character to verify (not null, unaffected)
     * @param b the key value
     */
    private void verifyParameters(PhysicsCharacter ch, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);
        assert ch.isContactResponse() == flag;
        assert ch.isUsingGhostSweepTest() == !flag;

        int index = Math.round(b / 0.3f);
        if (index == 0) {
            assert !ch.hasAnisotropicFriction(3);
        } else {
            assert ch.hasAnisotropicFriction(index);
            Vector3f c = ch.getAnisotropicFriction(null);
            assert c.x == b + 0.004f : c;
            assert c.y == b + 0.005f : c;
            assert c.z == b + 0.006f : c;
        }

        assert ch.getAngularDamping() == b + 0.01f;

        Vector3f w = ch.getAngularVelocity(null);
        assert w.x == b + 0.04f : w;
        assert w.y == b + 0.05f : w;
        assert w.z == b + 0.06f : w;

        assert ch.getCcdMotionThreshold() == b + 0.07f;
        assert ch.getCcdSweptSphereRadius() == b + 0.08f;
        assert ch.getContactDamping() == b + 0.084f;
        assert ch.getContactProcessingThreshold() == b + 0.0845f;
        assert ch.getContactStiffness() == b + 0.085f;
        assert ch.getDeactivationTime() == b + 0.087f;
        assert ch.getFallSpeed() == b + 0.09f;
        assert ch.getFriction() == b + 0.095f;

        Vector3f g = ch.getGravity(null);
        assert FastMath.approximateEquals(g.x, b + 0.10f) : g;
        assert FastMath.approximateEquals(g.y, b + 0.11f) : g;
        assert FastMath.approximateEquals(g.z, b + 0.12f) : g;

        assert ch.getJumpSpeed() == b + 0.125f;
        assert ch.getLinearDamping() == b + 0.13f;

        Vector3f v = ch.getLinearVelocity(null);
        assert FastMath.approximateEquals(v.x, b + 0.26f) : v;
        assert FastMath.approximateEquals(v.y, b + 0.27f) : v;
        assert FastMath.approximateEquals(v.z, b + 0.28f) : v;

        assert ch.getMaxPenetrationDepth() == b + 0.281f;
        assert ch.getMaxSlope() == b + 0.282f;

        Vector3f x = ch.getPhysicsLocation(null);
        assert x.x == b + 0.18f : x;
        assert x.y == b + 0.19f : x;
        assert x.z == b + 0.20f : x;

        assert ch.getRestitution() == b + 0.25f;
        assert ch.getRollingFriction() == b + 0.26f;
        assert ch.getSpinningFriction() == b + 0.27f;
        assert ch.getStepHeight() == b + 0.29f;

        Vector3f d = ch.getWalkDirection(null);
        assert d.x == b + 0.291f : d;
        assert d.y == b + 0.292f : d;
        assert d.z == b + 0.293f : d;
    }
}
