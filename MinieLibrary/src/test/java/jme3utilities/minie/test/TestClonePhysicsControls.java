/*
 Copyright (c) 2018-2020, Stephen Gold
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
import com.jme3.bullet.animation.DynamicAnimControl;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.control.AbstractPhysicsControl;
import com.jme3.bullet.control.BetterCharacterControl;
import com.jme3.bullet.control.CharacterControl;
import com.jme3.bullet.control.SoftBodyControl;
import com.jme3.bullet.objects.PhysicsCharacter;
import com.jme3.export.binary.BinaryExporter;
import com.jme3.math.Vector3f;
import com.jme3.system.NativeLibraryLoader;
import jme3utilities.Heart;
import org.junit.Assert;
import org.junit.Test;

/**
 * Test cloning/saving/loading abstract physics controls.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestClonePhysicsControls {
    // *************************************************************************
    // fields

    /**
     * AssetManager required by the BinaryImporter
     */
    final private AssetManager assetManager = new DesktopAssetManager();
    // *************************************************************************
    // new methods exposed

    /**
     * Test cloning/saving/loading abstract physics controls.
     */
    @Test
    public void testClonePhysicsControls() {
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);
        /*
         * BetterCharacterControl
         */
        float radius = 1f;
        float height = 3f;
        float mass = 1f;
        BetterCharacterControl bcc
                = new BetterCharacterControl(radius, height, mass);
        setParameters(bcc, 0f);
        verifyParameters(bcc, 0f);
        BetterCharacterControl bccClone
                = (BetterCharacterControl) Heart.deepCopy(bcc);
        cloneTest(bcc, bccClone);
        /*
         * CharacterControl
         */
        SphereCollisionShape shape = new SphereCollisionShape(2f);
        CharacterControl cc = new CharacterControl(shape, 0.5f);
        setParameters(cc, 0f);
        verifyParameters(cc, 0f);
        CharacterControl ccClone = (CharacterControl) Heart.deepCopy(cc);
        cloneTest(cc, ccClone);
        /*
         * DynamicAnimControl
         */
        DynamicAnimControl dac = new DynamicAnimControl();
        setParameters(dac, 0f);
        verifyParameters(dac, 0f);
        DynamicAnimControl dacClone = (DynamicAnimControl) Heart.deepCopy(dac);
        cloneTest(dac, dacClone);
        /*
         * SoftBodyControl
         */
        SoftBodyControl sbc = new SoftBodyControl();
        setParameters(sbc, 0f);
        verifyParameters(sbc, 0f);
        SoftBodyControl sbcClone = (SoftBodyControl) Heart.deepCopy(sbc);
        cloneTest(sbc, sbcClone);

        // TODO test cloning/saving/loading abstract physics controls
        // that have been added to a Spatial
    }
    // *************************************************************************
    // private methods

    /**
     * Verify that 2 vectors are equal to within some tolerance.
     */
    void assertEquals(float x, float y, float z, Vector3f actual,
            float tolerance) {
        Assert.assertEquals(x, actual.x, tolerance);
        Assert.assertEquals(y, actual.y, tolerance);
        Assert.assertEquals(z, actual.z, tolerance);
    }

    /**
     * Verify that 2 vectors are equal to within some tolerance.
     */
    private void assertEquals(Vector3f expected, Vector3f actual,
            float tolerance) {
        assertEquals(expected.x, expected.y, expected.z, actual, tolerance);
    }

    private void cloneTest(AbstractPhysicsControl control,
            AbstractPhysicsControl controlClone) {
        verifyParameters(control, 0f);
        verifyParameters(controlClone, 0f);

        setParameters(control, 0.3f);
        verifyParameters(control, 0.3f);
        verifyParameters(controlClone, 0f);

        setParameters(controlClone, 0.6f);
        verifyParameters(control, 0.3f);
        verifyParameters(controlClone, 0.6f);

        AbstractPhysicsControl controlCopy
                = BinaryExporter.saveAndLoad(assetManager, control);
        verifyParameters(controlCopy, 0.3f);

        AbstractPhysicsControl controlCloneCopy
                = BinaryExporter.saveAndLoad(assetManager, controlClone);
        verifyParameters(controlCloneCopy, 0.6f);
    }

    private void setParameters(AbstractPhysicsControl control, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);
        if (!(control instanceof DynamicAnimControl)) {
            control.setApplyPhysicsLocal(!flag);
        }
        control.setEnabled(flag);

        if (control instanceof BetterCharacterControl) {
            setBcc((BetterCharacterControl) control, b);
        } else if (control instanceof CharacterControl) {
            setCc((CharacterControl) control, b);
        } else if (control instanceof DynamicAnimControl) {
            setDac((DynamicAnimControl) control, b);
        } else if (control instanceof SoftBodyControl) {
            setSbc((SoftBodyControl) control, b);
        } else {
            throw new IllegalArgumentException(control.getClass().getName());
        }
    }

    /**
     * Modify BetterCharacterControl parameters based on the specified key
     * value.
     *
     * @param bcc the control to modify (not null)
     * @param b the key value
     */
    private void setBcc(BetterCharacterControl bcc, float b) {
        bcc.setDuckedFactor(b + 0.01f);
        bcc.setJumpForce(new Vector3f(b + 0.05f, b + 0.06f, b + 0.07f));
        bcc.setPhysicsDamping(b + 0.08f);
        bcc.setViewDirection(new Vector3f(b + 0.10f, b + 0.11f, b + 0.12f));
        bcc.setWalkDirection(new Vector3f(b + 0.13f, b + 0.14f, b + 0.15f));
    }

    private void setCc(CharacterControl cc, float b) {
        Vector3f upDirection
                = new Vector3f(b - 0.2f, b + 0.8f, b - 0.6f).normalize();
        Vector3f viewDirection
                = new Vector3f(b + 0.1f, b + 0.5f, b + 0.7f).normalizeLocal();
        /*
         * walk offset must be perpendicular to "up" direction
         */
        Vector3f walkOffset = new Vector3f(b + 0.6f, b + 0.2f, b + 0.4f)
                .cross(upDirection);

        PhysicsCharacter ch = cc.getCharacter();

        ch.setAngularDamping(b + 0.01f);
        ch.setAngularVelocity(new Vector3f(b + 0.04f, b + 0.05f, b + 0.06f));
        ch.setCcdMotionThreshold(b + 0.07f);
        ch.setCcdSweptSphereRadius(b + 0.08f);
        ch.setContactDamping(b + 0.084f);
        ch.setContactProcessingThreshold(b + 0.0845f);
        ch.setContactStiffness(b + 0.085f);
        ch.setDeactivationTime(b + 0.087f);
        cc.setFallSpeed(b + 0.01f);
        ch.setFriction(b + 0.095f);
        cc.setGravity(b + 0.015f);
        cc.setJumpSpeed(b + 0.02f);
        ch.setLinearDamping(b + 0.03f);
        ch.setMaxPenetrationDepth(b + 0.281f);
        ch.setMaxSlope(b + 0.282f);
        ch.setPhysicsLocation(new Vector3f(b + 0.18f, b + 0.19f, b + 0.20f));
        ch.setRestitution(b + 0.25f);
        ch.setRollingFriction(b + 0.26f);
        ch.setSpinningFriction(b + 0.27f);
        ch.setStepHeight(b + 0.29f);
        ch.setUp(upDirection);
        cc.setViewDirection(viewDirection);
        cc.setWalkDirection(walkOffset);
    }

    private void setDac(DynamicAnimControl dac, float b) {
        dac.setDamping(b + 0.01f);
        dac.setEventDispatchImpulseThreshold(b + 0.02f);
        dac.setGravity(new Vector3f(b + 0.03f, b + 0.04f, b + 0.05f));
    }

    private void setSbc(SoftBodyControl sbc, float b) {
        // TODO
    }

    private void verifyParameters(AbstractPhysicsControl control, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);
        if (!(control instanceof DynamicAnimControl)) {
            assert control.isApplyPhysicsLocal() == !flag;
        }
        assert control.isEnabled() == flag;

        if (control instanceof BetterCharacterControl) {
            verifyBcc((BetterCharacterControl) control, b);
        } else if (control instanceof CharacterControl) {
            verifyCc((CharacterControl) control, b);
        } else if (control instanceof DynamicAnimControl) {
            verifyDac((DynamicAnimControl) control, b);
        } else if (control instanceof SoftBodyControl) {
            verifySbc((SoftBodyControl) control, b);
        } else {
            throw new IllegalArgumentException(control.getClass().getName());
        }
    }

    /**
     * Verify that all BetterCharacterControl parameters have their expected
     * values for the specified key value.
     *
     * @param bcc the control to verify (not null, unaffected)
     * @param b the key value
     */
    private void verifyBcc(BetterCharacterControl bcc, float b) {
        assert bcc.getDuckedFactor() == b + 0.01f;
        assertEquals(b + 0.05f, b + 0.06f, b + 0.07f,
                bcc.getJumpForce(null), 0f);
        assert bcc.getPhysicsDamping() == b + 0.08f;
        assertEquals(b + 0.10f, b + 0.11f, b + 0.12f,
                bcc.getViewDirection(null), 0f);
        assertEquals(b + 0.13f, b + 0.14f, b + 0.15f,
                bcc.getWalkDirection(null), 0f);
    }

    private void verifyCc(CharacterControl cc, float b) {
        Vector3f upDirection
                = new Vector3f(b - 0.2f, b + 0.8f, b - 0.6f).normalize();
        Vector3f viewDirection
                = new Vector3f(b + 0.1f, b + 0.5f, b + 0.7f).normalizeLocal();
        /*
         * walk offset must be perpendicular to "up" direction
         */
        Vector3f walkOffset = new Vector3f(b + 0.6f, b + 0.2f, b + 0.4f)
                .cross(upDirection);

        PhysicsCharacter ch = cc.getCharacter();

        Assert.assertEquals(b + 0.01f, ch.getAngularDamping(), 0f);
        assertEquals(b + 0.04f, b + 0.05f, b + 0.06f,
                ch.getAngularVelocity(null), 0f);
        Assert.assertEquals(b + 0.07f, ch.getCcdMotionThreshold(), 0f);
        Assert.assertEquals(b + 0.08f, ch.getCcdSweptSphereRadius(), 0f);
        Assert.assertEquals(b + 0.084f, ch.getContactDamping(), 0f);
        Assert.assertEquals(b + 0.0845f, ch.getContactProcessingThreshold(),
                0f);
        Assert.assertEquals(b + 0.085f, ch.getContactStiffness(), 0f);
        Assert.assertEquals(b + 0.087f, ch.getDeactivationTime(), 0f);
        Assert.assertEquals(b + 0.01f, ch.getFallSpeed(), 0f);
        Assert.assertEquals(b + 0.095f, ch.getFriction(), 0f);
        Assert.assertEquals(b + 0.015f, ch.getGravity(null).length(), 1e-5f);
        Assert.assertEquals(b + 0.02f, ch.getJumpSpeed(), 0f);
        Assert.assertEquals(b + 0.03f, ch.getLinearDamping(), 0f);
        Assert.assertEquals(b + 0.281f, ch.getMaxPenetrationDepth(), 0f);
        Assert.assertEquals(b + 0.282f, ch.getMaxSlope(), 0f);
        assertEquals(b + 0.18f, b + 0.19f, b + 0.20f,
                ch.getPhysicsLocation(null), 0f);
        Assert.assertEquals(b + 0.25f, ch.getRestitution(), 0f);
        Assert.assertEquals(b + 0.26f, ch.getRollingFriction(), 0f);
        Assert.assertEquals(b + 0.27f, ch.getSpinningFriction(), 0f);
        Assert.assertEquals(b + 0.29f, ch.getStepHeight(), 0f);
        assertEquals(upDirection, ch.getUpDirection(null), 1e-5f);
        assertEquals(viewDirection, cc.getViewDirection(null), 0f);
        assertEquals(walkOffset, ch.getWalkDirection(null), 1e-5f);
    }

    private void verifyDac(DynamicAnimControl dac, float b) {
        assert dac.damping() == b + 0.01f;
        assert dac.eventDispatchImpulseThreshold() == b + 0.02f;
        assertEquals(b + 0.03f, b + 0.04f, b + 0.05f,
                dac.gravity(null), 0f);
    }

    private void verifySbc(SoftBodyControl sbc, float b) {
        // TODO
    }
}
