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
import com.jme3.bullet.animation.DynamicAnimControl;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.control.AbstractPhysicsControl;
import com.jme3.bullet.control.BetterCharacterControl;
import com.jme3.bullet.control.SoftBodyControl;
import com.jme3.bullet.objects.PhysicsCharacter;
import com.jme3.export.binary.BinaryExporter;
import com.jme3.math.Vector3f;
import com.jme3.system.NativeLibraryLoader;
import jme3utilities.Misc;
import jme3utilities.minie.MinieCharacterControl;
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
                = (BetterCharacterControl) Misc.deepCopy(bcc);
        cloneTest(bcc, bccClone);
        /*
         * DynamicAnimControl
         */
        DynamicAnimControl dac = new DynamicAnimControl();
        setParameters(dac, 0f);
        verifyParameters(dac, 0f);
        DynamicAnimControl dacClone = (DynamicAnimControl) Misc.deepCopy(dac);
        cloneTest(dac, dacClone);
        /*
         * MinieCharacterControl
         */
        CollisionShape shape = new SphereCollisionShape(2f);
        MinieCharacterControl mcc = new MinieCharacterControl(shape, 0.5f);
        setParameters(mcc, 0f);
        verifyParameters(mcc, 0f);
        MinieCharacterControl mccClone
                = (MinieCharacterControl) Misc.deepCopy(mcc);
        cloneTest(mcc, mccClone);
        /*
         * SoftBodyControl
         */
        SoftBodyControl sbc = new SoftBodyControl();
        setParameters(sbc, 0f);
        verifyParameters(sbc, 0f);
        SoftBodyControl sbcClone = (SoftBodyControl) Misc.deepCopy(sbc);
        cloneTest(sbc, sbcClone);

        // TODO test cloning/saving/loading abstract physics controls
        // that have been added to a Spatial
    }
    // *************************************************************************
    // private methods

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
        } else if (control instanceof DynamicAnimControl) {
            setDac((DynamicAnimControl) control, b);
        } else if (control instanceof MinieCharacterControl) {
            setMcc((MinieCharacterControl) control, b);
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
        bcc.setPhysicsDamping(b + 0.08f);

        bcc.setJumpForce(new Vector3f(b + 0.05f, b + 0.06f, b + 0.07f));
        bcc.setViewDirection(new Vector3f(b + 0.10f, b + 0.11f, b + 0.12f));
        bcc.setWalkDirection(new Vector3f(b + 0.13f, b + 0.14f, b + 0.15f));
    }

    private void setDac(DynamicAnimControl dac, float b) {
        dac.setDamping(b + 0.01f);
        dac.setEventDispatchImpulseThreshold(b + 0.02f);
        dac.setGravity(new Vector3f(b + 0.03f, b + 0.04f, b + 0.05f));
    }

    private void setMcc(MinieCharacterControl mcc, float b) {
        PhysicsCharacter pc = mcc.getCharacter();
        pc.setJumpSpeed(b + 0.01f);
        pc.setLinearDamping(b + 0.02f);
        pc.setWalkDirection(new Vector3f(b + 0.03f, b + 0.04f, b + 0.05f));

        mcc.setViewDirection(new Vector3f(b + 0.1f, b + 0.5f, b + 0.7f));
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
        } else if (control instanceof DynamicAnimControl) {
            verifyDac((DynamicAnimControl) control, b);
        } else if (control instanceof MinieCharacterControl) {
            verifyMcc((MinieCharacterControl) control, b);
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
        assert bcc.getPhysicsDamping() == b + 0.08f;

        Vector3f f = bcc.getJumpForce(null);
        assert f.x == b + 0.05f : f;
        assert f.y == b + 0.06f : f;
        assert f.z == b + 0.07f : f;

        Vector3f v = bcc.getViewDirection(null);
        assert v.x == b + 0.10f : v;
        assert v.y == b + 0.11f : v;
        assert v.z == b + 0.12f : v;

        Vector3f w = bcc.getWalkDirection(null);
        assert w.x == b + 0.13f : w;
        assert w.y == b + 0.14f : w;
        assert w.z == b + 0.15f : w;
    }

    private void verifyDac(DynamicAnimControl dac, float b) {
        assert dac.damping() == b + 0.01f;
        assert dac.eventDispatchImpulseThreshold() == b + 0.02f;

        Vector3f g = dac.gravity(null);
        assert g.x == b + 0.03f : g;
        assert g.y == b + 0.04f : g;
        assert g.z == b + 0.05f : g;
    }

    private void verifyMcc(MinieCharacterControl mcc, float b) {
        PhysicsCharacter pc = mcc.getCharacter();
        assert pc.getJumpSpeed() == b + 0.01f;
        assert pc.getLinearDamping() == b + 0.02f;

        Vector3f w = pc.getWalkDirection(null);
        assert w.x == b + 0.03f : w;
        assert w.y == b + 0.04f : w;
        assert w.z == b + 0.05f : w;

        Vector3f norm
                = new Vector3f(b + 0.1f, b + 0.5f, b + 0.7f).normalizeLocal();
        Vector3f v = mcc.getViewDirection(null);
        assert v.x == norm.x : v;
        assert v.y == norm.y : v;
        assert v.z == norm.z : v;
    }

    private void verifySbc(SoftBodyControl sbc, float b) {
        // TODO
    }
}
