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
import com.jme3.bullet.control.BetterCharacterControl;
import com.jme3.export.binary.BinaryExporter;
import com.jme3.math.Vector3f;
import com.jme3.system.NativeLibraryLoader;
import jme3utilities.Misc;
import org.junit.Test;

/**
 * Test cloning abstract physics controls.
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
     * Test cloning abstract physics controls.
     */
    @Test
    public void testClonePhysicsControls() {
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);
        /*
         * test BetterCharacterControl
         */
        float radius = 1f;
        float height = 3f;
        float mass = 1f;
        BetterCharacterControl bcc
                = new BetterCharacterControl(radius, height, mass);
        bcc.setEnabled(true);
        setParameters(bcc, 0f);
        verifyParameters(bcc, 0f);

        BetterCharacterControl bccClone
                = (BetterCharacterControl) Misc.deepCopy(bcc);
        verifyParameters(bcc, 0f);
        verifyParameters(bccClone, 0f);

        setParameters(bcc, 0.3f);
        verifyParameters(bcc, 0.3f);
        verifyParameters(bccClone, 0f);

        setParameters(bccClone, 0.6f);
        verifyParameters(bcc, 0.3f);
        verifyParameters(bccClone, 0.6f);

        BetterCharacterControl bccCopy
                = BinaryExporter.saveAndLoad(assetManager, bcc);
        verifyParameters(bccCopy, 0.3f);

        BetterCharacterControl bccCloneCopy
                = BinaryExporter.saveAndLoad(assetManager, bccClone);
        verifyParameters(bccCloneCopy, 0.6f);
        /*
         * test DynamicAnimControl TODO
         */
        DynamicAnimControl dac = new DynamicAnimControl();
        DynamicAnimControl dacClone
                = (DynamicAnimControl) Misc.deepCopy(dac);

        // TODO test cloning controls added to a scene graph
    }
    // *************************************************************************
    // private methods

    /**
     * Modify BetterCharacterControl parameters based on the specified key
     * value.
     *
     * @param bcc the control to modify (not null)
     * @param b the key value
     */
    private void setParameters(BetterCharacterControl bcc, float b) {
        bcc.setDuckedFactor(b + 0.01f);
        bcc.setPhysicsDamping(b + 0.08f);

        bcc.setJumpForce(new Vector3f(b + 0.05f, b + 0.06f, b + 0.07f));
        bcc.setViewDirection(new Vector3f(b + 0.10f, b + 0.11f, b + 0.12f));
        bcc.setWalkDirection(new Vector3f(b + 0.13f, b + 0.14f, b + 0.15f));
    }

    /**
     * Verify that all BetterCharacterControl parameters have their expected
     * values for the specified key value.
     *
     * @param bcc the control to verify (not null, unaffected)
     * @param b the key value
     */
    private void verifyParameters(BetterCharacterControl bcc, float b) {
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
}
