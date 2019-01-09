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

import com.jme3.asset.AssetManager;
import com.jme3.asset.DesktopAssetManager;
import com.jme3.asset.ModelKey;
import com.jme3.asset.plugins.ClasspathLocator;
import com.jme3.bullet.animation.DynamicAnimControl;
import com.jme3.bullet.animation.RangeOfMotion;
import com.jme3.export.binary.BinaryLoader;
import com.jme3.material.plugins.J3MLoader;
import com.jme3.scene.Node;
import com.jme3.system.NativeLibraryLoader;
import com.jme3.texture.plugins.AWTLoader;
import org.junit.Test;

/**
 * Link a bone that doesn't influence any mesh vertices.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestNoVertices {
    // *************************************************************************
    // new methods exposed

    @Test
    public void testNoVertices() {
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);

        AssetManager assetManager = new DesktopAssetManager();
        assetManager.registerLoader(AWTLoader.class, "jpg", "png");
        assetManager.registerLoader(BinaryLoader.class, "j3o");
        assetManager.registerLoader(J3MLoader.class, "j3m", "j3md");
        assetManager.registerLocator(null, ClasspathLocator.class);

        ModelKey key = new ModelKey("Models/Jaime/Jaime.j3o");
        Node model = (Node) assetManager.loadModel(key);
        DynamicAnimControl dac = new DynamicAnimControl();
        dac.link("IKeyeTarget.R", 1f, new RangeOfMotion(1f, 1f, 1f));
        model.addControl(dac);
    }
}
