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

import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.math.FastMath;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.system.NativeLibraryLoader;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import org.junit.Test;

/**
 * Test various PhysicsSoftBody methods.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestPhysicsSoftBody {
    // *************************************************************************
    // new methods exposed

    /**
     * Test the {@code applyTransform()} method.
     */
    @Test
    public void testPhysicsSoftBody() {
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);

        Vector3f non2 = new Vector3f(1f, 2f, 3f);

        PhysicsSoftBody psb = new PhysicsSoftBody();
        FloatBuffer nodeLocations = BufferUtils.createFloatBuffer(non2);
        psb.appendNodes(nodeLocations);

        Transform trs1 = new Transform();
        trs1.getScale().set(2f, 0.5f, 3f);
        trs1.getRotation().fromAngles(0f, FastMath.HALF_PI, 0f);
        trs1.getTranslation().set(0.5f, 0.4f, 0.3f);
        psb.applyTransform(trs1);

        Vector3f location = psb.nodeLocation(0, null);
        Utils.assertEquals(9.5f, 1.4f, -1.7f, location, 1e-5f);
    }
}
