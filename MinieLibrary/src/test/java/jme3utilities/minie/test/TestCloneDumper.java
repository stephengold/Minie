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

import jme3utilities.minie.DumpFlags;
import jme3utilities.minie.PhysicsDumper;
import org.junit.Assert;
import org.junit.Test;

/**
 * Test cloning a PhysicsDumper.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestCloneDumper {
    // *************************************************************************
    // new methods exposed

    /**
     * Test cloning a PhysicsDumper.
     */
    @Test
    public void testCloneDumper() {
        PhysicsDumper dumper = new PhysicsDumper();
        setParameters(dumper, 0f);
        verifyParameters(dumper, 0f);

        PhysicsDumper dumperClone;
        try {
            dumperClone = dumper.clone();
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }

        cloneTest(dumper, dumperClone);
    }
    // *************************************************************************
    // private methods

    private static void cloneTest(PhysicsDumper du, PhysicsDumper duClone) {
        Assert.assertNotSame(du, duClone);
        Assert.assertSame(du.getClass(), duClone.getClass());
        Assert.assertNotSame(duClone.getDescriber(), du.getDescriber());
        Assert.assertSame(du.getDescriber().getClass(),
                duClone.getDescriber().getClass());

        verifyParameters(du, 0f);
        verifyParameters(duClone, 0f);

        setParameters(du, 0.3f);
        verifyParameters(du, 0.3f);
        verifyParameters(duClone, 0f);

        setParameters(duClone, 0.6f);
        verifyParameters(du, 0.3f);
        verifyParameters(duClone, 0.6f);
    }

    /**
     * Modify PhysicsCharacter parameters based on the specified key value.
     *
     * @param du the dumper to modify (not null)
     * @param b the key value
     */
    private static void setParameters(PhysicsDumper du, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);
        du.setEnabled(DumpFlags.Buckets, flag);
        du.setEnabled(DumpFlags.ChildShapes, !flag);
        du.setEnabled(DumpFlags.ClustersInSofts, !flag);
        du.setEnabled(DumpFlags.CullHints, flag);
        du.setEnabled(DumpFlags.JointsInBodies, !flag);
        du.setEnabled(DumpFlags.JointsInSpaces, flag);
        du.setEnabled(DumpFlags.MatParams, !flag);
        du.setEnabled(DumpFlags.NodesInClusters, !flag);
        du.setEnabled(DumpFlags.NodesInSofts, flag);
        du.setEnabled(DumpFlags.Overrides, !flag);
        du.setEnabled(DumpFlags.Pcos, flag);
        du.setEnabled(DumpFlags.ShadowModes, flag);
        du.setEnabled(DumpFlags.Transforms, !flag);
        du.setEnabled(DumpFlags.UserData, flag);

        int count = Math.round(b / 0.3f);
        du.setMaxChildren(count);

        du.setIndentIncrement(Float.toString(b));
        du.getDescriber().setListSeparator(Float.toHexString(b));
    }

    /**
     * Verify that all PhysicsDumper parameters have their expected values for
     * the specified key value.
     *
     * @param du the dumper to verify (not null, unaffected)
     * @param b the key value
     */
    private static void verifyParameters(PhysicsDumper du, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);
        Assert.assertEquals(flag, du.isEnabled(DumpFlags.Buckets));
        Assert.assertEquals(!flag, du.isEnabled(DumpFlags.ChildShapes));
        Assert.assertEquals(!flag, du.isEnabled(DumpFlags.ClustersInSofts));
        Assert.assertEquals(flag, du.isEnabled(DumpFlags.CullHints));
        Assert.assertEquals(!flag, du.isEnabled(DumpFlags.JointsInBodies));
        Assert.assertEquals(flag, du.isEnabled(DumpFlags.JointsInSpaces));
        Assert.assertEquals(!flag, du.isEnabled(DumpFlags.MatParams));
        Assert.assertEquals(!flag, du.isEnabled(DumpFlags.NodesInClusters));
        Assert.assertEquals(flag, du.isEnabled(DumpFlags.NodesInSofts));
        Assert.assertEquals(!flag, du.isEnabled(DumpFlags.Overrides));
        Assert.assertEquals(flag, du.isEnabled(DumpFlags.Pcos));
        Assert.assertEquals(flag, du.isEnabled(DumpFlags.ShadowModes));
        Assert.assertEquals(!flag, du.isEnabled(DumpFlags.Transforms));
        Assert.assertEquals(flag, du.isEnabled(DumpFlags.UserData));

        int count = Math.round(b / 0.3f);
        Assert.assertEquals(count, du.maxChildren());
        Assert.assertEquals(Float.toString(b), du.indentIncrement());
        Assert.assertEquals(
                Float.toHexString(b), du.getDescriber().listSeparator());
    }
}
