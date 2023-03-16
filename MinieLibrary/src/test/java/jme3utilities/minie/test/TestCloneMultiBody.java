/*
 Copyright (c) 2020-2023, Stephen Gold
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
import com.jme3.bullet.MultiBody;
import com.jme3.bullet.MultiBodyLink;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.objects.MultiBodyCollider;
import com.jme3.export.binary.BinaryExporter;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.system.NativeLibraryLoader;
import java.util.List;
import jme3utilities.Heart;
import org.junit.Assert;
import org.junit.Test;

/**
 * Test cloning/saving/loading of multibodies. TODO replace asserts with JUnit
 * Assert
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestCloneMultiBody {
    // *************************************************************************
    // fields

    /**
     * AssetManager required by the BinaryImporter
     */
    final private static AssetManager assetManager = new DesktopAssetManager();
    // *************************************************************************
    // new methods exposed

    /**
     * Test cloning/saving/loading of multibodies.
     */
    @Test
    public void testCloneMultiBody() {
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);

        // mb1: MultiBody with a fixed base, no links, no collider
        int numLinks = 0;
        float mass = 1f;
        Vector3f inertia = new Vector3f(1f, 1f, 1f);
        boolean fixedBase = true;
        boolean canSleep = true;
        MultiBody mb1
                = new MultiBody(numLinks, mass, inertia, fixedBase, canSleep);
        assert mb1.listColliders().isEmpty();

        setParameters(mb1, 0f);
        verifyParameters(mb1, 0f);
        MultiBody mb1Clone = Heart.deepCopy(mb1);
        cloneTest(mb1, mb1Clone);

        // mb2: MultiBody with a fixed base, no links, single collider
        MultiBody mb2
                = new MultiBody(numLinks, mass, inertia, fixedBase, canSleep);
        CollisionShape shape = new SphereCollisionShape(0.2f);
        mb2.addBaseCollider(shape);
        assert mb2.listColliders().size() == 1;

        setParameters(mb2, 0f);
        verifyParameters(mb2, 0f);
        MultiBody mb2Clone = Heart.deepCopy(mb2);
        cloneTest(mb2, mb2Clone);

        // mb3: MultiBody with a movable base, one link, 2 colliders
        numLinks = 1;
        fixedBase = false;
        MultiBody mb3
                = new MultiBody(numLinks, mass, inertia, fixedBase, canSleep);
        mb3.addBaseCollider(shape);
        MultiBodyLink parent = null;
        boolean disableCollision = false;
        MultiBodyLink link = mb3.configureRevoluteLink(mass, inertia, parent,
                Quaternion.IDENTITY, Vector3f.UNIT_Y, Vector3f.ZERO,
                Vector3f.UNIT_Z, disableCollision);
        link.addCollider(shape);
        assert mb3.listColliders().size() == 2;

        setParameters(mb3, 0f);
        verifyParameters(mb3, 0f);
        MultiBody mb3Clone = Heart.deepCopy(mb3);
        cloneTest(mb3, mb3Clone);

        // TODO: various links, added to space, etc.
    }
    // *************************************************************************
    // private methods

    private static void cloneTest(MultiBody mb, MultiBody mbClone) {
        assert mbClone.nativeId() != mb.nativeId();

        verifyParameters(mb, 0f);
        verifyParameters(mbClone, 0f);

        setParameters(mb, 0.3f);
        verifyParameters(mb, 0.3f);
        verifyParameters(mbClone, 0f);

        setParameters(mbClone, 0.6f);
        verifyParameters(mb, 0.3f);
        verifyParameters(mbClone, 0.6f);

        MultiBody mbCopy = BinaryExporter.saveAndLoad(assetManager, mb);
        assert mbCopy != null;
        verifyParameters(mbCopy, 0.3f);

        MultiBody mbCloneCopy
                = BinaryExporter.saveAndLoad(assetManager, mbClone);
        verifyParameters(mbCloneCopy, 0.6f);
    }

    private static void setColliderParameters(
            MultiBodyCollider collider, float b) {
        collider.setCcdMotionThreshold(b + 0.004f);
        collider.setCcdSweptSphereRadius(b + 0.005f);
        collider.setContactDamping(b + 0.006f);
        collider.setContactProcessingThreshold(b + 0.007f);
        collider.setContactStiffness(b + 0.008f);
        collider.setDeactivationTime(b + 0.009f);
        collider.setFriction(b + 0.01f);
        collider.setRestitution(b + 0.018f);
        collider.setRollingFriction(b + 0.019f);
        collider.setSpinningFriction(b + 0.02f);
    }

    private static void setParameters(MultiBody multiBody, float b) {
        multiBody.setCollideWithGroups(Float.floatToIntBits(b) & 0xFFFF);
        multiBody.setCollisionGroup(1 << Math.round(b / 0.3f));

        List<MultiBodyCollider> colliders = multiBody.listColliders();
        int numColliders = colliders.size();
        for (int i = 0; i < numColliders; ++i) {
            MultiBodyCollider c = colliders.get(i);
            setColliderParameters(c, b + 0.02f * i);
        }
    }

    private static void
            verifyColliderParameters(MultiBodyCollider collider, float b) {
        Assert.assertEquals(b + 0.004f, collider.getCcdMotionThreshold(), 0f);
        Assert.assertEquals(b + 0.005f, collider.getCcdSweptSphereRadius(), 0f);
        Assert.assertEquals(b + 0.006f, collider.getContactDamping(), 0f);
        Assert.assertEquals(
                b + 0.007f, collider.getContactProcessingThreshold(), 0f);
        Assert.assertEquals(b + 0.008f, collider.getContactStiffness(), 0f);
        Assert.assertEquals(b + 0.01f, collider.getFriction(), 0f);
        Assert.assertEquals(b + 0.018f, collider.getRestitution(), 0f);
        Assert.assertEquals(b + 0.019f, collider.getRollingFriction(), 0f);
        Assert.assertEquals(b + 0.02f, collider.getSpinningFriction(), 0f);
    }

    private static void verifyParameters(MultiBody multiBody, float b) {
        Assert.assertEquals(Float.floatToIntBits(b) & 0xFFFF,
                multiBody.collideWithGroups());
        Assert.assertEquals(
                1 << Math.round(b / 0.3f), multiBody.collisionGroup());

        List<MultiBodyCollider> colliders = multiBody.listColliders();
        int numColliders = colliders.size();
        for (int i = 0; i < numColliders; ++i) {
            MultiBodyCollider collider = colliders.get(i);
            verifyColliderParameters(collider, b + 0.02f * i);
        }
    }
}
