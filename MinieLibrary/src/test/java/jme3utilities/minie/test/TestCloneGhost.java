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
import com.jme3.asset.AssetNotFoundException;
import com.jme3.asset.DesktopAssetManager;
import com.jme3.asset.ModelKey;
import com.jme3.asset.plugins.FileLocator;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.control.GhostControl;
import com.jme3.bullet.objects.PhysicsGhostObject;
import com.jme3.export.JmeExporter;
import com.jme3.export.binary.BinaryExporter;
import com.jme3.export.binary.BinaryLoader;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.control.Control;
import com.jme3.system.NativeLibraryLoader;
import java.io.File;
import java.io.IOException;
import jme3utilities.Misc;
import org.junit.Test;

/**
 * Test cloning/saving/loading on PhysicsGhostObject and its subclass.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestCloneGhost {
    // *************************************************************************
    // fields

    /**
     * asset manager to load the saved control from a temporary file
     */
    private AssetManager assetManager;
    /**
     * number of temporary files created
     */
    private int fileIndex = 0;
    // *************************************************************************
    // new methods exposed

    @Test
    public void testCloneGhost() {
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);
        assetManager = new DesktopAssetManager();
        assetManager.registerLoader(BinaryLoader.class, "j3o");
        assetManager.registerLocator(".", FileLocator.class);

        CollisionShape shape = new SphereCollisionShape(1f);
        /*
         * PhysicsGhostObject
         */
        PhysicsGhostObject pgo = new PhysicsGhostObject(shape);
        setParameters(pgo, 0f);
        verifyParameters(pgo, 0f);
        PhysicsGhostObject pgoClone = (PhysicsGhostObject) Misc.deepCopy(pgo);
        cloneTest(pgo, pgoClone);
        /*
         * GhostControl (a subclass of PhysicsGhostObject)
         */
        GhostControl gc = new GhostControl(shape);
        setParameters(gc, 0f);
        verifyParameters(gc, 0f);
        GhostControl gcClone = (GhostControl) Misc.deepCopy(gc);
        cloneTest(gc, gcClone);
    }
    // *************************************************************************
    // private methods

    private void cloneTest(PhysicsGhostObject pgo,
            PhysicsGhostObject pgoClone) {
        assert pgoClone.getObjectId() != pgo.getObjectId();

        verifyParameters(pgo, 0f);
        verifyParameters(pgoClone, 0f);

        setParameters(pgo, 0.3f);
        verifyParameters(pgo, 0.3f);
        verifyParameters(pgoClone, 0f);

        setParameters(pgoClone, 0.6f);
        verifyParameters(pgo, 0.3f);
        verifyParameters(pgoClone, 0.6f);

        if (pgo instanceof GhostControl) {
            GhostControl pgoCopy = saveThenLoad((GhostControl) pgo);
            verifyParameters(pgoCopy, 0.3f);

            GhostControl gcCloneCopy = saveThenLoad((GhostControl) pgoClone);
            verifyParameters(gcCloneCopy, 0.6f);
        }
    }

    /**
     * Clone a GhostControl by saving and then loading it.
     *
     * @param gc the control to copy (not null, unaffected)
     * @return a new control
     */
    private GhostControl saveThenLoad(GhostControl gc) {
        Node savedNode = new Node();
        /*
         * Add the Control to the Node without altering its physics transform.
         */
        Vector3f pl = gc.getPhysicsLocation(null);
        Matrix3f pr = gc.getPhysicsRotationMatrix(null);
        savedNode.addControl(gc);
        gc.setPhysicsLocation(pl);
        gc.setPhysicsRotation(pr);

        String fileName = String.format("tmp%d.j3o", ++fileIndex);
        File file = new File(fileName);

        JmeExporter exporter = BinaryExporter.getInstance();
        try {
            exporter.save(savedNode, file);
        } catch (IOException exception) {
            throw new RuntimeException(exception);
        }

        ModelKey key = new ModelKey(fileName);
        Spatial loadedNode = new Node();
        try {
            loadedNode = assetManager.loadAsset(key);
        } catch (AssetNotFoundException exception) {
            throw new RuntimeException(exception);
        }
        file.delete();
        Control loadedSgc = loadedNode.getControl(0);

        return (GhostControl) loadedSgc;
    }

    /**
     * Modify PhysicsGhostObject parameters based on the specified key value.
     *
     * @param ch the ghost object to modify (not null)
     * @param b the key value
     */
    private void setParameters(PhysicsGhostObject pgo, float b) {
        pgo.setCcdMotionThreshold(b + 0.07f);
        pgo.setCcdSweptSphereRadius(b + 0.08f);
        pgo.setPhysicsLocation(new Vector3f(b + 0.18f, b + 0.19f, b + 0.20f));

        Quaternion orient
                = new Quaternion(b + 0.21f, b + 0.22f, b + 0.23f, b + 0.24f);
        orient.normalizeLocal();
        Matrix3f matrix = orient.toRotationMatrix();
        pgo.setPhysicsRotation(matrix);
    }

    /**
     * Verify that all PhysicsGhostObject parameters have their expected values
     * for the specified key value.
     *
     * @param pgo the ghost object to verify (not null, unaffected)
     * @param b the key value
     */
    private void verifyParameters(PhysicsGhostObject pgo, float b) {
        assert pgo.getCcdMotionThreshold() == b + 0.07f;
        assert pgo.getCcdSweptSphereRadius() == b + 0.08f;

        Vector3f x = pgo.getPhysicsLocation(null);
        assert x.x == b + 0.18f : x;
        assert x.y == b + 0.19f : x;
        assert x.z == b + 0.20f : x;

        Quaternion orient
                = new Quaternion(b + 0.21f, b + 0.22f, b + 0.23f, b + 0.24f);
        orient.normalizeLocal();
        Matrix3f matrix = orient.toRotationMatrix();
        Matrix3f m = pgo.getPhysicsRotationMatrix(null);
        assert m.equals(matrix);
    }
}
