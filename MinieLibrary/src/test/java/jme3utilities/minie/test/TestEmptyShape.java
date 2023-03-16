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

import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.EmptyShape;
import com.jme3.bullet.collision.shapes.GImpactCollisionShape;
import com.jme3.bullet.collision.shapes.HeightfieldCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.MeshCollisionShape;
import com.jme3.bullet.collision.shapes.MultiSphere;
import com.jme3.bullet.collision.shapes.infos.IndexedMesh;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.system.NativeLibraryLoader;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.List;
import jme3utilities.math.MyVector3f;
import org.junit.Assert;
import org.junit.Test;

/**
 * Test various ways of creating an empty collision shape, including some
 * illegal ones.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestEmptyShape {
    // *************************************************************************
    // new methods exposed

    /**
     * Test various ways of creating an empty collision shape, including some
     * illegal ones.
     */
    @Test
    public void testEmptyShape() {
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);

        // Instantiate various "empty" objects.
        float[] floatArray = new float[0];
        IndexedMesh[] indexedMeshArray = new IndexedMesh[0];
        Mesh[] jmeMeshArray = new Mesh[0];

        List<Float> floatList = new ArrayList<>(1);
        List<IndexedMesh> indexedMeshList = new ArrayList<>(1);
        List<Vector3f> vectorList = new ArrayList<>(1);

        FloatBuffer buffer = BufferUtils.createFloatBuffer(0);
        buffer.flip();

        Mesh jmeMesh = new Mesh();
        jmeMesh.setBuffer(
                VertexBuffer.Type.Position, MyVector3f.numAxes, buffer);
        jmeMesh.updateBound();
        IndexedMesh indexedMesh = new IndexedMesh(jmeMesh);
        /*
         * Create empty shapes in legal ways
         * and use them to construct static rigid bodies.
         */
        CollisionShape shape = new EmptyShape(true);
        PhysicsRigidBody rigidBody
                = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);

        shape = new CompoundCollisionShape();
        rigidBody = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);

        shape = new GImpactCollisionShape(indexedMeshArray);
        rigidBody = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);

        shape = new GImpactCollisionShape(jmeMeshArray);
        rigidBody = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);

        // Attempt to create empty shapes in various illegal ways.
        try {
            shape = new HeightfieldCollisionShape(floatArray);
            Assert.fail("Expected an IllegalArgumentException");
        } catch (IllegalArgumentException exception) {
            // do nothing
        }

        try {
            shape = new HeightfieldCollisionShape(
                    floatArray, Vector3f.UNIT_XYZ);
            Assert.fail("Expected an IllegalArgumentException");
        } catch (IllegalArgumentException exception) {
            // do nothing
        }

        try {
            shape = new HullCollisionShape(vectorList);
            Assert.fail("Expected an IllegalArgumentException");
        } catch (IllegalArgumentException exception) {
            // do nothing
        }

        try {
            shape = new HullCollisionShape(floatArray);
            Assert.fail("Expected an IllegalArgumentException");
        } catch (IllegalArgumentException exception) {
            // do nothing
        }

        try {
            shape = new HullCollisionShape(buffer);
            Assert.fail("Expected an IllegalArgumentException");
        } catch (IllegalArgumentException exception) {
            // do nothing
        }

        try {
            shape = new HullCollisionShape(jmeMesh);
            Assert.fail("Expected an IllegalArgumentException");
        } catch (IllegalArgumentException exception) {
            // do nothing
        }

        try {
            shape = new HullCollisionShape(jmeMeshArray);
            Assert.fail("Expected an IllegalArgumentException");
        } catch (IllegalArgumentException exception) {
            // do nothing
        }

        try {
            shape = new MeshCollisionShape(true, indexedMesh);
            Assert.fail("Expected an IllegalArgumentException");
        } catch (IllegalArgumentException exception) {
            // do nothing
        }

        try {
            shape = new MeshCollisionShape(true, indexedMeshArray);
            Assert.fail("Expected an IllegalArgumentException");
        } catch (IllegalArgumentException exception) {
            // do nothing
        }

        try {
            shape = new MeshCollisionShape(true, indexedMeshList);
            Assert.fail("Expected an IllegalArgumentException");
        } catch (IllegalArgumentException exception) {
            // do nothing
        }

        try {
            shape = new MeshCollisionShape(jmeMesh, true);
            Assert.fail("Expected an IllegalArgumentException");
        } catch (IllegalArgumentException exception) {
            // do nothing
        }

        try {
            shape = new MultiSphere(vectorList, floatList);
        } catch (IllegalArgumentException exception) {
            // do nothing
        }
    }
}
