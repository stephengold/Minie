/*
 Copyright (c) 2019-2022, Stephen Gold
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

import com.jme3.bounding.BoundingBox;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.EmptyShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.system.NativeLibraryLoader;
import jme3utilities.math.RectangularSolid;
import org.junit.Test;

/**
 * Test AABB calculations for collision shapes.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestShapeAabb {
    // *************************************************************************
    // fields

    /**
     * storage for returned results
     */
    final private static BoundingBox aabb = new BoundingBox();
    // *************************************************************************
    // new methods exposed

    /**
     * Test AABB calculations for collision shapes.
     */
    @Test
    public void testShapeAabb() {
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);

        Quaternion rotation;
        Vector3f location;

        // BoxCollisionShape
        CollisionShape box = new BoxCollisionShape(2f);

        location = new Vector3f(0.5f, 0f, -0.4f);
        rotation = Quaternion.IDENTITY;
        box.boundingBox(location, rotation, aabb);
        checkAabb(-1.5f, -2f, -2.4f, 2.5f, 2f, 1.6f);

        location = new Vector3f(0f, 3f, 7f);
        rotation = new Quaternion();
        rotation.fromAngleAxis(FastMath.QUARTER_PI, Vector3f.UNIT_Y);
        box.boundingBox(location, rotation, aabb);
        float root8 = FastMath.sqrt(8f);
        checkAabb(-root8, 1f, 7f - root8, root8, 5f, 7f + root8);

        // EmptyShape
        CollisionShape empty = new EmptyShape(true);
        empty.boundingBox(location, rotation, aabb);
        checkAabb(-0.04f, 2.96f, 6.96f, 0.04f, 3.04f, 7.04f);

        empty.setMargin(0.11f);
        empty.boundingBox(location, rotation, aabb);
        checkAabb(-0.11f, 2.89f, 6.89f, 0.11f, 3.11f, 7.11f);

        empty.setScale(10f);
        empty.boundingBox(location, rotation, aabb);
        checkAabb(-0.11f, 2.89f, 6.89f, 0.11f, 3.11f, 7.11f);

        // HullCollisionShape
        Vector3f halfExtents = new Vector3f(1f, 2f, 3f);
        RectangularSolid rectangularSolid = new RectangularSolid(halfExtents);
        CollisionShape hull = new HullCollisionShape(rectangularSolid);
        rotation = Quaternion.IDENTITY;
        hull.boundingBox(location, rotation, aabb);
        checkAabb(-1.08f, 0.92f, 3.92f, 1.08f, 5.08f, 10.08f); // double margin

        hull.setMargin(0.55f);
        hull.boundingBox(location, rotation, aabb);
        checkAabb(-2.1f, -0.1f, 2.9f, 2.1f, 6.1f, 11.1f); // double margin

        // SphereCollisionShape
        CollisionShape sphere = new SphereCollisionShape(5f);

        location = new Vector3f(0.5f, 0f, -0.4f);
        rotation = Quaternion.IDENTITY;
        sphere.boundingBox(location, rotation, aabb);
        checkAabb(-4.5f, -5f, -5.4f, 5.5f, 5f, 4.6f);

        location = Vector3f.ZERO;
        rotation = new Quaternion().fromAngleAxis(0.6f, Vector3f.UNIT_Y);
        sphere.boundingBox(location, rotation, aabb);
        checkAabb(-5f, -5f, -5f, 5f, 5f, 5f);
    }
    // *************************************************************************
    // private methods

    private static void checkAabb(float xMin, float yMin, float zMin,
            float xMax, float yMax, float zMax) {
        Vector3f min = aabb.getMin(null);
        Utils.assertEquals(xMin, yMin, zMin, min, 1e-5f);

        Vector3f max = aabb.getMax(null);
        Utils.assertEquals(xMax, yMax, zMax, max, 1e-5f);
    }
}
