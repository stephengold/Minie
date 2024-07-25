/*
 Copyright (c) 2018-2024 Stephen Gold
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

import com.github.stephengold.shapes.custom.CustomEllipsoid;
import com.jme3.asset.AssetManager;
import com.jme3.asset.DesktopAssetManager;
import com.jme3.asset.ModelKey;
import com.jme3.asset.plugins.ClasspathLocator;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.Box2dShape;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.ConeCollisionShape;
import com.jme3.bullet.collision.shapes.ConicalFrustum;
import com.jme3.bullet.collision.shapes.Convex2dShape;
import com.jme3.bullet.collision.shapes.ConvexShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.EmptyShape;
import com.jme3.bullet.collision.shapes.GImpactCollisionShape;
import com.jme3.bullet.collision.shapes.HeightfieldCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.MeshCollisionShape;
import com.jme3.bullet.collision.shapes.MinkowskiSum;
import com.jme3.bullet.collision.shapes.MultiSphere;
import com.jme3.bullet.collision.shapes.PlaneCollisionShape;
import com.jme3.bullet.collision.shapes.SimplexCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.collision.shapes.SphericalSegment;
import com.jme3.export.binary.BinaryLoader;
import com.jme3.material.plugins.J3MLoader;
import com.jme3.math.Plane;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.system.NativeLibraryLoader;
import com.jme3.terrain.heightmap.HeightMap;
import com.jme3.terrain.heightmap.ImageBasedHeightMap;
import com.jme3.texture.Image;
import com.jme3.texture.Texture;
import com.jme3.texture.plugins.AWTLoader;
import java.util.ArrayList;
import java.util.List;
import jme3utilities.MyAsset;
import org.junit.Assert;
import org.junit.Test;

/**
 * Test the setMargin() function on collision shapes of all types.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestSetMargin {
    // *************************************************************************
    // new methods exposed

    /**
     * Test the setMargin() function on collision shapes of all types.
     */
    @Test
    public void testSetMargin() { // TODO split off testSetMarginConvex()
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);

        AssetManager assetManager = new DesktopAssetManager();
        assetManager.registerLoader(AWTLoader.class, "jpg", "png");
        assetManager.registerLoader(BinaryLoader.class, "j3o");
        assetManager.registerLoader(J3MLoader.class, "j3m", "j3md");
        assetManager.registerLocator(null, ClasspathLocator.class);

        // Box2dShape
        CollisionShape box2d = new Box2dShape(1f, 2f);
        Assert.assertEquals(0.04f, box2d.getMargin(), 0f);
        box2d.setMargin(0.1f);
        Assert.assertEquals(0.1f, box2d.getMargin(), 0f);

        // BoxCollisionShape
        CollisionShape box = new BoxCollisionShape(1f);
        Assert.assertEquals(0.04f, box.getMargin(), 0f);
        box.setMargin(0.11f);
        Assert.assertEquals(0.11f, box.getMargin(), 0f);

        // CapsuleCollisionShape
        CollisionShape capsule = new CapsuleCollisionShape(1f, 1f);
        Assert.assertEquals(0f, capsule.getMargin(), 0f);
        capsule.setMargin(0.12f); // cannot alter margin
        Assert.assertEquals(0f, capsule.getMargin(), 0f);

        // CompoundCollisionShape of a capsule
        CompoundCollisionShape compound = new CompoundCollisionShape(1);
        compound.addChildShape(capsule, 0f, 1f, 0f);
        Assert.assertEquals(0.04f, compound.getMargin(), 0f);
        compound.setMargin(0.13f);
        Assert.assertEquals(0.13f, compound.getMargin(), 0f);

        // ConeCollisionShape
        CollisionShape cone = new ConeCollisionShape(1f, 1f);
        Assert.assertEquals(0.04f, cone.getMargin(), 0f);
        cone.setMargin(0.14f);
        Assert.assertEquals(0.14f, cone.getMargin(), 0f);

        // ConicalFrustum:
        ConicalFrustum frustum = new ConicalFrustum(1f, 2f, 3f);
        Assert.assertEquals(0.04f, frustum.getMargin(), 0.04f);
        frustum.setMargin(0.142f);
        Assert.assertEquals(0.142f, frustum.getMargin(), 0f);

        // Convex2dShape of a cone
        ConeCollisionShape flatCone
                = new ConeCollisionShape(10f, 0f, PhysicsSpace.AXIS_Z);
        Convex2dShape convex2d = new Convex2dShape(flatCone);
        Assert.assertEquals(0.04f, convex2d.getMargin(), 0f);
        convex2d.setMargin(0.145f);
        Assert.assertEquals(0.145f, convex2d.getMargin(), 0f);

        // CustomEllipsoid
        CollisionShape ellipsoid = new CustomEllipsoid(1f, 2f, 3f, 0.2f);
        Assert.assertEquals(0.04f, ellipsoid.getMargin(), 0f);
        ellipsoid.setMargin(0.15f);
        Assert.assertEquals(0.15f, ellipsoid.getMargin(), 0f);

        // CylinderCollisionShape
        CollisionShape cylinder
                = new CylinderCollisionShape(new Vector3f(1f, 1f, 1f));
        Assert.assertEquals(0.04f, cylinder.getMargin(), 0f);
        cylinder.setMargin(0.15f);
        Assert.assertEquals(0.15f, cylinder.getMargin(), 0f);

        // EmptyShape
        CollisionShape empty = new EmptyShape(true);
        Assert.assertEquals(0.04f, empty.getMargin(), 0f);
        empty.setMargin(0.155f);
        Assert.assertEquals(0.155f, empty.getMargin(), 0f);

        // GImpactCollisionShape
        ModelKey key = new ModelKey("Models/Jaime/Jaime.j3o");
        Node model = (Node) assetManager.loadModel(key);
        Geometry geo = (Geometry) model.getChild(0);
        Mesh mesh = geo.getMesh();
        CollisionShape gimpact = new GImpactCollisionShape(mesh);
        Assert.assertEquals(0.04f, gimpact.getMargin(), 0f);
        gimpact.setMargin(0.16f);
        Assert.assertEquals(0.16f, gimpact.getMargin(), 0f);

        // HeightfieldCollisionShape
        Texture heightTexture = MyAsset.loadTexture(
                assetManager, "Textures/BumpMapTest/Simple_height.png", false);
        Image heightImage = heightTexture.getImage();
        float heightScale = 1f;
        HeightMap heightMap = new ImageBasedHeightMap(heightImage, heightScale);
        CollisionShape hcs = new HeightfieldCollisionShape(heightMap);
        Assert.assertEquals(0.04f, hcs.getMargin(), 0f);
        hcs.setMargin(0.17f);
        Assert.assertEquals(0.17f, hcs.getMargin(), 0f);

        // HullCollisionShape
        CollisionShape hull = new HullCollisionShape(mesh);
        Assert.assertEquals(0.04f, hull.getMargin(), 0f);
        hull.setMargin(0.18f);
        Assert.assertEquals(0.18f, hull.getMargin(), 0f);

        // MeshCollisionShape
        CollisionShape mcs = new MeshCollisionShape(mesh);
        Assert.assertEquals(0.04f, mcs.getMargin(), 0f);
        mcs.setMargin(0.19f);
        Assert.assertEquals(0.19f, mcs.getMargin(), 0f);

        // MinkowskiSum of cone + box
        ConvexShape cone1 = new ConeCollisionShape(1f, 1f);
        ConvexShape box1 = new BoxCollisionShape(1f);
        CollisionShape sum = new MinkowskiSum(cone1, box1);
        Assert.assertEquals(0.08f, sum.getMargin(), 0f);
        sum.setMargin(0.194f); // cannot directly alter margin
        Assert.assertEquals(0.08f, sum.getMargin(), 0f);

        // MinkowskiSum of sphere + cone
        ConvexShape sphere1 = new SphereCollisionShape(1f);
        CollisionShape sum1 = new MinkowskiSum(sphere1, cone1);
        Assert.assertEquals(1.04f, sum1.getMargin(), 0f);
        sum1.setMargin(0.195f); // cannot directly alter margin
        Assert.assertEquals(1.04f, sum1.getMargin(), 0f);

        // MinkowskiSum of sphere + capsule
        ConvexShape capsule1 = new CapsuleCollisionShape(1f, 1f);
        CollisionShape sum2 = new MinkowskiSum(sphere1, capsule1);
        Assert.assertEquals(2f, sum2.getMargin(), 0f);
        sum2.setMargin(0.196f); // cannot directly alter margin
        Assert.assertEquals(2f, sum2.getMargin(), 0f);

        // MultiSphere
        List<Float> radii = new ArrayList<>(3);
        List<Vector3f> centers = new ArrayList<>(3);
        radii.add(0.5f);
        centers.add(new Vector3f(0f, 1f, 0f));
        radii.add(1f);
        centers.add(new Vector3f(0f, 0f, 0f));
        radii.add(0.5f);
        centers.add(new Vector3f(1f, 0f, 0f));
        CollisionShape multiSphere = new MultiSphere(centers, radii);
        Assert.assertEquals(0.04f, multiSphere.getMargin(), 0f);
        multiSphere.setMargin(0.20f);
        Assert.assertEquals(0.20f, multiSphere.getMargin(), 0f);

        // PlaneCollisionShape
        Plane plane = new Plane(new Vector3f(0f, 1f, 0f), 0f);
        CollisionShape pcs = new PlaneCollisionShape(plane);
        Assert.assertEquals(0.04f, pcs.getMargin(), 0f);
        pcs.setMargin(0.21f);
        Assert.assertEquals(0.21f, pcs.getMargin(), 0f);

        // SimplexCollisionShape of 3 vertices
        Vector3f p1 = new Vector3f(0f, 1f, 1f);
        Vector3f p2 = new Vector3f(1f, 0f, 1f);
        Vector3f p3 = new Vector3f(1f, 1f, 0f);
        CollisionShape simplex = new SimplexCollisionShape(p1, p2, p3);
        Assert.assertEquals(0.04f, simplex.getMargin(), 0f);
        simplex.setMargin(0.22f);
        Assert.assertEquals(0.22f, simplex.getMargin(), 0f);

        // SphereCollisionShape
        CollisionShape sphere = new SphereCollisionShape(1f);
        Assert.assertEquals(0f, sphere.getMargin(), 0f);
        sphere.setMargin(0.3f); // cannot alter margin
        Assert.assertEquals(0f, sphere.getMargin(), 0f);

        // SphericalSegment
        SphericalSegment segment = new SphericalSegment(1f);
        Assert.assertEquals(0.04f, segment.getMargin(), 0.04f);
        segment.setMargin(0.31f);
        Assert.assertEquals(0.31f, segment.getMargin(), 0f);
    }
}
