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
import com.jme3.bullet.collision.shapes.Box2dShape;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.ConeCollisionShape;
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
import com.jme3.export.binary.BinaryExporter;
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
import jme3utilities.Heart;
import jme3utilities.MyAsset;
import org.junit.Assert;
import org.junit.Test;

/**
 * Test cloning/saving/loading collision shapes of all types.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestCloneShapes {
    // *************************************************************************
    // fields

    /**
     * AssetManager to load Jaime and Simple_height.png, also for
     * {@code BinaryExporter.saveAndLoad()}
     */
    final private static AssetManager assetManager = new DesktopAssetManager();
    // *************************************************************************
    // new methods exposed

    /**
     * Test cloning/saving/loading collision shapes of all types.
     */
    @Test
    public void testCloneShapes() {
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);
        assetManager.registerLoader(AWTLoader.class, "jpg", "png");
        assetManager.registerLoader(BinaryLoader.class, "j3o");
        assetManager.registerLoader(J3MLoader.class, "j3m", "j3md");
        assetManager.registerLocator(null, ClasspathLocator.class);

        cloneShapesConcave();
        cloneShapesConvex();

        // CompoundCollisionShape of a capsule
        CompoundCollisionShape compound = new CompoundCollisionShape(1);
        CollisionShape capsule = new CapsuleCollisionShape(1f, 1f);
        compound.addChildShape(capsule, 0f, 1f, 0f);
        setParameters(compound, 0f);
        verifyParameters(compound, 0f);
        CollisionShape compoundClone = Heart.deepCopy(compound);
        cloneTest(compound, compoundClone);
        Assert.assertEquals(0.04f, compoundClone.getMargin(), 0f);
        compound.setMargin(0.13f);
        Assert.assertEquals(0.04f, compoundClone.getMargin(), 0f);
    }
    // *************************************************************************
    // private methods

    private static void cloneShapesConcave() {
        // EmptyShape
        CollisionShape empty = new EmptyShape(true);
        setParameters(empty, 0f);
        verifyParameters(empty, 0f);
        CollisionShape emptyClone = Heart.deepCopy(empty);
        cloneTest(empty, emptyClone);
        Assert.assertEquals(0.04f, emptyClone.getMargin(), 0f);
        empty.setMargin(0.155f);
        Assert.assertEquals(0.04f, emptyClone.getMargin(), 0f);

        // GImpactCollisionShape
        ModelKey key = new ModelKey("Models/Jaime/Jaime.j3o");
        Node model = (Node) assetManager.loadModel(key);
        Geometry geo = (Geometry) model.getChild(0);
        Mesh mesh = geo.getMesh();
        CollisionShape gimpact = new GImpactCollisionShape(mesh);
        setParameters(gimpact, 0f);
        verifyParameters(gimpact, 0f);
        CollisionShape gimpactClone = Heart.deepCopy(gimpact);
        cloneTest(gimpact, gimpactClone);
        Assert.assertEquals(0.04f, gimpactClone.getMargin(), 0f);
        gimpact.setMargin(0.16f);
        Assert.assertEquals(0.04f, gimpactClone.getMargin(), 0f);

        // HeightfieldCollisionShape
        Texture heightTexture = MyAsset.loadTexture(
                assetManager, "Textures/BumpMapTest/Simple_height.png", false);
        Image heightImage = heightTexture.getImage();
        float heightScale = 1f;
        HeightMap heightMap = new ImageBasedHeightMap(heightImage, heightScale);
        CollisionShape hcs = new HeightfieldCollisionShape(heightMap);
        setParameters(hcs, 0f);
        verifyParameters(hcs, 0f);
        CollisionShape hcsClone = Heart.deepCopy(hcs);
        cloneTest(hcs, hcsClone);
        Assert.assertEquals(0.04f, hcsClone.getMargin(), 0f);
        hcs.setMargin(0.17f);
        Assert.assertEquals(0.04f, hcsClone.getMargin(), 0f);

        // MeshCollisionShape with quantized AABB compression
        CollisionShape mcs = new MeshCollisionShape(mesh, true);
        setParameters(mcs, 0f);
        verifyParameters(mcs, 0f);
        CollisionShape mcsClone = Heart.deepCopy(mcs);
        cloneTest(mcs, mcsClone);
        Assert.assertEquals(0.04f, mcsClone.getMargin(), 0f);
        mcs.setMargin(0.19f);
        Assert.assertEquals(0.04f, mcsClone.getMargin(), 0f);

        // MeshCollisionShape without compression
        CollisionShape uncompressed = new MeshCollisionShape(mesh, false);
        setParameters(uncompressed, 0f);
        verifyParameters(uncompressed, 0f);
        CollisionShape uncompressedClone = Heart.deepCopy(uncompressed);
        cloneTest(uncompressed, uncompressedClone);
        Assert.assertEquals(0.04f, uncompressedClone.getMargin(), 0f);
        uncompressed.setMargin(0.191f);
        Assert.assertEquals(0.04f, uncompressedClone.getMargin(), 0f);

        // PlaneCollisionShape
        Plane plane = new Plane(new Vector3f(0f, 1f, 0f), 0f);
        CollisionShape pcs = new PlaneCollisionShape(plane);
        setParameters(pcs, 0f);
        verifyParameters(pcs, 0f);
        CollisionShape pcsClone = Heart.deepCopy(pcs);
        cloneTest(pcs, pcsClone);
        Assert.assertEquals(0.04f, pcsClone.getMargin(), 0f);
        pcs.setMargin(0.21f);
        Assert.assertEquals(0.04f, pcsClone.getMargin(), 0f);
    }

    private static void cloneShapesConvex() {
        // Box2dShape
        CollisionShape box2d = new Box2dShape(1f, 2f);
        setParameters(box2d, 0f);
        verifyParameters(box2d, 0f);
        CollisionShape box2dClone = Heart.deepCopy(box2d);
        cloneTest(box2d, box2dClone);
        Assert.assertEquals(0.04f, box2dClone.getMargin(), 0f);
        box2d.setMargin(0.11f);
        Assert.assertEquals(0.04f, box2dClone.getMargin(), 0f);

        // BoxCollisionShape
        CollisionShape box = new BoxCollisionShape(1f);
        setParameters(box, 0f);
        verifyParameters(box, 0f);
        CollisionShape boxClone = Heart.deepCopy(box);
        cloneTest(box, boxClone);
        Assert.assertEquals(0.04f, boxClone.getMargin(), 0f);
        box.setMargin(0.11f);
        Assert.assertEquals(0.04f, boxClone.getMargin(), 0f);

        // CapsuleCollisionShape
        CollisionShape capsule = new CapsuleCollisionShape(1f, 1f);
        setParameters(capsule, 0f);
        verifyParameters(capsule, 0f);
        CollisionShape capsuleClone = Heart.deepCopy(capsule);
        cloneTest(capsule, capsuleClone);
        Assert.assertEquals(1f, capsuleClone.getScale(null).x, 0f);
        capsule.setScale(2f);
        Assert.assertEquals(1f, capsuleClone.getScale(null).x, 0f);

        // ConeCollisionShape
        ConeCollisionShape cone = new ConeCollisionShape(1f, 1f);
        setParameters(cone, 0f);
        verifyParameters(cone, 0f);
        CollisionShape coneClone = Heart.deepCopy(cone);
        cloneTest(cone, coneClone);
        Assert.assertEquals(0.04f, coneClone.getMargin(), 0f);
        cone.setMargin(0.14f);
        Assert.assertEquals(0.04f, coneClone.getMargin(), 0f);

        // Convex2dShape of a cone
        CollisionShape convex2d = new Convex2dShape(cone);
        setParameters(convex2d, 0f);
        verifyParameters(convex2d, 0f);
        CollisionShape convex2dClone = Heart.deepCopy(convex2d);
        cloneTest(convex2d, convex2dClone);
        Assert.assertEquals(0.04f, convex2dClone.getMargin(), 0f);
        convex2d.setMargin(0.14f);
        Assert.assertEquals(0.04f, convex2dClone.getMargin(), 0f);

        // CustomEllipsoid
        CustomEllipsoid ellipsoid = new CustomEllipsoid(2f, 3f, 4f, 0.2f);
        setParameters(ellipsoid, 0f);
        verifyParameters(ellipsoid, 0f);
        CollisionShape ellipsoidClone = Heart.deepCopy(ellipsoid);
        cloneTest(ellipsoid, ellipsoidClone);
        Assert.assertEquals(0.04f, ellipsoidClone.getMargin(), 0f);
        ellipsoid.setMargin(0.15f);
        Assert.assertEquals(0.04f, ellipsoidClone.getMargin(), 0f);

        // CylinderCollisionShape
        CollisionShape cylinder
                = new CylinderCollisionShape(new Vector3f(1f, 1f, 1f));
        setParameters(cylinder, 0f);
        verifyParameters(cylinder, 0f);
        CollisionShape cylinderClone = Heart.deepCopy(cylinder);
        cloneTest(cylinder, cylinderClone);
        Assert.assertEquals(0.04f, cylinderClone.getMargin(), 0f);
        cylinder.setMargin(0.15f);
        Assert.assertEquals(0.04f, cylinderClone.getMargin(), 0f);

        // HullCollisionShape
        ModelKey key = new ModelKey("Models/Jaime/Jaime.j3o");
        Node model = (Node) assetManager.loadModel(key);
        Geometry geo = (Geometry) model.getChild(0);
        Mesh mesh = geo.getMesh();
        CollisionShape hull = new HullCollisionShape(mesh);
        setParameters(hull, 0f);
        verifyParameters(hull, 0f);
        CollisionShape hullClone = Heart.deepCopy(hull);
        cloneTest(hull, hullClone);
        Assert.assertEquals(0.04f, hullClone.getMargin(), 0f);
        hull.setMargin(0.18f);
        Assert.assertEquals(0.04f, hullClone.getMargin(), 0f);

        // MinkowskiSum of cone + box
        ConvexShape cone1 = new ConeCollisionShape(1f, 1f);
        ConvexShape box1 = new BoxCollisionShape(1f);
        CollisionShape sum = new MinkowskiSum(cone1, box1);
        setParameters(sum, 0f);
        verifyParameters(sum, 0f);
        CollisionShape sumClone = Heart.deepCopy(sum);
        cloneTest(sum, sumClone);
        Assert.assertEquals(0.08f, sumClone.getMargin(), 0f);
        cone1.setMargin(0.18f);
        Assert.assertEquals(0.08f, sumClone.getMargin(), 0f);

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
        setParameters(multiSphere, 0f);
        verifyParameters(multiSphere, 0f);
        CollisionShape multiSphereClone = Heart.deepCopy(multiSphere);
        cloneTest(multiSphere, multiSphereClone);
        Assert.assertEquals(0.04f, multiSphereClone.getMargin(), 0f);
        multiSphere.setMargin(0.20f);
        Assert.assertEquals(0.04f, multiSphereClone.getMargin(), 0f);

        // SimplexCollisionShape of 1 vertex
        Vector3f p1 = new Vector3f(0f, 1f, 1f);
        CollisionShape simplex1 = new SimplexCollisionShape(p1);
        setParameters(simplex1, 0f);
        verifyParameters(simplex1, 0f);
        CollisionShape simplex1Clone = Heart.deepCopy(simplex1);
        cloneTest(simplex1, simplex1Clone);
        Assert.assertEquals(0.04f, simplex1Clone.getMargin(), 0f);
        simplex1.setMargin(0.22f);
        Assert.assertEquals(0.04f, simplex1Clone.getMargin(), 0f);

        // SimplexCollisionShape of 2 vertices
        Vector3f p2 = new Vector3f(1f, 0f, 1f);
        CollisionShape simplex2 = new SimplexCollisionShape(p1, p2);
        setParameters(simplex2, 0f);
        verifyParameters(simplex2, 0f);
        CollisionShape simplex2Clone = Heart.deepCopy(simplex2);
        cloneTest(simplex2, simplex2Clone);
        Assert.assertEquals(0.04f, simplex2Clone.getMargin(), 0f);
        simplex2.setMargin(0.22f);
        Assert.assertEquals(0.04f, simplex2Clone.getMargin(), 0f);

        // SimplexCollisionShape of 3 vertices
        Vector3f p3 = new Vector3f(1f, 1f, 0f);
        CollisionShape simplex3 = new SimplexCollisionShape(p1, p2, p3);
        setParameters(simplex3, 0f);
        verifyParameters(simplex3, 0f);
        CollisionShape simplex3Clone = Heart.deepCopy(simplex3);
        cloneTest(simplex3, simplex3Clone);
        Assert.assertEquals(0.04f, simplex3Clone.getMargin(), 0f);
        simplex3.setMargin(0.22f);
        Assert.assertEquals(0.04f, simplex3Clone.getMargin(), 0f);

        // SimplexCollisionShape of 4 vertices
        Vector3f p4 = new Vector3f(-1f, -1f, -1f);
        CollisionShape simplex4 = new SimplexCollisionShape(p1, p2, p3, p4);
        setParameters(simplex4, 0f);
        verifyParameters(simplex4, 0f);
        CollisionShape simplex4Clone = Heart.deepCopy(simplex4);
        cloneTest(simplex4, simplex4Clone);
        Assert.assertEquals(0.04f, simplex4Clone.getMargin(), 0f);
        simplex4.setMargin(0.22f);
        Assert.assertEquals(0.04f, simplex4Clone.getMargin(), 0f);

        // SphereCollisionShape
        CollisionShape sphere = new SphereCollisionShape(1f);
        setParameters(sphere, 0f);
        verifyParameters(sphere, 0f);
        CollisionShape sphereClone = Heart.deepCopy(sphere);
        cloneTest(sphere, sphereClone);
        Assert.assertEquals(1f, sphereClone.getScale(null).x, 0f);
        sphere.setScale(2f);
        Assert.assertEquals(1f, sphereClone.getScale(null).x, 0f);
    }

    private static void cloneTest(
            CollisionShape shape, CollisionShape shapeClone) {
        Utils.cloneTest(shape, shapeClone);

        verifyParameters(shape, 0f);
        verifyParameters(shapeClone, 0f);

        setParameters(shape, 0.3f);
        verifyParameters(shape, 0.3f);
        verifyParameters(shapeClone, 0f);

        setParameters(shapeClone, 0.6f);
        verifyParameters(shape, 0.3f);
        verifyParameters(shapeClone, 0.6f);

        CollisionShape.setDefaultMargin(1f);

        CollisionShape shapeCopy
                = BinaryExporter.saveAndLoad(assetManager, shape);
        Assert.assertNotNull(shapeCopy);
        verifyParameters(shapeCopy, 0.3f);
        Assert.assertEquals(shape.getMargin(), shapeCopy.getMargin(), 0f);

        CollisionShape shapeCloneCopy
                = BinaryExporter.saveAndLoad(assetManager, shapeClone);
        Assert.assertNotNull(shapeCloneCopy);
        verifyParameters(shapeCloneCopy, 0.6f);
        Assert.assertEquals(
                shapeClone.getMargin(), shapeCloneCopy.getMargin(), 0f);

        CollisionShape.setDefaultMargin(0.04f);
    }

    /**
     * Modify CollisionShape parameters based on the specified key value.
     *
     * @param shape the collision shape to modify (not null)
     * @param b the key value
     */
    private static void setParameters(CollisionShape shape, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);
        shape.setContactFilterEnabled(flag);
    }

    /**
     * Verify that all CollisionShape parameters have their expected values for
     * the specified key value.
     *
     * @param shape the collision shape to verify (not null, unaffected)
     * @param b the key value
     */
    private static void verifyParameters(CollisionShape shape, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);
        Assert.assertEquals(flag, shape.isContactFilterEnabled());
    }
}
