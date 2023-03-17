/*
 Copyright (c) 2018-2023, Stephen Gold
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
import com.jme3.bullet.collision.shapes.Box2dShape;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.ConeCollisionShape;
import com.jme3.bullet.collision.shapes.Convex2dShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.EmptyShape;
import com.jme3.bullet.collision.shapes.GImpactCollisionShape;
import com.jme3.bullet.collision.shapes.HeightfieldCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.MeshCollisionShape;
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
import org.junit.Test;

/**
 * Test cloning/saving/loading collision shapes of all types. TODO replace
 * asserts with JUnit Assert
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestCloneShapes {
    // *************************************************************************
    // fields

    /**
     * AssetManager to load Jaime, also required by the BinaryImporter
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

        // CompoundCollisionShape
        CompoundCollisionShape compound = new CompoundCollisionShape(1);
        CollisionShape capsule = new CapsuleCollisionShape(1f, 1f);
        compound.addChildShape(capsule, 0f, 1f, 0f);
        setParameters(compound, 0f);
        verifyParameters(compound, 0f);
        CollisionShape compoundClone = Heart.deepCopy(compound);
        cloneTest(compound, compoundClone);
        assert compoundClone.getMargin() == 0.04f;
        compound.setMargin(0.13f);
        assert compoundClone.getMargin() == 0.04f;
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
        assert emptyClone.nativeId() != empty.nativeId();
        assert emptyClone.getMargin() == 0.04f;
        empty.setMargin(0.155f);
        assert emptyClone.getMargin() == 0.04f;

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
        assert gimpactClone.getMargin() == 0.04f;
        gimpact.setMargin(0.16f);
        assert gimpactClone.getMargin() == 0.04f;

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
        assert hcsClone.getMargin() == 0.04f;
        hcs.setMargin(0.17f);
        assert hcsClone.getMargin() == 0.04f;

        // MeshCollisionShape with quantized AABB compression
        CollisionShape mcs = new MeshCollisionShape(mesh, true);
        setParameters(mcs, 0f);
        verifyParameters(mcs, 0f);
        CollisionShape mcsClone = Heart.deepCopy(mcs);
        cloneTest(mcs, mcsClone);
        assert mcsClone.getMargin() == 0.04f;
        mcs.setMargin(0.19f);
        assert mcsClone.getMargin() == 0.04f;

        // MeshCollisionShape without compression
        CollisionShape uncompressed = new MeshCollisionShape(mesh, false);
        setParameters(uncompressed, 0f);
        verifyParameters(uncompressed, 0f);
        CollisionShape uncompressedClone = Heart.deepCopy(uncompressed);
        cloneTest(uncompressed, uncompressedClone);
        assert uncompressedClone.getMargin() == 0.04f;
        uncompressed.setMargin(0.191f);
        assert uncompressedClone.getMargin() == 0.04f;

        // PlaneCollisionShape
        Plane plane = new Plane(new Vector3f(0f, 1f, 0f), 0f);
        CollisionShape pcs = new PlaneCollisionShape(plane);
        setParameters(pcs, 0f);
        verifyParameters(pcs, 0f);
        CollisionShape pcsClone = Heart.deepCopy(pcs);
        cloneTest(pcs, pcsClone);
        assert pcsClone.getMargin() == 0.04f;
        pcs.setMargin(0.21f);
        assert pcsClone.getMargin() == 0.04f;
    }

    private static void cloneShapesConvex() {
        // Box2dShape
        CollisionShape box2d = new Box2dShape(1f, 2f);
        setParameters(box2d, 0f);
        verifyParameters(box2d, 0f);
        CollisionShape box2dClone = Heart.deepCopy(box2d);
        cloneTest(box2d, box2dClone);
        assert box2dClone.getMargin() == 0.04f;
        box2d.setMargin(0.11f);
        assert box2dClone.getMargin() == 0.04f;

        // BoxCollisionShape
        CollisionShape box = new BoxCollisionShape(1f);
        setParameters(box, 0f);
        verifyParameters(box, 0f);
        CollisionShape boxClone = Heart.deepCopy(box);
        cloneTest(box, boxClone);
        assert boxClone.getMargin() == 0.04f;
        box.setMargin(0.11f);
        assert boxClone.getMargin() == 0.04f;

        // CapsuleCollisionShape
        CollisionShape capsule = new CapsuleCollisionShape(1f, 1f);
        setParameters(capsule, 0f);
        verifyParameters(capsule, 0f);
        CollisionShape capsuleClone = Heart.deepCopy(capsule);
        cloneTest(capsule, capsuleClone);
        assert capsuleClone.getScale(null).x == 1f;
        capsule.setScale(2f);
        assert capsuleClone.getScale(null).x == 1f;

        // ConeCollisionShape
        ConeCollisionShape cone = new ConeCollisionShape(1f, 1f);
        setParameters(cone, 0f);
        verifyParameters(cone, 0f);
        CollisionShape coneClone = Heart.deepCopy(cone);
        cloneTest(cone, coneClone);
        assert coneClone.getMargin() == 0.04f;
        cone.setMargin(0.14f);
        assert coneClone.getMargin() == 0.04f;

        // Convex2dShape
        CollisionShape convex2d = new Convex2dShape(cone);
        setParameters(convex2d, 0f);
        verifyParameters(convex2d, 0f);
        CollisionShape convex2dClone = Heart.deepCopy(convex2d);
        cloneTest(convex2d, convex2dClone);
        assert convex2dClone.getMargin() == 0.04f;
        convex2d.setMargin(0.14f);
        assert convex2dClone.getMargin() == 0.04f;

        // CylinderCollisionShape
        CollisionShape cylinder
                = new CylinderCollisionShape(new Vector3f(1f, 1f, 1f));
        setParameters(cylinder, 0f);
        verifyParameters(cylinder, 0f);
        CollisionShape cylinderClone = Heart.deepCopy(cylinder);
        cloneTest(cylinder, cylinderClone);
        assert cylinderClone.getMargin() == 0.04f;
        cylinder.setMargin(0.15f);
        assert cylinderClone.getMargin() == 0.04f;

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
        assert hullClone.getMargin() == 0.04f;
        hull.setMargin(0.18f);
        assert hullClone.getMargin() == 0.04f;

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
        assert multiSphereClone.getMargin() == 0.04f;
        multiSphere.setMargin(0.20f);
        assert multiSphereClone.getMargin() == 0.04f;

        // SimplexCollisionShape of 1 vertex
        Vector3f p1 = new Vector3f(0f, 1f, 1f);
        CollisionShape simplex1 = new SimplexCollisionShape(p1);
        setParameters(simplex1, 0f);
        verifyParameters(simplex1, 0f);
        CollisionShape simplex1Clone = Heart.deepCopy(simplex1);
        cloneTest(simplex1, simplex1Clone);
        assert simplex1Clone.getMargin() == 0.04f;
        simplex1.setMargin(0.22f);
        assert simplex1Clone.getMargin() == 0.04f;

        // SimplexCollisionShape of 2 vertices
        Vector3f p2 = new Vector3f(1f, 0f, 1f);
        CollisionShape simplex2 = new SimplexCollisionShape(p1, p2);
        setParameters(simplex2, 0f);
        verifyParameters(simplex2, 0f);
        CollisionShape simplex2Clone = Heart.deepCopy(simplex2);
        cloneTest(simplex2, simplex2Clone);
        assert simplex2Clone.getMargin() == 0.04f;
        simplex2.setMargin(0.22f);
        assert simplex2Clone.getMargin() == 0.04f;

        // SimplexCollisionShape of 3 vertices
        Vector3f p3 = new Vector3f(1f, 1f, 0f);
        CollisionShape simplex3 = new SimplexCollisionShape(p1, p2, p3);
        setParameters(simplex3, 0f);
        verifyParameters(simplex3, 0f);
        CollisionShape simplex3Clone = Heart.deepCopy(simplex3);
        cloneTest(simplex3, simplex3Clone);
        assert simplex3Clone.getMargin() == 0.04f;
        simplex3.setMargin(0.22f);
        assert simplex3Clone.getMargin() == 0.04f;

        // SimplexCollisionShape of 4 vertices
        Vector3f p4 = new Vector3f(-1f, -1f, -1f);
        CollisionShape simplex4 = new SimplexCollisionShape(p1, p2, p3, p4);
        setParameters(simplex4, 0f);
        verifyParameters(simplex4, 0f);
        CollisionShape simplex4Clone = Heart.deepCopy(simplex4);
        cloneTest(simplex4, simplex4Clone);
        assert simplex4Clone.getMargin() == 0.04f;
        simplex4.setMargin(0.22f);
        assert simplex4Clone.getMargin() == 0.04f;

        // SphereCollisionShape
        CollisionShape sphere = new SphereCollisionShape(1f);
        setParameters(sphere, 0f);
        verifyParameters(sphere, 0f);
        CollisionShape sphereClone = Heart.deepCopy(sphere);
        cloneTest(sphere, sphereClone);
        assert sphereClone.getScale(null).x == 1f;
        sphere.setScale(2f);
        assert sphereClone.getScale(null).x == 1f;
    }

    private static void cloneTest(
            CollisionShape shape, CollisionShape shapeClone) {
        //logger.log(Level.SEVERE, "{0}", shape.getClass());
        assert shapeClone.getClass() == shape.getClass();
        assert shapeClone.nativeId() != shape.nativeId();

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
        verifyParameters(shapeCopy, 0.3f);
        assert shapeCopy.getMargin() == shape.getMargin();

        CollisionShape shapeCloneCopy
                = BinaryExporter.saveAndLoad(assetManager, shapeClone);
        verifyParameters(shapeCloneCopy, 0.6f);
        assert shapeCloneCopy.getMargin() == shapeClone.getMargin();

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
        assert shape.isContactFilterEnabled() == flag;
    }
}
