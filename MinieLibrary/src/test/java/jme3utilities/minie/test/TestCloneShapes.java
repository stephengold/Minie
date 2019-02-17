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
import com.jme3.asset.plugins.ClasspathLocator;
import com.jme3.asset.plugins.FileLocator;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.ConeCollisionShape;
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
import com.jme3.bullet.control.GhostControl;
import com.jme3.export.JmeExporter;
import com.jme3.export.binary.BinaryExporter;
import com.jme3.export.binary.BinaryLoader;
import com.jme3.material.plugins.J3MLoader;
import com.jme3.math.Plane;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.system.NativeLibraryLoader;
import com.jme3.terrain.heightmap.AbstractHeightMap;
import com.jme3.terrain.heightmap.ImageBasedHeightMap;
import com.jme3.texture.Image;
import com.jme3.texture.Texture;
import com.jme3.texture.plugins.AWTLoader;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import jme3utilities.Misc;
import jme3utilities.MyAsset;
import org.junit.Test;

/**
 * Test cloning/saving/loading on collision shapes of all types.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestCloneShapes {
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
    public void testCloneShapes() {
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);

        assetManager = new DesktopAssetManager();
        assetManager.registerLoader(AWTLoader.class, "jpg", "png");
        assetManager.registerLoader(BinaryLoader.class, "j3o");
        assetManager.registerLoader(J3MLoader.class, "j3m", "j3md");
        assetManager.registerLocator(".", FileLocator.class);
        assetManager.registerLocator(null, ClasspathLocator.class);
        /*
         * Box
         */
        CollisionShape box = new BoxCollisionShape(new Vector3f(1f, 1f, 1f));
        setParameters(box, 0f);
        verifyParameters(box, 0f);
        CollisionShape boxClone = (CollisionShape) Misc.deepCopy(box);
        cloneTest(box, boxClone);
        assert boxClone.getMargin() == 0.04f;
        box.setMargin(0.11f);
        assert boxClone.getMargin() == 0.04f;
        /*
         * Capsule
         */
        CollisionShape capsule = new CapsuleCollisionShape(1f, 1f);
        setParameters(capsule, 0f);
        verifyParameters(capsule, 0f);
        CollisionShape capsuleClone = (CollisionShape) Misc.deepCopy(capsule);
        cloneTest(capsule, capsuleClone);
        assert capsuleClone.getScale(null).x == 1f;
        capsule.setScale(new Vector3f(2f, 2f, 2f));
        assert capsuleClone.getScale(null).x == 1f;
        /*
         * Compound
         */
        CompoundCollisionShape compound = new CompoundCollisionShape();
        compound.addChildShape(capsule, new Vector3f(0f, 1f, 0f));
        setParameters(compound, 0f);
        verifyParameters(compound, 0f);
        CollisionShape compoundClone = (CollisionShape) Misc.deepCopy(compound);
        cloneTest(compound, compoundClone);
        assert compoundClone.getMargin() == 0.04f;
        compound.setMargin(0.13f);
        assert compoundClone.getMargin() == 0.04f;
        /*
         * Cone
         */
        CollisionShape cone = new ConeCollisionShape(1f, 1f);
        setParameters(cone, 0f);
        verifyParameters(cone, 0f);
        CollisionShape coneClone = (CollisionShape) Misc.deepCopy(cone);
        cloneTest(cone, coneClone);
        assert coneClone.getMargin() == 0.04f;
        cone.setMargin(0.14f);
        assert coneClone.getMargin() == 0.04f;
        /*
         * Cylinder
         */
        CollisionShape cylinder
                = new CylinderCollisionShape(new Vector3f(1f, 1f, 1f));
        setParameters(cylinder, 0f);
        verifyParameters(cylinder, 0f);
        CollisionShape cylinderClone = (CollisionShape) Misc.deepCopy(cylinder);
        cloneTest(cylinder, cylinderClone);
        assert cylinderClone.getMargin() == 0.04f;
        cylinder.setMargin(0.15f);
        assert cylinderClone.getMargin() == 0.04f;
        /*
         * Empty
         */
        CollisionShape empty = new EmptyShape(true);
        setParameters(empty, 0f);
        verifyParameters(empty, 0f);
        CollisionShape emptyClone = (CollisionShape) Misc.deepCopy(empty);
        cloneTest(empty, emptyClone);
        assert emptyClone.getObjectId() != empty.getObjectId();
        assert emptyClone.getMargin() == 0.04f;
        empty.setMargin(0.155f);
        assert emptyClone.getMargin() == 0.04f;
        /*
         * GImpact
         */
        ModelKey key = new ModelKey("Models/Jaime/Jaime.j3o");
        Node model = (Node) assetManager.loadModel(key);
        Geometry geo = (Geometry) model.getChild(0);
        Mesh mesh = geo.getMesh();
        CollisionShape gimpact = new GImpactCollisionShape(mesh);
        setParameters(gimpact, 0f);
        verifyParameters(gimpact, 0f);
        CollisionShape gimpactClone = (CollisionShape) Misc.deepCopy(gimpact);
        cloneTest(gimpact, gimpactClone);
        assert gimpactClone.getMargin() == 0.04f;
        gimpact.setMargin(0.16f);
        assert gimpactClone.getMargin() == 0.04f;
        /*
         * Heightfield
         */
        Texture heightTexture = MyAsset.loadTexture(assetManager,
                "Textures/terrain/height/basin.png");
        Image heightImage = heightTexture.getImage();
        float heightScale = 1f;
        AbstractHeightMap heightMap
                = new ImageBasedHeightMap(heightImage, heightScale);
        heightMap.load();
        float[] heightArray = heightMap.getHeightMap();
        CollisionShape hcs = new HeightfieldCollisionShape(heightArray);
        setParameters(hcs, 0f);
        verifyParameters(hcs, 0f);
        CollisionShape hcsClone = (CollisionShape) Misc.deepCopy(hcs);
        cloneTest(hcs, hcsClone);
        assert hcsClone.getMargin() == 0.04f;
        hcs.setMargin(0.17f);
        assert hcsClone.getMargin() == 0.04f;
        /*
         * Hull
         */
        CollisionShape hull = new HullCollisionShape(mesh);
        setParameters(hull, 0f);
        verifyParameters(hull, 0f);
        CollisionShape hullClone = (CollisionShape) Misc.deepCopy(hull);
        cloneTest(hull, hullClone);
        assert hullClone.getMargin() == 0.04f;
        hull.setMargin(0.17f);
        assert hullClone.getMargin() == 0.04f;
        /*
         * Mesh
         */
        CollisionShape mcs = new MeshCollisionShape(mesh);
        setParameters(mcs, 0f);
        verifyParameters(mcs, 0f);
        CollisionShape mcsClone = (CollisionShape) Misc.deepCopy(mcs);
        cloneTest(mcs, mcsClone);
        assert mcsClone.getMargin() == 0.04f;
        mcs.setMargin(0.19f);
        assert mcsClone.getMargin() == 0.04f;
        /*
         * Multisphere
         */
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
        CollisionShape multiSphereClone
                = (CollisionShape) Misc.deepCopy(multiSphere);
        cloneTest(multiSphere, multiSphereClone);
        assert multiSphereClone.getMargin() == 0.04f;
        multiSphere.setMargin(0.20f);
        assert multiSphereClone.getMargin() == 0.04f;
        /*
         * Plane
         */
        Plane plane = new Plane(new Vector3f(0f, 1f, 0f), 0f);
        CollisionShape pcs = new PlaneCollisionShape(plane);
        setParameters(pcs, 0f);
        verifyParameters(pcs, 0f);
        CollisionShape pcsClone = (CollisionShape) Misc.deepCopy(pcs);
        cloneTest(pcs, pcsClone);
        assert pcsClone.getMargin() == 0.04f;
        pcs.setMargin(0.21f);
        assert pcsClone.getMargin() == 0.04f;
        /*
         * Simplex
         */
        Vector3f p1 = new Vector3f(0f, 1f, 1f);
        Vector3f p2 = new Vector3f(1f, 0f, 1f);
        Vector3f p3 = new Vector3f(1f, 1f, 0f);
        CollisionShape simplex = new SimplexCollisionShape(p1, p2, p3);
        setParameters(simplex, 0f);
        verifyParameters(simplex, 0f);
        CollisionShape simplexClone = (CollisionShape) Misc.deepCopy(simplex);
        cloneTest(simplex, simplexClone);
        assert simplexClone.getMargin() == 0.04f;
        simplex.setMargin(0.22f);
        assert simplexClone.getMargin() == 0.04f;
        /*
         * Sphere
         */
        CollisionShape sphere = new SphereCollisionShape(1f);
        setParameters(sphere, 0f);
        verifyParameters(sphere, 0f);
        CollisionShape sphereClone = (CollisionShape) Misc.deepCopy(sphere);
        cloneTest(sphere, sphereClone);
        assert sphereClone.getScale(null).x == 1f;
        sphere.setScale(new Vector3f(2f, 2f, 2f));
        assert sphereClone.getScale(null).x == 1f;
    }
    // *************************************************************************
    // private methods

    private void cloneTest(CollisionShape shape, CollisionShape shapeClone) {
        //logger.log(Level.SEVERE, "{0}", shape.getClass());
        assert shapeClone.getClass() == shape.getClass();
        assert shapeClone.getObjectId() != shape.getObjectId();

        verifyParameters(shape, 0f);
        verifyParameters(shapeClone, 0f);

        setParameters(shape, 0.3f);
        verifyParameters(shape, 0.3f);
        verifyParameters(shapeClone, 0f);

        setParameters(shapeClone, 0.6f);
        verifyParameters(shape, 0.3f);
        verifyParameters(shapeClone, 0.6f);

        if (shape instanceof GImpactCollisionShape
                || shape instanceof MeshCollisionShape) { // TODO
            return;
        }

        CollisionShape shapeCopy = saveThenLoad(shape);
        verifyParameters(shapeCopy, 0.3f);

        CollisionShape shapeCloneCopy = saveThenLoad(shapeClone);
        verifyParameters(shapeCloneCopy, 0.6f);
    }

    /**
     * Clone a CollisionShape by saving and then loading it.
     *
     * @param shape the shape to copy (not null, unaffected)
     * @return a new shape
     */
    private CollisionShape saveThenLoad(CollisionShape shape) {
        Node savedNode = new Node();
        GhostControl savedControl = new GhostControl(shape);
        savedNode.addControl(savedControl);

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
        GhostControl loadedSgc = (GhostControl) loadedNode.getControl(0);
        CollisionShape loadedShape = loadedSgc.getCollisionShape();

        return loadedShape;
    }

    /**
     * Modify CollisionShape parameters based on the specified key value.
     *
     * @param shape the collision shape to modify (not null)
     * @param b the key value
     */
    private void setParameters(CollisionShape shape, float b) {
    }

    /**
     * Verify that all CollisionShape parameters have their expected values for
     * the specified key value.
     *
     * @param shape the collision shape to verify (not null, unaffected)
     * @param b the key value
     */
    private void verifyParameters(CollisionShape shape, float b) {
    }
}
