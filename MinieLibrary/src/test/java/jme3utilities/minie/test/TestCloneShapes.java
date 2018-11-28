/*
 Copyright (c) 2018, Stephen Gold
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
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.ConeCollisionShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.GImpactCollisionShape;
import com.jme3.bullet.collision.shapes.HeightfieldCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.MeshCollisionShape;
import com.jme3.bullet.collision.shapes.MultiSphere;
import com.jme3.bullet.collision.shapes.PlaneCollisionShape;
import com.jme3.bullet.collision.shapes.SimplexCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.export.binary.BinaryLoader;
import com.jme3.material.plugins.J3MLoader;
import com.jme3.math.Plane;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.system.NativeLibraryLoader;
import com.jme3.terrain.heightmap.AbstractHeightMap;
import com.jme3.terrain.heightmap.ImageBasedHeightMap;
import com.jme3.texture.Image;
import com.jme3.texture.Texture;
import com.jme3.texture.plugins.AWTLoader;
import java.util.ArrayList;
import java.util.List;
import jme3utilities.Misc;
import jme3utilities.MyAsset;
import org.junit.Test;

/**
 * Deep clone collision shapes of all types.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestCloneShapes {
    // *************************************************************************
    // new methods exposed

    @Test
    public void testCloneShapes() {
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);

        AssetManager assetManager = new DesktopAssetManager();
        assetManager.registerLoader(AWTLoader.class, "jpg", "png");
        assetManager.registerLoader(BinaryLoader.class, "j3o");
        assetManager.registerLoader(J3MLoader.class, "j3m", "j3md");
        assetManager.registerLocator(null, ClasspathLocator.class);

        CollisionShape box = new BoxCollisionShape(new Vector3f(1f, 1f, 1f));
        CollisionShape boxClone = (CollisionShape) Misc.deepCopy(box);
        assert boxClone.getObjectId() != box.getObjectId();
        assert boxClone.getMargin() == 0.04f;
        box.setMargin(0.11f);
        assert boxClone.getMargin() == 0.04f;

        CollisionShape capsule = new CapsuleCollisionShape(1f, 1f);
        CollisionShape capsuleClone = (CollisionShape) Misc.deepCopy(capsule);
        assert capsuleClone.getObjectId() != capsule.getObjectId();
        assert capsuleClone.getScale(null).x == 1f;
        capsule.setScale(new Vector3f(2f, 2f, 2f));
        assert capsuleClone.getScale(null).x == 1f;

        CompoundCollisionShape compound = new CompoundCollisionShape();
        compound.addChildShape(capsule, new Vector3f(0f, 1f, 0f));
        CollisionShape compoundClone = (CollisionShape) Misc.deepCopy(compound);
        assert compoundClone.getObjectId() != compound.getObjectId();
        assert compoundClone.getMargin() == 0.04f;
        compound.setMargin(0.13f);
        assert compoundClone.getMargin() == 0.04f;

        CollisionShape cone = new ConeCollisionShape(1f, 1f);
        CollisionShape coneClone = (CollisionShape) Misc.deepCopy(cone);
        assert coneClone.getObjectId() != cone.getObjectId();
        assert coneClone.getMargin() == 0.04f;
        cone.setMargin(0.14f);
        assert coneClone.getMargin() == 0.04f;

        CollisionShape cylinder
                = new CylinderCollisionShape(new Vector3f(1f, 1f, 1f));
        CollisionShape cylinderClone = (CollisionShape) Misc.deepCopy(cylinder);
        assert cylinderClone.getObjectId() != cylinder.getObjectId();
        assert cylinderClone.getMargin() == 0.04f;
        cylinder.setMargin(0.15f);
        assert cylinderClone.getMargin() == 0.04f;

        ModelKey key = new ModelKey("Models/Jaime/Jaime.j3o");
        Node model = (Node) assetManager.loadModel(key);
        Geometry geo = (Geometry) model.getChild(0);
        Mesh mesh = geo.getMesh();
        CollisionShape gimpact = new GImpactCollisionShape(mesh);
        CollisionShape gimpactClone = (CollisionShape) Misc.deepCopy(gimpact);
        assert gimpactClone.getObjectId() != gimpact.getObjectId();
        assert gimpactClone.getMargin() == 0.04f;
        gimpact.setMargin(0.16f);
        assert gimpactClone.getMargin() == 0.04f;

        Texture heightTexture = MyAsset.loadTexture(assetManager,
                "Textures/terrain/height/basin.png");
        Image heightImage = heightTexture.getImage();
        float heightScale = 1f;
        AbstractHeightMap heightMap
                = new ImageBasedHeightMap(heightImage, heightScale);
        heightMap.load();
        float[] heightArray = heightMap.getHeightMap();
        CollisionShape hcs = new HeightfieldCollisionShape(heightArray);
        CollisionShape hcsClone = (CollisionShape) Misc.deepCopy(hcs);
        assert hcsClone.getObjectId() != hcs.getObjectId();
        assert hcsClone.getMargin() == 0.04f;
        hcs.setMargin(0.17f);
        assert hcsClone.getMargin() == 0.04f;

        CollisionShape hull = new HullCollisionShape(mesh);
        CollisionShape hullClone = (CollisionShape) Misc.deepCopy(hull);
        assert hullClone.getObjectId() != hull.getObjectId();
        assert hullClone.getMargin() == 0.04f;
        hull.setMargin(0.17f);
        assert hullClone.getMargin() == 0.04f;

        CollisionShape mcs = new MeshCollisionShape(mesh);
        CollisionShape mcsClone = (CollisionShape) Misc.deepCopy(mcs);
        assert mcsClone.getObjectId() != mcs.getObjectId();
        assert mcsClone.getMargin() == 0.04f;
        mcs.setMargin(0.19f);
        assert mcsClone.getMargin() == 0.04f;

        List<Float> radii = new ArrayList<>(3);
        List<Vector3f> centers = new ArrayList<>(3);
        radii.add(0.5f);
        centers.add(new Vector3f(0f, 1f, 0f));
        radii.add(1f);
        centers.add(new Vector3f(0f, 0f, 0f));
        radii.add(0.5f);
        centers.add(new Vector3f(1f, 0f, 0f));
        CollisionShape multiSphere = new MultiSphere(centers, radii);
        CollisionShape multiSphereClone
                = (CollisionShape) Misc.deepCopy(multiSphere);
        assert multiSphereClone.getObjectId() != multiSphere.getObjectId();
        assert multiSphereClone.getMargin() == 0.04f;
        multiSphere.setMargin(0.20f);
        assert multiSphereClone.getMargin() == 0.04f;

        Plane plane = new Plane(new Vector3f(0f, 1f, 0f), 0f);
        CollisionShape pcs = new PlaneCollisionShape(plane);
        CollisionShape pcsClone = (CollisionShape) Misc.deepCopy(pcs);
        assert pcsClone.getObjectId() != pcs.getObjectId();
        assert pcsClone.getMargin() == 0.04f;
        pcs.setMargin(0.21f);
        assert pcsClone.getMargin() == 0.04f;

        Vector3f p1 = new Vector3f(0f, 1f, 1f);
        Vector3f p2 = new Vector3f(1f, 0f, 1f);
        Vector3f p3 = new Vector3f(1f, 1f, 0f);
        CollisionShape simplex = new SimplexCollisionShape(p1, p2, p3);
        CollisionShape simplexClone = (CollisionShape) Misc.deepCopy(simplex);
        assert simplexClone.getObjectId() != simplex.getObjectId();
        assert simplexClone.getMargin() == 0.04f;
        simplex.setMargin(0.22f);
        assert simplexClone.getMargin() == 0.04f;

        CollisionShape sphere = new SphereCollisionShape(1f);
        CollisionShape sphereClone = (CollisionShape) Misc.deepCopy(sphere);
        assert sphereClone.getObjectId() != sphere.getObjectId();
        assert sphereClone.getScale(null).x == 1f;
        sphere.setScale(new Vector3f(2f, 2f, 2f));
        assert sphereClone.getScale(null).x == 1f;
    }
}
