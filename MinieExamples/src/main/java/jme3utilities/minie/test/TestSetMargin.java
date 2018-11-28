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

import com.jme3.app.SimpleApplication;
import com.jme3.asset.ModelKey;
import com.jme3.bullet.BulletAppState;
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
import com.jme3.math.Plane;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.terrain.heightmap.AbstractHeightMap;
import com.jme3.terrain.heightmap.ImageBasedHeightMap;
import com.jme3.texture.Image;
import com.jme3.texture.Texture;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Logger;
import jme3utilities.MyAsset;

/**
 * Try the setMargin() function on collision shapes of all types.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestSetMargin extends SimpleApplication {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(TestSetMargin.class.getName());
    // *************************************************************************
    // new methods exposed

    public static void main(String[] args) {
        TestSetMargin app = new TestSetMargin();
        app.start();
    }
    // *************************************************************************
    // SimpleApplication methods

    @Override
    public void simpleInitApp() {
        BulletAppState bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);

        CollisionShape box = new BoxCollisionShape(new Vector3f(1f, 1f, 1f));
        assert box.getMargin() == 0.04f;
        box.setMargin(0.11f);
        assert box.getMargin() == 0.11f;

        CollisionShape capsule = new CapsuleCollisionShape(1f, 1f);
        assert capsule.getMargin() == 0f;
        capsule.setMargin(0.12f); // cannot alter margin
        assert capsule.getMargin() == 0f;

        CompoundCollisionShape compound = new CompoundCollisionShape();
        compound.addChildShape(capsule, new Vector3f(0f, 1f, 0f));
        assert compound.getMargin() == 0.04f;
        compound.setMargin(0.13f);
        assert compound.getMargin() == 0.13f;

        CollisionShape cone = new ConeCollisionShape(1f, 1f);
        assert cone.getMargin() == 0.04f;
        cone.setMargin(0.14f);
        assert cone.getMargin() == 0.14f;

        CollisionShape cylinder
                = new CylinderCollisionShape(new Vector3f(1f, 1f, 1f));
        assert cylinder.getMargin() == 0.04f;
        cylinder.setMargin(0.15f);
        assert cylinder.getMargin() == 0.15f;

        ModelKey key = new ModelKey("Models/Jaime/Jaime.j3o");
        Node model = (Node) assetManager.loadModel(key);
        Geometry geo = (Geometry) model.getChild(0);
        Mesh mesh = geo.getMesh();
        CollisionShape gimpact = new GImpactCollisionShape(mesh);
        assert gimpact.getMargin() == 0.04f;
        gimpact.setMargin(0.16f);
        assert gimpact.getMargin() == 0.16f;

        Texture heightTexture = MyAsset.loadTexture(assetManager,
                "Textures/terrain/height/basin.png");
        Image heightImage = heightTexture.getImage();
        float heightScale = 1f;
        AbstractHeightMap heightMap
                = new ImageBasedHeightMap(heightImage, heightScale);
        heightMap.load();
        float[] heightArray = heightMap.getHeightMap();
        CollisionShape hcs = new HeightfieldCollisionShape(heightArray);
        assert hcs.getMargin() == 0.04f;
        hcs.setMargin(0.17f);
        assert hcs.getMargin() == 0.17f;

        CollisionShape hull = new HullCollisionShape(mesh);
        assert hull.getMargin() == 0.04f;
        hull.setMargin(0.18f);
        assert hull.getMargin() == 0.18f;

        CollisionShape mcs = new MeshCollisionShape(mesh);
        assert mcs.getMargin() == 0.04f;
        mcs.setMargin(0.19f);
        assert mcs.getMargin() == 0.19f;

        List<Float> radii = new ArrayList<>(3);
        List<Vector3f> centers = new ArrayList<>(3);
        radii.add(0.5f);
        centers.add(new Vector3f(0f, 1f, 0f));
        radii.add(1f);
        centers.add(new Vector3f(0f, 0f, 0f));
        radii.add(0.5f);
        centers.add(new Vector3f(1f, 0f, 0f));
        CollisionShape multiSphere = new MultiSphere(centers, radii);
        assert multiSphere.getMargin() == 0.04f;
        multiSphere.setMargin(0.20f);
        assert multiSphere.getMargin() == 0.20f;

        Plane plane = new Plane(new Vector3f(0f, 1f, 0f), 0f);
        CollisionShape pcs = new PlaneCollisionShape(plane);
        assert pcs.getMargin() == 0.04f;
        pcs.setMargin(0.21f);
        assert pcs.getMargin() == 0.21f;

        Vector3f p1 = new Vector3f(0f, 1f, 1f);
        Vector3f p2 = new Vector3f(1f, 0f, 1f);
        Vector3f p3 = new Vector3f(1f, 1f, 0f);
        CollisionShape simplex = new SimplexCollisionShape(p1, p2, p3);
        assert simplex.getMargin() == 0.04f;
        simplex.setMargin(0.22f);
        assert simplex.getMargin() == 0.22f;

        CollisionShape sphere = new SphereCollisionShape(1f);
        assert sphere.getMargin() == 0f;
        sphere.setMargin(0.3f); // cannot alter margin
        assert sphere.getMargin() == 0f;

        stop();
    }
}
