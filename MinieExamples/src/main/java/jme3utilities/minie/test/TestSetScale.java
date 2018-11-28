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
import jme3utilities.MyAsset;

/**
 * Try the setScale() function on collision shapes of all types.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestSetScale extends SimpleApplication {
    // *************************************************************************
    // new methods exposed

    public static void main(String[] args) {
        TestSetScale app = new TestSetScale();
        app.start();
    }
    // *************************************************************************
    // SimpleApplication methods

    @Override
    public void simpleInitApp() {
        Vector3f ident = new Vector3f(1f, 1f, 1f);
        Vector3f uni = new Vector3f(9f, 9f, 9f);
        Vector3f non = new Vector3f(9f, 9f, 1f);
        Vector3f non2 = new Vector3f(1f, 2f, 3f);

        CollisionShape box = new BoxCollisionShape(new Vector3f(2f, 2f, 2f));
        assert box.getScale(null).equals(ident);
        box.setScale(uni);
        assert box.getScale(null).equals(uni);
        box.setScale(non);
        assert box.getScale(null).equals(non);
        box.setScale(non2);
        assert box.getScale(null).equals(non2);

        CollisionShape capsule = new CapsuleCollisionShape(1f, 1f);
        assert capsule.getScale(null).equals(ident);
        capsule.setScale(uni);
        assert capsule.getScale(null).equals(uni);

        CompoundCollisionShape compound = new CompoundCollisionShape();
        compound.addChildShape(capsule, new Vector3f(0f, 1f, 0f));
        assert compound.getScale(null).equals(ident);
        compound.setScale(uni);
        assert compound.getScale(null).equals(uni);
        compound.setScale(non);
        assert compound.getScale(null).equals(non);
        compound.setScale(non2);
        assert compound.getScale(null).equals(non2);

        CollisionShape cone = new ConeCollisionShape(1f, 1f);
        assert cone.getScale(null).equals(ident);
        cone.setScale(uni);
        assert cone.getScale(null).equals(uni);

        CollisionShape cylinder
                = new CylinderCollisionShape(new Vector3f(3f, 3f, 3f));
        assert cylinder.getScale(null).equals(ident);
        cylinder.setScale(uni);
        assert cylinder.getScale(null).equals(uni);
        cylinder.setScale(non);
        assert cylinder.getScale(null).equals(non);

        ModelKey key = new ModelKey("Models/Jaime/Jaime.j3o");
        Node model = (Node) assetManager.loadModel(key);
        Geometry geo = (Geometry) model.getChild(0);
        Mesh mesh = geo.getMesh();
        CollisionShape gimpact = new GImpactCollisionShape(mesh);
        assert gimpact.getScale(null).equals(ident);
        gimpact.setScale(uni);
        assert gimpact.getScale(null).equals(uni);
        gimpact.setScale(non);
        assert gimpact.getScale(null).equals(non);
        gimpact.setScale(non2);
        assert gimpact.getScale(null).equals(non2);

        Texture heightTexture = MyAsset.loadTexture(assetManager,
                "Textures/terrain/height/basin.png");
        Image heightImage = heightTexture.getImage();
        float heightScale = 1f;
        AbstractHeightMap heightMap
                = new ImageBasedHeightMap(heightImage, heightScale);
        heightMap.load();
        float[] heightArray = heightMap.getHeightMap();
        CollisionShape hcs = new HeightfieldCollisionShape(heightArray);
        assert hcs.getScale(null).equals(ident);
        hcs.setScale(uni);
        assert hcs.getScale(null).equals(uni);
        hcs.setScale(non);
        assert hcs.getScale(null).equals(non);
        hcs.setScale(non2);
        assert hcs.getScale(null).equals(non2);

        CollisionShape hull = new HullCollisionShape(mesh);
        assert hull.getScale(null).equals(ident);
        hull.setScale(uni);
        assert hull.getScale(null).equals(uni);
        hull.setScale(non);
        assert hull.getScale(null).equals(non);
        hull.setScale(non2);
        assert hull.getScale(null).equals(non2);

        CollisionShape mcs = new MeshCollisionShape(mesh);
        assert mcs.getScale(null).equals(ident);
        mcs.setScale(uni);
        assert mcs.getScale(null).equals(uni);
        mcs.setScale(non);
        assert mcs.getScale(null).equals(non);
        mcs.setScale(non2);
        assert mcs.getScale(null).equals(non2);

        Plane plane = new Plane(new Vector3f(0f, 1f, 0f), 0f);
        CollisionShape pcs = new PlaneCollisionShape(plane);
        assert pcs.getScale(null).equals(ident);
        pcs.setScale(uni);
        assert pcs.getScale(null).equals(uni);
        pcs.setScale(non);
        assert pcs.getScale(null).equals(non);
        pcs.setScale(non2);
        assert pcs.getScale(null).equals(non2);

        Vector3f p1 = new Vector3f(0f, 1f, 1f);
        Vector3f p2 = new Vector3f(1f, 0f, 1f);
        Vector3f p3 = new Vector3f(1f, 1f, 0f);
        CollisionShape simplex = new SimplexCollisionShape(p1, p2, p3);
        assert simplex.getScale(null).equals(ident);
        simplex.setScale(uni);
        assert simplex.getScale(null).equals(uni);
        simplex.setScale(non);
        assert simplex.getScale(null).equals(non);
        simplex.setScale(non2);
        assert simplex.getScale(null).equals(non2);

        CollisionShape sphere = new SphereCollisionShape(1f);
        assert sphere.getScale(null).equals(ident);
        sphere.setScale(uni);
        assert sphere.getScale(null).equals(uni);

        stop();
    }
}
