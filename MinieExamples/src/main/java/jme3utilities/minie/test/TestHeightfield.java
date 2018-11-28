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
import com.jme3.asset.TextureKey;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.util.CollisionShapeFactory;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.terrain.geomipmap.TerrainQuad;
import com.jme3.terrain.heightmap.AbstractHeightMap;
import com.jme3.terrain.heightmap.ImageBasedHeightMap;
import com.jme3.texture.Image;
import com.jme3.texture.Texture;

/**
 * Test heightfield collision shapes.
 */
public class TestHeightfield extends SimpleApplication {
    // *************************************************************************
    // fields

    private TerrainQuad quad;
    final private BulletAppState bulletAppState = new BulletAppState();
    private float elapsedTime = 0f;
    private boolean debugFlag = false;
    // *************************************************************************
    // new methods exposed

    public static void main(String[] args) {
        TestHeightfield app = new TestHeightfield();
        app.start();
    }
    // *************************************************************************
    // SimpleApplication methods

    @Override
    public void simpleInitApp() {
        Vector3f cameraLocation = new Vector3f(8f, 7f, 3f);
        cam.setLocation(cameraLocation);
        Quaternion cameraRotation
                = new Quaternion(0.22f, -0.76f, 0.24f, 0.56f).normalizeLocal();
        cam.setRotation(cameraRotation);
        flyCam.setMoveSpeed(10f);

        Material material = new Material(assetManager,
                "Common/MatDefs/Light/Lighting.j3md");
        material.setBoolean("UseMaterialColors", true);
        ColorRGBA terrainColor
                = new ColorRGBA(0.65f, 0.8f, 0.2f, 1f);
        material.setColor("Diffuse", terrainColor.clone());

        AbstractHeightMap heightMap = loadHeightMap();
        int patchSize = 33; // in pixels
        int terrainDiameter = heightMap.getSize(); // in pixels
        int mapSize = terrainDiameter + 1; // number of samples on a side
        float[] heightArray = heightMap.getHeightMap();
        quad = new TerrainQuad("terrain", patchSize, mapSize, heightArray);
        quad.setLocalScale(0.01f);
        quad.setMaterial(material);
        quad.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);
        rootNode.attachChild(quad);

        Vector3f lightDirection = new Vector3f(-1f, -1f, -1f).normalizeLocal();
        ColorRGBA lightColor = new ColorRGBA(1f, 1f, 1f, 1f);
        DirectionalLight dl = new DirectionalLight(lightDirection, lightColor);
        rootNode.addLight(dl);

        stateManager.attach(bulletAppState);
        PhysicsSpace space = bulletAppState.getPhysicsSpace();

        CollisionShape shape = CollisionShapeFactory.createMeshShape(quad);
        float massForStatic = 0f;
        RigidBodyControl rbc = new RigidBodyControl(shape, massForStatic);
        rbc.setKinematic(true);
        rbc.setPhysicsSpace(space);
        rootNode.addControl(rbc);
    }

    @Override
    public void simpleUpdate(float tpf) {
        elapsedTime += tpf;
        if (elapsedTime > 2f) {
            elapsedTime = 0f;
            debugFlag = !debugFlag;
            bulletAppState.setDebugEnabled(debugFlag);
        }
    }
    // *************************************************************************
    // private methods

    private AbstractHeightMap loadHeightMap() {
        boolean flipY = false;
        TextureKey key = new TextureKey(
                "Textures/BumpMapTest/Simple_height.png", flipY);
        Texture texture = assetManager.loadTexture(key);
        Image heightImage = texture.getImage();
        float heightScale = 1f;
        AbstractHeightMap heightMap
                = new ImageBasedHeightMap(heightImage, heightScale);
        heightMap.load();

        return heightMap;
    }
}
