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

import com.jme3.app.Application;
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
import java.util.logging.Logger;
import jme3utilities.ui.ActionApplication;

/**
 * Test heightfield collision shapes.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestHeightfield extends ActionApplication {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(TestHeightfield.class.getName());
    // *************************************************************************
    // fields

    /**
     * AppState to manage the PhysicsSpace
     */
    final private BulletAppState bulletAppState = new BulletAppState();
    /**
     * seconds since the application started
     */
    private float elapsedTime = 0f;
    /**
     * lit green material to visualize the terrain
     */
    private Material terrainMaterial;
    /**
     * space for physics simulation
     */
    private PhysicsSpace physicsSpace;
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the TestHeightField application.
     *
     * @param ignored array of command-line arguments (not null)
     */
    public static void main(String[] ignored) {
        Application application = new TestHeightfield();
        application.start();
    }
    // *************************************************************************
    // ActionApplication methods

    /**
     * Initialize this application.
     */
    @Override
    public void actionInitializeApplication() {
        configureCamera();
        configureMaterials();
        configurePhysics();

        addLighting();
        addTerrain();
    }

    /**
     * Callback invoked once per frame.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void simpleUpdate(float tpf) {
        elapsedTime += tpf;
        if (elapsedTime > 2f) {
            elapsedTime = 0f;
            togglePhysicsDebug();
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Add lighting to the scene.
     */
    private void addLighting() {
        Vector3f lightDirection = new Vector3f(-1f, -1f, -1f).normalizeLocal();
        ColorRGBA lightColor = new ColorRGBA(1f, 1f, 1f, 1f);
        DirectionalLight dl = new DirectionalLight(lightDirection, lightColor);
        rootNode.addLight(dl);
    }

    /**
     * Add terrain to the scene.
     */
    private void addTerrain() {
        AbstractHeightMap heightMap = loadHeightMap();
        int patchSize = 33; // in pixels
        int terrainDiameter = heightMap.getSize(); // in pixels
        int mapSize = terrainDiameter + 1; // number of samples on a side
        float[] heightArray = heightMap.getHeightMap();
        TerrainQuad quad
                = new TerrainQuad("terrain", patchSize, mapSize, heightArray);
        quad.setLocalScale(0.01f);
        quad.setMaterial(terrainMaterial);
        quad.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);
        rootNode.attachChild(quad);

        CollisionShape shape = CollisionShapeFactory.createMeshShape(quad);
        float massForStatic = 0f;
        RigidBodyControl rbc = new RigidBodyControl(shape, massForStatic);
        rbc.setPhysicsSpace(physicsSpace);
        rootNode.addControl(rbc);
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        Vector3f cameraLocation = new Vector3f(8f, 7f, 3f);
        cam.setLocation(cameraLocation);
        Quaternion cameraRotation
                = new Quaternion(0.22f, -0.76f, 0.24f, 0.56f).normalizeLocal();
        cam.setRotation(cameraRotation);
        flyCam.setMoveSpeed(10f);
    }

    /**
     * Configure materials during startup.
     */
    private void configureMaterials() {
        terrainMaterial = new Material(assetManager,
                "Common/MatDefs/Light/Lighting.j3md");
        terrainMaterial.setBoolean("UseMaterialColors", true);
        ColorRGBA terrainColor
                = new ColorRGBA(0.65f, 0.8f, 0.2f, 1f);
        terrainMaterial.setColor("Diffuse", terrainColor.clone());
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        stateManager.attach(bulletAppState);
        physicsSpace = bulletAppState.getPhysicsSpace();
    }

    /**
     * Load a simple height map from a texture asset.
     *
     * @return a new instance (not null)
     */
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

    /**
     * Toggle physics-debug visualization on/off.
     */
    private void togglePhysicsDebug() {
        boolean enabled = bulletAppState.isDebugEnabled();
        bulletAppState.setDebugEnabled(!enabled);
    }
}
