/*
 * Copyright (c) 2009-2023 jMonkeyEngine
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of 'jMonkeyEngine' nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package jme3test.batching;

import com.jme3.app.SimpleApplication;
import com.jme3.asset.TextureKey;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.util.NativeLibrary;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.post.FilterPostProcessor;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.BatchNode;
import com.jme3.scene.Geometry;
import com.jme3.scene.shape.Box;
import com.jme3.scene.shape.Sphere;
import com.jme3.shadow.CompareMode;
import com.jme3.shadow.DirectionalLightShadowFilter;
import com.jme3.shadow.EdgeFilteringMode;
import com.jme3.system.AppSettings;
import com.jme3.texture.Texture;
import com.jme3.texture.Texture.WrapMode;
import java.util.concurrent.Callable;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.math.MyMath;

/**
 * Based on TestBatchNodeTower by double1984 and atom.
 */
public class TowerPerformance
        extends SimpleApplication
        implements PhysicsTickListener {
    // *************************************************************************
    // constants and loggers

    final private static float brickDepth = 0.25f;
    final private static float brickHeight = 0.25f;
    final private static float brickWidth = 0.75f;
    final private static float bulletRadius = 0.4f;
    final private static float radius = 3f;
    final private static int brickLayers = 30;
    final private static int bricksPerLayer = 8;
    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(TowerPerformance.class.getName());
    // *************************************************************************
    // fields

    final private BatchNode batchNode = new BatchNode("batch Node");
    private Box brick;
    private BulletAppState bulletAppState;
    private float angleDegrees;
    private int numSteps;
    private long physicsNs;
    private long preTickNs;
    private Material mat;
    private Material mat2;
    private Material mat3;
    private Sphere bullet;
    private SphereCollisionShape bulletCollisionShape;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the TowerPerformance application.
     */
    public TowerPerformance() { // to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the TowerPerformance application.
     *
     * @param args array of command-line arguments (ignored)
     */
    public static void main(String[] args) {
        TowerPerformance f = new TowerPerformance();
        AppSettings s = new AppSettings(true);
        s.setAudioRenderer(null);
        f.setSettings(s);
        f.start();
    }
    // *************************************************************************
    // PhysicsTickListener methods

    @Override
    public void physicsTick(PhysicsSpace space, float timeStep) {
        physicsNs += System.nanoTime() - preTickNs;
        ++numSteps;
        if ((numSteps % 50) == 0) {
            float millisPerStep = (physicsNs * 1e-6f) / 50;
            System.out.printf("millisPerStep = %.3f%n", millisPerStep);
            physicsNs = 0L;
        }

        if (numSteps == 200) {
            NativeLibrary.resetQuickprof();
            enqueue(new Callable<Void>() {
                @Override
                public Void call() throws Exception {
                    shoot();
                    return null;
                }
            });

        } else if (numSteps == 600) {
            NativeLibrary.dumpQuickprof();
            enqueue(new Callable<Void>() {
                @Override
                public Void call() throws Exception {
                    stop();
                    return null;
                }
            });
        }
    }

    @Override
    public void prePhysicsTick(PhysicsSpace space, float timeStep) {
        this.preTickNs = System.nanoTime();
    }
    // *************************************************************************
    // SimpleApplication methods

    @Override
    public void simpleInitApp() {
        if (Heart.areAssertionsEnabled()) {
            System.out.println("Warning: assertions are enabled.");
        }
        if (NativeLibrary.isDebug()) {
            System.out.println("Warning: using a Debug native library.");
        }

        this.bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);
        getPhysicsSpace().addTickListener(this);

        this.bullet = new Sphere(32, 32, bulletRadius, true, false);
        bullet.setTextureMode(Sphere.TextureMode.Projected);
        bulletCollisionShape = new SphereCollisionShape(bulletRadius);

        this.brick = new Box(brickWidth, brickHeight, brickDepth);
        brick.scaleTextureCoordinates(new Vector2f(1f, 0.5f));
        initMaterial();
        initTower();
        initFloor();

        cam.setLocation(new Vector3f(0f, 25f, 8f));
        cam.lookAt(Vector3f.ZERO, new Vector3f(0f, 1f, 0f));
        cam.setFrustumFar(80f);
        flyCam.setEnabled(false);

        rootNode.setShadowMode(RenderQueue.ShadowMode.Off);
        batchNode.batch();
        batchNode.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);
        rootNode.attachChild(batchNode);

        DirectionalLight dl = new DirectionalLight();
        dl.setDirection(new Vector3f(-1f, -1f, -1f).normalizeLocal());
        DirectionalLightShadowFilter shadowRenderer
                = new DirectionalLightShadowFilter(assetManager, 1024, 2);
        shadowRenderer.setEdgeFilteringMode(EdgeFilteringMode.PCF4);
        shadowRenderer.setLambda(0.55f);
        shadowRenderer.setLight(dl);
        shadowRenderer.setShadowCompareMode(CompareMode.Hardware);
        shadowRenderer.setShadowIntensity(0.6f);

        int numSamples = settings.getSamples();
        FilterPostProcessor fpp
                = Heart.getFpp(viewPort, assetManager, numSamples);
        fpp.addFilter(shadowRenderer);
        viewPort.addProcessor(fpp);
    }
    // *************************************************************************
    // private methods

    private void addBrick(Vector3f ori) {
        Geometry brickGeometry = new Geometry("brick", brick);
        brickGeometry.setMaterial(mat);
        brickGeometry.setLocalTranslation(ori);
        brickGeometry.rotate(0f, MyMath.toRadians(angleDegrees), 0f);
        brickGeometry.addControl(new RigidBodyControl(1.5f));
        brickGeometry.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);
        brickGeometry.getControl(RigidBodyControl.class).setFriction(1.6f);
        brickGeometry.getControl(RigidBodyControl.class)
                .setLinearSleepingThreshold(0f);

        batchNode.attachChild(brickGeometry);
        getPhysicsSpace().add(brickGeometry);
    }

    private PhysicsSpace getPhysicsSpace() {
        PhysicsSpace result = bulletAppState.getPhysicsSpace();
        return result;
    }

    private void initFloor() {
        Box floorBox = new Box(10f, 0.1f, 5f);
        floorBox.scaleTextureCoordinates(new Vector2f(3f, 6f));

        Geometry floor = new Geometry("floor", floorBox);
        floor.setMaterial(mat3);
        floor.setShadowMode(RenderQueue.ShadowMode.Receive);
        floor.addControl(new RigidBodyControl(PhysicsBody.massForStatic));
        rootNode.attachChild(floor);
        getPhysicsSpace().add(floor);
    }

    private void initMaterial() {
        this.mat = new Material(
                assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        TextureKey key
                = new TextureKey("Textures/Terrain/BrickWall/BrickWall.jpg");
        key.setGenerateMips(true);
        Texture tex = assetManager.loadTexture(key);
        mat.setTexture("ColorMap", tex);

        this.mat2 = new Material(
                assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        TextureKey key2 = new TextureKey("Textures/Terrain/Rock/Rock.PNG");
        key2.setGenerateMips(true);
        Texture tex2 = assetManager.loadTexture(key2);
        mat2.setTexture("ColorMap", tex2);

        this.mat3 = new Material(
                assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        TextureKey key3 = new TextureKey("Textures/Terrain/Pond/Pond.jpg");
        key3.setGenerateMips(true);
        Texture tex3 = assetManager.loadTexture(key3);
        tex3.setWrap(WrapMode.Repeat);
        mat3.setTexture("ColorMap", tex3);
    }

    private void initTower() {
        double tempX;
        double tempY = 0.0;
        double tempZ;
        this.angleDegrees = 0f;
        for (int i = 0; i < brickLayers; i++) {
            // Increment rows
            if (i != 0) {
                tempY += brickHeight * 2;
            } else {
                tempY = brickHeight;
            }
            // Alternate brick seams
            this.angleDegrees = 360f / bricksPerLayer * i / 2f;
            for (int j = 0; j < bricksPerLayer; j++) {
                tempZ = Math.cos(Math.toRadians(angleDegrees)) * radius;
                tempX = Math.sin(Math.toRadians(angleDegrees)) * radius;
                Vector3f vt = new Vector3f(
                        (float) tempX, (float) tempY, (float) tempZ);
                // Add crenelation
                if (i == brickLayers - 1) {
                    if (j % 2 == 0) {
                        addBrick(vt);
                    }
                } else { // Create main tower
                    addBrick(vt);
                }
                this.angleDegrees += 360f / bricksPerLayer;
            }
        }
    }

    private void shoot() {
        Geometry geometry = new Geometry("bullet", bullet);
        geometry.setMaterial(mat2);
        geometry.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);
        geometry.setLocalTranslation(cam.getLocation());

        RigidBodyControl bulletNode
                = new BombControl(assetManager, bulletCollisionShape, 1f);
        bulletNode.setLinearVelocity(cam.getDirection().mult(25f));
        geometry.addControl(bulletNode);
        rootNode.attachChild(geometry);
        getPhysicsSpace().add(bulletNode);

        System.out.println("shoot");
    }
}
