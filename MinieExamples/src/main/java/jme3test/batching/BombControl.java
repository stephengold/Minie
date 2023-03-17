/*
 * Copyright (c) 2009-2021 jMonkeyEngine
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

import com.jme3.asset.AssetManager;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.bullet.collision.PhysicsCollisionEvent;
import com.jme3.bullet.collision.PhysicsCollisionListener;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.objects.PhysicsGhostObject;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.effect.ParticleEmitter;
import com.jme3.effect.ParticleMesh.Type;
import com.jme3.effect.shapes.EmitterSphereShape;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.scene.Spatial;
import java.io.IOException;
import java.util.Iterator;
import java.util.logging.Logger;

/**
 * @author normenhansen
 */
class BombControl
        extends RigidBodyControl
        implements PhysicsCollisionListener, PhysicsTickListener {
    // *************************************************************************
    // constants and loggers

    final private static float explosionRadius = 10f;
    final private static float forceFactor = 1f;
    final private static float fxTime = 0.5f;
    final private static float maxTime = 4f;
    /**
     * message logger for this class
     */
    final static Logger logger4 = Logger.getLogger(BombControl.class.getName());
    // *************************************************************************
    // fields

    private float curTime = -1f;
    private float timer;
    private ParticleEmitter effect;
    private PhysicsGhostObject ghostObject;
    final private Vector3f vector = new Vector3f();
    final private Vector3f vector2 = new Vector3f();
    // *************************************************************************
    // constructors

    BombControl(AssetManager manager, CollisionShape shape, float mass) {
        super(shape, mass);
        createGhostObject();
        prepareEffect(manager);
    }
    // *************************************************************************
    // PhysicsCollisionListener methods

    @Override
    public void collision(PhysicsCollisionEvent event) {
        PhysicsSpace space = getPhysicsSpace();
        if (space == null) {
            return;
        }
        if (event.getObjectA() == this || event.getObjectB() == this) {
            space.add(ghostObject);
            ghostObject.setPhysicsLocation(getPhysicsLocation(vector));
            space.addTickListener(this);
            Spatial spatial = getSpatial();
            if (effect != null && spatial.getParent() != null) {
                curTime = 0;
                effect.setLocalTranslation(spatial.getLocalTranslation());
                spatial.getParent().attachChild(effect);
                effect.emitAllParticles();
            }
            space.remove(this);
            spatial.removeFromParent();
        }
    }
    // *************************************************************************
    // PhysicsTickListener methods

    @Override
    public void physicsTick(PhysicsSpace space, float f) {
        //get all overlapping objects and apply impulse to them
        for (PhysicsCollisionObject physicsCollisionObject
                : ghostObject.getOverlappingObjects()) {
            if (physicsCollisionObject instanceof PhysicsRigidBody) {
                PhysicsRigidBody rBody = (PhysicsRigidBody) physicsCollisionObject;
                rBody.getPhysicsLocation(vector2);
                vector2.subtractLocal(vector);
                float force = explosionRadius - vector2.length();
                force *= forceFactor;
                force = force > 0 ? force : 0;
                vector2.normalizeLocal();
                vector2.multLocal(force);
                ((PhysicsRigidBody) physicsCollisionObject).applyImpulse(vector2, Vector3f.ZERO);
            }
        }
        space.removeTickListener(this);
        space.remove(ghostObject);
    }

    @Override
    public void prePhysicsTick(PhysicsSpace space, float f) {
        space.removeCollisionListener(this);
    }
    // *************************************************************************
    // RigidBodyControl methods

    @Override
    public void read(JmeImporter im) throws IOException {
        throw new UnsupportedOperationException("Reading not supported.");
    }

    @Override
    public void setPhysicsSpace(PhysicsSpace space) {
        super.setPhysicsSpace(space);
        if (space != null) {
            space.addCollisionListener(this);
        }
    }

    @Override
    public void write(JmeExporter ex) throws IOException {
        throw new UnsupportedOperationException("Saving not supported.");
    }

    @Override
    public void update(float tpf) {
        super.update(tpf);
        boolean enabled = isEnabled();
        if (enabled) {
            timer += tpf;
            if (timer > maxTime) {
                Spatial spatial = getSpatial();
                if (spatial.getParent() != null) {
                    PhysicsSpace space = getPhysicsSpace();
                    space.removeCollisionListener(this);
                    space.remove(this);
                    spatial.removeFromParent();
                }
            }
        }
        if (enabled && curTime >= 0) {
            curTime += tpf;
            if (curTime > fxTime) {
                curTime = -1;
                effect.removeFromParent();
            }
        }
    }
    // *************************************************************************
    // private methods

    private void prepareEffect(AssetManager assetManager) {
        int COUNT_FACTOR = 1;
        float COUNT_FACTOR_F = 1f;
        effect = new ParticleEmitter("Flame", Type.Triangle, 32 * COUNT_FACTOR);
        effect.setSelectRandomImage(true);
        effect.setStartColor(new ColorRGBA(1f, 0.4f, 0.05f, (1f / COUNT_FACTOR_F)));
        effect.setEndColor(new ColorRGBA(.4f, .22f, .12f, 0f));
        effect.setStartSize(1.3f);
        effect.setEndSize(2f);
        effect.setShape(new EmitterSphereShape(Vector3f.ZERO, 1f));
        effect.setParticlesPerSec(0);
        effect.setGravity(0, -5f, 0);
        effect.setLowLife(.4f);
        effect.setHighLife(.5f);
        effect.getParticleInfluencer()
                .setInitialVelocity(new Vector3f(0, 7, 0));
        effect.getParticleInfluencer().setVelocityVariation(1f);
        effect.setImagesX(2);
        effect.setImagesY(2);
        Material mat = new Material(assetManager, "Common/MatDefs/Misc/Particle.j3md");
        mat.setTexture("Texture", assetManager.loadTexture("Effects/Explosion/flame.png"));
        effect.setMaterial(mat);
    }

    private void createGhostObject() {
        ghostObject = new PhysicsGhostObject(new SphereCollisionShape(explosionRadius));
    }
}
