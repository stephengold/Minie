/*
 Copyright (c) 2020-2023, Stephen Gold
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
package jme3utilities.tutorial;

import com.jme3.app.SimpleApplication;
import com.jme3.asset.TextureKey;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.bullet.RotationOrder;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.PlaneCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.debug.DebugInitListener;
import com.jme3.bullet.joints.New6Dof;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.util.PlaneDmiListener;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.material.Materials;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Matrix3f;
import com.jme3.math.Plane;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Limits;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.shadow.DirectionalLightShadowRenderer;
import com.jme3.shadow.EdgeFilteringMode;
import com.jme3.system.AppSettings;
import com.jme3.texture.Texture;
import jme3utilities.MeshNormals;

/**
 * A simple example of a PhysicsJoint.
 * <p>
 * Builds upon HelloKinematics.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class HelloJoint
        extends SimpleApplication
        implements PhysicsTickListener {
    // *************************************************************************
    // constants

    /**
     * physics-space Y coordinate of the ground plane
     */
    private final static float groundY = -4f;
    /**
     * half the height of the paddle (in physics-space units)
     */
    private final static float paddleHalfHeight = 1f;
    // *************************************************************************
    // fields

    /**
     * mouse-controlled kinematic paddle
     */
    private static PhysicsRigidBody paddleBody;
    /**
     * PhysicsSpace for simulation
     */
    private static PhysicsSpace physicsSpace;
    /**
     * latest ground location indicated by the mouse cursor
     */
    final private static Vector3f mouseLocation = new Vector3f();
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the HelloJoint application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        HelloJoint application = new HelloJoint();

        boolean loadDefaults = true;
        AppSettings settings = new AppSettings(loadDefaults);

        // Enable gamma correction for accurate lighting.
        settings.setGammaCorrection(true);

        // Disable VSync for more frequent mouse-position updates.
        settings.setVSync(false);
        application.setSettings(settings);

        application.start();
    }
    // *************************************************************************
    // SimpleApplication methods

    /**
     * Initialize this application.
     */
    @Override
    public void simpleInitApp() {
        configureCamera();
        physicsSpace = configurePhysics();

        // Add a static plane to represent the ground.
        addPlane(groundY);

        // Add a mouse-controlled kinematic paddle.
        addPaddle();

        // Add a dynamic yellow ball.
        PhysicsRigidBody ballBody = addBall();

        // Add a single-ended physics joint to constrain the ball's motion.
        Vector3f pivotInBall = new Vector3f(0f, 3f, 0f);
        Vector3f pivotInWorld = new Vector3f(0f, groundY + 4f, 0f);
        Matrix3f rotInBall = Matrix3f.IDENTITY;
        Matrix3f rotInWorld = Matrix3f.IDENTITY;
        New6Dof joint = new New6Dof(ballBody, pivotInBall, pivotInWorld,
                rotInBall, rotInWorld, RotationOrder.XYZ);
        physicsSpace.addJoint(joint);
    }

    /**
     * Callback invoked once per frame.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void simpleUpdate(float tpf) {
        // Calculate the ground location (if any) selected by the mouse cursor.
        Vector2f screenXy = inputManager.getCursorPosition();
        float nearZ = 0f;
        Vector3f nearLocation = cam.getWorldCoordinates(screenXy, nearZ);
        float farZ = 1f;
        Vector3f farLocation = cam.getWorldCoordinates(screenXy, farZ);
        if (nearLocation.y > groundY && farLocation.y < groundY) {
            float dy = nearLocation.y - farLocation.y;
            float t = (nearLocation.y - groundY) / dy;
            FastMath.interpolateLinear(
                    t, nearLocation, farLocation, mouseLocation);
        }
    }
    // *************************************************************************
    // PhysicsTickListener methods

    /**
     * Callback from Bullet, invoked just before each simulation step.
     *
     * @param space the space that's about to be stepped (not null)
     * @param timeStep the time per simulation step (in seconds, &ge;0)
     */
    @Override
    public void prePhysicsTick(PhysicsSpace space, float timeStep) {
        // Reposition the paddle based on the mouse location.
        Vector3f bodyLocation = mouseLocation.add(0f, paddleHalfHeight, 0f);
        paddleBody.setPhysicsLocation(bodyLocation);
    }

    /**
     * Callback from Bullet, invoked just after each simulation step.
     *
     * @param space the space that was just stepped (not null)
     * @param timeStep the time per simulation step (in seconds, &ge;0)
     */
    @Override
    public void physicsTick(PhysicsSpace space, float timeStep) {
        // do nothing
    }
    // *************************************************************************
    // private methods

    /**
     * Create a dynamic rigid body with a sphere shape and add it to the space.
     *
     * @return the new body
     */
    private PhysicsRigidBody addBall() {
        float radius = 0.4f;
        SphereCollisionShape shape = new SphereCollisionShape(radius);

        float mass = 0.2f;
        PhysicsRigidBody result = new PhysicsRigidBody(shape, mass);
        physicsSpace.addCollisionObject(result);

        // Disable sleep (deactivation).
        result.setEnableSleep(false);

        Material yellowMaterial = createLitMaterial(1f, 1f, 0f);
        result.setDebugMaterial(yellowMaterial);
        result.setDebugMeshNormals(MeshNormals.Facet);
        // faceted so that rotations will be visible

        return result;
    }

    /**
     * Add lighting and shadows to the specified scene.
     *
     * @param scene the scene to augment (not null)
     */
    private void addLighting(Spatial scene) {
        ColorRGBA ambientColor = new ColorRGBA(0.03f, 0.03f, 0.03f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        scene.addLight(ambient);
        ambient.setName("ambient");

        ColorRGBA directColor = new ColorRGBA(0.2f, 0.2f, 0.2f, 1f);
        Vector3f direction = new Vector3f(-7f, -3f, -5f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction, directColor);
        scene.addLight(sun);
        sun.setName("sun");

        // Render shadows based on the directional light.
        viewPort.clearProcessors();
        int shadowMapSize = 2_048; // in pixels
        int numSplits = 3;
        DirectionalLightShadowRenderer dlsr
                = new DirectionalLightShadowRenderer(
                        assetManager, shadowMapSize, numSplits);
        dlsr.setEdgeFilteringMode(EdgeFilteringMode.PCFPOISSON);
        dlsr.setEdgesThickness(5);
        dlsr.setLight(sun);
        dlsr.setShadowIntensity(0.6f);
        viewPort.addProcessor(dlsr);
    }

    /**
     * Create a kinematic body with a box shape and add it to the space.
     */
    private void addPaddle() {
        BoxCollisionShape shape
                = new BoxCollisionShape(0.3f, paddleHalfHeight, 1f);
        paddleBody = new PhysicsRigidBody(shape);
        paddleBody.setKinematic(true);

        physicsSpace.addCollisionObject(paddleBody);

        Material redMaterial = createLitMaterial(1f, 0.1f, 0.1f);
        paddleBody.setDebugMaterial(redMaterial);
        paddleBody.setDebugMeshNormals(MeshNormals.Facet);
    }

    /**
     * Add a horizontal plane body to the space.
     *
     * @param y (the desired elevation, in physics-space coordinates)
     */
    private void addPlane(float y) {
        Plane plane = new Plane(Vector3f.UNIT_Y, y);
        PlaneCollisionShape shape = new PlaneCollisionShape(plane);
        PhysicsRigidBody body
                = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);

        // Load a repeating tile texture.
        String assetPath = "Textures/greenTile.png";
        boolean flipY = false;
        TextureKey key = new TextureKey(assetPath, flipY);
        boolean generateMips = true;
        key.setGenerateMips(generateMips);
        Texture texture = assetManager.loadTexture(key);
        texture.setMinFilter(Texture.MinFilter.Trilinear);
        texture.setWrap(Texture.WrapMode.Repeat);

        // Enable anisotropic filtering, to reduce blurring.
        Integer maxDegree = renderer.getLimits().get(Limits.TextureAnisotropy);
        int degree = (maxDegree == null) ? 1 : Math.min(8, maxDegree);
        texture.setAnisotropicFilter(degree);

        // Apply a tiled, unshaded debug material to the body.
        Material material = new Material(assetManager, Materials.UNSHADED);
        material.setTexture("ColorMap", texture);
        body.setDebugMaterial(material);

        // Generate texture coordinates during debug-mesh initialization.
        float tileSize = 1f;
        PlaneDmiListener planeDmiListener = new PlaneDmiListener(tileSize);
        body.setDebugMeshInitListener(planeDmiListener);

        physicsSpace.addCollisionObject(body);
    }

    /**
     * Disable FlyByCamera during startup.
     */
    private void configureCamera() {
        flyCam.setEnabled(false);

        cam.setLocation(new Vector3f(0f, 5f, 10f));
        cam.setRotation(new Quaternion(0f, 0.95f, -0.3122f, 0f));
    }

    /**
     * Configure physics during startup.
     *
     * @return a new instance (not null)
     */
    private PhysicsSpace configurePhysics() {
        BulletAppState bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);

        // Enable debug visualization to reveal what occurs in physics space.
        bulletAppState.setDebugEnabled(true);

        // Add lighting and shadows to the debug scene.
        bulletAppState.setDebugInitListener(new DebugInitListener() {
            @Override
            public void bulletDebugInit(Node physicsDebugRootNode) {
                addLighting(physicsDebugRootNode);
            }
        });
        bulletAppState.setDebugShadowMode(
                RenderQueue.ShadowMode.CastAndReceive);

        PhysicsSpace result = bulletAppState.getPhysicsSpace();

        // To enable the callbacks, register the application as a tick listener.
        result.addTickListener(this);

        // Reduce the time step for better accuracy.
        result.setAccuracy(0.005f);

        return result;
    }

    /**
     * Create a single-sided lit material with the specified reflectivities.
     *
     * @param red the desired reflectivity for red light (&ge;0, &le;1)
     * @param green the desired reflectivity for green light (&ge;0, &le;1)
     * @param blue the desired reflectivity for blue light (&ge;0, &le;1)
     * @return a new instance (not null)
     */
    private Material createLitMaterial(float red, float green, float blue) {
        Material result = new Material(assetManager, Materials.LIGHTING);
        result.setBoolean("UseMaterialColors", true);

        float opacity = 1f;
        result.setColor("Ambient", new ColorRGBA(red, green, blue, opacity));
        result.setColor("Diffuse", new ColorRGBA(red, green, blue, opacity));

        return result;
    }
}
