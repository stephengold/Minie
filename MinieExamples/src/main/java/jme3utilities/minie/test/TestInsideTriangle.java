/*
 Copyright (c) 2021-2023, Stephen Gold
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
import com.jme3.bullet.util.NativeLibrary;
import com.jme3.input.CameraInput;
import com.jme3.input.KeyInput;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.system.AppSettings;
import java.util.HashSet;
import java.util.Set;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MyAsset;
import jme3utilities.MyString;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.noise.Generator;
import jme3utilities.mesh.Icosphere;
import jme3utilities.mesh.PointMesh;
import jme3utilities.ui.AcorusDemo;
import jme3utilities.ui.CameraOrbitAppState;
import jme3utilities.ui.InputMode;

/**
 * Test Bullet's btTriangleShape::isInside().
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestInsideTriangle extends AcorusDemo {
    // *************************************************************************
    // constants and loggers

    /**
     * number of sample points per trial
     */
    final private static int samplesPerTrial = 9_000;
    /**
     * message logger for this class
     */
    final protected static Logger logger
            = Logger.getLogger(TestInsideTriangle.class.getName());
    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName
            = TestInsideTriangle.class.getSimpleName();
    // *************************************************************************
    // fields

    /**
     * enhanced pseudo-random generator
     */
    final private static Generator generator = new Generator();
    /**
     * scene-graph node for the current trial
     */
    private static Node trialNode = null;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the TestInsideTriangle application.
     */
    public TestInsideTriangle() { // to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the TestInsideTriangle application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        String title = applicationName + " " + MyString.join(arguments);

        // Mute the chatty loggers in certain packages.
        Heart.setLoggingLevels(Level.WARNING);

        boolean loadDefaults = true;
        AppSettings settings = new AppSettings(loadDefaults);
        settings.setAudioRenderer(null);
        settings.setResizable(true);
        settings.setSamples(4); // anti-aliasing
        settings.setTitle(title); // Customize the window's title bar.

        Application application = new TestInsideTriangle();
        application.setSettings(settings);
        application.start();
    }
    // *************************************************************************
    // PhysicsDemo methods

    /**
     * Initialize this application.
     */
    @Override
    public void acorusInit() {
        super.acorusInit();
        configureCamera();
        attachWorldAxes(2f);

        float samplePointSize = 1f;
        Material sampleMaterial = MyAsset.createWireframeMaterial(
                assetManager, ColorRGBA.White, samplePointSize);
        registerMaterial("sample", sampleMaterial);

        float trianglePointSize = 5f;
        Material triangleMaterial = MyAsset.createWireframeMaterial(
                assetManager, ColorRGBA.Green, trianglePointSize);
        registerMaterial("triangle", triangleMaterial);

        ColorRGBA darkGray = new ColorRGBA(0.03f, 0.03f, 0.03f, 1f);
        viewPort.setBackgroundColor(darkGray);

        trial();
    }

    /**
     * Add application-specific hotkey bindings (and override existing ones, if
     * necessary).
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        dim.bindSignal(CameraInput.FLYCAM_LOWER, KeyInput.KEY_DOWN);
        dim.bindSignal(CameraInput.FLYCAM_RISE, KeyInput.KEY_UP);

        dim.bindSignal("orbitLeft", KeyInput.KEY_LEFT);
        dim.bindSignal("orbitRight", KeyInput.KEY_RIGHT);

        dim.bind(asToggleHelp, KeyInput.KEY_H);
        dim.bind(asToggleWorldAxes, KeyInput.KEY_SPACE);
    }
    // *************************************************************************
    // private methods

    /**
     * Visualize a single point in world space.
     *
     * @param location the desired location (in world coordinates, not null)
     * @param materialName the name of the Material to use (not null)
     * @param radius the radius of the visualization (in mesh units, &ge;0)
     */
    private void attachPoint(
            Vector3f location, String materialName, float radius) {
        Mesh mesh;
        if (radius == 0f) {
            mesh = new PointMesh();
        } else {
            int numRefineSteps = 1;
            mesh = new Icosphere(numRefineSteps, radius);
        }

        Geometry geometry = new Geometry("point", mesh);
        trialNode.attachChild(geometry);

        geometry.setLocalTranslation(location);
        Material material = findMaterial(materialName);
        geometry.setMaterial(material);
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        flyCam.setDragToRotate(true);
        CameraOrbitAppState orbitState
                = new CameraOrbitAppState(cam, "orbitLeft", "orbitRight");
        stateManager.attach(orbitState);
    }

    /**
     * Perform a new trial.
     */
    private void trial() {
        trialNode = new Node("trialNode");
        rootNode.attachChild(trialNode);

        // Generate a triangle.
        Vector3f v0 = new Vector3f(1f, 1f, 1f);
        Vector3f v1 = new Vector3f(-1f, 1f, -1f);
        Vector3f v2 = new Vector3f(1f, -1f, -1f);
        Set<Vector3f> list = new HashSet<>(3);
        list.add(v0);
        list.add(v1);
        list.add(v2);

        float maxSeparation = 2f;
        attachPoint(v0, "triangle", maxSeparation);
        attachPoint(v1, "triangle", maxSeparation);
        attachPoint(v2, "triangle", maxSeparation);

        Vector3f center = MyVector3f.mean(list, null);
        float radius = 0f;
        for (Vector3f p : list) {
            float d = p.distance(center);
            if (d > radius) {
                radius = d;
            }
        }

        // Visualize sample points in the vicinity of the triangle.
        Vector3f sampleLocation = new Vector3f();
        for (int numSamples = 0; numSamples < samplesPerTrial;) {
            generator.nextVector3f(sampleLocation);
            sampleLocation.multLocal(radius + maxSeparation + 1f);
            sampleLocation.addLocal(center);
            boolean isInside = NativeLibrary.isInsideTriangle(
                    sampleLocation, maxSeparation, v0, v1, v2);
            if (isInside) {
                attachPoint(sampleLocation, "sample", 0f);
                ++numSamples;
            }
        }
    }
}
