/*
 Copyright (c) 2018-2023, Stephen Gold
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
package jme3utilities.minie.test.issue;

import com.jme3.app.SimpleApplication;
import com.jme3.bullet.SoftBodyWorldInfo;
import com.jme3.bullet.control.SoftBodyControl;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.bullet.objects.infos.ConfigFlag;
import com.jme3.bullet.objects.infos.Sbcp;
import com.jme3.bullet.objects.infos.SoftBodyConfig;
import com.jme3.bullet.objects.infos.SoftBodyMaterial;
import com.jme3.bullet.util.NativeSoftBodyUtil;
import com.jme3.export.binary.BinaryExporter;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.debug.WireBox;
import java.util.logging.Logger;
import jme3utilities.Heart;

/**
 * Test case for Minie issue #9: JVM crash in
 * PhysicsCollisionObject.finalizeNative().
 *
 * If successful, the app will start up without a JVM crash.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class TestIssue9 extends SimpleApplication {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(TestIssue9.class.getName());
    // *************************************************************************
    // fields

    /**
     * wire mesh for generating soft bodies
     */
    final static private Mesh wireBox = new WireBox();
    // *************************************************************************
    // constructors

    /**
     * Instantiate the TestIssue9 application.
     */
    public TestIssue9() { // explicit to avoid a warning from JDK 18 javadoc
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the TestIssue9 application.
     *
     * @param arguments unused
     */
    public static void main(String[] arguments) {
        TestIssue9 app = new TestIssue9();
        app.start();
    }
    // *************************************************************************
    // SimpleApplication methods

    /**
     * Initialize this application.
     */
    @Override
    public void simpleInitApp() {
        for (int iteration = 0; iteration < 99; ++iteration) {
            clonePsb();
            cloneSbc();
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Clone soft bodies.
     */
    private void clonePsb() {
        // empty
        PhysicsSoftBody soft = new PhysicsSoftBody();
        setParameters(soft, 0f);
        PhysicsSoftBody softClone = Heart.deepCopy(soft);
        cloneTest(soft, softClone);

        // non-empty
        PhysicsSoftBody soft2 = new PhysicsSoftBody();
        NativeSoftBodyUtil.appendFromLineMesh(wireBox, soft2);
        setParameters(soft2, 0f);
        PhysicsSoftBody soft2Clone = Heart.deepCopy(soft2);
        cloneTest(soft2, soft2Clone);
    }

    /**
     * Clone soft-body controls.
     */
    private void cloneSbc() {
        boolean localPhysics = false;
        boolean updateNormals = true;
        boolean mergeVertices = true;
        SoftBodyControl sbc = new SoftBodyControl(
                localPhysics, updateNormals, mergeVertices);
        Geometry sbcGeom = new Geometry("sbcGeom", wireBox);
        sbcGeom.addControl(sbc);
        PhysicsSoftBody soft3 = sbc.getBody();
        setParameters(soft3, 0f);
        Geometry sbcGeomClone = Heart.deepCopy(sbcGeom);
        SoftBodyControl sbcClone = (SoftBodyControl) sbcGeomClone.getControl(0);
        PhysicsSoftBody soft3Clone = sbcClone.getBody();
        cloneTest(soft3, soft3Clone);
    }

    private void cloneTest(PhysicsBody body, PhysicsBody bodyClone) {
        assert bodyClone.nativeId() != body.nativeId();
        if (body instanceof PhysicsSoftBody) {
            PhysicsSoftBody sBody = (PhysicsSoftBody) body;
            PhysicsSoftBody sBodyClone = (PhysicsSoftBody) bodyClone;
            assert sBodyClone.getSoftConfig() != sBody.getSoftConfig();
            assert sBodyClone.getWorldInfo() != sBody.getWorldInfo();
        }

        setParameters(body, 0.3f);
        setParameters(bodyClone, 0.6f);
        BinaryExporter.saveAndLoad(assetManager, body);
        BinaryExporter.saveAndLoad(assetManager, bodyClone);
    }

    private static void setParameters(PhysicsBody pco, float b) {
        if (pco instanceof PhysicsSoftBody) {
            setSoft((PhysicsSoftBody) pco, b);
        } else {
            throw new IllegalArgumentException(pco.getClass().getName());
        }
    }

    private static void setSoft(PhysicsSoftBody body, float b) {
        boolean flag = (b > 0.15f && b < 0.45f);
        int n = Math.round(10f * b);

        body.setProtectWorldInfo(!flag);

        SoftBodyConfig config = body.getSoftConfig();
        for (Sbcp sbcp : Sbcp.values()) {
            float value = b + 0.001f * sbcp.ordinal();
            config.set(sbcp, value);
        }

        SoftBodyWorldInfo info = body.getWorldInfo();
        info.setAirDensity(b + 0.03f);
        info.setGravity(new Vector3f(b + 0.031f, b + 0.032f, b + 0.033f));
        info.setMaxDisplacement(b + 0.034f);
        info.setWaterDensity(b + 0.035f);
        info.setWaterOffset(b + 0.036f);

        Vector3f normal = new Vector3f(b + 0.1f, b + 0.2f, b + 0.3f);
        normal.normalizeLocal();
        info.setWaterNormal(normal);

        config.setClusterIterations(n);
        config.setDriftIterations(n + 1);
        config.setPositionIterations(n + 2);
        config.setVelocityIterations(n + 3);

        int flags;
        if (flag) {
            flags = ConfigFlag.CL_RS | ConfigFlag.CL_SS | ConfigFlag.CL_SELF;
        } else {
            flags = ConfigFlag.SDF_RS | ConfigFlag.VF_SS;
        }
        config.setCollisionFlags(flags);

        SoftBodyMaterial material = body.getSoftMaterial();
        material.setAngularStiffness(b + 0.04f);
        material.setLinearStiffness(b + 0.041f);
        material.setVolumeStiffness(b + 0.042f);
    }
}
