/*
 * Copyright (c) 2019-2023 jMonkeyEngine
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
package com.jme3.bullet.debug;

import com.jme3.bullet.joints.JointEnd;
import com.jme3.bullet.joints.SoftPhysicsJoint;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.material.Material;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.logging.Logger;

/**
 * A physics-debug control to visualize a SoftPhysicsJoint.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class SoftJointDebugControl extends AbstractPhysicsDebugControl {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(SoftJointDebugControl.class.getName());
    // *************************************************************************
    // fields

    /**
     * Geometry to visualize the joint's endpoints (not null)
     */
    final private Geometry endsGeometry;
    /**
     * joint to visualize (not null)
     */
    final private SoftPhysicsJoint joint;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled Control to visualize the specified
     * SoftPhysicsJoint.
     *
     * @param debugAppState which app state (not null, alias created)
     * @param joint the joint to visualize (not null, alias created)
     */
    SoftJointDebugControl(
            BulletDebugAppState debugAppState, SoftPhysicsJoint joint) {
        super(debugAppState);
        this.joint = joint;

        this.endsGeometry = createEndsGeometry();
    }
    // *************************************************************************
    // AbstractPhysicsDebugControl methods

    /**
     * Update this Control. Invoked once per frame during the logical-state
     * update, provided the Control is enabled and added to a scene. Should be
     * invoked only by a subclass or by AbstractControl.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    protected void controlUpdate(float tpf) {
        Mesh mesh = endsGeometry.getMesh();
        FloatBuffer positionBuffer
                = mesh.getFloatBuffer(VertexBuffer.Type.Position);

        PhysicsSoftBody softBodyA = joint.getSoftBodyA();
        int clusterIndex = joint.clusterIndexA();
        Vector3f aLocation
                = softBodyA.clusterCenter(clusterIndex, null); // TODO garbage
        positionBuffer.put(0, aLocation.x);
        positionBuffer.put(1, aLocation.y);
        positionBuffer.put(2, aLocation.z);

        Vector3f bLocation = new Vector3f(); // TODO garbage
        if (joint.isSoftSoft()) {
            PhysicsSoftBody softBodyB = joint.getSoftBodyB();
            clusterIndex = joint.clusterIndexB();
            softBodyB.clusterCenter(clusterIndex, bLocation);
        } else {
            PhysicsBody rigidBodyB
                    = joint.getBody(JointEnd.B);
            rigidBodyB.getPhysicsLocation(bLocation);
        }
        positionBuffer.put(3, bLocation.x);
        positionBuffer.put(4, bLocation.y);
        positionBuffer.put(5, bLocation.z);

        // TODO also visualize the joint's axis or location
        mesh.getBuffer(VertexBuffer.Type.Position).setUpdateNeeded();
        mesh.updateBound();
    }

    /**
     * Alter which Spatial is controlled. Invoked when the Control is added to
     * or removed from a Spatial. Should be invoked only by a subclass or from
     * Spatial. Do not invoke directly from user code.
     *
     * @param spatial the Spatial to control (or null)
     */
    @Override
    public void setSpatial(Spatial spatial) {
        if (spatial instanceof Node) {
            assert this.spatial == null;
            Node node = (Node) spatial;
            node.attachChild(endsGeometry);
        } else if (spatial == null && this.spatial != null) {
            Node node = (Node) this.spatial;
            node.detachChild(endsGeometry);
        }

        super.setSpatial(spatial);
    }
    // *************************************************************************
    // private methods

    /**
     * Create a Geometry to visualize the endpoints.
     *
     * @return a new Geometry
     */
    private Geometry createEndsGeometry() {
        Mesh mesh = new Mesh();

        int numVertices = 2;
        int numFloats = 3 * numVertices;
        FloatBuffer positions = BufferUtils.createFloatBuffer(numFloats);
        mesh.setBuffer(VertexBuffer.Type.Position, 3, positions);

        mesh.setMode(Mesh.Mode.Lines);
        mesh.setStreamed();

        Geometry result = new Geometry(joint.toString(), mesh);
        result.setShadowMode(RenderQueue.ShadowMode.Off);

        SoftDebugAppState sdas = (SoftDebugAppState) debugAppState;
        Material material = sdas.getAnchorMaterial();
        result.setMaterial(material);

        return result;
    }
}
