/*
 * Copyright (c) 2019 jMonkeyEngine
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

import com.jme3.bullet.joints.Anchor;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.material.Material;
import com.jme3.math.Transform;
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
import jme3utilities.Validate;

/**
 * A physics-debug control to visualize an Anchor.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class AnchorDebugControl extends AbstractPhysicsDebugControl {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(AnchorDebugControl.class.getName());
    // *************************************************************************
    // fields

    /**
     * Anchor to visualize (not null)
     */
    final private Anchor anchor;
    /**
     * Geometry to visualize the anchor's endpoints
     */
    final private Geometry anchorGeometry;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled Control to visualize the specified Anchor.
     *
     * @param debugAppState which app state (not null, alias created)
     * @param anchor the Anchor to visualize (not null, alias created)
     */
    AnchorDebugControl(BulletDebugAppState debugAppState, Anchor anchor) {
        super(debugAppState);
        Validate.nonNull(anchor, "anchor");
        this.anchor = anchor;

        this.anchorGeometry = createAnchorGeometry();
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
        Mesh mesh = anchorGeometry.getMesh();
        FloatBuffer positionBuffer
                = mesh.getFloatBuffer(VertexBuffer.Type.Position);

        PhysicsSoftBody softBodyA = anchor.getSoftBody();
        int nodeIndex = anchor.nodeIndex();
        Vector3f aLocation = softBodyA.nodeLocation(nodeIndex, null);
        positionBuffer.put(0, aLocation.x);
        positionBuffer.put(1, aLocation.y);
        positionBuffer.put(2, aLocation.z);

        PhysicsRigidBody rigidBodyB = anchor.getRigidBody();
        Vector3f pivotInB = anchor.copyPivot(null); // TODO garbage
        Transform t = rigidBodyB.getTransform(null); // TODO garbage
        t.getScale().set(1f, 1f, 1f);
        Vector3f bLocation = t.transformVector(pivotInB, null);
        positionBuffer.put(3, bLocation.x);
        positionBuffer.put(4, bLocation.y);
        positionBuffer.put(5, bLocation.z);

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
            node.attachChild(anchorGeometry);

        } else if (spatial == null && this.spatial != null) {
            Node node = (Node) this.spatial;
            node.detachChild(anchorGeometry);
        }

        super.setSpatial(spatial);
    }
    // *************************************************************************
    // private methods

    /**
     * Create a Geometry to visualize the anchor.
     *
     * @return a new Geometry
     */
    private Geometry createAnchorGeometry() {
        Mesh mesh = new Mesh();

        int numVertices = 2;
        int numFloats = 3 * numVertices;
        FloatBuffer positions = BufferUtils.createFloatBuffer(numFloats);
        mesh.setBuffer(VertexBuffer.Type.Position, 3, positions);

        mesh.setMode(Mesh.Mode.Lines);
        mesh.setStreamed();

        Geometry result = new Geometry(anchor.toString(), mesh);
        result.setShadowMode(RenderQueue.ShadowMode.Off);

        SoftDebugAppState sdas = (SoftDebugAppState) debugAppState;
        Material material = sdas.getAnchorMaterial();
        result.setMaterial(material);

        return result;
    }
}
