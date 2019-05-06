/*
 * Copyright (c) 2009-2016 jMonkeyEngine
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

import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.bullet.util.NativeSoftBodyUtil;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer;
import java.nio.IntBuffer;
import java.util.logging.Logger;

/**
 * A physics-debug control to visualize a PhysicsSoftBody.
 *
 * @author Stephen Gold sgold@sonic.net
 *
 * Based on BulletSoftBodyDebugControl by dokthar.
 */
public class SoftBodyDebugControl extends AbstractPhysicsDebugControl {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(SoftBodyDebugControl.class.getName());
    // *************************************************************************
    // fields

    /**
     * geometry to visualize faces (not null)
     */
    final private Geometry facesGeometry;
    /**
     * geometry to visualize links (not null)
     */
    final private Geometry linksGeometry;
    /**
     * rigid body to visualize (not null)
     */
    final private PhysicsSoftBody body;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled control to visualize the specified body.
     *
     * @param debugAppState which app state (not null, alias created)
     * @param body which body to visualize (not null, alias created)
     */
    public SoftBodyDebugControl(BulletDebugAppState debugAppState,
            PhysicsSoftBody body) {
        super(debugAppState);
        this.body = body;

        facesGeometry = createFacesGeometry();
        linksGeometry = createLinksGeometry();
    }
    // *************************************************************************
    // AbstractPhysicsDebugControl methods

    /**
     * Update this control. Invoked once per frame during the logical-state
     * update, provided the control is enabled and added to a scene. Should be
     * invoked only by a subclass or by AbstractControl.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    protected void controlUpdate(float tpf) {
        boolean localFlag = false; // use physics-space coordinates
        boolean normalsFlag = false; // don't update mesh normals
        IntBuffer noIndexMap = null; // node indices = vertex indices

        if (linksGeometry != null) {
            Mesh mesh = linksGeometry.getMesh();
            NativeSoftBodyUtil.updateMesh(body, noIndexMap, mesh, localFlag,
                    normalsFlag);
            linksGeometry.updateModelBound(); // TODO needed?
        }

        if (facesGeometry != null) {
            Mesh mesh = facesGeometry.getMesh();
            NativeSoftBodyUtil.updateMesh(body, noIndexMap, mesh, localFlag,
                    normalsFlag);
            facesGeometry.updateModelBound(); // TODO needed?
        }
    }

    /**
     * Alter which Spatial is controlled. Invoked when the Control is added to
     * or removed from a Spatial. Should be invoked only by a subclass or from
     * Spatial. Do not invoke directly from user code.
     *
     * @param spatial the spatial to control (or null)
     */
    @Override
    public void setSpatial(Spatial spatial) {
        if (spatial instanceof Node) {
            Node node = (Node) spatial;
            if (linksGeometry != null) {
                node.attachChild(linksGeometry);
            }
            if (facesGeometry != null) {
                node.attachChild(facesGeometry);
            }
        } else if (spatial == null && this.spatial != null) {
            Node node = (Node) this.spatial;
            if (linksGeometry != null) {
                node.detachChild(linksGeometry);
            }
            if (facesGeometry != null) {
                node.detachChild(facesGeometry);
            }
        }
        super.setSpatial(spatial);
    }
    // *************************************************************************
    // private methods

    private Geometry createFacesGeometry() {
        Geometry result = null;
        if (body.countFaces() > 0) {
            Mesh mesh = new Mesh();
            mesh.setBuffer(VertexBuffer.Type.Index, 3, body.copyFaces(null));
            mesh.setBuffer(VertexBuffer.Type.Position, 3,
                    body.copyLocations(null));
            mesh.setMode(Mesh.Mode.Triangles);
            mesh.setStreamed();

            mesh.getFloatBuffer(VertexBuffer.Type.Position).clear();
            mesh.getIndexBuffer().getBuffer().clear();
            mesh.updateCounts();
            mesh.updateBound();

            result = new Geometry(body.toString() + " faces", mesh);
            result.setMaterial(debugAppState.DEBUG_RED);
        }

        return result;
    }

    private Geometry createLinksGeometry() {
        Geometry result = null;
        if (body.countLinks() > 0) {
            Mesh mesh = new Mesh();
            mesh.setBuffer(VertexBuffer.Type.Index, 2, body.copyLinks(null));
            mesh.setBuffer(VertexBuffer.Type.Position, 3,
                    body.copyLocations(null));
            mesh.setMode(Mesh.Mode.Lines);
            mesh.setStreamed();

            mesh.getFloatBuffer(VertexBuffer.Type.Position).clear();
            mesh.getIndexBuffer().getBuffer().clear();
            mesh.updateCounts();
            mesh.updateBound();

            result = new Geometry(body.toString() + " links", mesh);
            SoftDebugAppState sdas = (SoftDebugAppState) debugAppState;
            result.setMaterial(sdas.DEBUG_ORANGE);
        }

        return result;
    }
}
