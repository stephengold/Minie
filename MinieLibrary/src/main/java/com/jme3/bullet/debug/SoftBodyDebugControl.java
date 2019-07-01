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

import com.jme3.bullet.collision.shapes.infos.DebugMeshNormals;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.bullet.util.NativeSoftBodyUtil;
import com.jme3.material.Material;
import com.jme3.math.Quaternion;
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
    /**
     * local copy of {@link com.jme3.math.Quaternion#IDENTITY}
     */
    final private static Quaternion rotateIdentity = new Quaternion();
    // *************************************************************************
    // fields

    /**
     * geometry to visualize clusters
     */
    final private Geometry clustersGeometry;
    /**
     * geometry to visualize faces
     */
    final private Geometry facesGeometry;
    /**
     * geometry to visualize links
     */
    final private Geometry linksGeometry;
    /**
     * soft body to visualize (not null)
     */
    final private PhysicsSoftBody body;
    /**
     * temporary storage for one vector per thread
     */
    final private static ThreadLocal<Vector3f> threadTmpVector
            = new ThreadLocal<Vector3f>() {
        @Override
        protected Vector3f initialValue() {
            return new Vector3f();
        }
    };
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled Control to visualize the specified body.
     *
     * @param debugAppState which app state (not null, alias created)
     * @param body which body to visualize (not null, alias created)
     */
    public SoftBodyDebugControl(BulletDebugAppState debugAppState,
            PhysicsSoftBody body) {
        super(debugAppState);
        this.body = body;

        clustersGeometry = createClustersGeometry();
        facesGeometry = createFacesGeometry();
        linksGeometry = createLinksGeometry();
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
        // TODO check for changes in the number of anchors/links/faces/clusters

        boolean localFlag = true; // copy local coords, not physics-space ones

        if (clustersGeometry != null) {
            Mesh mesh = clustersGeometry.getMesh();
            NativeSoftBodyUtil.updateClusterMesh(body, mesh, localFlag);
        }

        DebugMeshNormals normals = body.debugMeshNormals();
        IntBuffer noIndexMap = null; // node indices = vertex indices
        boolean normalsFlag = (normals != DebugMeshNormals.None);
        Transform noTransform = null; // physics locations = mesh positions

        if (linksGeometry != null) {
            Mesh mesh = linksGeometry.getMesh();
            NativeSoftBodyUtil.updateMesh(body, noIndexMap, mesh, localFlag,
                    normalsFlag, noTransform);
        }

        if (facesGeometry != null) {
            Mesh mesh = facesGeometry.getMesh();
            NativeSoftBodyUtil.updateMesh(body, noIndexMap, mesh, localFlag,
                    normalsFlag, noTransform);

            Material material = body.getDebugMaterial();
            if (material == null) { // apply one of the default materials
                int numSides = body.debugNumSides();
                SoftDebugAppState sdas = (SoftDebugAppState) debugAppState;
                material = sdas.getFaceMaterial(numSides);
            }
            facesGeometry.setMaterial(material);
        }

        Vector3f center = threadTmpVector.get();
        body.getPhysicsLocation(center);
        applyPhysicsTransform(center, rotateIdentity);
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
            assert this.spatial == null;
            spatial.setCullHint(Spatial.CullHint.Never);
            spatial.setShadowMode(RenderQueue.ShadowMode.Cast);

            Node node = (Node) spatial;
            if (clustersGeometry != null) {
                node.attachChild(clustersGeometry);
            }
            if (facesGeometry != null) {
                node.attachChild(facesGeometry);
            }
            if (linksGeometry != null) {
                node.attachChild(linksGeometry);
            }
        } else if (spatial == null && this.spatial != null) {
            Node node = (Node) this.spatial;
            if (clustersGeometry != null) {
                node.detachChild(clustersGeometry);
            }
            if (facesGeometry != null) {
                node.detachChild(facesGeometry);
            }
            if (linksGeometry != null) {
                node.detachChild(linksGeometry);
            }
        }
        super.setSpatial(spatial);
    }
    // *************************************************************************
    // private methods

    /**
     * Create a Geometry to visualize the body's clusters.
     *
     * @return a new Geometry, or null if no clusters
     */
    private Geometry createClustersGeometry() {
        Geometry result = null;
        int numClusters = body.countClusters();

        SoftDebugAppState sdas = (SoftDebugAppState) debugAppState;
        BulletDebugAppState.DebugAppStateFilter filter
                = sdas.getClusterFilter();
        if (filter == null || !filter.displayObject(body)) {
            numClusters = 0;
        }

        if (numClusters > 0) {
            Mesh mesh = new Mesh();

            int numFloats = 3 * numClusters;
            FloatBuffer centers = BufferUtils.createFloatBuffer(numFloats);
            mesh.setBuffer(VertexBuffer.Type.Position, 3, centers);

            mesh.setMode(Mesh.Mode.Points);
            mesh.setStreamed();

            result = new Geometry(body.toString() + " clusters", mesh);
            result.setShadowMode(RenderQueue.ShadowMode.Off);

            Material material = sdas.getClusterMaterial();
            result.setMaterial(material);
        }

        return result;
    }

    /**
     * Create a Geometry to visualize the body's faces.
     *
     * @return a new Geometry, or null if no faces
     */
    private Geometry createFacesGeometry() {
        Geometry result = null;
        if (body.countFaces() > 0) {
            Mesh mesh = new Mesh();
            mesh.setBuffer(VertexBuffer.Type.Index, 3, body.copyFaces(null));

            DebugMeshInitListener listener = body.debugMeshInitListener();
            DebugMeshNormals option = body.debugMeshNormals();
            if (listener == null) {
                /*
                 * Allocate buffers for positions and normals.
                 */
                int numNodes = body.countNodes();
                int numFloats = 3 * numNodes;
                FloatBuffer pos = BufferUtils.createFloatBuffer(numFloats);
                mesh.setBuffer(VertexBuffer.Type.Position, 3, pos);
                if (option != DebugMeshNormals.None) {
                    FloatBuffer norm = BufferUtils.createFloatBuffer(numFloats);
                    mesh.setBuffer(VertexBuffer.Type.Normal, 3, norm);
                }

            } else {
                /*
                 * Calculate positions, normals, and bounds in world coords.
                 */
                FloatBuffer pos = body.copyLocations(null);
                mesh.setBuffer(VertexBuffer.Type.Position, 3, pos);
                if (option != DebugMeshNormals.None) {
                    FloatBuffer norm = body.copyNormals(null);
                    mesh.setBuffer(VertexBuffer.Type.Normal, 3, norm);
                }
                mesh.updateBound();

                listener.debugMeshInit(mesh);
                /*
                 * After debugMeshInit, positions are calculated in
                 * local coordinates!
                 */
            }

            mesh.setMode(Mesh.Mode.Triangles);
            mesh.setStreamed();

            result = new Geometry(body.toString() + " faces", mesh);
            Material material = body.getDebugMaterial();
            if (material == null) { // use one of the default materials
                int numSides = body.debugNumSides();
                SoftDebugAppState sdas = (SoftDebugAppState) debugAppState;
                material = sdas.getFaceMaterial(numSides);
            }
            result.setMaterial(material);
        }

        return result;
    }

    /**
     * Create a Geometry to visualize the body's links.
     *
     * @return a new Geometry, or null if there are faces or no links
     */
    private Geometry createLinksGeometry() {
        Geometry result = null;
        if (body.countFaces() == 0 && body.countLinks() > 0) {
            Mesh mesh = new Mesh();
            mesh.setBuffer(VertexBuffer.Type.Index, 2, body.copyLinks(null));

            int numFloats = 3 * body.countNodes();
            FloatBuffer locations = BufferUtils.createFloatBuffer(numFloats);
            mesh.setBuffer(VertexBuffer.Type.Position, 3, locations);

            mesh.setMode(Mesh.Mode.Lines);
            mesh.setStreamed();

            result = new Geometry(body.toString() + " links", mesh);
            SoftDebugAppState sdas = (SoftDebugAppState) debugAppState;
            Material material = sdas.getLinkMaterial();
            result.setMaterial(material);
        }

        return result;
    }
}
