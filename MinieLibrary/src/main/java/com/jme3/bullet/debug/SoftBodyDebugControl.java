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
     * Geometry to visualize clusters
     */
    private Geometry clustersGeometry = null;
    /**
     * Geometry to visualize faces
     */
    private Geometry facesGeometry = null;
    /**
     * Geometry to visualize links
     */
    private Geometry linksGeometry = null;
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
        Node node = (Node) spatial;
        /*
         * Ensure that the clustersGeometry mesh is correctly sized.
         */
        if (!isClustersGeometrySized()) {
            if (clustersGeometry != null) {
                node.detachChild(clustersGeometry);
            }
            clustersGeometry = createClustersGeometry();
            assert isClustersGeometrySized();
            if (clustersGeometry != null) {
                node.attachChild(clustersGeometry);
            }
        }
        /*
         * Ensure that the facesGeometry mesh is correctly sized.
         */
        if (!isFacesGeometrySized()) {
            if (facesGeometry != null) {
                node.detachChild(facesGeometry);
            }
            facesGeometry = createFacesGeometry();
            assert isFacesGeometrySized();
            if (facesGeometry != null) {
                node.attachChild(facesGeometry);
            }
        }
        /*
         * Ensure that the linksGeometry mesh is correctly sized.
         */
        if (!isLinksGeometrySized()) {
            if (linksGeometry != null) {
                node.detachChild(linksGeometry);
            }
            linksGeometry = createLinksGeometry();
            assert isLinksGeometrySized();
            if (linksGeometry != null) {
                node.attachChild(linksGeometry);
            }
        }

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
     * @param spatial the Spatial to control (or null)
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
     * Count how many clusters to visualize. Clusters are only visualized when
     * selected by a display filter.
     *
     * @return the count (&ge;0)
     */
    private int countClustersToVisualize() {
        SoftDebugAppState sdas = (SoftDebugAppState) debugAppState;
        BulletDebugAppState.DebugAppStateFilter filter
                = sdas.getClusterFilter();

        int result = 0;
        if (filter != null && filter.displayObject(body)) {
            result = body.countClusters();
        }

        return result;
    }

    /**
     * Determine how many mesh elements (lines or triangles) are in the
     * specified Geometry.
     *
     * @param geometry (may be null, unaffected)
     * @return the count (&ge;0)
     */
    private static int countElements(Geometry geometry) {
        int result = 0;
        if (geometry != null) {
            result = geometry.getMesh().getTriangleCount();
        }

        return result;
    }

    /**
     * Determine how many mesh vertices are in the specified Geometry.
     *
     * @param geometry (may be null, unaffected)
     * @return the count (&ge;0)
     */
    private static int countVertices(Geometry geometry) {
        int result = 0;
        if (geometry != null) {
            result = geometry.getMesh().getVertexCount();
        }

        return result;
    }

    /**
     * Create a Geometry to visualize the body's clusters.
     *
     * @return a new Geometry, or null if no clusters
     */
    private Geometry createClustersGeometry() {
        Geometry result = null;
        int numClustersToVisualize = countClustersToVisualize();
        if (numClustersToVisualize > 0) {
            Mesh mesh = createClustersMesh(numClustersToVisualize);
            result = new Geometry(body.toString() + " clusters", mesh);
            result.setShadowMode(RenderQueue.ShadowMode.Off);

            SoftDebugAppState sdas = (SoftDebugAppState) debugAppState;
            Material material = sdas.getClusterMaterial();
            result.setMaterial(material);
        }

        return result;
    }

    /**
     * Create a Points-mode Mesh to visualize the body's clusters.
     *
     * @param numClusters the number of clusters to visualize (&gt;0)
     * @return a new Mesh
     */
    private Mesh createClustersMesh(int numClusters) {
        assert numClusters > 0 : numClusters;

        Mesh mesh = new Mesh();

        int numFloats = 3 * numClusters;
        FloatBuffer centers = BufferUtils.createFloatBuffer(numFloats);
        mesh.setBuffer(VertexBuffer.Type.Position, 3, centers);

        mesh.setMode(Mesh.Mode.Points);
        mesh.setStreamed();

        return mesh;
    }

    /**
     * Create a Geometry to visualize the body's faces.
     *
     * @return a new Geometry, or null if no faces
     */
    private Geometry createFacesGeometry() {
        Geometry result = null;
        if (body.countFaces() > 0) {
            Mesh mesh = createFacesMesh();
            result = new Geometry(body.toString() + " faces", mesh);

            Material material = body.getDebugMaterial();
            if (material == null) { // use one of the default materials
                SoftDebugAppState sdas = (SoftDebugAppState) debugAppState;
                int numSides = body.debugNumSides();
                material = sdas.getFaceMaterial(numSides);
            }
            result.setMaterial(material);
        }

        return result;
    }

    /**
     * Create a Triangles-mode Mesh to visualize the body's faces.
     *
     * @return a new Mesh
     */
    private Mesh createFacesMesh() {
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

        return mesh;
    }

    /**
     * Create a Geometry to visualize the body's links.
     *
     * @return a new Geometry, or null if there are faces or no links
     */
    private Geometry createLinksGeometry() {
        Geometry result = null;
        if (body.countFaces() == 0 && body.countLinks() > 0) {
            Mesh mesh = createLinksMesh();
            result = new Geometry(body.toString() + " links", mesh);

            SoftDebugAppState sdas = (SoftDebugAppState) debugAppState;
            Material material = sdas.getLinkMaterial();
            result.setMaterial(material);
        }

        return result;
    }

    /**
     * Create a Lines-mode Mesh to visualize the body's links.
     *
     * @return a new Mesh
     */
    private Mesh createLinksMesh() {
        Mesh result = new Mesh();
        result.setBuffer(VertexBuffer.Type.Index, 2, body.copyLinks(null));

        int numFloats = 3 * body.countNodes();
        FloatBuffer locations = BufferUtils.createFloatBuffer(numFloats);
        result.setBuffer(VertexBuffer.Type.Position, 3, locations);

        result.setMode(Mesh.Mode.Lines);
        result.setStreamed();

        return result;
    }

    /**
     * Test whether the clustersGeometry mesh is correctly sized.
     *
     * @return true if correct size, otherwise false
     */
    private boolean isClustersGeometrySized() {
        int correctNumVertices = countClustersToVisualize();
        boolean result = countVertices(clustersGeometry) == correctNumVertices;

        return result;
    }

    /**
     * Test whether the facesGeometry mesh is correctly sized.
     *
     * @return true if correct size, otherwise false
     */
    private boolean isFacesGeometrySized() {
        int correctNumTriangles = body.countFaces();

        int correctNumVertices;
        if (correctNumTriangles == 0) {
            correctNumVertices = 0;
        } else {
            correctNumVertices = body.countNodes();
        }

        boolean result = countElements(facesGeometry) == correctNumTriangles
                && countVertices(facesGeometry) == correctNumVertices;

        return result;
    }

    /**
     * Test whether the linksGeometry mesh is correctly sized.
     *
     * @return true if correct size, otherwise false
     */
    private boolean isLinksGeometrySized() {
        int correctNumLines = 0;
        int correctNumVertices = 0;
        if (body.countFaces() == 0 && body.countLinks() > 0) {
            correctNumLines = body.countLinks();
            correctNumVertices = body.countNodes();
        }

        boolean result = countElements(linksGeometry) == correctNumLines
                && countVertices(linksGeometry) == correctNumVertices;

        return result;
    }
}
