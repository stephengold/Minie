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
package com.jme3.bullet.debug;

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
import com.jme3.scene.debug.Arrow;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.logging.Logger;
import jme3utilities.MeshNormals;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyVector3f;

/**
 * A physics-debug control to visualize a PhysicsSoftBody.
 *
 * @author Stephen Gold sgold@sonic.net
 *
 * Based on BulletSoftBodyDebugControl by dokthar.
 */
class SoftBodyDebugControl extends AbstractPhysicsDebugControl {
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
     * temporary buffer for locations, allocated lazily
     */
    private static FloatBuffer tmpLocations = null;
    /**
     * temporary buffer for velocities, allocated lazily
     */
    private static FloatBuffer tmpVelocities = null;
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
     * Geometry to visualize pinned nodes
     */
    private Geometry pinsGeometry = null;
    /**
     * geometries to visualize velocity vectors
     */
    private Geometry[] velocityGeometries = null;
    /**
     * soft body to visualize (not null)
     */
    final private PhysicsSoftBody body;
    /**
     * temporary storage
     */
    final private static Vector3f tmpCenter = new Vector3f();
    final private static Vector3f tmpVector = new Vector3f();
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled Control to visualize the specified body.
     *
     * @param debugAppState which app state (not null, alias created)
     * @param body which body to visualize (not null, alias created)
     */
    SoftBodyDebugControl(
            BulletDebugAppState debugAppState, PhysicsSoftBody body) {
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

        // Ensure that the clustersGeometry mesh is correctly sized.
        if (!isClustersGeometrySized()) {
            if (clustersGeometry != null) {
                node.detachChild(clustersGeometry);
            }
            this.clustersGeometry = createClustersGeometry();
            assert isClustersGeometrySized();
            if (clustersGeometry != null) {
                node.attachChild(clustersGeometry);
            }
        }

        // Ensure that the facesGeometry mesh is correctly sized.
        if (!isFacesGeometrySized()) {
            if (facesGeometry != null) {
                node.detachChild(facesGeometry);
            }
            this.facesGeometry = createFacesGeometry();
            assert isFacesGeometrySized();
            if (facesGeometry != null) {
                node.attachChild(facesGeometry);
            }
        }

        // Ensure that the linksGeometry mesh is correctly sized.
        if (!isLinksGeometrySized()) {
            if (linksGeometry != null) {
                node.detachChild(linksGeometry);
            }
            this.linksGeometry = createLinksGeometry();
            assert isLinksGeometrySized();
            if (linksGeometry != null) {
                node.attachChild(linksGeometry);
            }
        }

        // Ensure that the pinsGeometry mesh is correctly sized.
        if (!isPinsGeometrySized()) {
            if (pinsGeometry != null) {
                node.detachChild(pinsGeometry);
            }
            this.pinsGeometry = createPinsGeometry();
            assert isPinsGeometrySized();
            if (pinsGeometry != null) {
                node.attachChild(pinsGeometry);
            }
        }

        // Ensure that the velocityGeometries array is correctly sized.
        if (!areVelocitiesSized()) {
            if (velocityGeometries != null) {
                for (Geometry geometry : velocityGeometries) {
                    node.detachChild(geometry);
                }
            }
            velocityGeometries = createVelocityGeometries();
            assert areVelocitiesSized();
            if (velocityGeometries != null) {
                for (Geometry geometry : velocityGeometries) {
                    node.attachChild(geometry);
                }
            }
        }

        boolean localFlag = true; // copy local coords, not physics-space ones
        if (clustersGeometry != null) {
            Mesh mesh = clustersGeometry.getMesh();
            NativeSoftBodyUtil.updateClusterMesh(body, mesh, localFlag);
        }

        MeshNormals normals = body.debugMeshNormals();
        IntBuffer noIndexMap = null; // node indices = vertex indices
        boolean normalsFlag = (normals != MeshNormals.None);
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

        if (pinsGeometry != null) {
            Mesh mesh = pinsGeometry.getMesh();
            NativeSoftBodyUtil.updatePinMesh(body, mesh, localFlag);
        }

        body.getPhysicsLocation(tmpCenter);
        if (velocityGeometries != null) {
            int numArrows = velocityGeometries.length;
            int numFloats = MyVector3f.numAxes * numArrows;
            if (tmpLocations == null
                    || numFloats > tmpLocations.capacity()) {
                tmpLocations = BufferUtils.createFloatBuffer(numFloats);
            }
            if (tmpVelocities == null
                    || numFloats > tmpVelocities.capacity()) {
                tmpVelocities = BufferUtils.createFloatBuffer(numFloats);
            }

            if (countClustersToVisualize() > 0) { // cluster velocities
                body.copyClusterCenters(tmpLocations);
                body.copyClusterVelocities(tmpVelocities);
            } else { // node velocities
                body.copyLocations(tmpLocations);
                body.copyVelocities(tmpVelocities);
            }

            for (int arrowIndex = 0; arrowIndex < numArrows; ++arrowIndex) {
                int startPosition = MyVector3f.numAxes * arrowIndex;
                Geometry geometry = velocityGeometries[arrowIndex];
                MyBuffer.get(tmpLocations, startPosition, tmpVector);
                tmpVector.subtractLocal(tmpCenter);
                geometry.setLocalTranslation(tmpVector);

                Mesh mesh = geometry.getMesh();
                Arrow arrow = (Arrow) mesh;
                MyBuffer.get(tmpVelocities, startPosition, tmpVector);
                arrow.setArrowExtent(tmpVector);
            }
        }

        applyPhysicsTransform(tmpCenter, rotateIdentity);
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
     * Test whether the velocityGeometries array is correctly sized.
     *
     * @return true if correct size, otherwise false
     */
    private boolean areVelocitiesSized() {
        int numArrows = 0;
        if (velocityGeometries != null) {
            numArrows = velocityGeometries.length;
        }
        int correctNumArrows = countVelocitiesToVisualize();
        boolean result = (numArrows == correctNumArrows);

        return result;
    }

    /**
     * Determine how many clusters to visualize. Clusters are only visualized
     * when selected by a display filter.
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
     * Determine how many velocity vectors to visualize. Velocities are only
     * visualized when selected by a display filter.
     *
     * @return the count (&ge;0)
     */
    private int countVelocitiesToVisualize() {
        SoftDebugAppState sdas = (SoftDebugAppState) debugAppState;
        BulletDebugAppState.DebugAppStateFilter filter
                = sdas.getConfiguration().getVelocityVectorFilter();

        int result = 0;
        if (filter != null && filter.displayObject(body)) {
            result = countClustersToVisualize();
            if (result == 0) { // visualize node velocities
                result = body.countNodes();
            }
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
     * @return a new Geometry, or null if no clusters to visualize
     */
    private Geometry createClustersGeometry() {
        Geometry result = null;

        int numClustersToVisualize = countClustersToVisualize();
        if (numClustersToVisualize > 0) {
            Mesh mesh = createPointsMesh(numClustersToVisualize);
            result = new Geometry(body + " clusters", mesh);
            result.setShadowMode(RenderQueue.ShadowMode.Off);

            SoftDebugAppState sdas = (SoftDebugAppState) debugAppState;
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
            Mesh mesh = createFacesMesh();
            result = new Geometry(body + " faces", mesh);

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
        MeshNormals option = body.debugMeshNormals();
        if (listener == null) {
            // Allocate buffers for positions and normals.
            int numNodes = body.countNodes();
            int numFloats = 3 * numNodes;
            FloatBuffer pos = BufferUtils.createFloatBuffer(numFloats);
            mesh.setBuffer(VertexBuffer.Type.Position, 3, pos);
            if (option != MeshNormals.None) {
                FloatBuffer norm = BufferUtils.createFloatBuffer(numFloats);
                mesh.setBuffer(VertexBuffer.Type.Normal, 3, norm);
            }

        } else {
            // Calculate positions, normals, and bounds in world coords.
            FloatBuffer pos = body.copyLocations(null);
            mesh.setBuffer(VertexBuffer.Type.Position, 3, pos);
            if (option != MeshNormals.None) {
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
            result = new Geometry(body + " links", mesh);

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
     * Create a Geometry to visualize the body's pinned nodes.
     *
     * @return a new Geometry, or null if no pinned nodes
     */
    private Geometry createPinsGeometry() {
        Geometry result = null;

        int numNodesToVisualize = body.countPinnedNodes();
        if (numNodesToVisualize > 0) {
            Mesh mesh = createPointsMesh(numNodesToVisualize);
            result = new Geometry(body + " pins", mesh);
            result.setShadowMode(RenderQueue.ShadowMode.Off);

            SoftDebugAppState sdas = (SoftDebugAppState) debugAppState;
            Material material = sdas.getPinMaterial();
            result.setMaterial(material);
        }

        return result;
    }

    /**
     * Create a Points-mode Mesh.
     *
     * @param numPoints the number of points to visualize (&gt;0)
     * @return a new Mesh
     */
    private static Mesh createPointsMesh(int numPoints) {
        assert numPoints > 0 : numPoints;

        Mesh result = new Mesh();

        int numFloats = MyVector3f.numAxes * numPoints;
        FloatBuffer centers = BufferUtils.createFloatBuffer(numFloats);
        result.setBuffer(
                VertexBuffer.Type.Position, MyVector3f.numAxes, centers);

        result.setMode(Mesh.Mode.Points);
        result.setStreamed();

        return result;
    }

    /**
     * Create geometries to visualize velocity vectors.
     *
     * @return a new array of geometries, or null if no velocities to visualize
     */
    private Geometry[] createVelocityGeometries() {
        Geometry[] result = null;

        int numGeometries = countVelocitiesToVisualize();
        if (numGeometries > 0) {
            result = new Geometry[numGeometries];
            for (int geomI = 0; geomI < numGeometries; ++geomI) {
                Arrow mesh = new Arrow(tmpVector);
                String name = String.format("velocity of %s[%d]", body, geomI);
                Geometry geometry = new Geometry(name, mesh);
                result[geomI] = geometry;

                Material material = debugAppState.getVelocityVectorMaterial();
                geometry.setMaterial(material);
                geometry.setShadowMode(RenderQueue.ShadowMode.Off);
            }
        }

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

    /**
     * Test whether the pinsGeometry mesh is correctly sized.
     *
     * @return true if correct size, otherwise false
     */
    private boolean isPinsGeometrySized() {
        int correctNumVertices = body.countPinnedNodes();
        boolean result = countVertices(pinsGeometry) == correctNumVertices;

        return result;
    }
}
