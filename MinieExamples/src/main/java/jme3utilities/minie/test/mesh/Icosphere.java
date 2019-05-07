/*
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
package jme3utilities.minie.test.mesh;

import com.jme3.math.FastMath;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * Approximate a spherical mesh by subdividing the triangles of a regular
 * icosahedron. This produces a more isotropic layout of vertices than a UV
 * sphere.
 *
 * @author jayfella
 */
public class Icosphere extends Mesh {
    // *************************************************************************
    // constants and loggers

    /**
     * golden ratio = 1.618...
     */
    final public static float phi = (1f + FastMath.sqrt(5f)) / 2f;
    /**
     * vertex indices of the 20 triangular faces in a regular icosahedron
     */
    final private static int[] icoIndices = {
        0, 11, 5, 0, 5, 1, 0, 1, 7, 0, 7, 10, 0, 10, 11,
        1, 5, 9, 5, 11, 4, 11, 10, 2, 10, 7, 6, 7, 1, 8,
        3, 9, 4, 3, 4, 2, 3, 2, 6, 3, 6, 8, 3, 8, 9,
        4, 9, 5, 2, 4, 11, 6, 2, 10, 8, 6, 7, 9, 8, 1
    };
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(Icosphere.class.getName());
    /**
     * vertex locations in a regular icosahedron with radius=1.9021...
     */
    final private static Vector3f[] icoLocations = {
        new Vector3f(-1f, phi, 0f), new Vector3f(1f, phi, 0f),
        new Vector3f(-1f, -phi, 0f), new Vector3f(1f, -phi, 0f),
        new Vector3f(0f, -1f, phi), new Vector3f(0f, 1f, phi),
        new Vector3f(0f, -1f, -phi), new Vector3f(0f, 1f, -phi),
        new Vector3f(phi, 0f, -1f), new Vector3f(phi, 0f, 1f),
        new Vector3f(-phi, 0f, -1f), new Vector3f(-phi, 0f, 1f)
    };
    // *************************************************************************
    // fields

    /**
     * distance of each vertex from the center (&gt;0)
     */
    final private float radius;
    /**
     * next vertex index to be assigned
     */
    private int nextVertexIndex = 0;
    /**
     * map vertex indices to location vectors in mesh coordinates, all with
     * length=radius
     */
    final private List<Vector3f> locations = new ArrayList<>();
    /**
     * map vertex indices to normal vectors, all with length=1
     */
    final private List<Vector3f> normals = new ArrayList<>();
    /**
     * cache to avoid duplicate vertices: map index pairs to midpoint indices
     */
    final private Map<Long, Integer> midpointCache = new HashMap<>();
    // *************************************************************************
    // constructors

    /**
     * Instantiate an icosphere centered on (0,0,0) with the specified radius
     * and number of refinement steps:
     * <ul><li>
     * 0 steps &rarr; 12 vertices, 30 edges, 20 triangular faces
     * </li><li>
     * 1 step &rarr; 42 vertices, 120 edges, 80 triangular faces
     * </li><li>
     * 2 steps &rarr; 162 vertices, 480 edges, 320 triangular faces
     * </li><li>
     * etcetera
     * </ul>
     *
     * All faces are outward-facing.
     *
     * @param numRefineSteps the number of refinement steps (&ge;0)
     * @param radius the radius (in mesh units, &gt;0)
     */
    public Icosphere(int numRefineSteps, float radius) {
        Validate.nonNegative(numRefineSteps, "number of refinement steps");
        Validate.positive(radius, "radius");

        this.radius = radius;
        /*
         *  Add the 12 vertices of a regular icosahedron of radius=1.
         */
        for (Vector3f icoLocation : icoLocations) {
            addVertex(icoLocation);
        }
        /*
         *  Add the 20 triangular faces of a regular icosahedron.
         */
        List<Integer> faces = new ArrayList<>();
        for (int icoIndex : icoIndices) {
            faces.add(icoIndex);
        }

        for (int stepIndex = 0; stepIndex < numRefineSteps; ++stepIndex) {
            List<Integer> newFaces = new ArrayList<>();
            /*
             * A refinement step: for each triangle in faces,
             * add 4 triangles to newFaces.
             */
            for (int j = 0; j < faces.size(); j += 3) {
                int v1 = faces.get(j);
                int v2 = faces.get(j + 1);
                int v3 = faces.get(j + 2);

                int a = midpointIndex(v1, v2);
                int b = midpointIndex(v2, v3);
                int c = midpointIndex(v3, v1);

                newFaces.add(v1);
                newFaces.add(a);
                newFaces.add(c);

                newFaces.add(v2);
                newFaces.add(b);
                newFaces.add(a);

                newFaces.add(v3);
                newFaces.add(c);
                newFaces.add(b);

                newFaces.add(a);
                newFaces.add(b);
                newFaces.add(c);
            }

            faces = newFaces;
        }

        Vector3f[] posArray = new Vector3f[locations.size()];
        posArray = locations.toArray(posArray);

        int[] indexArray = new int[faces.size()];
        for (int i = 0; i < faces.size(); ++i) {
            indexArray[i] = faces.get(i);
        }

        Vector2f[] uvArray = getUV(posArray);

        Vector3f[] normArray = new Vector3f[normals.size()];
        normArray = normals.toArray(normArray);

        FloatBuffer posBuffer = BufferUtils.createFloatBuffer(posArray);
        setBuffer(VertexBuffer.Type.Position, 3, posBuffer);

        IntBuffer indexBuffer = BufferUtils.createIntBuffer(indexArray);
        setBuffer(VertexBuffer.Type.Index, 3, indexBuffer);

        FloatBuffer uvBuffer = BufferUtils.createFloatBuffer(uvArray);
        setBuffer(VertexBuffer.Type.TexCoord, 2, uvBuffer);

        FloatBuffer normBuffer = BufferUtils.createFloatBuffer(normArray);
        setBuffer(VertexBuffer.Type.Normal, 3, normBuffer);

        updateBound();
    }
    // *************************************************************************
    // private methods

    /**
     * Add a vertex to the lists of locations and normals.
     *
     * @param location the approximate vertex location (in mesh coordinates, not
     * null, unaffected)
     * @return the index assigned to the new vertex (&ge;0)
     */
    private int addVertex(Vector3f location) {
        float length = location.length();
        locations.add(location.mult(radius / length));
        normals.add(location.divide(length));
        int result = nextVertexIndex;
        ++nextVertexIndex;

        return result;
    }

    /**
     * Transform 3-D Cartesian coordinates to longitude and latitude.
     *
     * @param input the location to transform (z=distance north of the
     * equatorial plane, not null, unaffected)
     * @return a new vector (x=west longitude in radians, y=north latitude in
     * radians)
     */
    private Vector2f cartesianToSpherical(Vector3f input) {
        Vector2f result = new Vector2f();
        float length = input.length();

        if (input.x != 0f || input.y != 0f) {
            result.x = -FastMath.atan2(input.y, input.x);
        } else {
            result.x = 0f;
        }

        if (length > 0f) {
            result.y = FastMath.asin(input.z / length);
        } else {
            result.y = 0f;
        }

        return result;
    }

    /**
     * Generate texture coordinates for the specified mesh locations using a
     * latitude/longitude mapping.
     *
     * @param meshLocations the locations (in mesh coordinates, not null,
     * unaffected)
     * @return a new array of new texture-coordinate vectors, each component
     * &ge;0 and &le;1
     */
    private Vector2f[] getUV(Vector3f[] meshLocations) {
        int numVertices = meshLocations.length;
        Vector2f[] result = new Vector2f[numVertices];

        for (int i = 0; i < numVertices; ++i) {
            result[i] = cartesianToSpherical(meshLocations[i]);
            result[i].x = 0.5f + result[i].x / FastMath.TWO_PI;
            result[i].y = 0.5f + result[i].y / FastMath.PI;
        }

        return result;
    }

    /**
     * Determine the index of the vertex halfway between the indexed vertices.
     *
     * @param p1 the index of the 1st input vertex (&ge;0)
     * @param p2 the index of the 2nd input vertex (&ge;0)
     * @return the midpoint index (&ge;0)
     */
    private int midpointIndex(int p1, int p2) {
        /*
         * Check whether the midpoint has already been assigned an index.
         */
        boolean firstIsSmaller = p1 < p2;
        long smallerIndex = firstIsSmaller ? p1 : p2;
        long greaterIndex = firstIsSmaller ? p2 : p1;
        long key = (smallerIndex << 32) + greaterIndex;
        Integer cachedIndex = midpointCache.get(key);
        if (cachedIndex != null) {
            return cachedIndex;
        }
        /*
         * The midpoint vertex is not in the cache: calculate its location.
         */
        Vector3f loc1 = locations.get(p1);
        Vector3f loc2 = locations.get(p2);
        Vector3f middleLocation = MyVector3f.midpoint(loc1, loc2, null);
        /*
         * addVertex() adjusts the location to the sphere.
         */
        int newIndex = addVertex(middleLocation);
        /*
         * Add the new vertex to the midpoint cache.
         */
        midpointCache.put(key, newIndex);

        return newIndex;
    }
}
