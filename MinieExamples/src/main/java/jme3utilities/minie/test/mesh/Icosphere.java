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
    private static final float t = (1.0f + FastMath.sqrt(5.0f)) / 2.0f;
    /**
     * vertex locations in a regular icosahedron with radius=1.9021...
     */
    private static final Vector3f[] ico_vertices = {
            new Vector3f(-1,  t,  0), new Vector3f( 1,  t,  0), new Vector3f(-1, -t,  0), new Vector3f( 1, -t,  0),
            new Vector3f( 0, -1,  t), new Vector3f( 0,  1,  t), new Vector3f( 0, -1, -t), new Vector3f( 0,  1, -t),
            new Vector3f( t,  0, -1), new Vector3f( t,  0,  1), new Vector3f(-t,  0, -1), new Vector3f(-t,  0,  1)
    };
    /**
     * vertex indices of the 20 triangular faces in a regular icosahedron
     */
    private static final int[] ico_indices = {
            0, 11, 5,   0, 5, 1,    0, 1, 7,    0, 7, 10,   0, 10, 11,
            1, 5, 9,    5, 11, 4,   11, 10, 2,  10, 7, 6,   7, 1, 8,
            3, 9, 4,    3, 4, 2,    3, 2, 6,    3, 6, 8,    3, 8, 9,
            4, 9, 5,    2, 4, 11,   6, 2, 10,   8, 6, 7,    9, 8, 1
    };
    // *************************************************************************
    // fields

    /**
     * distance of each vertex from the center (&gt;0)
     */
    private final float radius;
    /**
     * next vertex index to be assigned
     */
    private int index;
    /**
     * cache to avoid duplicate vertices: map index pairs to midpoint indices
     */
    private Map<Long, Integer> middlePointIndexCache = new HashMap<>();
    /**
     * map vertex indices to location vectors in mesh coordinates, all with
     * length=radius
     */
    private List<Vector3f> vertices = new ArrayList<>();
    /**
     * map vertex indices to normal vectors, all with length=1
     */
    private List<Vector3f> normals = new ArrayList<>();
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
     * @param recursionLevel the number of refinement steps (&ge;0)
     * @param radius the radius (in mesh units, &gt;0)
     */
    public Icosphere(int recursionLevel, float radius) {

        this.radius = radius;
        this.index = 0;

        /*
         *  Add the 12 vertices of a regular icosahedron of radius=1.
         */
        for (int i = 0; i < ico_vertices.length; i++) {
            addVertex(ico_vertices[i]);
        }
        /*
         *  Add the 20 triangular faces of a regular icosahedron.
         */
        List<Integer> faces = new ArrayList<>();
        for (int i = 0; i < ico_indices.length; i++) {
            faces.add(ico_indices[i]);
        }

        for (int i = 0; i < recursionLevel; i++)
        {
            List<Integer> faces2 = new ArrayList<>();
            /*
             * A refinement step: for each triangle in faces,
             * add 4 triangles to faces2.
             */
            for (int j = 0; j < faces.size(); j += 3)
            {
                int v1 = faces.get(j + 0);
                int v2 = faces.get(j + 1);
                int v3 = faces.get(j + 2);

                int a = getMiddlePoint(v1, v2);
                int b = getMiddlePoint(v2, v3);
                int c = getMiddlePoint(v3, v1);

                faces2.add(v1);
                faces2.add(a);
                faces2.add(c);

                faces2.add(v2);
                faces2.add(b);
                faces2.add(a);

                faces2.add(v3);
                faces2.add(c);
                faces2.add(b);

                faces2.add(a);
                faces2.add(b);
                faces2.add(c);
            }

            faces = faces2;
        }

        Vector3f[] vertexArray = new Vector3f[vertices.size()];
        vertexArray = vertices.toArray(vertexArray);

        int[] faceArray = new int[faces.size()];

        for (int i = 0; i < faces.size(); i++) {
            faceArray[i] = faces.get(i);
        }

        Vector2f[] texArray = getUV(vertexArray);

        Vector3f[] normArray = new Vector3f[normals.size()];
        normArray = normals.toArray(normArray);

        FloatBuffer pb = BufferUtils.createFloatBuffer(vertexArray);
        setBuffer(VertexBuffer.Type.Position, 3, pb);

        IntBuffer ib = BufferUtils.createIntBuffer(faceArray);
        setBuffer(VertexBuffer.Type.Index, 3, ib);

        FloatBuffer tb = BufferUtils.createFloatBuffer(texArray);
        setBuffer(VertexBuffer.Type.TexCoord, 2, tb);

        FloatBuffer nb = BufferUtils.createFloatBuffer(normArray);
        setBuffer(VertexBuffer.Type.Normal, 3, nb);

        updateBound();
    }
    // *************************************************************************
    // private methods

    /**
     * Add a vertex to the lists of locations and normals.
     *
     * @param p the approximate vertex location (in mesh coordinates, not null,
     * unaffected)
     * @return the index assigned to the new vertex (&ge;0)
     */
    private int addVertex(Vector3f p) {
        float length = FastMath.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        vertices.add(p.divide(length).mult(radius));
        normals.add(p.divide(length));
        return index++;
    }

    /**
     * Determine the index of the vertex halfway between the indexed vertices.
     *
     * @param p1 the index of the 1st input vertex (&ge;0)
     * @param p2 the index of the 2nd input vertex (&ge;0)
     * @return the midpoint index (&ge;0)
     */
    private int getMiddlePoint(int p1, int p2) {
        /*
         * Check whether the midpoint has already been assigned an index.
         */
        boolean firstIsSmaller = p1 < p2;
        long smallerIndex = firstIsSmaller ? p1 : p2;
        long greaterIndex = firstIsSmaller ? p2 : p1;
        long key = (smallerIndex << 32) + greaterIndex;

        Integer ret = this.middlePointIndexCache.get(key);

        if (ret != null) {
            return ret;
        }
        /*
         * The midpoint vertex is not in the cache: calculate its location.
         */
        Vector3f point1 = this.vertices.get(p1);
        Vector3f point2 = this.vertices.get(p2);

        Vector3f middle = point1.add(point2).divideLocal(2.0f);

        /*
         * addVertex() adjusts the location to the sphere.
         */
        int i = addVertex(middle);
        /*
         * Add the new vertex to the midpoint cache.
         */
        this.middlePointIndexCache.put(key, i);
        return i;
    }

    /**
     * Transform 3-D Cartesian coordinates to longitude and latitude.
     *
     * @param point the location to transform (z=distance north of the
     * equatorial plane, not null, unaffected)
     * @return a new vector (x=west longitude in radians, y=north latitude in
     * radians)
     */
    private Vector2f cartToLL(Vector3f point) {

        Vector2f coord = new Vector2f();
        float norm = point.length();

        if(point.x != 0.0f || point.y != 0.0f)
            coord.x = -FastMath.atan2(point.y, point.x);
        else
            coord.x = 0.0f;

        if(norm > 0.0f)
            coord.y = FastMath.asin(point.z / norm);
        else
            coord.y = 0.0f;

        return coord;
    }

    /**
     * Generate texture coordinates for the specified mesh locations using a
     * latitude/longitude mapping.
     *
     * @param vertices the locations (in mesh coordinates, not null, unaffected)
     * @return a new array of new texture-coordinate vectors, each component
     * &ge;0 and &le;1
     */
    private Vector2f[] getUV(Vector3f[] vertices) {

        int num = vertices.length;
        float pi = FastMath.PI;
        Vector2f[] UV = new Vector2f[num];

        for(int i=0; i < num; i++) {
            UV[i] = cartToLL(vertices[i]);
            UV[i].x = (UV[i].x + pi) / (2.0f * pi);
            UV[i].y = (UV[i].y + pi / 2.0f) / pi;
        }

        return UV;
    }
}
