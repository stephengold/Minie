/*

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the {organization} nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
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
 * Approximates a sphere as a simplicial polyhedron, formed by subdividing the triangles of a regular icosahedron into
 * smaller triangles. Normally used to achieve a more isotropical layout of vertices than a UV sphere.
 *
 * @author jayfella
 */

// to see inside, change indices from 1-2-3 to 1-3-2.
public class Icosphere extends Mesh {

    private static final float t = (1.0f + FastMath.sqrt(5.0f)) / 2.0f;

    private static final Vector3f[] ico_vertices = {
            new Vector3f(-1,  t,  0), new Vector3f( 1,  t,  0), new Vector3f(-1, -t,  0), new Vector3f( 1, -t,  0),
            new Vector3f( 0, -1,  t), new Vector3f( 0,  1,  t), new Vector3f( 0, -1, -t), new Vector3f( 0,  1, -t),
            new Vector3f( t,  0, -1), new Vector3f( t,  0,  1), new Vector3f(-t,  0, -1), new Vector3f(-t,  0,  1)
    };

    private static final int[] ico_indices = {
            0, 11, 5,   0, 5, 1,    0, 1, 7,    0, 7, 10,   0, 10, 11,
            1, 5, 9,    5, 11, 4,   11, 10, 2,  10, 7, 6,   7, 1, 8,
            3, 9, 4,    3, 4, 2,    3, 2, 6,    3, 6, 8,    3, 8, 9,
            4, 9, 5,    2, 4, 11,   6, 2, 10,   8, 6, 7,    9, 8, 1
    };

    private final float radius;
    private int index;
    private Map<Long, Integer> middlePointIndexCache = new HashMap<>();
    private List<Vector3f> vertices = new ArrayList<>();
    private List<Vector3f> normals = new ArrayList<>();

    public Icosphere(int recursionLevel, float radius) {

        this.radius = radius;
        this.index = 0;

        // create 12 vertices of a icosahedron
        for (int i = 0; i < ico_vertices.length; i++) {
            addVertex(ico_vertices[i]);
        }

        List<Integer> faces = new ArrayList<>();
        for (int i = 0; i < ico_indices.length; i++) {
            faces.add(ico_indices[i]);
        }

        // refine triangles
        for (int i = 0; i < recursionLevel; i++)
        {
            List<Integer> faces2 = new ArrayList<>();

            for (int j = 0; j < faces.size(); j += 3)
            {
                // replace triangle by 4 triangles
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

    // add vertex to mesh, fix position to be on unit sphere, return index
    private int addVertex(Vector3f p) {
        float length = FastMath.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        vertices.add(p.divide(length).mult(radius));
        normals.add(p.divide(length));
        return index++;
    }

    // return index of point in the middle of p1 and p2
    private int getMiddlePoint(int p1, int p2)
    {
        // first check if we have it already
        boolean firstIsSmaller = p1 < p2;
        long smallerIndex = firstIsSmaller ? p1 : p2;
        long greaterIndex = firstIsSmaller ? p2 : p1;
        long key = (smallerIndex << 32) + greaterIndex;

        Integer ret = this.middlePointIndexCache.get(key);

        if (ret != null) {
            return ret;
        }

        // not in cache, calculate it
        Vector3f point1 = this.vertices.get(p1);
        Vector3f point2 = this.vertices.get(p2);

        Vector3f middle = point1.add(point2).divideLocal(2.0f);

        // add vertex makes sure point is on unit sphere
        int i = addVertex(middle);

        // store it, return index
        this.middlePointIndexCache.put(key, i);
        return i;
    }

    // transform 3D cartesion coordinates to longitude, latitude
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

    // standard Longitude/Latitude mapping to (0,1)/(0,1)
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