/*
 Copyright (c) 2017-2019, Stephen Gold
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
package jme3utilities.minie.wizard;

import com.jme3.animation.Bone;
import com.jme3.animation.Skeleton;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer;
import java.nio.Buffer;
import java.nio.FloatBuffer;
import java.util.BitSet;
import java.util.List;
import java.util.logging.Logger;
import jme3utilities.MyMesh;

/**
 * Utility methods to determine which bones can influence mesh vertices. All
 * methods should be static.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class InfluenceUtil {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final static Logger logger
            = Logger.getLogger(InfluenceUtil.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private InfluenceUtil() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Add indices to the result for bones that influence (directly or
     * indirectly) vertices in the specified subtree of the scene graph. Note:
     * recursive!
     *
     * @param subtree subtree to traverse (may be null, unaffected)
     * @param skeleton skeleton (not null, unaffected)
     * @param storeResult (modified if not null)
     * @return the set of bones with influence (either storeResult or a new
     * instance, not null)
     */
    static BitSet addAllInfluencers(Spatial subtree, Skeleton skeleton,
            BitSet storeResult) {
        int numBones = skeleton.getBoneCount();
        BitSet result
                = (storeResult == null) ? new BitSet(numBones) : storeResult;

        addDirectInfluencers(subtree, result);

        for (int boneIndex = 0; boneIndex < numBones; boneIndex++) {
            if (result.get(boneIndex)) {
                Bone bone = skeleton.getBone(boneIndex);
                for (Bone parent = bone.getParent();
                        parent != null;
                        parent = parent.getParent()) {
                    int parentIndex = skeleton.getBoneIndex(parent);
                    result.set(parentIndex);
                }
            }
        }

        return result;
    }

    /**
     * Add indices to the result for bones that directly influence vertices in
     * the specified mesh.
     *
     * @param mesh animated mesh to analyze (not null, unaffected)
     * @param storeResult (modified if not null)
     * @return the set of bones with influence (either storeResult or a new
     * instance)
     */
    static BitSet addDirectInfluencers(Mesh mesh, BitSet storeResult) {
        BitSet result = storeResult == null ? new BitSet(120) : storeResult;

        int maxWeightsPerVert = mesh.getMaxNumWeights();
        if (maxWeightsPerVert <= 0) {
            maxWeightsPerVert = 1;
        }
        assert maxWeightsPerVert > 0 : maxWeightsPerVert;
        assert maxWeightsPerVert <= 4 : maxWeightsPerVert;

        VertexBuffer biBuf = mesh.getBuffer(VertexBuffer.Type.BoneIndex);
        Buffer boneIndexBuffer = biBuf.getDataReadOnly();
        boneIndexBuffer.rewind();
        int numBoneIndices = boneIndexBuffer.remaining();
        assert numBoneIndices % 4 == 0 : numBoneIndices;
        int numVertices = boneIndexBuffer.remaining() / 4;

        VertexBuffer wBuf = mesh.getBuffer(VertexBuffer.Type.BoneWeight);
        FloatBuffer weightBuffer = (FloatBuffer) wBuf.getDataReadOnly();
        weightBuffer.rewind();
        int numWeights = weightBuffer.remaining();
        assert numWeights == numVertices * 4 : numWeights;

        for (int vIndex = 0; vIndex < numVertices; vIndex++) {
            for (int wIndex = 0; wIndex < 4; wIndex++) {
                float weight = weightBuffer.get();
                int boneIndex = MyMesh.readIndex(boneIndexBuffer);
                if (wIndex < maxWeightsPerVert && weight != 0f) {
                    result.set(boneIndex);
                }
            }
        }

        return result;
    }

    /**
     * Add indices to the result for bones that directly influence vertices in
     * the specified subtree of the scene graph. Note: recursive!
     *
     * @param subtree subtree to traverse (may be null, unaffected)
     * @param storeResult (modified if not null)
     * @return the set of bones with influence (either storeResult or a new
     * instance)
     */
    static BitSet addDirectInfluencers(Spatial subtree,
            BitSet storeResult) {
        BitSet result = (storeResult == null) ? new BitSet(120) : storeResult;

        if (subtree instanceof Geometry) {
            Geometry geometry = (Geometry) subtree;
            Mesh mesh = geometry.getMesh();
            if (MyMesh.isAnimated(mesh)) {
                addDirectInfluencers(mesh, result);
            }

        } else if (subtree instanceof Node) {
            Node node = (Node) subtree;
            List<Spatial> children = node.getChildren();
            for (Spatial child : children) {
                addDirectInfluencers(child, result);
            }
        }

        return result;
    }
}
