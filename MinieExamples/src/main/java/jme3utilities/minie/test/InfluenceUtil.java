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
package jme3utilities.minie.test;

import com.jme3.animation.Bone;
import com.jme3.animation.Skeleton;
import com.jme3.animation.SkeletonControl;
import com.jme3.math.ColorRGBA;
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
import jme3utilities.debug.SkeletonVisualizer;

/**
 * Utility methods to determine which bones can influence mesh vertices. All
 * methods should be static. TODO use Heart library
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class InfluenceUtil {
    // *************************************************************************
    // constants and loggers

    /**
     * maximum number of bones that can influence any one vertex
     */
    final private static int maxWeights = 4;
    /**
     * message logger for this class
     */
    final private static Logger logger
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
     * De-visualize any bones that don't influence mesh vertices.
     *
     * @param visualizer the visualizer (not null, modified)
     * @param skeletonControl the control to analyze (not null, unaffected)
     */
    static void hideNonInfluencers(SkeletonVisualizer visualizer,
            SkeletonControl skeletonControl) {
        Spatial subtree = skeletonControl.getSpatial();
        Skeleton skeleton = ((SkeletonControl) skeletonControl).getSkeleton();
        BitSet influencers = addAllInfluencers(subtree, skeleton);

        int numBones = skeleton.getBoneCount();
        for (int boneIndex = 0; boneIndex < numBones; boneIndex++) {
            if (influencers.get(boneIndex) == false) {
                visualizer.setHeadColor(boneIndex, ColorRGBA.BlackNoAlpha);
            }
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Generate the set of indices for bones that influence (directly or
     * indirectly) vertices in the specified subtree of the scene graph. Note:
     * recursive!
     *
     * @param subtree the subtree to traverse (may be null, unaffected)
     * @param skeleton the skeleton containing the bones (not null, unaffected)
     * @return a new set of bone indices
     */
    private static BitSet addAllInfluencers(Spatial subtree,
            Skeleton skeleton) {
        int numBones = skeleton.getBoneCount();
        BitSet result = new BitSet(numBones);

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
     * @param addResult (not null, modified)
     */
    private static void addDirectInfluencers(Mesh mesh, BitSet addResult) {
        int maxWeightsPerVert = mesh.getMaxNumWeights();
        if (maxWeightsPerVert <= 0) {
            maxWeightsPerVert = 1;
        }
        assert maxWeightsPerVert > 0 : maxWeightsPerVert;
        assert maxWeightsPerVert <= maxWeights : maxWeightsPerVert;

        VertexBuffer biBuf = mesh.getBuffer(VertexBuffer.Type.BoneIndex);
        Buffer boneIndexBuffer = biBuf.getDataReadOnly();
        boneIndexBuffer.rewind();
        int numBoneIndices = boneIndexBuffer.remaining();
        assert numBoneIndices % maxWeights == 0 : numBoneIndices;
        int numVertices = boneIndexBuffer.remaining() / maxWeights;

        VertexBuffer wBuf = mesh.getBuffer(VertexBuffer.Type.BoneWeight);
        FloatBuffer weightBuffer = (FloatBuffer) wBuf.getDataReadOnly();
        weightBuffer.rewind();
        int numWeights = weightBuffer.remaining();
        assert numWeights == numVertices * maxWeights : numWeights;

        for (int vIndex = 0; vIndex < numVertices; vIndex++) {
            for (int wIndex = 0; wIndex < maxWeights; wIndex++) {
                float weight = weightBuffer.get();
                int boneIndex = MyMesh.readIndex(boneIndexBuffer);
                if (wIndex < maxWeightsPerVert && weight != 0f) {
                    addResult.set(boneIndex);
                }
            }
        }
    }

    /**
     * Add indices to the result for bones that directly influence vertices in
     * the specified subtree of the scene graph. Note: recursive!
     *
     * @param subtree subtree to traverse (may be null, unaffected)
     * @param addResult (not null, modified)
     */
    private static void addDirectInfluencers(Spatial subtree,
            BitSet addResult) {
        if (subtree instanceof Geometry) {
            Geometry geometry = (Geometry) subtree;
            Mesh mesh = geometry.getMesh();
            if (MyMesh.isAnimated(mesh)) {
                addDirectInfluencers(mesh, addResult);
            }

        } else if (subtree instanceof Node) {
            Node node = (Node) subtree;
            List<Spatial> children = node.getChildren();
            for (Spatial child : children) {
                addDirectInfluencers(child, addResult);
            }
        }
    }
}
