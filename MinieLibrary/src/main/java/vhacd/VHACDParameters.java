package vhacd;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

import vhacd.vhacd_native.VHACDNativeParameters;
/*
Copyright (c) 2016, Riccardo Balbo
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
import vhacd.vhacd_native.VhacdLibrary;

public class VHACDParameters extends VHACDNativeParameters implements Cloneable{
	
	private boolean DEBUG;

    public VHACDParameters(){
        super();
        VhacdLibrary.INSTANCE.initParams(this);
    }
	

	public void setDebugEnabled(boolean d) {
		DEBUG=d;
	}

	public boolean getDebugEnabled() {
		return DEBUG;
	}

	/**
	 * 
	 * @description Set maximum number of voxels generated during the voxelization stage
	 * @param v  default = 100000, min = 10000, max = 64000000
	 */
	public void setVoxelResolution(int v) {
		m_resolution=v;
	}

	public int getVoxelResolution() {
		return m_resolution;
	}

	/**
	 * 
	 * @description Set maximum number of clipping stages. During each split stage, all the model parts (with a concavity higher than the user defined threshold) are clipped according the "best" clipping plane
	 * @param  default = 20, min = 1, max = 32
	 */
	public void setClippingDepth(int v) {
		m_depth=v;
	}

	public int getClippingDepth() {
		return m_depth;
	}

	/**
	 * 
	 * @description Set maximum concavity
	 * @param default = 0.0025, min = 0.0, max = 1.0
	 */
	public void setMaxConcavity(double v) {
		m_concavity=v;
	}

	public double getMaxConcavity() {
		return m_concavity;
	}

	/**
	 * 
	 * @description Set granularity of the search for the "best" clipping plane
	 * @param v default = 4, min = 1, max = 16
	 */
	public void setPlaneDownSampling(int v) {
		m_planeDownsampling=v;
	}

	public int getPlaneDownSampling() {
		return m_planeDownsampling;
	}

	/**
	 * 
	 * @description Set precision of the convex-hull generation process during the clipping plane selection stage
	 * @param v  default = 4, min = 1, max = 16
	 */
	public void setConvexHullDownSampling(int v) {
		m_convexhullDownsampling=v;
	}

	public int getConvexHullDownSampling() {
		return m_convexhullDownsampling;
	}

	/**
	 * 
	 * @description Set bias toward clipping along symmetry planes
	 * @param v default = 0.05, min = 0.0, max = 1.0,
	 */
	public void setAlpha(double v) {
		m_alpha=v;
	}

	public double getAlpha() {
		return m_alpha;
	}

	/**
	 * 	
	 * @description Set bias toward clipping along revolution axes
	 * @param v default = 0.05, min = 0.0, max = 1.0
	 */
	public void setBeta(double v) {
		m_beta=v;
	}

	public double getBeta() {
		return m_beta;
	}

	/**
	 * 
	 * @description Set maximum allowed concavity during the merge stage
	 * @param v  default = 0.00125, min = 0.0, max = 1.0
	 */
	public void setGamma(double v) {
		m_gamma=v;
	}

	public double getGamma() {
		return m_gamma;
	}

	/**
	 * 
	 * @description Enable/disable normalizing the mesh before applying the convex decomposition
	 * @param v  default = False
	 */
	public void setPCA(boolean v) {
		m_pca=v?1:0;
	}

	public boolean getPCA() {
		return m_pca==1;
	}

	/**
	 * 
	 * @description Set approximate convex decomposition mode
	 * @param v  default = VOXEL
	 */
	public void setACDMode(ACDMode mode) {
		m_mode=mode.ordinal();
	}

	public ACDMode getACDMode() {
		return ACDMode.values()[m_mode];
	}

	/**
	 * 
	 * @description Set minimum volume to add vertices to convex-hulls
	 * @param v  default = 0.0001, min = 0.0, max = 0.01
	 */
	public void setMinVolumePerHull(double v) {
		m_minVolumePerCH=v;
	}

	public double getMinVolumePerHull() {
		return m_minVolumePerCH;
	}

	/**
	 * 
	 * @description Set maximum number of vertices per convex-hull
	 * @param v  default = 32, min = 4, max = 1024)
	 */
	public void setMaxVerticesPerHull(int v) {
		m_maxNumVerticesPerCH=v;
	}

	public int getMaxVerticesPerHull() {
		return m_maxNumVerticesPerCH;
	}

	@Override 
	public boolean equals(Object op2){
		if(op2 instanceof VHACDParameters){
			VHACDParameters p2=(VHACDParameters)op2;
			return this.m_alpha==p2.m_alpha
			&&
			this.m_beta==p2.m_beta		
			&&
			this.m_concavity==p2.m_concavity
			&&
			this.m_convexhullApproximation==p2.m_convexhullApproximation
			&&
			this.m_convexhullDownsampling==p2.m_convexhullDownsampling
			&&
			this.m_depth==p2.m_depth
			&&
			this.m_gamma==p2.m_gamma
			&&
			this.m_maxNumVerticesPerCH==p2.m_maxNumVerticesPerCH
			&&
			this.m_minVolumePerCH==p2.m_minVolumePerCH
			&&
			this.m_mode==p2.m_mode
			&&
			this.m_oclAcceleration==p2.m_oclAcceleration
			&&
			this.m_pca==p2.m_pca
			&&
			this.m_planeDownsampling==p2.m_planeDownsampling
			&&
			this.m_resolution==p2.m_resolution;
		}
		return false;
	}
	
	public void fromInputStream(InputStream is) throws IOException{
		DataInputStream dis=new DataInputStream(is);
		m_concavity=dis.readDouble();
		m_alpha=dis.readDouble();
		m_beta=dis.readDouble();
		m_gamma=dis.readDouble();
		m_minVolumePerCH=dis.readDouble();
		
		
		m_resolution=dis.readInt();
		m_maxNumVerticesPerCH=dis.readInt();
		m_depth=dis.readInt();
		m_planeDownsampling=dis.readInt();
		m_convexhullDownsampling=dis.readInt();
		m_pca=dis.readInt();
		m_mode=dis.readInt();
		m_convexhullApproximation=dis.readInt();
		m_oclAcceleration=dis.readInt();
	}
	
	public void toOutputStream(OutputStream os) throws IOException{
		DataOutputStream dos=new DataOutputStream(os);
		
		dos.writeDouble(m_concavity);
		dos.writeDouble(m_alpha);
		dos.writeDouble(m_beta);
		dos.writeDouble(m_gamma);
		dos.writeDouble(m_minVolumePerCH);
		
		
		dos.writeInt(m_resolution);
		dos.writeInt(m_maxNumVerticesPerCH);
		dos.writeInt(m_depth);
		dos.writeInt(m_planeDownsampling);
		dos.writeInt(m_convexhullDownsampling);
		dos.writeInt(m_pca);
		dos.writeInt(m_mode);
		dos.writeInt(m_convexhullApproximation);
		dos.writeInt(m_oclAcceleration);


	}
	
	@Override 
	public VHACDParameters clone(){
		try{
			return (VHACDParameters)super.clone();
		}catch(CloneNotSupportedException e){}
		return null;
	}
}
