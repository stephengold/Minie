package vhacd;

import java.util.ArrayList;

import com.sun.jna.Pointer;

import vhacd.vhacd_native.VHACDNativeConvexHull;
import vhacd.vhacd_native.VHACDNativeResults;
/*
Copyright (c) 2016, Riccardo Balbo
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

public class VHACDResults extends ArrayList<VHACDHull>{
	private static final long serialVersionUID=1L;

	protected VHACDResults(VHACDNativeResults nat){
		int nhulls=nat.n_hulls;
		VHACDNativeConvexHull[] hulls=(VHACDNativeConvexHull[])nat.hulls.toArray(nhulls);
	
		for(int i=0;i<nhulls;i++){
			VHACDNativeConvexHull nhull=hulls[i];
			
		    Pointer p_positions=nhull.positions.getPointer();            
			float positions[]=p_positions.getFloatArray(0,nhull.n_positions*3);			
            
		    Pointer p_indexes=nhull.indexes.getPointer();            
			int indexes[]=p_indexes.getIntArray(0,nhull.n_indexes*3);
			
			VHACDHull hull=new VHACDHull(positions,indexes);
			add(hull);
		}
	}
}
