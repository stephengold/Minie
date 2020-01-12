package vhacd;

import vhacd.vhacd_native.VhacdLibrary;
import vhacd.vhacd_native.VHACDNativeParameters;
import vhacd.vhacd_native.VHACDNativeResults;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
/*
Copyright (c) 2016, Riccardo Balbo
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


public class VHACD{
	public static VHACDResults compute(float positions[], int indexes[], VHACDParameters params) {
		int pos_buffer_l=positions.length;
		int index_buffer_l=indexes.length;

        FloatBuffer b_pos=FloatBuffer.wrap(positions);
        IntBuffer b_ind=IntBuffer.wrap(indexes);
        b_pos.rewind();
        b_ind.rewind();
        
		VHACDNativeResults r=VhacdLibrary.INSTANCE.compute(
            b_pos,
            pos_buffer_l,
            b_ind,
            index_buffer_l,
            params,
            params.getDebugEnabled()?(byte)1:(byte)0);  
             
		VHACDResults result=new VHACDResults(r);       
        VhacdLibrary.INSTANCE.release(r);
        
		return result;
	}
}
