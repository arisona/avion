/*
 * Copyright (c) 2013 - 2016 Stefan Muller Arisona, Simon Schubiger
 * Copyright (c) 2013 - 2016 FHNW & ETH Zurich
 * All rights reserved.
 *
 * Contributions by: Filip Schramka, Samuel von Stachelski
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *  Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *  Neither the name of FHNW / ETH Zurich nor the names of its contributors may
 *   be used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package ch.fhnw.ether.avion.example;

import java.net.URL;
import java.nio.ByteBuffer;

import org.lwjgl.BufferUtils;

import ch.fhnw.ether.avion.AVDecoder;
import ch.fhnw.ether.avion.AVDecoder.AudioEncoding;
import ch.fhnw.ether.avion.AVDecoder.AudioFormat;
import ch.fhnw.ether.avion.AVDecoder.VideoFormat;
import ch.fhnw.ether.avion.AVDecoder.VideoPixelFormat;
import ch.fhnw.ether.avion.Avion;

public final class AvionDecoderTest {
	public static void main(String[] args) {
		try {
			System.out.println("avion test flight: " + Avion.isReady());
	
	        int audioSize = 1024;
	        
	        AudioFormat audioFormat = new AudioFormat(AudioEncoding.PCM_32_FLOAT, 44100, audioSize, true);
	        VideoFormat videoFormat = new VideoFormat(VideoPixelFormat.RGBA, true);
	        AVDecoder decoder = Avion.createDecoder(new URL("file:///Users/radar/Desktop/simian_mobile_disco-audacity_of_huge_(2009).mp4"), audioFormat, videoFormat);
	        
	        //wrapper->seek(60);
	        
	        int size = decoder.getVideoWidth() * decoder.getVideoHeight() * 4;
	        ByteBuffer image = BufferUtils.createByteBuffer(size);
	        
	        ByteBuffer samples = BufferUtils.createByteBuffer(audioSize);
	        
	        int error = 0;
	        double[] pts = new double[1];
	        while (error != -1) {
	            error = decoder.decodeVideo(image, pts);
	            System.out.println("got video frame " + pts[0] + " " + error);
	
	            error = decoder.decodeAudio(samples, pts);
	            System.out.println("got audio frame " + pts[0] + " " + error);
	        }
	        
	        decoder.dispose();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
}
