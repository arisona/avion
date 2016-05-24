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

package ch.fhnw.ether.avion;

import java.net.URL;
import java.nio.ByteBuffer;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

public final class AVDecoder {
	
	public static final int NO_ERROR = 0;
	public static final int END_OF_STREAM = -1;
	public static final int NO_SUCH_STREAM = -2;
	public static final int INTERNAL_ERROR = -3;
	public static final int DECODER_DISPOSED = -4;
	
	public enum AudioEncoding {
		PCM_16_SIGNED,
		PCM_32_FLOAT,
	}
	
	public static final class AudioFormat {
		public final AudioEncoding encoding;
		public final double sampleRate;
		public final int bufferSize;
		public final boolean interleaved;
		
		public AudioFormat(AudioEncoding encoding, double sampleRate, int bufferSize, boolean interleaved) {
			this.encoding = encoding;
			this.sampleRate = sampleRate;
			this.bufferSize = bufferSize;
			this.interleaved = interleaved;
		}
	}
	
	public enum VideoPixelFormat {
		RGBA,
		BGRA
	}
	
	public static final class VideoFormat {
		public final VideoPixelFormat pixelFormat;
		public final boolean flip;
		
		public VideoFormat(VideoPixelFormat pixelFormat, boolean flip) {
			this.pixelFormat = pixelFormat;
			this.flip = flip;
		}
	}
	
	
	private volatile long nativeHandle;
	private final ReadWriteLock lock = new ReentrantReadWriteLock();

	private final URL url;
	private final boolean hasAudio;
	private final boolean hasVideo;
	private final double duration;
	private final double videoFrameRate;
	private final int videoWidth;
	private final int videoHeight;
	
	AVDecoder(URL url, AudioFormat audioFormat, VideoFormat videoFormat) {
		this.url = url;

		nativeHandle = Avion.decoderCreate(url.toString(), 
				audioFormat != null,
				audioFormat != null ? audioFormat.encoding.ordinal() : 0,
				audioFormat != null ? audioFormat.sampleRate : 0,
				audioFormat != null ? audioFormat.bufferSize : 0, 
				audioFormat != null ? audioFormat.interleaved : false,
				videoFormat != null,
				videoFormat != null ? videoFormat.pixelFormat.ordinal() : 0,
				videoFormat != null ? videoFormat.flip : false);	
						
		if (nativeHandle == 0)
			throw new IllegalArgumentException("cannot create av decoder from " + url);

		hasAudio = Avion.decoderHasAudio(nativeHandle);
		hasVideo = Avion.decoderHasVideo(nativeHandle);
		duration = Avion.decoderGetDuration(nativeHandle);
		videoFrameRate = Avion.decoderGetVideoFrameRate(nativeHandle);
		videoWidth = Avion.decoderGetVideoWidth(nativeHandle);
		videoHeight = Avion.decoderGetVideoHeight(nativeHandle);
	}

	public void dispose() {
		lock.writeLock().lock();
		try {
			Avion.decoderDispose(nativeHandle);
		} finally {
			nativeHandle = 0;
			lock.writeLock().unlock();
		}
	}

	public void setRange(double start, double end) {
		lock.writeLock().lock();
		try {
			if (nativeHandle != 0)
				Avion.decoderSetRange(nativeHandle, start, end);
		} finally {
			lock.writeLock().unlock();
		}
	}
	
	public boolean hasAudio() {
		return hasAudio;
	}
	
	public boolean hasVideo() {
		return hasVideo;
	}

	public URL getURL() {
		return url;
	}

	public double getDuration() {
		return duration;
	}

	public double getVideoFrameRate() {
		return videoFrameRate;
	}

	public int getVideoWidth() {
		return videoWidth;
	}

	public int getVideoHeight() {
		return videoHeight;
	}

	public int decodeAudio(ByteBuffer buffer, double[] pts) {
		lock.readLock().lock();
		try {
			if (nativeHandle == 0)
				return DECODER_DISPOSED;
			return Avion.decoderDecodeAudio(nativeHandle, buffer, pts);
		} finally {
			lock.readLock().unlock();
		}
	}

	public int decodeVideo(ByteBuffer buffer, double[] pts) {
		lock.readLock().lock();
		try {
			if (nativeHandle == 0)
				return DECODER_DISPOSED;
			return Avion.decoderDecodeVideo(nativeHandle, buffer, pts);
		} finally {
			lock.readLock().unlock();
		}
	}

	@Override
	public String toString() {
		return getURL() + " (d=" + getDuration() + " fr=" + getVideoFrameRate() + " w=" + getVideoWidth() + " h=" + getVideoHeight() + ")";
	}
}
