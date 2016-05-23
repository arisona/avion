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

#pragma once

#define MSG(...) { printf(__VA_ARGS__); fflush(stdout); }

#include <limits>
#include <string>

class Avion {
public:
    const int NO_ERROR = 0;
    const int END_OF_STREAM = -1;
    const int NO_SUCH_STREAM = -2;
    const int INTERNAL_ERROR = -3;
};

class AvionDecoder : public Avion {
public:
    enum AudioEncoding {
        PCM_16_SIGNED,
        PCM_32_FLOAT,
    };
    
    class AudioFormat {
    public:
        AudioFormat(bool decode, int encoding, double sampleRate, int bufferSize, bool interleaved) : decode(decode), encoding(static_cast<AudioEncoding>(encoding)), sampleRate(sampleRate), bufferSize(bufferSize), interleaved(interleaved) {}
        
        const bool decode;
        const AudioEncoding encoding;
        const double sampleRate;
        const int bufferSize;
        const bool interleaved;
    };
    
    enum VideoPixelFormat {
        RGBA,
        BGRA
    };
    
    class VideoFormat {
    public:
        VideoFormat(bool decode, int pixelFormat, bool flip) : decode(decode), pixelFormat(static_cast<VideoPixelFormat>(pixelFormat)), flip(flip) {}
        
        const bool decode;
        const VideoPixelFormat pixelFormat;
        const bool flip;
    };
    
    static AvionDecoder* create(std::string url, AudioFormat audioFormat, VideoFormat videoFormat);
    
    virtual ~AvionDecoder() {}
    
    virtual void setRange(double start, double end = std::numeric_limits<double>::infinity()) = 0;
    
    virtual bool hasAudio() = 0;
    
    virtual bool hasVideo() = 0;
    
    virtual double getDuration() = 0;
    
    virtual double getVideoFrameRate() = 0;
    
    virtual int getVideoWidth() = 0;
    
    virtual int getVideoHeight() = 0;
    
    virtual int decodeAudio(uint8_t* buffer, double& pts) = 0;

    virtual int decodeVideo(uint8_t* buffer, double& pts) = 0;
};
