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

#import <Foundation/Foundation.h>
#import <AVFoundation/AVFoundation.h>

#import "avion.hpp"

#include "audio_buffer.hpp"
/*
static uint32_t BGRA2RGBA(uint32_t value) {
    return (value & 0xff000000) >> 16 |
           (value & 0x00ff0000) |
           (value & 0x0000ff00) << 16 |
           (value & 0x000000ff);
}
*/

static uint32_t BGRA2RGBA(uint32_t value) {
    return
    (value & 0x00ff0000) >> 16 |
    (value & 0x0000ff00) |
    (value & 0x000000ff) << 16 |
    (value & 0xff000000);
//    (value & 0xff000000) >> 16 |
//    (value & 0x00ff0000) |
//    (value & 0x0000ff00) << 16 |
//    (value & 0x000000ff);
}

// TODO:
// - flip image vertically, optimise pixel transfer
// - audio buffer size & sample rate request support
// - complete audio
// - API: replace seek with range (start + end time)?

class AVAssetDecoder : public AvionDecoder {
private:
    const std::string url;

    AudioFormat audioFormat;
    VideoFormat videoFormat;

    AVAsset* asset = nullptr;
    AVAssetTrack* audioTrack = nullptr;
    AVAssetTrack* videoTrack = nullptr;
    
    double videoFrameRate = 0;
    CGSize videoSize = { 0, 0 };
    
    double duration = 0;

    AVAssetReader* audioReader = nullptr;
    AVAssetReader* videoReader = nullptr;

    AudioQueue<uint8_t> audioQueue;
    
public:
    AVAssetDecoder(std::string url, AudioFormat audioFormat, VideoFormat videoFormat) : url(url), audioFormat(audioFormat), videoFormat(videoFormat), audioQueue(audioFormat.sampleRate) {
        
        NSURL* nsUrl = [NSURL URLWithString:[NSString stringWithCString:url.c_str() encoding:NSUTF8StringEncoding]];
        if (!nsUrl) {
            MSG("avf: invalid url '%s'\n", url.c_str());
            throw std::invalid_argument("invalid url " + url);
        }
        
        NSDictionary* options = @{ AVURLAssetPreferPreciseDurationAndTimingKey : @YES };
        
        //---- asset
        asset = [AVURLAsset URLAssetWithURL:nsUrl options:options];
        if (!asset) {
            MSG("avf: invalid url '%s'\n", url.c_str());
            throw std::invalid_argument("invalid url " + url);
        }
        
        //--- audio track
        if (audioFormat.decode) {
            NSArray* audioTracks = [asset tracksWithMediaType:AVMediaTypeAudio];
            if ([audioTracks count] > 0) {
                audioTrack = [audioTracks objectAtIndex:0];
            } else {
                MSG("avf: no audio track for '%s'\n", url.c_str());
            }
        }
        
        //--- video track
        if (videoFormat.decode) {
            NSArray* videoTracks = [asset tracksWithMediaType:AVMediaTypeVideo];
            if ([videoTracks count] > 0) {
                videoTrack = [videoTracks objectAtIndex:0];
                videoFrameRate = [videoTrack nominalFrameRate];
                videoSize = [videoTrack naturalSize];
            } else {
                MSG("avf: no video track for '%s'\n", url.c_str());
            }
        }
        
        duration = CMTimeGetSeconds([asset duration]);
        
        setRange(0.0);
        
        //MSG("avf: %s: duration=%f framerate=%f size=%dx%d\n", url.c_str(), duration, videoFrameRate, (int)videoSize.width, (int)videoSize.height);
    }
    
    virtual ~AVAssetDecoder() {
        if (audioReader)
            [audioReader release];
        if (videoReader)
            [videoReader release];
    }
    
    void setRange(double start, double end = std::numeric_limits<double>::infinity()) {
        NSError* error = nil;
        CMTimeRange timeRange = CMTimeRangeMake(CMTimeMakeWithSeconds(start, 1), end == std::numeric_limits<double>::infinity() ? kCMTimePositiveInfinity : CMTimeMakeWithSeconds(end, 1));
        
        //---- setup audio reader
        if (audioTrack) {
            audioQueue.clear();
            
            if (audioReader != nullptr)
                [audioReader release];
            
            audioReader = [[AVAssetReader alloc] initWithAsset:asset error:&error];
            if (!audioReader || error) {
                MSG("avf: could not initialize audio reader for '%s'\n", url.c_str());
                throw std::invalid_argument("could not initialize audio reader for " + url);
            }
            
            NSDictionary* audioSettings = @{
                                            AVFormatIDKey : [NSNumber numberWithUnsignedInt:kAudioFormatLinearPCM],
                                            AVSampleRateKey : [NSNumber numberWithFloat:audioFormat.sampleRate],
                                            AVNumberOfChannelsKey : [NSNumber numberWithInt:2],
                                            AVLinearPCMBitDepthKey : [NSNumber numberWithInt:(audioFormat.encoding == PCM_32_FLOAT ? 32 : 16)],
                                            AVLinearPCMIsNonInterleaved : [NSNumber numberWithBool: (audioFormat.interleaved ? NO : YES)],
                                            AVLinearPCMIsFloatKey : [NSNumber numberWithBool:(audioFormat.encoding == PCM_32_FLOAT ? YES : NO)],
                                            AVLinearPCMIsBigEndianKey : [NSNumber numberWithBool:NO],
                                            };
            [audioReader addOutput:[AVAssetReaderAudioMixOutput assetReaderAudioMixOutputWithAudioTracks:@[audioTrack] audioSettings:audioSettings]];
            
            audioReader.timeRange = timeRange;
            
            if ([audioReader startReading] != YES) {
                [audioReader release];
                audioReader = nullptr;
                MSG("avf: could not start reading audio from '%s': %s\n", url.c_str(), [[[audioReader error] localizedDescription] UTF8String]);
                throw std::invalid_argument("could not start reading audio for " + url);
            }
        }
        
        //---- setup video reader
        if (videoTrack) {
            if (videoReader != nullptr)
                [videoReader release];
            
            videoReader = [[AVAssetReader alloc] initWithAsset:asset error:&error];
            if (!videoReader || error) {
                MSG("avf: could not initialize video reader for '%s'\n", url.c_str());
                throw std::invalid_argument("could not initialize video reader for " + url);
            }
            
            NSDictionary* videoSettings = @{
                                            (id)kCVPixelBufferPixelFormatTypeKey: [NSNumber numberWithUnsignedInt:kCVPixelFormatType_32BGRA]
                                            };
            [videoReader addOutput:[AVAssetReaderTrackOutput assetReaderTrackOutputWithTrack:videoTrack outputSettings:videoSettings]];
            
            videoReader.timeRange = timeRange;
            
            if ([videoReader startReading] != YES) {
                [videoReader release];
                videoReader = nullptr;
                MSG("avf: could not start reading video from '%s': %s\n", url.c_str(), [[[videoReader error] localizedDescription] UTF8String]);
                throw std::invalid_argument("could not start reading video for " + url);
            }
        }
    }
    
    bool hasAudio() {
        return audioTrack;
    }
    
    bool hasVideo() {
        return videoTrack;
    }
    
    double getDuration() {
        return duration;
    }
    
    double getVideoFrameRate() {
        return videoFrameRate;
    }
    
    int getVideoWidth() {
        return videoSize.width;
    }
    
    int getVideoHeight() {
        return videoSize.height;
    }

    int decodeAudio(uint8_t* buffer, double& pts) {
        if (!audioReader)
            return NO_SUCH_STREAM;
        
        if ([audioReader status] != AVAssetReaderStatusReading) {
            MSG("avf: get next audio frame: reached end of media\n");
            return END_OF_STREAM;
        }
        
        AVAssetReaderOutput* output = [audioReader.outputs objectAtIndex:0];
        while (audioQueue.size() < audioFormat.bufferSize) {
            CMSampleBufferRef sampleBuffer = [output copyNextSampleBuffer];
            if (!sampleBuffer) {
                MSG("avf: get next audio frame: could not copy audio sample buffer\n");
                break;
            }
            
            double srcPts = CMTimeGetSeconds(CMSampleBufferGetPresentationTimeStamp(sampleBuffer));
            
            CMBlockBufferRef blockBuffer = CMSampleBufferGetDataBuffer(sampleBuffer);
            if (!blockBuffer) {
                MSG("avf: get next audio frame: could not get audio block buffer\n");
                CFRelease(sampleBuffer);
                return INTERNAL_ERROR;
            }
            
            size_t srcLength = 0;
            uint8_t* srcSamples = nullptr;
            if (CMBlockBufferGetDataPointer(blockBuffer, 0, nullptr, &srcLength, (char**)&srcSamples) != kCMBlockBufferNoErr) {
                MSG("avf: get next audio frame: cannot get audio data\n");
                CFRelease(sampleBuffer);
                return INTERNAL_ERROR;
            }
            
            //MSG("avf: put audio samples: %ld %f %f %f %f \n", srcLength, srcSamples[0], srcSamples[1], srcSamples[2], srcSamples[3]);
            audioQueue.put(srcSamples, srcLength, srcPts);
            
            CFRelease(sampleBuffer);
        }
        
        if (!audioQueue.size())
            return END_OF_STREAM;
        
        int received = audioQueue.take(buffer, audioFormat.bufferSize, pts);
        
        //MSG("avf: get audio samples: %d %f %f %f %f \n", received, buffer[0], buffer[1], buffer[2], buffer[3]);
        
        return received;
    }

    int decodeVideo(uint8_t* buffer, double& pts) {
        if (!videoReader)
            return NO_SUCH_STREAM;
        
        if ([videoReader status] != AVAssetReaderStatusReading) {
            MSG("avf: get next video frame: reached end of media\n");
            return END_OF_STREAM;
        }
        
        AVAssetReaderOutput* output = [videoReader.outputs objectAtIndex:0];
        CMSampleBufferRef sampleBuffer = [output copyNextSampleBuffer];
        if (!sampleBuffer) {
            MSG("avf: get next video frame: could not copy video sample buffer\n");
            return END_OF_STREAM;
        }
        
        pts = CMTimeGetSeconds(CMSampleBufferGetPresentationTimeStamp(sampleBuffer));
        
        CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
        
        // lock the image buffer
        CVPixelBufferLockBaseAddress(imageBuffer, 0);
        
        // note: if movie width cannot be divided by 4 it seems the movie is scaled up to the next width that can
        // i.e. if you open a move with 1278 pixels width, here, the imageBuffer will have a width of 1280.
        // for now, we just skip the remaining pixel columns...
        int width = getVideoWidth();
        int height = getVideoHeight();
        int pixelsPerRow = (int)CVPixelBufferGetBytesPerRow(imageBuffer) / 4;

        //MSG("avf: w=%d h=%d bpr=%d length=%d\n", width, height, pixelsPerRow, width * height * 4);
        
        uint32_t* dst = (uint32_t*)buffer;
        uint32_t* src = (uint32_t*)CVPixelBufferGetBaseAddress(imageBuffer);
        for (int y = height; --y >= 0;) {
            uint32_t* row = src + y * pixelsPerRow;
            for (int x = 0; x < width; ++x) {
                *dst++ = ::BGRA2RGBA(*row++);
            }
        }
        
        // unlock the image buffer & cleanup
        CVPixelBufferUnlockBaseAddress(imageBuffer, 0);
        CFRelease(sampleBuffer);
        
        return NO_ERROR;
    }
};

AvionDecoder* AvionDecoder::create(std::string url, AudioFormat audioFormat, VideoFormat videoFormat) {
    return new AVAssetDecoder(url, audioFormat, videoFormat);
}
