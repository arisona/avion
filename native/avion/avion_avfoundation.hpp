/*
 * Copyright (c) 2013 - 2016 Stefan Muller Arisona, Simon Schubiger
 * Copyright (c) 2013 - 2016 FHNW
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
 *  Neither the name of FHNW nor the names of its contributors may be used to
 *   endorse or promote products derived from this software without specific
 *   prior written permission.
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

#import <Foundation/Foundation.h>
#import <AVFoundation/AVFoundation.h>

#include <mutex>

#include "avion.hpp"
#include "util.hpp"

static uint32_t BGRA2RGBA(uint32_t value) {
    return
    (value & 0x00ff0000) >> 16 |
    (value & 0x0000ff00) |
    (value & 0x000000ff) << 16 |
    (value & 0xff000000);
}

namespace {

class AutoReleasePool {
    NSAutoreleasePool* pool_;
public:
    AutoReleasePool() {
        pool_ = [[NSAutoreleasePool alloc] init];
    }
    
    ~AutoReleasePool() {
        [pool_ release];
    }
};
    
}


// TODO:
// - consistent error handling (exceptions vs returned errors)
// - flip / no flip image vertically, optimise pixel transfer
// - RGBA / BGRA support

namespace Avion {


class AVDecoderBase : public AvionDecoder {
protected:
    AVDecoderBase(std::string url, AudioFormat audioFormat, VideoFormat videoFormat)
    : AvionDecoder(url, audioFormat, videoFormat) {}
    
    ~AVDecoderBase() {}

    int decodeAudioSampleBuffer(CMSampleBufferRef sampleBuffer) {
        return NO_ERROR;
    }
    
    int decodeVideoSampleBuffer(CMSampleBufferRef sampleBuffer, uint8_t* buffer, double& pts) {
        int w = getVideoWidth();
        int h = getVideoHeight();
        uint32_t* dst = (uint32_t*)buffer;
        pts = CMTimeGetSeconds(CMSampleBufferGetPresentationTimeStamp(sampleBuffer));
        
        CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
        if (!imageBuffer) {
            MSG("avf: cannot get image buffer from sample buffer\n");
            return INTERNAL_ERROR;
        }
        
        // lock the image buffer
        CVPixelBufferLockBaseAddress(imageBuffer, 0);
        
        // note: if movie width cannot be divided by 4 it seems the movie is scaled up to the next width that can
        // i.e. if you open a move with 1278 pixels width, here, the imageBuffer will have a width of 1280.
        // for now, we just skip the remaining pixel columns...
        int pixelsPerRow = (int)CVPixelBufferGetBytesPerRow(imageBuffer) / 4;
        
        //MSG("avf: w=%d h=%d bpr=%d length=%d\n", w, h, pixelsPerRow, w * h * 4);
        
        uint32_t* src = (uint32_t*)CVPixelBufferGetBaseAddress(imageBuffer);
        for (int y = h; --y >= 0;) {
            uint32_t* row = src + y * pixelsPerRow;
            for (int x = 0; x < w; ++x) {
                *dst++ = ::BGRA2RGBA(*row++);
            }
        }
        
        // unlock the image buffer
        CVPixelBufferUnlockBaseAddress(imageBuffer, 0);
        
        return NO_ERROR;
    }
};



class AVAssetDecoder : public AVDecoderBase {
private:
    AVAsset* asset = nullptr;
    AVAssetTrack* audioTrack = nullptr;
    AVAssetTrack* videoTrack = nullptr;
    
    double videoFrameRate = 0;
    CGSize videoSize = { 0, 0 };
    
    double duration = 0;

    AVAssetReader* audioReader = nullptr;
    AVAssetReader* videoReader = nullptr;

    AudioBuffer<uint8_t> audioQueue;
    
public:
    AVAssetDecoder(std::string url, AudioFormat audioFormat, VideoFormat videoFormat)
    : AVDecoderBase(url, audioFormat, videoFormat),
      audioQueue(audioFormat.sampleRate) {

        AutoReleasePool pool;
        
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
    
    ~AVAssetDecoder() {

        AutoReleasePool pool;

        if (audioReader)
            [audioReader release];
        if (videoReader)
            [videoReader release];
    }
    
    int setRange(double start, double end = std::numeric_limits<double>::infinity()) override {
        
        AutoReleasePool pool;
        
        NSError* error = nullptr;
        CMTimeRange timeRange = CMTimeRangeMake(CMTimeMakeWithSeconds(start, 1), end == std::numeric_limits<double>::infinity() ? kCMTimePositiveInfinity : CMTimeMakeWithSeconds(end, 1));
        
        //---- setup audio reader
        if (audioTrack) {
            audioQueue.clear();
            
            if (audioReader != nullptr)
                [audioReader release];
            
            audioReader = [[AVAssetReader alloc] initWithAsset:asset error:&error];
            if (!audioReader || error) {
                MSG("avf: could not initialize audio reader for '%s'\n", url.c_str());
                throw std::runtime_error("could not initialize audio reader for " + url);
            }
            
            NSDictionary* audioSettings = @{
                AVFormatIDKey : @(kAudioFormatLinearPCM),
                AVSampleRateKey : @(audioFormat.sampleRate),
                AVNumberOfChannelsKey : @(2),
                AVLinearPCMBitDepthKey : @(audioFormat.encoding == PCM_32_FLOAT ? 32 : 16),
                AVLinearPCMIsNonInterleaved : @(audioFormat.interleaved ? NO : YES),
                AVLinearPCMIsFloatKey : @(audioFormat.encoding == PCM_32_FLOAT ? YES : NO),
                AVLinearPCMIsBigEndianKey : @(NO),
            };
            
            [audioReader addOutput:[AVAssetReaderAudioMixOutput assetReaderAudioMixOutputWithAudioTracks:@[audioTrack] audioSettings:audioSettings]];
            
            audioReader.timeRange = timeRange;
            
            if ([audioReader startReading] != YES) {
                [audioReader release];
                audioReader = nullptr;
                MSG("avf: could not start reading audio from '%s': %s\n", url.c_str(), [[[audioReader error] localizedDescription] UTF8String]);
                throw std::runtime_error("could not start reading audio for " + url);
            }
        }
        
        //---- setup video reader
        if (videoTrack) {
            if (videoReader != nullptr)
                [videoReader release];
            
            videoReader = [[AVAssetReader alloc] initWithAsset:asset error:&error];
            if (!videoReader || error) {
                MSG("avf: could not initialize video reader for '%s'\n", url.c_str());
                throw std::runtime_error("could not initialize video reader for " + url);
            }
            
            NSDictionary* videoSettings = @{
                (NSString*)kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_32BGRA)
            };

            [videoReader addOutput:[AVAssetReaderTrackOutput assetReaderTrackOutputWithTrack:videoTrack outputSettings:videoSettings]];
            
            videoReader.timeRange = timeRange;
            
            if ([videoReader startReading] != YES) {
                [videoReader release];
                videoReader = nullptr;
                MSG("avf: could not start reading video from '%s': %s\n", url.c_str(), [[[videoReader error] localizedDescription] UTF8String]);
                throw std::runtime_error("could not start reading video for " + url);
            }
        }
        return NO_ERROR;
    }
    
    bool hasAudio() const override {
        return audioTrack;
    }
    
    bool hasVideo() const override {
        return videoTrack;
    }
    
    double getDuration() const override {
        return duration;
    }
    
    double getVideoFrameRate() const override {
        return videoFrameRate;
    }
    
    int getVideoWidth() const override {
        return videoSize.width;
    }
    
    int getVideoHeight() const override {
        return videoSize.height;
    }

    int decodeAudio(uint8_t* buffer, double& pts) override {
        
        AutoReleasePool pool;
        
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

    int decodeVideo(uint8_t* buffer, double& pts) override {
        
        AutoReleasePool pool;
        
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
        
        int error = decodeVideoSampleBuffer(sampleBuffer, buffer, pts);
        CFRelease(sampleBuffer);
        
        return error;
    }
};
    
} // namespace



namespace Avion {
    
class AVCaptureDecoder;

} // namespace


@interface AudioDataDelegate : NSObject <AVCaptureAudioDataOutputSampleBufferDelegate>
@property Avion::AVCaptureDecoder* decoder;
- (id) initWithDecoder: (Avion::AVCaptureDecoder*) decoder;
@end

@interface VideoDataDelegate : NSObject <AVCaptureVideoDataOutputSampleBufferDelegate>
@property Avion::AVCaptureDecoder* decoder;
- (id) initWithDecoder: (Avion::AVCaptureDecoder*) decoder;
@end


namespace Avion {

class AVCaptureDecoder : public AVDecoderBase {
private:
    static const int MAX_AUDIO_QUEUE_SIZE = 16;
    static const int MAX_VIDEO_QUEUE_SIZE = 16;
    
    AVCaptureSession* audioSession = nullptr;
    AVCaptureDevice* audioDevice = nullptr;
    AVCaptureDeviceInput* audioInput = nullptr;
    AVCaptureAudioDataOutput* audioOutput = nullptr;
    dispatch_queue_t audioDispatchQueue = nullptr;
    AudioDataDelegate* audioDataDelegate = nullptr;

    BlockingQueue<CMSampleBufferRef> audioQueue;
    
    AudioBuffer<uint8_t> audioBuffer;

    
    AVCaptureSession* videoSession = nullptr;
    AVCaptureDevice* videoDevice = nullptr;
    AVCaptureDeviceInput* videoInput = nullptr;
    AVCaptureVideoDataOutput* videoOutput = nullptr;
    dispatch_queue_t videoDispatchQueue = nullptr;
    VideoDataDelegate* videoDataDelegate = nullptr;

    BlockingQueue<CMSampleBufferRef> videoQueue;
    
    double videoFrameRate = 0;
    CGSize videoSize = { 0, 0 };
    
    
public:
    AVCaptureDecoder(std::string url, const AudioFormat& audioFormat, const VideoFormat& videoFormat)
    : AVDecoderBase(url, audioFormat, videoFormat),
      audioQueue(MAX_AUDIO_QUEUE_SIZE), audioBuffer(audioFormat.sampleRate),
      videoQueue(MAX_VIDEO_QUEUE_SIZE) {
        
        AutoReleasePool pool;
        NSError* error = nullptr;
        
        if (audioFormat.decode) {
            // prepare audio session and inputs
            
            audioSession = [[AVCaptureSession alloc] init];
            audioDevice = [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeAudio];
            if (!audioDevice) {
                release();
                MSG("avf: could not get audio capture device '%s'\n", url.c_str());
                throw std::runtime_error("cannot open capture device");
            }

            audioInput = [AVCaptureDeviceInput deviceInputWithDevice:audioDevice error:&error];
            if (!audioInput) {
                release();
                MSG("avf: could not get audio capture device input '%s'\n", url.c_str());
                throw std::runtime_error("cannot open capture device");
            }
            
            if (![audioSession canAddInput:audioInput]) {
                release();
                MSG("avf: cannot add audio input to session '%s'\n", url.c_str());
                throw std::runtime_error("cannot open capture device");
            }
            [audioSession addInput:audioInput];
            
            
            // prepare audio data output
            
            audioOutput = [AVCaptureAudioDataOutput new];
            
            NSDictionary* audioSettings = @{
                AVFormatIDKey : @(kAudioFormatLinearPCM),
                AVSampleRateKey : @(audioFormat.sampleRate),
                AVNumberOfChannelsKey : @(2),
                AVLinearPCMBitDepthKey : @(audioFormat.encoding == PCM_32_FLOAT ? 32 : 16),
                AVLinearPCMIsNonInterleaved : @(audioFormat.interleaved ? NO : YES),
                AVLinearPCMIsFloatKey : @(audioFormat.encoding == PCM_32_FLOAT ? YES : NO),
                AVLinearPCMIsBigEndianKey : @(NO),
            };
            
            audioOutput.audioSettings = audioSettings;
            
            audioDispatchQueue = dispatch_queue_create("AudioDataOutputQueue", DISPATCH_QUEUE_SERIAL);
            
            audioDataDelegate = [[AudioDataDelegate alloc] initWithDecoder:this];
            [audioOutput setSampleBufferDelegate:audioDataDelegate queue:audioDispatchQueue];
            
            if (![audioSession canAddOutput:audioOutput]) {
                release();
                MSG("avf: cannot add audio output to audio session '%s'\n", url.c_str());
                throw std::runtime_error("cannot open capture device");
            }
            [audioSession addOutput:audioOutput];
        }

        
        if (videoFormat.decode) {
            // prepare video session and inputs
            
            videoSession = [[AVCaptureSession alloc] init];
            videoDevice = [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo];
            if (!videoDevice) {
                release();
                MSG("avf: could not get video capture device '%s'\n", url.c_str());
                throw std::runtime_error("cannot open capture device");
            }
            
            videoInput = [AVCaptureDeviceInput deviceInputWithDevice:videoDevice error:&error];
            if (!videoInput) {
                release();
                MSG("avf: could not get video capture device input '%s'\n", url.c_str());
                throw std::runtime_error("cannot open capture device");
            }
            
            if (![videoSession canAddInput:videoInput]) {
                release();
                MSG("avf: cannot add video input to session '%s'\n", url.c_str());
                throw std::runtime_error("cannot open capture device");
            }
            [videoSession addInput:videoInput];

            
            // prepare video data output
            
            videoOutput = [AVCaptureVideoDataOutput new];
            
            NSDictionary* videoSettings = @{
                (NSString*)kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_32BGRA)
            };
            
            videoOutput.videoSettings = videoSettings;
            
            [videoOutput setAlwaysDiscardsLateVideoFrames:NO];
            
            videoDispatchQueue = dispatch_queue_create("VideoDataOutputQueue", DISPATCH_QUEUE_SERIAL);
            
            videoDataDelegate = [[VideoDataDelegate alloc] initWithDecoder:this];
            [videoOutput setSampleBufferDelegate:videoDataDelegate queue:videoDispatchQueue];
            
            if (![videoSession canAddOutput:videoOutput]) {
                release();
                MSG("avf: cannot add video output to session '%s'\n", url.c_str());
                throw std::runtime_error("cannot open capture device");
            }
            [videoSession addOutput:videoOutput];
            
            
            // get video format parameters

            CMFormatDescriptionRef format = [[videoDevice activeFormat] formatDescription];
            CMVideoDimensions dimension = CMVideoFormatDescriptionGetDimensions(format);
            videoSize.width = dimension.width;
            videoSize.height = dimension.height;
            videoFrameRate = 1.0 / CMTimeGetSeconds([videoDevice activeVideoMinFrameDuration]);
        }
    }
    
    virtual ~AVCaptureDecoder() {
        stopCapture();
        release();
    }
    
    int startCapture() override {
        if (audioSession) {
            [audioSession startRunning];
            if (![audioSession isRunning])
                goto error;
        }
        if (videoSession) {
            [videoSession startRunning];
            if (![videoSession isRunning])
                goto error;
        }
        return NO_ERROR;
        
    error:
        stopCapture();
        return INTERNAL_ERROR;
    }
    
    int stopCapture() override {
        if (audioSession)
            [audioSession stopRunning];
        if (videoSession)
            [videoSession stopRunning];
        return NO_ERROR;
    }

    int setRange(double start, double end = std::numeric_limits<double>::infinity()) override {
        return UNSUPPORTED_OPERATION;
    }
    
    bool hasAudio() const override {
        return audioSession;
    }
    
    bool hasVideo() const override {
        return videoSession;
    }
    
    double getDuration() const override {
        return UNSUPPORTED_OPERATION;
    }
    
    double getVideoFrameRate() const override {
        return videoFrameRate;
    }
    
    int getVideoWidth() const override {
        return videoSize.width;
    }
    
    int getVideoHeight() const override {
        return videoSize.height;
    }
    
    int decodeAudio(uint8_t* buffer, double& pts) override {
        return UNSUPPORTED_OPERATION;
    }
    
    int decodeVideo(uint8_t* buffer, double& pts) override {
        return UNSUPPORTED_OPERATION;
    }
    
    void enqueueAudio(AVCaptureOutput* captureOutput, CMSampleBufferRef sampleBuffer, AVCaptureConnection* connection) {
        MSG("avf: enqueueAudio\n");
    }
    
    void enqueueVideo(AVCaptureOutput* captureOutput, CMSampleBufferRef sampleBuffer, AVCaptureConnection* connection) {
        MSG("avf: enqueueVideo\n");        
    }
    
private:
    void release() {
        if (audioDispatchQueue)
            dispatch_release(audioDispatchQueue);
        [audioDataDelegate release];
        [audioOutput release];
        [audioSession release];
        
        if (videoDispatchQueue)
            dispatch_release(videoDispatchQueue);
        [videoDataDelegate release];
        [videoOutput release];
        [videoSession release];
    }
};
    
} // namespace


@implementation AudioDataDelegate

- (id) initWithDecoder:(Avion::AVCaptureDecoder*)decoder {
    self = [super init];
    self.decoder = decoder;
    return self;
}

- (void)captureOutput:(AVCaptureOutput*)captureOutput didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer fromConnection:(AVCaptureConnection*)connection {
    self.decoder->enqueueAudio(captureOutput, sampleBuffer, connection);
}

@end


@implementation VideoDataDelegate

- (id) initWithDecoder:(Avion::AVCaptureDecoder*)decoder {
    self = [super init];
    self.decoder = decoder;
    return self;
}

- (void)captureOutput:(AVCaptureOutput*)captureOutput didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer fromConnection:(AVCaptureConnection*)connection {
    self.decoder->enqueueVideo(captureOutput, sampleBuffer, connection);
}

@end

