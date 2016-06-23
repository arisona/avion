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

#include <memory>
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

template<typename R>
class CFRef final {
    R ref;
    
public:
    explicit CFRef() : ref(nullptr) {}
    explicit CFRef(R ref, bool retain = true) : ref(ref) {
        if (this->ref && retain) CFRetain(ref);
    }
    
    CFRef(const CFRef& sb) {
        this->ref = sb.ref;
        if (this->ref) CFRetain(this->ref);
    }
    
    CFRef& operator=(const CFRef& sb) {
        this->ref = sb.ref;
        if (this->ref) CFRetain(this->ref);
        return *this;
    }
    
    CFRef(CFRef&& sb) {
        this->ref = sb.ref;
        sb.ref = nullptr;
    }
    
    CFRef& operator=(CFRef&& sb) {
        this->ref = sb.ref;
        sb.ref = nullptr;
        return *this;
    }

    ~CFRef() { if (ref) CFRelease(ref); }
    
    R get() const { return ref; }
    R operator*() const { return ref; }
    bool operator!() const { return ref; }
};
    
} // namespace


// TODO:
// - consistent error handling (exceptions vs returned errors)
// - flip / no flip image vertically, optimise pixel transfer
// - RGBA / BGRA support


namespace Avion { class AVCaptureDecoder; }

@interface AudioDataDelegate : NSObject <AVCaptureAudioDataOutputSampleBufferDelegate>
@property Avion::AVCaptureDecoder* decoder;
- (id) initWithDecoder: (Avion::AVCaptureDecoder*) decoder;
@end

@interface VideoDataDelegate : NSObject <AVCaptureVideoDataOutputSampleBufferDelegate>
@property Avion::AVCaptureDecoder* decoder;
- (id) initWithDecoder: (Avion::AVCaptureDecoder*) decoder;
@end


namespace Avion {

class AVDecoderBase : public AvionDecoder {
protected:
    static const int QUEUE_TAKE_TIMEOUT_MS = 1000;
    static const int MAX_AUDIO_QUEUE_SIZE = 16;
    static const int MAX_VIDEO_QUEUE_SIZE = 16;
    
    struct AudioType {
        typedef AVCaptureAudioDataOutput Output;
        typedef AudioDataDelegate Delegate;
        static std::string tag() { return "audio"; }
        static NSString* type() { return AVMediaTypeAudio; }
        static const int queueSize = MAX_AUDIO_QUEUE_SIZE;
        static AVAssetReaderOutput* output(AVAssetTrack* track, NSDictionary* settings) {
            return [AVAssetReaderAudioMixOutput assetReaderAudioMixOutputWithAudioTracks:@[track] audioSettings:settings];
        }
        static void settings(AVCaptureAudioDataOutput* output, NSDictionary* settings) { output.audioSettings = settings; }
    };
    
    struct VideoType {
        typedef AVCaptureVideoDataOutput Output;
        typedef VideoDataDelegate Delegate;
        static std::string tag() { return "video"; }
        static NSString* type() { return AVMediaTypeVideo; }
        static const int queueSize = MAX_VIDEO_QUEUE_SIZE;
        static AVAssetReaderOutput* output(AVAssetTrack* track, NSDictionary* settings) {
            return [AVAssetReaderTrackOutput assetReaderTrackOutputWithTrack:track outputSettings:settings];
        }
        static void settings(AVCaptureVideoDataOutput* output, NSDictionary* settings) { output.videoSettings = settings; }
    };
    
    AVDecoderBase(std::string url, AudioFormat audioFormat, VideoFormat videoFormat)
    : AvionDecoder(url, audioFormat, videoFormat) {}
    
    ~AVDecoderBase() {}

    int decodeAudioSampleBuffer(CMSampleBufferRef ref, AudioBuffer<uint8_t>& buffer) {
        double pts = CMTimeGetSeconds(CMSampleBufferGetPresentationTimeStamp(ref));
        
        CMBlockBufferRef blockBuffer = CMSampleBufferGetDataBuffer(ref);
        if (!blockBuffer)
            throw std::runtime_error("cannot get block buffer from sample buffer: " + url);
        
        size_t length = 0;
        uint8_t* samples = nullptr;
        if (CMBlockBufferGetDataPointer(blockBuffer, 0, nullptr, &length, (char**)&samples) != kCMBlockBufferNoErr)
            throw std::runtime_error("cannot get audio data from block buffer");
        
        //MSG("avf: put audio samples: %ld %f %f %f %f \n", length, samples[0], samples[1], samples[2], samples[3]);
        buffer.put(samples, length, pts);
        return NO_ERROR;
    }
    
    int decodeVideoSampleBuffer(CMSampleBufferRef ref, uint8_t* buffer, double& pts) {
        int w = getVideoWidth();
        int h = getVideoHeight();
        uint32_t* dst = (uint32_t*)buffer;
        pts = CMTimeGetSeconds(CMSampleBufferGetPresentationTimeStamp(ref));
        
        CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(ref);
        if (!imageBuffer)
            throw std::runtime_error("cannot get image buffer from sample buffer: " + url);
        
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
        
        CVPixelBufferUnlockBaseAddress(imageBuffer, 0);
        return NO_ERROR;
    }
    
    NSDictionary* getAudioSettings() {
        NSDictionary* settings = @{
            AVFormatIDKey : @(kAudioFormatLinearPCM),
            AVSampleRateKey : @(audioFormat.sampleRate),
            AVNumberOfChannelsKey : @(2),
            AVLinearPCMBitDepthKey : @(audioFormat.encoding == PCM_32_FLOAT ? 32 : 16),
            AVLinearPCMIsNonInterleaved : @(audioFormat.interleaved ? NO : YES),
            AVLinearPCMIsFloatKey : @(audioFormat.encoding == PCM_32_FLOAT ? YES : NO),
            AVLinearPCMIsBigEndianKey : @(NO),
        };
        return settings;
    }

    NSDictionary* getVideoSettings() {
        NSDictionary* settings = @{
            (NSString*)kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_32BGRA)
        };
        return settings;
    }
};



class AVAssetDecoder : public AVDecoderBase {
    
    template<typename TYPE>
    class Reader {
    public:
        AVAssetTrack* track = nullptr;
        AVAssetReader* reader = nullptr;
        AVAssetReaderOutput* output = nullptr;
        
        Reader(AVAsset* asset) {
            NSArray* tracks = [asset tracksWithMediaType:TYPE::type()];
            if ([tracks count] == 0)
                throw std::runtime_error("no tracks");
            track = [tracks objectAtIndex:0];
        }
        
        ~Reader() {
            [reader release];
        }
        
        void start(AVAsset* asset, NSDictionary* settings, CMTimeRange range) {
            if (reader) {
                [reader release];
                reader = nullptr;
                output = nullptr;
            }
            NSError* error = nullptr;
            reader = [[AVAssetReader alloc] initWithAsset:asset error:&error];
            if (!reader || error)
                throw std::runtime_error("could not initialize reader for");

            output = TYPE::output(track, settings);
            [reader addOutput:output];
            reader.timeRange = range;
            if ([reader startReading] != YES) {
                [reader release];
                reader = nullptr;
                throw std::runtime_error("could not start reading");
            }
        }
        
        bool running() {
            return reader && [reader status] == AVAssetReaderStatusReading;
        }
        
        CFRef<CMSampleBufferRef> nextBuffer() {
            auto ref = CFRef<CMSampleBufferRef>([output copyNextSampleBuffer], false);
            if (!ref)
                throw std::runtime_error("could not get sample buffer");
            return ref;
        }
    };

private:
    AVAsset* asset = nullptr;
    
    std::unique_ptr<Reader<AudioType>> audioReader;
    AudioBuffer<uint8_t> audioBuffer;

    std::unique_ptr<Reader<VideoType>> videoReader;
    double videoFrameRate = 0;
    CGSize videoSize = { 0, 0 };
    double duration = 0;
    
public:
    AVAssetDecoder(std::string url, AudioFormat audioFormat, VideoFormat videoFormat)
    : AVDecoderBase(url, audioFormat, videoFormat), audioBuffer(audioFormat.sampleRate) {
        @autoreleasepool {
            NSURL* nsUrl = [NSURL URLWithString:[NSString stringWithCString:url.c_str() encoding:NSUTF8StringEncoding]];
            if (!nsUrl)
                throw std::invalid_argument("invalid url: " + url);
              
            NSDictionary* options = @{ AVURLAssetPreferPreciseDurationAndTimingKey : @YES };
            asset = [AVURLAsset URLAssetWithURL:nsUrl options:options];
            if (!asset)
                throw std::invalid_argument("invalid url: " + url);
            
            if (audioFormat.decode) {
                try {
                    audioReader = std::unique_ptr<Reader<AudioType>>(new Reader<AudioType>(asset));
                } catch (...) {
                }
            }
            
            if (videoFormat.decode) {
                try {
                    videoReader = std::unique_ptr<Reader<VideoType>>(new Reader<VideoType>(asset));
                    videoFrameRate = [videoReader->track nominalFrameRate];
                    videoSize = [videoReader->track naturalSize];
                } catch (...) {
                }
            }
            duration = CMTimeGetSeconds([asset duration]);
            setRange(0.0);
            
            //MSG("avf: %s: duration=%f framerate=%f size=%dx%d\n", url.c_str(), duration, videoFrameRate, (int)videoSize.width, (int)videoSize.height);
        }
    }
    
    ~AVAssetDecoder() {
        @autoreleasepool {
            audioReader.reset();
            videoReader.reset();
        }
    }
    
    int setRange(double start, double end = std::numeric_limits<double>::infinity()) override {
        @autoreleasepool {
            CMTimeRange timeRange = CMTimeRangeMake(CMTimeMakeWithSeconds(start, 1), end == std::numeric_limits<double>::infinity() ? kCMTimePositiveInfinity : CMTimeMakeWithSeconds(end, 1));
            if (audioReader) {
                audioBuffer.clear();
                audioReader->start(asset, getAudioSettings(), timeRange);
            }
            if (videoReader) {
                videoReader->start(asset, getVideoSettings(), timeRange);
            }
            return NO_ERROR;
        }
    }
    
    bool hasAudio() const override {
        return audioReader ? true : false;
    }
    
    bool hasVideo() const override {
        return videoReader ? true : false;
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
        @autoreleasepool {
            if (!audioReader)
                return NO_SUCH_STREAM;
            if (!audioReader->running())
                return END_OF_STREAM;
            while (audioBuffer.size() < audioFormat.bufferSize) {
                CFRef<CMSampleBufferRef> ref = audioReader->nextBuffer();
                decodeAudioSampleBuffer(ref.get(), audioBuffer);
            }
            if (!audioBuffer.size())
                return END_OF_STREAM;
            return audioBuffer.take(buffer, audioFormat.bufferSize, pts);
        }
    }

    int decodeVideo(uint8_t* buffer, double& pts) override {
        @autoreleasepool {
            if (!videoReader)
                return NO_SUCH_STREAM;
            if (!videoReader->running())
                return END_OF_STREAM;
            CFRef<CMSampleBufferRef> ref = videoReader->nextBuffer();
            return decodeVideoSampleBuffer(ref.get(), buffer, pts);
        }
    }
};
    

class AVCaptureDecoder : public AVDecoderBase {

    template<typename TYPE>
    class Session {
    public:
        AVCaptureSession* session = nullptr;
        AVCaptureDevice* device = nullptr;
        AVCaptureDeviceInput* input = nullptr;
        typename TYPE::Output* output = nullptr;
        dispatch_queue_t dispatchQueue = nullptr;
        typename TYPE::Delegate* delegate = nullptr;
        
        BlockingQueue<CFRef<CMSampleBufferRef>> queue;
        
        Session(AVCaptureDecoder* decoder, NSDictionary* settings) : queue(TYPE::queueSize) {
            NSError* error = nullptr;

            // prepare session and inputs
            session = [[AVCaptureSession alloc] init];
            device = [AVCaptureDevice defaultDeviceWithMediaType:TYPE::type()];
            if (!device) {
                release();
                MSG("avf: could not get capture device\n");
                throw std::runtime_error("cannot open capture device");
            }
            input = [AVCaptureDeviceInput deviceInputWithDevice:device error:&error];
            if (!input) {
                release();
                MSG("avf: could not get capture device input\n");
                throw std::runtime_error("cannot open capture device");
            }
            if (![session canAddInput:input]) {
                release();
                MSG("avf: cannot add input to session\n");
                throw std::runtime_error("cannot open capture device");
            }
            [session addInput:input];
            
            // prepare data output
            output = [typename TYPE::Output new];
            TYPE::settings(output, settings);
            dispatchQueue = dispatch_queue_create("DataOutputQueue", DISPATCH_QUEUE_SERIAL);
            delegate = [[typename TYPE::Delegate alloc] initWithDecoder:decoder];
            [output setSampleBufferDelegate:delegate queue:dispatchQueue];
            if (![session canAddOutput:output]) {
                release();
                MSG("avf: cannot add output to audio session\n");
                throw std::runtime_error("cannot open capture device");
            }
            [session addOutput:output];
        }
        
        ~Session() {
            release();
        }
        
        void start() {
            [session startRunning];
            if (![session isRunning]) {
                MSG("avf: cannot start capture session\n");
                throw std::runtime_error("cannot start capture session");
            }
        }
        
        void stop() {
            [session stopRunning];
        }
        
        void release() {
            if (dispatchQueue)
                dispatch_release(dispatchQueue);
            [delegate release];
            [output release];
            [session release];
        }
    };
    
private:
    std::unique_ptr<Session<AudioType>> audioSession;
    AudioBuffer<uint8_t> audioBuffer;

    std::unique_ptr<Session<VideoType>> videoSession;
    double videoFrameRate = 0;
    CGSize videoSize = { 0, 0 };
    
public:
    AVCaptureDecoder(std::string url, const AudioFormat& audioFormat, const VideoFormat& videoFormat)
    : AVDecoderBase(url, audioFormat, videoFormat),
      audioBuffer(audioFormat.sampleRate) {
        @autoreleasepool {
            if (audioFormat.decode)
                audioSession = std::unique_ptr<Session<AudioType>>(new Session<AudioType>(this, getAudioSettings()));

            if (videoFormat.decode)
                videoSession = std::unique_ptr<Session<VideoType>>(new Session<VideoType>(this, getVideoSettings()));
        }
    }
    
    virtual ~AVCaptureDecoder() {
        @autoreleasepool {
            stopCapture();
            // explicitly reset, so we're within pool scope
            audioSession.reset();
            videoSession.reset();
        }
    }
    
    int startCapture() override {
        @autoreleasepool {
            try {
                if (audioSession)
                    audioSession->start();
                if (videoSession)
                    videoSession->start();
                return NO_ERROR;
            } catch (std::exception& e) {
                stopCapture();
                throw e;
            }
        }
    }
    
    int stopCapture() override {
        @autoreleasepool {
            if (audioSession)
                audioSession->stop();
            if (videoSession)
                videoSession->stop();
            return NO_ERROR;
        }
    }

    int setRange(double start, double end = std::numeric_limits<double>::infinity()) override {
        return UNSUPPORTED_OPERATION;
    }
    
    bool hasAudio() const override {
        return audioSession ? true : false;
    }
    
    bool hasVideo() const override {
        return videoSession ? true : false;
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
        @autoreleasepool {
            while (audioSession->queue.size() < audioFormat.bufferSize) {
                CFRef<CMSampleBufferRef> ref;
                auto result = audioSession->queue.take(ref, QUEUE_TAKE_TIMEOUT_MS);
                if (result == QueueTakeResult::ok) {
                    int error = decodeAudioSampleBuffer(ref.get(), audioBuffer);
                    if (error)
                        return error;
                } else if (result == QueueTakeResult::overflow) {
                    return QUEUE_OVERFLOW;
                } else if (result == QueueTakeResult::timeout) {
                    return WAIT_TIMEOUT;
                }
                return INTERNAL_ERROR;
            }
            if (!audioSession->queue.size())
                return END_OF_STREAM;
            int received = audioBuffer.take(buffer, audioFormat.bufferSize, pts);
            //MSG("avf: get audio samples: %d %f %f %f %f \n", received, buffer[0], buffer[1], buffer[2], buffer[3]);
            return received;
        }
    }
    
    int decodeVideo(uint8_t* buffer, double& pts) override {
        @autoreleasepool {
            CFRef<CMSampleBufferRef> ref;
            auto result = videoSession->queue.take(ref, QUEUE_TAKE_TIMEOUT_MS);
            if (result == QueueTakeResult::ok)
                return decodeVideoSampleBuffer(ref.get(), buffer, pts);
            else if (result == QueueTakeResult::overflow)
                return QUEUE_OVERFLOW;
            else if (result == QueueTakeResult::timeout)
                return WAIT_TIMEOUT;
            return INTERNAL_ERROR;
        }
    }
    
    void enqueueAudio(AVCaptureOutput* captureOutput, CMSampleBufferRef sampleBuffer, AVCaptureConnection* connection) {
        MSG("avf: enqueueAudio\n");
        if (!audioSession->queue.put(CFRef<CMSampleBufferRef>(sampleBuffer))) {
            MSG("avf: audio queue overflow\n");
        }
    }
    
    void enqueueVideo(AVCaptureOutput* captureOutput, CMSampleBufferRef sampleBuffer, AVCaptureConnection* connection) {
        MSG("avf: enqueueVideo\n");        
        if (!videoSession->queue.put(CFRef<CMSampleBufferRef>(sampleBuffer))) {
            MSG("avf: video queue overflow\n");
        }
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

