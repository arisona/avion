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


#include <string>

#include "avion.hpp"
#include "ch_fhnw_ether_avion_Avion.h"


extern "C" {
    
JNIEXPORT jlong JNICALL Java_ch_fhnw_ether_avion_Avion_decoderCreate(JNIEnv * env, jclass, jstring url,
    jboolean audioDecode, jint audioEncoding, jdouble audioSampleRate, jint audioBufferSize, jboolean audioInterleaved,
    jboolean videoDecode, jint videoPixelFormat, jboolean videoFlip) {
    const char* cUrl = env->GetStringUTFChars(url, JNI_FALSE);
    jlong nativeHandle = 0;
    try {
        AvionDecoder::AudioFormat audioFormat(audioDecode, audioEncoding, audioSampleRate, audioBufferSize, audioInterleaved);
        AvionDecoder::VideoFormat videoFormat(videoDecode, videoPixelFormat, videoFlip);
        nativeHandle = (jlong)AvionDecoder::create(cUrl, audioFormat, videoFormat);
    } catch (...) {
    }
    env->ReleaseStringUTFChars(url, cUrl);
    return nativeHandle;
}

JNIEXPORT void JNICALL Java_ch_fhnw_ether_avion_Avion_decoderDispose
(JNIEnv * env, jclass, jlong nativeHandle) {
    try {
        delete (AvionDecoder*)nativeHandle;
    } catch (...) {
    }
}

JNIEXPORT void JNICALL Java_ch_fhnw_ether_avion_Avion_decoderSetRange
(JNIEnv * env, jclass, jlong nativeHandle, jdouble start, jdouble end) {
    try {
        ((AvionDecoder*)nativeHandle)->setRange(start, end);
    } catch (...) {
    }
}

JNIEXPORT jboolean JNICALL Java_ch_fhnw_ether_avion_Avion_decoderHasAudio
(JNIEnv * env, jclass, jlong nativeHandle) {
    try {
        return ((AvionDecoder*)nativeHandle)->hasAudio();
    } catch (...) {
        return false;
    }
}

JNIEXPORT jboolean JNICALL Java_ch_fhnw_ether_avion_Avion_decoderHasVideo
(JNIEnv * env, jclass, jlong nativeHandle) {
    try {
        return ((AvionDecoder*)nativeHandle)->hasVideo();
    } catch (...) {
        return false;
    }
}

JNIEXPORT jdouble JNICALL Java_ch_fhnw_ether_avion_Avion_decoderGetDuration
(JNIEnv * env, jclass, jlong nativeHandle) {
    try {
        return ((AvionDecoder*)nativeHandle)->getDuration();
    } catch (...) {
        return Avion::INTERNAL_ERROR;
    }
}

JNIEXPORT jdouble JNICALL Java_ch_fhnw_ether_avion_Avion_decoderGetVideoFrameRate
(JNIEnv * env, jclass, jlong nativeHandle) {
    try {
        return ((AvionDecoder*)nativeHandle)->getVideoFrameRate();
    } catch (...) {
        return Avion::INTERNAL_ERROR;
    }
}

JNIEXPORT jint JNICALL Java_ch_fhnw_ether_avion_Avion_decoderGetVideoWidth
(JNIEnv * env, jclass, jlong nativeHandle) {
    try {
        return ((AvionDecoder*)nativeHandle)->getVideoWidth();
    } catch (...) {
        return Avion::INTERNAL_ERROR;
    }
}

JNIEXPORT jint JNICALL Java_ch_fhnw_ether_avion_Avion_decoderGetVideoHeight
(JNIEnv * env, jclass, jlong nativeHandle) {
    try {
        return ((AvionDecoder*)nativeHandle)->getVideoHeight();
    } catch (...) {
        return Avion::INTERNAL_ERROR;
    }
}

JNIEXPORT jint JNICALL Java_ch_fhnw_ether_avion_Avion_decoderDecodeAudio
(JNIEnv * env, jclass, jlong nativeHandle, jobject byteBuffer, jdoubleArray ptsArray) {
    try {
        uint8_t* buffer = (uint8_t*)env->GetDirectBufferAddress(byteBuffer);
        double pts = 0;
        int result = ((AvionDecoder*)nativeHandle)->decodeAudio(buffer, pts);
        if (ptsArray)
            env->SetDoubleArrayRegion(ptsArray, 0, 1, &pts);
        return result;
    } catch (...) {
        return Avion::INTERNAL_ERROR;
    }
}

JNIEXPORT jint JNICALL Java_ch_fhnw_ether_avion_Avion_decoderDecodeVideo
(JNIEnv * env, jclass, jlong nativeHandle, jobject byteBuffer, jdoubleArray ptsArray) {
    try {
        uint8_t* buffer = (uint8_t*)env->GetDirectBufferAddress(byteBuffer);
        double pts = 0;
        int result = ((AvionDecoder*)nativeHandle)->decodeVideo(buffer, pts);
        if (ptsArray)
            env->SetDoubleArrayRegion(ptsArray, 0, 1, &pts);
        return result;
    } catch (...) {
        return Avion::INTERNAL_ERROR;
    }
}

}
