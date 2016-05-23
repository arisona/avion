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

#include "avion.hpp"

#include "ch_fhnw_ether_avion_Avion.h"

int main(int argc, char *argv[]) {
    
    printf("avion test flight\n");

    @autoreleasepool {
        const int audioSize = 1024;
        
        AvionDecoder::AudioFormat audioFormat(true, AvionDecoder::PCM_32_FLOAT, 44100.0, 1024, true);
        AvionDecoder::VideoFormat videoFormat(true, AvionDecoder::RGBA, true);
        AvionDecoder* decoder = AvionDecoder::create("file:///Users/radar/Desktop/simian_mobile_disco-audacity_of_huge_(2009).mp4", audioFormat, videoFormat);
        
        //wrapper->seek(60);
        
        size_t size = decoder->getVideoWidth() * decoder->getVideoHeight() * 4;
        uint8_t image[size];
        
        uint8_t samples[audioSize];
        
        int error = 0;
        double pts = 0;
        while (error != -1) {
            error = decoder->decodeVideo(image, pts);
            printf("got video frame %f %d\n", pts, error);

            error = decoder->decodeAudio(samples, pts);
            printf("got audio frame %f %d\n", pts, error);
        }
        
        delete decoder;
    }
	
    return 0;
}

