# avion

Avion is an audio/video decoding and encoding framework for Java and C++. In contrast to other frameworks, it uses of the native media frameworks on individual platforms (AVFoundation on Mac OS, Microsoft Media Foundation on Windows, ffmpeg on Linux).

The main goal is to provide a simple, intuitive API for accessing raw audio and video data, or for encoding media files from raw data. It does not include higher level functionality such as GUI elements for media players. Also focus is on standard audio/video formats, and we do not aim at supporting legacy and rarely used format.

A typical application example is to decode video files, and use the video frames as OpenGL textures, e.g. for VJing tools or game engines.

## Features

- Simple, immediate API for the major desktop platforms
- Multi-threaded decoding of Audio and Video streams
- Support for standard formats and containers (e.g. H.264; .mov, .mp4)
- Support for video capture devices (web cams, etc.)


## TODO

- API finalization
- Windows and Linux support (Windows media foundation, ffmpeg)
- Video capture support
- Encoding support
