# avion

Avion is an audio/video decoding and encoding framework for Java and C++. In contrast to other frameworks, it uses of the native media frameworks on individual platforms (AVFoundation on Mac OS, Microsoft Media Foundation on Windows, ffmpeg on Linux).

The main goal is to provide a simple, intuitive API for accessing raw audio and video data, or for encoding media files from raw data. It does not include higher level functionality such as GUI elements for media players. Further, focus is on standard audio/video formats, and we do not aim at supporting legacy and rarely used format. Finally, by using the native frameworks, one does not have to deal with decoder / encoder licensing issues.

A typical application example is to decode video files, and use the video frames as OpenGL textures, e.g. for VJing tools or game engines.

## Features

- Simple, immediate API for the major desktop platforms
- Uses platform-native AV frameworks where available
- Multi-threaded decoding of Audio and Video streams
- Support for standard formats and containers (e.g. H.264; .mov, .mp4)
- Support for video capture devices (web cams, etc.)
- Small size (20KB...)
- BSD License; no licensing issues since using native OS AV libraries


## TODO

- API finalization
- Windows and Linux support (Microsoft Media Foundation, ffmpeg)
- Video capture support
- Encoding support


## RFC

In order to finalize the API, requests for changes, features, etc. are welcome. Also, if someone is experienced with Microsoft Media Foundation and would like to start hacking, please be in touch, or fork and I'll be happy to pull.
