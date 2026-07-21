#ifndef _FFMPEG_HEADERS_
#define _FFMPEG_HEADERS_

extern "C" {
// disable warnings about badly formed documentation from ffmpeg, which don't need at all
#pragma warning(disable : 4635)
// disable warning about conversion int64 to int32
#pragma warning(disable : 4244)

#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libavformat/avio.h"
#include "libavutil/imgutils.h"
#include "libavutil/opt.h"
#include "libswscale/swscale.h"
#include "libavdevice/avdevice.h"
}

#endif // _FFMPEG_HEADERS_
