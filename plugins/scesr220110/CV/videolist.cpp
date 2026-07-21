#include <map>
#include <boost/bimap.hpp>
#include <boost/assign.hpp>
#include "ffmpeg.h"
#include "videolist.h"

using bipixfmt = boost::bimaps::bimap<enum AVPixelFormat, unsigned int>;
extern const bipixfmt ff_raw_pix_fmt_tags;

#if defined(WINDOWS) || defined(_WINDOWS) || defined(WIN32) || defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <dshow.h>
#include <mfapi.h>
#include <dvdmedia.h>
#pragma comment(lib, "strmiids.lib")

const std::map<DeviceType, REFGUID> GuidMapper = {
    { DeviceType ::AudioInput, CLSID_AudioInputDeviceCategory }, // 麦克风
    { DeviceType ::AudioRenderer, CLSID_AudioRendererCategory }, // 扬声器
    { DeviceType ::VideoInput, CLSID_VideoInputDeviceCategory } // 摄像头
};

//-----------------------------------------------------------------------------
// https://docs.microsoft.com/zh-cn/windows/win32/medfound/saferelease
//-----------------------------------------------------------------------------

template <typename T>
void SafeRelease(T **ppT)
{
    if (*ppT) {
        (*ppT)->Release();
        *ppT = NULL;
    }
}

//-----------------------------------------------------------------------------

bool GetAudioVideoInputDevices(std::vector<DeviceInfo> &devices, DeviceType type)
{
    // 初始化
    devices.clear();
    // 初始化COM
    if (CoInitializeEx(NULL, COINIT_APARTMENTTHREADED) < 0) {
        return false;
    }
    // 创建系统设备枚举器实例
    ICreateDevEnum *dev_enum = NULL;
    if (CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER, //
            IID_ICreateDevEnum, (void **)&dev_enum)
        < 0) {
        CoUninitialize();
        return false;
    }

    IEnumMoniker *enummon = NULL;
    IMoniker *moniker = NULL;
    IPropertyBag *prop = NULL;
    HRESULT hr;

    // 获取设备类枚举器
    if (dev_enum->CreateClassEnumerator(GuidMapper.at(type), &enummon, 0) == S_OK) {
        // 枚举设备名称
        while (enummon->Next(1, &moniker, NULL) == S_OK) {
            hr = moniker->BindToStorage(0, 0, IID_IPropertyBag, (void **)&prop);
            if (SUCCEEDED(hr)) {
                DeviceInfo info;
                VARIANT name;
                LPOLESTR pOleDisplayName;

                VariantInit(&name);

                // 获取设备名
                hr = prop->Read(L"FriendlyName", &name, NULL);
                if (SUCCEEDED(hr)) {
                    info.DeviceName = name.bstrVal;
                } else {
                    goto next;
                }

                // 获取设备路径
                hr = prop->Read(L"DevicePath", &name, NULL);
                if (SUCCEEDED(hr)) {
                    info.DevicePath = name.bstrVal;
                } else {
                    goto next;
                }

                // 获取设备Moniker名
                pOleDisplayName = reinterpret_cast<LPOLESTR>(CoTaskMemAlloc(1024));
                if (pOleDisplayName != NULL) {
                    hr = moniker->GetDisplayName(NULL, NULL, &pOleDisplayName);
                    if (SUCCEEDED(hr)) {
                        info.MonikerName = pOleDisplayName;
                        CoTaskMemFree(pOleDisplayName);

                        info.DeviceType = type;
                        devices.push_back(info);
                    }
                }

            next:
                SafeRelease(&prop);
                VariantClear(&name);
            }
            SafeRelease(&moniker);
        }
        SafeRelease(&enummon);
    }
    CoUninitialize();
    return devices.size() > 0;
}

std::vector<std::vector<FormatInfo>> dshow_cycle_pins(IBaseFilter *device_filter);

bool GetCameraFormats(std::vector<std::vector<FormatInfo>> &formats, const DeviceInfo &device)
{
    if (device.DeviceType != DeviceType::VideoInput) {
        throw std::runtime_error("`GetDeviceFilterByName` only support video device now.");
    }

    // 初始化
    formats.clear();
    // 初始化COM
    if (CoInitializeEx(NULL, COINIT_APARTMENTTHREADED) < 0) {
        return false;
    }
    // 创建系统设备枚举器实例
    ICreateDevEnum *dev_enum = NULL;
    if (CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER, //
            IID_ICreateDevEnum, (void **)&dev_enum)
        < 0) {
        CoUninitialize();
        return false;
    }

    IEnumMoniker *enummon = NULL;
    IMoniker *moniker = NULL;
    IPropertyBag *prop = NULL;
    HRESULT hr;

    auto done = [&]() { return SUCCEEDED(hr); };

    // 获取设备类枚举器
    if (dev_enum->CreateClassEnumerator(GuidMapper.at(device.DeviceType), &enummon, 0) == S_OK) {
        // 枚举设备名称
        while (enummon->Next(1, &moniker, NULL) == S_OK) {
            hr = moniker->BindToStorage(0, 0, IID_IPropertyBag, (void **)&prop);
            if (SUCCEEDED(hr)) {
                VARIANT name;
                VariantInit(&name);

                // 获取设备名
                hr = prop->Read(L"FriendlyName", &name, NULL);
                if (SUCCEEDED(hr)) {
                    if (device.DeviceName == name.bstrVal) {
                        IBaseFilter *filter = NULL;
                        hr = moniker->BindToObject(0, 0, IID_IBaseFilter, (void **)&filter);
                        if (SUCCEEDED(hr)) {
                            formats = dshow_cycle_pins(filter);
                            SafeRelease(&filter);
                            goto exit;
                        }
                    }
                }
                VariantClear(&name);
                SafeRelease(&prop);
            }
            SafeRelease(&moniker);
        }
        SafeRelease(&enummon);
    }

exit:
    SafeRelease(&enummon);
    SafeRelease(&moniker);
    SafeRelease(&prop);
    CoUninitialize();
    return formats.size() > 0;
}

//-----------------------------------------------------------------------------
// https://github.com/cocos/cocomat/tree/main/ffmpeg
//-----------------------------------------------------------------------------

enum AVPixelFormat dshow_pixfmt(DWORD biCompression, WORD biBitCount)
{
    switch (biCompression) {
    case BI_BITFIELDS:
    case BI_RGB:
        switch (biBitCount) { /* 1-8 are untested */
        case 1:
            return AV_PIX_FMT_MONOWHITE;
        case 4:
            return AV_PIX_FMT_RGB4;
        case 8:
            return AV_PIX_FMT_RGB8;
        case 16:
            return AV_PIX_FMT_RGB555;
        case 24:
            return AV_PIX_FMT_BGR24;
        case 32:
            return AV_PIX_FMT_0RGB32;
        }
    }

    if (ff_raw_pix_fmt_tags.right.find(biCompression) != ff_raw_pix_fmt_tags.right.end()) {
        return ff_raw_pix_fmt_tags.right.at(biCompression);
    }
    return AV_PIX_FMT_NONE;
}

/**
 * Cycle through available formats using the specified pin,
 * try to set parameters specified through AVOptions and if successful
 * return 1 in *pformat_set.
 * If pformat_set is NULL, list all pin capabilities.
 */
std::vector<FormatInfo> dshow_cycle_formats(IPin *pin, int *pformat_set)
{
    std::vector<FormatInfo> infos;

    IAMStreamConfig *config = NULL;
    AM_MEDIA_TYPE *type = NULL;
    int format_set = 0;
    void *caps = NULL;
    int i, n, size, r;

    if (pin->QueryInterface(IID_IAMStreamConfig, (void **)&config) != S_OK)
        return infos;
    if (config->GetNumberOfCapabilities(&n, &size) != S_OK)
        goto end;

    caps = av_malloc(size);
    if (!caps)
        goto end;

    for (i = 0; i < n && !format_set; i++) {
        r = config->GetStreamCaps(i, &type, (BYTE *)caps);
        if (r != S_OK)
            goto next;

        {
            VIDEO_STREAM_CONFIG_CAPS *vcaps = (VIDEO_STREAM_CONFIG_CAPS *)caps;
            BITMAPINFOHEADER *bih;
            int64_t *fr;
            const AVCodecTag *const tags[] = { avformat_get_riff_video_tags(), NULL };

            if (IsEqualGUID(type->formattype, FORMAT_VideoInfo)) {
                VIDEOINFOHEADER *v = (VIDEOINFOHEADER *)type->pbFormat;
                fr = &v->AvgTimePerFrame;
                bih = &v->bmiHeader;
            } else if (IsEqualGUID(type->formattype, FORMAT_VideoInfo2)) {
                VIDEOINFOHEADER2 *v = (VIDEOINFOHEADER2 *)type->pbFormat;
                fr = &v->AvgTimePerFrame;
                bih = &v->bmiHeader;
            } else {
                goto next;
            }
            if (!pformat_set) {
                FormatInfo info;

                enum AVPixelFormat pix_fmt = dshow_pixfmt(bih->biCompression, bih->biBitCount);
                if (pix_fmt == AV_PIX_FMT_NONE) {
                    enum AVCodecID codec_id = av_codec_get_id(tags, bih->biCompression);
                    AVCodec *codec = avcodec_find_decoder(codec_id);
                    if (codec_id == AV_CODEC_ID_NONE || !codec) {
                        // printf("  unknown compression type 0x%X", (int)bih->biCompression);
                        info.compress = (int)bih->biCompression;
                    } else {
                        // printf("  vcodec=%s", codec->name);
                        info.vcodec = codec->name;
                    }
                } else {
                    // printf("  pixel_format=%s", av_get_pix_fmt_name(pix_fmt));
                    info.pixel_format = pix_fmt;
                }

                // printf("  min s=%ldx%ld fps=%g max s=%ldx%ld fps=%g\n", vcaps->MinOutputSize.cx,
                //    vcaps->MinOutputSize.cy, 1e7 / vcaps->MaxFrameInterval,
                //    vcaps->MaxOutputSize.cx, vcaps->MaxOutputSize.cy, 1e7 /
                //    vcaps->MinFrameInterval);

                info.min.width = vcaps->MinOutputSize.cx;
                info.min.height = vcaps->MinOutputSize.cy;
                info.min.fps = 1.0e7 / vcaps->MaxFrameInterval;
                info.max.width = vcaps->MaxOutputSize.cx;
                info.max.height = vcaps->MaxOutputSize.cy;
                info.max.fps = 1.0e7 / vcaps->MinFrameInterval;
                infos.push_back(info);
                continue;
            }
        }

        if (config->SetFormat(type) != S_OK)
            goto next;
        format_set = 1;
    next:
        if (type->pbFormat)
            CoTaskMemFree(type->pbFormat);
        CoTaskMemFree(type);
    }
end:
    SafeRelease(&config);
    av_free(caps);
    if (pformat_set)
        *pformat_set = format_set;

    return infos;
}

char *dup_wchar_to_utf8(wchar_t *w)
{
    char *s = NULL;
    int l = WideCharToMultiByte(CP_UTF8, 0, w, -1, 0, 0, 0, 0);
    s = (char *)av_malloc(l);
    if (s)
        WideCharToMultiByte(CP_UTF8, 0, w, -1, s, l, 0, 0);
    return s;
}

/**
 * Cycle through available pins using the device_filter device, of type
 * devtype, retrieve the first output pin and return the pointer to the
 * object found in *ppin.
 * If ppin is NULL, cycle through all pins listing audio/video capabilities.
 */
std::vector<std::vector<FormatInfo>> dshow_cycle_pins(IBaseFilter *device_filter)
{
    std::vector<std::vector<FormatInfo>> infoss;

    IEnumPins *pins = 0;
    IPin *pin;
    int r;

    r = device_filter->EnumPins(&pins);
    if (r != S_OK) {
        // printf("Could not enumerate pins.\n");
        return infoss;
    }

    while (pins->Next(1, &pin, NULL) == S_OK) {
        IKsPropertySet *p = NULL;
        PIN_INFO info = { 0 };
        GUID category;
        DWORD r2;

        pin->QueryPinInfo(&info);
        SafeRelease(&info.pFilter);

        if (info.dir != PINDIR_OUTPUT)
            goto next;
        if (pin->QueryInterface(IID_IKsPropertySet, (void **)&p) != S_OK)
            goto next;
        if (p->Get(AMPROPSETID_Pin, AMPROPERTY_PIN_CATEGORY, NULL, 0, &category, sizeof(GUID), &r2)
            != S_OK)
            goto next;
        if (!IsEqualGUID(category, PIN_CATEGORY_CAPTURE))
            goto next;

        infoss.push_back(dshow_cycle_formats(pin, NULL));

    next:
        SafeRelease(&p);
        SafeRelease(&pin);
    }
    SafeRelease(&pins);
    return infoss;
}

#else

bool GetAudioVideoInputDevices(std::vector<DeviceInfo> &devices, DeviceType type)
{
    return false;
}

bool GetCameraFormats(std::vector<std::vector<FormatInfo>> &formats, const DeviceInfo &device)
{
    return false;
}

#endif

//-----------------------------------------------------------------------------
// Example
//-----------------------------------------------------------------------------

// int main()
//{
//    std::vector<DeviceInfo> devices;
//
//    GetAudioVideoInputDevices(devices, DeviceType::VideoInput);
//    if (devices.size() > 0) {
//        std::vector<std::vector<FormatInfo>> formats;
//        if (GetCameraFormats(formats, devices[0])) {
//        }
//    }
//    return 0;
//}

//-----------------------------------------------------------------------------

const bipixfmt ff_raw_pix_fmt_tags = boost::assign::list_of<bipixfmt::relation>
    // clang-format off
    ( AV_PIX_FMT_YUV420P, MKTAG('I', '4', '2', '0') ) /* Planar formats */
    ( AV_PIX_FMT_YUV420P, MKTAG('I', 'Y', 'U', 'V') )
    ( AV_PIX_FMT_YUV420P, MKTAG('y', 'v', '1', '2') )
    ( AV_PIX_FMT_YUV420P, MKTAG('Y', 'V', '1', '2') )
    ( AV_PIX_FMT_YUV410P, MKTAG('Y', 'U', 'V', '9') )
    ( AV_PIX_FMT_YUV410P, MKTAG('Y', 'V', 'U', '9') )
    ( AV_PIX_FMT_YUV411P, MKTAG('Y', '4', '1', 'B') )
    ( AV_PIX_FMT_YUV422P, MKTAG('Y', '4', '2', 'B') )
    ( AV_PIX_FMT_YUV422P, MKTAG('P', '4', '2', '2') )
    ( AV_PIX_FMT_YUV422P, MKTAG('Y', 'V', '1', '6') )
    /* yuvjXXX formats are deprecated hacks specific to libav*,
       they are identical to yuvXXX  */
    ( AV_PIX_FMT_YUVJ420P, MKTAG('I', '4', '2', '0') ) /* Planar formats */
    ( AV_PIX_FMT_YUVJ420P, MKTAG('I', 'Y', 'U', 'V') )
    ( AV_PIX_FMT_YUVJ420P, MKTAG('Y', 'V', '1', '2') )
    ( AV_PIX_FMT_YUVJ422P, MKTAG('Y', '4', '2', 'B') )
    ( AV_PIX_FMT_YUVJ422P, MKTAG('P', '4', '2', '2') )
    ( AV_PIX_FMT_GRAY8, MKTAG('Y', '8', '0', '0') )
    ( AV_PIX_FMT_GRAY8, MKTAG('Y', '8', ' ', ' ') )

    ( AV_PIX_FMT_YUYV422, MKTAG('Y', 'U', 'Y', '2') ) /* Packed formats */
    ( AV_PIX_FMT_YUYV422, MKTAG('Y', '4', '2', '2') )
    ( AV_PIX_FMT_YUYV422, MKTAG('V', '4', '2', '2') )
    ( AV_PIX_FMT_YUYV422, MKTAG('V', 'Y', 'U', 'Y') )
    ( AV_PIX_FMT_YUYV422, MKTAG('Y', 'U', 'N', 'V') )
    ( AV_PIX_FMT_YUYV422, MKTAG('Y', 'U', 'Y', 'V') )
    ( AV_PIX_FMT_YVYU422, MKTAG('Y', 'V', 'Y', 'U') ) /* Philips */
    ( AV_PIX_FMT_UYVY422, MKTAG('U', 'Y', 'V', 'Y') )
    ( AV_PIX_FMT_UYVY422, MKTAG('H', 'D', 'Y', 'C') )
    ( AV_PIX_FMT_UYVY422, MKTAG('U', 'Y', 'N', 'V') )
    ( AV_PIX_FMT_UYVY422, MKTAG('U', 'Y', 'N', 'Y') )
    ( AV_PIX_FMT_UYVY422, MKTAG('u', 'y', 'v', '1') )
    ( AV_PIX_FMT_UYVY422, MKTAG('2', 'V', 'u', '1') )
    ( AV_PIX_FMT_UYVY422, MKTAG('A', 'V', 'R', 'n') ) /* Avid AVI Codec 1:1 */
    ( AV_PIX_FMT_UYVY422, MKTAG('A', 'V', '1', 'x') ) /* Avid 1:1x */
    ( AV_PIX_FMT_UYVY422, MKTAG('A', 'V', 'u', 'p') )
    ( AV_PIX_FMT_UYVY422, MKTAG('V', 'D', 'T', 'Z') ) /* SoftLab-NSK VideoTizer */
    ( AV_PIX_FMT_UYVY422, MKTAG('a', 'u', 'v', '2') )
    ( AV_PIX_FMT_UYVY422, MKTAG('c', 'y', 'u', 'v') ) /* CYUV is also Creative YUV */
    ( AV_PIX_FMT_UYYVYY411, MKTAG('Y', '4', '1', '1') )
    ( AV_PIX_FMT_GRAY8, MKTAG('G', 'R', 'E', 'Y') )
    ( AV_PIX_FMT_NV12, MKTAG('N', 'V', '1', '2') )
    ( AV_PIX_FMT_NV21, MKTAG('N', 'V', '2', '1') )

    /* nut */
    ( AV_PIX_FMT_RGB555LE, MKTAG('R', 'G', 'B', 15) )
    ( AV_PIX_FMT_BGR555LE, MKTAG('B', 'G', 'R', 15) )
    ( AV_PIX_FMT_RGB565LE, MKTAG('R', 'G', 'B', 16) )
    ( AV_PIX_FMT_BGR565LE, MKTAG('B', 'G', 'R', 16) )
    ( AV_PIX_FMT_RGB555BE, MKTAG(15, 'B', 'G', 'R') )
    ( AV_PIX_FMT_BGR555BE, MKTAG(15, 'R', 'G', 'B') )
    ( AV_PIX_FMT_RGB565BE, MKTAG(16, 'B', 'G', 'R') )
    ( AV_PIX_FMT_BGR565BE, MKTAG(16, 'R', 'G', 'B') )
    ( AV_PIX_FMT_RGB444LE, MKTAG('R', 'G', 'B', 12) )
    ( AV_PIX_FMT_BGR444LE, MKTAG('B', 'G', 'R', 12) )
    ( AV_PIX_FMT_RGB444BE, MKTAG(12, 'B', 'G', 'R') )
    ( AV_PIX_FMT_BGR444BE, MKTAG(12, 'R', 'G', 'B') )
    ( AV_PIX_FMT_RGBA64LE, MKTAG('R', 'B', 'A', 64) )
    ( AV_PIX_FMT_BGRA64LE, MKTAG('B', 'R', 'A', 64) )
    ( AV_PIX_FMT_RGBA64BE, MKTAG(64, 'R', 'B', 'A') )
    ( AV_PIX_FMT_BGRA64BE, MKTAG(64, 'B', 'R', 'A') )
    ( AV_PIX_FMT_RGBA, MKTAG('R', 'G', 'B', 'A') )
    ( AV_PIX_FMT_RGB0, MKTAG('R', 'G', 'B', 0) )
    ( AV_PIX_FMT_BGRA, MKTAG('B', 'G', 'R', 'A') )
    ( AV_PIX_FMT_BGR0, MKTAG('B', 'G', 'R', 0) )
    ( AV_PIX_FMT_ABGR, MKTAG('A', 'B', 'G', 'R') )
    ( AV_PIX_FMT_0BGR, MKTAG(0, 'B', 'G', 'R') )
    ( AV_PIX_FMT_ARGB, MKTAG('A', 'R', 'G', 'B') )
    ( AV_PIX_FMT_0RGB, MKTAG(0, 'R', 'G', 'B') )
    ( AV_PIX_FMT_RGB24, MKTAG('R', 'G', 'B', 24) )
    ( AV_PIX_FMT_BGR24, MKTAG('B', 'G', 'R', 24) )
    ( AV_PIX_FMT_YUV411P, MKTAG('4', '1', '1', 'P') )
    ( AV_PIX_FMT_YUV422P, MKTAG('4', '2', '2', 'P') )
    ( AV_PIX_FMT_YUVJ422P, MKTAG('4', '2', '2', 'P') )
    ( AV_PIX_FMT_YUV440P, MKTAG('4', '4', '0', 'P') )
    ( AV_PIX_FMT_YUVJ440P, MKTAG('4', '4', '0', 'P') )
    ( AV_PIX_FMT_YUV444P, MKTAG('4', '4', '4', 'P') )
    ( AV_PIX_FMT_YUVJ444P, MKTAG('4', '4', '4', 'P') )
    ( AV_PIX_FMT_MONOWHITE, MKTAG('B', '1', 'W', '0') )
    ( AV_PIX_FMT_MONOBLACK, MKTAG('B', '0', 'W', '1') )
    ( AV_PIX_FMT_BGR8, MKTAG('B', 'G', 'R', 8) )
    ( AV_PIX_FMT_RGB8, MKTAG('R', 'G', 'B', 8) )
    ( AV_PIX_FMT_BGR4, MKTAG('B', 'G', 'R', 4) )
    ( AV_PIX_FMT_RGB4, MKTAG('R', 'G', 'B', 4) )
    ( AV_PIX_FMT_RGB4_BYTE, MKTAG('B', '4', 'B', 'Y') )
    ( AV_PIX_FMT_BGR4_BYTE, MKTAG('R', '4', 'B', 'Y') )
    ( AV_PIX_FMT_RGB48LE, MKTAG('R', 'G', 'B', 48) )
    ( AV_PIX_FMT_RGB48BE, MKTAG(48, 'R', 'G', 'B') )
    ( AV_PIX_FMT_BGR48LE, MKTAG('B', 'G', 'R', 48) )
    ( AV_PIX_FMT_BGR48BE, MKTAG(48, 'B', 'G', 'R') )
    ( AV_PIX_FMT_GRAY9LE, MKTAG('Y', '1', 0, 9) )
    ( AV_PIX_FMT_GRAY9BE, MKTAG(9, 0, '1', 'Y') )
    ( AV_PIX_FMT_GRAY10LE, MKTAG('Y', '1', 0, 10) )
    ( AV_PIX_FMT_GRAY10BE, MKTAG(10, 0, '1', 'Y') )
    ( AV_PIX_FMT_GRAY12LE, MKTAG('Y', '1', 0, 12) )
    ( AV_PIX_FMT_GRAY12BE, MKTAG(12, 0, '1', 'Y') )
    ( AV_PIX_FMT_GRAY14LE, MKTAG('Y', '1', 0, 14) )
    ( AV_PIX_FMT_GRAY14BE, MKTAG(14, 0, '1', 'Y') )
    ( AV_PIX_FMT_GRAY16LE, MKTAG('Y', '1', 0, 16) )
    ( AV_PIX_FMT_GRAY16BE, MKTAG(16, 0, '1', 'Y') )
    ( AV_PIX_FMT_YUV420P9LE, MKTAG('Y', '3', 11, 9) )
    ( AV_PIX_FMT_YUV420P9BE, MKTAG(9, 11, '3', 'Y') )
    ( AV_PIX_FMT_YUV422P9LE, MKTAG('Y', '3', 10, 9) )
    ( AV_PIX_FMT_YUV422P9BE, MKTAG(9, 10, '3', 'Y') )
    ( AV_PIX_FMT_YUV444P9LE, MKTAG('Y', '3', 0, 9) )
    ( AV_PIX_FMT_YUV444P9BE, MKTAG(9, 0, '3', 'Y') )
    ( AV_PIX_FMT_YUV420P10LE, MKTAG('Y', '3', 11, 10) )
    ( AV_PIX_FMT_YUV420P10BE, MKTAG(10, 11, '3', 'Y') )
    ( AV_PIX_FMT_YUV422P10LE, MKTAG('Y', '3', 10, 10) )
    ( AV_PIX_FMT_YUV422P10BE, MKTAG(10, 10, '3', 'Y') )
    ( AV_PIX_FMT_YUV444P10LE, MKTAG('Y', '3', 0, 10) )
    ( AV_PIX_FMT_YUV444P10BE, MKTAG(10, 0, '3', 'Y') )
    ( AV_PIX_FMT_YUV420P12LE, MKTAG('Y', '3', 11, 12) )
    ( AV_PIX_FMT_YUV420P12BE, MKTAG(12, 11, '3', 'Y') )
    ( AV_PIX_FMT_YUV422P12LE, MKTAG('Y', '3', 10, 12) )
    ( AV_PIX_FMT_YUV422P12BE, MKTAG(12, 10, '3', 'Y') )
    ( AV_PIX_FMT_YUV444P12LE, MKTAG('Y', '3', 0, 12) )
    ( AV_PIX_FMT_YUV444P12BE, MKTAG(12, 0, '3', 'Y') )
    ( AV_PIX_FMT_YUV420P14LE, MKTAG('Y', '3', 11, 14) )
    ( AV_PIX_FMT_YUV420P14BE, MKTAG(14, 11, '3', 'Y') )
    ( AV_PIX_FMT_YUV422P14LE, MKTAG('Y', '3', 10, 14) )
    ( AV_PIX_FMT_YUV422P14BE, MKTAG(14, 10, '3', 'Y') )
    ( AV_PIX_FMT_YUV444P14LE, MKTAG('Y', '3', 0, 14) )
    ( AV_PIX_FMT_YUV444P14BE, MKTAG(14, 0, '3', 'Y') )
    ( AV_PIX_FMT_YUV420P16LE, MKTAG('Y', '3', 11, 16) )
    ( AV_PIX_FMT_YUV420P16BE, MKTAG(16, 11, '3', 'Y') )
    ( AV_PIX_FMT_YUV422P16LE, MKTAG('Y', '3', 10, 16) )
    ( AV_PIX_FMT_YUV422P16BE, MKTAG(16, 10, '3', 'Y') )
    ( AV_PIX_FMT_YUV444P16LE, MKTAG('Y', '3', 0, 16) )
    ( AV_PIX_FMT_YUV444P16BE, MKTAG(16, 0, '3', 'Y') )
    ( AV_PIX_FMT_YUVA420P, MKTAG('Y', '4', 11, 8) )
    ( AV_PIX_FMT_YUVA422P, MKTAG('Y', '4', 10, 8) )
    ( AV_PIX_FMT_YUVA444P, MKTAG('Y', '4', 0, 8) )
    ( AV_PIX_FMT_YA8, MKTAG('Y', '2', 0, 8) )
    ( AV_PIX_FMT_PAL8, MKTAG('P', 'A', 'L', 8) )

    ( AV_PIX_FMT_YUVA420P9LE, MKTAG('Y', '4', 11, 9) )
    ( AV_PIX_FMT_YUVA420P9BE, MKTAG(9, 11, '4', 'Y') )
    ( AV_PIX_FMT_YUVA422P9LE, MKTAG('Y', '4', 10, 9) )
    ( AV_PIX_FMT_YUVA422P9BE, MKTAG(9, 10, '4', 'Y') )
    ( AV_PIX_FMT_YUVA444P9LE, MKTAG('Y', '4', 0, 9) )
    ( AV_PIX_FMT_YUVA444P9BE, MKTAG(9, 0, '4', 'Y') )
    ( AV_PIX_FMT_YUVA420P10LE, MKTAG('Y', '4', 11, 10) )
    ( AV_PIX_FMT_YUVA420P10BE, MKTAG(10, 11, '4', 'Y') )
    ( AV_PIX_FMT_YUVA422P10LE, MKTAG('Y', '4', 10, 10) )
    ( AV_PIX_FMT_YUVA422P10BE, MKTAG(10, 10, '4', 'Y') )
    ( AV_PIX_FMT_YUVA444P10LE, MKTAG('Y', '4', 0, 10) )
    ( AV_PIX_FMT_YUVA444P10BE, MKTAG(10, 0, '4', 'Y') )
    ( AV_PIX_FMT_YUVA422P12LE, MKTAG('Y', '4', 10, 12) )
    ( AV_PIX_FMT_YUVA422P12BE, MKTAG(12, 10, '4', 'Y') )
    ( AV_PIX_FMT_YUVA444P12LE, MKTAG('Y', '4', 0, 12) )
    ( AV_PIX_FMT_YUVA444P12BE, MKTAG(12, 0, '4', 'Y') )
    ( AV_PIX_FMT_YUVA420P16LE, MKTAG('Y', '4', 11, 16) )
    ( AV_PIX_FMT_YUVA420P16BE, MKTAG(16, 11, '4', 'Y') )
    ( AV_PIX_FMT_YUVA422P16LE, MKTAG('Y', '4', 10, 16) )
    ( AV_PIX_FMT_YUVA422P16BE, MKTAG(16, 10, '4', 'Y') )
    ( AV_PIX_FMT_YUVA444P16LE, MKTAG('Y', '4', 0, 16) )
    ( AV_PIX_FMT_YUVA444P16BE, MKTAG(16, 0, '4', 'Y') )

    ( AV_PIX_FMT_GBRP, MKTAG('G', '3', 00, 8) )
    ( AV_PIX_FMT_GBRP9LE, MKTAG('G', '3', 00, 9) )
    ( AV_PIX_FMT_GBRP9BE, MKTAG(9, 00, '3', 'G') )
    ( AV_PIX_FMT_GBRP10LE, MKTAG('G', '3', 00, 10) )
    ( AV_PIX_FMT_GBRP10BE, MKTAG(10, 00, '3', 'G') )
    ( AV_PIX_FMT_GBRP12LE, MKTAG('G', '3', 00, 12) )
    ( AV_PIX_FMT_GBRP12BE, MKTAG(12, 00, '3', 'G') )
    ( AV_PIX_FMT_GBRP14LE, MKTAG('G', '3', 00, 14) )
    ( AV_PIX_FMT_GBRP14BE, MKTAG(14, 00, '3', 'G') )
    ( AV_PIX_FMT_GBRP16LE, MKTAG('G', '3', 00, 16) )
    ( AV_PIX_FMT_GBRP16BE, MKTAG(16, 00, '3', 'G') )

    ( AV_PIX_FMT_GBRAP, MKTAG('G', '4', 00, 8) )
    ( AV_PIX_FMT_GBRAP10LE, MKTAG('G', '4', 00, 10) )
    ( AV_PIX_FMT_GBRAP10BE, MKTAG(10, 00, '4', 'G') )
    ( AV_PIX_FMT_GBRAP12LE, MKTAG('G', '4', 00, 12) )
    ( AV_PIX_FMT_GBRAP12BE, MKTAG(12, 00, '4', 'G') )
    ( AV_PIX_FMT_GBRAP16LE, MKTAG('G', '4', 00, 16) )
    ( AV_PIX_FMT_GBRAP16BE, MKTAG(16, 00, '4', 'G') )

    ( AV_PIX_FMT_XYZ12LE, MKTAG('X', 'Y', 'Z', 36) )
    ( AV_PIX_FMT_XYZ12BE, MKTAG(36, 'Z', 'Y', 'X') )

    ( AV_PIX_FMT_BAYER_BGGR8, MKTAG(0xBA, 'B', 'G', 8) )
    ( AV_PIX_FMT_BAYER_BGGR16LE, MKTAG(0xBA, 'B', 'G', 16) )
    ( AV_PIX_FMT_BAYER_BGGR16BE, MKTAG(16, 'G', 'B', 0xBA) )
    ( AV_PIX_FMT_BAYER_RGGB8, MKTAG(0xBA, 'R', 'G', 8) )
    ( AV_PIX_FMT_BAYER_RGGB16LE, MKTAG(0xBA, 'R', 'G', 16) )
    ( AV_PIX_FMT_BAYER_RGGB16BE, MKTAG(16, 'G', 'R', 0xBA) )
    ( AV_PIX_FMT_BAYER_GBRG8, MKTAG(0xBA, 'G', 'B', 8) )
    ( AV_PIX_FMT_BAYER_GBRG16LE, MKTAG(0xBA, 'G', 'B', 16) )
    ( AV_PIX_FMT_BAYER_GBRG16BE, MKTAG(16, 'B', 'G', 0xBA) )
    ( AV_PIX_FMT_BAYER_GRBG8, MKTAG(0xBA, 'G', 'R', 8) )
    ( AV_PIX_FMT_BAYER_GRBG16LE, MKTAG(0xBA, 'G', 'R', 16) )
    ( AV_PIX_FMT_BAYER_GRBG16BE, MKTAG(16, 'R', 'G', 0xBA) )

    /* quicktime */
    ( AV_PIX_FMT_YUV420P, MKTAG('R', '4', '2', '0') ) /* Radius DV YUV PAL */
    ( AV_PIX_FMT_YUV411P, MKTAG('R', '4', '1', '1') ) /* Radius DV YUV NTSC */
    ( AV_PIX_FMT_UYVY422, MKTAG('2', 'v', 'u', 'y') )
    ( AV_PIX_FMT_UYVY422, MKTAG('2', 'V', 'u', 'y') )
    ( AV_PIX_FMT_UYVY422, MKTAG('A', 'V', 'U', 'I') ) /* FIXME merge both fields */
    ( AV_PIX_FMT_UYVY422, MKTAG('b', 'x', 'y', 'v') )
    ( AV_PIX_FMT_YUYV422, MKTAG('y', 'u', 'v', '2') )
    ( AV_PIX_FMT_YUYV422, MKTAG('y', 'u', 'v', 's') )
    ( AV_PIX_FMT_YUYV422, MKTAG('D', 'V', 'O', 'O') ) /* Digital Voodoo SD 8 Bit */
    ( AV_PIX_FMT_RGB555LE, MKTAG('L', '5', '5', '5') )
    ( AV_PIX_FMT_RGB565LE, MKTAG('L', '5', '6', '5') )
    ( AV_PIX_FMT_RGB565BE, MKTAG('B', '5', '6', '5') )
    ( AV_PIX_FMT_BGR24, MKTAG('2', '4', 'B', 'G') )
    ( AV_PIX_FMT_BGR24, MKTAG('b', 'x', 'b', 'g') )
    ( AV_PIX_FMT_BGRA, MKTAG('B', 'G', 'R', 'A') )
    ( AV_PIX_FMT_RGBA, MKTAG('R', 'G', 'B', 'A') )
    ( AV_PIX_FMT_RGB24, MKTAG('b', 'x', 'r', 'g') )
    ( AV_PIX_FMT_ABGR, MKTAG('A', 'B', 'G', 'R') )
    ( AV_PIX_FMT_GRAY16BE, MKTAG('b', '1', '6', 'g') )
    ( AV_PIX_FMT_RGB48BE, MKTAG('b', '4', '8', 'r') )
    ( AV_PIX_FMT_RGBA64BE, MKTAG('b', '6', '4', 'a') )

    /* vlc */
    ( AV_PIX_FMT_YUV410P, MKTAG('I', '4', '1', '0') )
    ( AV_PIX_FMT_YUV411P, MKTAG('I', '4', '1', '1') )
    ( AV_PIX_FMT_YUV422P, MKTAG('I', '4', '2', '2') )
    ( AV_PIX_FMT_YUV440P, MKTAG('I', '4', '4', '0') )
    ( AV_PIX_FMT_YUV444P, MKTAG('I', '4', '4', '4') )
    ( AV_PIX_FMT_YUVJ420P, MKTAG('J', '4', '2', '0') )
    ( AV_PIX_FMT_YUVJ422P, MKTAG('J', '4', '2', '2') )
    ( AV_PIX_FMT_YUVJ440P, MKTAG('J', '4', '4', '0') )
    ( AV_PIX_FMT_YUVJ444P, MKTAG('J', '4', '4', '4') )
    ( AV_PIX_FMT_YUVA444P, MKTAG('Y', 'U', 'V', 'A') )
    ( AV_PIX_FMT_YUVA420P, MKTAG('I', '4', '0', 'A') )
    ( AV_PIX_FMT_YUVA422P, MKTAG('I', '4', '2', 'A') )
    ( AV_PIX_FMT_RGB8, MKTAG('R', 'G', 'B', '2') )
    ( AV_PIX_FMT_RGB555LE, MKTAG('R', 'V', '1', '5') )
    ( AV_PIX_FMT_RGB565LE, MKTAG('R', 'V', '1', '6') )
    ( AV_PIX_FMT_BGR24, MKTAG('R', 'V', '2', '4') )
    ( AV_PIX_FMT_BGR0, MKTAG('R', 'V', '3', '2') )
    ( AV_PIX_FMT_RGBA, MKTAG('A', 'V', '3', '2') )
    ( AV_PIX_FMT_YUV420P9LE, MKTAG('I', '0', '9', 'L') )
    ( AV_PIX_FMT_YUV420P9BE, MKTAG('I', '0', '9', 'B') )
    ( AV_PIX_FMT_YUV422P9LE, MKTAG('I', '2', '9', 'L') )
    ( AV_PIX_FMT_YUV422P9BE, MKTAG('I', '2', '9', 'B') )
    ( AV_PIX_FMT_YUV444P9LE, MKTAG('I', '4', '9', 'L') )
    ( AV_PIX_FMT_YUV444P9BE, MKTAG('I', '4', '9', 'B') )
    ( AV_PIX_FMT_YUV420P10LE, MKTAG('I', '0', 'A', 'L') )
    ( AV_PIX_FMT_YUV420P10BE, MKTAG('I', '0', 'A', 'B') )
    ( AV_PIX_FMT_YUV422P10LE, MKTAG('I', '2', 'A', 'L') )
    ( AV_PIX_FMT_YUV422P10BE, MKTAG('I', '2', 'A', 'B') )
    ( AV_PIX_FMT_YUV444P10LE, MKTAG('I', '4', 'A', 'L') )
    ( AV_PIX_FMT_YUV444P10BE, MKTAG('I', '4', 'A', 'B') )
    ( AV_PIX_FMT_YUV420P12LE, MKTAG('I', '0', 'C', 'L') )
    ( AV_PIX_FMT_YUV420P12BE, MKTAG('I', '0', 'C', 'B') )
    ( AV_PIX_FMT_YUV422P12LE, MKTAG('I', '2', 'C', 'L') )
    ( AV_PIX_FMT_YUV422P12BE, MKTAG('I', '2', 'C', 'B') )
    ( AV_PIX_FMT_YUV444P12LE, MKTAG('I', '4', 'C', 'L') )
    ( AV_PIX_FMT_YUV444P12BE, MKTAG('I', '4', 'C', 'B') )
    ( AV_PIX_FMT_YUV420P16LE, MKTAG('I', '0', 'F', 'L') )
    ( AV_PIX_FMT_YUV420P16BE, MKTAG('I', '0', 'F', 'B') )
    ( AV_PIX_FMT_YUV444P16LE, MKTAG('I', '4', 'F', 'L') )
    ( AV_PIX_FMT_YUV444P16BE, MKTAG('I', '4', 'F', 'B') )

    /* special */
    ( AV_PIX_FMT_RGB565LE, MKTAG(3, 0, 0, 0) ) /* flipped RGB565LE */
    ( AV_PIX_FMT_YUV444P, MKTAG('Y', 'V', '2', '4') ) /* YUV444P, swapped UV */

    ( AV_PIX_FMT_NONE, 0 );
// clang-format on
