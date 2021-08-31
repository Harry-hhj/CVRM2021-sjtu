//
// Created by xinyang on 2021/3/6.
//

#include "MindVision.hpp"
#include <fmt/format.h>
#include <fmt/color.h>

#define MV_ASSERT_WARNING(expr, info, ...) do{        \
    if((expr) == false){                              \
        fmt::print(fg(fmt::color::orange),            \
                   "[WARNING] " #expr info "\n", ##__VA_ARGS__); \
    }                                                 \
}while(0)

#define MV_ASSERT_ERROR(expr, info, ...) do{          \
    if((expr) == false){                              \
        fmt::print(fg(fmt::color::red),               \
                   "[ERROR] " #expr info "\n", ##__VA_ARGS__);   \
        return false;                                 \
    }                                                 \
}while(0)

#define MV_ASSERT_THROW(expr, info, ...) do{                               \
    if((expr) == false){                                                   \
        throw MindVision_FrameError(                                       \
            fmt::format("'"#expr"' = ({}) " info, status, ##__VA_ARGS__)); \
    }                                                                      \
}while(0)

#define MV_CHECK_API_WARNING(expr, info, ...) do{                              \
    auto status = (expr);                                                      \
    if(status != CAMERA_STATUS_SUCCESS){                                       \
        fmt::print(fg(fmt::color::orange),                                     \
                   "[WARNING] '"#expr"' = ({}) " info, status, ##__VA_ARGS__); \
    }                                                                          \
}while(0)

#define MV_CHECK_API_ERROR(expr, info, ...)   do{                              \
    auto status = (expr);                                                      \
    if(status != CAMERA_STATUS_SUCCESS){                                       \
        fmt::print(fg(fmt::color::red),                                        \
                   "[ERROR] '"#expr"' = ({}) " info "\n", status, ##__VA_ARGS__);   \
        return false;                                                          \
    }                                                                          \
}while(0)

#define MV_CHECK_API_THROW(expr, info, ...)   do{                              \
    auto status = (expr);                                                      \
    if(status != CAMERA_STATUS_SUCCESS){                                       \
        throw MindVision_FrameError(                                           \
            fmt::format("'"#expr"' = ({}) " info, status, ##__VA_ARGS__));     \
    }                                                                          \
}while(0)

void capture_callback(CameraHandle hCamera, BYTE *pFrameBuffer, tSdkFrameHead *pFrameHead, PVOID pContext);

MindVision::MindVision(const char *camera_name, const char *camera_cfg)
        : camera_name(camera_name), camera_cfg(camera_cfg), handle(-1) {
    MV_CHECK_API_WARNING(CameraSdkInit(1), "");
}

MindVision::~MindVision() {
    close();
}

bool MindVision::open() {
    if (isOpen()) {
        if (!close()) return false;
    }

    tSdkCameraDevInfo infos[2];
    int dev_num = 2;
    MV_CHECK_API_ERROR(CameraEnumerateDevice(infos, &dev_num), "");
    MV_ASSERT_ERROR(dev_num > 0, "no device found!");

    MV_ASSERT_WARNING(!camera_name.empty(), "camera name is not specified. no name check!");
    for (auto &info : infos) {
        if (info.acFriendlyName == camera_name) {
            MV_CHECK_API_ERROR(CameraInit(&info, -1, -1, &handle), "");
            break;
        }
    }
    MV_ASSERT_ERROR(handle >= 0, "camera '{}' not found!", camera_name);

    MV_CHECK_API_WARNING(CameraReadParameterFromFile(handle, (char *) camera_cfg.data()),
                         "config file '{}' read error!", camera_cfg);
    MV_CHECK_API_ERROR(CameraSetTriggerMode(handle, 2), "");

    MV_CHECK_API_ERROR(CameraPlay(handle), "");

    return true;
}

bool MindVision::close() {
    MV_ASSERT_WARNING(handle >= 0, "camera already closed.");
    if (handle < 0) return true;
    MV_CHECK_API_ERROR(CameraUnInit(handle), "");
    handle = -1;
    return true;
}

bool MindVision::isOpen() const {
    return handle >= 0;
}

bool MindVision::read(cv::Mat &img) const {
    MV_ASSERT_ERROR(isOpen(), "camera not open.");
    tSdkFrameHead head;
    BYTE *buffer;
    MV_CHECK_API_ERROR(CameraGetImageBuffer(handle, &head, &buffer, 100), "");
    img = cv::Mat(head.iHeight, head.iWidth, CV_8UC3);
    MV_CHECK_API_ERROR(CameraImageProcess(handle, buffer, img.data, &head), "");
    MV_CHECK_API_ERROR(CameraReleaseImageBuffer(handle, buffer), "");
    return true;
}

bool MindVision::read(cv::Mat &img, double &timestamp_ms) const {
    MV_ASSERT_ERROR(isOpen(), "camera not open.");
    tSdkFrameHead head;
    BYTE *buffer;
    MV_CHECK_API_ERROR(CameraGetImageBuffer(handle, &head, &buffer, 100), "");
    img = cv::Mat(head.iHeight, head.iWidth, CV_8UC3);
    MV_CHECK_API_ERROR(CameraImageProcess(handle, buffer, img.data, &head), "");
    timestamp_ms = head.uiTimeStamp / 10.;
    MV_CHECK_API_ERROR(CameraReleaseImageBuffer(handle, buffer), "");
    return true;
}

bool MindVision::get_exposure_us(double &us) const {
    MV_ASSERT_ERROR(isOpen(), "camera not open.");
    MV_CHECK_API_ERROR(CameraGetExposureTime(handle, &us), "");
    return true;
}

bool MindVision::set_exposure_us(double us) const {
    MV_ASSERT_ERROR(isOpen(), "camera not open.");
    MV_CHECK_API_ERROR(CameraSetExposureTime(handle, us), "");
    return true;
}
