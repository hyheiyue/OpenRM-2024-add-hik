#include <map>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <string>
#include <iostream>
#include "video/MvCameraControl.h"
#include "video/hik_camera.h"
#include "uniterm/uniterm.h"
#include "utils/timer.h"
#include "structure/stamp.hpp"
#include "structure/camera.hpp"
#include "video/video.h"

using namespace rm;
using namespace std;

static std::map<int, void*> camap; 
struct CallbackParam {
    Camera* camera;
    int bayer_type;
    float* yaw;
    float* pitch;
    float* roll;
    bool flip = false;
};

void __stdcall OnFrameCallbackFun(unsigned char *pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser) {
    TimePoint time_stamp = getTime();
    CallbackParam* callback_param = reinterpret_cast<CallbackParam*>(pUser);
    
    Camera *camera = callback_param->camera;
    shared_ptr<Frame> frame = make_shared<Frame>();
 
    cv::Mat bayer_img(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC1, pData);
    frame->image = make_shared<cv::Mat>(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3);

    if (callback_param->bayer_type == PixelType_Gvsp_BayerBG8) {
        cv::cvtColor(bayer_img, *frame->image, cv::COLOR_BayerBG2RGB);
    } else if (callback_param->bayer_type == PixelType_Gvsp_BayerGB8) {
        cv::cvtColor(bayer_img, *frame->image, cv::COLOR_BayerGB2RGB);
    } else if (callback_param->bayer_type == PixelType_Gvsp_BayerRG8) {
        cv::cvtColor(bayer_img, *frame->image, cv::COLOR_BayerRG2RGB);
    } else if (callback_param->bayer_type == PixelType_Gvsp_BayerGR8) {
        cv::cvtColor(bayer_img, *frame->image, cv::COLOR_BayerGR2RGB);
    }
    

    frame->time_point = time_stamp;
    frame->camera_id = camera->camera_id;
    frame->width = pFrameInfo->nWidth;
    frame->height = pFrameInfo->nHeight;
    
    camera->buffer->push(frame);
}
bool rm::getHikCameraNum(int& num) {
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
        rm::message("Video Hikvision get devices failed", rm::MSG_ERROR);
        return false;
    }
    num = stDeviceList.nDeviceNum;
    return true;
}
bool rm::openHik(
    Camera *camera,
    int device_num,
    float *yaw_ptr,
    float *pitch_ptr,
    float *roll_ptr,
    bool flip,
    double exposure,
    double gain,
    double fps
) {
    if(camera == nullptr) {
        rm::message("Video Hikvision nullptr camera", rm::MSG_ERROR);
        return false;
    }
    device_num--;

    void* handle;
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    
    if (device_num >= stDeviceList.nDeviceNum) {
        rm::message("Video Hikvision invalid device index", rm::MSG_ERROR);
        return false;
    }
    
    int nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[device_num]);
    if (MV_OK != nRet) {
        rm::message("Video Hikvision create handle failed", rm::MSG_ERROR);
        return false;
    }
    
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet) {
        rm::message("Video Hikvision open failed", rm::MSG_ERROR);
        return false;
    }
    camap[device_num] = handle;
    
    MV_CC_SetEnumValue(handle, "PixelFormat", PixelType_Gvsp_BayerBG8);

    MVCC_INTVALUE stParam;
    MV_CC_GetIntValue(handle, "Width", &stParam);
    camera->width = stParam.nCurValue;
    MV_CC_GetIntValue(handle, "Height", &stParam);
    camera->height = stParam.nCurValue;

    if(camera->buffer) delete camera->buffer;
    camera->buffer = new SwapBuffer<Frame>();
    camera->camera_id = device_num;
    rm::setHikArgs(camera, exposure, gain, fps);
    
    MV_CC_SetEnumValue(handle, "AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);


    MVCC_ENUMVALUE stBayer;
    MV_CC_GetEnumValue(handle, "PixelFormat", &stBayer);
    

    CallbackParam* param = new CallbackParam;
    param->camera = camera;
    param->bayer_type = stBayer.nCurValue;
    param->yaw = yaw_ptr;
    param->pitch = pitch_ptr;
    param->roll = roll_ptr;
    param->flip = flip;

    nRet = MV_CC_RegisterImageCallBackEx(handle, OnFrameCallbackFun, param);
    if (MV_OK != nRet) {
        rm::message("Video Hikvision set callback failed", rm::MSG_ERROR);
        return false;
    }

    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet) {
        rm::message("Video Hikvision start grab failed", rm::MSG_ERROR);
        return false;
    }

    return true;
}

bool rm::setHikArgs(Camera *camera, double exposure, double gain, double fps) {
    void* handle = camap[camera->camera_id];

    int nRet = MV_CC_SetEnumValue(handle, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    nRet |= MV_CC_SetFloatValue(handle, "ExposureTime", exposure);
    nRet |= MV_CC_SetEnumValue(handle, "GainAuto", MV_GAIN_MODE_OFF);
    nRet |= MV_CC_SetFloatValue(handle, "Gain", gain);
    nRet |= MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", true);
    nRet |= MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", fps);
    nRet |= MV_CC_SetFloatValue(handle, "Gamma", 0.7);

    if (MV_OK != nRet) {
        rm::message("Video Hikvision set params failed", rm::MSG_ERROR);
        return false;
    }
    return true;
}

bool rm::closeHik() {
    for(auto& cam : camap) {
        void* handle = cam.second;
        MV_CC_StopGrabbing(handle);
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
    }
    camap.clear();
    rm::message("Video Hikvision closed", rm::MSG_WARNING);
    return true;
}
