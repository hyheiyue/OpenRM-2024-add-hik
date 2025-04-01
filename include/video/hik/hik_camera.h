#ifndef HIK_CAMERA_H
#define HIK_CAMERA_H

#include <map>
#include <memory>
#include <opencv2/core.hpp>
#include "MvCameraControl.h"
#include "uniterm/uniterm.h"
#include "utils/timer.h"
#include "structure/camera.hpp"
#include "video/video.h"

namespace rm {

// 枚举 Hik 相机数量，返回设备个数
bool getHikCameraNum(int &num);

// 设置 Hik 相机参数（曝光、增益、帧率）
// 传入的曝光、增益、帧率会先根据相机支持的范围进行 clamp 后再设置
bool setHikArgs(Camera* camera, double exposure, double gain, double fps);

// 打开 Hik 相机
// device_num: 设备编号
// yaw_ptr, pitch_ptr, roll_ptr: 姿态数据指针（如无可传入 nullptr）
// flip: 是否翻转图像
// exposure, gain, fps: 分别对应曝光时间、增益及帧率
bool getHikCameraNum(int &num);

bool setHikArgs(Camera *camera, double exposure, double gain, double fps);
bool openHik(
    Camera* camera,
    int device_num,
    float* yaw_ptr,
    float* pitch_ptr,
    float* roll_ptr,
    bool flip,
    double exposure,
    double gain,
    double fps
);

// 关闭所有已打开的 Hik 相机
bool closeHik();

}  // namespace rm

#endif  // HIK_CAMERA_H
