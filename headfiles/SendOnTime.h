#ifndef SERIALCOMMUNICATION_CPP_SENDONTIME_H
#define SERIALCOMMUNICATION_CPP_SENDONTIME_H

#include <cstdio>
#include <cstdlib>
#include <sys/time.h>
#include <unistd.h>
#include <iostream>
#include <time.h>
#include <memory>
#include <unistd.h>

#include "SerialCommunication.h"
#include "TrajectoryPlanning.h"
#include "Spline.h"


constexpr uint8_t BEGIN_BYTE = 0x7A;

/*
 * 设备类型枚举
 */
enum DeviceType : uint8_t {
    SERVER = 0,                                             // 服务端
    NUC,                                                    // NUC
    SLAVE,                                                  // 下位机
//    BUTTON,                                                 // 手柄按钮
    CLIENT,                                                  // 服务器
    DEVICE_TYPE_NUMBER,                                     // 设备类别数
};


/*
 * 数据帧类型枚举
 */
enum MessageType : uint8_t {
    NONE,                // 空帧
    MOVE,               //指定底盘移动
    POSE,               // 指定关节位姿
    ACTION,             //机器人位置信息
    LOCKED,             //锁死
    IMAGE,               //图像
    ARM,                 //机械臂
    MESSAGE_TYPE_NUMBER, // 消息类别数
};


/*
 * 数据包
 */
struct data_pack {
    uint8_t begin;              // 帧头字节：0x7A
    DeviceType device_type;     // 设备类型
    MessageType message_type;   // 帧消息类型
    uint32_t message_length;     // 帧消息长度
    uint8_t *message_content;   // 帧消息内容
    uint8_t lrc;                // LRC校验
} __attribute__((packed));

struct SendMessege{
    AfterSplinePoint final ;
};

long getCurrentTime();
void SendOnTime(int signal);
std::shared_ptr<uint8_t > Information(uint16_t ThetaOfOne, uint16_t ThetaOfTwo, uint16_t Z);
uint8_t *send_message_dynamic(MessageType msg_type, const uint8_t *msg_content,uint32_t length);

#endif //SERIALCOMMUNICATION_CPP_SENDONTIME_H