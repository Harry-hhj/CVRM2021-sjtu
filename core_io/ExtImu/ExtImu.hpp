//
// Created by xinyang on 2021/3/3.
//

#ifndef _EXTIMU_HPP_
#define _EXTIMU_HPP_

#include <serial/serial.h>

class ExtImu : private serial::Serial {
    using Serial = serial::Serial;
public:
    using Serial::Serial;
    using Serial::setPort;
    using Serial::getPort;
    using Serial::setTimeout;
    using Serial::getTimeout;
    using Serial::setBaudrate;
    using Serial::getBaudrate;
    using Serial::setBytesize;
    using Serial::getBytesize;
    using Serial::setParity;
    using Serial::getParity;
    using Serial::setStopbits;
    using Serial::getStopbits;
    using Serial::setFlowcontrol;
    using Serial::getFlowcontrol;

    using Serial::open;
    using Serial::close;
    using Serial::isOpen;

    using Serial::flushInput;
    using Serial::flushOutput;

    struct sensor_data {
        float acc[3];       // 三轴加速度[单位:g]
        float gyro[3];      // 三轴角速度[单位:dps]
        float q[4];         // 四元数姿态
        float e[3];         // 欧拉角姿态[单位:degree]
        float timestamp_ms;
        uint8_t trigger;    // 0:常规帧, 1:触发帧
    } __attribute__((packed));

    void stop_trigger();

    void once_trigger(int32_t delay_ms);

    void periodic_trigger(int32_t period_ms);

    void read_sensor(sensor_data *p_data);
};

#endif /* _EXTIMU_HPP_ */
