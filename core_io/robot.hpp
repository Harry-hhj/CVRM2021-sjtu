//
// Created by xinyang on 2021/3/7.
//

// Modified by Harry-hhj on 2021/05/04

#ifndef CVRM2021_ROBOT_HPP
#define CVRM2021_ROBOT_HPP

#include <cstdint>
#include <array>

enum class EnemyColor : uint8_t {
    RED = 0,
    BLUE = 1,
};

enum class ProgramMode : uint8_t {
    AUTO_AIM = 1,
    ANTIMISSLE = 2,
    SMALL_ENERGY = 4,
    BIG_ENERGY = 8,
};

// 低5位发射标志位，高3位状态标识位
enum class ShootMode : uint8_t {
    COMMON = 0,
    DISTANT = 1,
    ANTITOP = 2,
    SWITCH = 4,
    FOLLOW = 8,
    CRUISE = 16,
    EXIST_HERO = 32,
};

enum class GameState : uint8_t {
    SHOOT_NEAR_ONLY = 0,
    SHOOT_FAR = 1,
    COMMON = 255,
};

enum class Priority : uint8_t {
    CORE = 0,
    EMERGENCY = 1,
    NONE = 255,
};


// 旧通信协议
// 发送数据包
struct RobotStatus {
    EnemyColor enemy_color = EnemyColor::BLUE;
    float robot_speed_mps = 30.;
    uint16_t cruise_speed;
    uint16_t enemy0;
    uint16_t enemy1;
    uint16_t enemy2;
    uint16_t enemy3;
    uint16_t enemy4;
    uint16_t enemy5;
    GameState game_state = GameState::COMMON;
    uint8_t lrc = 0;
} __attribute__((packed));

// 新通信协议，分为长短包，长包通信频率 25 Hz，短包通信频率 10 Hz
// 接收数据包
// 40ms
struct ShortRobotStatus {
    EnemyColor enemy_color = EnemyColor::BLUE;
    uint8_t eject = 0;  // 0: not eject, 1+: ejecting, +1 every pkt
    uint8_t target_id = 255;   // 双板通信
    uint8_t lrc = 0;
} __attribute__((packed));

// 100ms
struct LongRobotStatus {
    ProgramMode program_mode = ProgramMode::AUTO_AIM;
    float robot_speed_mps = 28.;
    uint16_t cruise_speed;
    std::array<uint8_t, 6> enemy;       // 敌方哨兵0、英雄1、工程2、步兵3、步兵4、步兵5
                                        // int(真实血量 / 10)
    GameState game_state = GameState::COMMON;       // 是否设计远处
    uint8_t lrc = 0;
} __attribute__((packed));

// 发送数据包，新旧通信协议不变
struct RobotCmd {
    uint8_t start = (unsigned)'s';
    Priority priority;   // 双板通信
    uint8_t target_id = 255;   // 双板通信
    float pitch_angle = 0;
    float yaw_angle = 0;
    float pitch_speed = 0;
    float yaw_speed = 0;
    uint8_t distance = 0;  // 计算公式 (int)(distance * 10)
    uint8_t shoot_mode = static_cast<uint8_t>(ShootMode::CRUISE);
    uint8_t lrc = 0;
    uint8_t end = (unsigned)'e';
} __attribute__((packed));

#endif //CVRM2021_ROBOT_HPP
