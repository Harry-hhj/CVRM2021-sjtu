//
// Created by xinyang on 2021/3/7.
//

// Modified by Harry-hhj on 2021/05/04

#include "robot.hpp"
#include <serial/serial.h>
#include <fmt/format.h>
#include <fmt/color.h>
#include <umt/umt.hpp>
#include <chrono>
#include <thread>
#include <bitset>
#include <iomanip>

using namespace serial;
using namespace std::chrono;

static bool debug = true;

void robot_cmd_loop(Serial &robot_port, const bool &required_stop, bool &is_ok) {
    if (debug) std::cout << "============robot_cmd_loop===========\n";
    umt::Subscriber<RobotCmd> robot_cmd_sub("robot_cmd", 0);
    while (!required_stop) {
        try {
            auto robot_cmd = robot_cmd_sub.pop();
            for(auto *ptr=(uint8_t*)&robot_cmd.priority; ptr < &robot_cmd.lrc; ptr++){
                robot_cmd.lrc += *ptr;
            }
            try {
                robot_port.write((uint8_t *) &robot_cmd, sizeof(RobotCmd));
            } catch (SerialException &e) {
                fmt::print(fmt::fg(fmt::color::red), "[ERROR] {}\n", e.what());
                is_ok = false;
                break;
            }
        } catch (umt::MessageError &e) {
            std::this_thread::sleep_for(100ms);
            std::cout << "robot_cmd_loop pop error" << std::endl;
	    }
    }
}

// manifold-2g
bool robot_io_4pin(const std::string &robot_port_name = "") {
    if (debug) std::cout << "============robot_io_4pin===========\n";
    std::cout << "robot_port_name" << robot_port_name << std::endl;
    Serial robot_port(robot_port_name, 115200);
    if (!robot_port.isOpen()) {
        fmt::print(fmt::fg(fmt::color::red), "[ERROR]: robot serial init fail!\n");
        return false;
    }

    bool robot_cmd_required_stop = false;
    bool robot_cmd_is_ok = true;
    std::thread robot_cmd_thread(robot_cmd_loop, std::ref(robot_port),
                                 std::ref(robot_cmd_required_stop), std::ref(robot_cmd_is_ok));

    auto robot_status_short = umt::ObjManager<RobotStatus>::find_or_create("robot_status_short");

    while (robot_cmd_is_ok) {
	RobotStatus scurr;
        try {
            uint8_t start;
            robot_port.read(&start, 1);
//            std::cout << "sizeof(RobotStatus) = " << sizeof(RobotStatus) << std::endl;
//            std::cout << "start:" << std::hex << unsigned(start) << std::endl;
            while (unsigned(start) != unsigned('s')) robot_port.read(&start, 1);
            robot_port.read((uint8_t *) &scurr, sizeof(RobotStatus));
            uint8_t lrc = 0;
            //std::cout << "size:" << sizeof(RobotStatus) << std::endl;
            for(auto *ptr = ((uint8_t*)&(scurr.enemy_color)); ptr < ((uint8_t*)&(scurr.lrc)); ptr++){
                lrc += *ptr;
//                std::cout << " *ptr:" << std::hex << unsigned(*ptr) << " ptr:" << (void*)(ptr) << std::endl;
            }
            uint8_t end = 0;
            robot_port.read(&end, 1);
	    if (lrc == scurr.lrc && end == unsigned('e'))memcpy((uint8_t *)robot_status_short.get(),(uint8_t *)&scurr,sizeof(scurr));
	    else while (unsigned(end) != unsigned('e'))robot_port.read(&end,1);
//            std::cout << "end:" << std::hex << unsigned(end) << std::endl;
//            std::cout << "lrc: " << unsigned(lrc) << " robot_cmd_lrc: " << unsigned(robot_status_short->lrc) << std::endl;
        } catch (SerialException &e) {
            fmt::print(fmt::fg(fmt::color::red), "[ERROR] {}\n", e.what());
            break;
        }
    }

    robot_cmd_required_stop = true;
    robot_cmd_thread.join();

    return false;
}

void background_robot_io_4pin_auto_restart(const std::string &robot_port_name = "") {
    if (debug) std::cout << "============background_robot_io_4pin_auto_restart===========\n";
    std::thread([=]() {
        while (!robot_io_4pin(robot_port_name)) {
            std::this_thread::sleep_for(500ms);
        }
    }).detach();
}


// Xavier nx
bool robot_io_usb(const std::string &robot_usb_hid = "") {
    if (debug) std::cout << "============robot_io_usb===========\n";
    Serial robot_port;
    std::cout << "finding serial port" << std::endl;
    for (const auto &port_info : list_ports()) {
        std::cout << port_info.port << "|" << port_info.hardware_id << std::endl;
        if (port_info.hardware_id == robot_usb_hid) {
            robot_port.setPort(port_info.port);
            robot_port.setBaudrate(115200);
            auto timeout = Timeout::simpleTimeout(Timeout::max());
            robot_port.setTimeout(timeout);
            break;
        }
    }


    robot_port.open();
    if (!robot_port.isOpen()) {
        fmt::print(fmt::fg(fmt::color::red), "[ERROR] (robot_io_usb): robot serial init fail!\n");
        return false;
    }
    std::cout << "find serial port!" << std::endl;

    bool robot_cmd_required_stop = false;
    bool robot_cmd_is_ok = true;
    std::thread robot_cmd_thread(robot_cmd_loop, std::ref(robot_port),
                                 std::ref(robot_cmd_required_stop), std::ref(robot_cmd_is_ok));

    auto robot_status_short = umt::ObjManager<ShortRobotStatus>::find_or_create("robot_status_short");
    auto robot_status_long = umt::ObjManager<LongRobotStatus>::find_or_create("robot_status_long");

    while (robot_cmd_is_ok) {
        ShortRobotStatus scurr;
        LongRobotStatus hcurr;
        try {
            uint8_t start;
            robot_port.read(&start, 1);
//            std::cout << "sizeof(RobotStatus) = " << sizeof(RobotStatus) << std::endl;
//            std::cout << "start:" << std::hex << unsigned(start) << std::endl;
            while (unsigned(start) != unsigned('s') && unsigned(start) != unsigned('h')) robot_port.read(&start, 1);
            if (unsigned(start) == unsigned('s')) {
                robot_port.read((uint8_t *) &scurr, sizeof(ShortRobotStatus));
                uint8_t lrc = 0;
                for(auto *ptr = ((uint8_t*)&(scurr.enemy_color)); ptr < ((uint8_t*)&(scurr.lrc)); ptr++) lrc += *ptr;  //TODO:sizeof
                uint8_t end=0;
                robot_port.read(&end, 1);
	            if (lrc == scurr.lrc && end == unsigned('e')) memcpy((uint8_t *)robot_status_short.get(),(uint8_t *)&scurr,sizeof(scurr));
	            else while (unsigned(end) != unsigned('e')) robot_port.read(&end,1);
            }
            else if (unsigned(start) == unsigned('h')) {
                robot_port.read((uint8_t *) &hcurr, sizeof(LongRobotStatus));
                uint8_t lrc = 0;
                for(auto *ptr = ((uint8_t*)&(hcurr.program_mode)); ptr < ((uint8_t*)&(hcurr.lrc)); ptr++) lrc += *ptr;  //TODO:sizeof
                uint8_t end=0;
                robot_port.read(&end, 1);
	            if (lrc == hcurr.lrc && end == unsigned('e')) memcpy((uint8_t *)robot_status_long.get(),(uint8_t *)&hcurr,sizeof(hcurr));
	            else while (unsigned(end) != unsigned('e')) robot_port.read(&end,1);
            }
        } catch (SerialException &e) {
            fmt::print(fmt::fg(fmt::color::red), "[ERROR] (robot_io_usb) {}\n", e.what());
            break;
        }
    }

    robot_cmd_required_stop = true;
    robot_cmd_thread.join();

    return false;
}

void background_robot_io_usb_auto_restart(const std::string &robot_usb_hid = "") {
    if (debug) std::cout << "============background_robot_io_usb_auto_restart===========\n";
    std::thread([=]() {
        while (!robot_io_usb(robot_usb_hid)) {
            std::this_thread::sleep_for(500ms);
        }
    }).detach();
}

UMT_EXPORT_OBJMANAGER_ALIAS(ShortRobotStatus, ShortRobotStatus, c) {
    c.def_readwrite("enemy_color", &ShortRobotStatus::enemy_color);
    c.def_readwrite("eject", &ShortRobotStatus::eject);
    c.def_readwrite("target_id", &ShortRobotStatus::target_id);
}

UMT_EXPORT_OBJMANAGER_ALIAS(LongRobotStatus, LongRobotStatus, c) {
    c.def_readwrite("program_mode", &LongRobotStatus::program_mode);
    c.def_readwrite("robot_speed_mps", &LongRobotStatus::robot_speed_mps);
    c.def_readwrite("cruise_speed", &LongRobotStatus::cruise_speed);
    c.def_readwrite("enemy", &LongRobotStatus::enemy);
    c.def_readwrite("game_state", &LongRobotStatus::game_state);
}

UMT_EXPORT_MESSAGE_ALIAS(RobotCmd, RobotCmd, c) {
    c.def_readwrite("priority", &RobotCmd::priority);
    c.def_readwrite("target_id", &RobotCmd::target_id);
    c.def_readwrite("pitch_angle", &RobotCmd::pitch_angle);
    c.def_readwrite("yaw_angle", &RobotCmd::yaw_angle);
    c.def_readwrite("pitch_speed", &RobotCmd::pitch_speed);
    c.def_readwrite("yaw_speed", &RobotCmd::yaw_speed);
    c.def_readwrite("shoot_mode", &RobotCmd::shoot_mode);
}

PYBIND11_EMBEDDED_MODULE(RobotIO, m) {
    namespace py = pybind11;
    m.def("background_robot_io_4pin_auto_restart", background_robot_io_4pin_auto_restart,
          py::arg("robot_port_name") = "");
    m.def("background_robot_io_usb_auto_restart", background_robot_io_usb_auto_restart,
          py::arg("robot_usb_hid") = "");
    py::enum_<EnemyColor>(m ,"EnemyColor")
            .value("RED", EnemyColor::RED)
            .value("BLUE", EnemyColor::BLUE)
            ;
    py::enum_<ProgramMode>(m, "ProgramMode")
            .value("AUTO_AIM", ProgramMode::AUTO_AIM)
            .value("SMALL_ENERGY", ProgramMode::SMALL_ENERGY)
            .value("BIG_ENERGY", ProgramMode::BIG_ENERGY)
            ;
    py::enum_<ShootMode>(m, "ShootMode")
            .value("COMMON", ShootMode::COMMON)
            .value("DISTANT", ShootMode::DISTANT)
            .value("ANTITOP", ShootMode::ANTITOP)
            .value("SWITCH", ShootMode::SWITCH)
            .value("FOLLOW", ShootMode::FOLLOW)
            .value("CRUISE", ShootMode::CRUISE)
            ;
    py::enum_<GameState>(m, "GameState")
            .value("SHOOT_NEAR_ONLY", GameState::SHOOT_NEAR_ONLY)
            .value("SHOOT_FAR", GameState::SHOOT_FAR)
            .value("COMMON", GameState::COMMON)
            ;
    py::enum_<Priority>(m, "Priority")
            .value("CORE", Priority::CORE)
            .value("EMERGENCY", Priority::EMERGENCY)
            .value("NONE", Priority::NONE)
            ;
}
