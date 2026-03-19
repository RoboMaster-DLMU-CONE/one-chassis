#ifndef ONECHASSISNODE_HPP
#define ONECHASSISNODE_HPP
#include <memory>
#include <OF/lib/Node/Node.hpp>

#include <one/can/CanDriver.hpp>

#include <one/PID/PidParams.hpp>
#include <one/PID/PidConfig.hpp>
#include <one/PID/PidChain.hpp>

#include <OF/lib/algo/Mecanum.hpp>

#include <one/motor/dji/DjiMotor.hpp>
#include <OF/lib/VtHub/VtHub.hpp>
#include <OF/lib/NotifyHub/NotifyGuard.hpp>

using namespace OF;

using one::pid::PidParams;
using one::pid::PidConfig;
using one::pid::PidController;
using one::pid::PidChain;
using one::motor::dji::M3508;
using one::can::CanDriver;


static constexpr PidParams<> g_ang_params{
    .Kp = 0.6,
    .Ki = 0.001,
    .Kd = 0.11,
    .MaxOutput = 14000,
    .Deadband = 100,
    .IntegralLimit = 1000,
};

using namespace Units::literals;
static constexpr Algo::Mecanum::Config g_m_conf{
    154.5 / 2 * mm,
    386.2 * mm,
    384.12f * mm,
};

static constexpr Algo::Mecanum::Solver g_solver(g_m_conf);

class OneChassisNode : public Node<OneChassisNode>
{
public:
    struct Meta
    {
        static constexpr size_t stack_size = 4096;
        static constexpr int priority = 1;
        static constexpr auto name = "one_chassis";
    };

    struct Config
    {
        const device* can_dev;
    };

    inline static Config config = {};

    bool init();

    void run();

    void cleanup()
    {
        // TODO: Clean up resources
    }

private:
    CanDriver m_driver;
    // 2, 1, 3, 4
    M3508 m_fl;
    M3508 m_fr;
    M3508 m_bl;
    M3508 m_br;
    float m_target_yaw = 0.0f; // 期望的目标角度
    bool m_is_yaw_initialized = false; // 初始化标志
    NotifyGuard<LEDStatus> m_led_guard{"chassis"};
};


#endif //ONECHASSISNODE_HPP
