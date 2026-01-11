#ifndef ONECHASSISNODE_HPP
#define ONECHASSISNODE_HPP
#include <memory>
#include <OF/lib/Node/Node.hpp>

#include "OneMotor/Can/CanDriver.hpp"

#include <one/PID/PidParams.hpp>
#include <one/PID/PidConfig.hpp>
#include <one/PID/PidChain.hpp>

#include <OF/lib/algo/Mecanum.hpp>

#include <OneMotor/Motor/DJI/DjiMotor.hpp>

#include "OF/lib/ControllerHub/ControllerHub.hpp"

using namespace OF;

using one::pid::PidParams;
using one::pid::PidConfig;
using one::pid::PidChain;
using OneMotor::Motor::DJI::M3508;
using OneMotor::Motor::DJI::makeM3508;
using OneMotor::Can::CanDriver;
using OneMotor::Motor::DJI::PIDFeatures;


using enum ControllerHub::Channel;

static constexpr PidParams<> ANG_DEFAULT_PARAMS{
    .Kp = 0.4,
    .Ki = 0.01,
    .Kd = 0.01,
    .MaxOutput = 13000,
    .Deadband = 100,
    .IntegralLimit = 2000,
};

static constexpr PidConfig<one::pid::Positional, float, PIDFeatures> g_pid_conf = {ANG_DEFAULT_PARAMS};

static PidChain g_chain(g_pid_conf);

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
        static constexpr size_t stack_size = 8192;
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
    std::unique_ptr<CanDriver> m_driver;
    std::unique_ptr<M3508<2, decltype(g_chain)>> m_fl;
    std::unique_ptr<M3508<1, decltype(g_chain)>> m_fr;
    std::unique_ptr<M3508<3, decltype(g_chain)>> m_bl;
    std::unique_ptr<M3508<4, decltype(g_chain)>> m_br;
};


#endif //ONECHASSISNODE_HPP
