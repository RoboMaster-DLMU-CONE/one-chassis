#include <OneChassisData.hpp>
#include <OneChassisNode.hpp>
#include <OneMotor/Motor/DJI/DjiMotor.hpp>
#include <one/PID/PidParams.hpp>
#include <one/PID/PidConfig.hpp>
#include <one/PID/PidChain.hpp>
#include <OF/lib/ControllerHub/ControllerHub.hpp>
#include <OF/lib/algo/Mecanum.hpp>

#include "OF/lib/HubManager/HubManager.hpp"


/* TODO:
 * 1. 目前的电机基类不够优雅，CRTP导致必须要有模板参数，基本无法创建一个通用的类。
 * 或许应该添加一个wrapper Class或者去掉CRTP改称虚函数集成
 * 又或许给bind添加template头
 *
 * 2. Node的bind方法需要使用头文件暴露出Node类的Config, 进行配置可能会不太方便
 * 当然其实还好，改一下node的template，默认给一个hpp，在CMake里library_include_directories就行。
 *
 * 3. 各个Hub之间的方法不统一，早期开发的Hub只能获取单例指针来调用类方法，NotifyHub可以直接调用全局函数
 * 我觉得全局函数是对的，应该为ControllerHub和ImuHub增加方便的全局函数调用。
 *
 * 4. ControllerHub应该加入某种虚拟化机制，把不同的遥控器值映射到某个区间，并提供float百分比之类的转换函数
 * 也许后者就够了？
 */

using one::pid::PidParams;
using one::pid::PidConfig;
using one::pid::PidChain;
using OneMotor::Motor::DJI::M3508;
using OneMotor::Motor::DJI::makeM3508;
using OneMotor::Can::CanDriver;
using OneMotor::Motor::DJI::PIDFeatures;

using enum ControllerHub::Channel;

static constexpr PidParams<> ANG_DEFAULT_PARAMS{
    .Kp = 0.8,
    .Ki = 0.05,
    .Kd = 0.1,
    .MaxOutput = 8000,
    .Deadband = 10,
    .IntegralLimit = 100,
};

static constexpr PidConfig<one::pid::Positional, float, PIDFeatures> g_pid_conf = {ANG_DEFAULT_PARAMS};

static PidChain g_chain(g_pid_conf);

static constexpr OF::Algo::Mecanum::Config g_m_conf{
    1 * m,
    2 * m,
    3 * m
};

static constexpr OF::Algo::Mecanum::Solver g_solver(g_m_conf);


std::unique_ptr<M3508<1, decltype(g_chain)>> m_fl;
std::unique_ptr<M3508<2, decltype(g_chain)>> m_fr;
std::unique_ptr<M3508<3, decltype(g_chain)>> m_bl;
std::unique_ptr<M3508<4, decltype(g_chain)>> m_br;

// Register Topic for data communication
ONE_TOPIC_REGISTER(OneChassisData, topic_one_chassis, "one_chassis_data");


bool OneChassisNode::init()
{
    auto can_driver = CanDriver(config.can_dev);
    m_fl = std::make_unique<M3508<1, decltype(g_chain)>>(can_driver, g_chain);
    m_fr = std::make_unique<M3508<2, decltype(g_chain)>>(can_driver, g_chain);
    m_bl = std::make_unique<M3508<3, decltype(g_chain)>>(can_driver, g_chain);
    m_br = std::make_unique<M3508<4, decltype(g_chain)>>(can_driver, g_chain);
    return true;
}

void OneChassisNode::run()
{
    const auto* hub = getHub<ControllerHub>();
    while (true)
    {
        auto state = hub->getData();
        const auto leftX = state[LEFT_X];
        const auto leftY = state[LEFT_Y];
        const auto rightX = state[RIGHT_X];
        const auto rightY = state[RIGHT_Y];
        const auto swL = state[SW_L];
        const auto swR = state[SW_R];

        auto [fl_v, fr_v, bl_v, br_v] = g_solver.inverse({});

        (void)m_fl->setAngRef(fl_v);
        (void)m_fr->setAngRef(fr_v);
        (void)m_bl->setAngRef(bl_v);
        (void)m_br->setAngRef(br_v);
    }
}

// Register the Node with OneFramework
ONE_NODE_REGISTER(OneChassisNode);
