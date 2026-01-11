#include <OneChassisData.hpp>
#include <OneChassisNode.hpp>


#include "OF/lib/HubManager/HubManager.hpp"


/* TODO:
 *   1. 目前的电机基类不够优雅，CRTP导致必须要有模板参数，基本无法创建一个通用的类。
 *   或许应该添加一个wrapper Class或者去掉CRTP改称虚函数集成
 *   又或许给bind添加template头
 *   2. Node的bind方法需要使用头文件暴露出Node类的Config, 进行配置可能会不太方便
 *   当然其实还好，改一下node的template，默认给一个hpp，在CMake里library_include_directories就行。
 *   3. 各个Hub之间的方法不统一，早期开发的Hub只能获取单例指针来调用类方法，NotifyHub可以直接调用全局函数
 *   我觉得全局函数是对的，应该为ControllerHub和ImuHub增加方便的全局函数调用。
 *   4. ControllerHub应该加入某种虚拟化机制，把不同的遥控器值映射到某个区间，并提供float百分比之类的转换函数
 *   也许后者就够了？ （已解决）
 *   5. OneMotor Zephyr 的CanDriver ，需要比SharedPtr更优雅的方案
 *   6. 需要在模板内引入一点k_sleep，不然的话容易出现用户Node优先级太高导致日志无法输出的问题。
 *
 */

LOG_MODULE_REGISTER(OneChassisNode, CONFIG_ONE_CHASSIS_LOG_LEVEL);


// Register Topic for data communication
ONE_TOPIC_REGISTER(OneChassisData, topic_one_chassis, "one_chassis_data");


bool OneChassisNode::init()
{
    LOG_INF("init chassis");
    if (config.can_dev == nullptr) return false;
    m_driver = std::make_unique<CanDriver>(config.can_dev);
    m_fl = std::make_unique<M3508<2, decltype(g_chain)>>(*m_driver, g_chain);
    m_fr = std::make_unique<M3508<1, decltype(g_chain)>>(*m_driver, g_chain);
    m_bl = std::make_unique<M3508<3, decltype(g_chain)>>(*m_driver, g_chain);
    m_br = std::make_unique<M3508<4, decltype(g_chain)>>(*m_driver, g_chain);
    (void)m_fl->setAngRef(0 * rad / s);
    (void)m_fr->setAngRef(0 * rad / s);
    (void)m_bl->setAngRef(0 * rad / s);
    (void)m_br->setAngRef(0 * rad / s);
    (void)m_fl->enable();
    (void)m_fr->enable();
    (void)m_bl->enable();
    (void)m_br->enable();
    return true;
}

void OneChassisNode::run()
{
    LOG_INF("running chassis");
    while (true)
    {
        k_sleep(K_MSEC(10));

        auto state = getControllerData();
        if (!state || state.value()[SW_L] == 1)
        {
            LOG_INF("Disconnected or Emergency...");
            (void)m_fl->setAngRef(0 * rad / s);
            (void)m_fr->setAngRef(0 * rad / s);
            (void)m_bl->setAngRef(0 * rad / s);
            (void)m_br->setAngRef(0 * rad / s);
            k_sleep(K_MSEC(500));
            continue;
        }
        auto data = state.value();
        // const auto swR = data[SW_R];
        const auto leftY = data.percent(LEFT_Y); // 前后
        const auto leftX = data.percent(LEFT_X); // 左右
        const auto rightX = data.percent(RIGHT_X); // 旋转

        auto [fl_v, fr_v, bl_v, br_v] = g_solver.inverse({
            leftY * 2.5f * m / s,
            -leftX * 2.5f * m / s,
            -rightX * 2.0f * rad / s,
        });
        topic_one_chassis.write({
            fl_v.numerical_value_in(rad / s), fr_v.numerical_value_in(rad / s), bl_v.numerical_value_in(rad / s),
            br_v.numerical_value_in(rad / s)
        });

        (void)m_fl->setAngRef(fl_v);
        (void)m_fr->setAngRef(-fr_v);
        (void)m_bl->setAngRef(bl_v);
        (void)m_br->setAngRef(-br_v);
    }
}

// Register the Node with OneFramework
ONE_NODE_REGISTER(OneChassisNode);
