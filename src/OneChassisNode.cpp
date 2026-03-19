#include <OneChassisData.hpp>
#include <OneChassisNode.hpp>

#include <OF/lib/ImuHub/ImuHub.hpp>
#include <OF/lib/NotifyHub/Notify.hpp>
#include <OF/lib/VtHub/VtHub.hpp>

#include <numbers>

#include <ems_parser.hpp>

#include "RPL/Packets/VT03RemotePacket.hpp"

/* TODO:
 *   1. 目前的电机基类不够优雅，CRTP导致必须要有模板参数，基本无法创建一个通用的类。
 *   或许应该添加一个wrapper Class或者去掉CRTP改称虚函数集成
 *   又或许给bind添加template头 (已解决)
 *   2. Node的bind方法需要使用头文件暴露出Node类的Config, 进行配置可能会不太方便
 *   当然其实还好，改一下node的template，默认给一个hpp，在CMake里library_include_directories就行。
 *   需要在模板内引入一点k_sleep，不然的话容易出现用户Node优先级太高导致日志无法输出的问题。
 *   3. 各个Hub之间的方法不统一，早期开发的Hub只能获取单例指针来调用类方法，NotifyHub可以直接调用全局函数
 *   我觉得全局函数是对的，应该为ControllerHub和ImuHub增加方便的全局函数调用。
 *   4. ControllerHub应该加入某种虚拟化机制，把不同的遥控器值映射到某个区间，并提供float百分比之类的转换函数
 *   也许后者就够了？ （已解决）
 *   5. OneMotor Zephyr 的CanDriver ，需要比SharedPtr更优雅的方案
 *
 */

LOG_MODULE_REGISTER(OneChassisNode, CONFIG_ONE_CHASSIS_LOG_LEVEL);


// Register Topic for data communication
ONE_TOPIC_REGISTER(OneChassisData, topic_one_chassis, "one_chassis_data");

using namespace ems::literals;
constexpr auto melody =
    R"((200)
        2s`,,1s`,7,,1s`,2s`-,3`-2s`,1s`,,,
        2s`,,1s`,7,,1s`,2s`-,3`-2s`,1s`,,,
        2s`,,1s`,7,,1s`,2s`-,3`-2s`,1s`,,,
        2s`,,1s`,7,,1s`,2s`-,3`-2s`,1s`,,7-1s`-
        2s`,2s`,1s`,3`,2s`,1s`,1s`,1s`,7,3`,2s`,1s`,
        1s`,,7-1s`-2s`,,,,,,7,4s`,7`,
        6s`,,7`,6s`,,7`,6s`-5s`-4s`,,4s`,1s`,3`,
        3`,2s`,2s`,2s`,,,3`,2s`,1s`,2s`,,4s`,
        7,,,0
)"_ems;

static constexpr PidParams<> YAW_DEFAULT_PARAMS{
    .Kp = 3,
    .Ki = 0,
    .Kd = 0.1
};

static PidController m_yaw_pid(YAW_DEFAULT_PARAMS);

using one::motor::dji::AngMode;

bool OneChassisNode::init()
{
    LOG_INF("init chassis");
    if (config.can_dev == nullptr) return false;
    (void)m_driver.init(config.can_dev);
    (void)m_fl.init(m_driver, one::motor::dji::Param{2, AngMode{g_ang_params}});
    (void)m_fr.init(m_driver, one::motor::dji::Param{1, AngMode{g_ang_params}});
    (void)m_bl.init(m_driver, one::motor::dji::Param{3, AngMode{g_ang_params}});
    (void)m_br.init(m_driver, one::motor::dji::Param{4, AngMode{g_ang_params}});
    m_fl.setAngRef(0);
    m_fr.setAngRef(0);
    m_bl.setAngRef(0);
    m_br.setAngRef(0);
    (void)m_fl.enable();
    (void)m_fr.enable();
    (void)m_bl.enable();
    (void)m_br.enable();
    m_is_yaw_initialized = false;
    return true;
}

void OneChassisNode::run()
{
    constexpr led_color c_normal = COLOR_HEX("#2ec4b6");
    constexpr led_color c_warning = COLOR_HEX("#f95738");
    LOG_INF("running chassis");
    while (true)
    {
        k_sleep(K_MSEC(10));
        auto state = VtHub::get<VT03RemotePacket>();
        if (!state || state.value().switch_state == 1)
        {
            m_led_guard.set({c_warning, LEDMode::Breathing, 1, 300});
            LOG_INF("Disconnected or Emergency...");
            m_fl.setAngRef(0);
            m_fr.setAngRef(0);
            m_bl.setAngRef(0);
            m_br.setAngRef(0);
            k_sleep(K_MSEC(500));
            continue;
        }
        m_led_guard.set({c_normal, LEDMode::Breathing, 1, 300});
        auto data = state.value();
        // const auto swR = data[SW_R];
        const auto vx_local = vt_stick_percent(data.left_stick_y); // 前后
        const auto vy_local = -vt_stick_percent(data.left_stick_x); // 左右
        float vw_command = vt_stick_percent(data.right_stick_x);
        // 旋转

        const auto imu_data = getImuData();
        const float current_yaw = -imu_data.euler_angle.yaw;
        if (!m_is_yaw_initialized)
        {
            m_target_yaw = current_yaw;
            m_is_yaw_initialized = true;
        }

        if (constexpr float ROTATION_DEADZONE = 0.05f; std::abs(vw_command) > ROTATION_DEADZONE)
        {
            // 手动旋转
            // 直接使用摇杆值作为速度
            m_target_yaw = current_yaw;
        }
        else
        {
            // 航向锁定
            // 右摇杆归中，用 PID 计算修正量

            float yaw_error = m_target_yaw - current_yaw;

            // 过零点处理

            while (yaw_error > std::numbers::pi_v<float>) yaw_error -= 2 * std::numbers::pi_v<float>;
            while (yaw_error < -std::numbers::pi_v<float>) yaw_error += 2 * std::numbers::pi_v<float>;
            vw_command = m_yaw_pid.compute(yaw_error, 0);
        }

        auto [fl_v, fr_v, bl_v, br_v] = g_solver.inverse({
            vx_local * 3.0f * m / s,
            vy_local * 3.0f * m / s,
            -vw_command * 3.0f * rad / s,
        });
        topic_one_chassis.write({
            fl_v.numerical_value_in(rad / s), fr_v.numerical_value_in(rad / s), bl_v.numerical_value_in(rad / s),
            br_v.numerical_value_in(rad / s)
        });

        m_fl.setAngUnitRef(fl_v);
        m_fr.setAngUnitRef(-fr_v);
        m_bl.setAngUnitRef(bl_v);
        m_br.setAngUnitRef(-br_v);
    }
}

// Register the Node with OneFramework
ONE_NODE_REGISTER(OneChassisNode);
