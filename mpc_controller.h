/**
 * @file mpc_controller.h
 * @brief MPC 轨迹跟踪用 PID 残差反馈控制器。
 *
 * 反馈律：
 *   u = Kp * e_angle + Ki * integral(e_angle) + Kd * e_speed
 *   e_speed = target_speed - measured_speed
 *
 * 微分项使用角速度误差，避免对带噪的角度误差做差分。
 */
#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include <stdint.h>

/**
 * @brief 物理模型电流前馈参数。
 * J: kg*m^2, B: N*m*s/rad, K_t: N*m/A,
 * command_units_per_amp: DJI motor command units per ampere.
 */
typedef struct
{
    float J;        // kg*m^2，转动惯量
    float B;        // N*m*s/rad，填粘滞阻尼
    float K_t;      // N*m/A，扭矩常数
    float command_units_per_amp;       // 16384 / 3 A，GM6020电流控制报文
} MPC_Current_Feedforward_Config_s;

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float integral_limit; /* 积分项输出限幅，单位与输出一致；0 表示关闭积分。 */
    float angle_period;   /* 角度周期；0 表示不环绕，度制填 360，弧度制填 2*pi。 */
} MPC_PID_Config_s;

typedef struct
{
    MPC_PID_Config_s config;

    float angle_error;
    float previous_angle_error;
    float speed_error;
    float integral_output;
    float proportional_output;
    float derivative_output;
    float output;
    float dt;       /* 最近一次测得的控制周期，单位秒。 */
    uint32_t DWT_CNT; /* 用于计算控制周期的 DWT 计数快照。 */

    uint8_t has_previous_error;
} MPC_PID_t;

/**
 * @brief 初始化控制器。
 * @return 1 表示成功，0 表示参数无效或指针为空。
 */
uint8_t MPC_PIDInit(MPC_PID_t *controller, const MPC_PID_Config_s *config);

/** @brief 清除积分、历史误差和输出，保留参数配置。 */
void MPC_PID_Reset(MPC_PID_t *controller);

/**
 * @brief 按 J*alpha + B*omega 计算电流前馈。
 * @param config 物理参数及安培到 DJI 电机设定值的换算比例
 * @param target_acceleration 目标角加速度，deg/s^2
 * @param target_speed 目标角速度，deg/s
 * @return DJI 电机设定值单位；参数无效时返回 0
 * 输入使用度制；函数内部转换为 rad/s^2 和 rad/s 后按 SI 物理参数计算。
 */
float MPC_CurrentFeedforwardCalculate(const MPC_Current_Feedforward_Config_s *config,
                                      float target_acceleration,
                                      float target_speed);

/**
 * @brief 计算 MPC 轨迹跟踪反馈输出。
 * @param target_angle 目标角度
 * @param target_speed 目标角速度
 * @param measured_angle 实际角度
 * @param measured_speed 实际角速度
 * 控制周期由模块内部通过 DWT 计数器测量，单位为秒。
 * @return 反馈输出；输入无效或测得的周期无效时清空状态并返回 0。
 *
 * 角度与角速度单位必须一致（例如度、度/秒，或弧度、弧度/秒）。
 * 输出单位由调用方的增益定义，可用于力矩或电流修正量。
 */
float MPC_PIDCalculate(MPC_PID_t *controller,
                        float target_angle,
                        float target_speed,
                        float measured_angle,
                        float measured_speed);

#endif /* MPC_CONTROLLER_H */
