/**
 * @file mpc_pid.c
 * @brief MPC 轨迹跟踪用 PID 残差反馈控制器实现。
 */
#include "mpc_controller.h"

#include "bsp_dwt.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define MPC_DEG_TO_RAD 0.01745329252f

static float MPC_PID_WrapAngleError(float error, float period)
{
    if (period <= 0.0f)
        return error;

    error = fmodf(error, period);
    if (error > period * 0.5f)
        error -= period;
    else if (error < -period * 0.5f)
        error += period;

    return error;
}

static uint8_t MPC_PID_ConfigIsValid(const MPC_PID_Config_s *config)
{
    if (config == NULL)
        return 0U;

    if (!isfinite(config->Kp) || !isfinite(config->Ki) || !isfinite(config->Kd) ||
        !isfinite(config->integral_limit) ||
        !isfinite(config->angle_period))
        return 0U;

    if (config->integral_limit < 0.0f ||
        config->angle_period < 0.0f)
        return 0U;

    return 1U;
}

uint8_t MPC_PIDInit(MPC_PID_t *controller, const MPC_PID_Config_s *config)
{
    if (controller == NULL || !MPC_PID_ConfigIsValid(config))
        return 0U;

    memset(controller, 0, sizeof(*controller));
    controller->config = *config;
    (void)DWT_GetDeltaT(&controller->DWT_CNT); /* 建立首次计算的计数基准。 */
    return 1U;
}

void MPC_PID_Reset(MPC_PID_t *controller)
{
    if (controller == NULL)
        return;

    controller->angle_error = 0.0f;
    controller->previous_angle_error = 0.0f;
    controller->speed_error = 0.0f;
    controller->integral_output = 0.0f;
    controller->proportional_output = 0.0f;
    controller->derivative_output = 0.0f;
    controller->output = 0.0f;
    controller->has_previous_error = 0U;
    controller->dt = 0.0f;
    (void)DWT_GetDeltaT(&controller->DWT_CNT); /* 重置计时基准，避免累积停用期间的时间。 */
}

float MPC_CurrentFeedforwardCalculate(const MPC_Current_Feedforward_Config_s *config,
                                      float target_acceleration,
                                      float target_speed)
{
    float target_acceleration_rad;
    float target_speed_rad;
    float torque;
    float command;

    if (config == NULL ||
        !isfinite(config->J) || !isfinite(config->B) ||
        !isfinite(config->K_t) ||
        !isfinite(config->command_units_per_amp) ||
        !isfinite(target_acceleration) || !isfinite(target_speed) ||
        config->J < 0.0f || config->B < 0.0f ||
        config->K_t <= 0.0f || config->command_units_per_amp <= 0.0f)
        return 0.0f;

    /* 外部轨迹使用度制，J、B 的物理参数使用 SI 制，计算前统一换为弧度制。 */
    target_acceleration_rad = target_acceleration * MPC_DEG_TO_RAD;
    target_speed_rad = target_speed * MPC_DEG_TO_RAD;

    torque = config->J * target_acceleration_rad +
             config->B * target_speed_rad;
    command = torque / config->K_t * config->command_units_per_amp;
    return isfinite(command) ? command : 0.0f;
}

float MPC_PIDCalculate(MPC_PID_t *controller,
                        float target_angle,
                        float target_speed,
                        float measured_angle,
                        float measured_speed)
{
    float dt;
    float integral_delta;
    float candidate_integral;
    float previous_error;
    float unsaturated_output;

    if (controller == NULL ||
        !isfinite(target_angle) || !isfinite(target_speed) ||
        !isfinite(measured_angle) || !isfinite(measured_speed) ||
        !MPC_PID_ConfigIsValid(&controller->config))
    {
        MPC_PID_Reset(controller);
        return 0.0f;
    }

    dt = DWT_GetDeltaT(&controller->DWT_CNT);
    controller->dt = dt;
    if (!isfinite(dt) || dt <= 0.0f)
    {
        MPC_PID_Reset(controller);
        return 0.0f;
    }

    controller->angle_error = MPC_PID_WrapAngleError(
        target_angle - measured_angle, controller->config.angle_period);
    controller->speed_error = target_speed - measured_speed;
    controller->proportional_output = controller->config.Kp * controller->angle_error;
    controller->derivative_output = controller->config.Kd * controller->speed_error;

    if (!isfinite(controller->angle_error) || !isfinite(controller->speed_error) ||
        !isfinite(controller->proportional_output) ||
        !isfinite(controller->derivative_output))
    {
        MPC_PID_Reset(controller);
        return 0.0f;
    }

    /* 梯形积分；积分状态直接保存为输出量，便于按输出单位限幅。 */
    previous_error = controller->has_previous_error
                         ? controller->previous_angle_error
                         : controller->angle_error;
    integral_delta = controller->config.Ki *
                     (0.5f * (controller->angle_error + previous_error)) * dt;

    if (!isfinite(integral_delta))
    {
        MPC_PID_Reset(controller);
        return 0.0f;
    }

    candidate_integral = controller->config.integral_limit > 0.0f
                             ? controller->integral_output + integral_delta
                             : 0.0f;

    /* Clamp the integral contribution to its configured output limit. */
    if (controller->config.integral_limit > 0.0f)
    {
        if (candidate_integral > controller->config.integral_limit)
            candidate_integral = controller->config.integral_limit;
        else if (candidate_integral < -controller->config.integral_limit)
            candidate_integral = -controller->config.integral_limit;
    }

    /* 条件积分抗饱和：丢弃继续推动输出饱和的积分增量。 */
    unsaturated_output = controller->proportional_output +
                         candidate_integral +
                         controller->derivative_output;
    if (!isfinite(unsaturated_output))
    {
        MPC_PID_Reset(controller);
        return 0.0f;
    }

    controller->integral_output = candidate_integral;
    controller->output = unsaturated_output;
    if (!isfinite(controller->output))
    {
        MPC_PID_Reset(controller);
        return 0.0f;
    }

    controller->previous_angle_error = controller->angle_error;
    controller->has_previous_error = 1U;
    return controller->output;
}