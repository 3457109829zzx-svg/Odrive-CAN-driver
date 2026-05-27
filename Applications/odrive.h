#ifndef __ODRIVE_H
#define __ODRIVE_H

#include "main.h"
#include "stdio.h"
#include "string.h"

#define AXIS0_NODE_ID (1 << 5) // odrive ID
#define AXIS1_NODE_ID (2 << 5)

typedef union _float_to_uint8_t
{
  uint8_t raw[8];
  float value[2];
  uint32_t u32_data[2];
  int32_t int32_data[2];
} float_to_uint8_t;

typedef enum
{
  AXIS_0 = 0,
  AXIS_1 = 1
} Axis_t;

typedef enum
{
  AXIS_STATE_UNDEFINED = 0,                  //<! will fall through to idle
  AXIS_STATE_IDLE = 1,                       //<! disable PWM and do nothing
  AXIS_STATE_STARTUP_SEQUENCE = 2,           //<! the actual sequence is defined by the config.startup_... flags
  AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,  //<! run all calibration procedures, then idle
  AXIS_STATE_MOTOR_CALIBRATION = 4,          //<! run motor calibration
  AXIS_STATE_SENSORLESS_CONTROL = 5,         //<! run sensorless control
  AXIS_STATE_ENCODER_INDEX_SEARCH = 6,       //<! run encoder index search
  AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7, //<! run encoder offset calibration
  AXIS_STATE_CLOSED_LOOP_CONTROL = 8,        //<! run closed loop control
  AXIS_STATE_LOCKIN_SPIN = 9,                //<! run lockin spin
  AXIS_STATE_ENCODER_DIR_FIND = 10,
  AXIS_STATE_HOMING = 11, //<! run axis homing function
} State_t;

typedef struct
{
  uint32_t axis_error;
  uint32_t axis_current_state;
  uint32_t motor_error;
  uint32_t encoder_error;
  uint32_t sensorless_error;
  float encoder_pos_estimate;
  float encoder_vel_estimate;
  int32_t encoder_shadow_count;
  int32_t encoder_cpr_count;
  float iq_setpoint;
  float iq_measured;
  float sensorless_pos_estimate;
  float sensorless_vel_estimate;
  float vbus_voltage;
} OdriveAxisGetState_t;

typedef struct
{
  uint16_t axis_node_id;
  uint32_t requested_state;
  int32_t control_mode;
  int32_t input_mode;
  int16_t vel_ff;
  int16_t current_ff;
  int32_t input_pos;
  // int32_t input_vel;
  float input_vel;
  float torque_vel;
  int32_t input_current;
  float vel_limit;
  float traj_vel_limit;//梯形模式（traj）
  float traj_accel_limit;
  float traj_decel_limit;
  float traj_a_per_css;
} OdriveAxisSetState_t;

typedef enum
{
  MSG_CO_NMT_CTRL = 0x000,      // CANOpen NMT Message REC
  MSG_ODRIVE_HEARTBEAT,
  MSG_ODRIVE_ESTOP,
  MSG_GET_MOTOR_ERROR,          // Errors
  MSG_GET_ENCODER_ERROR,        //...
  MSG_GET_SENSORLESS_ERROR,     //...
  MSG_SET_AXIS_NODE_ID,         // 获取ID
  MSG_SET_AXIS_REQUESTED_STATE, // 命令axis0进入某个状态
  MSG_SET_AXIS_STARTUP_CONFIG,  //...
  MSG_GET_ENCODER_ESTIMATES,    // 当前预测到的位置值(单位:turn)和当前估算转速(单位:turn/s)
  MSG_GET_ENCODER_COUNT,        //...同上,单位为count和count/s
  MSG_SET_CONTROLLER_MODES,     // 设置控制模式和控制信号输入模式--
  MSG_SET_INPUT_POS = 0x00C,    // 输入的电机目标位置
  MSG_SET_INPUT_VEL = 0x00D,    // 输入的电机目标转速-------###
  MSG_SET_INPUT_CURRENT,        // 输入的电机输出的力矩大小
  MSG_SET_VEL_LIMIT,            // 设置速度和电流限制
  MSG_START_ANTICOGGING,        // 执行anticogging校准
  MSG_SET_TRAJ_VEL_LIMIT,       // 轨迹控制最大速度(即匀速阶段的转速)
  MSG_SET_TRAJ_ACCEL_LIMITS,    // 轨迹控制最大加速度和最大反向加速度 turn/s^2
  MSG_SET_TRAJ_A_PER_CSS,       // 轨迹控制负载惯量大小,默认为0
  MSG_GET_IQ,                   // 电流环控制输入的目标交轴电流值和通过电流采样获取的交轴电流值
  MSG_GET_SENSORLESS_ESTIMATES, // 得到当前MOSFET温度和电机温度
  MSG_RESET_ODRIVE,             // 重启ODrive硬件,同odrv0.reboot()
  MSG_GET_VBUS_VOLTAGE,         // 获取当前DC总线上的电压和电流
  MSG_CLEAR_ERRORS = 0x018,     // 清除错误
  MSG_CO_HEARTBEAT_CMD = 0x700, // CANOpen NMT Heartbeat SEND
} Odrive_Commond;

void odrive_set_speed(float speed); // TODO: check
uint8_t odrv_write_msg(Axis_t axis, Odrive_Commond cmd);

#endif
