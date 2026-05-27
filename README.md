# ODrive 电机 CAN 总线驱动库

基于 STM32F407 的 ODrive 无刷电机控制器 CAN 总线驱动，核心仅两个文件 **`odrive.c` + `odrive.h`**，实现 CANopen 协议栈，支持速度、位置、力矩控制及梯形轨迹规划，可直接移植到任意 STM32 项目中复用。

- **主控芯片**：STM32F407IGHx (Cortex-M4, 168MHz)
- **开发环境**：Keil MDK-ARM 5 + STM32CubeMX
- **固件库**：HAL 库
- **通信协议**：CAN 2.0 (CANopen)
- **核心文件**：`Applications/odrive.h` + `Applications/odrive.c`

## 核心文件

| 文件 | 内容 | 行数 |
|------|------|------|
| `Applications/odrive.h` | 数据结构定义、CANopen 命令枚举、电机状态枚举、API 声明 | ~110 |
| `Applications/odrive.c` | 协议栈核心实现 `odrv_write_msg()`、速度控制 `odrive_set_speed()` | ~180 |

使用者只需 `#include "odrive.h"`，调用 `odrive_set_speed()` 即可驱动电机。底层依赖 HAL 库的 `hcan1` 和 `<string.h>` 的 `memcpy`，无其他耦合。

### odrive.h 核心数据结构

```c
// 电机轴参数结构体（设定值）
typedef struct {
    uint16_t axis_node_id;
    uint32_t requested_state;     // 状态机（校准/闭环/空闲）
    int32_t control_mode;         // 控制模式
    int32_t input_mode;           // 输入模式
    int16_t vel_ff;               // 速度前馈
    int16_t current_ff;           // 力矩前馈
    int32_t input_pos;            // 目标位置
    float input_vel;              // 目标速度 (turn/s)
    float torque_vel;             // 力矩速度 (turn/s)
    float vel_limit;              // 速度限制
    float traj_vel_limit;         // 梯形轨迹最大速度
    float traj_accel_limit;       // 梯形轨迹加速度
    float traj_decel_limit;       // 梯形轨迹减速度
} OdriveAxisSetState_t;

// 零拷贝 CAN 数据打包
typedef union {
    uint8_t raw[8];
    float value[2];
    uint32_t u32_data[2];
    int32_t int32_data[2];
} float_to_uint8_t;
```

### odrive.h CANopen 命令枚举

共 23 条命令，覆盖电机全生命周期：

| 命令 | 值 | 功能 |
|------|-----|------|
| `MSG_CO_NMT_CTRL` | 0x000 | NMT 网络管理 |
| `MSG_ODRIVE_HEARTBEAT` | — | 心跳检测 |
| `MSG_ODRIVE_ESTOP` | — | 紧急停止 |
| `MSG_GET_MOTOR_ERROR` | — | 读取电机故障码 |
| `MSG_GET_ENCODER_ERROR` | — | 读取编码器故障码 |
| `MSG_SET_AXIS_REQUESTED_STATE` | — | 设置轴状态（空闲→校准→闭环） |
| `MSG_GET_ENCODER_ESTIMATES` | — | 读取编码器估计位置/速度 |
| `MSG_SET_CONTROLLER_MODES` | — | 设置控制模式+输入模式 |
| `MSG_SET_INPUT_POS` | 0x00C | 设定目标位置 |
| `MSG_SET_INPUT_VEL` | 0x00D | 设定目标速度 |
| `MSG_SET_INPUT_CURRENT` | 0x00E | 设定输出力矩 |
| `MSG_SET_VEL_LIMIT` | — | 设定速度限制 |
| `MSG_SET_TRAJ_VEL_LIMIT` | — | 梯形轨迹最大速度 |
| `MSG_SET_TRAJ_ACCEL_LIMITS` | — | 梯形轨迹加减速度 |
| `MSG_GET_IQ` | — | 读取交轴电流 |
| `MSG_GET_VBUS_VOLTAGE` | — | 读取总线电压 |
| `MSG_CLEAR_ERRORS` | 0x018 | 清除故障 |
| `MSG_RESET_ODRIVE` | — | 重启 ODrive |

### odrive.c 核心实现

`odrv_write_msg()` 是协议栈核心，根据命令类型构造 CAN 帧并通过 `HAL_CAN_AddTxMessage()` 发送：

```c
uint8_t odrv_write_msg(Axis_t axis, Odrive_Commond cmd)
{
    CAN_TxHeaderTypeDef header;
    float_to_uint8_t pack;
    
    // CAN ID = 轴节点ID + 命令码
    header.StdId = (axis == AXIS_0 ? AXIS0_NODE_ID : AXIS1_NODE_ID) + cmd;
    
    switch(cmd) {
        case MSG_SET_INPUT_VEL:
            pack.value[0] = odrive_set->input_vel;   // 目标速度
            pack.value[1] = odrive_set->torque_vel;   // 力矩前馈
            break;
        case MSG_SET_INPUT_POS:
            memcpy(data, &(odrive_set->input_pos), 4); // 目标位置
            data[4] = odrive_set->vel_ff & 0x00FF;     // 速度前馈低字节
            data[5] = odrive_set->vel_ff >> 8;          // 速度前馈高字节
            break;
        // ... 其余命令
    }
    HAL_CAN_AddTxMessage(&hcan1, &header, pack.raw, &send_mail_box);
}
```

## 控制模式

| 模式 | API | 说明 |
|------|-----|------|
| 速度控制 | `odrive_set_speed(turn/s)` | 设定转速 + 力矩前馈 |
| 位置控制 | `MSG_SET_INPUT_POS` | 设定位置 + 速度/力矩前馈 |
| 电流控制 | `MSG_SET_INPUT_CURRENT` | 设定输出力矩电流 |
| 轨迹控制 | `MSG_SET_TRAJ_*` | 梯形速度曲线：匀速速度、加速度、减速度 |

```c
// 速度控制
odrive_set_speed(5);     // 5 turn/s
odrive_set_speed(0);     // 停止
odrive_set_speed(-5);    // 反转
```

## 硬件接线

```
STM32F407                ODrive
────────────────────────────────
PD0 (CAN1_RX)  ←───→  CAN_H
PD1 (CAN1_TX)  ←───→  CAN_L
GND            ←───→  GND
PC6 (USART6_TX) ───→  调试串口 (可选)
```

## 目录结构

```
├── Applications/
│   ├── odrive.h          ★ 核心：数据结构、枚举、API 声明
│   └── odrive.c          ★ 核心：CANopen 协议栈实现
├── bsp/
│   ├── bsp_can.h/c        CAN 过滤器配置、发送封装
│   └── bsp_uart.h/c       串口调试封装
├── Core/                  CubeMX 生成的外设初始化代码
├── Drivers/               STM32F4 HAL 库 + CMSIS
├── MDK-ARM/               Keil 工程文件
└── odrive_motor.ioc       CubeMX 配置
```

## 移植指南

将 `odrive.h` 和 `odrive.c` 复制到目标工程，确保：
1. HAL CAN 初始化完成 (`hcan1` 可用)
2. 根据硬件配置修改 `AXIS0_NODE_ID` / `AXIS1_NODE_ID`
3. 调用 `CAN_Filter_Init()` 配置 CAN 过滤器
4. 使用 `odrive_set_speed()` 或 `odrv_write_msg()` 控制电机

## 许可证

仅用于学术竞赛与学习交流。

---

*2023 · ROBOCON 备赛*
