# ODrive 电机 CAN 总线驱动库

## 项目简介

基于 STM32F407 的 ODrive 无刷电机控制器 CAN 总线驱动库，实现 CANopen 协议通信，支持速度、位置、力矩三种控制模式及梯形轨迹规划，适用于 ROBOCON 等机器人竞赛中的电机驱动需求。

- **主控芯片**：STM32F407IGHx (Cortex-M4, 168MHz)
- **开发环境**：STM32CubeMX + Keil MDK-ARM
- **固件库**：HAL 库
- **通信协议**：CAN 2.0 (CANopen)
- **编程语言**：C

## 硬件架构

```
┌─────────────────────────────────┐
│         STM32F407IGHx            │
│                                  │
│  ┌──────────┐  ┌──────────────┐ │
│  │ CAN1     │  │ USART6       │ │
│  │ 与ODrive │  │ 调试串口      │ │
│  │ 通信     │  │              │ │
│  └──────────┘  └──────────────┘ │
│  ┌──────────┐                   │
│  │ GPIO     │                   │
│  │ 外设控制  │                   │
│  └──────────┘                   │
└─────────────────────────────────┘
         │ CAN Bus
    ┌────┴────┐
    │ ODrive  │  ← 双轴无刷电机控制器
    │ Axis0/1 │
    └────┬────┘
    ┌────┴────┐
    │ 无刷电机 │
    │ Motor0/1│
    └─────────┘
```

## 目录结构

```
├── Applications/
│   ├── odrive.c                 # ODrive CANopen 协议实现
│   └── odrive.h                 # 数据结构、命令枚举、API 声明
├── bsp/
│   ├── bsp_can.c/h              # CAN 过滤器配置与发送封装
│   └── bsp_uart.c/h             # 串口调试封装
├── Core/                        # CubeMX 外设初始化
│   ├── Inc/                     # can.h, usart.h, gpio.h, main.h
│   └── Src/                     # main.c, can.c, usart.c, gpio.c
├── Drivers/                     # STM32F4 HAL + CMSIS
├── MDK-ARM/                     # Keil 工程文件
│   ├── odrive_motor.uvprojx     # Keil 项目文件
│   └── startup_stm32f407xx.s   # 启动文件
└── odrive_motor.ioc             # CubeMX 配置
```

## 控制模式

| 模式 | 命令 | 说明 |
|------|------|------|
| 速度控制 | `MSG_SET_INPUT_VEL` (0x0D) | 设定目标转速 (turn/s)，含力矩前馈 |
| 位置控制 | `MSG_SET_INPUT_POS` (0x0C) | 设定目标位置，含速度/力矩前馈 |
| 电流控制 | `MSG_SET_INPUT_CURRENT` (0x0E) | 设定目标力矩电流 |
| 轨迹控制 | `MSG_SET_TRAJ_VEL_LIMIT` | 梯形速度轨迹：设定最大速度、加速度、减速度 |

## CANopen 协议实现

实现 20+ 条 ODrive CAN 命令，涵盖：

- **状态机管理**：空闲 → 校准（电机/编码器/偏置）→ 闭环控制
- **参数读取**：编码器位置/速度估计、IQ 电流、总线电压
- **参数设置**：速度限制、轨迹参数、控制模式
- **系统命令**：急停、错误清除、重启

```c
// 速度控制示例
odrive_set_speed(5);    // Axis0 以 5 turn/s 正转
HAL_Delay(2000);
odrive_set_speed(0);    // 停止
HAL_Delay(2000);
odrive_set_speed(-5);   // 反转
```

### 数据打包

使用 `union` 实现 float 与 uint8_t[8] 的零拷贝转换，适配 CAN 报文 8 字节数据段：

```c
typedef union _float_to_uint8_t {
    uint8_t raw[8];
    float value[2];
    uint32_t u32_data[2];
    int32_t int32_data[2];
} float_to_uint8_t;
```

## 硬件接线

| STM32 引脚 | 连接 |
|-----------|------|
| PD0/PD1 (CAN1) | ODrive CAN 接口 |
| PC6/PC7 (USART6) | 调试串口 |

## 快速开始

```bash
# 用 Keil MDK 打开工程
MDK-ARM/odrive_motor.uvprojx

# 编译并烧录
# 1. Build (F7)
# 2. Download (F8)
```

## 许可证

仅用于学术竞赛与学习交流目的。

---

*开发时间：2023 年 · ROBOCON 备赛*
