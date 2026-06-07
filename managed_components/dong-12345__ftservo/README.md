# ftservo -- FEETECH 飞特总线舵机 ESP-IDF 组件

[![Component Registry](https://components.espressif.com/components/dong-12345/ftservo/badge.svg)](https://components.espressif.com/components/dong-12345/ftservo)

基于 ESP-IDF 的 FEETECH (Futaba) 总线伺服电机控制库，支持多种 FEETECH 伺服电机系列。

## 支持的舵机系列

| 系列 | 位置范围 | 角度范围 | 特色功能 |
|------|---------|---------|---------|
| **SCSCL** (SCS15/SCS20/SCS30) | 0-1023 | 0-300 | 位置/速度控制、PWM模式 |
| **SMS_STS** (SMS03/SMS05/STS30) | 0-4095 | 0-360 | 位置+加速度控制、轮子模式 |
| **HLS** (HLS15/HLS20) | 0-4095 | 0-360 | 位置+加速度+扭矩控制 |

## 快速开始

### 1. 添加到您的项目

#### 方法一：通过 ESP Component Registry（推荐）

```bash
cd your_project/
idf.py add-dependency "dong-12345/ftservo^1.0.0"
```

#### 方法二：手动添加

将 `ftservo` 组件复制到项目的 `components/` 目录下：

```
your_project/
├── components/
│   └── ftservo/          # 本组件
├── main/
│   └── main.c
├── CMakeLists.txt
└── ...
```

### 2. 基本使用

```c
#include "SCServo.h"

void app_main(void)
{
    // 一键初始化（自动配置UART和协议参数）
    ftServo_InitWithType(SERVO_SCSCL, 47, 21, -1, 1000000);

    // 检测舵机
    Ping(1);

    // 位置控制
    WritePos(1, 512, 2000, 1000);

    // 读取反馈
    int pos = ReadPos(1);
    int temp = ReadTemper(1);
}
```

## API 概览

### 初始化

| 函数 | 说明 |
|------|------|
| `ftServo_InitWithType(type, tx, rx, rts, baud)` | 一键初始化（推荐） |
| `ftServo_Init(uart_num, tx, rx, rts, baud)` | 底层 UART 初始化 |
| `ftServo_Deinit(uart_num)` | 反初始化 |

### 通用控制

| 函数 | 说明 |
|------|------|
| `Ping(ID)` | 检测舵机是否在线 |
| `Reset(ID)` | 恢复出厂设置 |
| `WritePos(ID, pos, time, speed)` | 位置控制（SCSCL） |
| `WritePosEx(ID, pos, speed, acc)` | 位置控制（SMS_STS） |
| `WritePosEx2(ID, pos, speed, acc, torque)` | 位置控制（HLS） |
| `EnableTorque(ID, enable)` | 扭矩开关 |
| `FeedBack(ID)` | 一次性读取所有反馈 |

### 反馈读取

| 函数 | 说明 |
|------|------|
| `ReadPos(ID)` | 读取当前位置 |
| `ReadSpeed(ID)` | 读取当前速度 |
| `ReadLoad(ID)` | 读取当前负载 |
| `ReadVoltage(ID)` | 读取电压 |
| `ReadTemper(ID)` | 读取温度 |
| `ReadCurrent(ID)` | 读取电流 |
| `ReadMove(ID)` | 读取移动状态 |

### 高级控制

| 函数 | 说明 |
|------|------|
| `SyncWritePos()` | 同步写入（多舵机） |
| `SyncWritePosEx()` | 同步写入（SMS_STS） |
| `SyncWritePosEx2()` | 同步写入（HLS） |
| `PWMMode()` / `WritePWM()` | PWM模式（SCSCL） |
| `WheelMode()` / `WriteSpe()` | 轮子模式（SMS_STS） |
| `WriteSpeEx()` | 速度控制（HLS） |
| `CalibrationOfs()` | 零点校准 |
| `LockEprom()` / `unLockEprom()` | EEPROM 锁定/解锁 |

## 硬件连接

```
舵机总线         ESP32
DATA (黄) ────  TX (GPIO 47)
DATA (黄) ────  RX (GPIO 21)
GND (棕)  ────  GND
VCC (红)  ────  6-12V 外部电源
```

> 注意：舵机需要独立的外部电源（6-12V），不可直接从 ESP32 USB 取电。

## 示例

完整的示例程序请参考 [examples/basic/](examples/basic/) 目录：

```bash
cd examples/basic/
idf.py build
idf.py -p COMx flash monitor
```

## 依赖

- ESP-IDF >= 6.0
- 无需额外外部依赖

## 许可证

Apache License 2.0
