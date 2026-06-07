/**
 * @file ftservo_example.c
 * @brief FEETECH 总线舵机使用示例
 *
 * 本示例演示了 ftservo 组件的完整使用方法，覆盖三种舵机系列：
 *   - SCSCL 系列（如 SCS15/SCS20/SCS30）：基础位置控制、PWM模式
 *   - SMS_STS 系列（如 SMS03/SMS05/STS30）：位置+加速度、轮子模式
 *   - HLS 系列（如 HLS15/HLS20）：位置+加速度+扭矩限制
 *
 * 使用步骤：
 *   1. 将本目录复制到您的项目 components/ 目录旁
 *   2. 确保 components/ftservo 已正确添加到项目
 *   3. 根据实际硬件修改引脚号和舵机ID
 *   4. 运行 idf.py build 编译
 *
 * 硬件连接：
 *   - 舵机总线 DATA 线 -> ESP32 TX (GPIO) 和 RX (GPIO)
 *   - 舵机电源 VCC -> 6-12V 外部电源（注意：不要从ESP32的USB取电！）
 *   - 舵机电源 GND -> ESP32 GND（共地）
 *   - 如果使用 RS485 半双工模式，需连接 RTS 引脚到 MAX3485 的 RE/DE
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "SCServo.h"

/*
=============================================================================
    用户配置区 —— 根据您的硬件修改以下参数
=============================================================================
*/

/**
 * 舵机类型选择
 *   SERVO_SCSCL  - SCS系列标准舵机，如 SCS15/SCS20/SCS30
 *   SERVO_SMS_STS - SMS/STS系列舵机，如 SMS03/SMS05/STS30
 *   SERVO_HLS    - HLS系列舵机，如 HLS15/HLS20
 */
#define SERVO_TYPE          SERVO_SCSCL

/** UART 引脚配置 */
#define FTSERVO_TX_PIN      47          /**< UART 发送引脚 */
#define FTSERVO_RX_PIN      21          /**< UART 接收引脚 */
#define FTSERVO_RTS_PIN     -1          /**< RS485 方向控制引脚，-1=禁用RS485 */
#define FTSERVO_BAUD        1000000     /**< 通信波特率，通常为 1Mbps */

/** 舵机 ID 配置 */
#define SERVO_ID_1          1           /**< 示例舵机1的ID */
#define SERVO_ID_2          2           /**< 示例舵机2的ID */
#define SERVO_BROADCAST     0xFE        /**< 广播ID：同时控制所有舵机 */

/**
 * SCSCL 系列位置范围说明：
 *   取值 0-1023，对应角度 0-300
 *   （例如：512 ~ 150 居中位置）
 * SMS_STS / HLS 系列位置范围说明：
 *   取值 0-4095，对应角度 0-360
 *   （例如：2048 ~ 180 居中位置）
 */

/*
=============================================================================
    以下为示例代码，一般无需修改
=============================================================================
*/

static const char *TAG = "ftservo_example";

/**
 * @brief 检测舵机是否在线
 */
static void example_ping(void)
{
    ESP_LOGI(TAG, "--- Ping 检测 ---");
    int ret = Ping(SERVO_ID_1);
    if (ret > 0) {
        ESP_LOGI(TAG, "舵机 ID=%d 在线，返回 ID=%d", SERVO_ID_1, ret);
    } else if (ret == 0) {
        ESP_LOGE(TAG, "舵机 ID=%d 无应答，请检查：", SERVO_ID_1);
        ESP_LOGE(TAG, "  1. 舵机供电是否正常（6-12V）");
        ESP_LOGE(TAG, "  2. TX/RX 接线是否正确");
        ESP_LOGE(TAG, "  3. 舵机 ID 是否匹配");
        ESP_LOGE(TAG, "  4. 波特率是否为 %d", FTSERVO_BAUD);
    } else {
        ESP_LOGE(TAG, "Ping 通信错误：%d", ret);
    }
}

/**
 * @brief 读取舵机状态反馈
 */
static void example_feedback(void)
{
    ESP_LOGI(TAG, "--- 读取舵机反馈 ---");

    int pos = ReadPos(SERVO_ID_1);
    if (pos >= 0) {
        ESP_LOGI(TAG, "舵机 ID=%d 位置 = %d", SERVO_ID_1, pos);
    } else {
        ESP_LOGE(TAG, "读取位置失败：%d", pos);
    }

    int temp = ReadTemper(SERVO_ID_1);
    if (temp >= 0) {
        ESP_LOGI(TAG, "舵机 ID=%d 温度 = %d C", SERVO_ID_1, temp);
    }

    int volt = ReadVoltage(SERVO_ID_1);
    if (volt >= 0) {
        ESP_LOGI(TAG, "舵机 ID=%d 电压 = %d.%02dV", SERVO_ID_1, volt / 100, volt % 100);
    }

    int load = ReadLoad(SERVO_ID_1);
    if (load >= 0) {
        ESP_LOGI(TAG, "舵机 ID=%d 负载 = %d", SERVO_ID_1, load);
    }

    int speed = ReadSpeed(SERVO_ID_1);
    if (speed >= 0) {
        ESP_LOGI(TAG, "舵机 ID=%d 速度 = %d", SERVO_ID_1, speed);
    }

    int moving = ReadMove(SERVO_ID_1);
    if (moving >= 0) {
        ESP_LOGI(TAG, "舵机 ID=%d 移动状态 = %s", SERVO_ID_1, moving ? "正在移动" : "已停止");
    }

    int current = ReadCurrent(SERVO_ID_1);
    if (current >= 0) {
        ESP_LOGI(TAG, "舵机 ID=%d 电流 = %dmA", SERVO_ID_1, current);
    }

    int ret = FeedBack(SERVO_ID_1);
    if (ret > 0) {
        int cached_pos = ReadPos(-1);
        ESP_LOGI(TAG, "  (缓存) 位置 = %d", cached_pos);
    }
}

/**
 * @brief 读取舵机型号和ID
 */
static void example_reg_rw(void)
{
    ESP_LOGI(TAG, "--- 读取舵机寄存器 ---");

    int model_l = readByte(SERVO_ID_1, SCSCL_MODEL_L);
    int model_h = readByte(SERVO_ID_1, SCSCL_MODEL_H);
    if (model_l >= 0 && model_h >= 0) {
        int model = model_l | (model_h << 8);
        ESP_LOGI(TAG, "舵机 ID=%d 型号 = 0x%04X", SERVO_ID_1, model);
    }

    int id = readByte(SERVO_ID_1, SCSCL_ID);
    if (id >= 0) {
        ESP_LOGI(TAG, "舵机 ID = %d", id);
    }

    int baud = readByte(SERVO_ID_1, SCSCL_BAUD_RATE);
    if (baud >= 0) {
        ESP_LOGI(TAG, "舵机 波特率寄存器 = %d", baud);
    }
}

/**
 * @brief SCSCL系列：单个舵机位置控制
 */
static void example_scscl_single_pos(void)
{
    ESP_LOGI(TAG, "--- SCSCL 单个位置控制 ---");

    WritePos(SERVO_ID_1, 512, 2000, 1000);
    ESP_LOGI(TAG, "舵机 %d -> 位置 512 (居中), 时间 2000ms", SERVO_ID_1);
    vTaskDelay(pdMS_TO_TICKS(2500));

    WritePos(SERVO_ID_1, 0, 1500, 500);
    ESP_LOGI(TAG, "舵机 %d -> 位置 0 (左极限), 时间 1500ms", SERVO_ID_1);
    vTaskDelay(pdMS_TO_TICKS(2000));

    WritePos(SERVO_ID_1, 1023, 1500, 500);
    ESP_LOGI(TAG, "舵机 %d -> 位置 1023 (右极限), 时间 1500ms", SERVO_ID_1);
    vTaskDelay(pdMS_TO_TICKS(2000));

    WritePos(SERVO_ID_1, 512, 2000, 1000);
    ESP_LOGI(TAG, "舵机 %d -> 位置 512 (回中), 时间 2000ms", SERVO_ID_1);
    vTaskDelay(pdMS_TO_TICKS(2500));
}

/**
 * @brief 广播控制所有舵机
 */
static void example_scscl_broadcast(void)
{
    ESP_LOGI(TAG, "--- 广播控制所有舵机 ---");

    WritePos(SERVO_BROADCAST, 512, 2000, 1000);
    ESP_LOGI(TAG, "所有舵机 -> 位置 512 (居中)");
    vTaskDelay(pdMS_TO_TICKS(2500));

    WritePos(SERVO_BROADCAST, 0, 1500, 500);
    ESP_LOGI(TAG, "所有舵机 -> 位置 0 (左极限)");
    vTaskDelay(pdMS_TO_TICKS(2000));

    WritePos(SERVO_BROADCAST, 1023, 1500, 500);
    ESP_LOGI(TAG, "所有舵机 -> 位置 1023 (右极限)");
    vTaskDelay(pdMS_TO_TICKS(2000));

    WritePos(SERVO_BROADCAST, 512, 2000, 1000);
    ESP_LOGI(TAG, "所有舵机 -> 位置 512 (回中)");
    vTaskDelay(pdMS_TO_TICKS(2500));
}

/**
 * @brief SCSCL系列：同步控制多个舵机
 */
static void example_scscl_sync(void)
{
    ESP_LOGI(TAG, "--- SCSCL 同步控制 ---");

    uint8_t id_list[2] = {SERVO_ID_1, SERVO_ID_2};
    uint16_t pos_list[2] = {0, 1023};
    uint16_t time_list[2] = {2000, 2000};
    uint16_t speed_list[2] = {1000, 1000};

    SyncWritePos(id_list, 2, pos_list, time_list, speed_list);
    ESP_LOGI(TAG, "舵机 %d -> 位置 0, 舵机 %d -> 位置 1023", SERVO_ID_1, SERVO_ID_2);
    vTaskDelay(pdMS_TO_TICKS(2500));

    pos_list[0] = 1023;
    pos_list[1] = 0;
    SyncWritePos(id_list, 2, pos_list, time_list, speed_list);
    ESP_LOGI(TAG, "舵机 %d -> 位置 1023, 舵机 %d -> 位置 0", SERVO_ID_1, SERVO_ID_2);
    vTaskDelay(pdMS_TO_TICKS(2500));
}

/**
 * @brief 扭矩开关示例
 */
static void example_torque(void)
{
    ESP_LOGI(TAG, "--- 扭矩控制 ---");

    int ret = EnableTorque(SERVO_ID_1, 1);
    if (ret > 0) {
        ESP_LOGI(TAG, "舵机 %d 扭矩已开启", SERVO_ID_1);
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    EnableTorque(SERVO_ID_1, 0);
    ESP_LOGI(TAG, "舵机 %d 扭矩已关闭（可手动转动）", SERVO_ID_1);
    vTaskDelay(pdMS_TO_TICKS(3000));

    EnableTorque(SERVO_ID_1, 1);
    ESP_LOGI(TAG, "舵机 %d 扭矩已恢复", SERVO_ID_1);
    vTaskDelay(pdMS_TO_TICKS(500));
}

/**
 * @brief SCSCL系列：PWM模式
 */
static void example_pwm_mode(void)
{
    ESP_LOGI(TAG, "--- SCSCL PWM 模式 ---");

    int ret = PWMMode(SERVO_ID_1);
    if (ret > 0) {
        ESP_LOGI(TAG, "舵机 %d 已切换为 PWM 模式", SERVO_ID_1);
    } else {
        ESP_LOGE(TAG, "切换 PWM 模式失败");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    for (int pwm = 100; pwm <= 500; pwm += 100) {
        WritePWM(SERVO_ID_1, pwm);
        ESP_LOGI(TAG, "PWM 输出 = %d", pwm);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    WritePWM(SERVO_ID_1, 0);
    ESP_LOGI(TAG, "PWM 停止");
}

/**
 * @brief SMS_STS系列：位置+加速度控制
 */
static void example_sms_sts_pos(void)
{
    ESP_LOGI(TAG, "--- SMS_STS 位置+加速度控制 ---");

    WritePosEx(SERVO_ID_1, 2048, 500, 50);
    ESP_LOGI(TAG, "舵机 %d -> 位置 2048 (居中), 速度 500, 加速度 50", SERVO_ID_1);
    vTaskDelay(pdMS_TO_TICKS(3000));

    WritePosEx(SERVO_ID_1, 0, 500, 100);
    ESP_LOGI(TAG, "舵机 %d -> 位置 0, 速度 500, 加速度 100", SERVO_ID_1);
    vTaskDelay(pdMS_TO_TICKS(3000));

    WritePosEx(SERVO_ID_1, 4095, 500, 100);
    ESP_LOGI(TAG, "舵机 %d -> 位置 4095, 速度 500, 加速度 100", SERVO_ID_1);
    vTaskDelay(pdMS_TO_TICKS(3000));

    WritePosEx(SERVO_ID_1, 2048, 500, 50);
    vTaskDelay(pdMS_TO_TICKS(3000));
}

/**
 * @brief SMS_STS系列：轮子模式
 */
static void example_wheel_mode(void)
{
    ESP_LOGI(TAG, "--- SMS_STS 轮子模式 ---");

    int ret = WheelMode(SERVO_ID_1);
    if (ret > 0) {
        ESP_LOGI(TAG, "舵机 %d 已切换为轮子模式", SERVO_ID_1);
    } else {
        ESP_LOGE(TAG, "切换轮子模式失败");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    WriteSpe(SERVO_ID_1, 500, 50);
    ESP_LOGI(TAG, "舵机 %d 正转, 速度 500", SERVO_ID_1);
    vTaskDelay(pdMS_TO_TICKS(3000));

    WriteSpe(SERVO_ID_1, 0, 50);
    ESP_LOGI(TAG, "舵机 %d 停止", SERVO_ID_1);
    vTaskDelay(pdMS_TO_TICKS(1000));

    WriteSpe(SERVO_ID_1, -500, 50);
    ESP_LOGI(TAG, "舵机 %d 反转, 速度 -500", SERVO_ID_1);
    vTaskDelay(pdMS_TO_TICKS(3000));

    WriteSpe(SERVO_ID_1, 0, 50);
    ESP_LOGI(TAG, "舵机 %d 停止", SERVO_ID_1);
}

/**
 * @brief HLS系列：位置+加速度+扭矩控制
 */
static void example_hls_pos(void)
{
    ESP_LOGI(TAG, "--- HLS 位置+加速度+扭矩控制 ---");

    WritePosEx2(SERVO_ID_1, 2048, 500, 50, 2048);
    ESP_LOGI(TAG, "舵机 %d -> 位置 2048 (居中), 速度 500, 加速度 50, 扭矩 2048", SERVO_ID_1);
    vTaskDelay(pdMS_TO_TICKS(3000));

    WritePosEx2(SERVO_ID_1, 0, 500, 100, 1024);
    ESP_LOGI(TAG, "舵机 %d -> 位置 0, 速度 500, 加速度 100, 扭矩 1024", SERVO_ID_1);
    vTaskDelay(pdMS_TO_TICKS(3000));

    WritePosEx2(SERVO_ID_1, 4095, 500, 100, 4095);
    ESP_LOGI(TAG, "舵机 %d -> 位置 4095, 速度 500, 加速度 100, 扭矩 4095", SERVO_ID_1);
    vTaskDelay(pdMS_TO_TICKS(3000));

    WritePosEx2(SERVO_ID_1, 2048, 500, 50, 2048);
    vTaskDelay(pdMS_TO_TICKS(3000));
}

/**
 * @brief HLS系列：速度+扭矩控制
 */
static void example_hls_speed(void)
{
    ESP_LOGI(TAG, "--- HLS 速度控制 ---");

    WriteSpeEx(SERVO_ID_1, 500, 50, 2048);
    ESP_LOGI(TAG, "舵机 %d 正转, 速度 500, 加速度 50, 扭矩 2048", SERVO_ID_1);
    vTaskDelay(pdMS_TO_TICKS(3000));

    WriteSpeEx(SERVO_ID_1, 0, 50, 2048);
    ESP_LOGI(TAG, "舵机 %d 停止", SERVO_ID_1);
    vTaskDelay(pdMS_TO_TICKS(1000));

    WriteSpeEx(SERVO_ID_1, -500, 50, 2048);
    ESP_LOGI(TAG, "舵机 %d 反转, 速度 -500", SERVO_ID_1);
    vTaskDelay(pdMS_TO_TICKS(3000));

    WriteSpeEx(SERVO_ID_1, 0, 50, 2048);
    ESP_LOGI(TAG, "舵机 %d 停止", SERVO_ID_1);
}

/**
 * @brief EEPROM 操作（注意：解锁后需要重新锁定！）
 */
static void example_eeprom(void)
{
    ESP_LOGI(TAG, "--- EEPROM 操作 ---");
    ESP_LOGW(TAG, "警告：修改 EEPROM 可能导致舵机异常，请谨慎操作！");

    int ret = unLockEprom(SERVO_ID_1);
    if (ret > 0) {
        ESP_LOGI(TAG, "舵机 %d EEPROM 已解锁", SERVO_ID_1);
    } else {
        ESP_LOGE(TAG, "解锁 EEPROM 失败");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    int current_id = readByte(SERVO_ID_1, SCSCL_ID);
    ESP_LOGI(TAG, "当前舵机 ID = %d", current_id);

    LockEprom(SERVO_ID_1);
    ESP_LOGI(TAG, "舵机 %d EEPROM 已锁定", SERVO_ID_1);
}

/**
 * ===================================================================
 *  主函数入口
 * ===================================================================
 *
 * 根据 SERVO_TYPE 的选择，运行对应的示例组合。
 * 取消注释对应的示例函数即可运行。
 *
 * 接线确认：
 *   1. 舵机 DATA 线 -> TX=%d, RX=%d
 *   2. 舵机 GND -> ESP32 GND（共地）
 *   3. 舵机 VCC -> 6-12V 外部电源
 *   （注意：舵机电流较大，不要从 ESP32 的 USB 口取电！）
 */
void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "FEETECH 总线舵机示例程序");
    ESP_LOGI(TAG, "舵机类型: %d", SERVO_TYPE);
    ESP_LOGI(TAG, "引脚: TX=%d, RX=%d, RTS=%d", FTSERVO_TX_PIN, FTSERVO_RX_PIN, FTSERVO_RTS_PIN);
    ESP_LOGI(TAG, "波特率: %d", FTSERVO_BAUD);
    ESP_LOGI(TAG, "========================================");

    int ret = ftServo_InitWithType(SERVO_TYPE, FTSERVO_TX_PIN, FTSERVO_RX_PIN, FTSERVO_RTS_PIN, FTSERVO_BAUD);
    if (ret != 0) {
        ESP_LOGE(TAG, "舵机初始化失败！");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    /* --- 基础示例（所有系列通用） --- */
    example_ping();
    vTaskDelay(pdMS_TO_TICKS(1000));

    example_feedback();
    vTaskDelay(pdMS_TO_TICKS(1000));

    example_reg_rw();
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* --- SCSCL 系列示例（SERVO_SCSCL） --- */
    example_scscl_single_pos();
    example_scscl_broadcast();
    example_scscl_sync();
    example_torque();
    // example_pwm_mode();

    /* --- SMS_STS 系列示例（SERVO_SMS_STS） --- */
    // example_sms_sts_pos();
    // example_wheel_mode();

    /* --- HLS 系列示例（SERVO_HLS） --- */
    // example_hls_pos();
    // example_hls_speed();

    /* --- EEPROM 操作（小心使用！） --- */
    // example_eeprom();

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "所有示例执行完毕");
    ESP_LOGI(TAG, "========================================");
}
