/*
 * G3519 小车板全外设页面式测试主程序。
 *
 * 职责划分：
 * - 本文件只做系统启动顺序控制：SysConfig、调试串口、1ms 系统时基、按键中断、
 *   以及应用层外设测试框架。
 * - 具体外设测试逻辑放在 User/App/peripheral_test.c，避免 main.c 继续膨胀。
 *
 * 操作方式：
 * - 上电后显示所有已接入外设的初始化/在线情况，并同步从 UART0 打印。
 * - K1 切换测试页面。
 * - K2 执行当前页面动作，例如切 RGB/蜂鸣器、电机步骤、舵机目标、CAN 发送、
 *   WiFi/蓝牙 AT 自检等。
 */
#include "ti_msp_dl_config.h"
#include "bsp.h"
#include "App/peripheral_test.h"

int main(void)
{
    /*
     * 启动顺序说明：
     * 1. SYSCFG_DL_init() 完成所有由 SysConfig 管理的时钟、GPIO、UART、SPI、
     *    PWM、QEI、ADC、CAN 和 VREF 基础配置。
     * 2. uart0_init() 打开调试串口中断和 printf 重定向，后续所有状态都可从串口看到。
     * 3. system_time_init() 启动 TIMA1 1ms 节拍，按键消抖和页面刷新都依赖该时基。
     * 4. user_key_init() 打开 K1/K2 GPIO 中断，但具体动作在主循环中轮询消费。
     * 5. peripheral_test_init() 初始化各 BSP 驱动并进入“外设总览页”。
     */
    SYSCFG_DL_init();
    uart0_init(115200U);
    printf("\r\nG3519 Car app (g3519_car)\r\n");

    system_time_init();
    user_key_init();
    peripheral_test_init();

    while (1) {
        peripheral_test_task();
    }
}
