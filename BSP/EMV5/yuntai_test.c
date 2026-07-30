#include "bsp.h"
#include "yuntai.h"

/*===========================================================================
 * 云台基础测试：使能、复位、简单移动
 * 这个函数测试云台的基本功能，适合首次测试
 *===========================================================================*/
void yuntai_test_basic(void)
{
    printf("\r\n========== 云台基础测试开始 ==========\r\n");

    // 步骤1：初始化UART3
    printf("[1/6] 初始化UART3...\r\n");
    uart3_init(115200);
    delay_ms(500);
    printf("[1/6] UART3初始化完成\r\n");

    // 步骤2：使能水平电机
    printf("[2/6] 使能水平电机（地址0x01）...\r\n");
    Emm_V5_En_Control(MOTOR_YAW_ADDR, true, false);
    delay_ms(100);
    printf("[2/6] 水平电机已使能\r\n");

    // 步骤3：使能俯仰电机
    printf("[3/6] 使能俯仰电机（地址0x02）...\r\n");
    Emm_V5_En_Control(MOTOR_PITCH_ADDR, true, false);
    delay_ms(100);
    printf("[3/6] 俯仰电机已使能\r\n");

    // 步骤4：复位当前位置为零
    printf("[4/6] 复位位置...\r\n");
    Emm_V5_Reset_CurPos_To_Zero(MOTOR_YAW_ADDR);
    delay_ms(50);
    Emm_V5_Reset_CurPos_To_Zero(MOTOR_PITCH_ADDR);
    delay_ms(50);
    printf("[4/6] 位置已复位\r\n");

    // 步骤5：测试水平电机转动
    printf("[5/6] 测试水平电机转动...\r\n");
    printf("      - 向右转30度（约320脉冲）...\r\n");
    Emm_V5_Pos_Control(MOTOR_YAW_ADDR, 0, 200, 30, 320, false, false);
    delay_ms(2000);  // 等待运动完成

    printf("      - 回到中间...\r\n");
    Emm_V5_Pos_Control(MOTOR_YAW_ADDR, 1, 200, 30, 320, false, false);
    delay_ms(2000);

    // 步骤6：测试俯仰电机转动
    printf("[6/6] 测试俯仰电机转动...\r\n");
    printf("      - 向上转15度（约160脉冲）...\r\n");
    Emm_V5_Pos_Control(MOTOR_PITCH_ADDR, 0, 200, 30, 160, false, false);
    delay_ms(2000);

    printf("      - 回到中间...\r\n");
    Emm_V5_Pos_Control(MOTOR_PITCH_ADDR, 1, 200, 30, 160, false, false);
    delay_ms(2000);

    // 失能电机
    printf("[完成] 失能电机...\r\n");
    Emm_V5_En_Control(MOTOR_YAW_ADDR, false, false);
    Emm_V5_En_Control(MOTOR_PITCH_ADDR, false, false);

    printf("========== 云台基础测试完成 ==========\r\n\r\n");
}

/*===========================================================================
 * 云台扫描测试：水平和垂直扫描（使用高级API）
 *===========================================================================*/
void yuntai_test_scan_simple(void)
{
    printf("\r\n========== 云台扫描测试开始 ==========\r\n");

    // 初始化云台控制
    yuntai_control_init();
    delay_ms(100);

    // 使能电机
    yuntai_enable(true);
    delay_ms(100);

    // 复位位置
    yuntai_reset_position();
    delay_ms(500);

    // 水平扫描测试
    printf("--- 水平扫描测试 ---\r\n");
    yuntai_move_to_angle(30.0f, 0.0f);   // 右转30度
    delay_ms(3000);
    yuntai_move_to_angle(-30.0f, 0.0f);  // 左转30度
    delay_ms(3000);
    yuntai_move_to_angle(0.0f, 0.0f);    // 回中间
    delay_ms(2000);

    // 垂直扫描测试
    printf("--- 垂直扫描测试 ---\r\n");
    yuntai_move_to_angle(0.0f, 20.0f);   // 向上20度
    delay_ms(3000);
    yuntai_move_to_angle(0.0f, -20.0f);  // 向下20度
    delay_ms(3000);
    yuntai_move_to_angle(0.0f, 0.0f);    // 回中间
    delay_ms(2000);

    // 失能电机
    yuntai_enable(false);

    printf("========== 云台扫描测试完成 ==========\r\n\r\n");
}

/*===========================================================================
 * 云台单电机测试：只测试一个电机
 * motor_addr: 电机地址（0x01=水平, 0x02=俯仰）
 * angle: 转动角度（度）
 *===========================================================================*/
void yuntai_test_single_motor(uint8_t motor_addr, float angle)
{
    uint8_t dir;
    uint32_t pulses;

    printf("\r\n========== 单电机测试 ==========\r\n");
    printf("电机地址: 0x%02X\r\n", motor_addr);
    printf("目标角度: %.2f 度\r\n", angle);

    // 初始化UART3
    uart3_init(115200);
    delay_ms(100);

    // 使能电机
    printf("使能电机...\r\n");
    Emm_V5_En_Control(motor_addr, true, false);
    delay_ms(100);

    // 复位位置
    printf("复位位置...\r\n");
    Emm_V5_Reset_CurPos_To_Zero(motor_addr);
    delay_ms(100);

    // 计算方向和脉冲数
    if (angle >= 0.0f) {
        dir = 0;  // 顺时针
    } else {
        dir = 1;  // 逆时针
        angle = -angle;
    }
    pulses = (uint32_t)(angle * 17.78f);  // 6400脉冲/360度 = 17.78脉冲/度

    // 发送运动命令
    printf("开始运动: 方向=%d, 脉冲数=%lu\r\n", dir, pulses);
    Emm_V5_Pos_Control(motor_addr, dir, 200, 30, pulses, false, false);

    // 等待运动完成
    printf("等待运动完成...\r\n");
    delay_ms(3000);

    // 失能电机
    printf("失能电机...\r\n");
    Emm_V5_En_Control(motor_addr, false, false);

    printf("========== 测试完成 ==========\r\n\r\n");
}

/*===========================================================================
 * 快速测试：最小化测试，只转动一个电机一次
 *===========================================================================*/
void yuntai_test_quick(void)
{
    printf("\r\n========== 快速测试 ==========\r\n");

    // 初始化UART3
    printf("初始化UART3...\r\n");
    uart3_init(115200U);
    delay_ms(200);

    // 使能水平电机（地址0x01）
    printf("使能电机...\r\n");
    Emm_V5_En_Control(0x01, true, false);
    delay_ms(200);

    // 复位位置
    printf("复位位置...\r\n");
    Emm_V5_Reset_CurPos_To_Zero(0x01);
    delay_ms(200);

    // 转动45度（约800脉冲）
    printf("转动45度...\r\n");
    Emm_V5_Pos_Control(0x01, 0, 300, 50, 800, false, false);
    delay_ms(3000);  // 等待3秒

    // 转回来
    printf("转回原位...\r\n");
    Emm_V5_Pos_Control(0x01, 1, 300, 50, 800, false, false);
    delay_ms(3000);

    // 失能电机
    printf("失能电机...\r\n");
    Emm_V5_En_Control(0x01, false, false);

    printf("========== 测试完成 ==========\r\n\r\n");
}
