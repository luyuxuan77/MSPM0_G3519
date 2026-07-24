/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G351X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G351X
#define CONFIG_MSPM0G3519

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define GPIO_HFXT_PORT                                                     GPIOA
#define GPIO_HFXIN_PIN                                             DL_GPIO_PIN_5
#define GPIO_HFXIN_IOMUX                                         (IOMUX_PINCM10)
#define GPIO_HFXOUT_PIN                                            DL_GPIO_PIN_6
#define GPIO_HFXOUT_IOMUX                                        (IOMUX_PINCM11)
#define CPUCLK_FREQ                                                     80000000
/* Defines for SYSPLL_ERR_01 Workaround */
/* Represent 1.000 as 1000 */
#define FLOAT_TO_INT_SCALE                                               (1000U)
#define FCC_EXPECTED_RATIO                                                  2000
#define FCC_UPPER_BOUND                       (FCC_EXPECTED_RATIO * (1 + 0.003))
#define FCC_LOWER_BOUND                       (FCC_EXPECTED_RATIO * (1 - 0.003))

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);


/* Defines for TimerG6_PWM */
#define TimerG6_PWM_INST                                                   TIMG6
#define TimerG6_PWM_INST_IRQHandler                             TIMG6_IRQHandler
#define TimerG6_PWM_INST_INT_IRQN                               (TIMG6_INT_IRQn)
#define TimerG6_PWM_INST_CLK_FREQ                                       80000000
/* GPIO defines for channel 0 */
#define GPIO_TimerG6_PWM_C0_PORT                                           GPIOA
#define GPIO_TimerG6_PWM_C0_PIN                                   DL_GPIO_PIN_29
#define GPIO_TimerG6_PWM_C0_IOMUX                                 (IOMUX_PINCM4)
#define GPIO_TimerG6_PWM_C0_IOMUX_FUNC                IOMUX_PINCM4_PF_TIMG6_CCP0
#define GPIO_TimerG6_PWM_C0_IDX                              DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_TimerG6_PWM_C1_PORT                                           GPIOA
#define GPIO_TimerG6_PWM_C1_PIN                                   DL_GPIO_PIN_30
#define GPIO_TimerG6_PWM_C1_IOMUX                                 (IOMUX_PINCM5)
#define GPIO_TimerG6_PWM_C1_IOMUX_FUNC                IOMUX_PINCM5_PF_TIMG6_CCP1
#define GPIO_TimerG6_PWM_C1_IDX                              DL_TIMER_CC_1_INDEX

/* Defines for TIMG0_Buzzer */
#define TIMG0_Buzzer_INST                                                  TIMG0
#define TIMG0_Buzzer_INST_IRQHandler                            TIMG0_IRQHandler
#define TIMG0_Buzzer_INST_INT_IRQN                              (TIMG0_INT_IRQn)
#define TIMG0_Buzzer_INST_CLK_FREQ                                      40000000
/* GPIO defines for channel 1 */
#define GPIO_TIMG0_Buzzer_C1_PORT                                          GPIOB
#define GPIO_TIMG0_Buzzer_C1_PIN                                  DL_GPIO_PIN_11
#define GPIO_TIMG0_Buzzer_C1_IOMUX                               (IOMUX_PINCM28)
#define GPIO_TIMG0_Buzzer_C1_IOMUX_FUNC              IOMUX_PINCM28_PF_TIMG0_CCP1
#define GPIO_TIMG0_Buzzer_C1_IDX                             DL_TIMER_CC_1_INDEX

/* Defines for TIMG14_Motor */
#define TIMG14_Motor_INST                                                 TIMG14
#define TIMG14_Motor_INST_IRQHandler                           TIMG14_IRQHandler
#define TIMG14_Motor_INST_INT_IRQN                             (TIMG14_INT_IRQn)
#define TIMG14_Motor_INST_CLK_FREQ                                      40000000
/* GPIO defines for channel 0 */
#define GPIO_TIMG14_Motor_C0_PORT                                          GPIOB
#define GPIO_TIMG14_Motor_C0_PIN                                  DL_GPIO_PIN_28
#define GPIO_TIMG14_Motor_C0_IOMUX                               (IOMUX_PINCM65)
#define GPIO_TIMG14_Motor_C0_IOMUX_FUNC             IOMUX_PINCM65_PF_TIMG14_CCP0
#define GPIO_TIMG14_Motor_C0_IDX                             DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_TIMG14_Motor_C1_PORT                                          GPIOB
#define GPIO_TIMG14_Motor_C1_PIN                                  DL_GPIO_PIN_13
#define GPIO_TIMG14_Motor_C1_IOMUX                               (IOMUX_PINCM30)
#define GPIO_TIMG14_Motor_C1_IOMUX_FUNC             IOMUX_PINCM30_PF_TIMG14_CCP1
#define GPIO_TIMG14_Motor_C1_IDX                             DL_TIMER_CC_1_INDEX



/* Defines for TimerA1 */
#define TimerA1_INST                                                     (TIMA1)
#define TimerA1_INST_IRQHandler                                 TIMA1_IRQHandler
#define TimerA1_INST_INT_IRQN                                   (TIMA1_INT_IRQn)
#define TimerA1_INST_LOAD_VALUE                                             (9U)



/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                           20000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_20_MHZ_115200_BAUD                                      (10)
#define UART_0_FBRD_20_MHZ_115200_BAUD                                      (54)
/* Defines for UART_1 */
#define UART_1_INST                                                        UART1
#define UART_1_INST_FREQUENCY                                           20000000
#define UART_1_INST_IRQHandler                                  UART1_IRQHandler
#define UART_1_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_1_RX_PORT                                                GPIOA
#define GPIO_UART_1_TX_PORT                                                GPIOA
#define GPIO_UART_1_RX_PIN                                         DL_GPIO_PIN_9
#define GPIO_UART_1_TX_PIN                                         DL_GPIO_PIN_8
#define GPIO_UART_1_IOMUX_RX                                     (IOMUX_PINCM20)
#define GPIO_UART_1_IOMUX_TX                                     (IOMUX_PINCM19)
#define GPIO_UART_1_IOMUX_RX_FUNC                      IOMUX_PINCM20_PF_UART1_RX
#define GPIO_UART_1_IOMUX_TX_FUNC                      IOMUX_PINCM19_PF_UART1_TX
#define UART_1_BAUD_RATE                                                (115200)
#define UART_1_IBRD_20_MHZ_115200_BAUD                                      (10)
#define UART_1_FBRD_20_MHZ_115200_BAUD                                      (54)
/* Defines for UART_4 */
#define UART_4_INST                                                        UART4
#define UART_4_INST_FREQUENCY                                           40000000
#define UART_4_INST_IRQHandler                                  UART4_IRQHandler
#define UART_4_INST_INT_IRQN                                      UART4_INT_IRQn
#define GPIO_UART_4_RX_PORT                                                GPIOB
#define GPIO_UART_4_TX_PORT                                                GPIOB
#define GPIO_UART_4_RX_PIN                                        DL_GPIO_PIN_18
#define GPIO_UART_4_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_4_IOMUX_RX                                     (IOMUX_PINCM44)
#define GPIO_UART_4_IOMUX_TX                                     (IOMUX_PINCM27)
#define GPIO_UART_4_IOMUX_RX_FUNC                      IOMUX_PINCM44_PF_UART4_RX
#define GPIO_UART_4_IOMUX_TX_FUNC                      IOMUX_PINCM27_PF_UART4_TX
#define UART_4_BAUD_RATE                                                (115200)
#define UART_4_IBRD_40_MHZ_115200_BAUD                                      (21)
#define UART_4_FBRD_40_MHZ_115200_BAUD                                      (45)




/* Defines for SPI_0 */
#define SPI_0_INST                                                         SPI0
#define SPI_0_INST_IRQHandler                                   SPI0_IRQHandler
#define SPI_0_INST_INT_IRQN                                       SPI0_INT_IRQn
#define GPIO_SPI_0_PICO_PORT                                              GPIOB
#define GPIO_SPI_0_PICO_PIN                                       DL_GPIO_PIN_2
#define GPIO_SPI_0_IOMUX_PICO                                   (IOMUX_PINCM15)
#define GPIO_SPI_0_IOMUX_PICO_FUNC                   IOMUX_PINCM15_PF_SPI0_PICO
/* GPIO configuration for SPI_0 */
#define GPIO_SPI_0_SCLK_PORT                                              GPIOB
#define GPIO_SPI_0_SCLK_PIN                                       DL_GPIO_PIN_3
#define GPIO_SPI_0_IOMUX_SCLK                                   (IOMUX_PINCM16)
#define GPIO_SPI_0_IOMUX_SCLK_FUNC                   IOMUX_PINCM16_PF_SPI0_SCLK
/* Defines for SPI_1 */
#define SPI_1_INST                                                         SPI1
#define SPI_1_INST_IRQHandler                                   SPI1_IRQHandler
#define SPI_1_INST_INT_IRQN                                       SPI1_INT_IRQn
#define GPIO_SPI_1_PICO_PORT                                              GPIOB
#define GPIO_SPI_1_PICO_PIN                                      DL_GPIO_PIN_15
#define GPIO_SPI_1_IOMUX_PICO                                   (IOMUX_PINCM32)
#define GPIO_SPI_1_IOMUX_PICO_FUNC                   IOMUX_PINCM32_PF_SPI1_PICO
#define GPIO_SPI_1_POCI_PORT                                              GPIOB
#define GPIO_SPI_1_POCI_PIN                                      DL_GPIO_PIN_14
#define GPIO_SPI_1_IOMUX_POCI                                   (IOMUX_PINCM31)
#define GPIO_SPI_1_IOMUX_POCI_FUNC                   IOMUX_PINCM31_PF_SPI1_POCI
/* GPIO configuration for SPI_1 */
#define GPIO_SPI_1_SCLK_PORT                                              GPIOB
#define GPIO_SPI_1_SCLK_PIN                                      DL_GPIO_PIN_16
#define GPIO_SPI_1_IOMUX_SCLK                                   (IOMUX_PINCM33)
#define GPIO_SPI_1_IOMUX_SCLK_FUNC                   IOMUX_PINCM33_PF_SPI1_SCLK



/* Port definition for Pin Group key */
#define key_PORT                                                         (GPIOB)

/* Defines for user: GPIOB.31 with pinCMx 68 on package pin 27 */
// pins affected by this interrupt request:["user"]
#define key_INT_IRQN                                            (GPIOB_INT_IRQn)
#define key_INT_IIDX                            (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define key_user_IIDX                                       (DL_GPIO_IIDX_DIO31)
#define key_user_PIN                                            (DL_GPIO_PIN_31)
#define key_user_IOMUX                                           (IOMUX_PINCM68)
/* Port definition for Pin Group IMU */
#define IMU_PORT                                                         (GPIOB)

/* Defines for CS_IMU: GPIOB.12 with pinCMx 29 on package pin 36 */
#define IMU_CS_IMU_PIN                                          (DL_GPIO_PIN_12)
#define IMU_CS_IMU_IOMUX                                         (IOMUX_PINCM29)
/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOA)

/* Defines for L1: GPIOA.14 with pinCMx 36 on package pin 43 */
#define LED_L1_PIN                                              (DL_GPIO_PIN_14)
#define LED_L1_IOMUX                                             (IOMUX_PINCM36)
/* Defines for L2: GPIOA.17 with pinCMx 39 on package pin 54 */
#define LED_L2_PIN                                              (DL_GPIO_PIN_17)
#define LED_L2_IOMUX                                             (IOMUX_PINCM39)
/* Defines for RES: GPIOB.23 with pinCMx 51 on package pin 70 */
#define OLED_RES_PORT                                                    (GPIOB)
#define OLED_RES_PIN                                            (DL_GPIO_PIN_23)
#define OLED_RES_IOMUX                                           (IOMUX_PINCM51)
/* Defines for DC: GPIOC.8 with pinCMx 86 on package pin 65 */
#define OLED_DC_PORT                                                     (GPIOC)
#define OLED_DC_PIN                                              (DL_GPIO_PIN_8)
#define OLED_DC_IOMUX                                            (IOMUX_PINCM86)
/* Defines for CS: GPIOC.9 with pinCMx 87 on package pin 66 */
#define OLED_CS_PORT                                                     (GPIOC)
#define OLED_CS_PIN                                              (DL_GPIO_PIN_9)
#define OLED_CS_IOMUX                                            (IOMUX_PINCM87)
/* Defines for M0_PH: GPIOA.31 with pinCMx 6 on package pin 7 */
#define Motor_DIR_M0_PH_PORT                                             (GPIOA)
#define Motor_DIR_M0_PH_PIN                                     (DL_GPIO_PIN_31)
#define Motor_DIR_M0_PH_IOMUX                                     (IOMUX_PINCM6)
/* Defines for M1_PH: GPIOB.0 with pinCMx 12 on package pin 15 */
#define Motor_DIR_M1_PH_PORT                                             (GPIOB)
#define Motor_DIR_M1_PH_PIN                                      (DL_GPIO_PIN_0)
#define Motor_DIR_M1_PH_IOMUX                                    (IOMUX_PINCM12)
/* Defines for R1: GPIOC.0 with pinCMx 74 on package pin 46 */
#define MatrixKeypad_R1_PORT                                             (GPIOC)
#define MatrixKeypad_R1_PIN                                      (DL_GPIO_PIN_0)
#define MatrixKeypad_R1_IOMUX                                    (IOMUX_PINCM74)
/* Defines for R2: GPIOC.1 with pinCMx 75 on package pin 47 */
#define MatrixKeypad_R2_PORT                                             (GPIOC)
#define MatrixKeypad_R2_PIN                                      (DL_GPIO_PIN_1)
#define MatrixKeypad_R2_IOMUX                                    (IOMUX_PINCM75)
/* Defines for R3: GPIOC.2 with pinCMx 76 on package pin 50 */
#define MatrixKeypad_R3_PORT                                             (GPIOC)
#define MatrixKeypad_R3_PIN                                      (DL_GPIO_PIN_2)
#define MatrixKeypad_R3_IOMUX                                    (IOMUX_PINCM76)
/* Defines for R4: GPIOC.3 with pinCMx 77 on package pin 51 */
#define MatrixKeypad_R4_PORT                                             (GPIOC)
#define MatrixKeypad_R4_PIN                                      (DL_GPIO_PIN_3)
#define MatrixKeypad_R4_IOMUX                                    (IOMUX_PINCM77)
/* Defines for C1: GPIOC.4 with pinCMx 78 on package pin 52 */
#define MatrixKeypad_C1_PORT                                             (GPIOC)
#define MatrixKeypad_C1_PIN                                      (DL_GPIO_PIN_4)
#define MatrixKeypad_C1_IOMUX                                    (IOMUX_PINCM78)
/* Defines for C2: GPIOC.5 with pinCMx 79 on package pin 53 */
#define MatrixKeypad_C2_PORT                                             (GPIOC)
#define MatrixKeypad_C2_PIN                                      (DL_GPIO_PIN_5)
#define MatrixKeypad_C2_IOMUX                                    (IOMUX_PINCM79)
/* Defines for C3: GPIOB.21 with pinCMx 49 on package pin 68 */
#define MatrixKeypad_C3_PORT                                             (GPIOB)
#define MatrixKeypad_C3_PIN                                     (DL_GPIO_PIN_21)
#define MatrixKeypad_C3_IOMUX                                    (IOMUX_PINCM49)
/* Defines for C4: GPIOC.7 with pinCMx 85 on package pin 64 */
#define MatrixKeypad_C4_PORT                                             (GPIOC)
#define MatrixKeypad_C4_PIN                                      (DL_GPIO_PIN_7)
#define MatrixKeypad_C4_IOMUX                                    (IOMUX_PINCM85)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);
void SYSCFG_DL_TimerG6_PWM_init(void);
void SYSCFG_DL_TIMG0_Buzzer_init(void);
void SYSCFG_DL_TIMG14_Motor_init(void);
void SYSCFG_DL_TimerA1_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_UART_1_init(void);
void SYSCFG_DL_UART_4_init(void);
void SYSCFG_DL_SPI_0_init(void);
void SYSCFG_DL_SPI_1_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
