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
#define GPIO_LFXT_PORT                                                     GPIOA
#define GPIO_LFXIN_PIN                                             DL_GPIO_PIN_3
#define GPIO_LFXIN_IOMUX                                          (IOMUX_PINCM8)
#define GPIO_LFXOUT_PIN                                            DL_GPIO_PIN_4
#define GPIO_LFXOUT_IOMUX                                         (IOMUX_PINCM9)
#define CPUCLK_FREQ                                                     80000000



/* Defines for MOTOR_PWM */
#define MOTOR_PWM_INST                                                    TIMG14
#define MOTOR_PWM_INST_IRQHandler                              TIMG14_IRQHandler
#define MOTOR_PWM_INST_INT_IRQN                                (TIMG14_INT_IRQn)
#define MOTOR_PWM_INST_CLK_FREQ                                         40000000
/* GPIO defines for channel 0 */
#define GPIO_MOTOR_PWM_C0_PORT                                             GPIOA
#define GPIO_MOTOR_PWM_C0_PIN                                     DL_GPIO_PIN_29
#define GPIO_MOTOR_PWM_C0_IOMUX                                   (IOMUX_PINCM4)
#define GPIO_MOTOR_PWM_C0_IOMUX_FUNC                 IOMUX_PINCM4_PF_TIMG14_CCP0
#define GPIO_MOTOR_PWM_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_MOTOR_PWM_C1_PORT                                             GPIOA
#define GPIO_MOTOR_PWM_C1_PIN                                     DL_GPIO_PIN_30
#define GPIO_MOTOR_PWM_C1_IOMUX                                   (IOMUX_PINCM5)
#define GPIO_MOTOR_PWM_C1_IOMUX_FUNC                 IOMUX_PINCM5_PF_TIMG14_CCP1
#define GPIO_MOTOR_PWM_C1_IDX                                DL_TIMER_CC_1_INDEX
/* GPIO defines for channel 2 */
#define GPIO_MOTOR_PWM_C2_PORT                                             GPIOA
#define GPIO_MOTOR_PWM_C2_PIN                                     DL_GPIO_PIN_28
#define GPIO_MOTOR_PWM_C2_IOMUX                                   (IOMUX_PINCM3)
#define GPIO_MOTOR_PWM_C2_IOMUX_FUNC                 IOMUX_PINCM3_PF_TIMG14_CCP2
#define GPIO_MOTOR_PWM_C2_IDX                                DL_TIMER_CC_2_INDEX
/* GPIO defines for channel 3 */
#define GPIO_MOTOR_PWM_C3_PORT                                             GPIOB
#define GPIO_MOTOR_PWM_C3_PIN                                      DL_GPIO_PIN_1
#define GPIO_MOTOR_PWM_C3_IOMUX                                  (IOMUX_PINCM13)
#define GPIO_MOTOR_PWM_C3_IOMUX_FUNC                IOMUX_PINCM13_PF_TIMG14_CCP3
#define GPIO_MOTOR_PWM_C3_IDX                                DL_TIMER_CC_3_INDEX

/* Defines for PWM_1 */
#define PWM_1_INST                                                        TIMG12
#define PWM_1_INST_IRQHandler                                  TIMG12_IRQHandler
#define PWM_1_INST_INT_IRQN                                    (TIMG12_INT_IRQn)
#define PWM_1_INST_CLK_FREQ                                             80000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_1_C0_PORT                                                 GPIOA
#define GPIO_PWM_1_C0_PIN                                         DL_GPIO_PIN_15
#define GPIO_PWM_1_C0_IOMUX                                      (IOMUX_PINCM37)
#define GPIO_PWM_1_C0_IOMUX_FUNC                    IOMUX_PINCM37_PF_TIMG12_CCP0
#define GPIO_PWM_1_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_1_C1_PORT                                                 GPIOA
#define GPIO_PWM_1_C1_PIN                                         DL_GPIO_PIN_16
#define GPIO_PWM_1_C1_IOMUX                                      (IOMUX_PINCM38)
#define GPIO_PWM_1_C1_IOMUX_FUNC                    IOMUX_PINCM38_PF_TIMG12_CCP1
#define GPIO_PWM_1_C1_IDX                                    DL_TIMER_CC_1_INDEX




/* Defines for QEI_M0 */
#define QEI_M0_INST                                                        TIMG9
#define QEI_M0_INST_IRQHandler                                  TIMG9_IRQHandler
#define QEI_M0_INST_INT_IRQN                                    (TIMG9_INT_IRQn)
/* Pin configuration defines for QEI_M0 PHA Pin */
#define GPIO_QEI_M0_PHA_PORT                                               GPIOB
#define GPIO_QEI_M0_PHA_PIN                                       DL_GPIO_PIN_29
#define GPIO_QEI_M0_PHA_IOMUX                                    (IOMUX_PINCM66)
#define GPIO_QEI_M0_PHA_IOMUX_FUNC                   IOMUX_PINCM66_PF_TIMG9_CCP0
/* Pin configuration defines for QEI_M0 PHB Pin */
#define GPIO_QEI_M0_PHB_PORT                                               GPIOB
#define GPIO_QEI_M0_PHB_PIN                                       DL_GPIO_PIN_30
#define GPIO_QEI_M0_PHB_IOMUX                                    (IOMUX_PINCM67)
#define GPIO_QEI_M0_PHB_IOMUX_FUNC                   IOMUX_PINCM67_PF_TIMG9_CCP1

/* Defines for QEI_M1 */
#define QEI_M1_INST                                                        TIMG8
#define QEI_M1_INST_IRQHandler                                  TIMG8_IRQHandler
#define QEI_M1_INST_INT_IRQN                                    (TIMG8_INT_IRQn)
/* Pin configuration defines for QEI_M1 PHA Pin */
#define GPIO_QEI_M1_PHA_PORT                                               GPIOC
#define GPIO_QEI_M1_PHA_PIN                                        DL_GPIO_PIN_6
#define GPIO_QEI_M1_PHA_IOMUX                                    (IOMUX_PINCM84)
#define GPIO_QEI_M1_PHA_IOMUX_FUNC                   IOMUX_PINCM84_PF_TIMG8_CCP0
/* Pin configuration defines for QEI_M1 PHB Pin */
#define GPIO_QEI_M1_PHB_PORT                                               GPIOA
#define GPIO_QEI_M1_PHB_PIN                                       DL_GPIO_PIN_22
#define GPIO_QEI_M1_PHB_IOMUX                                    (IOMUX_PINCM47)
#define GPIO_QEI_M1_PHB_IOMUX_FUNC                   IOMUX_PINCM47_PF_TIMG8_CCP1


/* Defines for TimerA1 */
#define TimerA1_INST                                                     (TIMA1)
#define TimerA1_INST_IRQHandler                                 TIMA1_IRQHandler
#define TimerA1_INST_INT_IRQN                                   (TIMA1_INT_IRQn)
#define TimerA1_INST_LOAD_VALUE                                           (999U)



/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                           40000000
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
#define UART_0_IBRD_40_MHZ_115200_BAUD                                      (21)
#define UART_0_FBRD_40_MHZ_115200_BAUD                                      (45)
/* Defines for UART_4 */
#define UART_4_INST                                                        UART4
#define UART_4_INST_FREQUENCY                                           80000000
#define UART_4_INST_IRQHandler                                  UART4_IRQHandler
#define UART_4_INST_INT_IRQN                                      UART4_INT_IRQn
#define GPIO_UART_4_RX_PORT                                                GPIOB
#define GPIO_UART_4_TX_PORT                                                GPIOB
#define GPIO_UART_4_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_4_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_4_IOMUX_RX                                     (IOMUX_PINCM28)
#define GPIO_UART_4_IOMUX_TX                                     (IOMUX_PINCM27)
#define GPIO_UART_4_IOMUX_RX_FUNC                      IOMUX_PINCM28_PF_UART4_RX
#define GPIO_UART_4_IOMUX_TX_FUNC                      IOMUX_PINCM27_PF_UART4_TX
#define UART_4_BAUD_RATE                                                (115200)
#define UART_4_IBRD_80_MHZ_115200_BAUD                                      (43)
#define UART_4_FBRD_80_MHZ_115200_BAUD                                      (26)




/* Defines for SPI_0 */
#define SPI_0_INST                                                         SPI0
#define SPI_0_INST_IRQHandler                                   SPI0_IRQHandler
#define SPI_0_INST_INT_IRQN                                       SPI0_INT_IRQn
#define GPIO_SPI_0_PICO_PORT                                              GPIOB
#define GPIO_SPI_0_PICO_PIN                                      DL_GPIO_PIN_17
#define GPIO_SPI_0_IOMUX_PICO                                   (IOMUX_PINCM43)
#define GPIO_SPI_0_IOMUX_PICO_FUNC                   IOMUX_PINCM43_PF_SPI0_PICO
/* GPIO configuration for SPI_0 */
#define GPIO_SPI_0_SCLK_PORT                                              GPIOB
#define GPIO_SPI_0_SCLK_PIN                                      DL_GPIO_PIN_18
#define GPIO_SPI_0_IOMUX_SCLK                                   (IOMUX_PINCM44)
#define GPIO_SPI_0_IOMUX_SCLK_FUNC                   IOMUX_PINCM44_PF_SPI0_SCLK
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



/* Defines for ADC_gray */
#define ADC_gray_INST                                                       ADC0
#define ADC_gray_INST_IRQHandler                                 ADC0_IRQHandler
#define ADC_gray_INST_INT_IRQN                                   (ADC0_INT_IRQn)
#define ADC_gray_ADCMEM_0                                     DL_ADC12_MEM_IDX_0
#define ADC_gray_ADCMEM_0_REF               DL_ADC12_REFERENCE_VOLTAGE_VDDA_VSSA
#define GPIO_ADC_gray_C4_PORT                                              GPIOB
#define GPIO_ADC_gray_C4_PIN                                      DL_GPIO_PIN_25
#define GPIO_ADC_gray_IOMUX_C4                                   (IOMUX_PINCM56)
#define GPIO_ADC_gray_IOMUX_C4_FUNC               (IOMUX_PINCM56_PF_UNCONNECTED)


/* Defines for VREF */
#define VREF_VOLTAGE_MV                                                     3300
#define GPIO_VREF_VREFPOS_PORT                                             GPIOA
#define GPIO_VREF_VREFPOS_PIN                                     DL_GPIO_PIN_23
#define GPIO_VREF_IOMUX_VREFPOS                                  (IOMUX_PINCM53)
#define GPIO_VREF_IOMUX_VREFPOS_FUNC                IOMUX_PINCM53_PF_UNCONNECTED
#define GPIO_VREF_VREFNEG_PORT                                             GPIOA
#define GPIO_VREF_VREFNEG_PIN                                     DL_GPIO_PIN_21
#define GPIO_VREF_IOMUX_VREFNEG                                  (IOMUX_PINCM46)
#define GPIO_VREF_IOMUX_VREFNEG_FUNC                IOMUX_PINCM46_PF_UNCONNECTED
#define VREF_READY_DELAY                                                   (800)




/* Defines for L1: GPIOB.19 with pinCMx 45 on package pin 75 */
#define LED_L1_PORT                                                      (GPIOB)
#define LED_L1_PIN                                              (DL_GPIO_PIN_19)
#define LED_L1_IOMUX                                             (IOMUX_PINCM45)
/* Defines for L2: GPIOC.7 with pinCMx 85 on package pin 79 */
#define LED_L2_PORT                                                      (GPIOC)
#define LED_L2_PIN                                               (DL_GPIO_PIN_7)
#define LED_L2_IOMUX                                             (IOMUX_PINCM85)
/* Defines for RES: GPIOC.0 with pinCMx 74 on package pin 56 */
#define LCD_RES_PORT                                                     (GPIOC)
#define LCD_RES_PIN                                              (DL_GPIO_PIN_0)
#define LCD_RES_IOMUX                                            (IOMUX_PINCM74)
/* Defines for DC: GPIOA.14 with pinCMx 36 on package pin 53 */
#define LCD_DC_PORT                                                      (GPIOA)
#define LCD_DC_PIN                                              (DL_GPIO_PIN_14)
#define LCD_DC_IOMUX                                             (IOMUX_PINCM36)
/* Defines for CS: GPIOA.7 with pinCMx 14 on package pin 22 */
#define LCD_CS_PORT                                                      (GPIOA)
#define LCD_CS_PIN                                               (DL_GPIO_PIN_7)
#define LCD_CS_IOMUX                                             (IOMUX_PINCM14)
/* Port definition for Pin Group IMU */
#define IMU_PORT                                                         (GPIOC)

/* Defines for CS_IMU: GPIOC.4 with pinCMx 78 on package pin 67 */
#define IMU_CS_IMU_PIN                                           (DL_GPIO_PIN_4)
#define IMU_CS_IMU_IOMUX                                         (IOMUX_PINCM78)
/* Defines for IN1: GPIOC.2 with pinCMx 76 on package pin 65 */
#define IMU_IN1_PIN                                              (DL_GPIO_PIN_2)
#define IMU_IN1_IOMUX                                            (IOMUX_PINCM76)
/* Defines for IN2: GPIOC.3 with pinCMx 77 on package pin 66 */
#define IMU_IN2_PIN                                              (DL_GPIO_PIN_3)
#define IMU_IN2_IOMUX                                            (IOMUX_PINCM77)
/* Defines for nfault1: GPIOB.6 with pinCMx 23 on package pin 40 */
#define MOTOR_nfault1_PORT                                               (GPIOB)
#define MOTOR_nfault1_PIN                                        (DL_GPIO_PIN_6)
#define MOTOR_nfault1_IOMUX                                      (IOMUX_PINCM23)
/* Defines for nsleep2: GPIOB.5 with pinCMx 18 on package pin 26 */
#define MOTOR_nsleep2_PORT                                               (GPIOB)
#define MOTOR_nsleep2_PIN                                        (DL_GPIO_PIN_5)
#define MOTOR_nsleep2_IOMUX                                      (IOMUX_PINCM18)
/* Defines for nfault2: GPIOB.7 with pinCMx 24 on package pin 41 */
#define MOTOR_nfault2_PORT                                               (GPIOB)
#define MOTOR_nfault2_PIN                                        (DL_GPIO_PIN_7)
#define MOTOR_nfault2_IOMUX                                      (IOMUX_PINCM24)
/* Defines for nsleep1: GPIOB.4 with pinCMx 17 on package pin 25 */
#define MOTOR_nsleep1_PORT                                               (GPIOB)
#define MOTOR_nsleep1_PIN                                        (DL_GPIO_PIN_4)
#define MOTOR_nsleep1_IOMUX                                      (IOMUX_PINCM17)
/* Defines for nsleep3: GPIOB.28 with pinCMx 65 on package pin 29 */
#define MOTOR_nsleep3_PORT                                               (GPIOB)
#define MOTOR_nsleep3_PIN                                       (DL_GPIO_PIN_28)
#define MOTOR_nsleep3_IOMUX                                      (IOMUX_PINCM65)
/* Defines for nfault3: GPIOB.8 with pinCMx 25 on package pin 42 */
#define MOTOR_nfault3_PORT                                               (GPIOB)
#define MOTOR_nfault3_PIN                                        (DL_GPIO_PIN_8)
#define MOTOR_nfault3_IOMUX                                      (IOMUX_PINCM25)
/* Defines for nsleep4: GPIOB.31 with pinCMx 68 on package pin 32 */
#define MOTOR_nsleep4_PORT                                               (GPIOB)
#define MOTOR_nsleep4_PIN                                       (DL_GPIO_PIN_31)
#define MOTOR_nsleep4_IOMUX                                      (IOMUX_PINCM68)
/* Defines for nfault4: GPIOB.9 with pinCMx 26 on package pin 43 */
#define MOTOR_nfault4_PORT                                               (GPIOB)
#define MOTOR_nfault4_PIN                                        (DL_GPIO_PIN_9)
#define MOTOR_nfault4_IOMUX                                      (IOMUX_PINCM26)
/* Defines for PH1: GPIOA.31 with pinCMx 6 on package pin 7 */
#define MOTOR_PH1_PORT                                                   (GPIOA)
#define MOTOR_PH1_PIN                                           (DL_GPIO_PIN_31)
#define MOTOR_PH1_IOMUX                                           (IOMUX_PINCM6)
/* Defines for PH2: GPIOB.0 with pinCMx 12 on package pin 20 */
#define MOTOR_PH2_PORT                                                   (GPIOB)
#define MOTOR_PH2_PIN                                            (DL_GPIO_PIN_0)
#define MOTOR_PH2_IOMUX                                          (IOMUX_PINCM12)
/* Defines for PH3: GPIOB.2 with pinCMx 15 on package pin 23 */
#define MOTOR_PH3_PORT                                                   (GPIOB)
#define MOTOR_PH3_PIN                                            (DL_GPIO_PIN_2)
#define MOTOR_PH3_IOMUX                                          (IOMUX_PINCM15)
/* Defines for PH4: GPIOB.3 with pinCMx 16 on package pin 24 */
#define MOTOR_PH4_PORT                                                   (GPIOB)
#define MOTOR_PH4_PIN                                            (DL_GPIO_PIN_3)
#define MOTOR_PH4_IOMUX                                          (IOMUX_PINCM16)
/* Defines for Key1: GPIOA.17 with pinCMx 39 on package pin 69 */
#define KEY_Key1_PORT                                                    (GPIOA)
// pins affected by this interrupt request:["Key1"]
#define KEY_GPIOA_INT_IRQN                                      (GPIOA_INT_IRQn)
#define KEY_GPIOA_INT_IIDX                      (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define KEY_Key1_IIDX                                       (DL_GPIO_IIDX_DIO17)
#define KEY_Key1_PIN                                            (DL_GPIO_PIN_17)
#define KEY_Key1_IOMUX                                           (IOMUX_PINCM39)
/* Defines for Key2: GPIOC.5 with pinCMx 79 on package pin 68 */
#define KEY_Key2_PORT                                                    (GPIOC)
// pins affected by this interrupt request:["Key2"]
#define KEY_GPIOC_INT_IRQN                                      (GPIOC_INT_IRQn)
#define KEY_GPIOC_INT_IIDX                      (DL_INTERRUPT_GROUP1_IIDX_GPIOC)
#define KEY_Key2_IIDX                                        (DL_GPIO_IIDX_DIO5)
#define KEY_Key2_PIN                                             (DL_GPIO_PIN_5)
#define KEY_Key2_IOMUX                                           (IOMUX_PINCM79)
/* Port definition for Pin Group gw_ad */
#define gw_ad_PORT                                                       (GPIOB)

/* Defines for D0: GPIOB.20 with pinCMx 48 on package pin 82 */
#define gw_ad_D0_PIN                                            (DL_GPIO_PIN_20)
#define gw_ad_D0_IOMUX                                           (IOMUX_PINCM48)
/* Defines for D1: GPIOB.23 with pinCMx 51 on package pin 85 */
#define gw_ad_D1_PIN                                            (DL_GPIO_PIN_23)
#define gw_ad_D1_IOMUX                                           (IOMUX_PINCM51)
/* Defines for D2: GPIOB.26 with pinCMx 57 on package pin 96 */
#define gw_ad_D2_PIN                                            (DL_GPIO_PIN_26)
#define gw_ad_D2_IOMUX                                           (IOMUX_PINCM57)


/* Defines for TRNG */
/*
 * The TRNG interrupt is part of INT_GROUP1. Refer to the TRM for more details
 * on interrupt grouping
 */
#define TRNG_INT_IRQN                                            (TRNG_INT_IRQn)
#define TRNG_INT_IIDX                            (DL_INTERRUPT_GROUP1_IIDX_TRNG)



/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_MOTOR_PWM_init(void);
void SYSCFG_DL_PWM_1_init(void);
void SYSCFG_DL_QEI_M0_init(void);
void SYSCFG_DL_QEI_M1_init(void);
void SYSCFG_DL_TimerA1_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_UART_4_init(void);
void SYSCFG_DL_SPI_0_init(void);
void SYSCFG_DL_SPI_1_init(void);
void SYSCFG_DL_ADC_gray_init(void);
void SYSCFG_DL_VREF_init(void);

void SYSCFG_DL_TRNG_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
