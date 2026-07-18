/*
 ******************************************************************************
 * @file    read_data_polling.c
 * @author  Sensors Software Solution Team
 * @brief   This file shows how to get data from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/*
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - STEVAL_MKI109V3 +
 * - NUCLEO_F401RE + X-NUCLEO-IKS01A3
 * - DISCOVERY_SPC584B +
 * - NUCLEO_H503RB + X-NUCLEO-IKS4A1
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F401RE - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default) / SPI(supported)
 *
 * DISCOVERY_SPC584B  - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default) / SPI(supported)
 *
 * NUCLEO_STM32H503RG - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I3C(Default)
 *
 * If you need to run this example on a different hardware platform a
 * modification of the functions: `platform_write`, `platform_read`,
 * `tx_com` and 'platform_init' is required.
 *
 */

/* STMicroelectronics evaluation boards definition
 *
 * Please uncomment ONLY the evaluation boards in use.
 * If a different hardware is used please comment all
 * following target board and redefine yours.
 */

//#define STEVAL_MKI109V3  /* little endian */
//#define NUCLEO_F401RE    /* little endian */
//#define SPC584B_DIS      /* big endian */

/* ATTENTION: By default the driver is little endian. If you need switch
 *            to big endian please see "Endianness definitions" in the
 *            header file of the driver (_reg.h).
 */


#define SENSOR_BUS SPI1_Handler

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "lsm6dsv16x_reg.h"
#include "bsp.h"




/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            10 //ms

/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_temperature;
static float_t acceleration_mg[3];
static float_t angular_rate_mdps[3];
static float_t temperature_degC;
static uint8_t whoamI;
static uint8_t tx_buffer[1000];

static lsm6dsv16x_filt_settling_mask_t filt_settling_mask;
uint8_t spi0_read_write_byte(uint8_t dat)
{
    uint8_t data = 0;

    // 发送数据
    DL_SPI_transmitData8(SPI_0_INST, dat);
    // 等待SPI总线空闲
    while (DL_SPI_isBusy(SPI_0_INST))
        ;
    // 接收数据
    data = DL_SPI_receiveData8(SPI_0_INST);
    // 等待SPI总线空闲
    while (DL_SPI_isBusy(SPI_0_INST))
        ;

    return data;
}
/* Extern variables ----------------------------------------------------------*/
void sendData(float acc[], float gyro[])
{
    uint8_t tmpData[30];
    memcpy(tmpData, acc, 12);
    memcpy(tmpData + 12, gyro, 12);
    memcpy(tmpData + 24, "\x00\x00\x80\x7F", 4);
    
    //HAL_UART_Transmit(&UART1_Handler, tmpData, 28, HAL_MAX_DELAY);
}
/* Private functions ---------------------------------------------------------*/

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void *handle);

/* Main Example --------------------------------------------------------------*/
void lsm6dsv16x_read_data_polling(void)
{
  lsm6dsv16x_reset_t rst;
  stmdev_ctx_t dev_ctx;
  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  //dev_ctx.handle = &SENSOR_BUS;

  /* Init test platform */
  //platform_init(dev_ctx.handle);
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  /* Check device ID */
  lsm6dsv16x_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LSM6DSV16X_ID)
    while (1);

  /* Restore default configuration */
  lsm6dsv16x_reset_set(&dev_ctx, LSM6DSV16X_RESTORE_CTRL_REGS);
  do {
    lsm6dsv16x_reset_get(&dev_ctx, &rst);
  } while (rst != LSM6DSV16X_READY);

  /* Enable Block Data Update */
  lsm6dsv16x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate.
   * Selected data rate have to be equal or greater with respect
   * with MLC data rate.
   */
  lsm6dsv16x_xl_data_rate_set(&dev_ctx, LSM6DSV16X_ODR_AT_60Hz);
  lsm6dsv16x_gy_data_rate_set(&dev_ctx, LSM6DSV16X_ODR_AT_60Hz);
  /* Set full scale */
  lsm6dsv16x_xl_full_scale_set(&dev_ctx, LSM6DSV16X_4g);
  lsm6dsv16x_gy_full_scale_set(&dev_ctx, LSM6DSV16X_1000dps);
  /* Configure filtering chain */
  filt_settling_mask.drdy = PROPERTY_ENABLE;
  filt_settling_mask.irq_xl = PROPERTY_ENABLE;
  filt_settling_mask.irq_g = PROPERTY_ENABLE;
  lsm6dsv16x_filt_settling_mask_set(&dev_ctx, filt_settling_mask);
  lsm6dsv16x_filt_gy_lp1_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsv16x_filt_gy_lp1_bandwidth_set(&dev_ctx, LSM6DSV16X_GY_VERY_LIGHT);
  lsm6dsv16x_filt_xl_lp2_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsv16x_filt_xl_lp2_bandwidth_set(&dev_ctx, LSM6DSV16X_XL_VERY_LIGHT);

  /* Read samples in polling mode (no int) */
  while (1) {
    lsm6dsv16x_data_ready_t drdy;

    /* Read output only if new xl value is available */
    lsm6dsv16x_flag_data_ready_get(&dev_ctx, &drdy);

    if (drdy.drdy_xl) {
      /* Read acceleration field data */
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      lsm6dsv16x_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
      acceleration_mg[0] =
        lsm6dsv16x_from_fs4_to_mg(data_raw_acceleration[0]);
      acceleration_mg[1] =
        lsm6dsv16x_from_fs4_to_mg(data_raw_acceleration[1]);
      acceleration_mg[2] =
        lsm6dsv16x_from_fs4_to_mg(data_raw_acceleration[2]);
//      printf("Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
//              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
		sendData(acceleration_mg, angular_rate_mdps);
      //tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    /* Read output only if new xl value is available */
    if (drdy.drdy_gy) {
      /* Read angular rate field data */
      memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
      lsm6dsv16x_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
      angular_rate_mdps[0] =
        lsm6dsv16x_from_fs1000_to_mdps(data_raw_angular_rate[0])/1000.0f;
      angular_rate_mdps[1] =
        lsm6dsv16x_from_fs1000_to_mdps(data_raw_angular_rate[1])/1000.0f;
      angular_rate_mdps[2] =
        lsm6dsv16x_from_fs1000_to_mdps(data_raw_angular_rate[2])/1000.0f;
//      printf("Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
//              angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
      //tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

//    if (drdy.drdy_temp) {
//      /* Read temperature data */
//      memset(&data_raw_temperature, 0x00, sizeof(int16_t));
//      lsm6dsv16x_temperature_raw_get(&dev_ctx, &data_raw_temperature);
//      temperature_degC = lsm6dsv16x_from_lsb_to_celsius(
//                           data_raw_temperature);
//      snprintf((char *)tx_buffer, sizeof(tx_buffer),
//              "Temperature [degC]:%6.2f\r\n", temperature_degC);
//      tx_com(tx_buffer, strlen((char const *)tx_buffer));
//    }
  }
}

static uint8_t io_write_reg(uint8_t reg, uint8_t value)
{
    SPI1_CS_IMU(0);
    // 发送数据
    DL_SPI_transmitData8(SPI_0_INST, reg);
    // 等待SPI总线空闲
    while (DL_SPI_isBusy(SPI_0_INST))
        ;
    // 接收数据
    value = DL_SPI_receiveData8(SPI_0_INST);
    // 等待SPI总线空闲
    while (DL_SPI_isBusy(SPI_0_INST))
        ;
    SPI1_CS_IMU(1);

    return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{

//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//  HAL_SPI_Transmit(handle, &reg, 1, 1000);
//  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	
int rc;

	for (uint32_t i = 0; i < len; i++) 
	{
		rc = io_write_reg(reg + i, bufp[i]);
		if (rc)
			return rc;
	}
	return 0;

  //return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{

//  reg |= 0x80;
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//  HAL_SPI_Transmit(handle, &reg, 1, 1000);
//  HAL_SPI_Receive(handle, bufp, len, 1000);
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    reg |= 0x80;
    SPI1_CS_IMU(0);

    spi1_read_write_byte(reg);

    while(len)
	{
		*bufp = spi1_read_write_byte(0x00);
		len--;
		bufp++;
	}
    SPI1_CS_IMU(1);
  return 0;
}


/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  //HAL_UART_Transmit(&UART1_Handler, tx_buffer, len, 1000);
	//printf("%s", tx_buffer);
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
  delay_ms(ms*2);
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void *handle)
{
  //   GPIO_InitTypeDef GPIO_Initure;
	
  //   __HAL_RCC_GPIOA_CLK_ENABLE();           //ʹ��GPIOBʱ��
    
  //   //PA4
  //   GPIO_Initure.Pin=GPIO_PIN_4;            //PA4
  //   GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //�������
  //   GPIO_Initure.Pull=GPIO_PULLUP;          //����
  //   GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����         
  //   HAL_GPIO_Init(GPIOA,&GPIO_Initure);     //��ʼ��
	
  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	
	// SPI1_Init();		   			        //��ʼ��SPI
	// SPI1_SetSpeed(SPI_BAUDRATEPRESCALER_64); 
}
