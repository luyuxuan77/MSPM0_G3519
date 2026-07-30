/*
 ******************************************************************************
 * @file    sensor_fusion.c
 * @author  Sensors Software Solution Team
 * @brief   This file how to configure compressed FIFO and to retrieve acc
 *          and gyro data. This sample use a fifo utility library tool
 *          for FIFO decompression.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
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


//#define SENSOR_BUS SPI1_Handler



/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "lsm6dsv16x_reg.h"
#include "bsp.h"



/* Private macro -------------------------------------------------------------*/
/*
 * Select FIFO samples watermark, max value is 512
 * in FIFO are stored acc, gyro and timestamp samples
 */
#define BOOT_TIME         10
#define FIFO_WATERMARK    3

/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI;
static uint8_t tx_buffer[1000];

/* Private variables ---------------------------------------------------------*/
static lsm6dsv16x_fifo_sflp_raw_t fifo_sflp;
static int16_t *datax;
static int16_t *datay;
static int16_t *dataz;
static float angles[3];

/* Extern variables ----------------------------------------------------------*/

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

static float_t npy_half_to_float(uint16_t h)
{
    union { float_t ret; uint32_t retbits; } conv;
    conv.retbits = lsm6dsv16x_from_f16_to_f32(h);
    return conv.ret;
}

static void sflp2q(float_t quat[4], uint8_t sflp[3])
{
  float_t sumsq = 0;
  uint16_t sf[3];
  sf[0]=sflp[0]|sflp[1]<<8;
  sf[1]=sflp[2]|sflp[3]<<8;
  sf[2]=sflp[4]|sflp[5]<<8;
  quat[0] = npy_half_to_float(sf[0]);
  quat[1] = npy_half_to_float(sf[1]);
  quat[2] = npy_half_to_float(sf[2]);

  for (uint8_t i = 0; i < 3; i++)
    sumsq += quat[i] * quat[i];

  if (sumsq > 1.0f) {
    float_t n = sqrtf(sumsq);
    quat[0] /= n;
    quat[1] /= n;
    quat[2] /= n;
    sumsq = 1.0f;
  }

  quat[3] = sqrtf(1.0f - sumsq);
}

static void quaternions_to_angles(const float quat[4], float angles[3])
{
	const float RAD_2_DEG = (180.f / 3.14159265358979f);
	float       rot_matrix[9];

	// quaternion_to_rotation_matrix
	const float dTx  = 2.0f * quat[0];
	const float dTy  = 2.0f * quat[1];
	const float dTz  = 2.0f * quat[2];
	const float dTwx = dTx * quat[3];
	const float dTwy = dTy * quat[3];
	const float dTwz = dTz * quat[3];
	const float dTxx = dTx * quat[0];
	const float dTxy = dTy * quat[0];
	const float dTxz = dTz * quat[0];
	const float dTyy = dTy * quat[1];
	const float dTyz = dTz * quat[1];
	const float dTzz = dTz * quat[2];

	rot_matrix[0] = 1.0f - (dTyy + dTzz);
	rot_matrix[1] = dTxy - dTwz;
	rot_matrix[2] = dTxz + dTwy;
	rot_matrix[3] = dTxy + dTwz;
	rot_matrix[4] = 1.0f - (dTxx + dTzz);
	rot_matrix[5] = dTyz - dTwx;
	rot_matrix[6] = dTxz - dTwy;
	rot_matrix[7] = dTyz + dTwx;
	rot_matrix[8] = 1.0f - (dTxx + dTyy);

	angles[0] = atan2f(-rot_matrix[3], rot_matrix[0]) * RAD_2_DEG;
	angles[1] = atan2f(-rot_matrix[7], rot_matrix[8]) * RAD_2_DEG;
	angles[2] = asinf(-rot_matrix[6]) * RAD_2_DEG;

	if (angles[0] < 0.f)
		angles[0] += 360.f;
}

void lsm6dsv16x_sensor_fusion(void)
{
  lsm6dsv16x_fifo_status_t fifo_status;
  stmdev_ctx_t dev_ctx;
  lsm6dsv16x_reset_t rst;
  lsm6dsv16x_sflp_gbias_t gbias;

  /* Uncomment to configure INT 1 */
  //lsm6dsv16x_pin_int1_route_t int1_route;
  /* Uncomment to configure INT 2 */
  //lsm6dsv16x_pin_int2_route_t int2_route;
  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  //dev_ctx.handle = &SENSOR_BUS;

  /* Init test platform */
  platform_init(dev_ctx.handle);
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
  /* Set full scale */
  lsm6dsv16x_xl_full_scale_set(&dev_ctx, LSM6DSV16X_4g);
  lsm6dsv16x_gy_full_scale_set(&dev_ctx, LSM6DSV16X_2000dps);

  /*
   * Set FIFO watermark (number of unread sensor data TAG + 6 bytes
   * stored in FIFO) to FIFO_WATERMARK samples
   */
  lsm6dsv16x_fifo_watermark_set(&dev_ctx, FIFO_WATERMARK);
  lsm6dsv16x_fifo_gy_batch_set(&dev_ctx, LSM6DSV16X_GY_BATCHED_AT_60Hz);

  /* Set FIFO batch of sflp data */
  fifo_sflp.game_rotation = 1;
  fifo_sflp.gravity = 1;
  fifo_sflp.gbias = 0;
  lsm6dsv16x_fifo_sflp_batch_set(&dev_ctx, fifo_sflp);

  /* Set FIFO mode to Stream mode (aka Continuous Mode) */
  lsm6dsv16x_fifo_mode_set(&dev_ctx, LSM6DSV16X_STREAM_MODE);

  /* Set Output Data Rate */
  lsm6dsv16x_xl_data_rate_set(&dev_ctx, LSM6DSV16X_ODR_AT_60Hz);
  lsm6dsv16x_gy_data_rate_set(&dev_ctx, LSM6DSV16X_ODR_AT_60Hz);
  lsm6dsv16x_sflp_data_rate_set(&dev_ctx, LSM6DSV16X_SFLP_60Hz);

  lsm6dsv16x_sflp_game_rotation_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * here application may initialize offset with latest values
   * calculated from previous run and saved to non volatile memory.
   */
  gbias.gbias_x = 0.0f;
  gbias.gbias_y = 0.0f;
  gbias.gbias_z = 0.0f;
  lsm6dsv16x_sflp_game_gbias_set(&dev_ctx, &gbias);

  /* Wait samples */
  while (1) {
    uint16_t num = 0;

    /* Read watermark flag */
    lsm6dsv16x_fifo_status_get(&dev_ctx, &fifo_status);

    if (fifo_status.fifo_th == 1) {
      num = fifo_status.fifo_level;

      //snprintf((char *)tx_buffer, sizeof(tx_buffer), "-- FIFO num %d \r\n", num);
      //tx_com(tx_buffer, strlen((char const *)tx_buffer));

      while (num--) {
        lsm6dsv16x_fifo_out_raw_t f_data;
        uint8_t *axis;
        float_t quat[4];
        float_t gravity_mg[3];
        float_t gbias_mdps[3];
        int16_t gyr[3];
        /* Read FIFO sensor value */
        lsm6dsv16x_fifo_out_raw_get(&dev_ctx, &f_data);

        switch (f_data.tag) {
        case LSM6DSV16X_GY_NC_TAG:
          
  gyr[0]=f_data.data[0]|f_data.data[1]<<8;
  gyr[1]=f_data.data[2]|f_data.data[3]<<8;
  gyr[2]=f_data.data[4]|f_data.data[5]<<8;
        // datax = &gyr[0];
        // datay = &gyr[1];
        // dataz = &gyr[2];
        //  printf("GYR [dps]:\t%4.2f\t%4.2f\t%4.2f\r\n",
        //          (float)gyr[0]*70.0f,
        //          (float)gyr[1]*70.0f,
        //          (float)gyr[2]*70.0f);
          break;
        case LSM6DSV16X_SFLP_GYROSCOPE_BIAS_TAG:
          axis = &f_data.data[0];
          gbias_mdps[0] = lsm6dsv16x_from_fs125_to_mdps(axis[0] | (axis[1] << 8));
          gbias_mdps[1] = lsm6dsv16x_from_fs125_to_mdps(axis[2] | (axis[3] << 8));
          gbias_mdps[2] = lsm6dsv16x_from_fs125_to_mdps(axis[4] | (axis[5] << 8));
//          printf("GBIAS [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
//                         gbias_mdps[0], gbias_mdps[1], gbias_mdps[2]);
          //tx_com(tx_buffer, strlen((char const *)tx_buffer));
          break;
        case LSM6DSV16X_SFLP_GRAVITY_VECTOR_TAG:
          axis = &f_data.data[0];
          gravity_mg[0] = lsm6dsv16x_from_sflp_to_mg(axis[0] | (axis[1] << 8));
          gravity_mg[1] = lsm6dsv16x_from_sflp_to_mg(axis[2] | (axis[3] << 8));
          gravity_mg[2] = lsm6dsv16x_from_sflp_to_mg(axis[4] | (axis[5] << 8));
        //  printf("Gravity [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
        //                 gravity_mg[0], gravity_mg[1], gravity_mg[2]);
          //tx_com(tx_buffer, strlen((char const *)tx_buffer));
          break;
        case LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG:
          sflp2q(quat, f_data.data);
          //printf("data= %d \t %d \t %d \t %d \t %d \t %d \t\r\n",f_data.data[0],f_data.data[1],f_data.data[2],f_data.data[3],f_data.data[4],f_data.data[5]);
				quaternions_to_angles(quat, angles);
				 printf("%4.2f\t%4.2f\t%4.2f\n",
                          angles[0], angles[1], angles[2]);
          //snprintf((char *)tx_buffer, sizeof(tx_buffer), "[%02x %02x %02x %02x %02x %02x] Game Rotation \tX: %2.3f\tY: %2.3f\tZ: %2.3f\tW: %2.3f\r\n",
           //        f_data.data[0], f_data.data[1],f_data.data[2],f_data.data[3],f_data.data[4],f_data.data[5],
            //       (double_t)quat[0], (double_t)quat[1], (double_t)quat[2], (double_t)quat[3]);
          //tx_com(tx_buffer, strlen((char const *)tx_buffer));
          break;
        default:
         break;
        }
      }

      //snprintf((char *)tx_buffer, sizeof(tx_buffer), "------ \r\n\r\n");
      //tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }
		platform_delay(10);
  }
}

//#define	SPI_IMU_CS PAout(4)  //ѡ��IMU	
static uint8_t io_write_reg(uint8_t reg, uint8_t value)
{
    SPI1_CS_IMU(0);
    // 发送数据
    spi1_read_write_byte(reg); 

    // 接收数据
    spi1_read_write_byte( value);

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
  delay_ms(ms);
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
	
	// SPI1_
//	();		   			        //��ʼ��SPI
	// SPI1_SetSpeed(SPI_BAUDRATEPRESCALER_64); 
}
