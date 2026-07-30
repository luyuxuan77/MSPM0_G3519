#ifndef SOFTWARE_IIC_H
#define SOFTWARE_IIC_H

#include "ti_msp_dl_config.h"
#include "bsp.h"

/*
 * Software I2C — GPIO bit-bang I2C (匹配感为灰度3507参考工程)
 *
 * Pins:
 *   PA0 = SCL → IOMUX_PINCM1 (DIO00)
 *   PA1 = SDA → IOMUX_PINCM2 (DIO01)
 *
 * NOTE: 请插上SCL SDA跳线帽以便上拉总线，如果不插可能无法通讯
 *       传感器供电需要4.5-5V稳定供电
 *       保证单片机和传感器共地
 */

/* ---- 引脚定义 ---- */
#define SDA_PIN   DL_GPIO_PIN_1
#define SDA_PORT  GPIOA
#define SCL_PIN   DL_GPIO_PIN_0
#define SCL_PORT  GPIOA

#define SW_I2C_SDA_IOMUX  IOMUX_PINCM2
#define SW_I2C_SCL_IOMUX  IOMUX_PINCM1

/* ---- 基本I2C操作宏 ---- */
#define SDA_HIGH()  DL_GPIO_setPins(SDA_PORT, SDA_PIN)
#define SDA_LOW()   DL_GPIO_clearPins(SDA_PORT, SDA_PIN)
#define SCL_HIGH()  DL_GPIO_setPins(SCL_PORT, SCL_PIN)
#define SCL_LOW()   DL_GPIO_clearPins(SCL_PORT, SCL_PIN)
#define READ_SDA()  DL_GPIO_readPins(SDA_PORT, SDA_PIN)

/* ========== 软件I2C基础API ========== */
void     sw_i2c_init(void);
void     IIC_Start(void);
void     IIC_Stop(void);
uint8_t  IIC_WaitAck(void);
void     IIC_SendAck(void);
void     IIC_SendNAck(void);
uint8_t  IIC_SendByte(uint8_t dat);
uint8_t  IIC_RecvByte(void);

/* ========== 应用层读写API (与参考工程一致) ========== */
uint8_t  IIC_ReadByte(uint8_t Salve_Address);
uint8_t  IIC_ReadBytes(uint8_t Salve_Address, uint8_t Reg_Address,
                       uint8_t *Result, uint8_t len);
uint8_t  IIC_WriteByte(uint8_t Salve_Address, uint8_t Reg_Address,
                       uint8_t data);
uint8_t  IIC_WriteBytes(uint8_t Salve_Address, uint8_t Reg_Address,
                        uint8_t *data, uint8_t len);

/* ========== 传感器辅助函数 ========== */
uint8_t  gw_ping(void);
void     i2c_reset(void);
void     i2c_begin_transmission(uint8_t address);
uint8_t  i2c_write(uint8_t *data, uint8_t length);
void     i2c_end_transmission(void);

#endif /* SOFTWARE_IIC_H */
