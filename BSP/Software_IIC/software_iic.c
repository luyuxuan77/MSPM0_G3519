/*
 * software_iic.c
 * 软件I2C驱动 — 匹配感为灰度 MSPM0G3507 参考工程
 * 芯片: MSPM0G3519, MCLK=80MHz
 * 引脚: PA0=SCL, PA1=SDA
 */

#include "software_iic.h"

#define ACK  0x0U  /* acknowledge (SDA LOW) */
#define NACK 0x1U  /* not acknowledge (SDA HIGH) */
#define LOW  0x0U
#define HIGH 0x1U

#define I2C_READ  0x1U
#define I2C_WRITE 0x0U

/* ---- GW Gray Sensor register map ---- */
#define GW_GRAY_ADDR_DEF          0x4CU
#define GW_GRAY_PING              0xAAU
#define GW_GRAY_PING_OK           0x66U

/* ================================================================
 * 软件I2C时序延迟
 *   MCLK = 80MHz, delay_cycles(800) ≈ 10µs → ~50kHz I2C
 *   与参考工程 delay_us(10) 等效
 * ================================================================ */
static void IIC_Delay(void)
{
    delay_cycles(800);   /* 10µs at 80MHz, matches reference delay_us(10) */
}

/* ================================================================
 * GPIO初始化: SDA/SCL配置为开漏模拟
 *   - 使能内部上拉（弥补跳线帽未插的情况）
 *   - SDA可在运行时切换输入/输出方向
 * ================================================================ */
void sw_i2c_init(void)
{
    /* SDA (PA1): enable internal pull-up + start as output high */
    DL_GPIO_initDigitalInputFeatures(SW_I2C_SDA_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_setPins(SDA_PORT, SDA_PIN);
    DL_GPIO_enableOutput(SDA_PORT, SDA_PIN);

    /* SCL (PA0): enable internal pull-up + start as output high */
    DL_GPIO_initDigitalInputFeatures(SW_I2C_SCL_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_setPins(SCL_PORT, SCL_PIN);
    DL_GPIO_enableOutput(SCL_PORT, SCL_PIN);
}

/* ==================== 基本时序操作 (与参考工程一致) ==================== */

void IIC_Start(void)
{
    SDA_HIGH();
    SCL_HIGH();
    IIC_Delay();
    SDA_LOW();
    IIC_Delay();
    SCL_LOW();
}

void IIC_Stop(void)
{
    SDA_LOW();
    IIC_Delay();
    SCL_HIGH();
    IIC_Delay();
    SDA_HIGH();
    IIC_Delay();
}

uint8_t IIC_WaitAck(void)
{
    uint8_t ack;

    /* 切换SDA为输入模式，让从机可以安全地拉低ACK */
    DL_GPIO_initDigitalInput(SW_I2C_SDA_IOMUX);

    SCL_HIGH();
    IIC_Delay();
    ack = READ_SDA();
    SCL_LOW();
    IIC_Delay();

    /* 恢复SDA为输出模式 */
    DL_GPIO_initDigitalOutput(SW_I2C_SDA_IOMUX);
    DL_GPIO_setPins(SDA_PORT, SDA_PIN);
    DL_GPIO_enableOutput(SDA_PORT, SDA_PIN);

    return ack;
}

void IIC_SendAck(void)
{
    SDA_LOW();
    SCL_HIGH();
    IIC_Delay();
    SCL_LOW();
    SDA_HIGH();
}

void IIC_SendNAck(void)
{
    SDA_HIGH();
    SCL_HIGH();
    IIC_Delay();
    SCL_LOW();
}

uint8_t IIC_SendByte(uint8_t dat)
{
    for (uint8_t i = 0; i < 8; i++) {
        (dat & 0x80) ? SDA_HIGH() : SDA_LOW();
        dat <<= 1;
        SCL_HIGH();
        IIC_Delay();
        SCL_LOW();
        IIC_Delay();
    }
    return IIC_WaitAck();
}

uint8_t IIC_RecvByte(void)
{
    uint8_t dat = 0;
    SDA_HIGH();

    /* 接收数据前切换SDA为输入模式 */
    DL_GPIO_initDigitalInput(SW_I2C_SDA_IOMUX);

    for (uint8_t i = 0; i < 8; i++) {
        dat <<= 1;
        SCL_HIGH();
        IIC_Delay();
        if (READ_SDA()) dat |= 0x01;
        SCL_LOW();
        IIC_Delay();
    }

    /* 恢复SDA为输出模式 */
    DL_GPIO_initDigitalOutput(SW_I2C_SDA_IOMUX);
    DL_GPIO_setPins(SDA_PORT, SDA_PIN);
    DL_GPIO_enableOutput(SDA_PORT, SDA_PIN);

    return dat;
}

/* ==================== 应用层读写API (与参考工程一致) ==================== */

uint8_t IIC_ReadByte(uint8_t Salve_Address)
{
    uint8_t dat;

    IIC_Start();
    IIC_SendByte(Salve_Address | 0x01);  /* 读模式 */
    dat = IIC_RecvByte();
    IIC_SendNAck();
    IIC_Stop();

    return dat;
}

uint8_t IIC_ReadBytes(uint8_t Salve_Address, uint8_t Reg_Address,
                      uint8_t *Result, uint8_t len)
{
    IIC_Start();
    if (IIC_SendByte(Salve_Address & 0xFE)) {   /* 写模式: 发送寄存器地址 */
        IIC_Stop();
        return 0;
    }
    if (IIC_SendByte(Reg_Address)) {
        IIC_Stop();
        return 0;
    }
    IIC_Start();
    if (IIC_SendByte(Salve_Address | 0x01)) {   /* 读模式 */
        IIC_Stop();
        return 0;
    }

    for (uint8_t i = 0; i < len; i++) {
        Result[i] = IIC_RecvByte();
        (i == len - 1) ? IIC_SendNAck() : IIC_SendAck();
    }
    IIC_Stop();
    return 1;
}

uint8_t IIC_WriteByte(uint8_t Salve_Address, uint8_t Reg_Address,
                      uint8_t data)
{
    IIC_Start();
    if (IIC_SendByte(Salve_Address & 0xFE)) {   /* 写模式 */
        IIC_Stop();
        return 0;
    }
    if (IIC_SendByte(Reg_Address)) {
        IIC_Stop();
        return 0;
    }
    if (IIC_SendByte(data)) {
        IIC_Stop();
        return 0;
    }
    IIC_Stop();
    return 1;
}

uint8_t IIC_WriteBytes(uint8_t Salve_Address, uint8_t Reg_Address,
                       uint8_t *data, uint8_t len)
{
    IIC_Start();
    if (IIC_SendByte(Salve_Address & 0xFE)) {   /* 写模式 */
        IIC_Stop();
        return 0;
    }
    if (IIC_SendByte(Reg_Address)) {
        IIC_Stop();
        return 0;
    }

    for (uint8_t i = 0; i < len; i++) {
        if (IIC_SendByte(data[i])) {
            IIC_Stop();
            return 0;
        }
    }
    IIC_Stop();
    return 1;
}

/* ==================== 传感器辅助函数 (与参考工程一致) ==================== */

uint8_t gw_ping(void)
{
    uint8_t dat;
    IIC_ReadBytes(GW_GRAY_ADDR_DEF << 1, GW_GRAY_PING, &dat, 1);
    if (dat == GW_GRAY_PING_OK) {
        return 0;
    } else {
        return 1;
    }
}

/* 广播重置地址所需的魔数 */
static uint8_t reset_magic_number[8] = {
    0xB8, 0xD0, 0xCE, 0xAA,
    0xBF, 0xC6, 0xBC, 0xBC
};

void i2c_begin_transmission(uint8_t address)
{
    IIC_Start();
    if (IIC_SendByte(address << 1)) {  /* 左移1位，最低位为0表示写模式 */
        IIC_Stop();  /* 无应答则停止传输 */
    }
}

uint8_t i2c_write(uint8_t *data, uint8_t length)
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < length; i++) {
        if (IIC_SendByte(data[i]) == 0) {  /* 返回0表示收到ACK */
            count++;
        } else {
            break;  /* 收到NACK则停止 */
        }
    }
    return count;
}

void i2c_end_transmission(void)
{
    IIC_Stop();
}

/*
 * 广播重置 — 向广播地址(0x00)发送魔数序列来重置I2C设备地址为默认0x4C
 */
void i2c_reset(void)
{
    i2c_begin_transmission(0x00);   /* 0x00是广播地址 */
    i2c_write(reset_magic_number, 8);
    i2c_end_transmission();
}
