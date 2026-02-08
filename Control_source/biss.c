#include "biss.h"
#include "delay.h"

/* ===== GPIO 快速操作宏 ===== */
#define GPIO_SET(port, pin)     ((port)->BSRR = (pin))
#define GPIO_CLR(port, pin)     ((port)->BSRR = ((uint32_t)(pin) << 16U))

#define GPIO_READ(port, pin)    (((port)->IDR & (pin)) ? 1 : 0)


#define BISS_T_US   2   // 半周期 2 µs → 250 kHz

// 6. 优化版本：批量读取（减少函数调用开销）
#define BISS_POS_BITS   24          // 编码器角度位数
#define BISS_FULL_SCALE (1UL << BISS_POS_BITS)

//float BISS_ReadAngleDeg(BISS_Encoder_t *enc)
//{
//    uint32_t raw = 0;
//    uint8_t bit;

//    /* 关中断，保证时序 */
//    uint32_t primask = __get_PRIMASK();
//    __disable_irq();

//    /* CLK idle high */
//    enc->clk_port->BSRR = enc->clk_pin;
//    delay_us_tim7(1);

//    for (uint8_t i = 0; i < BISS_POS_BITS; i++)
//    {
//        /* CLK ↓ */
//        enc->clk_port->BSRR = (uint32_t)enc->clk_pin << 16;
//        delay_us_tim7(BISS_T_US);

//        /* DAT 采样 */
//        bit = (enc->dat_port->IDR & enc->dat_pin) ? 1 : 0;
//        raw = (raw << 1) | bit;

//        /* CLK ↑ */
//        enc->clk_port->BSRR = enc->clk_pin;
//        delay_us_tim7(BISS_T_US);
//    }

//    /* CLK idle high */
//    enc->clk_port->BSRR = enc->clk_pin;

//    /* 恢复中断 */
//    if (!primask) __enable_irq();

//    /* 映射到 0~360° */
//    return (float)raw * 360.0f / (float)BISS_FULL_SCALE;
//}
float BISS_ReadAngleDeg(BISS_Encoder_t *enc)
{
    uint32_t raw = 0;
    uint8_t bit;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    GPIO_SET(enc->clk_port, enc->clk_pin);
    delay_us_tim7(1);

    for (uint8_t i = 0; i < BISS_POS_BITS; i++)
    {
        GPIO_CLR(enc->clk_port, enc->clk_pin);
        delay_us_tim7(BISS_T_US);

        bit = GPIO_READ(enc->dat_port, enc->dat_pin);
        raw = (raw << 1) | bit;

        GPIO_SET(enc->clk_port, enc->clk_pin);
        delay_us_tim7(BISS_T_US);
    }

    if (!primask) __enable_irq();

    /* 去掉符号位，仅保留角度 */
    raw &= (BISS_FULL_SCALE - 1);

    /* 角度映射 */
    float angle = (float)raw * 360.0f / (float)BISS_FULL_SCALE;

    /* 保留两位小数 */
    angle = (float)((int32_t)(angle * 100.0f + 0.5f)) / 100.0f *2;

    return angle;
}





/*********************CRC 校验***************************/
uint8_t BISS_CRC6_Calc(uint32_t data)
{
    uint8_t crc = 0;
    uint32_t mask = 1UL << (BISS_POS_BITS - 1);

    for (uint8_t i = 0; i < BISS_POS_BITS; i++)
    {
        uint8_t bit = (data & mask) ? 1 : 0;
        mask >>= 1;

        crc = (crc << 1) | bit;

        if (crc & 0x40)
        {
            crc ^= 0x43;
        }
    }

    return crc & 0x3F;
}



        //编码器读取函数
    /* 电机 1 → 编码器 1 */
    BISS_Encoder_t encoder_motor1 =
    {
        .clk_port = GPIOC,
        .clk_pin  = GPIO_PIN_6,
        .dat_port = GPIOC,
        .dat_pin  = GPIO_PIN_7
    };

    /* 电机 2 → 编码器 2 */
    BISS_Encoder_t encoder_motor2 =
    {
        .clk_port = GPIOA,
        .clk_pin  = GPIO_PIN_15,
        .dat_port = GPIOB,
        .dat_pin  = GPIO_PIN_3
    };
    
    
    float motor1_pos, motor2_pos;
    float  motor1_crc_ok, motor2_crc_ok;
