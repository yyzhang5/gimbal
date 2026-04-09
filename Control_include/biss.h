#ifndef __BISS_H
#define __BISS_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ===== BiSS 参数 ===== */
#define BISS_POS_BITS   24
#define BISS_CRC_BITS   6

/* ===== 编码器对象 ===== */
typedef struct
{
    GPIO_TypeDef *clk_port;
    uint16_t      clk_pin;

    GPIO_TypeDef *dat_port;
    uint16_t      dat_pin;
} BISS_Encoder_t;

extern BISS_Encoder_t encoder_motor1;
extern BISS_Encoder_t encoder_motor2;

extern float motor1_pos, motor2_pos;
extern float  motor1_crc_ok, motor2_crc_ok;


/* ===== 接口函数 ===== */
uint32_t BISS_ReadPosition(BISS_Encoder_t *enc, uint8_t *crc_ok);

float BISS_ReadAngleDeg(BISS_Encoder_t *enc);

/* ===== CRC ===== */
uint8_t BISS_CRC6_Calc(uint32_t data);
uint8_t BISS_CRC6_Calc_Generic(uint32_t data_high, uint32_t data_low, uint8_t total_bits);



#endif
