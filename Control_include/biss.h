// #ifndef __BISS_H
// #define __BISS_H

// #include "stm32f4xx_hal.h"
// #include <stdint.h>

// /* ===== BiSS 参数 ===== */
// #define BISS_POS_BITS   24
// #define BISS_CRC_BITS   6

// /* ===== 编码器对象 ===== */
// typedef struct
// {
//     GPIO_TypeDef *clk_port;
//     uint16_t      clk_pin;

//     GPIO_TypeDef *dat_port;
//     uint16_t      dat_pin;
// } BISS_Encoder_t;

// // BISS状态结构体
// typedef struct {
//     uint8_t crc_ok;    // CRC校验结果
//     uint8_t nerr;      // 错误位 [7]，低有效
//     uint8_t nwarn;     // 报警位 [6]，低有效
// } BISS_Status_t;

// extern BISS_Status_t status_motor1, status_motor2;
// extern BISS_Encoder_t encoder_motor1;
// extern BISS_Encoder_t encoder_motor2;

// extern float motor1_pos, motor2_pos;
// extern float  motor1_crc_ok, motor2_crc_ok;


// /* ===== 接口函数 ===== */
// float BISS_ReadAngleDeg(BISS_Encoder_t *enc);

// /* ===== CRC ===== */
// uint8_t BISS_CRC6_Calc(uint32_t data);
// uint8_t BISS_HardwareCRC_Calc(uint32_t data);



// #endif



// !下述为考虑CRC校验的读取函数
#ifndef __BISS_H
#define __BISS_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ===== 编码器对象 ===== */
typedef struct
{
    // gpio hardware
    GPIO_TypeDef    *sck_gpio_port;
    uint16_t         sck_gpio_pin;
    GPIO_TypeDef    *dat_gpio_port;
    uint16_t         dat_gpio_pin;
    
    // raw data info
    uint32_t         abs_pos;
    bool             n_err;
    bool             n_warn;
    uint8_t          crc;

    // processed data info   
    double           pos_deg; // 角度值 (度) 经过零点校准
    double           pos_rad; // 角度值 (弧度) 经过零点校准
 
    //setting info
    uint32_t         zero_pos; // 零点位置

} MP55_Encoder_t;

// 外部声明编码器对象
extern MP55_Encoder_t encoder_motor1;
extern MP55_Encoder_t encoder_motor2;


/* ===== 接口函数 ===== */
extern double MP55_ReadFrame(MP55_Encoder_t *enc);
extern void MP55_SetZeroPos(MP55_Encoder_t *enc, uint32_t zero_pos);
extern uint8_t MP55_CRC6_Calc(uint64_t data);




#endif
