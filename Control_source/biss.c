#include "biss.h"
#include "delay.h"

/* ===== GPIO 快速操作宏 ===== */
#define GPIO_SET(port, pin)     ((port)->BSRR = (pin))
#define GPIO_CLR(port, pin)     ((port)->BSRR = ((uint32_t)(pin) << 16U))

#define GPIO_READ(port, pin)    (((port)->IDR & (pin)) ? 1 : 0)


#define BISS_T_US   2   // 半周期 2 µs → 250 kHz

// 6. 优化版本：批量读取（减少函数调用开销）
#define BISS_POS_BITS   24          // 编码器角度位数······················
#define BISS_FULL_SCALE (1UL << BISS_POS_BITS)  


float BISS_ReadAngleDeg(BISS_Encoder_t *enc)
{
    uint32_t raw = 0;
    uint8_t bit;

    // uint32_t primask = __get_PRIMASK();
    // __disable_irq();

    // GPIO_SET(enc->clk_port, enc->clk_pin); // 启动 将指定的时钟引                                                                                                                                                                                                                                                                                                                                        脚（enc->clk_pin）置为高电平。这使时钟线进入空闲高电平状态，准备开始通信。
    // delay_us_tim6(1);

    for (uint8_t i = 0; i < BISS_POS_BITS; i++)
    {
        GPIO_CLR(enc->clk_port, enc->clk_pin);
        delay_us_tim6(BISS_T_US);

        bit = GPIO_READ(enc->dat_port, enc->dat_pin);
        raw = (raw << 1) | bit;

        GPIO_SET(enc->clk_port, enc->clk_pin);
        delay_us_tim6(BISS_T_US);
    }

    // if (!primask) __enable_irq();                       

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




//! 下述为增加CRC的程序。校验始终不对返回-1

// #include "biss.h"
// #include "delay.h"

// /* ===== GPIO 快速操作宏 ===== */
// #define GPIO_SET(port, pin)     ((port)->BSRR = (pin))
// #define GPIO_CLR(port, pin)     ((port)->BSRR = ((uint32_t)(pin) << 16U))
// #define GPIO_READ(port, pin)    (((port)->IDR & (pin)) ? 1 : 0)

// #define BISS_T_US   2   // 半周期 2 µs → 250 kHz

// // 根据编码器手册定义
// #define BISS_ST_BITS      24          // 绝对位置位数
// #define BISS_ERR_BIT      1           // 错误位
// #define BISS_WARN_BIT     1           // 报警位
// #define BISS_CRC_BITS     6           // CRC位数
// #define BISS_FRAME_BITS   (BISS_ST_BITS + BISS_ERR_BIT + BISS_WARN_BIT + BISS_CRC_BITS) // 总共32位
// #define BISS_FULL_SCALE   (1UL << BISS_ST_BITS) // 用于角度映射

// /**
//  * @brief 读取编码器角度并进行 CRC 校验
//  * @param enc 编码器结构体指针
//  * @return 角度值 (度)，如果校验失败返回 -1.0f
//  */
// float BISS_ReadAngleDeg(BISS_Encoder_t *enc)
// {
//     uint64_t raw_frame = 0; // 使用 64 位整数存储完整的 56 位帧
//     uint8_t bit;

//     // 1. 读取完整的 56 位数据帧
//     for (uint8_t i = 0; i < 56; i++) // 硬编码 56 位，因为总帧长固定
//     {
//         GPIO_CLR(enc->clk_port, enc->clk_pin);
//         delay_us_tim6(BISS_T_US);
//         __NOP();

//         bit = GPIO_READ(enc->dat_port, enc->dat_pin);
//         raw_frame = (raw_frame << 1) | bit; // 左移，新位进入最低位

//         GPIO_SET(enc->clk_port, enc->clk_pin);
//         delay_us_tim6(BISS_T_US);
//     }

//     // 2. 提取接收到的 CRC (最低 6 位)
//     uint8_t rx_crc = (uint8_t)(raw_frame & 0x3F); // 0x3F = 0b111111

//     // 3. 提取用于计算 CRC 的数据 ([31:8], [7], [6])
//     // 首先右移去掉 CRC 位
//     uint64_t data_for_crc_raw = raw_frame >> BISS_CRC_BITS;
    
//     // 提取 [31:8] (ST[23:0]) 和 [7:6] (nERR, nWARN)
//     // 注意：[31:8] 是 24 位，[7:6] 是 2 位，总共 26 位
//     // 我们需要将这 26 位作为连续的数据来计算 CRC
//     uint32_t data_for_crc = (uint32_t)data_for_crc_raw; // 取低 32 位，包含 ST[23:0] 和 nERR/nWARN
    
//     // 4. 计算本地 CRC
//     // 使用与 BISS_CRC6_Calc 相同的算法，但针对 26 位数据
//     uint8_t calc_crc = 0;
//     {
//         uint8_t crc = 0;
//         uint32_t mask = 1UL << (BISS_ST_BITS + BISS_ERR_BIT + BISS_WARN_BIT - 1); // 26-1=25
//         for (uint8_t i = 0; i < (BISS_ST_BITS + BISS_ERR_BIT + BISS_WARN_BIT); i++)
//         {
//             uint8_t bit_val = (data_for_crc & mask) ? 1 : 0;
//             mask >>= 1;
//             crc = (crc << 1) | bit_val;
//             if (crc & 0x40) crc ^= 0x43;
//         }
//         calc_crc = crc & 0x3F;
//     }

//     // 5. 校验
//     if (calc_crc != rx_crc)
//     {
//         // CRC 校验失败
//         return -1.0f; 
//     }

//     // 6. 提取位置数据 (ST[23:0])
//     // 从 data_for_crc 中提取 [23:0] 位
//     uint32_t st_data = data_for_crc & ((1UL << BISS_ST_BITS) - 1); // 保留低 24 位

//     // 7. 计算角度
//     float angle = (float)st_data * 360.0f / (float)BISS_FULL_SCALE;
    
//     // 保留两位小数及特定倍数 (保持原逻辑)
//     angle = (float)((int32_t)(angle * 100.0f + 0.5f)) / 100.0f * 2;

//     return angle;
// }


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
