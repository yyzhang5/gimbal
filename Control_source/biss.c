// #include "biss.h"
// #include "delay.h"

// /* ===== GPIO 快速操作宏 ===== */
// #define GPIO_SET(port, pin)     ((port)->BSRR = (pin))
// #define GPIO_CLR(port, pin)     ((port)->BSRR = ((uint32_t)(pin) << 16U))

// #define GPIO_READ(port, pin)    (((port)->IDR & (pin)) ? 1 : 0)


// #define BISS_T_US   2   // 半周期 2 µs → 250 kHz

// // 6. 优化版本：批量读取（减少函数调用开销）
// #define BISS_POS_BITS   24          // 编码器角度位数······················
// #define BISS_FULL_SCALE (1UL << BISS_POS_BITS)  



// float BISS_ReadAngleDeg(BISS_Encoder_t *enc)
// {
//     uint32_t raw = 0;
//     uint8_t bit;


//     // 启动：设置时钟引脚为高电平
//     enc->clk_port->BSRR = enc->clk_pin;
//     delay_us(1);

//     for (uint8_t i = 0; i < BISS_POS_BITS; i++)
//     {
//         // 时钟下降沿
//         enc->clk_port->BSRR = ((uint32_t)enc->clk_pin << 16U);
//         delay_us(BISS_T_US);

//         // 读取数据
//         bit = (enc->dat_port->IDR & enc->dat_pin) ? 1 : 0;
//         raw = (raw << 1) | bit;

//         // 时钟上升沿
//         enc->clk_port->BSRR = enc->clk_pin;
//         delay_us(BISS_T_US);
//     }


//     /* 去掉符号位，仅保留角度 */
//     raw &= (BISS_FULL_SCALE - 1);

//     /* 角度映射 */
//     float angle = (float)raw * 360.0f / (float)BISS_FULL_SCALE;

//     /* 保留两位小数 */
//     angle = (float)((int32_t)(angle * 100.0f + 0.5f)) / 100.0f * 2.0f;

//     return angle;
// }


//     /* 电机 1 → 编码器 1 */
//     BISS_Encoder_t encoder_motor1 =
//     {
//         .clk_port = GPIOC,
//         .clk_pin  = GPIO_PIN_6,
//         .dat_port = GPIOC,
//         .dat_pin  = GPIO_PIN_7
//     };

//     /* 电机 2 → 编码器 2 */
//     BISS_Encoder_t encoder_motor2 =
//     {
//         .clk_port = GPIOA,
//         .clk_pin  = GPIO_PIN_15,
//         .dat_port = GPIOB,
//         .dat_pin  = GPIO_PIN_3
//     };
    
    
//     float motor1_pos, motor2_pos;
//     float  motor1_crc_ok, motor2_crc_ok;


// !下述为考虑CRC校验的读取函数
#include "biss.h"
// #include "delay.h"
#include "main.h"

// ! 根据 SSI 通信协议的帧格式，读取编码器角度
// ! BISS 格式的帧在起始时存在 0_1_0 的帧头

/* ===== GPIO 快速操作宏 ===== */
#define GPIO_SET(port, pin)     ((port)->BSRR = (pin))
#define GPIO_CLR(port, pin)     ((port)->BSRR = ((uint32_t)(pin) << 16U))
#define GPIO_READ(port, pin)    (((port)->IDR & (pin)) ? 1 : 0)


// 根据编码器手册定义
#define MP55_ST_BITS      24          // 绝对位置位数
#define MP55_ERR_BIT      1           // 错误位
#define MP55_WARN_BIT     1           // 报警位
#define MP55_CRC_BITS     6           // CRC位数
#define MP55_DATA_BITS    ( MP55_ST_BITS + MP55_ERR_BIT + MP55_WARN_BIT) // 数据位数，26位
#define MP55_FRAME_BITS   ( MP55_ST_BITS + MP55_ERR_BIT + MP55_WARN_BIT + MP55_CRC_BITS) // 总帧位数，32位

// 编码器角度转换比例
#define MP55_FULL_SCALE (1UL << MP55_ST_BITS) // 2^24
#define M_PI 3.1415926535897932384626


// 编码器零位
#define MP55_1_ZERO_POS 0x00000000 // 电机 1 编码器零位   //! 与offset偏置量一样？
#define MP55_2_ZERO_POS 0x00000000 // 电机 2 编码器零位


// 电机 1 → 编码器 1
MP55_Encoder_t encoder_motor1 = 
{
    .sck_gpio_port = PM1_ENC_SCK_GPIO_Port,
    .sck_gpio_pin = PM1_ENC_SCK_Pin,
    .dat_gpio_port = PM1_ENC_DAT_GPIO_Port,
    .dat_gpio_pin = PM1_ENC_DAT_Pin,

    .abs_pos = 0,
    .n_err = false,
    .n_warn = false,
    .crc = 0,

    .pos_deg = 0.0,
    .pos_rad = 0.0,

    .zero_pos = MP55_1_ZERO_POS
};

// 电机 2 → 编码器 2
MP55_Encoder_t encoder_motor2 = 
{
    .sck_gpio_port = PM2_ENC_SCK_GPIO_Port,
    .sck_gpio_pin = PM2_ENC_SCK_Pin,
    .dat_gpio_port = PM2_ENC_DAT_GPIO_Port,
    .dat_gpio_pin = PM2_ENC_DAT_Pin,

    .abs_pos = 0,
    .n_err = false,
    .n_warn = false,
    .crc = 0,

    .pos_deg = 0.0,
    .pos_rad = 0.0,

    .zero_pos = MP55_2_ZERO_POS
};

    volatile int a = 0;
    volatile int b = 0;

/**
 * @brief 读取编码器角度并进行 CRC 校验
 * @param enc 编码器结构体指针
 * @return 角度值 (度)`
 */
double MP55_ReadFrame(MP55_Encoder_t *enc)
{

    a++;

    volatile uint8_t bit[MP55_FRAME_BITS]; // 用于存储当前数据位

    // 1. 读取完整的 MP55_FRAME_BITS 位数据帧
    GPIO_CLR(enc->sck_gpio_port, enc->sck_gpio_pin); // 低电平开始
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();// 每个 __NOP() 20 ns @ 168MHz
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();// 每个 __NOP() 20 ns @ 168MHz
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();// 每个 __NOP() 20 ns @ 168MHz
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();// 每个 __NOP() 20 ns @ 168MHz
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();// 每个 __NOP() 20 ns @ 168MHz
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    for (uint8_t i = 0; i < MP55_FRAME_BITS; i++) // 一个循环读取一个位
    {
        GPIO_SET(enc->sck_gpio_port, enc->sck_gpio_pin); // clk 高电平
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();// 每个 __NOP() 20 ns @ 168MHz
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();// 每个 __NOP() 20 ns @ 168MHz
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();// 每个 __NOP() 20 ns @ 168MHz
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();// 每个 __NOP() 20 ns @ 168MHz
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();// 每个 __NOP() 20 ns @ 168MHz
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();

        bit[i] = GPIO_READ(enc->dat_gpio_port, enc->dat_gpio_pin); // 下降沿读取数据位

        GPIO_CLR(enc->sck_gpio_port, enc->sck_gpio_pin); // clk 低电平
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();// 每个 __NOP() 20 ns @ 168MHz
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();// 每个 __NOP() 20 ns @ 168MHz
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();// 每个 __NOP() 20 ns @ 168MHz
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();// 每个 __NOP() 20 ns @ 168MHz
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();// 每个 __NOP() 20 ns @ 168MHz
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    }
    GPIO_SET(enc->sck_gpio_port, enc->sck_gpio_pin); // 高电平结束
    // 2. 构建原始帧 MSB
    uint64_t raw_frame = 0;
    uint32_t abs_pos = 0;
    bool n_err = false;
    bool n_warn = false;        
    volatile uint8_t n_crc = 0;

    for (uint8_t i = 0; i < MP55_FRAME_BITS; i++)
    {
        raw_frame = (raw_frame << 1) | (bit[i] & 0x01);  

        if (i < MP55_ST_BITS)
        {
            abs_pos = (abs_pos << 1) | (bit[i] & 0x01);
        }
        else if (i >= MP55_ST_BITS && i < MP55_ST_BITS + MP55_ERR_BIT)
        {
            n_err = (n_err << 1) | (bit[i] & 0x01);
            // n_err = bit[i];
        }
        else if (i >= MP55_ST_BITS + MP55_ERR_BIT && i < MP55_ST_BITS + MP55_ERR_BIT + MP55_WARN_BIT)
        {
            n_warn = (n_warn << 1) | (bit[i] & 0x01); 
            // n_warn = bit[i];
        }
        else if (i >= MP55_ST_BITS + MP55_ERR_BIT + MP55_WARN_BIT && i < MP55_FRAME_BITS)
        { 
            n_crc = (n_crc << 1) | (bit[i] & 0x01);
        }
    }

    // 3. 校验 CRC 多项式为 0x43 起始值 0x00
    volatile int64_t raw_data = raw_frame >> (MP55_FRAME_BITS - MP55_DATA_BITS);
    volatile uint8_t crc_ok = MP55_CRC6_Calc(raw_data);
    volatile uint8_t crc = ~(n_crc) & 0x3F;

    // 4. 计算角度
    if (crc_ok == crc)
    // if (crc_ok == ((~crc) & 0x3F))
    {
    

        b++;
        

        enc->abs_pos = abs_pos;
        enc->n_err = n_err;
        enc->n_warn = n_warn;
        // enc->crc = crc;

        int32_t abs_pos_calibrated = (int32_t)((int64_t)abs_pos - (int64_t)enc->zero_pos);
        enc->pos_deg = (double)abs_pos_calibrated/(double)MP55_FULL_SCALE * (double)(360.0);
        enc->pos_rad = (double)abs_pos_calibrated/(double)MP55_FULL_SCALE * (double)(M_PI)*(double)(2.0);
    }

    
    return enc->pos_deg;
}

/**
 * @brief 设置编码器零点位置
 * @param enc 编码器结构体指针
 * @param zero_pos 零点位置
 */
void MP55_SetZeroPos(MP55_Encoder_t *enc, uint32_t zero_pos)
{
    enc->zero_pos = zero_pos;
}


/**
 * @brief 计算 CRC-6 校验和
 * @param data 输入数据
 * @return CRC-6 校验和
 */
uint8_t MP55_CRC6_Calc(uint64_t data)
{
    uint8_t crc = 0x00;
    uint8_t poly = 0x43;
    uint8_t poly_full = poly & 0x7F;

    for (uint8_t i = 0; i < MP55_DATA_BITS; i++)
    {
        uint8_t data_bit = (data >> (MP55_DATA_BITS - 1 - i)) & 0x01;

        uint8_t feedback = ((crc >> 5) & 0x01) ^ data_bit;

        crc = (crc << 1) & 0x7F;

        if (feedback)
        {
            crc ^= poly_full;
        }
    }

    return crc & 0x3F;
}