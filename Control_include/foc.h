    #ifndef __FOC_H
    #define __FOC_H
    
    #include "svpwm.h"
    #include "stm32f4xx_hal.h"  // 添加 HAL 库头文件
    #include <math.h>
    #include "biss.h" 
    
    // PI 控制器结构体
    typedef struct {
        float Kp;          // 比例系数
        float Ki;          // 积分系数
        float integral;    // 积分值
        float error_prev;  // 上次误差
        float output;      // 输出值
        float output_max;  // 输出最大值
        float output_min;  // 输出最小值
    } PI_Controller;
    
    // FOC 电机结构体
    typedef struct {
        // 电机参数
        float Rs;          // 定子电阻
        // float Ld;          // d 轴电感
        // float Lq;          // q 轴电感
        // float flux_linkage; // 永磁体磁链
        uint8_t pole_pairs; // 极对数
        
        // 电流反馈
        float Ia;          // A 相电流
        float Ib;          // B 相电流
        float Ic;          // C 相电流
        float Ialpha;      // α轴电流
        float Ibeta;       // β轴电流
        float Id;          // d 轴电流
        float Iq;          // q 轴电流
        
        // 电压指令
        float Valpha;      // α轴电压
        float Vbeta;       // β轴电压
        float Vd;          // d 轴电压
        float Vq;          // q 轴电压
        
        // 转子信息
        float theta_e;     // 电角度
        float theta_e_prev;
        float theta_m;     // 机械角度
        float speed_e;     // 电角速度
        float speed_m;     // 机械转速
        
        // 控制目标
        float Id_ref;      // d 轴电流参考
        float Iq_ref;      // q 轴电流参考
        float speed_ref;   // 转速参考
        
        // PI 控制器
        PI_Controller pi_id;  // d 轴电流环
        PI_Controller pi_iq;  // q 轴电流环
        PI_Controller pi_speed; // 速度环
        
        // SVPWM
        SVPWM_TypeDef svpwm;
        
        // 硬件相关
        TIM_TypeDef *TIM_PWM; // PWM 定时器
        float Vdc;            // 直流母线电压
    
        // === 新增编码器指针 ===
        MP55_Encoder_t *encoder;  // 指向对应的编码器实例

        // === 新增ADC ===
        uint8_t adc_idx_u;      // U 相 ADC 缓冲区索引
        uint8_t adc_idx_v;      // V 相 ADC 缓冲区索引
        uint8_t adc_idx_w;      // V 相 ADC 缓冲区索引
        float* p_current_offset; // 指向对应电机的偏置值
    } FOC_Motor;
    // 定义两个电机
    extern FOC_Motor motor1, motor2;
    
        typedef struct
    {
        float offset_a;
        float offset_b;
        float offset_c;
    } CurrentOffset_t;
    extern CurrentOffset_t M1_Offset;
    extern CurrentOffset_t M2_Offset;       

    extern volatile uint8_t adc1_ready;
    extern volatile uint8_t adc2_ready;
    // extern volatile uint8_t adc3_ready;

    
    
    // 函数声明
    void FOC_Init(FOC_Motor *motor, TIM_TypeDef *TIMx, float Vdc, uint8_t pole_pairs, MP55_Encoder_t *encoder, uint8_t adc_idx_u, uint8_t adc_idx_v, uint8_t adc_idx_w);
    void FOC_ClarkeTransform(FOC_Motor *motor);
    void FOC_ParkTransform(FOC_Motor *motor);
    void FOC_InverseParkTransform(FOC_Motor *motor);
    void FOC_PI_Update(PI_Controller *pi, float error, float Ts);
    void FOC_SetDutyCycle(FOC_Motor* motor, TIM_TypeDef* timer);
    
    void Loop_Speed(FOC_Motor *motor, float Ts);
    void Loop_Current(FOC_Motor *motor, float Ts);

    float Pos_ElecTheta(FOC_Motor *motor);
    float M1_Speed_Get(float M1_Pos_MechTheta, float M1_Time_interval);
    float M2_Speed_Get(float M2_Pos_MechTheta, float M2_Time_interval);
    float M1_Speed_Filter(float M1_Temp_spd_dms);
    float M2_Speed_Filter(float M2_Temp_spd_dms);


    void CurrentSensorSuite(FOC_Motor *motor);  
    // extern float ADC_To_Current(uint16_t adc);//ADC电流采集
    float ADC_To_Current_Offset(uint16_t adc);
    
    extern void Current_Offset_Calibration();   //电流偏置校准
    
    
    extern TIM_HandleTypeDef htim1;  // 添加外部声明
    extern TIM_HandleTypeDef htim8;  // 添加外部声明
    
    #endif