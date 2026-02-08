#include "gpio_dev.h"




void set_led(uint8_t led_index, uint8_t status)
{
    if (led_index == 0)
    {
        HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, status ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
    else if (led_index == 1)
    {
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, status ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}




void toggle_led(uint8_t led_index)
{
    if (led_index == 0)
    {
        HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    }
    else if (led_index == 1)
    {
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    }
}


uint8_t read_key(uint8_t key_index)
{
    if (key_index == 0)
    {
        return HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin);
        
        
    }
    else if (key_index == 1)
    {
        return HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);
        
    }
    return 0;
}


