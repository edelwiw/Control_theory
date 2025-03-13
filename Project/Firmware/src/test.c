#include <stm32f411xe.h>

void toggle_led(){
    GPIOA->ODR ^= GPIO_ODR_OD5; // toggle PA5
}