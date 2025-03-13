#include <stm32f411xe.h>

void SystemInit(){
	SCB->CPACR |= (0b11 << 10*2) | (0b11 << 11*2); // set CP10 and CP11 to full access (FPU)
}

void SystemInitError(uint16_t error_source) {
	(void) error_source;
	while(1);
}
