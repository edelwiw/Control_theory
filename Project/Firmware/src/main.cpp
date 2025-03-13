#include <stm32f411xe.h>

extern "C" {
	void toggle_led();
}

#define F_CPU 16000000UL 

class GPIO{
public:
	void init(){
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // enable GPIOA clock 
		GPIOA->MODER |= GPIO_MODER_MODER5_0; // set PA5 as output
		GPIOA->BSRR = GPIO_BSRR_BS_5; // set PA5 high
	}

	void toggle(){
		// GPIOA->ODR ^= GPIO_ODR_OD5; // toggle PA5
		toggle_led();
	}
};


int main(){	
	// GPIO init 
	
	GPIO gpio;
	gpio.init();

	while(1){
		gpio.toggle();
		for(int i = 0; i < 1000000; i++);
	}
}
