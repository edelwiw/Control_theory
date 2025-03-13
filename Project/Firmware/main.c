#include <stm32f411xe.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define F_CPU 16000000UL 

#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))

volatile uint32_t time = 0; 
volatile uint32_t target_pos = 100;

struct {
	double cart_pos;
	double pend_angle;

	double cart_speed;
	double pend_speed;

	double cart_acc;
	double pend_acc;

	double cart_last[2];
	double pend_last[2];
	double cart_speed_last[2];
	double pend_speed_last[2];
	double cart_acc_last[2];
	double pend_acc_last[2];
} telemetry;


void usart2_write_buf(char *buf, size_t len){
	for(size_t i = 0; i < len; i++){
		while (!(USART2->SR & USART_SR_TXE));
		USART2->DR = buf[i];
	}
}


int main(){	
	// GPIO init 
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // enable GPIOA clock 
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // enable GPIOB clock
	// USART init 
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // enable USART2 clock
	// TIMERS init 
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // enable TIM2 clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // enable TIM3 clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; // enable TIM4 clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; // enable TIM5 clock

	GPIOA->MODER |= GPIO_MODER_MODER5_0; // set PA5 as output
	GPIOA->BSRR = GPIO_BSRR_BS_5; // set PA5 high

	// USART init 
	GPIOA -> MODER |= (0b10 << GPIO_MODER_MODE2_Pos) | (0b10 << GPIO_MODER_MODE3_Pos);  // set PA2 and PA3 to alternate function
	GPIOA -> AFR[0] |= (7 << GPIO_AFRL_AFSEL2_Pos) | (7 << GPIO_AFRL_AFSEL3_Pos); // set PA2 and PA3 to AF7
	USART2->BRR = F_CPU / 115200; // set baudrate to 115200
	USART2->CR1 = USART_CR1_UE | USART_CR1_TE; // enable USART2 and enable transmit

	// pwm on TIM2 (motor control)
	TIM2->PSC = 16 - 1; // set prescaler 1 MHz
	TIM2->ARR = 100 - 1; 

	// PA0, PA1 as alternate function
	GPIOA->MODER |= (0b10 << GPIO_MODER_MODE0_Pos) | (0b10 << GPIO_MODER_MODE1_Pos);
	GPIOA->AFR[0] |= (1 << GPIO_AFRL_AFSEL0_Pos) | (1 << GPIO_AFRL_AFSEL1_Pos); // set PA0 and PA1 to AF1

	TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E; // enable channel capture/compare 1, 2 output	
	TIM2->CCMR1 |= (0b110 << TIM_CCMR1_OC1M_Pos); // PWM mode 1 on channel 1
	TIM2->CCMR1 |= (0b110 << TIM_CCMR1_OC2M_Pos); // PWM mode 1 on channel 2

	TIM2->CCMR1 |= TIM_CCMR1_OC1PE; // enable preload register on channel 1
	TIM2->CCMR1 |= TIM_CCMR1_OC2PE; // enable preload register on channel 2

	TIM2->CR1 |= TIM_CR1_CEN; // enable counter
	TIM2->EGR |= TIM_EGR_UG; // update generation

	TIM2->CCR1 = 0; 
	TIM2->CCR2 = 0;

	
	// TIM3 as encoder (cart)
	TIM3->SMCR |= (0b01 << TIM_SMCR_SMS_Pos); // Encoder Mode 1 (TI2FP1 counting only)
	// set GPIOA 6 and 7 to alternate function
	GPIOA->MODER |= (2 << GPIO_MODER_MODE6_Pos) | (2 << GPIO_MODER_MODE7_Pos);
	GPIOA->AFR[0] |= (2 << GPIO_AFRL_AFSEL6_Pos) | (2 << GPIO_AFRL_AFSEL7_Pos); // set PA6 and PA7 to AF2
	TIM3->CCMR1 |= (0b1 << TIM_CCMR1_CC1S_Pos) | (0b1 << TIM_CCMR1_CC2S_Pos); // set both channels to TI1 and TI2 respectively
	TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E; // enable both channels
	// invert the polarity of the second channel
	// TIM3->CCER |= TIM_CCER_CC2P;
	TIM3->CNT = 30000; // set counter 
	TIM3->CR1 |= TIM_CR1_CEN; // enable counter

	NVIC_EnableIRQ(TIM3_IRQn); // enable TIM3 interrupt

	
	// TIM4 as encoder (pendulum)
	TIM4->SMCR |= (0b01 << TIM_SMCR_SMS_Pos); // Encoder Mode 1 (TI2FP1 counting only)
	// set GPIOB 6 and 7 to alternate function
	GPIOB->MODER |= (2 << GPIO_MODER_MODE6_Pos) | (2 << GPIO_MODER_MODE7_Pos);
	GPIOB->AFR[0] |= (2 << GPIO_AFRL_AFSEL6_Pos) | (2 << GPIO_AFRL_AFSEL7_Pos); // set PB6 and PB7 to AF2
	TIM4->CCMR1 |= (0b1 << TIM_CCMR1_CC1S_Pos) | (0b1 << TIM_CCMR1_CC2S_Pos); // set both channels to TI1 and TI2 respectively
	TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E; // enable both channels
	// invert the polarity of the second channel
	TIM4->CCER |= TIM_CCER_CC2P;
	TIM4->CNT = 30000; // set counter 
	TIM4->CR1 |= TIM_CR1_CEN; // enable counter


	// TIM5 enable (millis timer)
	TIM5->PSC = 16 - 1; // set prescaler 
	TIM5->ARR = 1000 - 1; // 1 ms
	TIM5->DIER |= TIM_DIER_UIE; // enable update interrupt
	TIM5->CR1 |= TIM_CR1_CEN; // enable counter

	NVIC_EnableIRQ(TIM5_IRQn); // enable TIM5 interrupt
	NVIC_SetPriority(TIM5_IRQn, 0);

	
	

	NVIC_EnableIRQ(TIM4_IRQn); // enable TIM4 interrupt



	while(1){}
}

void apply_control(int32_t u){
	u = max(-100, min(100, u));
	double scale = 0.5;
	if(u > 0){
		TIM2->CCR1 = (uint32_t) (u * scale);
		TIM2->CCR2 = 0;
	} else {
		TIM2->CCR1 = 0;
		TIM2->CCR2 = (uint32_t)  (-u * scale);
	}
}

void write_telemetry(){
	char buf[100];
	sprintf(buf, ">c: %.2f\n>p: %.2f\n", telemetry.cart_pos, telemetry.pend_angle);
	usart2_write_buf(buf, strlen(buf));
}


void TIM5_IRQHandler(){
	TIM5->SR &= ~TIM_SR_UIF; // clear update interrupt flag
	GPIOA->ODR ^= GPIO_ODR_OD5; // toggle PA5
	time++; // millis timer 

	// update telemetry
	telemetry.cart_pos = ((int32_t) TIM3->CNT - 30000) / 2000.0 * 92.0;
	telemetry.pend_angle = ((int32_t) TIM4->CNT - 30000) / 2000.0 * 360.0;

	if (time % 100 == 0) write_telemetry();
}

