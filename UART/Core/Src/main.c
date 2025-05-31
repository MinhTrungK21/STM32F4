
/*
 * main.c
 *
 *  Created on: Mar 23, 2025
 *      Author: Dell
 */
#include"main.h"
#include"stdio.h"
#include"string.h"
#define GPIOD_BASE_ADDR  0x40020C00
#define GPIOA_BASE_ADDR  0x40020000
#define RCC_BASE_ADDR    0x40023800
#define EXTI_BASE_ADDR    0x40013C00
#define SYSCFG_BASE_ADDR  0x40013800
#define NVIC_ISER0        ((volatile uint32_t*)0xE000E100)  // Interrupt Set-Enable Register
#define NVIC_ISER1        ((volatile uint32_t*)0xE000E104)
#define UART1_BASE_ADDR 0x40011000
#define GPIOB_BASE_ADDR 0x40020400
uint32_t* RCC_AHB1ENR  = (uint32_t*)(RCC_BASE_ADDR + 0x30);
uint32_t* RCC_APB2ENR  = (uint32_t*)(RCC_BASE_ADDR + 0x44);
uint32_t* GPIOD_MODER  = (uint32_t*)(GPIOD_BASE_ADDR + 0x00);
uint32_t* GPIOD_BSRR   = (uint32_t*)(GPIOD_BASE_ADDR + 0x18);
uint32_t* GPIOA_IDR    = (uint32_t*)(GPIOA_BASE_ADDR + 0x10);
uint32_t* GPIOA_PUPDR  = (uint32_t*)(GPIOA_BASE_ADDR + 0x0C);
uint32_t* GPIOA_MODER  = (uint32_t*)(GPIOA_BASE_ADDR + 0x00);
uint32_t* GPIOD_ODR    = (uint32_t*)(GPIOA_BASE_ADDR + 0x014);
uint32_t* GPIOB_MODER  = (uint32_t*)(GPIOB_BASE_ADDR);
uint8_t   led_state    = 0;

// Thanh ghi EXTI
uint32_t* EXTI_IMR  = (uint32_t*)(EXTI_BASE_ADDR + 0x00);
uint32_t* EXTI_RTSR = (uint32_t*)(EXTI_BASE_ADDR + 0x08);
uint32_t* EXTI_FTSR = (uint32_t*)(EXTI_BASE_ADDR + 0x0C);
uint32_t* EXTI_PR   = (uint32_t*)(EXTI_BASE_ADDR + 0x14);

// SYSCFG
uint32_t* SYSCFG_EXTICR1 = (uint32_t*)(SYSCFG_BASE_ADDR + 0x08);
//FLASH
uint32_t* FLASH_SR 	 =  (uint32_t*)(0x40023C00+0x0C);
uint32_t* FLASH_CR   =  (uint32_t*)(0x40023C00+0x10);
uint32_t* FLASH_KEYR =  (uint32_t*)(0x40023C00+0x04);
typedef enum
{
	Green_LED  =12,
	Yellow_LED =13,
	Red_LED    =14,
	Blue_LED   =15,
}
LED_t;

void LedCtrl(LED_t led, int LED_stt)
{
#if 0
	if (LED_stt ==1)
	{
		*GPIOD_ODR |=(1<<12);
	else
		*GPIOD_ODR |=(1<<12)
	}
#else
	if (LED_stt ==1)
	{
		*GPIOD_BSRR|=(1<<led);
	}
	else
	{
		*GPIOD_BSRR=(1<<(led+16));
	}
#endif
}
char ButtonState()
{
	uint32_t* GPIOA_IDR = (uint32_t*)(GPIOA_BASE_ADDR + 0x10);
	return (*GPIOA_IDR >> 0) & 1;
}
void function()
{
	if(ButtonState())
		LedCtrl(Red_LED, 1);
	else
		LedCtrl(Red_LED, 0);
}
void LedInit()
{

	__HAL_RCC_GPIOD_CLK_ENABLE();
	uint32_t* GPIOD_MODER = (uint32_t*)(GPIOD_BASE_ADDR + 0x00);
	*GPIOD_MODER &= ~(0xff << 24);
	*GPIOD_MODER |=  (0b01 << 24);        // PD12 output
	*GPIOD_MODER |=  (0b01 << (28));  // PD14 output
}



void EXTI0Init()
{
	/*
	- Làm sao để EXTI0 gửi interrupt signal lên ARM?
		+ chọn cạnh (rising/falling/cả hai)
			+ set trong thanh ghi EXTI_RTSR và EXTI_FTSR
		+ enable exti0(set mark)
			+ set trong thanh ghi EXTI_IMR
	- ARM (NVIC) phải chấp nhận interrupt signal từ EXTI gửi lên?
		+ bước 1: xác định EXTI0 nằm ở position bao nhiêu trong vector table? (mở vector table ở chapter "10: interrupts and events" trong reference manual) --> 6
		+ bước 2: enable interrupt cho position 6
	*/
	*RCC_AHB1ENR  |= (1 << 0);   // GPIOA
	*RCC_APB2ENR  |= (1 << 14);  // SYSCFG

	*GPIOA_MODER &= ~(0b11 << 0);
    *GPIOA_PUPDR &= ~(0b11 << 0);  // Xóa cấu hình cũ của Pull-up/Pull-down
    *GPIOA_PUPDR |=  (0b10 << 0);  // Kích hoạt Pull-down (01)


    SYSCFG->EXTICR[0] &= ~(0xF << 0); //chon ngat pa0

    //ngat 2 canh
    EXTI->IMR  |= (1 << 0);
    EXTI->RTSR |= (1 << 0);  // Rising edge
    EXTI->FTSR |= (1 << 0);  // Falling edge

    //cho phep NVIC nhan ngat tu pa0
    *NVIC_ISER0 |= (1 << 6);
    // Move vector table lên RAM (0x20000000)
    	uint8_t* src = 0;
    	uint8_t* dis = (uint8_t*)0x20000000;
    	for(int i = 0; i < 0x198; i++)
    	{
    		//dis[i] = src[i];
    		*(dis+i) = *(src+i);
    	}
    	//Báo ARM vector table đã được offset lên RAM
    	uint32_t* VTOR = (uint32_t*)0xE000ED08;
    	*VTOR = 0x20000000;
    	//
    	int* ptr;
    	ptr= (int*)0x20000058;
    	*ptr = (int)function;

}


// Hàm xử lý ngắt EXTI0

void EXTI0_IRQHandler(void)
{
	if(ButtonState())
		LedCtrl(Red_LED, 1);
	else
		LedCtrl(Red_LED, 0);

	//clear interrupt flag
	uint32_t* EXTI_PR = (uint32_t*)(EXTI_BASE_ADDR + 0x14);
	*EXTI_PR |= (1<<0);
}
void UART_Init()
{

	__HAL_RCC_GPIOB_CLK_ENABLE();
	uint32_t* GPIOB_MODER = (uint32_t*)(GPIOB_BASE_ADDR + 0X00);
	*GPIOB_MODER &=(0b1111<12);
	*GPIOB_MODER |=(0b10<<12)|(0b10<<14);


	uint32_t* GPIOB_AFLR = (uint32_t*)(GPIOB_BASE_ADDR+0x20);
	*GPIOB_AFLR &=~(0xff<24);
	*GPIOB_AFLR |=(0b0111<<24)|(0b0111<<28);


	__HAL_RCC_USART1_CLK_ENABLE(); //16MHz
	uint32_t* BRR = (uint32_t*)(UART1_BASE_ADDR+0x08);
	*BRR = (104<<4)|(3<<0);
	uint32_t* CR1 = (uint32_t*)(UART1_BASE_ADDR+0x0C);
	*CR1 |= (1<<12)|(1<<10)|(1<<3)|(1<<2)|(1<<13);


	/* enable interrupt*/
	*CR1 |= (1<<5);
	*NVIC_ISER1 |= (1 << 5);


}



void UART_Transmit(uint8_t data)
{
	uint32_t* DR = (uint32_t*)(UART1_BASE_ADDR+0x04);
	uint32_t* SR = (uint32_t*)(UART1_BASE_ADDR+0x00);
	while (((*SR>>7)&1)==0);

	*DR |= data;
	while (((*SR>>6)&1)==0);

}
void UART_Print_Log(char* msg)
{
	int msg_len =  strlen(msg);
	for (int i =0; i<msg_len; i++)
	{

		UART_Transmit((uint8_t)msg[i]);
	}
}
char UART_Receive()
{
	uint32_t* DR = (uint32_t*)(UART1_BASE_ADDR+0x04);
	uint32_t* SR = (uint32_t*)(UART1_BASE_ADDR+0x00);
	while (((*SR>>5)&1)==0);
	char data=*DR;
	return data;
}
char StoreData[100];
int idx;

void USART1_IRQHandler()
{
	StoreData[idx++]= UART_Receive();

}


int main()
{
	HAL_Init();
	LedInit();
	EXTI0Init();
	UART_Init();


	while (1)
	{

		UART_Transmit('x');
		char data=UART_Receive();
		if(UART_Receive()=='a')
		{
			LedCtrl(Red_LED, 1);
		}
		else
		{
			LedCtrl(Red_LED, 0);
		}
		LedCtrl(Blue_LED,1);
		HAL_Delay(1000);
		LedCtrl(Blue_LED,0);
		HAL_Delay(1000);
	}
	return 0;
}
