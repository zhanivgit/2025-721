#include "stm32f10x.h"
#include <stdio.h>
#include <stdarg.h>

char Serial_RxPacket[100];          // ????? "@MSG\r\n"
uint8_t Serial_RxFlag = 0;          // ??????

// ???????
void Serial_Init(void)
{
    // 1. ????
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  // USART2?APB1???
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   // GPIOA?APB2???
    
    // 2. ??GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // PA2??USART2?TX (??????)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // PA3??USART2?RX (????)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 3. ??USART2
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART2, &USART_InitStructure);
    
    // 4. ????
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);  // ??????
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  // ???????
    
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
    
    // 5. ??USART2
    USART_Cmd(USART2, ENABLE);
}

// ??????
void Serial_SendByte(uint8_t Byte)
{
    USART_SendData(USART2, Byte);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);  // ??????
}

// ??????
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i++)
    {
        Serial_SendByte(Array[i]);
    }
}

// ?????
void Serial_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)
    {
        Serial_SendByte(String[i]);
    }
}

// ?????
uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--)
    {
        Result *= X;
    }
    return Result;
}

// ????
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
    }
}

// ???printf
int fputc(int ch, FILE *f)
{
    Serial_SendByte(ch);
    return ch;
}

// ?????
void Serial_Printf(char *format, ...)
{
    char String[100];
    va_list arg;
    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);
    Serial_SendString(String);
}

// USART2??????
void USART2_IRQHandler(void)
{
	static uint8_t pRxPacket = 0;
	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		uint8_t RxData = USART_ReceiveData(USART2);
		
		if (Serial_RxFlag == 0)
		{
			if (RxData != '\n')
			{
				if (RxData != '\r')
				{
					Serial_RxPacket[pRxPacket] = RxData;
					pRxPacket++;
				}
			}
			else
			{
				Serial_RxPacket[pRxPacket] = '\0';
				pRxPacket = 0;
				Serial_RxFlag = 1;
			}
		}
		
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}
}
