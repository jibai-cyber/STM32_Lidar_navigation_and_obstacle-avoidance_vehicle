#ifndef _USART_H
#define _USART_H

void uart(void);
void UARTx_SendByte(USART_TypeDef* USARTx,uint8_t Byte);
void Serial_SendString(USART_TypeDef* USARTx,char Strings[],FlagStatus LineBreak);



#endif

