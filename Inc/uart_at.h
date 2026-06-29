#ifndef UART_AT_H_
#define UART_AT_H_

void UART_AT_Init(void);     // enable RXNE interrupt (call after MX_USART1_UART_Init)
void UART_AT_RxISR(void);    // call from USART1_IRQHandler
void UART_AT_Poll(void);     // call from main loop

#endif /* UART_AT_H_ */
