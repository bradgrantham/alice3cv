#ifndef __UART_H__
#define __UART_H__

#ifdef __cplusplus
extern "C" {
#endif

void SERIAL_init();
void SERIAL_send_one_char(char c);

#ifdef __cplusplus
}
#endif

#endif /* __UART_H__ */
