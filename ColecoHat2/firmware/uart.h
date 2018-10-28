#ifndef __UART_H__
#define __UART_H__

#ifdef __cplusplus
extern "C" {
#endif

void SERIAL_try_to_transmit_buffers();
void SERIAL_flush();
void SERIAL_enqueue_one_char(char c);
extern void uart_received(char c);
void SERIAL_init();
void SERIAL_poll_continue();

#ifdef __cplusplus
}
#endif

#endif /* __UART_H__ */
