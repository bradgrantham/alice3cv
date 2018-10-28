#ifndef __DEFS_H__
#define __DEFS_H__

#ifdef __cplusplus
extern "C" {
#endif

#define XSTR(x) STR(x)
#define STR(x) # x

#define enable_interrupts() __enable_irq()
#define disable_interrupts() __disable_irq()

extern void panic(void);

#ifdef __cplusplus
}
#endif

#endif /* __DEFS_H__ */
