#ifndef __UTILITY_H__
#define __UTILITY_H__

#ifdef __cplusplus
extern "C" {
#endif

void dump_buffer_hex(int indent, const unsigned char *data, int size);
void bell();

#ifdef __cplusplus
}
#endif

#endif /* __UTILITY_H__ */
