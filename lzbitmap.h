#ifndef LZBITMAP_H
#define LZBITMAP_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum lzbitmap_algorithm {
    LZBITMAP_FAST_0 = 0x600,
    LZBITMAP_FAST_1 = 0x601,
    LZBITMAP_FAST_2 = 0x602,
    LZBITMAP_0 = 0x700,
    LZBITMAP_1 = 0x701,
    LZBITMAP_2 = 0x702
};

size_t lzbitmap_encode_bound(size_t size, uint32_t algorithm);
size_t lzbitmap_encode_buffer(void *destination, size_t capacity, const void *source, size_t size, uint32_t algorithm);
size_t lzbitmap_decode_buffer(void *destination, size_t capacity, const void *source, size_t size, uint32_t algorithm);

#ifdef __cplusplus
}
#endif

#endif
