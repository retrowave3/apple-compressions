#ifndef LZMESH_H
#define LZMESH_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum lzmesh_algorithm {
    LZMESH_0 = 0xE00,
    LZMESH_1 = 0xE01,
    LZMESH_5 = 0xE05,
    LZMESH_9 = 0xE09
};

size_t lzmesh_encode_bound(size_t size);
size_t lzmesh_encode_buffer(void *destination, size_t capacity, const void *source, size_t size, uint32_t algorithm);
size_t lzmesh_decode_buffer(void *destination, size_t capacity, const void *source, size_t size);

#ifdef __cplusplus
}
#endif

#endif
