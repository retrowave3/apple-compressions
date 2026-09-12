#include "../lzbitmap.h"
#include "../lzmesh.h"
#include "../lzraven.h"

#undef NDEBUG
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

struct test_vector {
    uint32_t algorithm;
    size_t source_size;
    size_t encoded_size;
    const uint8_t *source;
    const uint8_t *encoded;
};

#include "vectors.h"

static void test_lzbitmap(void)
{
    uint8_t output[65536];
    size_t count = sizeof(bitmap_vectors) / sizeof(bitmap_vectors[0]);

    for (size_t i = 0; i < count; ++i) {
        const struct test_vector *test = &bitmap_vectors[i];
        assert(test->source_size <= sizeof(output));
        assert(test->encoded_size <= sizeof(output));

        size_t size = lzbitmap_encode_buffer(output, sizeof(output), test->source, test->source_size, test->algorithm);
        assert(size == test->encoded_size);
        assert(memcmp(output, test->encoded, size) == 0);

        size = lzbitmap_decode_buffer(output, sizeof(output), test->encoded, test->encoded_size, test->algorithm);
        assert(size == test->source_size);
        assert(memcmp(output, test->source, size) == 0);
    }
    printf("lzbitmap: %zu encode/decode pairs passed\n", count);
}

static void test_lzmesh(void)
{
    uint8_t output[65536];
    size_t count = sizeof(mesh_vectors) / sizeof(mesh_vectors[0]);

    for (size_t i = 0; i < count; ++i) {
        const struct test_vector *test = &mesh_vectors[i];
        assert(test->source_size <= sizeof(output));
        assert(test->encoded_size <= sizeof(output));

        size_t size = lzmesh_encode_buffer(output, sizeof(output), test->source, test->source_size, test->algorithm);
        assert(size == test->encoded_size);
        assert(memcmp(output, test->encoded, size) == 0);

        size = lzmesh_decode_buffer(output, sizeof(output), test->encoded, test->encoded_size);
        assert(size == test->source_size);
        assert(memcmp(output, test->source, size) == 0);
    }
    printf("lzmesh: %zu encode/decode pairs passed\n", count);
}

static void test_lzraven(void)
{
    uint8_t output[65536];
    size_t count = sizeof(raven_vectors) / sizeof(raven_vectors[0]);

    for (size_t i = 0; i < count; ++i) {
        const struct test_vector *test = &raven_vectors[i];
        assert(test->source_size <= sizeof(output));
        assert(test->encoded_size <= sizeof(output));

        size_t size = lzraven_encode_buffer(output, sizeof(output), test->source, test->source_size);
        assert(size == test->encoded_size);
        assert(memcmp(output, test->encoded, size) == 0);

        size = lzraven_decode_buffer(output, sizeof(output), test->encoded, test->encoded_size);
        assert(size == test->source_size);
        assert(memcmp(output, test->source, size) == 0);
    }
    printf("lzraven: %zu encode/decode pairs passed\n", count);
}

int main(void)
{
    test_lzbitmap();
    test_lzmesh();
    test_lzraven();
    return 0;
}
