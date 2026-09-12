#include "lzmesh.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define MESH_MAX_SIZE ((size_t)0x7fffffff)
#define MESH_STREAM_CAPACITY UINT32_C(63008)
#define MESH_LANE_CAPACITY UINT32_C(17065)
#define MESH_FRAME_CAPACITY (UINT32_C(136556) + MESH_STREAM_CAPACITY + UINT32_C(64))
#define MESH_DECODE_WORKSPACE UINT32_C(65536)
#define MESH_HUFFMAN_WORKSPACE UINT32_C(2576)
#define MESH_RAW_HEADER_SIZE UINT32_C(5)
#define MESH_COMPRESSED_HEADER_SIZE UINT32_C(9)
#define MESH_FOOTER_SIZE UINT32_C(10)
#define MESH_HUFFMAN_BATCH UINT32_C(40)
#define MESH_DISTANCE_BATCH UINT32_C(16)

enum mesh_block_tag {
    MESH_BLOCK_RAW = 0,
    MESH_BLOCK_COMPRESSED = 1,
    MESH_BLOCK_END = 255
};

enum mesh_stream_mode {
    MESH_STREAM_RAW = 0,
    MESH_STREAM_REPEAT = 1,
    MESH_STREAM_HUFFMAN = 2
};

struct mesh_bit_reader {
    const uint8_t *data;
    size_t source_size;
    size_t payload_size;
    uint64_t state;
    uint32_t bit_offset;
};

struct mesh_huffman_table {
    uint16_t entries[1024];
    uint32_t code_bits;
};

struct mesh_decoded_streams {
    uint8_t *literals;
    uint8_t *tokens;
    uint8_t *length_bytes;
    uint32_t *distances;
    size_t literal_count;
    size_t token_count;
    size_t length_count;
    size_t distance_count;
};

struct mesh_bit_writer {
    uint8_t data[MESH_LANE_CAPACITY + 8];
    size_t bit_count;
};

struct mesh_huffman_node {
    uint32_t weight;
    uint32_t symbol;
    int32_t parent;
};

struct mesh_match {
    uint32_t position;
    uint32_t length;
    int32_t distance;
};

struct mesh_byte_buffer {
    uint8_t data[MESH_STREAM_CAPACITY];
    size_t size;
};

struct mesh_encoder {
    struct mesh_byte_buffer literals;
    struct mesh_byte_buffer tokens;
    struct mesh_byte_buffer lengths;
    struct mesh_byte_buffer distances;
    uint32_t distance_suffixes[16384];
    uint32_t recent_distances[4];
    uint32_t saved_distances[4];
    uint32_t short_hash_positions[4096];
    const uint8_t *source;
    uint32_t source_size;
    uint32_t level;
    uint32_t hash_shift;
    uint32_t last_hash_position;
    uint32_t literal_position;
    uint32_t block_start;
    uint32_t block_budget;
    int32_t remaining_budget;
    struct mesh_bit_writer lanes[8];
    uint8_t block_data[MESH_FRAME_CAPACITY];
    uint32_t hash_positions[];
};

static const uint32_t mesh_max_distance[7] = {0, 0, 0, 4096, 65536, 1048576, UINT32_C(0x80000000)};

static uint16_t mesh_read_u16(const uint8_t *data)
{
    return (uint16_t)((uint32_t)data[0] | (uint32_t)data[1] << 8);
}

static uint32_t mesh_read_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | (uint32_t)data[1] << 8 | (uint32_t)data[2] << 16 | (uint32_t)data[3] << 24;
}

static uint64_t mesh_read_u64(const uint8_t *data)
{
    return (uint64_t)mesh_read_u32(data) | (uint64_t)mesh_read_u32(data + 4) << 32;
}

static void mesh_write_u16(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static void mesh_write_u32(uint8_t *data, uint32_t value)
{
    mesh_write_u16(data, value);
    mesh_write_u16(data + 2, value >> 16);
}

static int32_t mesh_signed32(uint32_t value)
{
    if (value <= INT32_MAX) {
        return (int32_t)value;
    }
    return -1 - (int32_t)(UINT32_MAX - value);
}

static uint32_t mesh_bits_highest(uint32_t value)
{
    uint32_t bit_index = 0;

    while (value >>= 1) {
        ++bit_index;
    }
    return bit_index;
}

static uint32_t mesh_bits_reverse(uint32_t value, uint32_t count)
{
    uint32_t result = 0;

    while (count--) {
        result = result * 2 + (value & 1);
        value >>= 1;
    }
    return result;
}

static int mesh_bits_refill(struct mesh_bit_reader *reader)
{
    uint64_t state = reader->state;
    uint32_t consumed = 0;
    size_t byte_offset;

    if (reader->bit_offset > reader->payload_size * 8) {
        return 0;
    }
    while (consumed < 64 && !(state & (UINT64_C(1) << 63))) {
        state <<= 1;
        ++consumed;
    }
    reader->bit_offset += consumed;
    byte_offset = reader->bit_offset >> 3;

    if (byte_offset > reader->source_size || reader->source_size - byte_offset < 8) {
        return 0;
    }
    reader->state = (mesh_read_u64(reader->data + byte_offset) >> (reader->bit_offset & 7)) | (UINT64_C(1) << 63);
    return 1;
}

static uint32_t mesh_bits_read(struct mesh_bit_reader *reader, uint32_t count)
{
    uint32_t value = (uint32_t)(reader->state & ((UINT64_C(1) << count) - 1));

    reader->state >>= count;
    return value;
}

static int mesh_bits_split(struct mesh_bit_reader lanes[8], const uint8_t *data, size_t size, size_t source_size)
{
    uint32_t index_bits = 0;
    uint32_t i;
    size_t index_size;
    size_t payload_size = 0;
    uint32_t lane_offset = 0;

    if (size) {
        index_bits = data[size - 1] >> 3;
        if (index_bits < 1 || index_bits > 23) {
            return 0;
        }
        index_size = (7 * index_bits + 12) / 8;
        if (index_size < 4) {
            index_size = 4;
        }
        if (size <= index_size) {
            return 0;
        }
        payload_size = size - index_size;
    }
    for (i = 0; i < 8; ++i) {
        uint32_t index_offset = i * index_bits;
        size_t byte_offset = payload_size + (index_offset >> 3);

        lanes[i].data = data;
        lanes[i].source_size = source_size;
        lanes[i].payload_size = payload_size;
        lanes[i].state = UINT64_MAX;
        lanes[i].bit_offset = lane_offset * 8;
        if (size && i < 7) {
            if (byte_offset > source_size || source_size - byte_offset < 4) {
                return 0;
            }
            lane_offset += (mesh_read_u32(data + byte_offset) >> (index_offset & 7)) & ((UINT32_C(1) << index_bits) - 1);
            if (lane_offset > payload_size) {
                return 0;
            }
        }
    }
    return 1;
}

static int mesh_huffman_build_table(struct mesh_huffman_table *tree, const uint8_t *lengths, uint32_t symbol_count)
{
    uint32_t counts[11] = {0};
    uint32_t next[11] = {0};
    uint32_t i;
    uint32_t code = 0;
    uint32_t occupied = 0;
    uint32_t code_bits = 0;

    for (i = 0; i < symbol_count; ++i) {
        uint32_t length = lengths[i];

        if (length > 10) {
            return 0;
        }
        if (length) {
            ++counts[length];
            occupied += UINT32_C(1) << (10 - length);
            if (length > code_bits) {
                code_bits = length;
            }
        }
    }
    if (!code_bits || occupied != 1024) {
        return 0;
    }
    for (i = 1; i <= code_bits; ++i) {
        code = (code + counts[i - 1]) << 1;
        next[i] = code;
    }
    tree->code_bits = code_bits;
    for (i = 0; i < symbol_count; ++i) {
        uint32_t length = lengths[i];
        uint32_t index;

        if (!length) {
            continue;
        }
        index = mesh_bits_reverse(next[length]++, length);
        for (; index < (UINT32_C(1) << code_bits); index += UINT32_C(1) << length) {
            tree->entries[index] = (uint16_t)((i << 8) | length);
        }
    }
    return 1;
}

static int mesh_huffman_decode(struct mesh_bit_reader lanes[8], const struct mesh_huffman_table *tree, uint8_t *dst, size_t count)
{
    size_t position = 0;
    size_t batch_size;
    size_t i;
    uint32_t lane;
    uint32_t mask = (UINT32_C(1) << tree->code_bits) - 1;

    do {

        for (lane = 0; lane < 8; ++lane) {
            if (!mesh_bits_refill(&lanes[lane])) {
                return 0;
            }
        }
        batch_size = count - position;
        if (batch_size > MESH_HUFFMAN_BATCH) {
            batch_size = MESH_HUFFMAN_BATCH;
        }
        for (i = 0; i < batch_size; ++i) {
            struct mesh_bit_reader *reader = &lanes[i & 7];
            uint16_t entry = tree->entries[reader->state & mask];

            reader->state >>= entry & 255;
            dst[position + i] = (uint8_t)(entry >> 8);
        }
        position += batch_size;
    } while (batch_size == MESH_HUFFMAN_BATCH);
    return 1;
}

static int mesh_huffman_decode_stream(struct mesh_bit_reader lanes[8], uint8_t *dst, size_t count)
{
    uint8_t meta_lengths[11];
    uint8_t packed_lengths[256];
    uint8_t lengths[256] = {0};
    struct mesh_huffman_table tree;
    uint32_t bitmap;
    uint32_t i;
    uint32_t used = 0;
    uint32_t cursor = 0;

    if (!mesh_bits_refill(&lanes[0])) {
        return 0;
    }
    for (i = 0; i < 11; ++i) {
        meta_lengths[i] = (uint8_t)mesh_bits_read(&lanes[0], 3);
    }
    if (!mesh_bits_refill(&lanes[0])) {
        return 0;
    }
    bitmap = mesh_bits_read(&lanes[0], 32);
    if (!bitmap || !mesh_huffman_build_table(&tree, meta_lengths, 11)) {
        return 0;
    }
    for (i = 0; i < 32; ++i) {
        if ((bitmap >> i) & 1) {
            used += 8;
        }
    }
    if (!mesh_huffman_decode(lanes, &tree, packed_lengths, used)) {
        return 0;
    }
    for (i = 0; i < 32; ++i) {
        if ((bitmap >> i) & 1) {
            memcpy(lengths + i * 8, packed_lengths + cursor, 8);
            cursor += 8;
        }
    }
    return mesh_huffman_build_table(&tree, lengths, 256) && mesh_huffman_decode(lanes, &tree, dst, count);
}

static int mesh_decode_stream(struct mesh_bit_reader lanes[8], const uint8_t **byte_cursor, const uint8_t *byte_end, uint32_t mode, uint8_t *dst, size_t count)
{
    if (mode == MESH_STREAM_HUFFMAN) {
        return lanes[0].payload_size && mesh_huffman_decode_stream(lanes, dst, count);
    }
    if (mode == MESH_STREAM_REPEAT) {
        if (*byte_cursor == byte_end) {
            return 0;
        }
        memset(dst, *(*byte_cursor)++, count);
        return 1;
    }
    if (mode != MESH_STREAM_RAW || count > (size_t)(byte_end - *byte_cursor)) {
        return 0;
    }
    memcpy(dst, *byte_cursor, count);
    *byte_cursor += count;
    return 1;
}

static int mesh_decode_length(const uint8_t **length_cursor, const uint8_t *length_end, uint32_t short_value, uint32_t escape, uint32_t *result)
{
    uint32_t extra;

    if (short_value != escape) {
        *result = short_value;
        return 1;
    }
    if (*length_cursor == length_end) {
        return 0;
    }
    extra = *(*length_cursor)++;
    if (extra == 255) {
        if ((size_t)(length_end - *length_cursor) < 4) {
            return 0;
        }
        extra = mesh_read_u32(*length_cursor);
        *length_cursor += 4;
    }

    *result = escape + extra;
    return 1;
}

static int mesh_decode_distances(struct mesh_bit_reader lanes[8], const uint8_t *distance_symbols, size_t distance_count, uint32_t *distances)
{
    size_t position = 0;
    size_t batch_size;
    size_t i;
    uint32_t lane;

    if (!distance_count) {
        return 1;
    }
    do {

        for (lane = 0; lane < 8; ++lane) {
            if (!mesh_bits_refill(&lanes[lane])) {
                return 0;
            }
        }
        batch_size = distance_count - position;
        if (batch_size > MESH_DISTANCE_BATCH) {
            batch_size = MESH_DISTANCE_BATCH;
        }
        for (i = 0; i < batch_size; ++i) {
            uint32_t symbol = distance_symbols[position + i];
            uint32_t bit_count = symbol >> 3;
            uint32_t suffix = mesh_bits_read(&lanes[i & 7], bit_count);

            distances[position + i] = (UINT32_C(8) << bit_count) + (symbol & 7) + UINT32_C(8) * suffix - 7;
        }
        position += batch_size;
    } while (batch_size == MESH_DISTANCE_BATCH);
    return 1;
}

static int mesh_prepare_streams(struct mesh_decoded_streams *streams, const uint8_t *block, size_t source_size, size_t decoded_size, size_t bitstream_offset, size_t footer_offset, uint8_t *workspace)
{
    const uint8_t *footer = block + footer_offset;
    const uint8_t *byte_cursor = block + MESH_COMPRESSED_HEADER_SIZE;
    const uint8_t *byte_end = block + bitstream_offset;
    uint32_t flags = mesh_read_u16(footer);
    size_t distance_symbols_offset;
    size_t length_offset;
    size_t literal_offset;
    size_t token_offset;
    size_t workspace_size;
    uint8_t *distance_symbols;
    struct mesh_bit_reader lanes[8];

    streams->literal_count = mesh_read_u16(footer + 6);
    streams->token_count = mesh_read_u16(footer + 2);
    streams->length_count = mesh_read_u16(footer + 4);
    streams->distance_count = mesh_read_u16(footer + 8);
    if (!streams->token_count || streams->token_count > decoded_size || streams->length_count > decoded_size || streams->literal_count > decoded_size || streams->distance_count > streams->token_count) {
        return 0;
    }

    distance_symbols_offset = (4 * streams->distance_count + 31) & ~(size_t)31;
    length_offset = distance_symbols_offset + ((streams->distance_count + 31) & ~(size_t)31);
    literal_offset = length_offset + ((streams->length_count + 31) & ~(size_t)31);
    token_offset = literal_offset + ((streams->literal_count + 31) & ~(size_t)31);
    workspace_size = token_offset + ((streams->token_count + 31) & ~(size_t)31);
    if (workspace_size + MESH_HUFFMAN_WORKSPACE > MESH_DECODE_WORKSPACE || !mesh_bits_split(lanes, block + bitstream_offset, footer_offset - bitstream_offset, source_size - bitstream_offset)) {
        return 0;
    }
    streams->distances = (uint32_t *)(void *)workspace;
    distance_symbols = workspace + distance_symbols_offset;
    streams->length_bytes = workspace + length_offset;
    streams->literals = workspace + literal_offset;
    streams->tokens = workspace + token_offset;
    if (!mesh_decode_stream(lanes, &byte_cursor, byte_end, flags & 7, streams->literals, streams->literal_count) || !mesh_decode_stream(lanes, &byte_cursor, byte_end, (flags >> 3) & 7, streams->tokens, streams->token_count) ||
        !mesh_decode_stream(lanes, &byte_cursor, byte_end, (flags >> 6) & 7, streams->length_bytes, streams->length_count) || !mesh_decode_stream(lanes, &byte_cursor, byte_end, (flags >> 9) & 7, distance_symbols, streams->distance_count)) {
        return 0;
    }
    if (!mesh_decode_distances(lanes, distance_symbols, streams->distance_count, streams->distances)) {
        return 0;
    }
    return 1;
}

static int mesh_replay_tokens(uint8_t *dst, size_t *written, size_t capacity, size_t decoded_size, const struct mesh_decoded_streams *streams, uint32_t recent_distances[4])
{
    const uint8_t *length_cursor = streams->length_bytes;
    const uint8_t *length_end = streams->length_bytes + streams->length_count;
    size_t output_offset = *written;
    size_t output_end = output_offset + decoded_size;
    int partial = output_end > capacity;
    size_t literal_offset = 0;
    size_t distance_offset = 0;
    size_t i;

    if (partial) {
        output_end = capacity;
    }
    if (!output_offset) {
        if (!streams->literal_count) {
            return 0;
        }
        dst[output_offset++] = streams->literals[literal_offset++];
    }
    for (i = 0; i < streams->token_count; ++i) {
        uint32_t token = streams->tokens[i];
        uint32_t new_distance = token & 32;
        uint32_t selected = new_distance ? 4 : (token >> 3) & 3;
        uint32_t literal_length;
        uint32_t match_length;
        uint32_t distance;
        uint32_t mask = new_distance ? 31 : 7;
        uint32_t j;
        size_t literal_copy;

        if (!mesh_decode_length(&length_cursor, length_end, token >> 6, 3, &literal_length) || !mesh_decode_length(&length_cursor, length_end, token & mask, mask, &match_length)) {
            return 0;
        }
        match_length += 2;
        if (new_distance) {
            if (distance_offset == streams->distance_count) {
                return 0;
            }
            distance = streams->distances[distance_offset++];
        } else {
            distance = recent_distances[selected];
        }
        for (j = selected < 4 ? selected : 3; j; --j) {
            recent_distances[j] = recent_distances[j - 1];
        }
        recent_distances[0] = distance;
        literal_copy = literal_length;
        if (literal_copy > output_end - output_offset) {
            literal_copy = output_end - output_offset;
        }
        if (literal_copy > streams->literal_count - literal_offset) {
            return 0;
        }
        memcpy(dst + output_offset, streams->literals + literal_offset, literal_copy);
        output_offset += literal_copy;
        literal_offset += literal_copy;

        if (output_offset < output_end) {
            size_t copy = match_length;

            if (!distance || distance > output_offset) {
                return 0;
            }
            if (copy > output_end - output_offset) {
                copy = output_end - output_offset;
            }
            while (copy--) {
                dst[output_offset] = dst[output_offset - distance];
                ++output_offset;
            }
        }
        if (output_offset == output_end) {
            if (!partial && i + 1 != streams->token_count) {
                return 0;
            }
            break;
        }
    }
    if (output_offset != output_end || (!partial && (literal_offset != streams->literal_count || distance_offset != streams->distance_count || length_cursor != length_end))) {
        return 0;
    }
    *written = output_offset;
    return 1;
}

static size_t mesh_decode_blocks(void *dst_buffer, size_t capacity, const void *src_buffer, size_t size, uint8_t *workspace)
{
    const uint8_t *src = (const uint8_t *)src_buffer;
    uint8_t *dst = (uint8_t *)dst_buffer;
    uint32_t recent_distances[4] = {1, 1, 1, 1};
    size_t position = 0;
    size_t written = 0;

    while (written < capacity && position < size) {
        const uint8_t *block = src + position;
        uint32_t tag = block[0];
        size_t decoded_size;
        size_t bitstream_offset;
        size_t footer_offset;
        struct mesh_decoded_streams streams;

        if (tag == MESH_BLOCK_END) {
            return written;
        }
        if (size - position < MESH_RAW_HEADER_SIZE || tag > MESH_BLOCK_COMPRESSED) {
            return 0;
        }
        decoded_size = mesh_read_u32(block + 1);
        if (!decoded_size || decoded_size > MESH_MAX_SIZE) {
            return 0;
        }
        if (tag == MESH_BLOCK_RAW) {
            if (decoded_size > size - position - MESH_RAW_HEADER_SIZE) {
                decoded_size = size - position - MESH_RAW_HEADER_SIZE;
            }
            if (decoded_size > capacity - written) {
                decoded_size = capacity - written;
            }
            memcpy(dst + written, block + MESH_RAW_HEADER_SIZE, decoded_size);
            written += decoded_size;
            position += MESH_RAW_HEADER_SIZE + decoded_size;
            continue;
        }
        if (size - position < MESH_COMPRESSED_HEADER_SIZE) {
            return 0;
        }
        bitstream_offset = mesh_read_u16(block + 5);
        footer_offset = mesh_read_u16(block + 7);
        if (bitstream_offset < MESH_COMPRESSED_HEADER_SIZE || bitstream_offset > footer_offset || footer_offset >= decoded_size || footer_offset + MESH_FOOTER_SIZE >= size - position) {
            return 0;
        }
        if (!mesh_prepare_streams(&streams, block, size - position, decoded_size, bitstream_offset, footer_offset, workspace)) {
            return 0;
        }
        if (!mesh_replay_tokens(dst, &written, capacity, decoded_size, &streams, recent_distances)) {
            return 0;
        }
        position += footer_offset + MESH_FOOTER_SIZE;
    }
    return written == capacity ? written : 0;
}

static int mesh_bits_write(struct mesh_bit_writer *lane, uint32_t value, uint32_t count)
{
    size_t byte_offset = lane->bit_count >> 3;
    uint32_t shift = (uint32_t)(lane->bit_count & 7);
    uint64_t bits;
    uint32_t i;

    if (count > 32 || lane->bit_count + count > 8u * MESH_LANE_CAPACITY) {
        return 0;
    }

    bits = (uint64_t)value << shift;
    bits |= lane->data[byte_offset] & ((UINT32_C(1) << shift) - 1);
    for (i = 0; i < 8; ++i) {
        lane->data[byte_offset + i] = (uint8_t)(bits >> (8 * i));
    }
    lane->bit_count += count;
    return 1;
}

static int mesh_bits_pack(const struct mesh_bit_writer lanes[8], uint8_t *block_data, size_t capacity, size_t *output_offset)
{
    size_t offset = *output_offset;
    size_t index_offset;
    size_t lane_bytes[8];
    uint32_t largest_lane = 0;
    uint32_t i;

    for (i = 0; i < 8; ++i) {
        lane_bytes[i] = (lanes[i].bit_count + 7) >> 3;
        if (largest_lane < lane_bytes[i]) {
            largest_lane = (uint32_t)lane_bytes[i];
        }
        if (offset + lane_bytes[i] >= capacity - 64) {
            return 0;
        }
        memcpy(block_data + offset, lanes[i].data, lane_bytes[i]);
        offset += lane_bytes[i];
    }
    if (largest_lane) {
        uint32_t code_bits = mesh_bits_highest(largest_lane) + 1;
        size_t index_size = (7 * code_bits + 12) >> 3;
        size_t bit_offset = 0;

        if (index_size < 4) {
            index_size = 4;
        }
        index_offset = offset;
        memset(block_data + offset, 0, index_size);
        for (i = 0; i < 7; ++i) {
            uint32_t j;

            for (j = 0; j < code_bits; ++j, ++bit_offset) {
                block_data[index_offset + (bit_offset >> 3)] |= (uint8_t)(((lane_bytes[i] >> j) & 1) << (bit_offset & 7));
            }
        }
        offset += index_size;
        block_data[offset - 1] |= (uint8_t)(code_bits << 3);
    }
    *output_offset = offset;
    return 1;
}

static int mesh_huffman_compare_nodes(const void *left, const void *right)
{
    const struct mesh_huffman_node *left_node = left;
    const struct mesh_huffman_node *right_node = right;

    if (left_node->weight != right_node->weight) {
        return left_node->weight < right_node->weight ? -1 : 1;
    }
    return left_node->symbol < right_node->symbol ? -1 : left_node->symbol > right_node->symbol;
}

static void mesh_huffman_build_lengths(const uint32_t *counts, uint8_t *lengths, uint32_t symbol_count, uint32_t maximum_bits, uint32_t total_count)
{
    struct mesh_huffman_node nodes[511];
    uint32_t minimum_weight;
    uint32_t i;
    uint32_t leaves = 0;
    uint32_t maximum_length;

    memset(lengths, 0, symbol_count);
    for (i = 0; i < symbol_count; ++i) {
        if (counts[i]) {
            nodes[leaves].weight = counts[i];
            nodes[leaves].symbol = i;
            ++leaves;
        }
    }

    qsort(nodes, leaves, sizeof(nodes[0]), mesh_huffman_compare_nodes);
    if (maximum_bits > mesh_bits_highest(leaves) + 3) {
        maximum_bits = mesh_bits_highest(leaves) + 3;
    }
    minimum_weight = (total_count >> maximum_bits) + 1;
    do {
        uint32_t next_leaf = 0;
        uint32_t next_branch = leaves;
        uint32_t node_count = leaves;

        for (i = 0; i < leaves; ++i) {
            if (nodes[i].weight < minimum_weight) {
                nodes[i].weight = minimum_weight;
            }
            nodes[i].parent = -1;
        }
        while (node_count < 2 * leaves - 1) {
            uint32_t children[2];
            uint32_t j;

            for (j = 0; j < 2; ++j) {
                if (next_leaf < leaves && (next_branch == node_count || nodes[next_leaf].weight <= nodes[next_branch].weight)) {
                    children[j] = next_leaf++;
                } else {
                    children[j] = next_branch++;
                }
            }
            nodes[node_count].weight = nodes[children[0]].weight + nodes[children[1]].weight;
            nodes[node_count].symbol = node_count;
            nodes[node_count].parent = -1;
            nodes[children[0]].parent = (int32_t)node_count;
            nodes[children[1]].parent = (int32_t)node_count++;
        }
        maximum_length = 0;
        for (i = 0; i < leaves; ++i) {
            int32_t parent = nodes[i].parent;
            uint32_t length = 0;

            while (parent >= 0) {
                ++length;
                parent = nodes[parent].parent;
            }
            lengths[nodes[i].symbol] = (uint8_t)length;
            if (length > maximum_length) {
                maximum_length = length;
            }
        }
        minimum_weight *= 2;
    } while (maximum_length > maximum_bits);
}

static void mesh_huffman_build_codes(const uint8_t *lengths, uint32_t symbol_count, uint16_t *codes)
{
    uint32_t counts[11] = {0};
    uint32_t next[11] = {0};
    uint32_t i;
    uint32_t code = 0;

    for (i = 0; i < symbol_count; ++i) {
        if (lengths[i]) {
            ++counts[lengths[i]];
        }
    }
    for (i = 1; i <= 10; ++i) {
        code = (code + counts[i - 1]) << 1;
        next[i] = code;
    }
    for (i = 0; i < symbol_count; ++i) {
        codes[i] = lengths[i] ? (uint16_t)mesh_bits_reverse(next[lengths[i]]++, lengths[i]) : 0;
    }
}

static int32_t mesh_encode_stream(struct mesh_encoder *encoder, const struct mesh_byte_buffer *stream, size_t *output_offset)
{
    uint32_t frequencies[256] = {0};
    uint32_t group_bitmap = 0;
    uint8_t lengths[256];
    uint8_t secondary_lengths[11] = {0};
    uint32_t secondary_frequencies[11] = {0};
    uint16_t codes[256];
    uint16_t secondary_codes[11];
    size_t saved_bits[8];
    size_t i;
    size_t encoded_bits = 0;
    uint32_t distinct = 0;
    uint32_t symbol;
    uint32_t lane_index = 0;
    uint32_t packed_count = 0;

    if (!stream->size) {
        return MESH_STREAM_RAW;
    }
    for (i = 0; i < stream->size; ++i) {
        ++frequencies[stream->data[i]];
    }
    for (i = 0; i < 256; ++i) {
        distinct += frequencies[i] != 0;
    }
    if (distinct == 1) {
        encoder->block_data[(*output_offset)++] = stream->data[0];
        return MESH_STREAM_REPEAT;
    }
    if (8 * stream->size <= stream->size + 73) {
        memcpy(encoder->block_data + *output_offset, stream->data, stream->size);
        *output_offset += stream->size;
        return MESH_STREAM_RAW;
    }
    mesh_huffman_build_lengths(frequencies, lengths, 256, 10, (uint32_t)stream->size);
    for (i = 0; i < 256; ++i) {
        if (lengths[i]) {
            group_bitmap |= (uint32_t)1 << (i >> 3);
        }
    }
    for (i = 0; i < 256; ++i) {
        if ((group_bitmap >> (i >> 3)) & 1) {
            ++secondary_frequencies[lengths[i]];
            ++packed_count;
        }
    }
    distinct = 0;
    symbol = 0;
    for (i = 0; i < 11; ++i) {
        if (secondary_frequencies[i]) {
            ++distinct;
            symbol = (uint32_t)i;
        }
    }
    if (distinct == 1) {
        secondary_lengths[symbol] = 1;
        secondary_lengths[symbol ? 0 : 1] = 1;
    } else {
        mesh_huffman_build_lengths(secondary_frequencies, secondary_lengths, 11, 5, packed_count);
    }
    for (i = 0; i < 8; ++i) {
        saved_bits[i] = encoder->lanes[i].bit_count;
    }
    mesh_huffman_build_codes(lengths, 256, codes);
    mesh_huffman_build_codes(secondary_lengths, 11, secondary_codes);
    for (i = 0; i < 11; ++i) {
        if (!mesh_bits_write(&encoder->lanes[0], secondary_lengths[i], 3)) {
            return -1;
        }
    }
    if (!mesh_bits_write(&encoder->lanes[0], group_bitmap, 32)) {
        return -1;
    }
    for (symbol = 0; symbol < 256; ++symbol) {
        if ((group_bitmap >> (symbol >> 3)) & 1) {
            uint32_t length = lengths[symbol];

            if (!mesh_bits_write(&encoder->lanes[lane_index++ & 7], secondary_codes[length] | (uint32_t)secondary_lengths[length] << 12, secondary_lengths[length])) {
                return -1;
            }
        }
    }
    for (i = 0; i < stream->size; ++i) {
        symbol = stream->data[i];
        if (!mesh_bits_write(&encoder->lanes[i & 7], codes[symbol] | (uint32_t)lengths[symbol] << 12, lengths[symbol])) {
            return -1;
        }
    }
    for (i = 0; i < 8; ++i) {
        encoded_bits += encoder->lanes[i].bit_count - saved_bits[i];
    }
    if (encoded_bits >= 8 * stream->size) {
        for (i = 0; i < 8; ++i) {
            encoder->lanes[i].bit_count = saved_bits[i];
        }
        memcpy(encoder->block_data + *output_offset, stream->data, stream->size);
        *output_offset += stream->size;
        return MESH_STREAM_RAW;
    }
    return MESH_STREAM_HUFFMAN;
}

static uint32_t mesh_match_length(const struct mesh_encoder *encoder, uint32_t position, uint32_t candidate, uint32_t length)
{
    while (position + length < encoder->source_size && encoder->source[position + length] == encoder->source[candidate + length]) {
        ++length;
    }
    return length;
}

static uint32_t mesh_match_hash(uint64_t word, uint32_t shift)
{
    return (uint32_t)((word * UINT64_C(0x995d97cb4c1db100)) >> shift);
}

static uint32_t mesh_match_fast_hash(uint64_t word, uint32_t shift)
{
    return (uint32_t)((word * UINT64_C(0x5d97cb4c1db10000)) >> shift);
}

static uint32_t mesh_match_middle_hash(uint64_t word, uint32_t shift)
{
    return (uint32_t)((word * UINT64_C(0x97cb4c1db1000000)) >> shift);
}

static uint32_t mesh_match_short_hash(uint32_t word)
{
    uint32_t product = (uint32_t)((uint64_t)word * UINT64_C(930722048));

    return product >> 20;
}

static void mesh_match_insert(struct mesh_encoder *encoder, uint32_t position)
{
    uint64_t word = mesh_read_u64(encoder->source + position);

    encoder->short_hash_positions[mesh_match_short_hash((uint32_t)word)] = position;
    encoder->hash_positions[mesh_match_middle_hash(word, encoder->hash_shift)] = position;
    encoder->hash_positions[mesh_match_hash(word, encoder->hash_shift)] = position;
}

static struct mesh_match mesh_match_find_fast(struct mesh_encoder *encoder, uint32_t position)
{
    struct mesh_match match = {position, 0, 0};
    const uint8_t *source = encoder->source;
    uint32_t candidate = position - encoder->recent_distances[0];
    uint32_t hash;
    uint32_t preceding;
    uint32_t extension;
    uint64_t word;

    if (mesh_read_u16(source + position) == mesh_read_u16(source + candidate)) {
        match.length = mesh_match_length(encoder, position, candidate, 2);
        return match;
    }
    word = mesh_read_u64(source + position);
    hash = mesh_match_fast_hash(word, encoder->hash_shift);
    candidate = encoder->hash_positions[hash];
    ++encoder->last_hash_position;
    while (encoder->last_hash_position < position) {
        uint64_t previous = mesh_read_u64(source + encoder->last_hash_position);

        encoder->hash_positions[mesh_match_fast_hash(previous, encoder->hash_shift)] = encoder->last_hash_position;
        ++encoder->last_hash_position;
    }
    encoder->hash_positions[hash] = position;
    if ((word ^ mesh_read_u64(source + candidate)) & UINT64_C(0xffffffffffff)) {
        return match;
    }
    match.length = mesh_match_length(encoder, position, candidate, 6);
    match.distance = (int32_t)(position - candidate);
    preceding = position - encoder->literal_position;
    if (candidate >= 8 && preceding && source[position - 1] == source[candidate - 1]) {
        extension = 1;
        while (extension < 7 && extension < preceding && source[position - extension - 1] == source[candidate - extension - 1]) {
            ++extension;
        }
        match.position -= extension;
        match.length += extension;
    }
    return match;
}

static struct mesh_match mesh_match_find_hashed(struct mesh_encoder *encoder, uint32_t position)
{
    struct mesh_match match = {position, 0, 0};
    uint64_t word = mesh_read_u64(encoder->source + position);
    uint32_t hash = mesh_match_hash(word, encoder->hash_shift);
    uint32_t middle_hash = mesh_match_middle_hash(word, encoder->hash_shift);
    uint32_t short_hash = mesh_match_short_hash((uint32_t)word);
    uint32_t candidates[3];
    uint32_t i;

    candidates[0] = encoder->hash_positions[hash];
    candidates[1] = encoder->hash_positions[middle_hash];
    candidates[2] = encoder->short_hash_positions[short_hash];
    ++encoder->last_hash_position;
    while (encoder->last_hash_position < position) {
        mesh_match_insert(encoder, encoder->last_hash_position++);
    }
    mesh_match_insert(encoder, position);
    for (i = 0; i < 3; ++i) {
        uint32_t required = 7 - 2 * i;
        uint32_t candidate = candidates[i];
        uint64_t comparison = word ^ mesh_read_u64(encoder->source + candidate);

        if (!(comparison & ((UINT64_C(1) << (8 * required)) - 1))) {
            match.length = mesh_match_length(encoder, position, candidate, required);
            match.distance = (int32_t)(position - candidate);
            if (i && (uint32_t)match.distance >= mesh_max_distance[match.length < 6 ? match.length : 6]) {
                match.length = 0;
            }
            return match;
        }
    }
    return match;
}

static struct mesh_match mesh_match_find(struct mesh_encoder *encoder, uint32_t position)
{
    struct mesh_match match = {position, 0, 0};
    const uint8_t *source = encoder->source;
    uint32_t candidate;
    uint32_t i;

    for (i = 0; i < 3; ++i) {
        uint32_t required = i ? 4 : 2;

        candidate = position - encoder->recent_distances[i];
        if (!memcmp(source + position, source + candidate, required)) {
            match.length = mesh_match_length(encoder, position, candidate, required);
            match.distance = -(int32_t)i;
            return match;
        }
    }
    return mesh_match_find_hashed(encoder, position);
}

static struct mesh_match mesh_match_choose(struct mesh_encoder *encoder, struct mesh_match match)
{
    struct mesh_match next;
    const uint8_t *source = encoder->source;
    uint32_t candidate;
    uint32_t preceding;
    uint32_t extension;
    uint32_t cost;
    uint32_t next_cost;

    if (!match.length || match.distance <= 0) {
        return match;
    }
    candidate = match.position - (uint32_t)match.distance;
    preceding = match.position - encoder->literal_position;
    if (candidate >= 8 && preceding && source[match.position - 1] == source[candidate - 1]) {
        extension = 1;
        while (extension < 7 && extension < preceding && source[match.position - extension - 1] == source[candidate - extension - 1]) {
            ++extension;
        }
        match.position -= extension;
        match.length += extension;
        return match;
    }
    if (match.length > 39) {
        return match;
    }
    cost = mesh_bits_highest((uint32_t)match.distance + 7) + 5;
    next.position = match.position + 1;
    candidate = next.position - encoder->recent_distances[0];
    if (!memcmp(source + next.position, source + candidate, 2)) {
        next.length = mesh_match_length(encoder, next.position, candidate, 2);

        if (mesh_signed32(4u * (match.length - next.length) + 4) < (int32_t)cost) {
            next.distance = 0;
            return next;
        }
    }
    if (encoder->level == 9) {
        next = mesh_match_find_hashed(encoder, next.position);
        if (next.length) {
            next_cost = next.distance ? mesh_bits_highest((uint32_t)next.distance + 7) + 5 : 0;
            if (mesh_signed32(next_cost + 4u * (match.length - next.length) + 4) < (int32_t)cost) {
                return next;
            }
        }
    } else {
        candidate = encoder->hash_positions[mesh_match_hash(mesh_read_u64(source + next.position), encoder->hash_shift)];
        if (!memcmp(source + next.position, source + candidate, 7)) {
            next.length = mesh_match_length(encoder, next.position, candidate, 7);
            next.distance = (int32_t)(next.position - candidate);
            next_cost = mesh_bits_highest((uint32_t)next.distance + 7) + 5;
            if (mesh_signed32(next_cost + 4u * (match.length - next.length) + 4) < (int32_t)cost) {
                return next;
            }
        }
    }
    next.position = match.position + 2;
    candidate = next.position - encoder->recent_distances[0];
    if (!memcmp(source + next.position, source + candidate, 3)) {
        next.length = mesh_match_length(encoder, next.position, candidate, 3);
        if (mesh_signed32(4u * (match.length - next.length) + 8) < (int32_t)cost) {
            next.distance = 0;
            return next;
        }
    }
    return match;
}

static void mesh_encode_length(struct mesh_byte_buffer *stream, uint32_t value)
{
    stream->data[stream->size++] = (uint8_t)(value < 255 ? value : 255);
    if (value >= 255) {
        mesh_write_u32(stream->data + stream->size, value);
        stream->size += 4;
    }
}

static void mesh_encode_token(struct mesh_encoder *encoder, uint32_t position, uint32_t match_length, int32_t distance)
{
    uint32_t literal_count = position - encoder->literal_position;
    uint32_t length_code = match_length - 2;
    uint32_t resolved_distance;
    uint32_t selected = distance <= 0 ? (uint32_t)-distance : 4;
    uint32_t limit = selected < 4 ? 7 : 31;
    uint32_t token = (literal_count < 3 ? literal_count : 3) << 6;
    size_t previous_length_size = encoder->lengths.size;
    uint32_t i;

    memcpy(encoder->literals.data + encoder->literals.size, encoder->source + encoder->literal_position, literal_count);
    encoder->literals.size += literal_count;
    if (literal_count >= 3) {
        mesh_encode_length(&encoder->lengths, literal_count - 3);
    }
    token |= 8 * selected | (length_code < limit ? length_code : limit);
    if (length_code >= limit) {
        mesh_encode_length(&encoder->lengths, length_code - limit);
    }
    if (selected == 4) {
        uint32_t suffix_bits = mesh_bits_highest((uint32_t)distance + 7) - 3;

        encoder->distance_suffixes[encoder->distances.size] = ((uint32_t)distance + 7) >> 3;
        encoder->distances.data[encoder->distances.size++] = (uint8_t)((suffix_bits << 3) | (((uint32_t)distance + 7) & 7));
        encoder->remaining_budget -= 5;
        resolved_distance = (uint32_t)distance;
        selected = 3;
    } else {
        resolved_distance = encoder->recent_distances[selected];
    }
    for (i = selected; i; --i) {
        encoder->recent_distances[i] = encoder->recent_distances[i - 1];
    }
    encoder->recent_distances[0] = resolved_distance;
    encoder->tokens.data[encoder->tokens.size++] = (uint8_t)token;
    encoder->remaining_budget -= (int32_t)(literal_count + encoder->lengths.size - previous_length_size + 1);
    encoder->literal_position = position + match_length;
}

static uint32_t mesh_parse_block_literals(struct mesh_encoder *encoder, uint32_t *position)
{
    if (*position + 9 > encoder->source_size) {
        return encoder->source_size;
    }
    *position = encoder->literal_position + (uint32_t)encoder->remaining_budget;
    if (*position + 9 > encoder->source_size) {
        return encoder->source_size;
    }
    return *position;
}

static uint32_t mesh_parse_block(struct mesh_encoder *encoder, uint32_t *position)
{
    uint32_t stop_position = encoder->block_start + encoder->block_budget;

    if (!encoder->level) {
        return mesh_parse_block_literals(encoder, position);
    }
    while (*position + 9 <= encoder->source_size) {
        struct mesh_match match;

        if (encoder->level == 1) {
            match = mesh_match_find_fast(encoder, *position);
        } else {
            match = mesh_match_choose(encoder, mesh_match_find(encoder, *position));
        }

        if (match.length) {
            mesh_encode_token(encoder, match.position, match.length, match.distance);
            *position = match.position + match.length;
            stop_position = encoder->literal_position;
            if (encoder->tokens.size < 0x4000 && encoder->lengths.size < 0x4000) {
                stop_position += (uint32_t)encoder->remaining_budget;
            }
        } else {
            encoder->last_hash_position = *position + ((*position - encoder->literal_position) >> 8);
            *position = encoder->last_hash_position + 1;
        }
        if (*position >= stop_position) {
            break;
        }
    }
    if (*position + 9 > encoder->source_size) {
        return encoder->source_size;
    }
    if (encoder->remaining_budget > 39 && encoder->tokens.size < 0x4000 && encoder->lengths.size < 0x4000) {
        return encoder->literal_position + (uint32_t)encoder->remaining_budget;
    }
    return encoder->literal_position;
}

static size_t mesh_encode_block(struct mesh_encoder *encoder, uint32_t block_end)
{
    size_t position;
    size_t output_offset = MESH_COMPRESSED_HEADER_SIZE;
    uint32_t i;
    uint32_t flags = 0;
    const struct mesh_byte_buffer *streams[4];

    if (block_end > encoder->literal_position) {
        mesh_encode_token(encoder, block_end, 2, 0);
    }
    if ((encoder->lengths.size + encoder->literals.size + encoder->tokens.size + 5 * encoder->distances.size) >> 4 > 0xf40) {
        return 0;
    }
    memset(encoder->lanes, 0, sizeof(encoder->lanes));
    streams[0] = &encoder->literals;
    streams[1] = &encoder->tokens;
    streams[2] = &encoder->lengths;
    streams[3] = &encoder->distances;
    for (i = 0; i < 4; ++i) {
        int32_t mode = mesh_encode_stream(encoder, streams[i], &output_offset);

        if (mode < 0) {
            return 0;
        }
        flags |= (uint32_t)mode << (3 * i);
    }
    mesh_write_u16(encoder->block_data + 5, (uint32_t)output_offset);
    for (position = 0; position < encoder->distances.size; ++position) {
        if (!mesh_bits_write(&encoder->lanes[position & 7], encoder->distance_suffixes[position], encoder->distances.data[position] >> 3)) {
            return 0;
        }
    }
    if (!mesh_bits_pack(encoder->lanes, encoder->block_data, MESH_FRAME_CAPACITY, &output_offset)) {
        return 0;
    }
    if (output_offset >= block_end - encoder->block_start || output_offset > UINT16_MAX) {
        return 0;
    }
    encoder->block_data[0] = MESH_BLOCK_COMPRESSED;
    mesh_write_u32(encoder->block_data + 1, block_end - encoder->block_start);
    mesh_write_u16(encoder->block_data + 7, (uint32_t)output_offset);
    mesh_write_u16(encoder->block_data + output_offset, flags);
    mesh_write_u16(encoder->block_data + output_offset + 2, (uint32_t)encoder->tokens.size);
    mesh_write_u16(encoder->block_data + output_offset + 4, (uint32_t)encoder->lengths.size);
    mesh_write_u16(encoder->block_data + output_offset + 6, (uint32_t)encoder->literals.size);
    mesh_write_u16(encoder->block_data + output_offset + 8, (uint32_t)encoder->distances.size);
    return output_offset + MESH_FOOTER_SIZE;
}

static size_t mesh_encode_raw(uint8_t *destination, size_t capacity, const uint8_t *source, size_t size)
{
    if (size + MESH_RAW_HEADER_SIZE >= capacity) {
        return 0;
    }
    destination[0] = MESH_BLOCK_RAW;
    mesh_write_u32(destination + 1, (uint32_t)size);
    memcpy(destination + MESH_RAW_HEADER_SIZE, source, size);
    destination[size + MESH_RAW_HEADER_SIZE] = MESH_BLOCK_END;
    return size + MESH_RAW_HEADER_SIZE + 1;
}

static size_t mesh_encode_blocks(struct mesh_encoder *encoder, uint8_t *dst, size_t capacity, const uint8_t *src, size_t size, uint32_t level, uint32_t hash_bits)
{
    uint32_t position = 1;
    size_t output_offset = 0;
    size_t hash_count = level ? (size_t)1 << hash_bits : 0;
    uint32_t i;

    memset(encoder, 0, sizeof(*encoder) + hash_count * sizeof(*encoder->hash_positions));
    encoder->source = src;
    encoder->source_size = (uint32_t)size;
    encoder->level = level;
    encoder->hash_shift = 64 - hash_bits;
    encoder->literal_position = 1;
    encoder->literals.data[0] = src[0];
    encoder->literals.size = 1;
    encoder->block_budget = 0x4000;
    encoder->remaining_budget = 0x4000;
    for (i = 0; i < 4; ++i) {
        encoder->recent_distances[i] = 1;
        encoder->saved_distances[i] = 1;
    }
    while (encoder->block_start < size) {
        uint32_t block_end = mesh_parse_block(encoder, &position);
        size_t block_size = block_end - encoder->block_start;
        size_t encoded_size;

        if (capacity - output_offset < MESH_COMPRESSED_HEADER_SIZE) {
            return mesh_encode_raw(dst, capacity, src, size);
        }
        encoded_size = mesh_encode_block(encoder, block_end);
        if (encoded_size > capacity - output_offset) {
            encoded_size = 0;
        }
        if (!encoded_size) {
            memcpy(encoder->recent_distances, encoder->saved_distances, sizeof(encoder->recent_distances));
            if (block_size + MESH_RAW_HEADER_SIZE > capacity - output_offset) {
                return mesh_encode_raw(dst, capacity, src, size);
            }
            dst[output_offset] = MESH_BLOCK_RAW;
            mesh_write_u32(dst + output_offset + 1, (uint32_t)block_size);
            memcpy(dst + output_offset + MESH_RAW_HEADER_SIZE, src + encoder->block_start, block_size);
            output_offset += block_size + MESH_RAW_HEADER_SIZE;
        } else {
            memcpy(dst + output_offset, encoder->block_data, encoded_size);
            output_offset += encoded_size;
            memcpy(encoder->saved_distances, encoder->recent_distances, sizeof(encoder->saved_distances));
        }
        encoder->literals.size = 0;
        encoder->tokens.size = 0;
        encoder->lengths.size = 0;
        encoder->distances.size = 0;
        encoder->block_budget *= 2;
        if (encoder->block_budget > 0xf3f0) {
            encoder->block_budget = 0xf3f0;
        }
        encoder->remaining_budget = (int32_t)encoder->block_budget;
        encoder->block_start = block_end;
        encoder->literal_position = block_end;
    }
    if (output_offset > size) {
        return mesh_encode_raw(dst, capacity, src, size);
    }
    if (output_offset >= capacity) {
        return 0;
    }
    dst[output_offset++] = MESH_BLOCK_END;
    return output_offset;
}

size_t lzmesh_encode_bound(size_t size)
{
    if (size > MESH_MAX_SIZE - MESH_RAW_HEADER_SIZE - 1) {
        return 0;
    }
    return size ? size + MESH_RAW_HEADER_SIZE + 1 : 1;
}

size_t lzmesh_encode_buffer(void *destination, size_t capacity, const void *source, size_t size, uint32_t algorithm)
{
    struct mesh_encoder *encoder;
    size_t result;
    size_t hash_count = 0;
    uint32_t hash_bits = 0;
    uint32_t hash_limit;
    uint32_t level;

    switch (algorithm) {
    case LZMESH_0:
    case LZMESH_1:
    case LZMESH_5:
    case LZMESH_9:
        level = algorithm & 255;
        break;
    default:
        return 0;
    }
    if (!destination || !capacity || (size && !source) || size > MESH_MAX_SIZE) {
        return 0;
    }
    if (!size) {
        ((uint8_t *)destination)[0] = MESH_BLOCK_END;
        return 1;
    }
    if (capacity > MESH_MAX_SIZE) {
        capacity = MESH_MAX_SIZE;
    }
    if (level) {
        hash_limit = level == 9 ? 21 : 18;
        hash_bits = mesh_bits_highest((uint32_t)size + 1) + 5;
        if (hash_bits > hash_limit) {
            hash_bits = hash_limit;
        }
        hash_count = (size_t)1 << hash_bits;
    }
    encoder = malloc(sizeof(*encoder) + hash_count * sizeof(*encoder->hash_positions));
    if (!encoder) {
        return 0;
    }
    result = mesh_encode_blocks(encoder, (uint8_t *)destination, capacity, (const uint8_t *)source, size, level, hash_bits);
    free(encoder);
    return result;
}

size_t lzmesh_decode_buffer(void *destination, size_t capacity, const void *source, size_t size)
{
    uint8_t *workspace;
    size_t result;

    if (!source || (!destination && capacity) || size > MESH_MAX_SIZE || capacity > MESH_MAX_SIZE) {
        return 0;
    }
    workspace = malloc(MESH_DECODE_WORKSPACE);
    if (!workspace) {
        return 0;
    }
    result = mesh_decode_blocks(destination, capacity, source, size, workspace);
    free(workspace);
    return result;
}
