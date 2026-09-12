#include "lzbitmap.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define BITMAP_FAST_GROUP_SIZE UINT32_C(128)
#define BITMAP_FAST_GROUP_LOOKAHEAD UINT32_C(144)
#define BITMAP_FAST_GROUP_CAPACITY UINT32_C(164)
#define BITMAP_FAST_HASH_MULTIPLIER UINT32_C(0x9e3779b1)

struct bitmap_regular_state {
    uint8_t *hash;
    uint8_t *distances;
    uint16_t *symbols;
    uint32_t *histogram;
    uint16_t *codes;
    const uint8_t *source;
    uint32_t source_size;
    uint32_t level;
    uint32_t hash_bits;
    uint32_t block_size;
    uint32_t dictionary_size;
};

struct bitmap_regular_group {
    uint16_t distance;
    uint16_t mask;
    uint8_t width;
    uint8_t score;
};

struct bitmap_fast_match {
    uint16_t distance;
    uint8_t mask;
};

struct bitmap_fast_group {
    uint32_t flags;
    uint32_t mask_count;
    uint32_t distance_count;
    uint32_t literal_count;
    uint8_t masks[16];
    uint16_t distances[16];
    uint8_t literals[128];
};

static const uint8_t bitmap_regular_hash_update_order[8] = {1, 2, 3, 5, 6, 7, 4, 0};
static const uint8_t bitmap_regular_candidate_offsets[8] = {0, 4, 2, 3, 1, 5, 6, 7};

static uint16_t bitmap_read16(const uint8_t *data)
{
    return (uint16_t)((uint32_t)data[0] | (uint32_t)data[1] << 8);
}

static uint32_t bitmap_read24(const uint8_t *data)
{
    return (uint32_t)data[0] | (uint32_t)data[1] << 8 | (uint32_t)data[2] << 16;
}

static uint32_t bitmap_read32(const uint8_t *data)
{
    return bitmap_read24(data) | (uint32_t)data[3] << 24;
}

static uint64_t bitmap_read64(const uint8_t *data)
{
    return (uint64_t)bitmap_read32(data) | (uint64_t)bitmap_read32(data + 4) << 32;
}

static void bitmap_write16(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static void bitmap_write24(uint8_t *data, uint32_t value)
{
    bitmap_write16(data, value);
    data[2] = (uint8_t)(value >> 16);
}

static void bitmap_write32(uint8_t *data, uint32_t value)
{
    bitmap_write16(data, value);
    bitmap_write16(data + 2, value >> 16);
}

static uint32_t bitmap_popcount(uint32_t value)
{
    uint32_t count = 0;

    while (value) {
        value &= value - 1;
        ++count;
    }
    return count;
}

static uint32_t bitmap_regular_hash(const struct bitmap_regular_state *state, uint32_t position)
{
    uint32_t product;

    product = (uint32_t)((uint64_t)bitmap_read32(state->source + position) * UINT64_C(0x9e3779b1));
    return product >> (32 - state->hash_bits);
}

static uint16_t bitmap_regular_mask(const uint8_t *source, uint32_t position, uint16_t distance, uint32_t length)
{
    uint32_t mask = 0;
    uint32_t i;

    for (i = 0; i < length; ++i) {
        if (source[position + i] != source[position - distance + i]) {
            mask |= UINT32_C(1) << i;
        }
    }
    return (uint16_t)mask;
}

static void bitmap_regular_update_hash(struct bitmap_regular_state *state, const uint32_t hashes[8], uint32_t position)
{
    uint32_t i;

    for (i = 0; i < 8; ++i) {
        uint32_t offset = bitmap_regular_hash_update_order[i];
        uint8_t *entry = state->hash + 2 * hashes[offset];

        if (state->level == 2) {
            uint32_t previous = bitmap_read16(entry);

            bitmap_write32(entry, (uint16_t)(position + offset) | (previous << 16));
        } else {
            bitmap_write16(entry, (uint16_t)(position + offset));
        }
    }
}

static struct bitmap_regular_group bitmap_regular_choose(struct bitmap_regular_state *state, const uint32_t hashes[8], uint32_t position, uint16_t recent_distance)
{
    struct bitmap_regular_group candidates[17];
    uint32_t previous[8];
    uint32_t count = state->level == 2 ? 16 : 8;
    uint32_t lookahead = state->level ? 16 : 8;
    uint32_t best = count;
    uint32_t best_score;
    uint32_t i;
    int has_exact_match = 0;

    for (i = 0; i < 8; ++i) {
        previous[i] = bitmap_read32(state->hash + 2 * hashes[bitmap_regular_candidate_offsets[i]]);
    }
    bitmap_regular_update_hash(state, hashes, position);
    for (i = 0; i < count; ++i) {
        uint32_t old_position = previous[i & 7] >> (i >= 8 ? 16 : 0);
        uint16_t distance = (uint16_t)(position + bitmap_regular_candidate_offsets[i & 7] - old_position);
        uint16_t mask;
        uint32_t score;

        mask = bitmap_regular_mask(state->source, position, distance, lookahead);
        if (distance < 8) {
            mask = lookahead == 16 ? UINT16_MAX : UINT16_C(255);
        }
        candidates[i].distance = distance;
        candidates[i].mask = mask;
        candidates[i].width = distance > 255 ? 2 : 1;
        score = bitmap_popcount(mask & 255) + candidates[i].width;
        if (state->level) {
            score = 4 * score + bitmap_popcount(mask >> 8);
        }
        candidates[i].score = (uint8_t)score;
        if (!mask) {
            has_exact_match = 1;
        }
    }
    candidates[count].distance = recent_distance;
    candidates[count].mask = bitmap_regular_mask(state->source, position, recent_distance, lookahead);
    candidates[count].width = 0;
    best_score = bitmap_popcount(candidates[count].mask & 255);
    if (state->level) {
        best_score = 4 * best_score + bitmap_popcount(candidates[count].mask >> 8);
    }
    candidates[count].score = (uint8_t)best_score;
    if (state->level == 2 && has_exact_match) {
        uint32_t discount = 0;

        for (i = 0; i < count; ++i) {
            uint32_t score = candidates[i].score - discount;

            if (best_score > score) {
                discount = candidates[i].mask ? 0 : 4;
                best_score = score;
                best = i;
            }
        }
    } else {
        for (i = 0; i < count; ++i) {
            if (best_score > candidates[i].score) {
                best_score = candidates[i].score;
                best = i;
            }
        }
    }
    return candidates[best];
}

static uint32_t bitmap_regular_select_symbols(uint32_t *histogram, uint32_t keep)
{
    uint32_t count = 0;
    uint32_t begin = 0;
    uint32_t end;
    uint32_t i;

    for (i = 0; i < 768; ++i) {
        histogram[count] = histogram[i];
        if (histogram[i] >> 16) {
            ++count;
        }
    }
    if (count <= keep) {
        memset(histogram + count, 0, (keep - count) * sizeof(*histogram));
        return count;
    }
    end = count;
    for (;;) {
        uint32_t middle = (begin + end) / 2;
        uint32_t temporary;
        uint32_t pivot;
        uint32_t left = begin;
        uint32_t right = end - 1;
        uint32_t boundary;

        if (histogram[begin] > histogram[end - 1]) {
            temporary = histogram[begin];
            histogram[begin] = histogram[end - 1];
            histogram[end - 1] = temporary;
        }
        if (histogram[middle] > histogram[end - 1]) {
            temporary = histogram[middle];
            histogram[middle] = histogram[end - 1];
            histogram[end - 1] = temporary;
        }
        if (histogram[middle] > histogram[begin]) {
            temporary = histogram[middle];
            histogram[middle] = histogram[begin];
            histogram[begin] = temporary;
        }
        pivot = histogram[begin];
        for (;;) {
            while (histogram[left] > pivot) {
                ++left;
            }
            while (histogram[right] < pivot) {
                --right;
            }
            if (left >= right) {
                break;
            }
            temporary = histogram[left];
            histogram[left] = histogram[right];
            histogram[right] = temporary;
            ++left;
            --right;
        }
        boundary = right + 1;
        if (boundary == keep) {
            return keep;
        }
        if (boundary < keep) {
            begin = boundary;
        } else {
            end = boundary;
        }
    }
}

static uint32_t bitmap_regular_rle(uint8_t *destination, const uint8_t *source, uint32_t count)
{
    uint32_t position = 0;
    uint32_t written = 0;

    while (position < count) {
        uint32_t end = position + 1;
        uint32_t length;

        while (end < count && source[end] == source[position]) {
            ++end;
        }
        length = end - position;
        if (length >= 4) {
            destination[written++] = source[position];
            destination[written++] = 15;
            length -= 4;
            while (length >= 15) {
                destination[written++] = 15;
                length -= 15;
            }
            destination[written++] = (uint8_t)length;
            position = end;
        } else {
            destination[written++] = source[position++];
        }
    }
    return written;
}

static size_t bitmap_regular_compress_block(struct bitmap_regular_state *state, uint8_t *destination, size_t available, uint32_t begin, uint32_t size)
{
    uint32_t position = begin;
    uint32_t parse_end = begin + size;
    uint32_t padded_end = begin + ((size + 63) & ~UINT32_C(63));
    uint32_t group_count = (padded_end - begin) / 8;
    uint32_t literal_count = 0;
    uint32_t distance_count = 0;
    uint16_t recent_distance = 8;
    uint8_t dictionary[17] = {0};
    uint32_t escape_count = group_count;
    uint32_t code_count;
    uint32_t i;
    size_t distance_start;
    size_t escape_start;
    size_t code_start;
    size_t dictionary_start;
    uint64_t tail_end;

    if (available < 15) {
        return 0;
    }
    for (i = 0; i < 768; ++i) {
        state->histogram[i] = i;
    }
    if (parse_end > state->source_size - 16) {
        parse_end = state->source_size - 16;
    }
    while (position + 128 <= parse_end) {
        uint32_t hashes[128];
        uint32_t batch_begin = position;

        if (143 + (size_t)literal_count > available) {
            return 0;
        }
        for (i = 0; i < 128; ++i) {
            hashes[i] = bitmap_regular_hash(state, position + i);
        }
        if (!position) {
            state->symbols[0] = 255;
            state->histogram[255] += UINT32_C(0x10000);
            memcpy(destination + 15, state->source, 8);
            literal_count = 8;
            position = 8;
        }
        while ((uint32_t)(position - batch_begin) < 128) {
            uint32_t group_index = (position - begin) / 8;
            uint32_t *group_hashes = hashes + (uint32_t)(position - batch_begin);

            if (bitmap_read64(state->source + position) == bitmap_read64(state->source + position - recent_distance)) {
                state->symbols[group_index] = 0;
                state->histogram[0] += UINT32_C(0x10000);
                bitmap_regular_update_hash(state, group_hashes, position);
            } else {
                struct bitmap_regular_group group;
                uint32_t symbol;

                group = bitmap_regular_choose(state, group_hashes, position, recent_distance);
                recent_distance = group.distance;
                bitmap_write16(state->distances + distance_count, recent_distance);
                distance_count += group.width;
                for (i = 0; i < 8; ++i) {
                    if (group.mask & (UINT32_C(1) << i)) {
                        destination[15 + literal_count++] = state->source[position + i];
                    }
                }
                symbol = (group.mask & 255) | ((uint32_t)group.width << 8);
                state->symbols[group_index] = (uint16_t)symbol;
                state->histogram[symbol] += UINT32_C(0x10000);
            }
            position += 8;
        }
    }

    tail_end = (uint64_t)(uintptr_t)destination + 15 + literal_count + padded_end - position;
    if (tail_end > (uint64_t)(uintptr_t)destination + available) {
        return 0;
    }
    while (position < padded_end) {
        uint32_t mask = 0;

        for (i = 0; i < 8; ++i) {
            if (position + i < state->source_size && state->source[position + i] != state->source[position - recent_distance + i]) {
                mask |= UINT32_C(1) << i;
                destination[15 + literal_count++] = state->source[position + i];
            }
        }
        state->symbols[(position - begin) / 8] = (uint16_t)mask;
        state->histogram[mask] += UINT32_C(0x10000);
        position += 8;
    }
    distance_start = 15 + literal_count;
    escape_start = distance_start + distance_count;
    if (escape_start > available) {
        return 0;
    }
    memcpy(destination + distance_start, state->distances, distance_count);
    for (i = 0; i < 768; ++i) {
        state->codes[i] = (uint16_t)(256 + (i >> 8));
    }
    bitmap_regular_select_symbols(state->histogram, state->dictionary_size);
    for (i = 0; i < state->dictionary_size; ++i) {
        uint32_t symbol;
        uint32_t bit;
        uint32_t j;

        if (state->histogram[i] < UINT32_C(0x10000)) {
            break;
        }
        symbol = state->histogram[i] & 65535;
        bit = 10 * i;
        for (j = 0; j < 10; ++j) {
            dictionary[(bit + j) / 8] |= (uint8_t)(((symbol >> j) & 1) << ((bit + j) & 7));
        }
        state->codes[symbol] = (uint16_t)(i + 3);
        escape_count -= state->histogram[i] >> 16;
    }
    code_start = escape_start + escape_count;
    if (code_start > available) {
        return 0;
    }
    escape_count = 0;
    for (i = 0; i < group_count; ++i) {
        uint16_t code = state->codes[state->symbols[i]];

        state->distances[i] = (uint8_t)code;
        destination[escape_start + escape_count] = (uint8_t)state->symbols[i];
        escape_count += code >> 8;
    }
    if (code_start + group_count / 2 > available) {
        return 0;
    }
    if (state->level) {
        code_count = bitmap_regular_rle((uint8_t *)state->symbols, state->distances, group_count);
        memcpy(state->distances, state->symbols, code_count);
    } else {
        code_count = group_count;
    }
    memset(destination + code_start, 0, (code_count + 1) / 2);
    for (i = 0; i < code_count; ++i) {
        destination[code_start + i / 2] |= (uint8_t)(state->distances[i] << (4 * (i & 1)));
    }
    dictionary_start = code_start + (code_count + 1) / 2;
    if (dictionary_start + 17 > available) {
        return 0;
    }
    memcpy(destination + dictionary_start, dictionary, 17);
    bitmap_write24(destination + 6, (uint32_t)distance_start);
    bitmap_write24(destination + 9, (uint32_t)escape_start);
    bitmap_write24(destination + 12, (uint32_t)code_start);
    return dictionary_start + 17;
}

static size_t bitmap_regular_frame(struct bitmap_regular_state *state, uint8_t *destination, size_t capacity)
{
    size_t written = 4;
    size_t limit;
    uint32_t begin = 0;

    if (!destination || capacity < 35) {
        return 0;
    }
    limit = capacity - 31;
    bitmap_write32(destination, UINT32_C(0x004d425a) | ((state->level ? UINT32_C(9) : UINT32_C(12)) << 24));
    for (;;) {
        uint32_t size = begin < state->source_size ? state->source_size - begin : 0;
        size_t stored_size = 0;

        if (size > state->block_size) {
            size = state->block_size;
        }
        if (written > limit || limit - written < 6) {
            return 0;
        }
        if (size >= 145) {
            stored_size = bitmap_regular_compress_block(state, destination + written, limit - written, begin, size);
        }
        if (!stored_size || stored_size >= size + 6) {
            if (size > limit - written - 6) {
                return 0;
            }
            if (size) {
                memcpy(destination + written + 6, state->source + begin, size);
            }
            stored_size = size + 6;
        }
        bitmap_write24(destination + written, (uint32_t)stored_size);
        bitmap_write24(destination + written + 3, size);
        written += stored_size;
        if (!size) {
            return written;
        }

        begin += state->block_size;
    }
}

static size_t bitmap_encode_regular(uint8_t *destination, size_t capacity, const uint8_t *source, size_t size, uint32_t level)
{
    struct bitmap_regular_state state;
    uint8_t *allocation;
    uintptr_t base;
    uintptr_t distances;
    uintptr_t symbols;
    uintptr_t histogram;
    size_t scratch_size;
    size_t result;
    size_t i;
    uint32_t hash_size;
    volatile uint8_t *erase;

    if (level > 2) {
        return 0;
    }
    scratch_size = level ? 545755 : 78811;
    allocation = malloc(scratch_size);
    if (!allocation) {
        return 0;
    }
    result = 0;
    if (size <= UINT32_MAX && (source || !size)) {
        state.level = level;
        state.hash_bits = level ? 18 : 15;
        state.block_size = level ? 32768 : 16384;
        state.dictionary_size = level ? 12 : 13;
        state.source = source;
        state.source_size = (uint32_t)size;
        hash_size = (UINT32_C(2) << state.hash_bits) + 16;

        base = (uintptr_t)allocation;
        state.hash = (uint8_t *)((base + 199) & ~(uintptr_t)63);
        distances = ((base + 199) | 56) + hash_size;
        state.distances = (uint8_t *)(distances & ~(uintptr_t)63);
        symbols = (distances | 63) + ((state.block_size / 4) | 3);
        state.symbols = (uint16_t *)(symbols & ~(uintptr_t)63);
        histogram = (symbols + state.block_size / 4) & ~(uintptr_t)63;
        state.histogram = (uint32_t *)histogram;
        state.codes = (uint16_t *)(histogram + 3072);
        for (i = 0; i < hash_size; i += 2) {
            bitmap_write16(state.hash + i, 8);
        }
        if (capacity > UINT32_MAX) {
            capacity = UINT32_MAX;
        }
        result = bitmap_regular_frame(&state, destination, capacity);
    }
    erase = allocation;
    for (i = 0; i < scratch_size; ++i) {
        erase[i] = 0;
    }
    free(allocation);
    return result;
}

static uint32_t bitmap_fast_hash(const uint8_t *source, uint32_t level)
{
    uint32_t product = (uint32_t)((uint64_t)bitmap_read32(source) * BITMAP_FAST_HASH_MULTIPLIER);

    return product >> (level == 2 ? 17 : 19);
}

static uint8_t bitmap_fast_difference(const uint8_t *source, uint32_t position, uint16_t distance)
{
    uint32_t mask = 0;
    uint32_t i;

    for (i = 0; i < 8; ++i) {
        mask |= (uint32_t)(source[position + i] != source[position - distance + i]) << i;
    }
    return (uint8_t)mask;
}

static struct bitmap_fast_match bitmap_fast_choose(const uint8_t *source, uint32_t position, uint16_t *positions, uint32_t level, uint16_t recent_distance)
{
    struct bitmap_fast_match match;
    uint32_t hashes[8];
    uint16_t candidates[3];
    uint32_t candidate_count = level == 2 ? 3 : 2;
    uint32_t cost;
    uint32_t i;

    match.distance = recent_distance;
    match.mask = bitmap_fast_difference(source, position, recent_distance);
    if (!(match.mask & (match.mask - 1))) {
        return match;
    }
    cost = bitmap_popcount(match.mask);
    hashes[0] = bitmap_fast_hash(source + position, level);
    hashes[4] = bitmap_fast_hash(source + position + 4, level);
    if (!level) {
        candidates[0] = (uint16_t)(position - positions[hashes[0]]);
        candidates[1] = (uint16_t)(position + 4 - positions[hashes[4]]);
        positions[hashes[4]] = (uint16_t)(position + 4);
        positions[hashes[0]] = (uint16_t)position;
    } else {
        hashes[1] = bitmap_fast_hash(source + position + 1, level);
        hashes[2] = bitmap_fast_hash(source + position + 2, level);
        hashes[3] = bitmap_fast_hash(source + position + 3, level);
        hashes[5] = bitmap_fast_hash(source + position + 5, level);
        hashes[6] = bitmap_fast_hash(source + position + 6, level);
        hashes[7] = bitmap_fast_hash(source + position + 7, level);
        candidates[0] = (uint16_t)(position + 4 - positions[hashes[4]]);
        candidates[1] = (uint16_t)(position - positions[hashes[0]]);
        if (level == 2) {
            candidates[2] = (uint16_t)(position + 2 - positions[hashes[2]]);
        }

        positions[hashes[1]] = (uint16_t)(position + 1);
        positions[hashes[2]] = (uint16_t)(position + 2);
        positions[hashes[3]] = (uint16_t)(position + 3);
        positions[hashes[5]] = (uint16_t)(position + 5);
        positions[hashes[6]] = (uint16_t)(position + 6);
        positions[hashes[7]] = (uint16_t)(position + 7);
        positions[hashes[4]] = (uint16_t)(position + 4);
        positions[hashes[0]] = (uint16_t)position;
    }
    for (i = 0; i < candidate_count; ++i) {
        uint16_t distance = candidates[i];
        uint8_t mask = bitmap_fast_difference(source, position, distance);
        uint32_t candidate_cost;

        if (distance < 8) {
            mask = UINT8_MAX;
        }
        if (!mask) {
            match.distance = distance;
            match.mask = 0;
            return match;
        }
        candidate_cost = bitmap_popcount(mask) + 2;
        if (candidate_cost < cost) {
            match.distance = distance;
            match.mask = mask;
            cost = candidate_cost;
        }
    }
    return match;
}

static size_t bitmap_fast_write_raw(uint8_t *destination, const uint8_t *source, uint32_t size)
{
    bitmap_write16(destination, UINT16_MAX);
    bitmap_write16(destination + 2, size);
    destination[4] = 0;
    if (size) {
        memcpy(destination + 5, source, size);
    }
    return (size_t)size + 5;
}

static size_t bitmap_fast_encode_group(uint8_t *destination, const uint8_t *source, uint32_t position, uint16_t *positions, uint32_t level, uint16_t *recent_distance)
{
    struct bitmap_fast_group group = {0};
    uint16_t distance = *recent_distance;
    uint32_t i;
    size_t size;
    size_t offset;

    for (i = 0; i < 16; ++i) {
        uint32_t cell_position = position + 8 * i;
        struct bitmap_fast_match match = bitmap_fast_choose(source, cell_position, positions, level, distance);
        uint32_t j;

        if (match.distance != distance) {
            group.flags |= UINT32_C(1) << (16 + i);
            group.distances[group.distance_count++] = match.distance;
            distance = match.distance;
        }
        if (match.mask) {
            group.flags |= UINT32_C(1) << i;
            group.masks[group.mask_count++] = match.mask;
            for (j = 0; j < 8; ++j) {
                if (match.mask & (UINT32_C(1) << j)) {
                    group.literals[group.literal_count++] = source[cell_position + j];
                }
            }
        }
    }
    size = 4 + group.mask_count + 2 * group.distance_count + group.literal_count;
    if (size > BITMAP_FAST_GROUP_SIZE + 5) {

        return bitmap_fast_write_raw(destination, source + position, BITMAP_FAST_GROUP_SIZE);
    }
    bitmap_write32(destination, group.flags);
    memcpy(destination + 4, group.masks, group.mask_count);
    offset = 4 + group.mask_count;
    for (i = 0; i < group.distance_count; ++i) {
        bitmap_write16(destination + offset, group.distances[i]);
        offset += 2;
    }
    memcpy(destination + offset, group.literals, group.literal_count);
    *recent_distance = distance;
    return size;
}

static size_t bitmap_fast_encode_stream(uint8_t *destination, size_t capacity, const uint8_t *source, size_t size, uint16_t *positions, uint32_t level)
{
    size_t prefix_size = size < 8 ? size : 8;
    size_t output_offset;
    uint32_t position;
    uint32_t tail_size;
    uint16_t recent_distance = 8;
    uint32_t hash_count = level == 2 ? 32768 : 8192;
    uint32_t i;

    if (!size || size > UINT32_MAX || !destination || !source) {
        return 0;
    }
    if (capacity > UINT32_MAX) {
        capacity = UINT32_MAX;
    }
    if (capacity < prefix_size + 5) {
        return 0;
    }
    output_offset = bitmap_fast_write_raw(destination, source, (uint32_t)prefix_size);
    position = (uint32_t)prefix_size;
    for (i = 0; i < hash_count; ++i) {
        positions[i] = 8;
    }
    while (size - position >= BITMAP_FAST_GROUP_LOOKAHEAD) {

        if (capacity - output_offset < BITMAP_FAST_GROUP_CAPACITY) {
            return 0;
        }
        output_offset += bitmap_fast_encode_group(destination + output_offset, source, position, positions, level, &recent_distance);
        position += BITMAP_FAST_GROUP_SIZE;
    }
    tail_size = (uint32_t)size - position;
    if (capacity - output_offset < (size_t)tail_size + 5) {
        return 0;
    }
    output_offset += bitmap_fast_write_raw(destination + output_offset, source + position, tail_size);
    if (tail_size) {
        if (capacity - output_offset < 5) {
            return 0;
        }
        output_offset += bitmap_fast_write_raw(destination + output_offset, source + size, 0);
    }
    return output_offset;
}

static size_t bitmap_encode_fast(uint8_t *destination, size_t capacity, const uint8_t *source, size_t size, uint32_t level)
{
    uint16_t *positions;
    volatile uint8_t *wipe;
    size_t state_size;
    size_t result;
    size_t i;

    if (level > 2) {
        return 0;
    }
    state_size = level == 2 ? 65536 : 16384;
    positions = malloc(state_size);
    if (!positions) {
        return 0;
    }
    result = bitmap_fast_encode_stream(destination, capacity, source, size, positions, level);
    wipe = (volatile uint8_t *)positions;
    for (i = 0; i < state_size; ++i) {
        wipe[i] = 0;
    }
    free(positions);
    return result;
}

static int bitmap_decode_rle(uint8_t *codes, size_t count, const uint8_t *nibbles)
{
    size_t input = 0;
    size_t output = 0;

    if (!count || nibbles[0] == 15) {
        return 1;
    }
    while (output < count) {
        size_t copied = 0;

        if (input > 4160 - 16 || output > 4160 - 16) {
            return 0;
        }
        memcpy(codes + output, nibbles + input, 16);
        while (copied < 16 && nibbles[input + copied] != 15) {
            ++copied;
        }
        input += copied;
        output += copied;
        if (copied < 16) {
            uint8_t value;
            uint32_t extension;

            if (!input || output > 4160 - 16) {
                return 0;
            }
            value = nibbles[input - 1];
            memset(codes + output, value, 16);
            output += 3;
            ++input;
            do {
                if (input >= 4160 || output > 4160 - 16) {
                    return 0;
                }
                extension = nibbles[input++];
                memset(codes + output, value, 16);
                output += extension;
            } while (extension >= 15 && output < count);
        }
    }
    return 1;
}

static size_t bitmap_decode_regular(uint8_t *destination, size_t capacity, const uint8_t *source, size_t size, uint8_t *scratch)
{
    uint8_t literal_table[16] = {0};
    uint8_t distance_table[16] = {0};
    uint8_t *codes = scratch;
    uint8_t *masks = scratch + 4160;
    uint32_t block_limit;
    uint32_t direct_codes;
    size_t position = 4;
    size_t written = 0;

    if (size < 4 || bitmap_read24(source) != UINT32_C(0x4d425a) || source[3] > 15 || !(source[3] & 8) || (source[3] & 3) >= 2) {
        return 0;
    }
    block_limit = source[3] & 1 ? 32768 : 16384;
    direct_codes = (source[3] >> 2) & 1;
    while (written < capacity) {
        const uint8_t *block;
        size_t block_size;
        size_t decoded_size;
        size_t output_end;
        size_t literal_cursor;
        size_t distance_cursor;
        size_t escape_start;
        size_t code_start;
        size_t table_start;
        size_t code_bytes;
        size_t group_count;
        size_t escape_cursor;
        size_t group = 0;
        uint32_t recent_distance = 8;
        uint32_t index;
        int vector_batch;
        int partial_output;

        if (position > size || size - position < 6) {
            return 0;
        }
        block = source + position;
        block_size = bitmap_read24(block);
        decoded_size = bitmap_read24(block + 3);
        if (decoded_size > block_limit || block_size > decoded_size + 6) {
            return 0;
        }
        partial_output = decoded_size > capacity - written;
        output_end = partial_output ? capacity : written + decoded_size;
        if (block_size == decoded_size + 6) {
            size_t count = output_end - written;

            if (!decoded_size) {
                return written;
            }
            if (count > size - position - 6) {
                return 0;
            }
            memcpy(destination + written, block + 6, count);
            written = output_end;
            position += block_size;
            continue;
        }
        if (block_size < 33 || block_size > size - position) {
            return 0;
        }
        position += block_size;
        literal_cursor = 15;
        distance_cursor = bitmap_read24(block + 6);
        escape_start = bitmap_read24(block + 9);
        code_start = bitmap_read24(block + 12);
        table_start = block_size - 17;
        if (distance_cursor < 15 || distance_cursor > escape_start || escape_start > code_start || code_start > table_start) {
            return 0;
        }
        code_bytes = table_start - code_start;
        group_count = ((output_end - written + 63) >> 3) & ~(size_t)7;
        for (index = 0; index < 3; ++index) {
            literal_table[index] = 0;
            distance_table[index] = (uint8_t)index;
        }
        for (index = 3; index < 15 + direct_codes; ++index) {
            uint32_t bit = (index - 3) * 10;
            uint32_t entry = bitmap_read16(block + table_start + (bit >> 3)) >> (bit & 7);

            if (((entry >> 8) & 3) == 3) {
                return 0;
            }
            literal_table[index] = (uint8_t)entry;
            distance_table[index] = (uint8_t)((entry >> 8) & 3);
        }
        if (direct_codes) {
            size_t byte;

            if (code_bytes < group_count / 2) {
                return 0;
            }
            for (byte = 0; byte < group_count / 2; ++byte) {
                codes[2 * byte] = block[code_start + byte] & 15;
                codes[2 * byte + 1] = block[code_start + byte] >> 4;
            }
        } else {
            size_t byte;
            size_t unpacked;

            if (code_bytes > block_limit / 16) {
                return 0;
            }
            unpacked = (code_bytes + 15) & ~(size_t)15;
            for (byte = 0; byte < unpacked; ++byte) {
                masks[2 * byte] = block[code_start + byte] & 15;
                masks[2 * byte + 1] = block[code_start + byte] >> 4;
            }
            if (!bitmap_decode_rle(codes, group_count, masks)) {
                return 0;
            }
        }
        escape_cursor = escape_start;
        for (group = 0; group < group_count; group += 8) {
            uint32_t cell;

            if (escape_cursor > table_start) {
                return 0;
            }
            for (cell = 0; cell < 8; ++cell) {
                uint32_t code = codes[group + cell];

                masks[group + cell] = code < 16 ? literal_table[code] : 0;
                codes[group + cell] = code < 16 ? distance_table[code] : 0;
                if (code < 3) {
                    masks[group + cell] = block[escape_cursor++];
                }
            }
        }
        group = 0;
        vector_batch = output_end - written >= 64 && table_start >= 63;
        while (written < output_end) {
            uint32_t cells = vector_batch ? 8 : 1;
            uint32_t cell;

            if (!vector_batch && (distance_cursor > table_start || literal_cursor > table_start)) {
                return 0;
            }
            for (cell = 0; cell < cells; ++cell, ++group) {
                uint32_t width = codes[group];
                uint32_t mask = masks[group];
                size_t base;
                uint32_t byte;
                uint8_t previous[8];

                if (width == 1) {
                    recent_distance = block[distance_cursor++];
                } else if (width == 2) {
                    recent_distance = bitmap_read16(block + distance_cursor);
                    distance_cursor += 2;
                }
                if (!vector_batch && written < recent_distance && mask != 255) {
                    return 0;
                }
                base = written < recent_distance ? 0 : written - recent_distance;
                if (vector_batch) {
                    memcpy(previous, destination + base, 8);
                }
                for (byte = 0; byte < 8 && written < output_end; ++byte, ++written) {
                    if (mask & (UINT32_C(1) << byte)) {
                        destination[written] = block[literal_cursor++];
                    } else {
                        destination[written] = vector_batch ? previous[byte] : destination[base + byte];
                    }
                }
            }
            vector_batch = vector_batch && output_end - written >= 64 && distance_cursor <= table_start && literal_cursor + 48 <= table_start;
        }
        if (partial_output) {
            if (literal_cursor > bitmap_read24(block + 6) || distance_cursor > escape_start) {
                return 0;
            }
        } else if (literal_cursor != bitmap_read24(block + 6) || distance_cursor != escape_start) {
            return 0;
        }
    }
    return written;
}

static size_t bitmap_decode_fast(uint8_t *destination, size_t capacity, const uint8_t *source, size_t size)
{
    size_t position = 0;
    size_t written = 0;
    uint32_t recent_distance = 8;

    while (written < capacity) {
        uint32_t literal_flags;
        uint32_t distance_flags;
        size_t masks;
        size_t distances;
        size_t literals;
        uint32_t cell;
        int vector_batch;

        if (position > size || size - position < 5) {
            return 0;
        }
        literal_flags = bitmap_read16(source + position);
        distance_flags = bitmap_read16(source + position + 2);
        if (literal_flags == 65535 && source[position + 4] == 0) {
            size_t count = distance_flags;

            if (!count) {
                return written;
            }
            if (count > size - position - 5) {
                return 0;
            }
            if (count > capacity - written) {
                count = capacity - written;
            }
            memcpy(destination + written, source + position + 5, count);
            written += count;
            position += count + 5;
            continue;
        }
        masks = position + 4;
        if (bitmap_popcount(literal_flags) + 2 * bitmap_popcount(distance_flags) > size - masks) {
            return 0;
        }
        distances = masks + bitmap_popcount(literal_flags);
        literals = distances + 2 * bitmap_popcount(distance_flags);
        vector_batch = capacity - written >= 128 && size - position >= 180;
        for (cell = 0; cell < 16 && written < capacity; ++cell) {
            uint32_t mask = 0;
            uint32_t byte;
            size_t base;
            uint8_t previous[8];

            if (distance_flags & (UINT32_C(1) << cell)) {
                recent_distance = bitmap_read16(source + distances);
                distances += 2;
            }
            if (literal_flags & (UINT32_C(1) << cell)) {
                mask = source[masks++];
            }
            if (written < recent_distance && mask != 255) {
                return 0;
            }
            if (bitmap_popcount(mask) > size - literals) {
                return 0;
            }
            base = written < recent_distance ? 0 : written - recent_distance;
            if (vector_batch) {
                memcpy(previous, destination + base, 8);
            }
            for (byte = 0; byte < 8 && written < capacity; ++byte, ++written) {
                if (mask & (UINT32_C(1) << byte)) {
                    destination[written] = source[literals++];
                } else {
                    destination[written] = vector_batch ? previous[byte] : destination[base + byte];
                }
            }
        }
        position = literals;
    }
    return written;
}

size_t lzbitmap_encode_bound(size_t size, uint32_t algorithm)
{
    size_t overhead;
    size_t block_size;

    if (size > UINT32_MAX) {
        return 0;
    }
    switch (algorithm) {
    case LZBITMAP_FAST_0:
    case LZBITMAP_FAST_1:
    case LZBITMAP_FAST_2:
        if (!size) {
            return 0;
        }
        overhead = size <= 8 ? 10 : 5 * (size / 128) + 20;
        break;
    case LZBITMAP_0:
    case LZBITMAP_1:
    case LZBITMAP_2:
        block_size = algorithm == LZBITMAP_0 ? 16384 : 32768;
        overhead = 6 * (size / block_size + (size % block_size != 0)) + 41;
        break;
    default:
        return 0;
    }
    return size <= UINT32_MAX - overhead ? size + overhead : 0;
}

size_t lzbitmap_encode_buffer(void *destination, size_t capacity, const void *source, size_t size, uint32_t algorithm)
{
    if ((!destination && capacity) || (!source && size)) {
        return 0;
    }
    switch (algorithm) {
    case LZBITMAP_FAST_0:
    case LZBITMAP_FAST_1:
    case LZBITMAP_FAST_2:
        return bitmap_encode_fast(destination, capacity, source, size, algorithm - LZBITMAP_FAST_0);
    case LZBITMAP_0:
    case LZBITMAP_1:
    case LZBITMAP_2:
        return bitmap_encode_regular(destination, capacity, source, size, algorithm - LZBITMAP_0);
    default:
        return 0;
    }
}

size_t lzbitmap_decode_buffer(void *destination, size_t capacity, const void *source, size_t size, uint32_t algorithm)
{
    uint8_t *scratch;
    volatile uint8_t *wipe;
    size_t result;
    size_t index;

    if ((!destination && capacity) || (!source && size)) {
        return 0;
    }
    switch (algorithm) {
    case LZBITMAP_FAST_0:
    case LZBITMAP_FAST_1:
    case LZBITMAP_FAST_2:
        if (size > UINT32_MAX || capacity > UINT32_MAX) {
            return 0;
        }
        return bitmap_decode_fast(destination, capacity, source, size);
    case LZBITMAP_0:
    case LZBITMAP_1:
    case LZBITMAP_2:
        break;
    default:
        return 0;
    }
    scratch = malloc(8320);
    if (!scratch) {
        return 0;
    }
    result = 0;
    if (size <= UINT32_MAX && capacity <= UINT32_MAX) {
        result = bitmap_decode_regular(destination, capacity, source, size, scratch);
    }
    wipe = scratch;
    for (index = 0; index < 8320; ++index) {
        wipe[index] = 0;
    }
    free(scratch);
    return result;
}
