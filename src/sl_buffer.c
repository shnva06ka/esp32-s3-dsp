/*
 * Squeezelite buffer management ported for ESP32-S3
 * Based on squeezelite (c) Adrian Smith, Ralph Irving
 */
#include "squeezelite.h"

unsigned _buf_used(struct buffer *buf) {
    if (!buf || !buf->buf) return 0;
    return buf->writep >= buf->readp ? (unsigned)(buf->writep - buf->readp) : (unsigned)(buf->size - (buf->readp - buf->writep));
}

unsigned _buf_space(struct buffer *buf) {
    if (!buf || !buf->buf) return 0;
    return (unsigned)(buf->size - _buf_used(buf) - 1);
}

unsigned _buf_cont_read(struct buffer *buf) {
    if (!buf || !buf->buf) return 0;
    return buf->writep >= buf->readp ? (unsigned)(buf->writep - buf->readp) : (unsigned)(buf->wrap - buf->readp);
}

unsigned _buf_cont_write(struct buffer *buf) {
    if (!buf || !buf->buf) return 0;
    return buf->writep >= buf->readp ? (unsigned)(buf->wrap - buf->writep) : (unsigned)(buf->readp - buf->writep);
}

void _buf_inc_readp(struct buffer *buf, unsigned by) {
    if (!buf || !buf->buf) return;
    buf->readp += by;
    if (buf->readp >= buf->wrap) {
        buf->readp -= buf->size;
    }
}

void _buf_inc_writep(struct buffer *buf, unsigned by) {
    if (!buf || !buf->buf) return;
    buf->writep += by;
    if (buf->writep >= buf->wrap) {
        buf->writep -= buf->size;
    }
}

void buf_flush(struct buffer *buf) {
    if (!buf) return;
    mutex_lock(buf->mutex);
    buf->readp  = buf->buf;
    buf->writep = buf->buf;
    mutex_unlock(buf->mutex);
}

void _buf_flush(struct buffer *buf) {
    if (!buf) return;
    buf->readp  = buf->buf;
    buf->writep = buf->buf;
}

void buf_adjust(struct buffer *buf, size_t mod) {
    if (!buf) return;
    mutex_lock(buf->mutex);
    buf->base_size = ((size_t)(buf->size / mod)) * mod;
    buf->readp  = buf->writep = buf->buf;
    buf->wrap   = buf->buf + buf->base_size;
    buf->size   = buf->base_size;
    mutex_unlock(buf->mutex);
}

void buf_init(struct buffer *buf, size_t size) {
    if (!buf) return;
    memset(buf, 0, sizeof(struct buffer));
    mutex_create(buf->mutex);

    buf->buf = (u8_t *)heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf->buf) {
        buf->buf = (u8_t *)malloc(size);
    }

    if (!buf->buf) {
        LOG_ERROR("Failed to allocate buffer of %u bytes", (unsigned)size);
        buf->size = 0;
        return;
    }

    buf->readp  = buf->buf;
    buf->writep = buf->buf;
    buf->wrap   = buf->buf + size;
    buf->size   = size;
    buf->base_size = size;
}

void buf_destroy(struct buffer *buf) {
    if (!buf) return;
    mutex_lock(buf->mutex);
    if (buf->buf) {
        free(buf->buf);
        buf->buf = NULL;
    }
    buf->readp = buf->writep = buf->wrap = NULL;
    buf->size = buf->base_size = 0;
    mutex_unlock(buf->mutex);
    mutex_destroy(buf->mutex);
}

