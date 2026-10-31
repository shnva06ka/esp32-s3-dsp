/*
 * Squeezelite utilities ported for ESP32-S3
 * Based on squeezelite (c) Adrian Smith, Ralph Irving
 */
#include "squeezelite.h"

log_level loglevel = lINFO;

const char *logtime(void) {
    static char buf[32];
    uint32_t now = gettime_ms();
    uint32_t ms = now % 1000;
    uint32_t s  = (now / 1000) % 60;
    uint32_t m  = (now / 60000) % 60;
    uint32_t h  = (now / 3600000);
    snprintf(buf, sizeof(buf), "[%02u:%02u:%02u.%03u]", (unsigned)h, (unsigned)m, (unsigned)s, (unsigned)ms);
    return buf;
}

extern void console_log(const char* buf);

void logprint(const char *fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    console_log(buf);
}

u32_t gettime_ms(void) {
    return (u32_t)(esp_timer_get_time() / 1000ULL);
}

void set_nonblock(sockfd s) {
    int flags = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, flags | O_NONBLOCK);
}

int connect_timeout(sockfd sock, const struct sockaddr *addr, socklen_t addrlen, int timeout) {
    if (connect(sock, addr, addrlen) < 0) {
        if (last_error() != EINPROGRESS) {
            return -1;
        }
    } else {
        return 0; // Immediate connection
    }

    fd_set w, e;
    struct timeval tval;
    FD_ZERO(&w);
    FD_SET(sock, &w);
    e = w;
    tval.tv_sec = timeout;
    tval.tv_usec = 0;

    if (select(sock + 1, NULL, &w, &e, timeout ? &tval : NULL) == 1 && FD_ISSET(sock, &w)) {
        int error = 0;
        socklen_t len = sizeof(error);
        getsockopt(sock, SOL_SOCKET, SO_ERROR, (void *)&error, &len);
        return error == 0 ? 0 : -1;
    }

    return -1;
}

void _wake_create(event_event *e) {
    struct sockaddr_in addr;
    socklen_t len;

    e->mfds = e->fds[0] = e->fds[1] = -1;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;

    e->mfds = socket(AF_INET, SOCK_STREAM, 0);
    if (e->mfds < 0) {
        LOG_WARN("Failed to create wake master socket");
        return;
    }

    if (bind(e->mfds, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        LOG_WARN("Failed to bind wake master socket: %s", strerror(last_error()));
        closesocket(e->mfds);
        e->mfds = -1;
        return;
    }

    len = sizeof(addr);
    getsockname(e->mfds, (struct sockaddr *)&addr, &len);
    listen(e->mfds, 1);

    e->fds[0] = socket(AF_INET, SOCK_STREAM, 0);
    if (e->fds[0] < 0) {
        LOG_WARN("Failed to create wake receiver socket");
        closesocket(e->mfds);
        e->mfds = -1;
        return;
    }

    if (connect(e->fds[0], (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        LOG_WARN("Failed to connect wake socket: %s", strerror(last_error()));
        closesocket(e->fds[0]);
        closesocket(e->mfds);
        e->fds[0] = e->mfds = -1;
        return;
    }

    len = sizeof(addr);
    e->fds[1] = accept(e->mfds, (struct sockaddr *)&addr, &len);
    if (e->fds[1] < 0) {
        LOG_WARN("Failed to accept wake socket: %s", strerror(last_error()));
        closesocket(e->fds[0]);
        closesocket(e->mfds);
        e->fds[0] = e->mfds = -1;
        return;
    }

    set_nonblock(e->fds[0]);
    set_nonblock(e->fds[1]);
    LOG_INFO("Wake sockets created: fds[0]=%d, fds[1]=%d", e->fds[0], e->fds[1]);
}

void _wake_close(event_event *e) {
    if (e->mfds >= 0) closesocket(e->mfds);
    if (e->fds[0] >= 0) closesocket(e->fds[0]);
    if (e->fds[1] >= 0) closesocket(e->fds[1]);
    e->mfds = e->fds[0] = e->fds[1] = -1;
}

void set_readwake_handles(event_handle handles[], sockfd s, event_event e) {
    handles[0].fd = s;
    handles[0].events = POLLIN;
    handles[0].revents = 0;

    handles[1].fd = e.fds[0];
    handles[1].events = POLLIN;
    handles[1].revents = 0;
}

event_type wait_readwake(event_handle handles[], int timeout) {
    int nfds = (handles[1].fd >= 0) ? 2 : 1;
    int ret = poll(handles, nfds, timeout);
    if (ret > 0) {
        if (handles[0].revents & (POLLIN | POLLHUP | POLLERR)) {
            return EVENT_READ;
        }
        if (nfds > 1 && (handles[1].revents & (POLLIN | POLLHUP | POLLERR))) {
            wake_clear(handles[1].fd);
            return EVENT_WAKE;
        }
    }
    return EVENT_TIMEOUT;
}

void packN(u32_t *dest, u32_t val) {
    u8_t *ptr = (u8_t *)dest;
    ptr[0] = (val >> 24) & 0xFF;
    ptr[1] = (val >> 16) & 0xFF;
    ptr[2] = (val >> 8)  & 0xFF;
    ptr[3] = val & 0xFF;
}

void packn(u16_t *dest, u16_t val) {
    u8_t *ptr = (u8_t *)dest;
    ptr[0] = (val >> 8) & 0xFF;
    ptr[1] = val & 0xFF;
}

u32_t unpackN(const u32_t *src) {
    const u8_t *ptr = (const u8_t *)src;
    return ((u32_t)ptr[0] << 24) | ((u32_t)ptr[1] << 16) | ((u32_t)ptr[2] << 8) | (u32_t)ptr[3];
}

u16_t unpackn(const u16_t *src) {
    const u8_t *ptr = (const u8_t *)src;
    return ((u16_t)ptr[0] << 8) | (u16_t)ptr[1];
}

in_addr_t discover_server(char *default_server, int max) {
    struct sockaddr_in d, s;
    char buf[64];
    struct pollfd pollinfo;

    int disc_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (disc_sock < 0) return 0;

    int enable = 1;
    setsockopt(disc_sock, SOL_SOCKET, SO_BROADCAST, &enable, sizeof(enable));

    // Standard discovery packet from slimproto: 'e' followed by tags
    int len = snprintf(buf, sizeof(buf), "eJSON%cCLIP%c", '\0', '\0');

    memset(&d, 0, sizeof(d));
    d.sin_family = AF_INET;
    d.sin_port = htons(3483);
    d.sin_addr.s_addr = htonl(INADDR_BROADCAST);

    pollinfo.fd = disc_sock;
    pollinfo.events = POLLIN;

    int attempts = 0;
    while (max == 0 || attempts < max) {
        attempts++;
        LOG_INFO("Broadcasting LMS discovery packet (attempt %d)...", attempts);
        if (sendto(disc_sock, buf, len, 0, (struct sockaddr *)&d, sizeof(d)) < 0) {
            LOG_WARN("Error sending discovery packet");
        }

        if (poll(&pollinfo, 1, 2000) > 0 && (pollinfo.revents & POLLIN)) {
            socklen_t slen = sizeof(s);
            int rlen = recvfrom(disc_sock, buf, sizeof(buf) - 1, 0, (struct sockaddr *)&s, &slen);
            if (rlen > 0 && buf[0] == 'E') {
                closesocket(disc_sock);
                LOG_INFO("Discovered LMS server at %s", inet_ntoa(s.sin_addr));
                return s.sin_addr.s_addr;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    closesocket(disc_sock);
    return 0;
}
