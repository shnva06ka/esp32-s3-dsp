#include <esp_wifi.h>
#include "lwip/opt.h"
#include "lwip/tcp.h"
#include "lwip/priv/tcp_priv.h"
#include "lwip/sys.h"

#define TUNED_TCP_WND           65535U
#define STOCK_TCP_WND           5760U
#define TUNED_MBOX_MIN_SIZE     64
#define TUNED_WIFI_STATIC_RX    16
#define TUNED_WIFI_DYNAMIC_RX   64
#define TUNED_WIFI_RX_BA_WIN    16
#define TUNED_WIFI_CACHE_TX     32
#define TUNED_WIFI_DYNAMIC_TX   32

extern esp_err_t __real_esp_wifi_init(const wifi_init_config_t *config);
extern err_t __real_sys_mbox_new(sys_mbox_t *mbox, int size);
extern struct tcp_pcb *__real_tcp_alloc(u8_t prio);
extern err_t __real_tcp_enqueue_flags(struct tcp_pcb *pcb, u8_t flags);
extern err_t __real_tcp_close(struct tcp_pcb *pcb);

esp_err_t __wrap_esp_wifi_init(const wifi_init_config_t *config)
{
    if (!config) {
        return __real_esp_wifi_init(config);
    }
    wifi_init_config_t cfg = *config;
    if (cfg.static_rx_buf_num < TUNED_WIFI_STATIC_RX) {
        cfg.static_rx_buf_num = TUNED_WIFI_STATIC_RX;
    }
    if (cfg.dynamic_rx_buf_num < TUNED_WIFI_DYNAMIC_RX) {
        cfg.dynamic_rx_buf_num = TUNED_WIFI_DYNAMIC_RX;
    }
    if (cfg.rx_ba_win < TUNED_WIFI_RX_BA_WIN) {
        cfg.rx_ba_win = TUNED_WIFI_RX_BA_WIN;
    }
    if (cfg.tx_buf_type == 1 && cfg.dynamic_tx_buf_num < TUNED_WIFI_DYNAMIC_TX) {
        cfg.dynamic_tx_buf_num = TUNED_WIFI_DYNAMIC_TX;
    }
    if (cfg.cache_tx_buf_num > 0 && cfg.cache_tx_buf_num < TUNED_WIFI_CACHE_TX) {
        cfg.cache_tx_buf_num = TUNED_WIFI_CACHE_TX;
    }
    return __real_esp_wifi_init(&cfg);
}

err_t __wrap_sys_mbox_new(sys_mbox_t *mbox, int size)
{
    if (size > 0 && size < TUNED_MBOX_MIN_SIZE) {
        size = TUNED_MBOX_MIN_SIZE;
    }
    return __real_sys_mbox_new(mbox, size);
}

struct tcp_pcb *__wrap_tcp_alloc(u8_t prio)
{
    struct tcp_pcb *pcb = __real_tcp_alloc(prio);
    if (pcb != NULL) {
        pcb->rcv_wnd = (tcpwnd_size_t)TUNED_TCP_WND;
        pcb->rcv_ann_wnd = (tcpwnd_size_t)TUNED_TCP_WND;
    }
    return pcb;
}

err_t __wrap_tcp_enqueue_flags(struct tcp_pcb *pcb, u8_t flags)
{
    if (pcb != NULL && (flags & TCP_SYN)) {
        pcb->rcv_wnd = (tcpwnd_size_t)TUNED_TCP_WND;
        pcb->rcv_ann_wnd = (tcpwnd_size_t)TUNED_TCP_WND;
    }
    return __real_tcp_enqueue_flags(pcb, flags);
}

void __wrap_tcp_recved(struct tcp_pcb *pcb, u16_t len)
{
    if (pcb == NULL || pcb->state == LISTEN || pcb->state == CLOSED) {
        return;
    }
    LWIP_ASSERT_CORE_LOCKED();

    u32_t rcv_wnd = (u32_t)pcb->rcv_wnd + (u32_t)len;
    if (rcv_wnd > TUNED_TCP_WND) {
        pcb->rcv_wnd = (tcpwnd_size_t)TUNED_TCP_WND;
    } else {
        pcb->rcv_wnd = (tcpwnd_size_t)rcv_wnd;
    }

    u32_t wnd_inflation = tcp_update_rcv_ann_wnd(pcb);
    if (wnd_inflation >= (u32_t)(4U * TCP_MSS)) {
        tcp_ack_now(pcb);
        tcp_output(pcb);
    }
}

err_t __wrap_tcp_close(struct tcp_pcb *pcb)
{
    if (pcb != NULL && pcb->state != LISTEN && pcb->rcv_wnd > 0) {
        pcb->rcv_wnd = (tcpwnd_size_t)STOCK_TCP_WND;
    }
    return __real_tcp_close(pcb);
}
