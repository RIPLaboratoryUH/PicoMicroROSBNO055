#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#include <uxr/client/profile/transport/custom/custom_transport.h>
#include <picow_udp_transports.h>

uint8_t trans_recv_buff[512] = { 0 };
uint16_t trans_recv_len = 0;

void usleep(uint64_t us)
{
    sleep_us(us);
}

int clock_gettime(clockid_t unused, struct timespec *tp)
{
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}

#define TRANS_RECV_RING_SIZE 16
#define TRANS_RECV_MAX_LEN   512

uint8_t trans_recv_ring[TRANS_RECV_RING_SIZE][TRANS_RECV_MAX_LEN] = {0};
uint16_t trans_recv_len_ring[TRANS_RECV_RING_SIZE] = {0};
volatile uint8_t trans_recv_head = 0;
volatile uint8_t trans_recv_tail = 0;

static void callback_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    ST_PICOW_TRANSPORT_PARAMS* params = (ST_PICOW_TRANSPORT_PARAMS*) arg;

    cyw43_arch_lwip_begin();
    if (params) {
        if (ip_addr_cmp(addr, &params->ipaddr)) {
            uint16_t len = pbuf_copy_partial(p, trans_recv_ring[trans_recv_head], TRANS_RECV_MAX_LEN, 0);
            trans_recv_len_ring[trans_recv_head] = len;

            trans_recv_head = (trans_recv_head + 1) % TRANS_RECV_RING_SIZE;
            if (trans_recv_head == trans_recv_tail) {
                // Overflow, drop oldest
                trans_recv_tail = (trans_recv_tail + 1) % TRANS_RECV_RING_SIZE;
                //printf("callback_recv: ring buffer overflow, dropping oldest message\n");
            }
        }
    }
    pbuf_free(p);
    cyw43_arch_lwip_end();
}


bool picow_udp_transport_open(struct uxrCustomTransport * transport)
{
    ST_PICOW_TRANSPORT_PARAMS* params = (ST_PICOW_TRANSPORT_PARAMS*) transport->args;

    if (params) {
        // cyw43_arch_lwip_begin/end should be used around calls into lwIP to ensure correct locking.
        // You can omit them if you are in a callback from lwIP. Note that when using pico_cyw_arch_poll
        // these calls are a no-op and can be omitted, but it is a good practice to use them in
        // case you switch the cyw43_arch type later.
        cyw43_arch_lwip_begin();
        params->pcb = udp_new();
        ipaddr_aton(ROS_AGENT_IP_ADDR, &(params->ipaddr));
        params->port = ROS_AGENT_UDP_PORT;

        udp_recv(params->pcb, callback_recv, params);
        cyw43_arch_lwip_end();

        //printf("picow_udp_transport_open: SUCCESS\n");
        return true;
    }
    else {
        //printf("picow_udp_transport_open: FAILURE\n");
        return false;
    }
}

bool picow_udp_transport_close(struct uxrCustomTransport * transport)
{
    ST_PICOW_TRANSPORT_PARAMS* params = (ST_PICOW_TRANSPORT_PARAMS*) transport->args;

    if (params) {
        // cyw43_arch_lwip_begin/end should be used around calls into lwIP to ensure correct locking.
        // You can omit them if you are in a callback from lwIP. Note that when using pico_cyw_arch_poll
        // these calls are a no-op and can be omitted, but it is a good practice to use them in
        // case you switch the cyw43_arch type later.
        cyw43_arch_lwip_begin();
        udp_remove(params->pcb);
        cyw43_arch_lwip_end();

        //printf("picow_udp_transport_close: \n");
    }
    return true;
}

size_t picow_udp_transport_write(struct uxrCustomTransport* transport,
                                 const uint8_t* buf,
                                 size_t len,
                                 uint8_t* err)
{
    ST_PICOW_TRANSPORT_PARAMS* params = (ST_PICOW_TRANSPORT_PARAMS*) transport->args;

    struct pbuf* p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
    if (p == NULL) {
        *err = 1;
        return 0;
    }

    // Copy the outgoing data into the pbuf
    memcpy(p->payload, buf, len);

    // Send packet via UDP
    err_t lwip_err = udp_sendto(params->pcb, p, &params->ipaddr, params->port);

    // Free the pbuf regardless of send success
    pbuf_free(p);

    if (lwip_err != ERR_OK) {
        *err = 1;
        return 0;
    }

    return len;
}


size_t picow_udp_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{
    size_t recv_len = 0;
    *errcode = 1;   // failure

#if PICO_CYW43_ARCH_POLL
    cyw43_arch_poll();
#endif

    if (trans_recv_tail != trans_recv_head) {
        cyw43_arch_lwip_begin();

        uint16_t avail_len = trans_recv_len_ring[trans_recv_tail];
        recv_len = (avail_len >= len) ? len : avail_len;
        memcpy(buf, trans_recv_ring[trans_recv_tail], recv_len);

        // Advance tail
        trans_recv_tail = (trans_recv_tail + 1) % TRANS_RECV_RING_SIZE;

        *errcode = 0;   // success
        cyw43_arch_lwip_end();
    }

    return recv_len;
}


