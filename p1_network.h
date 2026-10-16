#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool initialized;
    bool link_up;
    bool discovered;
    bool control_seen;
    bool mox_requested;
    uint32_t rx_frequency_hz;
    uint32_t tx_frequency_hz;
    uint32_t discovery_count;
    uint32_t control_count;
    bool rx_streaming;
    uint32_t rx_packets_sent;
    uint32_t rx_packets_dropped;
    uint32_t tx_iq_frames_received;
    uint32_t tx_iq_frames_dropped;
    uint32_t tx_iq_underflows;
} p1_network_status_t;

bool p1_network_start(void);
void p1_network_stop(void);
void p1_network_task(void);
/* IRQ-safe, non-blocking. Queues one 126-frame EP6 IQ block. */
bool p1_network_queue_rx_iq(const int32_t *i24, const int32_t *q24);
/* Foreground-only TX IQ FIFO. Protocol-1 EP2 supplies 48 kHz interleaved I/Q. */
int p1_network_read_tx_iq(int16_t *iq_interleaved, int max_frames);
int p1_network_tx_iq_available(void);
void p1_network_flush_tx_iq(void);
void p1_network_note_tx_underflow(void);
const p1_network_status_t *p1_network_status(void);
