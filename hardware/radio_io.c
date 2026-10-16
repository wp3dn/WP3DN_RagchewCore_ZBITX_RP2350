#include "hardware/radio_io.h"

#include <stdio.h>

#include "pico/stdlib.h"

// Verified Waveshare RP2350 Pi Zero / ZBITX mapping.
#define PIN_RX_LINE  5
#define PIN_LPF_D    7
#define PIN_LPF_C    8
#define PIN_PTT     14
#define PIN_DASH    15
#define PIN_TX_LINE 23
#define PIN_LPF_A   24
#define PIN_LPF_B   25

#define INPUT_DEBOUNCE_MS 5u
#define SSB_TX_RELEASE_ZERO_US 2000u

static radio_io_state_t state_;
static bool raw_ptt_;
static bool raw_dash_;
static absolute_time_t ptt_changed_at_;
static absolute_time_t dash_changed_at_;
static bool manual_tx_override_;
static bool manual_tx_state_;
static bool tx_release_pending_;
static absolute_time_t tx_release_deadline_;

static void all_lpf_low(void) {
    gpio_put(PIN_LPF_A, 0);
    gpio_put(PIN_LPF_B, 0);
    gpio_put(PIN_LPF_C, 0);
    gpio_put(PIN_LPF_D, 0);
}

static void apply_selected_lpf(void) {
    all_lpf_low();
    if (!state_.tx) return;

    switch (state_.lpf) {
        case RADIO_LPF_A: gpio_put(PIN_LPF_A, 1); break;
        case RADIO_LPF_B: gpio_put(PIN_LPF_B, 1); break;
        case RADIO_LPF_C: gpio_put(PIN_LPF_C, 1); break;
        case RADIO_LPF_D: gpio_put(PIN_LPF_D, 1); break;
        default: break;
    }
}

const char *radio_io_lpf_name(radio_lpf_t lpf) {
    switch (lpf) {
        case RADIO_LPF_A: return "A";
        case RADIO_LPF_B: return "B";
        case RADIO_LPF_C: return "C";
        case RADIO_LPF_D: return "D";
        default: return "OFF";
    }
}

radio_lpf_t radio_io_lpf_for_frequency(uint32_t frequency_hz) {
    if (frequency_hz < 5500000u) return RADIO_LPF_D;
    if (frequency_hz < 10500000u) return RADIO_LPF_C;
    if (frequency_hz < 18500000u) return RADIO_LPF_B;
    if (frequency_hz < 30000000u) return RADIO_LPF_A;
    return RADIO_LPF_OFF;
}

void radio_io_enter_receive(void) {
    // Break TX first, then enable the receiver.
    all_lpf_low();
    gpio_put(PIN_TX_LINE, 0);
    sleep_us(50);
    gpio_put(PIN_RX_LINE, 1);
    state_.tx = false;
}

void radio_io_enter_transmit(void) {
    // Disable receiver first, then key TX and energize only the selected LPF.
    gpio_put(PIN_RX_LINE, 0);
    sleep_us(50);
    gpio_put(PIN_TX_LINE, 1);
    state_.tx = true;
    apply_selected_lpf();
}

static void update_tx_state(void) {
    bool requested_tx = manual_tx_override_ ? manual_tx_state_ : state_.ptt;

    if (requested_tx) {
        tx_release_pending_ = false;
        if (!state_.tx) radio_io_enter_transmit();
        return;
    }

    if (!state_.tx) {
        tx_release_pending_ = false;
        return;
    }

    /* Manual TX users (CW hang/keyer and Protocol 1 MOX) keep the proven
       immediate transition.  Only local physical-PTT release gets a tiny
       zero-audio guard interval so the DAC is at zero before RF switching. */
    if (manual_tx_override_) {
        tx_release_pending_ = false;
        radio_io_enter_receive();
        return;
    }

    if (!tx_release_pending_) {
        tx_release_pending_ = true;
        tx_release_deadline_ = make_timeout_time_us(SSB_TX_RELEASE_ZERO_US);
        return;
    }

    if (absolute_time_diff_us(get_absolute_time(), tx_release_deadline_) <= 0) {
        tx_release_pending_ = false;
        radio_io_enter_receive();
    }
}

void radio_io_init(uint32_t frequency_hz) {
    const uint outputs[] = {PIN_RX_LINE, PIN_TX_LINE, PIN_LPF_A, PIN_LPF_B, PIN_LPF_C, PIN_LPF_D};
    for (uint i = 0; i < sizeof(outputs) / sizeof(outputs[0]); ++i) {
        gpio_init(outputs[i]);
        gpio_set_dir(outputs[i], GPIO_OUT);
        gpio_put(outputs[i], 0);
    }

    gpio_init(PIN_PTT);
    gpio_set_dir(PIN_PTT, GPIO_IN);
    gpio_pull_up(PIN_PTT);
    gpio_init(PIN_DASH);
    gpio_set_dir(PIN_DASH, GPIO_IN);
    gpio_pull_up(PIN_DASH);

    state_.frequency_hz = frequency_hz;
    state_.lpf_mode = RADIO_LPF_MODE_AUTO;
    state_.lpf = radio_io_lpf_for_frequency(frequency_hz);
    state_.ptt = !gpio_get(PIN_PTT);
    state_.dash = !gpio_get(PIN_DASH);
    raw_ptt_ = state_.ptt;
    raw_dash_ = state_.dash;
    ptt_changed_at_ = get_absolute_time();
    dash_changed_at_ = get_absolute_time();
    manual_tx_override_ = false;
    manual_tx_state_ = false;
    tx_release_pending_ = false;
    tx_release_deadline_ = get_absolute_time();

    // Always boot safely in receive, regardless of input state during startup.
    radio_io_enter_receive();
}

void radio_io_poll(void) {
    bool new_raw_ptt = !gpio_get(PIN_PTT);
    bool new_raw_dash = !gpio_get(PIN_DASH);
    absolute_time_t now = get_absolute_time();

    if (new_raw_ptt != raw_ptt_) {
        raw_ptt_ = new_raw_ptt;
        ptt_changed_at_ = now;
    } else if (state_.ptt != raw_ptt_ && absolute_time_diff_us(ptt_changed_at_, now) >= (int64_t)INPUT_DEBOUNCE_MS * 1000) {
        state_.ptt = raw_ptt_;
    }

    if (new_raw_dash != raw_dash_) {
        raw_dash_ = new_raw_dash;
        dash_changed_at_ = now;
    } else if (state_.dash != raw_dash_ && absolute_time_diff_us(dash_changed_at_, now) >= (int64_t)INPUT_DEBOUNCE_MS * 1000) {
        state_.dash = raw_dash_;
    }

    update_tx_state();
}

bool radio_io_ptt(void) { return state_.ptt; }
bool radio_io_dash(void) { return state_.dash; }
bool radio_io_is_tx(void) { return state_.tx; }

void radio_io_set_manual_tx(bool tx) {
    manual_tx_override_ = true;
    manual_tx_state_ = tx;
    update_tx_state();
}

void radio_io_clear_manual_tx(void) {
    manual_tx_override_ = false;
    update_tx_state();
}

bool radio_io_manual_tx_active(void) { return manual_tx_override_; }

void radio_io_set_frequency(uint32_t frequency_hz) {
    state_.frequency_hz = frequency_hz;
    if (state_.lpf_mode == RADIO_LPF_MODE_AUTO) {
        const radio_lpf_t new_lpf = radio_io_lpf_for_frequency(frequency_hz);

        // Do not rewrite the LPF GPIOs on every tuning step.  Only touch the
        // hardware when the selected LPF actually changes.
        if (new_lpf != state_.lpf) {
            state_.lpf = new_lpf;
            apply_selected_lpf();
        }
    }
}

void radio_io_set_lpf_auto(void) {
    state_.lpf_mode = RADIO_LPF_MODE_AUTO;
    state_.lpf = radio_io_lpf_for_frequency(state_.frequency_hz);
    apply_selected_lpf();
}

void radio_io_set_lpf_manual(radio_lpf_t lpf) {
    state_.lpf_mode = RADIO_LPF_MODE_MANUAL;
    state_.lpf = lpf;
    apply_selected_lpf();
}

radio_lpf_t radio_io_get_lpf(void) { return state_.lpf; }
radio_lpf_mode_t radio_io_get_lpf_mode(void) { return state_.lpf_mode; }
const radio_io_state_t *radio_io_get_state(void) { return &state_; }

void radio_io_print_status(void) {
    printf("Radio I/O:\n");
    printf("  PTT GPIO14  = %s\n", state_.ptt ? "DOWN" : "UP");
    printf("  DASH GPIO15 = %s\n", state_.dash ? "DOWN" : "UP");
    printf("  State       = %s%s\n", state_.tx ? "TX" : "RX", manual_tx_override_ ? " (manual override)" : "");
    printf("  RXLINE GP5  = %s\n", gpio_get(PIN_RX_LINE) ? "HIGH" : "LOW");
    printf("  TXLINE GP23 = %s\n", gpio_get(PIN_TX_LINE) ? "HIGH" : "LOW");
    printf("  LPF         = %s (%s)\n", radio_io_lpf_name(state_.lpf),
           state_.lpf_mode == RADIO_LPF_MODE_AUTO ? "AUTO" : "MANUAL");
    printf("  LPF pins    = A:%d B:%d C:%d D:%d\n",
           gpio_get(PIN_LPF_A), gpio_get(PIN_LPF_B), gpio_get(PIN_LPF_C), gpio_get(PIN_LPF_D));
}
