# Phase 6.5 radio I/O

Fixed Waveshare RP2350 Pi Zero / ZBITX mapping:

- GP14: PTT input, active low, internal pull-up
- GP15: DASH input, active low, internal pull-up
- GP5: RXLINE output (HIGH in RX)
- GP23: TXLINE output (HIGH in TX)
- GP24: LPF A
- GP25: LPF B
- GP8: LPF C
- GP7: LPF D

Safe switching sequence:

- RX: all LPFs LOW, TXLINE LOW, then RXLINE HIGH
- TX: RXLINE LOW, short break delay, TXLINE HIGH, selected LPF HIGH

LPFs are never energized during receive. Automatic thresholds are the proven RadioCore values:
D below 5.5 MHz, C below 10.5 MHz, B below 18.5 MHz, A below 30 MHz.
