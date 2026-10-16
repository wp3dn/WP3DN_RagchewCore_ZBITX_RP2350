# RagchewCore 📻
### A tiny RP2350. A complete HF transceiver. The original ZBITX panel.

RagchewCore is bare-metal firmware for the **Waveshare RP2350 Pi Zero**, replacing the ZBITX pi02w radio controller while **retaining the original drexjj 5.13f ZBITX front panel**. It runs at **240 MHz**, with local DSP and digital modes—no Linux, external PC, or panel modification required for normal operation.

> **Hardware target:** Stock ZBITX radio board + Waveshare RP2350 Pi Zero, sdcard. 

## Features

- **HF voice and CW:** LSB, USB, AM, CWL and CWU; DSP filtering, AGC and noise reduction.
- **Digital modes:** Onboard FT8, including **touch-to-reply** on the original panel.
- **Display:** Stock ZBITX frequency controls, spectrum/waterfall and CW decoder.
- **Logging and storage:** SD-card settings, FT8 ADIF logging and logbook viewer.
- **USB connectivity:** CDC serial console in LOCAL mode; selectable **HPSDR Protocol 1** over USB for external SDR software.
- **Transmit calibration:** Independent **TXCAL** ceiling for nine amateur bands, shared by local and Protocol 1 transmit paths.

## RP2350 GPIO pinout

GPIO numbers refer to **RP2350 pins**, not physical header numbers.

| GPIO | Connection | GPIO | Connection |
|:---:|---|:---:|---|
| **2** | I²C SDA — Si5351, WM8731, RTC | **3** | I²C SCL |
| **5** | RXLINE | **23** | TXLINE |
| **6** | Panel I²CBB SCL | **13** | Panel I²CBB SDA |
| **7** | LPF D | **8** | LPF C |
| **24** | LPF A | **25** | LPF B |
| **14** | PTT / CW DIT | **15** | CW DAH |
| **18** | WM8731 I²S BCLK | **19** | WM8731 I²S LRCLK |
| **20** | WM8731 ADC → RP2350 | **21** | RP2350 → WM8731 DAC |
| **30** | Onboard SD SCK | **31** | Onboard SD MOSI |
| **40** | Onboard SD MISO | **43** | Onboard SD CS |

**ZBITX RF switching:** RXLINE is high during receive; TXLINE is high during transmit. LPF outputs are selected during transmit. Do not reuse this switching logic unchanged on an sBITX board.

## USB console commands

Connect to the USB CDC serial port in **LOCAL** mode (typically `/dev/ttyACM0` on Linux, `COMx` on Windows). Enter `help` for the complete built-in command listing.

| Group | Commands | Function |
|---|---|---|
| **General** | `help`, `?`, `status`, `s`, `save` | Help, status, save settings |
| **Tuning** | `freq`, `mode`, `band`, `vfo`, `vfoa`, `vfob`, `step`, `rit` | Frequency, modes and VFO controls |
| **RX / DSP** | `rxlow`, `rxhigh`, `amrx`, `agc`, `agctarget`, `agcmax`, `dsp`, `ifgain`, `volume`, `spectrum` | Receive filters, AGC, audio and display |
| **TX / audio** | `txlow`, `txhigh`, `micgain`, `txdrive`, `txif`, `amcarrier`, `rxcodecin`, `miccodecin`, `earcodecout`, `modcodecout` | Transmit filtering and levels |
| **CW** | `keyer`, `cwwpm`, `cwoffset`, `cwdecodepitch`, `cwdelay`, `sidetone` | Keyer and CW decoder |
| **Oscillator** | `bfo`, `bfodelta`, `globalif`, `clk1drive`, `clk2drive` | IF and Si5351 adjustments |
| **Clock** | `TIME`, `TIME?`, `RTC?` | Set/check UTC and RTC |
| **FT8** | `ft8tx`, `ft8abort`, `ft8first` | FT8 controls |
| **Diagnostics** | `tx`, `txzero`, `lpf` | Manual TX and LPF overrides — **use caution** |
| **Programming** | `bootsel`, `program` | Enter programming mode |

### TXCAL — nine-band transmit calibration

```text
TXCAL SHOW         Show runtime and saved values
TXCAL 20 35        Set 20m TX ceiling to 35%
TXCAL 40 25        Set 40m TX ceiling to 25%
TXCAL SAVE         Save TXCAL values to SD
TXCAL RESET        Restore runtime defaults (10% per band)
```

**Defaults are 10% on all nine bands.** Existing saved calibration values take precedence. `TXCAL RESET` affects runtime values; use `TXCAL SAVE` to make them persistent.

### Clock example

```text
TIME?              Show current time
RTC?               Read RTC
TIME 1792100000    Set UTC using Unix seconds (example)
```

## Operating notes

- The original panel's **USB MODE** control switches between local radio operation and USB Protocol 1.
- **FT8 touch-to-reply:** With `1-TAP` enabled on the stock panel, touch a decoded CQ; otherwise select it and press Enter.

---
References & Credits
Ashhar Farhan — HF Signals: Creator of the sBITX/ZBITX radio hardware. hfsignals.com (https://www.hfsignals.com/)
drexjj — ZBITX Front Panel: Original front-panel software and protocol reference. GitHub (https://github.com/drexjj/zbitxfrontpanel)
kgoba — ft8_lib: Open-source FT8 encoding and decoding library. GitHub (https://github.com/kgoba/ft8_lib)
WP3DN & BUG: RagchewCore development, integration, and testing.
73 de WP3DN & BUG
*Built in the spirit of amateur radio experimentation. 73 de WP3DN & KP4BUG — “One RadioCore to rule them all!”* 💍
