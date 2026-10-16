cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(radiocore_v08_oled C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

# N6 isolated performance test: optimize entire firmware target.
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)

option(RAGCHEW_ENABLE_SD "Use Waveshare RP2350-PiZero onboard microSD for settings" ON)
if(RAGCHEW_ENABLE_SD)
    include(FetchContent)
    FetchContent_Declare(ragchew_fatfs
        GIT_REPOSITORY https://github.com/inindev/pico-fatfs-sd.git
        GIT_TAG main
        GIT_SHALLOW TRUE)
    FetchContent_GetProperties(ragchew_fatfs)
    if(NOT ragchew_fatfs_POPULATED)
        FetchContent_Populate(ragchew_fatfs)
        add_subdirectory(${ragchew_fatfs_SOURCE_DIR}/src ${CMAKE_BINARY_DIR}/ragchew_fatfs)
    endif()
endif()
add_executable(radiocore_v08_oled
    main.c
    ft8_n6.c
    ft8_tx6.c
    ft8_lib/ft8/pack.c
    ft8_lib/ft8/encode.c
    rtc_ds3231.c
    ft8_lib/ft8/constants.c
    ft8_lib/ft8/crc.c
    ft8_lib/ft8/decode.c
    ft8_lib/ft8/ldpc.c
    ft8_lib/ft8/text.c
    ft8_lib/ft8/unpack.c
    fft_pico/kiss_fft.c
    fft_pico/kiss_fftr.c
    audio/wm8731_codec.c
    audio/rx_agc.c
    audio/ssb_tx.c
    audio/fft_ssb_tx.c
    audio/fft_ssb_rx.c
    audio/audio_filter.c
    audio/spectrum.c
    audio/cw_decoder.c
    p1_dsp.c
    p1_protocol.c
    p1_network.c
    usb_network.c
    usb_descriptors.c
    dhcpserver/dhcpserver.c
    ${PICO_TINYUSB_PATH}/lib/networking/rndis_reports.c
    radio/si5351_radio.c
    radio/power_calibration.c
    hardware/radio_io.c
    ui/front_panel.c
    i2cbb/i2cbb_pico.c
    i2cbb/zbitx_bus.c
)
if(RAGCHEW_ENABLE_SD)
    target_sources(radiocore_v08_oled PRIVATE settings_sd.c sd_hw_config.c)
else()
    target_sources(radiocore_v08_oled PRIVATE settings_flash.c)
endif()
target_compile_definitions(radiocore_v08_oled PRIVATE RAGCHEW_SD_DIAG=1)
# Core 1 uses ft8_core1_stack in main RAM. Give Core 0 both contiguous
# RP2350 scratch banks so FatFs and the audio IRQ cannot overflow into BSS.
if(PICO_PLATFORM STREQUAL "rp2350-arm-s")
    target_compile_definitions(radiocore_v08_oled PRIVATE
        PICO_STACK_SIZE=8192 PICO_CORE1_STACK_SIZE=0)
    file(READ "${PICO_SDK_PATH}/src/rp2_common/pico_crt0/rp2350/memmap_default.ld" radio_memmap)
    string(REPLACE "SCRATCH_X(rwx) : ORIGIN = 0x20080000, LENGTH = 4k"
                   "SCRATCH_X(rwx) : ORIGIN = 0x20080000, LENGTH = 0k" radio_memmap "${radio_memmap}")
    string(REPLACE "SCRATCH_Y(rwx) : ORIGIN = 0x20081000, LENGTH = 4k"
                   "SCRATCH_Y(rwx) : ORIGIN = 0x20080000, LENGTH = 8k" radio_memmap "${radio_memmap}")
    file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/radiocore_memmap.ld" "${radio_memmap}")
    pico_set_linker_script(radiocore_v08_oled "${CMAKE_CURRENT_BINARY_DIR}/radiocore_memmap.ld")
endif()
target_compile_options(radiocore_v08_oled PRIVATE -O3)
target_include_directories(radiocore_v08_oled PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${PICO_TINYUSB_PATH}/lib/networking)
pico_generate_pio_header(radiocore_v08_oled ${CMAKE_CURRENT_LIST_DIR}/wm8731_bidi_slave.pio)
pico_generate_pio_header(radiocore_v08_oled ${CMAKE_CURRENT_LIST_DIR}/wm8731_adc_rx.pio)
pico_generate_pio_header(radiocore_v08_oled ${CMAKE_CURRENT_LIST_DIR}/i2cbb/i2c_pio_edges.pio)
target_link_libraries(radiocore_v08_oled
    pico_stdlib pico_multicore
    tinyusb_device
    pico_lwip
    pico_lwip_nosys
    pico_unique_id
    hardware_i2c hardware_uart hardware_pio hardware_dma hardware_irq hardware_clocks hardware_vreg hardware_flash hardware_sync m)
if(RAGCHEW_ENABLE_SD)
    target_link_libraries(radiocore_v08_oled pico-fatfs-sd)
endif()
pico_enable_stdio_usb(radiocore_v08_oled 0)
pico_enable_stdio_uart(radiocore_v08_oled 0)
pico_add_extra_outputs(radiocore_v08_oled)
