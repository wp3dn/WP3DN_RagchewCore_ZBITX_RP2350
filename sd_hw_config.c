/* Waveshare RP2350-PiZero onboard microSD, SPI mode.
   Verified from Waveshare schematic:
     SCK  = GP30
     MOSI = GP31 (SD CMD)
     MISO = GP40 (SD D0)
     CS   = GP43 (SD D3)
   These signals use the RP2350 SPI1 peripheral.  SPI mode avoids the
   pin-layout restrictions of the PIO SDIO implementation and is more
   than fast enough for settings/logging/FT8 storage. */
#include "hw_config.h"
#include "sd_card.h"
#include "hardware/spi.h"

static spi_t sd_spi = {
    .hw_inst = spi1,
    .miso_gpio = 40,
    .mosi_gpio = 31,
    .sck_gpio = 30,
    .baud_rate = 12500000u,
};

static sd_spi_if_t sd_spi_if = {
    .spi = &sd_spi,
    .ss_gpio = 43,
};

static sd_card_t sd_card = {
    .type = SD_IF_SPI,
    .spi_if_p = &sd_spi_if,
};

size_t sd_get_num(void){ return 1; }
sd_card_t *sd_get_by_num(size_t num){ return num==0 ? &sd_card : NULL; }
