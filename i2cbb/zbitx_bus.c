#include "i2cbb/zbitx_bus.h"
static i2cbb_t bus;
static bool initialized=false;
i2cbb_t *zbitx_bus(void){
    if(!initialized){ i2cbb_init(&bus,ZBITX_BB_SDA,ZBITX_BB_SCL,2); initialized=true; }
    return &bus;
}
