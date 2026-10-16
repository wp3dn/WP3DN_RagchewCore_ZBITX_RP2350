#include "rtc_ds3231.h"
#include "i2cbb/zbitx_bus.h"
#include <stdint.h>

static int bcd(uint8_t b){return 10*(b>>4)+(b&15);}
static uint8_t enc(int v){return (uint8_t)(((v/10)<<4)|(v%10));}
/* Gregorian civil-date conversion, UTC, no timezone state. */
static int64_t days_from_civil(int y,unsigned m,unsigned d){
 y-=m<=2; int era=(y>=0?y:y-399)/400; unsigned yoe=(unsigned)(y-era*400);
 unsigned doy=(153*(m+(m>2?-3:9))+2)/5+d-1;
 unsigned ydoe=yoe*365+yoe/4-yoe/100+doy; return (int64_t)era*146097+(int64_t)ydoe-719468;
}
static void civil_from_days(int64_t z,int *y,unsigned *m,unsigned *d){
 z+=719468; int64_t era=(z>=0?z:z-146096)/146097; unsigned doe=(unsigned)(z-era*146097);
 unsigned yoe=(doe-doe/1460+doe/36524-doe/146096)/365; int yy=(int)yoe+(int)era*400;
 unsigned doy=doe-(365*yoe+yoe/4-yoe/100); unsigned mp=(5*doy+2)/153;
 *d=doy-(153*mp+2)/5+1; *m=mp+(mp<10?3:-9); *y=yy+(*m<=2);
}
bool rtc_ds3231_read(int64_t *epoch_seconds,bool *oscillator_stopped){
 i2cbb_t *bus=zbitx_bus(); uint8_t reg=0, t[7], status_reg=0x0f, status=0;
 if(!bus || !i2cbb_write_read(bus,0x68,&reg,1,t,7) ||
    !i2cbb_write_read(bus,0x68,&status_reg,1,&status,1))return false;
 if(oscillator_stopped)*oscillator_stopped=(status&0x80)!=0;
 if((t[0]&0x70)>0x50 || (t[1]&0x70)>0x50 || (t[2]&0x40))return false;
 int sec=bcd(t[0]&0x7f),min=bcd(t[1]&0x7f),hour=bcd(t[2]&0x3f);
 int day=bcd(t[4]&0x3f),mon=bcd(t[5]&0x1f),year=2000+bcd(t[6]) + ((t[5]&0x80)?100:0);
 if(sec>59||min>59||hour>23||day<1||day>31||mon<1||mon>12||year<2020||year>2099)return false;
 if(epoch_seconds)*epoch_seconds=days_from_civil(year,(unsigned)mon,(unsigned)day)*86400+hour*3600+min*60+sec;
 return true;
}
bool rtc_ds3231_write(int64_t epoch_seconds){
 if(epoch_seconds<1577836800LL || epoch_seconds>=4102444800LL)return false;
 int64_t days=epoch_seconds/86400;int rem=(int)(epoch_seconds%86400);int y;unsigned m,d;
 civil_from_days(days,&y,&m,&d); if(y<2000||y>2099)return false;
 uint8_t data[8]={0,enc(rem%60),enc((rem/60)%60),enc(rem/3600),1,enc((int)d),enc((int)m),enc(y-2000)};
 i2cbb_t *bus=zbitx_bus();if(!bus||!i2cbb_write(bus,0x68,data,8))return false;
 /* Clear oscillator-stop flag only after successful time programming. */
 uint8_t reg=0x0f,status=0;if(!i2cbb_write_read(bus,0x68,&reg,1,&status,1))return false;
 uint8_t clear[2]={0x0f,(uint8_t)(status&~0x80)};return i2cbb_write(bus,0x68,clear,2);
}
