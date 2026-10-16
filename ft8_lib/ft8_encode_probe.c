/* Host-side encoder compatibility probe; never keys transmitter. */
#include <stdio.h>
#include <stdint.h>
#include "ft8/pack.h"
#include "ft8/encode.h"
#include "ft8/constants.h"
int main(void) {
 const char *msgs[]={"CQ WP3DN FK68","WP3DN K1ABC -10","K1ABC WP3DN R-08","K1ABC WP3DN RR73","K1ABC WP3DN 73","CQ WP3DN/P FK68",NULL};
 for(int i=0;msgs[i];i++){
  uint8_t packed[10]={0},tones[FT8_NN]={0};
  int rc=pack77(msgs[i],packed);
  printf("%-26s pack77=%d",msgs[i],rc);
  if(rc>=0){ft8_encode(packed,tones);printf(" tones=");for(int j=0;j<FT8_NN;j++)putchar('0'+tones[j]);}
  putchar('\n');
 }
 return 0;
}
