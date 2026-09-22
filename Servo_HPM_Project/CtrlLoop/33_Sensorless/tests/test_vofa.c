#include <stdio.h>
#include <math.h>
#include "SensorlessVofa.h"
static int failures;
#define CHECK(x) do { if(!(x)) {printf("FAIL line %d\n",__LINE__);failures++;}}while(0)
static uint32_t bits(const uint8_t *p) {
 return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);
}
static float value(const uint8_t *p) {uint32_t u=bits(p);float f;memcpy(&f,&u,4);return f;}
int main(void) {
 uint8_t p[34]; memset(p,0xa5,sizeof(p));
 SlVofa_Pack(p+1,65536,131072,1,0,65537);
 CHECK(p[0]==0xa5 && p[33]==0xa5);
 CHECK(value(p+1)==65536 && value(p+5)==131072);
 CHECK(value(p+9)==180 && value(p+13)==90);
 CHECK(value(p+17)==1 && value(p+21)==0 && value(p+25)==1);
 CHECK(bits(p+29)==0x7f800000UL);
 SlVofa_Pack(p+1,65536,131072,0,1,2);
 for(unsigned i=0;i<4;i++) CHECK(bits(p+1+4*i)==0x7fc00000UL);
 CHECK(value(p+17)==0 && value(p+21)==1);
 SlVofa_Pack(p+1,131071,524287,1,1,0);
 CHECK(value(p+1)==131071 && value(p+5)==524287);
 CHECK(value(p+9)<360 && value(p+13)<360);
 printf("VOFA binary packet: %d failures\n",failures);
 return failures?1:0;
}
