
#ifndef __FFT_H
#define __FFT_H

#include "stm32f10x.h"
#include <math.h>

#define PI2    6.28318530717959
#define Fs     40000       

//#define NPT_64     64
//  #define NPT_256    256
  #define NPT_1024   1024

#ifdef NPT_64
#define NPT    64
#endif

#ifdef NPT_256
#define NPT    256
#endif

#ifdef NPT_1024
#define NPT    1024
#endif

void InitBufInArray(void);
void cr4_fft_test(void);
void GetPowerMag(void);

#endif /* __FFT_H */
