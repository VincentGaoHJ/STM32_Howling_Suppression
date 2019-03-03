#include "bsp_FFT.h"
#include "table_fft.h"

extern uint16_t TableFFT[];

extern int32_t lBufInArray[NPT];
extern int32_t lBufOutArray[NPT];
extern float lBufMagArray[NPT];



/**
  * @name   GetPowerMag()
	* @brief  计算各次谐波幅值
  * @param  无
  * @retval 无
  * @attention
	*         先将lBufOutArray分解成实部(X)和虚部(Y)，然后计算幅值
	*/
	
	
void GetPowerMag(void)
{
   uint16_t i;
	 int16_t lX, lY;
	 float X, Y, Mag;
	
	 for(i=0; i<NPT/2; i++)
	 {
      lX = (lBufOutArray[i] << 16) >> 16;      //取低16位虚部
		  lY = lBufOutArray[i] >> 16;              //取高16位实部
		  X = NPT * ((float)lX) / 32768;
		  Y = NPT * ((float)lY) / 32768;
		  Mag = sqrt(X * X + Y * Y) / NPT;         
		  
		 if(i == 0)
				 lBufMagArray[i] = (Mag * 32768);      //直流分量不需要乘2
			else
				 lBufMagArray[i] = (Mag * 65536);
   }
}
