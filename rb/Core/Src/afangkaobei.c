/*
 * afangkaobei.c
 *
 *  Created on: 2021年7月5日
 *      Author: Administrator
 */

#include "stm32f1xx_hal.h"
#include "afangkaobei.h"


const uint8_t zzzz[20] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xee,0xee,0xee,0xee};

uint32_t ChipUniqueID[9];
uint32_t ChipUniqueID2[9];
uint32_t JIAMIDIZHI1 =0x0800aa40;
uint32_t JIAMIDIZHI2 =0x0800ab40;
uint32_t JIAMIDIZHI3 =0x0800ac40;
uint32_t ASD =0x0800ac44;
uint32_t id1 =0x0eeee6d7;// 0X1FFFF7E8
uint32_t id2 =0x11111111;

uint32_t jiamisuanfa(uint32_t a)
{
	uint32_t b;
	b = (((((((((a+1) / 2) + 186) * 3) - 1111) / 2) + 18743) * 3) -19862);
	return b;
}
uint32_t fangkaobei(uint32_t a)
{
	JIAMIDIZHI1 =(uint32_t) &zzzz[0];
	JIAMIDIZHI2 = JIAMIDIZHI1 + 4;
	JIAMIDIZHI3 = JIAMIDIZHI2 + 4;
	ASD = (uint32_t) &zzzz[16];

	ChipUniqueID[0] = *(__IO uint32_t*)(id1 + id2);//?低字节
	ChipUniqueID[1] = (*(__IO uint32_t *)(id1 + id2 + 4));//?
	ChipUniqueID[2] = *(__IO uint32_t *)(id1 + id2 + 8); //?高字节

	ChipUniqueID[3]  = *(__IO uint32_t *)(JIAMIDIZHI1);
	ChipUniqueID[4]  = *(__IO uint32_t *)(JIAMIDIZHI2);
	ChipUniqueID[5]  = *(__IO uint32_t *)(JIAMIDIZHI3);

	ChipUniqueID2[0] = *(__IO uint32_t *)(ASD);

	ChipUniqueID[6] =  jiamisuanfa(ChipUniqueID[0]);
	ChipUniqueID[7] =  jiamisuanfa(ChipUniqueID[1]);
	ChipUniqueID[8] =  jiamisuanfa(ChipUniqueID[2]);

	if(ChipUniqueID2[0] == 0XEEEEEEEE && ChipUniqueID[3] == 0xffffffff && ChipUniqueID[4] == 0xffffffff && ChipUniqueID[5] == 0xffffffff)//如果是第一次运行 -20210703
	{
		HAL_Delay(10);
		//xianshishuzi(0,1,1);
		HAL_Delay(100);
		//xianshishuzi(3,3,JIAMIDIZHI1);

		HAL_FLASH_Unlock();
		HAL_Delay(100);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, JIAMIDIZHI1, ChipUniqueID[6]);
		HAL_Delay(100);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, JIAMIDIZHI2, ChipUniqueID[7]);
		HAL_Delay(100);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, JIAMIDIZHI3, ChipUniqueID[8]);
		HAL_Delay(100);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ASD, 0);
		HAL_Delay(100);
		HAL_FLASH_Lock();

		//xianshishuzi(0,1,2);

		return 2;

	}else
	{
		HAL_Delay(10);

		if(ChipUniqueID2[0] == 0 && ChipUniqueID[3] == ChipUniqueID[6] && ChipUniqueID[4] == ChipUniqueID[7] && ChipUniqueID[5] == ChipUniqueID[8])
		{
			//xianshishuzi(3,2,1234);

			return 0;
		}else
		{

			//xianshishuzi(3,2,5678);
			return 1;
		}

	}


	return 0;
}

