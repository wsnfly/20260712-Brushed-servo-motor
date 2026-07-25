/*
 * abujindianjipwm.c
 *
 *  Created on: 2023年3月23日
 *      Author: HuaWei
 *
 *      使用方法：
 *      设置一个定时器的pwm输出
 *      一般定时器频率为32000000赫兹
 *      在下面宏定义中设置陪我们宽度 看情况设置
 *
 *      需要使用时开启定时器中断 HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
 		HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
		在中断中调用pwmzhongduankongzhi()函数
 *      然后直接写入预定位置，电机会直接加速启动，可以在任意时刻设置任意位置，
 *      电机会自动控制加减速并达到目标位置
 *      不需要使用时可以关闭定时器节省CPU，HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
 *      每次开始使用时最好根据自己电机的情况刷新一遍速度控制表格，jiasuleibiaojisuan(32000000,500,10000,31004);
 *      然后如果对电机转速有需要，可以限制最大速度，其中S2B函数是将速度转化成列表中对应的值
 *      bujindianji[0].zuidasudu = s2b(32000000,20000);//设置最大速度
 *      最后直接将目标位置写入就ok
  	  	bujindianji[0].yudingweizhi = 200000;//设置位置

  	  	改方法最高频率大概为200khz，正常使用时不建议将频率超过40khz
  	  	主要时加速表使用了ram，而ram的内存有限，如果使用动态加速表，只能有限的资源，当加速度不是很大时ram会不够用
  	  	所以无法达到非常高的速度
  	  	解决方法是可以使用一张rom的表来存放加减速表格。
  	  	如果对位置精度要求不高的场合，可以使用动态速度控制。实时计算当前需要的速度。总之40k的速度对于一般场合够用了。

寄存器说明：
地址0： 定时器最大频率，默认32000000，不需要改变。
地址1：输出脉冲的最小频率，默认500步每秒，不需要改变。如果太小会导致定时器溢出。
地址2：输出脉冲的最大频率，默认32K步每秒，F4的芯片一般不得大于96K，否则计数器来不及导致脉冲数量错误。
地址3：电机加速度，默认64K步每秒，太大导致电机跟步上，ram太小导致达不到所需的速度。（如果使用主循环速度计算器无需考虑ram太小的问题
地址4：电机绝对位置，向该地址写入数据，电机会达到该指定位置
地址5：写入位置之后是否立即启动电机，0立即启动。1等待该位置2时启动。当该位为1时，写入地址4之后电机不会启动，要再向此位置写入2才会启动，然后该位会自动置1。
地址6：当写入地址4或者5之后，是否返回数据，0表示返回，1表示不返回


jiasuleibiaojisuan(32000000,500,100000,3004);//先计算一遍加速表
//
    bujindianji[0].zuidasudu = 2000;//设置�?????大�?�度
    chuankoufasong(219,bujindianji[0].zuidasudu);
    bujindianji[0].yudingweizhi = 200000;//设置位置


-----------------------------------------------------------------------
//外部计算调用此函数
void shishisuduyunsuan(void);
-------------------------------------------------------------------------
void EXTI9_5_IRQHandler(voids)
{
	if(bujindianji[0].shijifangxiang == 1)
	{
		bujindianji[0].jiluweizhi  ++;
	}
	if(bujindianji[0].shijifangxiang == 0)
	{
		bujindianji[0].jiluweizhi  --;
	}
	if(bujindianji[0].dangqianweizhi + 5 < bujindianji[0].jiluweizhi)bujindianji[0].dangqianweizhi = bujindianji[0].jiluweizhi;
	if(bujindianji[0].dangqianweizhi - 5 > bujindianji[0].jiluweizhi)bujindianji[0].dangqianweizhi = bujindianji[0].jiluweizhi;
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
}


  HAL_TIM_Base_Start_IT(&htim1);
  //HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim3);
  bujindianji[0].zuidasudu = s2b2(500,10000 ,6400);
  while (1)
  {
	  bujindianji[0].yudingweizhi = bujindianji[0].yudingweizhi + 3200;
	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  HAL_Delay(1000);
	  
 */
#include "main.h"
#include "math.h"
#include "stm32f1xx_hal.h"
#include "ajiajiansu.h"
#include "apwmliebiao.h"


#define DINGSHIQICHONGZHI TIM2->ARR//定时器重置值

extern TIM_HandleTypeDef htim2;

//uint16_t jiasuliebiao[3006];
bujinpwm bujindianji[4];
uint16_t jjj[10];
uint16_t dingshi;

/* 加速表指针(新增): 默认指向ROM表, AccTable_Calculate后指向Flash自定义表 */
const uint16_t *p_jiasuliebiao = jiasuliebiao;

/* 加速表计算状态(新增) */
static volatile uint16_t acc_calc_progress = 0;   /* 0~1000, 1000=完成 */
static volatile uint8_t  acc_calc_status = 0;     /* 0=idle, 1=calculating, 2=done, 3=error */
static uint32_t acc_timc = 32000000;  /* 当前表参数(默认ROM值) */
static uint32_t acc_u = 500;
static uint32_t acc_a = 320000;
static uint16_t acc_len = 10015;
void shizhongkaiguan(uint32_t a)
{
	if(a == 0)
	{
		/* 到达目标后停止TIM2, 节省CPU (新增, 原来引用htim7为空操作) */
		__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
		__HAL_TIM_DISABLE(&htim2);
	}else
	{
		__HAL_TIM_ENABLE(&htim2);
		__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
	}
}
void pwmzhongduankongzhi(void)
{
	/* 局部缓存dangqiansudu, 避免反复访问volatile int64(每次访问8字节非原子) */
	int64_t sd = bujindianji[0].dangqiansudu;

	//如果用的不是pwm方式，首先交换引脚电平给出脉冲
	//jjj[0] = TIM4->CNT;

	if(sd > 0) bujindianji[0].dangqianweizhi++;
	if(sd < 0) bujindianji[0].dangqianweizhi--;

	//判断电机正转还是反转
	if(sd > 0)
	{
		bujindianji[0].yudingsudu = bujindianji[0].zuidasudu;
		if(bujindianji[0].yudingsudu > sd) {
			bujindianji[0].sudukongzhi = 1;
		} else {
			if(bujindianji[0].yudingsudu == sd) {
				bujindianji[0].sudukongzhi = 0;
			} else {
				bujindianji[0].sudukongzhi = -1;
			}
		}
		bujindianji[0].tingzhiweizhi = bujindianji[0].dangqianweizhi + sd;
		if(bujindianji[0].tingzhiweizhi >= bujindianji[0].yudingweizhi) {
			bujindianji[0].sudukongzhi = -1;
		}
		sd = sd + bujindianji[0].sudukongzhi;
		bujindianji[0].dangqiansudu = sd;
		/* 索引钳位+ARR最小值保护: 防止越界读到0导致TIM2疯狂中断 */
		{
			int32_t idx = (int32_t)sd;
			uint16_t arr;
			if (idx < 0) idx = -idx;
			if (idx >= ACC_TABLE_MAX_LEN) idx = ACC_TABLE_MAX_LEN - 1;
			arr = p_jiasuliebiao[idx];
			if (arr < 50) arr = 50;  /* 最小ARR=50, 对应640kHz, 留够中断处理时间 */
			DINGSHIQICHONGZHI = arr;
		}
	}
	if(sd < 0)
	{
		bujindianji[0].yudingsudu = -bujindianji[0].zuidasudu;
		if(bujindianji[0].yudingsudu < sd) {
			bujindianji[0].sudukongzhi = -1;
		} else {
			if(bujindianji[0].yudingsudu == sd) {
				bujindianji[0].sudukongzhi = 0;
			} else {
				bujindianji[0].sudukongzhi = 1;
			}
		}
		bujindianji[0].tingzhiweizhi = bujindianji[0].dangqianweizhi + sd;
		if(bujindianji[0].tingzhiweizhi <= bujindianji[0].yudingweizhi) {
			bujindianji[0].sudukongzhi = 1;
		}
		sd = sd + bujindianji[0].sudukongzhi;
		bujindianji[0].dangqiansudu = sd;
		/* 索引钳位+ARR最小值保护 */
		{
			int32_t idx = (int32_t)sd;
			uint16_t arr;
			if (idx < 0) idx = -idx;
			if (idx >= ACC_TABLE_MAX_LEN) idx = ACC_TABLE_MAX_LEN - 1;
			arr = p_jiasuliebiao[idx];
			if (arr < 50) arr = 50;
			DINGSHIQICHONGZHI = arr;
		}
	}

	if(sd == 0)
	{
		if(bujindianji[0].dangqianweizhi < bujindianji[0].yudingweizhi)
		{
			bujindianji[0].yudingsudu = bujindianji[0].zuidasudu;
			bujindianji[0].dangqiansudu = 1;
			/* 启动时设置ARR(原代码漏设, 导致首拍用旧ARR) */
			DINGSHIQICHONGZHI = p_jiasuliebiao[1];
		} else if(bujindianji[0].dangqianweizhi > bujindianji[0].yudingweizhi) {
			bujindianji[0].yudingsudu = -bujindianji[0].zuidasudu;
			bujindianji[0].dangqiansudu = -1;
			DINGSHIQICHONGZHI = p_jiasuliebiao[1];
		} else {
			shizhongkaiguan(0);
		}
	}
	if(bujindianji[0].dangqiansudu > 0)
		bujindianji[0].shijifangxiang = 1;
	if(bujindianji[0].dangqiansudu < 0)
		bujindianji[0].shijifangxiang = 0;
}

//参数1 定时器最小分辨率
//参数2 初始速度
//参数3 加速度
//参数4 需要得到的列表成员数
void jiasuleibiaojisuan(uint64_t timc,uint64_t u,uint64_t a ,uint64_t len)
{
	uint64_t i;
	for(i = 0; i < len ; i++)
	{
		/* jiasuliebiao[i] = timc / sqrt(u² + 2*a*i)
		 * 注意: 此函数仅计算不存储, 实际存储由AccTable_Calculate完成 */
		volatile uint16_t val = (uint16_t)(timc / (uint64_t)sqrt((double)(u*u + 2*a*i)));
		(void)val;
	}
}

//输入目标速度，返回列表中对应的值
//参数1 定时器最大频率
//参数2 目标速度
uint32_t s2b(uint32_t timc,uint32_t s)
{
	uint32_t i = 0;
	if((timc / p_jiasuliebiao[ACC_TABLE_MAX_LEN-1]) < s)
	{
		return ACC_TABLE_MAX_LEN-1;
	}
	while((timc / p_jiasuliebiao[i]) < s )
	{
		i++;
	}
	return i;
}
//输入目标速度，返回列表中对应的值

//参数1 初始速度
//参数2 加速度
//参数3 目标速度
//返回达到目标速度所需的步数
uint64_t s2b2(uint64_t u,uint64_t a ,uint64_t s)
{
	uint64_t x;
	//chuankoufasong(210,s);

	x =(uint64_t) (s * s);
	//chuankoufasong(211,x);
	//chuankoufasong(213,a);
	x = (x - u*u)/2/a;
	//chuankoufasong(212,x);
	return x;
}

//使用外部速度计算器时，周期性调用此函数计算速度
//最小速度的平方加2倍加速度
void shishisuduyunsuan(void)
{
	if(bujindianji[0].dangqiansudu < 0)
	bujindianji[0].dingshi = 32000000 / ( sqrt(250000 + 2 * 1000 * (-bujindianji[0].dangqiansudu)));
	if(bujindianji[0].dangqiansudu >= 0)
	bujindianji[0].dingshi = 32000000 / ( sqrt(250000 + 2 * 1000 * (bujindianji[0].dangqiansudu)));
}


///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

//多轴插补算法
//开始不能开启定时器，定义所有的实际位置。初始位置一般为0
//bujindianji[0].zuidasudu = s2b2(500,8000 ,16000);//设置�??????大�?�度
//  zhoushijiweizhi[0] = 0;
//  zhoushijiweizhi[1] = 0;
//  zhoushijiweizhi[2] = 0;
//  zhoushijiweizhi[3] = 0;
//  zhoushijiweizhi[4] = 0;
//  zhoushijiweizhi[5] = 0;

//先关闭定时器
//HAL_TIM_Base_Stop_IT(&htim3);

//然后设定目标位置，设定位置时不能开启定时器，否则位置有误差,在这个函数里面，计算了绝对距离
//  chabu_6x(5000,-2000,30000,123000,-88653,9);
//设定好目标位置之后开启定时器
//HAL_TIM_Base_Start_IT(&htim3);
//此时已经启动了，只需实时从该函数更新位置就行了。
//gengxinshijiweizhi_6x();
//一共3个变量，实际位置，预定位置，启动位置，需要实时读取的时实际位置，其他两个用于插补时的计算用。


typedef struct
{
	 int64_t shijiweizhi;
	 int64_t yudingweizhi;
	int64_t qishiweizhi;
} ZHOUJISUAN;
ZHOUJISUAN zhou[10];

int64_t zhoushijiweizhi[10],zhouyudingweizhi[10],zhouqishiweizhi[10];


//--------------------------------------------------------
int64_t jisuanjueduijuli_2x(int64_t a,int64_t b)
{
	return sqrt(a*a + b*b);
}
int64_t jisuanjueduijuli_3x(int64_t a,int64_t b,int64_t c)
{
	return sqrt(a*a + b*b + c*c);
}
int64_t jisuanjueduijuli_6x(int64_t a,int64_t b,int64_t c,int64_t d,int64_t e,int64_t f)
{
	return sqrt(a*a + b*b + c*c + d*d +e*e + f*f);
}
//输出两轴绝对位置
void chabu_2x(int64_t a,int64_t b)
{
	//HAL_TIM_Base_Start_IT(&htim3);
	  bujindianji[0].dangqianweizhi = 0;
	  zhouqishiweizhi[0] = zhoushijiweizhi[0];
	  zhouqishiweizhi[1] = zhoushijiweizhi[1];
	  zhouyudingweizhi[0] = a;
	  zhouyudingweizhi[1] = b;



	  bujindianji[0].yudingweizhi = jisuanjueduijuli_2x(zhouqishiweizhi[0] - zhouyudingweizhi[0],zhouqishiweizhi[1] - zhouyudingweizhi[1]);

}
void chabu_3x(int64_t a,int64_t b,int64_t c)
{

	  bujindianji[0].dangqianweizhi = 0;
	  zhouqishiweizhi[0] = zhoushijiweizhi[0];
	  zhouqishiweizhi[1] = zhoushijiweizhi[1];
	  zhouqishiweizhi[2] = zhoushijiweizhi[2];
	  zhouyudingweizhi[0] = a;
	  zhouyudingweizhi[1] = b;
	  zhouyudingweizhi[2] = c;



	  bujindianji[0].yudingweizhi = jisuanjueduijuli_3x(zhouqishiweizhi[0] - zhouyudingweizhi[0],zhouqishiweizhi[1] - zhouyudingweizhi[1],zhouqishiweizhi[2] - zhouyudingweizhi[2]);
}
void chabu_6x(int64_t a,int64_t b,int64_t c,int64_t d,int64_t e,int64_t f)
{

	  bujindianji[0].dangqianweizhi = 0;
	  zhouqishiweizhi[0] = zhoushijiweizhi[0];
	  zhouqishiweizhi[1] = zhoushijiweizhi[1];
	  zhouqishiweizhi[2] = zhoushijiweizhi[2];
	  zhouqishiweizhi[3] = zhoushijiweizhi[3];
	  zhouqishiweizhi[4] = zhoushijiweizhi[4];
	  zhouqishiweizhi[5] = zhoushijiweizhi[5];
	  zhouyudingweizhi[0] = a;
	  zhouyudingweizhi[1] = b;
	  zhouyudingweizhi[2] = c;
	  zhouyudingweizhi[3] = d;
	  zhouyudingweizhi[4] = e;
	  zhouyudingweizhi[5] = f;



	  bujindianji[0].yudingweizhi = jisuanjueduijuli_6x(zhouqishiweizhi[0] - zhouyudingweizhi[0],zhouqishiweizhi[1] - zhouyudingweizhi[1],zhouqishiweizhi[2] - zhouyudingweizhi[2],zhouqishiweizhi[3] - zhouyudingweizhi[3],zhouqishiweizhi[4] - zhouyudingweizhi[4],zhouqishiweizhi[5] - zhouyudingweizhi[5]);

}
void gengxinshijiweizhi_2x(void)
{
	zhoushijiweizhi[0] = zhouqishiweizhi[0]+(zhouyudingweizhi[0] - zhouqishiweizhi[0])*bujindianji[0].dangqianweizhi/bujindianji[0].yudingweizhi;
	zhoushijiweizhi[1] = zhouqishiweizhi[1]+(zhouyudingweizhi[1] - zhouqishiweizhi[1])*bujindianji[0].dangqianweizhi/bujindianji[0].yudingweizhi;
//	zhoushijiweizhi[2] = zhouqishiweizhi[2]+(zhouyudingweizhi[2] - zhouqishiweizhi[2])*bujindianji[0].dangqianweizhi/bujindianji[0].yudingweizhi;
}
void gengxinshijiweizhi_3x(void)
{
	zhoushijiweizhi[0] = zhouqishiweizhi[0]+(zhouyudingweizhi[0] - zhouqishiweizhi[0])*bujindianji[0].dangqianweizhi/bujindianji[0].yudingweizhi;
	zhoushijiweizhi[1] = zhouqishiweizhi[1]+(zhouyudingweizhi[1] - zhouqishiweizhi[1])*bujindianji[0].dangqianweizhi/bujindianji[0].yudingweizhi;
	zhoushijiweizhi[2] = zhouqishiweizhi[2]+(zhouyudingweizhi[2] - zhouqishiweizhi[2])*bujindianji[0].dangqianweizhi/bujindianji[0].yudingweizhi;
}
void gengxinshijiweizhi_6x(void)
{
	zhoushijiweizhi[0] = zhouqishiweizhi[0]+(zhouyudingweizhi[0] - zhouqishiweizhi[0])*bujindianji[0].dangqianweizhi/bujindianji[0].yudingweizhi;
	zhoushijiweizhi[1] = zhouqishiweizhi[1]+(zhouyudingweizhi[1] - zhouqishiweizhi[1])*bujindianji[0].dangqianweizhi/bujindianji[0].yudingweizhi;
	zhoushijiweizhi[2] = zhouqishiweizhi[2]+(zhouyudingweizhi[2] - zhouqishiweizhi[2])*bujindianji[0].dangqianweizhi/bujindianji[0].yudingweizhi;
	zhoushijiweizhi[3] = zhouqishiweizhi[3]+(zhouyudingweizhi[3] - zhouqishiweizhi[3])*bujindianji[0].dangqianweizhi/bujindianji[0].yudingweizhi;
	zhoushijiweizhi[4] = zhouqishiweizhi[4]+(zhouyudingweizhi[4] - zhouqishiweizhi[4])*bujindianji[0].dangqianweizhi/bujindianji[0].yudingweizhi;
	zhoushijiweizhi[5] = zhouqishiweizhi[5]+(zhouyudingweizhi[5] - zhouqishiweizhi[5])*bujindianji[0].dangqianweizhi/bujindianji[0].yudingweizhi;
}


/* ==================== 步进加减速位置模式控制接口(新增) ==================== */

/* 开启/关闭步进加减速.
 * en=1: 用current_pos初始化算法状态, 配置并启动TIM2.
 * en=0: 停止TIM2. */
void StepAccel_Enable(uint8_t en, int64_t current_pos)
{
    if (en) {
        /* 初始化算法状态: 从当前位置静止开始 */
        __disable_irq();
        bujindianji[0].dangqianweizhi = current_pos;
        bujindianji[0].yudingweizhi   = current_pos;
        bujindianji[0].dangqiansudu   = 0;
        bujindianji[0].sudukongzhi    = 0;
        bujindianji[0].shijifangxiang = 0;
        __enable_irq();
        /* 默认最大速度6400步/秒 (索引63), 若用户已通过MODBUS设置则保留 */
        if (bujindianji[0].zuidasudu == 0) {
            bujindianji[0].zuidasudu = s2b2(500, 320000, 6400);
        }
        /* 配置TIM2: PSC=1→32MHz(与ROM表参数一致), ARR=起步速度对应值 */
        htim2.Init.Prescaler = 1;
        htim2.Init.Period    = p_jiasuliebiao[0];  /* 64000 → 500步/秒 */
        HAL_TIM_Base_Init(&htim2);
        /* 优先级设为3(低于TIM1电流环0和USART1), 避免影响PID实时性 */
        HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
        HAL_TIM_Base_Start_IT(&htim2);
    } else {
        /* 停止TIM2 */
        __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
        __HAL_TIM_DISABLE(&htim2);
        /* 速度归零, 防止残留状态 */
        __disable_irq();
        bujindianji[0].dangqiansudu = 0;
        bujindianji[0].sudukongzhi  = 0;
        __enable_irq();
    }
}

/* ==================== 加速表计算与Flash存储(新增) ==================== */

/* Flash头部结构(写在ACC_TABLE_FLASH_HEADER处, 共16字节)
 * [0..3]  magic (0xACC7AB1E)
 * [4..7]  timc
 * [8..11] a (加速度)
 * [12..13] u (最小速度, 16位足够)
 * [14..15] len (表长度) */

/* 检查Flash中是否有有效的自定义表 */
static uint8_t AccTable_CheckFlash(void)
{
    uint32_t *header = (uint32_t *)ACC_TABLE_FLASH_HEADER;
    if (header[0] == ACC_TABLE_FLASH_MAGIC) {
        uint16_t *p16 = (uint16_t *)(ACC_TABLE_FLASH_HEADER + 12);
        uint16_t u = p16[0];    /* 偏移12: u(最小速度) */
        uint16_t len = p16[1];  /* 偏移14: len(表长度) */
        if (u > 0 && len > 0 && len <= ACC_TABLE_MAX_LEN) return 1;
    }
    return 0;
}

/* 开机加载: 若Flash有自定义表则切换指针, 否则用ROM默认表 */
void AccTable_LoadOnBoot(void)
{
    if (AccTable_CheckFlash()) {
        uint32_t *header = (uint32_t *)ACC_TABLE_FLASH_HEADER;
        uint16_t *p16 = (uint16_t *)(ACC_TABLE_FLASH_HEADER + 12);
        acc_timc = header[1];
        acc_a    = header[2];
        acc_u    = p16[0];
        acc_len  = p16[1];
        p_jiasuliebiao = (const uint16_t *)ACC_TABLE_FLASH_DATA;
        acc_calc_status = 2;  /* done, 使用自定义表 */
    } else {
        p_jiasuliebiao = jiasuliebiao;
        acc_calc_status = 0;  /* idle, 使用ROM表 */
    }
    acc_calc_progress = 1000;
}

/* 计算加速表并写入Flash
 * 公式: table[i] = timc / sqrt(u² + 2*a*i), 与pwm.py一致
 * 返回: 0=成功, 1=参数非法, 2=Flash擦除失败, 3=Flash写入失败 */
uint8_t AccTable_Calculate(uint32_t timc, uint32_t u, uint32_t a, uint16_t len)
{
    if (timc == 0 || u == 0 || a == 0 || len == 0 || len > ACC_TABLE_MAX_LEN) return 1;

    acc_calc_status = 1;  /* calculating */
    acc_calc_progress = 0;

    /* 停止TIM2, 防止计算期间中断查表读到不完整数据 */
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
    __HAL_TIM_DISABLE(&htim2);

    HAL_FLASH_Unlock();

    /* 擦除: 头部页(1页) + 数据区(len*2字节向上取整到1KB页) */
    FLASH_EraseInitTypeDef erase;
    uint32_t page_error;
    uint32_t data_bytes = (uint32_t)len * 2;
    uint32_t data_pages = (data_bytes + 1023) / 1024;
    uint32_t total_pages = 1 + data_pages;

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = ACC_TABLE_FLASH_HEADER;
    erase.NbPages = total_pages;
    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) {
        HAL_FLASH_Lock();
        acc_calc_status = 3;
        /* 恢复ROM表 */
        p_jiasuliebiao = jiasuliebiao;
        return 2;
    }

    /* 逐个计算并写入数据区(每次写16位半字) */
    uint32_t data_addr = ACC_TABLE_FLASH_DATA;
    for (uint32_t i = 0; i < len; i++) {
        double denom = sqrt((double)((uint64_t)u * u + (uint64_t)2 * a * i));
        uint16_t val = (uint16_t)(timc / (uint64_t)denom);
        if (val == 0) val = 1;  /* 防止除零 */
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, data_addr + i * 2, val) != HAL_OK) {
            HAL_FLASH_Lock();
            acc_calc_status = 3;
            p_jiasuliebiao = jiasuliebiao;
            return 3;
        }
        /* 更新进度(每16个更新一次, 减少 volatile 写开销) */
        if ((i & 0xF) == 0) {
            acc_calc_progress = (uint16_t)(i * 1000 / len);
        }
    }

    /* 写入头部(参数信息) - 每个写入都检查返回值, 任一失败则报错 */
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ACC_TABLE_FLASH_HEADER,     ACC_TABLE_FLASH_MAGIC) != HAL_OK ||
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ACC_TABLE_FLASH_HEADER + 4,  timc) != HAL_OK ||
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ACC_TABLE_FLASH_HEADER + 8,  a) != HAL_OK ||
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, ACC_TABLE_FLASH_HEADER + 12, (uint16_t)u) != HAL_OK ||
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, ACC_TABLE_FLASH_HEADER + 14, len) != HAL_OK) {
        HAL_FLASH_Lock();
        acc_calc_status = 3;
        p_jiasuliebiao = jiasuliebiao;
        return 3;
    }

    HAL_FLASH_Lock();

    /* 回读验证: 确认头部magic和数据区首项正确写入 */
    uint32_t *verify = (uint32_t *)ACC_TABLE_FLASH_HEADER;
    if (verify[0] != ACC_TABLE_FLASH_MAGIC) {
        acc_calc_status = 3;
        p_jiasuliebiao = jiasuliebiao;
        return 3;
    }
    uint16_t *verify_data = (uint16_t *)ACC_TABLE_FLASH_DATA;
    double verify_denom = sqrt((double)((uint64_t)u * u));
    uint16_t verify_val = (uint16_t)(timc / (uint64_t)verify_denom);
    if (verify_val == 0) verify_val = 1;
    if (verify_data[0] != verify_val) {
        acc_calc_status = 3;
        p_jiasuliebiao = jiasuliebiao;
        return 3;
    }

    /* 切换到Flash自定义表 */
    acc_timc = timc;
    acc_u = u;
    acc_a = a;
    acc_len = len;
    p_jiasuliebiao = (const uint16_t *)ACC_TABLE_FLASH_DATA;
    acc_calc_progress = 1000;
    acc_calc_status = 2;

    return 0;
}

/* 恢复ROM默认表(擦除Flash头部魔数即可) */
void AccTable_RestoreROM(void)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase;
    uint32_t page_error;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = ACC_TABLE_FLASH_HEADER;
    erase.NbPages = 1;
    HAL_FLASHEx_Erase(&erase, &page_error);
    HAL_FLASH_Lock();

    p_jiasuliebiao = jiasuliebiao;
    acc_timc = 32000000;
    acc_u = 500;
    acc_a = 320000;
    acc_len = 10015;
    acc_calc_status = 0;
    acc_calc_progress = 1000;
}

uint8_t AccTable_IsCustom(void)
{
    return (p_jiasuliebiao != jiasuliebiao) ? 1 : 0;
}

void AccTable_GetParams(uint32_t *timc, uint32_t *u, uint32_t *a, uint16_t *len)
{
    *timc = acc_timc;
    *u = acc_u;
    *a = acc_a;
    *len = acc_len;
}

uint16_t AccTable_GetProgress(void)
{
    return acc_calc_progress;
}

/* 将算法的dangqianweizhi/yudingweizhi同步到指定位置(如编码器当前位置).
 * 用于使能后首次设定目标前, 或切换模式后重新进入位置模式时. */
void StepAccel_SyncPosition(int64_t pos)
{
    __disable_irq();
    bujindianji[0].dangqianweizhi = pos;
    bujindianji[0].yudingweizhi   = pos;
    bujindianji[0].dangqiansudu   = 0;
    bujindianji[0].sudukongzhi    = 0;
    __enable_irq();
}

/* 若TIM2已停止(到位后shizhongkaiguan(0)关闭), 重新启动以响应新目标.
 * 若TIM2仍在运行则什么都不做. */
void StepAccel_Restart(void)
{
    if ((TIM2->CR1 & TIM_CR1_CEN) == 0) {
        /* 计数器已停止, 重新配置并启动 */
        htim2.Init.Period = p_jiasuliebiao[0];
        HAL_TIM_Base_Init(&htim2);
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
        HAL_TIM_Base_Start_IT(&htim2);
    }
}


