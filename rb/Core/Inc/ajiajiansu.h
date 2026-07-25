/*
 * abujindianjipwm.h
 *
 *  Created on: Mar 22, 2023
 *      Author: HuaWei
 */

#ifndef ABUJINDIANJIPWM_H_
#define ABUJINDIANJIPWM_H_

typedef struct
{

//	__IO 	 int64_t jiasudu;  //加速度 步每秒
//	__IO 	 int64_t jiansudu;//减速度 步每秒

	__IO int64_t dangqiansudu;//当前运行速度
	__IO int64_t yudingsudu;//预定的运行速度
		 int64_t sudukongzhi;//速度控制器，控制加速或者减速

	__IO int64_t tingzhiweizhi;//最小停止位置20190702


	__IO int64_t zuidasudu ;//最大运行速度 步每秒
	__IO int64_t zuixiaosudu;//最小运行速度（启动速度） 步每秒

	int64_t jiluweizhi;//使用外部中断记录的位置
	__IO int64_t dangqianweizhi;//当前步位置
	     int64_t yudingweizhi;//预定将要到达的步位置
	     int shijifangxiang;
     uint16_t dingshi;
} bujinpwm;

/* 步进电机加减速参数数组(定义在ajiajiansu.c), 当前仅使用[0],
 * MODBUS寄存器0x006B~0x0090映射bujindianji[0]的各字段 */
extern bujinpwm bujindianji[4];

/* 加速表指针(新增): 默认指向ROM表jiasuliebiao,
 * 上位机触发计算后指向Flash自定义表区. 所有查表用此指针而非直接用jiasuliebiao */
extern const uint16_t *p_jiasuliebiao;

/* 加速表Flash存储区(新增):
 * 头部(0x0801A000): 魔数+参数+长度, 共1页(1KB)
 * 数据(0x0801A400起): uint16_t数组, 最多10015个元素(20030字节≈20KB)
 * 配置页(0x0801FC00): 原有电机配置, 不受影响 */
#define ACC_TABLE_FLASH_HEADER   0x0801A000  /* 头部页地址 */
#define ACC_TABLE_FLASH_DATA     0x0801A400  /* 数据区起始地址 */
#define ACC_TABLE_FLASH_MAGIC    0xACC7AB1E  /* 魔数标识 */
#define ACC_TABLE_MAX_LEN        10015       /* 最大表长度 */




void jiasuleibiaojisuan(uint64_t,uint64_t,uint64_t,uint64_t);
void pwmzhongduankongzhi(void);
uint32_t s2b(uint32_t ,uint32_t );
uint64_t s2b2(uint64_t ,uint64_t  ,uint64_t );
void shishisuduyunsuan(void);


void chabu_2x(int64_t ,int64_t );
void chabu_3x(int64_t ,int64_t ,int64_t );
void chabu_6x(int64_t ,int64_t ,int64_t ,int64_t ,int64_t ,int64_t );
void gengxinshijiweizhi_2x(void);
void gengxinshijiweizhi_3x(void);
void gengxinshijiweizhi_6x(void);

/* === 步进加减速位置模式控制接口(新增) ===
 * StepAccel_Enable: 开启/关闭步进加减速. 开启时用current_pos初始化算法状态并启动TIM2.
 * StepAccel_SyncPosition: 将算法的dangqianweizhi/yudingweizhi同步到指定位置(如编码器当前位置).
 * StepAccel_Restart: 若TIM2已停止(到位后), 重新启动以响应新目标. */
void StepAccel_Enable(uint8_t en, int64_t current_pos);
void StepAccel_SyncPosition(int64_t pos);
void StepAccel_Restart(void);

/* === 加速表计算与Flash存储接口(新增) ===
 * AccTable_Calculate: 用参数(timc,u,a,len)计算加速表并写入Flash.
 *   返回0=成功, 1=参数非法, 2=Flash擦除失败, 3=Flash写入失败
 * AccTable_RestoreROM: 恢复使用ROM默认表(清除Flash自定义表)
 * AccTable_IsCustom: 返回1=当前使用Flash自定义表, 0=使用ROM表
 * AccTable_GetParams: 读取当前Flash表参数(无自定义表则返回ROM默认参数)
 * AccTable_GetProgress: 计算进度(0~1000, 1000=完成) */
uint8_t AccTable_Calculate(uint32_t timc, uint32_t u, uint32_t a, uint16_t len);
void AccTable_RestoreROM(void);
uint8_t AccTable_IsCustom(void);
void AccTable_GetParams(uint32_t *timc, uint32_t *u, uint32_t *a, uint16_t *len);
uint16_t AccTable_GetProgress(void);
void AccTable_LoadOnBoot(void);  /* 开机时调用, 检查Flash并加载自定义表 */




#endif /* ABUJINDIANJIPWM_H_ */
