/**
  ******************************************************************************
  * @file    modbus.h
  * @brief   MODBUS RTU协议头文件
  ******************************************************************************
  */

#ifndef __MODBUS_H
#define __MODBUS_H

#include "main.h"
#include "motor_control.h"

/* MODBUS功能码 */
#define MB_FUNC_READ_HOLDING_REGS    0x03  /* 读保持寄存器 */
#define MB_FUNC_WRITE_SINGLE_REG     0x06  /* 写单个寄存器 */
#define MB_FUNC_WRITE_MULTIPLE_REGS  0x10  /* 写多个寄存器 */

/* MODBUS异常码 */
#define MB_EX_NONE                   0x00
#define MB_EX_ILLEGAL_FUNCTION       0x01
#define MB_EX_ILLEGAL_DATA_ADDRESS   0x02
#define MB_EX_ILLEGAL_DATA_VALUE     0x03
#define MB_EX_SLAVE_DEVICE_FAILURE   0x04

/* 寄存器地址定义 */
/* 控制寄存器 (读写) */
#define REG_CONTROL_WORD             0x0000  /* 模式设置 */
#define REG_MODE_SET                 0x0001  /* 启动方式 (0=直接启动, 1=标志位启动) */
#define REG_TARGET_POS_H3            0x0002  /* 目标位置高32位 */
#define REG_TARGET_POS_H2            0x0003  /* 目标位置高16位 */
#define REG_TARGET_POS_L2            0x0004  /* 目标位置低16位 */
#define REG_TARGET_POS_L1            0x0005  /* 目标位置最低位 */
#define REG_TARGET_SPEED_H           0x0006  /* 目标速度高16位 */
#define REG_TARGET_SPEED_L           0x0007  /* 目标速度低16位 */
#define REG_POS_KP                   0x0008  /* 位置环Kp (×100) */
#define REG_POS_KI                   0x0009  /* 位置环Ki (×100) */
#define REG_POS_KD                   0x000A  /* 位置环Kd (×100) */
#define REG_SPD_KP                   0x000B  /* 速度环Kp (×100) */
#define REG_SPD_KI                   0x000C  /* 速度环Ki (×100) */
#define REG_SPD_KD                   0x000D  /* 速度环Kd (×100) */
#define REG_DEAD_ZONE                0x000E  /* 死区 */
#define REG_MAX_OUTPUT               0x000F  /* 最大输出 */
#define REG_START_MODE               0x0010  /* 启动触发 (写1启动，仅在0x0001=1时需要) */
#define REG_DIRECTION                0x0011  /* 电机方向 (0=CW正转, 1=CCW反转) */
#define REG_ENCODER_DIRECTION        0x0012  /* 编码器方向 (0=正常, 1=反转) */
#define REG_START_FLAG               0x0013  /* 保留寄存器 */
#define REG_SLAVE_ADDR               0x0014  /* MODBUS从机地址 (1~247) */
#define REG_BAUD_RATE                0x0015  /* MODBUS波特率 (波特率/100) */
#define REG_SET_ORIGIN               0x0016  /* 设置当前位置为原点 (写1触发, 读取始终返回0) */
#define REG_HOME_MODE                0x0017  /* 复位模式 (0=关闭, 1=堵转复位, 2=开关精确复位, 3=双向堵转复位, 4=双向传感器居中复位) */
#define REG_HOME_DIRECTION           0x0018  /* 复位方向 (0=负方向, 1=正方向) */
#define REG_HOME_CURRENT             0x0019  /* 复位电流限制(PWM值, 10~1000) */
#define REG_HOME_SPEED_H             0x001A  /* 复位速度高16位 */
#define REG_HOME_SPEED_L             0x001B  /* 复位速度低16位 */
#define REG_HOME_MAX_DISTANCE_H      0x001C  /* 复位最大距离高16位 */
#define REG_HOME_MAX_DISTANCE_L      0x001D  /* 复位最大距离低16位 */
#define REG_HOME_BACK_DISTANCE_H     0x001E  /* 碰撞后退回距离高16位 */
#define REG_HOME_BACK_DISTANCE_L     0x001F  /* 碰撞后退回距离低16位 */
#define REG_HOME_TRIGGER             0x0020  /* 执行一次复位 (写1触发, 读取始终返回0) */
#define REG_HOME_AUTO_START          0x0021  /* 开机自动复位 (0=关闭, 1=开启, 默认1) */
#define REG_INPUT1_FUNC              0x0022  /* PB4功能 (0=脉冲,1=方向,2=原点位置开关,3=限位,4=外部目标位置电平,5=外部目标速度电平,6=执行复位操作,7=无功能,8=停止,9=启动标志位触发) */
#define REG_INPUT1_POLARITY          0x0023  /* PB4极性 (0=高电平有效, 1=低电平有效) */
#define REG_INPUT1_LIMIT_DIR         0x0024  /* PB4限位方向 (0=停止正方向, 1=停止负方向) */
#define REG_INPUT2_FUNC              0x0025  /* PB5功能 (同REG_INPUT1_FUNC) */
#define REG_INPUT2_POLARITY          0x0026  /* PB5极性 */
#define REG_INPUT2_LIMIT_DIR         0x0027  /* PB5限位方向 */
#define REG_MAX_RUN_SPEED_H          0x0028  /* 最大运行速度高16位(脉冲/秒, 0=无限制) */
#define REG_MAX_RUN_SPEED_L          0x0029  /* 最大运行速度低16位 */
#define REG_INPUT1_TARGET_POS_H3     0x002A  /* PB4目标位置电平预设位置 bit[63:48] */
#define REG_INPUT1_TARGET_POS_H2     0x002B  /* PB4目标位置电平预设位置 bit[47:32] */
#define REG_INPUT1_TARGET_POS_L2     0x002C  /* PB4目标位置电平预设位置 bit[31:16] */
#define REG_INPUT1_TARGET_POS_L1     0x002D  /* PB4目标位置电平预设位置 bit[15:0] */
#define REG_INPUT2_TARGET_POS_H3     0x002E  /* PB5目标位置电平预设位置 bit[63:48] */
#define REG_INPUT2_TARGET_POS_H2     0x002F  /* PB5目标位置电平预设位置 bit[47:32] */
#define REG_INPUT2_TARGET_POS_L2     0x0030  /* PB5目标位置电平预设位置 bit[31:16] */
#define REG_INPUT2_TARGET_POS_L1     0x0031  /* PB5目标位置电平预设位置 bit[15:0] */
#define REG_HOME_PRECISION_SPEED_H   0x0032  /* 开关精确复位低速检测速度高16位 */
#define REG_HOME_PRECISION_SPEED_L   0x0033  /* 开关精确复位低速检测速度低16位 */
#define REG_HOME_PRECISION_CYCLES    0x0034  /* 开关精确复位来回检测次数 */
#define REG_INPUT1_TARGET_SPEED_H    0x0035  /* PB4外部目标速度高16位 */
#define REG_INPUT1_TARGET_SPEED_L    0x0036  /* PB4外部目标速度低16位 */
#define REG_INPUT2_TARGET_SPEED_H    0x0037  /* PB5外部目标速度高16位 */
#define REG_INPUT2_TARGET_SPEED_L    0x0038  /* PB5外部目标速度低16位 */
#define REG_SPEED_ACQ_START          0x003A  /* 转速采集启动/状态: 写1启动采集, 读=状态(0=空闲,1=采集中,2=完成) */
#define REG_SPEED_ACQ_DIV            0x003B  /* 转速采集分频值(1=100us, 50=5ms, 默认50) */
#define REG_SPEED_ACQ_COUNT          0x003C  /* 采集已采样数量(只读, 0~5120) */
#define REG_SPEED_ACQ_STATUS         0x003D  /* 转速采集状态(只读, 同0x003A读取值) */
#define REG_SPEED_ACQ_TYPE           0x003E  /* 采集类型: 0=转速,1=PWM,2=位置,3=电流,4=PC0电压,5=PC2,6=PC3, 7~15=电流环开启PID输出(位置PID/速度PID/电流PID各P/I/D), 16~21=电流环关闭PID输出(位置PID/速度PID各P/I/D), 22=位置误差, 23=速度误差 */
#define REG_SPEED_ACQ_SIZE           0x003F  /* 采集点数(1~5120, 默认5120) */

/* 堵转保护寄存器 0x0040~0x0044 */
#define REG_STALL_PROT_EN            0x0040  /* 堵转保护使能: 0=关闭, 1=开启 */
#define REG_STALL_ERR_LIMIT          0x0041  /* 堵转误差阈值: 位置模式=脉冲, 速度模式=脉冲/秒 */
#define REG_STALL_TIME               0x0042  /* 堵转持续时间阈值(单位=PID周期5ms, 0=立即触发) */
#define REG_STALL_STATUS             0x0043  /* 堵转状态(只读): 0=正常, 1=已触发输出关闭 */
#define REG_STALL_RESET              0x0044  /* 堵转复位(写1清除堵转标志并恢复PID运行) */

/* 电流环寄存器 0x0045~0x004D (新增)
 * 电流环使能前需先标定 REG_CUR_OFFSET(零电流ADC值) 和 REG_CUR_SCALE(转换系数)
 * 硬件参考: INA240A1PWR(增益20V/V) + 10mΩ采样电阻
 *   - 总跨阻 0.2 V/A, 3.3V ADC, 零电流=1.65V(ADC=2048)
 *   - 满量程 ±8.25A 对应相对电流 ±1000
 *   - 默认 current_offset=2048, current_scale=1000/2048≈0.488
 *   - 1A实际 ≈ 121相对单位, 5A限流→REG_OVER_CUR_LIMIT≈606 */
#define REG_CUR_LOOP_EN              0x0045  /* 电流环使能: 0=关闭(位置/速度环直接输出PWM,原行为), 1=开启(三环级联) */
#define REG_CUR_KP                   0x0046  /* 电流环Kp(×100, 有符号) */
#define REG_CUR_KI                   0x0047  /* 电流环Ki(×100, 有符号) */
#define REG_CUR_KD                   0x0048  /* 电流环Kd(×1000, 3位小数, 有符号) */
#define REG_CUR_OFFSET               0x0049  /* 零电流ADC原始值(标定用, 0~4095, 默认2048=12位ADC中点) */
#define REG_CUR_SCALE                0x004A  /* ADC到相对电流转换系数(×10000, 默认4883对应0.4883) */
#define REG_OVER_CUR_LIMIT           0x004B  /* 过流保护阈值(相对值×10, 0=关闭; 例1210=121.0相对≈1A, 6060=606.0相对≈5A) */
#define REG_OVER_CUR_STATUS          0x004C  /* 过流状态(只读): 0=正常, 1=已触发输出关闭 */
#define REG_OVER_CUR_RESET           0x004D  /* 过流复位(写1清除过流标志并恢复运行) */

/* PC2/PC3 ADC功能寄存器 0x004E~0x0055 (新增)
 * PC2/PC3分别可选4种ADC模式或无功能; 冲突时PC2优先
 * ADC值归一化为0~1, 乘以下列预设值得到目标
 *   ADC转速/位置转速模式 → REG_ADC_MAX_SPEED (脉冲/秒)
 *   ADC开环模式          → REG_ADC_MAX_PWM   (-1000~+1000)
 *   ADC位置模式          → REG_ADC_MAX_POS   (脉冲)
 * 三类预设值公用, 切换模式时同一输入框的值直接生效 */
#define REG_PC2_FUNC                 0x004E  /* PC2功能 (0=无,1=ADC转速,2=ADC位置转速,3=ADC开环,4=ADC位置) */
#define REG_PC3_FUNC                 0x004F  /* PC3功能 (同上, PC2!=无时PC3被忽略) */
#define REG_ADC_MAX_SPEED_H          0x0050  /* ADC最大速度高16位(脉冲/秒, 默认10000) */
#define REG_ADC_MAX_SPEED_L          0x0051  /* ADC最大速度低16位 */
#define REG_ADC_MAX_PWM              0x0052  /* ADC最大PWM(-1000~+1000, 有符号, 默认500) */
#define REG_ADC_MAX_POS_H3           0x0053  /* ADC最大位置bit[63:48] (默认100000) */
#define REG_ADC_MAX_POS_H2           0x0054  /* ADC最大位置bit[47:32] */
#define REG_ADC_MAX_POS_L2           0x0055  /* ADC最大位置bit[31:16] */
#define REG_ADC_MAX_POS_L1           0x0056  /* ADC最大位置bit[15:0] */
/* ADC最小值寄存器 0x0057~0x005D (新增)
 * ADC值=0时对应下列最小值, ADC值=1时对应上述最大值
 * 最终输出 = 最小值 + ADC归一化值 × (最大值 - 最小值)
 * 所有最小值均可为负数, 以有符号格式存储 */
#define REG_ADC_MIN_SPEED_H          0x0057  /* ADC最小速度高16位(脉冲/秒, 有符号, 默认0) */
#define REG_ADC_MIN_SPEED_L          0x0058  /* ADC最小速度低16位 */
#define REG_ADC_MIN_PWM              0x0059  /* ADC最小PWM(-1000~+1000, 有符号, 默认0) */
#define REG_ADC_MIN_POS_H3           0x005A  /* ADC最小位置bit[63:48] (有符号, 默认0) */
#define REG_ADC_MIN_POS_H2           0x005B  /* ADC最小位置bit[47:32] */
#define REG_ADC_MIN_POS_L2           0x005C  /* ADC最小位置bit[31:16] */
#define REG_ADC_MIN_POS_L1           0x005D  /* ADC最小位置bit[15:0] */
/* ADC死区寄存器 0x005E~0x0061 (新增, 两个独立死区)
 * 每个死区可独立设置位置和宽度, ADC值落入任一死区时强制输出该位置对应的归一化值
 *   位置: 0=最小点(ADC=0), 1=中位点(ADC=2048), 2=最大点(ADC=4095)
 *   宽度: ADC原始值计数, 以选定位置为中心两侧各扩展width/2, 0=关闭死区
 * 用途: 防止电位器在停止位置附近因噪声导致电机缓慢转动 */
#define REG_ADC_DEAD_ZONE1_POS        0x005E  /* ADC死区1位置 (0=最小点, 1=中位点, 2=最大点, 默认0) */
#define REG_ADC_DEAD_ZONE1_WIDTH      0x005F  /* ADC死区1宽度 (0~4095, 0=关闭, 默认0) */
#define REG_ADC_DEAD_ZONE2_POS        0x0060  /* ADC死区2位置 (0=最小点, 1=中位点, 2=最大点, 默认0) */
#define REG_ADC_DEAD_ZONE2_WIDTH      0x0061  /* ADC死区2宽度 (0~4095, 0=关闭, 默认0) */

/* 电流限幅寄存器 (新增)
 * 限制电流环目标(current_target)的绝对值, 平滑限幅, 不触发保护.
 * 与REG_OVER_CUR_LIMIT区别: 限幅限制目标电流, 过流保护检测实际电流超限关闭输出.
 * 单位与电流相对值一致(×10, 1位小数), 0=不限制.
 * 例: 1A≈121相对 → REG_MAX_CURRENT=1210; 5A≈606相对 → 6060 */
#define REG_MAX_CURRENT               0x0062  /* 电流限幅(相对值×10, 0=不限制) */

/* 电流环使能时的速度环/位置环PID参数(双套PID自动切换, 新增) 0x0063~0x0068
 * 电流环使能时PID运算使用这些参数, 关闭时使用原 spd_Kp/Ki/Kd, pos_Kp/Ki/Kd
 * 典型值: 电流模式Kp约为电压模式1/3, Ki约为1/5 */
#define REG_SPD_KP_CUR                0x0063  /* 电流环使能时速度环Kp(×100) */
#define REG_SPD_KI_CUR                0x0064  /* 电流环使能时速度环Ki(×100) */
#define REG_SPD_KD_CUR                0x0065  /* 电流环使能时速度环Kd(×1000) */
#define REG_POS_KP_CUR                0x0066  /* 电流环使能时位置环Kp(×100) */
#define REG_POS_KI_CUR                0x0067  /* 电流环使能时位置环Ki(×100) */
#define REG_POS_KD_CUR                0x0068  /* 电流环使能时位置环Kd(×1000) */

/* 电流死区寄存器(新增)
 * 实际电流绝对值<此值且电流目标=0时, 强制PWM=0, 防止静止时细微电流.
 * 单位与电流相对值一致(×10, 1位小数), 0=关闭此功能.
 * 例: 30=3.0相对≈25mA, 100=10.0相对≈83mA */
#define REG_CUR_DEAD_ZONE             0x0069  /* 电流死区(相对值×10, 0=关闭) */

/* 外部ADC滤波寄存器(新增)
 * 滑动平均(Moving Average)窗口大小, 作用于PC2/PC3外部ADC通道.
 * 0或1=关闭滤波(直接用原始值), 2~128=滑动平均窗口大小.
 * 主循环5ms周期下延迟: 4点≈20ms, 16点≈80ms, 64点≈320ms, 100点≈500ms, 128点≈640ms(最大).
 * 越大越平滑但滞后越大. 推荐值4~32. */
#define REG_ADC_FILTER_SIZE           0x006A  /* 外部ADC滤波窗口(0~128, 0/1=关闭) */

/* 步进电机加减速参数寄存器 bujindianji[0] 0x006B~0x0090 (新增)
 * 映射 ajiajiansu.c 的 bujinpwm 结构体字段, 每个int64字段占4个连续寄存器
 * (H3=bit[63:48], H2=bit[47:32], L2=bit[31:16], L1=bit[15:0], 有符号)
 * 前3个字段为可写参数(写入立即生效, 不落Flash):
 *   0x006B~0x006E zuidasudu      最大运行速度(步/秒)
 *   0x006F~0x0072 zuixiaosudu    最小/启动速度(步/秒)
 *   0x0073~0x0076 yudingweizhi   预定目标位置(步)
 * 其余字段为运行状态(只读, 写入无效):
 *   0x0077~0x007A dangqiansudu   当前运行速度(步/秒)
 *   0x007B~0x007E yudingsudu     预定运行速度(步/秒)
 *   0x007F~0x0082 sudukongzhi    加减速控制(-1=减速, 0=匀速, 1=加速)
 *   0x0083~0x0086 tingzhiweizhi  停止位置(步)
 *   0x0087~0x008A jiluweizhi     外部中断记录位置(步)
 *   0x008B~0x008E dangqianweizhi 当前步位置(步)
 *   0x008F        shijifangxiang 实际方向(0=负方向, 1=正方向)
 *   0x0090        dingshi        定时器周期值
 * 注: 上位机用功能码03从0x006B一次读38个寄存器即可获取全部参数 */
#define REG_STEP_MAX_SPEED_H3         0x006B  /* 最大运行速度 bit[63:48] (读写) */
#define REG_STEP_MAX_SPEED_H2         0x006C  /* 最大运行速度 bit[47:32] */
#define REG_STEP_MAX_SPEED_L2         0x006D  /* 最大运行速度 bit[31:16] */
#define REG_STEP_MAX_SPEED_L1         0x006E  /* 最大运行速度 bit[15:0] */
#define REG_STEP_MIN_SPEED_H3         0x006F  /* 最小/启动速度 bit[63:48] (读写) */
#define REG_STEP_MIN_SPEED_H2         0x0070  /* 最小/启动速度 bit[47:32] */
#define REG_STEP_MIN_SPEED_L2         0x0071  /* 最小/启动速度 bit[31:16] */
#define REG_STEP_MIN_SPEED_L1         0x0072  /* 最小/启动速度 bit[15:0] */
#define REG_STEP_TARGET_POS_H3        0x0073  /* 预定目标位置 bit[63:48] (读写) */
#define REG_STEP_TARGET_POS_H2        0x0074  /* 预定目标位置 bit[47:32] */
#define REG_STEP_TARGET_POS_L2        0x0075  /* 预定目标位置 bit[31:16] */
#define REG_STEP_TARGET_POS_L1        0x0076  /* 预定目标位置 bit[15:0] */
#define REG_STEP_CUR_SPEED_H3         0x0077  /* 当前运行速度 bit[63:48] (只读) */
#define REG_STEP_CUR_SPEED_H2         0x0078  /* 当前运行速度 bit[47:32] */
#define REG_STEP_CUR_SPEED_L2         0x0079  /* 当前运行速度 bit[31:16] */
#define REG_STEP_CUR_SPEED_L1         0x007A  /* 当前运行速度 bit[15:0] */
#define REG_STEP_PRESET_SPEED_H3      0x007B  /* 预定运行速度 bit[63:48] (只读) */
#define REG_STEP_PRESET_SPEED_H2      0x007C  /* 预定运行速度 bit[47:32] */
#define REG_STEP_PRESET_SPEED_L2      0x007D  /* 预定运行速度 bit[31:16] */
#define REG_STEP_PRESET_SPEED_L1      0x007E  /* 预定运行速度 bit[15:0] */
#define REG_STEP_ACCEL_CTRL_H3        0x007F  /* 加减速控制 bit[63:48] (只读) */
#define REG_STEP_ACCEL_CTRL_H2        0x0080  /* 加减速控制 bit[47:32] */
#define REG_STEP_ACCEL_CTRL_L2        0x0081  /* 加减速控制 bit[31:16] */
#define REG_STEP_ACCEL_CTRL_L1        0x0082  /* 加减速控制 bit[15:0] */
#define REG_STEP_STOP_POS_H3          0x0083  /* 停止位置 bit[63:48] (只读) */
#define REG_STEP_STOP_POS_H2          0x0084  /* 停止位置 bit[47:32] */
#define REG_STEP_STOP_POS_L2          0x0085  /* 停止位置 bit[31:16] */
#define REG_STEP_STOP_POS_L1          0x0086  /* 停止位置 bit[15:0] */
#define REG_STEP_REC_POS_H3           0x0087  /* 外部中断记录位置 bit[63:48] (只读) */
#define REG_STEP_REC_POS_H2           0x0088  /* 外部中断记录位置 bit[47:32] */
#define REG_STEP_REC_POS_L2           0x0089  /* 外部中断记录位置 bit[31:16] */
#define REG_STEP_REC_POS_L1           0x008A  /* 外部中断记录位置 bit[15:0] */
#define REG_STEP_CUR_POS_H3           0x008B  /* 当前步位置 bit[63:48] (只读) */
#define REG_STEP_CUR_POS_H2           0x008C  /* 当前步位置 bit[47:32] */
#define REG_STEP_CUR_POS_L2           0x008D  /* 当前步位置 bit[31:16] */
#define REG_STEP_CUR_POS_L1           0x008E  /* 当前步位置 bit[15:0] */
#define REG_STEP_DIRECTION            0x008F  /* 实际方向(只读): 0=负方向, 1=正方向 */
#define REG_STEP_TIMING               0x0090  /* 定时器周期值(只读) */
#define REG_STEP_ACCEL_EN             0x0091  /* 步进加减速使能(读写): 0=关闭, 1=位置模式启用 */

/* 加速表计算寄存器(新增) 0x0092~0x009A
 * 上位机写入参数后触发计算, STM32计算加速表并存入Flash
 * 0x0092~0x0093: timc(主频, 32位, 如32000000)
 * 0x0094: u(最小速度, 16位, 如500)
 * 0x0095~0x0096: a(加速度, 32位, 如320000)
 * 0x0097: len(表长度, 16位, 最大10015)
 * 0x0098: 触发(写1=开始计算, 写2=恢复ROM默认表)
 * 0x0099: 状态(只读: 0=ROM表/idle, 1=计算中, 2=自定义表已启用, 3=错误)
 * 0x009A: 进度(只读: 0~1000, 1000=完成) */
#define REG_ACC_TABLE_TIMC_L          0x0092  /* 主频 低16位 (读写) */
#define REG_ACC_TABLE_TIMC_H          0x0093  /* 主频 高16位 */
#define REG_ACC_TABLE_U               0x0094  /* 最小速度 (读写) */
#define REG_ACC_TABLE_A_L             0x0095  /* 加速度 低16位 (读写) */
#define REG_ACC_TABLE_A_H             0x0096  /* 加速度 高16位 */
#define REG_ACC_TABLE_LEN             0x0097  /* 表长度 (读写, 最大10015) */
#define REG_ACC_TABLE_TRIGGER         0x0098  /* 触发(写1=计算, 写2=恢复ROM) */
#define REG_ACC_TABLE_STATUS          0x0099  /* 状态(只读) */
#define REG_ACC_TABLE_PROGRESS        0x009A  /* 进度(只读, 0~1000) */
#define REG_STEP_BASE                 REG_STEP_MAX_SPEED_H3  /* 加减速寄存器区起始地址 */
#define REG_STEP_END                  0x00A9  /* 寄存器区结束地址(含前馈0x009B + 多通道0x009C~0x009F + 自动校准0x00A0 + 电子齿轮0x00A1~0x00A2 + 中断优先级0x00A3~0x00A7 + 双向复位居中容差/超时0x00A8~0x00A9) */

/* 电流环前馈系数(新增) 0x009B
 * 速度前馈(反电动势补偿): PWM += Kff * velocity
 * 解决限流时电流目标被截顶→P项不足→PWM偏低→速度下降
 * ×10000, 建议150~250(=0.015~0.025), 0=关闭 */
#define REG_CUR_KFF                   0x009B  /* 电流环前馈系数(×10000, 0=关闭) */

/* 多通道采集寄存器(新增) 0x009C~0x009F
 * 支持同时采集1~4个通道, 每个采样周期依次采集所有通道,
 * 缓冲区数据交错存储(类似STM32 ADC扫描模式).
 * 通道1类型使用REG_SPEED_ACQ_TYPE(0x003E), 通道2~4使用下列寄存器.
 * 采样率不变(由REG_SPEED_ACQ_DIV控制), 缓冲区填充快N倍. */
#define REG_SPEED_ACQ_CHANNELS        0x009C  /* 采集通道数(1~4, 默认1=单通道兼容原行为) */
#define REG_SPEED_ACQ_TYPE2           0x009D  /* 通道2采集类型(编码同REG_SPEED_ACQ_TYPE) */
#define REG_SPEED_ACQ_TYPE3           0x009E  /* 通道3采集类型 */
#define REG_SPEED_ACQ_TYPE4           0x009F  /* 通道4采集类型 */

/* 开机自动零点电流校准(新增) 0x00A0
 * 0=关闭(默认): 使用Flash保存的offset, 手动校准后存Flash
 * 1=开启: 每次开机自动校准, 结果不存Flash(避免频繁烧写) */
#define REG_AUTO_CALIB_CURRENT        0x00A0  /* 开机自动零点电流校准: 0=关闭, 1=开启 */

/* 电子齿轮(脉冲倍频/分频, 新增) 0x00A1~0x00A2
 * 外部脉冲输入后, 实际移动量 = 输入脉冲 × (分子/分母)
 * 倍频: 分子>分母, 如 2/1 = 每脉冲移动2步
 * 分频: 分子<分母, 如 1/2 = 每2脉冲移动1步
 * 适用: 脉冲检测频率受限(如3KHz), 倍频提高等效速度, 分频配合高频源 */
#define REG_GEAR_NUMERATOR            0x00A1  /* 电子齿轮分子(1~65535, 默认1) */
#define REG_GEAR_DENOMINATOR          0x00A2  /* 电子齿轮分母(1~65535, 默认1) */

/* 中断优先级配置寄存器 0x00A3~0x00A7 (新增)
 * STM32F103有4位抢占优先级(0~15), 数值越小优先级越高
 * 默认值: USART1=1, TIM1=2, TIM2=3, EXTI4=1, EXTI9_5=1
 * 修改后立即生效并保存Flash, 掉电不丢失 */
#define REG_IRQ_PRIO_USART1           0x00A3  /* USART1中断优先级(0~15, 默认1) */
#define REG_IRQ_PRIO_TIM1             0x00A4  /* TIM1中断优先级(0~15, 默认2) */
#define REG_IRQ_PRIO_TIM2             0x00A5  /* TIM2中断优先级(0~15, 默认3) */
#define REG_IRQ_PRIO_EXTI4            0x00A6  /* EXTI4中断优先级(0~15, 默认1) */
#define REG_IRQ_PRIO_EXTI9_5          0x00A7  /* EXTI9_5中断优先级(0~15, 默认1) */

/* 双向复位居中容差/超时寄存器(新增) 0x00A8~0x00A9
 * 解决堵转双向复位后"差步导致原点偏移/复位无法结束"问题:
 *   - REG_HOME_CENTER_TOLERANCE: 到位误差容差(脉冲), 默认2. 误差在此范围内即判定复位成功.
 *   - REG_HOME_CENTER_TIMEOUT: 到位超时(单位=5ms tick, 0=不启用). 超过该时间仍未到位,
 *     强制视为复位成功, 原点直接采用计算出的中点+偏置位置, 避免一直差步卡死. */
#define REG_HOME_CENTER_TOLERANCE     0x00A8  /* 双向复位居中到位误差容差(脉冲, 默认10) */
#define REG_HOME_CENTER_TIMEOUT       0x00A9  /* 双向复位居中到位超时(5ms tick, 默认200=1秒, 0=不启用) */

#define REG_CALIB_CURRENT_END         0x00A9  /* 寄存器区结束地址(含双向复位居中容差/超时) */

/* 数据寄存器 (只读) - 采集缓冲区 0x0200~0x15FF (5120个int16_t, 10KB) */
#define REG_SPEED_DATA_BASE          0x0200  /* 采集数据起始地址 */
#define REG_SPEED_DATA_END           0x15FF  /* 采集数据结束地址 (0x0200 + SPEED_ACQ_BUF_SIZE - 1) */

/* 状态寄存器 (只读) */
#define REG_CURRENT_POS_H3           0x0100  /* 当前位置高32位 */
#define REG_CURRENT_POS_H2           0x0101  /* 当前位置高16位 */
#define REG_CURRENT_POS_L2           0x0102  /* 当前位置低16位 */
#define REG_CURRENT_POS_L1           0x0103  /* 当前位置最低位 */
#define REG_CURRENT_SPEED_H          0x0104  /* 当前速度高16位 */
#define REG_CURRENT_SPEED_L          0x0105  /* 当前速度低16位 */
#define REG_CURRENT_MODE             0x0106  /* 当前模式 */
#define REG_STATUS                   0x0107  /* 状态字 */
#define REG_CURRENT_PWM              0x0108  /* 当前PWM输出(有符号, -1000~+1000, 即±100%) */
#define REG_PID_ERROR_H              0x0109  /* PID当前误差高16位(×100, 有符号32位) */
#define REG_PID_ERROR_L              0x010A  /* PID当前误差低16位 */
#define REG_PID_P_H                  0x010B  /* PID比例项P输出高16位(×100) */
#define REG_PID_P_L                  0x010C  /* PID比例项P输出低16位 */
#define REG_PID_I_H                  0x010D  /* PID积分项I输出高16位(×100) */
#define REG_PID_I_L                  0x010E  /* PID积分项I输出低16位 */
#define REG_PID_D_H                  0x010F  /* PID微分项D输出高16位(×100) */
#define REG_PID_D_L                  0x0110  /* PID微分项D输出低16位 */

/* 电流环状态寄存器(只读) 0x0111~0x0112 (新增) */
#define REG_CURRENT_ACTUAL           0x0111  /* 实际电流(相对值×10, 有符号; ±10000=±1000相对=±8.25A) */
#define REG_CURRENT_TARGET_RO        0x0112  /* 电流目标(相对值×10, 有符号; 由速度环/位置环输出) */

/* ADC采样寄存器(只读) 0x0113~0x0115
 * 由ADC1 DMA循环模式持续刷新, 上位机可用功能码03读取:
 *   0x0113 = 供电电压(单位0.01V, PC0分压100K:10K, 实际电压=ADC电压×11)
 *            例: 3300 = 33.00V, 2400 = 24.00V
 *   0x0114 = 外部ADC1滤波后值(PC2, 12位 0~4095)
 *            滤波窗口由REG_ADC_FILTER_SIZE(0x006A)控制, 0或1=关闭返回原始值
 *   0x0115 = 外部ADC2滤波后值(PC3, 12位 0~4095), 同上 */
#define REG_SUPPLY_VOLTAGE           0x0113  /* 供电电压(只读, 单位0.01V) */
#define REG_EXT_ADC1                 0x0114  /* 外部ADC1滤波后值(只读, PC2) */
#define REG_EXT_ADC2                 0x0115  /* 外部ADC2滤波后值(只读, PC3) */

/* 三环PID调试寄存器(只读) 0x0116~0x0127 (新增)
 * 用于级联调参: 上位机可一次性读取位置环/速度环/电流环的error/P/I/D,
 * 实时观察三环工作状态, 定位"电流不上升/位置过冲/振荡"等问题.
 *
 * 寄存器布局(所有值×100, 有符号int16):
 *   位置环(5ms): 0x0116~0x0119 = error, P, I, D  (与0x0109~0x0110相同,此处独立便于批量读取)
 *   速度环(1ms): 0x011A~0x011D = error, P, I, D
 *   电流环(100us):0x011E~0x0121 = error, P, I, D
 *   中间变量:    0x0122=target_velocity, 0x0123=current_target, 0x0124=current_actual
 *
 * 注意: 单环模式(无max_run_speed)下速度环不运行, 0x011A~0x011D保持0.
 *       电流环关闭时0x011E~0x0121仍会更新(透传模式下error=0,P=I=D=0). */
#define REG_POS_LOOP_ERR             0x0116  /* 位置环误差(×100, 有符号) */
#define REG_POS_LOOP_P               0x0117  /* 位置环P项(×100) */
#define REG_POS_LOOP_I               0x0118  /* 位置环I项(×100) */
#define REG_POS_LOOP_D               0x0119  /* 位置环D项(×100) */
#define REG_SPD_LOOP_ERR             0x011A  /* 速度环误差(×100, 脉冲/秒) */
#define REG_SPD_LOOP_P               0x011B  /* 速度环P项(×100) */
#define REG_SPD_LOOP_I               0x011C  /* 速度环I项(×100) */
#define REG_SPD_LOOP_D               0x011D  /* 速度环D项(×100) */
#define REG_CUR_LOOP_ERR             0x011E  /* 电流环误差(×100, 相对值) */
#define REG_CUR_LOOP_P               0x011F  /* 电流环P项(×100) */
#define REG_CUR_LOOP_I               0x0120  /* 电流环I项(×100) */
#define REG_CUR_LOOP_D               0x0121  /* 电流环D项(×100) */
#define REG_TARGET_VELOCITY_RO       0x0122  /* 位置环输出的目标速度(×100, 脉冲/秒) */
#define REG_CURRENT_TARGET_DISP      0x0123  /* 电流目标(×10, 相对值, 与0x0112相同便于批量读取) */
#define REG_CURRENT_ACTUAL_DISP      0x0124  /* 实际电流(×10, 相对值, 与0x0111相同便于批量读取) */

/* 系统时间寄存器(只读) 0x0125~0x0128 (新增)
 * 系统时间(sys_time_100us)为64位整数, TIM1每100us中断+1, 单位0.1ms.
 * 上电后从0开始累计, 最大值 2^63-1 ≈ 9.22×10^18 (约2.9万年不会溢出).
 * 4个寄存器组成64位值(大端序):
 *   0x0125 = bit[63:48]   0x0126 = bit[47:32]
 *   0x0127 = bit[31:16]   0x0128 = bit[15:0]
 * 上位机读取后合并: time = (R0x0125<<48)|(R0x0126<<32)|(R0x0127<<16)|R0x0128
 * 实际时间 = time × 0.1ms, 即 time×100 微秒.
 * 用途: 上位机与下位机时间同步, 采集数据时间戳, 延时测量等. */
#define REG_SYS_TIME_H3              0x0125  /* 系统时间 bit[63:48] (只读, 单位0.1ms) */
#define REG_SYS_TIME_H2              0x0126  /* 系统时间 bit[47:32] */
#define REG_SYS_TIME_L2              0x0127  /* 系统时间 bit[31:16] */
#define REG_SYS_TIME_L1              0x0128  /* 系统时间 bit[15:0] */

/* 控制字位定义 */
#define CTRL_MODE_MASK               0x000F  /* 模式掩码 */

/* 状态字 (REG_STATUS 0x0107) 位定义
 *   bit15     = 启动标志已置位 (start_flag_set)
 *   bit14     = 正在执行复位 (homing)
 *   bit13     = 复位失败 (homing_failed)
 *   bit12     = 堵转保护已触发
 *   bit11     = 过流保护已触发
 *   bit10     = PB4引脚电平 (1=高, 0=低)
 *   bit9      = PB5引脚电平 (1=高, 0=低)
 *   bit8~bit4 = 保留(读作0)
 *   bit3~bit0 = 当前模式 (MotorMode_t) */
#define STATUS_START_FLAG            0x8000  /* 启动标志已置位 */
#define STATUS_HOMING                0x4000  /* 正在执行复位 */
#define STATUS_HOMING_FAILED         0x2000  /* 复位失败 */
#define STATUS_STALL_TRIPPED         0x1000  /* 堵转保护已触发 */
#define STATUS_OVER_CUR_TRIPPED      0x0800  /* 过流保护已触发 */
#define STATUS_PB4_LEVEL             0x0400  /* PB4引脚当前电平 */
#define STATUS_PB5_LEVEL             0x0200  /* PB5引脚当前电平 */

/* 模式定义 (与motor_control.h的MotorMode_t对应) */
#define MODE_POSITION                0  /* 位置模式 (POSITION_MODE) */
#define MODE_SPEED                   1  /* 速度模式 (SPEED_MODE) */
#define MODE_VELOCITY_POSITION       2  /* 位置速度模式 (VELOCITY_POSITION_MODE) */
#define MODE_OPENLOOP                3  /* 开环模式 (OPENLOOP_MODE) */
#define MODE_EXTERNAL_TARGET         4  /* 外部目标位置模式 (EXTERNAL_TARGET_MODE) */
#define MODE_EXTERNAL_TARGET_SPEED   5  /* 外部目标速度模式 (EXTERNAL_TARGET_SPEED_MODE) */
#define MODE_STANDBY                 6  /* 待机模式 (STANDBY_MODE) */
#define MODE_ADC_SPEED               7  /* ADC转速模式 (ADC_SPEED_MODE) */
#define MODE_ADC_POSITION_SPEED      8  /* ADC位置转速模式 (ADC_POSITION_SPEED_MODE) */
#define MODE_ADC_OPENLOOP            9  /* ADC开环模式 (ADC_OPENLOOP_MODE) */
#define MODE_ADC_POSITION            10 /* ADC位置模式 (ADC_POSITION_MODE) */
#define MODE_EXTERNAL_TARGET_POS_SPEED 11 /* 外部目标位置速度模式 (EXTERNAL_TARGET_POS_SPEED_MODE) */

/* MODBUS接收缓冲区大小 */
#define MB_RX_BUF_SIZE               128
#define MB_TX_BUF_SIZE               256  /* 支持一次读取最多125个寄存器(255字节) */

/*
 * 帧间超时最小阈值（TIM1 100us中断计数）
 * 实际阈值随当前波特率按3.5字符时间动态计算，且不低于4个100us计数
 */
#define MB_FRAME_TIMEOUT_TICKS       4

/* MODBUS上下文 */
typedef struct {
    uint8_t rx_buf[MB_RX_BUF_SIZE];          /* 接收缓冲区（中断中逐字节填充） */
    uint8_t rx_process_buf[MB_RX_BUF_SIZE];  /* 处理缓冲区（主循环使用） */
    uint8_t rx_byte;                          /* 单字节接收缓冲（HAL用） */
    volatile uint16_t rx_index;              /* 当前接收字节索引 */
    volatile uint16_t rx_len;                /* 接收到的数据长度 */
    volatile uint16_t timeout_ticks;         /* 帧间超时计数（TIM1每100us+1） */
    volatile uint16_t frame_timeout_ticks;   /* 帧间超时阈值（随波特率调整） */
    volatile uint8_t rx_complete;            /* 帧接收完成标志 */
    uint8_t tx_buf[MB_TX_BUF_SIZE];
    volatile uint8_t tx_busy;                /* 发送忙标志(中断发送中, 防止重入) */
    uint8_t slave_addr;
    MotorControl_t *motor;

    /* 调试统计 */
    volatile uint32_t rx_count;      /* 接收帧计数 */
    volatile uint32_t error_count;   /* 错误计数 */
} MB_Context_t;

/* 系统时间（100µs单位，TIM1中断中累加） */
extern volatile int64_t sys_time_100us;

/* 获取系统时间（原子读取） */
static inline int64_t SYS_GetTime100us(void) {
    int64_t t;
    __disable_irq();
    t = sys_time_100us;
    __enable_irq();
    return t;
}

/* 获取系统时间（ms单位） */
static inline int64_t SYS_GetTimeMs(void) {
    return SYS_GetTime100us() / 10;
}

/* 函数声明 */
void MODBUS_Init(MB_Context_t *ctx, uint8_t slave_addr, MotorControl_t *motor);
void MODBUS_SyncAccParams(void);  /* 开机后同步加速表参数到MODBUS缓存 */
void MODBUS_OnByteReceived(MB_Context_t *ctx, uint8_t byte);   /* UART中断中调用 */
void MODBUS_OnTimeoutTick(MB_Context_t *ctx);                  /* TIM1中断中调用 */
void MODBUS_Process(MB_Context_t *ctx);                        /* 主循环中调用 */
void MODBUS_SendResponse(MB_Context_t *ctx, uint8_t *data, uint8_t len);
uint8_t MODBUS_IsSupportedBaud(uint32_t baud_rate);
uint16_t MODBUS_EncodeBaud(uint32_t baud_rate);
uint32_t MODBUS_DecodeBaud(uint16_t value);
uint16_t MODBUS_GetFrameTimeoutTicks(uint32_t baud_rate);
HAL_StatusTypeDef MODBUS_ApplyBaudRate(MB_Context_t *ctx, uint32_t baud_rate);
void MODBUS_SyncTargetPosition(int64_t position);  /* 外部模块同步目标位置（脉冲输入等） */
void MODBUS_TriggerStartFlag(void);  /* 触发启动标志位(等同于写REG_START_MODE=1), 供PB4/PB5硬件触发调用 */

#endif /* __MODBUS_H */
