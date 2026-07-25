#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MODBUS RTU 调试工具
用于电机控制器的参数配置和状态监控
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox, filedialog
import serial
import serial.tools.list_ports
import struct
import threading
import time
import csv

import matplotlib
matplotlib.use('TkAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

class CRC16:
    """MODBUS CRC16计算"""
    @staticmethod
    def calculate(data):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

class ModbusRTU:
    """MODBUS RTU协议实现"""
    def __init__(self, serial_port, raw_tx_callback=None, raw_rx_callback=None):
        self.serial = serial_port
        self.raw_tx_callback = raw_tx_callback
        self.raw_rx_callback = raw_rx_callback

    def _clear_buffer(self):
        """清空输入缓冲区，防止数据错位"""
        try:
            self.serial.reset_input_buffer()
        except Exception:
            pass

    def read_holding_registers(self, slave_addr, start_addr, quantity):
        """读取保持寄存器 (功能码03)"""
        # 发送前清空缓冲区，防止残留数据导致错位
        self._clear_buffer()

        request = [slave_addr, 0x03]
        request.extend(struct.pack('>HH', start_addr, quantity))
        crc = CRC16.calculate(request)
        request.extend([crc & 0xFF, (crc >> 8) & 0xFF])

        request_bytes = bytes(request)
        if self.raw_tx_callback:
            self.raw_tx_callback(request_bytes)

        self.serial.write(request_bytes)
        time.sleep(0.05)

        # 计算期望的响应长度：地址(1)+功能码(1)+字节数(1)+数据(2*quantity)+CRC(2)
        expected_len = 5 + 2 * quantity

        # 一次性读取期望长度的数据
        response = self.serial.read(expected_len)
        if self.raw_rx_callback:
            self.raw_rx_callback(response)
        if len(response) < expected_len:
            self._clear_buffer()  # 数据不足，清空缓冲区防止错位
            raise Exception(f"响应超时或数据不足(期望{expected_len}字节,实际{len(response)}字节)")

        # 验证响应帧头：从机地址和功能码必须匹配请求
        if response[0] != slave_addr or response[1] != 0x03:
            self._clear_buffer()  # 帧头不匹配，清空缓冲区
            raise Exception(f"响应帧头错误(地址:{response[0]:02X},功能码:{response[1]:02X})")

        # 验证CRC
        recv_crc = response[-2] | (response[-1] << 8)
        calc_crc = CRC16.calculate(response[:-2])
        if recv_crc != calc_crc:
            self._clear_buffer()  # CRC错误，清空缓冲区
            raise Exception("CRC校验错误")

        # 检查异常响应
        if response[1] & 0x80:
            raise Exception(f"从机异常: 功能码 {response[1]:02X}, 异常码 {response[2]:02X}")

        # 解析数据
        byte_count = response[2]
        data = []
        for i in range(byte_count // 2):
            value = (response[3 + i*2] << 8) | response[4 + i*2]
            data.append(value)

        return data

    def write_single_register(self, slave_addr, addr, value):
        """写单个寄存器 (功能码06)"""
        # 发送前清空缓冲区，防止残留数据导致错位
        self._clear_buffer()

        # 处理负数：转换为16位补码
        if value < 0:
            value = value + 0x10000

        request = [slave_addr, 0x06]
        request.extend(struct.pack('>HH', addr, value))
        crc = CRC16.calculate(request)
        request.extend([crc & 0xFF, (crc >> 8) & 0xFF])

        request_bytes = bytes(request)
        if self.raw_tx_callback:
            self.raw_tx_callback(request_bytes)

        self.serial.write(request_bytes)
        time.sleep(0.05)

        # 一次性读取8字节响应
        response = self.serial.read(8)
        if self.raw_rx_callback:
            self.raw_rx_callback(response)
        if len(response) < 8:
            self._clear_buffer()
            raise Exception(f"响应超时或数据不足(期望8字节,实际{len(response)}字节)")

        # 验证响应帧头
        if response[0] != slave_addr or response[1] != 0x06:
            self._clear_buffer()
            raise Exception(f"响应帧头错误(地址:{response[0]:02X},功能码:{response[1]:02X})")

        # 验证CRC
        recv_crc = response[-2] | (response[-1] << 8)
        calc_crc = CRC16.calculate(response[:-2])
        if recv_crc != calc_crc:
            self._clear_buffer()
            raise Exception("CRC校验错误")

        # 检查异常响应
        if response[1] & 0x80:
            raise Exception(f"从机异常: 功能码 {response[1]:02X}, 异常码 {response[2]:02X}")

        return True

    def write_multiple_registers(self, slave_addr, start_addr, values):
        """写多个寄存器 (功能码16)"""
        # 发送前清空缓冲区，防止残留数据导致错位
        self._clear_buffer()

        quantity = len(values)
        byte_count = quantity * 2

        request = [slave_addr, 0x10]
        request.extend(struct.pack('>HH', start_addr, quantity))
        request.append(byte_count)

        for value in values:
            # 处理负数：转换为16位补码
            if value < 0:
                value = value + 0x10000
            request.extend(struct.pack('>H', value))

        crc = CRC16.calculate(request)
        request.extend([crc & 0xFF, (crc >> 8) & 0xFF])

        request_bytes = bytes(request)
        if self.raw_tx_callback:
            self.raw_tx_callback(request_bytes)

        self.serial.write(request_bytes)
        time.sleep(0.05)

        # 一次性读取8字节响应
        response = self.serial.read(8)
        if self.raw_rx_callback:
            self.raw_rx_callback(response)
        if len(response) < 8:
            self._clear_buffer()
            raise Exception(f"响应超时或数据不足(期望8字节,实际{len(response)}字节)")

        # 验证响应帧头
        if response[0] != slave_addr or response[1] != 0x10:
            self._clear_buffer()
            raise Exception(f"响应帧头错误(地址:{response[0]:02X},功能码:{response[1]:02X})")

        # 验证CRC
        recv_crc = response[-2] | (response[-1] << 8)
        calc_crc = CRC16.calculate(response[:-2])
        if recv_crc != calc_crc:
            self._clear_buffer()
            raise Exception("CRC校验错误")

        # 检查异常响应
        if response[1] & 0x80:
            raise Exception(f"从机异常: 功能码 {response[1]:02X}, 异常码 {response[2]:02X}")

        return True

class ModbusDebuggerApp:
    """MODBUS调试工具主界面"""
    
    # 寄存器地址定义
    REG_CONTROL_WORD = 0x0000    # 模式设置
    REG_MODE_SET = 0x0001        # 启动方式 (0=直接启动, 1=标志位启动)
    REG_TARGET_POS_H3 = 0x0002
    REG_TARGET_POS_H2 = 0x0003
    REG_TARGET_POS_L2 = 0x0004
    REG_TARGET_POS_L1 = 0x0005
    REG_TARGET_SPEED_H = 0x0006
    REG_TARGET_SPEED_L = 0x0007
    REG_POS_KP = 0x0008
    REG_POS_KI = 0x0009
    REG_POS_KD = 0x000A
    REG_SPD_KP = 0x000B
    REG_SPD_KI = 0x000C
    REG_SPD_KD = 0x000D
    REG_DEAD_ZONE = 0x000E
    REG_MAX_OUTPUT = 0x000F
    REG_START_MODE = 0x0010      # 启动触发 (写1启动)
    REG_DIRECTION = 0x0011        # 电机方向 (0=正转, 1=反转)
    REG_ENCODER_DIRECTION = 0x0012  # 编码器方向 (0=正常, 1=反转)
    REG_START_FLAG = 0x0013       # 保留寄存器
    REG_SLAVE_ADDR = 0x0014       # MODBUS从机地址 (1~247)
    REG_BAUD_RATE = 0x0015        # MODBUS波特率 (波特率/100)
    REG_SET_ORIGIN = 0x0016       # 设置当前位置为原点 (写1触发)
    REG_HOME_MODE = 0x0017        # 复位模式 (0=关闭, 1=堵转复位)
    REG_HOME_DIRECTION = 0x0018   # 复位方向 (0=负方向, 1=正方向)
    REG_HOME_CURRENT = 0x0019     # 复位电流限制(PWM值)
    REG_HOME_SPEED_H = 0x001A
    REG_HOME_SPEED_L = 0x001B
    REG_HOME_MAX_DISTANCE_H = 0x001C
    REG_HOME_MAX_DISTANCE_L = 0x001D
    REG_HOME_BACK_DISTANCE_H = 0x001E
    REG_HOME_BACK_DISTANCE_L = 0x001F
    REG_HOME_TRIGGER = 0x0020      # 执行一次复位 (写1触发)
    REG_HOME_AUTO_START = 0x0021   # 开机自动复位 (0=关闭, 1=开启)
    REG_INPUT1_FUNC = 0x0022       # PB4功能 (0=脉冲,1=方向,2=原点位置开关,3=限位,4=外部目标位置电平,5=外部目标速度电平,6=执行复位操作,7=无功能)
    REG_INPUT1_POLARITY = 0x0023   # PB4极性 (0=高有效,1=低有效)
    REG_INPUT1_LIMIT_DIR = 0x0024  # PB4限位方向 (0=正方向,1=负方向)
    REG_INPUT2_FUNC = 0x0025       # PB5功能
    REG_INPUT2_POLARITY = 0x0026   # PB5极性
    REG_INPUT2_LIMIT_DIR = 0x0027  # PB5限位方向
    REG_MAX_RUN_SPEED_H = 0x0028   # 最大运行速度高16位(0=无限制)
    REG_MAX_RUN_SPEED_L = 0x0029   # 最大运行速度低16位
    REG_INPUT1_TARGET_POS_H3 = 0x002A  # PB4目标位置电平预设位置 bit[63:48]
    REG_INPUT1_TARGET_POS_H2 = 0x002B
    REG_INPUT1_TARGET_POS_L2 = 0x002C
    REG_INPUT1_TARGET_POS_L1 = 0x002D
    REG_INPUT2_TARGET_POS_H3 = 0x002E  # PB5目标位置电平预设位置 bit[63:48]
    REG_INPUT2_TARGET_POS_H2 = 0x002F
    REG_INPUT2_TARGET_POS_L2 = 0x0030
    REG_INPUT2_TARGET_POS_L1 = 0x0031
    REG_HOME_PRECISION_SPEED_H = 0x0032
    REG_HOME_PRECISION_SPEED_L = 0x0033
    REG_HOME_PRECISION_CYCLES = 0x0034
    REG_INPUT1_TARGET_SPEED_H = 0x0035  # PB4外部目标速度高16位
    REG_INPUT1_TARGET_SPEED_L = 0x0036  # PB4外部目标速度低16位
    REG_INPUT2_TARGET_SPEED_H = 0x0037  # PB5外部目标速度高16位
    REG_INPUT2_TARGET_SPEED_L = 0x0038  # PB5外部目标速度低16位
    REG_SPEED_ACQ_START = 0x003A        # 转速采集启动/状态: 写1启动, 读=状态(0=空闲,1=采集中,2=完成)
    REG_SPEED_ACQ_DIV = 0x003B          # 转速采集分频值(1=100us, 50=5ms, 默认50)
    REG_SPEED_ACQ_COUNT = 0x003C        # 转速采集已采样数量(只读, 0~512)
    REG_SPEED_ACQ_STATUS = 0x003D       # 转速采集状态(只读)
    REG_SPEED_ACQ_TYPE = 0x003E         # 采集类型: 0=转速,1=PWM,2=位置,3=电流,4=PC0电压,5=PC2,6=PC3, 7~15=电流环开PID(位置/速度/电流各P/I/D), 16~21=电流环关PID(位置/速度各P/I/D)
    REG_SPEED_ACQ_SIZE = 0x003F         # 采集点数(1~5120)

    # 多通道采集寄存器(新增)
    REG_SPEED_ACQ_CHANNELS = 0x009C     # 采集通道数(1~4, 默认1)
    REG_SPEED_ACQ_TYPE2 = 0x009D        # 通道2采集类型
    REG_SPEED_ACQ_TYPE3 = 0x009E        # 通道3采集类型
    REG_SPEED_ACQ_TYPE4 = 0x009F        # 通道4采集类型

    REG_STALL_PROT_EN = 0x0040          # 堵转保护使能: 0=关闭, 1=开启
    REG_STALL_ERR_LIMIT = 0x0041        # 堵转误差阈值(位置=脉冲, 速度=脉冲/秒)
    REG_STALL_TIME = 0x0042             # 堵转持续时长(单位=PID周期5ms)
    REG_STALL_STATUS = 0x0043           # 堵转状态(只读): 0=正常, 1=已触发
    REG_STALL_RESET = 0x0044            # 堵转复位(写1清除)

    # 电流环寄存器 0x0045~0x004D (新增)
    # 硬件参考: INA240A1PWR(增益20V/V) + 10mΩ采样电阻
    #   总跨阻 0.2V/A, 3.3V ADC, 零电流=1.65V(ADC=2048)
    #   满量程 ±8.25A 对应相对电流 ±1000
    #   1A实际 ≈ 121相对单位
    REG_CUR_LOOP_EN = 0x0045            # 电流环使能: 0=关闭(原2环行为), 1=开启(3环级联)
    REG_CUR_KP = 0x0046                 # 电流环Kp(×100, 有符号)
    REG_CUR_KI = 0x0047                 # 电流环Ki(×100, 有符号)
    REG_CUR_KD = 0x0048                 # 电流环Kd(×1000, 有符号)
    REG_CUR_OFFSET = 0x0049             # 零电流ADC原始值(0~4095, 默认2048)
    REG_CUR_SCALE = 0x004A              # ADC到相对电流转换系数(×10000, 默认4883≈0.4883)
    REG_OVER_CUR_LIMIT = 0x004B         # 过流保护阈值(相对值×10, 0=关闭; 1210=1A, 6060=5A)
    REG_OVER_CUR_STATUS = 0x004C        # 过流状态(只读): 0=正常, 1=已触发
    REG_OVER_CUR_RESET = 0x004D         # 过流复位(写1清除)
    REG_MAX_CURRENT = 0x0062            # 电流限幅(相对值×10, 0=不限制; 1210=1A, 6060=5A)
    # 电流环使能时使用的速度环/位置环PID参数 (0x0063~0x0068, 新增)
    # 仅在电流环使能(3环级联)时生效, 用于区分2环/3环模式下的不同PID整定
    REG_SPD_KP_CUR = 0x0063              # 电流环使能时速度环Kp(×100)
    REG_SPD_KI_CUR = 0x0064              # 电流环使能时速度环Ki(×100)
    REG_SPD_KD_CUR = 0x0065              # 电流环使能时速度环Kd(×1000)
    REG_POS_KP_CUR = 0x0066              # 电流环使能时位置环Kp(×100)
    REG_POS_KI_CUR = 0x0067              # 电流环使能时位置环Ki(×100)
    REG_POS_KD_CUR = 0x0068              # 电流环使能时位置环Kd(×1000)
    REG_CUR_DEAD_ZONE = 0x0069           # 电流死区(相对值×10, 0=关闭; 30=3.0≈25mA, 100=10.0≈83mA)
    REG_ADC_FILTER_SIZE = 0x006A         # 外部ADC滤波窗口(0~128, 0/1=关闭, 4~32推荐)

    # 步进电机加减速参数寄存器 (0x006B~0x0090, 新增)
    # 映射 bujindianji[0] 结构体, 每个int64字段占4个连续寄存器
    REG_STEP_MAX_SPEED_H3 = 0x006B      # 最大运行速度 bit[63:48] (读写)
    REG_STEP_MAX_SPEED_H2 = 0x006C
    REG_STEP_MAX_SPEED_L2 = 0x006D
    REG_STEP_MAX_SPEED_L1 = 0x006E
    REG_STEP_TARGET_POS_H3 = 0x0073      # 预定目标位置 bit[63:48] (读写)
    REG_STEP_TARGET_POS_H2 = 0x0074
    REG_STEP_TARGET_POS_L2 = 0x0075
    REG_STEP_TARGET_POS_L1 = 0x0076
    REG_STEP_CUR_POS_H3 = 0x008B         # 当前步位置 bit[63:48] (只读)
    REG_STEP_CUR_POS_H2 = 0x008C
    REG_STEP_CUR_POS_L2 = 0x008D
    REG_STEP_CUR_POS_L1 = 0x008E

    REG_STEP_ACCEL_EN = 0x0091             # 步进加减速使能: 0=关闭, 1=位置模式启用

    # 加速表计算寄存器 (0x0092~0x009A, 新增)
    REG_ACC_TABLE_TIMC_L = 0x0092          # 主频 低16位
    REG_ACC_TABLE_TIMC_H = 0x0093          # 主频 高16位
    REG_ACC_TABLE_U = 0x0094               # 最小速度
    REG_ACC_TABLE_A_L = 0x0095             # 加速度 低16位
    REG_ACC_TABLE_A_H = 0x0096             # 加速度 高16位
    REG_ACC_TABLE_LEN = 0x0097             # 表长度(最大10015)
    REG_ACC_TABLE_TRIGGER = 0x0098         # 触发(1=计算, 2=恢复ROM)
    REG_ACC_TABLE_STATUS = 0x0099          # 状态(只读)
    REG_ACC_TABLE_PROGRESS = 0x009A        # 进度(只读, 0~1000)

    # PC2/PC3 ADC功能寄存器 (0x004E~0x005D, 新增最小值)
    REG_PC2_FUNC = 0x004E               # PC2功能 (0=无,1=ADC转速,2=ADC位置转速,3=ADC开环,4=ADC位置)
    REG_PC3_FUNC = 0x004F               # PC3功能 (同上, PC2!=无时PC3被忽略)
    REG_ADC_MAX_SPEED_H = 0x0050        # ADC最大速度高16位(脉冲/秒)
    REG_ADC_MAX_SPEED_L = 0x0051        # ADC最大速度低16位
    REG_ADC_MAX_PWM = 0x0052            # ADC最大PWM(-1000~1000, 有符号)
    REG_ADC_MAX_POS_H3 = 0x0053         # ADC最大位置bit[63:48]
    REG_ADC_MAX_POS_H2 = 0x0054         # ADC最大位置bit[47:32]
    REG_ADC_MAX_POS_L2 = 0x0055         # ADC最大位置bit[31:16]
    REG_ADC_MAX_POS_L1 = 0x0056         # ADC最大位置bit[15:0]
    # ADC最小值寄存器 (0x0057~0x005D, 新增)
    # ADC值=0时对应最小值, ADC值=1时对应最大值, 最终输出=最小值+ADC比例×(最大值-最小值)
    REG_ADC_MIN_SPEED_H = 0x0057        # ADC最小速度高16位(脉冲/秒, 有符号)
    REG_ADC_MIN_SPEED_L = 0x0058        # ADC最小速度低16位
    REG_ADC_MIN_PWM = 0x0059            # ADC最小PWM(-1000~1000, 有符号)
    REG_ADC_MIN_POS_H3 = 0x005A         # ADC最小位置bit[63:48] (有符号)
    REG_ADC_MIN_POS_H2 = 0x005B         # ADC最小位置bit[47:32]
    REG_ADC_MIN_POS_L2 = 0x005C         # ADC最小位置bit[31:16]
    REG_ADC_MIN_POS_L1 = 0x005D         # ADC最小位置bit[15:0]
    # ADC死区寄存器 (0x005E~0x0061, 新增, 两个独立死区)
    # 每个死区可独立设置位置和宽度, ADC值落入任一死区时强制输出该位置对应的归一化值
    # 位置: 0=最小点, 1=中位点, 2=最大点; 宽度: 0~4095, 0=关闭
    REG_ADC_DEAD_ZONE1_POS = 0x005E     # ADC死区1位置 (0=最小点, 1=中位点, 2=最大点)
    REG_ADC_DEAD_ZONE1_WIDTH = 0x005F   # ADC死区1宽度 (0~4095, 0=关闭)
    REG_ADC_DEAD_ZONE2_POS = 0x0060     # ADC死区2位置 (0=最小点, 1=中位点, 2=最大点)
    REG_ADC_DEAD_ZONE2_WIDTH = 0x0061   # ADC死区2宽度 (0~4095, 0=关闭)

    REG_SPEED_DATA_BASE = 0x0200        # 采集数据起始地址
    REG_SPEED_DATA_END = 0x15FF         # 采集数据结束地址 (5120个寄存器, 10KB)
    SPEED_ACQ_BUF_SIZE = 5120           # 采集缓冲区大小
    
    REG_CURRENT_POS_H3 = 0x0100
    REG_CURRENT_POS_H2 = 0x0101
    REG_CURRENT_POS_L2 = 0x0102
    REG_CURRENT_POS_L1 = 0x0103
    REG_CURRENT_SPEED_H = 0x0104
    REG_CURRENT_SPEED_L = 0x0105
    REG_CURRENT_MODE = 0x0106
    REG_STATUS = 0x0107
    REG_CURRENT_PWM = 0x0108
    REG_PID_ERROR_H = 0x0109
    REG_PID_ERROR_L = 0x010A
    REG_PID_P_H = 0x010B
    REG_PID_P_L = 0x010C
    REG_PID_I_H = 0x010D
    REG_PID_I_L = 0x010E
    REG_PID_D_H = 0x010F
    REG_PID_D_L = 0x0110
    REG_CURRENT_ACTUAL = 0x0111          # 实际电流(相对值×10, 有符号, ±10000=±8.25A)
    REG_CURRENT_TARGET_RO = 0x0112       # 电流目标(相对值×10, 有符号)
    
    # 模式定义
    MODE_POSITION = 0
    MODE_SPEED = 1
    MODE_VELOCITY_POSITION = 2  # 基于位置的速度模式
    MODE_OPENLOOP = 3           # 开环模式
    MODE_EXTERNAL_TARGET = 4     # 外部目标位置模式
    MODE_EXTERNAL_TARGET_SPEED = 5  # 外部目标速度模式
    MODE_STANDBY = 6                # 待机模式
    MODE_ADC_SPEED = 7              # ADC转速模式
    MODE_ADC_POSITION_SPEED = 8     # ADC位置转速模式
    MODE_ADC_OPENLOOP = 9           # ADC开环模式
    MODE_ADC_POSITION = 10          # ADC位置模式

    # 寄存器描述表（用于"寄存器列表"窗口显示）
    # 格式: (地址, 寄存器名, 描述, 格式/单位)
    # 覆盖 0x0000~0x0068 全部控制寄存器
    REGISTER_LIST = [
        (0x0000, "REG_CONTROL_WORD", "模式设置", "0=位置,1=速度,2=位置速度,3=开环,4=外部位置,5=外部速度,6=待机,7=ADC转速,8=ADC位置转速,9=ADC开环,10=ADC位置"),
        (0x0001, "REG_MODE_SET", "启动方式", "0=直接启动,1=标志位启动"),
        (0x0002, "REG_TARGET_POS_H3", "目标位置[63:48]", "int64高位"),
        (0x0003, "REG_TARGET_POS_H2", "目标位置[47:32]", "int64"),
        (0x0004, "REG_TARGET_POS_L2", "目标位置[31:16]", "int64"),
        (0x0005, "REG_TARGET_POS_L1", "目标位置[15:0]", "int64低位"),
        (0x0006, "REG_TARGET_SPEED_H", "目标速度高16位", "int32高位"),
        (0x0007, "REG_TARGET_SPEED_L", "目标速度低16位", "int32低位"),
        (0x0008, "REG_POS_KP", "位置环Kp", "×100"),
        (0x0009, "REG_POS_KI", "位置环Ki", "×100"),
        (0x000A, "REG_POS_KD", "位置环Kd", "×1000"),
        (0x000B, "REG_SPD_KP", "速度环Kp", "×100"),
        (0x000C, "REG_SPD_KI", "速度环Ki", "×100"),
        (0x000D, "REG_SPD_KD", "速度环Kd", "×1000"),
        (0x000E, "REG_DEAD_ZONE", "死区", "脉冲/脉冲·秒⁻¹"),
        (0x000F, "REG_MAX_OUTPUT", "最大输出", "PWM限幅×10"),
        (0x0010, "REG_START_MODE", "启动触发", "写1触发(只写)"),
        (0x0011, "REG_DIRECTION", "电机方向", "0=正转,1=反转"),
        (0x0012, "REG_ENCODER_DIRECTION", "编码器方向", "0=正常,1=反转"),
        (0x0013, "REG_START_FLAG", "保留寄存器", "保留"),
        (0x0014, "REG_SLAVE_ADDR", "MODBUS从机地址", "1~247"),
        (0x0015, "REG_BAUD_RATE", "MODBUS波特率", "波特率/100"),
        (0x0016, "REG_SET_ORIGIN", "设置原点", "写1触发(只写)"),
        (0x0017, "REG_HOME_MODE", "复位模式", "0=关闭,1=堵转,2=精确复位"),
        (0x0018, "REG_HOME_DIRECTION", "复位方向", "0=负方向,1=正方向"),
        (0x0019, "REG_HOME_CURRENT", "复位电流限制", "PWM值"),
        (0x001A, "REG_HOME_SPEED_H", "复位速度高16位", "int32高位"),
        (0x001B, "REG_HOME_SPEED_L", "复位速度低16位", "int32低位"),
        (0x001C, "REG_HOME_MAX_DISTANCE_H", "复位最大距离高16位", "int32高位"),
        (0x001D, "REG_HOME_MAX_DISTANCE_L", "复位最大距离低16位", "int32低位"),
        (0x001E, "REG_HOME_BACK_DISTANCE_H", "复位偏置高16位", "int32高位"),
        (0x001F, "REG_HOME_BACK_DISTANCE_L", "复位偏置低16位", "int32低位"),
        (0x0020, "REG_HOME_TRIGGER", "执行复位触发", "写1触发(只写)"),
        (0x0021, "REG_HOME_AUTO_START", "开机自复位", "0=关闭,1=开启"),
        (0x0022, "REG_INPUT1_FUNC", "PB4功能", "0=脉冲,1=方向,2=原点开关,3=限位,4=外部位置,5=外部速度,6=执行复位,7=无功能,8=停止,9=启动触发"),
        (0x0023, "REG_INPUT1_POLARITY", "PB4极性", "0=高有效,1=低有效"),
        (0x0024, "REG_INPUT1_LIMIT_DIR", "PB4限位方向", "0=停止正方向,1=停止负方向"),
        (0x0025, "REG_INPUT2_FUNC", "PB5功能", "同PB4"),
        (0x0026, "REG_INPUT2_POLARITY", "PB5极性", "0=高有效,1=低有效"),
        (0x0027, "REG_INPUT2_LIMIT_DIR", "PB5限位方向", "0=停止正方向,1=停止负方向"),
        (0x0028, "REG_MAX_RUN_SPEED_H", "最大运行速度高16位", "0=无限制"),
        (0x0029, "REG_MAX_RUN_SPEED_L", "最大运行速度低16位", "脉冲/秒"),
        (0x002A, "REG_INPUT1_TARGET_POS_H3", "PB4目标位置[63:48]", "int64高位"),
        (0x002B, "REG_INPUT1_TARGET_POS_H2", "PB4目标位置[47:32]", "int64"),
        (0x002C, "REG_INPUT1_TARGET_POS_L2", "PB4目标位置[31:16]", "int64"),
        (0x002D, "REG_INPUT1_TARGET_POS_L1", "PB4目标位置[15:0]", "int64低位"),
        (0x002E, "REG_INPUT2_TARGET_POS_H3", "PB5目标位置[63:48]", "int64高位"),
        (0x002F, "REG_INPUT2_TARGET_POS_H2", "PB5目标位置[47:32]", "int64"),
        (0x0030, "REG_INPUT2_TARGET_POS_L2", "PB5目标位置[31:16]", "int64"),
        (0x0031, "REG_INPUT2_TARGET_POS_L1", "PB5目标位置[15:0]", "int64低位"),
        (0x0032, "REG_HOME_PRECISION_SPEED_H", "精确检测速度高16位", "int32高位"),
        (0x0033, "REG_HOME_PRECISION_SPEED_L", "精确检测速度低16位", "int32低位"),
        (0x0034, "REG_HOME_PRECISION_CYCLES", "精确检测次数", "次数"),
        (0x0035, "REG_INPUT1_TARGET_SPEED_H", "PB4目标速度高16位", "int32高位"),
        (0x0036, "REG_INPUT1_TARGET_SPEED_L", "PB4目标速度低16位", "int32低位"),
        (0x0037, "REG_INPUT2_TARGET_SPEED_H", "PB5目标速度高16位", "int32高位"),
        (0x0038, "REG_INPUT2_TARGET_SPEED_L", "PB5目标速度低16位", "int32低位"),
        (0x0039, "REG_RESERVED_0x0039", "保留寄存器", "保留"),
        (0x003A, "REG_SPEED_ACQ_START", "转速采集启动/状态", "写1启动,读=状态(0=空闲,1=采集中,2=完成)"),
        (0x003B, "REG_SPEED_ACQ_DIV", "转速采集分频值", "1=100us,50=5ms,默认50"),
        (0x003C, "REG_SPEED_ACQ_COUNT", "已采样数量(只读)", "0~512"),
        (0x003D, "REG_SPEED_ACQ_STATUS", "采集状态(只读)", "0=空闲,1=采集中,2=完成"),
        (0x003E, "REG_SPEED_ACQ_TYPE", "采集类型", "0=转速,1=PWM,2=位置,3=电流,4=PC0电压,5=PC2,6=PC3,7~15=电流环开PID(位置/速度/电流各P/I/D),16~21=电流环关PID(位置/速度各P/I/D)"),
        (0x003F, "REG_SPEED_ACQ_SIZE", "采集点数", "1~5120"),
        (0x0040, "REG_STALL_PROT_EN", "堵转保护使能", "0=关闭,1=开启"),
        (0x0041, "REG_STALL_ERR_LIMIT", "堵转误差阈值", "脉冲/脉冲·秒⁻¹"),
        (0x0042, "REG_STALL_TIME", "堵转持续时长", "×5ms"),
        (0x0043, "REG_STALL_STATUS", "堵转状态(只读)", "0=正常,1=已触发"),
        (0x0044, "REG_STALL_RESET", "堵转复位", "写1清除(只写)"),
        (0x0045, "REG_CUR_LOOP_EN", "电流环使能", "0=关闭(2环),1=开启(3环)"),
        (0x0046, "REG_CUR_KP", "电流环Kp", "×100"),
        (0x0047, "REG_CUR_KI", "电流环Ki", "×100"),
        (0x0048, "REG_CUR_KD", "电流环Kd", "×1000"),
        (0x0049, "REG_CUR_OFFSET", "零电流ADC值", "0~4095,默认2048"),
        (0x004A, "REG_CUR_SCALE", "ADC到电流转换系数", "×10000,默认4883"),
        (0x004B, "REG_OVER_CUR_LIMIT", "过流保护阈值", "相对值×10,0=关闭,1210=1A"),
        (0x004C, "REG_OVER_CUR_STATUS", "过流状态(只读)", "0=正常,1=已触发"),
        (0x004D, "REG_OVER_CUR_RESET", "过流复位", "写1清除(只写)"),
        (0x004E, "REG_PC2_FUNC", "PC2功能", "0=无,1=ADC转速,2=ADC位置转速,3=ADC开环,4=ADC位置"),
        (0x004F, "REG_PC3_FUNC", "PC3功能", "同上,PC2!=无时PC3被忽略"),
        (0x0050, "REG_ADC_MAX_SPEED_H", "ADC最大速度高16位", "脉冲/秒"),
        (0x0051, "REG_ADC_MAX_SPEED_L", "ADC最大速度低16位", "脉冲/秒"),
        (0x0052, "REG_ADC_MAX_PWM", "ADC最大PWM", "-1000~1000,有符号"),
        (0x0053, "REG_ADC_MAX_POS_H3", "ADC最大位置[63:48]", "int64高位"),
        (0x0054, "REG_ADC_MAX_POS_H2", "ADC最大位置[47:32]", "int64"),
        (0x0055, "REG_ADC_MAX_POS_L2", "ADC最大位置[31:16]", "int64"),
        (0x0056, "REG_ADC_MAX_POS_L1", "ADC最大位置[15:0]", "int64低位"),
        (0x0057, "REG_ADC_MIN_SPEED_H", "ADC最小速度高16位", "脉冲/秒,有符号"),
        (0x0058, "REG_ADC_MIN_SPEED_L", "ADC最小速度低16位", "脉冲/秒,有符号"),
        (0x0059, "REG_ADC_MIN_PWM", "ADC最小PWM", "-1000~1000,有符号"),
        (0x005A, "REG_ADC_MIN_POS_H3", "ADC最小位置[63:48]", "int64高位,有符号"),
        (0x005B, "REG_ADC_MIN_POS_H2", "ADC最小位置[47:32]", "int64,有符号"),
        (0x005C, "REG_ADC_MIN_POS_L2", "ADC最小位置[31:16]", "int64,有符号"),
        (0x005D, "REG_ADC_MIN_POS_L1", "ADC最小位置[15:0]", "int64低位,有符号"),
        (0x005E, "REG_ADC_DEAD_ZONE1_POS", "ADC死区1位置", "0=最小点,1=中位点,2=最大点"),
        (0x005F, "REG_ADC_DEAD_ZONE1_WIDTH", "ADC死区1宽度", "0~4095,0=关闭"),
        (0x0060, "REG_ADC_DEAD_ZONE2_POS", "ADC死区2位置", "0=最小点,1=中位点,2=最大点"),
        (0x0061, "REG_ADC_DEAD_ZONE2_WIDTH", "ADC死区2宽度", "0~4095,0=关闭"),
        (0x0062, "REG_MAX_CURRENT", "电流限幅", "相对值×10,0=不限,1210=1A"),
        (0x0063, "REG_SPD_KP_CUR", "电流模式速度Kp", "×100"),
        (0x0064, "REG_SPD_KI_CUR", "电流模式速度Ki", "×100"),
        (0x0065, "REG_SPD_KD_CUR", "电流模式速度Kd", "×1000"),
        (0x0066, "REG_POS_KP_CUR", "电流模式位置Kp", "×100"),
        (0x0067, "REG_POS_KI_CUR", "电流模式位置Ki", "×100"),
        (0x0068, "REG_POS_KD_CUR", "电流模式位置Kd", "×1000"),
        (0x0069, "REG_CUR_DEAD_ZONE", "电流死区", "相对值×10,0=关闭,30=25mA"),
        (0x006A, "REG_ADC_FILTER_SIZE", "ADC滤波窗口", "0~128,0/1=关闭,4~32推荐"),
        (0x006B, "REG_STEP_MAX_SPEED_H3", "步进最大速度[63:48]", "zuidasudu高位(读写)"),
        (0x006C, "REG_STEP_MAX_SPEED_H2", "步进最大速度[47:32]", ""),
        (0x006D, "REG_STEP_MAX_SPEED_L2", "步进最大速度[31:16]", ""),
        (0x006E, "REG_STEP_MAX_SPEED_L1", "步进最大速度[15:0]", "步/秒,一般≤10000"),
        (0x006F, "REG_STEP_MIN_SPEED_H3", "步进最小速度[63:48]", "zuixiaosudu高位(读写)"),
        (0x0070, "REG_STEP_MIN_SPEED_H2", "步进最小速度[47:32]", ""),
        (0x0071, "REG_STEP_MIN_SPEED_L2", "步进最小速度[31:16]", ""),
        (0x0072, "REG_STEP_MIN_SPEED_L1", "步进最小速度[15:0]", "步/秒,启动速度"),
        (0x0073, "REG_STEP_TARGET_POS_H3", "步进目标位置[63:48]", "yudingweizhi高位(读写)"),
        (0x0074, "REG_STEP_TARGET_POS_H2", "步进目标位置[47:32]", ""),
        (0x0075, "REG_STEP_TARGET_POS_L2", "步进目标位置[31:16]", ""),
        (0x0076, "REG_STEP_TARGET_POS_L1", "步进目标位置[15:0]", "步"),
        (0x0077, "REG_STEP_CUR_SPEED_H3", "步进当前速度[63:48]", "dangqiansudu高位(只读)"),
        (0x0078, "REG_STEP_CUR_SPEED_H2", "步进当前速度[47:32]", ""),
        (0x0079, "REG_STEP_CUR_SPEED_L2", "步进当前速度[31:16]", ""),
        (0x007A, "REG_STEP_CUR_SPEED_L1", "步进当前速度[15:0]", "步/秒"),
        (0x007B, "REG_STEP_PRESET_SPEED_H3", "步进预定速度[63:48]", "yudingsudu高位(只读)"),
        (0x007C, "REG_STEP_PRESET_SPEED_H2", "步进预定速度[47:32]", ""),
        (0x007D, "REG_STEP_PRESET_SPEED_L2", "步进预定速度[31:16]", ""),
        (0x007E, "REG_STEP_PRESET_SPEED_L1", "步进预定速度[15:0]", "步/秒"),
        (0x007F, "REG_STEP_ACCEL_CTRL_H3", "加减速控制[63:48]", "-1=减速,0=匀速,1=加速(只读)"),
        (0x0080, "REG_STEP_ACCEL_CTRL_H2", "加减速控制[47:32]", ""),
        (0x0081, "REG_STEP_ACCEL_CTRL_L2", "加减速控制[31:16]", ""),
        (0x0082, "REG_STEP_ACCEL_CTRL_L1", "加减速控制[15:0]", ""),
        (0x0083, "REG_STEP_STOP_POS_H3", "停止位置[63:48]", "tingzhiweizhi高位(只读)"),
        (0x0084, "REG_STEP_STOP_POS_H2", "停止位置[47:32]", ""),
        (0x0085, "REG_STEP_STOP_POS_L2", "停止位置[31:16]", ""),
        (0x0086, "REG_STEP_STOP_POS_L1", "停止位置[15:0]", "步"),
        (0x0087, "REG_STEP_REC_POS_H3", "外部中断记录位置[63:48]", "jiluweizhi高位(只读)"),
        (0x0088, "REG_STEP_REC_POS_H2", "外部中断记录位置[47:32]", ""),
        (0x0089, "REG_STEP_REC_POS_L2", "外部中断记录位置[31:16]", ""),
        (0x008A, "REG_STEP_REC_POS_L1", "外部中断记录位置[15:0]", "步"),
        (0x008B, "REG_STEP_CUR_POS_H3", "步进当前位置[63:48]", "dangqianweizhi高位(只读)"),
        (0x008C, "REG_STEP_CUR_POS_H2", "步进当前位置[47:32]", ""),
        (0x008D, "REG_STEP_CUR_POS_L2", "步进当前位置[31:16]", ""),
        (0x008E, "REG_STEP_CUR_POS_L1", "步进当前位置[15:0]", "步"),
        (0x008F, "REG_STEP_DIRECTION", "步进实际方向", "0=负方向,1=正方向(只读)"),
        (0x0090, "REG_STEP_TIMING", "步进定时器周期", "只读"),
        (0x0091, "REG_STEP_ACCEL_EN", "步进加减速使能", "0=关闭,1=位置模式启用"),
        (0x0092, "REG_ACC_TABLE_TIMC_L", "加速表主频L", "32位低16位"),
        (0x0093, "REG_ACC_TABLE_TIMC_H", "加速表主频H", "32位高16位"),
        (0x0094, "REG_ACC_TABLE_U", "加速表最小速度", "步/秒"),
        (0x0095, "REG_ACC_TABLE_A_L", "加速表加速度L", "32位低16位"),
        (0x0096, "REG_ACC_TABLE_A_H", "加速表加速度H", "32位高16位"),
        (0x0097, "REG_ACC_TABLE_LEN", "加速表长度", "最大10015"),
        (0x0098, "REG_ACC_TABLE_TRIGGER", "加速表触发", "1=计算,2=恢复ROM"),
        (0x0099, "REG_ACC_TABLE_STATUS", "加速表状态", "0=ROM,1=计算中,2=自定义,3=错误"),
        (0x009A, "REG_ACC_TABLE_PROGRESS", "加速表进度", "0~1000"),
        (0x009C, "REG_SPEED_ACQ_CHANNELS", "采集通道数", "1~4, 默认1(单通道)"),
        (0x009D, "REG_SPEED_ACQ_TYPE2", "通道2采集类型", "编码同0x003E"),
        (0x009E, "REG_SPEED_ACQ_TYPE3", "通道3采集类型", "编码同0x003E"),
        (0x009F, "REG_SPEED_ACQ_TYPE4", "通道4采集类型", "编码同0x003E"),
    ]

    # 写入全部寄存器时跳过的触发型/只读寄存器集合
    REGISTER_SKIP_ON_WRITE = {0x0010, 0x0016, 0x0020, 0x0044, 0x004D}

    
    def __init__(self, root):
        self.root = root
        self.root.title("MODBUS RTU 电机控制器调试工具")
        self.root.geometry("1280x800")
        
        self.serial = None
        self.modbus = None
        self.monitoring = False
        self.monitor_thread = None
        self.serial_lock = threading.Lock()  # 串口互斥锁
        self.acq_reading = False  # 采集数据读取标志（读取时暂停监控）
        self.device_baud_var = tk.StringVar(value="115200")
        self.baud_window = None
        self._raw_entries = 0
        
        # 状态监控可选项复选框变量（控制各状态项UI是否更新）
        self.monitor_options = {
            'position': tk.BooleanVar(value=True),
            'speed': tk.BooleanVar(value=True),
            'mode': tk.BooleanVar(value=True),
            'pwm': tk.BooleanVar(value=True),
            'pid': tk.BooleanVar(value=True),
            'current': tk.BooleanVar(value=True),
            'adc': tk.BooleanVar(value=True),
            'io': tk.BooleanVar(value=True),
            'flags': tk.BooleanVar(value=True),
            'step': tk.BooleanVar(value=True),
        }

        # 监控频率设置变量（控制monitor_loop的读取间隔）
        self.monitor_freq_var = tk.StringVar(value="100ms")

        # 寄存器列表窗口相关
        self.reg_list_win = None
        self.reg_list_tree = None
        
        self.create_widgets()
        
    def create_widgets(self):
        """创建界面组件"""
        # 顶部菜单栏
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)

        # 文件菜单
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="文件", menu=file_menu)
        file_menu.add_command(label="保存当前配置到文件", command=self.save_all_config_to_file)
        file_menu.add_command(label="从文件加载配置", command=self.load_all_config_from_file)
        file_menu.add_separator()
        file_menu.add_command(label="读取设备所有寄存器", command=self.read_device_all_registers)
        file_menu.add_command(label="打开寄存器列表", command=self.open_register_list)
        file_menu.add_separator()
        file_menu.add_command(label="加速表生成", command=self.open_acc_table_window)
        file_menu.add_separator()
        file_menu.add_command(label="退出", command=self.root.quit)

        # 帮助菜单
        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="帮助", menu=help_menu)
        help_menu.add_command(label="关于", command=lambda: messagebox.showinfo("关于", "MODBUS RTU 电机控制器调试工具\n\n用于STM32电机控制器的参数配置、状态监控和数据采集。\n支持双套PID(电压/电流模式)、电流环控制、配置导入导出。"))

        # 串口配置区域
        serial_frame = ttk.LabelFrame(self.root, text="串口配置", padding=10)
        serial_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(serial_frame, text="串口:").grid(row=0, column=0, padx=5)
        self.port_combo = ttk.Combobox(serial_frame, width=15, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=5)
        self.refresh_ports()
        
        ttk.Button(serial_frame, text="刷新", command=self.refresh_ports).grid(row=0, column=2, padx=5)
        
        ttk.Label(serial_frame, text="波特率:").grid(row=0, column=3, padx=5)
        self.baud_combo = ttk.Combobox(serial_frame, width=10, values=["9600", "19200", "38400", "57600", "115200"], state="readonly")
        self.baud_combo.set("115200")
        self.baud_combo.grid(row=0, column=4, padx=5)
        
        ttk.Label(serial_frame, text="从机地址:").grid(row=0, column=5, padx=5)
        self.slave_addr_var = tk.StringVar(value="1")
        ttk.Entry(serial_frame, textvariable=self.slave_addr_var, width=5).grid(row=0, column=6, padx=5)
        
        self.connect_btn = ttk.Button(serial_frame, text="连接", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=7, padx=5)

        ttk.Label(serial_frame, text="设备地址寄存器:").grid(row=1, column=0, padx=5, pady=5)
        self.device_addr_var = tk.StringVar(value="1")
        ttk.Entry(serial_frame, textvariable=self.device_addr_var, width=5).grid(row=1, column=1, padx=5, pady=5)
        ttk.Button(serial_frame, text="读取地址", command=self.read_device_addr).grid(row=1, column=2, padx=5, pady=5)
        ttk.Button(serial_frame, text="设置地址", command=self.set_device_addr).grid(row=1, column=3, padx=5, pady=5)
        ttk.Button(serial_frame, text="修改STM32波特率", command=self.open_baud_window).grid(row=1, column=4, padx=5, pady=5)
        ttk.Button(serial_frame, text="读取所有寄存器", command=self.read_all_registers_to_main).grid(row=1, column=5, padx=5, pady=5)

        # 主内容区域 - 使用PanedWindow实现可拖动分隔线
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        paned = ttk.PanedWindow(main_frame, orient=tk.HORIZONTAL)
        paned.pack(fill=tk.BOTH, expand=True)
        
        # 左侧控制面板（可滚动）
        left_container = ttk.Frame(paned)
        paned.add(left_container, weight=1)

        left_canvas = tk.Canvas(left_container, highlightthickness=0)
        left_scrollbar = ttk.Scrollbar(left_container, orient=tk.VERTICAL, command=left_canvas.yview)
        left_hscroll = ttk.Scrollbar(left_container, orient=tk.HORIZONTAL, command=left_canvas.xview)
        left_canvas.configure(yscrollcommand=left_scrollbar.set,
                              xscrollcommand=left_hscroll.set)

        left_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        left_hscroll.pack(side=tk.BOTTOM, fill=tk.X)
        left_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        left_frame = ttk.Frame(left_canvas)
        left_canvas_window = left_canvas.create_window((0, 0), window=left_frame, anchor=tk.NW)

        def _on_left_configure(event):
            left_canvas.configure(scrollregion=left_canvas.bbox("all"))

        def _on_canvas_configure(event):
            # 宽度: 内容比canvas宽时保持自然宽度(启用水平滚动), 否则填满canvas
            content_w = left_frame.winfo_reqwidth()
            left_canvas.itemconfig(left_canvas_window, width=max(content_w, event.width))

        left_frame.bind('<Configure>', _on_left_configure)
        left_canvas.bind('<Configure>', _on_canvas_configure)

        def _on_left_mousewheel(event):
            left_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        # 鼠标进入左侧区域时绑定滚轮
        def _bind_left_wheel(event):
            left_canvas.bind_all('<MouseWheel>', _on_left_mousewheel)

        def _unbind_left_wheel(event):
            # 检查鼠标是否真的离开了 canvas 区域
            # 避免进入子 widget（如 mode_frame）时误触发解绑
            x = event.x_root
            y = event.y_root
            cx = left_canvas.winfo_rootx()
            cy = left_canvas.winfo_rooty()
            cw = left_canvas.winfo_width()
            ch = left_canvas.winfo_height()
            if cx <= x < cx + cw and cy <= y < cy + ch:
                return
            left_canvas.unbind_all('<MouseWheel>')

        left_canvas.bind('<Enter>', _bind_left_wheel)
        left_canvas.bind('<Leave>', _unbind_left_wheel)
        
        # 模式控制
        mode_frame = ttk.LabelFrame(left_frame, text="模式控制", padding=10)
        mode_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(mode_frame, text="位置模式", command=lambda: self.set_mode(self.MODE_POSITION)).pack(fill=tk.X, pady=2)
        ttk.Button(mode_frame, text="速度模式", command=lambda: self.set_mode(self.MODE_SPEED)).pack(fill=tk.X, pady=2)
        ttk.Button(mode_frame, text="位置速度模式", command=lambda: self.set_mode(self.MODE_VELOCITY_POSITION)).pack(fill=tk.X, pady=2)
        ttk.Button(mode_frame, text="开环模式", command=lambda: self.set_mode(self.MODE_OPENLOOP)).pack(fill=tk.X, pady=2)
        ttk.Button(mode_frame, text="外部目标位置模式", command=lambda: self.set_mode(self.MODE_EXTERNAL_TARGET)).pack(fill=tk.X, pady=2)
        ttk.Button(mode_frame, text="外部目标速度模式", command=lambda: self.set_mode(self.MODE_EXTERNAL_TARGET_SPEED)).pack(fill=tk.X, pady=2)
        ttk.Button(mode_frame, text="待机模式", command=lambda: self.set_mode(self.MODE_STANDBY)).pack(fill=tk.X, pady=2)
        ttk.Button(mode_frame, text="ADC转速模式", command=lambda: self.set_mode(self.MODE_ADC_SPEED)).pack(fill=tk.X, pady=2)
        ttk.Button(mode_frame, text="ADC位置转速模式", command=lambda: self.set_mode(self.MODE_ADC_POSITION_SPEED)).pack(fill=tk.X, pady=2)
        ttk.Button(mode_frame, text="ADC开环模式", command=lambda: self.set_mode(self.MODE_ADC_OPENLOOP)).pack(fill=tk.X, pady=2)
        ttk.Button(mode_frame, text="ADC位置模式", command=lambda: self.set_mode(self.MODE_ADC_POSITION)).pack(fill=tk.X, pady=2)
        
        # 目标位置
        pos_frame = ttk.LabelFrame(left_frame, text="目标位置", padding=10)
        pos_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(pos_frame, text="位置值:").grid(row=0, column=0, padx=5)
        self.target_pos_var = tk.StringVar(value="0")
        ttk.Entry(pos_frame, textvariable=self.target_pos_var, width=15).grid(row=0, column=1, padx=5)
        ttk.Button(pos_frame, text="设置", command=self.set_target_position).grid(row=0, column=2, padx=5)
        ttk.Button(pos_frame, text="设当前位置为原点",
                   command=self.set_current_as_origin).grid(row=1, column=0, columnspan=3, pady=5)
        # 步进加减速使能勾选框(新增): 勾选后位置模式目标经过加减速算法平滑过渡
        # 勾选框右侧添加最大速度设置框(zuidasudu, 0x006B~0x006E)
        # 输入真实速度(步/秒), 上位机自动转为索引写入, 旁边显示当前速度和索引(从STM32读取)
        # 缓存加速表参数(u=最小速度, a=加速度), 用于索引↔速度转换
        self.acc_u = 500       # 默认最小速度(步/秒)
        self.acc_a = 320000    # 默认加速度
        self.step_accel_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(pos_frame, text="加减速", variable=self.step_accel_var,
                        command=self.toggle_step_accel).grid(row=2, column=0, pady=2, sticky=tk.W)
        ttk.Label(pos_frame, text="最大速度:").grid(row=2, column=1, padx=5, sticky=tk.E)
        self.step_max_speed_var = tk.StringVar(value="")
        ttk.Entry(pos_frame, textvariable=self.step_max_speed_var, width=10).grid(row=2, column=2, padx=5)
        ttk.Label(pos_frame, text="步/秒", foreground="gray").grid(row=2, column=3, padx=2, sticky=tk.W)
        ttk.Button(pos_frame, text="设置", command=self.set_step_max_speed).grid(row=2, column=4, padx=5)
        self.step_cur_index_label = ttk.Label(pos_frame, text="当前: --", foreground="blue")
        self.step_cur_index_label.grid(row=2, column=5, padx=5, sticky=tk.W)
        
        # 目标速度
        speed_frame = ttk.LabelFrame(left_frame, text="目标速度", padding=10)
        speed_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(speed_frame, text="速度值:").grid(row=0, column=0, padx=5)
        self.target_speed_var = tk.StringVar(value="0")
        ttk.Entry(speed_frame, textvariable=self.target_speed_var, width=15).grid(row=0, column=1, padx=5)
        ttk.Button(speed_frame, text="设置", command=self.set_target_speed).grid(row=0, column=2, padx=5)
        
        # PID参数
        pid_frame = ttk.LabelFrame(left_frame, text="PID参数", padding=10)
        pid_frame.pack(fill=tk.X, pady=5)
        
        # 启动时PID参数框全部留空，强制用户先点击"读取PID"再写入，避免误写入导致电机参数错乱
        ttk.Label(pid_frame, text="位置环 Kp:").grid(row=0, column=0, padx=5, pady=2)
        self.pos_kp_var = tk.StringVar(value="")
        ttk.Entry(pid_frame, textvariable=self.pos_kp_var, width=10).grid(row=0, column=1, padx=5, pady=2)

        ttk.Label(pid_frame, text="位置环 Ki:").grid(row=1, column=0, padx=5, pady=2)
        self.pos_ki_var = tk.StringVar(value="")
        ttk.Entry(pid_frame, textvariable=self.pos_ki_var, width=10).grid(row=1, column=1, padx=5, pady=2)

        ttk.Label(pid_frame, text="位置环 Kd:").grid(row=2, column=0, padx=5, pady=2)
        self.pos_kd_var = tk.StringVar(value="")
        ttk.Entry(pid_frame, textvariable=self.pos_kd_var, width=10).grid(row=2, column=1, padx=5, pady=2)

        ttk.Label(pid_frame, text="速度环 Kp:").grid(row=3, column=0, padx=5, pady=2)
        self.spd_kp_var = tk.StringVar(value="")
        ttk.Entry(pid_frame, textvariable=self.spd_kp_var, width=10).grid(row=3, column=1, padx=5, pady=2)

        ttk.Label(pid_frame, text="速度环 Ki:").grid(row=4, column=0, padx=5, pady=2)
        self.spd_ki_var = tk.StringVar(value="")
        ttk.Entry(pid_frame, textvariable=self.spd_ki_var, width=10).grid(row=4, column=1, padx=5, pady=2)

        ttk.Label(pid_frame, text="速度环 Kd:").grid(row=5, column=0, padx=5, pady=2)
        self.spd_kd_var = tk.StringVar(value="")
        ttk.Entry(pid_frame, textvariable=self.spd_kd_var, width=10).grid(row=5, column=1, padx=5, pady=2)
        
        ttk.Button(pid_frame, text="应用PID", command=self.apply_pid).grid(row=6, column=0, pady=5)
        ttk.Button(pid_frame, text="读取PID", command=self.read_pid).grid(row=6, column=1, pady=5)
        
        # 其他参数
        other_frame = ttk.LabelFrame(left_frame, text="其他参数", padding=10)
        other_frame.pack(fill=tk.X, pady=5)
        
        # 启动时其他参数框全部留空，强制用户先点击"读取参数"再写入，避免误写入导致电机参数错乱
        ttk.Label(other_frame, text="死区:").grid(row=0, column=0, padx=5, pady=2)
        self.dead_zone_var = tk.StringVar(value="")
        ttk.Entry(other_frame, textvariable=self.dead_zone_var, width=10).grid(row=0, column=1, padx=5, pady=2)

        ttk.Label(other_frame, text="最大输出:").grid(row=1, column=0, padx=5, pady=2)
        self.max_output_var = tk.StringVar(value="")
        ttk.Entry(other_frame, textvariable=self.max_output_var, width=10).grid(row=1, column=1, padx=5, pady=2)

        ttk.Label(other_frame, text="最大运行速度:").grid(row=2, column=0, padx=5, pady=2)
        self.max_run_speed_var = tk.StringVar(value="")
        ttk.Entry(other_frame, textvariable=self.max_run_speed_var, width=10).grid(row=2, column=1, padx=5, pady=2)
        ttk.Label(other_frame, text="0=无限制").grid(row=2, column=2, padx=5, pady=2)

        ttk.Label(other_frame, text="启动方式:").grid(row=3, column=0, padx=5, pady=2)
        self.start_mode_var = tk.StringVar(value="")
        self.start_mode_combo = ttk.Combobox(other_frame, textvariable=self.start_mode_var, width=8,
                      values=["直接启动", "标志位"], state="readonly")
        self.start_mode_combo.grid(row=3, column=1, padx=5, pady=2)
        self.start_btn = ttk.Button(other_frame, text="启动", command=self.start_motor)
        self.start_btn.grid(row=3, column=2, padx=5, pady=2)
        self.start_mode_combo.bind('<<ComboboxSelected>>', self._on_start_mode_changed)
        self._on_start_mode_changed(None)

        ttk.Label(other_frame, text="电机方向:").grid(row=4, column=0, padx=5, pady=2)
        self.motor_dir_var = tk.StringVar(value="")
        ttk.Combobox(other_frame, textvariable=self.motor_dir_var, width=8,
                     values=["正转", "反转"], state="readonly").grid(row=4, column=1, padx=5, pady=2)

        ttk.Label(other_frame, text="编码器方向:").grid(row=5, column=0, padx=5, pady=2)
        self.encoder_dir_var = tk.StringVar(value="")
        ttk.Combobox(other_frame, textvariable=self.encoder_dir_var, width=8,
                     values=["正常", "反转"], state="readonly").grid(row=5, column=1, padx=5, pady=2)

        ttk.Label(other_frame, text="复位模式:").grid(row=6, column=0, padx=5, pady=2)
        self.home_mode_var = tk.StringVar(value="")
        ttk.Combobox(other_frame, textvariable=self.home_mode_var, width=8,
                     values=["关闭", "堵转", "精确复位"], state="readonly").grid(row=6, column=1, padx=5, pady=2)

        ttk.Label(other_frame, text="复位方向:").grid(row=7, column=0, padx=5, pady=2)
        self.home_dir_var = tk.StringVar(value="")
        ttk.Combobox(other_frame, textvariable=self.home_dir_var, width=8,
                     values=["负方向", "正方向"], state="readonly").grid(row=7, column=1, padx=5, pady=2)

        ttk.Label(other_frame, text="复位电流:").grid(row=8, column=0, padx=5, pady=2)
        self.home_current_var = tk.StringVar(value="")
        ttk.Entry(other_frame, textvariable=self.home_current_var, width=10).grid(row=8, column=1, padx=5, pady=2)

        ttk.Label(other_frame, text="复位速度:").grid(row=9, column=0, padx=5, pady=2)
        self.home_speed_var = tk.StringVar(value="")
        ttk.Entry(other_frame, textvariable=self.home_speed_var, width=10).grid(row=9, column=1, padx=5, pady=2)

        ttk.Label(other_frame, text="精确检测速度:").grid(row=10, column=0, padx=5, pady=2)
        self.home_precision_speed_var = tk.StringVar(value="")
        ttk.Entry(other_frame, textvariable=self.home_precision_speed_var, width=10).grid(row=10, column=1, padx=5, pady=2)

        ttk.Label(other_frame, text="检测次数:").grid(row=11, column=0, padx=5, pady=2)
        self.home_precision_cycles_var = tk.StringVar(value="")
        ttk.Entry(other_frame, textvariable=self.home_precision_cycles_var, width=10).grid(row=11, column=1, padx=5, pady=2)

        ttk.Label(other_frame, text="最大距离:").grid(row=12, column=0, padx=5, pady=2)
        self.home_max_distance_var = tk.StringVar(value="")
        ttk.Entry(other_frame, textvariable=self.home_max_distance_var, width=10).grid(row=12, column=1, padx=5, pady=2)

        ttk.Label(other_frame, text="复位偏置:").grid(row=13, column=0, padx=5, pady=2)
        self.home_back_distance_var = tk.StringVar(value="")
        ttk.Entry(other_frame, textvariable=self.home_back_distance_var, width=10).grid(row=13, column=1, padx=5, pady=2)

        ttk.Label(other_frame, text="开机自复位:").grid(row=14, column=0, padx=5, pady=2)
        self.home_auto_start_var = tk.StringVar(value="")
        ttk.Combobox(other_frame, textvariable=self.home_auto_start_var, width=8,
                     values=["关闭", "开启"], state="readonly").grid(row=14, column=1, padx=5, pady=2)

        ttk.Button(other_frame, text="读取参数", command=self.read_other_params).grid(row=16, column=0, pady=5)
        ttk.Button(other_frame, text="应用", command=self.apply_other_params).grid(row=16, column=1, pady=5)
        ttk.Button(other_frame, text="恢复默认", command=self.reset_config).grid(row=16, column=2, pady=5)
        ttk.Button(other_frame, text="执行复位", command=self.trigger_homing).grid(row=17, column=0, columnspan=3, pady=5)

        # PB4/PB5 引脚配置
        pin_frame = ttk.LabelFrame(left_frame, text="PB4/PB5 引脚配置", padding=10)
        pin_frame.pack(fill=tk.X, pady=5)

        ttk.Label(pin_frame, text="引脚").grid(row=0, column=0, padx=5, pady=2)
        ttk.Label(pin_frame, text="功能").grid(row=0, column=1, padx=5, pady=2)
        ttk.Label(pin_frame, text="有效电平").grid(row=0, column=2, padx=5, pady=2)
        ttk.Label(pin_frame, text="限位方向").grid(row=0, column=3, padx=5, pady=2)
        ttk.Label(pin_frame, text="电平目标位置").grid(row=0, column=4, padx=5, pady=2)
        ttk.Label(pin_frame, text="电平目标速度").grid(row=0, column=5, padx=5, pady=2)

        # 启动时引脚配置框全部留空，强制用户先点击"读取引脚配置"再写入，避免误写入导致电机参数错乱
        ttk.Label(pin_frame, text="PB4").grid(row=1, column=0, padx=5, pady=2)
        self.pin4_func_var = tk.StringVar(value="")
        ttk.Combobox(pin_frame, textvariable=self.pin4_func_var, width=14,
                     values=["脉冲", "方向", "原点位置开关", "限位开关", "外部目标位置", "外部目标速度", "执行复位操作", "无功能", "停止", "启动标志位触发"], state="readonly").grid(row=1, column=1, padx=2, pady=2)
        self.pin4_pol_var = tk.StringVar(value="")
        ttk.Combobox(pin_frame, textvariable=self.pin4_pol_var, width=8,
                     values=["高电平", "低电平"], state="readonly").grid(row=1, column=2, padx=2, pady=2)
        self.pin4_ldir_var = tk.StringVar(value="")
        ttk.Combobox(pin_frame, textvariable=self.pin4_ldir_var, width=10,
                     values=["停止正方向", "停止负方向"], state="readonly").grid(row=1, column=3, padx=2, pady=2)
        self.pin4_target_pos_var = tk.StringVar(value="")
        ttk.Entry(pin_frame, textvariable=self.pin4_target_pos_var, width=14).grid(row=1, column=4, padx=2, pady=2)
        self.pin4_target_speed_var = tk.StringVar(value="")
        ttk.Entry(pin_frame, textvariable=self.pin4_target_speed_var, width=14).grid(row=1, column=5, padx=2, pady=2)

        ttk.Label(pin_frame, text="PB5").grid(row=2, column=0, padx=5, pady=2)
        self.pin5_func_var = tk.StringVar(value="")
        ttk.Combobox(pin_frame, textvariable=self.pin5_func_var, width=14,
                     values=["脉冲", "方向", "原点位置开关", "限位开关", "外部目标位置", "外部目标速度", "执行复位操作", "无功能", "停止", "启动标志位触发"], state="readonly").grid(row=2, column=1, padx=2, pady=2)
        self.pin5_pol_var = tk.StringVar(value="")
        ttk.Combobox(pin_frame, textvariable=self.pin5_pol_var, width=8,
                     values=["高电平", "低电平"], state="readonly").grid(row=2, column=2, padx=2, pady=2)
        self.pin5_ldir_var = tk.StringVar(value="")
        ttk.Combobox(pin_frame, textvariable=self.pin5_ldir_var, width=10,
                     values=["停止正方向", "停止负方向"], state="readonly").grid(row=2, column=3, padx=2, pady=2)
        self.pin5_target_pos_var = tk.StringVar(value="")
        ttk.Entry(pin_frame, textvariable=self.pin5_target_pos_var, width=14).grid(row=2, column=4, padx=2, pady=2)
        self.pin5_target_speed_var = tk.StringVar(value="")
        ttk.Entry(pin_frame, textvariable=self.pin5_target_speed_var, width=14).grid(row=2, column=5, padx=2, pady=2)

        btn_frame = ttk.Frame(pin_frame)
        btn_frame.grid(row=3, column=0, columnspan=6, pady=5)
        ttk.Button(btn_frame, text="读取引脚配置", command=self.read_pin_config).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="写入引脚配置", command=self.write_pin_config).pack(side=tk.LEFT, padx=5)

        # PC2/PC3 ADC功能配置
        adc_frame = ttk.LabelFrame(left_frame, text="PC2/PC3 ADC功能配置 (冲突时PC2优先)", padding=10)
        adc_frame.pack(fill=tk.X, pady=5)

        ttk.Label(adc_frame, text="引脚").grid(row=0, column=0, padx=5, pady=2)
        ttk.Label(adc_frame, text="功能").grid(row=0, column=1, padx=5, pady=2)

        ttk.Label(adc_frame, text="PC2-ADC1").grid(row=1, column=0, padx=5, pady=2)
        self.pc2_func_var = tk.StringVar(value="")
        ttk.Combobox(adc_frame, textvariable=self.pc2_func_var, width=14,
                     values=["无功能", "ADC转速模式", "ADC位置转速模式", "ADC开环模式", "ADC位置模式"], state="readonly").grid(row=1, column=1, padx=2, pady=2)

        ttk.Label(adc_frame, text="PC3-ADC2").grid(row=2, column=0, padx=5, pady=2)
        self.pc3_func_var = tk.StringVar(value="")
        ttk.Combobox(adc_frame, textvariable=self.pc3_func_var, width=14,
                     values=["无功能", "ADC转速模式", "ADC位置转速模式", "ADC开环模式", "ADC位置模式"], state="readonly").grid(row=2, column=1, padx=2, pady=2)

        # 公用预设值: 不同模式使用同一组值
        ttk.Label(adc_frame, text="最大速度(脉冲/秒):").grid(row=3, column=0, padx=5, pady=2, sticky=tk.W)
        self.adc_max_speed_var = tk.StringVar(value="")
        ttk.Entry(adc_frame, textvariable=self.adc_max_speed_var, width=14).grid(row=3, column=1, padx=2, pady=2, sticky=tk.W)
        ttk.Label(adc_frame, text="转速/位置转速模式有效").grid(row=3, column=2, padx=5, pady=2, sticky=tk.W)

        ttk.Label(adc_frame, text="最大PWM(-1000~1000):").grid(row=4, column=0, padx=5, pady=2, sticky=tk.W)
        self.adc_max_pwm_var = tk.StringVar(value="")
        ttk.Entry(adc_frame, textvariable=self.adc_max_pwm_var, width=14).grid(row=4, column=1, padx=2, pady=2, sticky=tk.W)
        ttk.Label(adc_frame, text="开环模式有效").grid(row=4, column=2, padx=5, pady=2, sticky=tk.W)

        ttk.Label(adc_frame, text="最大位置(脉冲):").grid(row=5, column=0, padx=5, pady=2, sticky=tk.W)
        self.adc_max_pos_var = tk.StringVar(value="")
        ttk.Entry(adc_frame, textvariable=self.adc_max_pos_var, width=14).grid(row=5, column=1, padx=2, pady=2, sticky=tk.W)
        ttk.Label(adc_frame, text="位置模式有效").grid(row=5, column=2, padx=5, pady=2, sticky=tk.W)

        # 最小值 (ADC=0时对应, 可为负数)
        ttk.Label(adc_frame, text="最小速度(脉冲/秒):").grid(row=6, column=0, padx=5, pady=2, sticky=tk.W)
        self.adc_min_speed_var = tk.StringVar(value="")
        ttk.Entry(adc_frame, textvariable=self.adc_min_speed_var, width=14).grid(row=6, column=1, padx=2, pady=2, sticky=tk.W)
        ttk.Label(adc_frame, text="转速/位置转速模式有效").grid(row=6, column=2, padx=5, pady=2, sticky=tk.W)

        ttk.Label(adc_frame, text="最小PWM(-1000~1000):").grid(row=7, column=0, padx=5, pady=2, sticky=tk.W)
        self.adc_min_pwm_var = tk.StringVar(value="")
        ttk.Entry(adc_frame, textvariable=self.adc_min_pwm_var, width=14).grid(row=7, column=1, padx=2, pady=2, sticky=tk.W)
        ttk.Label(adc_frame, text="开环模式有效").grid(row=7, column=2, padx=5, pady=2, sticky=tk.W)

        ttk.Label(adc_frame, text="最小位置(脉冲):").grid(row=8, column=0, padx=5, pady=2, sticky=tk.W)
        self.adc_min_pos_var = tk.StringVar(value="")
        ttk.Entry(adc_frame, textvariable=self.adc_min_pos_var, width=14).grid(row=8, column=1, padx=2, pady=2, sticky=tk.W)
        ttk.Label(adc_frame, text="位置模式有效").grid(row=8, column=2, padx=5, pady=2, sticky=tk.W)

        # ADC死区1配置
        ttk.Label(adc_frame, text="死区1位置:").grid(row=9, column=0, padx=5, pady=2, sticky=tk.W)
        self.adc_dz1_pos_var = tk.StringVar(value="")
        ttk.Combobox(adc_frame, textvariable=self.adc_dz1_pos_var, width=12,
                     values=["最小点", "中位点", "最大点"], state="readonly").grid(row=9, column=1, padx=2, pady=2, sticky=tk.W)
        ttk.Label(adc_frame, text="ADC=0/2048/4095附近停转").grid(row=9, column=2, padx=5, pady=2, sticky=tk.W)

        ttk.Label(adc_frame, text="死区1宽度(0~4095):").grid(row=10, column=0, padx=5, pady=2, sticky=tk.W)
        self.adc_dz1_width_var = tk.StringVar(value="")
        ttk.Entry(adc_frame, textvariable=self.adc_dz1_width_var, width=14).grid(row=10, column=1, padx=2, pady=2, sticky=tk.W)
        ttk.Label(adc_frame, text="0=关闭").grid(row=10, column=2, padx=5, pady=2, sticky=tk.W)

        # ADC死区2配置
        ttk.Label(adc_frame, text="死区2位置:").grid(row=11, column=0, padx=5, pady=2, sticky=tk.W)
        self.adc_dz2_pos_var = tk.StringVar(value="")
        ttk.Combobox(adc_frame, textvariable=self.adc_dz2_pos_var, width=12,
                     values=["最小点", "中位点", "最大点"], state="readonly").grid(row=11, column=1, padx=2, pady=2, sticky=tk.W)
        ttk.Label(adc_frame, text="ADC=0/2048/4095附近停转").grid(row=11, column=2, padx=5, pady=2, sticky=tk.W)

        ttk.Label(adc_frame, text="死区2宽度(0~4095):").grid(row=12, column=0, padx=5, pady=2, sticky=tk.W)
        self.adc_dz2_width_var = tk.StringVar(value="")
        ttk.Entry(adc_frame, textvariable=self.adc_dz2_width_var, width=14).grid(row=12, column=1, padx=2, pady=2, sticky=tk.W)
        ttk.Label(adc_frame, text="0=关闭").grid(row=12, column=2, padx=5, pady=2, sticky=tk.W)

        # 外部ADC滤波(新增): 滑动平均窗口大小, 作用于PC2/PC3外部ADC通道
        # 应用在死区处理之前, 先滤波再判断死区, 避免噪声在死区边缘频繁触发
        # 主循环5ms周期下: 4点≈20ms, 16点≈80ms, 64点≈320ms, 100点≈500ms, 128点≈640ms(最大)
        ttk.Label(adc_frame, text="ADC滤波窗口:").grid(row=13, column=0, padx=5, pady=2, sticky=tk.W)
        self.adc_filter_size_var = tk.StringVar(value="")
        ttk.Entry(adc_frame, textvariable=self.adc_filter_size_var, width=14).grid(row=13, column=1, padx=2, pady=2, sticky=tk.W)
        ttk.Label(adc_frame, text="0/1=关闭,2~128平均,100≈500ms",
                  foreground="blue").grid(row=13, column=2, padx=5, pady=2, sticky=tk.W)

        adc_btn_frame = ttk.Frame(adc_frame)
        adc_btn_frame.grid(row=14, column=0, columnspan=3, pady=5)
        ttk.Button(adc_btn_frame, text="读取ADC配置", command=self.read_adc_config).pack(side=tk.LEFT, padx=5)
        ttk.Button(adc_btn_frame, text="写入ADC配置", command=self.write_adc_config).pack(side=tk.LEFT, padx=5)

        # === 堵转保护配置 ===
        stall_frame = ttk.LabelFrame(left_frame, text="堵转保护", padding=10)
        stall_frame.pack(fill=tk.X, pady=5)

        # 启动时堵转保护框全部留空，强制用户先点击"读取参数"再写入，避免误写入导致电机参数错乱
        ttk.Label(stall_frame, text="使能:").grid(row=0, column=0, padx=5, pady=2, sticky=tk.W)
        self.stall_en_var = tk.StringVar(value="")
        ttk.Combobox(stall_frame, textvariable=self.stall_en_var, width=6,
                     values=["关闭", "开启"], state="readonly").grid(row=0, column=1, padx=5, pady=2, sticky=tk.W)

        ttk.Label(stall_frame, text="误差阈值:").grid(row=0, column=2, padx=5, pady=2, sticky=tk.W)
        self.stall_err_var = tk.StringVar(value="")
        ttk.Entry(stall_frame, textvariable=self.stall_err_var, width=8).grid(row=0, column=3, padx=5, pady=2, sticky=tk.W)
        ttk.Label(stall_frame, text="(脉冲/脉冲·秒⁻¹)").grid(row=0, column=4, padx=2, pady=2, sticky=tk.W)

        ttk.Label(stall_frame, text="持续时长:").grid(row=1, column=0, padx=5, pady=2, sticky=tk.W)
        self.stall_time_var = tk.StringVar(value="")
        ttk.Entry(stall_frame, textvariable=self.stall_time_var, width=8).grid(row=1, column=1, padx=5, pady=2, sticky=tk.W)
        ttk.Label(stall_frame, text="(×5ms = 1.0s)").grid(row=1, column=2, columnspan=3, padx=2, pady=2, sticky=tk.W)

        stall_btn_frame = ttk.Frame(stall_frame)
        stall_btn_frame.grid(row=2, column=0, columnspan=5, pady=5)
        ttk.Button(stall_btn_frame, text="读取参数", command=self.read_stall_config).pack(side=tk.LEFT, padx=5)
        ttk.Button(stall_btn_frame, text="写入参数", command=self.write_stall_config).pack(side=tk.LEFT, padx=5)
        ttk.Button(stall_btn_frame, text="复位堵转", command=self.reset_stall).pack(side=tk.LEFT, padx=5)

        # === 电流环配置 ===
        cur_frame = ttk.LabelFrame(left_frame, text="电流环 (INA240A1PWR + 10mΩ)", padding=10)
        cur_frame.pack(fill=tk.X, pady=5)

        # 启动时电流环参数框全部留空, 强制先读取再写入, 避免误覆盖
        ttk.Label(cur_frame, text="电流环使能:").grid(row=0, column=0, padx=5, pady=2, sticky=tk.W)
        self.cur_loop_en_var = tk.StringVar(value="")
        ttk.Combobox(cur_frame, textvariable=self.cur_loop_en_var, width=8,
                     values=["关闭", "开启"], state="readonly").grid(row=0, column=1, padx=5, pady=2, sticky=tk.W)
        ttk.Label(cur_frame, text="(关闭=2环, 开启=3环)",
                  foreground="blue").grid(row=0, column=2, padx=5, pady=2, sticky=tk.W)

        ttk.Label(cur_frame, text="Kp:").grid(row=1, column=0, padx=5, pady=2, sticky=tk.W)
        self.cur_kp_var = tk.StringVar(value="")
        ttk.Entry(cur_frame, textvariable=self.cur_kp_var, width=10).grid(row=1, column=1, padx=5, pady=2, sticky=tk.W)
        ttk.Label(cur_frame, text="Ki:").grid(row=1, column=2, padx=5, pady=2, sticky=tk.W)
        self.cur_ki_var = tk.StringVar(value="")
        ttk.Entry(cur_frame, textvariable=self.cur_ki_var, width=10).grid(row=1, column=3, padx=5, pady=2, sticky=tk.W)
        ttk.Label(cur_frame, text="Kd:").grid(row=1, column=4, padx=5, pady=2, sticky=tk.W)
        self.cur_kd_var = tk.StringVar(value="")
        ttk.Entry(cur_frame, textvariable=self.cur_kd_var, width=10).grid(row=1, column=5, padx=5, pady=2, sticky=tk.W)

        ttk.Label(cur_frame, text="零点ADC:").grid(row=2, column=0, padx=5, pady=2, sticky=tk.W)
        self.cur_offset_var = tk.StringVar(value="")
        ttk.Entry(cur_frame, textvariable=self.cur_offset_var, width=10).grid(row=2, column=1, padx=5, pady=2, sticky=tk.W)
        ttk.Label(cur_frame, text="默认2048",
                  foreground="gray").grid(row=2, column=2, padx=5, pady=2, sticky=tk.W)

        ttk.Label(cur_frame, text="标定系数:").grid(row=2, column=3, padx=5, pady=2, sticky=tk.W)
        self.cur_scale_var = tk.StringVar(value="")
        ttk.Entry(cur_frame, textvariable=self.cur_scale_var, width=10).grid(row=2, column=4, padx=5, pady=2, sticky=tk.W)
        ttk.Label(cur_frame, text="默认0.4883",
                  foreground="gray").grid(row=2, column=5, padx=5, pady=2, sticky=tk.W)

        ttk.Label(cur_frame, text="过流阈值:").grid(row=3, column=0, padx=5, pady=2, sticky=tk.W)
        self.over_cur_limit_var = tk.StringVar(value="")
        ttk.Entry(cur_frame, textvariable=self.over_cur_limit_var, width=10).grid(row=3, column=1, padx=5, pady=2, sticky=tk.W)
        ttk.Label(cur_frame, text="A (0=关闭, 5=5A)",
                  foreground="blue").grid(row=3, column=2, columnspan=2, padx=5, pady=2, sticky=tk.W)

        ttk.Label(cur_frame, text="电流限幅(A,0=不限):").grid(row=3, column=4, padx=5, pady=2, sticky=tk.W)
        self.max_current_var = tk.StringVar(value="")
        ttk.Entry(cur_frame, textvariable=self.max_current_var, width=10).grid(row=3, column=5, padx=5, pady=2, sticky=tk.W)

        # 电流环使能时的速度环PID参数 (0x0063~0x0065)
        # 仅在电流环使能(3环级联)时使用, Kp/Ki ×100, Kd ×1000, 有符号16位
        ttk.Label(cur_frame, text="电流模式速度Kp:").grid(row=4, column=0, padx=5, pady=2, sticky=tk.W)
        self.spd_kp_cur_var = tk.StringVar(value="")
        ttk.Entry(cur_frame, textvariable=self.spd_kp_cur_var, width=10).grid(row=4, column=1, padx=5, pady=2, sticky=tk.W)
        ttk.Label(cur_frame, text="Ki:").grid(row=4, column=2, padx=5, pady=2, sticky=tk.W)
        self.spd_ki_cur_var = tk.StringVar(value="")
        ttk.Entry(cur_frame, textvariable=self.spd_ki_cur_var, width=10).grid(row=4, column=3, padx=5, pady=2, sticky=tk.W)
        ttk.Label(cur_frame, text="Kd:").grid(row=4, column=4, padx=5, pady=2, sticky=tk.W)
        self.spd_kd_cur_var = tk.StringVar(value="")
        ttk.Entry(cur_frame, textvariable=self.spd_kd_cur_var, width=10).grid(row=4, column=5, padx=5, pady=2, sticky=tk.W)

        # 电流环使能时的位置环PID参数 (0x0066~0x0068)
        # 仅在电流环使能(3环级联)时使用, Kp/Ki ×100, Kd ×1000, 有符号16位
        ttk.Label(cur_frame, text="电流模式位置Kp:").grid(row=5, column=0, padx=5, pady=2, sticky=tk.W)
        self.pos_kp_cur_var = tk.StringVar(value="")
        ttk.Entry(cur_frame, textvariable=self.pos_kp_cur_var, width=10).grid(row=5, column=1, padx=5, pady=2, sticky=tk.W)
        ttk.Label(cur_frame, text="Ki:").grid(row=5, column=2, padx=5, pady=2, sticky=tk.W)
        self.pos_ki_cur_var = tk.StringVar(value="")
        ttk.Entry(cur_frame, textvariable=self.pos_ki_cur_var, width=10).grid(row=5, column=3, padx=5, pady=2, sticky=tk.W)
        ttk.Label(cur_frame, text="Kd:").grid(row=5, column=4, padx=5, pady=2, sticky=tk.W)
        self.pos_kd_cur_var = tk.StringVar(value="")
        ttk.Entry(cur_frame, textvariable=self.pos_kd_cur_var, width=10).grid(row=5, column=5, padx=5, pady=2, sticky=tk.W)

        # 电流死区(新增): 防止静止时细微电流, 实际电流<此值且目标=0时强制PWM=0
        # 单独一行, 避免与过流阈值/电流限幅同行导致宽度过大不便读取
        ttk.Label(cur_frame, text="电流死区(A):").grid(row=6, column=0, padx=5, pady=2, sticky=tk.W)
        self.cur_dead_zone_var = tk.StringVar(value="")
        ttk.Entry(cur_frame, textvariable=self.cur_dead_zone_var, width=10).grid(row=6, column=1, padx=5, pady=2, sticky=tk.W)
        ttk.Label(cur_frame, text="0=关闭, 防止静止时细微电流",
                  foreground="blue").grid(row=6, column=2, columnspan=4, padx=5, pady=2, sticky=tk.W)

        cur_btn_frame = ttk.Frame(cur_frame)
        cur_btn_frame.grid(row=7, column=0, columnspan=4, pady=5)
        ttk.Button(cur_btn_frame, text="读取参数", command=self.read_current_loop_config).pack(side=tk.LEFT, padx=5)
        ttk.Button(cur_btn_frame, text="写入参数", command=self.write_current_loop_config).pack(side=tk.LEFT, padx=5)
        ttk.Button(cur_btn_frame, text="过流复位", command=self.reset_over_current).pack(side=tk.LEFT, padx=5)
        ttk.Button(cur_btn_frame, text="标定零点", command=self.calibrate_current_offset).pack(side=tk.LEFT, padx=5)

        # 右侧：Canvas+滚动条 包裹 PanedWindow（可整体滚动 + 可调整各窗口大小）
        right_frame = ttk.Frame(paned)
        paned.add(right_frame, weight=2)

        right_canvas = tk.Canvas(right_frame, highlightthickness=0)
        right_scrollbar = ttk.Scrollbar(right_frame, orient=tk.VERTICAL, command=right_canvas.yview)
        right_hscroll = ttk.Scrollbar(right_frame, orient=tk.HORIZONTAL, command=right_canvas.xview)
        right_canvas.configure(yscrollcommand=right_scrollbar.set,
                               xscrollcommand=right_hscroll.set)
        right_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        right_hscroll.pack(side=tk.BOTTOM, fill=tk.X)
        right_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        right_inner = ttk.Frame(right_canvas)
        right_canvas_window = right_canvas.create_window((0, 0), window=right_inner, anchor=tk.NW)

        def _on_right_configure(event):
            right_canvas.configure(scrollregion=right_canvas.bbox("all"))

        def _on_right_canvas_configure(event):
            # 宽度: 内容比canvas宽时保持自然宽度(启用水平滚动), 否则填满canvas
            # 高度: 始终匹配内容自然高度, 不强制填满canvas高度
            # 这样3个窗口(实时状态/采集曲线/串口数据)的总高度=各自内容高度之和,
            # 内容多时启用垂直滚动, 内容少时下方留白, 用户可拖动sash重新分配各窗口高度
            content_w = right_inner.winfo_reqwidth()
            content_h = right_inner.winfo_reqheight()
            right_canvas.itemconfig(right_canvas_window,
                                    width=max(content_w, event.width),
                                    height=content_h)

        right_inner.bind('<Configure>', _on_right_configure)
        right_canvas.bind('<Configure>', _on_right_canvas_configure)

        def _on_right_mousewheel(event):
            right_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        # 鼠标在右侧区域时启用滚轮
        def _bind_wheel(event):
            right_canvas.bind_all('<MouseWheel>', _on_right_mousewheel)

        def _unbind_wheel(event):
            right_canvas.unbind_all('<MouseWheel>')

        right_canvas.bind('<Enter>', _bind_wheel)
        right_canvas.bind('<Leave>', _unbind_wheel)

        # 右侧3个窗口垂直堆叠, 每个窗口独立高度, 互不压缩
        # 用自定义sash手柄替代PanedWindow: 拖拽只改变上方窗口高度, 下方窗口不受影响
        # 总高度=3个窗口高度之和, 超出canvas时滚动
        right_stack = ttk.Frame(right_inner)
        right_stack.pack(fill=tk.X)

        def _make_v_sash(target_widget, min_h=40):
            """创建垂直拖拽手柄, 拖拽只改变target_widget的高度, 不影响其他窗口.
            target_widget: 要调整高度的Frame(需已pack_propagate(False))
            返回sash widget(调用方负责pack)"""
            sash = ttk.Frame(right_stack, height=6, cursor='sb_v_double_arrow',
                             relief=tk.RAISED)
            state = {'dragging': False, 'start_y': 0, 'start_h': 0}

            def on_press(event):
                state['dragging'] = True
                state['start_y'] = event.y_root
                state['start_h'] = target_widget.winfo_height()

            def on_drag(event):
                if not state['dragging']:
                    return
                delta = event.y_root - state['start_y']
                new_h = max(min_h, state['start_h'] + delta)
                target_widget.config(height=new_h)
                # matplotlib canvas需要手动刷新尺寸
                if hasattr(self, 'speed_canvas') and target_widget is self._chart_container:
                    self.speed_canvas.get_tk_widget().config(height=new_h - 60)
                    self.speed_fig.tight_layout()
                    self.speed_canvas.draw()
                # 强制更新canvas窗口高度以匹配内容自然高度,
                # 否则pack管理器会压缩下方窗口(串口数据)来适应旧的canvas窗口高度
                right_inner.update_idletasks()
                content_h = right_inner.winfo_reqheight()
                right_canvas.itemconfig(right_canvas_window, height=content_h)
                right_canvas.configure(scrollregion=right_canvas.bbox("all"))

            def on_release(event):
                state['dragging'] = False

            sash.bind('<ButtonPress-1>', on_press)
            sash.bind('<B1-Motion>', on_drag)
            sash.bind('<ButtonRelease-1>', on_release)
            return sash

        # === 实时状态（自然高度） ===
        status_frame = ttk.LabelFrame(right_stack, text="实时状态", padding=10)
        status_frame.pack(fill=tk.X)

        self.status_labels = {}
        # 状态项分组: key -> [标签Label, 值Label, ...] 用于批量显示/隐藏
        self.status_group_widgets = {k: [] for k in self.monitor_options.keys()}

        # 3列竖行布局：每列包含 label(column=N) + value(column=N+1)
        # 第1列：位置/速度/模式相关
        lbl = ttk.Label(status_frame, text="当前位置:")
        lbl.grid(row=0, column=0, padx=5, pady=2, sticky=tk.W)
        self.status_labels['position'] = ttk.Label(status_frame, text="0", foreground="blue", font=("Arial", 12, "bold"))
        self.status_labels['position'].grid(row=0, column=1, padx=5, pady=2, sticky=tk.W)
        self.status_group_widgets['position'].extend([lbl, self.status_labels['position']])

        lbl = ttk.Label(status_frame, text="目标位置:")
        lbl.grid(row=1, column=0, padx=5, pady=2, sticky=tk.W)
        self.status_labels['target_position'] = ttk.Label(status_frame, text="0", foreground="green", font=("Arial", 12, "bold"))
        self.status_labels['target_position'].grid(row=1, column=1, padx=5, pady=2, sticky=tk.W)
        self.status_group_widgets['position'].extend([lbl, self.status_labels['target_position']])

        lbl = ttk.Label(status_frame, text="当前速度:")
        lbl.grid(row=2, column=0, padx=5, pady=2, sticky=tk.W)
        self.status_labels['speed'] = ttk.Label(status_frame, text="0", foreground="blue", font=("Arial", 12, "bold"))
        self.status_labels['speed'].grid(row=2, column=1, padx=5, pady=2, sticky=tk.W)
        self.status_group_widgets['speed'].extend([lbl, self.status_labels['speed']])

        lbl = ttk.Label(status_frame, text="当前模式:")
        lbl.grid(row=3, column=0, padx=5, pady=2, sticky=tk.W)
        self.status_labels['mode'] = ttk.Label(status_frame, text="未知", foreground="blue", font=("Arial", 12, "bold"))
        self.status_labels['mode'].grid(row=3, column=1, padx=5, pady=2, sticky=tk.W)
        self.status_group_widgets['mode'].extend([lbl, self.status_labels['mode']])

        lbl = ttk.Label(status_frame, text="启动标志:")
        lbl.grid(row=4, column=0, padx=5, pady=2, sticky=tk.W)
        self.status_labels['start_flag'] = ttk.Label(status_frame, text="未设置", foreground="red", font=("Arial", 12, "bold"))
        self.status_labels['start_flag'].grid(row=4, column=1, padx=5, pady=2, sticky=tk.W)
        self.status_group_widgets['flags'].extend([lbl, self.status_labels['start_flag']])

        # 第2列：方向/PWM/状态相关
        lbl = ttk.Label(status_frame, text="电机方向:")
        lbl.grid(row=0, column=2, padx=15, pady=2, sticky=tk.W)
        self.status_labels['motor_dir'] = ttk.Label(status_frame, text="正转", foreground="blue", font=("Arial", 12, "bold"))
        self.status_labels['motor_dir'].grid(row=0, column=3, padx=5, pady=2, sticky=tk.W)
        self.status_group_widgets['io'].extend([lbl, self.status_labels['motor_dir']])

        lbl = ttk.Label(status_frame, text="编码器方向:")
        lbl.grid(row=1, column=2, padx=15, pady=2, sticky=tk.W)
        self.status_labels['encoder_dir'] = ttk.Label(status_frame, text="正常", foreground="blue", font=("Arial", 12, "bold"))
        self.status_labels['encoder_dir'].grid(row=1, column=3, padx=5, pady=2, sticky=tk.W)
        self.status_group_widgets['io'].extend([lbl, self.status_labels['encoder_dir']])

        lbl = ttk.Label(status_frame, text="PWM输出:")
        lbl.grid(row=2, column=2, padx=15, pady=2, sticky=tk.W)
        self.status_labels['pwm'] = ttk.Label(status_frame, text="0.0%", foreground="purple", font=("Arial", 12, "bold"))
        self.status_labels['pwm'].grid(row=2, column=3, padx=5, pady=2, sticky=tk.W)
        self.status_group_widgets['pwm'].extend([lbl, self.status_labels['pwm']])

        lbl = ttk.Label(status_frame, text="复位状态:")
        lbl.grid(row=3, column=2, padx=15, pady=2, sticky=tk.W)
        self.status_labels['homing'] = ttk.Label(status_frame, text="空闲", foreground="gray", font=("Arial", 12, "bold"))
        self.status_labels['homing'].grid(row=3, column=3, padx=5, pady=2, sticky=tk.W)
        self.status_group_widgets['flags'].extend([lbl, self.status_labels['homing']])

        lbl = ttk.Label(status_frame, text="堵转保护:")
        lbl.grid(row=4, column=2, padx=15, pady=2, sticky=tk.W)
        self.status_labels['stall'] = ttk.Label(status_frame, text="正常", foreground="gray", font=("Arial", 12, "bold"))
        self.status_labels['stall'].grid(row=4, column=3, padx=5, pady=2, sticky=tk.W)
        self.status_group_widgets['flags'].extend([lbl, self.status_labels['stall']])

        # 第4列: 电流环实时值
        lbl = ttk.Label(status_frame, text="实际电流:")
        lbl.grid(row=0, column=6, padx=15, pady=2, sticky=tk.W)
        self.status_labels['current_actual'] = ttk.Label(status_frame, text="0.00 A", foreground="teal", font=("Arial", 12, "bold"))
        self.status_labels['current_actual'].grid(row=0, column=7, padx=5, pady=2, sticky=tk.W)
        self.status_group_widgets['current'].extend([lbl, self.status_labels['current_actual']])

        lbl = ttk.Label(status_frame, text="电流目标:")
        lbl.grid(row=1, column=6, padx=15, pady=2, sticky=tk.W)
        self.status_labels['current_target'] = ttk.Label(status_frame, text="0.00 A", foreground="teal", font=("Arial", 12, "bold"))
        self.status_labels['current_target'].grid(row=1, column=7, padx=5, pady=2, sticky=tk.W)
        self.status_group_widgets['current'].extend([lbl, self.status_labels['current_target']])

        lbl = ttk.Label(status_frame, text="过流保护:")
        lbl.grid(row=2, column=6, padx=15, pady=2, sticky=tk.W)
        self.status_labels['over_current'] = ttk.Label(status_frame, text="正常", foreground="gray", font=("Arial", 12, "bold"))
        self.status_labels['over_current'].grid(row=2, column=7, padx=5, pady=2, sticky=tk.W)
        self.status_group_widgets['current'].extend([lbl, self.status_labels['over_current']])

        lbl = ttk.Label(status_frame, text="电流环:")
        lbl.grid(row=3, column=6, padx=15, pady=2, sticky=tk.W)
        self.status_labels['cur_loop'] = ttk.Label(status_frame, text="关闭", foreground="gray", font=("Arial", 12, "bold"))
        self.status_labels['cur_loop'].grid(row=3, column=7, padx=5, pady=2, sticky=tk.W)
        self.status_group_widgets['current'].extend([lbl, self.status_labels['cur_loop']])

        # 步进电机当前位置(放在电流列第4行空位)
        lbl = ttk.Label(status_frame, text="步进位置:")
        lbl.grid(row=4, column=6, padx=15, pady=2, sticky=tk.W)
        self.status_labels['step_cur_pos'] = ttk.Label(status_frame, text="0", foreground="blue", font=("Arial", 12, "bold"))
        self.status_labels['step_cur_pos'].grid(row=4, column=7, padx=5, pady=2, sticky=tk.W)
        self.status_group_widgets['step'].extend([lbl, self.status_labels['step_cur_pos']])

        # 第3列：PID实时值
        lbl = ttk.Label(status_frame, text="PID误差:")
        lbl.grid(row=0, column=4, padx=15, pady=2, sticky=tk.W)
        self.status_labels['pid_error'] = ttk.Label(status_frame, text="0.00", foreground="teal", font=("Arial", 12, "bold"))
        self.status_labels['pid_error'].grid(row=0, column=5, padx=5, pady=2, sticky=tk.W)
        self.status_group_widgets['pid'].extend([lbl, self.status_labels['pid_error']])

        lbl = ttk.Label(status_frame, text="PID-P:")
        lbl.grid(row=1, column=4, padx=15, pady=2, sticky=tk.W)
        self.status_labels['pid_p'] = ttk.Label(status_frame, text="0.00", foreground="teal", font=("Arial", 12, "bold"))
        self.status_labels['pid_p'].grid(row=1, column=5, padx=5, pady=2, sticky=tk.W)
        self.status_group_widgets['pid'].extend([lbl, self.status_labels['pid_p']])

        lbl = ttk.Label(status_frame, text="PID-I:")
        lbl.grid(row=2, column=4, padx=15, pady=2, sticky=tk.W)
        self.status_labels['pid_i'] = ttk.Label(status_frame, text="0.00", foreground="teal", font=("Arial", 12, "bold"))
        self.status_labels['pid_i'].grid(row=2, column=5, padx=5, pady=2, sticky=tk.W)
        self.status_group_widgets['pid'].extend([lbl, self.status_labels['pid_i']])

        lbl = ttk.Label(status_frame, text="PID-D:")
        lbl.grid(row=3, column=4, padx=15, pady=2, sticky=tk.W)
        self.status_labels['pid_d'] = ttk.Label(status_frame, text="0.00", foreground="teal", font=("Arial", 12, "bold"))
        self.status_labels['pid_d'].grid(row=3, column=5, padx=5, pady=2, sticky=tk.W)
        self.status_group_widgets['pid'].extend([lbl, self.status_labels['pid_d']])

        # 步进电机加减速位置(放在PID列第4行空位)
        lbl = ttk.Label(status_frame, text="步进目标:")
        lbl.grid(row=4, column=4, padx=15, pady=2, sticky=tk.W)
        self.status_labels['step_target_pos'] = ttk.Label(status_frame, text="0", foreground="green", font=("Arial", 12, "bold"))
        self.status_labels['step_target_pos'].grid(row=4, column=5, padx=5, pady=2, sticky=tk.W)
        self.status_group_widgets['step'].extend([lbl, self.status_labels['step_target_pos']])

        # 步进电机预定速度(放在PID列第5行)
        lbl = ttk.Label(status_frame, text="步进目速:")
        lbl.grid(row=5, column=4, padx=15, pady=2, sticky=tk.W)
        self.status_labels['step_preset_speed'] = ttk.Label(status_frame, text="0", foreground="green", font=("Arial", 12, "bold"))
        self.status_labels['step_preset_speed'].grid(row=5, column=5, padx=5, pady=2, sticky=tk.W)
        self.status_group_widgets['step'].extend([lbl, self.status_labels['step_preset_speed']])

        # 步进电机当前速度(放在电流列第5行)
        lbl = ttk.Label(status_frame, text="步进速度:")
        lbl.grid(row=5, column=6, padx=15, pady=2, sticky=tk.W)
        self.status_labels['step_cur_speed'] = ttk.Label(status_frame, text="0", foreground="blue", font=("Arial", 12, "bold"))
        self.status_labels['step_cur_speed'].grid(row=5, column=7, padx=5, pady=2, sticky=tk.W)
        self.status_group_widgets['step'].extend([lbl, self.status_labels['step_cur_speed']])

        # 第5列：ADC采样值（供电电压/外部ADC1/外部ADC2）
        lbl = ttk.Label(status_frame, text="供电电压:")
        lbl.grid(row=0, column=8, padx=15, pady=2, sticky=tk.W)
        self.status_labels['supply_voltage'] = ttk.Label(status_frame, text="0.00V", foreground="teal", font=("Arial", 12, "bold"))
        self.status_labels['supply_voltage'].grid(row=0, column=9, padx=5, pady=2, sticky=tk.W)
        self.status_group_widgets['adc'].extend([lbl, self.status_labels['supply_voltage']])

        lbl = ttk.Label(status_frame, text="PC2(ADC1):")
        lbl.grid(row=1, column=8, padx=15, pady=2, sticky=tk.W)
        self.status_labels['ext_adc1'] = ttk.Label(status_frame, text="0", foreground="teal", font=("Arial", 12, "bold"))
        self.status_labels['ext_adc1'].grid(row=1, column=9, padx=5, pady=2, sticky=tk.W)
        self.status_group_widgets['adc'].extend([lbl, self.status_labels['ext_adc1']])

        lbl = ttk.Label(status_frame, text="PC3(ADC2):")
        lbl.grid(row=2, column=8, padx=15, pady=2, sticky=tk.W)
        self.status_labels['ext_adc2'] = ttk.Label(status_frame, text="0", foreground="teal", font=("Arial", 12, "bold"))
        self.status_labels['ext_adc2'].grid(row=2, column=9, padx=5, pady=2, sticky=tk.W)
        self.status_group_widgets['adc'].extend([lbl, self.status_labels['ext_adc2']])

        # PB4/PB5 引脚电平显示(放在ADC下方)
        lbl = ttk.Label(status_frame, text="PB4电平:")
        lbl.grid(row=3, column=8, padx=15, pady=2, sticky=tk.W)
        self.status_labels['pb4_level'] = ttk.Label(status_frame, text="低", foreground="gray", font=("Arial", 12, "bold"))
        self.status_labels['pb4_level'].grid(row=3, column=9, padx=5, pady=2, sticky=tk.W)
        self.status_group_widgets['io'].extend([lbl, self.status_labels['pb4_level']])

        lbl = ttk.Label(status_frame, text="PB5电平:")
        lbl.grid(row=4, column=8, padx=15, pady=2, sticky=tk.W)
        self.status_labels['pb5_level'] = ttk.Label(status_frame, text="低", foreground="gray", font=("Arial", 12, "bold"))
        self.status_labels['pb5_level'].grid(row=4, column=9, padx=5, pady=2, sticky=tk.W)
        self.status_group_widgets['io'].extend([lbl, self.status_labels['pb5_level']])

        # 最右侧的监控开关按钮（column=10, 跨5行，开启/关闭读取实时状态）
        self.monitor_btn = ttk.Button(status_frame, text="开始\n监控", command=self.toggle_monitor, width=8)
        self.monitor_btn.grid(row=0, column=10, rowspan=5, padx=10, pady=5, sticky="nsew")

        # 状态项显示选项（复选框行，row=5）
        opt_frame = ttk.Frame(status_frame)
        opt_frame.grid(row=5, column=0, columnspan=11, pady=2, sticky=tk.W)
        ttk.Label(opt_frame, text="显示:").pack(side=tk.LEFT, padx=2)
        for key, label in [('position', '位置'), ('speed', '速度'), ('mode', '模式'), ('pwm', 'PWM'),
                           ('pid', 'PID'), ('current', '电流'), ('adc', 'ADC'), ('io', 'IO'), ('flags', '标志'),
                           ('step', '步进')]:
            ttk.Checkbutton(opt_frame, text=label, variable=self.monitor_options[key],
                            command=self.update_status_visibility).pack(side=tk.LEFT, padx=3)

        # 在复选框行右侧添加频率设置(允许手动输入, 如150ms/0.5s/2s)
        ttk.Label(opt_frame, text="| 频率:").pack(side=tk.LEFT, padx=(15, 2))
        freq_combo = ttk.Combobox(opt_frame, textvariable=self.monitor_freq_var, width=8,
            values=["20ms", "50ms", "100ms", "200ms", "500ms", "1s"])
        freq_combo.pack(side=tk.LEFT, padx=2)

        # 寄存器列表按钮（row=6, 监控按钮已移至右侧 column=10）
        btn_row_frame = ttk.Frame(status_frame)
        btn_row_frame.grid(row=6, column=0, columnspan=11, pady=5)
        ttk.Button(btn_row_frame, text="寄存器列表", command=self.open_register_list).pack(side=tk.LEFT, padx=5)

        # 根据复选框初始状态显示/隐藏状态项标签
        self.update_status_visibility()

        # === 采集曲线图（独立高度, 可拖拽调整） ===
        # sash1: 拖拽调整采集曲线窗口高度(不影响其他窗口)
        self._chart_container = ttk.Frame(right_stack, height=350)
        self._chart_container.pack(fill=tk.X)
        self._chart_container.pack_propagate(False)  # 固定高度, 不随内容变化

        chart_frame = ttk.LabelFrame(self._chart_container, text="采集曲线", padding=5)
        chart_frame.pack(fill=tk.BOTH, expand=True)

        # sash手柄: 在chart_container和bottom_container之间
        sash1 = _make_v_sash(self._chart_container, min_h=80)
        sash1.pack(fill=tk.X, pady=(2, 0))

        # 采集控制工具栏(纵向布局: 通道配置行 / 通道列表纵向 / 参数按钮行)
        ctrl_bar = ttk.Frame(chart_frame)
        ctrl_bar.pack(fill=tk.X, pady=(0, 5))

        # 第1行: 通道数
        ch_count_frame = ttk.Frame(ctrl_bar)
        ch_count_frame.pack(fill=tk.X, pady=(0, 2))
        ttk.Label(ch_count_frame, text="通道数:").pack(side=tk.LEFT, padx=(0, 2))
        self.acq_channels_var = tk.StringVar(value="1")
        acq_channels_entry = ttk.Entry(ch_count_frame, textvariable=self.acq_channels_var, width=3)
        acq_channels_entry.pack(side=tk.LEFT, padx=2)
        ttk.Label(ch_count_frame, text="(1~4)").pack(side=tk.LEFT, padx=(0, 5))

        # 第2行起: 通道下拉列表(纵向排列, 每个通道独占一行)
        ch_list_frame = ttk.Frame(ctrl_bar)
        ch_list_frame.pack(fill=tk.X, pady=(0, 2))
        _acq_values = ["转速", "PWM", "位置", "电流", "PC0电压ADC", "PC2外部ADC", "PC3外部ADC",
                       "电流环开-位置P", "电流环开-位置I", "电流环开-位置D",
                       "电流环开-速度P", "电流环开-速度I", "电流环开-速度D",
                       "电流环开-电流P", "电流环开-电流I", "电流环开-电流D",
                       "电流环关-位置P", "电流环关-位置I", "电流环关-位置D",
                       "电流环关-速度P", "电流环关-速度I", "电流环关-速度D",
                       "位置误差", "速度误差"]
        self._acq_values = _acq_values
        self.acq_type_vars = []   # 各通道的StringVar
        self.acq_type_combos = [] # 各通道的 (Label, Combobox, ch_row) 元组
        for ch_idx in range(4):
            var = tk.StringVar(value="转速")
            self.acq_type_vars.append(var)
            ch_row = ttk.Frame(ch_list_frame)
            ch_row.pack(fill=tk.X, pady=1)
            lbl = ttk.Label(ch_row, text=f"CH{ch_idx+1}:")
            combo = ttk.Combobox(ch_row, textvariable=var, width=16,
                                 values=_acq_values, state="readonly")
            combo.bind("<<ComboboxSelected>>", self.on_acq_type_changed)
            lbl.pack(side=tk.LEFT, padx=(5, 0))
            combo.pack(side=tk.LEFT, padx=1)
            self.acq_type_combos.append((lbl, combo, ch_row))

        # 兼容旧代码: acq_type_var 指向通道1的var
        self.acq_type_var = self.acq_type_vars[0]

        # 通道数变化时更新下拉列表可见性
        self._update_channel_visibility()
        self.acq_channels_var.trace_add("write", lambda *_: self._update_channel_visibility())

        # 新起一行: 采集参数 + 操作按钮
        btn_frame = ttk.Frame(ctrl_bar)
        btn_frame.pack(fill=tk.X, pady=(2, 0))
        ttk.Label(btn_frame, text="分频:").pack(side=tk.LEFT, padx=(0, 2))
        self.acq_div_var = tk.StringVar(value="50")
        ttk.Entry(btn_frame, textvariable=self.acq_div_var, width=6).pack(side=tk.LEFT, padx=2)
        ttk.Label(btn_frame, text="(1=100us)").pack(side=tk.LEFT, padx=(0, 5))

        ttk.Label(btn_frame, text="采集点数:").pack(side=tk.LEFT, padx=(10, 2))
        self.acq_size_var = tk.StringVar(value="5120")
        ttk.Entry(btn_frame, textvariable=self.acq_size_var, width=7).pack(side=tk.LEFT, padx=2)
        ttk.Label(btn_frame, text="(1~5120)").pack(side=tk.LEFT, padx=(0, 5))

        ttk.Button(btn_frame, text="设置参数", command=self.set_speed_acq_params).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text="启动采集", command=self.start_speed_acq).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text="读取并绘图", command=self.read_and_plot_speed).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text="清空曲线", command=self.clear_speed_plot).pack(side=tk.LEFT, padx=2)

        self.acq_status_label = ttk.Label(btn_frame, text="状态: 空闲", foreground="gray")
        self.acq_status_label.pack(side=tk.LEFT, padx=10)

        # matplotlib 图表 + 工具栏（支持缩放/平移）
        self.speed_fig = Figure(figsize=(6, 3), dpi=100)
        self.speed_ax = self.speed_fig.add_subplot(111)
        self.speed_axes = [self.speed_ax]  # 多通道时存储各子图轴
        self.speed_ax.set_xlabel("采样点")
        self.speed_ax.set_ylabel("转速 (脉冲/秒)")
        self.speed_ax.set_title("采集曲线")
        self.speed_ax.grid(True)
        self.speed_fig.tight_layout()

        self.speed_canvas = FigureCanvasTkAgg(self.speed_fig, chart_frame)
        self.speed_canvas.draw()
        self.speed_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # matplotlib 工具栏（缩放/平移/保存等）
        self.speed_toolbar = NavigationToolbar2Tk(self.speed_canvas, chart_frame)
        self.speed_toolbar.update()
        self.speed_toolbar.pack(side=tk.BOTTOM, fill=tk.X)

        # === 底部：日志(左) + 原始数据包(右) - 独立高度容器 ===
        self._bottom_container = ttk.Frame(right_stack, height=220)
        self._bottom_container.pack(fill=tk.X)
        self._bottom_container.pack_propagate(False)

        bottom_hpaned = ttk.PanedWindow(self._bottom_container, orient=tk.HORIZONTAL)
        bottom_hpaned.pack(fill=tk.BOTH, expand=True)

        # 日志区域（左）
        log_frame = ttk.LabelFrame(bottom_hpaned, text="通信日志", padding=5)
        bottom_hpaned.add(log_frame, weight=1)

        self.log_text = scrolledtext.ScrolledText(log_frame, height=8, font=("Consolas", 9))
        self.log_text.pack(fill=tk.BOTH, expand=True)

        ttk.Button(log_frame, text="清除日志", command=self.clear_log).pack(pady=2)

        # 原始数据包窗口（右）
        raw_frame = ttk.LabelFrame(bottom_hpaned, text="原始数据包", padding=5)
        bottom_hpaned.add(raw_frame, weight=1)

        self.raw_text = scrolledtext.ScrolledText(raw_frame, height=8, font=("Consolas", 9), state=tk.DISABLED)
        self.raw_text.pack(fill=tk.BOTH, expand=True)

        ttk.Button(raw_frame, text="清除数据", command=self.clear_raw).pack(pady=2)

        # 设置初始分隔线位置
        def _set_initial_sashes(retry=0):
            main_width = paned.winfo_width()
            bottom_width = bottom_hpaned.winfo_width()
            if main_width > 100 and bottom_width > 100:
                paned.sashpos(0, main_width // 3)  # 左侧占1/3
                bottom_hpaned.sashpos(0, bottom_width // 2)  # 日志/原始数据各占一半
                return
            if retry < 15:
                self.root.after(50, lambda: _set_initial_sashes(retry + 1))

        self.root.after(50, _set_initial_sashes)

    def refresh_ports(self):
        """刷新串口列表"""
        ports = serial.tools.list_ports.comports()
        self.port_combo['values'] = [port.device for port in ports]
        if ports:
            self.port_combo.current(0)
            
    def toggle_connection(self):
        """切换连接状态"""
        if self.serial and self.serial.is_open:
            self.disconnect()
        else:
            self.connect()
            
    def connect(self):
        """连接串口"""
        try:
            port = self.port_combo.get()
            baud = int(self.baud_combo.get())
            
            self.serial = serial.Serial(port, baud, timeout=0.2)
            self.modbus = ModbusRTU(
                self.serial,
                raw_tx_callback=self.log_raw_tx,
                raw_rx_callback=self.log_raw_rx
            )
            
            self.connect_btn.config(text="断开")
            self.log(f"已连接到 {port} @ {baud}bps")
            
            # 连接成功后自动开启实时监控（避免重复开启）
            if not self.monitoring:
                self.toggle_monitor()
                
        except Exception as e:
            messagebox.showerror("连接错误", str(e))
            
    def disconnect(self):
        """断开连接"""
        if self.monitoring:
            self.toggle_monitor()
            
        if self.serial and self.serial.is_open:
            self.serial.close()
            
        self.connect_btn.config(text="连接")
        self.log("已断开连接")
        
    def get_slave_addr(self):
        """获取从机地址"""
        try:
            return int(self.slave_addr_var.get())
        except:
            return 1

    @staticmethod
    def int64_to_regs(value):
        if value < 0:
            value += 1 << 64
        return [(value >> 48) & 0xFFFF, (value >> 32) & 0xFFFF, (value >> 16) & 0xFFFF, value & 0xFFFF]

    @staticmethod
    def regs_to_int64(values):
        value = (values[0] << 48) | (values[1] << 32) | (values[2] << 16) | values[3]
        if value >= 0x8000000000000000:
            value -= 0x10000000000000000
        return value

    def read_device_addr(self):
        """读取设备地址寄存器"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return

        def do_read_addr():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        data = self.modbus.read_holding_registers(
                            self.get_slave_addr(), self.REG_SLAVE_ADDR, 1)
                        addr = data[0]
                        self.root.after(0, lambda: self.device_addr_var.set(str(addr)))
                        self.root.after(0, lambda: self.log(f"读取设备地址: {addr}"))
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙，请稍后再试"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"读取设备地址失败: {str(e)}"))

        threading.Thread(target=do_read_addr, daemon=True).start()

    def set_device_addr(self):
        """设置设备地址寄存器"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return

        try:
            new_addr = int(self.device_addr_var.get())
        except ValueError:
            messagebox.showerror("错误", "设备地址必须是整数")
            return

        if new_addr < 1 or new_addr > 247:
            messagebox.showerror("错误", "设备地址范围必须是 1~247")
            return

        old_addr = self.get_slave_addr()

        def do_set_addr():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        self.modbus.write_single_register(old_addr, self.REG_SLAVE_ADDR, new_addr)
                        self.root.after(0, lambda: self.slave_addr_var.set(str(new_addr)))
                        self.root.after(0, lambda: self.log(f"设备地址已从 {old_addr} 修改为 {new_addr}"))
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙，请稍后再试"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"设置设备地址失败: {str(e)}"))

        threading.Thread(target=do_set_addr, daemon=True).start()

    def open_baud_window(self):
        """打开STM32波特率设置窗口"""
        if self.baud_window and self.baud_window.winfo_exists():
            self.baud_window.lift()
            self.baud_window.focus_force()
            return

        self.baud_window = tk.Toplevel(self.root)
        self.baud_window.title("修改STM32波特率")
        self.baud_window.geometry("420x230")
        self.baud_window.resizable(False, False)
        self.baud_window.transient(self.root)

        window = self.baud_window
        main_frame = ttk.LabelFrame(window, text="STM32设备波特率", padding=15)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        current_baud = self.serial.baudrate if self.serial and self.serial.is_open else self.baud_combo.get()
        ttk.Label(main_frame, text=f"当前上位机串口波特率: {current_baud} bps").grid(row=0, column=0, columnspan=3, sticky=tk.W, pady=5)
        ttk.Label(main_frame, text="目标设备波特率:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=8)
        ttk.Combobox(main_frame, textvariable=self.device_baud_var, width=12,
                     values=["9600", "19200", "38400", "57600", "115200"], state="readonly").grid(row=1, column=1, sticky=tk.W, padx=5, pady=8)

        ttk.Label(main_frame, text="写入后STM32会先用旧波特率返回确认，再切换到新波特率。",
                  foreground="blue").grid(row=2, column=0, columnspan=3, sticky=tk.W, pady=8)
        ttk.Label(main_frame, text="成功后上位机会自动切换串口波特率。",
                  foreground="blue").grid(row=3, column=0, columnspan=3, sticky=tk.W, pady=2)

        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=4, column=0, columnspan=3, pady=15)
        ttk.Button(button_frame, text="读取当前设备波特率", command=self.read_device_baud).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="写入并切换", command=self.set_device_baud).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="关闭", command=self.close_baud_window).pack(side=tk.LEFT, padx=5)

        window.protocol("WM_DELETE_WINDOW", self.close_baud_window)

    def close_baud_window(self):
        """关闭STM32波特率设置窗口"""
        if self.baud_window and self.baud_window.winfo_exists():
            self.baud_window.destroy()
        self.baud_window = None

    def read_device_baud(self):
        """读取设备波特率寄存器"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return

        def do_read_baud():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        data = self.modbus.read_holding_registers(
                            self.get_slave_addr(), self.REG_BAUD_RATE, 1)
                        baud = data[0] * 100
                        self.root.after(0, lambda: self.device_baud_var.set(str(baud)))
                        self.root.after(0, lambda: self.log(f"读取设备波特率: {baud}bps"))
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙，请稍后再试"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"读取设备波特率失败: {str(e)}"))

        threading.Thread(target=do_read_baud, daemon=True).start()

    def set_device_baud(self):
        """设置设备波特率寄存器"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return

        try:
            new_baud = int(self.device_baud_var.get())
        except ValueError:
            messagebox.showerror("错误", "设备波特率必须是整数")
            return

        if new_baud not in (9600, 19200, 38400, 57600, 115200):
            messagebox.showerror("错误", "设备波特率必须是 9600、19200、38400、57600、115200")
            return

        old_baud = self.serial.baudrate

        def do_set_baud():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        self.modbus.write_single_register(self.get_slave_addr(), self.REG_BAUD_RATE, new_baud // 100)
                        self.serial.baudrate = new_baud
                        self.root.after(0, lambda: self.baud_combo.set(str(new_baud)))
                        self.root.after(0, lambda: self.log(f"设备波特率已从 {old_baud} 修改为 {new_baud}bps"))
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙，请稍后再试"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"设置设备波特率失败: {str(e)}"))

        threading.Thread(target=do_set_baud, daemon=True).start()
            
    def set_mode(self, mode):
        """设置模式"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return
        
        def do_set_mode():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        mode_names = {0: "位置", 1: "速度", 2: "位置速度", 3: "开环", 4: "外部目标位置", 5: "外部目标速度", 6: "待机", 7: "ADC转速", 8: "ADC位置转速", 9: "ADC开环", 10: "ADC位置"}
                        start_mode = 1 if self.start_mode_var.get() == "标志位" else 0
                        self.modbus.write_single_register(self.get_slave_addr(), self.REG_MODE_SET, start_mode)
                        self.modbus.write_single_register(self.get_slave_addr(), self.REG_CONTROL_WORD, mode)
                        if start_mode:
                            self.modbus.write_single_register(self.get_slave_addr(), self.REG_START_MODE, 1)
                        self.root.after(0, lambda: self.log(f"设置模式: {mode_names.get(mode, '未知')}"))
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙，请稍后再试"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"设置模式失败: {str(e)}"))
        
        threading.Thread(target=do_set_mode, daemon=True).start()
            
    def set_target_position(self):
        """设置目标位置"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return
        
        def do_set_position():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        pos = int(self.target_pos_var.get())
                        original_pos = pos
                        values = self.int64_to_regs(pos)
                        self.modbus.write_multiple_registers(self.get_slave_addr(), self.REG_TARGET_POS_H3, values)
                        self.root.after(0, lambda: self.log(f"设置目标位置: {original_pos}"))
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙，请稍后再试"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"设置目标位置失败: {str(e)}"))
        
        threading.Thread(target=do_set_position, daemon=True).start()

    def toggle_step_accel(self):
        """切换步进加减速使能(新增): 勾选写入1, 取消写入0, 保存到Flash"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            self.step_accel_var.set(False)
            return
        en = 1 if self.step_accel_var.get() else 0
        def do_toggle():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        self.modbus.write_single_register(self.get_slave_addr(), self.REG_STEP_ACCEL_EN, en)
                        self.root.after(0, lambda: self.log(f"步进加减速: {'开启' if en else '关闭'}"))
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙，请稍后再试"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"设置加减速失败: {str(e)}"))
        threading.Thread(target=do_toggle, daemon=True).start()

    def set_step_max_speed(self):
        """设置步进加减速最大速度(zuidasudu, 0x006B~0x006E, 64位)
        用户输入真实速度(步/秒), 上位机读取加速表参数(u, a)后用s2b2公式转为索引值写入.
        s2b2(u, a, s) = (s² - u²) / (2*a)
        """
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return
        val_str = self.step_max_speed_var.get().strip()
        if not val_str:
            messagebox.showwarning("警告", "请输入最大速度值")
            return
        try:
            speed = int(val_str)
        except ValueError:
            messagebox.showwarning("警告", "最大速度必须为整数")
            return
        if speed < 0:
            messagebox.showwarning("警告", "最大速度不能为负数")
            return
        if speed > 10000:
            if not messagebox.askyesno("确认", f"最大速度 {speed} 超过10K，确认继续？"):
                return

        def do_set():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        # 1. 读取加速表参数: u(最小速度, 0x0094), a(加速度, 0x0095~0x0096)
                        acc_params = self.modbus.read_holding_registers(
                            self.get_slave_addr(), self.REG_ACC_TABLE_U, 3)
                        if acc_params is None or len(acc_params) < 3:
                            self.root.after(0, lambda: messagebox.showerror("错误", "读取加速表参数失败"))
                            return
                        u = acc_params[0]                          # 最小速度
                        a = (acc_params[2] << 16) | acc_params[1]  # 加速度(32位)
                        # 缓存u和a, 供实时显示转换使用
                        self.acc_u = u
                        self.acc_a = a
                        if a == 0:
                            self.root.after(0, lambda: messagebox.showerror("错误", "加速表加速度为0，无法转换"))
                            return
                        # 2. s2b2公式: 索引 = (s² - u²) / (2*a)
                        idx = (speed * speed - u * u) // (2 * a)
                        if idx < 0:
                            idx = 0
                        # 3. 将索引值拆为4个16位寄存器写入
                        reg_values = [
                            (idx >> 48) & 0xFFFF,
                            (idx >> 32) & 0xFFFF,
                            (idx >> 16) & 0xFFFF,
                            idx & 0xFFFF,
                        ]
                        self.modbus.write_multiple_registers(
                            self.get_slave_addr(), self.REG_STEP_MAX_SPEED_H3, reg_values)
                        # 4. 回读验证: 确保写入的索引值正确
                        import time
                        time.sleep(0.05)  # 等待STM32处理写入
                        readback = self.modbus.read_holding_registers(
                            self.get_slave_addr(), self.REG_STEP_MAX_SPEED_H3, 4)
                        if readback is not None and len(readback) >= 4:
                            rb_idx = (readback[0] << 48) | (readback[1] << 32) | (readback[2] << 16) | readback[3]
                            if rb_idx != idx:
                                self.root.after(0, lambda: self.log(
                                    f"警告: 回读索引{rb_idx}与写入值{idx}不一致!"))
                        # 5. 更新显示(速度+索引)
                        self.root.after(0, lambda: self.step_cur_index_label.config(
                            text=f"当前: {speed}步/秒 (索引{idx})"))
                        self.root.after(0, lambda: self.log(
                            f"步进最大速度已设置: {speed}步/秒 → 索引{idx} (u={u}, a={a})"))
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙，请稍后再试"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"设置步进最大速度失败: {str(e)}"))
        threading.Thread(target=do_set, daemon=True).start()
            
    def set_current_as_origin(self):
        """将当前位置设为原点（写REG_SET_ORIGIN=1触发）"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return

        if not messagebox.askyesno("确认", "确定将当前位置设为原点吗？\n"
                                   "执行后编码器位置清零，目标位置同步为0。"):
            return

        def do_set_origin():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_SET_ORIGIN, 1)
                        # 同步界面上的目标位置输入框
                        self.root.after(0, lambda: self.target_pos_var.set("0"))
                        self.root.after(0, lambda: self.log("已将当前位置设为原点（编码器清零）"))
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙，请稍后再试"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"设置原点失败: {str(e)}"))

        threading.Thread(target=do_set_origin, daemon=True).start()

    def set_target_speed(self):
        """设置目标速度"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return
        
        def do_set_speed():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        speed = int(self.target_speed_var.get())
                        values = [
                            (speed >> 16) & 0xFFFF,
                            speed & 0xFFFF
                        ]
                        self.modbus.write_multiple_registers(self.get_slave_addr(), self.REG_TARGET_SPEED_H, values)
                        self.root.after(0, lambda: self.log(f"设置目标速度: {speed}"))
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙，请稍后再试"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"设置目标速度失败: {str(e)}"))
        
        threading.Thread(target=do_set_speed, daemon=True).start()
            
    def apply_pid(self):
        """应用PID参数"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return

        # 检查PID参数是否为空：为空表示用户尚未读取参数，禁止写入避免误覆盖电机参数
        empty_fields = []
        if not self.pos_kp_var.get().strip(): empty_fields.append("位置环Kp")
        if not self.pos_ki_var.get().strip(): empty_fields.append("位置环Ki")
        if not self.pos_kd_var.get().strip(): empty_fields.append("位置环Kd")
        if not self.spd_kp_var.get().strip(): empty_fields.append("速度环Kp")
        if not self.spd_ki_var.get().strip(): empty_fields.append("速度环Ki")
        if not self.spd_kd_var.get().strip(): empty_fields.append("速度环Kd")
        if empty_fields:
            messagebox.showwarning(
                "请先读取参数",
                "检测到以下参数框为空，未读取过电机参数：\n  - " +
                "\n  - ".join(empty_fields) +
                "\n\n为避免误写入导致电机参数错乱，请先点击【读取PID】按钮，"
                "从电机控制器读取当前PID后再修改并写入。"
            )
            return
        
        def clamp_pid(value, scale=100):
            """将PID参数限幅到16位有符号范围 (-32768~32767)"""
            v = int(float(value) * scale)
            if v > 32767:
                v = 32767
            elif v < -32768:
                v = -32768
            return v

        def do_apply_pid():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        pos_kp = clamp_pid(self.pos_kp_var.get())
                        pos_ki = clamp_pid(self.pos_ki_var.get())
                        pos_kd = clamp_pid(self.pos_kd_var.get(), 1000)  # D项×1000, 3位小数
                        self.modbus.write_multiple_registers(self.get_slave_addr(), self.REG_POS_KP, [pos_kp, pos_ki, pos_kd])

                        spd_kp = clamp_pid(self.spd_kp_var.get())
                        spd_ki = clamp_pid(self.spd_ki_var.get())
                        spd_kd = clamp_pid(self.spd_kd_var.get(), 1000)  # D项×1000, 3位小数
                        self.modbus.write_multiple_registers(self.get_slave_addr(), self.REG_SPD_KP, [spd_kp, spd_ki, spd_kd])
                        
                        self.root.after(0, lambda: self.log("应用PID参数成功"))
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙，请稍后再试"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"应用PID参数失败: {str(e)}"))
        
        threading.Thread(target=do_apply_pid, daemon=True).start()
    
    def read_pid(self):
        """读取PID参数"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return
        
        def do_read_pid():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        # 读取位置环PID (0x0008-0x000A)
                        pos_data = self.modbus.read_holding_registers(self.get_slave_addr(), self.REG_POS_KP, 3)
                        # 读取速度环PID (0x000B-0x000D)
                        spd_data = self.modbus.read_holding_registers(self.get_slave_addr(), self.REG_SPD_KP, 3)
                        
                        # 16位有符号转float (Kp/Ki精度0.01, Kd精度0.001)
                        def to_float(v, scale=100.0):
                            if v >= 0x8000:
                                v -= 0x10000
                            return v / scale

                        pos_kp = to_float(pos_data[0])
                        pos_ki = to_float(pos_data[1])
                        pos_kd = to_float(pos_data[2], 1000.0)  # D项×1000
                        spd_kp = to_float(spd_data[0])
                        spd_ki = to_float(spd_data[1])
                        spd_kd = to_float(spd_data[2], 1000.0)  # D项×1000

                        # 更新界面
                        self.root.after(0, lambda: self.pos_kp_var.set(f"{pos_kp:.2f}"))
                        self.root.after(0, lambda: self.pos_ki_var.set(f"{pos_ki:.2f}"))
                        self.root.after(0, lambda: self.pos_kd_var.set(f"{pos_kd:.3f}"))
                        self.root.after(0, lambda: self.spd_kp_var.set(f"{spd_kp:.2f}"))
                        self.root.after(0, lambda: self.spd_ki_var.set(f"{spd_ki:.2f}"))
                        self.root.after(0, lambda: self.spd_kd_var.set(f"{spd_kd:.3f}"))
                        self.root.after(0, lambda: self.log(
                            f"读取PID: 位置环 Kp={pos_kp:.2f} Ki={pos_ki:.2f} Kd={pos_kd:.3f} | "
                            f"速度环 Kp={spd_kp:.2f} Ki={spd_ki:.2f} Kd={spd_kd:.3f}"))
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙，请稍后再试"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"读取PID参数失败: {str(e)}"))
        
        threading.Thread(target=do_read_pid, daemon=True).start()
            
    def _on_start_mode_changed(self, event):
        """启动方式改变时，切换启动按钮的显示状态"""
        if self.start_mode_var.get() == "标志位":
            self.start_btn.grid()
        else:
            self.start_btn.grid_remove()

    def start_motor(self):
        """标志位启动：写REG_START_MODE=1触发"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return

        def do_start():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_START_MODE, 1)
                        self.root.after(0, lambda: self.log("已发送启动标志位"))
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙，请稍后再试"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"启动失败: {str(e)}"))

        threading.Thread(target=do_start, daemon=True).start()

    def apply_other_params(self):
        """应用其他参数"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return

        # 检查其他参数是否为空：为空表示用户尚未读取参数，禁止写入避免误覆盖电机参数
        empty_fields = []
        if not self.dead_zone_var.get().strip(): empty_fields.append("死区")
        if not self.max_output_var.get().strip(): empty_fields.append("最大输出")
        if not self.max_run_speed_var.get().strip(): empty_fields.append("最大运行速度")
        if not self.start_mode_var.get().strip(): empty_fields.append("启动方式")
        if not self.motor_dir_var.get().strip(): empty_fields.append("电机方向")
        if not self.encoder_dir_var.get().strip(): empty_fields.append("编码器方向")
        if not self.home_mode_var.get().strip(): empty_fields.append("复位模式")
        if not self.home_dir_var.get().strip(): empty_fields.append("复位方向")
        if not self.home_current_var.get().strip(): empty_fields.append("复位电流")
        if not self.home_speed_var.get().strip(): empty_fields.append("复位速度")
        if not self.home_precision_speed_var.get().strip(): empty_fields.append("精确检测速度")
        if not self.home_precision_cycles_var.get().strip(): empty_fields.append("检测次数")
        if not self.home_max_distance_var.get().strip(): empty_fields.append("最大距离")
        if not self.home_back_distance_var.get().strip(): empty_fields.append("复位偏置")
        if not self.home_auto_start_var.get().strip(): empty_fields.append("开机自复位")
        if empty_fields:
            messagebox.showwarning(
                "请先读取参数",
                "检测到以下参数框为空，未读取过电机参数：\n  - " +
                "\n  - ".join(empty_fields) +
                "\n\n为避免误写入导致电机参数错乱，请先点击【读取参数】按钮，"
                "从电机控制器读取当前参数后再修改并写入。"
            )
            return
        
        def do_apply_other():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        dead_zone = int(self.dead_zone_var.get())
                        max_output = int(self.max_output_var.get())
                        max_run_speed = int(self.max_run_speed_var.get())
                        start_mode = 1 if self.start_mode_var.get() == "标志位" else 0
                        motor_dir = 1 if self.motor_dir_var.get() == "反转" else 0
                        encoder_dir = 1 if self.encoder_dir_var.get() == "反转" else 0
                        home_mode_map = {"关闭": 0, "堵转": 1, "精确复位": 2}
                        home_mode = home_mode_map.get(self.home_mode_var.get(), 0)
                        home_dir = 1 if self.home_dir_var.get() == "正方向" else 0
                        home_current = int(self.home_current_var.get())
                        home_speed = int(self.home_speed_var.get())
                        home_max_distance = int(self.home_max_distance_var.get())
                        home_back_distance = int(self.home_back_distance_var.get())
                        home_auto_start = 1 if self.home_auto_start_var.get() == "开启" else 0
                        home_precision_speed = int(self.home_precision_speed_var.get())
                        home_precision_cycles = int(self.home_precision_cycles_var.get())
                        home_values = [
                            home_mode,
                            home_dir,
                            home_current,
                            (home_speed >> 16) & 0xFFFF,
                            home_speed & 0xFFFF,
                            (home_max_distance >> 16) & 0xFFFF,
                            home_max_distance & 0xFFFF,
                            (home_back_distance >> 16) & 0xFFFF,
                            home_back_distance & 0xFFFF,
                            0,  # REG_HOME_TRIGGER=0, 不触发
                            home_auto_start,
                        ]

                        self.modbus.write_single_register(self.get_slave_addr(), self.REG_DEAD_ZONE, dead_zone)
                        self.modbus.write_single_register(self.get_slave_addr(), self.REG_MAX_OUTPUT, max_output)
                        self.modbus.write_multiple_registers(
                            self.get_slave_addr(), self.REG_MAX_RUN_SPEED_H,
                            [(max_run_speed >> 16) & 0xFFFF, max_run_speed & 0xFFFF])
                        self.modbus.write_single_register(self.get_slave_addr(), self.REG_MODE_SET, start_mode)
                        self.modbus.write_single_register(self.get_slave_addr(), self.REG_DIRECTION, motor_dir)
                        self.modbus.write_single_register(self.get_slave_addr(), self.REG_ENCODER_DIRECTION, encoder_dir)
                        self.modbus.write_multiple_registers(self.get_slave_addr(), self.REG_HOME_MODE, home_values)
                        self.modbus.write_multiple_registers(
                            self.get_slave_addr(), self.REG_HOME_PRECISION_SPEED_H,
                            [(home_precision_speed >> 16) & 0xFFFF, home_precision_speed & 0xFFFF])
                        self.modbus.write_single_register(self.get_slave_addr(), self.REG_HOME_PRECISION_CYCLES, home_precision_cycles)

                        hm_name = {0: "关闭", 1: "堵转", 2: "精确复位"}
                        self.root.after(0, lambda: self.log(
                            f"应用参数成功: 死区={dead_zone}, 最大输出={max_output}, "
                            f"最大运行速度={max_run_speed}, "
                            f"启动方式={'标志位' if start_mode else '直接'}, "
                            f"电机={'反转' if motor_dir else '正转'}, "
                            f"编码器={'反转' if encoder_dir else '正常'}, "
                            f"复位={hm_name.get(home_mode, '?')}"))
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙，请稍后再试"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"应用参数失败: {str(e)}"))
        
        threading.Thread(target=do_apply_other, daemon=True).start()

    def read_other_params(self):
        """读取其他参数（含复位参数）"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return

        def do_read_other():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        start_mode_data = self.modbus.read_holding_registers(
                            self.get_slave_addr(), self.REG_MODE_SET, 1)
                        data = self.modbus.read_holding_registers(
                            self.get_slave_addr(), self.REG_DEAD_ZONE, 5)
                        home_data = self.modbus.read_holding_registers(
                            self.get_slave_addr(), self.REG_HOME_MODE, 11)
                        max_run_data = self.modbus.read_holding_registers(
                            self.get_slave_addr(), self.REG_MAX_RUN_SPEED_H, 2)
                        precision_data = self.modbus.read_holding_registers(
                            self.get_slave_addr(), self.REG_HOME_PRECISION_SPEED_H, 3)
                        dead_zone = data[0]
                        max_output = data[1]
                        max_run_speed = (max_run_data[0] << 16) | max_run_data[1]
                        start_mode = start_mode_data[0]
                        motor_dir = data[3]
                        encoder_dir = data[4]
                        home_mode = home_data[0]
                        home_dir = home_data[1]
                        home_current = home_data[2]
                        home_speed = (home_data[3] << 16) | home_data[4]
                        home_max_distance = (home_data[5] << 16) | home_data[6]
                        home_back_distance = (home_data[7] << 16) | home_data[8]
                        if home_back_distance >= 0x80000000:
                            home_back_distance -= 0x100000000
                        home_auto_start = home_data[10]
                        home_precision_speed = (precision_data[0] << 16) | precision_data[1]
                        home_precision_cycles = precision_data[2]

                        hm_display = {0: "关闭", 1: "堵转", 2: "精确复位"}
                        def update_ui():
                            self.dead_zone_var.set(str(dead_zone))
                            self.max_output_var.set(str(max_output))
                            self.max_run_speed_var.set(str(max_run_speed))
                            self.start_mode_var.set("标志位" if start_mode else "直接启动")
                            self.motor_dir_var.set("反转" if motor_dir else "正转")
                            self.encoder_dir_var.set("反转" if encoder_dir else "正常")
                            self.home_mode_var.set(hm_display.get(home_mode, "关闭"))
                            self.home_dir_var.set("正方向" if home_dir else "负方向")
                            self.home_current_var.set(str(home_current))
                            self.home_speed_var.set(str(home_speed))
                            self.home_max_distance_var.set(str(home_max_distance))
                            self.home_back_distance_var.set(str(home_back_distance))
                            self.home_auto_start_var.set("开启" if home_auto_start else "关闭")
                            self.home_precision_speed_var.set(str(home_precision_speed))
                            self.home_precision_cycles_var.set(str(home_precision_cycles))
                            self.log(
                                f"读取参数: 死区={dead_zone}, 最大输出={max_output}, "
                                f"最大运行速度={max_run_speed}, "
                                f"启动模式={'标志位' if start_mode else '直接'}, "
                                f"电机={'反转' if motor_dir else '正转'}, "
                                f"编码器={'反转' if encoder_dir else '正常'}, "
                                f"复位={hm_display.get(home_mode, '?')}")
                        self.root.after(0, update_ui)
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙，请稍后再试"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"读取参数失败: {str(e)}"))

        threading.Thread(target=do_read_other, daemon=True).start()

    def reset_config(self):
        """恢复默认参数（写入main.c中MotorControl_Init的默认值）"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return

        if not messagebox.askyesno("确认", "确定要恢复默认参数吗？\n这将覆盖当前PID和参数设置。"):
            return

        def do_reset():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        # 默认PID值 (与main.c MotorControl_Init一致, 精度×100)
                        pos_pid = [700, 110, 10]    # 7.0, 1.1, 0.1
                        spd_pid = [30, 300, 0]      # 0.30, 3.0, 0.0
                        # 写入位置环PID
                        self.modbus.write_multiple_registers(
                            self.get_slave_addr(), self.REG_POS_KP, pos_pid)
                        # 写入速度环PID
                        self.modbus.write_multiple_registers(
                            self.get_slave_addr(), self.REG_SPD_KP, spd_pid)
                        # 写入其他默认参数
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_DEAD_ZONE, 1)
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_MAX_OUTPUT, 900)
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_MODE_SET, 0)
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_START_MODE, 0)
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_DIRECTION, 0)
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_ENCODER_DIRECTION, 0)
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_START_FLAG, 0)
                        self.modbus.write_multiple_registers(
                            self.get_slave_addr(), self.REG_HOME_MODE,
                            [0, 0, 300, 0, 1000, 0, 10000, 0, 100, 0, 1])
                        # 引脚配置默认值: PB4=脉冲/高有效/止正, PB5=方向/高有效/止负
                        self.modbus.write_multiple_registers(
                            self.get_slave_addr(), self.REG_INPUT1_FUNC,
                            [0, 0, 0, 1, 0, 1])
                        self.modbus.write_multiple_registers(
                            self.get_slave_addr(), self.REG_MAX_RUN_SPEED_H,
                            [0, 0])
                        self.modbus.write_multiple_registers(
                            self.get_slave_addr(), self.REG_HOME_PRECISION_SPEED_H,
                            [0, 100])
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_HOME_PRECISION_CYCLES, 3)
                        self.modbus.write_multiple_registers(
                            self.get_slave_addr(), self.REG_INPUT1_TARGET_SPEED_H,
                            [0, 0, 0, 0])

                        def update_ui():
                            self.pos_kp_var.set("7.00")
                            self.pos_ki_var.set("1.10")
                            self.pos_kd_var.set("0.10")
                            self.spd_kp_var.set("0.30")
                            self.spd_ki_var.set("3.00")
                            self.spd_kd_var.set("0.00")
                            self.dead_zone_var.set("1")
                            self.max_output_var.set("900")
                            self.max_run_speed_var.set("0")
                            self.start_mode_var.set("直接启动")
                            self.motor_dir_var.set("正转")
                            self.encoder_dir_var.set("正常")
                            self.home_mode_var.set("关闭")
                            self.home_dir_var.set("负方向")
                            self.home_current_var.set("300")
                            self.home_speed_var.set("1000")
                            self.home_max_distance_var.set("10000")
                            self.home_back_distance_var.set("100")
                            self.home_auto_start_var.set("开启")
                            self.home_precision_speed_var.set("100")
                            self.home_precision_cycles_var.set("3")
                            self.pin4_func_var.set("脉冲")
                            self.pin4_pol_var.set("高电平")
                            self.pin4_ldir_var.set("停止正方向")
                            self.pin5_func_var.set("方向")
                            self.pin5_pol_var.set("高电平")
                            self.pin5_ldir_var.set("停止负方向")
                            self.pin4_target_speed_var.set("0")
                            self.pin5_target_speed_var.set("0")
                            self.log("已恢复默认参数并保存到Flash")
                        self.root.after(0, update_ui)
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙，请稍后再试"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"恢复默认失败: {str(e)}"))

        threading.Thread(target=do_reset, daemon=True).start()

    def trigger_homing(self):
        """写REG_HOME_TRIGGER=1，触发一次复位"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return

        if not messagebox.askyesno("确认", "确定立即执行一次复位操作吗？"):
            return

        def do_trigger():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_HOME_TRIGGER, 1)
                        self.root.after(0, lambda: self.log("已触发一次复位操作"))
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙，请稍后再试"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"触发复位失败: {str(e)}"))

        threading.Thread(target=do_trigger, daemon=True).start()

    def read_pin_config(self):
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return
        def do_read():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        data = self.modbus.read_holding_registers(self.get_slave_addr(), self.REG_INPUT1_FUNC, 6)
                        target_data = self.modbus.read_holding_registers(self.get_slave_addr(), self.REG_INPUT1_TARGET_POS_H3, 8)
                        speed_data = self.modbus.read_holding_registers(self.get_slave_addr(), self.REG_INPUT1_TARGET_SPEED_H, 4)
                        p4f, p4p, p4l, p5f, p5p, p5l = data
                        p4_target = self.regs_to_int64(target_data[0:4])
                        p5_target = self.regs_to_int64(target_data[4:8])
                        p4_speed = (speed_data[0] << 16) | speed_data[1]
                        if p4_speed >= 0x80000000:
                            p4_speed -= 0x100000000
                        p5_speed = (speed_data[2] << 16) | speed_data[3]
                        if p5_speed >= 0x80000000:
                            p5_speed -= 0x100000000
                        fm = {0:"脉冲",1:"方向",2:"原点位置开关",3:"限位开关",4:"外部目标位置",5:"外部目标速度",6:"执行复位操作",7:"无功能",8:"停止",9:"启动标志位触发"}
                        pm = {0:"高电平",1:"低电平"}
                        lm = {0:"停止正方向",1:"停止负方向"}
                        self.root.after(0, lambda: self.pin4_func_var.set(fm.get(p4f,"?")))
                        self.root.after(0, lambda: self.pin4_pol_var.set(pm.get(p4p,"?")))
                        self.root.after(0, lambda: self.pin4_ldir_var.set(lm.get(p4l,"?")))
                        self.root.after(0, lambda: self.pin5_func_var.set(fm.get(p5f,"?")))
                        self.root.after(0, lambda: self.pin5_pol_var.set(pm.get(p5p,"?")))
                        self.root.after(0, lambda: self.pin5_ldir_var.set(lm.get(p5l,"?")))
                        self.root.after(0, lambda: self.pin4_target_pos_var.set(str(p4_target)))
                        self.root.after(0, lambda: self.pin5_target_pos_var.set(str(p5_target)))
                        self.root.after(0, lambda: self.pin4_target_speed_var.set(str(p4_speed)))
                        self.root.after(0, lambda: self.pin5_target_speed_var.set(str(p5_speed)))
                        self.root.after(0, lambda: self.log(f"读取引脚: PB4={fm.get(p4f,'?')}/{pm.get(p4p,'?')}/{lm.get(p4l,'?')}/目标位置={p4_target}/目标速度={p4_speed} PB5={fm.get(p5f,'?')}/{pm.get(p5p,'?')}/{lm.get(p5l,'?')}/目标位置={p5_target}/目标速度={p5_speed}"))
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"读取引脚配置失败: {str(e)}"))
        threading.Thread(target=do_read, daemon=True).start()

    def write_pin_config(self):
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return

        # 检查引脚参数是否为空：为空表示用户尚未读取参数，禁止写入避免误覆盖电机参数
        empty_fields = []
        if not self.pin4_func_var.get().strip(): empty_fields.append("PB4功能")
        if not self.pin4_pol_var.get().strip(): empty_fields.append("PB4有效电平")
        if not self.pin4_ldir_var.get().strip(): empty_fields.append("PB4限位方向")
        if not self.pin4_target_pos_var.get().strip(): empty_fields.append("PB4电平目标位置")
        if not self.pin4_target_speed_var.get().strip(): empty_fields.append("PB4电平目标速度")
        if not self.pin5_func_var.get().strip(): empty_fields.append("PB5功能")
        if not self.pin5_pol_var.get().strip(): empty_fields.append("PB5有效电平")
        if not self.pin5_ldir_var.get().strip(): empty_fields.append("PB5限位方向")
        if not self.pin5_target_pos_var.get().strip(): empty_fields.append("PB5电平目标位置")
        if not self.pin5_target_speed_var.get().strip(): empty_fields.append("PB5电平目标速度")
        if empty_fields:
            messagebox.showwarning(
                "请先读取参数",
                "检测到以下引脚参数框为空，未读取过电机参数：\n  - " +
                "\n  - ".join(empty_fields) +
                "\n\n为避免误写入导致电机参数错乱，请先点击【读取引脚配置】按钮，"
                "从电机控制器读取当前引脚配置后再修改并写入。"
            )
            return

        fm = {"脉冲":0,"方向":1,"原点位置开关":2,"限位开关":3,"外部目标位置":4,"外部目标速度":5,"执行复位操作":6,"无功能":7,"停止":8,"启动标志位触发":9}
        pm = {"高电平":0,"低电平":1}
        lm = {"停止正方向":0,"停止负方向":1}
        try:
            pin4_target = int(self.pin4_target_pos_var.get())
            pin5_target = int(self.pin5_target_pos_var.get())
            pin4_speed = int(self.pin4_target_speed_var.get())
            pin5_speed = int(self.pin5_target_speed_var.get())
        except ValueError:
            messagebox.showerror("错误", "电平目标位置/速度必须是整数")
            return
        vals = [fm[self.pin4_func_var.get()], pm[self.pin4_pol_var.get()], lm[self.pin4_ldir_var.get()],
                fm[self.pin5_func_var.get()], pm[self.pin5_pol_var.get()], lm[self.pin5_ldir_var.get()]]
        target_vals = self.int64_to_regs(pin4_target) + self.int64_to_regs(pin5_target)
        speed_vals = [(pin4_speed >> 16) & 0xFFFF, pin4_speed & 0xFFFF,
                      (pin5_speed >> 16) & 0xFFFF, pin5_speed & 0xFFFF]
        def do_write():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        self.modbus.write_multiple_registers(self.get_slave_addr(), self.REG_INPUT1_FUNC, vals)
                        self.modbus.write_multiple_registers(self.get_slave_addr(), self.REG_INPUT1_TARGET_POS_H3, target_vals)
                        self.modbus.write_multiple_registers(self.get_slave_addr(), self.REG_INPUT1_TARGET_SPEED_H, speed_vals)
                        self.root.after(0, lambda: self.log(f"写入引脚配置成功: PB4目标位置={pin4_target}/目标速度={pin4_speed}, PB5目标位置={pin5_target}/目标速度={pin5_speed}"))
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"写入引脚配置失败: {str(e)}"))
        threading.Thread(target=do_write, daemon=True).start()

    # ========== PC2/PC3 ADC功能配置 ==========

    def read_adc_config(self):
        """读取PC2/PC3 ADC功能配置"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return
        def do_read():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        # 读 PC2/PC3功能 (2个寄存器)
                        func_data = self.modbus.read_holding_registers(self.get_slave_addr(), self.REG_PC2_FUNC, 2)
                        # 读 ADC最大速度 (2个寄存器)
                        speed_data = self.modbus.read_holding_registers(self.get_slave_addr(), self.REG_ADC_MAX_SPEED_H, 2)
                        # 读 ADC最大PWM (1个寄存器)
                        pwm_data = self.modbus.read_holding_registers(self.get_slave_addr(), self.REG_ADC_MAX_PWM, 1)
                        # 读 ADC最大位置 (4个寄存器)
                        pos_data = self.modbus.read_holding_registers(self.get_slave_addr(), self.REG_ADC_MAX_POS_H3, 4)
                        # 读 ADC最小速度 (2个寄存器)
                        min_speed_data = self.modbus.read_holding_registers(self.get_slave_addr(), self.REG_ADC_MIN_SPEED_H, 2)
                        # 读 ADC最小PWM (1个寄存器)
                        min_pwm_data = self.modbus.read_holding_registers(self.get_slave_addr(), self.REG_ADC_MIN_PWM, 1)
                        # 读 ADC最小位置 (4个寄存器)
                        min_pos_data = self.modbus.read_holding_registers(self.get_slave_addr(), self.REG_ADC_MIN_POS_H3, 4)
                        # 读 ADC死区1和死区2 (4个寄存器)
                        dz_data = self.modbus.read_holding_registers(self.get_slave_addr(), self.REG_ADC_DEAD_ZONE1_POS, 4)
                        # 读 ADC滤波窗口 (1个寄存器, 单独读)
                        filt_data = self.modbus.read_holding_registers(self.get_slave_addr(), self.REG_ADC_FILTER_SIZE, 1)
                        adc_filter_size = filt_data[0]

                        pc2_func = func_data[0]
                        pc3_func = func_data[1]
                        max_speed = (speed_data[0] << 16) | speed_data[1]
                        if max_speed >= 0x80000000:
                            max_speed -= 0x100000000
                        max_pwm = pwm_data[0]
                        if max_pwm >= 0x8000:
                            max_pwm -= 0x10000
                        max_pos = self.regs_to_int64(pos_data)

                        min_speed = (min_speed_data[0] << 16) | min_speed_data[1]
                        if min_speed >= 0x80000000:
                            min_speed -= 0x100000000
                        min_pwm = min_pwm_data[0]
                        if min_pwm >= 0x8000:
                            min_pwm -= 0x10000
                        min_pos = self.regs_to_int64(min_pos_data)

                        dz1_pos = dz_data[0]
                        dz1_width = dz_data[1]
                        dz2_pos = dz_data[2]
                        dz2_width = dz_data[3]
                        dz_fm = {0: "最小点", 1: "中位点", 2: "最大点"}

                        fm = {0: "无功能", 1: "ADC转速模式", 2: "ADC位置转速模式", 3: "ADC开环模式", 4: "ADC位置模式"}
                        self.root.after(0, lambda: self.pc2_func_var.set(fm.get(pc2_func, "无功能")))
                        self.root.after(0, lambda: self.pc3_func_var.set(fm.get(pc3_func, "无功能")))
                        self.root.after(0, lambda: self.adc_max_speed_var.set(str(max_speed)))
                        self.root.after(0, lambda: self.adc_max_pwm_var.set(str(max_pwm)))
                        self.root.after(0, lambda: self.adc_max_pos_var.set(str(max_pos)))
                        self.root.after(0, lambda: self.adc_min_speed_var.set(str(min_speed)))
                        self.root.after(0, lambda: self.adc_min_pwm_var.set(str(min_pwm)))
                        self.root.after(0, lambda: self.adc_min_pos_var.set(str(min_pos)))
                        self.root.after(0, lambda: self.adc_dz1_pos_var.set(dz_fm.get(dz1_pos, "最小点")))
                        self.root.after(0, lambda: self.adc_dz1_width_var.set(str(dz1_width)))
                        self.root.after(0, lambda: self.adc_dz2_pos_var.set(dz_fm.get(dz2_pos, "最小点")))
                        self.root.after(0, lambda: self.adc_dz2_width_var.set(str(dz2_width)))
                        self.root.after(0, lambda: self.adc_filter_size_var.set(str(adc_filter_size)))
                        self.root.after(0, lambda: self.log(f"读取ADC配置成功: PC2={fm.get(pc2_func,'?')}, PC3={fm.get(pc3_func,'?')}, 速度=[{min_speed}~{max_speed}], PWM=[{min_pwm}~{max_pwm}], 位置=[{min_pos}~{max_pos}], 死区1={dz_fm.get(dz1_pos,'?')}宽{dz1_width}, 死区2={dz_fm.get(dz2_pos,'?')}宽{dz2_width}, 滤波窗口={adc_filter_size}"))
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"读取ADC配置失败: {str(e)}"))
        threading.Thread(target=do_read, daemon=True).start()

    def write_adc_config(self):
        """写入PC2/PC3 ADC功能配置"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return
        # 空字段检查
        empty_fields = []
        if not self.pc2_func_var.get().strip(): empty_fields.append("PC2功能")
        if not self.pc3_func_var.get().strip(): empty_fields.append("PC3功能")
        if not self.adc_max_speed_var.get().strip(): empty_fields.append("最大速度")
        if not self.adc_max_pwm_var.get().strip(): empty_fields.append("最大PWM")
        if not self.adc_max_pos_var.get().strip(): empty_fields.append("最大位置")
        if not self.adc_min_speed_var.get().strip(): empty_fields.append("最小速度")
        if not self.adc_min_pwm_var.get().strip(): empty_fields.append("最小PWM")
        if not self.adc_min_pos_var.get().strip(): empty_fields.append("最小位置")
        if not self.adc_dz1_pos_var.get().strip(): empty_fields.append("死区1位置")
        if not self.adc_dz1_width_var.get().strip(): empty_fields.append("死区1宽度")
        if not self.adc_dz2_pos_var.get().strip(): empty_fields.append("死区2位置")
        if not self.adc_dz2_width_var.get().strip(): empty_fields.append("死区2宽度")
        if not self.adc_filter_size_var.get().strip(): empty_fields.append("ADC滤波窗口")
        if empty_fields:
            messagebox.showwarning("警告", "以下字段为空, 请先点击\"读取ADC配置\":\n" + "\n".join(empty_fields))
            return
        fm = {"无功能": 0, "ADC转速模式": 1, "ADC位置转速模式": 2, "ADC开环模式": 3, "ADC位置模式": 4}
        try:
            pc2_func = fm[self.pc2_func_var.get()]
            pc3_func = fm[self.pc3_func_var.get()]
            max_speed = int(self.adc_max_speed_var.get())
            max_pwm = int(self.adc_max_pwm_var.get())
            max_pos = int(self.adc_max_pos_var.get())
            min_speed = int(self.adc_min_speed_var.get())
            min_pwm = int(self.adc_min_pwm_var.get())
            min_pos = int(self.adc_min_pos_var.get())
            dz_fm = {"最小点": 0, "中位点": 1, "最大点": 2}
            dz1_pos = dz_fm[self.adc_dz1_pos_var.get()]
            dz1_width = int(self.adc_dz1_width_var.get())
            dz2_pos = dz_fm[self.adc_dz2_pos_var.get()]
            dz2_width = int(self.adc_dz2_width_var.get())
            adc_filter_size = int(self.adc_filter_size_var.get())
        except (ValueError, KeyError):
            messagebox.showerror("错误", "参数格式错误, 速度/PWM/位置必须是整数, 功能必须从下拉框选择")
            return
        # 范围检查
        if max_pwm > 1000 or max_pwm < -1000:
            messagebox.showerror("错误", "最大PWM范围: -1000~+1000")
            return
        if min_pwm > 1000 or min_pwm < -1000:
            messagebox.showerror("错误", "最小PWM范围: -1000~+1000")
            return
        if dz1_width < 0 or dz1_width > 4095:
            messagebox.showerror("错误", "死区1宽度范围: 0~4095")
            return
        if dz2_width < 0 or dz2_width > 4095:
            messagebox.showerror("错误", "死区2宽度范围: 0~4095")
            return
        if adc_filter_size < 0 or adc_filter_size > 128:
            messagebox.showerror("错误", "ADC滤波窗口范围: 0~128 (0/1=关闭, 100≈500ms延迟)")
            return
        # 构造写入值
        func_vals = [pc2_func, pc3_func]
        speed_vals = [(max_speed >> 16) & 0xFFFF, max_speed & 0xFFFF]
        pwm_val = max_pwm & 0xFFFF
        pos_vals = self.int64_to_regs(max_pos)
        min_speed_vals = [(min_speed >> 16) & 0xFFFF, min_speed & 0xFFFF]
        min_pwm_val = min_pwm & 0xFFFF
        min_pos_vals = self.int64_to_regs(min_pos)
        dz_vals = [dz1_pos, dz1_width, dz2_pos, dz2_width]
        def do_write():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        self.modbus.write_multiple_registers(self.get_slave_addr(), self.REG_PC2_FUNC, func_vals)
                        self.modbus.write_multiple_registers(self.get_slave_addr(), self.REG_ADC_MAX_SPEED_H, speed_vals)
                        self.modbus.write_single_register(self.get_slave_addr(), self.REG_ADC_MAX_PWM, pwm_val)
                        self.modbus.write_multiple_registers(self.get_slave_addr(), self.REG_ADC_MAX_POS_H3, pos_vals)
                        self.modbus.write_multiple_registers(self.get_slave_addr(), self.REG_ADC_MIN_SPEED_H, min_speed_vals)
                        self.modbus.write_single_register(self.get_slave_addr(), self.REG_ADC_MIN_PWM, min_pwm_val)
                        self.modbus.write_multiple_registers(self.get_slave_addr(), self.REG_ADC_MIN_POS_H3, min_pos_vals)
                        self.modbus.write_multiple_registers(self.get_slave_addr(), self.REG_ADC_DEAD_ZONE1_POS, dz_vals)
                        # 写入ADC滤波窗口 (0x006A, 单独写)
                        self.modbus.write_single_register(self.get_slave_addr(), self.REG_ADC_FILTER_SIZE, adc_filter_size & 0xFFFF)
                        self.root.after(0, lambda: self.log(f"写入ADC配置成功: PC2={self.pc2_func_var.get()}, PC3={self.pc3_func_var.get()}, 速度=[{min_speed}~{max_speed}], PWM=[{min_pwm}~{max_pwm}], 位置=[{min_pos}~{max_pos}], 死区1={self.adc_dz1_pos_var.get()}宽{dz1_width}, 死区2={self.adc_dz2_pos_var.get()}宽{dz2_width}, 滤波窗口={adc_filter_size}"))
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"写入ADC配置失败: {str(e)}"))
        threading.Thread(target=do_write, daemon=True).start()

    # ========== 堵转保护配置 ==========

    def read_stall_config(self):
        """读取堵转保护参数"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return
        def do_read():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        data = self.modbus.read_holding_registers(
                            self.get_slave_addr(), self.REG_STALL_PROT_EN, 3)
                        en = data[0]
                        err_limit_raw = data[1]
                        if err_limit_raw >= 0x8000:
                            err_limit_raw -= 0x10000
                        time_ticks = data[2]
                        self.root.after(0, lambda: self.stall_en_var.set("开启" if en else "关闭"))
                        self.root.after(0, lambda: self.stall_err_var.set(str(err_limit_raw)))
                        self.root.after(0, lambda: self.stall_time_var.set(str(time_ticks)))
                        self.root.after(0, lambda: self.log(
                            f"读取堵转保护: 使能={'开启' if en else '关闭'}, "
                            f"误差阈值={err_limit_raw}, 持续={time_ticks}×5ms={time_ticks*5}ms"))
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"读取堵转保护失败: {str(e)}"))
        threading.Thread(target=do_read, daemon=True).start()

    def write_stall_config(self):
        """写入堵转保护参数"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return

        # 检查堵转参数是否为空：为空表示用户尚未读取参数，禁止写入避免误覆盖电机参数
        empty_fields = []
        if not self.stall_en_var.get().strip(): empty_fields.append("使能")
        if not self.stall_err_var.get().strip(): empty_fields.append("误差阈值")
        if not self.stall_time_var.get().strip(): empty_fields.append("持续时长")
        if empty_fields:
            messagebox.showwarning(
                "请先读取参数",
                "检测到以下堵转参数框为空，未读取过电机参数：\n  - " +
                "\n  - ".join(empty_fields) +
                "\n\n为避免误写入导致电机参数错乱，请先点击【读取参数】按钮，"
                "从电机控制器读取当前堵转保护参数后再修改并写入。"
            )
            return

        try:
            err_limit = int(self.stall_err_var.get())
            time_ticks = int(self.stall_time_var.get())
            if time_ticks < 0:
                raise ValueError
        except ValueError:
            messagebox.showwarning("警告", "误差阈值和持续时长必须为整数")
            return
        en = 1 if self.stall_en_var.get() == "开启" else 0
        # 误差阈值转成16位有符号
        if err_limit < -32768 or err_limit > 32767:
            messagebox.showwarning("警告", "误差阈值范围: -32768~32767")
            return
        err_reg = err_limit & 0xFFFF
        def do_write():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        self.modbus.write_multiple_registers(
                            self.get_slave_addr(), self.REG_STALL_PROT_EN,
                            [en, err_reg, time_ticks & 0xFFFF])
                        self.root.after(0, lambda: self.log(
                            f"写入堵转保护: 使能={'开启' if en else '关闭'}, "
                            f"误差阈值={err_limit}, 持续={time_ticks}×5ms={time_ticks*5}ms"))
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"写入堵转保护失败: {str(e)}"))
        threading.Thread(target=do_write, daemon=True).start()

    def reset_stall(self):
        """清除堵转保护触发标志"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return
        def do_reset():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_STALL_RESET, 1)
                        self.root.after(0, lambda: self.log("已清除堵转保护触发标志"))
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"复位堵转失败: {str(e)}"))
        threading.Thread(target=do_reset, daemon=True).start()

    # ========== 电流环配置 ==========

    def read_current_loop_config(self):
        """读取电流环参数"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return
        def do_read():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        # 读取0x0045~0x004B (7个寄存器)
                        data = self.modbus.read_holding_registers(
                            self.get_slave_addr(), self.REG_CUR_LOOP_EN, 7)
                        en = data[0]
                        # Kp/Ki ×100, Kd ×1000, 有符号
                        def to_float_signed(v, scale):
                            if v >= 0x8000:
                                v -= 0x10000
                            return v / scale
                        kp = to_float_signed(data[1], 100.0)
                        ki = to_float_signed(data[2], 100.0)
                        kd = to_float_signed(data[3], 1000.0)
                        offset = data[4]
                        scale = data[5] / 10000.0  # ×10000
                        # 过流阈值 ×10, 有符号
                        ocl_raw = data[6]
                        if ocl_raw >= 0x8000:
                            ocl_raw -= 0x10000
                        # 相对值 → 安培: 1A ≈ 121相对单位
                        ocl_A = ocl_raw / 10.0 / 121.0

                        # 读取电流限幅 (0x0062, 与电流环不连续, 单独读)
                        mc_data = self.modbus.read_holding_registers(
                            self.get_slave_addr(), self.REG_MAX_CURRENT, 1)
                        mc_raw = mc_data[0]
                        if mc_raw >= 0x8000:
                            mc_raw -= 0x10000
                        # 相对值 → 安培: 1A ≈ 121相对单位
                        mc_A = mc_raw / 10.0 / 121.0

                        # 读取电流死区 (0x0069, 单独读)
                        cdz_data = self.modbus.read_holding_registers(
                            self.get_slave_addr(), self.REG_CUR_DEAD_ZONE, 1)
                        cdz_raw = cdz_data[0]
                        if cdz_raw >= 0x8000:
                            cdz_raw -= 0x10000
                        # 相对值 → 安培: 1A ≈ 121相对单位
                        cdz_A = cdz_raw / 10.0 / 121.0

                        # 读取电流环使能时的速度环/位置环PID参数 (0x0063~0x0068, 连续6个寄存器)
                        # Kp/Ki ×100, Kd ×1000, 有符号16位
                        pid_cur_data = self.modbus.read_holding_registers(
                            self.get_slave_addr(), self.REG_SPD_KP_CUR, 6)
                        spd_kp_cur = to_float_signed(pid_cur_data[0], 100.0)
                        spd_ki_cur = to_float_signed(pid_cur_data[1], 100.0)
                        spd_kd_cur = to_float_signed(pid_cur_data[2], 1000.0)
                        pos_kp_cur = to_float_signed(pid_cur_data[3], 100.0)
                        pos_ki_cur = to_float_signed(pid_cur_data[4], 100.0)
                        pos_kd_cur = to_float_signed(pid_cur_data[5], 1000.0)

                        def update_ui():
                            self.cur_loop_en_var.set("开启" if en else "关闭")
                            self.cur_kp_var.set(f"{kp:.2f}")
                            self.cur_ki_var.set(f"{ki:.2f}")
                            self.cur_kd_var.set(f"{kd:.3f}")
                            self.cur_offset_var.set(str(offset))
                            self.cur_scale_var.set(f"{scale:.4f}")
                            self.over_cur_limit_var.set(f"{ocl_A:.2f}")
                            self.max_current_var.set(f"{mc_A:.2f}")
                            self.cur_dead_zone_var.set(f"{cdz_A:.3f}")
                            # 电流环使能时的速度环/位置环PID
                            self.spd_kp_cur_var.set(f"{spd_kp_cur:.2f}")
                            self.spd_ki_cur_var.set(f"{spd_ki_cur:.2f}")
                            self.spd_kd_cur_var.set(f"{spd_kd_cur:.3f}")
                            self.pos_kp_cur_var.set(f"{pos_kp_cur:.2f}")
                            self.pos_ki_cur_var.set(f"{pos_ki_cur:.2f}")
                            self.pos_kd_cur_var.set(f"{pos_kd_cur:.3f}")
                            self.log(
                                f"读取电流环: 使能={'开启' if en else '关闭'}, "
                                f"Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.3f}, "
                                f"零点={offset}, 系数={scale:.4f}, "
                                f"过流={ocl_A:.2f}A, 限幅={mc_A:.2f}A, 死区={cdz_A:.3f}A, "
                                f"速度PID(电流模式)={spd_kp_cur:.2f}/{spd_ki_cur:.2f}/{spd_kd_cur:.3f}, "
                                f"位置PID(电流模式)={pos_kp_cur:.2f}/{pos_ki_cur:.2f}/{pos_kd_cur:.3f}")
                        self.root.after(0, update_ui)
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"读取电流环参数失败: {str(e)}"))
        threading.Thread(target=do_read, daemon=True).start()

    def write_current_loop_config(self):
        """写入电流环参数"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return

        # 检查参数是否为空
        empty_fields = []
        if not self.cur_loop_en_var.get().strip(): empty_fields.append("电流环使能")
        if not self.cur_kp_var.get().strip(): empty_fields.append("Kp")
        if not self.cur_ki_var.get().strip(): empty_fields.append("Ki")
        if not self.cur_kd_var.get().strip(): empty_fields.append("Kd")
        if not self.cur_offset_var.get().strip(): empty_fields.append("零点ADC")
        if not self.cur_scale_var.get().strip(): empty_fields.append("标定系数")
        if not self.over_cur_limit_var.get().strip(): empty_fields.append("过流阈值")
        if not self.max_current_var.get().strip(): empty_fields.append("电流限幅")
        if not self.cur_dead_zone_var.get().strip(): empty_fields.append("电流死区")
        # 电流环使能时使用的速度环/位置环PID参数
        if not self.spd_kp_cur_var.get().strip(): empty_fields.append("电流模式速度Kp")
        if not self.spd_ki_cur_var.get().strip(): empty_fields.append("电流模式速度Ki")
        if not self.spd_kd_cur_var.get().strip(): empty_fields.append("电流模式速度Kd")
        if not self.pos_kp_cur_var.get().strip(): empty_fields.append("电流模式位置Kp")
        if not self.pos_ki_cur_var.get().strip(): empty_fields.append("电流模式位置Ki")
        if not self.pos_kd_cur_var.get().strip(): empty_fields.append("电流模式位置Kd")
        if empty_fields:
            messagebox.showwarning(
                "请先读取参数",
                "检测到以下参数框为空，未读取过电机参数：\n  - " +
                "\n  - ".join(empty_fields) +
                "\n\n为避免误写入导致参数错乱，请先点击【读取参数】按钮。"
            )
            return

        try:
            en = 1 if self.cur_loop_en_var.get() == "开启" else 0
            kp = float(self.cur_kp_var.get())
            ki = float(self.cur_ki_var.get())
            kd = float(self.cur_kd_var.get())
            offset = int(self.cur_offset_var.get())
            scale = float(self.cur_scale_var.get())
            ocl_A = float(self.over_cur_limit_var.get())
            mc_A = float(self.max_current_var.get())
            cdz_A = float(self.cur_dead_zone_var.get())
            # 电流环使能时的速度环/位置环PID参数
            spd_kp_cur = float(self.spd_kp_cur_var.get())
            spd_ki_cur = float(self.spd_ki_cur_var.get())
            spd_kd_cur = float(self.spd_kd_cur_var.get())
            pos_kp_cur = float(self.pos_kp_cur_var.get())
            pos_ki_cur = float(self.pos_ki_cur_var.get())
            pos_kd_cur = float(self.pos_kd_cur_var.get())
        except ValueError:
            messagebox.showerror("错误", "参数格式错误，请检查输入")
            return

        if offset < 0 or offset > 4095:
            messagebox.showwarning("警告", "零点ADC范围: 0~4095")
            return
        if scale <= 0:
            messagebox.showwarning("警告", "标定系数必须大于0")
            return
        if ocl_A < 0:
            messagebox.showwarning("警告", "过流阈值不能为负数(0=关闭保护)")
            return
        if mc_A < 0:
            messagebox.showwarning("警告", "电流限幅不能为负数")
            return
        if cdz_A < 0:
            messagebox.showwarning("警告", "电流死区不能为负数")
            return

        # 限幅到16位有符号
        def clamp_signed(v, lo=-32768, hi=32767):
            if v < lo: return lo
            if v > hi: return hi
            return int(v)
        kp_reg = clamp_signed(round(kp * 100))
        ki_reg = clamp_signed(round(ki * 100))
        kd_reg = clamp_signed(round(kd * 1000))
        scale_reg = clamp_signed(round(scale * 10000), 0, 65535)
        # 过流阈值: A → 相对值×10, 1A ≈ 121相对单位
        ocl_rel = ocl_A * 121.0 * 10.0
        ocl_reg = clamp_signed(round(ocl_rel))
        # 电流限幅: A → 相对值×10, 1A ≈ 121相对单位
        mc_rel = mc_A * 121.0 * 10.0
        mc_reg = clamp_signed(round(mc_rel))
        # 电流死区: A → 相对值×10, 1A ≈ 121相对单位
        cdz_rel = cdz_A * 121.0 * 10.0
        cdz_reg = clamp_signed(round(cdz_rel))
        # 电流环使能时的速度环PID (Kp/Ki ×100, Kd ×1000, 有符号16位)
        spd_kp_cur_reg = clamp_signed(round(spd_kp_cur * 100))
        spd_ki_cur_reg = clamp_signed(round(spd_ki_cur * 100))
        spd_kd_cur_reg = clamp_signed(round(spd_kd_cur * 1000))
        # 电流环使能时的位置环PID (Kp/Ki ×100, Kd ×1000, 有符号16位)
        pos_kp_cur_reg = clamp_signed(round(pos_kp_cur * 100))
        pos_ki_cur_reg = clamp_signed(round(pos_ki_cur * 100))
        pos_kd_cur_reg = clamp_signed(round(pos_kd_cur * 1000))

        def do_write():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        # 注意: 使能单独写, 避免一次性写7个时顺序错乱
                        # 先写参数, 最后写使能, 避免参数未更新就切换模式
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_CUR_KP, kp_reg & 0xFFFF)
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_CUR_KI, ki_reg & 0xFFFF)
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_CUR_KD, kd_reg & 0xFFFF)
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_CUR_OFFSET, offset)
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_CUR_SCALE, scale_reg)
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_OVER_CUR_LIMIT, ocl_reg & 0xFFFF)
                        # 写入电流限幅 (0x0062, 单独写)
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_MAX_CURRENT, mc_reg & 0xFFFF)
                        # 写入电流死区 (0x0069, 单独写)
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_CUR_DEAD_ZONE, cdz_reg & 0xFFFF)
                        # 写入电流环使能时的速度环PID (0x0063~0x0065)
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_SPD_KP_CUR, spd_kp_cur_reg & 0xFFFF)
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_SPD_KI_CUR, spd_ki_cur_reg & 0xFFFF)
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_SPD_KD_CUR, spd_kd_cur_reg & 0xFFFF)
                        # 写入电流环使能时的位置环PID (0x0066~0x0068)
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_POS_KP_CUR, pos_kp_cur_reg & 0xFFFF)
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_POS_KI_CUR, pos_ki_cur_reg & 0xFFFF)
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_POS_KD_CUR, pos_kd_cur_reg & 0xFFFF)
                        # 最后写使能寄存器
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_CUR_LOOP_EN, en)
                        self.root.after(0, lambda: self.log(
                            f"写入电流环: 使能={'开启' if en else '关闭'}, "
                            f"Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.3f}, "
                            f"零点={offset}, 系数={scale:.4f}, "
                            f"过流={ocl_A:.2f}A (相对值{ocl_reg}), "
                            f"限幅={mc_A:.2f}A (相对值{mc_reg}), "
                            f"死区={cdz_A:.3f}A (相对值{cdz_reg}), "
                            f"速度PID(电流模式)={spd_kp_cur:.2f}/{spd_ki_cur:.2f}/{spd_kd_cur:.3f}, "
                            f"位置PID(电流模式)={pos_kp_cur:.2f}/{pos_ki_cur:.2f}/{pos_kd_cur:.3f}"))
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"写入电流环参数失败: {str(e)}"))
        threading.Thread(target=do_write, daemon=True).start()

    def reset_over_current(self):
        """清除过流保护触发标志"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return
        def do_reset():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_OVER_CUR_RESET, 1)
                        self.root.after(0, lambda: self.log("已清除过流保护触发标志"))
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"过流复位失败: {str(e)}"))
        threading.Thread(target=do_reset, daemon=True).start()

    def calibrate_current_offset(self):
        """自动零点标定: 发送0xFFFF到REG_CUR_OFFSET触发固件自动标定
        固件读取64次ADC平均作为新offset, 要求电机停止且无电流流过"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return
        if not messagebox.askyesno("确认",
                "自动零点标定前请确保:\n"
                "1. 电机已停止 (未启动或处于待机模式)\n"
                "2. 无电流流过采样电阻\n"
                "3. 电流采样电路已上电\n\n"
                "固件将读取64次ADC值取平均作为零点offset,\n"
                "标定后需点击'写入参数'保存到Flash.\n\n"
                "确定执行自动零点标定吗?"):
            return
        def do_calib():
            try:
                if self.serial_lock.acquire(timeout=2.0):
                    try:
                        # 发送0xFFFF触发固件自动标定
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_CUR_OFFSET, 0xFFFF)
                        # 等待固件标定完成(64次ADC采样+延时, 约50ms)
                        import time
                        time.sleep(0.1)
                        # 读取新的offset值
                        data = self.modbus.read_holding_registers(
                            self.get_slave_addr(), self.REG_CUR_OFFSET, 1)
                        new_offset = data[0]
                        # 读取标定后的电流值, 验证是否接近0
                        data2 = self.modbus.read_holding_registers(
                            self.get_slave_addr(), self.REG_CURRENT_ACTUAL, 1)
                        cur_raw = data2[0]
                        if cur_raw >= 0x8000:
                            cur_raw -= 0x10000
                        cur_A = cur_raw / 10.0 / 121.0
                        def show_result(off, ca):
                            self.cur_offset_var.set(str(off))
                            msg = (f"零点标定完成!\n\n"
                                   f"新零点ADC: {off} (默认2048)\n"
                                   f"标定后电流读数: {ca:+.4f} A\n\n")
                            if abs(ca) < 0.05:
                                msg += "电流接近0, 标定成功!\n请点击'写入参数'保存到Flash。"
                            else:
                                msg += ("电流仍偏离0, 可能原因:\n"
                                        "- 电机未完全停止\n"
                                        "- 采样电路有偏置\n"
                                        "建议多次标定或检查硬件。")
                            messagebox.showinfo("标定结果", msg)
                        self.root.after(0, lambda: show_result(new_offset, cur_A))
                        self.root.after(0, lambda: self.log(
                            f"自动零点标定: 新offset={new_offset}, 标定后电流={cur_A:+.4f}A"))
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"标定失败: {str(e)}"))
        threading.Thread(target=do_calib, daemon=True).start()

    def save_all_config_to_file(self):
        """保存当前界面所有配置参数到JSON文件"""
        import json
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON文件", "*.json"), ("所有文件", "*.*")],
            initialfile="motor_config.json")
        if not filename:
            return

        # 收集所有配置参数
        config = {}
        # 所有以_var结尾的StringVar属性
        for attr_name in dir(self):
            if attr_name.endswith('_var') and attr_name != 'slave_addr_var':
                val = getattr(self, attr_name, None)
                if isinstance(val, tk.StringVar):
                    try:
                        config[attr_name] = val.get()
                    except Exception:
                        pass

        # 单独保存slave_addr_var
        try:
            config['slave_addr_var'] = self.slave_addr_var.get()
        except Exception:
            pass

        try:
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(config, f, ensure_ascii=False, indent=2)
            messagebox.showinfo("成功", f"配置已保存到\n{filename}\n共{len(config)}项参数")
            self.log(f"配置保存: {filename} ({len(config)}项)")
        except Exception as e:
            messagebox.showerror("错误", f"保存失败: {e}")

    def load_all_config_from_file(self):
        """从JSON文件加载配置到界面"""
        import json
        filename = filedialog.askopenfilename(
            filetypes=[("JSON文件", "*.json"), ("所有文件", "*.*")])
        if not filename:
            return

        try:
            with open(filename, 'r', encoding='utf-8') as f:
                config = json.load(f)

            loaded = 0
            for key, value in config.items():
                val = getattr(self, key, None)
                if isinstance(val, tk.StringVar):
                    try:
                        val.set(str(value))
                        loaded += 1
                    except Exception:
                        pass

            messagebox.showinfo("成功", f"已加载{loaded}项参数\n\n请点击各区域的'写入参数'按钮\n将配置写入电机设备")
            self.log(f"配置加载: {filename} ({loaded}项)")
        except Exception as e:
            messagebox.showerror("错误", f"加载失败: {e}")

    def read_device_all_registers(self):
        """读取设备所有寄存器并显示在列表窗口"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return
        self.open_register_list()
        # 窗口创建后自动触发读取
        self.root.after(500, self._read_all_registers)

    def update_status_visibility(self):
        """根据复选框状态显示/隐藏状态项的值标签
        名字标签(如"当前位置:")始终显示, 只隐藏后面的值(如"0")"""
        value_widgets = set(self.status_labels.values())
        for key, var in self.monitor_options.items():
            visible = var.get()
            for widget in self.status_group_widgets.get(key, []):
                if widget in value_widgets:
                    # 值标签: 根据复选框状态显示/隐藏
                    if visible:
                        widget.grid()
                    else:
                        widget.grid_remove()
                else:
                    # 名字标签: 始终显示
                    widget.grid()

    def toggle_monitor(self):
        """切换监控状态"""
        if self.monitoring:
            self.monitoring = False
            self.monitor_btn.config(text="开始\n监控")
            self.log("停止监控")
        else:
            if not self.modbus:
                messagebox.showwarning("警告", "请先连接串口")
                return
                
            self.monitoring = True
            self.monitor_btn.config(text="停止\n监控")
            self.log("开始监控")
            self.monitor_thread = threading.Thread(target=self.monitor_loop, daemon=True)
            self.monitor_thread.start()
            
    def monitor_loop(self):
        """监控循环"""
        dir_read_counter = 0  # 方向寄存器读取计数器，降低读取频率
        # 频率映射表（监控间隔秒数）
        freq_map = {"20ms": 0.02, "50ms": 0.05, "100ms": 0.1,
                    "200ms": 0.2, "500ms": 0.5, "1s": 1.0}
        while self.monitoring:
            try:
                # 采集数据读取中时暂停监控，避免串口冲突
                if self.acq_reading:
                    time.sleep(0.1)
                    continue

                # 非阻塞获取锁，获取不到就跳过本次监控
                if not self.serial_lock.acquire(timeout=0.05):
                    time.sleep(0.05)
                    continue

                try:
                    # 根据勾选状态分批读取, 减少通信量
                    # 第1批: 0x0100~0x0115 连续22个寄存器 (位置/速度/模式/状态/PWM/PID/电流/ADC)
                    # 判断是否需要读取这批: position/speed/mode/pwm/pid/current/adc 任一勾选
                    need_batch1 = (self.monitor_options['position'].get() or
                                  self.monitor_options['speed'].get() or
                                  self.monitor_options['mode'].get() or
                                  self.monitor_options['pwm'].get() or
                                  self.monitor_options['pid'].get() or
                                  self.monitor_options['current'].get() or
                                  self.monitor_options['adc'].get())
                    data = None
                    if need_batch1:
                        data = self.modbus.read_holding_registers(
                            self.get_slave_addr(), self.REG_CURRENT_POS_H3, 22)

                    # 第2批: 目标位置 (仅position勾选时读)
                    target_data = None
                    if self.monitor_options['position'].get():
                        target_data = self.modbus.read_holding_registers(
                            self.get_slave_addr(), self.REG_TARGET_POS_H3, 4)

                    # 第3批: 启动标志 (仅flags勾选时读)
                    start_flag_data = None
                    if self.monitor_options['flags'].get():
                        start_flag_data = self.modbus.read_holding_registers(
                            self.get_slave_addr(), self.REG_START_MODE, 1)

                    # 第4批: 电流环使能+过流状态 (仅current勾选时读, 2次非连续读取)
                    cur_en_data = None
                    over_cur_data = None
                    if self.monitor_options['current'].get():
                        cur_en_data = self.modbus.read_holding_registers(
                            self.get_slave_addr(), self.REG_CUR_LOOP_EN, 1)
                        over_cur_data = self.modbus.read_holding_registers(
                            self.get_slave_addr(), self.REG_OVER_CUR_STATUS, 1)

                    # 第5批: 方向寄存器 (仅io勾选时, 每20次读1次)
                    dir_data = None
                    dir_read_counter += 1
                    if self.monitor_options['io'].get() and dir_read_counter >= 20:
                        dir_read_counter = 0
                        try:
                            dir_data = self.modbus.read_holding_registers(
                                self.get_slave_addr(), self.REG_DIRECTION, 2)
                        except Exception:
                            dir_data = None  # 读取失败跳过，不影响主流程

                    # 第6批: 步进加减速位置 (仅step勾选时读, 一次读0x0073~0x008E共28个寄存器)
                    step_data = None
                    if self.monitor_options['step'].get():
                        step_data = self.modbus.read_holding_registers(
                            self.get_slave_addr(), self.REG_STEP_TARGET_POS_H3, 28)
                finally:
                    self.serial_lock.release()

                # === 数据解析 ===
                # 解析目标位置（64位有符号）
                target_position = 0
                if target_data is not None:
                    target_position = (target_data[0] << 48) | (target_data[1] << 32) | (target_data[2] << 16) | target_data[3]
                    if target_position >= 0x8000000000000000:
                        target_position -= 0x10000000000000000

                # 解析步进加减速位置（64位有符号, step_data读自0x0073~0x008E共28个寄存器）
                step_target_pos = 0  # 预定目标位置(yudingweizhi, step_data[0]~[3])
                step_cur_pos = 0     # 当前步位置(dangqianweizhi, step_data[24]~[27])
                step_cur_speed = 0   # 当前运行速度(dangqiansudu, step_data[4]~[7])
                step_preset_speed = 0 # 预定运行速度(yudingsudu, step_data[8]~[11])
                if step_data is not None and len(step_data) >= 28:
                    step_target_pos = (step_data[0] << 48) | (step_data[1] << 32) | (step_data[2] << 16) | step_data[3]
                    if step_target_pos >= 0x8000000000000000:
                        step_target_pos -= 0x10000000000000000
                    step_cur_pos = (step_data[24] << 48) | (step_data[25] << 32) | (step_data[26] << 16) | step_data[27]
                    if step_cur_pos >= 0x8000000000000000:
                        step_cur_pos -= 0x10000000000000000
                    # dangqiansudu (当前运行速度, step_data[4]~[7], 0x0077~0x007A)
                    step_cur_speed = (step_data[4] << 48) | (step_data[5] << 32) | (step_data[6] << 16) | step_data[7]
                    if step_cur_speed >= 0x8000000000000000:
                        step_cur_speed -= 0x10000000000000000
                    # yudingsudu (预定运行速度, step_data[8]~[11], 0x007B~0x007E)
                    step_preset_speed = (step_data[8] << 48) | (step_data[9] << 32) | (step_data[10] << 16) | step_data[11]
                    if step_preset_speed >= 0x8000000000000000:
                        step_preset_speed -= 0x10000000000000000

                # 解析当前位置/速度/模式/状态/PWM/PID/电流/ADC（data可能为None）
                position = 0
                speed = 0
                mode = 0
                status = 0
                pwm_raw = 0
                supply_voltage_raw = 0
                ext_adc1_raw = 0
                ext_adc2_raw = 0
                pid_error = 0.0
                pid_p = 0.0
                pid_i = 0.0
                pid_d = 0.0
                cur_actual_raw = 0
                cur_target_raw = 0

                if data is not None:
                    # 解析当前位置（64位有符号）
                    position = (data[0] << 48) | (data[1] << 32) | (data[2] << 16) | data[3]
                    if position >= 0x8000000000000000:
                        position -= 0x10000000000000000

                    # 解析速度（32位）
                    speed = (data[4] << 16) | data[5]
                    if speed >= 0x80000000:
                        speed -= 0x100000000

                    # 解析模式和状态
                    mode = data[6]
                    status = data[7]

                    # 解析PWM输出 (16位有符号, 范围-1000~+1000, 转换为百分比)
                    pwm_raw = data[8]
                    if pwm_raw >= 0x8000:
                        pwm_raw -= 0x10000

                    # 解析PID实时值（32位有符号, ×100）
                    def parse_pid_32(h, l):
                        val = (h << 16) | l
                        if val >= 0x80000000:
                            val -= 0x100000000
                        return val / 100.0

                    pid_error = parse_pid_32(data[9], data[10])
                    pid_p = parse_pid_32(data[11], data[12])
                    pid_i = parse_pid_32(data[13], data[14])
                    pid_d = parse_pid_32(data[15], data[16])

                    # 解析电流环状态 (data[17]=电流实际, data[18]=电流目标, 均为×10有符号)
                    cur_actual_raw = data[17]
                    if cur_actual_raw >= 0x8000:
                        cur_actual_raw -= 0x10000
                    cur_target_raw = data[18]
                    if cur_target_raw >= 0x8000:
                        cur_target_raw -= 0x10000

                    # ADC采样值(0x0113~0x0115 = data[19]~data[21])
                    supply_voltage_raw = data[19]   # 供电电压(单位0.01V)
                    ext_adc1_raw = data[20]         # 外部ADC1(PC2, 0~4095)
                    ext_adc2_raw = data[21]         # 外部ADC2(PC3, 0~4095)

                # 派生状态值
                start_flag = start_flag_data[0] != 0 if start_flag_data is not None else False
                homing_active = (status & 0x4000) != 0
                homing_failed = (status & 0x2000) != 0
                stall_tripped = (status & 0x1000) != 0
                pb4_high = (status & 0x0400) != 0  # bit10 = PB4电平
                pb5_high = (status & 0x0200) != 0  # bit9  = PB5电平
                supply_voltage_V = supply_voltage_raw / 100.0  # 转换为V
                pwm_percent = pwm_raw / 10.0  # -1000~+1000 -> -100.0%~+100.0%
                # 相对值×10 → 安培: 1A ≈ 121相对单位
                cur_actual_A = cur_actual_raw / 10.0 / 121.0
                cur_target_A = cur_target_raw / 10.0 / 121.0
                cur_en = cur_en_data[0] != 0 if cur_en_data is not None else False
                over_cur_tripped = over_cur_data[0] != 0 if over_cur_data is not None else False

                # === UI更新（根据复选框状态+数据可用性决定是否更新） ===
                mode_names = {0: "位置", 1: "速度", 2: "位置速度", 3: "开环", 4: "外部目标位置", 5: "外部目标速度", 6: "待机", 7: "ADC转速", 8: "ADC位置转速", 9: "ADC开环", 10: "ADC位置"}

                # position
                if self.monitor_options['position'].get() and data is not None:
                    self.root.after(0, lambda p=position: self.status_labels['position'].config(text=str(p)))
                    if target_data is not None:
                        self.root.after(0, lambda tp=target_position: self.status_labels['target_position'].config(text=str(tp)))
                # step (步进加减速位置+速度)
                if self.monitor_options['step'].get() and step_data is not None:
                    self.root.after(0, lambda tp=step_target_pos: self.status_labels['step_target_pos'].config(text=str(tp)))
                    self.root.after(0, lambda cp=step_cur_pos: self.status_labels['step_cur_pos'].config(text=str(cp)))
                    # 索引转实际速度: speed = sqrt(u² + 2*a*idx)
                    import math
                    if self.acc_a > 0 and step_cur_speed >= 0:
                        cur_spd = int(math.sqrt(self.acc_u * self.acc_u + 2 * self.acc_a * step_cur_speed))
                    else:
                        cur_spd = step_cur_speed
                    if self.acc_a > 0 and step_preset_speed >= 0:
                        pre_spd = int(math.sqrt(self.acc_u * self.acc_u + 2 * self.acc_a * step_preset_speed))
                    else:
                        pre_spd = step_preset_speed
                    self.root.after(0, lambda cs=cur_spd: self.status_labels['step_cur_speed'].config(text=str(cs)))
                    self.root.after(0, lambda ps=pre_spd: self.status_labels['step_preset_speed'].config(text=str(ps)))
                # speed
                if self.monitor_options['speed'].get() and data is not None:
                    self.root.after(0, lambda s=speed: self.status_labels['speed'].config(text=str(s)))
                # mode
                if self.monitor_options['mode'].get() and data is not None:
                    self.root.after(0, lambda m=mode: self.status_labels['mode'].config(text=mode_names.get(m & 0x0F, "未知")))
                # flags (start_flag来自start_flag_data, homing/stall来自status(data[7]))
                if self.monitor_options['flags'].get():
                    if start_flag_data is not None:
                        self.root.after(0, lambda f=start_flag: self.status_labels['start_flag'].config(
                            text="已设置" if f else "未设置",
                            foreground="green" if f else "red"
                        ))
                    if data is not None:
                        self.root.after(0, lambda active=homing_active, failed=homing_failed: self.status_labels['homing'].config(
                            text="失败" if failed else ("复位中" if active else "空闲"),
                            foreground="red" if failed else ("orange" if active else "gray")
                        ))
                        self.root.after(0, lambda tripped=stall_tripped: self.status_labels['stall'].config(
                            text="已触发" if tripped else "正常",
                            foreground="red" if tripped else "gray"
                        ))
                # ADC采样值标签更新
                if self.monitor_options['adc'].get() and data is not None:
                    self.root.after(0, lambda v=supply_voltage_V: self.status_labels['supply_voltage'].config(
                        text=f"{v:.2f}V",
                        foreground="red" if v < 18.0 else "teal"  # <18V显示红色告警(典型24V系统)
                    ))
                    self.root.after(0, lambda v=ext_adc1_raw: self.status_labels['ext_adc1'].config(
                        text=f"{v}",
                        foreground="teal"
                    ))
                    self.root.after(0, lambda v=ext_adc2_raw: self.status_labels['ext_adc2'].config(
                        text=f"{v}",
                        foreground="teal"
                    ))
                # PB4/PB5引脚电平标签更新(来自status, 高电平=绿色"高", 低电平=灰色"低")
                if self.monitor_options['io'].get() and data is not None:
                    self.root.after(0, lambda h=pb4_high: self.status_labels['pb4_level'].config(
                        text="高" if h else "低",
                        foreground="green" if h else "gray"
                    ))
                    self.root.after(0, lambda h=pb5_high: self.status_labels['pb5_level'].config(
                        text="高" if h else "低",
                        foreground="green" if h else "gray"
                    ))
                # PWM显示：带方向颜色（正=红, 负=蓝, 零=灰）
                if self.monitor_options['pwm'].get() and data is not None:
                    def update_pwm(p):
                        self.status_labels['pwm'].config(
                            text=f"{p:+.1f}%",
                            foreground="red" if p > 0.1 else ("blue" if p < -0.1 else "gray")
                        )
                    self.root.after(0, lambda p=pwm_percent: update_pwm(p))

                # PID实时值标签更新
                if self.monitor_options['pid'].get() and data is not None:
                    self.root.after(0, lambda e=pid_error: self.status_labels['pid_error'].config(text=f"{e:+.2f}"))
                    self.root.after(0, lambda p=pid_p: self.status_labels['pid_p'].config(text=f"{p:+.2f}"))
                    self.root.after(0, lambda i=pid_i: self.status_labels['pid_i'].config(text=f"{i:+.2f}"))
                    self.root.after(0, lambda d=pid_d: self.status_labels['pid_d'].config(text=f"{d:+.2f}"))

                # 电流环状态标签更新
                if self.monitor_options['current'].get() and data is not None:
                    def update_cur_actual(a):
                        color = "red" if abs(a) > 5.0 else ("orange" if abs(a) > 2.0 else "teal")
                        self.status_labels['current_actual'].config(
                            text=f"{a:+.2f} A", foreground=color)
                    self.root.after(0, lambda a=cur_actual_A: update_cur_actual(a))
                    self.root.after(0, lambda a=cur_target_A: self.status_labels['current_target'].config(
                        text=f"{a:+.2f} A", foreground="teal"))
                    self.root.after(0, lambda t=over_cur_tripped: self.status_labels['over_current'].config(
                        text="已触发" if t else "正常",
                        foreground="red" if t else "gray"))
                    self.root.after(0, lambda e=cur_en: self.status_labels['cur_loop'].config(
                        text="开启" if e else "关闭",
                        foreground="green" if e else "gray"))

                # 方向寄存器读取成功才更新
                if dir_data is not None and self.monitor_options['io'].get():
                    motor_dir = dir_data[0]
                    encoder_dir = dir_data[1]
                    self.root.after(0, lambda md=motor_dir: self.status_labels['motor_dir'].config(
                        text="反转" if md else "正转"
                    ))
                    self.root.after(0, lambda ed=encoder_dir: self.status_labels['encoder_dir'].config(
                        text="反转" if ed else "正常"
                    ))

                # === 频率控制 ===
                # 支持下拉列表值(20ms/50ms/...)和手动输入(如150ms/0.5s/2s/300)
                freq_str = self.monitor_freq_var.get().strip().lower()
                sleep_time = freq_map.get(freq_str)
                if sleep_time is None:
                    # 手动输入解析: 支持"123ms"/"1.5s"/"300"(默认ms)/"0.1s"
                    import re
                    m = re.match(r'^([\d.]+)\s*(ms|s)?$', freq_str)
                    if m:
                        num = float(m.group(1))
                        unit = m.group(2) or 'ms'  # 无单位默认ms
                        sleep_time = num / 1000.0 if unit == 'ms' else num
                    else:
                        sleep_time = 0.1  # 解析失败用默认100ms
                # 限制范围: 10ms ~ 10s
                sleep_time = max(0.01, min(10.0, sleep_time))
                time.sleep(sleep_time)

            except Exception as e:
                if self.monitoring:
                    self.root.after(0, lambda: self.log(f"监控错误: {str(e)}"))
                time.sleep(0.5)
                
    def log(self, message):
        """添加日志"""
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        
    def clear_log(self):
        """清除日志"""
        self.log_text.delete(1.0, tk.END)

    def log_raw_tx(self, data):
        """记录原始发送数据包"""
        self.root.after(0, lambda d=data: self._append_raw("TX", d, "green"))

    def log_raw_rx(self, data):
        """记录原始接收数据包"""
        self.root.after(0, lambda d=data: self._append_raw("RX", d, "blue"))

    def _append_raw(self, direction, data, color):
        """向原始数据窗口添加一条记录"""
        hex_str = " ".join(f"{b:02X}" for b in data)
        timestamp = time.strftime("%H:%M:%S")
        tag = f"raw_{direction}_{timestamp}"
        self.raw_text.config(state=tk.NORMAL)
        self.raw_text.insert(tk.END, f"[{timestamp}] ", "")
        self.raw_text.insert(tk.END, f"{direction}: ", direction)
        self.raw_text.insert(tk.END, f"{hex_str}\n", "")
        self.raw_text.tag_config(direction, foreground=color, font=("Consolas", 9, "bold"))
        self.raw_text.see(tk.END)
        self.raw_text.config(state=tk.DISABLED)

        self._raw_entries += 1
        if self._raw_entries > 100000:
            half = self._raw_entries // 2
            self.raw_text.config(state=tk.NORMAL)
            self.raw_text.delete("1.0", f"{half}.0")
            self.raw_text.config(state=tk.DISABLED)
            self._raw_entries -= half

    def clear_raw(self):
        """清除原始数据"""
        self.raw_text.config(state=tk.NORMAL)
        self.raw_text.delete(1.0, tk.END)
        self.raw_text.config(state=tk.DISABLED)
        self._raw_entries = 0

    # ========== 转速采集功能 ==========

    def _update_channel_visibility(self):
        """根据通道数显示/隐藏通道下拉列表(纵向排列)"""
        try:
            nch = int(self.acq_channels_var.get())
        except ValueError:
            nch = 1
        nch = max(1, min(4, nch))
        # 先全部隐藏
        for lbl, combo, ch_row in self.acq_type_combos:
            ch_row.pack_forget()
        # 重新显示前nch个(纵向排列, 每个通道独占一行)
        for i in range(nch):
            lbl, combo, ch_row = self.acq_type_combos[i]
            ch_row.pack(fill=tk.X, pady=1)

    def _current_nch(self):
        """当前通道数(1~4)"""
        try:
            nch = int(self.acq_channels_var.get())
        except ValueError:
            nch = 1
        return max(1, min(4, nch))

    def _current_acq_types(self):
        """返回所有通道的采集类型列表, 长度=通道数"""
        nch = self._current_nch()
        types = []
        for i in range(nch):
            types.append(self._ACQ_LABEL_TO_TYPE.get(self.acq_type_vars[i].get(), 0))
        return types

    def on_acq_type_changed(self, event=None):
        """采集类型切换回调(多通道版)"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return
        # 更新各通道Y轴标签和颜色
        nch = self._current_nch()
        types = self._current_acq_types()
        for ch_idx in range(min(nch, len(self.speed_axes))):
            ax = self.speed_axes[ch_idx]
            acq_type = types[ch_idx] if ch_idx < len(types) else 0
            color = self._CH_COLORS[ch_idx % len(self._CH_COLORS)]
            ax.set_ylabel(self._acq_y_label(acq_type), color=color)
            ax.tick_params(axis='y', labelcolor=color)
        self.speed_canvas.draw()
        # 写寄存器放到线程中: 写通道数 + 所有通道类型
        nch = self._current_nch()
        types = self._current_acq_types()
        reg_map = [self.REG_SPEED_ACQ_TYPE, self.REG_SPEED_ACQ_TYPE2,
                   self.REG_SPEED_ACQ_TYPE3, self.REG_SPEED_ACQ_TYPE4]
        def _worker():
            try:
                with self.serial_lock:
                    self.modbus.write_single_register(self.get_slave_addr(), self.REG_SPEED_ACQ_CHANNELS, nch)
                    for i in range(nch):
                        self.modbus.write_single_register(self.get_slave_addr(), reg_map[i], types[i])
                labels = [self._acq_label(t) for t in types]
                self.root.after(0, lambda: self.log(f"设置采集: {nch}通道, 类型={labels}"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"设置采集类型失败: {e}"))
        threading.Thread(target=_worker, daemon=True).start()

    # 采集类型映射表: 0=转速, 1=PWM, 2=位置, 3=电流, 4=PC0电压ADC, 5=PC2外部ADC, 6=PC3外部ADC
    _ACQ_LABELS = {0: "转速", 1: "PWM输出", 2: "位置", 3: "电流",
                   4: "PC0电压ADC", 5: "PC2外部ADC", 6: "PC3外部ADC", 7: "电流环开-位置P", 8: "电流环开-位置I", 9: "电流环开-位置D", 10: "电流环开-速度P", 11: "电流环开-速度I", 12: "电流环开-速度D", 13: "电流环开-电流P", 14: "电流环开-电流I", 15: "电流环开-电流D", 16: "电流环关-位置P", 17: "电流环关-位置I", 18: "电流环关-位置D", 19: "电流环关-速度P", 20: "电流环关-速度I", 21: "电流环关-速度D", 22: "位置误差", 23: "速度误差"}
    _ACQ_Y_LABELS = {0: "转速 (脉冲/秒)", 1: "PWM输出", 2: "位置偏移 (脉冲)", 3: "电流 (A)",
                     4: "供电电压 (V)", 5: "PC2外部ADC原始值 (0~4095)", 6: "PC3外部ADC原始值 (0~4095)", 7: "位置环P输出", 8: "位置环I输出", 9: "位置环D输出", 10: "速度环P输出", 11: "速度环I输出", 12: "速度环D输出", 13: "电流环P输出", 14: "电流环I输出", 15: "电流环D输出", 16: "位置环P输出", 17: "位置环I输出", 18: "位置环D输出", 19: "速度环P输出", 20: "速度环I输出", 21: "速度环D输出", 22: "位置误差 (脉冲)", 23: "速度误差 (脉冲/秒)"}
    _ACQ_TITLES = {0: "转速采集曲线", 1: "PWM采集曲线", 2: "位置采集曲线", 3: "电流采集曲线",
                   4: "供电电压采集曲线", 5: "PC2外部ADC采集曲线", 6: "PC3外部ADC采集曲线", 7: "电流环开-位置P曲线", 8: "电流环开-位置I曲线", 9: "电流环开-位置D曲线", 10: "电流环开-速度P曲线", 11: "电流环开-速度I曲线", 12: "电流环开-速度D曲线", 13: "电流环开-电流P曲线", 14: "电流环开-电流I曲线", 15: "电流环开-电流D曲线", 16: "电流环关-位置P曲线", 17: "电流环关-位置I曲线", 18: "电流环关-位置D曲线", 19: "电流环关-速度P曲线", 20: "电流环关-速度I曲线", 21: "电流环关-速度D曲线", 22: "位置误差采集曲线", 23: "速度误差采集曲线"}
    # 数据换算系数: 原始值 × 系数 + 偏置 = 显示值
    # 电流: 固件存储A×1000 (1A=1000, 满量程±8.25A=±8250) → A = 原始值 × 0.001
    # PC0电压: ADC 0~4095, 100K:10K分压×11, 3.3V参考 → V = ADC × 3.3×11/4095
    _ACQ_SCALE = {3: 0.001, 4: 3.3 * 11.0 / 4095.0}
    _ACQ_LABEL_TO_TYPE = {"转速": 0, "PWM": 1, "位置": 2, "电流": 3,
                          "PC0电压ADC": 4, "PC2外部ADC": 5, "PC3外部ADC": 6, "电流环开-位置P": 7, "电流环开-位置I": 8, "电流环开-位置D": 9, "电流环开-速度P": 10, "电流环开-速度I": 11, "电流环开-速度D": 12, "电流环开-电流P": 13, "电流环开-电流I": 14, "电流环开-电流D": 15, "电流环关-位置P": 16, "电流环关-位置I": 17, "电流环关-位置D": 18, "电流环关-速度P": 19, "电流环关-速度I": 20, "电流环关-速度D": 21, "位置误差": 22, "速度误差": 23}

    def _current_acq_type(self):
        """当前采集类型 0=转速 1=PWM 2=位置"""
        return self._ACQ_LABEL_TO_TYPE.get(self.acq_type_var.get(), 0)

    def _acq_label(self, acq_type):
        """类型代号转中文标签"""
        return self._ACQ_LABELS.get(acq_type, "转速")

    def _acq_y_label(self, acq_type):
        """类型代号转Y轴标签"""
        return self._ACQ_Y_LABELS.get(acq_type, "转速 (脉冲/秒)")

    def _acq_title(self, acq_type):
        """类型代号转图表标题"""
        return self._ACQ_TITLES.get(acq_type, "转速采集曲线")

    def set_speed_acq_params(self):
        """设置采集分频值、采集点数、通道数和各通道类型"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return
        try:
            div = int(self.acq_div_var.get())
            if div < 1:
                raise ValueError
        except ValueError:
            messagebox.showwarning("警告", "分频值必须为正整数")
            return
        try:
            size = int(self.acq_size_var.get())
            if size < 1 or size > self.SPEED_ACQ_BUF_SIZE:
                raise ValueError
        except ValueError:
            messagebox.showwarning("警告", f"采集点数必须为1~{self.SPEED_ACQ_BUF_SIZE}的整数")
            return
        nch = self._current_nch()
        types = self._current_acq_types()
        reg_map = [self.REG_SPEED_ACQ_TYPE, self.REG_SPEED_ACQ_TYPE2,
                   self.REG_SPEED_ACQ_TYPE3, self.REG_SPEED_ACQ_TYPE4]
        # 写寄存器放到线程中，避免阻塞UI
        self.acq_status_label.config(text="状态: 设置中...", foreground="blue")
        def _worker():
            try:
                with self.serial_lock:
                    self.modbus.write_single_register(self.get_slave_addr(), self.REG_SPEED_ACQ_DIV, div)
                    self.modbus.write_single_register(self.get_slave_addr(), self.REG_SPEED_ACQ_SIZE, size)
                    self.modbus.write_single_register(self.get_slave_addr(), self.REG_SPEED_ACQ_CHANNELS, nch)
                    for i in range(nch):
                        self.modbus.write_single_register(self.get_slave_addr(), reg_map[i], types[i])
                labels = [self._acq_label(t) for t in types]
                self.root.after(0, lambda: self.acq_status_label.config(text="状态: 已设置", foreground="gray"))
                self.root.after(0, lambda: self.log(f"设置采集参数: {nch}通道, 分频={div}(周期={div*0.1}ms), 点数={size}, 类型={labels}"))
            except Exception as e:
                self.root.after(0, lambda: self.acq_status_label.config(text="状态: 设置失败", foreground="red"))
                self.root.after(0, lambda: messagebox.showerror("错误", f"设置参数失败: {e}"))
        threading.Thread(target=_worker, daemon=True).start()

    def start_speed_acq(self):
        """启动转速采集"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return
        # 写寄存器放到线程中，避免阻塞UI
        def _worker():
            try:
                with self.serial_lock:
                    self.modbus.write_single_register(self.get_slave_addr(), self.REG_SPEED_ACQ_START, 1)
                self.root.after(0, lambda: self.acq_status_label.config(text="状态: 采集中", foreground="orange"))
                self.root.after(0, lambda: self.log("启动转速采集"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"启动采集失败: {e}"))
        threading.Thread(target=_worker, daemon=True).start()

    def read_and_plot_speed(self):
        """读取转速数据并绘图（在线程中执行避免阻塞UI）"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return
        if self.acq_reading:
            messagebox.showwarning("提示", "正在读取采集数据，请等待完成")
            return
        self.acq_status_label.config(text="状态: 读取中...", foreground="blue")
        threading.Thread(target=self._read_and_plot_worker, daemon=True).start()

    def _read_and_plot_worker(self):
        """读取转速数据的工作线程"""
        self.acq_reading = True  # 暂停监控线程
        try:
            with self.serial_lock:
                # 读取采集状态和已采样数量
                status_data = self.modbus.read_holding_registers(
                    self.get_slave_addr(), self.REG_SPEED_ACQ_START, 4)
                status = status_data[0]  # 0=空闲, 1=采集中, 2=完成
                count = status_data[2]   # 已采样数量

            if count == 0:
                self.root.after(0, lambda: self.acq_status_label.config(
                    text="状态: 无数据", foreground="gray"))
                self.root.after(0, lambda: messagebox.showinfo("提示", "没有采集到数据"))
                return

            # 分批读取数据寄存器（每次最多125个），整体持锁避免监控线程干扰
            speed_data = []
            remaining = count
            offset = 0
            with self.serial_lock:  # 一次性持锁读完所有数据
                while remaining > 0:
                    chunk = min(remaining, 125)
                    data = self.modbus.read_holding_registers(
                        self.get_slave_addr(),
                        self.REG_SPEED_DATA_BASE + offset,
                        chunk)
                    for v in data:
                        # int16_t 有符号转换
                        if v >= 0x8000:
                            v -= 0x10000
                        speed_data.append(float(v))  # 脉冲/秒
                    offset += chunk
                    remaining -= chunk

            # 更新状态显示
            status_text = {0: "空闲", 1: "采集中", 2: "完成"}.get(status, "未知")
            status_color = {0: "gray", 1: "orange", 2: "green"}.get(status, "gray")
            self.root.after(0, lambda: self.acq_status_label.config(
                text=f"状态: {status_text} ({count}点)", foreground=status_color))

            # 多通道解交错: 缓冲区数据格式 [ch1_t0, ch2_t0, ..., chN_t0, ch1_t1, ...]
            nch = self._current_nch()
            types = self._current_acq_types()
            # 按通道拆分数据
            channel_data = [[] for _ in range(nch)]
            for i, v in enumerate(speed_data):
                ch = i % nch
                channel_data[ch].append(v)

            # 绘图
            self.root.after(0, lambda: self._plot_speed_data(channel_data, types))
            labels = [self._acq_label(t) for t in types]
            per_ch = len(channel_data[0]) if channel_data else 0
            self.root.after(0, lambda: self.log(f"读取{count}点({nch}通道×{per_ch}点/通道), 类型={labels}, 绘图完成"))

        except Exception as e:
            self.root.after(0, lambda: self.acq_status_label.config(
                text="状态: 读取失败", foreground="red"))
            self.root.after(0, lambda: messagebox.showerror("错误", f"读取数据失败: {e}"))
        finally:
            self.acq_reading = False  # 恢复监控线程

    # 多通道绘图颜色 (最多4通道)
    _CH_COLORS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']

    def _plot_speed_data(self, channel_data, types):
        """绘制多通道采集曲线, 每通道独立子图+独立Y轴
        多通道时共享X轴(缩放/平移同步), 各通道Y轴独立缩放
        channel_data: 各通道数据列表 [[ch1_data], [ch2_data], ...]
        types: 各通道采集类型列表"""
        nch = len(channel_data)
        # 清空figure, 按通道数创建子图
        self.speed_fig.clear()
        if nch == 1:
            axes = [self.speed_fig.add_subplot(111)]
        else:
            # 多通道: 第一个子图正常创建, 其余共享X轴(缩放/平移同步)
            axes = [self.speed_fig.add_subplot(nch, 1, 1)]
            for i in range(1, nch):
                axes.append(self.speed_fig.add_subplot(nch, 1, i + 1, sharex=axes[0]))
        self.speed_axes = axes
        self.speed_ax = axes[0]  # 兼容旧代码引用

        for ch_idx in range(nch):
            ax = axes[ch_idx]
            data = channel_data[ch_idx]
            acq_type = types[ch_idx] if ch_idx < len(types) else 0
            # 电流/电压类型: 换算为带物理单位的值
            scale = self._ACQ_SCALE.get(acq_type)
            if scale is not None:
                plot_data = [v * scale for v in data]
            else:
                plot_data = data
            color = self._CH_COLORS[ch_idx % len(self._CH_COLORS)]
            label = f"CH{ch_idx+1}: {self._acq_label(acq_type)}"
            ax.plot(range(len(plot_data)), plot_data,
                    color=color, linewidth=0.8, label=label)
            # 每个通道独立Y轴标签
            ax.set_ylabel(self._acq_y_label(acq_type), color=color)
            ax.tick_params(axis='y', labelcolor=color)
            ax.grid(True)
            ax.legend(loc='best', fontsize=8)
            # 只有最底部子图显示X轴标签和刻度
            if ch_idx == nch - 1:
                ax.set_xlabel("采样点 (每通道)")
            else:
                # 共享X轴时隐藏非底部子图的X轴刻度标签(避免重叠)
                ax.tick_params(labelbottom=False)

        # 标题
        if nch == 1:
            acq_type = types[0] if types else 0
            axes[0].set_title(f"{self._acq_title(acq_type)} ({len(channel_data[0])}点)")
        else:
            per_ch = len(channel_data[0]) if channel_data else 0
            self.speed_fig.suptitle(f"多通道采集 ({nch}通道×{per_ch}点/通道)", fontsize=10)
        self.speed_fig.tight_layout()
        self.speed_canvas.draw()

    def clear_speed_plot(self):
        """清空采集曲线"""
        nch = self._current_nch()
        types = self._current_acq_types()
        self.speed_fig.clear()
        if nch == 1:
            axes = [self.speed_fig.add_subplot(111)]
        else:
            # 多通道: 共享X轴(缩放/平移同步), 各通道Y轴独立
            axes = [self.speed_fig.add_subplot(nch, 1, 1)]
            for i in range(1, nch):
                axes.append(self.speed_fig.add_subplot(nch, 1, i + 1, sharex=axes[0]))
        self.speed_axes = axes
        self.speed_ax = axes[0]
        for ch_idx in range(nch):
            ax = axes[ch_idx]
            acq_type = types[ch_idx] if ch_idx < len(types) else 0
            color = self._CH_COLORS[ch_idx % len(self._CH_COLORS)]
            ax.set_ylabel(self._acq_y_label(acq_type), color=color)
            ax.tick_params(axis='y', labelcolor=color)
            ax.grid(True)
            if ch_idx == nch - 1:
                ax.set_xlabel("采样点 (每通道)")
            else:
                ax.tick_params(labelbottom=False)
        if nch == 1:
            acq_type = types[0] if types else 0
            axes[0].set_title(self._acq_title(acq_type))
        else:
            self.speed_fig.suptitle(f"多通道采集 ({nch}通道)", fontsize=10)
        self.speed_fig.tight_layout()
        self.speed_canvas.draw()
        self.acq_status_label.config(text="状态: 空闲", foreground="gray")
        self.log("清空采集曲线")

    # ========== 寄存器列表功能 ==========

    def open_acc_table_window(self):
        """打开加速表生成窗口(新增)"""
        win = tk.Toplevel(self.root)
        win.title("加速表生成")
        win.geometry("420x400")
        win.transient(self.root)
        win.grab_set()

        # 参数输入区
        param_frame = ttk.LabelFrame(win, text="参数设置", padding=15)
        param_frame.pack(fill=tk.X, padx=10, pady=5)

        ttk.Label(param_frame, text="主频 (Hz):").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        timc_var = tk.StringVar(value="32000000")
        ttk.Entry(param_frame, textvariable=timc_var, width=20).grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(param_frame, text="最小速度 (步/秒):").grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        u_var = tk.StringVar(value="500")
        ttk.Entry(param_frame, textvariable=u_var, width=20).grid(row=1, column=1, padx=5, pady=5)

        ttk.Label(param_frame, text="加速度 (步/秒²):").grid(row=2, column=0, padx=5, pady=5, sticky=tk.W)
        a_var = tk.StringVar(value="320000")
        ttk.Entry(param_frame, textvariable=a_var, width=20).grid(row=2, column=1, padx=5, pady=5)

        ttk.Label(param_frame, text="表长度 (最大10015):").grid(row=3, column=0, padx=5, pady=5, sticky=tk.W)
        len_var = tk.StringVar(value="10015")
        ttk.Entry(param_frame, textvariable=len_var, width=20).grid(row=3, column=1, padx=5, pady=5)

        # 说明
        ttk.Label(param_frame, text="公式: table[i] = 主频 / √(最小速度² + 2×加速度×i)",
                  foreground="gray").grid(row=4, column=0, columnspan=2, padx=5, pady=(10,0))

        # 操作按钮区
        btn_frame = ttk.Frame(win)
        btn_frame.pack(fill=tk.X, padx=10, pady=5)

        # 状态显示区
        status_frame = ttk.LabelFrame(win, text="状态", padding=10)
        status_frame.pack(fill=tk.X, padx=10, pady=5)

        status_label = ttk.Label(status_frame, text="就绪", font=("Arial", 11))
        status_label.pack(anchor=tk.W)

        progress_var = tk.DoubleVar(value=0)
        progress_bar = ttk.Progressbar(status_frame, variable=progress_var, maximum=1000, length=350)
        progress_bar.pack(fill=tk.X, pady=5)

        current_status_label = ttk.Label(status_frame, text="当前表: 未知", foreground="blue")
        current_status_label.pack(anchor=tk.W)

        def read_current_status():
            """读取当前加速表状态"""
            if not self.modbus:
                messagebox.showwarning("警告", "请先连接串口", parent=win)
                return
            def do_read():
                try:
                    if self.serial_lock.acquire(timeout=1.0):
                        try:
                            # 读取参数和状态
                            data = self.modbus.read_holding_registers(
                                self.get_slave_addr(), self.REG_ACC_TABLE_TIMC_L, 9)
                            timc = (data[1] << 16) | data[0]
                            u = data[2]
                            a_val = (data[4] << 16) | data[3]
                            length = data[5]
                            status = data[7]
                            progress = data[8]
                            status_map = {0: "ROM默认表", 1: "计算中...", 2: "Flash自定义表", 3: "错误"}
                            status_text = status_map.get(status, str(status))
                            def update_ui():
                                timc_var.set(str(timc))
                                u_var.set(str(u))
                                a_var.set(str(a_val))
                                len_var.set(str(length))
                                current_status_label.config(text=f"当前表: {status_text}")
                                progress_var.set(progress)
                                if status == 1:
                                    status_label.config(text="STM32正在计算...", foreground="orange")
                                elif status == 2:
                                    status_label.config(text=f"自定义表已启用 (进度:{progress}/1000)", foreground="green")
                                else:
                                    status_label.config(text=f"使用ROM默认表 (进度:{progress}/1000)", foreground="black")
                            self.root.after(0, update_ui)
                            self.root.after(0, lambda: self.log(f"读取加速表状态: {status_text}, timc={timc}, u={u}, a={a_val}, len={length}"))
                        finally:
                            self.serial_lock.release()
                    else:
                        self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙", parent=win))
                except Exception as e:
                    self.root.after(0, lambda: messagebox.showerror("错误", f"读取失败: {e}", parent=win))
            threading.Thread(target=do_read, daemon=True).start()

        def start_calculate():
            """发送参数并触发计算"""
            if not self.modbus:
                messagebox.showwarning("警告", "请先连接串口", parent=win)
                return
            try:
                timc = int(timc_var.get())
                u = int(u_var.get())
                a_val = int(a_var.get())
                length = int(len_var.get())
            except ValueError:
                messagebox.showerror("错误", "参数必须为整数", parent=win)
                return
            if timc <= 0 or u <= 0 or a_val <= 0 or length <= 0:
                messagebox.showerror("错误", "参数必须大于0", parent=win)
                return
            if length > 10015:
                messagebox.showerror("错误", "表长度最大10015", parent=win)
                return
            if not messagebox.askyesno("确认", f"将计算加速表并存入Flash:\n主频={timc}\n最小速度={u}\n加速度={a_val}\n长度={length}\n\n计算期间电机将暂停。确认?", parent=win):
                return

            def do_calculate():
                try:
                    if self.serial_lock.acquire(timeout=2.0):
                        try:
                            addr = self.get_slave_addr()
                            # 写入参数(用功能码0x10批量写)
                            self.modbus.write_multiple_registers(addr, self.REG_ACC_TABLE_TIMC_L,
                                [timc & 0xFFFF, (timc >> 16) & 0xFFFF, u & 0xFFFF,
                                 a_val & 0xFFFF, (a_val >> 16) & 0xFFFF, length & 0xFFFF])
                            # 触发计算(写0x0098=1)
                            self.modbus.write_single_register(addr, self.REG_ACC_TABLE_TRIGGER, 1)
                            self.root.after(0, lambda: status_label.config(text="已触发,等待STM32计算...", foreground="orange"))
                            self.root.after(0, lambda: self.log(f"触发加速表计算: timc={timc}, u={u}, a={a_val}, len={length}"))
                            # 轮询进度
                            import time
                            for _ in range(120):  # 最多等60秒
                                time.sleep(0.5)
                                try:
                                    st = self.modbus.read_holding_registers(addr, self.REG_ACC_TABLE_STATUS, 2)
                                    progress = st[1]
                                    status_val = st[0]
                                    self.root.after(0, lambda p=progress: progress_var.set(p))
                                    if status_val == 1:  # 计算中
                                        self.root.after(0, lambda p=progress: status_label.config(
                                            text=f"计算中... {p}/1000", foreground="orange"))
                                    elif status_val == 2:  # 完成
                                        self.root.after(0, lambda: status_label.config(
                                            text="计算完成! Flash自定义表已启用", foreground="green"))
                                        self.root.after(0, lambda: current_status_label.config(
                                            text="当前表: Flash自定义表", foreground="green"))
                                        self.root.after(0, lambda: self.log("加速表计算完成, Flash自定义表已启用"))
                                        return
                                    elif status_val == 3:  # 错误
                                        self.root.after(0, lambda: status_label.config(
                                            text="计算失败!", foreground="red"))
                                        self.root.after(0, lambda: messagebox.showerror("错误", "STM32计算失败,请检查参数", parent=win))
                                        return
                                    else:  # status==0, 意外回到ROM
                                        self.root.after(0, lambda: status_label.config(
                                            text="已完成(ROM表)", foreground="black"))
                                        return
                                except Exception:
                                    pass
                            self.root.after(0, lambda: status_label.config(text="超时,请手动刷新查看状态", foreground="red"))
                        finally:
                            self.serial_lock.release()
                    else:
                        self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙", parent=win))
                except Exception as e:
                    self.root.after(0, lambda: messagebox.showerror("错误", f"操作失败: {e}", parent=win))

            threading.Thread(target=do_calculate, daemon=True).start()

        def restore_rom():
            """恢复ROM默认表"""
            if not self.modbus:
                messagebox.showwarning("警告", "请先连接串口", parent=win)
                return
            if not messagebox.askyesno("确认", "恢复ROM默认加速表? 当前Flash自定义表将被清除", parent=win):
                return
            def do_restore():
                try:
                    if self.serial_lock.acquire(timeout=1.0):
                        try:
                            self.modbus.write_single_register(self.get_slave_addr(), self.REG_ACC_TABLE_TRIGGER, 2)
                            self.root.after(0, lambda: status_label.config(text="已恢复ROM默认表", foreground="black"))
                            self.root.after(0, lambda: current_status_label.config(text="当前表: ROM默认表", foreground="blue"))
                            self.root.after(0, lambda: progress_var.set(1000))
                            self.root.after(0, lambda: self.log("恢复ROM默认加速表"))
                        finally:
                            self.serial_lock.release()
                except Exception as e:
                    self.root.after(0, lambda: messagebox.showerror("错误", f"操作失败: {e}", parent=win))
            threading.Thread(target=do_restore, daemon=True).start()

        ttk.Button(btn_frame, text="读取当前状态", command=read_current_status).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="开始计算", command=start_calculate).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="恢复ROM默认", command=restore_rom).pack(side=tk.LEFT, padx=5)

        # 自动读取一次
        win.after(500, read_current_status)

    def open_register_list(self):
        """打开寄存器列表窗口"""
        # 窗口已存在则前置
        if self.reg_list_win is not None:
            try:
                if self.reg_list_win.winfo_exists():
                    self.reg_list_win.lift()
                    self.reg_list_win.focus_force()
                    return
            except Exception:
                pass

        win = tk.Toplevel(self.root)
        win.title("寄存器列表 (0x0000~0x0068)")
        win.geometry("950x650")

        # 工具栏
        toolbar = ttk.Frame(win)
        toolbar.pack(fill=tk.X, padx=5, pady=5)
        ttk.Button(toolbar, text="读取全部", command=self._read_all_registers).pack(side=tk.LEFT, padx=5)
        ttk.Button(toolbar, text="导出配置", command=self._export_config).pack(side=tk.LEFT, padx=5)
        ttk.Button(toolbar, text="导入配置", command=self._import_config).pack(side=tk.LEFT, padx=5)
        ttk.Button(toolbar, text="写入全部", command=self._write_all_registers).pack(side=tk.LEFT, padx=5)
        ttk.Button(toolbar, text="载入到运行窗口", command=self._load_to_main_window).pack(side=tk.LEFT, padx=5)
        ttk.Label(toolbar, text="(双击'原始值'列可编辑, 触发/只读寄存器写入时自动跳过)",
                  foreground="gray").pack(side=tk.LEFT, padx=10)

        # Treeview + 滚动条
        tree_frame = ttk.Frame(win)
        tree_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=(0, 5))

        columns = ("addr", "name", "desc", "value", "parsed")
        tree = ttk.Treeview(tree_frame, columns=columns, show="headings")
        tree.heading("addr", text="地址")
        tree.heading("name", text="寄存器名")
        tree.heading("desc", text="描述")
        tree.heading("value", text="原始值")
        tree.heading("parsed", text="解析值")
        tree.column("addr", width=70, anchor=tk.W)
        tree.column("name", width=220, anchor=tk.W)
        tree.column("desc", width=200, anchor=tk.W)
        tree.column("value", width=80, anchor=tk.W)
        tree.column("parsed", width=280, anchor=tk.W)

        vsb = ttk.Scrollbar(tree_frame, orient=tk.VERTICAL, command=tree.yview)
        hsb = ttk.Scrollbar(tree_frame, orient=tk.HORIZONTAL, command=tree.xview)
        tree.configure(yscrollcommand=vsb.set, xscrollcommand=hsb.set)

        tree.grid(row=0, column=0, sticky=tk.NSEW)
        vsb.grid(row=0, column=1, sticky=tk.NS)
        hsb.grid(row=1, column=0, sticky=tk.EW)
        tree_frame.rowconfigure(0, weight=1)
        tree_frame.columnconfigure(0, weight=1)

        # 填充寄存器列表
        for addr, name, desc, fmt in self.REGISTER_LIST:
            tree.insert("", tk.END, iid=str(addr),
                        values=(f"0x{addr:04X}", name, desc, "", fmt))

        # 双击"原始值"列编辑
        tree.bind("<Double-1>", lambda e: self._edit_register_value(tree))

        self.reg_list_tree = tree
        self.reg_list_win = win

        # 窗口关闭时清理引用
        def _on_close():
            self.reg_list_win = None
            self.reg_list_tree = None
            win.destroy()
        win.protocol("WM_DELETE_WINDOW", _on_close)

    def _edit_register_value(self, tree):
        """双击Treeview编辑原始值列"""
        sel = tree.selection()
        if not sel:
            return
        item = sel[0]
        # 判断双击的列: 只允许编辑"原始值"(#4)
        col_id = tree.identify("column", tree.winfo_pointerx() - tree.winfo_rootx(),
                                          tree.winfo_pointery() - tree.winfo_rooty())
        if col_id != "#4":
            return
        try:
            addr = int(item)
        except ValueError:
            return
        cur_vals = tree.item(item)['values']
        cur_val = cur_vals[3] if len(cur_vals) > 3 and cur_vals[3] not in (None, "") else ""

        dlg = tk.Toplevel(self.reg_list_win)
        dlg.title(f"编辑 0x{addr:04X}")
        dlg.geometry("340x160")
        dlg.transient(self.reg_list_win)
        dlg.grab_set()

        ttk.Label(dlg, text=f"寄存器: 0x{addr:04X}  ({cur_vals[1]})",
                  font=("Arial", 10, "bold")).pack(pady=5, anchor=tk.W, padx=10)
        ttk.Label(dlg, text=f"描述: {cur_vals[2]}").pack(pady=2, anchor=tk.W, padx=10)
        ttk.Label(dlg, text=f"格式: {cur_vals[4]}").pack(pady=2, anchor=tk.W, padx=10)
        val_var = tk.StringVar(value=str(cur_val))
        ttk.Entry(dlg, textvariable=val_var, width=22).pack(pady=5, padx=10)

        def _save():
            raw = val_var.get().strip()
            try:
                # 支持0x十六进制或十进制
                if raw.lower().startswith('0x'):
                    v = int(raw, 16)
                else:
                    v = int(raw)
            except ValueError:
                messagebox.showerror("错误", "请输入整数(支持0x十六进制)", parent=dlg)
                return
            parsed = self._parse_register(addr, v)
            tree.item(item, values=(
                f"0x{addr:04X}", cur_vals[1], cur_vals[2], str(v), parsed))
            dlg.destroy()

        btn_frame = ttk.Frame(dlg)
        btn_frame.pack(pady=5)
        ttk.Button(btn_frame, text="确定", command=_save).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="取消", command=dlg.destroy).pack(side=tk.LEFT, padx=5)

    def _read_all_registers(self):
        """批量读取所有寄存器并更新Treeview"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return
        if not (self.reg_list_tree and self.reg_list_win and self.reg_list_win.winfo_exists()):
            return

        tree = self.reg_list_tree

        def do_read():
            try:
                if not self.serial_lock.acquire(timeout=2.0):
                    self.reg_list_win.after(0, lambda: messagebox.showwarning("警告", "串口忙，请稍后再试"))
                    return
                try:
                    # 分段读取（每段最多100个寄存器, 避开MODBUS单次125上限留余量）
                    # 第1段: 0x0000~0x0044 (共0x45=69个)
                    # 第2段: 0x0045~0x006A (共0x26=38个, 含ADC滤波窗口REG_ADC_FILTER_SIZE=0x006A)
                    for start, count in [(0x0000, 0x0045), (0x0045, 0x0026), (0x006B, 0x0026), (0x0091, 0x0001), (0x0092, 0x0009)]:
                        try:
                            data = self.modbus.read_holding_registers(
                                self.get_slave_addr(), start, count)
                            for i, val in enumerate(data):
                                addr = start + i
                                if not tree.exists(str(addr)):
                                    continue
                                parsed = self._parse_register(addr, val)
                                old = tree.item(str(addr))['values']
                                # old: (addr, name, desc, value, parsed)
                                new_vals = (f"0x{addr:04X}", old[1], old[2], str(val), parsed)
                                # 在主线程更新UI
                                self.reg_list_win.after(0, lambda a=addr, v=new_vals:
                                    tree.item(str(a), values=v))
                        except Exception as e:
                            # 某段读取失败（可能含只写寄存器）记录日志并继续下一段
                            print(f"读取0x{start:04X}~0x{start+count-1:04X}失败: {e}")
                            self.reg_list_win.after(0, lambda s=start, c=count, err=str(e):
                                self.log(f"读取0x{s:04X}~0x{s+c-1:04X}失败: {err}"))
                    self.reg_list_win.after(0, lambda: self.log("寄存器批量读取完成"))
                finally:
                    self.serial_lock.release()
            except Exception as e:
                self.reg_list_win.after(0, lambda: messagebox.showerror("错误", f"读取失败: {e}"))

        threading.Thread(target=do_read, daemon=True).start()

    def _parse_register(self, addr, val):
        """根据寄存器地址解析值的可读含义，覆盖0x0000~0x006A全部寄存器"""
        try:
            # 16位有符号转换辅助
            def s16(v):
                return v - 0x10000 if v >= 0x8000 else v

            # === 0x0000~0x000F: 控制与PID ===
            if addr == 0x0000:  # REG_CONTROL_WORD - 模式设置
                names = {0: "0=位置", 1: "1=速度", 2: "2=位置速度", 3: "3=开环",
                         4: "4=外部位置", 5: "5=外部速度", 6: "6=待机",
                         7: "7=ADC转速", 8: "8=ADC位置转速", 9: "9=ADC开环", 10: "10=ADC位置"}
                return names.get(val, f"{val}=未知")
            elif addr == 0x0001:  # REG_MODE_SET - 启动方式
                return "直接启动" if val == 0 else ("标志位启动" if val == 1 else str(val))
            elif addr == 0x0002:  # 目标位置[63:48]
                return f"int64高位[63:48] (原始值:{val})"
            elif addr == 0x0003:  # 目标位置[47:32]
                return f"int64次高位[47:32] (原始值:{val})"
            elif addr == 0x0004:  # 目标位置[31:16]
                return f"int64次低位[31:16] (原始值:{val})"
            elif addr == 0x0005:  # 目标位置[15:0]
                return f"int64低位[15:0] (原始值:{val})"
            elif addr == 0x0006:  # 目标速度高16位
                return f"int32高位[31:16] (原始值:{val})"
            elif addr == 0x0007:  # 目标速度低16位
                return f"int32低位[15:0] (原始值:{val})"
            elif addr == 0x0008:  # 位置环Kp ×100
                return f"Kp={s16(val) / 100.0:.2f} (原始值:{val})"
            elif addr == 0x0009:  # 位置环Ki ×100
                return f"Ki={s16(val) / 100.0:.2f} (原始值:{val})"
            elif addr == 0x000A:  # 位置环Kd ×1000
                return f"Kd={s16(val) / 1000.0:.3f} (原始值:{val})"
            elif addr == 0x000B:  # 速度环Kp ×100
                return f"Kp={s16(val) / 100.0:.2f} (原始值:{val})"
            elif addr == 0x000C:  # 速度环Ki ×100
                return f"Ki={s16(val) / 100.0:.2f} (原始值:{val})"
            elif addr == 0x000D:  # 速度环Kd ×1000
                return f"Kd={s16(val) / 1000.0:.3f} (原始值:{val})"
            elif addr == 0x000E:  # REG_DEAD_ZONE - 死区
                return f"{val} 脉冲/脉冲·秒⁻¹ (原始值:{val})"
            elif addr == 0x000F:  # REG_MAX_OUTPUT - 最大输出 PWM限幅×10
                return f"{val / 10.0:.1f}% (原始值:{val})"

            # === 0x0010~0x001F: 触发/方向/地址/波特率/复位 ===
            elif addr == 0x0010:  # REG_START_MODE - 启动触发
                return "写1触发(只写)"
            elif addr == 0x0011:  # REG_DIRECTION - 电机方向
                return "正转" if val == 0 else ("反转" if val == 1 else str(val))
            elif addr == 0x0012:  # REG_ENCODER_DIRECTION - 编码器方向
                return "正常" if val == 0 else ("反转" if val == 1 else str(val))
            elif addr == 0x0013:  # REG_START_FLAG - 保留寄存器
                return "保留寄存器"
            elif addr == 0x0014:  # REG_SLAVE_ADDR - MODBUS从机地址
                return f"地址: {val} (1~247)"
            elif addr == 0x0015:  # REG_BAUD_RATE - MODBUS波特率 波特率/100
                return f"{val * 100}bps (原始值:{val})"
            elif addr == 0x0016:  # REG_SET_ORIGIN - 设置原点
                return "写1触发(只写)"
            elif addr == 0x0017:  # REG_HOME_MODE - 复位模式
                return {0: "关闭", 1: "堵转", 2: "精确复位"}.get(val, str(val))
            elif addr == 0x0018:  # REG_HOME_DIRECTION - 复位方向
                return "负方向" if val == 0 else ("正方向" if val == 1 else str(val))
            elif addr == 0x0019:  # REG_HOME_CURRENT - 复位电流限制 PWM值
                return f"PWM值: {val} (原始值:{val})"
            elif addr == 0x001A:  # 复位速度高16位
                return f"int32高位[31:16] (原始值:{val})"
            elif addr == 0x001B:  # 复位速度低16位
                return f"int32低位[15:0] (原始值:{val})"
            elif addr == 0x001C:  # 复位最大距离高16位
                return f"int32高位[31:16] (原始值:{val})"
            elif addr == 0x001D:  # 复位最大距离低16位
                return f"int32低位[15:0] (原始值:{val})"
            elif addr == 0x001E:  # 复位偏置高16位
                return f"int32高位[31:16] (原始值:{val})"
            elif addr == 0x001F:  # 复位偏置低16位
                return f"int32低位[15:0] (原始值:{val})"

            # === 0x0020~0x002F: 复位触发/引脚配置/最大速度/PB4目标位置 ===
            elif addr == 0x0020:  # REG_HOME_TRIGGER - 执行复位触发
                return "写1触发(只写)"
            elif addr == 0x0021:  # REG_HOME_AUTO_START - 开机自复位
                return "关闭" if val == 0 else "开启"
            elif addr == 0x0022:  # PB4功能
                names = {0: "0=脉冲", 1: "1=方向", 2: "2=原点开关", 3: "3=限位",
                         4: "4=外部位置", 5: "5=外部速度", 6: "6=执行复位",
                         7: "7=无功能", 8: "8=停止", 9: "9=启动触发"}
                return names.get(val, str(val))
            elif addr == 0x0023:  # PB4极性
                return "高有效" if val == 0 else ("低有效" if val == 1 else str(val))
            elif addr == 0x0024:  # PB4限位方向
                return "停止正方向" if val == 0 else ("停止负方向" if val == 1 else str(val))
            elif addr == 0x0025:  # PB5功能
                names = {0: "0=脉冲", 1: "1=方向", 2: "2=原点开关", 3: "3=限位",
                         4: "4=外部位置", 5: "5=外部速度", 6: "6=执行复位",
                         7: "7=无功能", 8: "8=停止", 9: "9=启动触发"}
                return names.get(val, str(val))
            elif addr == 0x0026:  # PB5极性
                return "高有效" if val == 0 else ("低有效" if val == 1 else str(val))
            elif addr == 0x0027:  # PB5限位方向
                return "停止正方向" if val == 0 else ("停止负方向" if val == 1 else str(val))
            elif addr == 0x0028:  # 最大运行速度高16位
                return f"int32高位[31:16] (原始值:{val})"
            elif addr == 0x0029:  # 最大运行速度低16位
                return f"int32低位[15:0] (原始值:{val}, 0=无限制)"
            elif addr == 0x002A:  # PB4目标位置[63:48]
                return f"int64高位[63:48] (原始值:{val})"
            elif addr == 0x002B:  # PB4目标位置[47:32]
                return f"int64次高位[47:32] (原始值:{val})"
            elif addr == 0x002C:  # PB4目标位置[31:16]
                return f"int64次低位[31:16] (原始值:{val})"
            elif addr == 0x002D:  # PB4目标位置[15:0]
                return f"int64低位[15:0] (原始值:{val})"
            elif addr == 0x002E:  # PB5目标位置[63:48]
                return f"int64高位[63:48] (原始值:{val})"
            elif addr == 0x002F:  # PB5目标位置[47:32]
                return f"int64次高位[47:32] (原始值:{val})"

            # === 0x0030~0x003F: PB5目标位置/精确检测/采集 ===
            elif addr == 0x0030:  # PB5目标位置[31:16]
                return f"int64次低位[31:16] (原始值:{val})"
            elif addr == 0x0031:  # PB5目标位置[15:0]
                return f"int64低位[15:0] (原始值:{val})"
            elif addr == 0x0032:  # 精确检测速度高16位
                return f"int32高位[31:16] (原始值:{val})"
            elif addr == 0x0033:  # 精确检测速度低16位
                return f"int32低位[15:0] (原始值:{val})"
            elif addr == 0x0034:  # REG_HOME_PRECISION_CYCLES - 精确检测次数
                return f"次数: {val}"
            elif addr == 0x0035:  # PB4目标速度高16位
                return f"int32高位[31:16] (原始值:{val})"
            elif addr == 0x0036:  # PB4目标速度低16位
                return f"int32低位[15:0] (原始值:{val})"
            elif addr == 0x0037:  # PB5目标速度高16位
                return f"int32高位[31:16] (原始值:{val})"
            elif addr == 0x0038:  # PB5目标速度低16位
                return f"int32低位[15:0] (原始值:{val})"
            elif addr == 0x0039:  # 保留寄存器
                return "保留寄存器"
            elif addr == 0x003A:  # REG_SPEED_ACQ_START - 转速采集启动/状态
                return {0: "0=空闲", 1: "1=采集中", 2: "2=完成"}.get(val, str(val))
            elif addr == 0x003B:  # REG_SPEED_ACQ_DIV - 转速采集分频值 1=100us
                us = val * 100
                ms = us / 1000.0
                return f"{us}us ({ms:.1f}ms) (原始值:{val})"
            elif addr == 0x003C:  # REG_SPEED_ACQ_COUNT - 已采样数量(只读)
                return f"已采样: {val} (0~512)"
            elif addr == 0x003D:  # REG_SPEED_ACQ_STATUS - 采集状态(只读)
                return {0: "0=空闲", 1: "1=采集中", 2: "2=完成"}.get(val, str(val))
            elif addr == 0x003E:  # REG_SPEED_ACQ_TYPE - 采集类型
                return {0: "0=转速", 1: "1=PWM", 2: "2=位置", 3: "3=电流",
                        4: "4=PC0电压", 5: "5=PC2外部ADC", 6: "6=PC3外部ADC", 7: "7=电流环开-位置P", 8: "8=电流环开-位置I", 9: "9=电流环开-位置D", 10: "10=电流环开-速度P", 11: "11=电流环开-速度I", 12: "12=电流环开-速度D", 13: "13=电流环开-电流P", 14: "14=电流环开-电流I", 15: "15=电流环开-电流D", 16: "16=电流环关-位置P", 17: "17=电流环关-位置I", 18: "18=电流环关-位置D", 19: "19=电流环关-速度P", 20: "20=电流环关-速度I", 21: "21=电流环关-速度D", 22: "22=位置误差", 23: "23=速度误差"}.get(val, str(val))
            elif addr == 0x003F:  # REG_SPEED_ACQ_SIZE - 采集点数
                return f"采集点数: {val} (1~5120)"

            # === 0x009C~0x009F: 多通道采集 ===
            elif addr == 0x009C:  # REG_SPEED_ACQ_CHANNELS - 采集通道数
                return f"{val}通道" + (" (单通道兼容)" if val == 1 else "")
            elif addr in (0x009D, 0x009E, 0x009F):  # 通道2~4采集类型
                ch_num = {0x009D: 2, 0x009E: 3, 0x009F: 4}.get(addr, 0)
                return {0: f"CH{ch_num}:转速", 1: f"CH{ch_num}:PWM", 2: f"CH{ch_num}:位置", 3: f"CH{ch_num}:电流",
                        4: f"CH{ch_num}:PC0电压", 5: f"CH{ch_num}:PC2外部ADC", 6: f"CH{ch_num}:PC3外部ADC",
                        7: f"CH{ch_num}:电流环开-位置P", 8: f"CH{ch_num}:电流环开-位置I", 9: f"CH{ch_num}:电流环开-位置D",
                        10: f"CH{ch_num}:电流环开-速度P", 11: f"CH{ch_num}:电流环开-速度I", 12: f"CH{ch_num}:电流环开-速度D",
                        13: f"CH{ch_num}:电流环开-电流P", 14: f"CH{ch_num}:电流环开-电流I", 15: f"CH{ch_num}:电流环开-电流D",
                        16: f"CH{ch_num}:电流环关-位置P", 17: f"CH{ch_num}:电流环关-位置I", 18: f"CH{ch_num}:电流环关-位置D",
                        19: f"CH{ch_num}:电流环关-速度P", 20: f"CH{ch_num}:电流环关-速度I", 21: f"CH{ch_num}:电流环关-速度D",
                        22: f"CH{ch_num}:位置误差", 23: f"CH{ch_num}:速度误差"}.get(val, str(val))

            # === 0x0040~0x004F: 堵转/电流环/PC2PC3 ===
            elif addr == 0x0040:  # REG_STALL_PROT_EN - 堵转保护使能
                return "关闭" if val == 0 else "开启"
            elif addr == 0x0041:  # REG_STALL_ERR_LIMIT - 堵转误差阈值
                sv = s16(val)
                return f"{sv} 脉冲/脉冲·秒⁻¹ (原始值:{val})"
            elif addr == 0x0042:  # REG_STALL_TIME - 堵转持续时长 ×5ms
                return f"{val * 5}ms (原始值:{val}×5ms)"
            elif addr == 0x0043:  # REG_STALL_STATUS - 堵转状态(只读)
                return "正常" if val == 0 else "已触发"
            elif addr == 0x0044:  # REG_STALL_RESET - 堵转复位
                return "写1清除(只写)"
            elif addr == 0x0045:  # REG_CUR_LOOP_EN - 电流环使能
                return "关闭(2环)" if val == 0 else "开启(3环)"
            elif addr == 0x0046:  # 电流环Kp ×100
                return f"Kp={s16(val) / 100.0:.2f} (原始值:{val})"
            elif addr == 0x0047:  # 电流环Ki ×100
                return f"Ki={s16(val) / 100.0:.2f} (原始值:{val})"
            elif addr == 0x0048:  # 电流环Kd ×1000
                return f"Kd={s16(val) / 1000.0:.3f} (原始值:{val})"
            elif addr == 0x0049:  # REG_CUR_OFFSET - 零电流ADC值
                return f"ADC值: {val} (默认2048)"
            elif addr == 0x004A:  # REG_CUR_SCALE - ADC到电流转换系数 ×10000
                return f"{s16(val) / 10000.0:.4f} (原始值:{val})"
            elif addr == 0x004B:  # REG_OVER_CUR_LIMIT - 过流保护阈值 相对值×10
                sv = s16(val)
                if sv == 0:
                    return "关闭 (0=关闭)"
                return f"{sv / 10.0 / 121.0:.2f}A (原始值:{val})"
            elif addr == 0x004C:  # REG_OVER_CUR_STATUS - 过流状态(只读)
                return "正常" if val == 0 else "已触发"
            elif addr == 0x004D:  # REG_OVER_CUR_RESET - 过流复位
                return "写1清除(只写)"
            elif addr == 0x004E:  # REG_PC2_FUNC - PC2功能
                return {0: "0=无", 1: "1=ADC转速", 2: "2=ADC位置转速",
                        3: "3=ADC开环", 4: "4=ADC位置"}.get(val, str(val))
            elif addr == 0x004F:  # REG_PC3_FUNC - PC3功能
                return {0: "0=无", 1: "1=ADC转速", 2: "2=ADC位置转速",
                        3: "3=ADC开环", 4: "4=ADC位置"}.get(val, str(val))

            # === 0x0050~0x005F: ADC最大/最小值/死区 ===
            elif addr == 0x0050:  # ADC最大速度高16位
                return f"int32高位[31:16] (原始值:{val})"
            elif addr == 0x0051:  # ADC最大速度低16位
                return f"int32低位[15:0] (原始值:{val})"
            elif addr == 0x0052:  # REG_ADC_MAX_PWM - ADC最大PWM 有符号
                return f"PWM: {s16(val)} (-1000~1000) (原始值:{val})"
            elif addr == 0x0053:  # ADC最大位置[63:48]
                return f"int64高位[63:48] (原始值:{val})"
            elif addr == 0x0054:  # ADC最大位置[47:32]
                return f"int64次高位[47:32] (原始值:{val})"
            elif addr == 0x0055:  # ADC最大位置[31:16]
                return f"int64次低位[31:16] (原始值:{val})"
            elif addr == 0x0056:  # ADC最大位置[15:0]
                return f"int64低位[15:0] (原始值:{val})"
            elif addr == 0x0057:  # ADC最小速度高16位 有符号
                return f"int32高位[31:16] (原始值:{val})"
            elif addr == 0x0058:  # ADC最小速度低16位 有符号
                return f"int32低位[15:0] (原始值:{val})"
            elif addr == 0x0059:  # REG_ADC_MIN_PWM - ADC最小PWM 有符号
                return f"PWM: {s16(val)} (-1000~1000) (原始值:{val})"
            elif addr == 0x005A:  # ADC最小位置[63:48] 有符号
                return f"int64高位[63:48] (原始值:{val})"
            elif addr == 0x005B:  # ADC最小位置[47:32] 有符号
                return f"int64次高位[47:32] (原始值:{val})"
            elif addr == 0x005C:  # ADC最小位置[31:16] 有符号
                return f"int64次低位[31:16] (原始值:{val})"
            elif addr == 0x005D:  # ADC最小位置[15:0] 有符号
                return f"int64低位[15:0] (原始值:{val})"
            elif addr == 0x005E:  # REG_ADC_DEAD_ZONE1_POS - ADC死区1位置
                return {0: "0=最小点", 1: "1=中位点", 2: "2=最大点"}.get(val, str(val))
            elif addr == 0x005F:  # REG_ADC_DEAD_ZONE1_WIDTH - ADC死区1宽度
                return f"宽度: {val} (0~4095, 0=关闭)"

            # === 0x0060~0x0068: 死区2/电流限幅/电流模式PID ===
            elif addr == 0x0060:  # REG_ADC_DEAD_ZONE2_POS - ADC死区2位置
                return {0: "0=最小点", 1: "1=中位点", 2: "2=最大点"}.get(val, str(val))
            elif addr == 0x0061:  # REG_ADC_DEAD_ZONE2_WIDTH - ADC死区2宽度
                return f"宽度: {val} (0~4095, 0=关闭)"
            elif addr == 0x0062:  # REG_MAX_CURRENT - 电流限幅 相对值×10
                sv = s16(val)
                if sv == 0:
                    return "不限制 (0=不限)"
                return f"{sv / 10.0 / 121.0:.2f}A (原始值:{val})"
            elif addr == 0x0063:  # 电流模式速度Kp ×100
                return f"Kp={s16(val) / 100.0:.2f} (原始值:{val})"
            elif addr == 0x0064:  # 电流模式速度Ki ×100
                return f"Ki={s16(val) / 100.0:.2f} (原始值:{val})"
            elif addr == 0x0065:  # 电流模式速度Kd ×1000
                return f"Kd={s16(val) / 1000.0:.3f} (原始值:{val})"
            elif addr == 0x0066:  # 电流模式位置Kp ×100
                return f"Kp={s16(val) / 100.0:.2f} (原始值:{val})"
            elif addr == 0x0067:  # 电流模式位置Ki ×100
                return f"Ki={s16(val) / 100.0:.2f} (原始值:{val})"
            elif addr == 0x0068:  # 电流模式位置Kd ×1000
                return f"Kd={s16(val) / 1000.0:.3f} (原始值:{val})"
            elif addr == 0x0069:  # REG_CUR_DEAD_ZONE - 电流死区 相对值×10
                # 相对值 → 安培: 1A ≈ 121相对单位
                return f"死区={s16(val) / 10.0 / 121.0:.3f}A (原始值:{val})"
            elif addr == 0x006A:  # REG_ADC_FILTER_SIZE - 外部ADC滤波窗口
                # 显示窗口大小和对应的近似延迟(主循环5ms周期)
                delay_ms = val * 5 if val > 1 else 0
                return f"窗口={val} ({delay_ms}ms延迟)" if val > 1 else f"窗口={val} (关闭)"
            elif addr == 0x0091:  # REG_STEP_ACCEL_EN - 步进加减速使能
                return f"{'开启' if val else '关闭'} (原始值:{val})"

            return str(val)
        except Exception:
            return str(val)

    def _export_config(self):
        """导出Treeview寄存器值到CSV"""
        if not (self.reg_list_tree and self.reg_list_win and self.reg_list_win.winfo_exists()):
            return
        filename = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV文件", "*.csv"), ("所有文件", "*.*")],
            initialfile="motor_config.csv",
            parent=self.reg_list_win)
        if not filename:
            return
        try:
            with open(filename, 'w', newline='', encoding='utf-8-sig') as f:
                writer = csv.writer(f)
                writer.writerow(["地址", "寄存器名", "描述", "原始值", "解析值"])
                for item in self.reg_list_tree.get_children():
                    values = self.reg_list_tree.item(item)['values']
                    writer.writerow(list(values))
            messagebox.showinfo("成功", f"配置已导出到\n{filename}", parent=self.reg_list_win)
            self.log(f"配置导出: {filename}")
        except Exception as e:
            messagebox.showerror("错误", f"导出失败: {e}", parent=self.reg_list_win)

    def _import_config(self):
        """从CSV导入配置，更新Treeview"""
        if not (self.reg_list_tree and self.reg_list_win and self.reg_list_win.winfo_exists()):
            return
        filename = filedialog.askopenfilename(
            filetypes=[("CSV文件", "*.csv"), ("所有文件", "*.*")],
            parent=self.reg_list_win)
        if not filename:
            return
        try:
            with open(filename, 'r', encoding='utf-8-sig') as f:
                reader = csv.DictReader(f)
                imported = 0
                for row in reader:
                    addr_str = str(row.get('地址', '')).strip()
                    if not addr_str:
                        continue
                    try:
                        # 支持"0x1234"或十进制
                        if addr_str.lower().startswith('0x'):
                            addr = int(addr_str, 16)
                        else:
                            addr = int(addr_str)
                    except ValueError:
                        continue
                    val_str = str(row.get('原始值', '')).strip()
                    try:
                        val = int(val_str)
                    except ValueError:
                        continue
                    if self.reg_list_tree.exists(str(addr)):
                        parsed = self._parse_register(addr, val)
                        name = row.get('寄存器名', '')
                        desc = row.get('描述', '')
                        self.reg_list_tree.item(str(addr), values=(
                            f"0x{addr:04X}", name, desc, str(val), parsed))
                        imported += 1
            messagebox.showinfo("成功",
                f"已导入{imported}个寄存器\n点击'写入全部'将配置写入电机",
                parent=self.reg_list_win)
            self.log(f"配置导入: {filename} ({imported}个寄存器)")
        except Exception as e:
            messagebox.showerror("错误", f"导入失败: {e}", parent=self.reg_list_win)

    def _write_all_registers(self):
        """将Treeview中所有值写入电机（跳过触发型/只读寄存器）"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return
        if not (self.reg_list_tree and self.reg_list_win and self.reg_list_win.winfo_exists()):
            return
        if not messagebox.askyesno("确认",
            "将把列表中的所有配置写入电机?\n(只写寄存器和触发寄存器会跳过)",
            parent=self.reg_list_win):
            return
        SKIP_REGS = self.REGISTER_SKIP_ON_WRITE
        tree = self.reg_list_tree

        def do_write():
            try:
                if not self.serial_lock.acquire(timeout=2.0):
                    self.reg_list_win.after(0, lambda: messagebox.showwarning("警告", "串口忙"))
                    return
                try:
                    written = 0
                    errors = 0
                    for item in tree.get_children():
                        addr = int(item)
                        if addr in SKIP_REGS:
                            continue
                        values = tree.item(item)['values']
                        try:
                            val = int(values[3])
                        except (ValueError, IndexError):
                            continue
                        try:
                            self.modbus.write_single_register(self.get_slave_addr(), addr, val)
                            written += 1
                            time.sleep(0.01)  # 10ms间隔
                        except Exception:
                            errors += 1
                    msg = f"写入完成: 成功{written}个"
                    if errors:
                        msg += f", 失败{errors}个"
                    self.reg_list_win.after(0, lambda: messagebox.showinfo("完成", msg))
                    self.log(msg)
                finally:
                    self.serial_lock.release()
            except Exception as e:
                self.reg_list_win.after(0, lambda: messagebox.showerror("错误", str(e)))

        threading.Thread(target=do_write, daemon=True).start()

    def _load_to_main_window(self):
        """将寄存器列表中的值转换后填入主界面输入框"""
        if not (self.reg_list_tree and self.reg_list_win and self.reg_list_win.winfo_exists()):
            return

        tree = self.reg_list_tree

        # 辅助：从Treeview获取指定地址的原始值（int），失败返回None
        def get_raw(addr):
            iid = str(addr)
            if not tree.exists(iid):
                return None
            try:
                values = tree.item(iid)['values']
                return int(values[3])
            except (ValueError, IndexError):
                return None

        self._apply_registers_to_main(get_raw, parent=self.reg_list_win)

    def _apply_registers_to_main(self, get_raw, parent=None):
        """根据get_raw(addr)回调获取原始寄存器值，转换后填入主界面输入框。
        get_raw: 回调，接收地址(int)，返回原始值(int)或None
        parent: 完成提示框的父窗口，默认为主窗口
        """
        if parent is None:
            parent = self.root

        # 16位有符号转换
        def s16(v):
            return v - 0x10000 if v >= 0x8000 else v

        # 32位组合（可选符号扩展）
        def combine32(h, l, signed=False):
            v = (h << 16) | l
            if signed and v >= 0x80000000:
                v -= 0x100000000
            return v

        # 64位组合（符号扩展，与regs_to_int64一致）
        def combine64(v0, v1, v2, v3):
            v = (v0 << 48) | (v1 << 32) | (v2 << 16) | v3
            if v >= 0x8000000000000000:
                v -= 0x10000000000000000
            return v

        # 设置StringVar辅助
        def set_var(name, value):
            var = getattr(self, name, None)
            if isinstance(var, tk.StringVar):
                var.set(str(value))
                return True
            return False

        loaded = 0
        errors = 0

        try:
            # === 单寄存器映射 (地址 -> (StringVar属性名, 转换函数)) ===
            # 转换函数接收原始值(int)，返回主界面要显示的字符串
            PIN_FUNC_MAP = {0: "脉冲", 1: "方向", 2: "原点位置开关", 3: "限位开关",
                            4: "外部目标位置", 5: "外部目标速度", 6: "执行复位操作",
                            7: "无功能", 8: "停止", 9: "启动标志位触发"}
            PC_FUNC_MAP = {0: "无功能", 1: "ADC转速模式", 2: "ADC位置转速模式",
                           3: "ADC开环模式", 4: "ADC位置模式"}
            DZ_POS_MAP = {0: "最小点", 1: "中位点", 2: "最大点"}

            REG_TO_VAR = [
                (0x0001, 'start_mode_var', lambda v: "标志位" if v == 1 else "直接启动"),
                (0x0008, 'pos_kp_var', lambda v: f"{s16(v) / 100.0:.2f}"),
                (0x0009, 'pos_ki_var', lambda v: f"{s16(v) / 100.0:.2f}"),
                (0x000A, 'pos_kd_var', lambda v: f"{s16(v) / 1000.0:.3f}"),
                (0x000B, 'spd_kp_var', lambda v: f"{s16(v) / 100.0:.2f}"),
                (0x000C, 'spd_ki_var', lambda v: f"{s16(v) / 100.0:.2f}"),
                (0x000D, 'spd_kd_var', lambda v: f"{s16(v) / 1000.0:.3f}"),
                (0x000E, 'dead_zone_var', lambda v: str(v)),
                (0x000F, 'max_output_var', lambda v: str(v)),
                (0x0011, 'motor_dir_var', lambda v: "反转" if v == 1 else "正转"),
                (0x0012, 'encoder_dir_var', lambda v: "反转" if v == 1 else "正常"),
                (0x0014, 'device_addr_var', lambda v: str(v)),
                (0x0015, 'device_baud_var', lambda v: str(v * 100)),
                (0x0017, 'home_mode_var', lambda v: {0: "关闭", 1: "堵转", 2: "精确复位"}.get(v, str(v))),
                (0x0018, 'home_dir_var', lambda v: "正方向" if v == 1 else "负方向"),
                (0x0019, 'home_current_var', lambda v: str(v)),
                (0x0021, 'home_auto_start_var', lambda v: "开启" if v == 1 else "关闭"),
                (0x0022, 'pin4_func_var', lambda v: PIN_FUNC_MAP.get(v, str(v))),
                (0x0023, 'pin4_pol_var', lambda v: "低电平" if v == 1 else "高电平"),
                (0x0024, 'pin4_ldir_var', lambda v: "停止负方向" if v == 1 else "停止正方向"),
                (0x0025, 'pin5_func_var', lambda v: PIN_FUNC_MAP.get(v, str(v))),
                (0x0026, 'pin5_pol_var', lambda v: "低电平" if v == 1 else "高电平"),
                (0x0027, 'pin5_ldir_var', lambda v: "停止负方向" if v == 1 else "停止正方向"),
                (0x0034, 'home_precision_cycles_var', lambda v: str(v)),
                (0x0040, 'stall_en_var', lambda v: "开启" if v == 1 else "关闭"),
                (0x0041, 'stall_err_var', lambda v: str(s16(v))),
                (0x0042, 'stall_time_var', lambda v: str(v)),
                (0x0045, 'cur_loop_en_var', lambda v: "开启" if v == 1 else "关闭"),
                (0x0046, 'cur_kp_var', lambda v: f"{s16(v) / 100.0:.2f}"),
                (0x0047, 'cur_ki_var', lambda v: f"{s16(v) / 100.0:.2f}"),
                (0x0048, 'cur_kd_var', lambda v: f"{s16(v) / 1000.0:.3f}"),
                (0x0049, 'cur_offset_var', lambda v: str(v)),
                (0x004A, 'cur_scale_var', lambda v: f"{s16(v) / 10000.0:.4f}"),
                (0x004B, 'over_cur_limit_var', lambda v: f"{s16(v) / 10.0 / 121.0:.2f}"),
                (0x004E, 'pc2_func_var', lambda v: PC_FUNC_MAP.get(v, str(v))),
                (0x004F, 'pc3_func_var', lambda v: PC_FUNC_MAP.get(v, str(v))),
                (0x0052, 'adc_max_pwm_var', lambda v: str(s16(v))),
                (0x0059, 'adc_min_pwm_var', lambda v: str(s16(v))),
                (0x005E, 'adc_dz1_pos_var', lambda v: DZ_POS_MAP.get(v, str(v))),
                (0x005F, 'adc_dz1_width_var', lambda v: str(v)),
                (0x0060, 'adc_dz2_pos_var', lambda v: DZ_POS_MAP.get(v, str(v))),
                (0x0061, 'adc_dz2_width_var', lambda v: str(v)),
                (0x0062, 'max_current_var', lambda v: f"{s16(v) / 10.0 / 121.0:.2f}"),
                (0x0063, 'spd_kp_cur_var', lambda v: f"{s16(v) / 100.0:.2f}"),
                (0x0064, 'spd_ki_cur_var', lambda v: f"{s16(v) / 100.0:.2f}"),
                (0x0065, 'spd_kd_cur_var', lambda v: f"{s16(v) / 1000.0:.3f}"),
                (0x0066, 'pos_kp_cur_var', lambda v: f"{s16(v) / 100.0:.2f}"),
                (0x0067, 'pos_ki_cur_var', lambda v: f"{s16(v) / 100.0:.2f}"),
                (0x0068, 'pos_kd_cur_var', lambda v: f"{s16(v) / 1000.0:.3f}"),
                # 电流死区(0x0069): 相对值×10 → 安培, 1A ≈ 121相对单位
                (0x0069, 'cur_dead_zone_var', lambda v: f"{s16(v) / 10.0 / 121.0:.3f}"),
                # 外部ADC滤波窗口(0x006A): 直接是窗口大小0~128
                (0x006A, 'adc_filter_size_var', lambda v: str(v)),
            ]

            for addr, var_name, conv in REG_TO_VAR:
                raw_val = get_raw(addr)
                if raw_val is None:
                    continue
                try:
                    converted = conv(raw_val)
                    if set_var(var_name, converted):
                        loaded += 1
                except Exception:
                    errors += 1

            # 步进加减速使能(0x0091, 新增): BooleanVar, 需单独处理
            step_en_raw = get_raw(0x0091)
            if step_en_raw is not None:
                self.step_accel_var.set(step_en_raw != 0)
                loaded += 1

            # 缓存加速表参数u(0x0094)和a(0x0095~0x0096), 供索引↔速度转换
            u_raw = get_raw(0x0094)
            a_l_raw = get_raw(0x0095)
            a_h_raw = get_raw(0x0096)
            if u_raw is not None:
                self.acc_u = u_raw
            if a_l_raw is not None and a_h_raw is not None:
                self.acc_a = (a_h_raw << 16) | a_l_raw

            # 步进最大速度索引(0x006B~0x006E, 新增): 读取索引并转换为实际速度显示
            step_ms_raw = [get_raw(a) for a in [0x006B, 0x006C, 0x006D, 0x006E]]
            if all(v is not None for v in step_ms_raw):
                idx_val = (step_ms_raw[0] << 48) | (step_ms_raw[1] << 32) | (step_ms_raw[2] << 16) | step_ms_raw[3]
                if idx_val >= 0x8000000000000000:
                    idx_val -= 0x10000000000000000
                # 索引转速度: speed = sqrt(u² + 2*a*idx)
                if self.acc_a > 0 and idx_val >= 0:
                    import math
                    speed_val = int(math.sqrt(self.acc_u * self.acc_u + 2 * self.acc_a * idx_val))
                    self.step_cur_index_label.config(text=f"当前: {speed_val}步/秒 (索引{idx_val})")
                else:
                    self.step_cur_index_label.config(text=f"当前: 索引{idx_val}")
                loaded += 1

            # === 32位寄存器对映射 (高位地址, 低位地址, StringVar属性名, 是否有符号) ===
            REG32_PAIRS = [
                (0x0006, 0x0007, 'target_speed_var', False),        # 目标速度
                (0x001A, 0x001B, 'home_speed_var', False),          # 复位速度
                (0x001C, 0x001D, 'home_max_distance_var', False),  # 复位最大距离
                (0x001E, 0x001F, 'home_back_distance_var', True),  # 复位偏置(有符号)
                (0x0028, 0x0029, 'max_run_speed_var', False),      # 最大运行速度
                (0x0032, 0x0033, 'home_precision_speed_var', False),  # 精确检测速度
                (0x0035, 0x0036, 'pin4_target_speed_var', True),   # PB4目标速度(有符号)
                (0x0037, 0x0038, 'pin5_target_speed_var', True),   # PB5目标速度(有符号)
                (0x0050, 0x0051, 'adc_max_speed_var', True),       # ADC最大速度(有符号)
                (0x0057, 0x0058, 'adc_min_speed_var', True),      # ADC最小速度(有符号)
            ]

            for h_addr, l_addr, var_name, signed in REG32_PAIRS:
                h = get_raw(h_addr)
                l = get_raw(l_addr)
                if h is None or l is None:
                    continue
                try:
                    val = combine32(h, l, signed=signed)
                    if set_var(var_name, val):
                        loaded += 1
                except Exception:
                    errors += 1

            # === 64位寄存器组映射 (4个地址列表, StringVar属性名) ===
            # 组合方式与regs_to_int64一致(有符号)
            REG64_GROUPS = [
                ([0x0002, 0x0003, 0x0004, 0x0005], 'target_pos_var'),          # 目标位置
                ([0x002A, 0x002B, 0x002C, 0x002D], 'pin4_target_pos_var'),     # PB4目标位置
                ([0x002E, 0x002F, 0x0030, 0x0031], 'pin5_target_pos_var'),     # PB5目标位置
                ([0x0053, 0x0054, 0x0055, 0x0056], 'adc_max_pos_var'),         # ADC最大位置
                ([0x005A, 0x005B, 0x005C, 0x005D], 'adc_min_pos_var'),         # ADC最小位置
                # 步进最大速度(0x006B~0x006E)不在此处映射: 存储的是索引值,
                # 已在上方单独处理: 索引转实际速度后显示在标签, 不填入输入框
            ]

            for addrs, var_name in REG64_GROUPS:
                vals = [get_raw(a) for a in addrs]
                if any(v is None for v in vals):
                    continue
                try:
                    val = combine64(vals[0], vals[1], vals[2], vals[3])
                    if set_var(var_name, val):
                        loaded += 1
                except Exception:
                    errors += 1
        except Exception:
            errors += 1

        msg = f"已载入{loaded}项参数到运行窗口"
        if errors:
            msg += f" ({errors}项失败)"
        messagebox.showinfo("完成", msg, parent=parent)
        self.log(msg)

    def read_all_registers_to_main(self):
        """读取所有寄存器并填入主界面设置框（顶部按钮）"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return

        def do_read():
            try:
                if not self.serial_lock.acquire(timeout=2.0):
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙，请稍后再试"))
                    return
                try:
                    reg_dict = {}
                    # 分段读取（与寄存器列表"读取全部"一致，避开MODBUS单次125上限）
                    for start, count in [(0x0000, 0x0045), (0x0045, 0x0026), (0x006B, 0x0026), (0x0091, 0x0001), (0x0092, 0x0009)]:
                        try:
                            data = self.modbus.read_holding_registers(
                                self.get_slave_addr(), start, count)
                            for i, val in enumerate(data):
                                reg_dict[start + i] = val
                        except Exception as e:
                            print(f"读取0x{start:04X}~0x{start+count-1:04X}失败: {e}")
                            self.root.after(0, lambda s=start, c=count, err=str(e):
                                self.log(f"读取0x{s:04X}~0x{s+c-1:04X}失败: {err}"))

                    if not reg_dict:
                        self.root.after(0, lambda: messagebox.showerror("错误", "未能读取到任何寄存器，请检查连接"))
                        return

                    def get_raw(addr):
                        return reg_dict.get(addr)

                    self.root.after(0, lambda: self._apply_registers_to_main(get_raw, parent=self.root))
                finally:
                    self.serial_lock.release()
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"读取失败: {e}"))

        threading.Thread(target=do_read, daemon=True).start()

if __name__ == "__main__":
    root = tk.Tk()
    app = ModbusDebuggerApp(root)
    root.mainloop()
