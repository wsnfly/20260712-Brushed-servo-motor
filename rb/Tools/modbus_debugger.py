#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MODBUS RTU 调试工具
用于电机控制器的参数配置和状态监控
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import serial.tools.list_ports
import struct
import threading
import time

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
    REG_SPEED_ACQ_TYPE = 0x003E         # 采集类型: 0=转速, 1=PWM, 2=位置(相对起始偏移, ±32767脉冲), 3=电流(相对值, ±1000=±8.25A)
    REG_SPEED_ACQ_SIZE = 0x003F         # 采集点数(1~5120)

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
        
        self.create_widgets()
        
    def create_widgets(self):
        """创建界面组件"""
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
        left_canvas.configure(yscrollcommand=left_scrollbar.set)

        left_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        left_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        left_frame = ttk.Frame(left_canvas)
        left_canvas_window = left_canvas.create_window((0, 0), window=left_frame, anchor=tk.NW)

        def _on_left_configure(event):
            left_canvas.configure(scrollregion=left_canvas.bbox("all"))

        def _on_canvas_configure(event):
            left_canvas.itemconfig(left_canvas_window, width=event.width)

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

        adc_btn_frame = ttk.Frame(adc_frame)
        adc_btn_frame.grid(row=13, column=0, columnspan=3, pady=5)
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

        cur_btn_frame = ttk.Frame(cur_frame)
        cur_btn_frame.grid(row=4, column=0, columnspan=6, pady=5)
        ttk.Button(cur_btn_frame, text="读取参数", command=self.read_current_loop_config).pack(side=tk.LEFT, padx=5)
        ttk.Button(cur_btn_frame, text="写入参数", command=self.write_current_loop_config).pack(side=tk.LEFT, padx=5)
        ttk.Button(cur_btn_frame, text="过流复位", command=self.reset_over_current).pack(side=tk.LEFT, padx=5)
        ttk.Button(cur_btn_frame, text="标定零点", command=self.calibrate_current_offset).pack(side=tk.LEFT, padx=5)

        # 右侧：Canvas+滚动条 包裹 PanedWindow（可整体滚动 + 可调整各窗口大小）
        right_frame = ttk.Frame(paned)
        paned.add(right_frame, weight=2)

        right_canvas = tk.Canvas(right_frame, highlightthickness=0)
        right_scrollbar = ttk.Scrollbar(right_frame, orient=tk.VERTICAL, command=right_canvas.yview)
        right_canvas.configure(yscrollcommand=right_scrollbar.set)
        right_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        right_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        right_inner = ttk.Frame(right_canvas)
        right_canvas_window = right_canvas.create_window((0, 0), window=right_inner, anchor=tk.NW)

        def _on_right_configure(event):
            right_canvas.configure(scrollregion=right_canvas.bbox("all"))

        def _on_right_canvas_configure(event):
            # 宽度跟随Canvas，高度根据内容自适应（最小不低于Canvas高度）
            content_h = right_inner.winfo_reqheight()
            min_h = event.height
            right_canvas.itemconfig(right_canvas_window, width=event.width, height=max(content_h, min_h))

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

        right_vpaned = ttk.PanedWindow(right_inner, orient=tk.VERTICAL)
        right_vpaned.pack(fill=tk.BOTH, expand=True)

        # === 实时状态（可调高度） ===
        status_frame = ttk.LabelFrame(right_vpaned, text="实时状态", padding=10)
        right_vpaned.add(status_frame, weight=1)

        self.status_labels = {}

        # 3列竖行布局：每列包含 label(column=N) + value(column=N+1)
        # 第1列：位置/速度/模式相关
        ttk.Label(status_frame, text="当前位置:").grid(row=0, column=0, padx=5, pady=2, sticky=tk.W)
        self.status_labels['position'] = ttk.Label(status_frame, text="0", foreground="blue", font=("Arial", 12, "bold"))
        self.status_labels['position'].grid(row=0, column=1, padx=5, pady=2, sticky=tk.W)

        ttk.Label(status_frame, text="目标位置:").grid(row=1, column=0, padx=5, pady=2, sticky=tk.W)
        self.status_labels['target_position'] = ttk.Label(status_frame, text="0", foreground="green", font=("Arial", 12, "bold"))
        self.status_labels['target_position'].grid(row=1, column=1, padx=5, pady=2, sticky=tk.W)

        ttk.Label(status_frame, text="当前速度:").grid(row=2, column=0, padx=5, pady=2, sticky=tk.W)
        self.status_labels['speed'] = ttk.Label(status_frame, text="0", foreground="blue", font=("Arial", 12, "bold"))
        self.status_labels['speed'].grid(row=2, column=1, padx=5, pady=2, sticky=tk.W)

        ttk.Label(status_frame, text="当前模式:").grid(row=3, column=0, padx=5, pady=2, sticky=tk.W)
        self.status_labels['mode'] = ttk.Label(status_frame, text="未知", foreground="blue", font=("Arial", 12, "bold"))
        self.status_labels['mode'].grid(row=3, column=1, padx=5, pady=2, sticky=tk.W)

        ttk.Label(status_frame, text="启动标志:").grid(row=4, column=0, padx=5, pady=2, sticky=tk.W)
        self.status_labels['start_flag'] = ttk.Label(status_frame, text="未设置", foreground="red", font=("Arial", 12, "bold"))
        self.status_labels['start_flag'].grid(row=4, column=1, padx=5, pady=2, sticky=tk.W)

        # 第2列：方向/PWM/状态相关
        ttk.Label(status_frame, text="电机方向:").grid(row=0, column=2, padx=15, pady=2, sticky=tk.W)
        self.status_labels['motor_dir'] = ttk.Label(status_frame, text="正转", foreground="blue", font=("Arial", 12, "bold"))
        self.status_labels['motor_dir'].grid(row=0, column=3, padx=5, pady=2, sticky=tk.W)

        ttk.Label(status_frame, text="编码器方向:").grid(row=1, column=2, padx=15, pady=2, sticky=tk.W)
        self.status_labels['encoder_dir'] = ttk.Label(status_frame, text="正常", foreground="blue", font=("Arial", 12, "bold"))
        self.status_labels['encoder_dir'].grid(row=1, column=3, padx=5, pady=2, sticky=tk.W)

        ttk.Label(status_frame, text="PWM输出:").grid(row=2, column=2, padx=15, pady=2, sticky=tk.W)
        self.status_labels['pwm'] = ttk.Label(status_frame, text="0.0%", foreground="purple", font=("Arial", 12, "bold"))
        self.status_labels['pwm'].grid(row=2, column=3, padx=5, pady=2, sticky=tk.W)

        ttk.Label(status_frame, text="复位状态:").grid(row=3, column=2, padx=15, pady=2, sticky=tk.W)
        self.status_labels['homing'] = ttk.Label(status_frame, text="空闲", foreground="gray", font=("Arial", 12, "bold"))
        self.status_labels['homing'].grid(row=3, column=3, padx=5, pady=2, sticky=tk.W)

        ttk.Label(status_frame, text="堵转保护:").grid(row=4, column=2, padx=15, pady=2, sticky=tk.W)
        self.status_labels['stall'] = ttk.Label(status_frame, text="正常", foreground="gray", font=("Arial", 12, "bold"))
        self.status_labels['stall'].grid(row=4, column=3, padx=5, pady=2, sticky=tk.W)

        # 第4列: 电流环实时值
        ttk.Label(status_frame, text="实际电流:").grid(row=0, column=6, padx=15, pady=2, sticky=tk.W)
        self.status_labels['current_actual'] = ttk.Label(status_frame, text="0.00 A", foreground="teal", font=("Arial", 12, "bold"))
        self.status_labels['current_actual'].grid(row=0, column=7, padx=5, pady=2, sticky=tk.W)

        ttk.Label(status_frame, text="电流目标:").grid(row=1, column=6, padx=15, pady=2, sticky=tk.W)
        self.status_labels['current_target'] = ttk.Label(status_frame, text="0.00 A", foreground="teal", font=("Arial", 12, "bold"))
        self.status_labels['current_target'].grid(row=1, column=7, padx=5, pady=2, sticky=tk.W)

        ttk.Label(status_frame, text="过流保护:").grid(row=2, column=6, padx=15, pady=2, sticky=tk.W)
        self.status_labels['over_current'] = ttk.Label(status_frame, text="正常", foreground="gray", font=("Arial", 12, "bold"))
        self.status_labels['over_current'].grid(row=2, column=7, padx=5, pady=2, sticky=tk.W)

        ttk.Label(status_frame, text="电流环:").grid(row=3, column=6, padx=15, pady=2, sticky=tk.W)
        self.status_labels['cur_loop'] = ttk.Label(status_frame, text="关闭", foreground="gray", font=("Arial", 12, "bold"))
        self.status_labels['cur_loop'].grid(row=3, column=7, padx=5, pady=2, sticky=tk.W)

        # 第3列：PID实时值
        ttk.Label(status_frame, text="PID误差:").grid(row=0, column=4, padx=15, pady=2, sticky=tk.W)
        self.status_labels['pid_error'] = ttk.Label(status_frame, text="0.00", foreground="teal", font=("Arial", 12, "bold"))
        self.status_labels['pid_error'].grid(row=0, column=5, padx=5, pady=2, sticky=tk.W)

        ttk.Label(status_frame, text="PID-P:").grid(row=1, column=4, padx=15, pady=2, sticky=tk.W)
        self.status_labels['pid_p'] = ttk.Label(status_frame, text="0.00", foreground="teal", font=("Arial", 12, "bold"))
        self.status_labels['pid_p'].grid(row=1, column=5, padx=5, pady=2, sticky=tk.W)

        ttk.Label(status_frame, text="PID-I:").grid(row=2, column=4, padx=15, pady=2, sticky=tk.W)
        self.status_labels['pid_i'] = ttk.Label(status_frame, text="0.00", foreground="teal", font=("Arial", 12, "bold"))
        self.status_labels['pid_i'].grid(row=2, column=5, padx=5, pady=2, sticky=tk.W)

        ttk.Label(status_frame, text="PID-D:").grid(row=3, column=4, padx=15, pady=2, sticky=tk.W)
        self.status_labels['pid_d'] = ttk.Label(status_frame, text="0.00", foreground="teal", font=("Arial", 12, "bold"))
        self.status_labels['pid_d'].grid(row=3, column=5, padx=5, pady=2, sticky=tk.W)

        # 第5列：ADC采样值（供电电压/外部ADC1/外部ADC2）
        ttk.Label(status_frame, text="供电电压:").grid(row=0, column=8, padx=15, pady=2, sticky=tk.W)
        self.status_labels['supply_voltage'] = ttk.Label(status_frame, text="0.00V", foreground="teal", font=("Arial", 12, "bold"))
        self.status_labels['supply_voltage'].grid(row=0, column=9, padx=5, pady=2, sticky=tk.W)

        ttk.Label(status_frame, text="PC2(ADC1):").grid(row=1, column=8, padx=15, pady=2, sticky=tk.W)
        self.status_labels['ext_adc1'] = ttk.Label(status_frame, text="0", foreground="teal", font=("Arial", 12, "bold"))
        self.status_labels['ext_adc1'].grid(row=1, column=9, padx=5, pady=2, sticky=tk.W)

        ttk.Label(status_frame, text="PC3(ADC2):").grid(row=2, column=8, padx=15, pady=2, sticky=tk.W)
        self.status_labels['ext_adc2'] = ttk.Label(status_frame, text="0", foreground="teal", font=("Arial", 12, "bold"))
        self.status_labels['ext_adc2'].grid(row=2, column=9, padx=5, pady=2, sticky=tk.W)

        # PB4/PB5 引脚电平显示(放在ADC下方)
        ttk.Label(status_frame, text="PB4电平:").grid(row=3, column=8, padx=15, pady=2, sticky=tk.W)
        self.status_labels['pb4_level'] = ttk.Label(status_frame, text="低", foreground="gray", font=("Arial", 12, "bold"))
        self.status_labels['pb4_level'].grid(row=3, column=9, padx=5, pady=2, sticky=tk.W)

        ttk.Label(status_frame, text="PB5电平:").grid(row=4, column=8, padx=15, pady=2, sticky=tk.W)
        self.status_labels['pb5_level'] = ttk.Label(status_frame, text="低", foreground="gray", font=("Arial", 12, "bold"))
        self.status_labels['pb5_level'].grid(row=4, column=9, padx=5, pady=2, sticky=tk.W)

        self.monitor_btn = ttk.Button(status_frame, text="开始监控", command=self.toggle_monitor)
        self.monitor_btn.grid(row=5, column=0, columnspan=10, pady=5)

        # === 采集曲线图（可调高度） ===
        chart_frame = ttk.LabelFrame(right_vpaned, text="采集曲线", padding=5)
        right_vpaned.add(chart_frame, weight=3)

        # 采集控制工具栏
        ctrl_bar = ttk.Frame(chart_frame)
        ctrl_bar.pack(fill=tk.X, pady=(0, 5))

        ttk.Label(ctrl_bar, text="采集类型:").pack(side=tk.LEFT, padx=(0, 2))
        self.acq_type_var = tk.StringVar(value="转速")
        acq_type_combo = ttk.Combobox(ctrl_bar, textvariable=self.acq_type_var, width=12,
                                      values=["转速", "PWM", "位置", "电流", "PC0电压ADC", "PC2外部ADC", "PC3外部ADC"], state="readonly")
        acq_type_combo.pack(side=tk.LEFT, padx=2)
        acq_type_combo.bind("<<ComboboxSelected>>", self.on_acq_type_changed)

        ttk.Label(ctrl_bar, text="分频:").pack(side=tk.LEFT, padx=(10, 2))
        self.acq_div_var = tk.StringVar(value="50")
        ttk.Entry(ctrl_bar, textvariable=self.acq_div_var, width=6).pack(side=tk.LEFT, padx=2)
        ttk.Label(ctrl_bar, text="(1=100us)").pack(side=tk.LEFT, padx=(0, 5))

        ttk.Label(ctrl_bar, text="采集点数:").pack(side=tk.LEFT, padx=(10, 2))
        self.acq_size_var = tk.StringVar(value="5120")
        ttk.Entry(ctrl_bar, textvariable=self.acq_size_var, width=7).pack(side=tk.LEFT, padx=2)
        ttk.Label(ctrl_bar, text="(1~5120)").pack(side=tk.LEFT, padx=(0, 5))

        ttk.Button(ctrl_bar, text="设置参数", command=self.set_speed_acq_params).pack(side=tk.LEFT, padx=2)
        ttk.Button(ctrl_bar, text="启动采集", command=self.start_speed_acq).pack(side=tk.LEFT, padx=2)
        ttk.Button(ctrl_bar, text="读取并绘图", command=self.read_and_plot_speed).pack(side=tk.LEFT, padx=2)
        ttk.Button(ctrl_bar, text="清空曲线", command=self.clear_speed_plot).pack(side=tk.LEFT, padx=2)

        self.acq_status_label = ttk.Label(ctrl_bar, text="状态: 空闲", foreground="gray")
        self.acq_status_label.pack(side=tk.LEFT, padx=10)

        # matplotlib 图表 + 工具栏（支持缩放/平移）
        self.speed_fig = Figure(figsize=(6, 3), dpi=100)
        self.speed_ax = self.speed_fig.add_subplot(111)
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

        # === 底部：日志(左) + 原始数据包(右) - 水平PanedWindow可拖动分隔 ===
        bottom_hpaned = ttk.PanedWindow(right_vpaned, orient=tk.HORIZONTAL)
        right_vpaned.add(bottom_hpaned, weight=2)

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
            right_height = right_vpaned.winfo_height()
            bottom_width = bottom_hpaned.winfo_width()
            if main_width > 100 and right_height > 200 and bottom_width > 100:
                paned.sashpos(0, main_width // 3)  # 左侧占1/3
                bottom_hpaned.sashpos(0, bottom_width // 2)
                # 垂直分隔：状态区占约25%，曲线区占约50%，日志区占约25%
                right_vpaned.sashpos(0, int(right_height * 0.25))
                right_vpaned.sashpos(1, int(right_height * 0.75))
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
                        self.root.after(0, lambda: self.log(f"读取ADC配置成功: PC2={fm.get(pc2_func,'?')}, PC3={fm.get(pc3_func,'?')}, 速度=[{min_speed}~{max_speed}], PWM=[{min_pwm}~{max_pwm}], 位置=[{min_pos}~{max_pos}], 死区1={dz_fm.get(dz1_pos,'?')}宽{dz1_width}, 死区2={dz_fm.get(dz2_pos,'?')}宽{dz2_width}"))
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
                        self.root.after(0, lambda: self.log(f"写入ADC配置成功: PC2={self.pc2_func_var.get()}, PC3={self.pc3_func_var.get()}, 速度=[{min_speed}~{max_speed}], PWM=[{min_pwm}~{max_pwm}], 位置=[{min_pos}~{max_pos}], 死区1={self.adc_dz1_pos_var.get()}宽{dz1_width}, 死区2={self.adc_dz2_pos_var.get()}宽{dz2_width}"))
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

                        def update_ui():
                            self.cur_loop_en_var.set("开启" if en else "关闭")
                            self.cur_kp_var.set(f"{kp:.2f}")
                            self.cur_ki_var.set(f"{ki:.2f}")
                            self.cur_kd_var.set(f"{kd:.3f}")
                            self.cur_offset_var.set(str(offset))
                            self.cur_scale_var.set(f"{scale:.4f}")
                            self.over_cur_limit_var.set(f"{ocl_A:.2f}")
                            self.log(
                                f"读取电流环: 使能={'开启' if en else '关闭'}, "
                                f"Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.3f}, "
                                f"零点={offset}, 系数={scale:.4f}, "
                                f"过流={ocl_A:.2f}A")
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
        except ValueError:
            messagebox.showerror("错误", "参数格式错误，请检查输入")
            return

        if offset < 0 or offset > 4095:
            messagebox.showwarning("警告", "零点ADC范围: 0~4095")
            return
        if scale <= 0:
            messagebox.showwarning("警告", "标定系数必须大于0")
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
                        # 最后写使能寄存器
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_CUR_LOOP_EN, en)
                        self.root.after(0, lambda: self.log(
                            f"写入电流环: 使能={'开启' if en else '关闭'}, "
                            f"Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.3f}, "
                            f"零点={offset}, 系数={scale:.4f}, "
                            f"过流={ocl_A:.2f}A (相对值{ocl_reg})"))
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
        """标定零点: 读取当前ADC值作为零电流参考
        要求电机停止且无电流流过时执行"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            return
        if not messagebox.askyesno("确认",
                "标定零点前请确保:\n"
                "1. 电机已停止 (未启动)\n"
                "2. 无电流流过采样电阻\n"
                "3. 电流采样电路已上电\n\n"
                "确定执行零点标定吗?"):
            return
        def do_calib():
            try:
                if self.serial_lock.acquire(timeout=1.0):
                    try:
                        # 读取实际电流寄存器, 提取ADC原始值
                        # 实际电流 = (ADC - offset) * scale, 我们需要反推ADC
                        # 但更直接的方式: 读取多个值取平均作为offset
                        # 这里简化处理: 读实际电流值, 如果接近0就用当前offset
                        # 实际上offset需要下位机配合, 我们这里只提示用户
                        # 当前实现: 读取一次实际电流, 显示给用户参考
                        data = self.modbus.read_holding_registers(
                            self.get_slave_addr(), self.REG_CURRENT_ACTUAL, 1)
                        cur_raw = data[0]
                        if cur_raw >= 0x8000:
                            cur_raw -= 0x10000
                        cur_A = cur_raw / 10.0 / 121.0
                        self.root.after(0, lambda: messagebox.showinfo("标定结果",
                            f"当前电流读数: {cur_A:.3f} A (相对值{cur_raw/10.0:.1f})\n\n"
                            f"如果电机已停止且电流接近0, 说明当前offset={self.cur_offset_var.get()}正确。\n"
                            f"如果电流偏离0较多, 需要手动调整零点ADC值后写入。\n\n"
                            f"提示: 零点ADC = 当前ADC原始读数, 但需要下位机提供原始ADC值。"))
                    finally:
                        self.serial_lock.release()
                else:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "串口忙"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"标定失败: {str(e)}"))
        threading.Thread(target=do_calib, daemon=True).start()

    def toggle_monitor(self):
        """切换监控状态"""
        if self.monitoring:
            self.monitoring = False
            self.monitor_btn.config(text="开始监控")
            self.log("停止监控")
        else:
            if not self.modbus:
                messagebox.showwarning("警告", "请先连接串口")
                return
                
            self.monitoring = True
            self.monitor_btn.config(text="停止监控")
            self.log("开始监控")
            self.monitor_thread = threading.Thread(target=self.monitor_loop, daemon=True)
            self.monitor_thread.start()
            
    def monitor_loop(self):
        """监控循环"""
        dir_read_counter = 0  # 方向寄存器读取计数器，降低读取频率
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
                    # 读取状态寄存器（当前位置、速度、模式、状态、PWM输出、PID实时值、电流实际/目标、ADC采样）
                    # 0x0100~0x0115 共22个寄存器 (含REG_SUPPLY_VOLTAGE/EXT_ADC1/EXT_ADC2)
                    data = self.modbus.read_holding_registers(
                        self.get_slave_addr(),
                        self.REG_CURRENT_POS_H3,
                        22
                    )
                    # 读取目标位置寄存器（64位）
                    target_data = self.modbus.read_holding_registers(
                        self.get_slave_addr(),
                        self.REG_TARGET_POS_H3,
                        4
                    )
                    start_flag_data = self.modbus.read_holding_registers(
                        self.get_slave_addr(),
                        self.REG_START_MODE,
                        1
                    )
                    # 读取电流环使能+过流状态 (0x0045和0x004C, 不连续, 分2次读)
                    cur_en_data = self.modbus.read_holding_registers(
                        self.get_slave_addr(),
                        self.REG_CUR_LOOP_EN,
                        1
                    )
                    over_cur_data = self.modbus.read_holding_registers(
                        self.get_slave_addr(),
                        self.REG_OVER_CUR_STATUS,
                        1
                    )
                    # 方向寄存器每20次才读1次（方向很少变化，减少通信占用）
                    dir_data = None
                    dir_read_counter += 1
                    if dir_read_counter >= 20:
                        dir_read_counter = 0
                        try:
                            dir_data = self.modbus.read_holding_registers(
                                self.get_slave_addr(),
                                self.REG_DIRECTION,
                                2
                            )
                        except Exception:
                            dir_data = None  # 读取失败跳过，不影响主流程
                finally:
                    self.serial_lock.release()
                
                # 解析目标位置（64位有符号）
                target_position = (target_data[0] << 48) | (target_data[1] << 32) | (target_data[2] << 16) | target_data[3]
                if target_position >= 0x8000000000000000:
                    target_position -= 0x10000000000000000
                
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
                start_flag = start_flag_data[0] != 0
                homing_active = (status & 0x4000) != 0
                homing_failed = (status & 0x2000) != 0
                stall_tripped = (status & 0x1000) != 0
                pb4_high = (status & 0x0400) != 0  # bit10 = PB4电平
                pb5_high = (status & 0x0200) != 0  # bit9  = PB5电平
                # ADC采样值(0x0113~0x0115 = data[19]~data[21])
                supply_voltage_raw = data[19]   # 供电电压(单位0.01V)
                ext_adc1_raw = data[20]         # 外部ADC1(PC2, 0~4095)
                ext_adc2_raw = data[21]         # 外部ADC2(PC3, 0~4095)
                supply_voltage_V = supply_voltage_raw / 100.0  # 转换为V

                # 解析PWM输出 (16位有符号, 范围-1000~+1000, 转换为百分比)
                pwm_raw = data[8]
                if pwm_raw >= 0x8000:
                    pwm_raw -= 0x10000
                pwm_percent = pwm_raw / 10.0  # -1000~+1000 -> -100.0%~+100.0%

                # 更新界面
                mode_names = {0: "位置", 1: "速度", 2: "位置速度", 3: "开环", 4: "外部目标位置", 5: "外部目标速度", 6: "待机", 7: "ADC转速", 8: "ADC位置转速", 9: "ADC开环", 10: "ADC位置"}

                self.root.after(0, lambda p=position: self.status_labels['position'].config(text=str(p)))
                self.root.after(0, lambda tp=target_position: self.status_labels['target_position'].config(text=str(tp)))
                self.root.after(0, lambda s=speed: self.status_labels['speed'].config(text=str(s)))
                self.root.after(0, lambda m=mode: self.status_labels['mode'].config(text=mode_names.get(m & 0x0F, "未知")))
                self.root.after(0, lambda f=start_flag: self.status_labels['start_flag'].config(
                    text="已设置" if f else "未设置",
                    foreground="green" if f else "red"
                ))
                self.root.after(0, lambda active=homing_active, failed=homing_failed: self.status_labels['homing'].config(
                    text="失败" if failed else ("复位中" if active else "空闲"),
                    foreground="red" if failed else ("orange" if active else "gray")
                ))
                self.root.after(0, lambda tripped=stall_tripped: self.status_labels['stall'].config(
                    text="已触发" if tripped else "正常",
                    foreground="red" if tripped else "gray"
                ))
                # ADC采样值标签更新
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
                # PB4/PB5引脚电平标签更新(高电平=绿色"高", 低电平=灰色"低")
                self.root.after(0, lambda h=pb4_high: self.status_labels['pb4_level'].config(
                    text="高" if h else "低",
                    foreground="green" if h else "gray"
                ))
                self.root.after(0, lambda h=pb5_high: self.status_labels['pb5_level'].config(
                    text="高" if h else "低",
                    foreground="green" if h else "gray"
                ))
                # PWM显示：带方向颜色（正=红, 负=蓝, 零=灰）
                def update_pwm(p):
                    self.status_labels['pwm'].config(
                        text=f"{p:+.1f}%",
                        foreground="red" if p > 0.1 else ("blue" if p < -0.1 else "gray")
                    )
                self.root.after(0, lambda p=pwm_percent: update_pwm(p))

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

                self.root.after(0, lambda e=pid_error: self.status_labels['pid_error'].config(text=f"{e:+.2f}"))
                self.root.after(0, lambda p=pid_p: self.status_labels['pid_p'].config(text=f"{p:+.2f}"))
                self.root.after(0, lambda i=pid_i: self.status_labels['pid_i'].config(text=f"{i:+.2f}"))
                self.root.after(0, lambda d=pid_d: self.status_labels['pid_d'].config(text=f"{d:+.2f}"))

                # 解析电流环状态 (data[17]=电流实际, data[18]=电流目标, 均为×10有符号)
                cur_actual_raw = data[17]
                if cur_actual_raw >= 0x8000:
                    cur_actual_raw -= 0x10000
                cur_target_raw = data[18]
                if cur_target_raw >= 0x8000:
                    cur_target_raw -= 0x10000
                # 相对值×10 → 安培: 1A ≈ 121相对单位
                cur_actual_A = cur_actual_raw / 10.0 / 121.0
                cur_target_A = cur_target_raw / 10.0 / 121.0
                cur_en = cur_en_data[0] != 0
                over_cur_tripped = over_cur_data[0] != 0

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
                if dir_data is not None:
                    motor_dir = dir_data[0]
                    encoder_dir = dir_data[1]
                    self.root.after(0, lambda md=motor_dir: self.status_labels['motor_dir'].config(
                        text="反转" if md else "正转"
                    ))
                    self.root.after(0, lambda ed=encoder_dir: self.status_labels['encoder_dir'].config(
                        text="反转" if ed else "正常"
                    ))
                
                time.sleep(0.1)
                
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

    def on_acq_type_changed(self, event=None):
        """采集类型切换回调"""
        if not self.modbus:
            messagebox.showwarning("警告", "请先连接串口")
            # 恢复为默认值
            self.acq_type_var.set("转速")
            return
        acq_type = self._current_acq_type()
        # 更新Y轴标签(即时响应)
        self.speed_ax.set_ylabel(self._acq_y_label(acq_type))
        self.speed_canvas.draw()
        # 写寄存器放到线程中，避免阻塞UI
        def _worker():
            try:
                with self.serial_lock:
                    self.modbus.write_single_register(self.get_slave_addr(), self.REG_SPEED_ACQ_TYPE, acq_type)
                self.root.after(0, lambda: self.log(f"设置采集类型: {self._acq_label(acq_type)}"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"设置采集类型失败: {e}"))
        threading.Thread(target=_worker, daemon=True).start()

    # 采集类型映射表: 0=转速, 1=PWM, 2=位置, 3=电流, 4=PC0电压ADC, 5=PC2外部ADC, 6=PC3外部ADC
    _ACQ_LABELS = {0: "转速", 1: "PWM输出", 2: "位置", 3: "电流",
                   4: "PC0电压ADC", 5: "PC2外部ADC", 6: "PC3外部ADC"}
    _ACQ_Y_LABELS = {0: "转速 (脉冲/秒)", 1: "PWM输出", 2: "位置偏移 (脉冲)", 3: "电流 (A)",
                     4: "供电电压 (V)", 5: "PC2外部ADC原始值 (0~4095)", 6: "PC3外部ADC原始值 (0~4095)"}
    _ACQ_TITLES = {0: "转速采集曲线", 1: "PWM采集曲线", 2: "位置采集曲线", 3: "电流采集曲线",
                   4: "供电电压采集曲线", 5: "PC2外部ADC采集曲线", 6: "PC3外部ADC采集曲线"}
    # 数据换算系数: 原始值 × 系数 + 偏置 = 显示值
    # 电流: 相对值±1000=±8.25A → A = 相对值 × 0.00825
    # PC0电压: ADC 0~4095, 100K:10K分压×11, 3.3V参考 → V = ADC × 3.3×11/4095
    _ACQ_SCALE = {3: 0.00825, 4: 3.3 * 11.0 / 4095.0}
    _ACQ_LABEL_TO_TYPE = {"转速": 0, "PWM": 1, "位置": 2, "电流": 3,
                          "PC0电压ADC": 4, "PC2外部ADC": 5, "PC3外部ADC": 6}

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
        """设置采集分频值和采集点数"""
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
        # 写寄存器放到线程中，避免阻塞UI
        self.acq_status_label.config(text="状态: 设置中...", foreground="blue")
        def _worker():
            try:
                with self.serial_lock:
                    self.modbus.write_single_register(self.get_slave_addr(), self.REG_SPEED_ACQ_DIV, div)
                    self.modbus.write_single_register(self.get_slave_addr(), self.REG_SPEED_ACQ_SIZE, size)
                self.root.after(0, lambda: self.acq_status_label.config(text="状态: 已设置", foreground="gray"))
                self.root.after(0, lambda: self.log(f"设置采集参数: 分频={div}(周期={div*0.1}ms), 点数={size}"))
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

            # 绘图
            acq_type = self._current_acq_type()
            self.root.after(0, lambda: self._plot_speed_data(speed_data))
            self.root.after(0, lambda: self.log(f"读取{len(speed_data)}个{self._acq_label(acq_type)}数据并绘图"))

        except Exception as e:
            self.root.after(0, lambda: self.acq_status_label.config(
                text="状态: 读取失败", foreground="red"))
            self.root.after(0, lambda: messagebox.showerror("错误", f"读取数据失败: {e}"))
        finally:
            self.acq_reading = False  # 恢复监控线程

    def _plot_speed_data(self, speed_data):
        """绘制采集曲线"""
        acq_type = self._current_acq_type()
        y_label = self._acq_y_label(acq_type)
        title = self._acq_title(acq_type)
        # 电流/电压类型: 将原始值换算为带物理单位的值
        scale = self._ACQ_SCALE.get(acq_type)
        if scale is not None:
            plot_data = [v * scale for v in speed_data]
        else:
            plot_data = speed_data
        self.speed_ax.clear()
        self.speed_ax.plot(range(len(plot_data)), plot_data, 'b-', linewidth=0.8)
        self.speed_ax.set_xlabel("采样点")
        self.speed_ax.set_ylabel(y_label)
        self.speed_ax.set_title(f"{title} ({len(plot_data)}点)")
        self.speed_ax.grid(True)
        self.speed_fig.tight_layout()
        self.speed_canvas.draw()

    def clear_speed_plot(self):
        """清空采集曲线"""
        acq_type = self._current_acq_type()
        y_label = self._acq_y_label(acq_type)
        title = self._acq_title(acq_type)
        self.speed_ax.clear()
        self.speed_ax.set_xlabel("采样点")
        self.speed_ax.set_ylabel(y_label)
        self.speed_ax.set_title(title)
        self.speed_ax.grid(True)
        self.speed_fig.tight_layout()
        self.speed_canvas.draw()
        self.acq_status_label.config(text="状态: 空闲", foreground="gray")
        self.log("清空采集曲线")

if __name__ == "__main__":
    root = tk.Tk()
    app = ModbusDebuggerApp(root)
    root.mainloop()
