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
    REG_INPUT1_FUNC = 0x0022       # PB4功能 (0=脉冲,1=方向,2=复位,3=限位,4=目标位置电平)
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
    REG_TIM2_ARR = 0x0039               # TIM2自动重装载值(ARR), 决定脉冲采样中断周期
                                        # 周期=(ARR+1)/64MHz, 默认639->10us
                                        # 范围99~65535, 即约1.56us~1.024ms
    REG_SPEED_ACQ_START = 0x003A        # 转速采集启动/状态: 写1启动, 读=状态(0=空闲,1=采集中,2=完成)
    REG_SPEED_ACQ_DIV = 0x003B          # 转速采集分频值(1=100us, 50=5ms, 默认50)
    REG_SPEED_ACQ_COUNT = 0x003C        # 转速采集已采样数量(只读, 0~512)
    REG_SPEED_ACQ_STATUS = 0x003D       # 转速采集状态(只读)
    REG_SPEED_ACQ_TYPE = 0x003E         # 采集类型: 0=转速, 1=PWM, 2=位置(相对起始偏移, ±32767脉冲)
    REG_SPEED_ACQ_SIZE = 0x003F         # 采集点数(1~5120)

    REG_STALL_PROT_EN = 0x0040          # 堵转保护使能: 0=关闭, 1=开启
    REG_STALL_ERR_LIMIT = 0x0041        # 堵转误差阈值(位置=脉冲, 速度=脉冲/秒)
    REG_STALL_TIME = 0x0042             # 堵转持续时长(单位=PID周期5ms)
    REG_STALL_STATUS = 0x0043           # 堵转状态(只读): 0=正常, 1=已触发
    REG_STALL_RESET = 0x0044            # 堵转复位(写1清除)

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
    
    # 模式定义
    MODE_POSITION = 0
    MODE_SPEED = 1
    MODE_VELOCITY_POSITION = 2  # 基于位置的速度模式
    MODE_OPENLOOP = 3           # 开环模式
    MODE_EXTERNAL_TARGET = 4     # 外部目标位置模式
    MODE_EXTERNAL_TARGET_SPEED = 5  # 外部目标速度模式
    
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

        def _on_mousewheel(event):
            left_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        left_canvas.bind_all('<MouseWheel>', _on_mousewheel)
        
        # 模式控制
        mode_frame = ttk.LabelFrame(left_frame, text="模式控制", padding=10)
        mode_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(mode_frame, text="位置模式", command=lambda: self.set_mode(self.MODE_POSITION)).pack(fill=tk.X, pady=2)
        ttk.Button(mode_frame, text="速度模式", command=lambda: self.set_mode(self.MODE_SPEED)).pack(fill=tk.X, pady=2)
        ttk.Button(mode_frame, text="位置速度模式", command=lambda: self.set_mode(self.MODE_VELOCITY_POSITION)).pack(fill=tk.X, pady=2)
        ttk.Button(mode_frame, text="开环模式", command=lambda: self.set_mode(self.MODE_OPENLOOP)).pack(fill=tk.X, pady=2)
        ttk.Button(mode_frame, text="外部目标位置模式", command=lambda: self.set_mode(self.MODE_EXTERNAL_TARGET)).pack(fill=tk.X, pady=2)
        ttk.Button(mode_frame, text="外部目标速度模式", command=lambda: self.set_mode(self.MODE_EXTERNAL_TARGET_SPEED)).pack(fill=tk.X, pady=2)
        
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
        
        ttk.Label(pid_frame, text="位置环 Kp:").grid(row=0, column=0, padx=5, pady=2)
        self.pos_kp_var = tk.StringVar(value="7.0")
        ttk.Entry(pid_frame, textvariable=self.pos_kp_var, width=10).grid(row=0, column=1, padx=5, pady=2)
        
        ttk.Label(pid_frame, text="位置环 Ki:").grid(row=1, column=0, padx=5, pady=2)
        self.pos_ki_var = tk.StringVar(value="0.1")
        ttk.Entry(pid_frame, textvariable=self.pos_ki_var, width=10).grid(row=1, column=1, padx=5, pady=2)
        
        ttk.Label(pid_frame, text="位置环 Kd:").grid(row=2, column=0, padx=5, pady=2)
        self.pos_kd_var = tk.StringVar(value="0.1")
        ttk.Entry(pid_frame, textvariable=self.pos_kd_var, width=10).grid(row=2, column=1, padx=5, pady=2)
        
        ttk.Label(pid_frame, text="速度环 Kp:").grid(row=3, column=0, padx=5, pady=2)
        self.spd_kp_var = tk.StringVar(value="0.3")
        ttk.Entry(pid_frame, textvariable=self.spd_kp_var, width=10).grid(row=3, column=1, padx=5, pady=2)
        
        ttk.Label(pid_frame, text="速度环 Ki:").grid(row=4, column=0, padx=5, pady=2)
        self.spd_ki_var = tk.StringVar(value="3.0")
        ttk.Entry(pid_frame, textvariable=self.spd_ki_var, width=10).grid(row=4, column=1, padx=5, pady=2)
        
        ttk.Label(pid_frame, text="速度环 Kd:").grid(row=5, column=0, padx=5, pady=2)
        self.spd_kd_var = tk.StringVar(value="0.0")
        ttk.Entry(pid_frame, textvariable=self.spd_kd_var, width=10).grid(row=5, column=1, padx=5, pady=2)
        
        ttk.Button(pid_frame, text="应用PID", command=self.apply_pid).grid(row=6, column=0, pady=5)
        ttk.Button(pid_frame, text="读取PID", command=self.read_pid).grid(row=6, column=1, pady=5)
        
        # 其他参数
        other_frame = ttk.LabelFrame(left_frame, text="其他参数", padding=10)
        other_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(other_frame, text="死区:").grid(row=0, column=0, padx=5, pady=2)
        self.dead_zone_var = tk.StringVar(value="1")
        ttk.Entry(other_frame, textvariable=self.dead_zone_var, width=10).grid(row=0, column=1, padx=5, pady=2)
        
        ttk.Label(other_frame, text="最大输出:").grid(row=1, column=0, padx=5, pady=2)
        self.max_output_var = tk.StringVar(value="900")
        ttk.Entry(other_frame, textvariable=self.max_output_var, width=10).grid(row=1, column=1, padx=5, pady=2)

        ttk.Label(other_frame, text="最大运行速度:").grid(row=2, column=0, padx=5, pady=2)
        self.max_run_speed_var = tk.StringVar(value="0")
        ttk.Entry(other_frame, textvariable=self.max_run_speed_var, width=10).grid(row=2, column=1, padx=5, pady=2)
        ttk.Label(other_frame, text="0=无限制").grid(row=2, column=2, padx=5, pady=2)
        
        ttk.Label(other_frame, text="启动方式:").grid(row=3, column=0, padx=5, pady=2)
        self.start_mode_var = tk.StringVar(value="直接启动")
        self.start_mode_combo = ttk.Combobox(other_frame, textvariable=self.start_mode_var, width=8,
                      values=["直接启动", "标志位"], state="readonly")
        self.start_mode_combo.grid(row=3, column=1, padx=5, pady=2)
        self.start_btn = ttk.Button(other_frame, text="启动", command=self.start_motor)
        self.start_btn.grid(row=3, column=2, padx=5, pady=2)
        self.start_mode_combo.bind('<<ComboboxSelected>>', self._on_start_mode_changed)
        self._on_start_mode_changed(None)

        ttk.Label(other_frame, text="电机方向:").grid(row=4, column=0, padx=5, pady=2)
        self.motor_dir_var = tk.StringVar(value="正转")
        ttk.Combobox(other_frame, textvariable=self.motor_dir_var, width=8,
                     values=["正转", "反转"], state="readonly").grid(row=4, column=1, padx=5, pady=2)
        
        ttk.Label(other_frame, text="编码器方向:").grid(row=5, column=0, padx=5, pady=2)
        self.encoder_dir_var = tk.StringVar(value="正常")
        ttk.Combobox(other_frame, textvariable=self.encoder_dir_var, width=8,
                     values=["正常", "反转"], state="readonly").grid(row=5, column=1, padx=5, pady=2)

        ttk.Label(other_frame, text="复位模式:").grid(row=6, column=0, padx=5, pady=2)
        self.home_mode_var = tk.StringVar(value="关闭")
        ttk.Combobox(other_frame, textvariable=self.home_mode_var, width=8,
                     values=["关闭", "堵转", "精确复位"], state="readonly").grid(row=6, column=1, padx=5, pady=2)

        ttk.Label(other_frame, text="复位方向:").grid(row=7, column=0, padx=5, pady=2)
        self.home_dir_var = tk.StringVar(value="负方向")
        ttk.Combobox(other_frame, textvariable=self.home_dir_var, width=8,
                     values=["负方向", "正方向"], state="readonly").grid(row=7, column=1, padx=5, pady=2)

        ttk.Label(other_frame, text="复位电流:").grid(row=8, column=0, padx=5, pady=2)
        self.home_current_var = tk.StringVar(value="300")
        ttk.Entry(other_frame, textvariable=self.home_current_var, width=10).grid(row=8, column=1, padx=5, pady=2)

        ttk.Label(other_frame, text="复位速度:").grid(row=9, column=0, padx=5, pady=2)
        self.home_speed_var = tk.StringVar(value="1000")
        ttk.Entry(other_frame, textvariable=self.home_speed_var, width=10).grid(row=9, column=1, padx=5, pady=2)
        
        ttk.Label(other_frame, text="精确检测速度:").grid(row=10, column=0, padx=5, pady=2)
        self.home_precision_speed_var = tk.StringVar(value="100")
        ttk.Entry(other_frame, textvariable=self.home_precision_speed_var, width=10).grid(row=10, column=1, padx=5, pady=2)
        
        ttk.Label(other_frame, text="检测次数:").grid(row=11, column=0, padx=5, pady=2)
        self.home_precision_cycles_var = tk.StringVar(value="3")
        ttk.Entry(other_frame, textvariable=self.home_precision_cycles_var, width=10).grid(row=11, column=1, padx=5, pady=2)

        ttk.Label(other_frame, text="最大距离:").grid(row=12, column=0, padx=5, pady=2)
        self.home_max_distance_var = tk.StringVar(value="10000")
        ttk.Entry(other_frame, textvariable=self.home_max_distance_var, width=10).grid(row=12, column=1, padx=5, pady=2)

        ttk.Label(other_frame, text="复位偏置:").grid(row=13, column=0, padx=5, pady=2)
        self.home_back_distance_var = tk.StringVar(value="100")
        ttk.Entry(other_frame, textvariable=self.home_back_distance_var, width=10).grid(row=13, column=1, padx=5, pady=2)

        ttk.Label(other_frame, text="开机自复位:").grid(row=14, column=0, padx=5, pady=2)
        self.home_auto_start_var = tk.StringVar(value="开启")
        ttk.Combobox(other_frame, textvariable=self.home_auto_start_var, width=8,
                     values=["关闭", "开启"], state="readonly").grid(row=14, column=1, padx=5, pady=2)

        ttk.Label(other_frame, text="TIM2采样周期:").grid(row=15, column=0, padx=5, pady=2)
        self.tim2_arr_var = tk.StringVar(value="639")
        ttk.Entry(other_frame, textvariable=self.tim2_arr_var, width=10).grid(row=15, column=1, padx=5, pady=2)
        ttk.Label(other_frame, text="ARR值, 639=10us").grid(row=15, column=2, padx=5, pady=2)

        ttk.Button(other_frame, text="应用", command=self.apply_other_params).grid(row=16, column=0, pady=5)
        ttk.Button(other_frame, text="读取参数", command=self.read_other_params).grid(row=16, column=1, pady=5)
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

        ttk.Label(pin_frame, text="PB4").grid(row=1, column=0, padx=5, pady=2)
        self.pin4_func_var = tk.StringVar(value="脉冲")
        ttk.Combobox(pin_frame, textvariable=self.pin4_func_var, width=9,
                     values=["脉冲", "方向", "复位开关", "限位开关", "目标位置速度", "无功能"], state="readonly").grid(row=1, column=1, padx=2, pady=2)
        self.pin4_pol_var = tk.StringVar(value="高电平")
        ttk.Combobox(pin_frame, textvariable=self.pin4_pol_var, width=8,
                     values=["高电平", "低电平"], state="readonly").grid(row=1, column=2, padx=2, pady=2)
        self.pin4_ldir_var = tk.StringVar(value="停止正方向")
        ttk.Combobox(pin_frame, textvariable=self.pin4_ldir_var, width=10,
                     values=["停止正方向", "停止负方向"], state="readonly").grid(row=1, column=3, padx=2, pady=2)
        self.pin4_target_pos_var = tk.StringVar(value="0")
        ttk.Entry(pin_frame, textvariable=self.pin4_target_pos_var, width=14).grid(row=1, column=4, padx=2, pady=2)
        self.pin4_target_speed_var = tk.StringVar(value="0")
        ttk.Entry(pin_frame, textvariable=self.pin4_target_speed_var, width=14).grid(row=1, column=5, padx=2, pady=2)

        ttk.Label(pin_frame, text="PB5").grid(row=2, column=0, padx=5, pady=2)
        self.pin5_func_var = tk.StringVar(value="方向")
        ttk.Combobox(pin_frame, textvariable=self.pin5_func_var, width=9,
                     values=["脉冲", "方向", "复位开关", "限位开关", "目标位置速度", "无功能"], state="readonly").grid(row=2, column=1, padx=2, pady=2)
        self.pin5_pol_var = tk.StringVar(value="高电平")
        ttk.Combobox(pin_frame, textvariable=self.pin5_pol_var, width=8,
                     values=["高电平", "低电平"], state="readonly").grid(row=2, column=2, padx=2, pady=2)
        self.pin5_ldir_var = tk.StringVar(value="停止负方向")
        ttk.Combobox(pin_frame, textvariable=self.pin5_ldir_var, width=10,
                     values=["停止正方向", "停止负方向"], state="readonly").grid(row=2, column=3, padx=2, pady=2)
        self.pin5_target_pos_var = tk.StringVar(value="0")
        ttk.Entry(pin_frame, textvariable=self.pin5_target_pos_var, width=14).grid(row=2, column=4, padx=2, pady=2)
        self.pin5_target_speed_var = tk.StringVar(value="0")
        ttk.Entry(pin_frame, textvariable=self.pin5_target_speed_var, width=14).grid(row=2, column=5, padx=2, pady=2)

        btn_frame = ttk.Frame(pin_frame)
        btn_frame.grid(row=3, column=0, columnspan=6, pady=5)
        ttk.Button(btn_frame, text="读取引脚配置", command=self.read_pin_config).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="写入引脚配置", command=self.write_pin_config).pack(side=tk.LEFT, padx=5)

        # === 堵转保护配置 ===
        stall_frame = ttk.LabelFrame(left_frame, text="堵转保护", padding=10)
        stall_frame.pack(fill=tk.X, pady=5)

        ttk.Label(stall_frame, text="使能:").grid(row=0, column=0, padx=5, pady=2, sticky=tk.W)
        self.stall_en_var = tk.StringVar(value="关闭")
        ttk.Combobox(stall_frame, textvariable=self.stall_en_var, width=6,
                     values=["关闭", "开启"], state="readonly").grid(row=0, column=1, padx=5, pady=2, sticky=tk.W)

        ttk.Label(stall_frame, text="误差阈值:").grid(row=0, column=2, padx=5, pady=2, sticky=tk.W)
        self.stall_err_var = tk.StringVar(value="200")
        ttk.Entry(stall_frame, textvariable=self.stall_err_var, width=8).grid(row=0, column=3, padx=5, pady=2, sticky=tk.W)
        ttk.Label(stall_frame, text="(脉冲/脉冲·秒⁻¹)").grid(row=0, column=4, padx=2, pady=2, sticky=tk.W)

        ttk.Label(stall_frame, text="持续时长:").grid(row=1, column=0, padx=5, pady=2, sticky=tk.W)
        self.stall_time_var = tk.StringVar(value="200")
        ttk.Entry(stall_frame, textvariable=self.stall_time_var, width=8).grid(row=1, column=1, padx=5, pady=2, sticky=tk.W)
        ttk.Label(stall_frame, text="(×5ms = 1.0s)").grid(row=1, column=2, columnspan=3, padx=2, pady=2, sticky=tk.W)

        stall_btn_frame = ttk.Frame(stall_frame)
        stall_btn_frame.grid(row=2, column=0, columnspan=5, pady=5)
        ttk.Button(stall_btn_frame, text="读取参数", command=self.read_stall_config).pack(side=tk.LEFT, padx=5)
        ttk.Button(stall_btn_frame, text="写入参数", command=self.write_stall_config).pack(side=tk.LEFT, padx=5)
        ttk.Button(stall_btn_frame, text="复位堵转", command=self.reset_stall).pack(side=tk.LEFT, padx=5)

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

        ttk.Label(status_frame, text="电机方向:").grid(row=5, column=0, padx=5, pady=2, sticky=tk.W)
        self.status_labels['motor_dir'] = ttk.Label(status_frame, text="正转", foreground="blue", font=("Arial", 12, "bold"))
        self.status_labels['motor_dir'].grid(row=5, column=1, padx=5, pady=2, sticky=tk.W)

        ttk.Label(status_frame, text="编码器方向:").grid(row=6, column=0, padx=5, pady=2, sticky=tk.W)
        self.status_labels['encoder_dir'] = ttk.Label(status_frame, text="正常", foreground="blue", font=("Arial", 12, "bold"))
        self.status_labels['encoder_dir'].grid(row=6, column=1, padx=5, pady=2, sticky=tk.W)

        ttk.Label(status_frame, text="PWM输出:").grid(row=7, column=0, padx=5, pady=2, sticky=tk.W)
        self.status_labels['pwm'] = ttk.Label(status_frame, text="0.0%", foreground="purple", font=("Arial", 12, "bold"))
        self.status_labels['pwm'].grid(row=7, column=1, padx=5, pady=2, sticky=tk.W)

        ttk.Label(status_frame, text="复位状态:").grid(row=8, column=0, padx=5, pady=2, sticky=tk.W)
        self.status_labels['homing'] = ttk.Label(status_frame, text="空闲", foreground="gray", font=("Arial", 12, "bold"))
        self.status_labels['homing'].grid(row=8, column=1, padx=5, pady=2, sticky=tk.W)

        ttk.Label(status_frame, text="堵转保护:").grid(row=9, column=0, padx=5, pady=2, sticky=tk.W)
        self.status_labels['stall'] = ttk.Label(status_frame, text="正常", foreground="gray", font=("Arial", 12, "bold"))
        self.status_labels['stall'].grid(row=9, column=1, padx=5, pady=2, sticky=tk.W)

        ttk.Label(status_frame, text="PID误差:").grid(row=10, column=0, padx=5, pady=2, sticky=tk.W)
        self.status_labels['pid_error'] = ttk.Label(status_frame, text="0.00", foreground="teal", font=("Arial", 12, "bold"))
        self.status_labels['pid_error'].grid(row=10, column=1, padx=5, pady=2, sticky=tk.W)

        ttk.Label(status_frame, text="PID-P:").grid(row=11, column=0, padx=5, pady=2, sticky=tk.W)
        self.status_labels['pid_p'] = ttk.Label(status_frame, text="0.00", foreground="teal", font=("Arial", 12, "bold"))
        self.status_labels['pid_p'].grid(row=11, column=1, padx=5, pady=2, sticky=tk.W)

        ttk.Label(status_frame, text="PID-I:").grid(row=12, column=0, padx=5, pady=2, sticky=tk.W)
        self.status_labels['pid_i'] = ttk.Label(status_frame, text="0.00", foreground="teal", font=("Arial", 12, "bold"))
        self.status_labels['pid_i'].grid(row=12, column=1, padx=5, pady=2, sticky=tk.W)

        ttk.Label(status_frame, text="PID-D:").grid(row=13, column=0, padx=5, pady=2, sticky=tk.W)
        self.status_labels['pid_d'] = ttk.Label(status_frame, text="0.00", foreground="teal", font=("Arial", 12, "bold"))
        self.status_labels['pid_d'].grid(row=13, column=1, padx=5, pady=2, sticky=tk.W)

        self.monitor_btn = ttk.Button(status_frame, text="开始监控", command=self.toggle_monitor)
        self.monitor_btn.grid(row=14, column=0, columnspan=2, pady=5)

        # === 采集曲线图（可调高度） ===
        chart_frame = ttk.LabelFrame(right_vpaned, text="采集曲线", padding=5)
        right_vpaned.add(chart_frame, weight=3)

        # 采集控制工具栏
        ctrl_bar = ttk.Frame(chart_frame)
        ctrl_bar.pack(fill=tk.X, pady=(0, 5))

        ttk.Label(ctrl_bar, text="采集类型:").pack(side=tk.LEFT, padx=(0, 2))
        self.acq_type_var = tk.StringVar(value="转速")
        acq_type_combo = ttk.Combobox(ctrl_bar, textvariable=self.acq_type_var, width=8,
                                      values=["转速", "PWM", "位置"], state="readonly")
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
                        mode_names = {0: "位置", 1: "速度", 2: "位置速度", 3: "开环", 4: "外部目标位置", 5: "外部目标速度"}
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
                        tim2_arr = int(self.tim2_arr_var.get())
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
                        # TIM2 ARR (范围99~65535, 超界由固件拒绝)
                        if 99 <= tim2_arr <= 65535:
                            self.modbus.write_single_register(self.get_slave_addr(), self.REG_TIM2_ARR, tim2_arr)

                        hm_name = {0: "关闭", 1: "堵转", 2: "精确复位"}
                        self.root.after(0, lambda: self.log(
                            f"应用参数成功: 死区={dead_zone}, 最大输出={max_output}, "
                            f"最大运行速度={max_run_speed}, "
                            f"启动方式={'标志位' if start_mode else '直接'}, "
                            f"电机={'反转' if motor_dir else '正转'}, "
                            f"编码器={'反转' if encoder_dir else '正常'}, "
                            f"复位={hm_name.get(home_mode, '?')}, "
                            f"TIM2_ARR={tim2_arr}"))
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
                        tim2_arr_data = self.modbus.read_holding_registers(
                            self.get_slave_addr(), self.REG_TIM2_ARR, 1)
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
                        tim2_arr = tim2_arr_data[0]

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
                            self.tim2_arr_var.set(str(tim2_arr))
                            self.log(
                                f"读取参数: 死区={dead_zone}, 最大输出={max_output}, "
                                f"最大运行速度={max_run_speed}, "
                                f"启动模式={'标志位' if start_mode else '直接'}, "
                                f"电机={'反转' if motor_dir else '正转'}, "
                                f"编码器={'反转' if encoder_dir else '正常'}, "
                                f"复位={hm_display.get(home_mode, '?')}, "
                                f"TIM2_ARR={tim2_arr}")
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
                        # TIM2 ARR 恢复默认639 (10us周期)
                        self.modbus.write_single_register(
                            self.get_slave_addr(), self.REG_TIM2_ARR, 639)

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
                            self.tim2_arr_var.set("639")
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
                        fm = {0:"脉冲",1:"方向",2:"复位开关",3:"限位开关",4:"目标位置速度",5:"无功能"}
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
        fm = {"脉冲":0,"方向":1,"复位开关":2,"限位开关":3,"目标位置速度":4,"无功能":5}
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
                    # 读取状态寄存器（当前位置、速度、模式、状态、PWM输出、PID实时值）
                    data = self.modbus.read_holding_registers(
                        self.get_slave_addr(),
                        self.REG_CURRENT_POS_H3,
                        17
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

                # 解析PWM输出 (16位有符号, 范围-1000~+1000, 转换为百分比)
                pwm_raw = data[8]
                if pwm_raw >= 0x8000:
                    pwm_raw -= 0x10000
                pwm_percent = pwm_raw / 10.0  # -1000~+1000 -> -100.0%~+100.0%

                # 更新界面
                mode_names = {0: "位置", 1: "速度", 2: "位置速度", 3: "开环", 4: "外部目标位置", 5: "外部目标速度"}

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

    # 采集类型映射表: 0=转速, 1=PWM, 2=位置
    _ACQ_LABELS = {0: "转速", 1: "PWM输出", 2: "位置"}
    _ACQ_Y_LABELS = {0: "转速 (脉冲/秒)", 1: "PWM输出", 2: "位置偏移 (脉冲)"}
    _ACQ_TITLES = {0: "转速采集曲线", 1: "PWM采集曲线", 2: "位置采集曲线"}
    _ACQ_LABEL_TO_TYPE = {"转速": 0, "PWM": 1, "位置": 2}

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
        self.speed_ax.clear()
        self.speed_ax.plot(range(len(speed_data)), speed_data, 'b-', linewidth=0.8)
        self.speed_ax.set_xlabel("采样点")
        self.speed_ax.set_ylabel(y_label)
        self.speed_ax.set_title(f"{title} ({len(speed_data)}点)")
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
