#!/usr/bin/env python3
"""
PID 自动调参器模块，采用继电器法（Relay Method）
"""
import time
import numpy as np
from collections import deque

class PIDAutoTuner:
    """
    一个高层PID控制器自动调参器。
    它使用继电器法来激励系统产生持续振荡，然后测量振荡的幅值和周期，
    最终根据Ziegler-Nichols规则计算出推荐的P, I, D增益。
    """
    
    def __init__(self, logger, output_amplitude=0.3, input_tolerance=0.02, sample_time=0.1):
        """
        初始化调参器。
        
        Args:
            logger: 日志记录器。
            output_amplitude (float): 继电器输出的幅值（例如，速度指令 m/s）。
            input_tolerance (float): 继电器切换的死区（例如，位置误差 m）。
            sample_time (float): 控制和采样的周期（秒）。
        """
        self.logger = logger
        self.d = output_amplitude  # 继电器输出幅值
        self.h = input_tolerance   # 继电器切换死区
        self.sample_time = sample_time

        self.reset()

    def reset(self):
        """重置调参器状态，以便开始新的调参任务。"""
        self.logger.info("[AutoTuner] 状态已重置。")
        self.is_tuning = False
        self.start_time = 0
        self.history = deque(maxlen=200)  # 存储最近20秒的数据
        self.last_output = self.d
        self.peak_timestamps = []
        self.peak_values = []

    def start(self):
        """开始调参过程。"""
        self.reset()
        self.is_tuning = True
        self.start_time = time.time()
        self.logger.info(f"[AutoTuner] 开始自动调参... 继电器幅值: {self.d} m/s, 死区: {self.h} m")

    def step(self, current_error: float) -> float:
        """
        在每个控制周期调用此函数。
        
        Args:
            current_error (float): 当前的系统误差（例如，x轴或y轴的位置误差）。
            
        Returns:
            float: 应发送给执行器（无人机）的控制指令（例如，速度）。
        """
        if not self.is_tuning:
            return 0.0

        # 记录历史数据
        self.history.append({'time': time.time() - self.start_time, 'error': current_error})
        # 继电器逻辑
        if current_error > self.h:
            output = -self.d
        elif current_error < -self.h:
            output = self.d
        else:
            output = self.last_output  # 在死区内保持上一次的输出
        
        self.last_output = output
        return output

    def analyze(self) -> dict:
        """
        分析收集到的数据并计算PID参数。
        
        Returns:
            dict: 包含计算出的PID参数，如果数据不足则返回None。
        """
        self.is_tuning = False
        self.logger.info("[AutoTuner] 正在分析数据...")
        
        # 寻找所有波峰和波谷
        peaks = []
        for i in range(1, len(self.history) - 1):
            if (self.history[i]['error'] > self.history[i-1]['error'] and 
                self.history[i]['error'] > self.history[i+1]['error']) or \
               (self.history[i]['error'] < self.history[i-1]['error'] and 
                self.history[i]['error'] < self.history[i+1]['error']):
                peaks.append(self.history[i])

        if len(peaks) < 5:  # 需要足够的振荡周期来确保准确性
            self.logger.warning(f"[AutoTuner] 分析失败：振荡不明显或数据不足（找到 {len(peaks)} 个峰值）。")
            return None

        # 计算振荡周期 (Tu) - 取最后几个周期的平均值
        last_peaks_times = [p['time'] for p in peaks[-5:]]
        periods = np.diff(last_peaks_times)
        Tu = np.mean(periods) * 2  # 周期是两个相邻峰值时间的两倍

        # 计算振荡幅值 (a) - 取最后几个周期的平均幅值
        last_peaks_errors = [abs(p['error']) for p in peaks[-5:]]
        a = np.mean(last_peaks_errors)

        if a < self.h:
            self.logger.warning(f"[AutoTuner] 分析失败：振荡幅值 ({a:.3f}) 小于死区 ({self.h})。")
            return None

        # 计算临界增益 (Ku)
        Ku = (4 * self.d) / (np.pi * a)
        
        self.logger.info(f"[AutoTuner] 分析完成：临界增益 Ku={Ku:.4f}, 临界周期 Tu={Tu:.4f}s")

        # 根据Ziegler-Nichols规则计算PID参数
        # 经典规则
        kp_classic = 0.6 * Ku
        ki_classic = 1.2 * Ku / Tu
        kd_classic = 0.075 * Ku * Tu
        
        # Pessen积分规则 (较少超调)
        kp_pessen = 0.7 * Ku
        ki_pessen = 1.75 * Ku / Tu
        kd_pessen = 0.105 * Ku * Tu

        # 无超调规则
        kp_no_overshoot = 0.2 * Ku
        ki_no_overshoot = 0.4 * Ku / Tu
        kd_no_overshoot = 0.066 * Ku * Tu

        results = {
            'classic': {'kp': kp_classic, 'ki': ki_classic, 'kd': kd_classic},
            'pessen': {'kp': kp_pessen, 'ki': ki_pessen, 'kd': kd_pessen},
            'no_overshoot': {'kp': kp_no_overshoot, 'ki': ki_no_overshoot, 'kd': kd_no_overshoot},
            'Ku': Ku,
            'Tu': Tu
        }
        
        self.logger.info(f"[AutoTuner] 建议PID参数 (经典): Kp={kp_classic:.4f}, Ki={ki_classic:.4f}, Kd={kd_classic:.4f}")
        self.logger.info(f"[AutoTuner] 建议PID参数 (较少超调): Kp={kp_pessen:.4f}, Ki={ki_pessen:.4f}, Kd={kd_pessen:.4f}")
        self.logger.info(f"[AutoTuner] 建议PID参数 (无超调): Kp={kp_no_overshoot:.4f}, Ki={ki_no_overshoot:.4f}, Kd={kd_no_overshoot:.4f}")

        return results