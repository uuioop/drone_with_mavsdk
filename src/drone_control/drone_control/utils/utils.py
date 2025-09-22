#!/usr/bin/env python3
"""
几何计算工具模块
"""

import math
import numpy as np
import asyncio

def calculate_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    计算两个GPS坐标点之间的距离（米）
    
    Args:
        lat1: 第一个点的纬度
        lon1: 第一个点的经度
        lat2: 第二个点的纬度
        lon2: 第二个点的经度
        
    Returns:
        两点之间的距离（米）
    """
    R = 6371000  # 地球半径（米）
    
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad
    
    a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    return R * c

def enu_to_ned(x: float, y: float, z: float) -> tuple:
    """
    ENU坐标系转换为NED坐标系
    
    Args:
        x: ENU坐标系中的x坐标
        y: ENU坐标系中的y坐标
        z: ENU坐标系中的z坐标
        
    Returns:
        (x_ned, y_ned, z_ned): NED坐标系中的坐标
    """
    x_ned = y
    y_ned = x
    z_ned = -z
    return x_ned, y_ned, z_ned

def ned_to_enu(x_ned: float, y_ned: float, z_ned: float) -> tuple:
    """
    NED坐标系转换为ENU坐标系
    
    Args:
        x_ned: NED坐标系中的x坐标
        y_ned: NED坐标系中的y坐标
        z_ned: NED坐标系中的z坐标
        
    Returns:
        (x, y, z): ENU坐标系中的坐标
    """
    x = y_ned
    y = x_ned
    z = -z_ned
    return x, y, z

def calculate_gps_offset(lat: float, lon: float, x_offset: float, y_offset: float) -> tuple:
    """
    根据当前位置和偏移量计算目标GPS坐标
    
    Args:
        lat: 当前纬度
        lon: 当前经度
        x_offset: 东向偏移（米）
        y_offset: 北向偏移（米）
        
    Returns:
        (target_lat, target_lon): 目标GPS坐标
    """
    # 使用更精确的地球模型
    # 地球椭球体参数（WGS84）
    a = 6378137.0  # 长半轴（米）
    b = 6356752.314245  # 短半轴（米）
    e2 = 1 - (b * b) / (a * a)  # 第一偏心率平方
    
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    
    # 计算子午线曲率半径
    N = a / math.sqrt(1 - e2 * math.sin(lat_rad) ** 2)
    
    # 计算纬度偏移
    lat_offset = y_offset / (N * (1 - e2 * math.sin(lat_rad) ** 2))
    target_lat = lat + math.degrees(lat_offset)
    
    # 计算经度偏移
    lon_offset = x_offset / (N * math.cos(lat_rad))
    target_lon = lon + math.degrees(lon_offset)
    
    return target_lat, target_lon

def calculate_heading_from_points(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    计算两点之间的航向角
    
    Args:
        lat1: 起始点纬度
        lon1: 起始点经度
        lat2: 目标点纬度
        lon2: 目标点经度
        
    Returns:
        航向角（度，0-360）
    """
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    
    dlon = lon2_rad - lon1_rad
    
    y = math.sin(dlon) * math.cos(lat2_rad)
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
    
    heading_rad = math.atan2(y, x)
    heading_deg = math.degrees(heading_rad)
    
    # 转换为0-360度
    if heading_deg < 0:
        heading_deg += 360
        
    return heading_deg

def normalize_angle(angle: float) -> float:
    """
    将角度标准化到-180到180度范围
    
    Args:
        angle: 输入角度（度）
        
    Returns:
        标准化后的角度（度）
    """
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle 

def calculate_relative_position_target(current_lat: float, current_lon: float, current_alt: float, 
                                    relative_n: float, relative_e: float, relative_d: float) -> tuple:
    """
    根据当前位置和NED相对偏移计算目标位置
    
    Args:
        current_lat: 当前纬度（度）
        current_lon: 当前经度（度）
        current_alt: 当前高度（米）
        relative_n: 北向相对偏移（米），正值向北
        relative_e: 东向相对偏移（米），正值向东
        relative_d: 下向相对偏移（米），正值向下（下降），负值向上（上升）
        
    Returns:
        (target_lat, target_lon, target_alt): 目标位置的经纬度和高度
    """
    # 使用WGS84椭球体模型进行精确计算
    # 地球椭球体参数（WGS84）
    a = 6378137.0  # 长半轴（米）
    b = 6356752.314245  # 短半轴（米）
    e2 = 1 - (b * b) / (a * a)  # 第一偏心率平方
    
    lat_rad = math.radians(current_lat)
    lon_rad = math.radians(current_lon)
    
    # 计算子午线曲率半径
    N = a / math.sqrt(1 - e2 * math.sin(lat_rad) ** 2)
    
    # 计算纬度偏移（北向偏移）
    lat_offset_rad = relative_n / (N * (1 - e2 * math.sin(lat_rad) ** 2))
    target_lat = current_lat + math.degrees(lat_offset_rad)
    
    # 计算经度偏移（东向偏移）
    lon_offset_rad = relative_e / (N * math.cos(lat_rad))
    target_lon = current_lon + math.degrees(lon_offset_rad)
    
    # 计算高度偏移（注意：relative_d为正值表示下降，负值表示上升）
    target_alt = current_alt - relative_d
    
    return target_lat, target_lon, target_alt

def validate_gps_coordinates(lat: float, lon: float, alt: float) -> bool:
    """
    验证GPS坐标的有效性
    
    Args:
        lat: 纬度（度）
        lon: 经度（度）
        alt: 高度（米）
        
    Returns:
        bool: 坐标是否有效
    """
    # 检查纬度范围
    if not (-90 <= lat <= 90):
        return False
    
    # 检查经度范围
    if not (-180 <= lon <= 180):
        return False
    
    # 检查高度（允许负值，但通常不会太低）
    if alt < -1000:  # 允许-1000米以下
        return False
    
    return True 

async def observe_is_in_air(drone, logger):
    """ 
    监控无人机是否在空中，着陆后立即上锁
    
    Args:
        drone: 无人机对象
        logger: 日志记录器
    """
    was_in_air = False
    await drone.action.land()
    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air
            if not was_in_air:  # 刚起飞
                logger.info("无人机已起飞")

        if was_in_air and not is_in_air:
            logger.info("无人机已降落，准备上锁")
            
            # 等待一段时间确保无人机完全稳定
            await asyncio.sleep(2)    
            try:
                # 尝试上锁
                await drone.action.disarm()
                logger.info("无人机上锁成功")
            except Exception as e:
                logger.error(f"上锁失败: {e}")
            return

def calculate_ned_from_origin(origin_lat: float, origin_lon: float, origin_alt: float,
                             current_lat: float, current_lon: float, current_alt: float) -> tuple:
    """
    计算当前位置相对于原点的NED坐标系坐标
    
    Args:
        origin_lat: 原点纬度（度）
        origin_lon: 原点经度（度）
        origin_alt: 原点高度（米）
        current_lat: 当前位置纬度（度）
        current_lon: 当前位置经度（度）
        current_alt: 当前位置高度（米）
        
    Returns:
        (n, e, d): NED坐标系中的坐标（米）
            n: 北向坐标（正值向北）
            e: 东向坐标（正值向东）
            d: 下向坐标（正值向下，即高度降低）
    """
    # 使用WGS84椭球体模型进行精确计算
    # 地球椭球体参数（WGS84）
    a = 6378137.0  # 长半轴（米）
    b = 6356752.314245  # 短半轴（米）
    e2 = 1 - (b * b) / (a * a)  # 第一偏心率平方
    
    # 转换为弧度
    origin_lat_rad = math.radians(origin_lat)
    origin_lon_rad = math.radians(origin_lon)
    current_lat_rad = math.radians(current_lat)
    current_lon_rad = math.radians(current_lon)
    
    # 计算子午线曲率半径（在原点位置）
    N = a / math.sqrt(1 - e2 * math.sin(origin_lat_rad) ** 2)
    
    # 计算纬度差（北向距离）
    lat_diff_rad = current_lat_rad - origin_lat_rad
    n = lat_diff_rad * N * (1 - e2 * math.sin(origin_lat_rad) ** 2)
    
    # 计算经度差（东向距离）
    lon_diff_rad = current_lon_rad - origin_lon_rad
    e = lon_diff_rad * N * math.cos(origin_lat_rad)
    
    # 计算高度差（下向距离，注意：高度降低为正值）
    d = origin_alt - current_alt
    
    return n, e, d
