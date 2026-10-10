"""Prius 几何与运动学参数。

数值来自 prius_description/urdf/prius.urdf：
  chassis 坐标系下前转向节 y=-1.41、后轮 y=+1.45  -> 轴距 2.86
  base_link -> chassis 平移 (1.45, 0, 0)、绕 Z 转 1.57
  代入后 base_link 恰好落在后轴中心，且 +X 为车头方向。
因此自行车模型直接以 base_link 为参考点，无需额外偏移。
"""

import math

WHEELBASE = 2.86
MAX_STEER = 0.6458
WIDTH = 1.80
LENGTH = 4.70
REAR_OVERHANG = 0.95
FRONT_OVERHANG = 0.89

MIN_TURN_RADIUS = WHEELBASE / math.tan(MAX_STEER)

# 后轴中心到车身几何中心的距离，用于「车位中心 <-> 停车目标位姿」换算。
REAR_AXLE_TO_CENTER = LENGTH / 2.0 - REAR_OVERHANG


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def steer_from_curvature(curvature):
    """曲率 -> 前轮转角，并限幅到机械极限。"""
    steer = math.atan(curvature * WHEELBASE)
    return max(-MAX_STEER, min(MAX_STEER, steer))


def goal_from_spot_center(cx, cy, yaw):
    """车位中心 -> 停好后的后轴中心位姿。

    车身在车位内纵向居中，后轴中心因此落在中心点后方 REAR_AXLE_TO_CENTER 处。
    """
    return (
        cx - REAR_AXLE_TO_CENTER * math.cos(yaw),
        cy - REAR_AXLE_TO_CENTER * math.sin(yaw),
        yaw,
    )


def footprint(x, y, yaw, margin=0.0):
    """车辆轮廓四角，输入为后轴中心位姿，返回闭合折线（5 个点）。"""
    front = FRONT_OVERHANG + WHEELBASE + margin
    rear = REAR_OVERHANG + margin
    half = WIDTH / 2.0 + margin
    corners = [
        (front, half),
        (front, -half),
        (-rear, -half),
        (-rear, half),
        (front, half),
    ]
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    return [
        (x + lx * cos_y - ly * sin_y, y + lx * sin_y + ly * cos_y)
        for lx, ly in corners
    ]
