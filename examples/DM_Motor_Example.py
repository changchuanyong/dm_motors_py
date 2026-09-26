"""DM-J6006-2EC 单电机位置速度控制示例。

运行前直接修改本文件顶部的“用户配置”区域，然后执行：

    python examples/DM_Motor_Example.py

示例自行创建串口和电机，每周期直接调用 SDK 的 ``control_Pos_Vel``。
多电机及其他模式见 README。
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import serial

# 允许直接运行 `python examples/DM_Motor_Example.py`，无需先安装本项目。
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from motor import Control_Type, DM_Motor_Type, DM_variable, Motor, MotorControl


# ============================ 用户配置 ============================

# USB2CAN 串口。
SERIAL_PORT = "COM5"
SERIAL_BAUDRATE = 921600
SERIAL_TIMEOUT = 0.5

RUN_SECONDS = 50.0  # 控制运行时间，单位 s
CONTROL_PERIOD = 0.01  # 控制目标周期，单位 s；实际受通信耗时影响
STATUS_INTERVAL = 0.25  # 状态打印间隔，单位 s
FEEDBACK_TIMEOUT = 0.2  # 反馈超时，单位 s；超时即停止控制并尝试失能

# 本示例的目标；也可在下面的 while 循环中根据轨迹或反馈计算每周期目标。
TARGET_POSITION = 10.0  # 绝对位置 rad；None 保持进入循环前的反馈位置
TARGET_VELOCITY = 2.0  # 期望速度 rad/s

SLAVE_ID = 0x01  # 0x00..0x0F
MASTER_ID = 0x11
DAMPING = None  # None 不写 Damp
SAVE_PARAMETERS = False  # True 将模式和可选阻尼配置保存到电机 flash


def print_status(motor: Motor) -> None:
    print(
        f"电机 Slave=0x{motor.SlaveID:02X}: "
        f"pos={motor.getPosition(): .3f} rad, "
        f"vel={motor.getVelocity(): .3f} rad/s, "
        f"tau={motor.getTorque(): .3f} N·m, "
        f"状态={motor.status_name}(0x{motor.getError():X}), "
        f"反馈 {motor.feedback_count} 帧"
    )


def main() -> None:
    if not all(
        math.isfinite(value)
        for value in (
            RUN_SECONDS,
            CONTROL_PERIOD,
            STATUS_INTERVAL,
            FEEDBACK_TIMEOUT,
        )
    ):
        raise ValueError("循环时间参数必须是有限数")
    if RUN_SECONDS < 0 or CONTROL_PERIOD <= 0 or STATUS_INTERVAL < 0:
        raise ValueError("运行时间和打印间隔不能为负，控制周期必须大于 0")
    if FEEDBACK_TIMEOUT <= CONTROL_PERIOD:
        raise ValueError("反馈超时必须大于控制周期")
    if DAMPING is not None and (not math.isfinite(DAMPING) or not 0 < DAMPING <= 100):
        raise ValueError("DAMPING 必须大于 0 且不超过 100")

    motor = Motor(DM_Motor_Type.DM6006, SLAVE_ID, MASTER_ID)
    serial_device = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=SERIAL_TIMEOUT)
    control = None
    try:
        control = MotorControl(serial_device, feedback_timeout=FEEDBACK_TIMEOUT)
        control.addMotor(motor)
        control.disable(motor)
        control.sync_motor_limits(motor)
        # 换模式需同时修改这里的 Control_Type 和下面循环中的控制函数。
        # 当前 POS_VEL 为位置速度；其他常用模式为 MIT、VEL、Torque_Pos。
        # 保持“失能 -> 切换并确认模式 -> 使能”的顺序，模式不匹配时 SDK 会报错。
        if not control.switchControlMode(motor, Control_Type.POS_VEL):
            raise RuntimeError("切换位置速度模式失败")
        if DAMPING is not None:
            if not control.change_motor_param(motor, DM_variable.Damp, DAMPING):
                raise RuntimeError("修改 Damp 失败")
        if SAVE_PARAMETERS:
            control.save_motor_param(motor)
        control.enable(motor)
        print(f"已连接 {SERIAL_PORT}，PMAX/VMAX/TMAX={motor.limits}")
        print_status(motor)
        initial_position = float(motor.getPosition())

        start = time.monotonic()
        last_status = start
        while time.monotonic() - start < RUN_SECONDS:
            cycle_start = time.monotonic()
            # 在这里计算本周期目标；读取反馈可用 motor.getPosition()/getVelocity()。
            target_position = (
                initial_position if TARGET_POSITION is None else TARGET_POSITION
            )
            # 当前为 Control_Type.POS_VEL：位置 P_desired 单位 rad，速度 V_desired 单位 rad/s。
            control.control_Pos_Vel(
                motor,
                P_desired=target_position,
                V_desired=TARGET_VELOCITY,
            )

            # 换用以下任一模式时，用对应调用替换上面整段 control_Pos_Vel，并修改启动模式。
            # 同一台电机每周期只使用与已配置模式对应的调用。
            #
            # MIT：启动选择 Control_Type.MIT。
            # kp: 0..500，kd: 0..5；q: rad，dq: rad/s，tau: 前馈力矩 N·m。
            # control.controlMIT(motor, kp=10.0, kd=1.0, q=target_position, dq=0.0, tau=0.0)
            #
            # 速度：启动选择 Control_Type.VEL；Vel_desired 单位 rad/s，可为负，不使用位置目标。
            # control.control_Vel(motor, Vel_desired=TARGET_VELOCITY)
            #
            # 力位混合：启动选择 Control_Type.Torque_Pos；Pos_des 单位 rad。
            # Vel_des 是速度乘 100 的整数，100 = 1 rad/s；
            # i_des 是最大电流比例乘 10000 的整数，3000 = 30%，不是 mA。
            # control.control_pos_force(motor, Pos_des=target_position, Vel_des=100, i_des=3000)

            now = time.monotonic()
            if now - last_status >= STATUS_INTERVAL:
                print_status(motor)
                last_status = now
            time.sleep(max(0.0, CONTROL_PERIOD - (time.monotonic() - cycle_start)))
    except KeyboardInterrupt:
        print("收到停止信号")
    finally:
        try:
            if control is not None:
                try:
                    control.disable(motor)
                except Exception as exc:
                    print(f"失能发送失败：{exc}", file=sys.stderr)
        finally:
            serial_device.close()
        print("已执行退出清理，串口已关闭")


if __name__ == "__main__":
    main()
