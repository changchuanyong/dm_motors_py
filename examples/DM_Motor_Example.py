"""DM-J6006-2EC 多电机控制示例。

运行前直接修改本文件顶部的“用户配置”区域，然后执行：

    python examples/DM_Motor_Example.py

示例只负责读取配置和调用 ``MotorManager``，电机创建、模式切换、
参数修改、控制循环和串口清理都由 ``motor.motor_manager`` 负责。
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Dict, List

# 允许直接运行 `python examples/DM_Motor_Example.py`，无需先安装本项目。
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from motor import (
    MITControl,
    MotorConfig,
    MotorManager,
    PositionForceControl,
    PositionVelocityControl,
    RunConfig,
    SineMotion,
    VelocityControl,
)


# ============================ 用户配置 ============================

# USB2CAN 串口。
SERIAL_PORT = "COM5"
SERIAL_BAUDRATE = 921600
SERIAL_TIMEOUT = 0.5

RUN_SECONDS = 50.0       # 控制运行时间，单位 s
CONTROL_PERIOD = 0.01   # 一轮所有电机的目标周期，单位 s；实际受通信耗时影响
STATUS_INTERVAL = 0.25  # 状态打印间隔，单位 s
FEEDBACK_TIMEOUT = 0.2  # 反馈超时，单位 s；超时即停止控制并失能全部

# MIT：controlMIT
MIT_PARAMS = MITControl(
    kp=10,   # 0..500
    kd=1.0,    # 0..5
    q=0,    # 绝对位置 rad；None 取本轮开始时的反馈位置
    dq=0.1,    # 期望速度 rad/s
    tau=0,   # 前馈力矩 N·m
)

# 位置速度：control_Pos_Vel
POSITION_VELOCITY_PARAMS = PositionVelocityControl(
    P_desired=10,  # 绝对位置 rad；None 取本轮开始时的反馈位置
    V_desired=2,  # 期望速度 rad/s
)

# 速度：control_Vel
VELOCITY_PARAMS = VelocityControl(
    Vel_desired=1  # rad/s，可为负
) 

# 力位混合：control_pos_force
POSITION_FORCE_PARAMS = PositionForceControl(
    Pos_des=None,  # 绝对位置 rad；None 取本轮开始时的反馈位置
    Vel_des=100,   # SDK 原始整数：100 = 1 rad/s
    i_des=3000,    # SDK 原始整数：3000 = 最大电流的 30%，不是 mA
)

# 只创建这里列出的电机；ID 可以不连续，最多 16 台。
MOTOR_CONFIGS = (
    MotorConfig(
        slave_id=0x01,       # 0x00..0x0F
        master_id=0x11,
        control=MIT_PARAMS,  # 选择上面的参数类，也可在此新建独立参数对象
        #motion=SineMotion(amplitude=0.10, frequency=0.5, phase=0.0),
        motion=None,
        # motion=None 时按 SDK 目标原样控制；速度模式必须用 None
        damping=None,       # None 不写 Damp
        save_parameters=False,
    ),
)


def print_parameters(parameters: Dict[int, Dict[str, Any]]) -> None:
    for slave_id, values in parameters.items():
        print(f"电机 Slave=0x{slave_id:02X} 参数:")
        for name, value in values.items():
            print(f"  {name}: {value}")


def print_status(statuses: List[Dict[str, Any]]) -> None:
    for status in statuses:
        age = status["age"]
        age_text = "从未收到" if age is None else f"{age * 1000:.0f} ms 前"
        print(
            f"电机 Slave=0x{status['slave_id']:02X}: "
            f"pos={status['position']: .3f} rad, "
            f"vel={status['velocity']: .3f} rad/s, "
            f"tau={status['torque']: .3f} N·m, "
            f"状态={status['status_name']}(0x{status['error']:X}), "
            f"反馈 {status['feedback_count']} 帧/最近 {age_text}"
        )


def main() -> None:
    run_config = RunConfig(
        seconds=RUN_SECONDS,
        period=CONTROL_PERIOD,
        status_interval=STATUS_INTERVAL,
        feedback_timeout=FEEDBACK_TIMEOUT,
    )
    manager = MotorManager.connect(
        SERIAL_PORT,
        configs=MOTOR_CONFIGS,
        baudrate=SERIAL_BAUDRATE,
        timeout=SERIAL_TIMEOUT,
    )
    try:
        print(f"已连接 {SERIAL_PORT}，共 {manager.motor_count} 个 DM-J6006-2EC")
        manager.refresh_status()
        print_parameters(manager.read_parameters())

        manager.configure()
        manager.refresh_status()
        manager.enable_all()
        print_status(manager.status_snapshot())  # 使能后的实测反馈码，确认电机真的已使能
        manager.run(run_config, on_status=print_status)
    except KeyboardInterrupt:
        print("收到停止信号")
    finally:
        manager.close()
        print("所有电机已失能，串口已关闭")


if __name__ == "__main__":
    main()
