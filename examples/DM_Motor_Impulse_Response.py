"""MIT 模式下的电机单位脉冲力矩响应测试。

测试过程：启动后先记录当前位置，在一个控制周期内发送 ``tau=1 N·m``，
然后保持 ``kp=kd=0``、``tau=0`` 继续发送控制帧，直到运行时间结束。
每个周期的反馈会写入 CSV（可通过 ``CSV_OUTPUT = None`` 关闭），并在结束时
报告是否发生位移以及末段速度是否降到阈值以内。

运行前修改本文件顶部的“用户配置”区域，然后执行：

    python examples/DM_Motor_Impulse_Response.py
"""

from __future__ import annotations

import csv
import math
import sys
import time
from pathlib import Path
from typing import Iterable, Mapping

import serial

# 允许直接运行 ``python examples/DM_Motor_Impulse_Response.py``。
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from motor import Control_Type, DM_Motor_Type, Motor, MotorControl


# ============================ 用户配置 ============================

SERIAL_PORT = "COM3"
SERIAL_BAUDRATE = 921600
SERIAL_TIMEOUT = 0.5
SERIAL_WRITE_TIMEOUT = 0.1

SLAVE_ID = 0x01
MASTER_ID = 0x11
MOTOR_TYPE = DM_Motor_Type.DM6006

CONTROL_PERIOD = 0.01  # s；单位脉冲持续一个控制周期
RUN_SECONDS = 5.0  # s；包含脉冲后的自由减速时间
PULSE_TORQUE = 1.0  # N·m；单位脉冲力矩
STATUS_INTERVAL = 0.25  # s；设为 0 关闭周期状态输出
FEEDBACK_TIMEOUT = 0.2

MIN_POSITION_CHANGE = 0.01  # rad；判定“确实转动”的最小位移
MAX_FINAL_SPEED = 0.20  # rad/s；末段速度阈值
SETTLE_WINDOW = 0.5  # s；在最后这段时间检查是否停下
CSV_OUTPUT: str | Path | None = "results/impulse_response.csv"


def validate_config() -> None:
    """在连接硬件前检查时间、脉冲和判定参数。"""
    finite = (
        CONTROL_PERIOD,
        RUN_SECONDS,
        PULSE_TORQUE,
        STATUS_INTERVAL,
        FEEDBACK_TIMEOUT,
        MIN_POSITION_CHANGE,
        MAX_FINAL_SPEED,
        SETTLE_WINDOW,
    )
    if not all(math.isfinite(value) for value in finite):
        raise ValueError("测试参数必须是有限数")
    if CONTROL_PERIOD <= 0:
        raise ValueError("CONTROL_PERIOD 必须大于 0")
    if RUN_SECONDS <= CONTROL_PERIOD:
        raise ValueError("RUN_SECONDS 必须大于一个控制周期")
    if PULSE_TORQUE == 0:
        raise ValueError("PULSE_TORQUE 不能为 0")
    if STATUS_INTERVAL < 0 or MIN_POSITION_CHANGE <= 0 or MAX_FINAL_SPEED < 0:
        raise ValueError("状态间隔和判定阈值无效")
    if SETTLE_WINDOW <= 0 or SETTLE_WINDOW > RUN_SECONDS:
        raise ValueError("SETTLE_WINDOW 必须位于 (0, RUN_SECONDS] 内")
    if FEEDBACK_TIMEOUT <= CONTROL_PERIOD:
        raise ValueError("FEEDBACK_TIMEOUT 必须大于 CONTROL_PERIOD")
    if (
        isinstance(SERIAL_WRITE_TIMEOUT, bool)
        or not isinstance(SERIAL_WRITE_TIMEOUT, (int, float))
        or not math.isfinite(SERIAL_WRITE_TIMEOUT)
        or SERIAL_WRITE_TIMEOUT <= 0
    ):
        raise ValueError("SERIAL_WRITE_TIMEOUT 必须是正有限数")


def analyze_response(
    samples: Iterable[Mapping[str, float]],
    initial_position: float,
    *,
    min_position_change: float = MIN_POSITION_CHANGE,
    max_final_speed: float = MAX_FINAL_SPEED,
    settle_window: float = SETTLE_WINDOW,
) -> dict[str, float | bool]:
    """根据反馈样本判断电机是否转动并在末段基本停下。

    ``samples`` 中每项至少包含 ``time``、``position`` 和 ``velocity``。
    判定只用于实验报告，不会替代 SDK 的故障/反馈超时保护。
    """
    rows = list(samples)
    if not rows:
        raise ValueError("没有可分析的反馈样本")
    last_time = float(rows[-1]["time"])
    final_rows = [
        row
        for row in rows
        if last_time - float(row["time"]) <= settle_window
    ]
    max_displacement = max(
        abs(float(row["position"]) - initial_position) for row in rows
    )
    final_max_speed = max(abs(float(row["velocity"])) for row in final_rows)
    return {
        "max_displacement": max_displacement,
        "final_max_speed": final_max_speed,
        "moved": max_displacement >= min_position_change,
        "stopped": final_max_speed <= max_final_speed,
        "passed": (
            max_displacement >= min_position_change
            and final_max_speed <= max_final_speed
        ),
    }


def write_samples(path: str | Path, samples: Iterable[Mapping[str, float]]) -> None:
    """将反馈样本保存为便于绘图的 CSV 文件。"""
    rows = list(samples)
    if not rows:
        return
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=(
                "time",
                "command_torque",
                "position",
                "velocity",
                "torque",
                "error",
            ),
        )
        writer.writeheader()
        writer.writerows(rows)


def print_status(motor: Motor, elapsed: float, command_torque: float) -> None:
    print(
        f"t={elapsed: .3f} s, cmd_tau={command_torque: .3f} N·m, "
        f"pos={motor.getPosition(): .5f} rad, "
        f"vel={motor.getVelocity(): .5f} rad/s, "
        f"fb_tau={motor.getTorque(): .5f} N·m, "
        f"状态={motor.status_name}(0x{motor.getError():X})"
    )


def main() -> None:
    validate_config()
    motor = Motor(MOTOR_TYPE, SLAVE_ID, MASTER_ID)
    serial_device = serial.Serial(
        SERIAL_PORT,
        SERIAL_BAUDRATE,
        timeout=SERIAL_TIMEOUT,
        write_timeout=SERIAL_WRITE_TIMEOUT,
    )
    control = None
    samples: list[dict[str, float]] = []
    try:
        control = MotorControl(serial_device, feedback_timeout=FEEDBACK_TIMEOUT)
        control.addMotor(motor)
        control.disable(motor)
        control.sync_motor_limits(motor)
        if abs(PULSE_TORQUE) > motor.limits[2]:
            raise ValueError(
                f"PULSE_TORQUE={PULSE_TORQUE} 超出电机实际力矩量程 ±{motor.limits[2]}"
            )
        if not control.switchControlMode(motor, Control_Type.MIT):
            raise RuntimeError("切换 MIT 模式失败")
        control.enable(motor)
        control.refresh_status()
        initial_position = float(motor.getPosition())
        print(
            f"已连接 {SERIAL_PORT}，MIT 脉冲={PULSE_TORQUE:g} N·m，"
            f"持续 {CONTROL_PERIOD:g} s，初始位置={initial_position:.5f} rad"
        )

        start = time.monotonic()
        last_status = start
        pulse_sent = False
        while time.monotonic() - start < RUN_SECONDS:
            cycle_start = time.monotonic()
            command_torque = 0.0 if pulse_sent else PULSE_TORQUE
            # kp=kd=0：q/dq 不产生反馈力矩，tau 是唯一施加的控制输入。
            control.controlMIT(
                motor,
                kp=0.0,
                kd=0.0,
                q=initial_position,
                dq=0.0,
                tau=command_torque,
            )
            pulse_sent = True
            elapsed = time.monotonic() - start
            samples.append(
                {
                    "time": elapsed,
                    "command_torque": command_torque,
                    "position": float(motor.getPosition()),
                    "velocity": float(motor.getVelocity()),
                    "torque": float(motor.getTorque()),
                    "error": float(motor.getError()),
                }
            )
            now = time.monotonic()
            if STATUS_INTERVAL and now - last_status >= STATUS_INTERVAL:
                print_status(motor, elapsed, command_torque)
                last_status = now
            time.sleep(max(0.0, CONTROL_PERIOD - (time.monotonic() - cycle_start)))

        result = analyze_response(initial_position=initial_position, samples=samples)
        if CSV_OUTPUT is not None:
            write_samples(CSV_OUTPUT, samples)
            print(f"反馈已保存到 {CSV_OUTPUT}")
        print(
            f"最大位移={result['max_displacement']:.5f} rad，"
            f"末段最大速度={result['final_max_speed']:.5f} rad/s"
        )
        print("脉冲响应判定：通过" if result["passed"] else "脉冲响应判定：未通过")
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
