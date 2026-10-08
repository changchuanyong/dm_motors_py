"""达妙电机速度模式正弦速度跟随基线测试。"""

from __future__ import annotations

import csv
import math
import sys
import time
from pathlib import Path

import serial

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from motor import Control_Type, DM_Motor_Type, Motor, MotorControl


SERIAL_PORT = "COM3"
SERIAL_BAUDRATE = 921600
SERIAL_TIMEOUT = 0.5
SERIAL_WRITE_TIMEOUT = 0.1

SLAVE_ID = 0x01
MASTER_ID = 0x11
MOTOR_TYPE = DM_Motor_Type.DM6006

CONTROL_PERIOD = 0.01
RUN_SECONDS = 10.0
VELOCITY_AMPLITUDE = 1.0  # rad/s
VELOCITY_FREQUENCY = 0.2  # Hz
FEEDBACK_TIMEOUT = 0.2
STATUS_INTERVAL = 0.5
CSV_OUTPUT = "results/sine_velocity_com3.csv"


def validate_config() -> None:
    values = (
        CONTROL_PERIOD,
        RUN_SECONDS,
        VELOCITY_AMPLITUDE,
        VELOCITY_FREQUENCY,
        FEEDBACK_TIMEOUT,
        STATUS_INTERVAL,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("测试参数必须是有限数")
    if CONTROL_PERIOD <= 0 or RUN_SECONDS <= CONTROL_PERIOD:
        raise ValueError("控制周期和测试时长无效")
    if VELOCITY_AMPLITUDE <= 0 or VELOCITY_FREQUENCY <= 0:
        raise ValueError("速度振幅和频率必须大于 0")
    if FEEDBACK_TIMEOUT <= CONTROL_PERIOD:
        raise ValueError("反馈超时必须大于控制周期")


def summarize(samples: list[dict[str, float]]) -> dict[str, float]:
    if not samples:
        raise ValueError("没有反馈样本")
    errors = [row["error"] for row in samples]
    torques = [row["torque"] for row in samples]
    rmse = math.sqrt(sum(error * error for error in errors) / len(errors))
    torque_mean = sum(torques) / len(torques)
    torque_std = math.sqrt(
        sum((torque - torque_mean) ** 2 for torque in torques) / len(torques)
    )
    return {
        "velocity_rmse": rmse,
        "velocity_mae": sum(abs(error) for error in errors) / len(errors),
        "velocity_max_error": max(abs(error) for error in errors),
        "torque_mean": torque_mean,
        "torque_std": torque_std,
        "torque_peak_to_peak": max(torques) - min(torques),
    }


def write_csv(samples: list[dict[str, float]]) -> None:
    Path(CSV_OUTPUT).parent.mkdir(parents=True, exist_ok=True)
    with Path(CSV_OUTPUT).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=("time", "target_velocity", "velocity", "error", "torque", "status"),
        )
        writer.writeheader()
        writer.writerows(samples)


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
        if VELOCITY_AMPLITUDE > motor.limits[1]:
            raise ValueError(
                f"速度振幅 {VELOCITY_AMPLITUDE} 超出实际速度量程 ±{motor.limits[1]}"
            )
        if not control.switchControlMode(motor, Control_Type.VEL):
            raise RuntimeError("切换速度模式失败")
        control.enable(motor)
        control.refresh_status()
        start = time.monotonic()
        last_status = start
        while time.monotonic() - start < RUN_SECONDS:
            cycle_start = time.monotonic()
            elapsed = cycle_start - start
            target_velocity = VELOCITY_AMPLITUDE * math.sin(
                2.0 * math.pi * VELOCITY_FREQUENCY * elapsed
            )
            control.control_Vel(motor, target_velocity)
            actual_velocity = float(motor.getVelocity())
            samples.append(
                {
                    "time": elapsed,
                    "target_velocity": target_velocity,
                    "velocity": actual_velocity,
                    "error": actual_velocity - target_velocity,
                    "torque": float(motor.getTorque()),
                    "status": float(motor.getError()),
                }
            )
            now = time.monotonic()
            if now - last_status >= STATUS_INTERVAL:
                print(
                    f"t={elapsed: .2f}s target={target_velocity: .3f} "
                    f"actual={actual_velocity: .3f} "
                    f"tau={motor.getTorque(): .3f} status=0x{motor.getError():X}"
                )
                last_status = now
            time.sleep(max(0.0, CONTROL_PERIOD - (time.monotonic() - cycle_start)))

        result = summarize(samples)
        write_csv(samples)
        print(
            f"RMSE={result['velocity_rmse']:.5f} rad/s, "
            f"MAE={result['velocity_mae']:.5f} rad/s, "
            f"最大误差={result['velocity_max_error']:.5f} rad/s"
        )
        print(
            f"力矩均值={result['torque_mean']:.5f} N·m, "
            f"标准差={result['torque_std']:.5f} N·m, "
            f"峰峰值={result['torque_peak_to_peak']:.5f} N·m"
        )
        print(f"数据已保存到 {CSV_OUTPUT}")
    except KeyboardInterrupt:
        print("收到停止信号")
    finally:
        try:
            if control is not None:
                try:
                    control.control_Vel(motor, 0.0)
                except Exception:
                    pass
                try:
                    control.disable(motor)
                except Exception as exc:
                    print(f"失能发送失败：{exc}", file=sys.stderr)
        finally:
            serial_device.close()
        print("已执行退出清理，串口已关闭")


if __name__ == "__main__":
    main()
