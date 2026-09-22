"""DM-J6006-2EC 官方 USB2CAN 控制示例。

运行前请确认：
1. 已安装项目依赖：``python -m pip install -r requirements.txt``。
2. USB2CAN 已连接，下面的 ``SERIAL_PORT`` 与设备管理器中的端口一致。
3. 电机供电为 24-48 V，CAN 线连接正确，电机 Slave ID 为 1，Master ID 为 0x11。

从项目根目录运行：

    python examples/DM_Motor_Example.py

示例默认使用位置速度模式，让电机在小范围内往复运动。首次联调时建议先把
``RUN_SECONDS`` 设为较小值，并准备好断电或停止电机的方式。
"""

from __future__ import annotations

import math
import time

import serial

from motor.DM_CAN import Control_Type, DM_Motor_Type, DM_variable, Motor, MotorControl


SERIAL_PORT = "COM5"
SERIAL_BAUDRATE = 921600
MOTOR_SLAVE_ID = 0x01
MOTOR_MASTER_ID = 0x11
RUN_SECONDS = 10.0
CONTROL_PERIOD = 0.01


def main() -> None:
    motor = Motor(
        DM_Motor_Type.DM6006,  # DM-J6006-2EC
        MOTOR_SLAVE_ID,
        MOTOR_MASTER_ID,
    )
    serial_device = serial.Serial(
        SERIAL_PORT,
        SERIAL_BAUDRATE,
        timeout=0.5,
    )
    control = MotorControl(serial_device)
    control.addMotor(motor)

    try:
        print(f"已连接 {SERIAL_PORT}，电机 ID=0x{MOTOR_SLAVE_ID:02X}")

        # 官方函数发送查询后立即读取；USB2CAN 反馈可能稍后到达，因此补一次等待和读取。
        control.refresh_motor_status(motor)
        time.sleep(0.1)
        control.recv()
        print(f"当前反馈位置: {motor.getPosition():.3f} rad")

        # 位置速度模式是 DM-J6006-2EC 支持的模式之一。
        if control.switchControlMode(motor, Control_Type.POS_VEL):
            print("已切换到位置速度模式")
        else:
            print("切换位置速度模式失败，请检查固件版本和电机参数")

        print("减速比:", control.read_motor_param(motor, DM_variable.Gr))
        print("位置范围: ±12.5 rad")
        print("速度范围: ±45 rad/s")
        print("扭矩范围: ±20 N·m")

        # 修改参数前保持失能；完成参数设置后再使能。
        control.enable(motor)
        print("电机已使能，开始控制")

        start = time.monotonic()
        while time.monotonic() - start < RUN_SECONDS:
            elapsed = time.monotonic() - start
            target_position = 2.0 * math.sin(elapsed)
            target_velocity = 2.0

            # p_des 单位为 rad，v_des 单位为 rad/s。
            control.control_Pos_Vel(motor, target_position, target_velocity)
            print(
                f"位置={motor.getPosition(): .3f} rad, "
                f"速度={motor.getVelocity(): .3f} rad/s, "
                f"扭矩={motor.getTorque(): .3f} N·m, "
                f"故障=0x{motor.getError():X}"
            )
            time.sleep(CONTROL_PERIOD)
    except KeyboardInterrupt:
        print("收到停止信号")
    finally:
        # 无论正常结束还是 Ctrl+C，都先让电机失能再关闭串口。
        control.disable(motor)
        serial_device.close()
        print("电机已失能，串口已关闭")


if __name__ == "__main__":
    main()
