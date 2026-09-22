# DM-J6006-2EC Python SDK

本项目直接使用达妙官方完整电机 SDK，不使用额外的简化控制层。项目说明、许可证和依赖位于根目录，电机代码位于 `motor/`，运行示例位于 `examples/`。

官方 SDK 保留了 `DM_CAN.py` 的全部函数，包括：

- MIT、位置速度、速度、力位混合和 CSP 控制
- 使能、失能、设置零点和状态刷新
- 电机参数读取、修改、模式切换和保存
- 多电机管理及反馈状态解析

## 安装

```powershell
python -m pip install -e .
# 或
python -m pip install -r requirements.txt
```

USB2CAN 串口默认使用 921600 波特率；电机 CAN 总线使用 1 Mbps。

## DM-J6006-2EC 示例

```python
import serial
from motor import DM_Motor_Type, Motor, MotorControl

motor = Motor(DM_Motor_Type.DM6006, 0x01, 0x11)
serial_device = serial.Serial("COM8", 921600, timeout=0.5)
control = MotorControl(serial_device)
control.addMotor(motor)

try:
    control.enable(motor)
    control.controlMIT(motor, kp=20, kd=2, q=0, dq=0, tau=0)
    print(motor.getPosition(), motor.getVelocity(), motor.getTorque())
finally:
    control.disable(motor)
    serial_device.close()
```

使用说明和接口注释见 [motor/DM_CAN.py](motor/DM_CAN.py)。完整的 DM-J6006-2EC 运行示例见 [examples/DM_Motor_Example.py](examples/DM_Motor_Example.py)。许可证见 [LICENSE](LICENSE)。
