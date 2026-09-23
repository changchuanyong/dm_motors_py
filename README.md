# DM-J6006-2EC Python SDK

本项目保留达妙官方完整电机 SDK。底层控制代码位于 `motor/DM_CAN.py`，按模式分类的参数和总线管理位于 `motor/motor_manager.py`，运行示例位于 `examples/`。关键判定的官方出处与「与官方 SDK 的差异」清单见 [docs/官方依据.md](docs/官方依据.md)。

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

## 多电机和模式示例

示例通过 `motor/motor_manager.py` 中的 `MotorManager` 在启动时创建电机。单条 CAN 总线最多配置 16 台，Slave ID 使用 `0x00..0x0F`。运行前直接修改 `examples/DM_Motor_Example.py` 顶部的用户配置区域，然后从项目根目录运行：

```powershell
# 修改顶部配置后直接运行
python examples/DM_Motor_Example.py
```

示例会先读取电机的控制模式、阻尼、减速比和 PMAX/VMAX/TMAX，用读回的实际量程对齐 SDK 的编解码表，再在失能状态下切换模式和修改参数，最后统一使能；按 `Ctrl+C` 或运行时间结束后会统一失能并关闭串口。

启动与控制过程中会核对反馈：使能前每台电机必须已回过反馈，使能后必须收到新反馈且反馈码发生变化，运行中反馈超时（`feedback_timeout`）或反馈码偏离使能稳态码都会停止控制并失能全部电机。`q/P_desired/Pos_des=None` 只在确实收到过反馈时才取当前位置。

反馈中的 `ERR` 是**状态字段**，不能按“非零即故障”判断：`0x08..0x0E` 依次为过压、欠压、过流、MOS 过温、电机线圈过温、通信丢失和过载，`0x00`/`0x01` 为使能状态。程序对实际故障及未知状态停止发送控制并尝试失能全部电机；使能状态取**实测值**作为稳态码，不依赖“哪个值表示已使能”的书面约定（出处见 [docs/官方依据.md](docs/官方依据.md)）。

四种参数类直接对应 SDK。选择参数类即可确定模式，不能传入其他模式的字段：

| 参数类 | SDK 函数 | 字段 |
|---|---|---|
| `MITControl` | `controlMIT` | `kp, kd, q, dq, tau` |
| `PositionVelocityControl` | `control_Pos_Vel` | `P_desired, V_desired` |
| `VelocityControl` | `control_Vel` | `Vel_desired` |
| `PositionForceControl` | `control_pos_force` | `Pos_des, Vel_des, i_des` |

位置单位为 rad，MIT/位置速度/速度模式的速度单位为 rad/s。力位混合的 `Vel_des` 是速度乘 100 的整数，`i_des` 是电流标幺值乘 10000 的整数（0..10000），均按 SDK 原样传递。

每台电机可选择不同的参数类，也可创建同一类的不同参数：

```python
from motor import MotorManager, MotorConfig, MITControl, VelocityControl, RunConfig

configs = (
    MotorConfig(0x01, 0x11, MITControl(kp=20.0, kd=2.0, q=None, dq=0.0, tau=0.0)),
    MotorConfig(0x03, 0x21, VelocityControl(Vel_desired=0.30)),
)
manager = MotorManager.connect("COM5", configs=configs)
try:
    manager.configure()
    manager.refresh_status()
    manager.enable_all()
    manager.run(RunConfig(seconds=5.0))
finally:
    manager.close()
```

电机创建和管理接口统一位于 `motor.motor_manager`。

默认示例只配置实际存在的 `0x01:0x11`。常改项位于文件顶部，四种模式各有一段可执行的参数配置，通过 `MotorConfig.control` 选择使用哪一段。

`q/P_desired/Pos_des=None` 是封装提供的便利值，表示取本轮控制开始时的缓存反馈位置；示例在运行前刷新反馈。填写数字时按绝对位置解释。可选 `SineMotion` 将正弦偏移叠加到这个位置基值；`motion=None` 使用恒定 SDK 目标。速度模式直接使用 `Vel_desired`，不能配置位置轨迹。

## 反馈有效性

- `ManagedMotor` 记录每台电机的 `feedback_count` 与 `last_feedback_at`（`create_motor()` 返回的即是它，仍是 `Motor` 子类）。
- `enable_all()`：使能前要求每台电机已回过反馈；使能后要求收到新反馈、且反馈码相对使能前发生变化（否则报「使能未生效」）；随后把实测反馈码记为**使能稳态码**。
- `run()`：每周期检查反馈年龄（`RunConfig.feedback_timeout`，必须大于 `period`，建议至少 10 倍）与反馈码是否偏离稳态码（运行中掉使能、出现新状态），并拒绝在无反馈时用 `None` 取当前位置。
- `status_snapshot()` 额外提供 `feedback_count` 与 `age`；`MotorManager.link_stats` 提供总线级计数（无主参数应答、被忽略的适配器帧）。

## 位置目标范围

位置目标（`q` / `P_desired` / `Pos_des`）必须落在**该电机实测的** `±PMAX` 内：启动时校验显式目标，运行时每个周期校验 `base + 正弦` 的最终值，超出直接报错（SDK 原样会按 16 位字段**静默截断**）。当前位置距读数窗口边界不足 `POSITION_MARGIN_WARN`（0.5 rad）时会打印回绕告警。

注意 **`±PMAX`（本题 12.5 rad）只是读数窗口，不是机械限位**：电机内部位置是多圈的，超出窗口后反馈值会回绕 `±2×PMAX`。实测过一次：内部位置 +12.8 rad 被报成 −12.20 rad，于是"目标 0"变成了绕远下行 12.8 rad（峰值 28.5 rad/s）。需要依赖绝对坐标或连续同向运动时，先把机构摆到已知姿态并调用 `manager.control.set_zero_position(motor)` 对齐零点。

## 与官方 SDK 的差异

`motor/DM_CAN.py` 保留官方 `u2can/DM_CAN.py` 的全部函数，实质改动只有接收分发：`recv()` 统一拆帧并按报文类别分发、参数类应答按待处理请求归属、`read_motor_param()` 不再提前返回。逐条清单与行为变化见 [docs/官方依据.md](docs/官方依据.md) 第 7 节；升级官方 SDK 时按该清单重新应用。

## 已知缺口

- Python SDK 没有官方的清错命令（`0xFB`，见官方 `dm_motor_drv.c`），固件闩锁故障后无法通过 SDK 清错。
- `MotorControl.send_data_frame` 是类级共享数组，多总线/多线程并行会互相覆盖发送缓冲：本封装只保证单线程、单 `MotorControl` 实例的用法。
- 参数命令后的 0.1 s 保护窗口内，无主的参数类应答会被丢弃（可用 `link_stats` 观测）；窗口外若仍有极晚到达的参数应答，会被当成反馈解析并污染一次缓存位置（概率极低，换来的是反馈帧不会因字节巧合被持续丢弃）。

无硬件测试：`python -m unittest discover -s tests -v`。
