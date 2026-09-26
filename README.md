# DM-J6006-2EC Python SDK

本项目基于达妙官方 Python SDK，底层代码位于 `motor/DM_CAN.py`。直接使用 `Motor` 和 `MotorControl`：SDK 负责通信、启动确认和指令检查，示例负责串口、控制循环、目标计算和退出清理。关键判定及与官方 SDK 的差异见 [docs/官方依据.md](docs/官方依据.md)。

保留官方控制函数名，包括 MIT、位置速度、速度、力位混合、CSP，以及使能、失能、零点和参数操作；初始化要求及失败行为有变化，见下文。

## 安装

```powershell
python -m pip install -e .
# 或
python -m pip install -r requirements.txt
```

USB2CAN 串口默认使用 921600 波特率；电机 CAN 总线使用 1 Mbps。

## 直接调用 SDK

```python
import serial
from motor import Control_Type, DM_Motor_Type, Motor, MotorControl

motor = Motor(DM_Motor_Type.DM6006, 0x01, 0x11)
serial_device = serial.Serial("COM5", 921600, timeout=0.5)
control = None
try:
    control = MotorControl(serial_device, feedback_timeout=0.2)
    control.addMotor(motor)
    control.disable(motor)
    control.sync_motor_limits(motor)
    if not control.switchControlMode(motor, Control_Type.POS_VEL):
        raise RuntimeError("切换模式失败")
    control.enable(motor)

    # 每个控制周期直接传入最新数值；SDK 自动检查目标及反馈状态。
    control.control_Pos_Vel(motor, P_desired=10.0, V_desired=2.0)
    print(motor.getPosition(), motor.getVelocity(), motor.getTorque())
finally:
    try:
        if control is not None:
            try:
                control.disable(motor)
            except Exception as exc:
                print(f"失能发送失败：{exc}")
    finally:
        serial_device.close()
```

使用说明和接口注释见 [motor/DM_CAN.py](motor/DM_CAN.py)。完整控制循环见 [examples/DM_Motor_Example.py](examples/DM_Motor_Example.py)。许可证见 [LICENSE](LICENSE)。

## 逐周期控制示例

运行前修改 `examples/DM_Motor_Example.py` 顶部的用户配置区域，再从项目根目录运行：

```powershell
python examples/DM_Motor_Example.py
```

默认串口为 `COM5`，一台 `0x01:0x11` 电机使用位置速度模式，运行 50 s、目标周期 10 ms。`TARGET_POSITION=10.0` 是绝对位置 10 rad，`TARGET_VELOCITY=2.0` 是速度参数 2 rad/s。`TARGET_POSITION=None` 时，示例在使能确认后捕获一次反馈位置并保持该目标。轨迹或传感器反馈计算直接写在循环内，再将数值传给 `control_Pos_Vel()`。

启动顺序为：注册电机 → 失能 → 读取实际 PMAX/VMAX/TMAX 并对齐 SDK 量程 → 切换并确认模式 → 使能并确认新反馈。可选的 `DAMPING` 修改和 `SAVE_PARAMETERS` 保存放在使能之前；保存会写电机 flash，不应每个控制周期执行。

运行结束、异常或 `Ctrl+C` 都通过 `finally` 尝试失能并关闭串口。关闭串口不等于电机已确认失能；示例的清理只保证尝试发送失能命令，发送失败会明确打印。

四种常用模式直接对应以下 SDK 接口，不需要配置类或指令对象：

| 启动模式 | 每周期接口 | 参数 |
|---|---|---|
| `Control_Type.MIT` | `controlMIT` | `motor, kp, kd, q, dq, tau` |
| `Control_Type.POS_VEL` | `control_Pos_Vel` | `motor, P_desired, V_desired` |
| `Control_Type.VEL` | `control_Vel` | `motor, Vel_desired` |
| `Control_Type.Torque_Pos` | `control_pos_force` | `motor, Pos_des, Vel_des, i_des` |

位置单位为 rad，MIT/位置速度/速度模式的速度单位为 rad/s。力位混合的 `Vel_des` 是速度乘 100 的整数，`i_des` 是电流标幺值乘 10000 的整数（0..10000）。位置参数必须是有限数；`None` 仅是本示例的起始位置约定，不能直接传给 SDK。

这里的“逐周期控制”表示调用方每次传入最新目标，不保证硬实时。`recv()` 只处理串口当前已到达的字节，发送后立刻读取不保证已收到这次命令的应答。`getPosition()` 等读取的是缓存；SDK 按最近反馈时间检查超时。`control_Pos_Vel()` 内部有 1 ms 等待，实际周期还受通信、打印和系统调度影响。

## 多电机

单条 CAN 总线最多 16 台，Slave ID 使用 `0x00..0x0F`。同一控制器中的 Slave ID、Master ID 映射不得冲突；每台电机可以使用不同模式。启动时先全部注册，再逐台配置、使能：

```python
import time
import serial
from motor import Control_Type, DM_Motor_Type, Motor, MotorControl

motors = (
    Motor(DM_Motor_Type.DM6006, 0x01, 0x11),
    Motor(DM_Motor_Type.DM6006, 0x03, 0x21),
)
modes = (Control_Type.MIT, Control_Type.VEL)
serial_device = serial.Serial("COM5", 921600, timeout=0.5)
control = None
try:
    control = MotorControl(serial_device, feedback_timeout=0.2)
    for motor in motors:
        control.addMotor(motor)
    for motor, mode in zip(motors, modes):
        control.disable(motor)
        control.sync_motor_limits(motor)
        if not control.switchControlMode(motor, mode):
            raise RuntimeError(f"电机 0x{motor.SlaveID:02X} 切换模式失败")
        control.enable(motor)

    # 逐台启动会耗时；统一刷新，避免第一台的反馈在进入循环前已经过期。
    control.refresh_status()
    control.check_status()
    mit_motor, velocity_motor = motors
    target_position = mit_motor.getPosition()
    start = time.monotonic()
    while time.monotonic() - start < 5.0:
        cycle_start = time.monotonic()
        # 在这里计算每台电机本周期的目标。
        control.controlMIT(mit_motor, kp=20.0, kd=2.0, q=target_position, dq=0.0, tau=0.0)
        control.control_Vel(velocity_motor, Vel_desired=0.30)
        time.sleep(max(0.0, 0.01 - (time.monotonic() - cycle_start)))
finally:
    try:
        if control is not None:
            for motor in motors:
                try:
                    control.disable(motor)
                except Exception as exc:
                    print(f"电机 0x{motor.SlaveID:02X} 失能发送失败：{exc}")
    finally:
        serial_device.close()
```

清理必须逐台尝试失能，并在外层 `finally` 关闭串口，不能因为某台失能失败而跳过其他电机。单个控制接口自动检查当前电机；需要在某个阶段检查全部电机时，调用 `control.check_status()`。

## 启动和反馈检查

- `Motor.feedback_count`、`last_feedback_at` 只在收到有效反馈时更新；`status_name` 提供状态名称。
- `sync_motor_limits(motor)` 从电机读取 PMAX/VMAX/TMAX，记录在 `motor.limits` 并对齐当前控制器的编解码表。同型号电机读到不同量程时拒绝继续。尝试写量程参数（即使写入失败），或用 `change_limit_param()` 改动对应型号的表，都会使相关 `limits` 失效；必须重新同步后才能控制。
- `read_motor_param()`、`change_motor_param()` 和 `switchControlMode()` 先消费旧帧、清除目标缓存，再等待本次应答，避免旧值冒充成功。
- `switchControlMode(motor, mode)` 只有确认应答后才登记 `NowControlMode`；必须检查返回值。新建电机默认没有已确认模式。
- `enable(motor)` 先查询新状态，再发送使能，并要求新的 `0x01` 反馈；`enable_old()` 也要求本次新反馈，失败会尝试失能。只有成功的使能流程才设置 `motor.isEnable`，收到非 `0x01` 状态或调用 `disable()` 会清除它；单个 `0x01` 回包不能代替使能确认。
- `refresh_status(wait=0.1)` 批量查询全部已注册电机，等待后任一电机没有新反馈都会抛出 `RuntimeError`。
- `check_status(motor=None)` 先收取到达的反馈，再按构造器的 `feedback_timeout` 检查一个或全部已注册电机。超时、失能、故障或未知状态会报错。
- 各控制接口在发送前检查本机状态、已确认模式、已同步量程及指令参数，发送后再次收取并检查本机反馈；不需要在示例里重复堆叠同样检查。
- 任何检查失败都会阻止继续运行并抛出异常；调用方必须保留 `finally` 清理。SDK 不接管串口生命周期或应用循环。

反馈 `ERR` 是**状态字段**，不能按“非零即故障”判断。`0x08..0x0E` 依次为过压、欠压、过流、MOS 过温、电机线圈过温、通信丢失和过载。本项目按已有 DM-J6006-2EC 实机记录使用 `0x00` 表示失能、`0x01` 表示使能；故障及未知状态均停止控制。适用依据见 [docs/官方依据.md](docs/官方依据.md)。

## 位置目标范围

`q` / `P_desired` / `Pos_des` 必须落在**该电机实测的** `±PMAX` 内，SDK 检查每周期最终位置，越界直接报错。MIT 的速度和力矩仍沿用官方编码的饱和处理。

**`±PMAX`（已有实机记录为 12.5 rad）只是读数窗口，不是机械限位**：电机内部位置是多圈的，超出窗口后反馈值会回绕 `±2×PMAX`。曾实测内部位置 +12.8 rad 被报成 −12.20 rad，使“目标 0”成为绕远下行 12.8 rad（峰值 28.5 rad/s）。需要绝对坐标或连续同向运动时，先把机构摆到已知姿态，再显式调用 `control.set_zero_position(motor)` 对齐零点。

## 与官方 SDK 的差异

`motor/DM_CAN.py` 保留官方 `u2can/DM_CAN.py` 的函数名，修正统一接收、完整电机 ID 与请求匹配、参数读取重试及噪声后的残帧缓存（最多 15 字节），并将必要的启动和控制检查放入 SDK。逐条清单、迁移说明与行为变化见 [docs/官方依据.md](docs/官方依据.md) 第 7 节；升级官方 SDK 时按该清单重新应用。

## 已知缺口

- Python SDK 没有官方的清错命令（`0xFB`，见官方 `dm_motor_drv.c`），固件闩锁故障后无法通过 SDK 清错。
- `Limit_Param` 已按控制器实例复制，但 `MotorControl.send_data_frame` 仍是类级共享数组，多总线/多线程并行会互相覆盖发送缓冲；当前用法限于单线程、单 `MotorControl` 实例。
- 已注册电机的参数类应答按完整 Slave ID、CAN ID、操作码、RID 和请求截止时间匹配。迟到或不匹配的应答均丢弃，包含 RID 50..55、80/81，不更新位置及反馈时间戳。
- 失能状态下，极少数位置/速度编码可能与参数应答逐字节重合，协议本身没有独立类型标记。这类歧义帧保守按参数类处理；若因此收不到可确认的状态，`refresh_status()` / `enable()` 会报错。

无硬件回归测试：`python -m unittest discover -s tests -v`。测试保留通信解析、SDK 控制检查和示例退出清理；测试通过不代表完成实机验证。
