"""单条 CAN 总线的启动配置与控制，参数名称对应 DM_CAN.py。"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, fields
from typing import Any, Callable, ClassVar, Optional, Sequence, Union

from .DM_CAN import Control_Type, DM_Motor_Type, DM_variable, Motor, MotorControl


MAX_MOTORS_PER_BUS = 16
SLAVE_ID_MIN = 0x00
SLAVE_ID_MAX = 0x0F
# 位置读数窗口告警余量：当前位置距 ±PMAX 边界不足这个距离时提示回绕风险
POSITION_MARGIN_WARN = 0.5
# 反馈帧 data[0] 高四位的 ERR 是状态码，不是故障位掩码：0x00/0x01 为使能状态，0x08..0x0E 为故障。
# 故障名出处：damiao/SDK/电机SDK/C++例程/u2can/include/damiao.h 的 CAN_Receive_Frame 注释
# （8 超压 9 欠压 A 过流 B MOS过温 C 电机线圈过温 D 通讯丢失 E 过载），ROS1 例程同位置一致。
# 注意：0x00/0x01 与失能/使能的对应关系暂无文字出处，代码不依赖它：使能后的实测码记录为稳态码
# （见 MotorManager.enable_all），运行中偏离稳态码即报错。
_STATUS_NAMES = {
    0x00: "已失能",
    0x01: "已使能",
    0x08: "过压",
    0x09: "欠压",
    0x0A: "过流",
    0x0B: "MOS 过温",
    0x0C: "电机线圈过温",
    0x0D: "通信丢失",
    0x0E: "过载",
}
PARAMETERS_TO_READ = (
    ("mode", DM_variable.CTRL_MODE),
    ("damping", DM_variable.Damp),
    ("gear_ratio", DM_variable.Gr),
    ("pmax", DM_variable.PMAX),
    ("vmax", DM_variable.VMAX),
    ("tmax", DM_variable.TMAX),
)

# 参与编解码的量程参数（PARAMETERS_TO_READ 的子集）；读取后用于对齐 SDK 的 Limit_Param。
LIMIT_PARAMETERS = (
    ("pmax", DM_variable.PMAX),
    ("vmax", DM_variable.VMAX),
    ("tmax", DM_variable.TMAX),
)


def _same(values: Sequence[float], other: Sequence[float]) -> bool:
    return all(math.isclose(a, b, rel_tol=1e-6, abs_tol=1e-6) for a, b in zip(values, other))


def _finite_fields(config: Any, optional: tuple[str, ...] = ()) -> None:
    for field in fields(config):
        value = getattr(config, field.name)
        if value is None and field.name in optional:
            continue
        if not math.isfinite(value):
            raise ValueError(f"{field.name} 必须是有限数")


@dataclass(frozen=True)
class MITControl:
    """controlMIT(kp, kd, q, dq, tau)；q=None 时取运行开始时的位置。"""

    kp: float
    kd: float
    q: Optional[float]  # rad；None 表示沿用起始反馈位置
    dq: float          # rad/s
    tau: float         # N·m
    mode: ClassVar[Control_Type] = Control_Type.MIT

    def __post_init__(self) -> None:
        _finite_fields(self, optional=("q",))
        if not 0 <= self.kp <= 500 or not 0 <= self.kd <= 5:
            raise ValueError("MIT 的 kp 必须在 0..500，kd 必须在 0..5")


@dataclass(frozen=True)
class PositionVelocityControl:
    """control_Pos_Vel(P_desired, V_desired)。"""

    P_desired: Optional[float]  # rad；None 表示沿用起始反馈位置
    V_desired: float            # rad/s
    mode: ClassVar[Control_Type] = Control_Type.POS_VEL

    def __post_init__(self) -> None:
        _finite_fields(self, optional=("P_desired",))


@dataclass(frozen=True)
class VelocityControl:
    """control_Vel(Vel_desired)，单位 rad/s。"""

    Vel_desired: float
    mode: ClassVar[Control_Type] = Control_Type.VEL

    def __post_init__(self) -> None:
        _finite_fields(self)


@dataclass(frozen=True)
class PositionForceControl:
    """control_pos_force(Pos_des, Vel_des, i_des)，整数值按 SDK 原样传递。"""

    Pos_des: Optional[float]  # rad；None 表示沿用起始反馈位置
    Vel_des: int             # 速度 × 100；100 表示 1 rad/s
    i_des: int               # 电流标幺值 × 10000；3000 表示最大电流的 30%
    mode: ClassVar[Control_Type] = Control_Type.Torque_Pos

    def __post_init__(self) -> None:
        _finite_fields(self, optional=("Pos_des",))
        for name, limit in (("Vel_des", 0xFFFF), ("i_des", 10000)):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool):
                raise TypeError(f"{name} 必须使用 SDK 的整数编码值")
            if not 0 <= value <= limit:
                raise ValueError(f"{name} 必须在 0..{limit}")


ControlCommand = Union[
    MITControl, PositionVelocityControl, VelocityControl, PositionForceControl
]
_CONTROL_TYPES = (MITControl, PositionVelocityControl, VelocityControl, PositionForceControl)
# 各控制模式的位置目标字段名；速度模式没有位置目标，不做范围校验
_TARGET_FIELDS = {
    MITControl: "q",
    PositionVelocityControl: "P_desired",
    PositionForceControl: "Pos_des",
}


@dataclass(frozen=True)
class SineMotion:
    """可选的位置往复运动：位置基值 + amplitude * sin(2π * frequency * t + phase)。"""

    amplitude: float        # rad
    frequency: float = 0.5  # Hz
    phase: float = 0.0      # rad

    def __post_init__(self) -> None:
        _finite_fields(self)
        if self.amplitude < 0 or self.frequency < 0:
            raise ValueError("amplitude 和 frequency 不能为负数")


def _validate_ids(slave_id: int, master_id: int) -> None:
    for name, value, lower, upper in (
        ("Slave ID", slave_id, SLAVE_ID_MIN, SLAVE_ID_MAX),
        ("Master ID", master_id, 1, 0x7FF),
    ):
        if not isinstance(value, int) or isinstance(value, bool):
            raise TypeError(f"{name} 必须是整数")
        if not lower <= value <= upper:
            raise ValueError(f"{name} 必须在 0x{lower:02X}..0x{upper:02X}")
    if slave_id == master_id:
        raise ValueError("Slave ID 和 Master ID 不能相同")


@dataclass(frozen=True)
class MotorConfig:
    """一台电机的 ID、控制参数和可选位置轨迹。模式由 control 的类型确定。"""

    slave_id: int
    master_id: int
    control: ControlCommand
    motor_type: DM_Motor_Type = DM_Motor_Type.DM6006
    motion: Optional[SineMotion] = None
    damping: Optional[float] = None
    save_parameters: bool = False

    def __post_init__(self) -> None:
        _validate_ids(self.slave_id, self.master_id)
        object.__setattr__(self, "motor_type", DM_Motor_Type(self.motor_type))
        if not isinstance(self.control, _CONTROL_TYPES):
            raise TypeError("control 必须是四种控制参数类之一")
        if self.motion is not None:
            if not isinstance(self.motion, SineMotion):
                raise TypeError("motion 必须是 SineMotion 或 None")
            if isinstance(self.control, VelocityControl):
                raise ValueError("速度模式没有位置目标，不能配置 SineMotion")
        if self.damping is not None and (
            not math.isfinite(self.damping) or not 0 < self.damping <= 100
        ):
            raise ValueError("damping 必须大于 0 且不超过 100")


class ManagedMotor(Motor):
    """带反馈簿记的 Motor：反馈帧计数与最近一帧时间戳，供上层判定反馈有效性。"""

    def __init__(self, motor_type: DM_Motor_Type, slave_id: int, master_id: int) -> None:
        super().__init__(motor_type, slave_id, master_id)
        self.feedback_count = 0
        self.last_feedback_at: Optional[float] = None

    def recv_data(self, q: float, dq: float, tau: float, err: int) -> None:
        super().recv_data(q, dq, tau, err)
        self.feedback_count += 1
        self.last_feedback_at = time.monotonic()


def create_motor(
    slave_id: int, master_id: int, motor_type: DM_Motor_Type = DM_Motor_Type.DM6006
) -> Motor:
    _validate_ids(slave_id, master_id)
    return ManagedMotor(DM_Motor_Type(motor_type), slave_id, master_id)


def create_motors(configs: Sequence[MotorConfig]) -> list[Motor]:
    """启动时创建最多 16 台电机，允许不连续 ID，拒绝重复映射。"""
    if len(configs) > MAX_MOTORS_PER_BUS:
        raise ValueError("单条 CAN 总线最多支持 16 个电机")
    occupied = set()
    for config in configs:
        if not isinstance(config, MotorConfig):
            raise TypeError("每台电机必须用 MotorConfig 配置")
        ids = {config.slave_id, config.master_id}
        if ids & occupied:
            raise ValueError("不同电机的 Slave/Master ID 不能重复或交叉占用")
        occupied.update(ids)
    return [
        create_motor(config.slave_id, config.master_id, config.motor_type)
        for config in configs
    ]


@dataclass(frozen=True)
class RunConfig:
    """总线循环时序；每台电机的控制参数在 MotorConfig.control 中。"""

    seconds: float = 5.0
    period: float = 0.01
    status_interval: float = 0.25
    feedback_timeout: float = 0.2  # s；超过这么久没有新反馈就停止控制并失能

    def __post_init__(self) -> None:
        _finite_fields(self)
        if self.seconds < 0 or self.period <= 0 or self.status_interval < 0:
            raise ValueError("seconds/status_interval 不能为负，period 必须大于 0")
        if self.feedback_timeout <= self.period:
            raise ValueError(
                "feedback_timeout 必须大于 period（每周期才刷新一次反馈，建议至少 10 倍）"
            )


class MotorManager:
    """一条 CAN 总线共用一个 MotorControl，电机列表在构造时确定。"""

    def __init__(
        self,
        control: MotorControl,
        configs: Sequence[MotorConfig],
        serial_device: Any = None,
    ) -> None:
        self.configs = tuple(configs)
        self._motors = tuple(create_motors(self.configs))
        if control.motors_map:
            raise ValueError("请为这条总线提供尚未添加电机的 MotorControl")
        self.control = control
        self.serial_device = serial_device
        self._enabled: dict[int, Motor] = {}
        self._enabled_code: dict[int, int] = {}  # 使能后实测的反馈码（使能稳态码）
        self._limits: dict[int, dict[str, float]] = {}  # 各电机实测量程（align_limits 填充）
        self._configured = False
        for motor in self._motors:
            self.control.addMotor(motor)

    @classmethod
    def connect(
        cls,
        port: str,
        configs: Sequence[MotorConfig],
        *,
        baudrate: int = 921600,
        timeout: float = 0.5,
    ) -> "MotorManager":
        import serial

        configs = tuple(configs)
        create_motors(configs)  # 打开串口前校验整组配置
        serial_device = serial.Serial(baudrate=baudrate, timeout=timeout)
        serial_device.port = port  # 由 SDK 构造函数打开，避免重复开关串口
        try:
            return cls(MotorControl(serial_device), configs, serial_device)
        except BaseException:
            serial_device.close()
            raise

    @property
    def motors(self) -> tuple[Motor, ...]:
        return self._motors

    @property
    def motor_count(self) -> int:
        return len(self._motors)

    def align_limits(self) -> dict[int, dict[str, float]]:
        """读取每台电机的实际量程，并按实际值对齐 SDK 的编解码表。

        官方 C 驱动同样用电机实际量程编解码（dm_motor_drv.c 的 dm_motor_fbdata 配合
        dm_motor_ctrl.c 的 RID 读取），这里复用 SDK 现成的 change_limit_param()。
        读不到或读到非法值直接报错；同型号电机实际量程不一致也会报错。
        """
        ranges: dict[int, dict[str, float]] = {}
        by_type: dict[DM_Motor_Type, tuple[float, float, float]] = {}
        for motor in self.motors:
            values = {}
            for name, rid in LIMIT_PARAMETERS:
                motor.temp_param_dict.pop(rid, None)
                value = self.control.read_motor_param(motor, rid)
                if (
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(value)
                    or value <= 0
                ):
                    raise RuntimeError(
                        f"电机 0x{motor.SlaveID:02X} 的 {name.upper()} 读取失败（{value!r}），无法对齐量程"
                    )
                values[name] = float(value)
            limits = (values["pmax"], values["vmax"], values["tmax"])
            known = by_type.get(motor.MotorType)
            if known is not None and not _same(known, limits):
                raise RuntimeError(
                    f"同型号电机 {motor.MotorType.name} 的实际量程不一致（{known} 与 {limits}），"
                    "无法共用一张编解码表"
                )
            by_type[motor.MotorType] = limits
            table = self.control.Limit_Param[motor.MotorType]
            if not _same(table, limits):
                print(
                    f"电机 0x{motor.SlaveID:02X} 量程与 SDK 表不一致：表 {tuple(table)} → 实际 {limits}，"
                    "已按实际值对齐"
                )
                self.control.change_limit_param(motor.MotorType, *limits)
            ranges[motor.SlaveID] = values
            self._limits[motor.SlaveID] = dict(values)
        return ranges

    def configure(self) -> None:
        """先发送失能、对齐量程，再逐台应用参数类对应的模式和可选 Damp。"""
        self._configured = False
        self._enabled.update((motor.SlaveID, motor) for motor in self.motors)
        self.disable_all()
        self.align_limits()
        for motor, config in zip(self.motors, self.configs):
            motor.temp_param_dict.pop(DM_variable.CTRL_MODE, None)
            if not self.control.switchControlMode(motor, config.control.mode):
                raise RuntimeError(f"电机 0x{motor.SlaveID:02X} 切换模式失败")
            if config.damping is not None:
                motor.temp_param_dict.pop(DM_variable.Damp, None)
                if not self.control.change_motor_param(motor, DM_variable.Damp, config.damping):
                    raise RuntimeError(f"电机 0x{motor.SlaveID:02X} 修改 Damp 失败")
                if config.save_parameters:
                    self.control.save_motor_param(motor)
        self._configured = True

    def refresh_status(self, wait: float = 0.1) -> tuple[Motor, ...]:
        if not math.isfinite(wait) or wait < 0:
            raise ValueError("wait 必须是非负有限数")
        for motor in self.motors:
            self.control.refresh_motor_status(motor)
        time.sleep(wait)
        self.control.recv()
        return self.motors

    def read_parameters(self, parameters=PARAMETERS_TO_READ) -> dict[int, dict[str, Any]]:
        parameters = tuple(parameters)
        result = {}
        for motor in self.motors:
            values = {}
            for name, rid in parameters:
                motor.temp_param_dict.pop(rid, None)
                values[name] = self.control.read_motor_param(motor, rid)
            result[motor.SlaveID] = values
        return result

    def status_snapshot(self) -> list[dict[str, Any]]:
        """error 沿用 SDK 名称，实际为状态码：0/1 为状态，8..E 为故障。"""
        now = time.monotonic()
        return [
            dict(
                slave_id=motor.SlaveID,
                master_id=motor.MasterID,
                position=float(motor.getPosition()),
                velocity=float(motor.getVelocity()),
                torque=float(motor.getTorque()),
                error=int(motor.getError()),
                status_name=_STATUS_NAMES.get(int(motor.getError()), "未知状态"),
                feedback_count=motor.feedback_count,
                age=None if motor.last_feedback_at is None else now - motor.last_feedback_at,
            )
            for motor in self.motors
        ]

    @property
    def link_stats(self) -> dict[str, int]:
        """总线级计数：无主的参数类应答帧数、被忽略的适配器帧数（0xEE 通信错误等）。"""
        return dict(
            dropped_param_replies=self.control.dropped_param_replies,
            ignored_frames=self.control.ignored_frames,
        )

    def enable_all(self) -> None:
        """使能全部电机，并确认每台都回了新反馈、反馈码确实发生了变化。"""
        if not self._configured:
            raise RuntimeError("请先调用 configure() 应用电机配置")
        silent = [motor.SlaveID for motor in self.motors if motor.feedback_count == 0]
        if silent:
            raise RuntimeError(
                "以下电机尚未收到任何反馈，无法使能："
                + "、".join(f"0x{slave_id:02X}" for slave_id in silent)
                + "（检查 Slave ID、接线、PC 侧与 CAN 侧波特率）"
            )
        self._enabled_code.clear()
        try:
            before_count = {motor.SlaveID: motor.feedback_count for motor in self.motors}
            before_code = {motor.SlaveID: int(motor.getError()) for motor in self.motors}
            for motor in self.motors:
                self._enabled[motor.SlaveID] = motor
                self.control.enable(motor)
            for motor in self.motors:
                slave_id = motor.SlaveID
                if motor.feedback_count == before_count[slave_id]:
                    raise RuntimeError(
                        f"电机 0x{slave_id:02X} 使能后没有收到新反馈，无法确认已使能"
                    )
                code = int(motor.getError())
                if code == before_code[slave_id]:
                    raise RuntimeError(
                        f"电机 0x{slave_id:02X} 使能后反馈码仍是 0x{code:X}"
                        f"（{_STATUS_NAMES.get(code, '未知状态')}），使能未生效"
                    )
                self._enabled_code[slave_id] = code
            self._check_errors()
        except BaseException:
            self._enabled_code.clear()
            self.disable_all()
            raise

    def disable_all(self) -> None:
        first_error = None
        for motor in reversed(tuple(self._enabled.values())):
            try:
                self.control.disable(motor)
            except Exception as exc:
                if first_error is None:
                    first_error = exc
            else:
                del self._enabled[motor.SlaveID]
        if first_error is not None:
            raise first_error

    def _send_control(self, motor: Motor, command: ControlCommand, position: float) -> None:
        if isinstance(command, MITControl):
            self.control.controlMIT(
                motor, kp=command.kp, kd=command.kd, q=position,
                dq=command.dq, tau=command.tau,
            )
        elif isinstance(command, PositionVelocityControl):
            self.control.control_Pos_Vel(
                motor, P_desired=position, V_desired=command.V_desired,
            )
        elif isinstance(command, VelocityControl):
            self.control.control_Vel(motor, Vel_desired=command.Vel_desired)
        elif isinstance(command, PositionForceControl):
            self.control.control_pos_force(
                motor, Pos_des=position, Vel_des=command.Vel_des, i_des=command.i_des,
            )

    def _check_errors(self) -> None:
        for motor in self.motors:
            code = int(motor.getError())
            if code not in (0x00, 0x01):
                reason = _STATUS_NAMES.get(code, "未知状态")
                raise RuntimeError(
                    f"电机 0x{motor.SlaveID:02X} 反馈码 0x{code:X}（{reason}），控制已停止"
                )

    def _check_feedback(self, motor: ManagedMotor, timeout: float) -> None:
        """反馈超时或从未收到反馈时停止控制。"""
        last = motor.last_feedback_at
        if last is None:
            raise RuntimeError(f"电机 0x{motor.SlaveID:02X} 从未收到反馈，控制已停止")
        age = time.monotonic() - last
        if age > timeout:
            raise RuntimeError(
                f"电机 0x{motor.SlaveID:02X} 反馈超时（{age:.3f}s > {timeout:.3f}s），控制已停止"
            )

    def _check_enabled_state(self, motor: ManagedMotor) -> None:
        """反馈码偏离使能稳态码：失去使能或出现新状态。"""
        steady = self._enabled_code.get(motor.SlaveID)
        code = int(motor.getError())
        if steady is not None and code != steady:
            raise RuntimeError(
                f"电机 0x{motor.SlaveID:02X} 反馈码由 0x{steady:X}"
                f"（{_STATUS_NAMES.get(steady, '未知状态')}）变为 0x{code:X}"
                f"（{_STATUS_NAMES.get(code, '未知状态')}），控制已停止"
            )

    def _pmax(self, motor: ManagedMotor) -> float:
        """该电机实测的位置量程；未对齐时说明原因。"""
        limits = self._limits.get(motor.SlaveID)
        if not limits:
            raise RuntimeError(
                f"电机 0x{motor.SlaveID:02X} 尚未对齐量程，无法校验位置目标（请先 configure()）"
            )
        return limits["pmax"]

    def _check_target(self, motor: ManagedMotor, command: ControlCommand, value: float) -> None:
        """位置目标必须落在实测 ±PMAX 内：SDK 原样会静默截断，这里改为直接报错。"""
        field = _TARGET_FIELDS.get(type(command))
        if field is None:  # 速度模式没有位置目标
            return
        limit = self._pmax(motor)
        if abs(value) > limit:
            raise RuntimeError(
                f"电机 0x{motor.SlaveID:02X} 的目标 {field}={value:.4f} rad "
                f"超出量程 ±{limit} rad，控制已停止"
            )

    def _warn_position_margin(self, margin: float = POSITION_MARGIN_WARN) -> None:
        """当前位置贴近 ±PMAX 读数窗口边界时告警：再同向运动会回绕 ±2×PMAX。"""
        for motor in self.motors:
            limits = self._limits.get(motor.SlaveID)
            if not limits:
                continue
            pmax = limits["pmax"]
            position = float(motor.getPosition())
            room = pmax - abs(position)
            if room < margin:
                print(
                    f"电机 0x{motor.SlaveID:02X} 当前位置 {position:.3f} rad 距读数窗口边界 ±{pmax} rad "
                    f"只剩 {room:.3f} rad：同向继续运动会回绕 ±{2 * pmax:.1f} rad。"
                    "若要在此方向继续运动，请先把机构摆到已知姿态并用 set_zero_position 对齐零点。"
                )

    def run(
        self, config: RunConfig, *,
        on_status: Optional[Callable[[list[dict[str, Any]]], None]] = None,
    ) -> None:
        """按独立控制参数发送；反馈超时、反馈码异常或故障时停止并失能全部电机。"""
        if not self.motors or len(self._enabled) != len(self.motors):
            raise RuntimeError("请先配置并使能电机")
        try:
            self._warn_position_margin()
            bases = []
            for motor, motor_config in zip(self.motors, self.configs):
                command = motor_config.control
                if isinstance(command, MITControl):
                    target = command.q
                elif isinstance(command, PositionVelocityControl):
                    target = command.P_desired
                elif isinstance(command, PositionForceControl):
                    target = command.Pos_des
                else:
                    target = 0.0
                # 显式数字按 SDK 绝对位置解释；仅 None 使用本轮开始时的反馈。
                if target is None:
                    if motor.feedback_count == 0:
                        raise RuntimeError(
                            f"电机 0x{motor.SlaveID:02X} 没有可用反馈，无法用 None 表示当前位置"
                            "（请先 refresh_status() 或显式给出位置）"
                        )
                    bases.append(float(motor.getPosition()))
                else:
                    self._check_target(motor, command, target)
                    bases.append(target)

            start = time.monotonic()
            last_status = start
            while time.monotonic() - start < config.seconds:
                cycle_start = time.monotonic()
                elapsed = cycle_start - start
                for motor, motor_config, base in zip(self.motors, self.configs, bases):
                    self._check_feedback(motor, config.feedback_timeout)
                    self._check_errors()
                    self._check_enabled_state(motor)
                    position = base
                    motion = motor_config.motion
                    if motion is not None:
                        position += motion.amplitude * math.sin(
                            2 * math.pi * motion.frequency * elapsed + motion.phase
                        )
                    # 最终目标（base + 正弦）也要在量程内，避免叠加后越界
                    self._check_target(motor, motor_config.control, position)
                    self._send_control(motor, motor_config.control, position)
                    self._check_errors()
                now = time.monotonic()
                if on_status is not None and now - last_status >= config.status_interval:
                    on_status(self.status_snapshot())
                    last_status = now
                time.sleep(max(0.0, config.period - (time.monotonic() - cycle_start)))
        finally:
            self.disable_all()

    def close(self) -> None:
        try:
            self.disable_all()
        finally:
            if self.serial_device is not None:
                self.serial_device.close()
