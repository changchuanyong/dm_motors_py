"""使用内存 SDK 替身验证配置与控制，不打开串口。"""

import io
import math
from contextlib import redirect_stdout
from pathlib import Path
import runpy
import unittest
from unittest.mock import Mock, patch

from motor import (
    Control_Type,
    DM_Motor_Type,
    DM_variable,
    MITControl,
    MotorConfig,
    MotorControl,
    MotorManager,
    PositionForceControl,
    PositionVelocityControl,
    RunConfig,
    SineMotion,
    VelocityControl,
    create_motor,
    create_motors,
)


def make_frame(can_id: int, data: bytes, cmd: int = 0x11) -> bytes:
    """按 USB2CAN 适配器的 16 字节格式封装一帧（帧头 0xAA、帧尾 0x55）。"""
    packet = bytearray(16)
    packet[0] = 0xAA
    packet[1] = cmd
    packet[2] = len(data)
    packet[3:7] = can_id.to_bytes(4, "little")
    packet[7:15] = data
    packet[15] = 0x55
    return bytes(packet)


class FakeSerial:
    """内存串口替身：write() 记录发送内容，read_all() 依次吐出预置帧。"""

    def __init__(self, frames=()):
        self.frames = list(frames)
        self.written = []
        self.is_open = False

    def write(self, data):
        self.written.append(bytes(data))

    def read_all(self):
        return self.frames.pop(0) if self.frames else b""

    def open(self):
        self.is_open = True

    def close(self):
        self.is_open = False


class FakeControl:
    """内存 SDK 替身：记录事件与指令，并可模拟反馈、掉线与故障。"""

    def __init__(self, limits=(12.5, 45.0, 20.0)):
        self.motors_map = {}
        self.events = []
        self.commands = []
        self.fault_on_command = None
        self.enabled_code = 0x01     # 使能后的反馈码
        self.disabled_code = 0x00    # 失能后的反馈码
        self.command_replies = True  # False：控制帧不再产生反馈（模拟运行中掉线）
        self.enable_replies = True   # False：使能后不回帧
        self.disable_replies = True  # False：失能后不回帧
        self.Limit_Param = {DM_Motor_Type.DM6006: list(limits)}
        self.param_values = {}       # (slave_id, rid) -> 读参数返回值；缺省用量程表
        self.limit_calls = []
        self.dropped_param_replies = 0
        self.ignored_frames = 0

    def addMotor(self, motor):
        self.motors_map[motor.SlaveID] = motor
        self.motors_map[motor.MasterID] = motor

    def disable(self, motor):
        self.events.append(("disable", motor.SlaveID))
        if self.disable_replies:
            self._feedback(motor, self.disabled_code)

    def enable(self, motor):
        self.events.append(("enable", motor.SlaveID))
        if self.enable_replies:
            self._feedback(motor, self.enabled_code)

    def switchControlMode(self, motor, mode):
        self.events.append(("mode", motor.SlaveID, mode))
        return True

    def change_motor_param(self, motor, parameter, value):
        self.events.append(("parameter", motor.SlaveID, parameter, value))
        return True

    def save_motor_param(self, motor):
        self.events.append(("save", motor.SlaveID))

    def refresh_motor_status(self, motor):
        pass

    def recv(self):
        pass

    def read_motor_param(self, motor, rid):
        if (motor.SlaveID, rid) in self.param_values:
            return self.param_values[(motor.SlaveID, rid)]
        table = self.Limit_Param[motor.MotorType]
        return {
            DM_variable.PMAX: table[0],
            DM_variable.VMAX: table[1],
            DM_variable.TMAX: table[2],
        }.get(rid)

    def change_limit_param(self, motor_type, pmax, vmax, tmax):
        self.limit_calls.append((motor_type, pmax, vmax, tmax))
        self.Limit_Param[motor_type] = [pmax, vmax, tmax]

    def _feedback(self, motor, code):
        motor.recv_data(motor.getPosition(), motor.getVelocity(), motor.getTorque(), code)

    def _command(self, name, motor, *values):
        self.commands.append((name, motor.SlaveID, *values))
        if self.fault_on_command == motor.SlaveID:
            motor.state_err = 0x08
        elif self.command_replies:
            self._feedback(motor, self.enabled_code)

    def controlMIT(self, motor, kp, kd, q, dq, tau):
        self._command("mit", motor, kp, kd, q, dq, tau)

    def control_Pos_Vel(self, motor, P_desired, V_desired):
        self._command("pos_vel", motor, P_desired, V_desired)

    def control_Vel(self, motor, Vel_desired):
        self._command("vel", motor, Vel_desired)

    def control_pos_force(self, motor, Pos_des, Vel_des, i_des):
        self._command("pos_force", motor, Pos_des, Vel_des, i_des)


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class MotorManagerTests(unittest.TestCase):
    def make_manager(self, *configs):
        self.clock = FakeClock()
        for target in ("monotonic", "sleep"):
            patcher = patch(f"motor.motor_manager.time.{target}", getattr(self.clock, target))
            patcher.start()
            self.addCleanup(patcher.stop)
        control = FakeControl()
        manager = MotorManager(control, configs=configs)
        self.addCleanup(manager.close)
        manager.configure()
        manager.enable_all()
        return manager, control

    def run_manager(self, manager, *, seconds=0.01, period=0.01, feedback_timeout=0.2):
        manager.run(RunConfig(seconds=seconds, period=period, feedback_timeout=feedback_timeout))

    def test_four_modes_forward_distinct_sdk_parameters(self):
        manager, control = self.make_manager(
            MotorConfig(0x01, 0x11, MITControl(kp=12.0, kd=1.5, q=-0.2, dq=0.7, tau=-0.3)),
            MotorConfig(0x03, 0x21, PositionVelocityControl(P_desired=0.8, V_desired=0.4)),
            MotorConfig(0x07, 0x25, VelocityControl(Vel_desired=-0.6)),
            MotorConfig(0x0F, 0x2F, PositionForceControl(Pos_des=-0.9, Vel_des=100, i_des=3000)),
        )
        self.run_manager(manager)
        self.assertEqual(
            control.commands,
            [
                ("mit", 0x01, 12.0, 1.5, -0.2, 0.7, -0.3),
                ("pos_vel", 0x03, 0.8, 0.4),
                ("vel", 0x07, -0.6),
                ("pos_force", 0x0F, -0.9, 100, 3000),
            ],
        )
        self.assertEqual(
            [event for event in control.events if event[0] == "mode"],
            [
                ("mode", 0x01, Control_Type.MIT),
                ("mode", 0x03, Control_Type.POS_VEL),
                ("mode", 0x07, Control_Type.VEL),
                ("mode", 0x0F, Control_Type.Torque_Pos),
            ],
        )

    def test_same_mode_preserves_each_motors_parameters(self):
        manager, control = self.make_manager(
            MotorConfig(1, 0x11, MITControl(kp=3.0, kd=0.2, q=0.0, dq=-0.5, tau=0.1)),
            MotorConfig(5, 0x15, MITControl(kp=7.0, kd=0.8, q=1.0, dq=1.5, tau=-0.2)),
        )
        for motor in manager.motors:
            motor.state_q = -2.4
        self.run_manager(manager)
        self.assertEqual(control.commands, [
            ("mit", 1, 3.0, 0.2, 0.0, -0.5, 0.1),
            ("mit", 5, 7.0, 0.8, 1.0, 1.5, -0.2),
        ])

    def test_mode_classes_reject_other_modes_and_old_parameter_names(self):
        cases = (
            (MITControl, {"kp": 2.0, "kd": 0.1, "q": 0.0, "dq": 0.0, "tau": 0.0, "speed": 1.0}),
            (MITControl, {"kp": 2.0, "kd": 0.1, "q": 0.0, "dq": 0.0, "tau": 0.0, "torque": 1.0}),
            (PositionVelocityControl, {"P_desired": 0.0, "V_desired": 1.0, "kp": 2.0}),
            (VelocityControl, {"Vel_desired": 1.0, "q": 0.0}),
            (PositionForceControl, {"Pos_des": 0.0, "Vel_des": 100, "i_des": 3000, "current": 1}),
        )
        for constructor, arguments in cases:
            with self.subTest(constructor=constructor.__name__, arguments=arguments):
                with self.assertRaises(TypeError):
                    constructor(**arguments)

    def test_velocity_rejects_position_motion(self):
        with self.assertRaises(ValueError):
            MotorConfig(1, 0x11, VelocityControl(Vel_desired=0.2), motion=SineMotion(amplitude=0.1))

    def test_only_position_fields_accept_none(self):
        invalid = (
            lambda: MITControl(kp=2.0, kd=0.1, q=None, dq=None, tau=0.0),
            lambda: MITControl(kp=2.0, kd=0.1, q=None, dq=0.0, tau=None),
            lambda: PositionVelocityControl(P_desired=None, V_desired=None),
            lambda: VelocityControl(Vel_desired=None),
            lambda: RunConfig(period=None),
        )
        for constructor in invalid:
            with self.subTest(constructor=constructor):
                with self.assertRaises(TypeError):
                    constructor()

    def test_run_config_rejects_feedback_timeout_not_larger_than_period(self):
        for timeout in (0.0, -1.0, 0.01):
            with self.subTest(feedback_timeout=timeout):
                with self.assertRaises(ValueError):
                    RunConfig(seconds=0.1, period=0.01, feedback_timeout=timeout)

    def test_position_force_requires_valid_sdk_integer_encodings(self):
        for field, value in (("Vel_des", 100.0), ("i_des", 3000.0), ("Vel_des", True), ("i_des", False)):
            with self.subTest(field=field, value=value):
                arguments = dict(Pos_des=0.0, Vel_des=100, i_des=3000)
                arguments[field] = value
                with self.assertRaises(TypeError):
                    PositionForceControl(**arguments)
        for field, value in (("Vel_des", -1), ("Vel_des", 0x10000), ("i_des", -1), ("i_des", 10001)):
            with self.subTest(field=field, value=value):
                arguments = dict(Pos_des=0.0, Vel_des=100, i_des=3000)
                arguments[field] = value
                with self.assertRaises(ValueError):
                    PositionForceControl(**arguments)

    def test_position_force_encoding_matches_actual_sdk_frame(self):
        command = PositionForceControl(Pos_des=-0.5, Vel_des=100, i_des=3000)
        motor = create_motor(1, 0x11)
        sdk = MotorControl.__new__(MotorControl)
        sdk.motors_map = {motor.SlaveID: motor}
        sdk._MotorControl__send_data = Mock()
        sdk.recv = Mock()
        sdk.control_pos_force(motor, command.Pos_des, command.Vel_des, command.i_des)
        sdk._MotorControl__send_data.assert_called_once()
        can_id, payload = sdk._MotorControl__send_data.call_args.args
        self.assertEqual(can_id, 0x301)
        self.assertEqual(payload[4:8].tolist(), [100, 0, 184, 11])
        sdk.recv.assert_called_once_with()

    def test_unspecified_positions_use_initial_cache_in_all_position_modes(self):
        for position in (0.0, -1.25):
            with self.subTest(position=position):
                manager, control = self.make_manager(
                    MotorConfig(1, 0x11, MITControl(kp=2.0, kd=0.1, q=None, dq=0.3, tau=0.0)),
                    MotorConfig(3, 0x13, PositionVelocityControl(P_desired=None, V_desired=0.2)),
                    MotorConfig(5, 0x15, PositionForceControl(Pos_des=None, Vel_des=100, i_des=3000)),
                )
                for motor in manager.motors:
                    motor.state_q = position
                self.run_manager(manager)
                self.assertEqual(control.commands[0][4], position)
                self.assertEqual(control.commands[1][2], position)
                self.assertEqual(control.commands[2][2], position)

    def test_sine_motion_uses_frequency_phase_and_position_base(self):
        manager, control = self.make_manager(MotorConfig(
            1, 0x11,
            PositionVelocityControl(P_desired=-0.5, V_desired=0.2),
            motion=SineMotion(amplitude=0.1, frequency=1.0, phase=math.pi / 2),
        ))
        self.run_manager(manager, seconds=0.5, period=0.25, feedback_timeout=1.0)
        self.assertEqual(len(control.commands), 2)
        self.assertAlmostEqual(control.commands[0][2], -0.4)
        self.assertAlmostEqual(control.commands[1][2], -0.5)

    def test_configuration_disables_before_switching_and_applies_internal_parameter(self):
        control = FakeControl()
        manager = MotorManager(control, configs=(
            MotorConfig(1, 0x11, VelocityControl(Vel_desired=0.1), damping=2.0, save_parameters=True),
            MotorConfig(3, 0x13, VelocityControl(Vel_desired=0.2)),
        ))
        self.addCleanup(manager.close)
        manager.configure()
        self.assertCountEqual(control.events[:2], [("disable", 1), ("disable", 3)])
        self.assertIn(("parameter", 1, DM_variable.Damp, 2.0), control.events)
        self.assertIn(("save", 1), control.events)
        self.assertNotIn(("save", 3), control.events)

    def test_one_bus_accepts_all_sixteen_ids(self):
        configs = tuple(MotorConfig(i, 0x20 + i, VelocityControl(Vel_desired=0.0)) for i in range(16))
        motors = create_motors(configs)
        self.assertEqual([motor.SlaveID for motor in motors], list(range(16)))

    def test_invalid_or_colliding_ids_are_rejected(self):
        for slave_id in (-1, 0x10):
            with self.subTest(slave_id=slave_id):
                with self.assertRaises(ValueError):
                    create_motor(slave_id, 0x11)
        for ids in (((1, 0x11), (1, 0x12)), ((1, 0x11), (3, 0x11)), ((1, 3), (3, 0x13))):
            with self.subTest(ids=ids):
                with self.assertRaises(ValueError):
                    create_motors(tuple(MotorConfig(s, m, VelocityControl(Vel_desired=0.0)) for s, m in ids))
        configs = tuple(MotorConfig(i, 0x20 + i, VelocityControl(Vel_desired=0.0)) for i in range(16))
        with self.assertRaises(ValueError):
            create_motors(configs + (configs[0],))

    def test_enabled_sdk_feedback_allows_control_and_normal_shutdown(self):
        manager, control = self.make_manager(
            MotorConfig(1, 0x11, VelocityControl(Vel_desired=0.1)),
        )
        sdk = MotorControl.__new__(MotorControl)
        sdk.motors_map = control.motors_map
        sdk._MotorControl__process_packet(bytes([0x11, 0x80, 0x00, 0x80, 0x08, 0x00, 25, 25]), 0x11, 0x11)
        self.assertEqual(manager.motors[0].getError(), 0x01)
        control.events.clear()

        self.run_manager(manager)

        self.assertEqual(control.commands, [("vel", 1, 0.1)])
        self.assertEqual(control.events, [("disable", 1)])

    def test_unknown_sdk_feedback_stops_before_control_and_disables_all(self):
        manager, control = self.make_manager(
            MotorConfig(1, 0x11, VelocityControl(Vel_desired=0.1)),
            MotorConfig(3, 0x13, VelocityControl(Vel_desired=0.2)),
        )
        sdk = MotorControl.__new__(MotorControl)
        sdk.motors_map = control.motors_map
        sdk._MotorControl__process_packet(bytes([0x21, 0x80, 0x00, 0x80, 0x08, 0x00, 25, 25]), 0x11, 0x11)
        self.assertEqual(manager.motors[0].getError(), 0x02)
        control.events.clear()

        with self.assertRaises(RuntimeError):
            self.run_manager(manager)

        self.assertEqual(control.commands, [])
        self.assertCountEqual(control.events, [("disable", 1), ("disable", 3)])

    def test_feedback_fault_stops_remaining_commands_and_disables_all(self):
        manager, control = self.make_manager(
            MotorConfig(1, 0x11, VelocityControl(Vel_desired=0.1)),
            MotorConfig(3, 0x13, VelocityControl(Vel_desired=0.2)),
        )
        control.events.clear()
        control.fault_on_command = 1
        with self.assertRaises(RuntimeError):
            self.run_manager(manager)
        self.assertEqual(control.commands, [("vel", 1, 0.1)])
        self.assertCountEqual(control.events, [("disable", 1), ("disable", 3)])

    def test_run_refuses_without_any_feedback(self):
        """P1-1 回归：一次反馈都没收到时不允许使能，更不允许进入控制循环。"""
        control = FakeControl()
        control.disable_replies = False
        manager = MotorManager(
            control, configs=(MotorConfig(1, 0x11, VelocityControl(Vel_desired=0.1)),)
        )
        self.addCleanup(manager.close)
        manager.configure()
        with self.assertRaises(RuntimeError) as caught:
            manager.enable_all()
        self.assertIn("尚未收到任何反馈", str(caught.exception))
        self.assertEqual(control.commands, [])

    def test_none_position_requires_feedback_before_use(self):
        control = FakeControl()
        control.disable_replies = False
        manager = MotorManager(
            control,
            configs=(
                MotorConfig(1, 0x11, MITControl(kp=2.0, kd=0.1, q=None, dq=0.0, tau=0.0)),
            ),
        )
        self.addCleanup(manager.close)
        manager.configure()
        manager._enabled[1] = manager.motors[0]  # 绕过 enable_all 门禁，直接检验 run() 的兜底
        manager._enabled_code[1] = 0x00
        with self.assertRaises(RuntimeError) as caught:
            manager.run(RunConfig(seconds=0.01))
        self.assertIn("没有可用反馈", str(caught.exception))
        self.assertEqual(control.commands, [])

    def test_feedback_timeout_stops_control_and_disables_all(self):
        manager, control = self.make_manager(
            MotorConfig(1, 0x11, VelocityControl(Vel_desired=0.1)),
            MotorConfig(3, 0x13, VelocityControl(Vel_desired=0.2)),
        )
        control.events.clear()
        control.command_replies = False  # 运行中掉线：控制帧不再产生反馈
        with self.assertRaises(RuntimeError) as caught:
            self.run_manager(manager, seconds=1.0, period=0.01, feedback_timeout=0.2)
        self.assertIn("反馈超时", str(caught.exception))
        self.assertCountEqual(control.events, [("disable", 1), ("disable", 3)])

    def test_enable_without_new_feedback_is_rejected(self):
        control = FakeControl()
        control.enable_replies = False
        manager = MotorManager(
            control, configs=(MotorConfig(1, 0x11, VelocityControl(Vel_desired=0.1)),)
        )
        self.addCleanup(manager.close)
        manager.configure()
        with self.assertRaises(RuntimeError) as caught:
            manager.enable_all()
        self.assertIn("没有收到新反馈", str(caught.exception))
        self.assertEqual(control.events[-1], ("disable", 1))  # 失败后已尝试失能

    def test_enable_without_state_change_is_rejected(self):
        control = FakeControl()
        control.enabled_code = 0x00  # 使能前后反馈码不变：使能未生效
        manager = MotorManager(
            control, configs=(MotorConfig(1, 0x11, VelocityControl(Vel_desired=0.1)),)
        )
        self.addCleanup(manager.close)
        manager.configure()
        with self.assertRaises(RuntimeError) as caught:
            manager.enable_all()
        self.assertIn("使能未生效", str(caught.exception))

    def test_lost_enable_state_stops_control(self):
        manager, control = self.make_manager(
            MotorConfig(1, 0x11, VelocityControl(Vel_desired=0.1)),
            MotorConfig(3, 0x13, VelocityControl(Vel_desired=0.2)),
        )
        control.events.clear()
        control.enabled_code = 0x00  # 运行中掉使能：反馈码偏离使能稳态码
        with self.assertRaises(RuntimeError) as caught:
            self.run_manager(manager, seconds=1.0, period=0.01)
        self.assertIn("反馈码由 0x1", str(caught.exception))
        self.assertCountEqual(control.events, [("disable", 1), ("disable", 3)])

    def test_limits_are_aligned_with_motor_values(self):
        control = FakeControl()
        control.param_values.update(
            {
                (1, DM_variable.PMAX): 6.25,
                (1, DM_variable.VMAX): 45.0,
                (1, DM_variable.TMAX): 12.0,
            }
        )
        manager = MotorManager(
            control, configs=(MotorConfig(1, 0x11, VelocityControl(Vel_desired=0.1)),)
        )
        self.addCleanup(manager.close)
        output = io.StringIO()
        with redirect_stdout(output):
            manager.configure()
        self.assertEqual(control.limit_calls, [(DM_Motor_Type.DM6006, 6.25, 45.0, 12.0)])
        self.assertEqual(control.Limit_Param[DM_Motor_Type.DM6006], [6.25, 45.0, 12.0])
        self.assertIn("量程与 SDK 表不一致", output.getvalue())

    def test_matching_limits_need_no_realignment(self):
        _, control = self.make_manager(
            MotorConfig(1, 0x11, VelocityControl(Vel_desired=0.1))
        )
        self.assertEqual(control.limit_calls, [])

    def test_unreadable_limits_are_rejected(self):
        control = FakeControl()
        control.param_values[(1, DM_variable.TMAX)] = None
        manager = MotorManager(
            control, configs=(MotorConfig(1, 0x11, VelocityControl(Vel_desired=0.1)),)
        )
        self.addCleanup(manager.close)
        with self.assertRaises(RuntimeError) as caught:
            manager.configure()
        self.assertIn("TMAX 读取失败", str(caught.exception))
        self.assertEqual(control.limit_calls, [])

    def test_conflicting_limits_of_same_type_are_rejected(self):
        control = FakeControl()
        control.param_values[(3, DM_variable.PMAX)] = 6.25  # 与 0x01 同型号但量程不同
        manager = MotorManager(
            control,
            configs=(
                MotorConfig(1, 0x11, VelocityControl(Vel_desired=0.1)),
                MotorConfig(3, 0x13, VelocityControl(Vel_desired=0.2)),
            ),
        )
        self.addCleanup(manager.close)
        with self.assertRaises(RuntimeError) as caught:
            manager.configure()
        self.assertIn("实际量程不一致", str(caught.exception))

    def test_link_stats_exposes_bus_counters(self):
        manager, _ = self.make_manager(
            MotorConfig(1, 0x11, VelocityControl(Vel_desired=0.1))
        )
        self.assertEqual(
            manager.link_stats, {"dropped_param_replies": 0, "ignored_frames": 0}
        )

    def test_position_target_beyond_pmax_is_rejected(self):
        control = FakeControl()
        manager = MotorManager(
            control,
            configs=(
                MotorConfig(1, 0x11, MITControl(kp=2.0, kd=0.1, q=13.0, dq=0.0, tau=0.0)),
            ),
        )
        self.addCleanup(manager.close)
        manager.configure()
        manager.enable_all()
        with self.assertRaises(RuntimeError) as caught:
            manager.run(RunConfig(seconds=0.01))
        self.assertIn("q=13.0000", str(caught.exception))
        self.assertIn("±12.5 rad", str(caught.exception))
        self.assertEqual(control.commands, [])

    def test_target_at_pmax_boundary_is_allowed(self):
        manager, control = self.make_manager(
            MotorConfig(1, 0x11, MITControl(kp=2.0, kd=0.1, q=12.5, dq=0.0, tau=0.0))
        )
        self.run_manager(manager)
        self.assertEqual(control.commands[0][4], 12.5)

    def test_sine_overshoot_beyond_pmax_is_rejected(self):
        """base 合法但叠加正弦越界，同样要拦下。"""
        manager, _ = self.make_manager(
            MotorConfig(
                1, 0x11,
                MITControl(kp=2.0, kd=0.1, q=12.0, dq=0.0, tau=0.0),
                motion=SineMotion(amplitude=1.0, frequency=0.5),
            )
        )
        with self.assertRaises(RuntimeError) as caught:
            self.run_manager(manager, seconds=1.0, period=0.25, feedback_timeout=1.0)
        self.assertIn("超出量程", str(caught.exception))

    def test_range_check_uses_motor_limits_not_defaults(self):
        """限值来自电机实测 PMAX（6.25），而不是 SDK 表里的 12.5。"""
        control = FakeControl()
        control.param_values[(1, DM_variable.PMAX)] = 6.25
        manager = MotorManager(
            control,
            configs=(
                MotorConfig(1, 0x11, MITControl(kp=2.0, kd=0.1, q=7.0, dq=0.0, tau=0.0)),
            ),
        )
        self.addCleanup(manager.close)
        manager.configure()
        manager.enable_all()
        with self.assertRaises(RuntimeError) as caught:
            manager.run(RunConfig(seconds=0.01))
        self.assertIn("±6.25 rad", str(caught.exception))

    def test_position_velocity_target_message_names_field(self):
        control = FakeControl()
        manager = MotorManager(
            control,
            configs=(
                MotorConfig(1, 0x11, PositionVelocityControl(P_desired=13.0, V_desired=0.5)),
            ),
        )
        self.addCleanup(manager.close)
        manager.configure()
        manager.enable_all()
        with self.assertRaises(RuntimeError) as caught:
            manager.run(RunConfig(seconds=0.01))
        self.assertIn("P_desired=13.0000", str(caught.exception))

    def test_near_edge_position_prints_warning(self):
        manager, _ = self.make_manager(
            MotorConfig(1, 0x11, VelocityControl(Vel_desired=0.1))
        )
        manager.motors[0].state_q = 12.2  # 距 ±12.5 边界只剩 0.3 rad
        output = io.StringIO()
        with redirect_stdout(output):
            self.run_manager(manager)
        self.assertIn("读数窗口边界", output.getvalue())

    def test_example_import_does_not_connect_or_parse_arguments(self):
        example_path = Path(__file__).resolve().parents[1] / "examples" / "DM_Motor_Example.py"
        with patch.object(MotorManager, "connect") as connect, patch("sys.argv", [str(example_path), "--unused"]):
            namespace = runpy.run_path(str(example_path), run_name="example_import_test")
        connect.assert_not_called()
        configs = namespace["MOTOR_CONFIGS"]
        self.assertEqual([(config.slave_id, config.master_id) for config in configs], [(1, 0x11)])


class MotorControlRoutingTests(unittest.TestCase):
    """DM_CAN.py 接收分发测试：用内存串口，不打开串口、不接电机。"""

    def make_sdk(self, motor, frames=()):
        sdk = MotorControl(FakeSerial(frames))
        sdk.addMotor(motor)
        return sdk

    def test_param_reply_within_guard_is_not_parsed_as_feedback(self):
        """P1-2 回归：参数命令后保护窗口内滞留的应答不得被解析成位置反馈。"""
        motor = create_motor(0x01, 0x11)
        motor.state_q = 1.25  # 先有一个正常位置
        sdk = self.make_sdk(
            motor, [make_frame(0x11, bytes([0x01, 0x00, 0x33, 10, 0x00, 0x00, 0x80, 0x3F]))]
        )
        now = [0.0]
        with patch("motor.DM_CAN.sleep"), patch("motor.DM_CAN.monotonic", lambda: now[0]):
            sdk.save_motor_param(motor)  # 0.0：登记请求（窗口到 1.0，保护窗口到 1.1）
            now[0] = 1.05                # 请求已过期，但仍在保护窗口内
            sdk.recv()
        # 若按反馈解析：data[1..2] = 0x0033 → 位置会被改成约 -12.48 rad
        self.assertEqual(motor.getPosition(), 1.25)
        self.assertEqual(sdk.dropped_param_replies, 1)

    def test_hardware_enable_reply_is_not_dropped(self):
        """实机回归：使能应答帧 data[2] 恰好是 0xAA，但它不是参数应答。"""
        motor = create_motor(0x01, 0x11)
        sdk = self.make_sdk(
            motor, [make_frame(0x11, bytes([0x11, 0x71, 0xAA, 0x7F, 0xF8, 0x00, 0x21, 0x1E]))]
        )
        sdk.recv()
        self.assertEqual(motor.getError(), 0x01)  # 高四位 0x1：已使能
        self.assertEqual(motor.feedback_count, 1)
        self.assertEqual(sdk.dropped_param_replies, 0)
        self.assertAlmostEqual(motor.getPosition(), -1.3998, delta=0.001)

    def test_adapter_frames_are_counted(self):
        motor = create_motor(0x01, 0x11)
        sdk = self.make_sdk(motor, [make_frame(0x00, bytes(8), cmd=0xEE)])
        sdk.recv()
        self.assertEqual(sdk.ignored_frames, 1)
        self.assertEqual(motor.feedback_count, 0)

    def test_read_motor_param_accepts_matching_reply(self):
        motor = create_motor(0x01, 0x11)
        sdk = self.make_sdk(
            motor, [make_frame(0x11, bytes([0x01, 0x00, 0x33, 10, 0x01, 0x00, 0x00, 0x00]))]
        )
        with patch("motor.DM_CAN.sleep"):
            value = sdk.read_motor_param(motor, DM_variable.CTRL_MODE)
        self.assertEqual(value, 1)
        self.assertEqual(motor.temp_param_dict[DM_variable.CTRL_MODE], 1)
        self.assertEqual(motor.feedback_count, 0)  # 参数应答不产生反馈
        self.assertEqual(sdk.dropped_param_replies, 0)

    def test_read_motor_param_keeps_retrying_until_reply(self):
        """早退修复：应答晚于第一次 recv() 时仍能读到值（修复前会立刻返回 None）。"""
        motor = create_motor(0x01, 0x11)
        sdk = self.make_sdk(
            motor,
            [b"", make_frame(0x11, bytes([0x01, 0x00, 0x33, 10, 0x02, 0x00, 0x00, 0x00]))],
        )
        with patch("motor.DM_CAN.sleep"):
            value = sdk.read_motor_param(motor, DM_variable.CTRL_MODE)
        self.assertEqual(value, 2)

    def test_save_reply_is_swallowed_without_parsing(self):
        """0xAA 保存应答只清待处理标志，既不写寄存器也不污染反馈。"""
        motor = create_motor(0x01, 0x11)
        motor.state_q = 1.25
        sdk = self.make_sdk(
            motor, [make_frame(0x11, bytes([0x01, 0x00, 0xAA, 0x01, 0x00, 0x00, 0x00, 0x00]))]
        )
        with patch("motor.DM_CAN.sleep"):
            sdk.save_motor_param(motor)
        sdk.recv()
        self.assertIsNone(sdk._param_request)      # 标志已清
        self.assertEqual(sdk.dropped_param_replies, 0)
        self.assertEqual(motor.temp_param_dict, {})  # 0xAA 不解析成寄存器值
        self.assertEqual(motor.getPosition(), 1.25)

    def test_param_reply_beyond_guard_is_parsed_as_feedback(self):
        """已知取舍：超出保护窗口后，孤立的参数类应答会被当成反馈解析。

        这是刻意选择——静止电机的反馈帧同样可能命中标记字节，而"永久丢弃"会让电机在
        该位置彻底失明（实机已复现）。代价是这种极晚到达的应答会污染一次缓存位置。
        """
        motor = create_motor(0x01, 0x11)
        motor.state_q = 1.25
        sdk = self.make_sdk(
            motor, [make_frame(0x11, bytes([0x01, 0x00, 0x33, 10, 0x02, 0x00, 0x00, 0x00]))]
        )
        now = [0.0]
        with patch("motor.DM_CAN.sleep"), patch("motor.DM_CAN.monotonic", lambda: now[0]):
            sdk.save_motor_param(motor)
            now[0] = 2.0  # 请求和保护窗口都已过期
            sdk.recv()
        self.assertEqual(sdk.dropped_param_replies, 0)
        self.assertAlmostEqual(motor.getPosition(), -12.48, delta=0.01)

    def test_feedback_is_still_parsed_during_param_window(self):
        motor = create_motor(0x01, 0x11)
        sdk = self.make_sdk(
            motor, [make_frame(0x11, bytes([0x11, 0x80, 0x00, 0x80, 0x08, 0x00, 0x19, 0x19]))]
        )
        with patch("motor.DM_CAN.sleep"):
            sdk.save_motor_param(motor)
        sdk.recv()
        self.assertEqual(motor.getError(), 0x01)      # 窗口内的反馈帧照常解析
        self.assertIsNotNone(sdk._param_request)      # 请求仍在等自己的应答
        self.assertEqual(sdk.dropped_param_replies, 0)


if __name__ == "__main__":
    unittest.main()
